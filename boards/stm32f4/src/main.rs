//! STM32F4 ECU Application
//!
//! Bare-metal ECU implementation for STM32F405 microcontroller.
//! Uses the split runtime/board crates with STM32F4-specific HAL implementations.
//!
//! # Hardware Configuration
//!
//! - **Clock**: 168 MHz system clock
//! - **Timer**: TIM2 configured for microsecond counting
//! - **Trigger Input**: PA0 (rising edge interrupt)
//! - **Outputs**:
//!   - PB0: Injector 1
//!   - PB1: Injector 2
//!   - PB2: Ignition coil 1
//!   - PB3: Ignition coil 2
//!
//! # Memory Safety
//!
//! Uses critical sections (`cortex_m::interrupt::free`) to safely access
//! shared state between ISR and main loop.

#![no_std]
#![no_main]

#[cfg(all(not(test), not(feature = "capture-tim"), not(feature = "capture-gpio")))]
compile_error!("production STM32F4 builds require one trigger capture path: enable `capture-tim` or `capture-gpio`");
#[cfg(all(not(test), feature = "capture-tim", feature = "capture-gpio"))]
compile_error!("features `capture-tim` and `capture-gpio` are mutually exclusive");

use cortex_m_rt::entry;
use ecu_domain::{Kpa10, Micros};
use ecu_scheduler::TransitionDrainBuffer;
use ecu_target_common::{
    adapter::{
        BoardAdapter, BoardEvent, CommonObservabilityDrainCycleReport, CommonObservabilityRecord,
        CommonObservabilityRecordKind, CommonObservabilitySample,
        CommonObservabilityTraceCycleReport, FixedCommonObservabilityTracePair,
    },
    bringup::bringup_fuel_model,
    control_inputs::{split_control_frame_from, WarmBringupControlSignals},
    noop::{NoopCapture, NoopStore, NoopTransport},
    outputs::{ScheduledActionExecutor, ScheduledOutputs4},
    sensor_sample::{BoardSensorSnapshotSampleSource, FixedLoadSensor},
    split_tick::run_runtime_scheduled_output_tick_and_push_to_trace_pair,
    trigger_adapter::{
        apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair, SplitTriggerAdapter,
    },
};
#[cfg(feature = "ts-usb-hw")]
use ecu_ts::persistence::written_pages_require_runtime_fuel_retune;
#[cfg(not(test))]
use panic_halt as _;
use stm32f4xx_hal::{pac, pac::interrupt, prelude::*};

#[cfg(feature = "transport-can")]
mod can_support;
mod capture;
mod hal_impl;
#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
mod identity_provisioning;
#[cfg(feature = "transport-can")]
mod obd2_support;
#[cfg(feature = "flash-kv")]
mod store_support;
#[cfg(feature = "ts-usb")]
mod ts_support;
use hal_impl::Stm32Time;
use hal_impl::Stm32Watchdog;
#[cfg(feature = "obd2-identity-can-provisioning")]
use identity_provisioning::Obd2IdentityProvisioningOperatorArm;

// Current STM32F4 bring-up exposes exactly two injector and two ignition
// channels through `ScheduledOutputs4`. Worst-case pending work before the next
// drain is therefore two combined injection+ignition windows, or eight queued
// transitions total.
const BOARD_SPLIT_SCHEDULER_QUEUE_CAP: usize = 8;

#[cfg(feature = "transport-can")]
fn stm32f4_obd2_identity_record(
) -> ecu_target_common::transport_service::Obd2ProvisionedIdentityRecord {
    ecu_target_common::transport_service::Obd2ProvisionedIdentityRecord::from_ascii(
        Some(b"pipstm32f40000001"),
        Some(b"pipcalstm32f40001"),
        Some(b"stm4a1"),
    )
}

// Arduino-style global pin declarations for easy remapping
// Adjust these macros to change physical pin assignments
macro_rules! INJ1_PIN {
    ($gpiob:ident) => {
        $gpiob.pb0.into_push_pull_output()
    };
}
macro_rules! INJ2_PIN {
    ($gpiob:ident) => {
        $gpiob.pb1.into_push_pull_output()
    };
}
macro_rules! IGN1_PIN {
    ($gpiob:ident) => {
        $gpiob.pb2.into_push_pull_output()
    };
}
macro_rules! IGN2_PIN {
    ($gpiob:ident) => {
        $gpiob.pb3.into_push_pull_output()
    };
}

#[inline]
fn empty_observability_record() -> CommonObservabilityRecord {
    CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample::default(),
    }
}

#[inline]
fn empty_drain_report() -> CommonObservabilityDrainCycleReport {
    CommonObservabilityDrainCycleReport {
        sample: CommonObservabilityTraceCycleReport {
            drained: 0,
            overflow_count: 0,
            status: Default::default(),
        },
        record: CommonObservabilityTraceCycleReport {
            drained: 0,
            overflow_count: 0,
            status: Default::default(),
        },
    }
}

#[inline]
fn drain_observability_pair<const S: usize, const R: usize, const SO: usize, const RO: usize>(
    traces: &mut FixedCommonObservabilityTracePair<S, R>,
    sample_out: &mut [CommonObservabilitySample; SO],
    record_out: &mut [CommonObservabilityRecord; RO],
    report: &mut CommonObservabilityDrainCycleReport,
) {
    *report = traces.drain_cycle(sample_out, record_out);
}

#[entry]
fn main() -> ! {
    #[allow(unused_mut)]
    let mut dp = pac::Peripherals::take().unwrap();

    // Setup clocks (168MHz)
    let rcc = dp.RCC.constrain();
    // Configure system clock; for USB we need a valid 48MHz USB clock derived from PLL
    #[allow(unused_variables)]
    let clocks = rcc.cfgr.sysclk(168.MHz()).require_pll48clk().freeze();

    // Setup timer for microsecond counter
    // TIM2 is 32-bit, perfect for microsecond timing
    //
    // IMPORTANT: Timer prescaler calculation
    // - APB1 clock = 168MHz / 2 = 84MHz (divided by APB1 prescaler)
    // - Timer clock = 84MHz * 2 = 168MHz (TIMx clock multiplier when APB prescaler != 1)
    // - For 1μs ticks: prescaler = 168
    // - PSC register value = 168 - 1 = 167
    //
    // With PSC=167: 168MHz / 168 = 1MHz = 1μs per tick ✓
    dp.TIM2.psc.write(|w| w.psc().bits(168 - 1));
    dp.TIM2.arr.write(|w| w.arr().bits(u32::MAX)); // Max count for 32-bit timer
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit()); // Enable counter

    // Create time source (zero-sized type, just uses TIM2 pointer)
    let time_source = Stm32Time;

    let gpioa = dp.GPIOA.split();

    #[cfg(all(feature = "capture-tim", not(feature = "capture-gpio")))]
    capture::configure_capture_inputs(gpioa.pa0, &mut dp.TIM2);
    #[cfg(all(feature = "capture-gpio", not(feature = "capture-tim")))]
    capture::configure_capture_inputs(gpioa.pa0, dp.SYSCFG, &mut dp.EXTI);

    #[cfg(feature = "ts-usb")]
    let ts_state =
        cortex_m::singleton!(: ts_support::BoardTsState = ts_support::BoardTsState::new())
            .expect("STM32 TS board state singleton already taken");

    // Start independent watchdog (~250ms) and clear reset flags (best-effort)
    let mut iwdg = Stm32Watchdog::new(dp.IWDG);
    iwdg.start(250);

    // Enable timer IRQ only in capture-tim path (done above)

    // Setup output pins for injectors/coils
    let gpiob = dp.GPIOB.split();
    #[cfg(feature = "transport-can")]
    let mut obd2_service = {
        use bxcan::filter::Mask32;
        use bxcan::Fifo;
        use stm32f4xx_hal::can::CanExt as _;

        let can = dp.CAN1.can((gpiob.pb9, gpiob.pb8));
        let mut can = bxcan::Can::builder(can)
            .set_bit_timing(0x001c_0000)
            .enable();
        let mut filters = can.modify_filters();
        filters.enable_bank(0, Fifo::Fifo0, Mask32::accept_all());
        drop(filters);
        obd2_support::new_service(can_support::new_transport(can))
    };
    // Use macros above to obtain output pins; edit macros to remap
    let mut inj1 = INJ1_PIN!(gpiob);
    let mut inj2 = INJ2_PIN!(gpiob);
    let mut ign1 = IGN1_PIN!(gpiob);
    let mut ign2 = IGN2_PIN!(gpiob);

    // Force safe state on boot and wrap via split scheduled outputs.
    inj1.set_low();
    inj2.set_low();
    ign1.set_low();
    ign2.set_low();
    let mut outputs = ScheduledOutputs4::new(inj1, inj2, ign1, ign2);
    let mut drain = TransitionDrainBuffer::<BOARD_SPLIT_SCHEDULER_QUEUE_CAP>::new();
    let mut observability_traces: FixedCommonObservabilityTracePair<16, 16> =
        FixedCommonObservabilityTracePair::new();
    let mut observability_sample_scratch = [CommonObservabilitySample::default(); 16];
    let mut observability_record_scratch = [empty_observability_record(); 16];
    let mut last_drain_report = empty_drain_report();
    let mut adapter = BoardAdapter::new(
        BoardSensorSnapshotSampleSource::new(FixedLoadSensor::new(Stm32Time, Kpa10::new(700))),
        NoopCapture,
        ScheduledActionExecutor::<BOARD_SPLIT_SCHEDULER_QUEUE_CAP>::new(),
        iwdg,
        NoopTransport,
        NoopStore,
    );
    #[cfg(all(feature = "transport-can", not(feature = "flash-kv")))]
    adapter.install_provisioned_obd2_identity_record(stm32f4_obd2_identity_record());
    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    let mut retained_history_store = store_support::FlashKv::new();
    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    adapter.install_provisioned_obd2_identity_record(
        retained_history_store
            .load_obd2_identity_record()
            .unwrap_or_else(stm32f4_obd2_identity_record),
    );
    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    adapter.set_obd2_identity_key_lifecycle_status(Some(
        identity_provisioning::obd2_identity_key_lifecycle_status_from_optional_audit(
            retained_history_store.load_obd2_identity_key_audit(),
        ),
    ));
    #[cfg(feature = "obd2-identity-can-provisioning")]
    let mut obd2_identity_operator_arm =
        Obd2IdentityProvisioningOperatorArm::from_persisted_key_or_compile_time_env(
            retained_history_store
                .load_obd2_identity_provisioning_key_record()
                .map(|record| (record.active_key(), record.last_arm_nonce)),
        );
    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    adapter.record_store_integrity_status(store_support::boot_store_integrity());
    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    let mut obd2_retained_history_persistence =
        ecu_target_common::transport_service::Obd2RetainedHistoryPersistenceState::restore_from_store(
            &mut adapter,
            &retained_history_store,
        );
    if adapter
        .configure_fuel_model_and_push_to_trace_pair(
            bringup_fuel_model(),
            &mut observability_traces,
        )
        .is_ok()
    {
        drain_observability_pair(
            &mut observability_traces,
            &mut observability_sample_scratch,
            &mut observability_record_scratch,
            &mut last_drain_report,
        );
    }
    let mut control_signals = WarmBringupControlSignals;
    let mut trigger_adapter = SplitTriggerAdapter::new(Stm32Time);

    // Optional: TunerStudio USB hardware bring-up (feature-gated)
    #[cfg(feature = "ts-usb-hw")]
    let (mut maybe_ts, mut maybe_cdc) = {
        use ecu_target_common::ts::usb_cdc::CdcSerial;
        use stm32f4xx_hal::otg_fs::{UsbBusType, USB};
        use usb_device::class_prelude::UsbBusAllocator;
        use usb_device::prelude::*;
        use usbd_serial::USB_CLASS_CDC;
        // Configure USB pins PA11/PA12 to AF10
        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &clocks,
        );
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        let bus = cortex_m::singleton!(
            : UsbBusAllocator<UsbBusType> =
                UsbBusType::new(usb, unsafe { &mut *core::ptr::addr_of_mut!(EP_MEMORY) })
        )
        .expect("STM32 USB allocator singleton already taken");
        let cdc_opt = {
            let serial = usbd_serial::SerialPort::new(bus);
            let dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x1d50, 0x6130))
                .device_class(USB_CLASS_CDC)
                .strings(&[StringDescriptors::default()
                    .manufacturer("IPW")
                    .product("TS-ECU")
                    .serial_number("STM32F4-TS")])
                .unwrap()
                .build();
            Some(CdcSerial { serial, dev })
        };
        let svc = ts_support::new_service(ts_state, adapter.runtime());
        if adapter
            .configure_runtime_fuel_strategy_and_push_to_trace_pair(
                ecu_runtime::runtime_fuel_strategy_from_fuel_tune(
                    &svc.server.store().runtime_fuel_tune(),
                ),
                &mut observability_traces,
            )
            .is_ok()
        {
            drain_observability_pair(
                &mut observability_traces,
                &mut observability_sample_scratch,
                &mut observability_record_scratch,
                &mut last_drain_report,
            );
        }
        (Some(svc), cdc_opt)
    };
    #[cfg(all(not(feature = "ts-usb-hw"), feature = "ts-usb"))]
    if adapter
        .configure_runtime_fuel_strategy_and_push_to_trace_pair(
            ecu_runtime::runtime_fuel_strategy_from_fuel_tune(
                &ecu_calibration::FuelRuntimeTune::new(
                    ts_state.config.ve_table,
                    ts_state.config.afr_table,
                    ts_state.config.required_fuel_us,
                    ts_state.config.injector_deadtime_us,
                    ts_state.config.ve_load_source,
                ),
            ),
            &mut observability_traces,
        )
        .is_ok()
    {
        drain_observability_pair(
            &mut observability_traces,
            &mut observability_sample_scratch,
            &mut observability_record_scratch,
            &mut last_drain_report,
        );
    }
    #[cfg(all(not(feature = "ts-usb-hw"), not(feature = "ts-usb")))]
    if adapter
        .configure_runtime_fuel_strategy_and_push_to_trace_pair(
            ecu_runtime::runtime_fuel_strategy_from_fuel_tune(
                &ecu_calibration::FuelRuntimeTune::new(
                    [[100; 16]; 16],
                    [[147; 16]; 16],
                    1000,
                    800,
                    0,
                ),
            ),
            &mut observability_traces,
        )
        .is_ok()
    {
        drain_observability_pair(
            &mut observability_traces,
            &mut observability_sample_scratch,
            &mut observability_record_scratch,
            &mut last_drain_report,
        );
    }
    // Main loop - feed capture into the split runtime and apply due scheduled outputs.
    loop {
        #[cfg(feature = "transport-can")]
        {
            #[cfg(feature = "flash-kv")]
            adapter.set_obd2_flash_write_fault_status(Some(
                store_support::last_flash_write_fault_status(),
            ));
            #[cfg(not(feature = "flash-kv"))]
            let _ = obd2_service.pump_once(&mut adapter);
            #[cfg(all(feature = "flash-kv", not(feature = "obd2-identity-can-provisioning")))]
            let _ = obd2_service.pump_once(&mut adapter);
            #[cfg(feature = "obd2-identity-can-provisioning")]
            let pump_result = obd2_service.pump_once(&mut adapter);
            #[cfg(feature = "obd2-identity-can-provisioning")]
            if let Ok(Some(
                ecu_target_common::transport_service::Obd2MultiServiceTransportServiceOutcome::Ignored(
                    message,
                ),
            )) = pump_result
            {
                if let Ok(Some(response)) =
                    identity_provisioning::obd2_identity_operator_arm_message_with_persisted_nonce(
                        &mut obd2_identity_operator_arm,
                        &mut retained_history_store,
                        &message,
                    )
                {
                    let _ = ecu_transport::Transport::send(obd2_service.transport_mut(), &response);
                } else if let Ok(Some(response)) =
                    identity_provisioning::provision_flash_kv_obd2_identity_key_operator_message_with_audit(
                        &mut obd2_identity_operator_arm,
                        &mut retained_history_store,
                        &message,
                    )
                {
                    adapter.set_obd2_identity_key_lifecycle_status(Some(
                        identity_provisioning::obd2_identity_key_lifecycle_status_after_operator_response(
                            identity_provisioning::obd2_identity_key_audit_from_message(&response),
                            retained_history_store.load_obd2_identity_key_audit(),
                        ),
                    ));
                    let _ = ecu_transport::Transport::send(obd2_service.transport_mut(), &response);
                } else if let Ok(Some(response)) =
                    identity_provisioning::provision_flash_kv_obd2_identity_operator_message_with_persisted_audit(
                        &mut obd2_identity_operator_arm,
                        &mut retained_history_store,
                        &message,
                    )
                {
                    let _ = ecu_transport::Transport::send(obd2_service.transport_mut(), &response);
                }
            }
        }

        // Refresh the TS sensor overlay until real sensors land.
        #[cfg(feature = "ts-usb")]
        {
            ts_support::update_sensors();
        }

        while let Some(ts) = capture::pop() {
            let at_us = Micros::new(ts);
            if apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
                &mut adapter,
                &mut trigger_adapter,
                ts,
                &mut observability_traces,
            )
            .is_ok()
            {
                drain_observability_pair(
                    &mut observability_traces,
                    &mut observability_sample_scratch,
                    &mut observability_record_scratch,
                    &mut last_drain_report,
                );
            }
            if adapter
                .apply_event_and_push_to_trace_pair(
                    BoardEvent::CamEdge {
                        at_us,
                        cam_seen: false,
                    },
                    &mut observability_traces,
                )
                .is_ok()
            {
                drain_observability_pair(
                    &mut observability_traces,
                    &mut observability_sample_scratch,
                    &mut observability_record_scratch,
                    &mut last_drain_report,
                );
            }
        }
        if adapter
            .poll_sensor_and_push_to_trace_pair(&mut observability_traces)
            .is_ok()
        {
            drain_observability_pair(
                &mut observability_traces,
                &mut observability_sample_scratch,
                &mut observability_record_scratch,
                &mut last_drain_report,
            );
        }

        let now = Micros::new(time_source.micros());
        let frame = split_control_frame_from(&mut control_signals, now, trigger_adapter.rpm())
            .unwrap_or_else(|never| match never {});
        if adapter
            .set_shift_arming_and_push_to_trace_pair(
                frame.launch_armed,
                frame.flat_shift_armed,
                &mut observability_traces,
            )
            .is_ok()
        {
            drain_observability_pair(
                &mut observability_traces,
                &mut observability_sample_scratch,
                &mut observability_record_scratch,
                &mut last_drain_report,
            );
        }
        if run_runtime_scheduled_output_tick_and_push_to_trace_pair(
            &mut adapter,
            now,
            frame.control,
            &mut observability_traces,
            &mut outputs,
            &mut drain,
        )
        .is_ok()
        {
            drain_observability_pair(
                &mut observability_traces,
                &mut observability_sample_scratch,
                &mut observability_record_scratch,
                &mut last_drain_report,
            );
        }

        #[cfg(feature = "ts-usb-hw")]
        {
            if let (Some(ref mut svc), Some(ref mut cdc)) = (&mut maybe_ts, &mut maybe_cdc) {
                svc.pump_with_budget(cdc, 4);
                if let Some(pages) = svc.server.store_mut().take_written_pages() {
                    if written_pages_require_runtime_fuel_retune(pages) {
                        let tune = svc.server.store().runtime_fuel_tune();
                        let _ = adapter.configure_runtime_fuel_strategy_and_push_to_trace_pair(
                            ecu_runtime::runtime_fuel_strategy_from_fuel_tune(&tune),
                            &mut observability_traces,
                        );
                        drain_observability_pair(
                            &mut observability_traces,
                            &mut observability_sample_scratch,
                            &mut observability_record_scratch,
                            &mut last_drain_report,
                        );
                    }
                }
            }
        }
        #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
        {
            let _ = obd2_retained_history_persistence
                .persist_if_changed(&adapter, &mut retained_history_store);
        }
    }
}

/// TIM2 interrupt handler
/// - Captures timestamps on CH1 rising edges and pushes into ring buffer
/// - Schedules events when decoder is updated (minimal extra work here)
#[cfg(all(feature = "capture-tim", not(feature = "capture-gpio")))]
#[cortex_m_rt::interrupt]
fn TIM2() {
    capture::handle_tim2_interrupt();
}

#[cfg(all(feature = "capture-gpio", not(feature = "capture-tim")))]
#[cortex_m_rt::interrupt]
fn EXTI0() {
    capture::handle_exti0_interrupt();
}
