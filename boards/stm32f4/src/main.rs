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
    adapter::{BoardAdapter, BoardEvent},
    bringup::bringup_fuel_model,
    control_inputs::{split_control_inputs_from, WarmBringupControlSignals},
    noop::{NoopCapture, NoopStore, NoopTransport},
    outputs::{ScheduledActionExecutor, ScheduledOutputs4},
    sensor_sample::{BoardSensorSnapshotSampleSource, FixedLoadSensor},
    split_tick::run_runtime_scheduled_output_tick,
    trigger_adapter::{apply_trigger_timestamp_to_runtime_adapter, SplitTriggerAdapter},
};
#[cfg(feature = "ts-usb-hw")]
use ecu_ts::persistence::written_pages_require_runtime_fuel_retune;
#[cfg(not(test))]
use panic_halt as _;
use stm32f4xx_hal::{pac, pac::interrupt, prelude::*};

mod capture;
mod hal_impl;
#[cfg(feature = "ts-usb")]
mod ts_support;
use hal_impl::Stm32Time;
use hal_impl::Stm32Watchdog;

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
    let mut drain = TransitionDrainBuffer::<8>::new();
    let mut adapter = BoardAdapter::new(
        BoardSensorSnapshotSampleSource::new(FixedLoadSensor::new(Stm32Time, Kpa10::new(700))),
        NoopCapture,
        ScheduledActionExecutor::<8>::new(),
        iwdg,
        NoopTransport,
        NoopStore,
    );
    adapter.configure_fuel_model(bringup_fuel_model());
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
        static mut EP_MEMORY: [u32; 1024] = [0; 1024];
        static mut USB_ALLOC: Option<UsbBusAllocator<UsbBusType>> = None;
        // Configure USB pins PA11/PA12 to AF10
        let usb = USB::new(
            (dp.OTG_FS_GLOBAL, dp.OTG_FS_DEVICE, dp.OTG_FS_PWRCLK),
            (gpioa.pa11, gpioa.pa12),
            &clocks,
        );
        let cdc_opt = unsafe {
            USB_ALLOC = Some(UsbBusType::new(usb, &mut EP_MEMORY));
            let bus = USB_ALLOC.as_ref().unwrap();
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
        adapter.configure_runtime_fuel_strategy(ecu_runtime::runtime_fuel_strategy_from_fuel_tune(
            &svc.server.store().runtime_fuel_tune(),
        ));
        (Some(svc), cdc_opt)
    };
    #[cfg(all(not(feature = "ts-usb-hw"), feature = "ts-usb"))]
    adapter.configure_runtime_fuel_strategy(ecu_runtime::runtime_fuel_strategy_from_fuel_tune(
        &ecu_calibration::FuelRuntimeTune::new(
            ts_state.config.ve_table,
            ts_state.config.afr_table,
            ts_state.config.required_fuel_us,
            ts_state.config.injector_deadtime_us,
            ts_state.config.ve_load_source,
        ),
    ));
    #[cfg(all(not(feature = "ts-usb-hw"), not(feature = "ts-usb")))]
    adapter.configure_runtime_fuel_strategy(ecu_runtime::runtime_fuel_strategy_from_fuel_tune(
        &ecu_calibration::FuelRuntimeTune::new([[100; 16]; 16], [[147; 16]; 16], 1000, 800, 0),
    ));

    // Main loop - feed capture into the split runtime and apply due scheduled outputs.
    loop {
        // Refresh the TS sensor overlay until real sensors land.
        #[cfg(feature = "ts-usb")]
        {
            ts_support::update_sensors();
        }

        while let Some(ts) = capture::pop() {
            let at_us = Micros::new(ts);
            let _ =
                apply_trigger_timestamp_to_runtime_adapter(&mut adapter, &mut trigger_adapter, ts);
            let _ = adapter.apply_event(BoardEvent::CamEdge {
                at_us,
                cam_seen: false,
            });
        }
        let _ = adapter.poll_sensor();

        let now = Micros::new(time_source.micros());
        let _ = run_runtime_scheduled_output_tick(
            &mut adapter,
            now,
            split_control_inputs_from(&mut control_signals, now, trigger_adapter.rpm())
                .unwrap_or_else(|never| match never {}),
            &mut outputs,
            &mut drain,
        );

        #[cfg(feature = "ts-usb-hw")]
        {
            if let (Some(ref mut svc), Some(ref mut cdc)) = (&mut maybe_ts, &mut maybe_cdc) {
                svc.pump_with_budget(cdc, 4);
                if let Some(pages) = svc.server.store_mut().take_written_pages() {
                    if written_pages_require_runtime_fuel_retune(pages) {
                        let tune = svc.server.store().runtime_fuel_tune();
                        adapter.configure_runtime_fuel_strategy(
                            ecu_runtime::runtime_fuel_strategy_from_fuel_tune(&tune),
                        );
                    }
                }
            }
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
