#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(all(
    not(test),
    not(feature = "capture-pio"),
    not(feature = "synthetic-trigger-demo")
))]
compile_error!(
    "`ts-ecu` needs `capture-pio` for flashable firmware; use `synthetic-trigger-demo` only for no-hardware demos"
);

#[cfg(not(test))]
use cortex_m_rt::entry;
#[cfg(not(test))]
use panic_halt as _;

use embedded_hal::digital::v2::OutputPin;
use hal::adc::{Adc, AdcPin};
use hal::clocks::init_clocks_and_plls;
#[cfg(feature = "capture-pio")]
use hal::gpio::FunctionPio0;
#[cfg(feature = "capture-cam")]
use hal::gpio::Interrupt::EdgeHigh;
#[cfg(any(feature = "capture-pio", feature = "capture-cam"))]
use hal::pac::interrupt;
#[cfg(feature = "capture-pio")]
use hal::pio::PIOExt;
use hal::usb::UsbBus;
use hal::watchdog::Watchdog;
use hal::{pac, sio::Sio};
use rp2040_hal as hal;
#[path = "../ts_usb_cdc.rs"]
mod ts_usb_cdc;
use ts_usb_cdc::{CdcSerial, CdcSerialStats};
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort as UsbdSerial;
use usbd_serial::USB_CLASS_CDC;

/// Output-test safety bounds. A host-driven output test busy-blocks the
/// real-time loop, so on/off durations and repetition counts are clamped to
/// conservative maxima and each delay is split into chunks short enough that
/// the hardware watchdog (500 ms) can be fed between them.
const OUTPUT_TEST_MAX_MS: u32 = 1000;
const OUTPUT_TEST_MAX_REPS: u8 = 5;
const OUTPUT_TEST_CHUNK_MS: u32 = 50;
// Current live RP2040 bring-up exposes exactly two injector and two ignition
// channels through `ScheduledOutputs4`. Worst-case pending work before the next
// drain is therefore two combined injection+ignition windows, or eight queued
// transitions total.
const BOARD_SPLIT_SCHEDULER_QUEUE_CAP: usize = 8;

#[cfg(not(test))]
#[inline]
fn perform_system_reboot() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

#[cfg(test)]
#[inline]
fn perform_system_reboot() {}

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

#[cfg(feature = "capture-cam")]
use core::sync::atomic::{AtomicBool, Ordering};
use ecu_calibration::DfcoConfig;
use ecu_calibration::{CalibrationHardwareTargetId, CalibrationRuntimeBuildId};
use ecu_compat::ts::CompatCalibrationSession;
use ecu_control::{AccelerationConfig, AfterStartConfig, WarmupConfig};
use ecu_domain::Micros;
use ecu_rp2040_pico::{PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP, RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU};
use ecu_scheduler::TransitionDrainBuffer;
#[cfg(feature = "capture-cam")]
use ecu_target_common::adapter::BoardEvent;
#[cfg(not(feature = "flash-kv"))]
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::ts::service::TsService;
use ecu_target_common::ts::state_ptr::StateRef;
use ecu_target_common::{
    adapter::BoardAdapter,
    adapter::{
        CommonObservabilityDrainCycleReport, CommonObservabilityRecord,
        CommonObservabilityRecordKind, CommonObservabilitySample,
        CommonObservabilityTraceCycleReport, FixedCommonObservabilityTracePair,
    },
    bringup::bringup_fuel_model,
    control_inputs::split_control_frame_from,
    outputs::{ScheduledActionExecutor, ScheduledOutputs4},
    sensor_sample::{BoardSensorSnapshotSampleSource, LiveLoadSensor},
    split_tick::run_runtime_scheduled_output_tick_and_push_to_trace_pair,
    trigger_adapter::{
        apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair, SplitTriggerAdapter,
    },
};
use ecu_ts::outpc::Outpc;
use ecu_ts::persistence::{
    written_pages_require_runtime_fuel_retune, PersistedTsPageStore, TsPackageCommandStore,
};
use ecu_ts::serial::{FrameAssembler, SerialPort};
use ecu_ts::server::OutpcProvider;
use ecu_ts::RuntimeSnapshotAdapter;
#[cfg(all(feature = "flash-kv", target_arch = "arm"))]
#[path = "../flash_kv.rs"]
mod flash_kv;
#[cfg(all(feature = "flash-kv", not(target_arch = "arm")))]
mod seq_kv;
#[cfg(all(feature = "flash-kv", target_arch = "arm"))]
use flash_kv::FlashKv;
#[cfg(all(feature = "flash-kv", not(target_arch = "arm")))]
use seq_kv::SeqKv;
#[path = "ts_ecu/board_io.rs"]
mod board_io;
#[path = "ts_ecu/ts_pages.rs"]
mod ts_pages;
#[path = "ts_ecu/ts_runtime.rs"]
mod ts_runtime;
#[path = "ts_ecu/ts_usb.rs"]
mod ts_usb;
use board_io::{fed_delay_ms, AbsentCapture, AbsentStore, AbsentTransport, RpWatchdog};
use ecu_board_api::Watchdog as _;
use ts_pages::{
    refresh_ts_outpc_config_from_state as read_ts_outpc_config_from_state,
    EcuStatePageStoreProvider,
};
use ts_runtime::{
    with_main_state, AdcPins, BoardEcuState, IdlePwmRuntime, Rp2040ControlSignals, Rp2040MapLoad,
    RpTime,
};
use ts_usb::{
    encode_tooth_composite_log_from_records, encode_tooth_stats_reply,
    handle_compatibility_info_cmd, handle_output_test_cmd, handle_reboot_cmd,
    handle_version_info_cmd, TsBenchCommandOwner,
};

#[cfg(all(feature = "flash-kv", target_arch = "arm"))]
fn ecu_state_engine_running(_context: *const ()) -> bool {
    // Trigger/sync state lives in the split runtime, not the board ECU state, so
    // the legacy guard (which read the always-default board rpm/sync fields)
    // never reported "running"; preserve that behavior.
    false
}

#[cfg(feature = "flash-kv")]
fn persist_crc_fault_event() -> ecu_domain::diag::DiagEvent {
    use ecu_domain::diag::{DiagCode, DiagEvent, DiagSource};
    DiagEvent {
        code: DiagCode::PersistCrcFault,
        timestamp: ecu_domain::Micros::new(0),
        source: DiagSource::User,
        context: None,
        start_us: 0,
        end_us: 0,
    }
}

#[cfg(all(feature = "flash-kv", target_arch = "arm"))]
fn latch_persist_crc_fault(state: &mut BoardEcuState) {
    state.diag_log.push(persist_crc_fault_event());
}

// Arduino-style outputs — edit these to remap pins quickly
macro_rules! INJ1_GPIO {
    ($pins:ident) => {
        $pins.gpio0.into_push_pull_output()
    };
}
macro_rules! INJ2_GPIO {
    ($pins:ident) => {
        $pins.gpio1.into_push_pull_output()
    };
}
macro_rules! IGN1_GPIO {
    ($pins:ident) => {
        $pins.gpio2.into_push_pull_output()
    };
}
macro_rules! IGN2_GPIO {
    ($pins:ident) => {
        $pins.gpio3.into_push_pull_output()
    };
}
// Additional outputs for idle PWM and fan relay (adjust pins as needed)
macro_rules! IDLE_GPIO {
    ($pins:ident) => {
        $pins.gpio6.into_push_pull_output()
    };
}
macro_rules! FAN_GPIO {
    ($pins:ident) => {
        $pins.gpio7.into_push_pull_output()
    };
}
#[cfg(feature = "capture-pio")]
const TRIGGER_PIN: u8 = 4; // use with GPIO-IRQ or PIO example bins
#[cfg(feature = "capture-cam")]
const CAM_PIN: u8 = 5;

ecu_target_common::capture_ring!(CAPTURE, 128);

// Main-loop and TS-shared state collapsed into one struct behind a single
// critical-section accessor (`with_main_state`). This reduces the double-borrow
// panic class to a single borrow site. ISR-shared data (capture ring, cam phase)
// stays separate.
// INVARIANT: All access happens via `with_main_state` (interrupts disabled inside
// cortex_m::interrupt::free). The RefCell borrow rules are enforced by the
// single-threaded nature of RP2040 (no Send/Sync concerns).
#[cfg(feature = "capture-cam")]
static CAM_PHASE: AtomicBool = AtomicBool::new(false);

fn refresh_ts_outpc_config_from_state(state: &BoardEcuState) {
    let config = read_ts_outpc_config_from_state(state);
    with_main_state(|s| s.ts_outpc_config = config);
}

struct Provider {
    runtime: StateRef<ecu_runtime::EngineRuntime>,
    runtime_adapter: RuntimeSnapshotAdapter,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        let snapshot = unsafe { self.runtime.with(|runtime| runtime.snapshot()) };
        self.runtime_adapter.fill_outpc(&snapshot, out);
        // Use critical section for shared mutable state access - extract individual Copy values
        let (
            tps_percent,
            clt_c,
            iat_c,
            vbatt_mv,
            lambda_valid,
            lambda_x100,
            mapdot_kpa_s,
            tpsdot_pct_s,
            ts_config,
        ) = with_main_state(|s| {
            (
                s.sens.tps_percent,
                s.sens.clt_c,
                s.sens.iat_c,
                s.sens.vbatt_mv,
                s.sens.lambda_valid,
                s.sens.lambda_x100,
                s.sens.mapdot_kpa_s,
                s.sens.tpsdot_pct_s,
                s.ts_outpc_config,
            )
        });
        out.tps_percent = tps_percent;
        out.clt_c = clt_c;
        out.iat_c = iat_c;
        out.vbatt_mv = vbatt_mv;
        if lambda_valid {
            out.lambda_x100 = lambda_x100;
        } else {
            out.lambda_x100 = 100;
        }
        // Example fuel calc with live enrichment state and a simple CL loop.
        let wue_pct_x100 = WarmupConfig::DEFAULT.compute_percent_x100(clt_c);
        let ae_pct_x100 = with_main_state(|s| {
            s.ae.update(
                Micros::new(RpTime.micros()),
                tpsdot_pct_s,
                mapdot_kpa_s,
                &AccelerationConfig::DEFAULT,
            )
        });
        // Compute target lambda from AFR target (approx gasoline stoich 14.7)
        let target_lambda_x100 = ((ts_config.target_afr_x10 as u32) * 1000 / 147) as i32;
        let lambda_meas_x100 = lambda_x100 as i32;
        let error = target_lambda_x100 - lambda_meas_x100; // positive -> richer target than measured
                                                           // Proportional gain: kp_i is percent per 100 lambda error
        let kp = ts_config.cl_kp_i as i32;
        let mut cl_delta: i32 = 0;
        if lambda_valid {
            cl_delta = (error * kp) / 100; // i16 percent
                                           // Integral term (simple accumulator, clamped)
            let ki = ts_config.cl_ki_i as i32;
            if ki > 0 {
                let st = with_main_state(|s| {
                    s.cl.integ = (s.cl.integ + (error * ki) / 100).clamp(-50, 50);
                    s.cl.integ
                });
                cl_delta += st;
            } else {
                with_main_state(|s| s.cl.integ = 0);
            }
            // Clamp to +/-25%
            cl_delta = cl_delta.clamp(-25, 25);
        } else {
            with_main_state(|s| s.cl.integ = 0);
        }
        // Extended fields
        out.target_afr_x10 = ts_config.target_afr_x10;
        out.ego_correction_percent = (100 + cl_delta).clamp(0, 200) as u8;
        out.ego_sensor = if lambda_valid {
            ts_config.ego_sensor
        } else {
            0
        };
        out.mapdot_kpa_s = mapdot_kpa_s;
        out.tpsdot_pct_s = tpsdot_pct_s;
        // Approximate injector duty_x10 = 10 * pw_us * rpm / 120_000_000
        let duty = ((out.pw_us as u32)
            .saturating_mul(out.rpm as u32)
            .saturating_mul(10))
            / 120_000_000;
        out.inj_duty_x10 = duty.min(1000) as u16;
        out.idle_duty_x10 = 0;
        out.fan_state = 0;
        // Engine state bits: bit0=WUE, bit1=ASE, bit2=CL, bit3=DFCO, bit4=AE, bit5=EMERGENCY
        let mut flags: u16 = 0;
        // WUE (based on config and CLT); x100 multiplier, 100 = no enrichment
        if wue_pct_x100 > 100 {
            flags |= 1 << 0;
        }
        // AE: already updated above in ae_pct_x100 calculation
        if ae_pct_x100 > 100 {
            flags |= 1 << 4;
        }
        // ASE: trigger when leaving cranking
        let just_started = with_main_state(|s| {
            let prev_crank = s.crank_gate.is_cranking();
            let _ = s.crank_gate.update(out.rpm);
            prev_crank && !s.crank_gate.is_cranking()
        });
        let ase_x100 = with_main_state(|s| {
            s.ase.update(
                Micros::new(RpTime.micros()),
                just_started,
                &AfterStartConfig::DEFAULT,
            )
        });
        if ase_x100 > 100 {
            flags |= 1 << 1;
        }
        // DFCO
        let dfco = with_main_state(|s| {
            s.dfco.update(
                RpTime.micros(),
                out.rpm,
                tps_percent,
                out.map_kpa_x10 / 10,
                &DfcoConfig::DEFAULT,
            )
        });
        if dfco {
            flags |= 1 << 3;
        }
        // Emergency mode (bit5)
        if ts_config.emergency_mode {
            flags |= 1 << 5;
        }
        out.engine_state = flags;
        out.baro_kpa = 100;
        out.gear = 0;
    }

    fn engine_running(&self) -> bool {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        let snapshot = unsafe { self.runtime.with(|runtime| runtime.snapshot()) };
        matches!(snapshot.engine.sync, ecu_domain::SyncState::Locked { .. })
            && snapshot.engine.rpm.get() > 0
    }
}

#[cfg_attr(not(test), entry)]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().expect("Peripherals already taken");
    let _core = pac::CorePeripherals::take().expect("CorePeripherals already taken");
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .expect("clock initialization failed");

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Optional: configure PIO capture on TRIGGER_PIN
    #[cfg(feature = "capture-pio")]
    {
        // Route the configured trigger pin to PIO0. Keep GPIO0..GPIO3 owned by
        // injector/ignition outputs below.
        let _trigger = pins.gpio4.into_function::<FunctionPio0>();
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let program = pio_proc::pio_asm!(
            ".wrap_target",
            "wait 0 pin 0",
            "wait 1 pin 0",
            "irq 0",
            ".wrap",
        );
        let installed = pio
            .install(&program.program)
            .expect("PIO program install failed");
        let (mut sm, _rx, _tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            .in_pin_base(TRIGGER_PIN)
            .clock_divisor_fixed_point(1, 0)
            .build(sm0);
        sm.set_pindirs([]);
        let irq0 = pio.irq0();
        pio.clear_irq(1);
        irq0.enable_sm_interrupt(0);
        let _sm = sm.start();
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
        }
    }

    // USB bus allocator
    let usb_bus: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let serial = UsbdSerial::new(&usb_bus);
    let dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000A))
        .manufacturer("IPW")
        .product("TS ECU")
        .serial_number("IPW-TS-ECU-0001")
        .device_class(USB_CLASS_CDC)
        .build();
    let mut cdc = CdcSerial {
        serial,
        dev,
        stats: CdcSerialStats::new(),
    };

    // Board-owned ECU calibration/diagnostic state for the TS page surface.
    let state = cortex_m::singleton!(: BoardEcuState = BoardEcuState::new())
        .expect("BoardEcuState singleton already taken");

    // Prepare scheduled outputs using the split scheduler path.
    let inj1 = INJ1_GPIO!(pins);
    let inj2 = INJ2_GPIO!(pins);
    let ign1 = IGN1_GPIO!(pins);
    let ign2 = IGN2_GPIO!(pins);
    let mut outputs = ScheduledOutputs4::new(inj1, inj2, ign1, ign2);
    let mut drain = TransitionDrainBuffer::<BOARD_SPLIT_SCHEDULER_QUEUE_CAP>::new();
    let mut observability_traces: FixedCommonObservabilityTracePair<16, 16> =
        FixedCommonObservabilityTracePair::new();
    let mut observability_sample_scratch = [CommonObservabilitySample::default(); 16];
    let mut observability_record_scratch = [empty_observability_record(); 16];
    let mut last_drain_report = empty_drain_report();
    // Hardware watchdog for the safety role. 500 ms is comfortably above the
    // worst-case main-loop iteration (sensor sample + USB pump + scheduler tick,
    // plus any clamped output test which feeds the watchdog mid-loop) yet tight
    // enough that a true hang resets the chip well within a second.
    let watchdog = RpWatchdog::arm(watchdog, fugit::MicrosDurationU32::millis(500));
    let mut adapter = BoardAdapter::new(
        BoardSensorSnapshotSampleSource::new(LiveLoadSensor::new(RpTime, Rp2040MapLoad)),
        AbsentCapture,
        ScheduledActionExecutor::<BOARD_SPLIT_SCHEDULER_QUEUE_CAP>::new(),
        watchdog,
        AbsentTransport,
        AbsentStore,
    );
    let _ = adapter.configure_fuel_model_and_push_to_trace_pair(
        bringup_fuel_model(),
        &mut observability_traces,
    );
    drain_observability_pair(
        &mut observability_traces,
        &mut observability_sample_scratch,
        &mut observability_record_scratch,
        &mut last_drain_report,
    );
    let mut control_signals = Rp2040ControlSignals;
    let mut trigger_adapter = SplitTriggerAdapter::new(RpTime);
    #[cfg(feature = "synthetic-trigger-demo")]
    let mut simulated_trigger_tooth: u8 = 0;
    // Extra outputs
    let mut idle_pin = IDLE_GPIO!(pins);
    let mut fan_pin = FAN_GPIO!(pins);
    let mut idle_pwm = IdlePwmRuntime::new();

    // TS service (uses shared TsService; KV is feature-selectable)
    refresh_ts_outpc_config_from_state(state);
    let provider = Provider {
        runtime: StateRef::new(adapter.runtime()),
        runtime_adapter: RuntimeSnapshotAdapter::new(),
    };
    #[cfg(all(feature = "flash-kv", feature = "transport-can", target_arch = "arm"))]
    let (mut store, mut obd2_retained_history_persistence) = {
        let kv = FlashKv::new_with_engine_guard(
            state as *const BoardEcuState as *const (),
            ecu_state_engine_running,
        );
        // ADR-0003: a slot carrying our format but failing CRC means corruption
        // (power loss mid-write), not a blank first boot. Boot on built-in
        // defaults and latch a diagnostic fault visible on the TS Diag page.
        // `ValidWithCorruptSibling` boots the valid (older committed) slot, but
        // a torn write rolled the tuner's last change back, so latch the same
        // informational diagnostic while still loading the good tune.
        let boot_integrity = kv.boot_integrity();
        match boot_integrity {
            ecu_target_common::kv::ab::StoreIntegrityStatus::Corrupt
            | ecu_target_common::kv::ab::StoreIntegrityStatus::ValidWithCorruptSibling => {
                latch_persist_crc_fault(state);
                adapter.record_store_integrity_status(boot_integrity);
            }
            ecu_target_common::kv::ab::StoreIntegrityStatus::Blank
            | ecu_target_common::kv::ab::StoreIntegrityStatus::Valid => {}
        }
        let obd2_retained_history_persistence =
            ecu_target_common::transport_service::Obd2RetainedHistoryPersistenceState::restore_from_store(
                &mut adapter,
                &kv,
            );
        (
            PersistedTsPageStore::new(EcuStatePageStoreProvider::new(state, adapter.runtime()), kv),
            obd2_retained_history_persistence,
        )
    };
    #[cfg(all(
        feature = "flash-kv",
        not(feature = "transport-can"),
        target_arch = "arm"
    ))]
    let mut store = {
        let kv = FlashKv::new_with_engine_guard(
            state as *const BoardEcuState as *const (),
            ecu_state_engine_running,
        );
        // ADR-0003: a slot carrying our format but failing CRC means corruption
        // (power loss mid-write), not a blank first boot. Boot on built-in
        // defaults and latch a diagnostic fault visible on the TS Diag page.
        // `ValidWithCorruptSibling` boots the valid (older committed) slot, but
        // a torn write rolled the tuner's last change back, so latch the same
        // informational diagnostic while still loading the good tune.
        let boot_integrity = kv.boot_integrity();
        match boot_integrity {
            ecu_target_common::kv::ab::StoreIntegrityStatus::Corrupt
            | ecu_target_common::kv::ab::StoreIntegrityStatus::ValidWithCorruptSibling => {
                latch_persist_crc_fault(state);
                adapter.record_store_integrity_status(boot_integrity);
            }
            ecu_target_common::kv::ab::StoreIntegrityStatus::Blank
            | ecu_target_common::kv::ab::StoreIntegrityStatus::Valid => {}
        }
        PersistedTsPageStore::new(EcuStatePageStoreProvider::new(state, adapter.runtime()), kv)
    };
    #[cfg(all(feature = "flash-kv", not(target_arch = "arm")))]
    let mut store = PersistedTsPageStore::new(
        EcuStatePageStoreProvider::new(state, adapter.runtime()),
        SeqKv::new(),
    );
    #[cfg(not(feature = "flash-kv"))]
    let mut store = PersistedTsPageStore::new(
        EcuStatePageStoreProvider::new(state, adapter.runtime()),
        RamKv512::new(),
    );
    store.try_load();
    let _ = adapter.configure_runtime_fuel_strategy_and_push_to_trace_pair(
        ecu_runtime::runtime_fuel_strategy_from_fuel_tune(&store.runtime_fuel_tune()),
        &mut observability_traces,
    );
    drain_observability_pair(
        &mut observability_traces,
        &mut observability_sample_scratch,
        &mut observability_record_scratch,
        &mut last_drain_report,
    );
    refresh_ts_outpc_config_from_state(state);
    let runtime_build_id =
        CalibrationRuntimeBuildId::new(RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU.get());
    let hardware_target_id =
        CalibrationHardwareTargetId::new(PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP.get());
    let mut session = CompatCalibrationSession::from_persisted_store(&store);
    let _ = store.try_load_and_import_package(&mut session, runtime_build_id, hardware_target_id);
    let store = TsPackageCommandStore::new(store, session, runtime_build_id, hardware_target_id);
    let mut ts = TsService::new(ecu_ts::TS_SIGNATURE, provider, store);

    // Framing buffers (for custom commands only)
    let mut asm = FrameAssembler::new();
    let mut inbuf = [0u8; 512];
    let mut out = [0u8; 512];
    let mut reboot_requested = false;

    // ADC setup and channel pins
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pins = AdcPins {
        map: AdcPin::new(pins.gpio26.into_pull_down_disabled()),
        tps: AdcPin::new(pins.gpio27.into_pull_down_disabled()),
        clt: AdcPin::new(pins.gpio28.into_pull_down_disabled()),
        iat: AdcPin::new(pins.gpio29.into_pull_down_disabled()),
    };

    // Optional: configure CAM input via IO_IRQ_BANK0
    #[cfg(feature = "capture-cam")]
    {
        let cam = pins.gpio5.into_pull_up_input();
        cam.set_interrupt_enabled(EdgeHigh, true);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }
    }

    #[cfg(feature = "capture-cam")]
    let mut last_cam_phase = CAM_PHASE.load(Ordering::Relaxed);

    loop {
        // Pump USB
        let mut tmp = [0u8; 64];
        let n = cdc.read(&mut tmp);
        if n > 0 {
            asm.feed(&tmp[..n]);
        }
        if let Some(len) = asm.try_pop(&mut inbuf) {
            let compatibility_report = ts.server.store().compatibility_report();
            // The shared TS server owns the command dispatch, but this live board
            // path still supplies per-frame bench-tooling hooks over local
            // adapter/output state.
            let adapter_ptr: *mut _ = &mut adapter;
            let outputs_ptr: *mut _ = &mut outputs;
            let observability_record_scratch_ptr: *const [CommonObservabilityRecord; 16] =
                &observability_record_scratch;
            let last_drain_report_ptr: *const CommonObservabilityDrainCycleReport =
                &last_drain_report;
            let mut bench_tooling = TsBenchCommandOwner::new(
                |payload: &[u8], out: &mut [u8]| {
                    handle_output_test_cmd(
                        payload,
                        |chan, on_ms, off_ms, reps| {
                            let adapter = unsafe { &mut *adapter_ptr };
                            let outputs = unsafe { &mut *outputs_ptr };
                            // Refuse to drive outputs while the engine turns:
                            // a synced runtime with non-zero rpm means a
                            // running engine, and busy-blocking the loop then
                            // would stall scheduling/safety on a live engine.
                            let snapshot = adapter.runtime().snapshot();
                            let engine_running = matches!(
                                snapshot.engine.sync,
                                ecu_domain::SyncState::Locked { .. }
                            ) && snapshot.engine.rpm.get() > 0;
                            if engine_running {
                                return;
                            }
                            let on_ms = on_ms.min(OUTPUT_TEST_MAX_MS);
                            let off_ms = off_ms.min(OUTPUT_TEST_MAX_MS);
                            let reps = reps.min(OUTPUT_TEST_MAX_REPS);
                            for _ in 0..reps {
                                {
                                    let (injectors, ignition) = outputs.as_scheduled_pins();
                                    match chan {
                                        0 => injectors[0].set_scheduled_high(),
                                        1 => injectors[1].set_scheduled_high(),
                                        2 => ignition[0].set_scheduled_high(),
                                        3 => ignition[1].set_scheduled_high(),
                                        _ => {}
                                    }
                                }
                                fed_delay_ms(on_ms, OUTPUT_TEST_CHUNK_MS, || {
                                    let _ = adapter.watchdog().feed();
                                });
                                {
                                    let (injectors, ignition) = outputs.as_scheduled_pins();
                                    match chan {
                                        0 => injectors[0].set_scheduled_low(),
                                        1 => injectors[1].set_scheduled_low(),
                                        2 => ignition[0].set_scheduled_low(),
                                        3 => ignition[1].set_scheduled_low(),
                                        _ => {}
                                    }
                                }
                                fed_delay_ms(off_ms, OUTPUT_TEST_CHUNK_MS, || {
                                    let _ = adapter.watchdog().feed();
                                });
                            }
                        },
                        out,
                    )
                },
                |payload: &[u8], out: &mut [u8]| {
                    if !payload.is_empty() {
                        return None;
                    }
                    let adapter = unsafe { &*adapter_ptr };
                    let snapshot = adapter.runtime().snapshot();
                    let synced =
                        matches!(snapshot.engine.sync, ecu_domain::SyncState::Locked { .. });
                    encode_tooth_stats_reply(snapshot.engine.rpm.get(), synced, out)
                },
                |payload: &[u8], out: &mut [u8]| {
                    if !payload.is_empty() {
                        return None;
                    }
                    let records = unsafe { &*observability_record_scratch_ptr };
                    let drain_report = unsafe { &*last_drain_report_ptr };
                    let overflow_count =
                        drain_report.record.overflow_count.min(u32::from(u16::MAX)) as u16;
                    encode_tooth_composite_log_from_records(
                        records,
                        drain_report.record.drained,
                        overflow_count,
                        out,
                    )
                },
                |payload: &[u8], out: &mut [u8]| {
                    handle_version_info_cmd(
                        payload,
                        ecu_ts::TS_SIGNATURE,
                        RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU.get(),
                        PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP.get(),
                        out,
                    )
                },
                |payload: &[u8], out: &mut [u8]| {
                    handle_compatibility_info_cmd(payload, compatibility_report, out)
                },
                |payload: &[u8], out: &mut [u8]| {
                    handle_reboot_cmd(payload, || reboot_requested = true, out)
                },
            );
            let reply_len =
                ts.handle_frame_with_bench_tooling(&inbuf[..len], &mut out, &mut bench_tooling);
            refresh_ts_outpc_config_from_state(state);
            if let Some(mr) = reply_len {
                let _ = cdc.write(&out[..mr]);
            }
            if reboot_requested {
                perform_system_reboot();
            }
            if let Some(pages) = ts.server.store_mut().take_written_pages() {
                if written_pages_require_runtime_fuel_retune(pages) {
                    let tune = ts.server.store().runtime_fuel_tune();
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

        // Update sensors
        let sensors_cal = state.config.sensors_cal;
        with_main_state(|s| s.sens.update(&mut adc, &mut adc_pins, &sensors_cal, state));
        refresh_ts_outpc_config_from_state(state);

        // Fan control (on/off with hysteresis)
        {
            let fan_cfg = state.config.fan_config;
            if fan_cfg.enable {
                let clt = with_main_state(|s| s.sens.clt_c);
                if clt >= fan_cfg.on_c {
                    let _ = fan_pin.set_high();
                } else if clt <= fan_cfg.off_c {
                    let _ = fan_pin.set_low();
                }
            } else {
                let _ = fan_pin.set_low();
            }
        }

        // Idle PWM (open-loop)
        {
            let cfg = state.config.idle_config;
            if !cfg.enable || cfg.duty_x10 == 0 || cfg.freq_hz == 0 {
                let _ = idle_pin.set_low();
                idle_pwm.pin_is_high = false;
                idle_pwm.last_start_us = RpTime.micros();
            } else {
                let now = RpTime.micros();
                let period_us =
                    (1_000_000u32).saturating_div(core::cmp::max(1, cfg.freq_hz as u32));
                let on_us = (period_us as u64 * (cfg.duty_x10 as u64) / 1000u64) as u32;
                let elapsed = now.wrapping_sub(idle_pwm.last_start_us);
                if elapsed >= period_us {
                    idle_pwm.last_start_us = now;
                    if on_us > 0 {
                        let _ = idle_pin.set_high();
                        idle_pwm.pin_is_high = true;
                    } else {
                        let _ = idle_pin.set_low();
                        idle_pwm.pin_is_high = false;
                    }
                } else if idle_pwm.pin_is_high && elapsed >= on_us {
                    let _ = idle_pin.set_low();
                    idle_pwm.pin_is_high = false;
                }
            }
        }

        while let Some(ts) = capture_pop() {
            let _ = apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
                &mut adapter,
                &mut trigger_adapter,
                ts,
                &mut observability_traces,
            );
            drain_observability_pair(
                &mut observability_traces,
                &mut observability_sample_scratch,
                &mut observability_record_scratch,
                &mut last_drain_report,
            );
        }
        let _ = adapter.poll_sensor_and_push_to_trace_pair(&mut observability_traces);
        drain_observability_pair(
            &mut observability_traces,
            &mut observability_sample_scratch,
            &mut observability_record_scratch,
            &mut last_drain_report,
        );

        let now = Micros::new(RpTime.micros());
        let frame = split_control_frame_from(&mut control_signals, now, trigger_adapter.rpm())
            .unwrap_or_else(|never| match never {});
        let _ = adapter.set_shift_arming_and_push_to_trace_pair(
            frame.launch_armed,
            frame.flat_shift_armed,
            &mut observability_traces,
        );
        drain_observability_pair(
            &mut observability_traces,
            &mut observability_sample_scratch,
            &mut observability_record_scratch,
            &mut last_drain_report,
        );
        let _ = run_runtime_scheduled_output_tick_and_push_to_trace_pair(
            &mut adapter,
            now,
            frame.control,
            &mut observability_traces,
            &mut outputs,
            &mut drain,
        );
        drain_observability_pair(
            &mut observability_traces,
            &mut observability_sample_scratch,
            &mut observability_record_scratch,
            &mut last_drain_report,
        );

        // Demo-only synthetic trigger source. Flashable firmware uses PIO capture.
        #[cfg(feature = "synthetic-trigger-demo")]
        {
            let delay_cycles = if simulated_trigger_tooth == 57 {
                48_000
            } else {
                24_000
            };
            cortex_m::asm::delay(delay_cycles);
            capture_push(RpTime.micros());
            simulated_trigger_tooth = if simulated_trigger_tooth == 57 {
                0
            } else {
                simulated_trigger_tooth + 1
            };
        }

        // If cam phase toggled, notify the split runtime.
        #[cfg(feature = "capture-cam")]
        {
            let cam_phase = CAM_PHASE.load(Ordering::Relaxed);
            if cam_phase != last_cam_phase {
                last_cam_phase = cam_phase;
                let _ = adapter.apply_event_and_push_to_trace_pair(
                    BoardEvent::CamEdge {
                        at_us: Micros::new(RpTime.micros()),
                        cam_seen: cam_phase,
                    },
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

        #[cfg(all(feature = "flash-kv", feature = "transport-can", target_arch = "arm"))]
        {
            let _ = obd2_retained_history_persistence
                .persist_if_changed(&adapter, ts.server.store_mut().store_mut().kv_mut());
        }
    }
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "flash-kv")]
    #[test]
    fn persist_crc_fault_event_has_shared_obd2_meaning_shape() {
        let event = super::persist_crc_fault_event();
        assert_eq!(event.code, ecu_domain::diag::DiagCode::PersistCrcFault);
        assert_eq!(event.timestamp, ecu_domain::Micros::new(0));
        assert_eq!(event.source, ecu_domain::diag::DiagSource::User);
        assert_eq!(event.context, None);
        assert_eq!(event.start_us, 0);
        assert_eq!(event.end_us, 0);
    }
}

#[cfg(feature = "capture-pio")]
#[allow(non_snake_case)]
#[interrupt]
fn PIO0_IRQ_0() {
    capture_push(RpTime.micros());
    let pio = unsafe { &*pac::PIO0::ptr() };
    pio.irq.write(|w| unsafe { w.irq().bits(1) });
}

#[cfg(feature = "capture-cam")]
#[allow(non_snake_case)]
#[interrupt]
fn IO_IRQ_BANK0() {
    // Toggle phase on cam rising edge
    CAM_PHASE.store(!CAM_PHASE.load(Ordering::Relaxed), Ordering::Relaxed);
    let io = unsafe { &*pac::IO_BANK0::ptr() };
    let group = (CAM_PIN as usize) / 8;
    let edge_high_mask = 0b1000u32 << (((CAM_PIN as u32) & 0x7) * 4);
    io.intr[group].write(|w| unsafe { w.bits(edge_high_mask) });
}
