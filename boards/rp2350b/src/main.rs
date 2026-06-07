#![no_std]
#![no_main]

use cortex_m_rt::entry;
use ecu_calibration::FuelRuntimeTune;
use ecu_domain::{Kpa10, Micros, Rpm, SyncState};
use ecu_runtime::{
    runtime_semantic_calibration_from_fuel_tune, runtime_semantic_evaluate_fuel,
    RuntimeSemanticAfrOverride, RuntimeSemanticEngineMode, RuntimeSemanticInputSnapshot,
    RuntimeSemanticState,
};
use hal::{clocks::ClockSource, pac};
use panic_halt as _;
use rp235x_hal as hal;

fn semantic_input(
    now_us: u32,
    map_kpa10: u16,
    tps_x100: u16,
    load_kpa10: u16,
) -> RuntimeSemanticInputSnapshot {
    RuntimeSemanticInputSnapshot {
        t_us: Micros::new(now_us),
        rpm: Rpm::new(3_000),
        map_kpa10: Kpa10::new(map_kpa10),
        load_kpa10: Kpa10::new(load_kpa10),
        tps_x100,
        clt_c10: 800,
        iat_c10: 250,
        baro_kpa10: Kpa10::new(1013),
        vbatt_mv: 13_500,
        sync: SyncState::Locked { cam_ref: false },
        mode: RuntimeSemanticEngineMode::Running,
        fuel_cut: false,
        spark_cut: false,
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        target_afr_override_x100: RuntimeSemanticAfrOverride::None,
    }
}

fn fuel_tune(ve_load_source: u8) -> FuelRuntimeTune {
    FuelRuntimeTune::new([[100; 16]; 16], [[147; 16]; 16], 2200, 800, ve_load_source)
}

fn safe_halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

fn take_or_halt<T>(value: Option<T>) -> T {
    match value {
        Some(value) => value,
        None => safe_halt(),
    }
}

#[entry]
fn main() -> ! {
    let mut pac_periph = take_or_halt(pac::Peripherals::take());
    let core = take_or_halt(cortex_m::Peripherals::take());
    let mut watchdog = hal::Watchdog::new(pac_periph.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac_periph.XOSC,
        pac_periph.CLOCKS,
        pac_periph.PLL_SYS,
        pac_periph.PLL_USB,
        &mut pac_periph.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap_or_else(|| safe_halt());
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.get_freq().to_Hz());

    let mut semantic_state = RuntimeSemanticState::default();
    let map_cal = runtime_semantic_calibration_from_fuel_tune(&fuel_tune(0)); // MAP
    let low = runtime_semantic_evaluate_fuel(
        &map_cal,
        semantic_input(1_000, 600, 250, 600),
        semantic_state,
    )
    .unwrap_or_else(|_| safe_halt());
    semantic_state = RuntimeSemanticState::default();
    let high = runtime_semantic_evaluate_fuel(
        &map_cal,
        semantic_input(2_000, 1200, 250, 1200),
        semantic_state,
    )
    .unwrap_or_else(|_| safe_halt());
    if high.pw_corr_us < low.pw_corr_us {
        safe_halt();
    }

    let alpha_cal = runtime_semantic_calibration_from_fuel_tune(&fuel_tune(1)); // TPS / Alpha-N
    let alpha_n = runtime_semantic_evaluate_fuel(
        &alpha_cal,
        semantic_input(3_000, 600, 800, 800),
        RuntimeSemanticState::default(),
    )
    .unwrap_or_else(|_| safe_halt());
    if alpha_n.pw_corr_us == 0 {
        safe_halt();
    }

    loop {
        delay.delay_ms(500);
    }
}
