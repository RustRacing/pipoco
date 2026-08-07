//! Differential tests (review 011): feed the spec oracle wrapper and the
//! compat sensor-processing path identical streams and assert identical
//! verdicts. Both sides now delegate to the canonical
//! `ecu_control::sensors::{plausibility, slew}` implementations, so this
//! pins behavioral equivalence by construction.

use ecu_compat::sensors::plausibility::PlausibilityFault;
use ecu_compat::sensors::plausibility::{PlausibilityState, RateConfig, RateValidationState};
use ecu_spec::{
    sensor_plausibility_step, sensor_slew_step, DiagnosticCode, SensorPlausibilityInput,
    SensorPlausibilityState, SensorSlewInput, SensorSlewState,
};

// ---------------------------------------------------------------------------
// Slew / rate limiting
// ---------------------------------------------------------------------------

/// RateConfig matching the canonical spec slew rates. Compat works in
/// percent (0-100) with 500%/s; spec works in x100 (0-10_000) with
/// 50_000 x100/s - numerically the same per-second limit.
const MATCHED_RATE_CONFIG: RateConfig = RateConfig {
    enable: true,
    max_tps_rate_per_sec: 500,     // = spec TPS_MAX_RATE_PER_S / 100
    max_map_rate_per_sec: 2_000,   // = spec MAP_MAX_RATE_PER_S
    min_sample_interval_us: 1_000, // = spec MIN_SLEW_DT_US
};

fn spec_slew_input(
    t_us: u32,
    tps_x100: u16,
    map_kpa10: u16,
    previous: &SensorSlewState,
) -> SensorSlewInput {
    // Keep every non-compared channel in-range and constant so only the
    // compared channels can trip the rate limiter.
    SensorSlewInput {
        t_us,
        clt_c10: 800,
        iat_c10: 250,
        map_kpa10,
        tps_x100,
        maf_x100: if previous.initialized {
            previous.maf_x100
        } else {
            12_000
        },
        o2_afr_x100: if previous.initialized {
            previous.o2_afr_x100
        } else {
            1470
        },
        knock_intensity_x100: 100,
        baro_kpa10: if previous.initialized {
            previous.baro_kpa10
        } else {
            1000
        },
        vbat_mv: if previous.initialized {
            previous.vbat_mv
        } else {
            12_000
        },
    }
}

#[test]
fn slew_differential_identical_streams_produce_identical_verdicts() {
    let stream: [(u32, u8, u16); 6] = [
        (0, 50, 700),        // initialize
        (10_000, 100, 700),  // TPS spike, MAP flat
        (110_000, 55, 900),  // MAP jump, TPS drift
        (210_000, 100, 950), // TPS spike again, MAP flat
        (310_000, 100, 100), // MAP out-of-window spike
        (410_000, 60, 800),  // recovery
    ];

    let mut compat = RateValidationState::new();
    let mut spec_state = SensorSlewState::default();

    for (now_us, tps_pct, map_kpa10) in stream {
        let spec_input = spec_slew_input(now_us, (tps_pct as u16) * 100, map_kpa10, &spec_state);
        let spec = sensor_slew_step(spec_input, spec_state);
        spec_state = spec.next_state;

        let (tps_out, map_out, tps_rej, map_rej) =
            compat.validate(tps_pct, map_kpa10, &MATCHED_RATE_CONFIG, now_us);

        assert_eq!(
            spec.limited.tps_x100 as u16 / 100,
            tps_out as u16,
            "tps verdict drift at t={now_us}"
        );
        assert_eq!(
            spec.limited.map_kpa10, map_out,
            "map verdict drift at t={now_us}"
        );
        assert_eq!(
            spec.limited.tps_x100 != spec_input.tps_x100,
            tps_rej,
            "tps clamp flag drift at t={now_us}"
        );
        assert_eq!(
            spec.limited.map_kpa10 != spec_input.map_kpa10,
            map_rej,
            "map clamp flag drift at t={now_us}"
        );
    }
}

// ---------------------------------------------------------------------------
// Plausibility
// ---------------------------------------------------------------------------

fn spec_plausibility_input(
    t_us: u32,
    tps_pct: u8,
    map_kpa10: u16,
    rpm: u16,
    maf_x100: u16,
) -> SensorPlausibilityInput {
    // All non-compared channels in-range; `maf_x100` alternates so the
    // canonical stuck-value detector cannot fire (we want the TPS/MAP
    // cross-check to be the only fault source, mirroring compat).
    SensorPlausibilityInput {
        t_us,
        rpm,
        clt_c10: 800,
        iat_c10: 250,
        map_kpa10,
        tps_x100: (tps_pct as u16) * 100,
        maf_x100,
        o2_afr_x100: 1470,
        knock_intensity_x100: 100,
        baro_kpa10: 1000,
        vbat_mv: 12_000,
    }
}

#[test]
fn plausibility_differential_identical_streams_produce_identical_verdicts() {
    let mut compat = PlausibilityState::new();
    let config = ecu_calibration::PlausibilityConfig::DEFAULT;
    let mut spec_state = SensorPlausibilityState::default();

    let mut maf = 12_000u16;
    let mut now = 0u32;
    let stream: [(u8, u16, u16); 8] = [
        (50, 700, 3000), // clean, running
        (85, 250, 3000), // TPS high / MAP low fault begins
        (85, 250, 3000), // still faulted
        (50, 700, 3000), // clean (starts clear debounce)
        (50, 700, 900),  // below min rpm gate
        (85, 250, 3000), // fault again
        (85, 250, 3000), // latches
        (50, 700, 3000), // clear begins
    ];

    for (tps_pct, map_kpa10, rpm) in stream {
        now += 400_000;
        maf = if maf == 12_000 { 12_001 } else { 12_000 };

        let spec_input = spec_plausibility_input(now, tps_pct, map_kpa10, rpm, maf);
        let spec = sensor_plausibility_step(spec_input, spec_state);
        spec_state = spec.next_state;

        let compat_fault = compat.check(tps_pct, map_kpa10, rpm, &config, now);
        let compat_confirmed = compat_fault != PlausibilityFault::None;

        assert_eq!(
            spec.diagnostic == DiagnosticCode::SensorPlausibilityFault,
            compat_confirmed,
            "plausibility verdict drift at t={now}: tps={tps_pct} map={map_kpa10} rpm={rpm}"
        );
        assert_eq!(
            spec.diagnostic == DiagnosticCode::SensorPlausibilityFault,
            compat.has_fault(),
            "compat latch state drift at t={now}"
        );
    }
}
