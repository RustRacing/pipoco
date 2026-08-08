//! Differential tests (review 011): feed the spec oracle wrapper and the
//! compat sensor-processing path identical streams and assert identical
//! verdicts. The spec side derives the slew window independently of
//! `ecu-control` (see `crates/spec/src/sensors/slew.rs`), so agreement here is
//! evidence rather than a tautology.

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

/// A long gap between samples pushes the rate window past `u16::MAX`
/// (2000 kPa10/s x 33 s = 66_000). Both sides must still admit the full
/// reading; an implementation that narrows the window to the channel width
/// before clamping wraps it to 464 and freezes the channel instead.
#[test]
fn slew_differential_survives_a_window_wider_than_the_channel() {
    let mut compat = RateValidationState::new();
    let mut spec_state = SensorSlewState::default();

    for (now_us, map_kpa10) in [(0u32, 700u16), (33_000_000, 4_000)] {
        let spec_input = spec_slew_input(now_us, 5_000, map_kpa10, &spec_state);
        let spec = sensor_slew_step(spec_input, spec_state);
        spec_state = spec.next_state;

        let (_, map_out, _, map_rej) = compat.validate(50, map_kpa10, &MATCHED_RATE_CONFIG, now_us);

        assert_eq!(
            spec.limited.map_kpa10, map_out,
            "map verdict drift at t={now_us}"
        );
        assert_eq!(
            spec.limited.map_kpa10 != spec_input.map_kpa10,
            map_rej,
            "map clamp flag drift at t={now_us}"
        );
    }

    assert_eq!(
        spec_state.map_kpa10, 4_000,
        "the wide window must admit the full reading"
    );
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

// ---------------------------------------------------------------------------
// Canonical implementation vs the independent spec oracle
//
// `ecu-compat` carries its own sensor logic, so the tests above compare compat
// against the oracle. That leaves `ecu-control` -- the implementation the boards
// actually run -- unpinned by any oracle. These tests close that gap: the same
// stream through `ecu_control` and through the spec must yield the same
// verdicts, and because the two derivations are structurally independent a
// defect in either one shows up here.
// ---------------------------------------------------------------------------

fn control_plausibility_input(
    t_us: u32,
    rpm: u16,
    tps_x100: u16,
    map_kpa10: u16,
    maf_x100: u16,
) -> ecu_control::sensors::plausibility::PlausibilityInput {
    ecu_control::sensors::plausibility::PlausibilityInput {
        t_us,
        rpm,
        clt_c10: 800,
        iat_c10: 250,
        map_kpa10,
        tps_x100,
        maf_x100,
        o2_afr_x100: 1470,
        knock_intensity_x100: 100,
        baro_kpa10: 1000,
        vbat_mv: 12_000,
    }
}

#[test]
fn canonical_plausibility_matches_the_spec_oracle() {
    // Walks the load-disagreement boundary, the debounce edges, the rpm gate,
    // and the stuck-value detector.
    let stream: [(u32, u16, u16, u16, u16); 20] = [
        // (dt_us, rpm, tps_x100, map_kpa10, maf_x100)
        (400_000, 3000, 2500, 700, 12_000),  // clean
        (400_000, 3000, 7_999, 300, 12_001), // just below the disagreement edge
        (400_000, 3000, 8_000, 300, 12_002), // exactly at it: fault begins
        (100_000, 3000, 8_000, 300, 12_003), // still short of debounce
        (400_000, 3000, 8_000, 300, 12_004), // debounce elapses: latches
        (400_000, 900, 2500, 700, 12_005),   // below rpm gate: verdict held
        (400_000, 3000, 2500, 700, 12_006),  // clean again, clear streak starts
        (400_000, 3000, 2500, 700, 12_007),  // clear debounce elapses
        (400_000, 3000, 1_000, 950, 12_008), // opposite disagreement edge
        (400_000, 3000, 1_001, 950, 12_009), // just outside it
        (400_000, 3000, 2500, 700, 12_010),  // clean
        (400_000, 3000, 2500, 700, 12_010),  // every channel repeats: stuck
        // The rpm gate is a boundary, so straddle it exactly rather than
        // sampling far to either side.
        (400_000, 999, 2500, 700, 12_011), // one below the gate: verdict held
        (400_000, 1000, 2500, 700, 12_012), // exactly at the gate: active
        (400_000, 1001, 2500, 700, 12_013), // one above
        // Out-of-band probes, one channel at a time, just past each edge.
        (400_000, 3000, 10_001, 700, 12_014), // tps above band
        (400_000, 3000, 2500, 99, 12_015),    // map below band
        (400_000, 3000, 2500, 3_001, 12_016), // map above band
        (400_000, 3000, 2500, 700, 60_001),   // maf above band
        (400_000, 3000, 2500, 700, 12_017),   // back in band
    ];

    let mut control_state = ecu_control::sensors::plausibility::PlausibilityState::default();
    let mut spec_state = SensorPlausibilityState::default();
    let mut now = 0u32;

    for (dt_us, rpm, tps_x100, map_kpa10, maf_x100) in stream {
        now += dt_us;
        let input = control_plausibility_input(now, rpm, tps_x100, map_kpa10, maf_x100);

        let control = ecu_control::sensors::plausibility::plausibility_step(input, control_state);
        control_state = control.next_state;

        let spec = sensor_plausibility_step(input, spec_state);
        spec_state = spec.next_state;

        assert_eq!(
            control.latched,
            spec.diagnostic == DiagnosticCode::SensorPlausibilityFault,
            "verdict drift at t={now}: rpm={rpm} tps={tps_x100} map={map_kpa10}"
        );
        assert_eq!(
            control_state.assert_counter_us, spec_state.assert_counter_us,
            "assert streak drift at t={now}"
        );
        assert_eq!(
            control_state.clear_counter_us, spec_state.clear_counter_us,
            "clear streak drift at t={now}"
        );
    }
}

#[test]
fn canonical_slew_matches_the_spec_oracle() {
    // Includes a window wider than the channel (the truncation defect class)
    // and a sub-minimum dt that must hold the previous reading.
    let stream: [(u32, u16, u16, i16); 7] = [
        // (t_us, map_kpa10, maf_x100, clt_c10)
        (0, 700, 12_000, 800),
        (500, 4_000, 60_000, 1_500),        // below MIN_SLEW_DT_US: held
        (100_000, 4_000, 60_000, 1_500),    // normal window: clamped
        (200_000, 700, 12_000, 800),        // back down, still clamped
        (33_200_000, 4_000, 60_000, 1_500), // window wider than the channel
        (33_201_000, 100, 0, -400),         // minimum dt at the extremes
        (66_500_000, 3_000, 30_000, 1_000), // another wide window
    ];

    let mut control_state = ecu_control::sensors::slew::SlewState::default();
    let mut spec_state = SensorSlewState::default();

    for (t_us, map_kpa10, maf_x100, clt_c10) in stream {
        let input = ecu_control::sensors::slew::SlewInput {
            t_us,
            clt_c10,
            iat_c10: 250,
            map_kpa10,
            tps_x100: 2_500,
            maf_x100,
            o2_afr_x100: 1470,
            knock_intensity_x100: 100,
            baro_kpa10: 1000,
            vbat_mv: 12_000,
        };

        let control = ecu_control::sensors::slew::slew_step(input, control_state);
        control_state = control.next_state;

        let spec = sensor_slew_step(input, spec_state);
        spec_state = spec.next_state;

        assert_eq!(
            control.limited, spec.limited,
            "limited snapshot drift at t={t_us}"
        );
        assert_eq!(
            control.exceeded,
            spec.diagnostic == DiagnosticCode::SensorPlausibilityFault,
            "clamp flag drift at t={t_us}"
        );
    }
}
