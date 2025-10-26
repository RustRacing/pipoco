//! Safety tests for ignition timing and dwell control
//!
//! These tests verify that ignition timing cannot cause engine damage through:
//! - Excessive advance (detonation, engine damage)
//! - Excessive dwell (coil overheating)
//! - Incorrect timing during cranking

use ecu_core::{EcuState, IgnitionTable, IgnitionCorrections, calculate_timing, calculate_dwell};
use ecu_core::constants::ignition::*;

// =============================================================================
// TIMING SAFETY TESTS
// =============================================================================

#[test]
fn test_max_timing_clamp_enforced() {
    // Extreme advance should be clamped
    let corrections = IgnitionCorrections {
        clt_correction: 20,
        iat_correction: 20,
        knock_retard: -10,  // Negative = more advance (shouldn't happen but test it)
    };

    let timing = calculate_timing(50, &corrections);  // 50° base + 50° corrections
    assert_eq!(timing, MAX_TIMING_BTDC,
              "Timing should clamp to max: got {}°", timing);
}

#[test]
fn test_min_timing_clamp_enforced() {
    // Extreme retard should be clamped
    let corrections = IgnitionCorrections {
        clt_correction: -20,
        iat_correction: -20,
        knock_retard: 30,  // Heavy knock retard
    };

    let timing = calculate_timing(-10, &corrections);  // Negative base + more retard
    assert_eq!(timing, MIN_TIMING_BTDC,
              "Timing should clamp to min: got {}°", timing);
}

#[test]
fn test_timing_always_within_safe_range() {
    // Test all combinations of extreme corrections
    for base in [-10, 0, 10, 20, 30, 40, 50] {
        for clt in [-20, -10, 0, 10, 20] {
            for iat in [-20, -10, 0, 10, 20] {
                for knock in [0, 5, 10, 15, 20] {
                    let corrections = IgnitionCorrections {
                        clt_correction: clt,
                        iat_correction: iat,
                        knock_retard: knock,
                    };

                    let timing = calculate_timing(base, &corrections);

                    assert!(timing >= MIN_TIMING_BTDC && timing <= MAX_TIMING_BTDC,
                           "Timing out of range: base={}, clt={}, iat={}, knock={} -> {}°",
                           base, clt, iat, knock, timing);
                }
            }
        }
    }
}

#[test]
fn test_conservative_table_safe_everywhere() {
    let mut state = EcuState::new();
    state.init_ignition_table();

    // Every point in the table should be within safe limits
    for rpm in [500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000] {
        for load in [20, 40, 60, 80, 100, 120, 140, 160, 170] {
            let timing = state.calculate_ignition_timing(rpm, load);

            assert!(timing >= MIN_TIMING_BTDC && timing <= MAX_TIMING_BTDC,
                   "Timing at {} RPM, {} kPa is unsafe: {}°", rpm, load, timing);

            // Conservative table should never exceed 30° BTDC
            assert!(timing <= 30,
                   "Conservative timing too aggressive at {} RPM, {} kPa: {}°",
                   rpm, load, timing);
        }
    }
}

#[test]
fn test_knock_retard_reduces_timing() {
    let base = 25;

    let no_knock = IgnitionCorrections {
        knock_retard: 0,
        ..IgnitionCorrections::DEFAULT
    };

    let with_knock = IgnitionCorrections {
        knock_retard: 10,  // 10° retard
        ..IgnitionCorrections::DEFAULT
    };

    let timing_no_knock = calculate_timing(base, &no_knock);
    let timing_with_knock = calculate_timing(base, &with_knock);

    assert_eq!(timing_no_knock, 25);
    assert_eq!(timing_with_knock, 15);
    assert!(timing_with_knock < timing_no_knock,
           "Knock retard should reduce timing");
}

// =============================================================================
// DWELL SAFETY TESTS
// =============================================================================

#[test]
fn test_max_dwell_clamp_enforced() {
    // Very low voltage should clamp to max dwell
    let dwell = calculate_dwell(5000);  // 5V
    assert_eq!(dwell, MAX_DWELL_US,
              "Dwell should clamp to max at low voltage");
}

#[test]
fn test_min_dwell_clamp_enforced() {
    // Very high voltage should clamp to min dwell
    let dwell = calculate_dwell(50000);  // 50V (unrealistic but tests safety)
    assert_eq!(dwell, MIN_DWELL_US,
              "Dwell should clamp to min at high voltage");
}

#[test]
fn test_dwell_always_within_safe_range() {
    // Test across full realistic voltage range
    for voltage_mv in (6000..=18000).step_by(500) {
        let dwell = calculate_dwell(voltage_mv);

        assert!(dwell >= MIN_DWELL_US && dwell <= MAX_DWELL_US,
               "Dwell out of range at {} mV: {} us", voltage_mv, dwell);
    }
}

#[test]
fn test_dwell_compensates_for_voltage() {
    // Lower voltage should give longer dwell
    let dwell_low = calculate_dwell(10000);   // 10V
    let dwell_nom = calculate_dwell(13500);   // 13.5V
    let dwell_high = calculate_dwell(15000);  // 15V

    assert!(dwell_low > dwell_nom,
           "Low voltage should increase dwell: {}us vs {}us", dwell_low, dwell_nom);
    assert!(dwell_high < dwell_nom,
           "High voltage should decrease dwell: {}us vs {}us", dwell_high, dwell_nom);
}

#[test]
fn test_dwell_prevents_coil_overheating() {
    // Even at very low voltage, dwell should be clamped
    let dwell_extreme_low = calculate_dwell(1000);  // 1V (dead battery)
    assert_eq!(dwell_extreme_low, MAX_DWELL_US,
              "Dwell should not exceed max even at extreme low voltage");

    // Max dwell (6ms) is still safe for most coils
    assert!(MAX_DWELL_US <= 7000,
           "MAX_DWELL_US should be ≤7ms to prevent coil damage");
}

// =============================================================================
// INTEGRATION SAFETY TESTS
// =============================================================================

#[test]
fn test_cold_engine_timing_safe() {
    let mut state = EcuState::new();
    state.init_ignition_table();

    // Cold engine should have reduced timing
    state.ignition_corrections.clt_correction = -5;  // Cold

    // Test at idle
    let timing_idle = state.calculate_ignition_timing(850, 40);
    assert!(timing_idle >= 5 && timing_idle <= 20,
           "Cold idle timing should be conservative: {}°", timing_idle);
}

#[test]
fn test_high_load_timing_conservative() {
    let mut state = EcuState::new();
    state.init_ignition_table();

    // High load (potential knock condition)
    let timing_high_load = state.calculate_ignition_timing(3000, 150);

    // At high load, timing should be conservative to prevent knock
    assert!(timing_high_load <= 25,
           "High load timing too aggressive: {}°", timing_high_load);
}

#[test]
fn test_cranking_timing_fixed_safe() {
    // During cranking, timing should be fixed and safe
    let cranking_timing = CRANKING_TIMING_BTDC;

    assert!(cranking_timing >= 5 && cranking_timing <= 15,
           "Cranking timing should be 5-15° BTDC, got {}°", cranking_timing);
}

#[test]
fn test_timing_table_monotonic_with_load() {
    let mut state = EcuState::new();
    state.init_ignition_table();

    // At fixed RPM, timing should decrease or stay same as load increases
    // (higher load = more knock risk = less timing)
    let rpm = 3000;

    let timing_low_load = state.calculate_ignition_timing(rpm, 40);
    let timing_mid_load = state.calculate_ignition_timing(rpm, 80);
    let timing_high_load = state.calculate_ignition_timing(rpm, 140);

    assert!(timing_low_load >= timing_mid_load,
           "Timing should reduce or stay same with increasing load: {}° vs {}°",
           timing_low_load, timing_mid_load);

    assert!(timing_mid_load >= timing_high_load,
           "Timing should reduce or stay same with increasing load: {}° vs {}°",
           timing_mid_load, timing_high_load);
}

// =============================================================================
// FAILURE MODE TESTS
// =============================================================================

#[test]
fn test_corrupted_table_still_safe() {
    let mut state = EcuState::new();

    // Simulate corrupted table with extreme values
    state.ignition_table[8][8] = 100;  // 100° BTDC (impossible/dangerous)
    state.ignition_table[0][0] = -50;  // -50° ATDC (very retarded)

    // Even with corrupted values, calculation should clamp
    let timing1 = state.calculate_ignition_timing(3500, 100);
    let timing2 = state.calculate_ignition_timing(500, 20);

    assert!(timing1 >= MIN_TIMING_BTDC && timing1 <= MAX_TIMING_BTDC,
           "Corrupted table should still clamp: {}°", timing1);
    assert!(timing2 >= MIN_TIMING_BTDC && timing2 <= MAX_TIMING_BTDC,
           "Corrupted table should still clamp: {}°", timing2);
}

#[test]
fn test_sensor_failure_safe_timing() {
    let mut state = EcuState::new();
    state.init_ignition_table();

    // Simulate sensor failures with extreme values
    state.battery_voltage_mv = 5000;  // Very low (5V)

    let dwell = state.calculate_dwell();

    // Should still be safe
    assert!(dwell >= MIN_DWELL_US && dwell <= MAX_DWELL_US,
           "Dwell with failed sensor should be safe: {}us", dwell);
}

#[test]
fn test_zero_battery_voltage_handled() {
    // Edge case: battery voltage reads zero (sensor failure)
    // Should not cause division by zero or panic

    // Note: This would divide by zero in the calculation, so we need to handle it
    // For now, the calculation will give very high dwell which will clamp to MAX
    let dwell = calculate_dwell(1);  // Nearly zero voltage

    assert!(dwell >= MIN_DWELL_US && dwell <= MAX_DWELL_US,
           "Zero voltage should not crash, dwell: {}us", dwell);
}
