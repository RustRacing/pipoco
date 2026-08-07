#[cfg(test)]
use super::EcuState;
use crate::compat::{DiagnosticFlags, RuntimeSignals};
use crate::constants::fuel::{DEFAULT_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US, MIN_PULSE_WIDTH_US};
use crate::*;

#[test]
fn test_scale_u16_normal() {
    assert_eq!(scale_u16(1000, 150), 1500); // 1.5x
    assert_eq!(scale_u16(1000, 80), 800); // 0.8x
    assert_eq!(scale_u16(1000, 100), 1000); // 1.0x
    assert_eq!(scale_u16(500, 200), 1000); // 2.0x
}

#[test]
fn test_scale_u16_saturation() {
    // Test overflow protection
    assert_eq!(scale_u16(u16::MAX, 200), u16::MAX); // Would overflow
    assert_eq!(scale_u16(50000, 200), u16::MAX); // Would overflow
}

#[test]
fn test_fuel_calculation_clamping() {
    let mut state = EcuState::new();

    // Test minimum clamping (with very low correction)
    state.corrections_mut().clt = 10; // 0.1x (very low)
    let pw = state.calculate_fuel(3000, 60);
    assert_eq!(pw, MIN_PULSE_WIDTH_US);

    // Test maximum clamping (with very high base value and correction)
    // First set a high base value in the table
    // 3000 RPM maps to RPM bin index 5, 60 kPa maps to load bin index 4
    // Table is [load_idx][rpm_idx]
    state.config.ipw_table[4][5] = 15000; // 15ms base
    state.corrections_mut().clt = 255; // 2.55x (very high)
    state.corrections_mut().iat = 255;
    state.corrections_mut().vbatt = 255;
    // This should result in: 15000 * 2.55 * 2.55 * 2.55 = 249,146 which exceeds MAX
    let pw = state.calculate_fuel(3000, 60); // Maps to bin [4][5]
    assert_eq!(pw, MAX_PULSE_WIDTH_US);
}

#[test]
fn test_process_sensor_update_keeps_getters_in_sync() {
    let mut state = EcuState::new();

    let (map, tps) = state.process_sensor_update(Micros::new(0), Kpa10::new(777), 42);

    assert_eq!(map.raw(), 777);
    assert_eq!(tps, 42);
    assert_eq!(state.map_kpa_x10(), 777);
    assert_eq!(state.tps_percent(), 42);
}

#[test]
fn test_runtime_signals_view_tracks_scalar_mirrors() {
    let mut state = EcuState::new();
    state.set_rpm(2750);
    state.set_synced(true);
    state.set_tooth_count(7);
    state.set_battery_voltage_mv(12_450);
    state.set_clt_x10(830);
    state.set_iat_x10(410);
    state.set_tps_percent(17);
    state.set_map_kpa_x10(812);

    let runtime = state.runtime_signals();
    assert_eq!(
        runtime,
        RuntimeSignals {
            rpm: 2750,
            synced: true,
            tooth_count: 7,
            battery_voltage_mv: 12_450,
            clt_x10: 830,
            iat_x10: 410,
            tps_percent: 17,
            map_kpa_x10: 812,
        }
    );
    assert_eq!(state.current_rpm(), runtime.rpm);
    assert_eq!(state.current_synced(), runtime.synced);
    assert_eq!(state.rpm(), runtime.rpm);
    assert_eq!(state.synced(), runtime.synced);
    assert_eq!(state.tooth_count(), runtime.tooth_count);
    assert_eq!(state.battery_voltage_mv(), runtime.battery_voltage_mv);
    assert_eq!(state.clt_x10(), runtime.clt_x10);
    assert_eq!(state.iat_x10(), runtime.iat_x10);
    assert_eq!(state.tps_percent(), runtime.tps_percent);
    assert_eq!(state.map_kpa_x10(), runtime.map_kpa_x10);
    // Single source of truth: runtime_signals() reads the EcuInputs mirror.
    assert_eq!(state.runtime_signals(), runtime);
}

#[test]
fn test_diagnostic_and_safety_views_track_legacy_accessors() {
    let mut state = EcuState::new();
    state.set_emergency_trigger_map_oob(true);
    state.set_emergency_trigger_tps_oob(false);
    state.set_emergency_mode(true);
    state.rev_limiter_state.active = true;
    state.rev_limiter_state.fuel_cut_percent = 100;
    state.rev_limiter_state.ignition_retard = 12;

    let diagnostic = state.diagnostic_flags();
    let safety = state.safety_status();

    assert_eq!(
        diagnostic,
        DiagnosticFlags {
            emergency_trigger_map_oob: true,
            emergency_trigger_tps_oob: false,
            emergency_mode: true,
        }
    );
    assert_eq!(state.current_fault_flags(), (true, false, true));
    assert!(safety.fuel_cut_active);
    assert!(safety.spark_cut_active);
    assert!(state.fuel_cut_active());
    assert!(state.spark_cut_active());
}

#[test]
fn test_clear_diagnostics_resets_live_faults_and_log() {
    let mut state = EcuState::new();
    state.set_emergency_trigger_map_oob(true);
    state.set_emergency_trigger_tps_oob(true);
    state.set_emergency_mode(true);
    state.diag_map.latch(Micros::new(10));
    state.diag_tps.latch(Micros::new(20));
    state.diag_cam.latch(Micros::new(30));
    state.diag_log_mut().push(diag::DiagEvent {
        code: diag::DiagCode::MapRange,
        timestamp: Micros::new(100),
        source: diag::DiagSource::Sensor,
        context: Some(100),
        start_us: 10,
        end_us: 40,
    });
    state.diag_log_mut().push(diag::DiagEvent {
        code: diag::DiagCode::CamMissing,
        timestamp: Micros::new(200),
        source: diag::DiagSource::Trigger,
        context: None,
        start_us: 30,
        end_us: 60,
    });

    let summary = state.clear_diagnostics();

    assert_eq!(
        summary,
        diag::DiagClearSummary {
            cleared_active_count: 3,
            cleared_log_entries: 2,
            emergency_cleared: true,
        }
    );
    assert!(!state.diag_map.is_active());
    assert!(!state.diag_tps.is_active());
    assert!(!state.diag_cam.is_active());
    assert!(!state.emergency_mode());
    assert!(state.diag_log().events.iter().all(|entry| entry.is_none()));
    assert_eq!(state.diag_log().head, 0);
    assert!(state.emergency_trigger_map_oob());
    assert!(state.emergency_trigger_tps_oob());
}

#[test]
fn test_clear_diagnostics_is_noop_for_clean_state() {
    let mut state = EcuState::new();

    let summary = state.clear_diagnostics();

    assert_eq!(summary, diag::DiagClearSummary::default());
    assert!(!state.diag_map.is_active());
    assert!(!state.diag_tps.is_active());
    assert!(!state.diag_cam.is_active());
    assert!(!state.emergency_mode());
    assert!(state.diag_log().events.iter().all(|entry| entry.is_none()));
}

#[test]
fn test_fuel_calculation_normal() {
    let state = EcuState::new();

    // With default corrections (1.0x), should return table value
    let pw = state.calculate_fuel(3000, 60);
    assert_eq!(pw, DEFAULT_PULSE_WIDTH_US);
}

#[test]
fn test_final_pw_base_only() {
    let state = EcuState::new();
    let base = state.calculate_fuel(3000, 60);

    assert_eq!(
        state.final_pw(Rpm::new(3000), Kpa10::new(60)),
        Micros::new(base as u32)
    );
}

#[test]
fn test_final_pw_wue_active() {
    let mut state = EcuState::new();
    state.derived.wue_percent = 20;
    state.derived.ase_percent = 0;
    state.derived.ae_percent = 0;
    state.set_stft_x10(0);
    state.set_fuel_mult_x100(100);

    let base = state.calculate_fuel(3000, 60) as u32;
    assert_eq!(
        state.final_pw(Rpm::new(3000), Kpa10::new(60)),
        Micros::new((base * 120) / 100)
    );
}

#[test]
fn test_final_pw_stft_plus_four_percent() {
    let mut state = EcuState::new();
    state.derived.wue_percent = 0;
    state.derived.ase_percent = 0;
    state.derived.ae_percent = 0;
    state.set_stft_x10(40);
    state.set_fuel_mult_x100(100);

    let base = state.calculate_fuel(3000, 60) as u32;
    assert_eq!(
        state.final_pw(Rpm::new(3000), Kpa10::new(60)),
        Micros::new((base * 104) / 100)
    );
}

#[test]
fn test_final_pw_torque_multiplier_70_percent() {
    let mut state = EcuState::new();
    state.derived.wue_percent = 0;
    state.derived.ase_percent = 0;
    state.derived.ae_percent = 0;
    state.set_stft_x10(0);
    state.set_fuel_mult_x100(70);

    let base = state.calculate_fuel(3000, 60) as u32;
    assert_eq!(
        state.final_pw(Rpm::new(3000), Kpa10::new(60)),
        Micros::new((base * 70) / 100)
    );
}

#[test]
fn test_linear_table_initialization() {
    let mut state = EcuState::new();
    state.init_linear_table();

    // Verify table has been populated
    // First cell should be base + 0 - 0
    assert_eq!(state.config.ipw_table[0][0], DEFAULT_PULSE_WIDTH_US);

    // Last cell should be base + 750 - 150
    let expected = DEFAULT_PULSE_WIDTH_US + 750 - 150;
    assert_eq!(state.config.ipw_table[15][15], expected);

    // Verify middle cell has reasonable value
    assert!(state.config.ipw_table[8][8] > DEFAULT_PULSE_WIDTH_US);
}

// --- Knock Integration Tests ---

#[test]
fn test_ecustate_knock_process_sample() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.knock_controller.config.enable = true;
    state.knock_controller.config.threshold = 100;
    state.knock_controller.config.debounce_count = 1; // Immediate detection

    // Below threshold - no knock
    let detected = state.process_knock_sample(0, 50, 80, 1000);
    assert!(!detected);
    assert!(!state.has_knock_retard());

    // Above threshold - knock detected
    let detected = state.process_knock_sample(0, 150, 80, 2000);
    assert!(detected);
    assert!(state.has_knock_retard());

    // Check that diag log contains knock event
    assert!(state
        .diag_log()
        .events
        .iter()
        .filter_map(|e| e.as_ref())
        .any(|e| e.code == diag::DiagCode::KnockDetected));
}

#[test]
fn test_ecustate_knock_affects_timing() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.knock_controller.config.enable = true;
    state.knock_controller.config.threshold = 100;
    state.knock_controller.config.debounce_count = 1;
    state.knock_controller.config.retard_step_x10 = 30; // 3 degrees per knock

    // Get base timing
    let base = state.calculate_ignition_timing_with_limiter_cyl(3000, 80, 0);

    // Trigger knock
    state.process_knock_sample(0, 200, 80, 1000);

    // Check timing is retarded
    let after_knock = state.calculate_ignition_timing_with_limiter_cyl(3000, 80, 0);
    assert!(after_knock < base, "Timing should be retarded after knock");
    assert_eq!(base - after_knock, 3, "Should retard by 3 degrees");
}

#[test]
fn test_ecustate_knock_recovery() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.knock_controller.config.enable = true;
    state.knock_controller.config.threshold = 100;
    state.knock_controller.config.debounce_count = 1;
    state.knock_controller.config.retard_step_x10 = 50; // 5 degrees
    state.knock_controller.config.recovery_rate_x10 = 100; // 10 degrees/sec for faster test

    // Trigger knock
    state.process_knock_sample(0, 200, 80, 0);
    assert!(state.has_knock_retard());

    // Recover over time (need enough time for 50 x10 units at 100 x10/sec = 0.5 sec)
    // With 100ms intervals, need 5 calls
    for i in 0..6 {
        state.update_knock_recovery(100_000 + i * 100_000);
    }

    // Should have recovered
    assert!(!state.has_knock_retard());
}

#[test]
fn test_ecustate_knock_disabled_conditions() {
    let mut state = EcuState::new();
    state.knock_controller.config.enable = true;
    state.knock_controller.config.threshold = 100;
    state.knock_controller.config.debounce_count = 1; // Immediate detection
    state.knock_controller.config.min_rpm = 2000;
    state.knock_controller.config.min_clt_c = 60;

    // Low RPM - disabled
    state.set_rpm(1500);
    let detected = state.process_knock_sample(0, 200, 80, 1000);
    assert!(!detected);

    // Cold engine - disabled
    state.set_rpm(3000);
    let detected = state.process_knock_sample(0, 200, 50, 2000);
    assert!(!detected);

    // Warm engine, good RPM - enabled
    let detected = state.process_knock_sample(0, 200, 80, 3000);
    assert!(detected);
}

// --- LTFT Integration Tests ---

#[test]
fn test_ecustate_ltft_learning() {
    let mut state = EcuState::new();
    state.set_rpm(2500);
    state.set_map_kpa_x10(600);
    state.lambda_state.active = true;
    state.set_stft_x10(30); // 3% rich
    state.ltft_manager_mut().config.enable = true;

    // Initial trim should be 0
    let trim = state.get_total_fuel_trim();
    assert_eq!(trim, 30); // Just STFT

    // Update LTFT several times with steady conditions
    for i in 0..20 {
        state.update_ltft(80, i * 1_000_000);
    }

    // Should be learning
    assert!(state.is_ltft_learning() || state.ltft_learned_cell_count() > 0);
}

#[test]
fn test_ecustate_ltft_disabled_cold() {
    let mut state = EcuState::new();
    state.set_rpm(2500);
    state.set_map_kpa_x10(600);
    state.lambda_state.active = true;
    state.set_stft_x10(30);
    state.ltft_manager_mut().config.enable = true;
    state.ltft_manager_mut().config.min_clt_c = 70;

    // Cold engine - LTFT should not learn
    for i in 0..20 {
        state.update_ltft(50, i * 1_000_000);
    }

    assert_eq!(state.ltft_learned_cell_count(), 0);
}

#[test]
fn test_ecustate_ltft_reset() {
    let mut state = EcuState::new();
    state.set_rpm(2500);
    state.set_map_kpa_x10(600);
    state.lambda_state.active = true;
    state.set_stft_x10(30);
    state.ltft_manager_mut().config.enable = true;

    // Learn for a while
    for i in 0..20 {
        state.update_ltft(80, i * 1_000_000);
    }

    // Reset
    state.reset_ltft();

    // All cells should be cleared
    assert_eq!(state.ltft_learned_cell_count(), 0);
}

// --- Torque Integration Tests ---

#[test]
fn test_ecustate_torque_driver_request() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_map_kpa_x10(800);

    // First update to get max available
    state.update_torque(25);

    // Driver pedal at 50%
    state.request_driver_torque(50, 1000);
    let torque = state.update_torque(25);

    // Should have ~50% of max available
    assert!(torque > 0);
    assert!(torque <= state.torque_controller.max_available_x10);
}

#[test]
fn test_ecustate_torque_safety_limits() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_map_kpa_x10(800);

    // Update and request full power
    state.update_torque(25);
    state.request_driver_torque(100, 1000);
    let full_power = state.update_torque(25);

    // Activate rev limiter
    state.rev_limiter_state.active = true;
    state.apply_safety_torque_limits(2000);
    let limited = state.update_torque(25);

    // Should be severely limited
    assert!(limited < full_power, "Rev limiter should limit torque");
    assert!(state.is_torque_limited());
}

#[test]
fn test_ecustate_torque_actuators() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_map_kpa_x10(800);

    // Update and request 50%
    state.update_torque(25);
    state.request_driver_torque(50, 1000);
    state.update_torque(25);

    let targets = state.get_torque_actuators();

    // Should have some fuel reduction or timing retard if limited
    // At 50% pedal with full MAP, driver usually gets what they want
    assert!(targets.fuel_mult_x100 <= 100);
}

#[test]
fn test_ecustate_torque_zero_rpm() {
    let mut state = EcuState::new();
    state.set_rpm(0);
    state.set_map_kpa_x10(800);

    // Should handle 0 RPM gracefully
    let torque = state.update_torque(25);
    assert_eq!(torque, 0); // No torque at 0 RPM
}

// --- Cross-Module Integration Tests ---

#[test]
fn test_integration_knock_reduces_torque() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_map_kpa_x10(800);
    state.knock_controller.config.enable = true;
    state.knock_controller.config.threshold = 100;
    state.knock_controller.config.debounce_count = 1;
    state.knock_controller.config.retard_step_x10 = 50; // 5 degrees

    // Update torque to get max available
    state.update_torque(25);
    state.request_driver_torque(100, 1000);
    let base_torque = state.update_torque(25);

    // Trigger knock
    state.process_knock_sample(0, 200, 80, 2000);
    assert!(state.has_knock_retard());

    // Submit knock-based torque request
    let knock_retard = state.knock_controller.state.get_retard(0);
    state
        .torque_controller
        .arbiter
        .request(torque::request::knock_torque_request(
            knock_retard,
            state.torque_controller.max_available_x10,
            3000,
        ));
    let reduced_torque = state
        .torque_controller
        .arbiter
        .arbitrate(state.torque_controller.max_available_x10);

    // Knock should reduce available torque
    assert!(
        reduced_torque < base_torque,
        "Knock should reduce arbitrated torque"
    );
}

#[test]
fn test_integration_torque_affects_actuators() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_map_kpa_x10(800);

    // Full power request
    state.update_torque(25);
    state.request_driver_torque(100, 1000);
    state.update_torque(25);

    let full_power_targets = state.get_torque_actuators();

    // Activate rev limiter
    state.rev_limiter_state.active = true;
    state.apply_safety_torque_limits(2000);
    state.update_torque(25);

    let limited_targets = state.get_torque_actuators();

    // Rev limiter should cause actuator changes
    assert!(
        limited_targets.fuel_cut
            || limited_targets.timing_reduced
            || limited_targets.fuel_mult_x100 < full_power_targets.fuel_mult_x100,
        "Rev limiter should cause actuator intervention"
    );
}

#[test]
fn test_integration_limp_mode_propagation() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_map_kpa_x10(800);

    // Full power request
    state.update_torque(25);
    state.request_driver_torque(100, 1000);
    let full_power = state.update_torque(25);

    // Trigger load failure -> limp mode
    state.load_failure_tracker.in_limp = true;
    state.apply_safety_torque_limits(2000);
    let limp_torque = state.update_torque(25);

    // Limp mode should limit torque to ~30%
    assert!(
        limp_torque < full_power / 2,
        "Limp mode should severely limit torque"
    );
    assert!(state.is_torque_limited());
}

#[test]
fn test_integration_multiple_safety_systems() {
    let mut state = EcuState::new();
    state.set_rpm(6500);
    state.set_map_kpa_x10(900);
    state.knock_controller.config.enable = true;
    state.knock_controller.config.threshold = 100;
    state.knock_controller.config.debounce_count = 1;
    state.rev_limiter_config_mut().max_rpm = 6500;

    // Driver wants full power
    state.update_torque(25);
    state.request_driver_torque(100, 1000);

    // Trigger knock
    state.process_knock_sample(0, 200, 80, 2000);

    // Update rev limiter (at hard limit)
    let rev_config = *state.rev_limiter_config();
    rev_limiter::update_limiter(state.rpm(), &rev_config, &mut state.rev_limiter_state);

    // Apply safety limits
    state.apply_safety_torque_limits(3000);

    // Get final timing with all corrections
    let final_timing = state.calculate_ignition_timing_with_limiter_cyl(state.rpm(), 80, 0);
    let base_timing = state.calculate_ignition_timing(state.rpm(), 80);

    // Timing should be reduced by both knock and rev limiter
    assert!(
        final_timing < base_timing,
        "Safety systems should reduce timing"
    );
}

#[test]
fn test_integration_lambda_ltft_combined_trim() {
    let mut state = EcuState::new();
    state.set_rpm(2500);
    state.set_map_kpa_x10(600);
    state.lambda_state.active = true;
    state.set_stft_x10(30); // 3% STFT
    state.ltft_manager_mut().config.enable = true;

    // Pre-learn some LTFT (stft=20, rate=50, max_trim=200)
    let rpm = state.rpm();
    let map_kpa_x10 = state.map_kpa_x10();
    state
        .ltft_manager_mut()
        .table
        .learn(rpm, map_kpa_x10, 20, 50, 200);

    // Get combined trim
    let total_trim = state.get_total_fuel_trim();

    // Should combine STFT + LTFT
    assert!(total_trim > 30, "Combined trim should include LTFT");
    assert!(total_trim <= 200, "Combined trim should be clamped");
}

#[test]
fn test_integration_sync_loss_disables_injection() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_synced(true);

    // Should allow injection when synced
    assert!(state.should_inject_with_all_safety(0));

    // Record sync loss
    let should_shutdown = state.record_sync_loss(1000);

    if !should_shutdown {
        // First loss doesn't shutdown, but should still not inject
        assert!(!state.synced());
        assert!(
            !state.should_inject_with_all_safety(0),
            "Should not inject without sync"
        );
    }
}

#[test]
fn test_integration_voltage_affects_safety() {
    let mut state = EcuState::new();
    state.set_rpm(3000);
    state.set_synced(true);

    // Normal voltage - should inject
    assert!(state.should_inject_with_all_safety(0));

    // Critical low voltage
    state.voltage_monitor.limp_active = true;

    // Apply safety torque limits
    state.update_torque(25);
    state.request_driver_torque(100, 1000);
    state.apply_safety_torque_limits(2000);
    let _limited_torque = state.update_torque(25);

    // Limp mode should be active
    assert!(state.is_torque_limited());
}
