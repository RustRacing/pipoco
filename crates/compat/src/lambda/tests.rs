use super::*;

fn running_conditions() -> (i16, u8, u16) {
    // CLT, TPS, RPM for normal running
    (70, 30, 2500)
}

#[test]
fn test_lambda_disabled_when_config_disabled() {
    let mut state = LambdaState::new();
    let config = LambdaConfig {
        enable: false,
        ..LambdaConfig::DEFAULT
    };

    let (clt, tps, rpm) = running_conditions();
    let stft = state.update(450, clt, tps, rpm, &config, 0);

    assert_eq!(stft, 0);
    assert!(!state.is_active());
    assert_eq!(state.disable_reason, Some(DisableReason::ConfigDisabled));
}

#[test]
fn test_lambda_disabled_when_cold() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;

    let stft = state.update(450, 40, 30, 2500, &config, 0); // 40°C < 60°C min

    assert_eq!(stft, 0);
    assert!(!state.is_active());
    assert_eq!(state.disable_reason, Some(DisableReason::CoolantTooLow));
}

#[test]
fn test_lambda_disabled_at_wot() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;

    let stft = state.update(450, 70, 90, 2500, &config, 0); // 90% > 80% max

    assert_eq!(stft, 0);
    assert!(!state.is_active());
    assert_eq!(state.disable_reason, Some(DisableReason::WideOpenThrottle));
}

#[test]
fn test_lambda_disabled_at_low_rpm() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;

    let stft = state.update(450, 70, 30, 800, &config, 0); // 800 < 1200 min

    assert_eq!(stft, 0);
    assert!(!state.is_active());
    assert_eq!(state.disable_reason, Some(DisableReason::RpmTooLow));
}

#[test]
fn test_lambda_activates_in_normal_conditions() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    state.update(450, clt, tps, rpm, &config, 0);

    assert!(state.is_active());
    assert_eq!(state.disable_reason, None);
}

#[test]
fn test_lambda_leans_on_rich_signal() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // Rich signal (high voltage, above 450mV threshold)
    let stft = state.update(800, clt, tps, rpm, &config, 0);

    // Should lean out (negative STFT)
    assert!(stft < 0);
}

#[test]
fn test_lambda_richens_on_lean_signal() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // Lean signal (low voltage, below 450mV threshold)
    let stft = state.update(100, clt, tps, rpm, &config, 0);

    // Should richen (positive STFT)
    assert!(stft > 0);
}

#[test]
fn test_lambda_deadband() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // Exactly at threshold - within deadband.
    state.update(450, clt, tps, rpm, &config, 0);

    assert!(state.in_deadband);
}

#[test]
fn test_lambda_authority_limits() {
    let mut state = LambdaState::new();
    let config = LambdaConfig {
        authority_max_x10: 100, // ±10%
        ..LambdaConfig::DEFAULT
    };
    let (clt, tps, rpm) = running_conditions();

    // Very lean signal - should hit authority limit.
    // Run multiple updates to accumulate integral.
    for i in 0..20 {
        state.update(100, clt, tps, rpm, &config, i * 200_000);
    }

    // Should be clamped to +10%.
    assert!(state.stft_x10 <= 100);
    assert!(state.stft_x10 >= -100);
}

#[test]
fn test_lambda_integral_accumulates() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // Persistent lean condition.
    state.update(200, clt, tps, rpm, &config, 0);
    let stft1 = state.stft_x10;

    state.update(200, clt, tps, rpm, &config, 200_000);
    let stft2 = state.stft_x10;

    // Integral should increase correction over time.
    assert!(stft2 >= stft1);
}

#[test]
fn test_lambda_reset() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // Build up some correction.
    for i in 0..5 {
        state.update(200, clt, tps, rpm, &config, i * 200_000);
    }
    assert!(state.stft_x10 > 0);
    assert!(state.integral > 0);

    // Reset.
    state.reset();

    assert_eq!(state.stft_x10, 0);
    assert_eq!(state.integral, 0);
}

#[test]
fn test_lambda_wideband_sensor() {
    let mut state = LambdaState::new();
    state.set_sensor_type(O2SensorType::Wideband);
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // 2500mV = 15.0 AFR (lean for stoich target).
    state.update(2500, clt, tps, rpm, &config, 0);

    // Should be calculating AFR.
    assert!(state.last_afr_x10 > 100);
}

#[test]
fn test_lambda_respects_update_interval() {
    let mut state = LambdaState::new();
    let config = LambdaConfig::DEFAULT;
    let (clt, tps, rpm) = running_conditions();

    // First update.
    state.update(200, clt, tps, rpm, &config, 0);
    let stft1 = state.stft_x10;

    // Update too soon - should return same value.
    let stft2 = state.update(200, clt, tps, rpm, &config, 50_000); // 50ms < 100ms interval

    assert_eq!(stft1, stft2);
}

// ========================================================================
// LTFT Tests
// ========================================================================

#[test]
fn test_ltft_cell_new() {
    let cell = LtftCell::new();
    assert_eq!(cell.trim_x10, 0);
    assert_eq!(cell.sample_count, 0);
    assert!(!cell.is_learned());
}

#[test]
fn test_ltft_cell_is_learned() {
    let mut cell = LtftCell::new();
    assert!(!cell.is_learned());

    cell.sample_count = ltft_constants::MIN_SAMPLES - 1;
    assert!(!cell.is_learned());

    cell.sample_count = ltft_constants::MIN_SAMPLES;
    assert!(cell.is_learned());
}

#[test]
fn test_ltft_table_find_bin() {
    // RPM bins: [1000, 2000, 3500, 5500]
    assert_eq!(LtftTable::find_bin(500, &ltft_constants::RPM_BINS), 0);
    assert_eq!(LtftTable::find_bin(1000, &ltft_constants::RPM_BINS), 0);
    assert_eq!(LtftTable::find_bin(1500, &ltft_constants::RPM_BINS), 0);
    assert_eq!(LtftTable::find_bin(2000, &ltft_constants::RPM_BINS), 1);
    assert_eq!(LtftTable::find_bin(3000, &ltft_constants::RPM_BINS), 1);
    assert_eq!(LtftTable::find_bin(3500, &ltft_constants::RPM_BINS), 2);
    assert_eq!(LtftTable::find_bin(5000, &ltft_constants::RPM_BINS), 2);
    assert_eq!(LtftTable::find_bin(5500, &ltft_constants::RPM_BINS), 3);
    assert_eq!(LtftTable::find_bin(7000, &ltft_constants::RPM_BINS), 3);
}

#[test]
fn test_ltft_table_learn_basic() {
    let mut table = LtftTable::new();

    // Learn with positive STFT (lean condition).
    table.learn(2500, 600, 50, 64, 100); // STFT = 5%

    // Should have learned something.
    assert!(table.cells[1][1].trim_x10 > 0);
    assert_eq!(table.cells[1][1].sample_count, 1);
}

#[test]
fn test_ltft_table_learn_convergence() {
    let mut table = LtftTable::new();
    let rate = 64u8; // Faster rate for test.
    let stft = 50i16; // Target 5% LTFT.

    // Learn many times.
    for _ in 0..100 {
        table.learn(2500, 600, stft, rate, 100);
    }

    // Should have converged close to STFT.
    let learned = table.cells[1][1].trim_x10;
    assert!(
        learned > 40 && learned < 60,
        "Expected ~50, got {}",
        learned
    );
}

#[test]
fn test_ltft_table_learn_clamping() {
    let mut table = LtftTable::new();

    // Try to learn extreme value.
    for _ in 0..200 {
        table.learn(2500, 600, 500, 64, 100); // Way above max.
    }

    // Should be clamped to max.
    assert!(table.cells[1][1].trim_x10 <= 100);
}

#[test]
fn test_ltft_table_lookup() {
    let mut table = LtftTable::new();
    // RPM bins: [1000, 2000, 3500, 5500], Load bins: [300, 600, 900, 1200].
    // 2500 RPM -> bin 1, 700 kPa x10 -> bin 1.
    table.cells[1][1].trim_x10 = 35;

    // Lookup should return the value for the correct bin.
    let ltft = table.lookup(2500, 700);
    assert_eq!(ltft, 35);
}

#[test]
fn test_ltft_table_reset() {
    let mut table = LtftTable::new();
    table.learn(2500, 600, 50, 64, 100);
    assert!(table.cells[1][1].sample_count > 0);

    table.reset();

    // All cells should be reset.
    for row in &table.cells {
        for cell in row {
            assert_eq!(cell.trim_x10, 0);
            assert_eq!(cell.sample_count, 0);
        }
    }
}

#[test]
fn test_ltft_table_serialization() {
    let mut table = LtftTable::new();
    table.cells[0][0].trim_x10 = 25;
    table.cells[0][0].sample_count = 100;
    table.cells[1][2].trim_x10 = -15;
    table.cells[1][2].sample_count = 50;

    let bytes = table.to_bytes();
    let restored = LtftTable::from_bytes(&bytes);

    assert_eq!(restored.cells[0][0].trim_x10, 25);
    assert_eq!(restored.cells[0][0].sample_count, 100);
    assert_eq!(restored.cells[1][2].trim_x10, -15);
    assert_eq!(restored.cells[1][2].sample_count, 50);
}

#[test]
fn test_ltft_table_learned_cell_count() {
    let mut table = LtftTable::new();
    assert_eq!(table.learned_cell_count(), 0);

    table.cells[0][0].sample_count = ltft_constants::MIN_SAMPLES;
    assert_eq!(table.learned_cell_count(), 1);

    table.cells[1][1].sample_count = ltft_constants::MIN_SAMPLES;
    table.cells[2][2].sample_count = ltft_constants::MIN_SAMPLES;
    assert_eq!(table.learned_cell_count(), 3);
}

#[test]
fn test_ltft_state_disabled_when_config_off() {
    let mut state = LtftState::new();
    let config = LtftConfig {
        enable: false,
        ..LtftConfig::DEFAULT
    };

    let should = state.should_learn(2500, 600, 80, 10, true, &config, 0);
    assert!(!should);
    assert!(!state.learning_active);
}

#[test]
fn test_ltft_state_disabled_when_cold() {
    let mut state = LtftState::new();
    let config = LtftConfig::DEFAULT;

    // Cold engine (below 70°C).
    let should = state.should_learn(2500, 600, 50, 10, true, &config, 0);
    assert!(!should);
}

#[test]
fn test_ltft_state_disabled_when_lambda_inactive() {
    let mut state = LtftState::new();
    let config = LtftConfig::DEFAULT;

    let should = state.should_learn(2500, 600, 80, 10, false, &config, 0);
    assert!(!should);
}

#[test]
fn test_ltft_state_steady_state_detection() {
    let mut state = LtftState::new();
    let config = LtftConfig::DEFAULT;

    // First update - not in steady state yet.
    state.should_learn(2500, 600, 80, 10, true, &config, 0);
    assert!(!state.learning_active);

    // Same conditions - now entering steady state.
    state.should_learn(2500, 600, 80, 10, true, &config, 1_000_000);
    assert!(state.in_steady_state);
    assert!(!state.learning_active); // Not enough time yet.

    // After steady state time elapsed.
    let should = state.should_learn(2500, 600, 80, 10, true, &config, 3_000_000);
    assert!(should);
    assert!(state.learning_active);
}

#[test]
fn test_ltft_state_steady_state_broken_by_rpm_change() {
    let mut state = LtftState::new();
    let config = LtftConfig::DEFAULT;

    // Enter steady state.
    state.should_learn(2500, 600, 80, 10, true, &config, 0);
    state.should_learn(2500, 600, 80, 10, true, &config, 1_000_000);
    assert!(state.in_steady_state);

    // Large RPM change breaks steady state.
    state.should_learn(3000, 600, 80, 10, true, &config, 1_500_000);
    assert!(!state.in_steady_state);
}

#[test]
fn test_ltft_manager_update() {
    let mut manager = LtftManager::new();
    manager.config.learn_rate = 64; // Faster for test.
    manager.config.steady_state_time_us = 0; // Instant for test.

    // Update with STFT.
    let ltft = manager.update(2500, 600, 80, 20, true, 1_000_000);
    assert_eq!(ltft, 0); // First update, no learning yet.

    // Second update should learn.
    manager.update(2500, 600, 80, 20, true, 2_000_000);
    let ltft = manager.table.lookup(2500, 600);
    assert!(ltft > 0);
}

#[test]
fn test_ltft_manager_total_trim() {
    let mut manager = LtftManager::new();
    manager.table.cells[1][1].trim_x10 = 30; // 3% LTFT.

    let stft = 20i16; // 2% STFT.
    let total = manager.get_total_trim(stft, 2500, 600);

    assert_eq!(total, 50); // 5% total.
}

#[test]
fn test_ltft_manager_total_trim_clamping() {
    let mut manager = LtftManager::new();
    manager.table.cells[1][1].trim_x10 = 100; // 10% LTFT.

    let stft = 150i16; // 15% STFT.
    let total = manager.get_total_trim(stft, 2500, 600);

    // Should be clamped to 20%.
    assert_eq!(total, 200);
}

#[test]
fn test_ltft_manager_reset() {
    let mut manager = LtftManager::new();
    manager.config.steady_state_time_us = 0;
    manager.update(2500, 600, 80, 20, true, 1_000_000);
    manager.update(2500, 600, 80, 20, true, 2_000_000);
    assert!(manager.table.cells[1][1].sample_count > 0);

    manager.reset();

    assert_eq!(manager.table.cells[1][1].sample_count, 0);
    assert_eq!(manager.table.cells[1][1].trim_x10, 0);
    assert_eq!(manager.state.learn_count, 0);
}

#[test]
fn test_ltft_manager_persistence() {
    let mut manager = LtftManager::new();
    manager.table.cells[0][0].trim_x10 = 45;
    manager.table.cells[0][0].sample_count = 100;

    let bytes = manager.save_to_bytes();

    let mut manager2 = LtftManager::new();
    manager2.load_from_bytes(&bytes);

    assert_eq!(manager2.table.cells[0][0].trim_x10, 45);
    assert_eq!(manager2.table.cells[0][0].sample_count, 100);
}
