use super::*;

#[test]
fn test_no_fault_normal_conditions() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Normal: 50% TPS, 70 kPa
    let fault = state.check(50, 700, 3000, &config, 0);
    assert_eq!(fault, PlausibilityFault::None);
    assert!(!state.has_fault());
}

#[test]
fn test_no_fault_at_low_rpm() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Implausible values but at low RPM - should not fault
    let fault = state.check(90, 200, 500, &config, 0);
    assert_eq!(fault, PlausibilityFault::None);
}

#[test]
fn test_tps_high_map_low_fault() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // TPS high (85%), MAP low (25 kPa) at running RPM
    // First check - starts debounce
    let fault = state.check(85, 250, 3000, &config, 0);
    assert_eq!(fault, PlausibilityFault::None); // Not confirmed yet
    assert_eq!(state.pending_fault, PlausibilityFault::TpsHighMapLow);

    // Before debounce time
    let fault = state.check(85, 250, 3000, &config, 400_000);
    assert_eq!(fault, PlausibilityFault::None);

    // After debounce time - fault confirmed
    let fault = state.check(85, 250, 3000, &config, 600_000);
    assert_eq!(fault, PlausibilityFault::TpsHighMapLow);
    assert!(state.has_fault());
}

#[test]
fn test_tps_low_map_high_fault() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // TPS low (5%), MAP high (96 kPa) at running RPM
    state.check(5, 960, 3000, &config, 0);
    state.check(5, 960, 3000, &config, 600_000);

    assert_eq!(state.confirmed_fault, PlausibilityFault::TpsLowMapHigh);
}

#[test]
fn test_fault_clears_with_debounce() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Create fault
    state.check(85, 250, 3000, &config, 0);
    state.check(85, 250, 3000, &config, 600_000);
    assert!(state.has_fault());

    // Condition clears - start recovery
    state.check(50, 600, 3000, &config, 700_000);
    assert!(state.has_fault()); // Still faulted

    // After recovery debounce
    let fault = state.check(50, 600, 3000, &config, 1_300_000);
    assert_eq!(fault, PlausibilityFault::None);
    assert!(!state.has_fault());
}

#[test]
fn test_debounce_resets_on_condition_change() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Start one fault
    state.check(85, 250, 3000, &config, 0);
    assert_eq!(state.pending_fault, PlausibilityFault::TpsHighMapLow);

    // Condition clears before debounce
    state.check(50, 600, 3000, &config, 400_000);
    assert_eq!(state.pending_fault, PlausibilityFault::None);
    assert!(!state.has_fault());

    // New fault starts fresh debounce
    state.check(85, 250, 3000, &config, 500_000);
    assert_eq!(state.fault_start_us, 500_000);
}

#[test]
fn test_disabled_config() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig {
        enable: false,
        ..PlausibilityConfig::DEFAULT
    };

    // Implausible condition but checking disabled
    state.check(85, 250, 3000, &config, 0);
    state.check(85, 250, 3000, &config, 600_000);
    assert!(!state.has_fault());
}

#[test]
fn test_reset() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Create confirmed fault
    state.check(85, 250, 3000, &config, 0);
    state.check(85, 250, 3000, &config, 600_000);
    assert!(state.has_fault());

    // Reset
    state.reset();
    assert!(!state.has_fault());
    assert_eq!(state.confirmed_fault, PlausibilityFault::None);
}

#[test]
fn test_edge_cases_at_thresholds() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Exactly at TPS high threshold, exactly at MAP low threshold
    // (should trigger fault - >= and <=)
    state.check(80, 300, 3000, &config, 0);
    state.check(80, 300, 3000, &config, 600_000);
    assert_eq!(state.confirmed_fault, PlausibilityFault::TpsHighMapLow);
}

#[test]
fn test_wot_with_good_map() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // WOT with good MAP (high pressure) - should be fine
    let fault = state.check(100, 950, 5000, &config, 0);
    assert_eq!(fault, PlausibilityFault::None);
}

#[test]
fn test_closed_throttle_with_vacuum() {
    let mut state = PlausibilityState::new();
    let config = PlausibilityConfig::DEFAULT;

    // Closed throttle with vacuum - should be fine
    let fault = state.check(0, 300, 3000, &config, 0);
    assert_eq!(fault, PlausibilityFault::None);
}

// =========================================================================
// Rate Validator Tests
// =========================================================================

#[test]
fn test_rate_validator_accepts_first_sample() {
    let mut validator = RateValidator::new(50);

    let result = validator.validate(75, 10_000, 500, 1000);
    assert_eq!(result, 75);
    assert!(!validator.was_rejected());
}

#[test]
fn test_rate_validator_accepts_normal_change() {
    let mut validator = RateValidator::new(50);

    // Initialize with first sample
    validator.validate(50, 0, 500, 1000);

    // Change of 25 in 100ms = 250/s, which is below 500/s limit
    let result = validator.validate(75, 100_000, 500, 1000);
    assert_eq!(result, 75);
    assert!(!validator.was_rejected());
}

#[test]
fn test_rate_validator_rejects_spike() {
    let mut validator = RateValidator::new(50);

    // Initialize with first sample
    validator.validate(50, 0, 500, 1000);

    // Change of 80 in 10ms = 8000/s, way above 500/s limit
    let result = validator.validate(130, 10_000, 500, 1000);
    assert_eq!(result, 50); // Returns last-known-good
    assert!(validator.was_rejected());
}

#[test]
fn test_rate_validator_accepts_after_5_rejections() {
    let mut validator = RateValidator::new(50);

    // Initialize with first sample
    validator.validate(50, 0, 500, 1000);

    // 5 consecutive rejections should force acceptance
    for i in 0..4 {
        let result = validator.validate(130, (i + 1) * 10_000, 500, 1000);
        assert_eq!(result, 50); // Still rejecting
    }

    // 5th rejection - should accept
    let result = validator.validate(130, 50_000, 500, 1000);
    assert_eq!(result, 130);
}

#[test]
fn test_rate_validator_reset_clears_state() {
    let mut validator = RateValidator::new(50);

    // Initialize with first sample
    validator.validate(50, 0, 500, 1000);

    // Reject a sample
    validator.validate(130, 10_000, 500, 1000);
    assert!(validator.was_rejected());

    // Reset
    validator.reset(75, 20_000);

    assert!(!validator.was_rejected());
    assert_eq!(validator.get_value(), 75);
    assert_eq!(validator.reject_count, 0);
}

#[test]
fn test_rate_validator_ignores_samples_within_min_interval() {
    let mut validator = RateValidator::new(50);

    // First sample
    validator.validate(50, 0, 500, 1000);

    // Sample within min interval - should accept without rate check
    let result = validator.validate(130, 500, 500, 1000);
    assert_eq!(result, 130); // Accepted even though rate would exceed
    assert!(!validator.was_rejected());
}

#[test]
fn test_rate_validation_state_tps_and_map() {
    let mut state = RateValidationState::new();
    let config = RateConfig::DEFAULT;

    // Initialize
    state.validate(50, 800, &config, 0);

    // Normal change
    let (tps, map, tps_rej, map_rej) = state.validate(55, 820, &config, 100_000);
    assert_eq!(tps, 55);
    assert_eq!(map, 820);
    assert!(!tps_rej);
    assert!(!map_rej);
}

#[test]
fn test_rate_validation_state_rejects_tps_spike() {
    let mut state = RateValidationState::new();
    let config = RateConfig::DEFAULT;

    // Initialize
    state.validate(50, 800, &config, 0);

    // TPS spike (0 to 100 in 10ms = 10000/s)
    let (tps, map, tps_rej, map_rej) = state.validate(100, 820, &config, 10_000);
    assert_eq!(tps, 50); // Rejected, returns last-good
    assert_eq!(map, 820); // MAP OK
    assert!(tps_rej);
    assert!(!map_rej);
    assert!(state.any_rejected());
}

#[test]
fn test_rate_validation_disabled() {
    let mut state = RateValidationState::new();
    let config = RateConfig {
        enable: false,
        ..RateConfig::DEFAULT
    };

    state.validate(50, 800, &config, 0);

    // Spike should be passed through when disabled
    let (tps, map, tps_rej, map_rej) = state.validate(100, 2000, &config, 10_000);
    assert_eq!(tps, 100);
    assert_eq!(map, 2000);
    assert!(!tps_rej);
    assert!(!map_rej);
}
