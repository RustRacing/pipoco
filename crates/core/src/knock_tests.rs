use super::*;

#[test]
fn test_knock_config_default() {
    let config = KnockConfig::DEFAULT;
    assert!(config.enable);
    assert_eq!(config.max_cylinders, 4);
    assert_eq!(config.retard_step_x10, 20); // 2.0 degrees
    assert_eq!(config.retard_max_x10, 150); // 15.0 degrees
}

#[test]
fn test_default_i6_knock_config_is_six_cylinder_without_custom_window_guess() {
    let config = KnockConfig::DEFAULT_I6;

    assert!(config.enable);
    assert_eq!(config.max_cylinders, 6);
    assert_eq!(
        config.window_start_btdc,
        KnockConfig::DEFAULT.window_start_btdc
    );
    assert_eq!(config.window_end_btdc, KnockConfig::DEFAULT.window_end_btdc);
}

#[test]
fn test_knock_state_new() {
    let state = KnockState::new();
    assert_eq!(state.global_retard_x10, 0);
    assert_eq!(state.total_knock_count, 0);
    assert!(!state.active);

    for cyl in &state.cylinders {
        assert_eq!(cyl.retard_x10, 0);
        assert_eq!(cyl.knock_count, 0);
    }
}

#[test]
fn test_knock_detection_below_threshold() {
    let mut state = KnockState::new();
    let config = KnockConfig::DEFAULT;

    // Level below threshold should not trigger
    let detected = state.process_sample(0, 50, &config, 0);
    assert!(!detected);
    assert_eq!(state.cylinders[0].knock_count, 0);
    assert_eq!(state.cylinders[0].retard_x10, 0);
}

#[test]
fn test_knock_detection_with_debounce() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        debounce_count: 2,
        ..KnockConfig::DEFAULT
    };

    // First knock - not enough for debounce
    let detected = state.process_sample(0, 150, &config, 0);
    assert!(!detected);
    assert_eq!(state.cylinders[0].consecutive_knocks, 1);

    // Second knock - should trigger
    let detected = state.process_sample(0, 150, &config, 1000);
    assert!(detected);
    assert_eq!(state.cylinders[0].knock_count, 1);
    assert!(state.cylinders[0].retard_x10 > 0);
}

#[test]
fn test_knock_retard_application() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        debounce_count: 1,   // Instant trigger
        retard_step_x10: 20, // 2.0 degrees
        ..KnockConfig::DEFAULT
    };

    // Trigger knock
    state.process_sample(0, 150, &config, 0);
    assert_eq!(state.cylinders[0].retard_x10, 20);

    // Another knock
    state.process_sample(0, 150, &config, 1000);
    assert_eq!(state.cylinders[0].retard_x10, 40);
}

#[test]
fn test_knock_retard_max_clamping() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        debounce_count: 1,
        retard_step_x10: 50, // 5.0 degrees
        retard_max_x10: 100, // 10.0 degrees max
        ..KnockConfig::DEFAULT
    };

    // Trigger many knocks
    for i in 0..10 {
        state.process_sample(0, 150, &config, i * 1000);
    }

    // Should be clamped to max
    assert_eq!(state.cylinders[0].retard_x10, 100);
}

#[test]
fn test_knock_recovery() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        debounce_count: 1,
        retard_step_x10: 50,   // 5.0 degrees
        recovery_rate_x10: 10, // 1.0 degree per second
        ..KnockConfig::DEFAULT
    };

    // Trigger knock at t=0
    state.process_sample(0, 150, &config, 0);
    assert_eq!(state.cylinders[0].retard_x10, 50);

    // Set last_recovery_us to start recovery
    state.last_recovery_us = 0;

    // After 1 second, should recover 1 degree (10 x10)
    state.update_recovery(&config, 1_000_000);
    assert_eq!(state.cylinders[0].retard_x10, 40);

    // After 5 seconds total
    state.update_recovery(&config, 5_000_000);
    assert_eq!(state.cylinders[0].retard_x10, 0); // Fully recovered
}

#[test]
fn test_knock_per_cylinder_tracking() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        debounce_count: 1,
        max_cylinders: 4,
        ..KnockConfig::DEFAULT
    };

    // Knock on cylinder 0
    state.process_sample(0, 150, &config, 0);
    // Knock on cylinder 2
    state.process_sample(2, 150, &config, 1000);

    assert_eq!(state.cylinders[0].knock_count, 1);
    assert_eq!(state.cylinders[1].knock_count, 0);
    assert_eq!(state.cylinders[2].knock_count, 1);
    assert_eq!(state.cylinders[3].knock_count, 0);

    assert!(state.cylinders[0].retard_x10 > 0);
    assert_eq!(state.cylinders[1].retard_x10, 0);
    assert!(state.cylinders[2].retard_x10 > 0);
}

#[test]
fn test_knock_get_retard() {
    let mut state = KnockState::new();
    state.cylinders[0].retard_x10 = 30;
    state.global_retard_x10 = 10;

    // Per-cylinder + global
    assert_eq!(state.get_retard(0), 40);
    // Just global for other cylinders
    assert_eq!(state.get_retard(1), 10);
}

#[test]
fn test_knock_get_retard_degrees() {
    let mut state = KnockState::new();
    state.cylinders[0].retard_x10 = 35; // 3.5 degrees

    let retard = state.get_retard_degrees(0);
    assert_eq!(retard, -3); // Negative for retard, truncated
}

#[test]
fn test_knock_disabled_below_min_rpm() {
    let state = KnockState::new();
    let config = KnockConfig {
        min_rpm: 1500,
        ..KnockConfig::DEFAULT
    };

    assert!(!state.should_enable(&config, 1000, 80));
    assert!(state.should_enable(&config, 2000, 80));
}

#[test]
fn test_knock_disabled_when_cold() {
    let state = KnockState::new();
    let config = KnockConfig {
        min_clt_c: 60,
        ..KnockConfig::DEFAULT
    };

    assert!(!state.should_enable(&config, 2000, 40));
    assert!(state.should_enable(&config, 2000, 70));
}

#[test]
fn test_knock_reset() {
    let mut state = KnockState::new();

    state.cylinders[0].retard_x10 = 50;
    state.cylinders[0].knock_count = 10;
    state.total_knock_count = 10;
    state.global_retard_x10 = 20;

    state.reset();

    assert_eq!(state.cylinders[0].retard_x10, 0);
    assert_eq!(state.cylinders[0].knock_count, 0);
    assert_eq!(state.total_knock_count, 0);
    assert_eq!(state.global_retard_x10, 0);
}

#[test]
fn test_knock_worst_cylinder() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        max_cylinders: 4,
        ..KnockConfig::DEFAULT
    };

    state.cylinders[0].knock_count = 5;
    state.cylinders[1].knock_count = 10;
    state.cylinders[2].knock_count = 3;
    state.cylinders[3].knock_count = 8;

    assert_eq!(state.get_worst_cylinder(&config), 1);
}

#[test]
fn test_knock_inject_simulation() {
    let mut state = KnockState::new();
    let config = KnockConfig::DEFAULT;

    state.inject_knock(0, 200, &config, 0);

    assert_eq!(state.cylinders[0].knock_count, 1);
    assert!(state.cylinders[0].retard_x10 > 0);
    assert_eq!(state.peak_knock_level, 200);
}

#[test]
fn test_knock_controller_integration() {
    let mut controller = KnockController::new();
    controller.config.debounce_count = 1;

    // Cold engine - should not process
    let detected = controller.process(0, 150, 2000, 40, 0);
    assert!(!detected);
    assert!(!controller.state.active);

    // Warm engine - should process
    let detected = controller.process(0, 150, 2000, 70, 1000);
    assert!(detected);
    assert!(controller.state.active);

    let retard = controller.get_retard_degrees(0);
    assert!(retard < 0);
}

#[test]
fn test_knock_debounce_reset_on_no_knock() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        debounce_count: 3,
        ..KnockConfig::DEFAULT
    };

    // Two knocks - not enough
    state.process_sample(0, 150, &config, 0);
    state.process_sample(0, 150, &config, 1000);
    assert_eq!(state.cylinders[0].consecutive_knocks, 2);

    // Below threshold - resets counter
    state.process_sample(0, 50, &config, 2000);
    assert_eq!(state.cylinders[0].consecutive_knocks, 0);

    // Need 3 consecutive again
    state.process_sample(0, 150, &config, 3000);
    assert_eq!(state.cylinders[0].consecutive_knocks, 1);
}

#[test]
fn test_knock_global_retard() {
    let mut state = KnockState::new();
    let config = KnockConfig {
        retard_max_x10: 100,
        ..KnockConfig::DEFAULT
    };

    state.apply_global_retard(50, &config);
    assert_eq!(state.global_retard_x10, 50);

    // Check it's added to per-cylinder
    state.cylinders[0].retard_x10 = 20;
    assert_eq!(state.get_retard(0), 70);

    // Clamp to max
    state.apply_global_retard(200, &config);
    assert_eq!(state.global_retard_x10, 100);
}
