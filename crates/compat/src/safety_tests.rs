use super::*;

#[test]
fn cranking_gate_hysteresis() {
    let mut cg = CrankingGate::new();
    // Below threshold -> cranking
    assert!(cg.update(300));
    // Slightly above threshold but below exit -> still cranking
    assert!(cg.update(CRANKING_RPM_THRESHOLD + 50));
    // Above exit -> not cranking
    assert!(!cg.update(CRANKING_EXIT_RPM));
    // Drop below threshold -> cranking
    assert!(cg.update(CRANKING_RPM_THRESHOLD - 1));
}

#[test]
fn test_flood_clear_inactive_during_normal_running() {
    let mut state = FloodClearState::new();

    // Normal running (2000 RPM, 50% throttle)
    let active = update_flood_clear(2000, 50, &mut state);

    assert!(!active);
    assert!(!state.active);
}

#[test]
fn test_flood_clear_inactive_when_cranking_without_wot() {
    let mut state = FloodClearState::new();

    // Cranking without WOT (300 RPM, 30% throttle)
    let active = update_flood_clear(300, 30, &mut state);

    assert!(!active);
    assert!(!state.active);
}

#[test]
fn test_flood_clear_active_when_cranking_with_wot() {
    let mut state = FloodClearState::new();

    // Cranking with WOT (300 RPM, 95% throttle)
    let active = update_flood_clear(300, 95, &mut state);

    assert!(active);
    assert!(state.active);
}

#[test]
fn test_flood_clear_deactivates_when_engine_starts() {
    let mut state = FloodClearState::new();

    // Cranking with WOT - activates flood clear
    update_flood_clear(300, 95, &mut state);
    assert!(state.active);

    // Engine starts and revs up
    let active = update_flood_clear(800, 95, &mut state);

    assert!(!active);
    assert!(!state.active);
}

#[test]
fn test_flood_clear_counts_active_cycles() {
    let mut state = FloodClearState::new();

    // Multiple cranking cycles with WOT
    for i in 1..=5 {
        update_flood_clear(300, 95, &mut state);
        assert_eq!(state.active_cycles, i);
    }
}

#[test]
fn test_sync_loss_tracker_first_loss_allows_recovery() {
    let mut tracker = SyncLossTracker::new();

    let should_shutdown = tracker.record_sync_loss(1000);

    assert!(!should_shutdown);
    assert_eq!(tracker.loss_count, 1);
    assert_eq!(tracker.total_losses, 1);
}

#[test]
fn test_sync_loss_tracker_shuts_down_after_multiple_losses() {
    let mut tracker = SyncLossTracker::new();

    // First loss - recovery
    let should_shutdown = tracker.record_sync_loss(1000);
    assert!(!should_shutdown);

    // Second loss (within window) - recovery
    let should_shutdown = tracker.record_sync_loss(2000);
    assert!(!should_shutdown);

    // Third loss (within window) - shutdown
    let should_shutdown = tracker.record_sync_loss(3000);
    assert!(should_shutdown);
    assert!(tracker.is_shutdown());
}

#[test]
fn test_sync_loss_tracker_resets_window_after_timeout() {
    let mut tracker = SyncLossTracker::new();

    // First loss
    tracker.record_sync_loss(1000);
    assert_eq!(tracker.loss_count, 1);

    // Second loss after window expires (6 seconds later)
    // Should start new window
    let should_shutdown = tracker.record_sync_loss(6_000_000 + 1000);
    assert!(!should_shutdown);
    assert_eq!(tracker.loss_count, 1); // Reset to 1 in new window
}

#[test]
fn test_sync_loss_tracker_handles_esd_glitches() {
    let mut tracker = SyncLossTracker::new();

    // Simulate isolated ESD events spread over time
    // Loss 1 at T=0
    tracker.record_sync_loss(0);
    tracker.record_recovery();

    // Loss 2 at T=6s (new window)
    tracker.record_sync_loss(6_000_000);
    tracker.record_recovery();

    // Loss 3 at T=12s (new window)
    tracker.record_sync_loss(12_000_000);
    tracker.record_recovery();

    // Should not shut down - losses are spread out (ESD pattern)
    assert!(!tracker.is_shutdown());
    assert_eq!(tracker.total_losses, 3);
    assert_eq!(tracker.successful_recoveries, 3);
}

#[test]
fn test_sync_loss_tracker_detects_real_failure() {
    let mut tracker = SyncLossTracker::new();

    // Simulate rapid repeated losses (real failure pattern)
    // All within 1 second
    tracker.record_sync_loss(0);
    tracker.record_sync_loss(100_000); // 0.1s later
    tracker.record_sync_loss(200_000); // 0.2s later

    // Should shut down - too many losses too quickly
    assert!(tracker.is_shutdown());
}

#[test]
fn test_should_allow_injection_normal() {
    let allow = should_allow_injection(false, false);
    assert!(allow);
}

#[test]
fn test_should_allow_injection_blocks_on_flood_clear() {
    let allow = should_allow_injection(true, false);
    assert!(!allow);
}

#[test]
fn test_should_allow_injection_blocks_on_shutdown() {
    let allow = should_allow_injection(false, true);
    assert!(!allow);
}

#[test]
fn test_should_allow_injection_blocks_on_both() {
    let allow = should_allow_injection(true, true);
    assert!(!allow);
}

#[test]
fn test_clear_shutdown_resets_state() {
    let mut tracker = SyncLossTracker::new();

    // Trigger shutdown
    tracker.record_sync_loss(0);
    tracker.record_sync_loss(1000);
    tracker.record_sync_loss(2000);
    assert!(tracker.is_shutdown());

    // Manual clear (key cycle)
    tracker.clear_shutdown();

    assert!(!tracker.is_shutdown());
    assert_eq!(tracker.loss_count, 0);
}

#[test]
fn test_reset_window_clears_loss_count() {
    let mut tracker = SyncLossTracker::new();

    // Record some losses
    tracker.record_sync_loss(0);
    tracker.record_sync_loss(1000);
    assert_eq!(tracker.loss_count, 2);

    // After sustained good operation, reset window
    tracker.reset_window();

    assert_eq!(tracker.loss_count, 0);
    assert!(!tracker.is_shutdown());
    // Total losses preserved for diagnostics
    assert_eq!(tracker.total_losses, 2);
}

// =========================================================================
// Voltage Monitor Tests
// =========================================================================

#[test]
fn test_voltage_monitor_normal_operation() {
    let mut monitor = VoltageMonitor::new();

    // Normal voltage
    let state = monitor.update(13500, 0);
    assert_eq!(state, PowerState::Normal);
    assert!(!monitor.should_block_fuel());
    assert!(!monitor.should_limit_rpm());
    assert_eq!(monitor.get_rpm_limit(), None);
}

#[test]
fn test_voltage_monitor_warning_threshold() {
    let mut monitor = VoltageMonitor::new();

    // Just below warning threshold (10V)
    let state = monitor.update(9500, 0);
    assert_eq!(state, PowerState::Warning);
    assert!(!monitor.should_block_fuel()); // Warning doesn't cut fuel
    assert!(monitor.should_limit_rpm());
    assert_eq!(monitor.get_rpm_limit(), Some(LIMP_RPM_LIMIT));
}

#[test]
fn test_voltage_monitor_critical_threshold_with_debounce() {
    let mut monitor = VoltageMonitor::new();

    // First critical reading - should not cut fuel yet (debounce)
    monitor.update(7000, 0);
    assert_eq!(monitor.critical_count, 1);
    assert!(!monitor.fuel_cut_active);

    // Second critical reading
    monitor.update(7000, 1000);
    assert_eq!(monitor.critical_count, 2);
    assert!(!monitor.fuel_cut_active);

    // Third critical reading - should cut fuel
    let state = monitor.update(7000, 2000);
    assert_eq!(state, PowerState::Critical);
    assert!(monitor.should_block_fuel());
    assert!(monitor.should_limit_rpm());
}

#[test]
fn test_voltage_monitor_critical_debounce_resets() {
    let mut monitor = VoltageMonitor::new();

    // Two critical readings
    monitor.update(7000, 0);
    monitor.update(7000, 1000);
    assert_eq!(monitor.critical_count, 2);

    // Voltage recovers above critical - count should reset
    monitor.update(9000, 2000);
    assert_eq!(monitor.critical_count, 0);

    // Another critical reading - count starts fresh
    monitor.update(7000, 3000);
    assert_eq!(monitor.critical_count, 1);
    assert!(!monitor.fuel_cut_active);
}

#[test]
fn test_voltage_monitor_recovery_from_warning() {
    let mut monitor = VoltageMonitor::new();

    // Enter warning state
    monitor.update(9500, 0);
    assert_eq!(monitor.state, PowerState::Warning);

    // Voltage recovers but not enough time
    monitor.update(12000, 1_000_000);
    assert!(monitor.limp_active); // Still in limp

    // After recovery time (2s)
    let state = monitor.update(12000, 3_000_000);
    assert_eq!(state, PowerState::Normal);
    assert!(!monitor.should_limit_rpm());
}

#[test]
fn test_voltage_monitor_recovery_interrupted() {
    let mut monitor = VoltageMonitor::new();

    // Enter warning state
    monitor.update(9500, 0);

    // Start recovering
    monitor.update(12000, 1_000_000);

    // Voltage drops again - recovery timer should reset
    monitor.update(9500, 1_500_000);
    assert_eq!(monitor.recovery_start_us, 0);

    // Voltage recovers again - needs full 2s from here
    monitor.update(12000, 2_000_000);

    // Not enough time yet
    monitor.update(12000, 3_500_000);
    assert!(monitor.limp_active);

    // Now enough time (2s from 2_000_000)
    let state = monitor.update(12000, 4_000_000);
    assert_eq!(state, PowerState::Normal);
}

#[test]
fn test_voltage_monitor_overvoltage() {
    let mut monitor = VoltageMonitor::new();

    // Load dump - overvoltage
    let state = monitor.update(17000, 0);
    assert_eq!(state, PowerState::Overvoltage);
    assert!(!monitor.should_block_fuel()); // Don't cut fuel on load dump
    assert!(!monitor.should_limit_rpm());

    // Voltage returns to normal
    let state = monitor.update(14000, 1000);
    assert_eq!(state, PowerState::Normal);
}

#[test]
fn test_voltage_monitor_reset() {
    let mut monitor = VoltageMonitor::new();

    // Put into critical state
    for i in 0..CRITICAL_DEBOUNCE_COUNT {
        monitor.update(7000, i as u32 * 1000);
    }
    assert!(monitor.fuel_cut_active);
    assert_eq!(monitor.state, PowerState::Critical);

    // Reset (key cycle)
    monitor.reset();

    assert_eq!(monitor.state, PowerState::Normal);
    assert!(!monitor.fuel_cut_active);
    assert!(!monitor.limp_active);
    assert_eq!(monitor.critical_count, 0);
}

#[test]
fn test_voltage_monitor_recovery_from_critical() {
    let mut monitor = VoltageMonitor::new();

    // Enter critical state
    for i in 0..CRITICAL_DEBOUNCE_COUNT {
        monitor.update(7000, i as u32 * 1000);
    }
    assert!(monitor.fuel_cut_active);

    // Voltage recovers above recovery threshold
    monitor.update(12000, 10_000);

    // Wait for recovery time
    let state = monitor.update(12000, 2_010_000);
    assert_eq!(state, PowerState::Normal);
    assert!(!monitor.should_block_fuel());
}

// =========================================================================
// Load Failure Tracker Tests
// =========================================================================

#[test]
fn test_load_failure_no_fault_at_low_rpm() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // MAP fault at low RPM should not trigger limp
    let in_limp = tracker.check(true, 2000, &config, 0);
    assert!(!in_limp);

    // Even after debounce time
    let in_limp = tracker.check(true, 2000, &config, 200_000);
    assert!(!in_limp);
}

#[test]
fn test_load_failure_no_fault_without_map_error() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // High RPM without MAP fault should not trigger limp
    let in_limp = tracker.check(false, 5000, &config, 0);
    assert!(!in_limp);

    // Even after debounce time
    let in_limp = tracker.check(false, 5000, &config, 200_000);
    assert!(!in_limp);
}

#[test]
fn test_load_failure_triggers_at_high_rpm_with_map_fault() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // MAP fault at high RPM - first check starts debounce
    let in_limp = tracker.check(true, 5000, &config, 0);
    assert!(!in_limp);
    assert!(tracker.fault_pending);

    // Before debounce time - not in limp yet
    let in_limp = tracker.check(true, 5000, &config, 50_000);
    assert!(!in_limp);

    // After debounce time - enter limp
    let in_limp = tracker.check(true, 5000, &config, 150_000);
    assert!(in_limp);
    assert_eq!(tracker.reason, Some(LoadFailureReason::MapFailureHighRpm));
}

#[test]
fn test_load_failure_debounce_resets_on_recovery() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // Start fault condition
    tracker.check(true, 5000, &config, 0);
    assert!(tracker.fault_pending);

    // Fault clears before debounce completes
    tracker.check(false, 5000, &config, 50_000);
    assert!(!tracker.fault_pending);

    // New fault starts fresh debounce
    tracker.check(true, 5000, &config, 100_000);
    assert!(tracker.fault_pending);
    assert_eq!(tracker.fault_detected_us, 100_000);
}

#[test]
fn test_load_failure_recovery_requires_time() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // Enter limp mode
    tracker.check(true, 5000, &config, 0);
    tracker.check(true, 5000, &config, 150_000);
    assert!(tracker.in_limp);

    // Fault clears - start recovery
    tracker.check(false, 5000, &config, 200_000);
    assert!(tracker.in_limp); // Still in limp

    // Not enough recovery time
    tracker.check(false, 5000, &config, 1_000_000);
    assert!(tracker.in_limp);

    // After recovery time
    let in_limp = tracker.check(false, 5000, &config, 2_500_000);
    assert!(!in_limp);
}

#[test]
fn test_load_failure_recovery_interrupted() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // Enter limp mode
    tracker.check(true, 5000, &config, 0);
    tracker.check(true, 5000, &config, 150_000);
    assert!(tracker.in_limp);

    // Start recovery
    tracker.check(false, 5000, &config, 200_000);
    assert!(tracker.good_since_us > 0);

    // Fault returns - recovery timer resets
    tracker.check(true, 5000, &config, 500_000);
    assert_eq!(tracker.good_since_us, 0);
    assert!(tracker.in_limp);
}

#[test]
fn test_load_failure_disabled_config() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig {
        enable: false,
        ..LoadFailureConfig::DEFAULT
    };

    // MAP fault at high RPM with disabled config
    tracker.check(true, 5000, &config, 0);
    tracker.check(true, 5000, &config, 200_000);
    assert!(!tracker.in_limp);
}

#[test]
fn test_load_failure_rpm_limit() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // Not in limp - no limit
    assert_eq!(tracker.get_rpm_limit(&config), None);

    // Enter limp mode
    tracker.check(true, 5000, &config, 0);
    tracker.check(true, 5000, &config, 150_000);

    // In limp - return configured limit
    assert_eq!(tracker.get_rpm_limit(&config), Some(config.limp_rpm_limit));
    assert!(tracker.should_limit_rpm());
}

#[test]
fn test_load_failure_reset() {
    let mut tracker = LoadFailureTracker::new();
    let config = LoadFailureConfig::DEFAULT;

    // Enter limp mode
    tracker.check(true, 5000, &config, 0);
    tracker.check(true, 5000, &config, 150_000);
    assert!(tracker.in_limp);

    // Reset
    tracker.reset();

    assert!(!tracker.in_limp);
    assert_eq!(tracker.reason, None);
    assert!(!tracker.fault_pending);
}
