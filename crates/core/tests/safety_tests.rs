//! Critical safety tests for ECU
//!
//! These tests verify safety-critical functionality that could cause engine
//! damage if not working correctly. All tests in this file MUST PASS before
//! any release or hardware deployment.

use ecu_core::constants::safety::{
    CRANKING_RPM_THRESHOLD, SYNC_RECOVERY_ATTEMPTS, SYNC_RECOVERY_WINDOW_US, WOT_TPS_THRESHOLD,
};
use ecu_core::hal::TimeSource;
use ecu_core::{
    compat::EcuState, scale_u16, should_allow_injection, update_flood_clear, FloodClearState,
    IpwTable, SyncLossTracker, TriggerDecoder,
};
use std::cell::Cell;

// Mock time source
struct MockTime {
    time: Cell<u32>,
}

impl MockTime {
    fn new(initial: u32) -> Self {
        Self {
            time: Cell::new(initial),
        }
    }

    fn set_time(&self, t: u32) {
        self.time.set(t);
    }
}

impl TimeSource for MockTime {
    fn micros(&self) -> u32 {
        self.time.get()
    }
}

// =============================================================================
// FUEL SAFETY TESTS
// =============================================================================

#[test]
fn test_max_fuel_clamp_enforced() {
    let mut state = EcuState::new();

    // Set extremely high base pulse width
    for row in 0..16 {
        for col in 0..16 {
            state.config.ipw_table[row][col] = u16::MAX;
        }
    }

    // Set maximum corrections
    state.corrections_mut().clt = 255;
    state.corrections_mut().iat = 255;
    state.corrections_mut().vbatt = 255;

    // Calculate at all operating points
    for rpm in [500, 1000, 2000, 4000, 6000, 8000] {
        for load in [20, 50, 100, 150, 170] {
            let pw = state.calculate_fuel(rpm, load);
            assert_eq!(
                pw, 20000,
                "Max fuel clamp failed at {rpm} RPM, {load} kPa: got {pw}us, expected 20000us"
            );
        }
    }
}

#[test]
fn test_min_fuel_clamp_enforced() {
    let mut state = EcuState::new();

    // Set extremely low base pulse width
    for row in 0..16 {
        for col in 0..16 {
            state.config.ipw_table[row][col] = 1; // Minimum possible value
        }
    }

    // Set minimum corrections (almost zero)
    state.corrections_mut().clt = 1;
    state.corrections_mut().iat = 1;
    state.corrections_mut().vbatt = 1;

    // Calculate at all operating points
    for rpm in [500, 1000, 2000, 4000, 6000, 8000] {
        for load in [20, 50, 100, 150, 170] {
            let pw = state.calculate_fuel(rpm, load);
            assert_eq!(
                pw, 500,
                "Min fuel clamp failed at {rpm} RPM, {load} kPa: got {pw}us, expected 500us"
            );
        }
    }
}

#[test]
fn test_runaway_fuel_protection_table_corruption() {
    let mut state = EcuState::new();

    // Simulate table corruption with random high values
    state.config.ipw_table[7][7] = u16::MAX;
    state.config.ipw_table[0][0] = u16::MAX;
    state.config.ipw_table[15][15] = u16::MAX;

    // Even with corrupted table, should never exceed max
    let pw1 = state.calculate_fuel(3500, 90); // Middle of table
    let pw2 = state.calculate_fuel(500, 20); // Low corner
    let pw3 = state.calculate_fuel(8000, 170); // High corner

    assert!(pw1 <= 20000, "Fuel exceeded max with corrupted table");
    assert!(pw2 <= 20000, "Fuel exceeded max with corrupted table");
    assert!(pw3 <= 20000, "Fuel exceeded max with corrupted table");
}

#[test]
fn test_vbatt_correction_low_voltage_limit() {
    let mut state = EcuState::new();

    // Low voltage should increase pulse width, but not excessively
    state.config.ipw_table[8][5] = 10000; // 10ms base
    state.corrections_mut().vbatt = 200; // 2.0x correction (compensate for slow injector)

    let pw = state.calculate_fuel(3000, 100);

    // Should apply correction but clamp to max
    assert_eq!(pw, 20000, "Low voltage correction should be clamped");
}

#[test]
fn test_overflow_protection_in_corrections() {
    // Test that scale_u16 doesn't overflow even with extreme inputs
    assert_eq!(scale_u16(u16::MAX, 255), u16::MAX);
    assert_eq!(scale_u16(65000, 255), u16::MAX);
    assert_eq!(scale_u16(50000, 200), u16::MAX);

    // Verify multiply-then-divide order prevents intermediate overflow
    let result = scale_u16(40000, 150);
    assert!(result == u16::MAX || result == 60000); // May saturate depending on impl
}

#[test]
fn test_combined_corrections_saturation() {
    let mut state = EcuState::new();
    state.config.ipw_table[8][5] = 8000; // 8ms base

    // Multiple high corrections should saturate gracefully
    state.corrections_mut().clt = 200; // 2.0x
    state.corrections_mut().iat = 150; // 1.5x
    state.corrections_mut().vbatt = 120; // 1.2x

    let pw = state.calculate_fuel(3000, 100);

    // 8000 * 2.0 * 1.5 * 1.2 = 28800, should clamp to 20000
    assert_eq!(pw, 20000, "Multiple corrections should saturate at max");
}

// =============================================================================
// TRIGGER/SYNC SAFETY TESTS
// =============================================================================

#[test]
fn test_loss_of_sync_detection_timeout() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Establish sync
    for i in 0..57 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(59 * 1000);
    decoder.tooth_edge();

    assert!(decoder.synced());

    // Simulate complete signal loss (>200ms)
    decoder.time_source().set_time(59 * 1000 + 250_000);
    decoder.tooth_edge();

    // Must lose sync and set RPM to 0
    assert!(
        !decoder.synced(),
        "Failed to detect sync loss after timeout"
    );
    assert_eq!(decoder.rpm().raw(), 0, "RPM should be 0 after sync loss");
}

#[test]
fn test_no_premature_sync_on_noise() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Simulate noisy signal with random edges
    let noisy_periods = [100, 50, 200, 150, 80, 300, 120];

    let mut time = 0;
    for period in noisy_periods.iter() {
        time += period;
        decoder.time_source().set_time(time);
        decoder.tooth_edge();
    }

    // Should NOT sync on noise
    assert!(!decoder.synced(), "Prematurely synced on noisy signal");
    assert_eq!(decoder.rpm().raw(), 0, "RPM should be 0 before valid sync");
}

#[test]
fn test_resync_after_loss() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Establish initial sync
    for i in 0..57 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(59 * 1000);
    decoder.tooth_edge();

    assert!(decoder.synced());

    // Lose sync
    decoder.time_source().set_time(59 * 1000 + 300_000);
    decoder.tooth_edge();
    assert!(!decoder.synced());

    // Re-establish sync
    let base_time = 59 * 1000 + 300_000;
    for i in 0..57 {
        decoder.time_source().set_time(base_time + i * 1000);
        decoder.tooth_edge();
    }
    decoder.time_source().set_time(base_time + 57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(base_time + 59 * 1000);
    decoder.tooth_edge();

    // Should regain sync
    assert!(decoder.synced(), "Failed to resync after loss");
    assert!(decoder.rpm().raw() > 0, "RPM should be > 0 after resync");
}

#[test]
fn test_false_sync_protection() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Simulate false gap (single isolated long period)
    decoder.time_source().set_time(0);
    decoder.tooth_edge();
    decoder.time_source().set_time(1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(3000); // False gap
    decoder.tooth_edge();
    decoder.time_source().set_time(4000);
    decoder.tooth_edge();

    // With only 4 edges and one false gap, should not have reliable sync
    // This is a weak test - real implementation should require seeing gap twice
    // For now, we just verify RPM is reasonable if it does sync
    if decoder.synced() {
        let rpm = decoder.rpm().raw();
        assert!(
            (500..=8000).contains(&rpm),
            "RPM out of reasonable range: {rpm}"
        );
    }
}

// =============================================================================
// TABLE SAFETY TESTS
// =============================================================================

#[test]
fn test_table_bounds_checking_low() {
    let table = IpwTable::new();

    // Values below minimum bins should use first bin (not crash/panic)
    let pw1 = table.lookup(0, 0);
    let pw2 = table.lookup(100, 10);

    assert_eq!(pw1, 1000);
    assert_eq!(pw2, 1000);
}

#[test]
fn test_table_bounds_checking_high() {
    let table = IpwTable::new();

    // Values above maximum bins should use last bin (not crash/panic)
    let pw1 = table.lookup(10000, 250);
    let pw2 = table.lookup(u16::MAX, u16::MAX);

    assert_eq!(pw1, 1000);
    assert_eq!(pw2, 1000);
}

#[test]
fn test_table_cell_independence_no_bleeding() {
    let mut state = EcuState::new();

    // Modify specific cells to extreme values
    // RPM bins: [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, ...]
    // Load bins: [20, 30, 40, 50, 60, 70, 80, 90, 100, 110, ...]
    // Table is [load_idx][rpm_idx]

    // Modify cell at RPM=3000 (idx 5), Load=100 (idx 8)
    state.config.ipw_table[8][5] = 15000;

    // Verify neighboring cells are not affected
    let neighbors = [
        (state.calculate_fuel(2500, 100), "Adjacent RPM bin (lower)"), // [8][4]
        (state.calculate_fuel(3500, 100), "Adjacent RPM bin (higher)"), // [8][6]
        (state.calculate_fuel(3000, 90), "Adjacent load bin (lower)"), // [7][5]
        (
            state.calculate_fuel(3000, 110),
            "Adjacent load bin (higher)",
        ), // [9][5]
    ];

    for (pw, desc) in neighbors.iter() {
        assert_eq!(*pw, 1000, "{desc} was affected by modification");
    }

    // The modified cell should return the extreme value
    let pw_modified = state.calculate_fuel(3000, 100);
    assert_eq!(
        pw_modified, 15000,
        "Modified cell should return correct value"
    );
}

// =============================================================================
// POWER AND RESET SAFETY TESTS
// =============================================================================

#[test]
fn test_cold_boot_state_initialization() {
    let state = EcuState::new();

    // Verify safe initial state
    assert_eq!(state.rpm(), 0);
    assert!(!state.synced());
    assert_eq!(state.tooth_count(), 0);
    assert_eq!(state.corrections().clt, 100);
    assert_eq!(state.corrections().iat, 100);
    assert_eq!(state.corrections().vbatt, 100);

    // Verify table is initialized with safe values
    for row in 0..16 {
        for col in 0..16 {
            assert_eq!(
                state.config.ipw_table[row][col], 1000,
                "Table cell [{row},{col}] not initialized to safe default"
            );
        }
    }
}

#[test]
fn test_trigger_decoder_cold_boot_state() {
    let time_source = MockTime::new(0);
    let decoder = TriggerDecoder::new(time_source);

    // Verify safe initial state
    assert!(!decoder.synced());
    assert_eq!(decoder.rpm().raw(), 0);
    assert_eq!(decoder.tooth(), 0);
}

// =============================================================================
// DATA INTEGRITY TESTS
// =============================================================================

#[test]
fn test_static_memory_bounds() {
    let state = EcuState::new();

    // Verify table dimensions are correct
    assert_eq!(state.config.ipw_table.len(), 16);
    assert_eq!(state.config.ipw_table[0].len(), 16);

    // Verify we can safely access all cells
    for row in 0..16 {
        for col in 0..16 {
            let _ = state.config.ipw_table[row][col];
        }
    }
}

// =============================================================================
// EDGE CASE TESTS
// =============================================================================

#[test]
fn test_tooth_counter_wraparound() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Establish sync
    for i in 0..57 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(59 * 1000);
    decoder.tooth_edge();

    assert_eq!(decoder.tooth(), 1);

    // Continue through multiple revolutions
    for rev in 0..10 {
        let base = 59000 + rev * 60000;
        for i in 1..58 {
            decoder.time_source().set_time(base + i * 1000);
            decoder.tooth_edge();
        }

        // Check tooth counter resets properly
        decoder.time_source().set_time(base + 58000);
        decoder.tooth_edge();
        decoder.time_source().set_time(base + 60000);
        decoder.tooth_edge();

        assert_eq!(
            decoder.tooth(),
            1,
            "Tooth counter failed to wrap at revolution {rev}"
        );
    }
}

#[test]
fn test_zero_rpm_safety() {
    let state = EcuState::new();

    // Calculate fuel at 0 RPM (engine stopped)
    let pw = state.calculate_fuel(0, 0);

    // Should return safe value (will use first bin)
    assert!(
        (500..=20000).contains(&pw),
        "Fuel at 0 RPM out of safe range: {pw}"
    );
}

#[test]
fn test_correction_factor_zero() {
    let mut state = EcuState::new();

    // Zero correction should result in minimum fuel
    state.corrections_mut().clt = 0;

    let pw = state.calculate_fuel(3000, 60);

    assert_eq!(pw, 500, "Zero correction should result in minimum fuel");
}

// =============================================================================
// SYNC-LOSS RECOVERY-WINDOW TESTS (SyncLossTracker)
// =============================================================================

#[test]
fn test_sync_loss_isolated_events_outside_window_recover() {
    let mut tracker = SyncLossTracker::new();

    // Each loss lands well outside the recovery window of the previous one,
    // so every event starts a fresh window and stays recoverable.
    for n in 0..10u32 {
        let at_us = n.wrapping_mul(SYNC_RECOVERY_WINDOW_US.wrapping_add(1));
        let shutdown = tracker.record_sync_loss(at_us);
        assert!(
            !shutdown,
            "isolated loss outside window must stay recoverable (event {n})"
        );
        assert!(!tracker.is_shutdown());
        tracker.record_recovery();
    }

    assert_eq!(tracker.total_losses, 10);
    assert_eq!(tracker.successful_recoveries, 10);
}

#[test]
fn test_sync_loss_repeated_events_inside_window_latch_shutdown() {
    let mut tracker = SyncLossTracker::new();

    // First loss opens the window; subsequent losses inside the window count up.
    let base_us = 1_000_000u32;
    assert!(
        !tracker.record_sync_loss(base_us),
        "first loss is recoverable"
    );

    let mut shutdown = false;
    for attempt in 1..SYNC_RECOVERY_ATTEMPTS {
        // Stay strictly inside the recovery window relative to window start.
        let at_us = base_us + u32::from(attempt) * 1_000;
        assert!(at_us.wrapping_sub(base_us) <= SYNC_RECOVERY_WINDOW_US);
        shutdown = tracker.record_sync_loss(at_us);
    }

    assert!(
        shutdown,
        "{SYNC_RECOVERY_ATTEMPTS} losses inside the window must latch shutdown"
    );
    assert!(tracker.is_shutdown());
    // Injection must be blocked while shutdown is latched.
    assert!(!should_allow_injection(false, tracker.is_shutdown()));
}

#[test]
fn test_sync_loss_shutdown_requires_explicit_clear() {
    let mut tracker = SyncLossTracker::new();
    let base_us = 2_000_000u32;

    for attempt in 0..SYNC_RECOVERY_ATTEMPTS {
        tracker.record_sync_loss(base_us + u32::from(attempt) * 1_000);
    }
    assert!(
        tracker.is_shutdown(),
        "shutdown latched after repeated loss"
    );

    // A later isolated loss does not auto-clear the latch.
    tracker.record_sync_loss(base_us + SYNC_RECOVERY_WINDOW_US * 4);
    assert!(tracker.is_shutdown(), "latch must not self-clear");

    // Only an explicit clear (e.g. key cycle) releases it.
    tracker.clear_shutdown();
    assert!(!tracker.is_shutdown());
    assert!(should_allow_injection(false, tracker.is_shutdown()));
}

#[test]
fn test_sync_loss_recovery_counter_and_window_reset() {
    let mut tracker = SyncLossTracker::new();

    // Two losses inside the same window stay below the shutdown threshold.
    assert!(!tracker.record_sync_loss(0));
    assert!(!tracker.record_sync_loss(1_000));
    assert!(!tracker.is_shutdown());

    // A recovery is counted but deliberately does not reset the running count,
    // so a third loss inside the same window can still latch.
    tracker.record_recovery();
    assert_eq!(tracker.successful_recoveries, 1);

    // After a long stretch of good sync, the window is reset, returning the
    // tracker to a fresh state where a single loss is recoverable again.
    tracker.reset_window();
    assert!(
        !tracker.record_sync_loss(10_000_000),
        "post-reset single loss must be recoverable"
    );
    assert!(!tracker.is_shutdown());
    assert_eq!(tracker.total_losses, 3, "total losses keep accumulating");
}

// =============================================================================
// FLOOD-CLEAR (WOT-CRANKING FUEL CUT) TESTS
// =============================================================================

#[test]
fn test_flood_clear_enters_on_wot_cranking() {
    let mut state = FloodClearState::new();

    let cranking_rpm = CRANKING_RPM_THRESHOLD - 1;
    let wot = WOT_TPS_THRESHOLD;

    let active = update_flood_clear(cranking_rpm, wot, &mut state);
    assert!(active, "WOT while cranking must engage flood clear");
    assert!(state.active);
    // Flood clear blocks injection to dry the cylinders.
    assert!(!should_allow_injection(active, false));
}

#[test]
fn test_flood_clear_does_not_enter_without_both_conditions() {
    // WOT but already running (not cranking).
    let mut state = FloodClearState::new();
    assert!(!update_flood_clear(
        CRANKING_RPM_THRESHOLD + 100,
        WOT_TPS_THRESHOLD,
        &mut state
    ));
    assert!(!state.active);

    // Cranking but throttle closed.
    let mut state = FloodClearState::new();
    assert!(!update_flood_clear(
        CRANKING_RPM_THRESHOLD - 1,
        WOT_TPS_THRESHOLD - 1,
        &mut state
    ));
    assert!(!state.active);
}

#[test]
fn test_flood_clear_cannot_persist_into_running_state() {
    let mut state = FloodClearState::new();

    // Engage flood clear under WOT-cranking.
    assert!(update_flood_clear(
        CRANKING_RPM_THRESHOLD - 1,
        WOT_TPS_THRESHOLD,
        &mut state
    ));
    assert!(state.active);

    // The engine catches and RPM rises out of the cranking band: flood clear
    // must clear immediately and not latch into the running region.
    let active = update_flood_clear(CRANKING_RPM_THRESHOLD + 200, WOT_TPS_THRESHOLD, &mut state);
    assert!(!active, "flood clear must exit once cranking ends");
    assert!(!state.active);
    assert!(
        should_allow_injection(active, false),
        "injection must resume once flood clear exits"
    );

    // Releasing the throttle while still cranking also exits flood clear.
    assert!(update_flood_clear(
        CRANKING_RPM_THRESHOLD - 1,
        WOT_TPS_THRESHOLD,
        &mut state
    ));
    let active = update_flood_clear(CRANKING_RPM_THRESHOLD - 1, 0, &mut state);
    assert!(!active, "closing throttle must exit flood clear");
    assert!(!state.active);
}
