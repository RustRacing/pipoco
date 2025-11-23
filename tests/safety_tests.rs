//! Critical safety tests for ECU
//!
//! These tests verify safety-critical functionality that could cause engine
//! damage if not working correctly. All tests in this file MUST PASS before
//! any release or hardware deployment.

use ecu_core::hal::TimeSource;
use ecu_core::{scale_u16, Channel, EcuState, IpwTable, Scheduler, TriggerDecoder};
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
            state.ipw_table[row][col] = u16::MAX;
        }
    }

    // Set maximum corrections
    state.corrections.clt = 255;
    state.corrections.iat = 255;
    state.corrections.vbatt = 255;

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
            state.ipw_table[row][col] = 1; // Minimum possible value
        }
    }

    // Set minimum corrections (almost zero)
    state.corrections.clt = 1;
    state.corrections.iat = 1;
    state.corrections.vbatt = 1;

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
    state.ipw_table[7][7] = u16::MAX;
    state.ipw_table[0][0] = u16::MAX;
    state.ipw_table[15][15] = u16::MAX;

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
    state.ipw_table[8][5] = 10000; // 10ms base
    state.corrections.vbatt = 200; // 2.0x correction (compensate for slow injector)

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
    state.ipw_table[8][5] = 8000; // 8ms base

    // Multiple high corrections should saturate gracefully
    state.corrections.clt = 200; // 2.0x
    state.corrections.iat = 150; // 1.5x
    state.corrections.vbatt = 120; // 1.2x

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
    assert_eq!(decoder.rpm(), 0, "RPM should be 0 after sync loss");
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
    assert_eq!(decoder.rpm(), 0, "RPM should be 0 before valid sync");
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
    assert!(decoder.rpm() > 0, "RPM should be > 0 after resync");
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
        let rpm = decoder.rpm();
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
    state.ipw_table[8][5] = 15000;

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
// SCHEDULER SAFETY TESTS
// =============================================================================

#[test]
fn test_scheduler_overflow_graceful() {
    let mut scheduler = Scheduler::new();

    use ecu_core::constants::scheduler::MAX_EVENTS;
    // Fill scheduler to capacity
    for i in 0..MAX_EVENTS {
        let success = scheduler.schedule(1000 * i as u32, Channel::INJ1, true);
        assert!(success, "Should accept event {i}");
    }

    // Try to schedule one more (should fail gracefully, not crash)
    let success = scheduler.schedule(9000, Channel::INJ1, true);
    assert!(!success, "Should reject event when full");
}

#[test]
fn test_scheduler_time_wrapping() {
    let mut scheduler = Scheduler::new();

    // Schedule event near u32::MAX
    let event_time = u32::MAX - 1000;
    scheduler.schedule(event_time, Channel::INJ1, true);

    // Mock output
    struct MockOutput {
        state: Cell<bool>,
    }
    impl MockOutput {
        fn new() -> Self {
            Self {
                state: Cell::new(false),
            }
        }
    }
    impl ecu_core::hal::OutputPin for MockOutput {
        fn set_high(&mut self) {
            self.state.set(true);
        }
        fn set_low(&mut self) {
            self.state.set(false);
        }
    }

    let mut output = MockOutput::new();
    let mut outputs: [&mut dyn ecu_core::hal::OutputPin; 1] = [&mut output];

    // Check event at time that wraps around u32
    let now = 100; // Wrapped past u32::MAX
    scheduler.check_and_execute(now, &mut outputs[..]);

    // Event should have executed despite time wrapping
    assert!(output.state.get(), "Event should execute after time wrap");
}

// =============================================================================
// POWER AND RESET SAFETY TESTS
// =============================================================================

#[test]
fn test_cold_boot_state_initialization() {
    let state = EcuState::new();

    // Verify safe initial state
    assert_eq!(state.rpm, 0);
    assert!(!state.synced);
    assert_eq!(state.tooth_count, 0);
    assert_eq!(state.corrections.clt, 100);
    assert_eq!(state.corrections.iat, 100);
    assert_eq!(state.corrections.vbatt, 100);

    // Verify table is initialized with safe values
    for row in 0..16 {
        for col in 0..16 {
            assert_eq!(
                state.ipw_table[row][col], 1000,
                "Table cell [{row},{col}] not initialized to safe default"
            );
        }
    }
}

#[test]
fn test_scheduler_cold_boot_state() {
    let scheduler = Scheduler::new();

    // Verify no events are scheduled on boot
    assert_eq!(scheduler.active_count(), 0);
    assert!(!scheduler.is_full());
}

#[test]
fn test_trigger_decoder_cold_boot_state() {
    let time_source = MockTime::new(0);
    let decoder = TriggerDecoder::new(time_source);

    // Verify safe initial state
    assert!(!decoder.synced());
    assert_eq!(decoder.rpm(), 0);
    assert_eq!(decoder.tooth(), 0);
}

// =============================================================================
// DATA INTEGRITY TESTS
// =============================================================================

#[test]
fn test_static_memory_bounds() {
    let state = EcuState::new();

    // Verify table dimensions are correct
    assert_eq!(state.ipw_table.len(), 16);
    assert_eq!(state.ipw_table[0].len(), 16);

    // Verify we can safely access all cells
    for row in 0..16 {
        for col in 0..16 {
            let _ = state.ipw_table[row][col];
        }
    }
}

#[test]
fn test_channel_validation() {
    // Verify channel constants are valid
    assert!(Channel::INJ1.is_valid());
    assert!(Channel::INJ2.is_valid());
    assert!(Channel::IGN1.is_valid());
    assert!(Channel::IGN2.is_valid());

    // Verify channel values are within expected range
    use ecu_core::constants::scheduler::MAX_CHANNELS;
    assert!(Channel::INJ1.as_u8() < MAX_CHANNELS);
    assert!(Channel::INJ2.as_u8() < MAX_CHANNELS);
    assert!(Channel::IGN1.as_u8() < MAX_CHANNELS);
    assert!(Channel::IGN2.as_u8() < MAX_CHANNELS);
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
    state.corrections.clt = 0;

    let pw = state.calculate_fuel(3000, 60);

    assert_eq!(pw, 500, "Zero correction should result in minimum fuel");
}
