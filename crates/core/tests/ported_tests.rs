/// Tests ported from RusEFI and Speeduino
/// Adapted to work with our simplified IPW-based architecture
use ecu_core::{hal::TimeSource, scale_u16, EcuState, IpwTable, TriggerDecoder};
use std::cell::Cell;

// Mock time source for testing with interior mutability
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

/// Ported from RusEFI: test_trigger_decoder.cpp::testNoStartUpWarnings
/// Verify decoder doesn't sync with insufficient teeth
#[test]
fn test_no_premature_sync() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Send only a few teeth - should NOT sync yet
    for i in 0..10 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }

    assert!(!decoder.synced(), "Should not sync with only 10 teeth");
    assert_eq!(decoder.rpm().raw(), 0, "RPM should be 0 before sync");
}

/// Ported from RusEFI: test_trigger_decoder.cpp
/// Test that decoder maintains sync after achieving it
#[test]
fn test_sync_persistence() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Simulate full revolution to achieve sync
    for i in 0..57 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }

    // Missing tooth for sync
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(57 * 1000 + 2000);
    decoder.tooth_edge();

    assert!(decoder.synced(), "Should be synced after missing tooth");

    // Continue through multiple revolutions
    for revolution in 0..5 {
        for tooth in 0..57 {
            let time = (60000 + revolution * 60000 + tooth * 1000) as u32;
            decoder.time_source().set_time(time);
            decoder.tooth_edge();
        }
        // Missing tooth
        let time = (60000 + revolution * 60000 + 57 * 1000 + 2000) as u32;
        decoder.time_source().set_time(time);
        decoder.tooth_edge();

        assert!(
            decoder.synced(),
            "Should maintain sync through revolution {revolution}"
        );
    }
}

/// Ported from RusEFI: test_trigger_decoder.cpp
/// Test RPM calculation at various speeds
#[test]
fn test_rpm_at_various_speeds() {
    let test_cases = vec![
        (500, 2068),  // 500 RPM -> ~2ms per tooth
        (1000, 1034), // 1000 RPM -> ~1ms per tooth
        (2000, 517),  // 2000 RPM -> ~0.5ms per tooth
    ];

    for (target_rpm, tooth_period_us) in test_cases {
        let time_source = MockTime::new(0);
        let mut decoder = TriggerDecoder::new(time_source);

        // Simulate revolution
        for i in 0..57 {
            decoder.time_source().set_time(tooth_period_us * i);
            decoder.tooth_edge();
        }

        // Missing tooth for sync
        decoder.time_source().set_time(tooth_period_us * 57);
        decoder.tooth_edge();
        decoder
            .time_source()
            .set_time(tooth_period_us * 57 + tooth_period_us * 2);
        decoder.tooth_edge();

        let measured_rpm = decoder.rpm().raw();
        let tolerance = target_rpm as f32 * 0.25; // 25% tolerance for MVP approximation
        let mr = measured_rpm as i32;
        let tr = target_rpm;

        assert!(
            (mr - tr).abs() < tolerance as i32,
            "RPM at {target_rpm} target: got {measured_rpm}, expected within 25% tolerance",
        );
    }
}

/// Ported from Speeduino: test_table3d_native.cpp
/// Test table lookup correctness
#[test]
fn test_table_lookup_corners() {
    let table = IpwTable::new();

    // Test corner lookups
    let pw_low_low = table.lookup(500, 20); // Bottom-left
    let pw_low_high = table.lookup(500, 170); // Top-left
    let pw_high_low = table.lookup(8000, 20); // Bottom-right
    let pw_high_high = table.lookup(8000, 170); // Top-right

    // All should be default value
    assert_eq!(pw_low_low, 1000);
    assert_eq!(pw_low_high, 1000);
    assert_eq!(pw_high_low, 1000);
    assert_eq!(pw_high_high, 1000);
}

/// Ported from Speeduino: test_table3d_native.cpp
/// Test table lookup with out-of-bounds values
#[test]
fn test_table_lookup_out_of_bounds() {
    let table = IpwTable::new();

    // Values below minimum bins should use first bin
    let pw_below = table.lookup(0, 0);
    assert_eq!(pw_below, 1000);

    // Values above maximum bins should use last bin
    let pw_above = table.lookup(10000, 200);
    assert_eq!(pw_above, 1000);
}

/// Ported from Speeduino: test_fuel.cpp
/// Test correction factors apply correctly
#[test]
fn test_multiple_corrections() {
    let mut state = EcuState::new();

    // Apply multiple corrections
    state.corrections_mut().clt = 120; // 1.2x for cold
    state.corrections_mut().iat = 110; // 1.1x for cold air
    state.corrections_mut().vbatt = 95; // 0.95x for low voltage

    let pw = state.calculate_fuel(3000, 60);

    // Expected: 1000 * 1.2 * 1.1 * 0.95 = 1254
    // With integer rounding it should be close
    assert!((1250..=1260).contains(&pw), "Expected ~1254, got {pw}");
}

/// Test that individual table cells are independent
#[test]
fn test_table_cell_independence() {
    let mut state = EcuState::new();

    // Modify several cells - remember table is [load_idx][rpm_idx]
    // RPM 500 -> idx 0, Load 20 -> idx 0
    state.config.ipw_table[0][0] = 500; // Low RPM, low load
                                        // RPM 8000 -> idx 15, Load 170 -> idx 15
    state.config.ipw_table[15][15] = 3000; // High RPM, high load
                                           // RPM 3500 -> idx 6, Load 100 -> idx 8
    state.config.ipw_table[8][6] = 1500; // Middle

    // Verify each lookup returns correct value
    let pw1 = state.calculate_fuel(500, 20); // Should hit [0][0]
    let pw2 = state.calculate_fuel(8000, 170); // Should hit [15][15]
    let pw3 = state.calculate_fuel(3500, 100); // Should hit [8][6]

    assert_eq!(pw1, 500);
    assert_eq!(pw2, 3000);
    assert_eq!(pw3, 1500);
}

/// Test correction clamping to prevent overflow
#[test]
fn test_extreme_corrections() {
    let mut state = EcuState::new();

    // Extreme low correction
    state.corrections_mut().clt = 1; // 0.01x (almost zero)
    let pw_low = state.calculate_fuel(3000, 60);
    assert_eq!(pw_low, 500, "Should clamp to minimum");

    // Extreme high correction - need high base value to exceed max
    // 3000 RPM -> idx 5, 60 kPa -> idx 4, table is [load_idx][rpm_idx]
    state.config.ipw_table[4][5] = 15000;
    state.corrections_mut().clt = 255;
    state.corrections_mut().iat = 255;
    state.corrections_mut().vbatt = 255;
    let pw_high = state.calculate_fuel(3000, 60);
    assert_eq!(pw_high, 20000, "Should clamp to maximum");
}

/// Test timer overflow wrapping
#[test]
fn test_time_wrapping() {
    let time_source = MockTime::new(u32::MAX - 10000);
    let mut decoder = TriggerDecoder::new(time_source);

    // Simulate teeth near overflow
    for i in 0..57 {
        let time = (u32::MAX - 10000).wrapping_add(i * 1000);
        decoder.time_source().set_time(time);
        decoder.tooth_edge();
    }

    // Missing tooth that wraps around
    decoder
        .time_source()
        .set_time((u32::MAX - 10000).wrapping_add(57 * 1000));
    decoder.tooth_edge();
    decoder
        .time_source()
        .set_time((u32::MAX - 10000).wrapping_add(59 * 1000));
    decoder.tooth_edge();

    // Should still sync correctly with wrapping
    assert!(decoder.synced(), "Should handle time wrapping");
}

/// Test loss of sync detection
#[test]
fn test_sync_loss_on_stall() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Establish sync
    for i in 0..57 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(59000);
    decoder.tooth_edge();

    assert!(decoder.synced());
    let last_time = 59000;

    // Simulate stall - next tooth comes after > 200ms timeout
    // The period between teeth will be > SYNC_TIMEOUT_US (200ms)
    decoder.time_source().set_time(last_time + 250_000); // 250ms later
    decoder.tooth_edge();

    // Should have lost sync due to timeout
    assert!(!decoder.synced(), "Should lose sync after timeout");
    assert_eq!(decoder.rpm().raw(), 0, "RPM should be 0 after sync loss");
}

/// Test linear table initialization helper
#[test]
fn test_linear_table_init() {
    let mut state = EcuState::new();
    state.init_linear_table();

    // Verify table has increasing values with load
    let pw_low_load = state.calculate_fuel(3000, 20); // Low load
    let pw_high_load = state.calculate_fuel(3000, 170); // High load

    assert!(
        pw_high_load > pw_low_load,
        "Higher load should give more fuel: {pw_high_load} vs {pw_low_load}"
    );
}

/// Test that scale_u16 uses saturating arithmetic
#[test]
fn test_scale_u16_saturation() {
    // Test normal scaling
    assert_eq!(scale_u16(1000, 150), 1500);
    assert_eq!(scale_u16(1000, 50), 500);

    // Test saturation at max
    assert_eq!(scale_u16(u16::MAX, 200), u16::MAX);
    assert_eq!(scale_u16(50000, 200), u16::MAX);

    // Test that zero works
    assert_eq!(scale_u16(1000, 0), 0);
    assert_eq!(scale_u16(0, 150), 0);
}
