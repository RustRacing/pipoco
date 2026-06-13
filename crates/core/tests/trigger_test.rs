use ecu_core::hal::TimeSource;
use ecu_core::{compat::EcuState, scale_u16, IpwTable, TriggerDecoder};
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

#[test]
fn test_trigger_sync_detection() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Simulate normal teeth (1ms each)
    for i in 0..57 {
        // 57 normal teeth before missing tooth
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }

    // Should not be synced yet (need to see the gap)
    assert!(!decoder.synced());

    // Simulate missing tooth gap (2ms instead of 1ms)
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(57 * 1000 + 2000); // Missing tooth gap
    decoder.tooth_edge();

    // Should sync now
    assert!(decoder.synced());
    assert_eq!(decoder.tooth(), 1);
}

#[test]
fn test_rpm_calculation() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // Simulate 1000 RPM
    // 1000 RPM = 16.67 rev/s = 60ms/rev
    // 58 teeth per rev = ~1034us per tooth
    let tooth_period = 1034;

    // Simulate teeth until sync
    for i in 0..57 {
        decoder.time_source().set_time(tooth_period * i);
        decoder.tooth_edge();
    }

    // Trigger sync with missing tooth gap (2x normal period)
    decoder.time_source().set_time(tooth_period * 57);
    decoder.tooth_edge();
    decoder
        .time_source()
        .set_time(tooth_period * 57 + tooth_period * 2);
    decoder.tooth_edge();

    // Check RPM (should be close to 1000)
    // Due to approximation in RPM calc, accept wide range for MVP
    let rpm = decoder.rpm().raw();
    assert!((800..=1200).contains(&rpm), "RPM was {rpm}, expected ~1000");
}

#[test]
fn test_loss_of_sync() {
    let time_source = MockTime::new(0);
    let mut decoder = TriggerDecoder::new(time_source);

    // First, establish sync
    for i in 0..57 {
        decoder.time_source().set_time(i * 1000);
        decoder.tooth_edge();
    }
    decoder.time_source().set_time(57 * 1000);
    decoder.tooth_edge();
    decoder.time_source().set_time(59 * 1000); // Missing tooth gap
    decoder.tooth_edge();

    assert!(decoder.synced());

    // Now simulate signal loss (no teeth for 200ms+)
    decoder.time_source().set_time(59 * 1000 + 250_000); // 250ms later
    decoder.tooth_edge();

    // Should lose sync
    assert!(!decoder.synced());
    assert_eq!(decoder.rpm().raw(), 0);
}

#[test]
fn test_table_lookup() {
    let table = IpwTable::new();

    // Test lookup at known point
    let pw = table.lookup(3000, 60);

    // Should return default value (1000us)
    assert_eq!(pw, 1000);
}

#[test]
fn test_correction_multiply() {
    // Test 1.5x correction (150)
    assert_eq!(scale_u16(1000, 150), 1500);

    // Test 0.8x correction (80)
    assert_eq!(scale_u16(1000, 80), 800);

    // Test 1.0x correction (100)
    assert_eq!(scale_u16(1000, 100), 1000);
}

#[test]
fn test_fuel_calculation() {
    let state = EcuState::new();

    // Calculate fuel at 3000 RPM, 60 kPa load
    let pw = state.calculate_fuel(3000, 60);

    // Should be default value with 1.0x corrections
    assert_eq!(pw, 1000);
}

#[test]
fn test_fuel_calculation_with_corrections() {
    let mut state = EcuState::new();

    // Apply 1.5x coolant correction
    state.corrections_mut().clt = 150;

    let pw = state.calculate_fuel(3000, 60);

    // Should be 1000 * 1.5 = 1500
    assert_eq!(pw, 1500);
}

#[test]
fn test_table_cell_independence() {
    let mut state = EcuState::new();

    // Modify one table cell - table is [load_idx][rpm_idx]
    // 3000 RPM -> idx 5, 60 kPa -> idx 4
    state.config.ipw_table[4][5] = 2000;

    // Lookup should return modified value
    let pw = state.calculate_fuel(3000, 60); // Maps to bin [4][5]
    assert_eq!(pw, 2000);

    // Other cells should be unaffected
    let pw2 = state.calculate_fuel(1000, 30); // Different bin
    assert_eq!(pw2, 1000);
}
