use ecu_core::compat::EcuState;
use ecu_core::{Kpa10, Micros};

#[test]
fn map_oob_triggers_emergency_and_clears_after_stable() {
    let mut s = EcuState::new();
    s.set_emergency_trigger_map_oob(true);
    s.config.sensors_limits.map_min_kpa_x10 = 200; // 20.0 kPa
    s.config.sensors_limits.map_max_kpa_x10 = 3000; // 300.0 kPa
    s.config.sensors_limits.clear_time_s = 1; // 1 second

    // Out of range low
    let (map, _tps) = s.process_sensor_update(Micros::new(0), Kpa10::new(100), 50);
    assert_eq!(map.raw(), 200);
    assert!(s.diag_map.is_active());
    assert!(s.emergency_mode());

    // Back in range briefly -> not clear yet
    let (_map, _tps) = s.process_sensor_update(Micros::new(500_000), Kpa10::new(500), 50);
    assert!(s.diag_map.is_active());
    assert!(s.emergency_mode());

    // After clear_time_s in-range, diag clears and emergency clears
    let (_map, _tps) = s.process_sensor_update(Micros::new(1_600_000), Kpa10::new(600), 50);
    assert!(!s.diag_map.is_active());
    assert!(!s.emergency_mode());
    // Log should have at least one event recorded
    assert!(s.diag_log().events.iter().any(|e| e.is_some()));
}
