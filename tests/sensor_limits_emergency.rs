use ecu_core::EcuState;

#[test]
fn map_oob_triggers_emergency_and_clears_after_stable() {
    let mut s = EcuState::new();
    s.emergency_trigger_map_oob = true;
    s.sensors_limits.map_min_kpa_x10 = 200; // 20.0 kPa
    s.sensors_limits.map_max_kpa_x10 = 3000; // 300.0 kPa
    s.sensors_limits.clear_time_s = 1; // 1 second

    // Out of range low
    let (map, _tps) = s.process_sensor_update(0, 100, 50);
    assert_eq!(map, 200);
    assert!(s.diag_map.active);
    assert!(s.emergency_mode);

    // Back in range briefly -> not clear yet
    let (_map, _tps) = s.process_sensor_update(500_000, 500, 50);
    assert!(s.diag_map.active);
    assert!(s.emergency_mode);

    // After clear_time_s in-range, diag clears and emergency clears
    let (_map, _tps) = s.process_sensor_update(1_600_000, 600, 50);
    assert!(!s.diag_map.active);
    assert!(!s.emergency_mode);
    // Log should have at least one event recorded
    assert!(s.diag_log.events.iter().any(|e| e.is_some()));
}
