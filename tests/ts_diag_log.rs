use ecu_core::ts::pages::{EcuPageStore, PAGE_DIAG_LOG};
use ecu_core::ts::PageStore;
use ecu_core::EcuState;

#[test]
fn diag_log_contains_events_after_fault() {
    let mut state = EcuState::new();
    state.emergency_trigger_map_oob = false; // don't care for this test
    state.sensors_limits.map_min_kpa_x10 = 500;
    state.sensors_limits.map_max_kpa_x10 = 3000;
    state.sensors_limits.clear_time_s = 1;

    // Simulate an out-of-range then sustained in-range to push an event into log
    let _ = state.process_sensor_update(0, 100, 10);
    let _ = state.process_sensor_update(1_500_000, 1000, 10); // back in range
    let _ = state.process_sensor_update(3_000_000, 1000, 10); // sustain past clear time

    let pages = EcuPageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
        sens: &mut state.sensors_cal,
        ae: &mut state.ae_config,
        dfco: &mut state.dfco_config,
        limits: &mut state.sensors_limits,
        emerg_trig_map: &mut state.emergency_trigger_map_oob,
        emerg_trig_tps: &mut state.emergency_trigger_tps_oob,
        diag_emergency: &state.emergency_mode,
        diag_map: &state.diag_map,
        diag_tps: &state.diag_tps,
        diag_cam: &state.diag_cam,
        diag_log: &state.diag_log,
        angles_inj: &mut state.inj_angle_btdc_x10,
        angles_tdc: &mut state.tdc_per_cyl_x10,
        tooth0_angle_x10: &mut state.tooth0_angle_x10,
        cam_timeout_ms: &mut state.cam_missing_timeout_ms,
    };

    let mut out = [0u8; 16 * 9];
    let n = pages.read_page(PAGE_DIAG_LOG, &mut out).expect("read diag log");
    assert_eq!(n, 16 * 9);
    // First entry should be non-zero code
    assert!(out[0] != 0, "expected at least one diag event present");
}
