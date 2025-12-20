use ecu_core::ts::pages::{EcuPageStore, PAGE_DIAG_LOG};
use ecu_core::ts::PageStore;
use ecu_core::EcuState;

#[test]
fn diag_log_wraps_and_retains_recent() {
    let mut state = EcuState::new();
    // Configure limits and clear time so we can generate events
    state.sensors_limits.map_min_kpa_x10 = 500;
    state.sensors_limits.map_max_kpa_x10 = 3000;
    state.sensors_limits.clear_time_s = 1;

    // Generate >16 events by toggling OOB -> in-range sustained
    // Each cycle: push OOB at t, then two in-range updates to exceed clear_time and emit event
    let mut now = 0u32;
    for _ in 0..20 {
        let _ = state.process_sensor_update(now, 100, 10); // OOB
        now = now.wrapping_add(1_500_000); // 1.5s later back in-range
        let _ = state.process_sensor_update(now, 1000, 10);
        now = now.wrapping_add(1_500_000); // sustain
        let _ = state.process_sensor_update(now, 1000, 10);
        now = now.wrapping_add(10_000);
    }

    let pages = EcuPageStore { fuel: &mut state.ipw_table, ign: &mut state.ignition_table, sens: &mut state.sensors_cal, idle: &mut state.idle_config, fan: &mut state.fan_config, cl: &mut state.cl_config, wue: &mut state.wue_config, ase: &mut state.ase_config, ae: &mut state.ae_config, dfco: &mut state.dfco_config, limits: &mut state.sensors_limits, emerg_trig_map: &mut state.emergency_trigger_map_oob, emerg_trig_tps: &mut state.emergency_trigger_tps_oob, diag_emergency: &state.emergency_mode, diag_map: &state.diag_map, diag_tps: &state.diag_tps, diag_cam: &state.diag_cam, diag_log: &state.diag_log, angles_inj: &mut state.inj_angle_btdc_x10, angles_tdc: &mut state.tdc_per_cyl_x10, tooth0_angle_x10: &mut state.tooth0_angle_x10, cam_timeout_ms: &mut state.cam_missing_timeout_ms };

    let mut out = [0u8; 16 * 9];
    let _ = pages.read_page(PAGE_DIAG_LOG, &mut out).expect("read log");
    // Count non-empty entries (code != 0)
    let count = out
        .chunks_exact(9)
        .filter(|e| e[0] != 0)
        .count();
    assert_eq!(count, 16, "ring buffer should contain 16 recent events");
}
