use ecu_core::ts::pages::{EcuPageStore, PAGE_DIAG};
use ecu_core::ts::PageStore;
use ecu_core::EcuState;

#[test]
fn diag_page_flags_reflect_state() {
    let mut state = EcuState::new();
    // Enable emergency triggers to demonstrate flag set when out of range
    state.emergency_trigger_map_oob = true;
    state.sensors_limits.map_min_kpa_x10 = 500;
    state.sensors_limits.map_max_kpa_x10 = 3000;
    state.sensors_limits.clear_time_s = 1;

    // Initially, no flags should be set
    let mut out = [0u8; 4];
    {
        let pages = EcuPageStore {
            fuel: &mut state.ipw_table,
            ign: &mut state.ignition_table,
            sens: &mut state.sensors_cal,
            idle: &mut state.idle_config,
            fan: &mut state.fan_config,
            cl: &mut state.cl_config,
            ae: &mut state.ae_config,
            wue: &mut state.wue_config,
            ase: &mut state.ase_config,
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
        let n = pages.read_page(PAGE_DIAG, &mut out).expect("read diag");
        assert_eq!(n, 4);
        let flags = u16::from_le_bytes([out[0], out[1]]);
        assert_eq!(flags, 0);
    }

    // Cause MAP out-of-range -> diag_map active and emergency true
    let _ = state.process_sensor_update(0, 100, 10);
    {
        let pages = EcuPageStore {
            fuel: &mut state.ipw_table,
            ign: &mut state.ignition_table,
            sens: &mut state.sensors_cal,
            idle: &mut state.idle_config,
            fan: &mut state.fan_config,
            cl: &mut state.cl_config,
            ae: &mut state.ae_config,
            wue: &mut state.wue_config,
            ase: &mut state.ase_config,
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
        let n = pages.read_page(PAGE_DIAG, &mut out).expect("read diag");
        assert_eq!(n, 4);
        let flags = u16::from_le_bytes([out[0], out[1]]);
        assert!(flags & (1 << 0) != 0, "MAP diag should be active");
        assert!(flags & (1 << 3) != 0, "Emergency should be active");
    }
}

#[test]
fn diag_page_cam_flag_sets_when_missing() {
    let mut state = EcuState::new();
    state.diag_cam.active = true;

    let pages = EcuPageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
        sens: &mut state.sensors_cal,
        idle: &mut state.idle_config,
        fan: &mut state.fan_config,
        cl: &mut state.cl_config,
        ae: &mut state.ae_config,
        wue: &mut state.wue_config,
        ase: &mut state.ase_config,
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
    let mut out = [0u8; 4];
    let n = pages.read_page(PAGE_DIAG, &mut out).expect("read diag");
    assert_eq!(n, 4);
    let flags = u16::from_le_bytes([out[0], out[1]]);
    assert!(flags & (1 << 2) != 0, "cam diag flag should be set");
}
