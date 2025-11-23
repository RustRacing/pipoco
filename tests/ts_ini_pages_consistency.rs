use ecu_core::ts::pages::{EcuPageStore, PAGE_LIMITS, PAGE_DIAG, PAGE_DIAG_LOG, PAGE_ANGLES};
use ecu_core::ts::PageStore;
use ecu_core::EcuState;
use std::collections::HashMap;
use std::fs;

#[test]
fn ini_page_sizes_match_firmware() {
    // Parse sizes from INI sections
    let ini = fs::read_to_string("../ts/IPW-ECU.ini").expect("load ini");
    let mut sizes: HashMap<String, usize> = HashMap::new();
    let mut current = String::new();
    for line in ini.lines() {
        let l = line.trim();
        if l.starts_with('[') && l.ends_with(']') {
            current = l.trim_matches(&['[', ']'][..]).to_string();
        } else if l.starts_with("size") {
            if let Some(v) = l.split('=').nth(1) { if let Ok(n) = v.trim().parse::<usize>() { sizes.insert(current.clone(), n); } }
        }
    }
    // Build a page store to read code-side sizes
    let mut state = EcuState::new();
    let store = EcuPageStore {
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
    // Compare known pages
    let diag_size = store.page_len(PAGE_DIAG).unwrap();
    let diag_log_size = store.page_len(PAGE_DIAG_LOG).unwrap();
    let limits_size = store.page_len(PAGE_LIMITS).unwrap();
    let angles_size = store.page_len(PAGE_ANGLES).unwrap();
    assert_eq!(sizes.get("Diag").copied(), Some(diag_size));
    assert_eq!(sizes.get("DiagLog").copied(), Some(diag_log_size));
    assert_eq!(sizes.get("Limits").copied(), Some(limits_size));
    assert_eq!(sizes.get("Angles").copied(), Some(angles_size));
}

