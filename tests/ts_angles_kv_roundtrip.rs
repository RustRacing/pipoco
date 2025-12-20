use ecu_core::EcuState;
use ecu_target_common::kv::ram::RamKv512;
use ecu_core::ts::PageStore;
use ecu_target_common::ts::store::PersistedEcuPageStore;
use ecu_core::persist::KvStore;

// Shared KV wrapper so we can reuse the same backing buffer across two stores
struct SharedKv {
    inner: std::rc::Rc<std::cell::RefCell<RamKv512>>,
}
impl SharedKv {
    fn new() -> Self {
        Self { inner: std::rc::Rc::new(std::cell::RefCell::new(RamKv512::new())) }
    }
}
impl Clone for SharedKv {
    fn clone(&self) -> Self { Self { inner: self.inner.clone() } }
}
impl KvStore for SharedKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, ecu_core::persist::KvError> { self.inner.borrow_mut().read(key, out) }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), ecu_core::persist::KvError> { self.inner.borrow_mut().write(key, data) }
}

#[test]
fn angles_kv_roundtrip() {
    let mut state = EcuState::new();
    // Set non-default angle values
    state.inj_angle_btdc_x10[0] = 150; // 15.0°
    state.inj_angle_btdc_x10[1] = 220; // 22.0°
    state.tdc_per_cyl_x10[0] = 100; // 10.0°
    state.tdc_per_cyl_x10[1] = 280; // 28.0°
    state.tooth0_angle_x10 = 35; // 3.5°
    state.cam_missing_timeout_ms = 750;

    // Encode angles page and write to KV directly (avoid 512-byte table writes)
    let kv = SharedKv::new();
    {
        let mut buf = [0u8; 68];
        // Build a transient page store to read angles into buf
        let pages = ecu_core::ts::pages::EcuPageStore {
            fuel: &mut state.ipw_table,
            ign: &mut state.ignition_table,
            sens: &mut state.sensors_cal,
            idle: &mut state.idle_config,
            fan: &mut state.fan_config,
            cl: &mut state.cl_config,
            wue: &mut state.wue_config,
            ase: &mut state.ase_config,
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
        let n = pages.read_page(ecu_core::ts::pages::PAGE_ANGLES, &mut buf).expect("angles read");
        assert_eq!(n, 68);
        let mut kvw = kv.clone();
        kvw.write(b"angles", &buf).expect("kv write angles");
    }

    // Reset angles to zeros
    state.inj_angle_btdc_x10 = [0; 16];
    state.tdc_per_cyl_x10 = [0; 16];
    state.tooth0_angle_x10 = 0;
    state.cam_missing_timeout_ms = 0;

    // Load back from KV via PersistedEcuPageStore
    let mut store2 = PersistedEcuPageStore::new(&mut state, kv.clone());
    store2.try_load();

    assert_eq!(state.inj_angle_btdc_x10[0], 150);
    assert_eq!(state.inj_angle_btdc_x10[1], 220);
    assert_eq!(state.tdc_per_cyl_x10[0], 100);
    assert_eq!(state.tdc_per_cyl_x10[1], 280);
    assert_eq!(state.tooth0_angle_x10, 35);
    assert_eq!(state.cam_missing_timeout_ms, 750);
}
