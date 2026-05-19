use ecu_core::persist::{KvError, KvStore};
use ecu_core::ts::PageStore;
use ecu_core::EcuState;
use ecu_target_common::ts::store::PersistedEcuPageStore;

// KV that simulates CRC/version mismatch by returning NotFound/Io
#[derive(Clone, Default)]
struct FailingKv;
impl KvStore for FailingKv {
    fn read(&mut self, _key: &[u8], _out: &mut [u8]) -> Result<usize, KvError> {
        Err(KvError::NotFound)
    }
    fn write(&mut self, _key: &[u8], _data: &[u8]) -> Result<(), KvError> {
        Err(KvError::Io)
    }
}

#[test]
fn try_load_with_kv_failure_keeps_defaults() {
    let mut state = EcuState::new();
    // Perturb state to verify no change on failed load
    state.config.ipw_table[0][0] = 1234;
    state.config.ignition_table[0][0] = -7;
    state.config.inj_angle_btdc_x10[0] = 111;
    state.config.cam_missing_timeout_ms = 250;

    let mut store = PersistedEcuPageStore::new(&mut state, FailingKv);
    store.try_load();
    // Expect unmodified because KV has no data
    assert_eq!(state.config.ipw_table[0][0], 1234);
    assert_eq!(state.config.ignition_table[0][0], -7);
    assert_eq!(state.config.inj_angle_btdc_x10[0], 111);
    assert_eq!(state.config.cam_missing_timeout_ms, 250);
}

#[test]
fn factory_reset_restores_safe_defaults() {
    let mut state = EcuState::new();
    // Modify state
    state.config.ipw_table[0][0] = 9999;
    state.config.ignition_table[0][0] = 45;
    state.config.inj_angle_btdc_x10[0] = 99;
    state.config.cam_missing_timeout_ms = 250;
    let mut store = PersistedEcuPageStore::new(&mut state, FailingKv);
    store.factory_reset();

    // Read back via pages
    let mut fuel = [0u8; 512];
    let mut ign = [0u8; 512];
    let mut angles = [0u8; 68];
    let n1 = store
        .read_page(ecu_core::ts::pages::PAGE_FUEL, &mut fuel)
        .unwrap();
    let n2 = store
        .read_page(ecu_core::ts::pages::PAGE_IGN, &mut ign)
        .unwrap();
    let n3 = store
        .read_page(ecu_core::ts::pages::PAGE_ANGLES, &mut angles)
        .unwrap();
    assert_eq!(n1, 512);
    assert_eq!(n2, 512);
    assert_eq!(n3, 68);
    // Defaults for tables are serialized from state defaults in writer
    // Verify angles cam timeout default 500
    let cam_to = u16::from_le_bytes([angles[66], angles[67]]);
    assert_eq!(cam_to, 500);
}
