use ecu_calibration::{FuelRuntimeTune, KvError, KvStore};
use ecu_core::compat::EcuState;
use ecu_core::ts::pages::{EcuPageStore, PAGE_ANGLES, PAGE_FUEL, PAGE_IGN};
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore};
use ecu_ts::server::PageStore;

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

struct TestEcuStatePageStoreProvider {
    state: *mut EcuState,
}

impl TestEcuStatePageStoreProvider {
    fn new(state: &mut EcuState) -> Self {
        Self {
            state: state as *mut EcuState,
        }
    }
}

impl PageStoreProvider for TestEcuStatePageStoreProvider {
    type Pages<'a> = EcuPageStore<'a>;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R {
        // SAFETY: this test owns the EcuState and the provider only rebuilds a
        // page view for the duration of each persisted-store call.
        let state = unsafe { &mut *self.state };
        let mut pages = state.page_store();
        f(&mut pages)
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        // SAFETY: see `with_pages_mut`.
        let state = unsafe { &mut *self.state };
        let pages = state.page_store();
        f(&pages)
    }

    fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        // SAFETY: see `with_pages_mut`.
        let state = unsafe { &*self.state };
        FuelRuntimeTune::new(
            state.config.ve_table,
            state.config.afr_table,
            state.config.required_fuel_us,
            state.config.injector_deadtime_us,
            state.config.ve_load_source,
        )
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

    let mut store =
        PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), FailingKv);
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
    let mut store =
        PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), FailingKv);
    store.factory_reset();

    // Read back via pages
    let mut fuel = [0u8; 512];
    let mut ign = [0u8; 512];
    let mut angles = [0u8; 68];
    let n1 = store.read_page(PAGE_FUEL, &mut fuel).unwrap();
    let n2 = store.read_page(PAGE_IGN, &mut ign).unwrap();
    let n3 = store.read_page(PAGE_ANGLES, &mut angles).unwrap();
    assert_eq!(n1, 512);
    assert_eq!(n2, 512);
    assert_eq!(n3, 68);
    // Defaults for tables are serialized from state defaults in writer
    // Verify angles cam timeout default 500
    let cam_to = u16::from_le_bytes([angles[66], angles[67]]);
    assert_eq!(cam_to, 500);
}
