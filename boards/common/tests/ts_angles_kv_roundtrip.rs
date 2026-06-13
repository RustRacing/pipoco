use ecu_calibration::FuelRuntimeTune;
use ecu_calibration::{KvError, KvStore};
use ecu_core::compat::EcuState;
use ecu_core::ts::pages::{EcuPageStore, PAGE_ANGLES};
use ecu_target_common::kv::ram::RamKv512;
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore};
use ecu_ts::server::PageStore;

// Shared KV wrapper so we can reuse the same backing buffer across two stores
struct SharedKv {
    inner: std::rc::Rc<std::cell::RefCell<RamKv512>>,
}
impl SharedKv {
    fn new() -> Self {
        Self {
            inner: std::rc::Rc::new(std::cell::RefCell::new(RamKv512::new())),
        }
    }
}
impl Clone for SharedKv {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}
impl KvStore for SharedKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        self.inner.borrow_mut().read(key, out)
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        self.inner.borrow_mut().write(key, data)
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
fn angles_kv_roundtrip() {
    let mut state = EcuState::new();
    // Set non-default angle values
    state.config.inj_angle_btdc_x10[0] = 150; // 15.0°
    state.config.inj_angle_btdc_x10[1] = 220; // 22.0°
    state.config.tdc_per_cyl_x10[0] = 100; // 10.0°
    state.config.tdc_per_cyl_x10[1] = 280; // 28.0°
    state.config.tooth0_angle_x10 = 35; // 3.5°
    state.config.cam_missing_timeout_ms = 750;

    // Encode angles page and write to KV directly (avoid 512-byte table writes)
    let kv = SharedKv::new();
    {
        let mut buf = [0u8; 68];
        // Build a transient page store to read angles into buf
        let pages = state.page_store();
        let n = pages.read_page(PAGE_ANGLES, &mut buf).expect("angles read");
        assert_eq!(n, 68);
        let mut kvw = kv.clone();
        kvw.write(b"angles", &buf).expect("kv write angles");
    }

    // Reset angles to zeros
    state.config.inj_angle_btdc_x10 = [0; 16];
    state.config.tdc_per_cyl_x10 = [0; 16];
    state.config.tooth0_angle_x10 = 0;
    state.config.cam_missing_timeout_ms = 0;

    // Load back from KV via the generic persisted TS store.
    let mut store2 =
        PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
    store2.try_load();

    assert_eq!(state.config.inj_angle_btdc_x10[0], 150);
    assert_eq!(state.config.inj_angle_btdc_x10[1], 220);
    assert_eq!(state.config.tdc_per_cyl_x10[0], 100);
    assert_eq!(state.config.tdc_per_cyl_x10[1], 280);
    assert_eq!(state.config.tooth0_angle_x10, 35);
    assert_eq!(state.config.cam_missing_timeout_ms, 750);
}
