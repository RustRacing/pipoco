use ecu_calibration::FuelRuntimeTune;
use ecu_core::compat::EcuState;
use ecu_core::constants::{fuel as fuel_consts, ignition as ign_consts};
use ecu_core::ts::pages::EcuPageStore;
use ecu_target_common::kv::ram::RamKv512;
use ecu_ts::pages::{PAGE_FUEL, PAGE_IGN, TABLE_PAGE_BYTES};
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore};
use ecu_ts::server::{PageError, PageStore, PersistError};

struct FactoryResetSafePageStore<'a> {
    inner: EcuPageStore<'a>,
}

impl PageStore for FactoryResetSafePageStore<'_> {
    fn page_len(&self, page: u8) -> Option<usize> {
        self.inner.page_len(page)
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        self.inner.read_page(page, out)
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        if data.len() == TABLE_PAGE_BYTES && data.iter().all(|byte| *byte == 0) {
            match page {
                PAGE_FUEL => return self.inner.write_page(page, &default_fuel_page()),
                PAGE_IGN => return self.inner.write_page(page, &default_ignition_page()),
                _ => {}
            }
        }
        self.inner.write_page(page, data)
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        self.inner.burn()
    }
}

fn default_fuel_page() -> [u8; TABLE_PAGE_BYTES] {
    let mut page = [0u8; TABLE_PAGE_BYTES];
    for cell in page.chunks_exact_mut(2) {
        cell.copy_from_slice(&fuel_consts::DEFAULT_PULSE_WIDTH_US.to_le_bytes());
    }
    page
}

fn default_ignition_page() -> [u8; TABLE_PAGE_BYTES] {
    let mut page = [0u8; TABLE_PAGE_BYTES];
    for cell in page.chunks_exact_mut(2) {
        cell.copy_from_slice(&ign_consts::DEFAULT_TIMING_BTDC.to_le_bytes());
    }
    page
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
    type Pages<'a> = FactoryResetSafePageStore<'a>;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R {
        // SAFETY: this test owns the EcuState and rebuilds a page view only for
        // the duration of each persisted-store call.
        let state = unsafe { &mut *self.state };
        let mut pages = FactoryResetSafePageStore {
            inner: state.page_store(),
        };
        f(&mut pages)
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        // SAFETY: see `with_pages_mut`.
        let state = unsafe { &mut *self.state };
        let pages = FactoryResetSafePageStore {
            inner: state.page_store(),
        };
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
fn factory_reset_restores_defaults() {
    let mut state = EcuState::new();

    // Mutate a couple of cells to non-defaults
    state.config.ipw_table[0][0] = 1234;
    state.config.ignition_table[15][15] = -7;

    // Empty KV
    let kv = RamKv512::new();

    // 1) try_load should not overwrite when KV empty
    {
        let provider = TestEcuStatePageStoreProvider::new(&mut state);
        let mut store = PersistedTsPageStore::new(provider, kv);
        store.try_load();
        // drop before reading state
    }
    assert_eq!(state.config.ipw_table[0][0], 1234);
    assert_eq!(state.config.ignition_table[15][15], -7);

    // 2) factory_reset sets defaults
    {
        let provider = TestEcuStatePageStoreProvider::new(&mut state);
        let mut store = PersistedTsPageStore::new(provider, RamKv512::new());
        store.factory_reset();
    }
    assert_eq!(
        state.config.ipw_table[0][0],
        fuel_consts::DEFAULT_PULSE_WIDTH_US
    );
    assert_eq!(
        state.config.ipw_table[7][8],
        fuel_consts::DEFAULT_PULSE_WIDTH_US
    );
    assert_eq!(
        state.config.ignition_table[0][0],
        ign_consts::DEFAULT_TIMING_BTDC
    );
    assert_eq!(
        state.config.ignition_table[15][15],
        ign_consts::DEFAULT_TIMING_BTDC
    );
}
