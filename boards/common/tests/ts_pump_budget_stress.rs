use ecu_calibration::FuelRuntimeTune;
use ecu_core::ts::pages::EcuPageStore;
use ecu_core::EcuState;
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::ts::service::TsService;
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore};
use ecu_ts::proto::{encode_reply, Cmd};

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
        // SAFETY: this test owns the EcuState and rebuilds a page view only for
        // the duration of each persisted-store call.
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

struct FakePort {
    inq: std::collections::VecDeque<u8>,
    out_count: usize,
    bytes_per_write: usize,
}

impl FakePort {
    fn new() -> Self {
        Self {
            inq: std::collections::VecDeque::new(),
            out_count: 0,
            bytes_per_write: 0,
        }
    }
    fn push_frame(&mut self, frame: &[u8]) {
        for b in frame {
            self.inq.push_back(*b);
        }
    }
}

impl ecu_ts::serial::SerialPort for FakePort {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let n = core::cmp::min(buf.len(), self.inq.len());
        if n == 0 {
            return 0;
        }
        for slot in buf.iter_mut().take(n) {
            *slot = self.inq.pop_front().unwrap();
        }
        n
    }
    fn write(&mut self, _buf: &[u8]) -> usize {
        self.out_count += 1;
        self.bytes_per_write
    }
}

#[test]
fn ts_pump_enforces_budget() {
    // Build service
    let mut state = EcuState::new();
    let provider = crate::ts_provider_for(&state);
    let page_provider = TestEcuStatePageStoreProvider::new(&mut state);
    let mut store = PersistedTsPageStore::new(page_provider, RamKv512::new());
    store.try_load();
    let mut svc = TsService::new(ecu_ts::TS_SIGNATURE, provider, store);

    // Prepare input: multiple valid frames
    let mut req = [0u8; 64];
    let len = encode_reply(Cmd::Ping, &[], &mut req).unwrap();
    let mut port = FakePort::new();
    for _ in 0..5 {
        port.push_frame(&req[..len]);
    }

    // Pump with budget 2 -> should process only 2 frames
    svc.pump_with_budget(&mut port, 2);
    assert_eq!(port.out_count, 2);

    // Next pump with budget 2 -> cumulative 4
    svc.pump_with_budget(&mut port, 2);
    assert_eq!(port.out_count, 4);

    // Finish the rest
    svc.pump_with_budget(&mut port, 4);
    assert_eq!(port.out_count, 5);
}

#[test]
fn ts_pump_records_serial_write_failures() {
    let mut state = EcuState::new();
    let provider = crate::ts_provider_for(&state);
    let page_provider = TestEcuStatePageStoreProvider::new(&mut state);
    let mut store = PersistedTsPageStore::new(page_provider, RamKv512::new());
    store.try_load();
    let mut svc = TsService::new(ecu_ts::TS_SIGNATURE, provider, store);

    let mut req = [0u8; 64];
    let len = encode_reply(Cmd::Ping, &[], &mut req).unwrap();
    let mut port = FakePort::new();
    port.push_frame(&req[..len]);

    svc.pump_with_budget(&mut port, 1);

    let stats = svc.stats();
    assert_eq!(stats.serial_write_failures, 1);
    assert!(stats.last_serial_write_expected > 0);
    assert_eq!(stats.last_serial_write_actual, 0);

    svc.reset_stats();
    assert_eq!(svc.stats().serial_write_failures, 0);
}

// Minimal Outpc provider for tests
mod test_provider {
    use ecu_core::EcuState;
    use ecu_ts::outpc::Outpc;
    use ecu_ts::server::OutpcProvider;

    pub struct Provider {
        pub state: *const EcuState,
    }
    impl OutpcProvider for Provider {
        fn fill_outpc(&self, out: &mut Outpc) {
            let s = unsafe { &*self.state };
            out.rpm = s.rpm();
        }
    }
}

fn ts_provider_for(state: &EcuState) -> test_provider::Provider {
    test_provider::Provider {
        state: state as *const _,
    }
}
