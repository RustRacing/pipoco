use ecu_core::ts::pages::{EcuStatePageStore, PersistedPageStore};
use ecu_core::ts::proto::{encode_reply, Cmd};
use ecu_core::EcuState;
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::ts::service::TsService;

struct FakePort {
    inq: std::collections::VecDeque<u8>,
    out_count: usize,
}

impl FakePort {
    fn new() -> Self {
        Self {
            inq: std::collections::VecDeque::new(),
            out_count: 0,
        }
    }
    fn push_frame(&mut self, frame: &[u8]) {
        for b in frame {
            self.inq.push_back(*b);
        }
    }
}

impl ecu_core::ts::serial::SerialPort for FakePort {
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
        0
    }
}

#[test]
fn ts_pump_enforces_budget() {
    // Build service
    let mut state = EcuState::new();
    let provider = crate::ts_provider_for(&state);
    let pages = EcuStatePageStore {
        fuel: &mut state.ipw_table,
        ign: &mut state.ignition_table,
    };
    let mut store = PersistedPageStore::new(pages, RamKv512::new());
    store.try_load();
    let mut svc = TsService::new(b"IPW-ECU V0.1", provider, store);

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

// Minimal Outpc provider for tests
mod test_provider {
    use ecu_core::ts::outpc::Outpc;
    use ecu_core::ts::OutpcProvider;
    use ecu_core::EcuState;
    pub struct Provider {
        pub state: *const EcuState,
    }
    impl OutpcProvider for Provider {
        fn fill_outpc(&self, out: &mut Outpc) {
            let s = unsafe { &*self.state };
            out.rpm = s.rpm;
        }
    }
}

fn ts_provider_for(state: &EcuState) -> test_provider::Provider {
    test_provider::Provider {
        state: state as *const _,
    }
}
