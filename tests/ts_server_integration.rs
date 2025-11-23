//! Integration test for TunerStudio server and pages

use ecu_core::persist::KvStore;
use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::pages::{EcuStatePageStore, PersistedPageStore, PAGE_FUEL, PAGE_IGN};
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::{OutpcProvider, TunerstudioServer};
use ecu_core::EcuState;
use ecu_target_common::kv::ram::RamKv512;

// Shared KV so we can reuse across server instances (simulate reboot)
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
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, ecu_core::persist::KvError> {
        self.inner.borrow_mut().read(key, out)
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), ecu_core::persist::KvError> {
        self.inner.borrow_mut().write(key, data)
    }
}

// Minimal provider reading from EcuState
#[derive(Copy, Clone)]
struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm;
        out.tps_percent = s.tps_percent;
        out.vbatt_mv = s.battery_voltage_mv;
        out.synced = if s.synced { 1 } else { 0 };
    }
}

#[test]
fn ts_sig_outpc_read_write_burn_roundtrip() {
    // Initial state
    let mut state = EcuState::new();
    state.rpm = 1500;
    state.tps_percent = 12;
    state.battery_voltage_mv = 12800;
    state.synced = true;

    // Shared KV and persisted page store
    let kv: SharedKv = SharedKv::new();
    let provider = Provider {
        state: &state as *const _,
    };

    // Use PersistedPageStore to allow Burn and later reload. Create server in a
    // short scope to release &mut borrows before we inspect/modify state again.
    let mut req = [0u8; 1024];
    let mut out = [0u8; 2048];
    {
        let store = PersistedPageStore::new(
            EcuStatePageStore {
                fuel: &mut state.ipw_table,
                ign: &mut state.ignition_table,
            },
            kv.clone(),
        );
        let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);

        // === SIG ===
        let len = proto::encode_reply(Cmd::Sig, &[], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("sig reply");
        assert!(rlen > 5);

        // === OUTPC ===
        let len = proto::encode_reply(Cmd::Outpc, &[], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("outpc reply");
        assert!(rlen >= 5 + core::mem::size_of::<Outpc>());

        // === WRITE PAGE (FUEL) ===
        // Build a 512‑byte buffer setting [0][0] = 0x1234, rest unchanged
        let mut page = [0u8; 1 + 512];
        page[0] = PAGE_FUEL;
        page[1] = 0x34; // little‑endian 0x1234
        page[2] = 0x12;
        let len = proto::encode_reply(Cmd::WritePage, &page, &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("write reply");
        assert!(rlen > 0);
    }

    // Now we can inspect state (borrow released)
    assert_eq!(state.ipw_table[0][0], 0x1234);

    // === READ PAGE (FUEL) in a new server instance ===
    {
        let store = PersistedPageStore::new(
            EcuStatePageStore {
                fuel: &mut state.ipw_table,
                ign: &mut state.ignition_table,
            },
            kv.clone(),
        );
        let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);
        let len = proto::encode_reply(Cmd::ReadPage, &[PAGE_FUEL], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("read reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("read decode");
        assert_eq!(cmd, Cmd::ReadPage);
        assert_eq!(payload.len(), 512);
        assert_eq!(payload[0], 0x34);
        assert_eq!(payload[1], 0x12);
    }

    // === BURN ===
    {
        let store = PersistedPageStore::new(
            EcuStatePageStore {
                fuel: &mut state.ipw_table,
                ign: &mut state.ignition_table,
            },
            kv.clone(),
        );
        let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);
        let len = proto::encode_reply(Cmd::Burn, &[], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("burn reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("burn decode");
        assert_eq!(cmd, Cmd::Burn);
        assert_eq!(payload, b"OK");
    }

    // Simulate reset: clear state tables then reload from KV via a fresh store
    state.ipw_table[0][0] = 0;
    let mut reload_store = PersistedPageStore::new(
        EcuStatePageStore {
            fuel: &mut state.ipw_table,
            ign: &mut state.ignition_table,
        },
        kv.clone(),
    );
    reload_store.try_load();
    assert_eq!(
        state.ipw_table[0][0], 0x1234,
        "fuel should reload from KV after burn"
    );

    // Also try WRITE of IGN page minimal smoke test: Set ignition [0][0] = -5
    {
        let store = PersistedPageStore::new(
            EcuStatePageStore {
                fuel: &mut state.ipw_table,
                ign: &mut state.ignition_table,
            },
            kv.clone(),
        );
        let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);
        let mut ign_buf = [0u8; 1 + 512];
        ign_buf[0] = PAGE_IGN;
        let v = (-5i16).to_le_bytes();
        ign_buf[1] = v[0];
        ign_buf[2] = v[1];
        let len = proto::encode_reply(Cmd::WritePage, &ign_buf, &mut req).unwrap();
        let _ = server.handle(&req[..len], &mut out).expect("write ign");
    }
    assert_eq!(state.ignition_table[0][0], -5);
}
