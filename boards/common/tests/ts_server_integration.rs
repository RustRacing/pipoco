//! Integration test for TunerStudio server and pages

use ecu_calibration::{FuelRuntimeTune, KvError, KvStore};
use ecu_core::ts::pages::{
    EcuPageStore, PAGE_AFR_TABLE, PAGE_FUEL, PAGE_IGN, PAGE_VE_TABLE, PAGE_VE_TUNE,
};
use ecu_core::EcuState;
use ecu_target_common::kv::ram::RamKv512;
use ecu_ts::outpc::Outpc;
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore};
use ecu_ts::proto::{self, Cmd};
use ecu_ts::server::{OutpcProvider, TunerstudioServer};

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
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        self.inner.borrow_mut().read(key, out)
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        self.inner.borrow_mut().write(key, data)
    }
}

// Provider that rebuilds the EcuState page projection for the duration of each
// persisted-store call, keeping the projection logic in core via
// `state.page_store()`.
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

// Minimal provider reading from EcuState
#[derive(Copy, Clone)]
struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm();
        out.tps_percent = s.tps_percent();
        out.vbatt_mv = s.battery_voltage_mv();
        out.synced = if s.synced() { 1 } else { 0 };
    }
}

#[test]
fn ts_sig_outpc_read_write_burn_roundtrip() {
    // Initial state
    let mut state = EcuState::new();
    state.set_rpm(1500);
    state.set_tps_percent(12);
    state.set_battery_voltage_mv(12800);
    state.set_synced(true);

    // Shared KV and persisted page store
    let kv: SharedKv = SharedKv::new();
    let provider = Provider {
        state: &state as *const _,
    };

    // Use PersistedTsPageStore to allow Burn and later reload. Create server in a
    // short scope to release &mut borrows before we inspect/modify state again.
    let mut req = [0u8; 1024];
    let mut out = [0u8; 2048];
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);

        // === SIG ===
        let len = proto::encode_reply(Cmd::Sig, &[], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("sig reply");
        assert!(rlen > 5);

        // === OUTPC ===
        let len = proto::encode_reply(Cmd::Outpc, &[], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("outpc reply");
        assert!(rlen >= 5 + core::mem::size_of::<Outpc>());

        // === WRITE PAGE (FUEL) ===
        // Build a 512-byte buffer setting [0][0] = 0x1234, rest unchanged
        let mut page = [0u8; 1 + 512];
        page[0] = PAGE_FUEL;
        page[1] = 0x34; // little-endian 0x1234
        page[2] = 0x12;
        let len = proto::encode_reply(Cmd::WritePage, &page, &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("write reply");
        assert!(rlen > 0);
    }

    // Now we can inspect state (borrow released)
    assert_eq!(state.config.ipw_table[0][0], 0x1234);

    // === READ PAGE (FUEL) in a new server instance ===
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
        let len = proto::encode_reply(Cmd::ReadPage, &[PAGE_FUEL], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("read reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("read decode");
        assert_eq!(cmd, Cmd::ReadPage);
        assert_eq!(payload.len(), 512);
        assert_eq!(payload[0], 0x34);
        assert_eq!(payload[1], 0x12);
    }

    // === OFFSET READ/WRITE PAGE (spec-shaped payload) ===
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);

        let mut patch = [0u8; 7];
        patch[0] = PAGE_FUEL;
        patch[1..3].copy_from_slice(&2u16.to_le_bytes());
        patch[3..5].copy_from_slice(&2u16.to_le_bytes());
        patch[5..7].copy_from_slice(&0x5678u16.to_le_bytes());
        let len = proto::encode_reply(Cmd::WritePage, &patch, &mut req).unwrap();
        let rlen = server
            .handle(&req[..len], &mut out)
            .expect("offset write reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("offset write decode");
        assert_eq!(cmd, Cmd::WritePage);
        assert_eq!(payload, b"OK");

        let mut read = [0u8; 5];
        read[0] = PAGE_FUEL;
        read[1..3].copy_from_slice(&2u16.to_le_bytes());
        read[3..5].copy_from_slice(&2u16.to_le_bytes());
        let len = proto::encode_reply(Cmd::ReadPage, &read, &mut req).unwrap();
        let rlen = server
            .handle(&req[..len], &mut out)
            .expect("offset read reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("offset read decode");
        assert_eq!(cmd, Cmd::ReadPage);
        assert_eq!(payload, &0x5678u16.to_le_bytes());
    }
    assert_eq!(state.config.ipw_table[0][0], 0x1234);
    assert_eq!(state.config.ipw_table[0][1], 0x5678);

    // === BURN ===
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
        let len = proto::encode_reply(Cmd::Burn, &[], &mut req).unwrap();
        let rlen = server.handle(&req[..len], &mut out).expect("burn reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("burn decode");
        assert_eq!(cmd, Cmd::Burn);
        assert_eq!(payload, b"OK");
    }

    // Simulate reset: clear state tables then reload from KV via a fresh store
    state.config.ipw_table[0][0] = 0;
    let mut reload_store =
        PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
    reload_store.try_load();
    assert_eq!(
        state.config.ipw_table[0][0], 0x1234,
        "fuel should reload from KV after burn"
    );

    // Also try WRITE of IGN page minimal smoke test: Set ignition [0][0] = -5
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
        let mut ign_buf = [0u8; 1 + 512];
        ign_buf[0] = PAGE_IGN;
        let v = (-5i16).to_le_bytes();
        ign_buf[1] = v[0];
        ign_buf[2] = v[1];
        let len = proto::encode_reply(Cmd::WritePage, &ign_buf, &mut req).unwrap();
        let _ = server.handle(&req[..len], &mut out).expect("write ign");
    }
    assert_eq!(state.config.ignition_table[0][0], -5);

    // VE tune page smoke via full page store: update CL target/gains + required anchor and roundtrip
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
        let mut ve_buf = [0u8; 1 + 16];
        ve_buf[0] = PAGE_VE_TUNE;
        ve_buf[1..3].copy_from_slice(&150u16.to_le_bytes()); // target afr x10
        ve_buf[3..5].copy_from_slice(&11u16.to_le_bytes()); // kp
        ve_buf[5..7].copy_from_slice(&7u16.to_le_bytes()); // ki
        ve_buf[7..9].copy_from_slice(&2400u16.to_le_bytes()); // required fuel us
        ve_buf[9..11].copy_from_slice(&900u16.to_le_bytes()); // deadtime us
        ve_buf[11] = 1; // load source TPS
        ve_buf[13..15].copy_from_slice(&500u16.to_le_bytes());
        ve_buf[15..17].copy_from_slice(&20_000u16.to_le_bytes());
        let len = proto::encode_reply(Cmd::WritePage, &ve_buf, &mut req).unwrap();
        let _ = server.handle(&req[..len], &mut out).expect("write ve_tune");

        let len = proto::encode_reply(Cmd::ReadPage, &[PAGE_VE_TUNE], &mut req).unwrap();
        let rlen = server
            .handle(&req[..len], &mut out)
            .expect("read ve_tune reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("ve_tune decode");
        assert_eq!(cmd, Cmd::ReadPage);
        assert_eq!(payload.len(), 16);
        assert_eq!(u16::from_le_bytes([payload[0], payload[1]]), 150);
        assert_eq!(u16::from_le_bytes([payload[2], payload[3]]), 11);
        assert_eq!(u16::from_le_bytes([payload[4], payload[5]]), 7);
        assert_eq!(u16::from_le_bytes([payload[6], payload[7]]), 2400);
        assert_eq!(u16::from_le_bytes([payload[8], payload[9]]), 900);
        assert_eq!(payload[10], 1);
    }

    // VE table page smoke (16x16) via full page store
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
        let mut ve_tbl = [0u8; 1 + 512];
        ve_tbl[0] = PAGE_VE_TABLE;
        ve_tbl[1..3].copy_from_slice(&1800u16.to_le_bytes());
        ve_tbl[3..5].copy_from_slice(&1900u16.to_le_bytes());
        let len = proto::encode_reply(Cmd::WritePage, &ve_tbl, &mut req).unwrap();
        let _ = server
            .handle(&req[..len], &mut out)
            .expect("write ve_table");

        let len = proto::encode_reply(Cmd::ReadPage, &[PAGE_VE_TABLE], &mut req).unwrap();
        let rlen = server
            .handle(&req[..len], &mut out)
            .expect("read ve_table reply");
        let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("ve_table decode");
        assert_eq!(cmd, Cmd::ReadPage);
        assert_eq!(payload.len(), 512);
        assert_eq!(u16::from_le_bytes([payload[0], payload[1]]), 1800);
        assert_eq!(u16::from_le_bytes([payload[2], payload[3]]), 1900);
    }

    // AFR table page smoke: full surface stores per-cell and first cell updates CL target anchor
    {
        let store =
            PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
        let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
        let mut afr_tbl = [0u8; 1 + 512];
        afr_tbl[0] = PAGE_AFR_TABLE;
        for chunk in afr_tbl[1..].chunks_exact_mut(2) {
            chunk.copy_from_slice(&155u16.to_le_bytes());
        }
        afr_tbl[3..5].copy_from_slice(&166u16.to_le_bytes());
        let len = proto::encode_reply(Cmd::WritePage, &afr_tbl, &mut req).unwrap();
        let _ = server
            .handle(&req[..len], &mut out)
            .expect("write afr_table");

        let len = proto::encode_reply(Cmd::ReadPage, &[PAGE_AFR_TABLE], &mut req).unwrap();
        let rlen = server
            .handle(&req[..len], &mut out)
            .expect("read afr_table reply");
        let (_, payload) = proto::decode_request(&out[..rlen]).expect("afr_table decode");
        assert_eq!(u16::from_le_bytes([payload[0], payload[1]]), 155);
        assert_eq!(u16::from_le_bytes([payload[2], payload[3]]), 166);
    }

    assert_eq!(state.config.cl_config.target_afr_x10, 155);
}
