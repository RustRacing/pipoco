//! Integration test for TunerStudio server and pages

use ecu_calibration::{
    CalibrationHardwareTargetId, CalibrationRuntimeBuildId, ExpertTriggerCalibration,
    FuelRuntimeTune, KvError, KvStore,
};
use ecu_compat::compat::EcuState;
use ecu_compat::ts::pages::{
    EcuPageStore, PAGE_AFR_TABLE, PAGE_FUEL, PAGE_IGN, PAGE_VE_TABLE, PAGE_VE_TUNE,
};
use ecu_compat::ts::CompatCalibrationSession;
use ecu_target_common::kv::ram::RamKv512;
use ecu_ts::outpc::Outpc;
use ecu_ts::pages::TS_PAGE_COUNT;
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore, TsPackageCommandStore};
use ecu_ts::proto::{self, Cmd};
use ecu_ts::server::{
    encode_compatibility_info_reply, encode_tooth_composite_log_reply, encode_version_info_reply,
    BenchToolingOwner, CompatibilityInfoReport, CompatibilityMigrationCode,
    CompatibilityStatusCode, NoPages, OutpcProvider, ToothCompositeLogEntry, ToothCompositeLogKind,
    TunerstudioServer, PAGE_CRC_INFO_REPLY_VERSION,
};
use ecu_ts::{decode_expert_trigger_page, PAGE_EXPERT_TRIGGER};

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

#[derive(Copy, Clone)]
struct BenchProvider;

impl OutpcProvider for BenchProvider {
    fn fill_outpc(&self, _out: &mut Outpc) {}
}

#[derive(Default)]
struct BenchOwner {
    last_output_test: Option<(u8, u32, u32, u8)>,
    reboot_requested: bool,
}

impl BenchToolingOwner for BenchOwner {
    fn handle_output_test(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if payload.len() < 6 {
            return None;
        }
        let chan = payload[0];
        let on_ms = u16::from_le_bytes([payload[1], payload[2]]) as u32;
        let off_ms = u16::from_le_bytes([payload[3], payload[4]]) as u32;
        let reps = payload[5];
        self.last_output_test = Some((chan, on_ms, off_ms, reps));
        proto::encode_reply(Cmd::OutputTest, b"OK", out)
    }

    fn handle_tooth_stats(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        proto::encode_reply(Cmd::ToothStats, &[0xb2, 0x0c, 1], out)
    }

    fn handle_tooth_composite_log(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        let entries = [
            ToothCompositeLogEntry {
                kind: ToothCompositeLogKind::TriggerEdge,
                flags: 0x03,
                rpm: 3_210,
                angle_x10: 450,
                at_us: 123,
            },
            ToothCompositeLogEntry {
                kind: ToothCompositeLogKind::CamEdge,
                flags: 0x03,
                rpm: 3_210,
                angle_x10: 480,
                at_us: 130,
            },
        ];
        encode_tooth_composite_log_reply(&entries, 2, out)
    }

    fn handle_version_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        encode_version_info_reply(b"bench-ts", 0x1234_5678, 0x2040, out)
    }

    fn handle_compatibility_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        encode_compatibility_info_reply(
            CompatibilityInfoReport {
                status: CompatibilityStatusCode::Compatible,
                migration: CompatibilityMigrationCode::None,
                expected_schema_version: 7,
                actual_schema_version: 7,
                expected_runtime_build_id: 0x1234_5678,
                actual_runtime_build_id: 0x1234_5678,
                expected_hardware_target_id: 0x2040,
                actual_hardware_target_id: 0x2040,
            },
            out,
        )
    }

    fn handle_reboot(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        self.reboot_requested = true;
        proto::encode_reply(Cmd::Reboot, b"OK", out)
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

#[test]
fn ts_package_commands_roundtrip_through_compat_session_owner() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let store = PersistedTsPageStore::new(
        TestEcuStatePageStoreProvider::new(&mut state),
        SharedKv::new(),
    );
    let session = CompatCalibrationSession::from_persisted_store(&store);
    let command_store =
        TsPackageCommandStore::new(store, session, runtime_build_id, hardware_target_id);
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, command_store);
    let mut req = [0u8; 1024];
    let mut out = [0u8; 2048];

    let export_len = proto::encode_reply(Cmd::PackageExport, &[], &mut req).unwrap();
    let export_reply = server
        .handle(&req[..export_len], &mut out)
        .expect("package export reply");
    let (cmd, payload) =
        proto::decode_request(&out[..export_reply]).expect("decode package export");
    assert_eq!(cmd, Cmd::PackageExport);
    assert_eq!(
        payload.len(),
        ecu_calibration::PersistedCalibrationPackage::WIRE_LEN
    );

    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_2001,
        fixed_timing_deg10: 175,
        ..ExpertTriggerCalibration::default()
    };
    let mut candidate_wire = [0u8; ecu_calibration::PersistedCalibrationPackage::WIRE_LEN];
    CompatCalibrationSession::new(candidate_trigger)
        .surface()
        .export_current_package(runtime_build_id, hardware_target_id)
        .encode_wire(&mut candidate_wire)
        .expect("encode candidate wire");

    let import_len = proto::encode_reply(Cmd::PackageImport, &candidate_wire, &mut req).unwrap();
    let import_reply = server
        .handle(&req[..import_len], &mut out)
        .expect("package import reply");
    let (cmd, payload) =
        proto::decode_request(&out[..import_reply]).expect("decode package import");
    assert_eq!(cmd, Cmd::PackageImport);
    assert_eq!(payload, b"OK");

    let read_len = proto::encode_reply(Cmd::ReadPage, &[PAGE_EXPERT_TRIGGER], &mut req).unwrap();
    let read_reply = server
        .handle(&req[..read_len], &mut out)
        .expect("expert trigger read reply");
    let (cmd, payload) =
        proto::decode_request(&out[..read_reply]).expect("decode expert trigger read");
    assert_eq!(cmd, Cmd::ReadPage);
    assert_eq!(
        decode_expert_trigger_page(payload).expect("decode expert trigger page"),
        candidate_trigger
    );
    assert_eq!(
        server
            .store()
            .session()
            .surface()
            .expert_trigger_calibration(),
        candidate_trigger
    );
}

#[test]
fn ts_package_import_burn_roundtrip_reloads_candidate_after_reboot() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let kv: SharedKv = SharedKv::new();
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let store =
        PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv.clone());
    let session = CompatCalibrationSession::from_persisted_store(&store);
    let command_store =
        TsPackageCommandStore::new(store, session, runtime_build_id, hardware_target_id);
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, command_store);
    let mut req = [0u8; 1024];
    let mut out = [0u8; 2048];

    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_2002,
        fixed_timing_deg10: 225,
        ..ExpertTriggerCalibration::default()
    };
    let mut candidate_wire = [0u8; ecu_calibration::PersistedCalibrationPackage::WIRE_LEN];
    CompatCalibrationSession::new(candidate_trigger)
        .surface()
        .export_current_package(runtime_build_id, hardware_target_id)
        .encode_wire(&mut candidate_wire)
        .expect("encode candidate wire");

    let import_len = proto::encode_reply(Cmd::PackageImport, &candidate_wire, &mut req).unwrap();
    let import_reply = server
        .handle(&req[..import_len], &mut out)
        .expect("package import reply");
    let (cmd, payload) =
        proto::decode_request(&out[..import_reply]).expect("decode package import");
    assert_eq!(cmd, Cmd::PackageImport);
    assert_eq!(payload, b"OK");

    let burn_len = proto::encode_reply(Cmd::Burn, &[], &mut req).unwrap();
    let burn_reply = server
        .handle(&req[..burn_len], &mut out)
        .expect("package burn reply");
    let (cmd, payload) = proto::decode_request(&out[..burn_reply]).expect("decode package burn");
    assert_eq!(cmd, Cmd::Burn);
    assert_eq!(payload, b"OK");

    let mut rebooted_state = EcuState::new();
    let mut rebooted_store = PersistedTsPageStore::new(
        TestEcuStatePageStoreProvider::new(&mut rebooted_state),
        kv.clone(),
    );
    rebooted_store.try_load();
    let mut rebooted_session = CompatCalibrationSession::from_persisted_store(&rebooted_store);
    let applied = rebooted_store
        .try_load_and_import_package(&mut rebooted_session, runtime_build_id, hardware_target_id)
        .expect("persisted package should reload");

    assert!(matches!(
        applied,
        ecu_ts::CalibrationPackageApplyResult::AppliedUnchanged { .. }
    ));
    assert_eq!(
        rebooted_store.expert_trigger_calibration(),
        candidate_trigger
    );
    assert_eq!(
        rebooted_session.surface().expert_trigger_calibration(),
        candidate_trigger
    );
}

#[test]
fn ts_page_crc_info_reports_live_page_surface() {
    let mut state = EcuState::new();
    let provider = Provider {
        state: &state as *const _,
    };
    let kv: SharedKv = SharedKv::new();
    let store = PersistedTsPageStore::new(TestEcuStatePageStoreProvider::new(&mut state), kv);
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, provider, store);
    let mut req = [0u8; 64];
    let mut out = [0u8; 256];

    let len = proto::encode_reply(Cmd::PageCrcInfo, &[], &mut req).unwrap();
    let rlen = server
        .handle(&req[..len], &mut out)
        .expect("page-crc reply");
    let (cmd, payload) = proto::decode_request(&out[..rlen]).expect("decode page-crc reply");
    assert_eq!(cmd, Cmd::PageCrcInfo);
    assert_eq!(payload[0], PAGE_CRC_INFO_REPLY_VERSION);
    assert_eq!(payload[1] as usize, TS_PAGE_COUNT);
    assert_eq!(payload[2], PAGE_FUEL);
    assert_eq!(u16::from_le_bytes([payload[3], payload[4]]), 512);
    assert_eq!(u16::from_le_bytes([payload[5], payload[6]]), 1);
    assert_ne!(u16::from_le_bytes([payload[7], payload[8]]), 0);
    assert_eq!(payload[9] & 0x03, 0x03);
}

#[test]
fn ts_bench_commands_reply_err_by_default_without_owner() {
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, BenchProvider, NoPages);
    let mut req = [0u8; 64];
    let mut out = [0u8; 128];

    let output_len = proto::encode_reply(Cmd::OutputTest, &[0, 0, 0, 0, 0, 0], &mut req).unwrap();
    let output_reply = server
        .handle(&req[..output_len], &mut out)
        .expect("default output-test reply");
    let (cmd, payload) =
        proto::decode_request(&out[..output_reply]).expect("decode default output-test reply");
    assert_eq!(cmd, Cmd::OutputTest);
    assert_eq!(payload, b"ERR");

    let tooth_len = proto::encode_reply(Cmd::ToothStats, &[], &mut req).unwrap();
    let tooth_reply = server
        .handle(&req[..tooth_len], &mut out)
        .expect("default tooth-stats reply");
    let (cmd, payload) =
        proto::decode_request(&out[..tooth_reply]).expect("decode default tooth-stats reply");
    assert_eq!(cmd, Cmd::ToothStats);
    assert_eq!(payload, b"ERR");

    let log_len = proto::encode_reply(Cmd::ToothCompositeLog, &[], &mut req).unwrap();
    let log_reply = server
        .handle(&req[..log_len], &mut out)
        .expect("default tooth-composite-log reply");
    let (cmd, payload) =
        proto::decode_request(&out[..log_reply]).expect("decode default tooth-composite-log reply");
    assert_eq!(cmd, Cmd::ToothCompositeLog);
    assert_eq!(payload, b"ERR");

    let version_len = proto::encode_reply(Cmd::VersionInfo, &[], &mut req).unwrap();
    let version_reply = server
        .handle(&req[..version_len], &mut out)
        .expect("default version-info reply");
    let (cmd, payload) =
        proto::decode_request(&out[..version_reply]).expect("decode default version-info reply");
    assert_eq!(cmd, Cmd::VersionInfo);
    assert_eq!(payload, b"ERR");

    let compatibility_len = proto::encode_reply(Cmd::CompatibilityInfo, &[], &mut req).unwrap();
    let compatibility_reply = server
        .handle(&req[..compatibility_len], &mut out)
        .expect("default compatibility-info reply");
    let (cmd, payload) = proto::decode_request(&out[..compatibility_reply])
        .expect("decode default compatibility-info reply");
    assert_eq!(cmd, Cmd::CompatibilityInfo);
    assert_eq!(payload, b"ERR");

    let reboot_len = proto::encode_reply(Cmd::Reboot, &[], &mut req).unwrap();
    let reboot_reply = server
        .handle(&req[..reboot_len], &mut out)
        .expect("default reboot reply");
    let (cmd, payload) =
        proto::decode_request(&out[..reboot_reply]).expect("decode default reboot reply");
    assert_eq!(cmd, Cmd::Reboot);
    assert_eq!(payload, b"ERR");
}

#[test]
fn ts_bench_command_owner_handles_bench_tooling_commands() {
    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, BenchProvider, NoPages);
    let mut owner = BenchOwner::default();
    let mut req = [0u8; 64];
    let mut out = [0u8; 128];

    let output_len =
        proto::encode_reply(Cmd::OutputTest, &[2, 0x32, 0x00, 0x14, 0x00, 3], &mut req).unwrap();
    let output_reply = server
        .handle_with_bench_tooling(&req[..output_len], &mut out, &mut owner)
        .expect("owned output-test reply");
    let (cmd, payload) =
        proto::decode_request(&out[..output_reply]).expect("decode owned output-test reply");
    assert_eq!(cmd, Cmd::OutputTest);
    assert_eq!(payload, b"OK");
    assert_eq!(owner.last_output_test, Some((2, 50, 20, 3)));

    let tooth_len = proto::encode_reply(Cmd::ToothStats, &[], &mut req).unwrap();
    let tooth_reply = server
        .handle_with_bench_tooling(&req[..tooth_len], &mut out, &mut owner)
        .expect("owned tooth-stats reply");
    let (cmd, payload) =
        proto::decode_request(&out[..tooth_reply]).expect("decode owned tooth-stats reply");
    assert_eq!(cmd, Cmd::ToothStats);
    assert_eq!(payload, &[0xb2, 0x0c, 1]);

    let log_len = proto::encode_reply(Cmd::ToothCompositeLog, &[], &mut req).unwrap();
    let log_reply = server
        .handle_with_bench_tooling(&req[..log_len], &mut out, &mut owner)
        .expect("owned tooth-composite-log reply");
    let (cmd, payload) =
        proto::decode_request(&out[..log_reply]).expect("decode owned tooth-composite-log reply");
    assert_eq!(cmd, Cmd::ToothCompositeLog);
    assert_eq!(
        payload[0],
        ecu_ts::server::TOOTH_COMPOSITE_LOG_REPLY_VERSION
    );
    assert_eq!(payload[1], 2);
    assert_eq!(u16::from_le_bytes([payload[2], payload[3]]), 2);
    assert_eq!(payload[4], ToothCompositeLogKind::TriggerEdge as u8);
    assert_eq!(payload[5], 0x03);
    assert_eq!(u16::from_le_bytes([payload[6], payload[7]]), 3_210);
    assert_eq!(i16::from_le_bytes([payload[8], payload[9]]), 450);
    assert_eq!(
        u32::from_le_bytes([payload[10], payload[11], payload[12], payload[13]]),
        123
    );
    assert_eq!(payload[14], ToothCompositeLogKind::CamEdge as u8);
    assert_eq!(payload[15], 0x03);
    assert_eq!(u16::from_le_bytes([payload[16], payload[17]]), 3_210);
    assert_eq!(i16::from_le_bytes([payload[18], payload[19]]), 480);
    assert_eq!(
        u32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]),
        130
    );

    let version_len = proto::encode_reply(Cmd::VersionInfo, &[], &mut req).unwrap();
    let version_reply = server
        .handle_with_bench_tooling(&req[..version_len], &mut out, &mut owner)
        .expect("owned version-info reply");
    let mut expected = [0u8; 128];
    let expected_len = encode_version_info_reply(b"bench-ts", 0x1234_5678, 0x2040, &mut expected)
        .expect("encode expected version-info reply");
    assert_eq!(&out[..version_reply], &expected[..expected_len]);

    let compatibility_len = proto::encode_reply(Cmd::CompatibilityInfo, &[], &mut req).unwrap();
    let compatibility_reply = server
        .handle_with_bench_tooling(&req[..compatibility_len], &mut out, &mut owner)
        .expect("owned compatibility-info reply");
    let expected_len = encode_compatibility_info_reply(
        CompatibilityInfoReport {
            status: CompatibilityStatusCode::Compatible,
            migration: CompatibilityMigrationCode::None,
            expected_schema_version: 7,
            actual_schema_version: 7,
            expected_runtime_build_id: 0x1234_5678,
            actual_runtime_build_id: 0x1234_5678,
            expected_hardware_target_id: 0x2040,
            actual_hardware_target_id: 0x2040,
        },
        &mut expected,
    )
    .expect("encode expected compatibility-info reply");
    assert_eq!(&out[..compatibility_reply], &expected[..expected_len]);

    let reboot_len = proto::encode_reply(Cmd::Reboot, &[], &mut req).unwrap();
    let reboot_reply = server
        .handle_with_bench_tooling(&req[..reboot_len], &mut out, &mut owner)
        .expect("owned reboot reply");
    let (cmd, payload) =
        proto::decode_request(&out[..reboot_reply]).expect("decode owned reboot reply");
    assert_eq!(cmd, Cmd::Reboot);
    assert_eq!(payload, b"OK");
    assert!(owner.reboot_requested);
}
