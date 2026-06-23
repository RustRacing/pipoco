//! TunerStudio server: minimal handler for SIG/OUTPC/PING and page R/W (Phase 2)
use super::{outpc::Outpc, proto};
use crate::pages::{ts_page_schema_version, TsPageDescriptor, TS_PAGE_COUNT, TS_PAGE_DESCRIPTORS};

/// Provider for OUTPC data (decoupled from EcuState)
pub trait OutpcProvider {
    fn fill_outpc(&self, out: &mut Outpc);
    fn engine_running(&self) -> bool {
        false
    }
}

/// Backing store for pages exposed to TunerStudio (fuel table, ignition table, etc.)
pub trait PageStore {
    fn page_len(&self, page: u8) -> Option<usize>;
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize>;
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError>;
    fn burn(&mut self) -> Result<(), PersistError> {
        Err(PersistError::Unsupported)
    }
    fn export_package_wire(&self, _out: &mut [u8]) -> Result<usize, PageError> {
        Err(PageError::Invalid)
    }
    fn import_package_wire(&mut self, _data: &[u8]) -> Result<(), PageError> {
        Err(PageError::Invalid)
    }
}

/// Product-owned owner surface for bounded TS bench-tooling commands.
pub trait BenchToolingOwner {
    fn handle_output_test(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize>;
    fn handle_tooth_stats(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize>;
    fn handle_tooth_composite_log(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize>;
    fn handle_version_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize>;
    fn handle_compatibility_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize>;
    fn handle_reboot(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize>;
}

#[derive(Copy, Clone, Default)]
pub struct NoBenchTooling;

impl BenchToolingOwner for NoBenchTooling {
    fn handle_output_test(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if payload.len() < 6 {
            return None;
        }
        proto::encode_reply(proto::Cmd::OutputTest, b"ERR", out)
    }

    fn handle_tooth_stats(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        proto::encode_reply(proto::Cmd::ToothStats, b"ERR", out)
    }

    fn handle_tooth_composite_log(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        proto::encode_reply(proto::Cmd::ToothCompositeLog, b"ERR", out)
    }

    fn handle_version_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        proto::encode_reply(proto::Cmd::VersionInfo, b"ERR", out)
    }

    fn handle_compatibility_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        proto::encode_reply(proto::Cmd::CompatibilityInfo, b"ERR", out)
    }

    fn handle_reboot(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        if !payload.is_empty() {
            return None;
        }
        proto::encode_reply(proto::Cmd::Reboot, b"ERR", out)
    }
}

pub const TOOTH_COMPOSITE_LOG_REPLY_VERSION: u8 = 1;
pub const TOOTH_COMPOSITE_LOG_MAX_ENTRIES: usize = 16;
pub const TOOTH_COMPOSITE_LOG_HEADER_LEN: usize = 4;
pub const TOOTH_COMPOSITE_LOG_ENTRY_WIRE_LEN: usize = 10;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ToothCompositeLogKind {
    #[default]
    TriggerEdge = 0,
    CamEdge = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct ToothCompositeLogEntry {
    pub kind: ToothCompositeLogKind,
    pub flags: u8,
    pub rpm: u16,
    pub angle_x10: i16,
    pub at_us: u32,
}

pub fn encode_tooth_composite_log_reply(
    entries: &[ToothCompositeLogEntry],
    overflow_count: u16,
    out: &mut [u8],
) -> Option<usize> {
    let count = entries.len().min(TOOTH_COMPOSITE_LOG_MAX_ENTRIES);
    let payload_len = TOOTH_COMPOSITE_LOG_HEADER_LEN + count * TOOTH_COMPOSITE_LOG_ENTRY_WIRE_LEN;
    let mut payload = [0u8; TOOTH_COMPOSITE_LOG_HEADER_LEN
        + TOOTH_COMPOSITE_LOG_MAX_ENTRIES * TOOTH_COMPOSITE_LOG_ENTRY_WIRE_LEN];
    payload[0] = TOOTH_COMPOSITE_LOG_REPLY_VERSION;
    payload[1] = count as u8;
    payload[2..4].copy_from_slice(&overflow_count.to_le_bytes());
    let mut offset = TOOTH_COMPOSITE_LOG_HEADER_LEN;
    for entry in entries.iter().take(count) {
        payload[offset] = entry.kind as u8;
        payload[offset + 1] = entry.flags;
        payload[offset + 2..offset + 4].copy_from_slice(&entry.rpm.to_le_bytes());
        payload[offset + 4..offset + 6].copy_from_slice(&entry.angle_x10.to_le_bytes());
        payload[offset + 6..offset + 10].copy_from_slice(&entry.at_us.to_le_bytes());
        offset += TOOTH_COMPOSITE_LOG_ENTRY_WIRE_LEN;
    }
    proto::encode_reply(proto::Cmd::ToothCompositeLog, &payload[..payload_len], out)
}

pub const VERSION_INFO_REPLY_VERSION: u8 = 1;

pub fn encode_version_info_reply(
    signature: &[u8],
    runtime_build_id: u32,
    hardware_target_id: u16,
    out: &mut [u8],
) -> Option<usize> {
    let signature_len = signature.len().min(u8::MAX as usize);
    let payload_len = 1 + 4 + 2 + 1 + signature_len;
    let mut payload = [0u8; 1 + 4 + 2 + 1 + 64];
    if signature_len > 64 {
        return None;
    }
    payload[0] = VERSION_INFO_REPLY_VERSION;
    payload[1..5].copy_from_slice(&runtime_build_id.to_le_bytes());
    payload[5..7].copy_from_slice(&hardware_target_id.to_le_bytes());
    payload[7] = signature_len as u8;
    payload[8..8 + signature_len].copy_from_slice(&signature[..signature_len]);
    proto::encode_reply(proto::Cmd::VersionInfo, &payload[..payload_len], out)
}

pub const COMPATIBILITY_INFO_REPLY_VERSION: u8 = 1;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CompatibilityStatusCode {
    #[default]
    Compatible = 0,
    SchemaVersionMismatch = 1,
    RuntimeBuildMismatch = 2,
    HardwareTargetMismatch = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CompatibilityMigrationCode {
    #[default]
    None = 0,
    MigrationRequired = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CompatibilityInfoReport {
    pub status: CompatibilityStatusCode,
    pub migration: CompatibilityMigrationCode,
    pub expected_schema_version: u16,
    pub actual_schema_version: u16,
    pub expected_runtime_build_id: u32,
    pub actual_runtime_build_id: u32,
    pub expected_hardware_target_id: u16,
    pub actual_hardware_target_id: u16,
}

pub fn encode_compatibility_info_reply(
    report: CompatibilityInfoReport,
    out: &mut [u8],
) -> Option<usize> {
    let mut payload = [0u8; 1 + 1 + 1 + 2 + 2 + 4 + 4 + 2 + 2];
    payload[0] = COMPATIBILITY_INFO_REPLY_VERSION;
    payload[1] = report.status as u8;
    payload[2] = report.migration as u8;
    payload[3..5].copy_from_slice(&report.expected_schema_version.to_le_bytes());
    payload[5..7].copy_from_slice(&report.actual_schema_version.to_le_bytes());
    payload[7..11].copy_from_slice(&report.expected_runtime_build_id.to_le_bytes());
    payload[11..15].copy_from_slice(&report.actual_runtime_build_id.to_le_bytes());
    payload[15..17].copy_from_slice(&report.expected_hardware_target_id.to_le_bytes());
    payload[17..19].copy_from_slice(&report.actual_hardware_target_id.to_le_bytes());
    proto::encode_reply(proto::Cmd::CompatibilityInfo, &payload, out)
}

pub const PAGE_CRC_INFO_REPLY_VERSION: u8 = 1;
pub const PAGE_CRC_INFO_MAX_ENTRIES: usize = TS_PAGE_COUNT;
pub const PAGE_CRC_INFO_HEADER_LEN: usize = 2;
pub const PAGE_CRC_INFO_ENTRY_WIRE_LEN: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PageCrcInfoEntry {
    pub page: u8,
    pub len: u16,
    pub schema_version: u16,
    pub layout_crc: u16,
    pub flags: u8,
}

fn page_crc_info_flags(descriptor: TsPageDescriptor) -> u8 {
    u8::from(descriptor.writable) | (u8::from(descriptor.persisted_setup_page) << 1)
}

fn page_crc_layout_crc(descriptor: TsPageDescriptor, len: u16, schema_version: u16) -> u16 {
    let mut payload = [0u8; 32];
    let mut offset = 0usize;
    payload[offset] = descriptor.page;
    offset += 1;
    payload[offset..offset + 2].copy_from_slice(&len.to_le_bytes());
    offset += 2;
    payload[offset..offset + 2].copy_from_slice(&schema_version.to_le_bytes());
    offset += 2;
    payload[offset] = page_crc_info_flags(descriptor);
    offset += 1;
    let label = descriptor.label.as_bytes();
    payload[offset..offset + label.len()].copy_from_slice(label);
    offset += label.len();
    proto::crc16_ccitt(&payload[..offset])
}

fn page_crc_info_entry(descriptor: TsPageDescriptor, len: u16) -> PageCrcInfoEntry {
    let schema_version = ts_page_schema_version(descriptor.page).unwrap_or(0);
    PageCrcInfoEntry {
        page: descriptor.page,
        len,
        schema_version,
        layout_crc: page_crc_layout_crc(descriptor, len, schema_version),
        flags: page_crc_info_flags(descriptor),
    }
}

pub fn encode_page_crc_info_reply(entries: &[PageCrcInfoEntry], out: &mut [u8]) -> Option<usize> {
    let count = entries.len().min(PAGE_CRC_INFO_MAX_ENTRIES);
    let payload_len = PAGE_CRC_INFO_HEADER_LEN + count * PAGE_CRC_INFO_ENTRY_WIRE_LEN;
    let mut payload =
        [0u8; PAGE_CRC_INFO_HEADER_LEN + PAGE_CRC_INFO_MAX_ENTRIES * PAGE_CRC_INFO_ENTRY_WIRE_LEN];
    payload[0] = PAGE_CRC_INFO_REPLY_VERSION;
    payload[1] = count as u8;
    let mut offset = PAGE_CRC_INFO_HEADER_LEN;
    for entry in entries.iter().take(count) {
        payload[offset] = entry.page;
        payload[offset + 1..offset + 3].copy_from_slice(&entry.len.to_le_bytes());
        payload[offset + 3..offset + 5].copy_from_slice(&entry.schema_version.to_le_bytes());
        payload[offset + 5..offset + 7].copy_from_slice(&entry.layout_crc.to_le_bytes());
        payload[offset + 7] = entry.flags;
        offset += PAGE_CRC_INFO_ENTRY_WIRE_LEN;
    }
    proto::encode_reply(proto::Cmd::PageCrcInfo, &payload[..payload_len], out)
}

#[derive(Debug, Copy, Clone)]
pub enum PageError {
    Invalid,
    WrongSize,
}
#[derive(Debug, Copy, Clone)]
pub enum PersistError {
    Unsupported,
    Fail,
    EngineRunning,
}

const PERSIST_ERR_RUNNING: &[u8] = &[1, b'R', b'U', b'N'];

#[derive(Default)]
pub struct NoPages;
impl PageStore for NoPages {
    fn page_len(&self, _page: u8) -> Option<usize> {
        None
    }
    fn read_page(&self, _page: u8, _out: &mut [u8]) -> Option<usize> {
        None
    }
    fn write_page(&mut self, _page: u8, _data: &[u8]) -> Result<(), PageError> {
        Err(PageError::Invalid)
    }
}

#[derive(Copy, Clone, Default, Debug)]
pub struct ServerStats {
    pub rx_ok: u32,
    pub rx_invalid: u32,
    pub read_ok: u32,
    pub write_ok: u32,
    pub write_err: u32,
    pub burn_ok: u32,
    pub burn_err: u32,
}

pub struct TunerstudioServer<P: OutpcProvider, S: PageStore> {
    signature: &'static [u8],
    provider: P,
    store: S,
    stats: ServerStats,
}

pub type TsServer<P, S> = TunerstudioServer<P, S>;

impl<P: OutpcProvider, S: PageStore> TunerstudioServer<P, S> {
    pub const fn new(signature: &'static [u8], provider: P, store: S) -> Self {
        Self {
            signature,
            provider,
            store,
            stats: ServerStats {
                rx_ok: 0,
                rx_invalid: 0,
                read_ok: 0,
                write_ok: 0,
                write_err: 0,
                burn_ok: 0,
                burn_err: 0,
            },
        }
    }

    pub fn store(&self) -> &S {
        &self.store
    }

    pub fn store_mut(&mut self) -> &mut S {
        &mut self.store
    }

    /// Handle a single request frame in `req`, writing reply into `out`.
    /// Returns reply length on success.
    pub fn handle(&mut self, req: &[u8], out: &mut [u8]) -> Option<usize> {
        let mut bench_tooling = NoBenchTooling;
        self.handle_with_bench_tooling(req, out, &mut bench_tooling)
    }

    /// Handle a single request frame in `req`, writing reply into `out`, while
    /// routing bounded bench-tooling commands through the supplied owner.
    pub fn handle_with_bench_tooling<B: BenchToolingOwner>(
        &mut self,
        req: &[u8],
        out: &mut [u8],
        bench_tooling: &mut B,
    ) -> Option<usize> {
        let decoded = proto::decode_request(req);
        let (cmd, payload) = match decoded {
            Some(v) => v,
            None => {
                self.stats.rx_invalid = self.stats.rx_invalid.saturating_add(1);
                return None;
            }
        };
        self.stats.rx_ok = self.stats.rx_ok.saturating_add(1);
        match cmd {
            proto::Cmd::Sig => proto::encode_reply(proto::Cmd::Sig, self.signature, out),
            proto::Cmd::Outpc => {
                let mut block = Outpc::default();
                self.provider.fill_outpc(&mut block);
                let mut encoded = [0u8; Outpc::WIRE_LEN];
                let len = block.encode(&mut encoded)?;
                proto::encode_reply(proto::Cmd::Outpc, &encoded[..len], out)
            }
            proto::Cmd::Ping => proto::encode_reply(proto::Cmd::Ping, b"PONG", out),
            proto::Cmd::ReadPage => {
                if payload.len() != 1 && payload.len() != 5 {
                    return None;
                }
                let page = payload[0];
                let page_len = self.store.page_len(page)?;
                let (offset, len) = if payload.len() == 5 {
                    let offset = u16::from_le_bytes([payload[1], payload[2]]) as usize;
                    let len = u16::from_le_bytes([payload[3], payload[4]]) as usize;
                    if offset.checked_add(len)? > page_len {
                        return None;
                    }
                    (offset, len)
                } else {
                    (0, page_len)
                };
                let mut buf = [0u8; 1024];
                let n = self.store.read_page(page, &mut buf[..page_len])?;
                if n < offset + len {
                    return None;
                }
                self.stats.read_ok = self.stats.read_ok.saturating_add(1);
                proto::encode_reply(proto::Cmd::ReadPage, &buf[offset..offset + len], out)
            }
            proto::Cmd::WritePage => {
                if payload.len() < 2 {
                    return None;
                }
                let page = payload[0];
                if self.provider.engine_running() {
                    self.stats.write_err = self.stats.write_err.saturating_add(1);
                    return proto::encode_reply(proto::Cmd::WritePage, b"ERR", out);
                }
                let page_len = match self.store.page_len(page) {
                    Some(len) => len,
                    None => {
                        self.stats.write_err = self.stats.write_err.saturating_add(1);
                        return proto::encode_reply(proto::Cmd::WritePage, b"ERR", out);
                    }
                };
                let write_result = if payload.len() == page_len + 1 {
                    self.store.write_page(page, &payload[1..])
                } else if payload.len() >= 5 {
                    let offset = u16::from_le_bytes([payload[1], payload[2]]) as usize;
                    let len = u16::from_le_bytes([payload[3], payload[4]]) as usize;
                    let data = &payload[5..];
                    if len == 0 || data.len() != len {
                        self.stats.write_err = self.stats.write_err.saturating_add(1);
                        return proto::encode_reply(proto::Cmd::WritePage, b"ERR", out);
                    }
                    if offset.checked_add(len).is_none_or(|end| end > page_len) {
                        Err(PageError::WrongSize)
                    } else {
                        let mut page_buf = [0u8; 1024];
                        match self.store.read_page(page, &mut page_buf[..page_len]) {
                            Some(n) if n == page_len => {
                                page_buf[offset..offset + len].copy_from_slice(data);
                                self.store.write_page(page, &page_buf[..page_len])
                            }
                            _ => Err(PageError::Invalid),
                        }
                    }
                } else {
                    Err(PageError::WrongSize)
                };
                if write_result.is_ok() {
                    self.stats.write_ok = self.stats.write_ok.saturating_add(1);
                    proto::encode_reply(proto::Cmd::WritePage, b"OK", out)
                } else {
                    self.stats.write_err = self.stats.write_err.saturating_add(1);
                    proto::encode_reply(proto::Cmd::WritePage, b"ERR", out)
                }
            }
            proto::Cmd::Burn => {
                if !payload.is_empty() && payload.len() != 1 {
                    return None;
                }
                if self.provider.engine_running() {
                    self.stats.burn_err = self.stats.burn_err.saturating_add(1);
                    proto::encode_reply(proto::Cmd::Burn, PERSIST_ERR_RUNNING, out)
                } else {
                    match self.store.burn() {
                        Ok(()) => {
                            self.stats.burn_ok = self.stats.burn_ok.saturating_add(1);
                            proto::encode_reply(proto::Cmd::Burn, b"OK", out)
                        }
                        Err(PersistError::EngineRunning) => {
                            self.stats.burn_err = self.stats.burn_err.saturating_add(1);
                            proto::encode_reply(proto::Cmd::Burn, PERSIST_ERR_RUNNING, out)
                        }
                        Err(_) => {
                            self.stats.burn_err = self.stats.burn_err.saturating_add(1);
                            proto::encode_reply(proto::Cmd::Burn, b"ERR", out)
                        }
                    }
                }
            }
            proto::Cmd::PackageExport => {
                if !payload.is_empty() {
                    return None;
                }
                let mut buf = [0u8; 512];
                match self.store.export_package_wire(&mut buf) {
                    Ok(len) => {
                        self.stats.read_ok = self.stats.read_ok.saturating_add(1);
                        proto::encode_reply(proto::Cmd::PackageExport, &buf[..len], out)
                    }
                    Err(_) => proto::encode_reply(proto::Cmd::PackageExport, b"ERR", out),
                }
            }
            proto::Cmd::PackageImport => {
                if payload.is_empty() {
                    return None;
                }
                if self.provider.engine_running() {
                    self.stats.write_err = self.stats.write_err.saturating_add(1);
                    return proto::encode_reply(proto::Cmd::PackageImport, b"ERR", out);
                }
                match self.store.import_package_wire(payload) {
                    Ok(()) => {
                        self.stats.write_ok = self.stats.write_ok.saturating_add(1);
                        proto::encode_reply(proto::Cmd::PackageImport, b"OK", out)
                    }
                    Err(_) => {
                        self.stats.write_err = self.stats.write_err.saturating_add(1);
                        proto::encode_reply(proto::Cmd::PackageImport, b"ERR", out)
                    }
                }
            }
            proto::Cmd::PageCrcInfo => {
                if !payload.is_empty() {
                    return None;
                }
                let mut entries = [PageCrcInfoEntry::default(); PAGE_CRC_INFO_MAX_ENTRIES];
                let mut count = 0usize;
                for descriptor in TS_PAGE_DESCRIPTORS {
                    let Some(len) = self.store.page_len(descriptor.page) else {
                        continue;
                    };
                    let Ok(len) = u16::try_from(len) else {
                        return None;
                    };
                    if count == entries.len() {
                        break;
                    }
                    entries[count] = page_crc_info_entry(descriptor, len);
                    count += 1;
                }
                self.stats.read_ok = self.stats.read_ok.saturating_add(1);
                encode_page_crc_info_reply(&entries[..count], out)
            }
            proto::Cmd::OutputTest => bench_tooling.handle_output_test(payload, out),
            proto::Cmd::ToothStats => bench_tooling.handle_tooth_stats(payload, out),
            proto::Cmd::ToothCompositeLog => bench_tooling.handle_tooth_composite_log(payload, out),
            proto::Cmd::VersionInfo => bench_tooling.handle_version_info(payload, out),
            proto::Cmd::CompatibilityInfo => bench_tooling.handle_compatibility_info(payload, out),
            proto::Cmd::Reboot => bench_tooling.handle_reboot(payload, out),
        }
    }

    pub fn stats(&self) -> ServerStats {
        self.stats
    }
    pub fn reset_stats(&mut self) {
        self.stats = ServerStats::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;
    struct Dummy;
    impl OutpcProvider for Dummy {
        fn fill_outpc(&self, out: &mut Outpc) {
            out.rpm = 1234;
            out.tps_percent = 42;
        }
    }

    #[test]
    fn test_sig_outpc_ping() {
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, Dummy, NoPages);
        // SIG
        let mut req = [0u8; 64];
        let len = proto::encode_reply(proto::Cmd::Sig, &[], &mut req).unwrap();
        let mut out = [0u8; 128];
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        assert!(rlen > 0);
        // OUTPC request
        let len = proto::encode_reply(proto::Cmd::Outpc, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        assert!(rlen > 0);
        // PING
        let len = proto::encode_reply(proto::Cmd::Ping, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        assert!(rlen > 0);
    }

    #[derive(Default)]
    struct DummyStore {
        page: [u8; 8],
    }
    impl PageStore for DummyStore {
        fn page_len(&self, page: u8) -> Option<usize> {
            if page == 1 {
                Some(8)
            } else {
                None
            }
        }
        fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
            if page != 1 {
                return None;
            };
            out[..8].copy_from_slice(&self.page);
            Some(8)
        }
        fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
            if page != 1 || data.len() != 8 {
                return Err(PageError::WrongSize);
            };
            self.page.copy_from_slice(&data[..8]);
            Ok(())
        }
    }

    struct RunState(bool);
    impl OutpcProvider for RunState {
        fn fill_outpc(&self, out: &mut Outpc) {
            out.rpm = 1234;
        }
        fn engine_running(&self) -> bool {
            self.0
        }
    }

    struct ToggleRunState {
        running: Rc<Cell<bool>>,
    }
    impl OutpcProvider for ToggleRunState {
        fn fill_outpc(&self, out: &mut Outpc) {
            out.rpm = 1234;
        }
        fn engine_running(&self) -> bool {
            self.running.get()
        }
    }

    #[derive(Default)]
    struct RunningBurnStore;
    impl PageStore for RunningBurnStore {
        fn page_len(&self, _page: u8) -> Option<usize> {
            None
        }
        fn read_page(&self, _page: u8, _out: &mut [u8]) -> Option<usize> {
            None
        }
        fn write_page(&mut self, _page: u8, _data: &[u8]) -> Result<(), PageError> {
            Err(PageError::Invalid)
        }
        fn burn(&mut self) -> Result<(), PersistError> {
            Err(PersistError::EngineRunning)
        }
    }

    #[derive(Clone)]
    struct GuardedBurnStore {
        staged: Rc<RefCell<[u8; 8]>>,
        persisted: Rc<RefCell<[u8; 8]>>,
        burn_called: Rc<Cell<bool>>,
    }

    impl Default for GuardedBurnStore {
        fn default() -> Self {
            Self {
                staged: Rc::new(RefCell::new([0u8; 8])),
                persisted: Rc::new(RefCell::new([0u8; 8])),
                burn_called: Rc::new(Cell::new(false)),
            }
        }
    }

    impl PageStore for GuardedBurnStore {
        fn page_len(&self, page: u8) -> Option<usize> {
            if page == 1 {
                Some(8)
            } else {
                None
            }
        }
        fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
            if page != 1 || out.len() < 8 {
                return None;
            }
            let persisted = self.persisted.borrow();
            out[..8].copy_from_slice(&persisted[..]);
            Some(8)
        }
        fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
            if page != 1 || data.len() != 8 {
                return Err(PageError::WrongSize);
            }
            self.staged.borrow_mut().copy_from_slice(data);
            Ok(())
        }
        fn burn(&mut self) -> Result<(), PersistError> {
            self.burn_called.set(true);
            let staged = self.staged.borrow();
            self.persisted.borrow_mut().copy_from_slice(&staged[..]);
            Ok(())
        }
    }

    struct BenchOwner {
        output_called: Rc<Cell<bool>>,
    }

    impl BenchToolingOwner for BenchOwner {
        fn handle_output_test(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
            if payload.len() < 6 {
                return None;
            }
            self.output_called.set(true);
            proto::encode_reply(proto::Cmd::OutputTest, b"OK", out)
        }

        fn handle_tooth_stats(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
            if !payload.is_empty() {
                return None;
            }
            proto::encode_reply(proto::Cmd::ToothStats, &[0xb2, 0x0c, 1], out)
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
            encode_version_info_reply(crate::TS_SIGNATURE, 0x1234_5678, 0x2040, out)
        }

        fn handle_compatibility_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
            if !payload.is_empty() {
                return None;
            }
            encode_compatibility_info_reply(
                CompatibilityInfoReport {
                    status: CompatibilityStatusCode::Compatible,
                    migration: CompatibilityMigrationCode::None,
                    expected_schema_version: 1,
                    actual_schema_version: 1,
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
            proto::encode_reply(proto::Cmd::Reboot, b"OK", out)
        }
    }

    #[test]
    fn test_read_write_page() {
        let store = DummyStore::default();
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, Dummy, store);
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];
        // Write page 1
        let mut payload = [0u8; 9];
        payload[0] = 1;
        payload[1..].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let len = proto::encode_reply(proto::Cmd::WritePage, payload.as_slice(), &mut req).unwrap();
        let _ = srv.handle(&req[..len], &mut out).unwrap();
        // Read page 1
        let len = proto::encode_reply(proto::Cmd::ReadPage, &[1], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let decoded = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(decoded.0, proto::Cmd::ReadPage);
        assert_eq!(decoded.1, &[1, 2, 3, 4, 5, 6, 7, 8][..]);
    }

    #[test]
    fn test_write_rejected_when_engine_running() {
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];
        let mut payload = [0u8; 9];
        payload[0] = 1;
        payload[1..].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let store = DummyStore::default();
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, RunState(false), store);
        let len = proto::encode_reply(proto::Cmd::WritePage, payload.as_slice(), &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::WritePage);
        assert_eq!(payload, b"OK");
        assert_eq!(srv.stats().write_ok, 1);

        let store = DummyStore::default();
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, RunState(true), store);
        let len = proto::encode_reply(proto::Cmd::WritePage, payload, &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::WritePage);
        assert_eq!(payload, b"ERR");
        assert_eq!(srv.stats().write_err, 1);
    }

    #[test]
    fn persist_while_running() {
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];
        let store = RunningBurnStore;
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, RunState(false), store);
        let len = proto::encode_reply(proto::Cmd::Burn, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::Burn);
        assert_eq!(payload[0], 1);
        assert_eq!(&payload[1..], b"RUN");
        assert_eq!(srv.stats().burn_err, 1);
    }

    #[test]
    fn burn_is_rejected_before_persistence_when_engine_is_running() {
        let running = Rc::new(Cell::new(false));
        let provider = ToggleRunState {
            running: running.clone(),
        };
        let store = GuardedBurnStore::default();
        let burn_called = store.burn_called.clone();
        let persisted = store.persisted.clone();
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, provider, store);
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];

        let mut payload = [0u8; 9];
        payload[0] = 1;
        payload[1..].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        let len = proto::encode_reply(proto::Cmd::WritePage, payload.as_slice(), &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::WritePage);
        assert_eq!(payload, b"OK");

        running.set(true);
        let len = proto::encode_reply(proto::Cmd::Burn, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::Burn);
        assert_eq!(payload, PERSIST_ERR_RUNNING);
        assert_eq!(srv.stats().burn_err, 1);
        assert!(!burn_called.get());
        assert_eq!(&*persisted.borrow(), &[0u8; 8]);
    }

    #[test]
    fn unsupported_bench_commands_reply_err_instead_of_returning_none() {
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, Dummy, NoPages);
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];

        let output_len =
            proto::encode_reply(proto::Cmd::OutputTest, &[0, 0, 0, 0, 0, 0], &mut req).unwrap();
        let output_reply = srv.handle(&req[..output_len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..output_reply]).unwrap();
        assert_eq!(cmd, proto::Cmd::OutputTest);
        assert_eq!(payload, b"ERR");

        let tooth_len = proto::encode_reply(proto::Cmd::ToothStats, &[], &mut req).unwrap();
        let tooth_reply = srv.handle(&req[..tooth_len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..tooth_reply]).unwrap();
        assert_eq!(cmd, proto::Cmd::ToothStats);
        assert_eq!(payload, b"ERR");
    }

    #[test]
    fn custom_bench_owner_handles_output_test_and_logger_commands() {
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, Dummy, NoPages);
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];
        let output_called = Rc::new(Cell::new(false));
        let mut owner = BenchOwner {
            output_called: output_called.clone(),
        };

        let output_len =
            proto::encode_reply(proto::Cmd::OutputTest, &[2, 5, 0, 6, 0, 3], &mut req).unwrap();
        let output_reply = srv
            .handle_with_bench_tooling(&req[..output_len], &mut out, &mut owner)
            .unwrap();
        let (cmd, payload) = proto::decode_request(&out[..output_reply]).unwrap();
        assert_eq!(cmd, proto::Cmd::OutputTest);
        assert_eq!(payload, b"OK");
        assert!(output_called.get());

        let tooth_len = proto::encode_reply(proto::Cmd::ToothStats, &[], &mut req).unwrap();
        let tooth_reply = srv
            .handle_with_bench_tooling(&req[..tooth_len], &mut out, &mut owner)
            .unwrap();
        let (cmd, payload) = proto::decode_request(&out[..tooth_reply]).unwrap();
        assert_eq!(cmd, proto::Cmd::ToothStats);
        assert_eq!(payload, &[0xb2, 0x0c, 1]);

        let log_len = proto::encode_reply(proto::Cmd::ToothCompositeLog, &[], &mut req).unwrap();
        let log_reply = srv
            .handle_with_bench_tooling(&req[..log_len], &mut out, &mut owner)
            .unwrap();
        let (cmd, payload) = proto::decode_request(&out[..log_reply]).unwrap();
        assert_eq!(cmd, proto::Cmd::ToothCompositeLog);
        assert_eq!(payload[0], TOOTH_COMPOSITE_LOG_REPLY_VERSION);
        assert_eq!(payload[1], 2);
    }

    #[test]
    fn tooth_composite_log_reply_packs_header_and_entries() {
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
        let mut out = [0u8; 128];
        let len = encode_tooth_composite_log_reply(&entries, 2, &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..len]).unwrap();
        assert_eq!(cmd, proto::Cmd::ToothCompositeLog);
        assert_eq!(payload[0], TOOTH_COMPOSITE_LOG_REPLY_VERSION);
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
    }

    #[test]
    fn unsupported_tooth_composite_log_replies_err_without_owner() {
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, Dummy, NoPages);
        let mut req = [0u8; 64];
        let mut out = [0u8; 128];
        let len = proto::encode_reply(proto::Cmd::ToothCompositeLog, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::ToothCompositeLog);
        assert_eq!(payload, b"ERR");
    }

    #[test]
    fn version_info_reply_packs_runtime_target_and_signature() {
        let mut out = [0u8; 128];
        let len =
            encode_version_info_reply(crate::TS_SIGNATURE, 0x1234_5678, 0x2040, &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..len]).unwrap();
        assert_eq!(cmd, proto::Cmd::VersionInfo);
        assert_eq!(payload[0], VERSION_INFO_REPLY_VERSION);
        assert_eq!(
            u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]),
            0x1234_5678
        );
        assert_eq!(u16::from_le_bytes([payload[5], payload[6]]), 0x2040);
        assert_eq!(payload[7] as usize, crate::TS_SIGNATURE.len());
        assert_eq!(
            &payload[8..8 + crate::TS_SIGNATURE.len()],
            crate::TS_SIGNATURE
        );
    }

    #[test]
    fn compatibility_info_reply_packs_current_review_shape() {
        let report = CompatibilityInfoReport {
            status: CompatibilityStatusCode::RuntimeBuildMismatch,
            migration: CompatibilityMigrationCode::MigrationRequired,
            expected_schema_version: 1,
            actual_schema_version: 0,
            expected_runtime_build_id: 0x1234_5678,
            actual_runtime_build_id: 0xDEAD_BEEF,
            expected_hardware_target_id: 0x2040,
            actual_hardware_target_id: 0x2041,
        };
        let mut out = [0u8; 128];
        let len = encode_compatibility_info_reply(report, &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..len]).unwrap();
        assert_eq!(cmd, proto::Cmd::CompatibilityInfo);
        assert_eq!(payload[0], COMPATIBILITY_INFO_REPLY_VERSION);
        assert_eq!(
            payload[1],
            CompatibilityStatusCode::RuntimeBuildMismatch as u8
        );
        assert_eq!(
            payload[2],
            CompatibilityMigrationCode::MigrationRequired as u8
        );
        assert_eq!(u16::from_le_bytes([payload[3], payload[4]]), 1);
        assert_eq!(u16::from_le_bytes([payload[5], payload[6]]), 0);
        assert_eq!(
            u32::from_le_bytes([payload[7], payload[8], payload[9], payload[10]]),
            0x1234_5678
        );
        assert_eq!(
            u32::from_le_bytes([payload[11], payload[12], payload[13], payload[14]]),
            0xDEAD_BEEF
        );
        assert_eq!(u16::from_le_bytes([payload[15], payload[16]]), 0x2040);
        assert_eq!(u16::from_le_bytes([payload[17], payload[18]]), 0x2041);
    }

    #[test]
    fn page_crc_info_reply_packs_header_and_entries() {
        let entries = [
            PageCrcInfoEntry {
                page: 1,
                len: 512,
                schema_version: 1,
                layout_crc: 0x1234,
                flags: 0x03,
            },
            PageCrcInfoEntry {
                page: 7,
                len: 32,
                schema_version: 2,
                layout_crc: 0x5678,
                flags: 0x00,
            },
        ];
        let mut out = [0u8; 256];
        let len = encode_page_crc_info_reply(&entries, &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..len]).unwrap();
        assert_eq!(cmd, proto::Cmd::PageCrcInfo);
        assert_eq!(payload[0], PAGE_CRC_INFO_REPLY_VERSION);
        assert_eq!(payload[1], 2);
        assert_eq!(payload[2], 1);
        assert_eq!(u16::from_le_bytes([payload[3], payload[4]]), 512);
        assert_eq!(u16::from_le_bytes([payload[5], payload[6]]), 1);
        assert_eq!(u16::from_le_bytes([payload[7], payload[8]]), 0x1234);
        assert_eq!(payload[9], 0x03);
        assert_eq!(payload[10], 7);
        assert_eq!(u16::from_le_bytes([payload[11], payload[12]]), 32);
        assert_eq!(u16::from_le_bytes([payload[13], payload[14]]), 2);
        assert_eq!(u16::from_le_bytes([payload[15], payload[16]]), 0x5678);
        assert_eq!(payload[17], 0x00);
    }

    #[test]
    fn page_crc_info_command_reports_registered_page_surface() {
        let mut srv = TunerstudioServer::new(crate::TS_SIGNATURE, Dummy, DummyStore::default());
        let mut req = [0u8; 64];
        let mut out = [0u8; 256];
        let len = proto::encode_reply(proto::Cmd::PageCrcInfo, &[], &mut req).unwrap();
        let rlen = srv.handle(&req[..len], &mut out).unwrap();
        let (cmd, payload) = proto::decode_request(&out[..rlen]).unwrap();
        assert_eq!(cmd, proto::Cmd::PageCrcInfo);
        assert_eq!(payload[0], PAGE_CRC_INFO_REPLY_VERSION);
        assert_eq!(payload[1], 1);
        let fuel = crate::pages::ts_page_descriptor(1).unwrap();
        let expected = page_crc_info_entry(fuel, 8);
        assert_eq!(payload[2], expected.page);
        assert_eq!(u16::from_le_bytes([payload[3], payload[4]]), expected.len);
        assert_eq!(
            u16::from_le_bytes([payload[5], payload[6]]),
            expected.schema_version
        );
        assert_eq!(
            u16::from_le_bytes([payload[7], payload[8]]),
            expected.layout_crc
        );
        assert_eq!(payload[9], expected.flags);
    }
}
