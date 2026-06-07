use crate::persist_spec::{
    persist_decode, persist_encode, EncodedPersistRecord, PersistPage, PersistPageId,
    PERSIST_ANGLES_PAGE_BYTES, PERSIST_FUEL_PAGE_BYTES, PERSIST_IGNITION_PAGE_BYTES,
    PERSIST_MAX_PAYLOAD_BYTES, PERSIST_RECORD_MAX_BYTES, PERSIST_SCHEMA_VERSION_CURRENT,
};

pub const TS_OUTPC_PAGE_BYTES: usize = 64;
pub const TS_PROTO_MAGIC: u16 = 0x55AA;
pub const TS_DIAG_LOG_CAPACITY: usize = 64;
pub const TS_DIAG_LOG_ENTRY_BYTES: usize = 9;
pub const TS_DIAG_LOG_MAX_ENCODED_BYTES: usize = TS_DIAG_LOG_CAPACITY * TS_DIAG_LOG_ENTRY_BYTES;

const CMD_GET_SIGNATURE: u8 = 0x10;
const CMD_GET_OUTPC: u8 = 0x11;
const CMD_READ_PAGE: u8 = 0x20;
const CMD_WRITE_PAGE: u8 = 0x21;
const CMD_BURN: u8 = 0x22;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsBurnSaveError {
    UnknownPage,
    WriteOutOfRange,
    EngineRunning,
    CrcMismatch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsStagedPage {
    pub len: u16,
    pub bytes: [u8; PERSIST_MAX_PAYLOAD_BYTES],
}

impl TsStagedPage {
    pub const fn new(page_len: usize) -> Self {
        Self {
            len: page_len as u16,
            bytes: [0u8; PERSIST_MAX_PAYLOAD_BYTES],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsCommittedPage {
    pub len: u16,
    pub bytes: [u8; PERSIST_RECORD_MAX_BYTES],
}

impl TsCommittedPage {
    pub const fn empty() -> Self {
        Self {
            len: 0,
            bytes: [0u8; PERSIST_RECORD_MAX_BYTES],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsBurnSaveStore {
    pub staged_fuel: TsStagedPage,
    pub staged_ignition: TsStagedPage,
    pub staged_angles: TsStagedPage,
    pub committed_fuel: TsCommittedPage,
    pub committed_ignition: TsCommittedPage,
    pub committed_angles: TsCommittedPage,
}

impl Default for TsBurnSaveStore {
    fn default() -> Self {
        Self {
            staged_fuel: TsStagedPage::new(PERSIST_FUEL_PAGE_BYTES),
            staged_ignition: TsStagedPage::new(PERSIST_IGNITION_PAGE_BYTES),
            staged_angles: TsStagedPage::new(PERSIST_ANGLES_PAGE_BYTES),
            committed_fuel: TsCommittedPage::empty(),
            committed_ignition: TsCommittedPage::empty(),
            committed_angles: TsCommittedPage::empty(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsPageId {
    Fuel = 1,
    Ignition = 2,
    Angles = 3,
    Outpc = 4,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsPageMeta {
    pub page_id: TsPageId,
    pub signature: u32,
    pub payload_size: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsPageMetaError {
    UnknownPage,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct OutpcFrame {
    pub rpm: u16,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub pw_corr_us: u16,
    pub advance_deg10: i16,
    pub sync_state_code: u8,
    pub cut_reason_code: u8,
    pub status_flags: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OutpcCodecError {
    InvalidLength,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsDispatchState {
    Idle,
    RxFrame,
    Decode,
    Execute,
    EncodeReply,
    ErrorReply,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsDecodeError {
    FrameTooShort,
    BadMagic,
    InvalidLength,
    Truncated,
    CrcMismatch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsCommandDecodeError {
    WrongPayloadLength {
        command_id: u8,
        expected: u16,
        actual: u16,
    },
    PayloadTooShort {
        command_id: u8,
        minimum: u16,
        actual: u16,
    },
    UnknownPage {
        command_id: u8,
        page_number: u8,
    },
    PageRangeOutOfBounds {
        command_id: u8,
        page_number: u8,
        offset: u16,
        len: u16,
        page_len: u16,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsDispatchError {
    Decode(TsDecodeError),
    CommandDecode(TsCommandDecodeError),
    UnknownCommand(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TsEffect<'a> {
    ReadPage {
        page_number: u8,
        offset: u16,
        len: u16,
    },
    WritePage {
        page_number: u8,
        offset: u16,
        bytes: &'a [u8],
    },
    Burn {
        page_number: u8,
    },
    GetOutpc,
    GetSignature {
        page_number: u8,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsDispatchResult<'a> {
    pub state_len: u8,
    pub states: [TsDispatchState; 6],
    pub effect: Result<TsEffect<'a>, TsDispatchError>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct TsDiagLogEntry {
    pub timestamp_us: u32,
    pub code: u16,
    pub source: u8,
    pub context: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsDiagLogRing {
    pub len: u8,
    pub head: u8,
    pub tail: u8,
    pub entries: [TsDiagLogEntry; TS_DIAG_LOG_CAPACITY],
}

impl Default for TsDiagLogRing {
    fn default() -> Self {
        Self {
            len: 0,
            head: 0,
            tail: 0,
            entries: [TsDiagLogEntry::default(); TS_DIAG_LOG_CAPACITY],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TsDiagLogEncoded {
    pub len: u16,
    pub bytes: [u8; TS_DIAG_LOG_MAX_ENCODED_BYTES],
}

impl Default for TsDiagLogEncoded {
    fn default() -> Self {
        Self {
            len: 0,
            bytes: [0u8; TS_DIAG_LOG_MAX_ENCODED_BYTES],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DecodedFrame<'a> {
    command_id: u8,
    payload: &'a [u8],
}

pub const fn page_meta(page_number: u8) -> Result<TsPageMeta, TsPageMetaError> {
    match page_number {
        1 => Ok(TsPageMeta {
            page_id: TsPageId::Fuel,
            signature: 0x46554C31, // "FUL1"
            payload_size: PERSIST_FUEL_PAGE_BYTES as u16,
        }),
        2 => Ok(TsPageMeta {
            page_id: TsPageId::Ignition,
            signature: 0x49474E31, // "IGN1"
            payload_size: PERSIST_IGNITION_PAGE_BYTES as u16,
        }),
        3 => Ok(TsPageMeta {
            page_id: TsPageId::Angles,
            signature: 0x414E4731, // "ANG1"
            payload_size: PERSIST_ANGLES_PAGE_BYTES as u16,
        }),
        4 => Ok(TsPageMeta {
            page_id: TsPageId::Outpc,
            signature: 0x4F555431, // "OUT1"
            payload_size: TS_OUTPC_PAGE_BYTES as u16,
        }),
        _ => Err(TsPageMetaError::UnknownPage),
    }
}

pub fn write_page(
    store: &mut TsBurnSaveStore,
    page_number: u8,
    offset: u16,
    bytes: &[u8],
) -> Result<(), TsBurnSaveError> {
    let staged = staged_page_mut(store, page_number)?;
    let start = offset as usize;
    let end = start.saturating_add(bytes.len());
    let page_len = staged.len as usize;
    if end > page_len {
        return Err(TsBurnSaveError::WriteOutOfRange);
    }

    let mut idx = 0usize;
    while idx < bytes.len() {
        staged.bytes[start + idx] = bytes[idx];
        idx += 1;
    }
    Ok(())
}

pub fn burn_page(
    store: &mut TsBurnSaveStore,
    page_number: u8,
    engine_running: bool,
) -> Result<(), TsBurnSaveError> {
    burn_page_internal(store, page_number, engine_running, false)
}

pub fn save_all(store: &mut TsBurnSaveStore, engine_running: bool) -> Result<(), TsBurnSaveError> {
    burn_page(store, 1, engine_running)?;
    burn_page(store, 2, engine_running)?;
    burn_page(store, 3, engine_running)?;
    Ok(())
}

fn burn_page_internal(
    store: &mut TsBurnSaveStore,
    page_number: u8,
    engine_running: bool,
    inject_crc_fault: bool,
) -> Result<(), TsBurnSaveError> {
    if engine_running {
        return Err(TsBurnSaveError::EngineRunning);
    }

    let page_id = page_number_to_persist(page_number)?;
    let staged = staged_page(store, page_number)?;
    let page_len = staged.len as usize;
    let page = PersistPage::new(
        PERSIST_SCHEMA_VERSION_CURRENT,
        page_id,
        &staged.bytes[..page_len],
    )
    .map_err(|_| TsBurnSaveError::WriteOutOfRange)?;

    let mut encoded = persist_encode(&page).map_err(|_| TsBurnSaveError::WriteOutOfRange)?;
    if inject_crc_fault {
        encoded.bytes[encoded.len as usize - 1] ^= 0x01;
    }
    if persist_decode(&encoded.bytes[..encoded.len as usize]).is_err() {
        return Err(TsBurnSaveError::CrcMismatch);
    }

    let committed = committed_page_mut(store, page_number)?;
    *committed = TsCommittedPage {
        len: encoded.len,
        bytes: encoded.bytes,
    };
    Ok(())
}

pub fn committed_page_record(
    store: &TsBurnSaveStore,
    page_number: u8,
) -> Result<EncodedPersistRecord, TsBurnSaveError> {
    let committed = committed_page(store, page_number)?;
    if committed.len == 0 {
        return Err(TsBurnSaveError::WriteOutOfRange);
    }
    Ok(EncodedPersistRecord {
        len: committed.len,
        bytes: committed.bytes,
    })
}

fn page_number_to_persist(page_number: u8) -> Result<PersistPageId, TsBurnSaveError> {
    match page_number {
        1 => Ok(PersistPageId::Fuel),
        2 => Ok(PersistPageId::Ignition),
        3 => Ok(PersistPageId::Angles),
        _ => Err(TsBurnSaveError::UnknownPage),
    }
}

fn staged_page(store: &TsBurnSaveStore, page_number: u8) -> Result<&TsStagedPage, TsBurnSaveError> {
    match page_number {
        1 => Ok(&store.staged_fuel),
        2 => Ok(&store.staged_ignition),
        3 => Ok(&store.staged_angles),
        _ => Err(TsBurnSaveError::UnknownPage),
    }
}

fn staged_page_mut(
    store: &mut TsBurnSaveStore,
    page_number: u8,
) -> Result<&mut TsStagedPage, TsBurnSaveError> {
    match page_number {
        1 => Ok(&mut store.staged_fuel),
        2 => Ok(&mut store.staged_ignition),
        3 => Ok(&mut store.staged_angles),
        _ => Err(TsBurnSaveError::UnknownPage),
    }
}

fn committed_page(
    store: &TsBurnSaveStore,
    page_number: u8,
) -> Result<&TsCommittedPage, TsBurnSaveError> {
    match page_number {
        1 => Ok(&store.committed_fuel),
        2 => Ok(&store.committed_ignition),
        3 => Ok(&store.committed_angles),
        _ => Err(TsBurnSaveError::UnknownPage),
    }
}

fn committed_page_mut(
    store: &mut TsBurnSaveStore,
    page_number: u8,
) -> Result<&mut TsCommittedPage, TsBurnSaveError> {
    match page_number {
        1 => Ok(&mut store.committed_fuel),
        2 => Ok(&mut store.committed_ignition),
        3 => Ok(&mut store.committed_angles),
        _ => Err(TsBurnSaveError::UnknownPage),
    }
}

pub const fn encode_outpc(frame: OutpcFrame) -> [u8; TS_OUTPC_PAGE_BYTES] {
    let mut out = [0u8; TS_OUTPC_PAGE_BYTES];

    let rpm = frame.rpm.to_le_bytes();
    out[0] = rpm[0];
    out[1] = rpm[1];

    let map = frame.map_kpa10.to_le_bytes();
    out[2] = map[0];
    out[3] = map[1];

    let tps = frame.tps_x100.to_le_bytes();
    out[4] = tps[0];
    out[5] = tps[1];

    let clt = frame.clt_c10.to_le_bytes();
    out[6] = clt[0];
    out[7] = clt[1];

    let iat = frame.iat_c10.to_le_bytes();
    out[8] = iat[0];
    out[9] = iat[1];

    let pw = frame.pw_corr_us.to_le_bytes();
    out[10] = pw[0];
    out[11] = pw[1];

    let adv = frame.advance_deg10.to_le_bytes();
    out[12] = adv[0];
    out[13] = adv[1];

    out[14] = frame.sync_state_code;
    out[15] = frame.cut_reason_code;

    let status = frame.status_flags.to_le_bytes();
    out[16] = status[0];
    out[17] = status[1];
    out[18] = status[2];
    out[19] = status[3];

    out
}

pub fn decode_outpc(bytes: &[u8]) -> Result<OutpcFrame, OutpcCodecError> {
    if bytes.len() != TS_OUTPC_PAGE_BYTES {
        return Err(OutpcCodecError::InvalidLength);
    }

    Ok(OutpcFrame {
        rpm: u16::from_le_bytes([bytes[0], bytes[1]]),
        map_kpa10: u16::from_le_bytes([bytes[2], bytes[3]]),
        tps_x100: u16::from_le_bytes([bytes[4], bytes[5]]),
        clt_c10: i16::from_le_bytes([bytes[6], bytes[7]]),
        iat_c10: i16::from_le_bytes([bytes[8], bytes[9]]),
        pw_corr_us: u16::from_le_bytes([bytes[10], bytes[11]]),
        advance_deg10: i16::from_le_bytes([bytes[12], bytes[13]]),
        sync_state_code: bytes[14],
        cut_reason_code: bytes[15],
        status_flags: u32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]),
    })
}

pub fn ts_diag_log_push(ring: &mut TsDiagLogRing, entry: TsDiagLogEntry) {
    let head_idx = ring.head as usize;
    ring.entries[head_idx] = entry;

    let next_head = ((head_idx + 1) % TS_DIAG_LOG_CAPACITY) as u8;
    if ring.len < TS_DIAG_LOG_CAPACITY as u8 {
        ring.len += 1;
    } else {
        ring.tail = ((ring.tail as usize + 1) % TS_DIAG_LOG_CAPACITY) as u8;
    }
    ring.head = next_head;
}

pub fn ts_diag_log_pop_oldest(ring: &mut TsDiagLogRing) -> Option<TsDiagLogEntry> {
    if ring.len == 0 {
        return None;
    }

    let tail_idx = ring.tail as usize;
    let entry = ring.entries[tail_idx];
    ring.tail = ((tail_idx + 1) % TS_DIAG_LOG_CAPACITY) as u8;
    ring.len -= 1;
    Some(entry)
}

pub fn encode_ts_diag_log_oldest_first(ring: &TsDiagLogRing) -> TsDiagLogEncoded {
    let mut out = TsDiagLogEncoded::default();
    let len = core::cmp::min(ring.len as usize, TS_DIAG_LOG_CAPACITY);
    out.len = (len * TS_DIAG_LOG_ENTRY_BYTES) as u16;

    let mut i = 0usize;
    while i < len {
        let ring_idx = ((ring.tail as usize) + i) % TS_DIAG_LOG_CAPACITY;
        let offset = i * TS_DIAG_LOG_ENTRY_BYTES;
        let entry = ring.entries[ring_idx];

        let ts = entry.timestamp_us.to_le_bytes();
        out.bytes[offset] = ts[0];
        out.bytes[offset + 1] = ts[1];
        out.bytes[offset + 2] = ts[2];
        out.bytes[offset + 3] = ts[3];

        let code = entry.code.to_le_bytes();
        out.bytes[offset + 4] = code[0];
        out.bytes[offset + 5] = code[1];

        out.bytes[offset + 6] = entry.source;

        let context = entry.context.to_le_bytes();
        out.bytes[offset + 7] = context[0];
        out.bytes[offset + 8] = context[1];
        i += 1;
    }

    out
}

pub fn ts_dispatch_step(frame: &[u8]) -> TsDispatchResult<'_> {
    let mut states = [TsDispatchState::Idle; 6];
    states[0] = TsDispatchState::Idle;
    states[1] = TsDispatchState::RxFrame;
    states[2] = TsDispatchState::Decode;

    let decoded = match decode_ts_frame(frame) {
        Ok(decoded) => decoded,
        Err(err) => {
            states[3] = TsDispatchState::ErrorReply;
            states[4] = TsDispatchState::Idle;
            return TsDispatchResult {
                state_len: 5,
                states,
                effect: Err(TsDispatchError::Decode(err)),
            };
        }
    };

    states[3] = TsDispatchState::Execute;

    let effect = match decode_command(decoded.command_id, decoded.payload) {
        Ok(effect) => {
            states[4] = TsDispatchState::EncodeReply;
            states[5] = TsDispatchState::Idle;
            TsDispatchResult {
                state_len: 6,
                states,
                effect: Ok(effect),
            }
        }
        Err(err) => {
            states[4] = TsDispatchState::ErrorReply;
            states[5] = TsDispatchState::Idle;
            TsDispatchResult {
                state_len: 6,
                states,
                effect: Err(err),
            }
        }
    };

    effect
}

fn decode_ts_frame(frame: &[u8]) -> Result<DecodedFrame<'_>, TsDecodeError> {
    if frame.len() < 5 {
        return Err(TsDecodeError::FrameTooShort);
    }

    let magic = u16::from_le_bytes([frame[0], frame[1]]);
    if magic != TS_PROTO_MAGIC {
        return Err(TsDecodeError::BadMagic);
    }

    let len = u16::from_le_bytes([frame[2], frame[3]]);
    if len < 3 {
        return Err(TsDecodeError::InvalidLength);
    }

    let total = 4usize + len as usize;
    if frame.len() < total {
        return Err(TsDecodeError::Truncated);
    }

    let command_id = frame[4];
    let payload_len = len as usize - 3;
    let payload_end = 5 + payload_len;
    let payload = &frame[5..payload_end];
    let received_crc = u16::from_le_bytes([frame[payload_end], frame[payload_end + 1]]);
    let computed_crc = crc16_ccitt(&frame[4..payload_end]);

    if received_crc != computed_crc {
        return Err(TsDecodeError::CrcMismatch);
    }

    Ok(DecodedFrame {
        command_id,
        payload,
    })
}

fn decode_command(command_id: u8, payload: &[u8]) -> Result<TsEffect<'_>, TsDispatchError> {
    match command_id {
        CMD_READ_PAGE => {
            expect_payload_len(command_id, payload.len(), 5)?;
            let page_number = payload[0];
            let offset = u16::from_le_bytes([payload[1], payload[2]]);
            let len = u16::from_le_bytes([payload[3], payload[4]]);
            validate_page_range(command_id, page_number, offset, len)?;
            Ok(TsEffect::ReadPage {
                page_number,
                offset,
                len,
            })
        }
        CMD_WRITE_PAGE => {
            expect_payload_min_len(command_id, payload.len(), 3)?;
            let page_number = payload[0];
            let offset = u16::from_le_bytes([payload[1], payload[2]]);
            let len = (payload.len() - 3) as u16;
            validate_page_range(command_id, page_number, offset, len)?;
            Ok(TsEffect::WritePage {
                page_number,
                offset,
                bytes: &payload[3..],
            })
        }
        CMD_BURN => {
            expect_payload_len(command_id, payload.len(), 1)?;
            validate_persist_page(command_id, payload[0])?;
            Ok(TsEffect::Burn {
                page_number: payload[0],
            })
        }
        CMD_GET_OUTPC => {
            expect_payload_len(command_id, payload.len(), 0)?;
            Ok(TsEffect::GetOutpc)
        }
        CMD_GET_SIGNATURE => {
            expect_payload_len(command_id, payload.len(), 1)?;
            page_meta(payload[0]).map_err(|_| {
                TsDispatchError::CommandDecode(TsCommandDecodeError::UnknownPage {
                    command_id,
                    page_number: payload[0],
                })
            })?;
            Ok(TsEffect::GetSignature {
                page_number: payload[0],
            })
        }
        _ => Err(TsDispatchError::UnknownCommand(command_id)),
    }
}

fn expect_payload_len(command_id: u8, actual: usize, expected: u16) -> Result<(), TsDispatchError> {
    if actual == expected as usize {
        Ok(())
    } else {
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::WrongPayloadLength {
                command_id,
                expected,
                actual: actual as u16,
            },
        ))
    }
}

fn expect_payload_min_len(
    command_id: u8,
    actual: usize,
    minimum: u16,
) -> Result<(), TsDispatchError> {
    if actual >= minimum as usize {
        Ok(())
    } else {
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::PayloadTooShort {
                command_id,
                minimum,
                actual: actual as u16,
            },
        ))
    }
}

fn validate_persist_page(command_id: u8, page_number: u8) -> Result<TsPageMeta, TsDispatchError> {
    let meta = page_meta(page_number).map_err(|_| {
        TsDispatchError::CommandDecode(TsCommandDecodeError::UnknownPage {
            command_id,
            page_number,
        })
    })?;
    match meta.page_id {
        TsPageId::Fuel | TsPageId::Ignition | TsPageId::Angles => Ok(meta),
        TsPageId::Outpc => Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::UnknownPage {
                command_id,
                page_number,
            },
        )),
    }
}

fn validate_page_range(
    command_id: u8,
    page_number: u8,
    offset: u16,
    len: u16,
) -> Result<(), TsDispatchError> {
    let meta = validate_persist_page(command_id, page_number)?;
    if (offset as u32) + (len as u32) <= meta.payload_size as u32 {
        Ok(())
    } else {
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::PageRangeOutOfBounds {
                command_id,
                page_number,
                offset,
                len,
                page_len: meta.payload_size,
            },
        ))
    }
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
    }
    crc
}

#[cfg(test)]
#[path = "ts_spec_tests.rs"]
mod tests;
