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
    WrongPayloadLength,
    PayloadTooShort,
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
            if payload.len() != 5 {
                return Err(TsDispatchError::CommandDecode(
                    TsCommandDecodeError::WrongPayloadLength,
                ));
            }
            Ok(TsEffect::ReadPage {
                page_number: payload[0],
                offset: u16::from_le_bytes([payload[1], payload[2]]),
                len: u16::from_le_bytes([payload[3], payload[4]]),
            })
        }
        CMD_WRITE_PAGE => {
            if payload.len() < 3 {
                return Err(TsDispatchError::CommandDecode(
                    TsCommandDecodeError::PayloadTooShort,
                ));
            }
            Ok(TsEffect::WritePage {
                page_number: payload[0],
                offset: u16::from_le_bytes([payload[1], payload[2]]),
                bytes: &payload[3..],
            })
        }
        CMD_BURN => {
            if payload.len() != 1 {
                return Err(TsDispatchError::CommandDecode(
                    TsCommandDecodeError::WrongPayloadLength,
                ));
            }
            Ok(TsEffect::Burn {
                page_number: payload[0],
            })
        }
        CMD_GET_OUTPC => {
            if !payload.is_empty() {
                return Err(TsDispatchError::CommandDecode(
                    TsCommandDecodeError::WrongPayloadLength,
                ));
            }
            Ok(TsEffect::GetOutpc)
        }
        CMD_GET_SIGNATURE => {
            if payload.len() != 1 {
                return Err(TsDispatchError::CommandDecode(
                    TsCommandDecodeError::WrongPayloadLength,
                ));
            }
            Ok(TsEffect::GetSignature {
                page_number: payload[0],
            })
        }
        _ => Err(TsDispatchError::UnknownCommand(command_id)),
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
mod tests {
    use super::*;

    #[allow(clippy::manual_unwrap_or_default)]
    fn must_ok<T: Default, E>(result: Result<T, E>) -> T {
        assert!(result.is_ok(), "expected Ok(..)");
        match result {
            Ok(value) => value,
            Err(_) => T::default(),
        }
    }

    fn build_frame(command_id: u8, payload: &[u8]) -> [u8; 80] {
        let mut out = [0u8; 80];
        out[0..2].copy_from_slice(&TS_PROTO_MAGIC.to_le_bytes());

        let len = (1 + payload.len() + 2) as u16;
        out[2..4].copy_from_slice(&len.to_le_bytes());

        out[4] = command_id;
        if !payload.is_empty() {
            out[5..5 + payload.len()].copy_from_slice(payload);
        }

        let crc = crc16_ccitt(&out[4..5 + payload.len()]);
        let crc_off = 5 + payload.len();
        out[crc_off..crc_off + 2].copy_from_slice(&crc.to_le_bytes());

        out
    }

    #[test]
    fn page_meta_returns_frozen_known_pages() {
        assert_eq!(
            page_meta(1),
            Ok(TsPageMeta {
                page_id: TsPageId::Fuel,
                signature: 0x46554C31,
                payload_size: 512,
            })
        );
        assert_eq!(
            page_meta(2),
            Ok(TsPageMeta {
                page_id: TsPageId::Ignition,
                signature: 0x49474E31,
                payload_size: 512,
            })
        );
        assert_eq!(
            page_meta(3),
            Ok(TsPageMeta {
                page_id: TsPageId::Angles,
                signature: 0x414E4731,
                payload_size: 68,
            })
        );
        assert_eq!(
            page_meta(4),
            Ok(TsPageMeta {
                page_id: TsPageId::Outpc,
                signature: 0x4F555431,
                payload_size: 64,
            })
        );
    }

    #[test]
    fn page_meta_unknown_for_all_other_pages() {
        let mut page = 0u16;
        while page <= u8::MAX as u16 {
            let page_u8 = page as u8;
            let result = page_meta(page_u8);
            if (1..=4).contains(&page_u8) {
                assert!(result.is_ok());
            } else {
                assert_eq!(result, Err(TsPageMetaError::UnknownPage));
            }
            page += 1;
        }
    }

    #[test]
    fn encode_outpc_matches_frozen_layout_offsets_and_zeroes_reserved_tail() {
        let frame = OutpcFrame {
            rpm: 2500,
            map_kpa10: 980,
            tps_x100: 4500,
            clt_c10: 865,
            iat_c10: -120,
            pw_corr_us: 4200,
            advance_deg10: -35,
            sync_state_code: 1,
            cut_reason_code: 4,
            status_flags: 0xA5A5_55AA,
        };
        let bytes = encode_outpc(frame);

        assert_eq!(&bytes[0..2], &2500u16.to_le_bytes());
        assert_eq!(&bytes[2..4], &980u16.to_le_bytes());
        assert_eq!(&bytes[4..6], &4500u16.to_le_bytes());
        assert_eq!(&bytes[6..8], &865i16.to_le_bytes());
        assert_eq!(&bytes[8..10], &(-120i16).to_le_bytes());
        assert_eq!(&bytes[10..12], &4200u16.to_le_bytes());
        assert_eq!(&bytes[12..14], &(-35i16).to_le_bytes());
        assert_eq!(bytes[14], 1);
        assert_eq!(bytes[15], 4);
        assert_eq!(&bytes[16..20], &0xA5A5_55AAu32.to_le_bytes());
        for byte in bytes.iter().skip(20) {
            assert_eq!(*byte, 0);
        }
    }

    #[test]
    fn decode_outpc_roundtrip_recovers_fields() {
        let frame = OutpcFrame {
            rpm: 1024,
            map_kpa10: 1100,
            tps_x100: 1234,
            clt_c10: -340,
            iat_c10: 560,
            pw_corr_us: u16::MAX,
            advance_deg10: 125,
            sync_state_code: 0xFE,
            cut_reason_code: 0x07,
            status_flags: 0xDEAD_BEEF,
        };

        let bytes = encode_outpc(frame);
        let decoded = must_ok(decode_outpc(&bytes));
        assert_eq!(decoded, frame);
    }

    #[test]
    fn decode_outpc_rejects_non_64_byte_payloads() {
        assert_eq!(decode_outpc(&[]), Err(OutpcCodecError::InvalidLength));
        assert_eq!(
            decode_outpc(&[0u8; TS_OUTPC_PAGE_BYTES - 1]),
            Err(OutpcCodecError::InvalidLength)
        );
        assert_eq!(
            decode_outpc(&[0u8; TS_OUTPC_PAGE_BYTES + 1]),
            Err(OutpcCodecError::InvalidLength)
        );
    }

    #[test]
    fn ts_dispatch_maps_read_page_to_effect() {
        let payload = [3u8, 0x34, 0x12, 0x78, 0x56];
        let frame = build_frame(CMD_READ_PAGE, &payload);
        let total = 4 + (1 + payload.len() + 2);

        let result = ts_dispatch_step(&frame[..total]);
        assert_eq!(result.state_len, 6);
        assert_eq!(
            &result.states[..result.state_len as usize],
            &[
                TsDispatchState::Idle,
                TsDispatchState::RxFrame,
                TsDispatchState::Decode,
                TsDispatchState::Execute,
                TsDispatchState::EncodeReply,
                TsDispatchState::Idle,
            ]
        );
        assert_eq!(
            result.effect,
            Ok(TsEffect::ReadPage {
                page_number: 3,
                offset: 0x1234,
                len: 0x5678,
            })
        );
    }

    #[test]
    fn ts_dispatch_returns_unknown_command_without_panic() {
        let frame = build_frame(0x7F, &[]);
        let total = 4 + (1 + 2);

        let result = ts_dispatch_step(&frame[..total]);
        assert_eq!(result.state_len, 6);
        assert_eq!(result.effect, Err(TsDispatchError::UnknownCommand(0x7F)));
        assert_eq!(
            &result.states[..result.state_len as usize],
            &[
                TsDispatchState::Idle,
                TsDispatchState::RxFrame,
                TsDispatchState::Decode,
                TsDispatchState::Execute,
                TsDispatchState::ErrorReply,
                TsDispatchState::Idle,
            ]
        );
    }

    #[test]
    fn ts_dispatch_reports_malformed_crc_as_typed_decode_error() {
        let mut frame = build_frame(CMD_GET_OUTPC, &[]);
        let total = 4 + (1 + 2);
        frame[total - 1] ^= 0xFF;

        let result = ts_dispatch_step(&frame[..total]);
        assert_eq!(result.state_len, 5);
        assert_eq!(
            result.effect,
            Err(TsDispatchError::Decode(TsDecodeError::CrcMismatch))
        );
        assert_eq!(
            &result.states[..result.state_len as usize],
            &[
                TsDispatchState::Idle,
                TsDispatchState::RxFrame,
                TsDispatchState::Decode,
                TsDispatchState::ErrorReply,
                TsDispatchState::Idle,
            ]
        );
    }

    #[test]
    fn ts_dispatch_decode_stage_is_always_entered() {
        let good_decode_state = {
            let frame = build_frame(CMD_GET_OUTPC, &[]);
            let total = 4 + (1 + 2);
            let result = ts_dispatch_step(&frame[..total]);
            result.states[2]
        };
        let bad = ts_dispatch_step(&[]);

        assert_eq!(good_decode_state, TsDispatchState::Decode);
        assert_eq!(bad.states[2], TsDispatchState::Decode);
    }

    #[test]
    fn burn_commits_crc_valid_record_and_rejects_when_engine_running() {
        let mut store = TsBurnSaveStore::default();
        assert!(write_page(&mut store, 1, 0, &[0xAA, 0xBB]).is_ok());
        assert_eq!(
            burn_page(&mut store, 1, true),
            Err(TsBurnSaveError::EngineRunning)
        );
        assert_eq!(store.committed_fuel.len, 0);

        assert!(burn_page(&mut store, 1, false).is_ok());
        let committed = must_ok(committed_page_record(&store, 1));
        assert!(persist_decode(&committed.bytes[..committed.len as usize]).is_ok());
    }

    #[test]
    fn burn_crc_mismatch_rolls_back_and_keeps_previous_commit() {
        let mut store = TsBurnSaveStore::default();
        assert!(write_page(&mut store, 2, 0, &[0x11, 0x22]).is_ok());
        assert!(burn_page(&mut store, 2, false).is_ok());
        let before = store.committed_ignition;

        assert_eq!(
            burn_page_internal(&mut store, 2, false, true),
            Err(TsBurnSaveError::CrcMismatch)
        );
        assert_eq!(store.committed_ignition, before);
    }

    #[test]
    fn save_all_burns_in_frozen_page_order() {
        let mut store = TsBurnSaveStore::default();
        assert!(write_page(&mut store, 1, 0, &[1]).is_ok());
        assert!(write_page(&mut store, 2, 0, &[2]).is_ok());
        assert!(write_page(&mut store, 3, 0, &[3]).is_ok());

        assert!(save_all(&mut store, false).is_ok());
        let fuel = must_ok(committed_page_record(&store, 1));
        let ign = must_ok(committed_page_record(&store, 2));
        let ang = must_ok(committed_page_record(&store, 3));

        assert_eq!(fuel.bytes[6], 1);
        assert_eq!(ign.bytes[6], 2);
        assert_eq!(ang.bytes[6], 3);
    }

    #[test]
    fn diag_log_pop_empty_returns_none_without_mutating_indexes() {
        let mut ring = TsDiagLogRing::default();
        assert_eq!(ts_diag_log_pop_oldest(&mut ring), None);
        assert_eq!(ring.len, 0);
        assert_eq!(ring.head, 0);
        assert_eq!(ring.tail, 0);
    }

    #[test]
    fn diag_log_encode_orders_oldest_to_newest_without_wrap() {
        let mut ring = TsDiagLogRing::default();
        ts_diag_log_push(
            &mut ring,
            TsDiagLogEntry {
                timestamp_us: 10,
                code: 100,
                source: 1,
                context: 1000,
            },
        );
        ts_diag_log_push(
            &mut ring,
            TsDiagLogEntry {
                timestamp_us: 20,
                code: 200,
                source: 2,
                context: 2000,
            },
        );

        let encoded = encode_ts_diag_log_oldest_first(&ring);
        assert_eq!(encoded.len as usize, 2 * TS_DIAG_LOG_ENTRY_BYTES);
        assert_eq!(&encoded.bytes[0..4], &10u32.to_le_bytes());
        assert_eq!(&encoded.bytes[4..6], &100u16.to_le_bytes());
        assert_eq!(encoded.bytes[6], 1);
        assert_eq!(&encoded.bytes[7..9], &1000u16.to_le_bytes());

        assert_eq!(&encoded.bytes[9..13], &20u32.to_le_bytes());
        assert_eq!(&encoded.bytes[13..15], &200u16.to_le_bytes());
        assert_eq!(encoded.bytes[15], 2);
        assert_eq!(&encoded.bytes[16..18], &2000u16.to_le_bytes());
    }

    #[test]
    fn diag_log_wrap_overwrites_oldest_and_encoder_starts_with_oldest_retained() {
        let mut ring = TsDiagLogRing::default();
        let mut i = 0u16;
        while i < 70 {
            ts_diag_log_push(
                &mut ring,
                TsDiagLogEntry {
                    timestamp_us: i as u32,
                    code: i,
                    source: (i & 0xFF) as u8,
                    context: i + 1,
                },
            );
            i += 1;
        }

        assert_eq!(ring.len as usize, TS_DIAG_LOG_CAPACITY);
        let encoded = encode_ts_diag_log_oldest_first(&ring);
        assert_eq!(encoded.len as usize, TS_DIAG_LOG_MAX_ENCODED_BYTES);

        let mut idx = 0u16;
        while idx < TS_DIAG_LOG_CAPACITY as u16 {
            let expected = idx + 6;
            let offset = idx as usize * TS_DIAG_LOG_ENTRY_BYTES;
            assert_eq!(
                u32::from_le_bytes([
                    encoded.bytes[offset],
                    encoded.bytes[offset + 1],
                    encoded.bytes[offset + 2],
                    encoded.bytes[offset + 3]
                ]),
                expected as u32
            );
            assert_eq!(
                u16::from_le_bytes([encoded.bytes[offset + 4], encoded.bytes[offset + 5]]),
                expected
            );
            assert_eq!(encoded.bytes[offset + 6], (expected & 0xFF) as u8);
            assert_eq!(
                u16::from_le_bytes([encoded.bytes[offset + 7], encoded.bytes[offset + 8]]),
                expected + 1
            );
            idx += 1;
        }
    }
}
