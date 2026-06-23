#![cfg(any(feature = "flash-kv", test))]
#![allow(dead_code)]

#[cfg(feature = "transport-can")]
use crate::identity_provisioning::{
    Obd2IdentityProvisioningCommandAudit, Obd2IdentityProvisioningField,
    Obd2IdentityProvisioningKeyAudit, Obd2IdentityProvisioningLengths,
    Obd2IdentityProvisioningStatus,
};
#[cfg(feature = "flash-kv")]
use core::sync::atomic::{AtomicU32, AtomicU8, Ordering};
#[cfg(any(feature = "flash-kv", feature = "transport-can"))]
use ecu_calibration::KvError;
#[cfg(feature = "flash-kv")]
use ecu_calibration::KvStore;
use ecu_target_common::kv::ab::StoreIntegrityStatus;
use ecu_target_common::kv::layout::{
    ANGLES_PAGE_LEN, FUEL_PAGE_LEN, IGN_PAGE_LEN, PAGE_HEADER_LEN,
};
#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
use ecu_target_common::transport_service::{
    apply_retained_history_page_update,
    install_optional_obd2_retained_history_snapshot_halfwords_with,
    persist_retained_history_flash_rewrite, prepare_retained_history_preserved_page_rewrite,
    prepare_retained_history_snapshot_rewrite, read_obd2_retained_history_snapshot_with,
    Obd2RetainedHistoryPagesMut,
};
#[cfg(all(test, feature = "transport-can"))]
use ecu_target_common::transport_service::{
    decode_obd2_retained_history_sidecar_at, install_obd2_retained_history_snapshot_halfwords_with,
};
#[cfg(feature = "transport-can")]
use ecu_target_common::transport_service::{
    Obd2ProvisionedIdentityRecord, Obd2RetainedDiagnosticHistorySnapshot,
    OBD2_RETAINED_HISTORY_SIDECAR_BYTES,
};
#[cfg(feature = "flash-kv")]
use stm32f4xx_hal::pac;

#[cfg(feature = "flash-kv")]
const FLASH_SR_ERROR_MASK: u32 = (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) | (1 << 7);

#[cfg(feature = "flash-kv")]
const FLASH_SR_CLEAR_MASK: u32 = FLASH_SR_ERROR_MASK | 1;

#[cfg(feature = "flash-kv")]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum FlashWritePhase {
    None,
    Preflight,
    Erase,
    Program,
}

#[cfg(feature = "flash-kv")]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct FlashWriteFaultSnapshot {
    pub(crate) phase: FlashWritePhase,
    pub(crate) sr_bits: u32,
}

#[cfg(feature = "flash-kv")]
static LAST_FLASH_WRITE_FAULT_PHASE: AtomicU8 = AtomicU8::new(0);

#[cfg(feature = "flash-kv")]
static LAST_FLASH_WRITE_FAULT_SR_BITS: AtomicU32 = AtomicU32::new(0);

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum FlashPageKey {
    Fuel,
    Ign,
    Angles,
}

impl FlashPageKey {
    pub(crate) const fn from_key(key: &[u8]) -> Option<Self> {
        if matches_key(key, b"fuel") {
            Some(Self::Fuel)
        } else if matches_key(key, b"ign") {
            Some(Self::Ign)
        } else if matches_key(key, b"angles") {
            Some(Self::Angles)
        } else {
            None
        }
    }
}

const fn matches_key(lhs: &[u8], rhs: &[u8]) -> bool {
    if lhs.len() != rhs.len() {
        return false;
    }
    let mut i = 0;
    while i < lhs.len() {
        if lhs[i] != rhs[i] {
            return false;
        }
        i += 1;
    }
    true
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct FlashPageLayout {
    pub(crate) offset: usize,
    pub(crate) len: usize,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum FlashLayoutError {
    HeaderTooSmall,
    PageOverflow,
    SectorAlias,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct FlashLayout {
    pub(crate) base_a: u32,
    pub(crate) base_b: u32,
    pub(crate) sector_a: u8,
    pub(crate) sector_b: u8,
    pub(crate) header_len: usize,
    pub(crate) fuel: FlashPageLayout,
    pub(crate) ign: FlashPageLayout,
    pub(crate) angles: FlashPageLayout,
}

#[cfg(feature = "flash-kv")]
#[derive(Clone)]
struct FlashPageImages {
    fuel: [u8; FUEL_PAGE_LEN],
    ign: [u8; IGN_PAGE_LEN],
    angles: [u8; ANGLES_PAGE_LEN],
}

#[cfg(feature = "flash-kv")]
impl Obd2RetainedHistoryPagesMut for FlashPageImages {
    fn fuel_mut(&mut self) -> &mut [u8] {
        &mut self.fuel
    }

    fn ign_mut(&mut self) -> &mut [u8] {
        &mut self.ign
    }

    fn angles_mut(&mut self) -> &mut [u8] {
        &mut self.angles
    }
}

impl FlashLayout {
    pub(crate) const MAGIC: u32 = 0x3250_4B56;
    pub(crate) const VALID_COMMITTED: u16 = 0x0000;
    pub(crate) const INVALID_ERASED: u16 = 0xFFFF;
    pub(crate) const SECTOR_BYTES: usize = 128 * 1024;

    pub(crate) const STM32F405_TS_KV: Result<Self, FlashLayoutError> = Self::new(
        0x080C_0000,
        0x080E_0000,
        10,
        11,
        PAGE_HEADER_LEN,
        FUEL_PAGE_LEN,
        IGN_PAGE_LEN,
        ANGLES_PAGE_LEN,
    );

    pub(crate) const fn new(
        base_a: u32,
        base_b: u32,
        sector_a: u8,
        sector_b: u8,
        header_len: usize,
        fuel_len: usize,
        ign_len: usize,
        angles_len: usize,
    ) -> Result<Self, FlashLayoutError> {
        if header_len < 22 {
            return Err(FlashLayoutError::HeaderTooSmall);
        }
        if base_a == base_b || sector_a == sector_b {
            return Err(FlashLayoutError::SectorAlias);
        }
        let fuel = FlashPageLayout {
            offset: header_len,
            len: fuel_len,
        };
        let ign = FlashPageLayout {
            offset: header_len + fuel_len,
            len: ign_len,
        };
        let angles = FlashPageLayout {
            offset: header_len + fuel_len + ign_len,
            len: angles_len,
        };
        if angles.offset + angles.len > Self::SECTOR_BYTES {
            return Err(FlashLayoutError::PageOverflow);
        }
        Ok(Self {
            base_a,
            base_b,
            sector_a,
            sector_b,
            header_len,
            fuel,
            ign,
            angles,
        })
    }

    pub(crate) const fn page(self, key: FlashPageKey) -> FlashPageLayout {
        match key {
            FlashPageKey::Fuel => self.fuel,
            FlashPageKey::Ign => self.ign,
            FlashPageKey::Angles => self.angles,
        }
    }
}

pub(crate) const STM32F405_TS_KV_LAYOUT: FlashLayout = match FlashLayout::STM32F405_TS_KV {
    Ok(layout) => layout,
    Err(_) => panic!("invalid STM32F4 TS flash KV layout"),
};

#[cfg(feature = "transport-can")]
const OBD2_SNAPSHOT_OFFSET: usize =
    STM32F405_TS_KV_LAYOUT.angles.offset + STM32F405_TS_KV_LAYOUT.angles.len;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_RECORD_OFFSET: usize =
    OBD2_SNAPSHOT_OFFSET + OBD2_RETAINED_HISTORY_SIDECAR_BYTES;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_RECORD_BYTES: usize = 52;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_RECORD_MAGIC: u32 = 0x3249_424F;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_RECORD_VERSION: u16 = 1;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_COMMAND_AUDIT_OFFSET: usize =
    OBD2_IDENTITY_RECORD_OFFSET + OBD2_IDENTITY_RECORD_BYTES;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_COMMAND_AUDIT_BYTES: usize = 20;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_COMMAND_AUDIT_MAGIC: u32 = 0x4149_424F;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_COMMAND_AUDIT_VERSION: u16 = 1;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_PROVISIONING_KEY_OFFSET: usize =
    OBD2_IDENTITY_COMMAND_AUDIT_OFFSET + OBD2_IDENTITY_COMMAND_AUDIT_BYTES;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_PROVISIONING_KEY_BYTES: usize = 50;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_PROVISIONING_KEY_MAGIC: u32 = 0x4B49_424F;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_PROVISIONING_KEY_VERSION: u16 = 1;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_PROVISIONING_KEY_ACTIVE: u8 = 0x01;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_PROVISIONING_KEY_REVOKED: u8 = 0x02;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_KEY_AUDIT_OFFSET: usize =
    OBD2_IDENTITY_PROVISIONING_KEY_OFFSET + OBD2_IDENTITY_PROVISIONING_KEY_BYTES;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_KEY_AUDIT_BYTES: usize = 22;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_KEY_AUDIT_MAGIC: u32 = 0x4C49_424F;
#[cfg(feature = "transport-can")]
const OBD2_IDENTITY_KEY_AUDIT_VERSION: u16 = 1;

#[cfg(feature = "transport-can")]
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) struct Obd2IdentityProvisioningKeyRecord {
    pub(crate) key: [u8; 32],
    pub(crate) generation: u32,
    pub(crate) last_arm_nonce: u32,
    pub(crate) revoked: bool,
}

#[cfg(feature = "transport-can")]
impl Obd2IdentityProvisioningKeyRecord {
    pub(crate) const fn active(key: [u8; 32], generation: u32) -> Self {
        Self {
            key,
            generation,
            last_arm_nonce: 0,
            revoked: false,
        }
    }

    pub(crate) const fn revoked(key: [u8; 32], generation: u32) -> Self {
        Self {
            key,
            generation,
            last_arm_nonce: 0,
            revoked: true,
        }
    }

    pub(crate) const fn with_last_arm_nonce(mut self, last_arm_nonce: u32) -> Self {
        self.last_arm_nonce = last_arm_nonce;
        self
    }

    pub(crate) const fn active_key(self) -> Option<[u8; 32]> {
        if self.revoked {
            None
        } else {
            Some(self.key)
        }
    }
}

#[cfg(feature = "transport-can")]
const fn validate_obd2_snapshot_layout() -> Result<(), FlashLayoutError> {
    if OBD2_SNAPSHOT_OFFSET + OBD2_RETAINED_HISTORY_SIDECAR_BYTES > FlashLayout::SECTOR_BYTES {
        return Err(FlashLayoutError::PageOverflow);
    }
    if OBD2_IDENTITY_RECORD_OFFSET + OBD2_IDENTITY_RECORD_BYTES > FlashLayout::SECTOR_BYTES {
        return Err(FlashLayoutError::PageOverflow);
    }
    if OBD2_IDENTITY_COMMAND_AUDIT_OFFSET + OBD2_IDENTITY_COMMAND_AUDIT_BYTES
        > FlashLayout::SECTOR_BYTES
    {
        return Err(FlashLayoutError::PageOverflow);
    }
    if OBD2_IDENTITY_PROVISIONING_KEY_OFFSET + OBD2_IDENTITY_PROVISIONING_KEY_BYTES
        > FlashLayout::SECTOR_BYTES
    {
        return Err(FlashLayoutError::PageOverflow);
    }
    if OBD2_IDENTITY_KEY_AUDIT_OFFSET + OBD2_IDENTITY_KEY_AUDIT_BYTES > FlashLayout::SECTOR_BYTES {
        return Err(FlashLayoutError::PageOverflow);
    }
    Ok(())
}

#[cfg(feature = "transport-can")]
const _: () = match validate_obd2_snapshot_layout() {
    Ok(()) => (),
    Err(_) => panic!("invalid STM32F4 OBD-II snapshot flash layout"),
};

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub(crate) enum ActiveSector {
    A,
    B,
}

#[derive(Copy, Clone)]
pub(crate) struct KvHeader {
    pub(crate) ok: bool,
    pub(crate) seq: u16,
    pub(crate) fuel_len: u16,
    pub(crate) ign_len: u16,
    pub(crate) angles_len: u16,
    pub(crate) fuel_crc: u16,
    pub(crate) ign_crc: u16,
    pub(crate) angles_crc: u16,
}

pub(crate) fn newest_sector(a: KvHeader, b: KvHeader) -> Option<ActiveSector> {
    match (a.ok, b.ok) {
        (true, true) => {
            if b.seq.wrapping_sub(a.seq) < 0x8000 {
                Some(ActiveSector::B)
            } else {
                Some(ActiveSector::A)
            }
        }
        (true, false) => Some(ActiveSector::A),
        (false, true) => Some(ActiveSector::B),
        (false, false) => None,
    }
}

pub(crate) fn target_sector_for_next_write(a: KvHeader, b: KvHeader) -> ActiveSector {
    match newest_sector(a, b) {
        Some(ActiveSector::A) => ActiveSector::B,
        Some(ActiveSector::B) => ActiveSector::A,
        None => ActiveSector::A,
    }
}

pub(crate) fn header_page_len(hdr: KvHeader, key: FlashPageKey) -> u16 {
    match key {
        FlashPageKey::Fuel => hdr.fuel_len,
        FlashPageKey::Ign => hdr.ign_len,
        FlashPageKey::Angles => hdr.angles_len,
    }
}

fn header_page_crc(hdr: KvHeader, key: FlashPageKey) -> u16 {
    match key {
        FlashPageKey::Fuel => hdr.fuel_crc,
        FlashPageKey::Ign => hdr.ign_crc,
        FlashPageKey::Angles => hdr.angles_crc,
    }
}

pub(crate) fn crc16(data: &[u8]) -> u16 {
    ecu_ts::proto::crc16_ccitt(data)
}

#[cfg(feature = "transport-can")]
pub(crate) fn encode_obd2_identity_record(
    record: Obd2ProvisionedIdentityRecord,
) -> [u8; OBD2_IDENTITY_RECORD_BYTES] {
    let mut out = [0xFFu8; OBD2_IDENTITY_RECORD_BYTES];
    out[0..4].copy_from_slice(&OBD2_IDENTITY_RECORD_MAGIC.to_le_bytes());
    out[4..6].copy_from_slice(&OBD2_IDENTITY_RECORD_VERSION.to_le_bytes());
    out[6] = record.vin_len.min(ecu_transport::CAN_OBD2_VIN_LEN as u8);
    out[7] = record
        .calibration_id_len
        .min(ecu_transport::CAN_OBD2_VIN_LEN as u8);
    out[8] = record.board_build_identity_len.min(6);
    out[9..26].copy_from_slice(&record.vin);
    out[26..43].copy_from_slice(&record.calibration_id);
    out[43..49].copy_from_slice(&record.board_build_identity);
    let crc = crc16(&out[..50]);
    out[50..52].copy_from_slice(&crc.to_le_bytes());
    out
}

#[cfg(feature = "transport-can")]
pub(crate) fn decode_obd2_identity_record(
    bytes: &[u8; OBD2_IDENTITY_RECORD_BYTES],
) -> Option<Obd2ProvisionedIdentityRecord> {
    if bytes.iter().all(|byte| *byte == 0xFF) {
        return None;
    }
    let magic = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if magic != OBD2_IDENTITY_RECORD_MAGIC {
        return None;
    }
    let version = u16::from_le_bytes([bytes[4], bytes[5]]);
    if version != OBD2_IDENTITY_RECORD_VERSION {
        return None;
    }
    let vin_len = bytes[6];
    let calibration_id_len = bytes[7];
    let board_build_identity_len = bytes[8];
    if vin_len > ecu_transport::CAN_OBD2_VIN_LEN as u8
        || calibration_id_len > ecu_transport::CAN_OBD2_VIN_LEN as u8
        || board_build_identity_len > 6
    {
        return None;
    }
    let expected_crc = u16::from_le_bytes([bytes[50], bytes[51]]);
    if crc16(&bytes[..50]) != expected_crc {
        return None;
    }
    let mut record = Obd2ProvisionedIdentityRecord::default();
    record.vin_len = vin_len;
    record.calibration_id_len = calibration_id_len;
    record.board_build_identity_len = board_build_identity_len;
    record.vin.copy_from_slice(&bytes[9..26]);
    record.calibration_id.copy_from_slice(&bytes[26..43]);
    record.board_build_identity.copy_from_slice(&bytes[43..49]);
    Some(record)
}

#[cfg(feature = "transport-can")]
pub(crate) fn read_obd2_identity_record_with(
    read_record: impl FnOnce(&mut [u8; OBD2_IDENTITY_RECORD_BYTES]),
) -> Option<Obd2ProvisionedIdentityRecord> {
    let mut bytes = [0u8; OBD2_IDENTITY_RECORD_BYTES];
    read_record(&mut bytes);
    decode_obd2_identity_record(&bytes)
}

#[cfg(feature = "transport-can")]
pub(crate) fn encode_obd2_identity_command_audit(
    audit: Obd2IdentityProvisioningCommandAudit,
) -> [u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES] {
    let mut out = [0xFFu8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES];
    out[0..4].copy_from_slice(&OBD2_IDENTITY_COMMAND_AUDIT_MAGIC.to_le_bytes());
    out[4..6].copy_from_slice(&OBD2_IDENTITY_COMMAND_AUDIT_VERSION.to_le_bytes());
    out[6..10].copy_from_slice(&audit.request_id.to_le_bytes());
    let mut flags = 0u8;
    if audit.authorized {
        flags |= 0x01;
    }
    if audit.authorization_failed {
        flags |= 0x02;
    }
    if let Some(status) = audit.provisioning_status {
        flags |= 0x04;
        if status.attempted {
            flags |= 0x08;
        }
        if status.accepted {
            flags |= 0x10;
        }
        if status.store_failed {
            flags |= 0x20;
        }
        out[11] = encode_obd2_identity_rejected_field(status.rejected_field);
        if let Some(lengths) = status.normalized_lengths {
            out[12] = lengths.vin;
            out[13] = lengths.calibration_id;
            out[14] = lengths.board_build_identity;
        }
    }
    out[10] = flags;
    let crc = crc16(&out[..18]);
    out[18..20].copy_from_slice(&crc.to_le_bytes());
    out
}

#[cfg(feature = "transport-can")]
pub(crate) fn decode_obd2_identity_command_audit(
    bytes: &[u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES],
) -> Option<Obd2IdentityProvisioningCommandAudit> {
    if bytes.iter().all(|byte| *byte == 0xFF) {
        return None;
    }
    let magic = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if magic != OBD2_IDENTITY_COMMAND_AUDIT_MAGIC {
        return None;
    }
    let version = u16::from_le_bytes([bytes[4], bytes[5]]);
    if version != OBD2_IDENTITY_COMMAND_AUDIT_VERSION {
        return None;
    }
    let expected_crc = u16::from_le_bytes([bytes[18], bytes[19]]);
    if crc16(&bytes[..18]) != expected_crc {
        return None;
    }
    let flags = bytes[10];
    let has_status = flags & 0x04 != 0;
    let provisioning_status = if has_status {
        Some(Obd2IdentityProvisioningStatus {
            attempted: flags & 0x08 != 0,
            accepted: flags & 0x10 != 0,
            rejected_field: decode_obd2_identity_rejected_field(bytes[11])?,
            store_failed: flags & 0x20 != 0,
            normalized_lengths: decode_obd2_identity_audit_lengths(bytes[12], bytes[13], bytes[14]),
        })
    } else {
        None
    };
    Some(Obd2IdentityProvisioningCommandAudit {
        request_id: u32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]),
        authorized: flags & 0x01 != 0,
        authorization_failed: flags & 0x02 != 0,
        provisioning_status,
    })
}

#[cfg(feature = "transport-can")]
pub(crate) fn read_obd2_identity_command_audit_with(
    read_record: impl FnOnce(&mut [u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES]),
) -> Option<Obd2IdentityProvisioningCommandAudit> {
    let mut bytes = [0u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES];
    read_record(&mut bytes);
    decode_obd2_identity_command_audit(&bytes)
}

#[cfg(feature = "transport-can")]
pub(crate) fn encode_obd2_identity_provisioning_key_record(
    record: Obd2IdentityProvisioningKeyRecord,
) -> [u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES] {
    let mut out = [0xFFu8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES];
    out[0..4].copy_from_slice(&OBD2_IDENTITY_PROVISIONING_KEY_MAGIC.to_le_bytes());
    out[4..6].copy_from_slice(&OBD2_IDENTITY_PROVISIONING_KEY_VERSION.to_le_bytes());
    out[6] = if record.revoked {
        OBD2_IDENTITY_PROVISIONING_KEY_REVOKED
    } else {
        OBD2_IDENTITY_PROVISIONING_KEY_ACTIVE
    };
    out[8..12].copy_from_slice(&record.generation.to_le_bytes());
    out[12..44].copy_from_slice(&record.key);
    out[44..48].copy_from_slice(&record.last_arm_nonce.to_le_bytes());
    let crc = crc16(&out[..48]);
    out[48..50].copy_from_slice(&crc.to_le_bytes());
    out
}

#[cfg(feature = "transport-can")]
pub(crate) fn decode_obd2_identity_provisioning_key_record(
    bytes: &[u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES],
) -> Option<Obd2IdentityProvisioningKeyRecord> {
    if bytes.iter().all(|byte| *byte == 0xFF) {
        return None;
    }
    let magic = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if magic != OBD2_IDENTITY_PROVISIONING_KEY_MAGIC {
        return None;
    }
    let version = u16::from_le_bytes([bytes[4], bytes[5]]);
    if version != OBD2_IDENTITY_PROVISIONING_KEY_VERSION {
        return None;
    }
    let flags = bytes[6];
    let lifecycle_flags =
        flags & (OBD2_IDENTITY_PROVISIONING_KEY_ACTIVE | OBD2_IDENTITY_PROVISIONING_KEY_REVOKED);
    if flags & !(OBD2_IDENTITY_PROVISIONING_KEY_ACTIVE | OBD2_IDENTITY_PROVISIONING_KEY_REVOKED)
        != 0
        || lifecycle_flags.count_ones() != 1
    {
        return None;
    }
    let expected_crc = u16::from_le_bytes([bytes[48], bytes[49]]);
    if crc16(&bytes[..48]) != expected_crc {
        return None;
    }
    let mut key = [0u8; 32];
    key.copy_from_slice(&bytes[12..44]);
    Some(Obd2IdentityProvisioningKeyRecord {
        key,
        generation: u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
        last_arm_nonce: u32::from_le_bytes([bytes[44], bytes[45], bytes[46], bytes[47]]),
        revoked: flags & OBD2_IDENTITY_PROVISIONING_KEY_REVOKED != 0,
    })
}

#[cfg(feature = "transport-can")]
pub(crate) fn read_obd2_identity_provisioning_key_record_with(
    read_record: impl FnOnce(&mut [u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES]),
) -> Option<Obd2IdentityProvisioningKeyRecord> {
    let mut bytes = [0u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES];
    read_record(&mut bytes);
    decode_obd2_identity_provisioning_key_record(&bytes)
}

#[cfg(feature = "transport-can")]
pub(crate) fn encode_obd2_identity_key_audit(
    audit: Obd2IdentityProvisioningKeyAudit,
) -> [u8; OBD2_IDENTITY_KEY_AUDIT_BYTES] {
    let mut out = [0xFFu8; OBD2_IDENTITY_KEY_AUDIT_BYTES];
    out[0..4].copy_from_slice(&OBD2_IDENTITY_KEY_AUDIT_MAGIC.to_le_bytes());
    out[4..6].copy_from_slice(&OBD2_IDENTITY_KEY_AUDIT_VERSION.to_le_bytes());
    out[6..10].copy_from_slice(&audit.request_id.to_le_bytes());
    let mut flags = 0u8;
    if audit.authorized {
        flags |= 0x01;
    }
    if audit.accepted {
        flags |= 0x02;
    }
    if audit.store_failed {
        flags |= 0x04;
    }
    out[10] = flags;
    out[11] = audit.rejected_reason;
    out[12..16].copy_from_slice(&audit.generation.to_le_bytes());
    let crc = crc16(&out[..20]);
    out[20..22].copy_from_slice(&crc.to_le_bytes());
    out
}

#[cfg(feature = "transport-can")]
pub(crate) fn decode_obd2_identity_key_audit(
    bytes: &[u8; OBD2_IDENTITY_KEY_AUDIT_BYTES],
) -> Option<Obd2IdentityProvisioningKeyAudit> {
    if bytes.iter().all(|byte| *byte == 0xFF) {
        return None;
    }
    let magic = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
    if magic != OBD2_IDENTITY_KEY_AUDIT_MAGIC {
        return None;
    }
    let version = u16::from_le_bytes([bytes[4], bytes[5]]);
    if version != OBD2_IDENTITY_KEY_AUDIT_VERSION {
        return None;
    }
    let expected_crc = u16::from_le_bytes([bytes[20], bytes[21]]);
    if crc16(&bytes[..20]) != expected_crc {
        return None;
    }
    let flags = bytes[10];
    if flags & !0x07 != 0 {
        return None;
    }
    Some(Obd2IdentityProvisioningKeyAudit {
        request_id: u32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]),
        authorized: flags & 0x01 != 0,
        accepted: flags & 0x02 != 0,
        store_failed: flags & 0x04 != 0,
        rejected_reason: bytes[11],
        generation: u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
    })
}

#[cfg(feature = "transport-can")]
pub(crate) fn read_obd2_identity_key_audit_with(
    read_record: impl FnOnce(&mut [u8; OBD2_IDENTITY_KEY_AUDIT_BYTES]),
) -> Option<Obd2IdentityProvisioningKeyAudit> {
    let mut bytes = [0u8; OBD2_IDENTITY_KEY_AUDIT_BYTES];
    read_record(&mut bytes);
    decode_obd2_identity_key_audit(&bytes)
}

#[cfg(feature = "transport-can")]
fn encode_obd2_identity_rejected_field(field: Option<Obd2IdentityProvisioningField>) -> u8 {
    match field {
        None => 0,
        Some(Obd2IdentityProvisioningField::Vin) => 1,
        Some(Obd2IdentityProvisioningField::CalibrationId) => 2,
        Some(Obd2IdentityProvisioningField::BoardBuildIdentity) => 3,
    }
}

#[cfg(feature = "transport-can")]
fn decode_obd2_identity_rejected_field(value: u8) -> Option<Option<Obd2IdentityProvisioningField>> {
    match value {
        0 | 0xFF => Some(None),
        1 => Some(Some(Obd2IdentityProvisioningField::Vin)),
        2 => Some(Some(Obd2IdentityProvisioningField::CalibrationId)),
        3 => Some(Some(Obd2IdentityProvisioningField::BoardBuildIdentity)),
        _ => None,
    }
}

#[cfg(feature = "transport-can")]
fn decode_obd2_identity_audit_lengths(
    vin: u8,
    calibration_id: u8,
    board_build_identity: u8,
) -> Option<Obd2IdentityProvisioningLengths> {
    if vin == 0xFF && calibration_id == 0xFF && board_build_identity == 0xFF {
        None
    } else {
        Some(Obd2IdentityProvisioningLengths {
            vin,
            calibration_id,
            board_build_identity,
        })
    }
}

#[cfg(feature = "transport-can")]
fn validate_obd2_identity_record(record: Obd2ProvisionedIdentityRecord) -> Result<(), KvError> {
    if record.vin_len > ecu_transport::CAN_OBD2_VIN_LEN as u8
        || record.calibration_id_len > ecu_transport::CAN_OBD2_VIN_LEN as u8
        || record.board_build_identity_len > 6
    {
        return Err(KvError::Io);
    }
    Ok(())
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
fn prepare_obd2_identity_record_rewrite(
    pages: FlashPageImages,
    snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
    record: Obd2ProvisionedIdentityRecord,
) -> Result<
    (
        ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<FlashPageImages>,
        [u8; OBD2_IDENTITY_RECORD_BYTES],
    ),
    KvError,
> {
    validate_obd2_identity_record(record)?;
    Ok((
        ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite { pages, snapshot },
        encode_obd2_identity_record(record),
    ))
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
fn prepare_obd2_identity_command_audit_rewrite(
    pages: FlashPageImages,
    snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
    identity: Option<Obd2ProvisionedIdentityRecord>,
    audit: Obd2IdentityProvisioningCommandAudit,
) -> (
    ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<FlashPageImages>,
    Option<[u8; OBD2_IDENTITY_RECORD_BYTES]>,
    [u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES],
) {
    (
        ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite { pages, snapshot },
        identity.map(encode_obd2_identity_record),
        encode_obd2_identity_command_audit(audit),
    )
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
fn prepare_obd2_identity_provisioning_key_rewrite(
    pages: FlashPageImages,
    snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
    identity: Option<Obd2ProvisionedIdentityRecord>,
    audit: Option<Obd2IdentityProvisioningCommandAudit>,
    key_record: Obd2IdentityProvisioningKeyRecord,
) -> (
    ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<FlashPageImages>,
    Option<[u8; OBD2_IDENTITY_RECORD_BYTES]>,
    Option<[u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES]>,
    [u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES],
) {
    (
        ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite { pages, snapshot },
        identity.map(encode_obd2_identity_record),
        audit.map(encode_obd2_identity_command_audit),
        encode_obd2_identity_provisioning_key_record(key_record),
    )
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
fn prepare_obd2_identity_key_audit_rewrite(
    pages: FlashPageImages,
    snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
    identity: Option<Obd2ProvisionedIdentityRecord>,
    audit: Option<Obd2IdentityProvisioningCommandAudit>,
    key_record: Option<Obd2IdentityProvisioningKeyRecord>,
    key_audit: Obd2IdentityProvisioningKeyAudit,
) -> (
    ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<FlashPageImages>,
    Option<[u8; OBD2_IDENTITY_RECORD_BYTES]>,
    Option<[u8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES]>,
    Option<[u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES]>,
    [u8; OBD2_IDENTITY_KEY_AUDIT_BYTES],
) {
    (
        ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite { pages, snapshot },
        identity.map(encode_obd2_identity_record),
        audit.map(encode_obd2_identity_command_audit),
        key_record.map(encode_obd2_identity_provisioning_key_record),
        encode_obd2_identity_key_audit(key_audit),
    )
}

#[cfg(feature = "transport-can")]
fn verify_obd2_identity_key_audit_readback(
    expected_key_record: Option<Obd2IdentityProvisioningKeyRecord>,
    expected_key_audit: Obd2IdentityProvisioningKeyAudit,
    load_key_record: impl FnOnce() -> Option<Obd2IdentityProvisioningKeyRecord>,
    load_key_audit: impl FnOnce() -> Option<Obd2IdentityProvisioningKeyAudit>,
) -> Result<(), KvError> {
    if load_key_audit() == Some(expected_key_audit) && load_key_record() == expected_key_record {
        Ok(())
    } else {
        Err(KvError::Io)
    }
}

pub(crate) fn validate_page_image(
    layout: FlashLayout,
    hdr: KvHeader,
    key: FlashPageKey,
    data: &[u8],
) -> bool {
    let page = layout.page(key);
    hdr.ok
        && header_page_len(hdr, key) as usize == page.len
        && data.len() == page.len
        && crc16(data) == header_page_crc(hdr, key)
}

pub(crate) unsafe fn read_header(base: u32) -> KvHeader {
    let p = base as *const u8;
    let magic = core::ptr::read_volatile(p as *const u32);
    if magic != FlashLayout::MAGIC {
        return invalid_header();
    }
    let valid = core::ptr::read_volatile(p.add(6) as *const u16);
    if valid != 0 {
        return invalid_header();
    }
    KvHeader {
        ok: true,
        seq: core::ptr::read_volatile(p.add(8) as *const u16),
        fuel_len: core::ptr::read_volatile(p.add(10) as *const u16),
        ign_len: core::ptr::read_volatile(p.add(12) as *const u16),
        angles_len: core::ptr::read_volatile(p.add(18) as *const u16),
        fuel_crc: core::ptr::read_volatile(p.add(14) as *const u16),
        ign_crc: core::ptr::read_volatile(p.add(16) as *const u16),
        angles_crc: core::ptr::read_volatile(p.add(20) as *const u16),
    }
}

pub(crate) unsafe fn read_page(base: u32, off: usize, out: &mut [u8]) {
    let p = base as *const u8;
    for (i, byte) in out.iter_mut().enumerate() {
        *byte = core::ptr::read_volatile(p.add(off + i));
    }
}

fn invalid_header() -> KvHeader {
    KvHeader {
        ok: false,
        seq: 0,
        fuel_len: 0,
        ign_len: 0,
        angles_len: 0,
        fuel_crc: 0,
        ign_crc: 0,
        angles_crc: 0,
    }
}

#[derive(Copy, Clone)]
enum SectorIntegrity {
    Blank,
    Valid,
    Corrupt,
}

fn sector_header_is_blank(base: u32, header_len: usize) -> bool {
    let p = base as *const u8;
    let mut i = 0;
    while i < header_len {
        if unsafe { core::ptr::read_volatile(p.add(i)) } != 0xFF {
            return false;
        }
        i += 1;
    }
    true
}

fn classify_sector(layout: FlashLayout, base: u32) -> SectorIntegrity {
    if sector_header_is_blank(base, layout.header_len) {
        return SectorIntegrity::Blank;
    }
    let hdr = unsafe { read_header(base) };
    if !hdr.ok {
        return SectorIntegrity::Corrupt;
    }

    let mut fuel = [0u8; FUEL_PAGE_LEN];
    let mut ign = [0u8; IGN_PAGE_LEN];
    let mut angles = [0u8; ANGLES_PAGE_LEN];
    unsafe {
        read_page(base, layout.fuel.offset, &mut fuel);
        read_page(base, layout.ign.offset, &mut ign);
        read_page(base, layout.angles.offset, &mut angles);
    }
    if !validate_page_image(layout, hdr, FlashPageKey::Fuel, &fuel)
        || !validate_page_image(layout, hdr, FlashPageKey::Ign, &ign)
        || !validate_page_image(layout, hdr, FlashPageKey::Angles, &angles)
    {
        return SectorIntegrity::Corrupt;
    }
    SectorIntegrity::Valid
}

fn store_integrity_from_sectors(a: SectorIntegrity, b: SectorIntegrity) -> StoreIntegrityStatus {
    match (a, b) {
        (SectorIntegrity::Blank, SectorIntegrity::Blank) => StoreIntegrityStatus::Blank,
        (SectorIntegrity::Valid, SectorIntegrity::Corrupt)
        | (SectorIntegrity::Corrupt, SectorIntegrity::Valid) => {
            StoreIntegrityStatus::ValidWithCorruptSibling
        }
        (SectorIntegrity::Valid, SectorIntegrity::Blank)
        | (SectorIntegrity::Blank, SectorIntegrity::Valid)
        | (SectorIntegrity::Valid, SectorIntegrity::Valid) => StoreIntegrityStatus::Valid,
        (SectorIntegrity::Blank, SectorIntegrity::Corrupt)
        | (SectorIntegrity::Corrupt, SectorIntegrity::Blank)
        | (SectorIntegrity::Corrupt, SectorIntegrity::Corrupt) => StoreIntegrityStatus::Corrupt,
    }
}

#[cfg(feature = "flash-kv")]
unsafe fn read_seq(base: u32) -> Option<u16> {
    let p = base as *const u8;
    if core::ptr::read_volatile(p as *const u32) != FlashLayout::MAGIC {
        return None;
    }
    if core::ptr::read_volatile(p.add(6) as *const u16) != FlashLayout::VALID_COMMITTED {
        return None;
    }
    Some(core::ptr::read_volatile(p.add(8) as *const u16))
}

#[cfg(feature = "flash-kv")]
fn encode_rewrite_header(
    new_seq: u16,
    fuel: &[u8; FUEL_PAGE_LEN],
    ign: &[u8; IGN_PAGE_LEN],
    angles: &[u8; ANGLES_PAGE_LEN],
) -> [u8; PAGE_HEADER_LEN] {
    let mut hdr = [0xFFu8; PAGE_HEADER_LEN];
    hdr[0] = 0x56;
    hdr[1] = 0x4B;
    hdr[2] = 0x50;
    hdr[3] = 0x32;
    hdr[4] = 2;
    hdr[5] = 0;
    hdr[8] = (new_seq & 0xFF) as u8;
    hdr[9] = (new_seq >> 8) as u8;
    hdr[10] = (FUEL_PAGE_LEN as u16 & 0xFF) as u8;
    hdr[11] = (FUEL_PAGE_LEN as u16 >> 8) as u8;
    hdr[12] = (IGN_PAGE_LEN as u16 & 0xFF) as u8;
    hdr[13] = (IGN_PAGE_LEN as u16 >> 8) as u8;
    hdr[18] = (ANGLES_PAGE_LEN as u16 & 0xFF) as u8;
    hdr[19] = (ANGLES_PAGE_LEN as u16 >> 8) as u8;
    let fuel_crc = crc16(fuel);
    let ign_crc = crc16(ign);
    let angles_crc = crc16(angles);
    hdr[14] = (fuel_crc & 0xFF) as u8;
    hdr[15] = (fuel_crc >> 8) as u8;
    hdr[16] = (ign_crc & 0xFF) as u8;
    hdr[17] = (ign_crc >> 8) as u8;
    hdr[20] = (angles_crc & 0xFF) as u8;
    hdr[21] = (angles_crc >> 8) as u8;
    hdr
}

#[cfg(feature = "flash-kv")]
fn decode_flash_write_phase(value: u8) -> FlashWritePhase {
    match value {
        1 => FlashWritePhase::Preflight,
        2 => FlashWritePhase::Erase,
        3 => FlashWritePhase::Program,
        _ => FlashWritePhase::None,
    }
}

#[cfg(feature = "flash-kv")]
fn encode_flash_write_phase(phase: FlashWritePhase) -> u8 {
    match phase {
        FlashWritePhase::None => 0,
        FlashWritePhase::Preflight => 1,
        FlashWritePhase::Erase => 2,
        FlashWritePhase::Program => 3,
    }
}

#[cfg(feature = "flash-kv")]
fn clear_last_flash_write_fault() {
    LAST_FLASH_WRITE_FAULT_SR_BITS.store(0, Ordering::Relaxed);
    LAST_FLASH_WRITE_FAULT_PHASE.store(0, Ordering::Relaxed);
}

#[cfg(feature = "flash-kv")]
fn record_flash_write_fault(phase: FlashWritePhase, sr_bits: u32) {
    LAST_FLASH_WRITE_FAULT_SR_BITS.store(sr_bits, Ordering::Relaxed);
    LAST_FLASH_WRITE_FAULT_PHASE.store(encode_flash_write_phase(phase), Ordering::Relaxed);
}

#[cfg(feature = "flash-kv")]
pub(crate) fn last_flash_write_fault() -> Option<FlashWriteFaultSnapshot> {
    let phase = decode_flash_write_phase(LAST_FLASH_WRITE_FAULT_PHASE.load(Ordering::Relaxed));
    if phase == FlashWritePhase::None {
        return None;
    }
    Some(FlashWriteFaultSnapshot {
        phase,
        sr_bits: LAST_FLASH_WRITE_FAULT_SR_BITS.load(Ordering::Relaxed),
    })
}

#[cfg(all(feature = "flash-kv", feature = "transport-can"))]
pub(crate) fn last_flash_write_fault_status() -> ecu_transport::CanObd2FlashWriteFaultStatus {
    let Some(snapshot) = last_flash_write_fault() else {
        return ecu_transport::CanObd2FlashWriteFaultStatus::absent();
    };
    let phase = match snapshot.phase {
        FlashWritePhase::None => ecu_transport::CanObd2FlashWriteFaultPhase::None,
        FlashWritePhase::Preflight => ecu_transport::CanObd2FlashWriteFaultPhase::Preflight,
        FlashWritePhase::Erase => ecu_transport::CanObd2FlashWriteFaultPhase::Erase,
        FlashWritePhase::Program => ecu_transport::CanObd2FlashWriteFaultPhase::Program,
    };
    ecu_transport::CanObd2FlashWriteFaultStatus {
        present: true,
        phase,
        sr_bits: snapshot.sr_bits,
    }
}

#[cfg(feature = "flash-kv")]
fn flash_status_result_for_phase(phase: FlashWritePhase, sr_bits: u32) -> Result<(), KvError> {
    if sr_bits & FLASH_SR_ERROR_MASK == 0 {
        Ok(())
    } else {
        if phase != FlashWritePhase::None {
            record_flash_write_fault(phase, sr_bits);
        }
        Err(KvError::Io)
    }
}

#[cfg(feature = "flash-kv")]
fn flash_status_result(sr_bits: u32) -> Result<(), KvError> {
    flash_status_result_for_phase(FlashWritePhase::None, sr_bits)
}

#[cfg(feature = "flash-kv")]
fn wait_for_flash_ready(
    flash: &pac::flash::RegisterBlock,
    phase: FlashWritePhase,
) -> Result<(), KvError> {
    while flash.sr.read().bsy().bit_is_set() {}
    flash_status_result_for_phase(phase, flash.sr.read().bits())
}

#[cfg(feature = "flash-kv")]
fn clear_flash_status(flash: &pac::flash::RegisterBlock) {
    flash.sr.write(|w| unsafe { w.bits(FLASH_SR_CLEAR_MASK) });
}

#[cfg(feature = "flash-kv")]
unsafe fn rewrite_sector(
    layout: FlashLayout,
    seq_a: Option<u16>,
    seq_b: Option<u16>,
    fuel: &[u8; FUEL_PAGE_LEN],
    ign: &[u8; IGN_PAGE_LEN],
    angles: &[u8; ANGLES_PAGE_LEN],
    #[cfg(feature = "transport-can")] snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
    #[cfg(feature = "transport-can")] identity_record: Option<&Obd2ProvisionedIdentityRecord>,
    #[cfg(feature = "transport-can")] identity_command_audit: Option<
        &Obd2IdentityProvisioningCommandAudit,
    >,
    #[cfg(feature = "transport-can")] identity_provisioning_key: Option<
        &Obd2IdentityProvisioningKeyRecord,
    >,
    #[cfg(feature = "transport-can")] identity_key_audit: Option<&Obd2IdentityProvisioningKeyAudit>,
) -> Result<(), KvError> {
    let cur_seq = match (seq_a, seq_b) {
        (Some(a), Some(b)) => {
            if b.wrapping_sub(a) < 0x8000 {
                b
            } else {
                a
            }
        }
        (Some(a), None) => a,
        (None, Some(b)) => b,
        _ => 0,
    };
    let (target_sector, target_base) = match (seq_a, seq_b) {
        (Some(a), Some(b)) => {
            if b.wrapping_sub(a) < 0x8000 {
                (layout.sector_a, layout.base_a)
            } else {
                (layout.sector_b, layout.base_b)
            }
        }
        (Some(_), None) => (layout.sector_b, layout.base_b),
        (None, Some(_)) => (layout.sector_a, layout.base_a),
        (None, None) => (layout.sector_a, layout.base_a),
    };
    let new_seq = cur_seq.wrapping_add(1);

    let hdr = encode_rewrite_header(new_seq, fuel, ign, angles);
    cortex_m::interrupt::free(|_| -> Result<(), KvError> {
        let flash = &*pac::FLASH::ptr();

        while flash.sr.read().bsy().bit_is_set() {}
        clear_last_flash_write_fault();
        clear_flash_status(flash);
        if flash.cr.read().lock().bit_is_set() {
            flash.keyr.write(|w| w.key().bits(0x4567_0123));
            flash.keyr.write(|w| w.key().bits(0xCDEF_89AB));
        }

        let program_result = (|| -> Result<(), KvError> {
            wait_for_flash_ready(flash, FlashWritePhase::Preflight)?;
            flash
                .cr
                .modify(|_, w| w.ser().set_bit().snb().bits(target_sector));
            flash.cr.modify(|_, w| w.strt().set_bit());
            let erase_result = wait_for_flash_ready(flash, FlashWritePhase::Erase);
            flash.cr.modify(|_, w| w.ser().clear_bit());
            erase_result?;

            flash.cr.modify(|_, w| w.psize().bits(0b01).pg().set_bit());
            let prog_half = |addr: u32, val: u16| -> Result<(), KvError> {
                core::ptr::write_volatile(addr as *mut u16, val);
                wait_for_flash_ready(flash, FlashWritePhase::Program)
            };

            let mut addr = target_base;
            for i in (0..layout.header_len).step_by(2) {
                let value = (hdr[i] as u16) | ((hdr[i + 1] as u16) << 8);
                prog_half(addr, value)?;
                addr += 2;
            }
            let mut addr = target_base + (layout.fuel.offset as u32);
            for i in (0..layout.fuel.len).step_by(2) {
                let value = (fuel[i] as u16) | ((fuel[i + 1] as u16) << 8);
                prog_half(addr, value)?;
                addr += 2;
            }
            let mut addr = target_base + (layout.ign.offset as u32);
            for i in (0..layout.ign.len).step_by(2) {
                let value = (ign[i] as u16) | ((ign[i + 1] as u16) << 8);
                prog_half(addr, value)?;
                addr += 2;
            }
            let mut addr = target_base + (layout.angles.offset as u32);
            for i in (0..layout.angles.len).step_by(2) {
                let value = (angles[i] as u16) | ((angles[i + 1] as u16) << 8);
                prog_half(addr, value)?;
                addr += 2;
            }
            #[cfg(feature = "transport-can")]
            let mut retained_history_program_error = None;
            #[cfg(feature = "transport-can")]
            install_optional_obd2_retained_history_snapshot_halfwords_with(
                snapshot,
                |offset, value| {
                    if retained_history_program_error.is_some() {
                        return;
                    }
                    if let Err(err) = prog_half(
                        target_base + (OBD2_SNAPSHOT_OFFSET as u32) + (offset as u32),
                        value,
                    ) {
                        retained_history_program_error = Some(err);
                    }
                },
            )
            .expect("snapshot staging");
            #[cfg(feature = "transport-can")]
            if let Some(err) = retained_history_program_error {
                return Err(err);
            }
            #[cfg(feature = "transport-can")]
            if let Some(identity_record) = identity_record {
                let bytes = encode_obd2_identity_record(*identity_record);
                let mut addr = target_base + (OBD2_IDENTITY_RECORD_OFFSET as u32);
                for i in (0..bytes.len()).step_by(2) {
                    let value = (bytes[i] as u16) | ((bytes[i + 1] as u16) << 8);
                    prog_half(addr, value)?;
                    addr += 2;
                }
            }
            #[cfg(feature = "transport-can")]
            if let Some(identity_command_audit) = identity_command_audit {
                let bytes = encode_obd2_identity_command_audit(*identity_command_audit);
                let mut addr = target_base + (OBD2_IDENTITY_COMMAND_AUDIT_OFFSET as u32);
                for i in (0..bytes.len()).step_by(2) {
                    let value = (bytes[i] as u16) | ((bytes[i + 1] as u16) << 8);
                    prog_half(addr, value)?;
                    addr += 2;
                }
            }
            #[cfg(feature = "transport-can")]
            if let Some(identity_provisioning_key) = identity_provisioning_key {
                let bytes =
                    encode_obd2_identity_provisioning_key_record(*identity_provisioning_key);
                let mut addr = target_base + (OBD2_IDENTITY_PROVISIONING_KEY_OFFSET as u32);
                for i in (0..bytes.len()).step_by(2) {
                    let value = (bytes[i] as u16) | ((bytes[i + 1] as u16) << 8);
                    prog_half(addr, value)?;
                    addr += 2;
                }
            }
            #[cfg(feature = "transport-can")]
            if let Some(identity_key_audit) = identity_key_audit {
                let bytes = encode_obd2_identity_key_audit(*identity_key_audit);
                let mut addr = target_base + (OBD2_IDENTITY_KEY_AUDIT_OFFSET as u32);
                for i in (0..bytes.len()).step_by(2) {
                    let value = (bytes[i] as u16) | ((bytes[i + 1] as u16) << 8);
                    prog_half(addr, value)?;
                    addr += 2;
                }
            }

            let valid_addr = target_base + 6;
            prog_half(valid_addr, 0x0000)?;
            Ok(())
        })();

        flash.cr.modify(|_, w| w.ser().clear_bit().pg().clear_bit());
        flash.cr.modify(|_, w| w.lock().set_bit());
        program_result
    })
}

#[cfg(feature = "flash-kv")]
pub struct FlashKv;

#[cfg(feature = "flash-kv")]
impl FlashKv {
    pub const fn new() -> Self {
        Self
    }

    const LAYOUT: FlashLayout = STM32F405_TS_KV_LAYOUT;

    fn load_current_pages(&mut self) -> FlashPageImages {
        let mut fuel = [0u8; FUEL_PAGE_LEN];
        let mut ign = [0u8; IGN_PAGE_LEN];
        let mut angles = [0u8; ANGLES_PAGE_LEN];
        let _ = self.read(b"fuel", &mut fuel);
        let _ = self.read(b"ign", &mut ign);
        let _ = self.read(b"angles", &mut angles);
        FlashPageImages { fuel, ign, angles }
    }

    #[cfg(feature = "transport-can")]
    pub fn load_retained_obd2_history_snapshot(
        &self,
    ) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr)?;
        let base = match active {
            ActiveSector::A => Self::LAYOUT.base_a,
            ActiveSector::B => Self::LAYOUT.base_b,
        };
        read_obd2_retained_history_snapshot_with(|out| unsafe {
            read_page(base, OBD2_SNAPSHOT_OFFSET, out);
        })
    }

    #[cfg(feature = "transport-can")]
    pub fn load_obd2_identity_record(&self) -> Option<Obd2ProvisionedIdentityRecord> {
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr)?;
        let base = match active {
            ActiveSector::A => Self::LAYOUT.base_a,
            ActiveSector::B => Self::LAYOUT.base_b,
        };
        read_obd2_identity_record_with(|out| unsafe {
            read_page(base, OBD2_IDENTITY_RECORD_OFFSET, out);
        })
    }

    #[cfg(feature = "transport-can")]
    pub fn load_obd2_identity_command_audit(&self) -> Option<Obd2IdentityProvisioningCommandAudit> {
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr)?;
        let base = match active {
            ActiveSector::A => Self::LAYOUT.base_a,
            ActiveSector::B => Self::LAYOUT.base_b,
        };
        read_obd2_identity_command_audit_with(|out| unsafe {
            read_page(base, OBD2_IDENTITY_COMMAND_AUDIT_OFFSET, out);
        })
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn load_obd2_identity_provisioning_key_record(
        &self,
    ) -> Option<Obd2IdentityProvisioningKeyRecord> {
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr)?;
        let base = match active {
            ActiveSector::A => Self::LAYOUT.base_a,
            ActiveSector::B => Self::LAYOUT.base_b,
        };
        read_obd2_identity_provisioning_key_record_with(|out| unsafe {
            read_page(base, OBD2_IDENTITY_PROVISIONING_KEY_OFFSET, out);
        })
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn load_obd2_identity_key_audit(&self) -> Option<Obd2IdentityProvisioningKeyAudit> {
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr)?;
        let base = match active {
            ActiveSector::A => Self::LAYOUT.base_a,
            ActiveSector::B => Self::LAYOUT.base_b,
        };
        read_obd2_identity_key_audit_with(|out| unsafe {
            read_page(base, OBD2_IDENTITY_KEY_AUDIT_OFFSET, out);
        })
    }

    #[cfg(feature = "transport-can")]
    fn persist_retained_history_rewrite(
        &mut self,
        prepare: impl FnOnce(
            FlashPageImages,
            Option<Obd2RetainedDiagnosticHistorySnapshot>,
        ) -> Result<
            ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<FlashPageImages>,
            KvError,
        >,
    ) -> Result<(), KvError> {
        let current_identity = self.load_obd2_identity_record();
        let current_identity_command_audit = self.load_obd2_identity_command_audit();
        let current_identity_provisioning_key = self.load_obd2_identity_provisioning_key_record();
        let current_identity_key_audit = self.load_obd2_identity_key_audit();
        persist_retained_history_flash_rewrite(
            self.load_current_pages(),
            self.load_retained_obd2_history_snapshot(),
            prepare,
            |rewrite| {
                let (seq_a, seq_b) =
                    unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
                unsafe {
                    rewrite_sector(
                        Self::LAYOUT,
                        seq_a,
                        seq_b,
                        &rewrite.pages.fuel,
                        &rewrite.pages.ign,
                        &rewrite.pages.angles,
                        rewrite.snapshot.as_ref(),
                        current_identity.as_ref(),
                        current_identity_command_audit.as_ref(),
                        current_identity_provisioning_key.as_ref(),
                        current_identity_key_audit.as_ref(),
                    )?;
                }
                Ok(())
            },
        )
    }

    #[cfg(feature = "transport-can")]
    pub fn save_retained_obd2_history_snapshot(
        &mut self,
        snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    ) -> Result<(), KvError> {
        self.persist_retained_history_rewrite(|current_pages, _| {
            Ok(prepare_retained_history_snapshot_rewrite(
                current_pages,
                snapshot,
            ))
        })
    }

    #[cfg(feature = "transport-can")]
    pub fn write_obd2_identity_record(
        &mut self,
        record: Obd2ProvisionedIdentityRecord,
    ) -> Result<(), KvError> {
        let (rewrite, _) = prepare_obd2_identity_record_rewrite(
            self.load_current_pages(),
            self.load_retained_obd2_history_snapshot(),
            record,
        )?;
        let (seq_a, seq_b) =
            unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
        unsafe {
            rewrite_sector(
                Self::LAYOUT,
                seq_a,
                seq_b,
                &rewrite.pages.fuel,
                &rewrite.pages.ign,
                &rewrite.pages.angles,
                rewrite.snapshot.as_ref(),
                Some(&record),
                self.load_obd2_identity_command_audit().as_ref(),
                self.load_obd2_identity_provisioning_key_record().as_ref(),
                self.load_obd2_identity_key_audit().as_ref(),
            )?;
        }
        if self.load_obd2_identity_record() == Some(record) {
            Ok(())
        } else {
            Err(KvError::Io)
        }
    }

    #[cfg(feature = "transport-can")]
    pub fn write_obd2_identity_command_audit(
        &mut self,
        audit: Obd2IdentityProvisioningCommandAudit,
    ) -> Result<(), KvError> {
        let identity = self.load_obd2_identity_record();
        let (rewrite, _, _) = prepare_obd2_identity_command_audit_rewrite(
            self.load_current_pages(),
            self.load_retained_obd2_history_snapshot(),
            identity,
            audit,
        );
        let (seq_a, seq_b) =
            unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
        unsafe {
            rewrite_sector(
                Self::LAYOUT,
                seq_a,
                seq_b,
                &rewrite.pages.fuel,
                &rewrite.pages.ign,
                &rewrite.pages.angles,
                rewrite.snapshot.as_ref(),
                identity.as_ref(),
                Some(&audit),
                self.load_obd2_identity_provisioning_key_record().as_ref(),
                self.load_obd2_identity_key_audit().as_ref(),
            )?;
        }
        if self.load_obd2_identity_command_audit() == Some(audit) {
            Ok(())
        } else {
            Err(KvError::Io)
        }
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn write_obd2_identity_provisioning_key_record(
        &mut self,
        key_record: Obd2IdentityProvisioningKeyRecord,
    ) -> Result<(), KvError> {
        let identity = self.load_obd2_identity_record();
        let audit = self.load_obd2_identity_command_audit();
        let (rewrite, _, _, _) = prepare_obd2_identity_provisioning_key_rewrite(
            self.load_current_pages(),
            self.load_retained_obd2_history_snapshot(),
            identity,
            audit,
            key_record,
        );
        let (seq_a, seq_b) =
            unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
        unsafe {
            rewrite_sector(
                Self::LAYOUT,
                seq_a,
                seq_b,
                &rewrite.pages.fuel,
                &rewrite.pages.ign,
                &rewrite.pages.angles,
                rewrite.snapshot.as_ref(),
                identity.as_ref(),
                audit.as_ref(),
                Some(&key_record),
                self.load_obd2_identity_key_audit().as_ref(),
            )?;
        }
        if self.load_obd2_identity_provisioning_key_record() == Some(key_record) {
            Ok(())
        } else {
            Err(KvError::Io)
        }
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn write_obd2_identity_key_audit(
        &mut self,
        key_audit: Obd2IdentityProvisioningKeyAudit,
    ) -> Result<(), KvError> {
        self.write_obd2_identity_key_audit_with_key_record(
            key_audit,
            self.load_obd2_identity_provisioning_key_record(),
        )
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn write_obd2_identity_key_audit_with_key_record(
        &mut self,
        key_audit: Obd2IdentityProvisioningKeyAudit,
        key_record: Option<Obd2IdentityProvisioningKeyRecord>,
    ) -> Result<(), KvError> {
        let identity = self.load_obd2_identity_record();
        let audit = self.load_obd2_identity_command_audit();
        let (rewrite, _, _, _, _) = prepare_obd2_identity_key_audit_rewrite(
            self.load_current_pages(),
            self.load_retained_obd2_history_snapshot(),
            identity,
            audit,
            key_record,
            key_audit,
        );
        let (seq_a, seq_b) =
            unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
        unsafe {
            rewrite_sector(
                Self::LAYOUT,
                seq_a,
                seq_b,
                &rewrite.pages.fuel,
                &rewrite.pages.ign,
                &rewrite.pages.angles,
                rewrite.snapshot.as_ref(),
                identity.as_ref(),
                audit.as_ref(),
                key_record.as_ref(),
                Some(&key_audit),
            )?;
        }
        verify_obd2_identity_key_audit_readback(
            key_record,
            key_audit,
            || self.load_obd2_identity_provisioning_key_record(),
            || self.load_obd2_identity_key_audit(),
        )
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn write_obd2_identity_provisioning_key_record_with_audit(
        &mut self,
        key_record: Obd2IdentityProvisioningKeyRecord,
        key_audit: Obd2IdentityProvisioningKeyAudit,
    ) -> Result<(), KvError> {
        let identity = self.load_obd2_identity_record();
        let audit = self.load_obd2_identity_command_audit();
        let (rewrite, _, _, _, _) = prepare_obd2_identity_key_audit_rewrite(
            self.load_current_pages(),
            self.load_retained_obd2_history_snapshot(),
            identity,
            audit,
            Some(key_record),
            key_audit,
        );
        let (seq_a, seq_b) =
            unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
        unsafe {
            rewrite_sector(
                Self::LAYOUT,
                seq_a,
                seq_b,
                &rewrite.pages.fuel,
                &rewrite.pages.ign,
                &rewrite.pages.angles,
                rewrite.snapshot.as_ref(),
                identity.as_ref(),
                audit.as_ref(),
                Some(&key_record),
                Some(&key_audit),
            )?;
        }
        verify_obd2_identity_key_audit_readback(
            Some(key_record),
            key_audit,
            || self.load_obd2_identity_provisioning_key_record(),
            || self.load_obd2_identity_key_audit(),
        )
    }

    #[cfg(feature = "transport-can")]
    pub(crate) fn write_obd2_identity_provisioning_arm_nonce(
        &mut self,
        current_key: [u8; 32],
        nonce: u32,
    ) -> Result<(), KvError> {
        let record = match self.load_obd2_identity_provisioning_key_record() {
            Some(record) if record.active_key() == Some(current_key) => record,
            Some(_) => return Err(KvError::Io),
            None => Obd2IdentityProvisioningKeyRecord::active(current_key, 0),
        };
        self.write_obd2_identity_provisioning_key_record(record.with_last_arm_nonce(nonce))
    }
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
pub(crate) fn provision_obd2_identity_record(
    store: &mut FlashKv,
    record: Obd2ProvisionedIdentityRecord,
) -> Result<(), KvError> {
    store.write_obd2_identity_record(record)
}

#[cfg(feature = "flash-kv")]
impl KvStore for FlashKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr).ok_or(KvError::NotFound)?;
        let (base, hdr) = match active {
            ActiveSector::A => (Self::LAYOUT.base_a, a_hdr),
            ActiveSector::B => (Self::LAYOUT.base_b, b_hdr),
        };
        let key = FlashPageKey::from_key(key).ok_or(KvError::Io)?;
        let page = Self::LAYOUT.page(key);
        let actual_len = header_page_len(hdr, key);
        if actual_len as usize != page.len || out.len() < page.len {
            return Err(KvError::NotFound);
        }
        unsafe {
            read_page(base, page.offset, &mut out[..page.len]);
        }
        if !validate_page_image(Self::LAYOUT, hdr, key, &out[..page.len]) {
            return Err(KvError::Io);
        }
        Ok(page.len)
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        self.persist_retained_history_rewrite(|current_pages, current_snapshot| {
            prepare_retained_history_preserved_page_rewrite(
                current_pages,
                current_snapshot,
                |pages| {
                    apply_retained_history_page_update(pages, key, data).map_err(|_| KvError::Io)
                },
            )
        })
    }
}

#[cfg(all(feature = "flash-kv", feature = "transport-can"))]
impl ecu_target_common::transport_service::Obd2RetainedHistoryStore for FlashKv {
    type Error = KvError;

    fn load_retained_obd2_history_snapshot(&self) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
        FlashKv::load_retained_obd2_history_snapshot(self)
    }

    fn save_retained_obd2_history_snapshot(
        &mut self,
        snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    ) -> Result<(), Self::Error> {
        FlashKv::save_retained_obd2_history_snapshot(self, snapshot)
    }
}

#[cfg(feature = "flash-kv")]
pub(crate) fn boot_store_integrity() -> StoreIntegrityStatus {
    let a = classify_sector(STM32F405_TS_KV_LAYOUT, STM32F405_TS_KV_LAYOUT.base_a);
    let b = classify_sector(STM32F405_TS_KV_LAYOUT, STM32F405_TS_KV_LAYOUT.base_b);
    store_integrity_from_sectors(a, b)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "transport-can")]
    use ecu_domain::{
        diag::{DiagCode, DiagEvent, DiagSource},
        FaultCode, Micros,
    };
    #[cfg(feature = "transport-can")]
    use ecu_transport::Message;
    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    use std::vec::Vec;

    #[test]
    fn store_integrity_reports_blank_only_when_both_sectors_are_blank() {
        assert_eq!(
            store_integrity_from_sectors(SectorIntegrity::Blank, SectorIntegrity::Blank),
            StoreIntegrityStatus::Blank
        );
        assert_eq!(
            store_integrity_from_sectors(SectorIntegrity::Blank, SectorIntegrity::Valid),
            StoreIntegrityStatus::Valid
        );
    }

    #[test]
    fn store_integrity_reports_valid_with_corrupt_sibling_when_one_sector_is_good() {
        assert_eq!(
            store_integrity_from_sectors(SectorIntegrity::Valid, SectorIntegrity::Corrupt),
            StoreIntegrityStatus::ValidWithCorruptSibling
        );
    }

    #[test]
    fn store_integrity_reports_corrupt_when_no_valid_sector_exists() {
        assert_eq!(
            store_integrity_from_sectors(SectorIntegrity::Blank, SectorIntegrity::Corrupt),
            StoreIntegrityStatus::Corrupt
        );
        assert_eq!(
            store_integrity_from_sectors(SectorIntegrity::Corrupt, SectorIntegrity::Corrupt),
            StoreIntegrityStatus::Corrupt
        );
    }

    #[cfg(feature = "flash-kv")]
    #[test]
    fn flash_status_result_reports_any_hardware_error_flag() {
        assert_eq!(flash_status_result(0), Ok(()));
        for bit in [1u32, 4, 5, 6, 7] {
            assert_eq!(flash_status_result(1 << bit), Err(KvError::Io));
        }
        assert_eq!(
            flash_status_result(FLASH_SR_ERROR_MASK | (1 << 16)),
            Err(KvError::Io)
        );
    }

    #[cfg(feature = "flash-kv")]
    #[test]
    fn flash_status_result_records_phase_and_raw_status_bits_for_board_evidence() {
        clear_last_flash_write_fault();
        assert_eq!(last_flash_write_fault(), None);

        assert_eq!(
            flash_status_result_for_phase(FlashWritePhase::Erase, 1 << 4),
            Err(KvError::Io)
        );
        assert_eq!(
            last_flash_write_fault(),
            Some(FlashWriteFaultSnapshot {
                phase: FlashWritePhase::Erase,
                sr_bits: 1 << 4,
            })
        );

        clear_last_flash_write_fault();
        assert_eq!(
            flash_status_result_for_phase(FlashWritePhase::Program, 1 << 7),
            Err(KvError::Io)
        );
        assert_eq!(
            last_flash_write_fault(),
            Some(FlashWriteFaultSnapshot {
                phase: FlashWritePhase::Program,
                sr_bits: 1 << 7,
            })
        );
    }

    #[cfg(feature = "flash-kv")]
    fn emulated_header_valid(header: &[u8; PAGE_HEADER_LEN]) -> bool {
        header[6] == 0 && header[7] == 0
    }

    #[cfg(feature = "flash-kv")]
    fn emulate_rewrite_header_until_commit(
        header: [u8; PAGE_HEADER_LEN],
        fail_after_programmed_header_halfwords: Option<usize>,
    ) -> [u8; PAGE_HEADER_LEN] {
        let mut sector_header = [0xFFu8; PAGE_HEADER_LEN];
        let mut programmed = 0usize;
        for i in (0..PAGE_HEADER_LEN).step_by(2) {
            if fail_after_programmed_header_halfwords == Some(programmed) {
                return sector_header;
            }
            sector_header[i] = header[i];
            sector_header[i + 1] = header[i + 1];
            programmed += 1;
        }
        sector_header
    }

    #[cfg(feature = "flash-kv")]
    fn decode_emulated_header(header: &[u8; PAGE_HEADER_LEN]) -> KvHeader {
        let magic = u32::from_le_bytes([header[0], header[1], header[2], header[3]]);
        let committed = u16::from_le_bytes([header[6], header[7]]);
        let mut hdr = KvHeader {
            ok: magic == FlashLayout::MAGIC && committed == FlashLayout::VALID_COMMITTED,
            seq: u16::from_le_bytes([header[8], header[9]]),
            fuel_len: u16::from_le_bytes([header[10], header[11]]),
            ign_len: u16::from_le_bytes([header[12], header[13]]),
            fuel_crc: u16::from_le_bytes([header[14], header[15]]),
            ign_crc: u16::from_le_bytes([header[16], header[17]]),
            angles_len: u16::from_le_bytes([header[18], header[19]]),
            angles_crc: u16::from_le_bytes([header[20], header[21]]),
        };
        hdr.ok = hdr.ok
            && hdr.fuel_len as usize == FUEL_PAGE_LEN
            && hdr.ign_len as usize == IGN_PAGE_LEN
            && hdr.angles_len as usize == ANGLES_PAGE_LEN;
        hdr
    }

    #[cfg(feature = "flash-kv")]
    #[test]
    fn emulated_rewrite_header_faults_do_not_commit_before_valid_marker() {
        let fuel = [0x11; FUEL_PAGE_LEN];
        let ign = [0x22; IGN_PAGE_LEN];
        let angles = [0x33; ANGLES_PAGE_LEN];
        let header = encode_rewrite_header(7, &fuel, &ign, &angles);

        let erase_failed_header = [0xFFu8; PAGE_HEADER_LEN];
        let program_failed_header = emulate_rewrite_header_until_commit(header, Some(4));
        let programmed_header = emulate_rewrite_header_until_commit(header, None);
        let mut committed_header = programmed_header;
        committed_header[6] = 0;
        committed_header[7] = 0;

        assert!(!emulated_header_valid(&erase_failed_header));
        assert!(!emulated_header_valid(&program_failed_header));
        assert!(!emulated_header_valid(&programmed_header));
        assert!(emulated_header_valid(&committed_header));
    }

    #[cfg(feature = "flash-kv")]
    #[test]
    fn target_sector_selection_uses_newest_valid_sector() {
        let invalid = KvHeader {
            ok: false,
            seq: 0,
            fuel_len: 0,
            ign_len: 0,
            angles_len: 0,
            fuel_crc: 0,
            ign_crc: 0,
            angles_crc: 0,
        };
        let valid_a = KvHeader {
            ok: true,
            seq: 7,
            ..invalid
        };
        let valid_b = KvHeader {
            ok: true,
            seq: 8,
            ..invalid
        };
        let wrapped_newer = KvHeader {
            ok: true,
            seq: 1,
            ..invalid
        };
        let wrapped_older = KvHeader {
            ok: true,
            seq: 0xFFFE,
            ..invalid
        };

        assert_eq!(
            target_sector_for_next_write(invalid, invalid),
            ActiveSector::A
        );
        assert_eq!(
            target_sector_for_next_write(valid_a, invalid),
            ActiveSector::B
        );
        assert_eq!(
            target_sector_for_next_write(invalid, valid_b),
            ActiveSector::A
        );
        assert_eq!(
            target_sector_for_next_write(valid_a, valid_b),
            ActiveSector::A
        );
        assert_eq!(
            newest_sector(wrapped_older, wrapped_newer),
            Some(ActiveSector::B)
        );
        assert_eq!(
            target_sector_for_next_write(wrapped_older, wrapped_newer),
            ActiveSector::A
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    struct EmulatedRewrite {
        fuel: [u8; FUEL_PAGE_LEN],
        ign: [u8; IGN_PAGE_LEN],
        angles: [u8; ANGLES_PAGE_LEN],
        key_record: [u8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES],
        key_audit: [u8; OBD2_IDENTITY_KEY_AUDIT_BYTES],
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    fn emulated_rewrite(
        pages: FlashPageImages,
        key_record: Obd2IdentityProvisioningKeyRecord,
        key_audit: Obd2IdentityProvisioningKeyAudit,
    ) -> EmulatedRewrite {
        let (_, _, _, key_record, key_audit) = prepare_obd2_identity_key_audit_rewrite(
            pages.clone(),
            None,
            None,
            None,
            Some(key_record),
            key_audit,
        );
        EmulatedRewrite {
            fuel: pages.fuel,
            ign: pages.ign,
            angles: pages.angles,
            key_record: key_record.expect("key record bytes"),
            key_audit,
        }
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    fn program_halfwords_with_failure(
        sector: &mut [u8],
        offset: usize,
        bytes: &[u8],
        fail_after_halfwords: Option<usize>,
        programmed: &mut usize,
    ) -> bool {
        for i in (0..bytes.len()).step_by(2) {
            if fail_after_halfwords == Some(*programmed) {
                return false;
            }
            sector[offset + i] = bytes[i];
            sector[offset + i + 1] = bytes[i + 1];
            *programmed += 1;
        }
        true
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    fn emulate_full_rewrite(
        seq: u16,
        rewrite: &EmulatedRewrite,
        fail_after_halfwords: Option<usize>,
    ) -> Vec<u8> {
        let mut sector = vec![0xFFu8; FlashLayout::SECTOR_BYTES];
        let header = encode_rewrite_header(seq, &rewrite.fuel, &rewrite.ign, &rewrite.angles);
        let mut programmed = 0usize;
        let chunks = [
            (0usize, header.as_slice()),
            (STM32F405_TS_KV_LAYOUT.fuel.offset, rewrite.fuel.as_slice()),
            (STM32F405_TS_KV_LAYOUT.ign.offset, rewrite.ign.as_slice()),
            (
                STM32F405_TS_KV_LAYOUT.angles.offset,
                rewrite.angles.as_slice(),
            ),
            (
                OBD2_IDENTITY_PROVISIONING_KEY_OFFSET,
                rewrite.key_record.as_slice(),
            ),
            (OBD2_IDENTITY_KEY_AUDIT_OFFSET, rewrite.key_audit.as_slice()),
        ];
        for (offset, bytes) in chunks {
            if !program_halfwords_with_failure(
                &mut sector,
                offset,
                bytes,
                fail_after_halfwords,
                &mut programmed,
            ) {
                return sector;
            }
        }
        program_halfwords_with_failure(
            &mut sector,
            6,
            &[0, 0],
            fail_after_halfwords,
            &mut programmed,
        );
        sector
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    fn decode_emulated_sector_header(sector: &[u8]) -> KvHeader {
        let mut header = [0xFFu8; PAGE_HEADER_LEN];
        header.copy_from_slice(&sector[..PAGE_HEADER_LEN]);
        decode_emulated_header(&header)
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    fn decode_emulated_key_record(sector: &[u8]) -> Option<Obd2IdentityProvisioningKeyRecord> {
        let mut bytes = [0xFFu8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES];
        bytes.copy_from_slice(
            &sector[OBD2_IDENTITY_PROVISIONING_KEY_OFFSET
                ..OBD2_IDENTITY_PROVISIONING_KEY_OFFSET + OBD2_IDENTITY_PROVISIONING_KEY_BYTES],
        );
        decode_obd2_identity_provisioning_key_record(&bytes)
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    fn decode_emulated_key_audit(sector: &[u8]) -> Option<Obd2IdentityProvisioningKeyAudit> {
        let mut bytes = [0xFFu8; OBD2_IDENTITY_KEY_AUDIT_BYTES];
        bytes.copy_from_slice(
            &sector[OBD2_IDENTITY_KEY_AUDIT_OFFSET
                ..OBD2_IDENTITY_KEY_AUDIT_OFFSET + OBD2_IDENTITY_KEY_AUDIT_BYTES],
        );
        decode_obd2_identity_key_audit(&bytes)
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn emulated_sidecar_failure_keeps_old_sector_current_until_new_sector_commits() {
        let old_pages = FlashPageImages {
            fuel: [0x11; FUEL_PAGE_LEN],
            ign: [0x22; IGN_PAGE_LEN],
            angles: [0x33; ANGLES_PAGE_LEN],
        };
        let new_pages = FlashPageImages {
            fuel: [0x44; FUEL_PAGE_LEN],
            ign: [0x55; IGN_PAGE_LEN],
            angles: [0x66; ANGLES_PAGE_LEN],
        };
        let old_key = Obd2IdentityProvisioningKeyRecord::active([0xA5; 32], 4);
        let new_key = Obd2IdentityProvisioningKeyRecord::active([0x5A; 32], 5);
        let old_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 10,
            authorized: true,
            accepted: true,
            store_failed: false,
            rejected_reason: 0,
            generation: 4,
        };
        let new_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 11,
            authorized: true,
            accepted: true,
            store_failed: false,
            rejected_reason: 0,
            generation: 5,
        };
        let old_rewrite = emulated_rewrite(old_pages, old_key, old_audit);
        let new_rewrite = emulated_rewrite(new_pages, new_key, new_audit);

        let old_sector = emulate_full_rewrite(1, &old_rewrite, None);
        let key_audit_start_halfword = (PAGE_HEADER_LEN
            + FUEL_PAGE_LEN
            + IGN_PAGE_LEN
            + ANGLES_PAGE_LEN
            + OBD2_IDENTITY_PROVISIONING_KEY_BYTES)
            / 2;
        let failed_new_sector =
            emulate_full_rewrite(2, &new_rewrite, Some(key_audit_start_halfword + 1));
        let committed_new_sector = emulate_full_rewrite(2, &new_rewrite, None);

        let old_header = decode_emulated_sector_header(&old_sector);
        let failed_header = decode_emulated_sector_header(&failed_new_sector);
        let committed_header = decode_emulated_sector_header(&committed_new_sector);

        assert_eq!(
            newest_sector(old_header, failed_header),
            Some(ActiveSector::A)
        );
        assert_eq!(
            target_sector_for_next_write(old_header, failed_header),
            ActiveSector::B
        );
        assert_eq!(decode_emulated_key_record(&old_sector), Some(old_key));
        assert_eq!(decode_emulated_key_audit(&old_sector), Some(old_audit));
        assert_eq!(
            validate_page_image(
                STM32F405_TS_KV_LAYOUT,
                old_header,
                FlashPageKey::Fuel,
                &old_rewrite.fuel
            ),
            true
        );
        assert_eq!(decode_emulated_key_audit(&failed_new_sector), None);

        assert_eq!(
            newest_sector(old_header, committed_header),
            Some(ActiveSector::B)
        );
        assert_eq!(
            decode_emulated_key_record(&committed_new_sector),
            Some(new_key)
        );
        assert_eq!(
            decode_emulated_key_audit(&committed_new_sector),
            Some(new_audit)
        );
    }

    #[cfg(feature = "transport-can")]
    fn sample_obd2_snapshot() -> Obd2RetainedDiagnosticHistorySnapshot {
        Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: Message::SensorData {
                map_kpa_x10: 940,
                tps_percent: 24,
                iat_offset: 65,
                clt_offset: 72,
                voltage_x10: 138,
                lambda_x100: 101,
                flags: 0,
                timestamp_us: 12_345,
            },
            freeze_frame_value_source: Some(Message::SensorData {
                map_kpa_x10: 880,
                tps_percent: 18,
                iat_offset: 63,
                clt_offset: 70,
                voltage_x10: 136,
                lambda_x100: 99,
                flags: 0,
                timestamp_us: 11_111,
            }),
            stored_dtcs: [
                DiagCode::PersistCrcFault,
                DiagCode::LowVoltage,
                DiagCode::MapRange,
            ],
            stored_dtc_count: 2,
            freeze_frame_dtc: Some(DiagCode::PersistCrcFault),
            current_diag_event: Some(DiagEvent {
                code: DiagCode::PersistCrcFault,
                timestamp: Micros::new(2_000),
                source: DiagSource::User,
                context: Some(7),
                start_us: 1_000,
                end_us: 2_000,
            }),
            freeze_frame_event: Some(DiagEvent {
                code: DiagCode::LowVoltage,
                timestamp: Micros::new(3_000),
                source: DiagSource::Sensor,
                context: Some(12_300),
                start_us: 2_500,
                end_us: 3_000,
            }),
            last_fault: FaultCode::CalibrationInvalid,
            last_observed_diag_code: Some(DiagCode::PersistCrcFault),
            last_observed_diag_timestamp_us: 4_000,
        }
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_snapshot_sidecar_bytes_roundtrip_preserves_retained_history_snapshot() {
        let snapshot = sample_obd2_snapshot();

        let mut bytes = [0u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES];
        assert!(install_obd2_retained_history_snapshot_halfwords_with(
            &snapshot,
            |offset, value| {
                bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
            }
        ));
        assert_eq!(
            decode_obd2_retained_history_sidecar_at(&bytes, 0),
            Some(snapshot)
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_flash_record_roundtrip_preserves_provisioned_identity() {
        let record = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000001"),
            Some(b"flashcal00000001"),
            Some(b"stm4b1"),
        );

        assert_eq!(
            decode_obd2_identity_record(&encode_obd2_identity_record(record)),
            Some(record)
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_flash_record_decode_rejects_blank_wrong_version_crc_and_lengths() {
        let record = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000002"),
            Some(b"flashcal00000002"),
            Some(b"stm4b2"),
        );
        let blank = [0xFFu8; OBD2_IDENTITY_RECORD_BYTES];
        assert_eq!(decode_obd2_identity_record(&blank), None);

        let mut wrong_magic = encode_obd2_identity_record(record);
        wrong_magic[0] ^= 0x01;
        assert_eq!(decode_obd2_identity_record(&wrong_magic), None);

        let mut wrong_version = encode_obd2_identity_record(record);
        wrong_version[4..6].copy_from_slice(&2u16.to_le_bytes());
        assert_eq!(decode_obd2_identity_record(&wrong_version), None);

        let mut wrong_crc = encode_obd2_identity_record(record);
        wrong_crc[9] ^= 0x01;
        assert_eq!(decode_obd2_identity_record(&wrong_crc), None);

        let mut over_length = encode_obd2_identity_record(record);
        over_length[6] = ecu_transport::CAN_OBD2_VIN_LEN as u8 + 1;
        let crc = crc16(&over_length[..50]);
        over_length[50..52].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_obd2_identity_record(&over_length), None);

        let mut over_calibration_id_length = encode_obd2_identity_record(record);
        over_calibration_id_length[7] = ecu_transport::CAN_OBD2_VIN_LEN as u8 + 1;
        let crc = crc16(&over_calibration_id_length[..50]);
        over_calibration_id_length[50..52].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            decode_obd2_identity_record(&over_calibration_id_length),
            None
        );

        let mut over_board_build_identity_length = encode_obd2_identity_record(record);
        over_board_build_identity_length[8] = 7;
        let crc = crc16(&over_board_build_identity_length[..50]);
        over_board_build_identity_length[50..52].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            decode_obd2_identity_record(&over_board_build_identity_length),
            None
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_flash_record_read_helper_loads_valid_record_and_rejects_corrupt_record() {
        let record = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000004"),
            Some(b"flashcal00000004"),
            Some(b"stm4b4"),
        );
        let encoded = encode_obd2_identity_record(record);
        assert_eq!(
            read_obd2_identity_record_with(|out| out.copy_from_slice(&encoded)),
            Some(record)
        );

        let mut corrupt = encoded;
        corrupt[10] ^= 0x55;
        assert_eq!(
            read_obd2_identity_record_with(|out| out.copy_from_slice(&corrupt)),
            None
        );
    }

    #[cfg(feature = "transport-can")]
    fn sample_obd2_identity_command_audit() -> Obd2IdentityProvisioningCommandAudit {
        Obd2IdentityProvisioningCommandAudit {
            request_id: 77,
            authorized: true,
            authorization_failed: false,
            provisioning_status: Some(Obd2IdentityProvisioningStatus {
                attempted: true,
                accepted: true,
                rejected_field: None,
                store_failed: false,
                normalized_lengths: Some(Obd2IdentityProvisioningLengths {
                    vin: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                    calibration_id: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                    board_build_identity: 6,
                }),
            }),
        }
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_command_audit_record_roundtrip_preserves_status() {
        let audit = sample_obd2_identity_command_audit();

        assert_eq!(
            decode_obd2_identity_command_audit(&encode_obd2_identity_command_audit(audit)),
            Some(audit)
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_command_audit_decode_rejects_blank_wrong_version_and_crc() {
        let audit = sample_obd2_identity_command_audit();
        let blank = [0xFFu8; OBD2_IDENTITY_COMMAND_AUDIT_BYTES];
        assert_eq!(decode_obd2_identity_command_audit(&blank), None);

        let mut wrong_magic = encode_obd2_identity_command_audit(audit);
        wrong_magic[0] ^= 0x01;
        assert_eq!(decode_obd2_identity_command_audit(&wrong_magic), None);

        let mut wrong_version = encode_obd2_identity_command_audit(audit);
        wrong_version[4..6].copy_from_slice(&2u16.to_le_bytes());
        assert_eq!(decode_obd2_identity_command_audit(&wrong_version), None);

        let mut wrong_crc = encode_obd2_identity_command_audit(audit);
        wrong_crc[6] ^= 0x01;
        assert_eq!(decode_obd2_identity_command_audit(&wrong_crc), None);
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_command_audit_read_helper_loads_valid_record() {
        let audit = sample_obd2_identity_command_audit();
        let encoded = encode_obd2_identity_command_audit(audit);

        assert_eq!(
            read_obd2_identity_command_audit_with(|out| out.copy_from_slice(&encoded)),
            Some(audit)
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_provisioning_key_record_roundtrip_preserves_lifecycle_state() {
        let active =
            Obd2IdentityProvisioningKeyRecord::active([0xA5; 32], 42).with_last_arm_nonce(99);
        let revoked = Obd2IdentityProvisioningKeyRecord::revoked([0x5A; 32], 43);

        assert_eq!(
            decode_obd2_identity_provisioning_key_record(
                &encode_obd2_identity_provisioning_key_record(active)
            ),
            Some(active)
        );
        assert_eq!(active.active_key(), Some([0xA5; 32]));
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(
                &encode_obd2_identity_provisioning_key_record(revoked)
            ),
            Some(revoked)
        );
        assert_eq!(revoked.active_key(), None);
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_provisioning_key_record_decode_rejects_blank_wrong_flags_version_and_crc() {
        let record = Obd2IdentityProvisioningKeyRecord::active([0x11; 32], 7);
        let blank = [0xFFu8; OBD2_IDENTITY_PROVISIONING_KEY_BYTES];
        assert_eq!(decode_obd2_identity_provisioning_key_record(&blank), None);

        let mut wrong_magic = encode_obd2_identity_provisioning_key_record(record);
        wrong_magic[0] ^= 0x01;
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(&wrong_magic),
            None
        );

        let mut wrong_version = encode_obd2_identity_provisioning_key_record(record);
        wrong_version[4..6].copy_from_slice(&2u16.to_le_bytes());
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(&wrong_version),
            None
        );

        let mut invalid_flags = encode_obd2_identity_provisioning_key_record(record);
        invalid_flags[6] =
            OBD2_IDENTITY_PROVISIONING_KEY_ACTIVE | OBD2_IDENTITY_PROVISIONING_KEY_REVOKED;
        let crc = crc16(&invalid_flags[..48]);
        invalid_flags[48..50].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(&invalid_flags),
            None
        );

        let mut missing_lifecycle_flag = encode_obd2_identity_provisioning_key_record(record);
        missing_lifecycle_flag[6] = 0;
        let crc = crc16(&missing_lifecycle_flag[..48]);
        missing_lifecycle_flag[48..50].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(&missing_lifecycle_flag),
            None
        );

        let mut wrong_crc = encode_obd2_identity_provisioning_key_record(record);
        wrong_crc[12] ^= 0x01;
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(&wrong_crc),
            None
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_provisioning_key_read_helper_loads_valid_record() {
        let record = Obd2IdentityProvisioningKeyRecord::active([0x22; 32], 9);
        let encoded = encode_obd2_identity_provisioning_key_record(record);

        assert_eq!(
            read_obd2_identity_provisioning_key_record_with(|out| {
                out.copy_from_slice(&encoded)
            }),
            Some(record)
        );
    }

    #[cfg(feature = "transport-can")]
    fn sample_obd2_identity_key_audit() -> Obd2IdentityProvisioningKeyAudit {
        Obd2IdentityProvisioningKeyAudit {
            request_id: 88,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason:
                crate::identity_provisioning::OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE,
            generation: 12,
        }
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_key_audit_roundtrip_preserves_lifecycle_status() {
        let audit = sample_obd2_identity_key_audit();

        assert_eq!(
            decode_obd2_identity_key_audit(&encode_obd2_identity_key_audit(audit)),
            Some(audit)
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn obd2_identity_key_audit_decode_rejects_blank_wrong_version_flags_and_crc() {
        let audit = sample_obd2_identity_key_audit();
        let blank = [0xFFu8; OBD2_IDENTITY_KEY_AUDIT_BYTES];
        assert_eq!(decode_obd2_identity_key_audit(&blank), None);

        let mut wrong_magic = encode_obd2_identity_key_audit(audit);
        wrong_magic[0] ^= 0x01;
        assert_eq!(decode_obd2_identity_key_audit(&wrong_magic), None);

        let mut wrong_version = encode_obd2_identity_key_audit(audit);
        wrong_version[4..6].copy_from_slice(&2u16.to_le_bytes());
        assert_eq!(decode_obd2_identity_key_audit(&wrong_version), None);

        let mut invalid_flags = encode_obd2_identity_key_audit(audit);
        invalid_flags[10] = 0x08;
        let crc = crc16(&invalid_flags[..20]);
        invalid_flags[20..22].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_obd2_identity_key_audit(&invalid_flags), None);

        let mut wrong_crc = encode_obd2_identity_key_audit(audit);
        wrong_crc[12] ^= 0x01;
        assert_eq!(decode_obd2_identity_key_audit(&wrong_crc), None);
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_key_audit_read_helper_loads_valid_record() {
        let audit = sample_obd2_identity_key_audit();
        let encoded = encode_obd2_identity_key_audit(audit);

        assert_eq!(
            read_obd2_identity_key_audit_with(|out| out.copy_from_slice(&encoded)),
            Some(audit)
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_record_write_staging_preserves_pages_and_retained_history() {
        let pages = FlashPageImages {
            fuel: [0x11; FUEL_PAGE_LEN],
            ign: [0x22; IGN_PAGE_LEN],
            angles: [0x33; ANGLES_PAGE_LEN],
        };
        let snapshot = Some(sample_obd2_snapshot());
        let record = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000005"),
            Some(b"flashcal00000005"),
            Some(b"stm4b5"),
        );

        let (rewrite, identity_bytes) =
            prepare_obd2_identity_record_rewrite(pages.clone(), snapshot.clone(), record)
                .expect("valid identity record should stage");

        assert_eq!(rewrite.pages.fuel, pages.fuel);
        assert_eq!(rewrite.pages.ign, pages.ign);
        assert_eq!(rewrite.pages.angles, pages.angles);
        assert_eq!(rewrite.snapshot, snapshot);
        assert_eq!(decode_obd2_identity_record(&identity_bytes), Some(record));
        assert_eq!(
            read_obd2_identity_record_with(|out| out.copy_from_slice(&identity_bytes)),
            Some(record)
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_command_audit_write_staging_preserves_pages_snapshot_and_identity() {
        let pages = FlashPageImages {
            fuel: [0x11; FUEL_PAGE_LEN],
            ign: [0x22; IGN_PAGE_LEN],
            angles: [0x33; ANGLES_PAGE_LEN],
        };
        let snapshot = Some(sample_obd2_snapshot());
        let identity = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000007"),
            Some(b"flashcal00000007"),
            Some(b"stm4b7"),
        );
        let audit = sample_obd2_identity_command_audit();

        let (rewrite, identity_bytes, audit_bytes) = prepare_obd2_identity_command_audit_rewrite(
            pages.clone(),
            snapshot.clone(),
            Some(identity),
            audit,
        );

        assert_eq!(rewrite.pages.fuel, pages.fuel);
        assert_eq!(rewrite.pages.ign, pages.ign);
        assert_eq!(rewrite.pages.angles, pages.angles);
        assert_eq!(rewrite.snapshot, snapshot);
        assert_eq!(
            identity_bytes.and_then(|bytes| decode_obd2_identity_record(&bytes)),
            Some(identity)
        );
        assert_eq!(
            decode_obd2_identity_command_audit(&audit_bytes),
            Some(audit)
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_key_write_staging_preserves_pages_snapshot_identity_and_audit() {
        let pages = FlashPageImages {
            fuel: [0x11; FUEL_PAGE_LEN],
            ign: [0x22; IGN_PAGE_LEN],
            angles: [0x33; ANGLES_PAGE_LEN],
        };
        let snapshot = Some(sample_obd2_snapshot());
        let identity = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000008"),
            Some(b"flashcal00000008"),
            Some(b"stm4b8"),
        );
        let audit = sample_obd2_identity_command_audit();
        let key_record = Obd2IdentityProvisioningKeyRecord::active([0x44; 32], 12);

        let (rewrite, identity_bytes, audit_bytes, key_bytes) =
            prepare_obd2_identity_provisioning_key_rewrite(
                pages.clone(),
                snapshot.clone(),
                Some(identity),
                Some(audit),
                key_record,
            );

        assert_eq!(rewrite.pages.fuel, pages.fuel);
        assert_eq!(rewrite.pages.ign, pages.ign);
        assert_eq!(rewrite.pages.angles, pages.angles);
        assert_eq!(rewrite.snapshot, snapshot);
        assert_eq!(
            identity_bytes.and_then(|bytes| decode_obd2_identity_record(&bytes)),
            Some(identity)
        );
        assert_eq!(
            audit_bytes.and_then(|bytes| decode_obd2_identity_command_audit(&bytes)),
            Some(audit)
        );
        assert_eq!(
            decode_obd2_identity_provisioning_key_record(&key_bytes),
            Some(key_record)
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_key_audit_write_staging_preserves_all_sidecars() {
        let pages = FlashPageImages {
            fuel: [0x11; FUEL_PAGE_LEN],
            ign: [0x22; IGN_PAGE_LEN],
            angles: [0x33; ANGLES_PAGE_LEN],
        };
        let snapshot = Some(sample_obd2_snapshot());
        let identity = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000009"),
            Some(b"flashcal00000009"),
            Some(b"stm4b9"),
        );
        let audit = sample_obd2_identity_command_audit();
        let key_record = Obd2IdentityProvisioningKeyRecord::active([0x45; 32], 13);
        let key_audit = sample_obd2_identity_key_audit();

        let (rewrite, identity_bytes, audit_bytes, key_bytes, key_audit_bytes) =
            prepare_obd2_identity_key_audit_rewrite(
                pages.clone(),
                snapshot.clone(),
                Some(identity),
                Some(audit),
                Some(key_record),
                key_audit,
            );

        assert_eq!(rewrite.pages.fuel, pages.fuel);
        assert_eq!(rewrite.pages.ign, pages.ign);
        assert_eq!(rewrite.pages.angles, pages.angles);
        assert_eq!(rewrite.snapshot, snapshot);
        assert_eq!(
            identity_bytes.and_then(|bytes| decode_obd2_identity_record(&bytes)),
            Some(identity)
        );
        assert_eq!(
            audit_bytes.and_then(|bytes| decode_obd2_identity_command_audit(&bytes)),
            Some(audit)
        );
        assert_eq!(
            key_bytes.and_then(|bytes| decode_obd2_identity_provisioning_key_record(&bytes)),
            Some(key_record)
        );
        assert_eq!(
            decode_obd2_identity_key_audit(&key_audit_bytes),
            Some(key_audit)
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_key_audit_readback_gate_rejects_key_or_audit_mismatch() {
        let key_record = Obd2IdentityProvisioningKeyRecord::active([0x61; 32], 31);
        let key_audit = sample_obd2_identity_key_audit();
        let wrong_key_record = Obd2IdentityProvisioningKeyRecord::active([0x62; 32], 31);
        let wrong_key_audit = Obd2IdentityProvisioningKeyAudit {
            generation: key_audit.generation.wrapping_add(1),
            ..key_audit
        };

        assert_eq!(
            verify_obd2_identity_key_audit_readback(
                Some(key_record),
                key_audit,
                || Some(key_record),
                || Some(key_audit),
            ),
            Ok(())
        );
        assert_eq!(
            verify_obd2_identity_key_audit_readback(
                Some(key_record),
                key_audit,
                || Some(wrong_key_record),
                || Some(key_audit),
            ),
            Err(KvError::Io)
        );
        assert_eq!(
            verify_obd2_identity_key_audit_readback(
                Some(key_record),
                key_audit,
                || Some(key_record),
                || Some(wrong_key_audit),
            ),
            Err(KvError::Io)
        );
        assert_eq!(
            verify_obd2_identity_key_audit_readback(None, key_audit, || None, || Some(key_audit)),
            Ok(())
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_key_and_audit_atomic_staging_preserves_all_sidecars() {
        let pages = FlashPageImages {
            fuel: [0x41; FUEL_PAGE_LEN],
            ign: [0x42; IGN_PAGE_LEN],
            angles: [0x43; ANGLES_PAGE_LEN],
        };
        let snapshot = Some(sample_obd2_snapshot());
        let identity = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000010"),
            Some(b"flashcal00000010"),
            Some(b"stm410"),
        );
        let audit = sample_obd2_identity_command_audit();
        let key_record = Obd2IdentityProvisioningKeyRecord::active([0x51; 32], 21);
        let key_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 0x3333_4444,
            authorized: true,
            accepted: true,
            store_failed: false,
            rejected_reason:
                crate::identity_provisioning::OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE,
            generation: key_record.generation,
        };

        let (rewrite, identity_bytes, audit_bytes, key_bytes, key_audit_bytes) =
            prepare_obd2_identity_key_audit_rewrite(
                pages.clone(),
                snapshot.clone(),
                Some(identity),
                Some(audit),
                Some(key_record),
                key_audit,
            );

        assert_eq!(rewrite.pages.fuel, pages.fuel);
        assert_eq!(rewrite.pages.ign, pages.ign);
        assert_eq!(rewrite.pages.angles, pages.angles);
        assert_eq!(rewrite.snapshot, snapshot);
        assert_eq!(
            identity_bytes.and_then(|bytes| decode_obd2_identity_record(&bytes)),
            Some(identity)
        );
        assert_eq!(
            audit_bytes.and_then(|bytes| decode_obd2_identity_command_audit(&bytes)),
            Some(audit)
        );
        assert_eq!(
            key_bytes.and_then(|bytes| decode_obd2_identity_provisioning_key_record(&bytes)),
            Some(key_record)
        );
        assert_eq!(
            decode_obd2_identity_key_audit(&key_audit_bytes),
            Some(key_audit)
        );
    }

    #[cfg(all(feature = "transport-can", feature = "flash-kv"))]
    #[test]
    fn obd2_identity_record_write_staging_rejects_invalid_lengths_before_rewrite() {
        let pages = FlashPageImages {
            fuel: [0x11; FUEL_PAGE_LEN],
            ign: [0x22; IGN_PAGE_LEN],
            angles: [0x33; ANGLES_PAGE_LEN],
        };
        let snapshot = Some(sample_obd2_snapshot());
        let mut record = Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"flashvin00000006"),
            Some(b"flashcal00000006"),
            Some(b"stm4b6"),
        );
        record.vin_len = ecu_transport::CAN_OBD2_VIN_LEN as u8 + 1;

        assert_eq!(
            prepare_obd2_identity_record_rewrite(pages, snapshot, record).map(|_| ()),
            Err(KvError::Io)
        );
    }

    #[cfg(feature = "transport-can")]
    #[test]
    fn decoded_obd2_identity_flash_record_reaches_mode09_dispatch_inputs() {
        let record = decode_obd2_identity_record(&encode_obd2_identity_record(
            Obd2ProvisionedIdentityRecord::from_ascii(
                Some(b"flashvin00000003"),
                Some(b"flashcal00000003"),
                Some(b"stm4b3"),
            ),
        ))
        .expect("encoded record should decode");
        let identity = ecu_target_common::transport_service::Obd2VehicleIdentity::from_signature(
            ecu_ts::TS_SIGNATURE,
        )
        .with_provisioned_identity(record.into_provisioned_identity());
        let inputs = ecu_transport::CanObd2VehicleInfoInputs {
            ecu_name_len: identity.ecu_name_len,
            ecu_name: identity.ecu_name,
            vin_len: identity.vin_len,
            vin: identity.vin,
            calibration_id_len: identity.calibration_id_len,
            calibration_id: identity.calibration_id,
            identity_key_lifecycle: None,
            flash_write_fault: None,
        };
        let vin = ecu_transport::CanObd2SegmentedVehicleInfoResponseSurface::assemble(
            &Message::Obd2Request {
                service: 0x09,
                parameter_id: Some(0x02),
                payload_len: 0,
                payload: [0; 6],
            },
            inputs,
        )
        .expect("decoded VIN should assemble");
        let calibration_id = ecu_transport::CanObd2SegmentedVehicleInfoResponseSurface::assemble(
            &Message::Obd2Request {
                service: 0x09,
                parameter_id: Some(0x04),
                payload_len: 0,
                payload: [0; 6],
            },
            inputs,
        )
        .expect("decoded calibration ID should assemble");

        assert_eq!(vin.info_type_id, 0x02);
        assert_eq!(
            vin.segments[0].as_ref(),
            Some(&Message::Obd2SegmentedResponse {
                service: 0x49,
                parameter_id: Some(0x02),
                sequence_index: 0,
                segment_count: 3,
                total_payload_len: 16,
                segment_len: 6,
                segment: *b"FLASHV",
            })
        );
        assert_eq!(calibration_id.info_type_id, 0x04);
        assert_eq!(
            calibration_id.segments[0].as_ref(),
            Some(&Message::Obd2SegmentedResponse {
                service: 0x49,
                parameter_id: Some(0x04),
                sequence_index: 0,
                segment_count: 3,
                total_payload_len: 16,
                segment_len: 6,
                segment: *b"FLASHC",
            })
        );
    }
}
