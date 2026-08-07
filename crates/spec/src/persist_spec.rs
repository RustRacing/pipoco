pub const PERSIST_SCHEMA_VERSION_CURRENT: u16 = 3;
pub const PERSIST_RECORD_HEADER_BYTES: usize = 6;
pub const PERSIST_RECORD_CRC_BYTES: usize = 4;
pub const PERSIST_FUEL_PAGE_BYTES: usize = 512;
pub const PERSIST_IGNITION_PAGE_BYTES: usize = 512;
pub const PERSIST_ANGLES_PAGE_BYTES: usize = 68;
pub const PERSIST_MAX_PAYLOAD_BYTES: usize = PERSIST_FUEL_PAGE_BYTES;
pub const PERSIST_RECORD_MAX_BYTES: usize =
    PERSIST_RECORD_HEADER_BYTES + PERSIST_MAX_PAYLOAD_BYTES + PERSIST_RECORD_CRC_BYTES;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PersistPageId {
    #[default]
    Fuel = 1,
    Ignition = 2,
    Angles = 3,
}

impl PersistPageId {
    pub const fn payload_len(self) -> usize {
        match self {
            Self::Fuel => PERSIST_FUEL_PAGE_BYTES,
            Self::Ignition => PERSIST_IGNITION_PAGE_BYTES,
            Self::Angles => PERSIST_ANGLES_PAGE_BYTES,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PersistDecodeError {
    UnknownPageId,
    UnsupportedSchemaVersion,
    InvalidPayloadLength,
    MalformedRecordLength,
    CrcMismatch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PersistEncodeError {
    UnsupportedSchemaVersion,
    InvalidPayloadLength,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PersistMigrationError {
    UnsupportedFromVersion,
    UnsupportedToVersion,
    UnsupportedMigrationPath,
    MalformedSourcePayload,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PersistPage {
    pub schema_version: u16,
    pub page_id: PersistPageId,
    pub payload_len: u16,
    pub payload: [u8; PERSIST_MAX_PAYLOAD_BYTES],
}

impl Default for PersistPage {
    fn default() -> Self {
        Self {
            schema_version: PERSIST_SCHEMA_VERSION_CURRENT,
            page_id: PersistPageId::Fuel,
            payload_len: PERSIST_FUEL_PAGE_BYTES as u16,
            payload: [0; PERSIST_MAX_PAYLOAD_BYTES],
        }
    }
}

impl PersistPage {
    pub fn new(
        schema_version: u16,
        page_id: PersistPageId,
        payload: &[u8],
    ) -> Result<Self, PersistEncodeError> {
        if !is_supported_schema_version(schema_version) {
            return Err(PersistEncodeError::UnsupportedSchemaVersion);
        }
        if payload.len() != page_id.payload_len() {
            return Err(PersistEncodeError::InvalidPayloadLength);
        }

        let mut page = Self {
            schema_version,
            page_id,
            payload_len: payload.len() as u16,
            payload: [0u8; PERSIST_MAX_PAYLOAD_BYTES],
        };
        let mut idx = 0usize;
        while idx < payload.len() {
            page.payload[idx] = payload[idx];
            idx += 1;
        }
        Ok(page)
    }

    pub fn payload_slice(&self) -> &[u8] {
        &self.payload[..self.payload_len as usize]
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EncodedPersistRecord {
    pub len: u16,
    pub bytes: [u8; PERSIST_RECORD_MAX_BYTES],
}

impl Default for EncodedPersistRecord {
    fn default() -> Self {
        Self {
            len: 0,
            bytes: [0; PERSIST_RECORD_MAX_BYTES],
        }
    }
}

pub fn persist_encode(page: &PersistPage) -> Result<EncodedPersistRecord, PersistEncodeError> {
    if !is_supported_schema_version(page.schema_version) {
        return Err(PersistEncodeError::UnsupportedSchemaVersion);
    }
    if page.payload_len as usize != page.page_id.payload_len() {
        return Err(PersistEncodeError::InvalidPayloadLength);
    }

    let payload_len = page.payload_len as usize;
    let encoded_len = PERSIST_RECORD_HEADER_BYTES + payload_len + PERSIST_RECORD_CRC_BYTES;
    let mut record = EncodedPersistRecord {
        len: encoded_len as u16,
        bytes: [0u8; PERSIST_RECORD_MAX_BYTES],
    };

    write_u16_le(&mut record.bytes[0..2], page.schema_version);
    write_u16_le(&mut record.bytes[2..4], page.page_id as u16);
    write_u16_le(&mut record.bytes[4..6], page.payload_len);

    match page.page_id {
        PersistPageId::Fuel => copy_payload_into_record_fixed::<PERSIST_FUEL_PAGE_BYTES>(
            &mut record.bytes,
            &page.payload,
        ),
        PersistPageId::Ignition => copy_payload_into_record_fixed::<PERSIST_IGNITION_PAGE_BYTES>(
            &mut record.bytes,
            &page.payload,
        ),
        PersistPageId::Angles => copy_payload_into_record_fixed::<PERSIST_ANGLES_PAGE_BYTES>(
            &mut record.bytes,
            &page.payload,
        ),
    }

    let crc_offset = PERSIST_RECORD_HEADER_BYTES + payload_len;
    let crc = crc32c(&record.bytes[..crc_offset]);
    write_u32_le(&mut record.bytes[crc_offset..crc_offset + 4], crc);
    Ok(record)
}

pub fn persist_decode(record: &[u8]) -> Result<PersistPage, PersistDecodeError> {
    if record.len() < PERSIST_RECORD_HEADER_BYTES + PERSIST_RECORD_CRC_BYTES {
        return Err(PersistDecodeError::MalformedRecordLength);
    }

    let schema_version = read_u16_le(&record[0..2]);
    if !is_supported_schema_version(schema_version) {
        return Err(PersistDecodeError::UnsupportedSchemaVersion);
    }

    let page_id = match read_u16_le(&record[2..4]) {
        1 => PersistPageId::Fuel,
        2 => PersistPageId::Ignition,
        3 => PersistPageId::Angles,
        _ => return Err(PersistDecodeError::UnknownPageId),
    };
    let payload_len = read_u16_le(&record[4..6]) as usize;
    if payload_len != page_id.payload_len() {
        return Err(PersistDecodeError::InvalidPayloadLength);
    }

    let expected_len = PERSIST_RECORD_HEADER_BYTES + payload_len + PERSIST_RECORD_CRC_BYTES;
    if record.len() != expected_len {
        return Err(PersistDecodeError::MalformedRecordLength);
    }

    let crc_offset = PERSIST_RECORD_HEADER_BYTES + payload_len;
    let expected_crc = crc32c(&record[..crc_offset]);
    let stored_crc = read_u32_le(&record[crc_offset..crc_offset + 4]);
    if expected_crc != stored_crc {
        return Err(PersistDecodeError::CrcMismatch);
    }

    let mut page = PersistPage {
        schema_version,
        page_id,
        payload_len: payload_len as u16,
        payload: [0u8; PERSIST_MAX_PAYLOAD_BYTES],
    };
    match page_id {
        PersistPageId::Fuel => {
            copy_payload_from_record_fixed::<PERSIST_FUEL_PAGE_BYTES>(&mut page.payload, record)
        }
        PersistPageId::Ignition => {
            copy_payload_from_record_fixed::<PERSIST_IGNITION_PAGE_BYTES>(&mut page.payload, record)
        }
        PersistPageId::Angles => {
            copy_payload_from_record_fixed::<PERSIST_ANGLES_PAGE_BYTES>(&mut page.payload, record)
        }
    }
    Ok(page)
}

pub fn persist_migrate(
    page_id: PersistPageId,
    from_version: u16,
    to_version: u16,
    source_payload: &[u8],
) -> Result<PersistPage, PersistMigrationError> {
    if !is_supported_schema_version(from_version) {
        return Err(PersistMigrationError::UnsupportedFromVersion);
    }
    if !is_supported_schema_version(to_version) {
        return Err(PersistMigrationError::UnsupportedToVersion);
    }
    if source_payload.len() != required_payload_len(page_id, from_version) {
        return Err(PersistMigrationError::MalformedSourcePayload);
    }

    if from_version == to_version {
        return persist_page_from_payload(page_id, to_version, source_payload);
    }

    match (from_version, to_version) {
        (1, 2) => migrate_v1_to_v2(page_id, source_payload),
        (2, 3) => migrate_v2_to_v3(page_id, source_payload),
        (1, 3) => {
            let v2_page = migrate_v1_to_v2(page_id, source_payload)?;
            migrate_v2_to_v3(page_id, v2_page.payload_slice())
        }
        _ => Err(PersistMigrationError::UnsupportedMigrationPath),
    }
}

pub fn factory_reset(page_bytes: &[u8]) -> Result<EncodedPersistRecord, PersistDecodeError> {
    let decoded = persist_decode(page_bytes)?;
    let reset_page = persist_factory_reset_page(decoded.page_id);
    match persist_encode(&reset_page) {
        Ok(record) => Ok(record),
        Err(_) => Err(PersistDecodeError::InvalidPayloadLength),
    }
}

const fn is_supported_schema_version(schema_version: u16) -> bool {
    schema_version >= 1 && schema_version <= PERSIST_SCHEMA_VERSION_CURRENT
}

const fn required_payload_len(page_id: PersistPageId, _schema_version: u16) -> usize {
    page_id.payload_len()
}

fn persist_page_from_payload(
    page_id: PersistPageId,
    schema_version: u16,
    payload: &[u8],
) -> Result<PersistPage, PersistMigrationError> {
    if payload.len() != page_id.payload_len() {
        return Err(PersistMigrationError::MalformedSourcePayload);
    }

    let mut page = PersistPage {
        schema_version,
        page_id,
        payload_len: page_id.payload_len() as u16,
        payload: [0u8; PERSIST_MAX_PAYLOAD_BYTES],
    };
    let mut idx = 0usize;
    while idx < payload.len() {
        page.payload[idx] = payload[idx];
        idx += 1;
    }
    Ok(page)
}

fn migrate_v1_to_v2(
    page_id: PersistPageId,
    source_payload: &[u8],
) -> Result<PersistPage, PersistMigrationError> {
    let payload_len = page_id.payload_len();
    if source_payload.len() != payload_len {
        return Err(PersistMigrationError::MalformedSourcePayload);
    }

    let mut page = PersistPage {
        schema_version: 2,
        page_id,
        payload_len: payload_len as u16,
        payload: [0u8; PERSIST_MAX_PAYLOAD_BYTES],
    };

    let mut idx = 0usize;
    while idx < payload_len {
        page.payload[idx] = source_payload[idx];
        idx += 1;
    }
    Ok(page)
}

fn migrate_v2_to_v3(
    page_id: PersistPageId,
    source_payload: &[u8],
) -> Result<PersistPage, PersistMigrationError> {
    let payload_len = page_id.payload_len();
    if source_payload.len() != payload_len {
        return Err(PersistMigrationError::MalformedSourcePayload);
    }

    let mut page = PersistPage {
        schema_version: 3,
        page_id,
        payload_len: payload_len as u16,
        payload: canonical_defaults(page_id),
    };

    let mut idx = 0usize;
    while idx < payload_len {
        page.payload[idx] = source_payload[idx];
        idx += 1;
    }
    Ok(page)
}

const fn canonical_defaults(_page_id: PersistPageId) -> [u8; PERSIST_MAX_PAYLOAD_BYTES] {
    [0u8; PERSIST_MAX_PAYLOAD_BYTES]
}

fn persist_factory_reset_page(page_id: PersistPageId) -> PersistPage {
    PersistPage {
        schema_version: PERSIST_SCHEMA_VERSION_CURRENT,
        page_id,
        payload_len: page_id.payload_len() as u16,
        payload: canonical_defaults(page_id),
    }
}

fn read_u16_le(bytes: &[u8]) -> u16 {
    (bytes[0] as u16) | ((bytes[1] as u16) << 8)
}

fn read_u32_le(bytes: &[u8]) -> u32 {
    (bytes[0] as u32)
        | ((bytes[1] as u32) << 8)
        | ((bytes[2] as u32) << 16)
        | ((bytes[3] as u32) << 24)
}

fn write_u16_le(bytes: &mut [u8], value: u16) {
    bytes[0] = (value & 0x00ff) as u8;
    bytes[1] = (value >> 8) as u8;
}

fn write_u32_le(bytes: &mut [u8], value: u32) {
    bytes[0] = (value & 0x0000_00ff) as u8;
    bytes[1] = ((value >> 8) & 0x0000_00ff) as u8;
    bytes[2] = ((value >> 16) & 0x0000_00ff) as u8;
    bytes[3] = ((value >> 24) & 0x0000_00ff) as u8;
}

fn crc32c(bytes: &[u8]) -> u32 {
    let mut crc = 0xffff_ffffu32;
    let mut idx = 0usize;
    while idx + 18 <= bytes.len() {
        crc = crc32c_step(crc, bytes[idx]);
        crc = crc32c_step(crc, bytes[idx + 1]);
        crc = crc32c_step(crc, bytes[idx + 2]);
        crc = crc32c_step(crc, bytes[idx + 3]);
        crc = crc32c_step(crc, bytes[idx + 4]);
        crc = crc32c_step(crc, bytes[idx + 5]);
        crc = crc32c_step(crc, bytes[idx + 6]);
        crc = crc32c_step(crc, bytes[idx + 7]);
        crc = crc32c_step(crc, bytes[idx + 8]);
        crc = crc32c_step(crc, bytes[idx + 9]);
        crc = crc32c_step(crc, bytes[idx + 10]);
        crc = crc32c_step(crc, bytes[idx + 11]);
        crc = crc32c_step(crc, bytes[idx + 12]);
        crc = crc32c_step(crc, bytes[idx + 13]);
        crc = crc32c_step(crc, bytes[idx + 14]);
        crc = crc32c_step(crc, bytes[idx + 15]);
        crc = crc32c_step(crc, bytes[idx + 16]);
        crc = crc32c_step(crc, bytes[idx + 17]);
        idx += 18;
    }
    while idx < bytes.len() {
        crc = crc32c_step(crc, bytes[idx]);
        idx += 1;
    }
    !crc
}

fn crc32c_step(mut crc: u32, byte: u8) -> u32 {
    crc ^= byte as u32;
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    crc = if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    };
    if (crc & 1) == 1 {
        (crc >> 1) ^ 0x82f6_3b78
    } else {
        crc >> 1
    }
}

fn copy_payload_into_record_fixed<const LEN: usize>(
    record_bytes: &mut [u8; PERSIST_RECORD_MAX_BYTES],
    payload: &[u8; PERSIST_MAX_PAYLOAD_BYTES],
) {
    let mut idx = 0usize;
    while idx < LEN {
        let rem = LEN - idx;
        record_bytes[PERSIST_RECORD_HEADER_BYTES + idx] = payload[idx];
        if rem >= 2 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 1] = payload[idx + 1];
        }
        if rem >= 3 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 2] = payload[idx + 2];
        }
        if rem >= 4 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 3] = payload[idx + 3];
        }
        if rem >= 5 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 4] = payload[idx + 4];
        }
        if rem >= 6 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 5] = payload[idx + 5];
        }
        if rem >= 7 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 6] = payload[idx + 6];
        }
        if rem >= 8 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 7] = payload[idx + 7];
        }
        if rem >= 9 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 8] = payload[idx + 8];
        }
        if rem >= 10 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 9] = payload[idx + 9];
        }
        if rem >= 11 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 10] = payload[idx + 10];
        }
        if rem >= 12 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 11] = payload[idx + 11];
        }
        if rem >= 13 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 12] = payload[idx + 12];
        }
        if rem >= 14 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 13] = payload[idx + 13];
        }
        if rem >= 15 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 14] = payload[idx + 14];
        }
        if rem >= 16 {
            record_bytes[PERSIST_RECORD_HEADER_BYTES + idx + 15] = payload[idx + 15];
        }
        idx += 16;
    }
}

fn copy_payload_from_record_fixed<const LEN: usize>(
    payload: &mut [u8; PERSIST_MAX_PAYLOAD_BYTES],
    record: &[u8],
) {
    let mut idx = 0usize;
    while idx < LEN {
        let rem = LEN - idx;
        payload[idx] = record[PERSIST_RECORD_HEADER_BYTES + idx];
        if rem >= 2 {
            payload[idx + 1] = record[PERSIST_RECORD_HEADER_BYTES + idx + 1];
        }
        if rem >= 3 {
            payload[idx + 2] = record[PERSIST_RECORD_HEADER_BYTES + idx + 2];
        }
        if rem >= 4 {
            payload[idx + 3] = record[PERSIST_RECORD_HEADER_BYTES + idx + 3];
        }
        if rem >= 5 {
            payload[idx + 4] = record[PERSIST_RECORD_HEADER_BYTES + idx + 4];
        }
        if rem >= 6 {
            payload[idx + 5] = record[PERSIST_RECORD_HEADER_BYTES + idx + 5];
        }
        if rem >= 7 {
            payload[idx + 6] = record[PERSIST_RECORD_HEADER_BYTES + idx + 6];
        }
        if rem >= 8 {
            payload[idx + 7] = record[PERSIST_RECORD_HEADER_BYTES + idx + 7];
        }
        if rem >= 9 {
            payload[idx + 8] = record[PERSIST_RECORD_HEADER_BYTES + idx + 8];
        }
        if rem >= 10 {
            payload[idx + 9] = record[PERSIST_RECORD_HEADER_BYTES + idx + 9];
        }
        if rem >= 11 {
            payload[idx + 10] = record[PERSIST_RECORD_HEADER_BYTES + idx + 10];
        }
        if rem >= 12 {
            payload[idx + 11] = record[PERSIST_RECORD_HEADER_BYTES + idx + 11];
        }
        if rem >= 13 {
            payload[idx + 12] = record[PERSIST_RECORD_HEADER_BYTES + idx + 12];
        }
        if rem >= 14 {
            payload[idx + 13] = record[PERSIST_RECORD_HEADER_BYTES + idx + 13];
        }
        if rem >= 15 {
            payload[idx + 14] = record[PERSIST_RECORD_HEADER_BYTES + idx + 14];
        }
        if rem >= 16 {
            payload[idx + 15] = record[PERSIST_RECORD_HEADER_BYTES + idx + 15];
        }
        idx += 16;
    }
}

#[cfg(test)]
#[path = "persist_spec_tests.rs"]
mod tests;
