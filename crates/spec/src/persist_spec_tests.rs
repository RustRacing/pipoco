use super::*;

#[allow(clippy::manual_unwrap_or_default)]
fn must_ok<T: Default, E>(result: Result<T, E>) -> T {
    assert!(result.is_ok(), "expected Ok(..)");
    match result {
        Ok(value) => value,
        Err(_) => T::default(),
    }
}

fn payload_with_seed(len: usize, seed: u8) -> [u8; PERSIST_MAX_PAYLOAD_BYTES] {
    let mut payload = [0u8; PERSIST_MAX_PAYLOAD_BYTES];
    let mut idx = 0usize;
    while idx < len {
        payload[idx] = seed.wrapping_add((idx & 0xff) as u8);
        idx += 1;
    }
    payload
}

#[test]
fn encode_decode_roundtrip_for_angles_page() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 17);
    let page = PersistPage {
        schema_version: 3,
        page_id: PersistPageId::Angles,
        payload_len: PERSIST_ANGLES_PAGE_BYTES as u16,
        payload,
    };
    let encoded = must_ok(persist_encode(&page));
    let decoded = must_ok(persist_decode(&encoded.bytes[..encoded.len as usize]));
    assert_eq!(decoded, page);
}

#[test]
fn decode_detects_crc_mismatch() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 23);
    let page = PersistPage {
        schema_version: 3,
        page_id: PersistPageId::Angles,
        payload_len: PERSIST_ANGLES_PAGE_BYTES as u16,
        payload,
    };
    let mut encoded = must_ok(persist_encode(&page));
    encoded.bytes[6] ^= 0x01;
    let err = persist_decode(&encoded.bytes[..encoded.len as usize]).expect_err("decode must fail");
    assert_eq!(err, PersistDecodeError::CrcMismatch);
}

#[test]
fn migrate_identity_3_to_3_keeps_payload() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 3);
    let migrated = persist_migrate(
        PersistPageId::Angles,
        3,
        3,
        &payload[..PERSIST_ANGLES_PAGE_BYTES],
    );
    let migrated = must_ok(migrated);
    assert_eq!(migrated.schema_version, 3);
    assert_eq!(
        migrated.payload_slice(),
        &payload[..PERSIST_ANGLES_PAGE_BYTES]
    );
}

#[test]
fn migrate_1_to_2_copies_payload() {
    let payload = payload_with_seed(PERSIST_FUEL_PAGE_BYTES, 11);
    let migrated = persist_migrate(
        PersistPageId::Fuel,
        1,
        2,
        &payload[..PERSIST_FUEL_PAGE_BYTES],
    );
    let migrated = must_ok(migrated);
    assert_eq!(migrated.schema_version, 2);
    assert_eq!(
        migrated.payload_slice(),
        &payload[..PERSIST_FUEL_PAGE_BYTES]
    );
}

#[test]
fn migrate_2_to_3_copies_payload_with_defaults() {
    let payload = payload_with_seed(PERSIST_IGNITION_PAGE_BYTES, 19);
    let migrated = persist_migrate(
        PersistPageId::Ignition,
        2,
        3,
        &payload[..PERSIST_IGNITION_PAGE_BYTES],
    );
    let migrated = must_ok(migrated);
    assert_eq!(migrated.schema_version, 3);
    assert_eq!(
        migrated.payload_slice(),
        &payload[..PERSIST_IGNITION_PAGE_BYTES]
    );
}

#[test]
fn migrate_1_to_3_is_sequential_and_deterministic() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 29);
    let direct = persist_migrate(
        PersistPageId::Angles,
        1,
        3,
        &payload[..PERSIST_ANGLES_PAGE_BYTES],
    );
    let direct = must_ok(direct);
    let step_v2 = persist_migrate(
        PersistPageId::Angles,
        1,
        2,
        &payload[..PERSIST_ANGLES_PAGE_BYTES],
    );
    let step_v2 = must_ok(step_v2);
    let step_v3 = must_ok(persist_migrate(
        PersistPageId::Angles,
        2,
        3,
        step_v2.payload_slice(),
    ));
    assert_eq!(direct, step_v3);
}

#[test]
fn migrate_rejects_unknown_from_version() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 41);
    let err = persist_migrate(
        PersistPageId::Angles,
        4,
        3,
        &payload[..PERSIST_ANGLES_PAGE_BYTES],
    )
    .expect_err("migration must fail");
    assert_eq!(err, PersistMigrationError::UnsupportedFromVersion);
}

#[test]
fn migrate_rejects_unknown_to_version() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 43);
    let err = persist_migrate(
        PersistPageId::Angles,
        3,
        4,
        &payload[..PERSIST_ANGLES_PAGE_BYTES],
    )
    .expect_err("migration must fail");
    assert_eq!(err, PersistMigrationError::UnsupportedToVersion);
}

#[test]
fn migrate_rejects_unsupported_path() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 47);
    let err = persist_migrate(
        PersistPageId::Angles,
        2,
        1,
        &payload[..PERSIST_ANGLES_PAGE_BYTES],
    )
    .expect_err("migration must fail");
    assert_eq!(err, PersistMigrationError::UnsupportedMigrationPath);
}

#[test]
fn migrate_rejects_missing_required_source_bytes() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 53);
    let err = persist_migrate(
        PersistPageId::Angles,
        1,
        2,
        &payload[..PERSIST_ANGLES_PAGE_BYTES - 1],
    )
    .expect_err("migration must fail");
    assert_eq!(err, PersistMigrationError::MalformedSourcePayload);
}

#[test]
fn factory_reset_sets_current_schema_and_canonical_payload() {
    let payload = payload_with_seed(PERSIST_ANGLES_PAGE_BYTES, 61);
    let page = PersistPage {
        schema_version: 1,
        page_id: PersistPageId::Angles,
        payload_len: PERSIST_ANGLES_PAGE_BYTES as u16,
        payload,
    };
    let encoded = must_ok(persist_encode(&page));
    let reset = must_ok(factory_reset(&encoded.bytes[..encoded.len as usize]));
    let decoded = must_ok(persist_decode(&reset.bytes[..reset.len as usize]));
    assert_eq!(decoded.schema_version, PERSIST_SCHEMA_VERSION_CURRENT);
    assert_eq!(decoded.page_id, PersistPageId::Angles);
    assert_eq!(decoded.payload_slice(), &[0u8; PERSIST_ANGLES_PAGE_BYTES]);
}
