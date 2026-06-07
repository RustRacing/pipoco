use ecu_spec::{
    burn_page, committed_page_record, factory_reset, persist_decode, persist_encode,
    persist_migrate, write_page, PersistMigrationError, PersistPage, PersistPageId,
    TsBurnSaveStore, PERSIST_ANGLES_PAGE_BYTES, PERSIST_FUEL_PAGE_BYTES,
    PERSIST_IGNITION_PAGE_BYTES, PERSIST_SCHEMA_VERSION_CURRENT,
};
use proptest::prelude::*;

fn page_strategy() -> impl Strategy<Value = (u16, PersistPageId, Vec<u8>)> {
    prop_oneof![
        (
            1u16..=3,
            proptest::collection::vec(any::<u8>(), PERSIST_FUEL_PAGE_BYTES)
        )
            .prop_map(|(schema, payload)| (schema, PersistPageId::Fuel, payload)),
        (
            1u16..=3,
            proptest::collection::vec(any::<u8>(), PERSIST_IGNITION_PAGE_BYTES)
        )
            .prop_map(|(schema, payload)| (schema, PersistPageId::Ignition, payload)),
        (
            1u16..=3,
            proptest::collection::vec(any::<u8>(), PERSIST_ANGLES_PAGE_BYTES)
        )
            .prop_map(|(schema, payload)| (schema, PersistPageId::Angles, payload)),
    ]
}

fn migration_supported_strategy() -> impl Strategy<Value = (PersistPageId, u16, u16, Vec<u8>)> {
    prop_oneof![(
        prop_oneof![
            Just(PersistPageId::Fuel),
            Just(PersistPageId::Ignition),
            Just(PersistPageId::Angles)
        ],
        prop_oneof![
            Just((1u16, 1u16)),
            Just((1u16, 2u16)),
            Just((1u16, 3u16)),
            Just((2u16, 2u16)),
            Just((2u16, 3u16)),
            Just((3u16, 3u16)),
        ],
        proptest::collection::vec(any::<u8>(), PERSIST_FUEL_PAGE_BYTES)
    )
        .prop_map(|(page_id, versions, payload)| {
            let (from_version, to_version) = versions;
            let payload = match page_id {
                PersistPageId::Fuel => payload[..PERSIST_FUEL_PAGE_BYTES].to_vec(),
                PersistPageId::Ignition => payload[..PERSIST_IGNITION_PAGE_BYTES].to_vec(),
                PersistPageId::Angles => payload[..PERSIST_ANGLES_PAGE_BYTES].to_vec(),
            };
            (page_id, from_version, to_version, payload)
        }),]
}

fn migration_unsupported_strategy() -> impl Strategy<Value = (PersistPageId, u16, u16, Vec<u8>)> {
    prop_oneof![(
        prop_oneof![
            Just(PersistPageId::Fuel),
            Just(PersistPageId::Ignition),
            Just(PersistPageId::Angles)
        ],
        prop_oneof![Just((2u16, 1u16)), Just((3u16, 1u16)), Just((3u16, 2u16)),],
        proptest::collection::vec(any::<u8>(), PERSIST_FUEL_PAGE_BYTES)
    )
        .prop_map(|(page_id, versions, payload)| {
            let (from_version, to_version) = versions;
            let payload = match page_id {
                PersistPageId::Fuel => payload[..PERSIST_FUEL_PAGE_BYTES].to_vec(),
                PersistPageId::Ignition => payload[..PERSIST_IGNITION_PAGE_BYTES].to_vec(),
                PersistPageId::Angles => payload[..PERSIST_ANGLES_PAGE_BYTES].to_vec(),
            };
            (page_id, from_version, to_version, payload)
        }),]
}

proptest! {
    #[test]
    fn prop_persist_roundtrip((schema_version, page_id, payload) in page_strategy()) {
        let page = PersistPage::new(schema_version, page_id, &payload).expect("new page");
        let encoded = persist_encode(&page).expect("encode");
        let decoded = persist_decode(&encoded.bytes[..encoded.len as usize]).expect("decode");
        prop_assert_eq!(decoded, page);
    }

    #[test]
    fn prop_factory_reset_roundtrip_equals_reset_twice((schema_version, page_id, payload) in page_strategy()) {
        let page = PersistPage::new(schema_version, page_id, &payload).expect("new page");
        let encoded = persist_encode(&page).expect("encode");

        let reset_once = factory_reset(&encoded.bytes[..encoded.len as usize]).expect("reset once");
        let reset_once_decoded = persist_decode(&reset_once.bytes[..reset_once.len as usize]).expect("decode reset once");
        prop_assert_eq!(reset_once_decoded.schema_version, PERSIST_SCHEMA_VERSION_CURRENT);
        prop_assert_eq!(reset_once_decoded.page_id, page_id);
        prop_assert!(reset_once_decoded.payload_slice().iter().all(|byte| *byte == 0));

        let reset_twice = factory_reset(&reset_once.bytes[..reset_once.len as usize]).expect("reset twice");
        prop_assert_eq!(reset_twice, reset_once);
    }

    #[test]
    fn prop_persist_migration_supported_paths(
        (page_id, from_version, to_version, payload) in migration_supported_strategy()
    ) {
        let migrated = persist_migrate(page_id, from_version, to_version, &payload).expect("supported migration must succeed");
        prop_assert_eq!(migrated.page_id, page_id);
        prop_assert_eq!(migrated.schema_version, to_version);
        prop_assert_eq!(migrated.payload_slice(), payload.as_slice());
    }

    #[test]
    fn prop_persist_migration_direct_equals_stepwise(
        page_id in prop_oneof![Just(PersistPageId::Fuel), Just(PersistPageId::Ignition), Just(PersistPageId::Angles)],
        payload in proptest::collection::vec(any::<u8>(), PERSIST_FUEL_PAGE_BYTES),
    ) {
        let payload = match page_id {
            PersistPageId::Fuel => payload[..PERSIST_FUEL_PAGE_BYTES].to_vec(),
            PersistPageId::Ignition => payload[..PERSIST_IGNITION_PAGE_BYTES].to_vec(),
            PersistPageId::Angles => payload[..PERSIST_ANGLES_PAGE_BYTES].to_vec(),
        };

        let direct = persist_migrate(page_id, 1, 3, &payload).expect("direct migration must succeed");
        let v2 = persist_migrate(page_id, 1, 2, &payload).expect("v1->v2 migration must succeed");
        let step = persist_migrate(page_id, 2, 3, v2.payload_slice()).expect("v2->v3 migration must succeed");
        prop_assert_eq!(direct, step);
    }

    #[test]
    fn prop_persist_migration_unsupported_paths_return_error(
        (page_id, from_version, to_version, payload) in migration_unsupported_strategy()
    ) {
        let err = persist_migrate(page_id, from_version, to_version, &payload).expect_err("unsupported migration path must fail");
        prop_assert_eq!(err, PersistMigrationError::UnsupportedMigrationPath);
    }

    #[test]
    fn prop_ts_burn_interleaved_commits_are_atomic(
        fuel_a in any::<u8>(),
        fuel_b in any::<u8>(),
        ign_a in any::<u8>(),
        ign_b in any::<u8>(),
    ) {
        let mut store = TsBurnSaveStore::default();

        write_page(&mut store, 1, 0, &[fuel_a]).expect("stage fuel_a");
        burn_page(&mut store, 1, false).expect("burn fuel_a");

        write_page(&mut store, 2, 0, &[ign_a]).expect("stage ign_a");
        burn_page(&mut store, 2, false).expect("burn ign_a");

        write_page(&mut store, 1, 0, &[fuel_b]).expect("stage fuel_b");
        burn_page(&mut store, 1, false).expect("burn fuel_b");

        write_page(&mut store, 2, 0, &[ign_b]).expect("stage ign_b");
        burn_page(&mut store, 2, false).expect("burn ign_b");

        let fuel = committed_page_record(&store, 1).expect("fuel committed");
        let ign = committed_page_record(&store, 2).expect("ign committed");
        let fuel_decoded = persist_decode(&fuel.bytes[..fuel.len as usize]).expect("fuel decode");
        let ign_decoded = persist_decode(&ign.bytes[..ign.len as usize]).expect("ign decode");

        prop_assert_eq!(fuel_decoded.page_id, PersistPageId::Fuel);
        prop_assert_eq!(ign_decoded.page_id, PersistPageId::Ignition);
        prop_assert_eq!(fuel_decoded.payload_slice()[0], fuel_b);
        prop_assert_eq!(ign_decoded.payload_slice()[0], ign_b);
    }
}
