use ecu_spec::{
    burn_page, committed_page_record, decode_outpc, encode_outpc, encode_ts_diag_log_oldest_first,
    page_meta, save_all, ts_diag_log_push, ts_dispatch_step, write_page, OutpcFrame, PersistPageId,
    TsBurnSaveStore, TsDiagLogEntry, TsDiagLogRing, TsDispatchState, TsPageId, TsPageMeta,
    TsPageMetaError, TS_DIAG_LOG_CAPACITY, TS_DIAG_LOG_ENTRY_BYTES,
};
use proptest::prelude::*;

fn decode_diag_entry(bytes: &[u8]) -> TsDiagLogEntry {
    TsDiagLogEntry {
        timestamp_us: u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
        code: u16::from_le_bytes([bytes[4], bytes[5]]),
        source: bytes[6],
        context: u16::from_le_bytes([bytes[7], bytes[8]]),
    }
}

fn valid_dispatch_shape(state_len: u8, states: &[TsDispatchState; 6]) -> bool {
    if state_len != 5 && state_len != 6 {
        return false;
    }

    if states[0] != TsDispatchState::Idle
        || states[1] != TsDispatchState::RxFrame
        || states[2] != TsDispatchState::Decode
    {
        return false;
    }

    if state_len == 5 {
        states[3] == TsDispatchState::ErrorReply && states[4] == TsDispatchState::Idle
    } else {
        states[3] == TsDispatchState::Execute
            && (states[4] == TsDispatchState::EncodeReply
                || states[4] == TsDispatchState::ErrorReply)
            && states[5] == TsDispatchState::Idle
    }
}

fn outpc_frame_strategy() -> impl Strategy<Value = OutpcFrame> {
    (
        any::<u16>(),
        any::<u16>(),
        any::<u16>(),
        any::<i16>(),
        any::<i16>(),
        any::<u16>(),
        any::<i16>(),
        any::<u8>(),
        any::<u8>(),
        any::<u32>(),
    )
        .prop_map(
            |(
                rpm,
                map_kpa10,
                tps_x100,
                clt_c10,
                iat_c10,
                pw_corr_us,
                advance_deg10,
                sync_state_code,
                cut_reason_code,
                status_flags,
            )| OutpcFrame {
                rpm,
                map_kpa10,
                tps_x100,
                clt_c10,
                iat_c10,
                pw_corr_us,
                advance_deg10,
                sync_state_code,
                cut_reason_code,
                status_flags,
            },
        )
}

fn diag_entry_strategy() -> impl Strategy<Value = TsDiagLogEntry> {
    (any::<u32>(), any::<u16>(), any::<u8>(), any::<u16>()).prop_map(
        |(timestamp_us, code, source, context)| TsDiagLogEntry {
            timestamp_us,
            code,
            source,
            context,
        },
    )
}

proptest! {
    #[test]
    fn prop_ts_page_meta_totality(page in any::<u8>()) {
        let meta = page_meta(page);
        match page {
            1 => prop_assert_eq!(
                meta,
                Ok(TsPageMeta {
                    page_id: TsPageId::Fuel,
                    signature: 0x46554C31,
                    payload_size: 512,
                })
            ),
            2 => prop_assert_eq!(
                meta,
                Ok(TsPageMeta {
                    page_id: TsPageId::Ignition,
                    signature: 0x49474E31,
                    payload_size: 512,
                })
            ),
            3 => prop_assert_eq!(
                meta,
                Ok(TsPageMeta {
                    page_id: TsPageId::Angles,
                    signature: 0x414E4731,
                    payload_size: 68,
                })
            ),
            4 => prop_assert_eq!(
                meta,
                Ok(TsPageMeta {
                    page_id: TsPageId::Outpc,
                    signature: 0x4F555431,
                    payload_size: 64,
                })
            ),
            _ => prop_assert_eq!(meta, Err(TsPageMetaError::UnknownPage)),
        }
    }

    #[test]
    fn prop_ts_outpc_roundtrip(frame in outpc_frame_strategy()) {
        let bytes = encode_outpc(frame);
        let decoded = decode_outpc(&bytes).expect("encoded OUTPC always decodes");
        prop_assert_eq!(decoded, frame);
    }

    #[test]
    fn prop_ts_dispatch_totality(frame in proptest::collection::vec(any::<u8>(), 0..=64)) {
        let result = ts_dispatch_step(&frame);
        prop_assert!(valid_dispatch_shape(result.state_len, &result.states));
        prop_assert!(result.effect.is_ok() || result.effect.is_err());
    }

    #[test]
    fn prop_ts_burn_save_commit_atomicity(
        fuel0 in any::<u8>(),
        fuel1 in any::<u8>(),
        ign0 in any::<u8>(),
        ign1 in any::<u8>(),
        ang0 in any::<u8>(),
    ) {
        let mut store = TsBurnSaveStore::default();

        write_page(&mut store, 1, 0, &[fuel0]).expect("stage fuel0");
        burn_page(&mut store, 1, false).expect("commit fuel0");
        let before_running_reject = committed_page_record(&store, 1).expect("committed fuel");

        write_page(&mut store, 1, 0, &[fuel1]).expect("stage fuel1");
        let reject = burn_page(&mut store, 1, true);
        prop_assert!(reject.is_err());
        let after_running_reject = committed_page_record(&store, 1).expect("committed fuel");
        prop_assert_eq!(after_running_reject, before_running_reject);

        write_page(&mut store, 1, 0, &[fuel1]).expect("restage fuel1");
        write_page(&mut store, 2, 0, &[ign0]).expect("stage ign0");
        write_page(&mut store, 3, 0, &[ang0]).expect("stage ang0");
        save_all(&mut store, false).expect("save all");

        write_page(&mut store, 2, 0, &[ign1]).expect("stage ign1");
        burn_page(&mut store, 2, false).expect("commit ign1");

        let fuel = committed_page_record(&store, 1).expect("fuel committed");
        let ign = committed_page_record(&store, 2).expect("ignition committed");
        let ang = committed_page_record(&store, 3).expect("angles committed");

        let fuel_page = ecu_spec::persist_decode(&fuel.bytes[..fuel.len as usize]).expect("fuel decode");
        let ign_page = ecu_spec::persist_decode(&ign.bytes[..ign.len as usize]).expect("ign decode");
        let ang_page = ecu_spec::persist_decode(&ang.bytes[..ang.len as usize]).expect("ang decode");

        prop_assert_eq!(fuel_page.page_id, PersistPageId::Fuel);
        prop_assert_eq!(ign_page.page_id, PersistPageId::Ignition);
        prop_assert_eq!(ang_page.page_id, PersistPageId::Angles);
        prop_assert_eq!(fuel_page.payload_slice()[0], fuel1);
        prop_assert_eq!(ign_page.payload_slice()[0], ign1);
        prop_assert_eq!(ang_page.payload_slice()[0], ang0);
    }

    #[test]
    fn prop_ts_diag_log_wrap_oldest_first(entries in proptest::collection::vec(diag_entry_strategy(), 0..=96)) {
        let mut ring = TsDiagLogRing::default();
        for entry in &entries {
            ts_diag_log_push(&mut ring, *entry);
        }

        let encoded = encode_ts_diag_log_oldest_first(&ring);
        let expected_len = core::cmp::min(entries.len(), TS_DIAG_LOG_CAPACITY);
        prop_assert_eq!(ring.len as usize, expected_len);
        prop_assert_eq!(encoded.len as usize, expected_len * TS_DIAG_LOG_ENTRY_BYTES);

        let expected_tail = if entries.len() > TS_DIAG_LOG_CAPACITY {
            &entries[entries.len() - TS_DIAG_LOG_CAPACITY..]
        } else {
            &entries[..]
        };

        let mut idx = 0usize;
        while idx < expected_tail.len() {
            let offset = idx * TS_DIAG_LOG_ENTRY_BYTES;
            let decoded = decode_diag_entry(&encoded.bytes[offset..offset + TS_DIAG_LOG_ENTRY_BYTES]);
            prop_assert_eq!(decoded, expected_tail[idx]);
            idx += 1;
        }
    }
}
