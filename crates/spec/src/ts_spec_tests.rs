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
    let payload = [3u8, 0x04, 0x00, 0x08, 0x00];
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
            offset: 4,
            len: 8,
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
fn ts_dispatch_reports_command_length_with_command_context() {
    let payload = [1u8, 0, 0, 1];
    let frame = build_frame(CMD_READ_PAGE, &payload);
    let total = 4 + (1 + payload.len() + 2);

    let result = ts_dispatch_step(&frame[..total]);

    assert_eq!(
        result.effect,
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::WrongPayloadLength {
                command_id: CMD_READ_PAGE,
                expected: 5,
                actual: 4,
            }
        ))
    );
}

#[test]
fn ts_dispatch_reports_write_payload_too_short_with_command_context() {
    let payload = [1u8, 0];
    let frame = build_frame(CMD_WRITE_PAGE, &payload);
    let total = 4 + (1 + payload.len() + 2);

    let result = ts_dispatch_step(&frame[..total]);

    assert_eq!(
        result.effect,
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::PayloadTooShort {
                command_id: CMD_WRITE_PAGE,
                minimum: 3,
                actual: 2,
            }
        ))
    );
}

#[test]
fn ts_dispatch_rejects_unknown_persist_page_before_effect() {
    let payload = [4u8, 0, 0, 1, 0];
    let frame = build_frame(CMD_READ_PAGE, &payload);
    let total = 4 + (1 + payload.len() + 2);

    let result = ts_dispatch_step(&frame[..total]);

    assert_eq!(
        result.effect,
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::UnknownPage {
                command_id: CMD_READ_PAGE,
                page_number: 4,
            }
        ))
    );
}

#[test]
fn ts_dispatch_rejects_page_range_overflow_before_effect() {
    let payload = [1u8, 0xFF, 0x01, 0x02, 0x00];
    let frame = build_frame(CMD_READ_PAGE, &payload);
    let total = 4 + (1 + payload.len() + 2);

    let result = ts_dispatch_step(&frame[..total]);

    assert_eq!(
        result.effect,
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::PageRangeOutOfBounds {
                command_id: CMD_READ_PAGE,
                page_number: 1,
                offset: 511,
                len: 2,
                page_len: PERSIST_FUEL_PAGE_BYTES as u16,
            }
        ))
    );
}

#[test]
fn ts_dispatch_rejects_burn_of_non_persist_page() {
    let payload = [4u8];
    let frame = build_frame(CMD_BURN, &payload);
    let total = 4 + (1 + payload.len() + 2);

    let result = ts_dispatch_step(&frame[..total]);

    assert_eq!(
        result.effect,
        Err(TsDispatchError::CommandDecode(
            TsCommandDecodeError::UnknownPage {
                command_id: CMD_BURN,
                page_number: 4,
            }
        ))
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
