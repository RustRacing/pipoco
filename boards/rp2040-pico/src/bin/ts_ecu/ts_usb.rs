use ecu_ts::proto::{self, Cmd};

use crate::ts_runtime::BoardEcuState;

pub(crate) fn handle_output_test_cmd(
    payload: &[u8],
    mut fire_output: impl FnMut(u8, u32, u32, u8),
    out: &mut [u8],
) -> Option<usize> {
    if payload.len() < 6 {
        return None;
    }

    let chan = payload[0];
    let on_ms = u16::from_le_bytes([payload[1], payload[2]]) as u32;
    let off_ms = u16::from_le_bytes([payload[3], payload[4]]) as u32;
    let reps = payload[5];
    fire_output(chan, on_ms, off_ms, reps);

    proto::encode_reply(Cmd::OutputTest, b"OK", out)
}

pub(crate) fn encode_tooth_stats_reply(_state: &BoardEcuState, out: &mut [u8]) -> Option<usize> {
    // The rp2040 trigger/sync state lives in the split runtime, not the board ECU
    // state, so the legacy board rpm/sync fields were always default here;
    // preserve that 0/unsynced reply.
    let rpm: u16 = 0;
    let synced: u8 = 0;
    let mut buf = [0u8; 3];
    buf[0] = (rpm & 0xff) as u8;
    buf[1] = (rpm >> 8) as u8;
    buf[2] = synced;
    proto::encode_reply(Cmd::ToothStats, &buf, out)
}
