//! Negative tests for TS framing and decode robustness

use ecu_core::ts::proto::{decode_request, encode_reply, Cmd};
use ecu_core::ts::serial::FrameAssembler;

#[test]
fn decode_request_bad_crc_rejected() {
    let mut frame = [0u8; 64];
    let n = encode_reply(Cmd::Sig, b"IPW-ECU V0.1", &mut frame).unwrap();
    // Corrupt CRC
    frame[n - 1] ^= 0xFF;
    assert!(decode_request(&frame[..n]).is_none());
}

#[test]
fn assembler_drops_garbage_and_recovers() {
    let mut asm = FrameAssembler::new();
    let mut good = [0u8; 64];
    let len = encode_reply(Cmd::Ping, &[], &mut good).unwrap();

    // Feed garbage bytes that look like magic but with bad CRC
    let mut bad = good;
    bad[len - 1] ^= 0xAA; // flip crc
    asm.feed(&bad[..len]);
    let mut out = [0u8; 64];
    // Should not produce a frame yet
    assert!(asm.try_pop(&mut out).is_none());

    // Now feed a valid frame; assembler should output it
    asm.feed(&good[..len]);
    let n = asm
        .try_pop(&mut out)
        .expect("should recover and produce frame");
    assert_eq!(n, len);
    let (cmd, _payload) = decode_request(&out[..n]).unwrap();
    assert_eq!(cmd, Cmd::Ping);
}
