use ecu_core::ts::proto::{self, Cmd};
use std::panic::catch_unwind;

struct Lcg(u64);

impl Lcg {
    const fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 32) as u32
    }

    fn fill_bytes(&mut self, buf: &mut [u8]) {
        for chunk in buf.chunks_mut(4) {
            let n = self.next_u32().to_le_bytes();
            let len = chunk.len();
            chunk.copy_from_slice(&n[..len]);
        }
    }
}

#[test]
fn decode_request_fuzz_never_panics() {
    let mut rng = Lcg::new(0x4d59_5df4_d0f3_3173);
    let mut buf = [0u8; 128];
    let mut reply = [0u8; 128];

    for i in 0..10_000u32 {
        let len = (rng.next_u32() as usize) % buf.len();
        rng.fill_bytes(&mut buf[..len]);

        // Inject a mix of valid and invalid headers / frame lengths.
        if i % 3 == 0 && len >= 5 {
            let valid = proto::encode_reply(Cmd::Ping, b"OK", &mut reply).unwrap();
            let copy_len = len.min(valid);
            buf[..copy_len].copy_from_slice(&reply[..copy_len]);
        } else if len >= 5 {
            buf[0] = 0xAA;
            buf[1] = 0x55;
        }

        let decoded = catch_unwind(|| proto::decode_request(&buf[..len]));
        assert!(decoded.is_ok(), "decode_request panicked for iteration {i}");

        if let Some((cmd, payload)) = decoded.unwrap() {
            let encoded = proto::encode_reply(cmd, payload, &mut reply).expect("re-encode");
            let roundtrip = proto::decode_request(&reply[..encoded]).expect("roundtrip decode");
            assert_eq!(roundtrip.0, cmd, "cmd mismatch on iteration {i}");
            assert_eq!(roundtrip.1, payload, "payload mismatch on iteration {i}");
        }
    }
}

#[test]
fn decode_request_rejects_truncated_frames() {
    let mut reply = [0u8; 64];
    let len = proto::encode_reply(Cmd::Sig, b"IPW-ECU V0.1", &mut reply).unwrap();
    for cut in 0..len {
        let decoded = catch_unwind(|| proto::decode_request(&reply[..cut]));
        assert!(decoded.is_ok(), "decode_request panicked at cut {cut}");
        assert!(
            decoded.unwrap().is_none(),
            "truncated frame should reject at cut {cut}"
        );
    }
}
