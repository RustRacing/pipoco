//! Simple serial framing utilities for TS protocol
//! Accumulates bytes and extracts complete frames by magic/len/CRC.
use super::proto::{decode_request, MAGIC};

pub trait SerialPort {
    fn read(&mut self, buf: &mut [u8]) -> usize;
    fn write(&mut self, buf: &[u8]) -> usize;
    fn available(&self) -> usize {
        0
    }
}

pub struct FrameAssembler {
    buf: [u8; 2048],
    len: usize,
}

impl FrameAssembler {
    pub const fn new() -> Self {
        Self {
            buf: [0; 2048],
            len: 0,
        }
    }

    pub fn feed(&mut self, data: &[u8]) {
        let take = core::cmp::min(data.len(), self.buf.len().saturating_sub(self.len));
        if take > 0 {
            self.buf[self.len..self.len + take].copy_from_slice(&data[..take]);
            self.len += take;
        }
    }

    pub fn poll_port<P: SerialPort>(&mut self, port: &mut P) {
        let mut tmp = [0u8; 128];
        let n = port.read(&mut tmp);
        if n > 0 {
            self.feed(&tmp[..n]);
        }
    }

    /// Try to pop one frame into `out`, return length on success.
    pub fn try_pop(&mut self, out: &mut [u8]) -> Option<usize> {
        // Find magic
        let mut i = 0;
        while i + 4 <= self.len {
            let mg = (self.buf[i + 1] as u16) << 8 | (self.buf[i] as u16);
            if mg == MAGIC {
                break;
            }
            i += 1;
        }
        if i > 0 {
            // shift left to discard leading garbage
            for j in 0..(self.len - i) {
                self.buf[j] = self.buf[i + j];
            }
            self.len -= i;
        }
        if self.len < 4 {
            return None;
        }
        let length = (self.buf[3] as usize) << 8 | (self.buf[2] as usize);
        if length < 3 || length > self.buf.len().saturating_sub(4) {
            // Invalid or unbufferable length. Drop magic byte and resume scanning.
            for j in 0..(self.len - 1) {
                self.buf[j] = self.buf[j + 1];
            }
            self.len -= 1;
            return None;
        }
        let total = 4 + length;
        if self.len < total {
            return None;
        }
        if out.len() < total {
            for j in 0..(self.len - 1) {
                self.buf[j] = self.buf[j + 1];
            }
            self.len -= 1;
            return None;
        }
        out[..total].copy_from_slice(&self.buf[..total]);
        // verify decodes to a request; if not, drop one byte and retry next call
        if decode_request(&out[..total]).is_none() {
            // drop header byte to recover
            for j in 0..(self.len - 1) {
                self.buf[j] = self.buf[j + 1];
            }
            self.len -= 1;
            return None;
        }
        // consume from buffer
        for j in 0..(self.len - total) {
            self.buf[j] = self.buf[total + j];
        }
        self.len -= total;
        Some(total)
    }
}

impl Default for FrameAssembler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{encode_reply, Cmd};

    fn encoded(cmd: Cmd, payload: &[u8]) -> ([u8; 64], usize) {
        let mut out = [0u8; 64];
        let len = encode_reply(cmd, payload, &mut out).unwrap();
        (out, len)
    }

    #[test]
    fn assemble_one() {
        let mut asm = FrameAssembler::new();
        let (req, len) = encoded(Cmd::Ping, &[]);
        // feed in two parts
        asm.feed(&req[..3]);
        asm.feed(&req[3..len]);
        let mut out = [0u8; 64];
        let n = asm.try_pop(&mut out).unwrap();
        assert_eq!(n, len);
    }

    #[test]
    fn oversized_length_header_does_not_wedge_assembler() {
        let mut asm = FrameAssembler::new();
        asm.feed(&[0xAA, 0x55, 0xFF, 0xFF]);

        let (good, len) = encoded(Cmd::Ping, &[]);
        asm.feed(&good[..len]);

        let mut out = [0u8; 64];
        assert!(asm.try_pop(&mut out).is_none());
        let n = asm.try_pop(&mut out).expect("assembler should recover");
        assert_eq!(n, len);
    }

    #[test]
    fn bad_crc_frame_is_dropped_before_next_valid_frame() {
        let mut asm = FrameAssembler::new();
        let (mut bad, bad_len) = encoded(Cmd::Ping, b"bad");
        bad[bad_len - 1] ^= 0x55;
        let (good, good_len) = encoded(Cmd::Sig, b"ok");

        asm.feed(&bad[..bad_len]);
        asm.feed(&good[..good_len]);

        let mut out = [0u8; 64];
        assert!(asm.try_pop(&mut out).is_none());
        let n = asm.try_pop(&mut out).expect("valid frame after bad CRC");
        assert_eq!(n, good_len);
        assert_eq!(&out[..n], &good[..good_len]);
    }

    #[test]
    fn unknown_command_frame_is_dropped_before_next_valid_frame() {
        let mut asm = FrameAssembler::new();
        let (mut bad, bad_len) = encoded(Cmd::Ping, b"bad");
        bad[4] = 0xFE;
        let crc = crate::proto::crc16_ccitt(&bad[4..bad_len - 2]);
        bad[bad_len - 2] = (crc & 0xFF) as u8;
        bad[bad_len - 1] = (crc >> 8) as u8;
        let (good, good_len) = encoded(Cmd::Ping, b"ok");

        asm.feed(&bad[..bad_len]);
        asm.feed(&good[..good_len]);

        let mut out = [0u8; 64];
        assert!(asm.try_pop(&mut out).is_none());
        let n = asm
            .try_pop(&mut out)
            .expect("valid frame after unknown cmd");
        assert_eq!(n, good_len);
        assert_eq!(&out[..n], &good[..good_len]);
    }

    #[test]
    fn too_small_output_buffer_drops_bad_candidate_and_recovers() {
        let mut asm = FrameAssembler::new();
        let (first, first_len) = encoded(Cmd::Ping, b"payload");
        let (second, second_len) = encoded(Cmd::Ping, &[]);
        asm.feed(&first[..first_len]);
        asm.feed(&second[..second_len]);

        let mut too_small = [0u8; 4];
        assert!(asm.try_pop(&mut too_small).is_none());

        let mut out = [0u8; 64];
        for _ in 0..first_len {
            if let Some(n) = asm.try_pop(&mut out) {
                assert_eq!(n, second_len);
                assert_eq!(&out[..n], &second[..second_len]);
                return;
            }
        }
        panic!("assembler did not recover to second valid frame");
    }
}
