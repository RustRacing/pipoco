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
        let total = 4 + length;
        if self.len < total {
            return None;
        }
        if out.len() < total {
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
    use crate::ts::proto::{encode_reply, Cmd};
    #[test]
    fn assemble_one() {
        let mut asm = FrameAssembler::new();
        let mut req = [0u8; 64];
        let len = encode_reply(Cmd::Ping, &[], &mut req).unwrap();
        // feed in two parts
        asm.feed(&req[..3]);
        asm.feed(&req[3..len]);
        let mut out = [0u8; 64];
        let n = asm.try_pop(&mut out).unwrap();
        assert_eq!(n, len);
    }
}
