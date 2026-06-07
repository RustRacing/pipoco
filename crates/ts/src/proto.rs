//! Custom CRC-framed protocol for TunerStudio
//! magic (0xAA55 LE) + len(u16) + cmd(u8) + payload + crc16

pub const MAGIC: u16 = 0x55AA;

#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub enum Cmd {
    Sig = 0x10,
    Outpc = 0x11,
    Ping = 0x12,
    ReadPage = 0x20,
    WritePage = 0x21,
    Burn = 0x22,
    OutputTest = 0x30,
    ToothStats = 0x31,
}

pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Build reply frame into `out`. Returns length on success.
pub fn encode_reply(cmd: Cmd, payload: &[u8], out: &mut [u8]) -> Option<usize> {
    let header_len = 2 + 2 + 1; // magic + len + cmd
    let total = header_len + payload.len() + 2; // crc16
    if out.len() < total {
        return None;
    }
    out[0] = (MAGIC & 0xff) as u8;
    out[1] = (MAGIC >> 8) as u8;
    let len = (1 + payload.len() + 2) as u16; // cmd + payload + crc
    out[2] = (len & 0xff) as u8;
    out[3] = (len >> 8) as u8;
    out[4] = cmd as u8;
    if !payload.is_empty() {
        out[5..5 + payload.len()].copy_from_slice(payload);
    }
    let crc = crc16_ccitt(&out[4..4 + 1 + payload.len()]);
    let off = 5 + payload.len();
    out[off] = (crc & 0xff) as u8;
    out[off + 1] = (crc >> 8) as u8;
    Some(total)
}

/// Very small decoder: validate magic/len/crc, return (cmd, payload)
pub fn decode_request(buf: &[u8]) -> Option<(Cmd, &[u8])> {
    if buf.len() < 5 {
        return None;
    }
    let mg = (buf[1] as u16) << 8 | (buf[0] as u16);
    if mg != MAGIC {
        return None;
    }
    let len = (buf[3] as u16) << 8 | (buf[2] as u16);
    if len < 3 {
        return None;
    }
    let total = 4 + len as usize;
    if buf.len() < total {
        return None;
    }
    let cmd_b = buf[4];
    let payload_len = len as usize - 1 - 2; // cmd + crc
    let payload = &buf[5..5 + payload_len];
    let crc = (buf[5 + payload_len + 1] as u16) << 8 | (buf[5 + payload_len] as u16);
    if crc16_ccitt(&buf[4..4 + 1 + payload_len]) != crc {
        return None;
    }
    let cmd = match cmd_b {
        0x10 => Cmd::Sig,
        0x11 => Cmd::Outpc,
        0x12 => Cmd::Ping,
        0x20 => Cmd::ReadPage,
        0x21 => Cmd::WritePage,
        0x22 => Cmd::Burn,
        0x30 => Cmd::OutputTest,
        0x31 => Cmd::ToothStats,
        _ => return None,
    };
    Some((cmd, payload))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_sig() {
        let mut out = [0u8; 64];
        let len = encode_reply(Cmd::Sig, crate::TS_SIGNATURE, &mut out).unwrap();
        let decoded = decode_request(&out[..len]);
        assert!(decoded.is_some());
    }
}
