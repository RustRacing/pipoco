#![cfg(feature = "flash-kv")]
//! Flash-backed key/value store for RP2040 Pico
//!
//! This implements ecu_core::persist::KvStore over a reserved flash sector
//! using the RP2040 ROM flash routines. Storage layout is simple and safe:
//! - One 4 KiB sector at `KV_OFFSET` (from flash base) holds header + pages.
//! - Header contains magic, version, lengths and CRC16 for each key.
//! - Values are fixed-size for keys: b"fuel" (512), b"ign" (512), b"angles" (68).
//!
//! To customize the reserved region, adjust KV_OFFSET below or provide a
//! board-specific configuration.

use ecu_core::persist::{KvStore, KvError};

// RP2040 XIP flash base
const XIP_BASE: u32 = 0x1000_0000;
// Default KV region offset from flash start (set near end of 2 MiB flash)
// Adjust per board if needed.
const KV_OFFSET: u32 = 0x001F_0000; // 0x1000_0000 + 0x001F_0000 = 0x101F_0000
const SECTOR_SIZE: usize = 4096;
const PAGE_SIZE: usize = 256;

const MAGIC: u32 = 0x4950574B; // 'IPWK'
const VERSION: u16 = 2;

const LEN_FUEL: usize = 512;
const LEN_IGN: usize = 512;
const LEN_ANGLES: usize = 68;

#[repr(C, packed)]
struct Header {
    magic: u32,
    version: u16,
    _rsv: u16,
    fuel_len: u16,
    ign_len: u16,
    angles_len: u16,
    fuel_crc: u16,
    ign_crc: u16,
    angles_crc: u16,
}

fn crc16_ccitt(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 { crc = (crc << 1) ^ 0x1021; } else { crc <<= 1; }
        }
    }
    crc
}

fn kv_base_ptr() -> *const u8 { (XIP_BASE + KV_OFFSET) as *const u8 }
fn kv_base_off() -> u32 { KV_OFFSET }

pub struct FlashKv;

impl FlashKv {
    pub const fn new() -> Self { Self }

    fn read_header(&self) -> Option<Header> {
        unsafe {
            let hdr_ptr = kv_base_ptr() as *const Header;
            let hdr = core::ptr::read_volatile(hdr_ptr);
            if hdr.magic != MAGIC || hdr.version != VERSION { return None; }
            Some(hdr)
        }
    }

    fn read_block(offset: usize, out: &mut [u8]) {
        unsafe {
            let src = kv_base_ptr().add(offset);
            let dst = out.as_mut_ptr();
            for i in 0..out.len() { let b = core::ptr::read_volatile(src.add(i)); core::ptr::write_volatile(dst.add(i), b); }
        }
    }

    unsafe fn flash_erase(addr_off: u32, len: usize) {
        use rp2040_hal::rom_data::{flash_exit_xip, flash_range_erase, flash_enter_cmd_xip, flash_flush_cache};
        flash_exit_xip();
        // erase in 4K sectors
        let mut rem = len;
        let mut off = addr_off;
        while rem > 0 {
            flash_range_erase(off, SECTOR_SIZE, 4096, 0x20);
            off += SECTOR_SIZE as u32;
            rem = rem.saturating_sub(SECTOR_SIZE);
        }
        flash_flush_cache();
        flash_enter_cmd_xip();
    }

    unsafe fn flash_program(addr_off: u32, data: &[u8]) {
        use rp2040_hal::rom_data::{flash_exit_xip, flash_range_program, flash_enter_cmd_xip, flash_flush_cache};
        flash_exit_xip();
        // program in 256-byte chunks
        let mut off = 0usize;
        while off < data.len() {
            let end = core::cmp::min(off + PAGE_SIZE, data.len());
            let ptr = data.as_ptr().add(off);
            flash_range_program(addr_off + off as u32, ptr, end - off);
            off = end;
        }
        flash_flush_cache();
        flash_enter_cmd_xip();
    }
}

impl KvStore for FlashKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        let hdr = self.read_header().ok_or(KvError::NotFound)?;
        match key {
            b"fuel" => {
                if out.len() < LEN_FUEL { return Err(KvError::Io); }
                let off = core::mem::size_of::<Header>();
                let mut tmp = [0u8; LEN_FUEL];
                Self::read_block(off, &mut tmp);
                if crc16_ccitt(0xFFFF, &tmp) != hdr.fuel_crc { return Err(KvError::Io); }
                out[..LEN_FUEL].copy_from_slice(&tmp);
                Ok(LEN_FUEL)
            }
            b"ign" => {
                if out.len() < LEN_IGN { return Err(KvError::Io); }
                let off = core::mem::size_of::<Header>() + LEN_FUEL;
                let mut tmp = [0u8; LEN_IGN];
                Self::read_block(off, &mut tmp);
                if crc16_ccitt(0xFFFF, &tmp) != hdr.ign_crc { return Err(KvError::Io); }
                out[..LEN_IGN].copy_from_slice(&tmp);
                Ok(LEN_IGN)
            }
            b"angles" => {
                if hdr.angles_len as usize != LEN_ANGLES { return Err(KvError::NotFound); }
                if out.len() < LEN_ANGLES { return Err(KvError::Io); }
                let off = core::mem::size_of::<Header>() + LEN_FUEL + LEN_IGN;
                let mut tmp = [0u8; LEN_ANGLES];
                Self::read_block(off, &mut tmp);
                if crc16_ccitt(0xFFFF, &tmp) != hdr.angles_crc { return Err(KvError::Io); }
                out[..LEN_ANGLES].copy_from_slice(&tmp);
                Ok(LEN_ANGLES)
            }
            _ => Err(KvError::NotFound),
        }
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        // Read current values (or zero) into staging buffers
        let mut fuel = [0u8; LEN_FUEL];
        let mut ign = [0u8; LEN_IGN];
        let mut angles = [0u8; LEN_ANGLES];
        let _ = self.read(b"fuel", &mut fuel);
        let _ = self.read(b"ign", &mut ign);
        let _ = self.read(b"angles", &mut angles);
        match key {
            b"fuel" => { if data.len()!=LEN_FUEL { return Err(KvError::Io) } fuel.copy_from_slice(data); }
            b"ign" => { if data.len()!=LEN_IGN { return Err(KvError::Io) } ign.copy_from_slice(data); }
            b"angles" => { if data.len()!=LEN_ANGLES { return Err(KvError::Io) } angles.copy_from_slice(data); }
            _ => return Err(KvError::NotFound),
        }

        // Build header
        let hdr = Header {
            magic: MAGIC,
            version: VERSION,
            _rsv: 0,
            fuel_len: LEN_FUEL as u16,
            ign_len: LEN_IGN as u16,
            angles_len: LEN_ANGLES as u16,
            fuel_crc: crc16_ccitt(0xFFFF, &fuel),
            ign_crc: crc16_ccitt(0xFFFF, &ign),
            angles_crc: crc16_ccitt(0xFFFF, &angles),
        };

        // Serialize into a sector-sized buffer
        let mut sector = [0xFFu8; SECTOR_SIZE];
        let mut off = 0;
        let hdr_bytes: &[u8; core::mem::size_of::<Header>()] = unsafe { core::mem::transmute(&hdr) };
        sector[..hdr_bytes.len()].copy_from_slice(hdr_bytes);
        off += hdr_bytes.len();
        sector[off..off + LEN_FUEL].copy_from_slice(&fuel);
        off += LEN_FUEL;
        sector[off..off + LEN_IGN].copy_from_slice(&ign);
        off += LEN_IGN;
        sector[off..off + LEN_ANGLES].copy_from_slice(&angles);

        unsafe {
            // Erase and program
            Self::flash_erase(kv_base_off(), SECTOR_SIZE);
            Self::flash_program(kv_base_off(), &sector);
        }
        Ok(())
    }
}
