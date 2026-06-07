#![cfg(feature = "flash-kv")]
#![allow(dead_code)]
//! Sequential-storage backed KV for RP2040 flash
//!
//! Uses `sequential-storage` over a small reserved flash region, implementing
//! the ecu_calibration::KvStore interface.

use ecu_calibration::{KvError, KvStore};
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};
use sequential_storage::map::{self, StorageItem, StorageItemError};

// Reserved flash region (offset from flash start)
const REGION_OFFSET: u32 = 0x001E_0000; // 128 KiB from end on 2MiB flash
const REGION_SIZE: usize = 64 * 1024; // 64 KiB region for logs

// RP2040 flash characteristics
const ERASE_SIZE: usize = 4096; // 4 KiB sectors
const WRITE_CHUNK: usize = 256; // ROM program granularity

// Item keys
const KEY_FUEL: u8 = 1;
const KEY_IGN: u8 = 2;
const TABLE_LEN: usize = 512;
const STORAGE_RANGE: core::ops::Range<u32> = 0..REGION_SIZE as u32;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum RpFlashBoundsError {
    OutOfRange,
    UnalignedErase,
}

fn validate_region_access(offset: u32, len: usize) -> Result<(), RpFlashBoundsError> {
    if len > REGION_SIZE {
        return Err(RpFlashBoundsError::OutOfRange);
    }
    if offset as usize > REGION_SIZE.saturating_sub(len) {
        return Err(RpFlashBoundsError::OutOfRange);
    }
    Ok(())
}

fn validate_erase_range(from: u32, to: u32) -> Result<(), RpFlashBoundsError> {
    if to < from {
        return Err(RpFlashBoundsError::OutOfRange);
    }
    validate_region_access(from, (to - from) as usize)?;
    if (from as usize) % ERASE_SIZE != 0 || (to as usize) % ERASE_SIZE != 0 {
        return Err(RpFlashBoundsError::UnalignedErase);
    }
    Ok(())
}

#[derive(Debug, Copy, Clone)]
pub struct FlashErr;
impl NorFlashError for FlashErr {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

// RP2040 XIP flash adapter implementing embedded-storage NorFlash
pub struct RpFlash;

impl ErrorType for RpFlash {
    type Error = FlashErr;
}

impl RpFlash {
    const CAP: usize = REGION_SIZE;
    const BASE: u32 = REGION_OFFSET;

    fn abs(off: u32) -> Result<u32, FlashErr> {
        validate_region_access(off, 0).map_err(|_| FlashErr)?;
        Ok(Self::BASE + off)
    }

    fn read_block(addr: u32, out: &mut [u8]) -> Result<(), FlashErr> {
        validate_region_access(addr - Self::BASE, out.len()).map_err(|_| FlashErr)?;
        let src = (0x1000_0000 + addr) as *const u8;
        for i in 0..out.len() {
            // SAFETY: `validate_region_access` proves the address range is
            // inside the reserved XIP-backed flash window.
            unsafe {
                out[i] = core::ptr::read_volatile(src.add(i));
            }
        }
        Ok(())
    }

    unsafe fn rom_erase(addr_off: u32, len: usize) {
        use rp2040_hal::rom_data::{
            flash_enter_cmd_xip, flash_exit_xip, flash_flush_cache, flash_range_erase,
        };
        flash_exit_xip();
        let mut r = len;
        let mut off = addr_off;
        while r > 0 {
            flash_range_erase(off, ERASE_SIZE, ERASE_SIZE as u32, 0x20);
            off += ERASE_SIZE as u32;
            r = r.saturating_sub(ERASE_SIZE);
        }
        flash_flush_cache();
        flash_enter_cmd_xip();
    }

    unsafe fn rom_program(addr_off: u32, data: &[u8]) {
        use rp2040_hal::rom_data::{
            flash_enter_cmd_xip, flash_exit_xip, flash_flush_cache, flash_range_program,
        };
        flash_exit_xip();
        let mut off = 0usize;
        while off < data.len() {
            let end = core::cmp::min(off + WRITE_CHUNK, data.len());
            flash_range_program(addr_off + off as u32, data[off..end].as_ptr(), end - off);
            off = end;
        }
        flash_flush_cache();
        flash_enter_cmd_xip();
    }
}

impl ReadNorFlash for RpFlash {
    const READ_SIZE: usize = 1;
    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::check_read(self, offset, bytes.len()).map_err(|_| FlashErr)?;
        validate_region_access(offset, bytes.len()).map_err(|_| FlashErr)?;
        Self::read_block(Self::abs(offset)?, bytes)?;
        Ok(())
    }
    fn capacity(&self) -> usize {
        Self::CAP
    }
}

impl NorFlash for RpFlash {
    const WRITE_SIZE: usize = 1;
    const ERASE_SIZE: usize = ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::check_erase(self, from, to).map_err(|_| FlashErr)?;
        validate_erase_range(from, to).map_err(|_| FlashErr)?;
        unsafe { Self::rom_erase(Self::abs(from)?, (to - from) as usize) };
        Ok(())
    }
    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        embedded_storage::nor_flash::check_write(self, offset, bytes.len())
            .map_err(|_| FlashErr)?;
        validate_region_access(offset, bytes.len()).map_err(|_| FlashErr)?;
        // Read-modify-write the containing sectors; keep it simple per 4K page
        let mut remaining = bytes.len();
        let mut src_off = 0usize;
        while remaining > 0 {
            let abs = (offset as usize + src_off) as u32;
            let page_start_off = (abs as usize / ERASE_SIZE) * ERASE_SIZE;
            let within = (abs as usize) - page_start_off;
            // amount to write in this page
            let write_len = core::cmp::min(remaining, ERASE_SIZE - within);

            // Load page
            let mut page = [0xFFu8; ERASE_SIZE];
            Self::read_block(Self::abs(page_start_off as u32)?, &mut page)?;
            page[within..within + write_len].copy_from_slice(&bytes[src_off..src_off + write_len]);

            // Erase + Program
            unsafe {
                let page_addr = Self::abs(page_start_off as u32)?;
                Self::rom_erase(page_addr, ERASE_SIZE);
                Self::rom_program(page_addr, &page);
            }

            src_off += write_len;
            remaining -= write_len;
        }
        Ok(())
    }
}

// Storage item for sequential-storage. Encodes a key and 512 bytes payload.
#[derive(Debug, PartialEq, Eq)]
struct TableItem {
    key: u8,
    payload: [u8; TABLE_LEN],
}

#[derive(Debug, PartialEq, Eq)]
enum TableItemErr {
    BufferTooSmall,
    BadHeader,
}
impl StorageItemError for TableItemErr {
    fn is_buffer_too_small(&self) -> bool {
        matches!(self, TableItemErr::BufferTooSmall)
    }
}

impl StorageItem for TableItem {
    type Key = u8;
    type Error = TableItemErr;

    fn serialize_into(&self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        // Header: magic 2B, key 1B, len 2B, reserved 1B, CRC16 2B = 8B
        const HDR: usize = 8;
        if buf.len() < HDR + TABLE_LEN {
            return Err(TableItemErr::BufferTooSmall);
        }
        buf[0] = 0x49;
        buf[1] = 0x50; // 'IP'
        buf[2] = self.key;
        buf[3..5].copy_from_slice(&(TABLE_LEN as u16).to_le_bytes());
        buf[5] = 0x00;
        buf[HDR..HDR + TABLE_LEN].copy_from_slice(&self.payload);
        let crc = crc16_ccitt(0xFFFF, &buf[HDR..HDR + TABLE_LEN]);
        buf[6] = (crc & 0xFF) as u8;
        buf[7] = (crc >> 8) as u8;
        Ok(HDR + TABLE_LEN)
    }

    fn deserialize_from(buf: &[u8]) -> Result<(Self, usize), Self::Error>
    where
        Self: Sized,
    {
        const HDR: usize = 8;
        if buf.len() < HDR {
            return Err(TableItemErr::BufferTooSmall);
        }
        if buf[0] != 0x49 || buf[1] != 0x50 {
            return Err(TableItemErr::BadHeader);
        }
        let key = buf[2];
        let len = (buf[4] as usize) << 8 | (buf[3] as usize);
        if len != TABLE_LEN || buf.len() < HDR + TABLE_LEN {
            return Err(TableItemErr::BufferTooSmall);
        }
        let mut payload = [0u8; TABLE_LEN];
        payload.copy_from_slice(&buf[HDR..HDR + TABLE_LEN]);
        let crc = ((buf[7] as u16) << 8) | (buf[6] as u16);
        if crc16_ccitt(0xFFFF, &payload) != crc {
            return Err(TableItemErr::BadHeader);
        }
        Ok((Self { key, payload }, HDR + 512))
    }

    fn key(&self) -> Self::Key {
        self.key
    }
}

fn crc16_ccitt(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            crc = if (crc & 0x8000) != 0 {
                (crc << 1) ^ 0x1021
            } else {
                crc << 1
            };
        }
    }
    crc
}

pub struct SeqKv;

impl SeqKv {
    pub const fn new() -> Self {
        Self
    }
}

impl KvStore for SeqKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        if out.len() < TABLE_LEN {
            return Err(KvError::Io);
        }
        let k = match key {
            b"fuel" => KEY_FUEL,
            b"ign" => KEY_IGN,
            _ => return Err(KvError::NotFound),
        };
        let mut flash = RpFlash;
        match map::fetch_item::<TableItem, _>(&mut flash, STORAGE_RANGE, k) {
            Ok(Some(item)) => {
                out[..TABLE_LEN].copy_from_slice(&item.payload);
                Ok(TABLE_LEN)
            }
            Ok(None) => Err(KvError::NotFound),
            Err(_) => Err(KvError::Io),
        }
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        if data.len() != TABLE_LEN {
            return Err(KvError::Io);
        }
        let k = match key {
            b"fuel" => KEY_FUEL,
            b"ign" => KEY_IGN,
            _ => return Err(KvError::NotFound),
        };
        let mut pl = [0u8; TABLE_LEN];
        pl.copy_from_slice(data);
        let item = TableItem {
            key: k,
            payload: pl,
        };
        let mut flash = RpFlash;
        map::store_item::<TableItem, _, { ERASE_SIZE }>(&mut flash, STORAGE_RANGE, item)
            .map_err(|_| KvError::Io)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rp_flash_bounds_reject_out_of_region_access() {
        assert_eq!(validate_region_access(0, REGION_SIZE), Ok(()));
        assert_eq!(
            validate_region_access(REGION_SIZE as u32, 1),
            Err(RpFlashBoundsError::OutOfRange)
        );
        assert_eq!(
            validate_region_access((REGION_SIZE - 1) as u32, 2),
            Err(RpFlashBoundsError::OutOfRange)
        );
    }

    #[test]
    fn erase_range_requires_sector_alignment() {
        assert_eq!(validate_erase_range(0, ERASE_SIZE as u32), Ok(()));
        assert_eq!(
            validate_erase_range(1, ERASE_SIZE as u32),
            Err(RpFlashBoundsError::UnalignedErase)
        );
        assert_eq!(
            validate_erase_range(0, (REGION_SIZE + ERASE_SIZE) as u32),
            Err(RpFlashBoundsError::OutOfRange)
        );
    }

    #[test]
    fn table_item_rejects_short_or_corrupt_payload() {
        let item = TableItem {
            key: KEY_FUEL,
            payload: [0x5a; TABLE_LEN],
        };
        let mut buf = [0u8; 520];
        let len = item.serialize_into(&mut buf).unwrap();
        assert_eq!(len, 520);
        assert_eq!(
            TableItem::deserialize_from(&buf[..len]).unwrap(),
            (item, len)
        );

        assert_eq!(
            TableItem::deserialize_from(&buf[..7]),
            Err(TableItemErr::BufferTooSmall)
        );
        buf[8] ^= 1;
        assert_eq!(
            TableItem::deserialize_from(&buf[..len]),
            Err(TableItemErr::BadHeader)
        );
    }
}
