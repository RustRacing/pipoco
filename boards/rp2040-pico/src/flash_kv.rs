#![cfg(feature = "flash-kv")]
//! Flash-backed key/value store for RP2040 Pico (two-slot A/B layout).
//!
//! Implements `ecu_calibration::KvStore` over two reserved flash sectors using
//! the RP2040 ROM flash routines. The slot serialization, CRC, and boot
//! selection logic live in `ecu_target_common::kv::ab` so they are host-tested
//! independently of the flash device.
//!
//! - Slot A: 4 KiB sector at `KV_OFFSET`.
//! - Slot B: 4 KiB sector at `KV_OFFSET + SECTOR_SIZE`.
//! - Each slot holds an A/B header (magic, version, monotonic seq, lengths,
//!   per-key CRC16) followed by the fuel/ign/angles payload.
//! - Writes target the stale slot and stamp `seq + 1`; the slot's header write
//!   is the commit point. Boot selects the highest-seq slot with valid CRCs.
//! - A power loss mid-write corrupts only the stale slot, so boot still finds
//!   the previously committed slot intact.

use ecu_calibration::{KvError, KvStore};
use ecu_target_common::kv::ab::{
    self, BootScan, SlotContents, LEN_ANGLES, LEN_FUEL, LEN_IGN, SLOT_USED_LEN,
};

// RP2040 XIP flash base
const XIP_BASE: u32 = 0x1000_0000;
// KV region offset from flash start (near end of 2 MiB flash).
const KV_OFFSET: u32 = 0x001F_0000; // 0x1000_0000 + 0x001F_0000 = 0x101F_0000
const SECTOR_SIZE: usize = 4096;
const PAGE_SIZE: usize = 256;
const SLOT_COUNT: usize = 2;

const fn slot_offset(slot: u8) -> u32 {
    KV_OFFSET + (slot as u32) * (SECTOR_SIZE as u32)
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum FlashLayoutError {
    RegionOverflow,
    UnalignedErase,
    UnalignedProgram,
}

const fn validate_layout() -> Result<(), FlashLayoutError> {
    if SLOT_USED_LEN > SECTOR_SIZE {
        return Err(FlashLayoutError::RegionOverflow);
    }
    if !SECTOR_SIZE.is_multiple_of(4096) {
        return Err(FlashLayoutError::UnalignedErase);
    }
    if !SECTOR_SIZE.is_multiple_of(PAGE_SIZE) {
        return Err(FlashLayoutError::UnalignedProgram);
    }
    Ok(())
}

const _: () = match validate_layout() {
    Ok(()) => (),
    Err(_) => panic!("rp2040 flash KV layout is invalid"),
};

/// Boot-time integrity status of the persisted calibration, surfaced so the
/// boot path can latch a diagnostic fault on corruption (ADR-0003).
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum BootIntegrity {
    /// No persisted slot present: first boot, defaults loaded silently.
    Blank,
    /// A valid slot was selected and the sibling slot was clean.
    Valid,
    /// A valid slot was selected, but the sibling slot carried our format and
    /// failed validation: a torn write rolled back to the older committed
    /// tune. The selected tune is good; the tuner's last write did not stick,
    /// so a diagnostic is surfaced while still booting the valid slot.
    ValidWithCorruptSibling,
    /// A slot carried our format but failed CRC/header validation.
    Corrupt,
}

fn slot_base_ptr(slot: u8) -> *const u8 {
    (XIP_BASE + slot_offset(slot)) as *const u8
}

fn read_slot_into(slot: u8, out: &mut [u8; SECTOR_SIZE]) {
    let src = slot_base_ptr(slot);
    unsafe {
        for (i, byte) in out.iter_mut().enumerate() {
            *byte = core::ptr::read_volatile(src.add(i));
        }
    }
}

pub struct FlashKv {
    /// Opaque engine-running context owned by board code.
    /// # Safety Invariant: When non-null, this pointer must remain valid for
    /// the lifetime of FlashKv and match `engine_running_predicate`.
    engine_running_context: *const (),
    engine_running_predicate: fn(*const ()) -> bool,
}

impl FlashKv {
    /// Construct FlashKv with an opaque engine-running guard.
    ///
    /// # Safety Invariant
    /// - The context pointer must remain valid for the lifetime of FlashKv.
    /// - The predicate must interpret the context pointer using the same type.
    /// - Mutable page access remains owned by the caller of the `KvStore` API.
    pub const fn new_with_engine_guard(
        engine_running_context: *const (),
        engine_running_predicate: fn(*const ()) -> bool,
    ) -> Self {
        Self {
            engine_running_context,
            engine_running_predicate,
        }
    }

    fn engine_running(&self) -> bool {
        let context = self.engine_running_context;
        if context.is_null() {
            return false;
        }
        (self.engine_running_predicate)(context)
    }

    fn scan(&self) -> BootScan {
        let mut a = [0xFFu8; SECTOR_SIZE];
        let mut b = [0xFFu8; SECTOR_SIZE];
        read_slot_into(0, &mut a);
        read_slot_into(1, &mut b);
        ab::scan(&a, &b)
    }

    /// Read-only boot integrity classification (does not erase or program).
    pub fn boot_integrity(&self) -> BootIntegrity {
        match self.scan() {
            BootScan::Blank => BootIntegrity::Blank,
            BootScan::Valid {
                saw_corrupt: true, ..
            } => BootIntegrity::ValidWithCorruptSibling,
            BootScan::Valid { .. } => BootIntegrity::Valid,
            BootScan::Corrupt => BootIntegrity::Corrupt,
        }
    }

    fn load_current(&self, current: BootScan) -> SlotContents {
        let mut c = SlotContents::zeroed();
        if let BootScan::Valid { slot, seq, .. } = current {
            let mut buf = [0xFFu8; SECTOR_SIZE];
            read_slot_into(slot, &mut buf);
            c.seq = seq;
            let _ = ab::read_key(&buf, b"fuel", &mut c.fuel);
            let _ = ab::read_key(&buf, b"ign", &mut c.ign);
            let _ = ab::read_key(&buf, b"angles", &mut c.angles);
        }
        c
    }

    unsafe fn flash_erase(slot: u8) {
        use rp2040_hal::rom_data::{
            flash_enter_cmd_xip, flash_exit_xip, flash_flush_cache, flash_range_erase,
        };
        flash_exit_xip();
        flash_range_erase(slot_offset(slot), SECTOR_SIZE, 4096, 0x20);
        flash_flush_cache();
        flash_enter_cmd_xip();
    }

    unsafe fn flash_program(slot: u8, data: &[u8]) {
        use rp2040_hal::rom_data::{
            flash_enter_cmd_xip, flash_exit_xip, flash_flush_cache, flash_range_program,
        };
        flash_exit_xip();
        let base = slot_offset(slot);
        let mut off = 0usize;
        while off < data.len() {
            let end = core::cmp::min(off + PAGE_SIZE, data.len());
            let ptr = data.as_ptr().add(off);
            flash_range_program(base + off as u32, ptr, end - off);
            off = end;
        }
        flash_flush_cache();
        flash_enter_cmd_xip();
    }
}

impl KvStore for FlashKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        match self.scan() {
            BootScan::Valid { slot, .. } => {
                let mut buf = [0xFFu8; SECTOR_SIZE];
                read_slot_into(slot, &mut buf);
                match key {
                    b"fuel" | b"ign" | b"angles" => ab::read_key(&buf, key, out).ok_or(KvError::Io),
                    _ => Err(KvError::NotFound),
                }
            }
            _ => Err(KvError::NotFound),
        }
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        if self.engine_running() {
            return Err(KvError::EngineRunning);
        }
        match key {
            b"fuel" if data.len() != LEN_FUEL => return Err(KvError::Io),
            b"ign" if data.len() != LEN_IGN => return Err(KvError::Io),
            b"angles" if data.len() != LEN_ANGLES => return Err(KvError::Io),
            b"fuel" | b"ign" | b"angles" => {}
            _ => return Err(KvError::NotFound),
        }

        let current = self.scan();
        let mut c = self.load_current(current);
        match key {
            b"fuel" => c.fuel.copy_from_slice(data),
            b"ign" => c.ign.copy_from_slice(data),
            b"angles" => c.angles.copy_from_slice(data),
            _ => return Err(KvError::NotFound),
        }
        c.seq = ab::next_seq(current);
        let target = ab::write_target(current);
        debug_assert!((target as usize) < SLOT_COUNT);

        let mut sector = [0xFFu8; SECTOR_SIZE];
        let _ = ab::serialize_slot(&c, &mut sector);
        unsafe {
            Self::flash_erase(target);
            Self::flash_program(target, &sector);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn layout_fits_one_sector_and_program_granularity() {
        assert_eq!(validate_layout(), Ok(()));
        assert!(SLOT_USED_LEN <= SECTOR_SIZE);
        assert_eq!(SECTOR_SIZE % PAGE_SIZE, 0);
    }

    #[test]
    fn slot_offsets_are_distinct_consecutive_sectors() {
        assert_eq!(slot_offset(0), KV_OFFSET);
        assert_eq!(slot_offset(1), KV_OFFSET + SECTOR_SIZE as u32);
    }
}
