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
    self, store_integrity_from_boot_scan, BootScan, SlotContents, StoreIntegrityStatus,
    SLOT_USED_LEN,
};
use ecu_target_common::transport_service::{
    apply_retained_history_page_update,
    install_optional_obd2_retained_history_snapshot_halfwords_with,
    persist_retained_history_flash_rewrite, prepare_retained_history_preserved_page_rewrite,
    prepare_retained_history_snapshot_rewrite, read_obd2_retained_history_snapshot_with,
    Obd2RetainedDiagnosticHistorySnapshot, Obd2RetainedHistoryPageUpdateError,
};
#[cfg(test)]
use ecu_target_common::transport_service::{
    decode_obd2_retained_history_sidecar_at, encode_obd2_retained_history_sidecar_at,
};

// RP2040 XIP flash base
const XIP_BASE: u32 = 0x1000_0000;
// KV region offset from flash start (near end of 2 MiB flash).
const KV_OFFSET: u32 = 0x001F_0000; // 0x1000_0000 + 0x001F_0000 = 0x101F_0000
const SECTOR_SIZE: usize = 4096;
const PAGE_SIZE: usize = 256;
const SLOT_COUNT: usize = 2;
const OBD2_SNAPSHOT_OFFSET: usize = SLOT_USED_LEN;

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

fn install_retained_history_snapshot_into_sector(
    sector: &mut [u8; SECTOR_SIZE],
    snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
) -> Option<()> {
    install_optional_obd2_retained_history_snapshot_halfwords_with(snapshot, |offset, value| {
        sector[OBD2_SNAPSHOT_OFFSET + offset..OBD2_SNAPSHOT_OFFSET + offset + 2]
            .copy_from_slice(&value.to_le_bytes());
    })
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
    pub fn boot_integrity(&self) -> StoreIntegrityStatus {
        store_integrity_from_boot_scan(self.scan())
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

    fn load_current_obd2_snapshot(
        &self,
        current: BootScan,
    ) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
        if let BootScan::Valid { slot, .. } = current {
            read_obd2_retained_history_snapshot_with(|out| unsafe {
                let src = slot_base_ptr(slot).add(OBD2_SNAPSHOT_OFFSET);
                for (i, byte) in out.iter_mut().enumerate() {
                    *byte = core::ptr::read_volatile(src.add(i));
                }
            })
        } else {
            None
        }
    }

    pub fn load_retained_obd2_history_snapshot(
        &self,
    ) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
        self.load_current_obd2_snapshot(self.scan())
    }

    fn stage_retained_history_sector(
        pages: &SlotContents,
        snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
    ) -> Result<[u8; SECTOR_SIZE], KvError> {
        let mut sector = [0xFFu8; SECTOR_SIZE];
        let _ = ab::serialize_slot(pages, &mut sector);
        install_retained_history_snapshot_into_sector(&mut sector, snapshot).ok_or(KvError::Io)?;
        Ok(sector)
    }

    fn rewrite_retained_history_sector(
        target: u8,
        pages: &SlotContents,
        snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
    ) -> Result<(), KvError> {
        debug_assert!((target as usize) < SLOT_COUNT);
        let sector = Self::stage_retained_history_sector(pages, snapshot)?;
        unsafe {
            Self::flash_erase(target);
            Self::flash_program(target, &sector);
        }
        Ok(())
    }

    fn finalize_retained_history_rewrite(
        current: BootScan,
        mut rewrite: ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<
            SlotContents,
        >,
    ) -> (
        u8,
        ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<SlotContents>,
    ) {
        rewrite.pages.seq = ab::next_seq(current);
        let target = ab::write_target(current);
        (target, rewrite)
    }

    fn persist_retained_history_rewrite(
        &mut self,
        prepare: impl FnOnce(
            SlotContents,
            Option<Obd2RetainedDiagnosticHistorySnapshot>,
        ) -> Result<
            ecu_target_common::transport_service::Obd2RetainedHistoryFlashRewrite<SlotContents>,
            KvError,
        >,
    ) -> Result<(), KvError> {
        if self.engine_running() {
            return Err(KvError::EngineRunning);
        }
        let current = self.scan();
        persist_retained_history_flash_rewrite(
            self.load_current(current),
            self.load_current_obd2_snapshot(current),
            prepare,
            |rewrite| {
                let (target, rewrite) = Self::finalize_retained_history_rewrite(current, rewrite);
                Self::rewrite_retained_history_sector(
                    target,
                    &rewrite.pages,
                    rewrite.snapshot.as_ref(),
                )
            },
        )
    }

    pub fn save_retained_obd2_history_snapshot(
        &mut self,
        snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    ) -> Result<(), KvError> {
        self.persist_retained_history_rewrite(|current_pages, _| {
            Ok(prepare_retained_history_snapshot_rewrite(
                current_pages,
                snapshot,
            ))
        })
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
        self.persist_retained_history_rewrite(|current_pages, current_snapshot| {
            prepare_retained_history_preserved_page_rewrite(
                current_pages,
                current_snapshot,
                |pages| {
                    apply_retained_history_page_update(pages, key, data).map_err(
                        |error| match error {
                            Obd2RetainedHistoryPageUpdateError::UnknownKey => KvError::NotFound,
                            Obd2RetainedHistoryPageUpdateError::InvalidLength => KvError::Io,
                        },
                    )
                },
            )
        })?;
        Ok(())
    }
}

#[cfg(feature = "transport-can")]
impl ecu_target_common::transport_service::Obd2RetainedHistoryStore for FlashKv {
    type Error = KvError;

    fn load_retained_obd2_history_snapshot(&self) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
        FlashKv::load_retained_obd2_history_snapshot(self)
    }

    fn save_retained_obd2_history_snapshot(
        &mut self,
        snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    ) -> Result<(), Self::Error> {
        FlashKv::save_retained_obd2_history_snapshot(self, snapshot)
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

    #[test]
    fn obd2_snapshot_sidecar_roundtrip_preserves_shared_snapshot() {
        let snapshot = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: ecu_transport::Message::SensorData {
                map_kpa_x10: 321,
                tps_percent: 20,
                iat_offset: 66,
                clt_offset: 70,
                voltage_x10: 124,
                lambda_x100: 101,
                flags: 0,
                timestamp_us: 55,
            },
            freeze_frame_value_source: Some(ecu_transport::Message::SensorData {
                map_kpa_x10: 654,
                tps_percent: 30,
                iat_offset: 64,
                clt_offset: 68,
                voltage_x10: 123,
                lambda_x100: 99,
                flags: 1,
                timestamp_us: 77,
            }),
            stored_dtcs: [
                ecu_domain::diag::DiagCode::PersistCrcFault,
                ecu_domain::diag::DiagCode::MapRange,
                ecu_domain::diag::DiagCode::LowVoltage,
            ],
            stored_dtc_count: 2,
            freeze_frame_dtc: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
            current_diag_event: Some(ecu_domain::diag::DiagEvent {
                code: ecu_domain::diag::DiagCode::MapRange,
                timestamp: ecu_domain::Micros::new(77),
                source: ecu_domain::diag::DiagSource::Sensor,
                context: Some(88),
                start_us: 11,
                end_us: 22,
            }),
            freeze_frame_event: Some(ecu_domain::diag::DiagEvent {
                code: ecu_domain::diag::DiagCode::PersistCrcFault,
                timestamp: ecu_domain::Micros::new(99),
                source: ecu_domain::diag::DiagSource::User,
                context: None,
                start_us: 33,
                end_us: 44,
            }),
            last_fault: ecu_domain::FaultCode::CalibrationInvalid,
            last_observed_diag_code: Some(ecu_domain::diag::DiagCode::MapRange),
            last_observed_diag_timestamp_us: 77,
        };
        let mut slot = [0xFFu8; SECTOR_SIZE];
        assert!(encode_obd2_retained_history_sidecar_at(
            &snapshot,
            &mut slot,
            OBD2_SNAPSHOT_OFFSET
        ));
        assert_eq!(
            decode_obd2_retained_history_sidecar_at(&slot, OBD2_SNAPSHOT_OFFSET),
            Some(snapshot)
        );
    }

    #[test]
    fn install_retained_history_snapshot_into_sector_writes_at_shared_offset() {
        let snapshot = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: ecu_transport::Message::SensorData {
                map_kpa_x10: 321,
                tps_percent: 20,
                iat_offset: 66,
                clt_offset: 70,
                voltage_x10: 124,
                lambda_x100: 101,
                flags: 0,
                timestamp_us: 55,
            },
            freeze_frame_value_source: None,
            stored_dtcs: [ecu_domain::diag::DiagCode::PersistCrcFault; 3],
            stored_dtc_count: 1,
            freeze_frame_dtc: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
            current_diag_event: None,
            freeze_frame_event: None,
            last_fault: ecu_domain::FaultCode::CalibrationInvalid,
            last_observed_diag_code: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
            last_observed_diag_timestamp_us: 55,
        };
        let mut sector = [0xFFu8; SECTOR_SIZE];
        assert_eq!(
            install_retained_history_snapshot_into_sector(&mut sector, Some(&snapshot)),
            Some(())
        );
        assert_eq!(
            decode_obd2_retained_history_sidecar_at(&sector, OBD2_SNAPSHOT_OFFSET),
            Some(snapshot)
        );
    }

    #[test]
    fn install_retained_history_snapshot_into_sector_skips_none() {
        let mut sector = [0xAAu8; SECTOR_SIZE];
        assert_eq!(
            install_retained_history_snapshot_into_sector(&mut sector, None),
            Some(())
        );
        assert_eq!(sector, [0xAAu8; SECTOR_SIZE]);
    }

    #[test]
    fn stage_retained_history_sector_contains_pages_and_snapshot() {
        let mut pages = SlotContents::zeroed();
        pages.seq = 7;
        pages.fuel[0] = 0x11;
        pages.ign[0] = 0x22;
        pages.angles[0] = 0x33;
        let snapshot = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: ecu_transport::Message::SensorData {
                map_kpa_x10: 321,
                tps_percent: 20,
                iat_offset: 66,
                clt_offset: 70,
                voltage_x10: 124,
                lambda_x100: 101,
                flags: 0,
                timestamp_us: 55,
            },
            freeze_frame_value_source: None,
            stored_dtcs: [ecu_domain::diag::DiagCode::PersistCrcFault; 3],
            stored_dtc_count: 1,
            freeze_frame_dtc: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
            current_diag_event: None,
            freeze_frame_event: None,
            last_fault: ecu_domain::FaultCode::CalibrationInvalid,
            last_observed_diag_code: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
            last_observed_diag_timestamp_us: 55,
        };

        let sector = FlashKv::stage_retained_history_sector(&pages, Some(&snapshot)).unwrap();
        let mut fuel = [0u8; SECTOR_SIZE];
        let mut ign = [0u8; SECTOR_SIZE];
        let mut angles = [0u8; SECTOR_SIZE];
        assert_eq!(
            ab::read_key(&sector, b"fuel", &mut fuel),
            Some(pages.fuel.len())
        );
        assert_eq!(
            ab::read_key(&sector, b"ign", &mut ign),
            Some(pages.ign.len())
        );
        assert_eq!(
            ab::read_key(&sector, b"angles", &mut angles),
            Some(pages.angles.len())
        );
        assert_eq!(&fuel[..pages.fuel.len()], &pages.fuel);
        assert_eq!(&ign[..pages.ign.len()], &pages.ign);
        assert_eq!(&angles[..pages.angles.len()], &pages.angles);
        assert_eq!(
            decode_obd2_retained_history_sidecar_at(&sector, OBD2_SNAPSHOT_OFFSET),
            Some(snapshot)
        );
    }

    #[test]
    fn finalize_retained_history_rewrite_stamps_seq_and_target_from_valid_scan() {
        let current = BootScan::Valid {
            slot: 1,
            seq: 41,
            saw_corrupt: false,
        };
        let rewrite = prepare_retained_history_snapshot_rewrite(
            SlotContents::zeroed(),
            &Obd2RetainedDiagnosticHistorySnapshot {
                current_data_value_source: ecu_transport::Message::SensorData {
                    map_kpa_x10: 321,
                    tps_percent: 20,
                    iat_offset: 66,
                    clt_offset: 70,
                    voltage_x10: 124,
                    lambda_x100: 101,
                    flags: 0,
                    timestamp_us: 55,
                },
                freeze_frame_value_source: None,
                stored_dtcs: [ecu_domain::diag::DiagCode::PersistCrcFault; 3],
                stored_dtc_count: 1,
                freeze_frame_dtc: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
                current_diag_event: None,
                freeze_frame_event: None,
                last_fault: ecu_domain::FaultCode::CalibrationInvalid,
                last_observed_diag_code: Some(ecu_domain::diag::DiagCode::PersistCrcFault),
                last_observed_diag_timestamp_us: 55,
            },
        );
        let (target, rewrite) = FlashKv::finalize_retained_history_rewrite(current, rewrite);
        assert_eq!(target, 0);
        assert_eq!(rewrite.pages.seq, 42);
    }

    #[test]
    fn finalize_retained_history_rewrite_uses_blank_defaults() {
        let rewrite =
            prepare_retained_history_preserved_page_rewrite(SlotContents::zeroed(), None, |_| {
                Ok::<(), KvError>(())
            })
            .unwrap();
        let (target, rewrite) =
            FlashKv::finalize_retained_history_rewrite(BootScan::Blank, rewrite);
        assert_eq!(target, 0);
        assert_eq!(rewrite.pages.seq, 1);
    }
}
