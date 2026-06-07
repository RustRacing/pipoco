//! TunerStudio support module for STM32F4 target.
//!
//! This module intentionally isolates all TS-related state and persistence
//! plumbing from the main entrypoint. Behavior remains unchanged and all public
//! entrypoints are wired through narrow helpers used by `main.rs`.

use core::cell::RefCell;
use ecu_calibration::EcuConfig;
use ecu_calibration::FuelRuntimeTune;
#[cfg(feature = "flash-kv")]
use ecu_calibration::{KvError, KvStore};
use ecu_domain::diag::DiagLog;
use ecu_domain::Micros;
#[cfg(any(feature = "flash-kv", test))]
use ecu_target_common::kv::layout::{
    ANGLES_PAGE_LEN, FUEL_PAGE_LEN, IGN_PAGE_LEN, PAGE_HEADER_LEN,
};
use ecu_target_common::ts::page_store::{
    build_ecu_page_store, build_system_snapshot, diag_log_entries_from, PageRuntime, SnapshotInputs,
};
use ecu_target_common::ts::service::TsService;
use ecu_target_common::ts::state_ptr::{StatePtr, StateRef};
use ecu_ts::outpc::Outpc;
use ecu_ts::pages::{EcuPageStore, ExpertTriggerPageState, SystemSnapshot, DIAG_LOG_ENTRY_COUNT};
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore};
use ecu_ts::server::OutpcProvider;
use ecu_ts::RuntimeSnapshotAdapter;
#[cfg(feature = "flash-kv")]
use stm32f4xx_hal::pac;

use cortex_m::interrupt::free;
use cortex_m::interrupt::Mutex;
#[cfg(not(feature = "flash-kv"))]
use ecu_target_common::kv::ram::RamKv512;

/// Board-owned TunerStudio state, replacing the legacy compatibility-core
/// engine-state singleton. STM32F4 is a bring-up target with no flash-kv
/// calibration source, so the config starts at calibration defaults and the
/// runtime page surface is served with zeroed/idle snapshot scalars (the board
/// never ran the core snapshot refresh, so this preserves the prior byte
/// output).
pub(crate) struct BoardTsState {
    pub config: EcuConfig,
    diag_log: DiagLog<DIAG_LOG_ENTRY_COUNT>,
    snapshot: SystemSnapshot,
    expert_trigger: ExpertTriggerPageState,
    emerg_trig_map: bool,
    emerg_trig_tps: bool,
    tooth_count: u8,
    sync_loss_counter: u16,
}

/// Calibration-default [`EcuConfig`] for STM32F4 bring-up.
///
/// Mirrors the field values the legacy compatibility-core engine state produced
/// at construction, so the served calibration pages are byte-identical to the
/// previous behavior.
pub(crate) const fn default_ecu_config() -> EcuConfig {
    use ecu_calibration::configs::{
        AeConfig, AseConfig, ClConfig, DfcoConfig, FanConfig, IdleConfig, LambdaConfig,
        LoadFailureConfig, PlausibilityConfig, RateConfig, RevLimiterConfig, SensorsLimits,
        WueConfig,
    };
    use ecu_calibration::sensors::SensorsCal;

    EcuConfig {
        ipw_table: [[1000; 16]; 16],
        ve_table: [[100; 16]; 16],
        afr_table: [[147; 16]; 16],
        required_fuel_us: 1000,
        injector_deadtime_us: 800,
        ve_load_source: 0,
        ignition_table: [[15; 16]; 16],
        sensors_cal: SensorsCal::default(),
        sensors_limits: SensorsLimits::default(),
        ae_config: AeConfig::DEFAULT,
        wue_config: WueConfig::DEFAULT,
        ase_config: AseConfig::DEFAULT,
        dfco_config: DfcoConfig::DEFAULT,
        idle_config: IdleConfig::DEFAULT,
        fan_config: FanConfig::DEFAULT,
        cl_config: ClConfig::DEFAULT,
        load_failure_config: LoadFailureConfig::DEFAULT,
        plausibility_config: PlausibilityConfig::DEFAULT,
        rate_config: RateConfig::DEFAULT,
        lambda_config: LambdaConfig::DEFAULT,
        rev_limiter_config: RevLimiterConfig::DEFAULT,
        inj_angle_btdc_x10: [0; 16],
        tdc_per_cyl_x10: [0; 16],
        tooth0_angle_x10: 0,
        cam_missing_timeout_ms: 500,
    }
}

impl BoardTsState {
    pub(crate) fn new() -> Self {
        Self {
            config: default_ecu_config(),
            diag_log: DiagLog::new(),
            snapshot: build_system_snapshot(SnapshotInputs {
                rpm: 0,
                synced: false,
                base_pw_us: 0,
                enrich_mult_x100: 100,
                stft_x10: 0,
                fuel_mult_x100: 100,
                final_pw: Micros::new(0),
                last_fault_code: 0,
                isr_count: 0,
                isr_max_us: 0,
                isr_avg_us: 0,
            }),
            expert_trigger: ExpertTriggerPageState::new(),
            emerg_trig_map: false,
            emerg_trig_tps: false,
            tooth_count: 0,
            sync_loss_counter: 0,
        }
    }

    fn page_store(&mut self) -> EcuPageStore<'_> {
        let diag_log_entries = diag_log_entries_from(&self.diag_log);
        build_ecu_page_store(
            &mut self.config,
            PageRuntime {
                snapshot: &self.snapshot,
                tooth_count: &self.tooth_count,
                sync_loss_counter: self.sync_loss_counter,
                emerg_trig_map: &mut self.emerg_trig_map,
                emerg_trig_tps: &mut self.emerg_trig_tps,
                diag_log_entries,
                expert_trigger: &mut self.expert_trigger,
            },
        )
    }
}

// TS-only board overlay for values not already provided by the runtime snapshot.
#[derive(Copy, Clone)]
struct TsSensorOverlay {
    tps_percent: u8,
    clt_c: i16,
    iat_c: i16,
    vbatt_mv: u16,
}

impl TsSensorOverlay {
    const fn new() -> Self {
        Self {
            tps_percent: 0,
            clt_c: 20,
            iat_c: 25,
            vbatt_mv: 12500,
        }
    }

    fn update(&mut self) {
        self.vbatt_mv = 12500;
    }
}

static TS_SENSORS: Mutex<RefCell<TsSensorOverlay>> =
    Mutex::new(RefCell::new(TsSensorOverlay::new()));

pub(crate) fn update_sensors() {
    free(|cs| {
        TS_SENSORS.borrow(cs).borrow_mut().update();
    });
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg(any(feature = "flash-kv", test))]
enum FlashPageKey {
    Fuel,
    Ign,
    Angles,
}

#[cfg(any(feature = "flash-kv", test))]
impl FlashPageKey {
    const fn from_key(key: &[u8]) -> Option<Self> {
        if matches_key(key, b"fuel") {
            Some(Self::Fuel)
        } else if matches_key(key, b"ign") {
            Some(Self::Ign)
        } else if matches_key(key, b"angles") {
            Some(Self::Angles)
        } else {
            None
        }
    }
}

#[cfg(any(feature = "flash-kv", test))]
const fn matches_key(lhs: &[u8], rhs: &[u8]) -> bool {
    if lhs.len() != rhs.len() {
        return false;
    }
    let mut i = 0;
    while i < lhs.len() {
        if lhs[i] != rhs[i] {
            return false;
        }
        i += 1;
    }
    true
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg(any(feature = "flash-kv", test))]
struct FlashPageLayout {
    offset: usize,
    len: usize,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg(any(feature = "flash-kv", test))]
struct FlashLayout {
    base_a: u32,
    base_b: u32,
    sector_a: u8,
    sector_b: u8,
    header_len: usize,
    fuel: FlashPageLayout,
    ign: FlashPageLayout,
    angles: FlashPageLayout,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg(any(feature = "flash-kv", test))]
enum FlashLayoutError {
    HeaderTooSmall,
    PageOverflow,
    SectorAlias,
}

#[cfg(any(feature = "flash-kv", test))]
impl FlashLayout {
    const MAGIC: u32 = 0x3250_4B56; // 'V''K''P''2' little-endian
    const VALID_COMMITTED: u16 = 0x0000;
    const INVALID_ERASED: u16 = 0xFFFF;
    const SECTOR_BYTES: usize = 128 * 1024;

    const STM32F405_TS_KV: Result<Self, FlashLayoutError> = Self::new(
        0x080C_0000,
        0x080E_0000,
        10,
        11,
        PAGE_HEADER_LEN,
        FUEL_PAGE_LEN,
        IGN_PAGE_LEN,
        ANGLES_PAGE_LEN,
    );

    const fn new(
        base_a: u32,
        base_b: u32,
        sector_a: u8,
        sector_b: u8,
        header_len: usize,
        fuel_len: usize,
        ign_len: usize,
        angles_len: usize,
    ) -> Result<Self, FlashLayoutError> {
        if header_len < 22 {
            return Err(FlashLayoutError::HeaderTooSmall);
        }
        if base_a == base_b || sector_a == sector_b {
            return Err(FlashLayoutError::SectorAlias);
        }
        let fuel = FlashPageLayout {
            offset: header_len,
            len: fuel_len,
        };
        let ign = FlashPageLayout {
            offset: header_len + fuel_len,
            len: ign_len,
        };
        let angles = FlashPageLayout {
            offset: header_len + fuel_len + ign_len,
            len: angles_len,
        };
        if angles.offset + angles.len > Self::SECTOR_BYTES {
            return Err(FlashLayoutError::PageOverflow);
        }
        Ok(Self {
            base_a,
            base_b,
            sector_a,
            sector_b,
            header_len,
            fuel,
            ign,
            angles,
        })
    }

    const fn page(self, key: FlashPageKey) -> FlashPageLayout {
        match key {
            FlashPageKey::Fuel => self.fuel,
            FlashPageKey::Ign => self.ign,
            FlashPageKey::Angles => self.angles,
        }
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
#[cfg(any(feature = "flash-kv", test))]
enum ActiveSector {
    A,
    B,
}

#[cfg(any(feature = "flash-kv", test))]
fn newest_sector(a: KvHeader, b: KvHeader) -> Option<ActiveSector> {
    match (a.ok, b.ok) {
        (true, true) => {
            if b.seq.wrapping_sub(a.seq) < 0x8000 {
                Some(ActiveSector::B)
            } else {
                Some(ActiveSector::A)
            }
        }
        (true, false) => Some(ActiveSector::A),
        (false, true) => Some(ActiveSector::B),
        (false, false) => None,
    }
}

#[cfg(any(feature = "flash-kv", test))]
fn target_sector_for_next_write(a: KvHeader, b: KvHeader) -> ActiveSector {
    match newest_sector(a, b) {
        Some(ActiveSector::A) => ActiveSector::B,
        Some(ActiveSector::B) => ActiveSector::A,
        None => ActiveSector::A,
    }
}

#[cfg(any(feature = "flash-kv", test))]
fn header_page_len(hdr: KvHeader, key: FlashPageKey) -> u16 {
    match key {
        FlashPageKey::Fuel => hdr.fuel_len,
        FlashPageKey::Ign => hdr.ign_len,
        FlashPageKey::Angles => hdr.angles_len,
    }
}

#[cfg(any(feature = "flash-kv", test))]
fn header_page_crc(hdr: KvHeader, key: FlashPageKey) -> u16 {
    match key {
        FlashPageKey::Fuel => hdr.fuel_crc,
        FlashPageKey::Ign => hdr.ign_crc,
        FlashPageKey::Angles => hdr.angles_crc,
    }
}

#[cfg(any(feature = "flash-kv", test))]
fn crc16(data: &[u8]) -> u16 {
    ecu_ts::proto::crc16_ccitt(data)
}

#[cfg(any(feature = "flash-kv", test))]
fn validate_page_image(layout: FlashLayout, hdr: KvHeader, key: FlashPageKey, data: &[u8]) -> bool {
    let page = layout.page(key);
    hdr.ok
        && header_page_len(hdr, key) as usize == page.len
        && data.len() == page.len
        && crc16(data) == header_page_crc(hdr, key)
}

#[cfg(feature = "flash-kv")]
pub struct FlashKv;

#[derive(Copy, Clone)]
#[cfg(any(feature = "flash-kv", test))]
struct KvHeader {
    ok: bool,
    seq: u16,
    fuel_len: u16,
    ign_len: u16,
    angles_len: u16,
    fuel_crc: u16,
    ign_crc: u16,
    angles_crc: u16,
}

#[cfg(feature = "flash-kv")]
impl FlashKv {
    pub const fn new() -> Self {
        Self
    }

    const LAYOUT: FlashLayout = match FlashLayout::STM32F405_TS_KV {
        Ok(layout) => layout,
        Err(_) => panic!("invalid STM32F4 TS flash KV layout"),
    };
}

#[cfg(feature = "flash-kv")]
impl KvStore for FlashKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        unsafe fn read_header(base: u32) -> KvHeader {
            let p = base as *const u8;
            let magic = core::ptr::read_volatile(p as *const u32);
            if magic != FlashLayout::MAGIC {
                return KvHeader {
                    ok: false,
                    seq: 0,
                    fuel_len: 0,
                    ign_len: 0,
                    angles_len: 0,
                    fuel_crc: 0,
                    ign_crc: 0,
                    angles_crc: 0,
                };
            }
            let valid = core::ptr::read_volatile(p.add(6) as *const u16);
            if valid != 0 {
                return KvHeader {
                    ok: false,
                    seq: 0,
                    fuel_len: 0,
                    ign_len: 0,
                    angles_len: 0,
                    fuel_crc: 0,
                    ign_crc: 0,
                    angles_crc: 0,
                };
            }
            KvHeader {
                ok: true,
                seq: core::ptr::read_volatile(p.add(8) as *const u16),
                fuel_len: core::ptr::read_volatile(p.add(10) as *const u16),
                ign_len: core::ptr::read_volatile(p.add(12) as *const u16),
                angles_len: core::ptr::read_volatile(p.add(18) as *const u16),
                fuel_crc: core::ptr::read_volatile(p.add(14) as *const u16),
                ign_crc: core::ptr::read_volatile(p.add(16) as *const u16),
                angles_crc: core::ptr::read_volatile(p.add(20) as *const u16),
            }
        }

        unsafe fn read_page(base: u32, off: usize, out: &mut [u8]) {
            let p = base as *const u8;
            for i in 0..out.len() {
                out[i] = core::ptr::read_volatile(p.add(off + i));
            }
        }

        // Choose newest valid sector by seq
        let a_hdr = unsafe { read_header(Self::LAYOUT.base_a) };
        let b_hdr = unsafe { read_header(Self::LAYOUT.base_b) };
        let active = newest_sector(a_hdr, b_hdr).ok_or(KvError::NotFound)?;
        let (base, hdr) = match active {
            ActiveSector::A => (Self::LAYOUT.base_a, a_hdr),
            ActiveSector::B => (Self::LAYOUT.base_b, b_hdr),
        };
        let key = FlashPageKey::from_key(key).ok_or(KvError::Io)?;
        let page = Self::LAYOUT.page(key);
        let actual_len = header_page_len(hdr, key);
        if actual_len as usize != page.len || out.len() < page.len {
            return Err(KvError::NotFound);
        }
        unsafe {
            read_page(base, page.offset, &mut out[..page.len]);
        }
        if !validate_page_image(Self::LAYOUT, hdr, key, &out[..page.len]) {
            return Err(KvError::Io);
        }
        Ok(page.len)
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        unsafe fn read_seq(base: u32) -> Option<u16> {
            let p = base as *const u8;
            if core::ptr::read_volatile(p as *const u32) != FlashLayout::MAGIC {
                return None;
            }
            if core::ptr::read_volatile(p.add(6) as *const u16) != FlashLayout::VALID_COMMITTED {
                return None;
            }
            Some(core::ptr::read_volatile(p.add(8) as *const u16))
        }

        let mut fuel = [0u8; FUEL_PAGE_LEN];
        let mut ign = [0u8; IGN_PAGE_LEN];
        let mut angles = [0u8; ANGLES_PAGE_LEN];
        let _ = self.read(b"fuel", &mut fuel); // ignore NotFound
        let _ = self.read(b"ign", &mut ign);
        let _ = self.read(b"angles", &mut angles);
        let (seq_a, seq_b) =
            unsafe { (read_seq(Self::LAYOUT.base_a), read_seq(Self::LAYOUT.base_b)) };
        let cur_seq = match (seq_a, seq_b) {
            (Some(a), Some(b)) => {
                if b.wrapping_sub(a) < 0x8000 {
                    b
                } else {
                    a
                }
            }
            (Some(a), None) => a,
            (None, Some(b)) => b,
            _ => 0,
        };
        match key {
            b"fuel" => {
                if data.len() != FUEL_PAGE_LEN {
                    return Err(KvError::Io);
                }
                fuel.copy_from_slice(data);
            }
            b"ign" => {
                if data.len() != IGN_PAGE_LEN {
                    return Err(KvError::Io);
                }
                ign.copy_from_slice(data);
            }
            b"angles" => {
                if data.len() != ANGLES_PAGE_LEN {
                    return Err(KvError::Io);
                }
                angles.copy_from_slice(data);
            }
            _ => return Err(KvError::Io),
        }

        // Prepare header (valid field stays 0xFFFF until the very end)
        let mut hdr = [0xFFu8; PAGE_HEADER_LEN];
        hdr[0] = 0x56;
        hdr[1] = 0x4B;
        hdr[2] = 0x50;
        hdr[3] = 0x32;
        // version
        hdr[4] = 2;
        hdr[5] = 0;
        // valid (u16) at [6..8] left as 0xFFFF until commit
        let new_seq = cur_seq.wrapping_add(1);
        hdr[8] = (new_seq & 0xFF) as u8;
        hdr[9] = (new_seq >> 8) as u8;
        hdr[10] = (FUEL_PAGE_LEN as u16 & 0xFF) as u8;
        hdr[11] = (FUEL_PAGE_LEN as u16 >> 8) as u8; // fuel 512
        hdr[12] = (IGN_PAGE_LEN as u16 & 0xFF) as u8;
        hdr[13] = (IGN_PAGE_LEN as u16 >> 8) as u8; // ign  512
        hdr[18] = (ANGLES_PAGE_LEN as u16 & 0xFF) as u8;
        hdr[19] = (ANGLES_PAGE_LEN as u16 >> 8) as u8;
        let fuel_crc = crc16(&fuel);
        let ign_crc = crc16(&ign);
        let angles_crc = crc16(&angles);
        hdr[14] = (fuel_crc & 0xFF) as u8;
        hdr[15] = (fuel_crc >> 8) as u8;
        hdr[16] = (ign_crc & 0xFF) as u8;
        hdr[17] = (ign_crc >> 8) as u8;
        hdr[20] = (angles_crc & 0xFF) as u8;
        hdr[21] = (angles_crc >> 8) as u8;

        free(|_| unsafe {
            let flash = &*pac::FLASH::ptr();
            let target = match (seq_a, seq_b) {
                (Some(a), Some(b)) => {
                    if b.wrapping_sub(a) < 0x8000 {
                        (Self::LAYOUT.sector_a, Self::LAYOUT.base_a)
                    } else {
                        (Self::LAYOUT.sector_b, Self::LAYOUT.base_b)
                    }
                }
                (Some(_), None) => (Self::LAYOUT.sector_b, Self::LAYOUT.base_b),
                (None, Some(_)) => (Self::LAYOUT.sector_a, Self::LAYOUT.base_a),
                (None, None) => (Self::LAYOUT.sector_a, Self::LAYOUT.base_a),
            };
            let (target_sector, target_base) = target;

            while flash.sr.read().bsy().bit_is_set() {}
            if flash.cr.read().lock().bit_is_set() {
                flash.keyr.write(|w| w.key().bits(0x4567_0123));
                flash.keyr.write(|w| w.key().bits(0xCDEF_89AB));
            }

            while flash.sr.read().bsy().bit_is_set() {}
            flash
                .cr
                .modify(|_, w| w.ser().set_bit().snb().bits(target_sector));
            flash.cr.modify(|_, w| w.strt().set_bit());
            while flash.sr.read().bsy().bit_is_set() {}
            flash.cr.modify(|_, w| w.ser().clear_bit());

            flash.cr.modify(|_, w| w.psize().bits(0b01).pg().set_bit());
            let prog_half = |addr: u32, val: u16| {
                core::ptr::write_volatile(addr as *mut u16, val);
                while flash.sr.read().bsy().bit_is_set() {}
            };

            let mut a = target_base;
            for i in (0..Self::LAYOUT.header_len).step_by(2) {
                let v = (hdr[i] as u16) | ((hdr[i + 1] as u16) << 8);
                prog_half(a, v);
                a += 2;
            }
            let mut a = target_base + (Self::LAYOUT.fuel.offset as u32);
            for i in (0..Self::LAYOUT.fuel.len).step_by(2) {
                let v = (fuel[i] as u16) | ((fuel[i + 1] as u16) << 8);
                prog_half(a, v);
                a += 2;
            }
            let mut a = target_base + (Self::LAYOUT.ign.offset as u32);
            for i in (0..Self::LAYOUT.ign.len).step_by(2) {
                let v = (ign[i] as u16) | ((ign[i + 1] as u16) << 8);
                prog_half(a, v);
                a += 2;
            }
            let mut a = target_base + (Self::LAYOUT.angles.offset as u32);
            for i in (0..Self::LAYOUT.angles.len).step_by(2) {
                let v = (angles[i] as u16) | ((angles[i + 1] as u16) << 8);
                prog_half(a, v);
                a += 2;
            }

            let valid_addr = target_base + 6;
            prog_half(valid_addr, 0x0000);

            flash.cr.modify(|_, w| w.pg().clear_bit());
            flash.cr.modify(|_, w| w.lock().set_bit());
        });

        Ok(())
    }
}

#[cfg(feature = "flash-kv")]
type StoreKv = FlashKv;
#[cfg(not(feature = "flash-kv"))]
type StoreKv = RamKv512;

pub(crate) struct BoardTsPageStoreProvider {
    state: StatePtr<BoardTsState>,
}

impl BoardTsPageStoreProvider {
    pub(crate) fn new(state: &mut BoardTsState) -> Self {
        Self {
            state: StatePtr::new(state),
        }
    }
}

impl PageStoreProvider for BoardTsPageStoreProvider {
    type Pages<'a> = EcuPageStore<'a>;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let mut pages = state.page_store();
                f(&mut pages)
            })
        }
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let pages = state.page_store();
                f(&pages)
            })
        }
    }

    fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with(|state| {
                FuelRuntimeTune::new(
                    state.config.ve_table,
                    state.config.afr_table,
                    state.config.required_fuel_us,
                    state.config.injector_deadtime_us,
                    state.config.ve_load_source,
                )
            })
        }
    }
}

pub(crate) struct Provider {
    runtime: StateRef<ecu_runtime::EngineRuntime>,
    runtime_adapter: RuntimeSnapshotAdapter,
}

impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        let snapshot = unsafe { self.runtime.with(|runtime| runtime.snapshot()) };
        self.runtime_adapter.fill_outpc(&snapshot, out);
        free(|cs| {
            let sens = TS_SENSORS.borrow(cs).borrow();
            out.tps_percent = sens.tps_percent;
            out.clt_c = sens.clt_c;
            out.iat_c = sens.iat_c;
            out.vbatt_mv = sens.vbatt_mv;
            out.lambda_x100 = 100;
        });
    }

    fn engine_running(&self) -> bool {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        let snapshot = unsafe { self.runtime.with(|runtime| runtime.snapshot()) };
        matches!(snapshot.engine.sync, ecu_domain::SyncState::Locked { .. })
            && snapshot.engine.rpm.get() > 0
    }
}

type Service = TsService<Provider, PersistedTsPageStore<BoardTsPageStoreProvider, StoreKv>>;

pub(crate) fn new_service(
    state: &mut BoardTsState,
    runtime: &ecu_runtime::EngineRuntime,
) -> Service {
    let provider = Provider {
        runtime: StateRef::new(runtime),
        runtime_adapter: RuntimeSnapshotAdapter::new(),
    };
    let mut store = PersistedTsPageStore::new(BoardTsPageStoreProvider::new(state), StoreKv::new());
    store.try_load();
    TsService::new(ecu_ts::TS_SIGNATURE, provider, store)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_ts::persistence::PageStoreProvider;
    use ecu_ts::server::PageStore;

    const PAGE_FUEL: u8 = 1;

    fn committed_header(seq: u16, fuel: &[u8], ign: &[u8], angles: &[u8]) -> KvHeader {
        KvHeader {
            ok: true,
            seq,
            fuel_len: fuel.len() as u16,
            ign_len: ign.len() as u16,
            angles_len: angles.len() as u16,
            fuel_crc: crc16(fuel),
            ign_crc: crc16(ign),
            angles_crc: crc16(angles),
        }
    }

    #[test]
    fn flash_layout_places_pages_inside_reserved_sector() {
        let layout = FlashLayout::STM32F405_TS_KV.unwrap();

        assert_eq!(layout.base_a, 0x080C_0000);
        assert_eq!(layout.base_b, 0x080E_0000);
        assert_eq!(layout.fuel.offset, PAGE_HEADER_LEN);
        assert_eq!(layout.ign.offset, PAGE_HEADER_LEN + FUEL_PAGE_LEN);
        assert_eq!(
            layout.angles.offset,
            PAGE_HEADER_LEN + FUEL_PAGE_LEN + IGN_PAGE_LEN
        );
        assert!(layout.angles.offset + layout.angles.len <= FlashLayout::SECTOR_BYTES);
    }

    #[test]
    fn flash_layout_rejects_invalid_bounds_and_aliases() {
        assert_eq!(
            FlashLayout::new(0x080C_0000, 0x080E_0000, 10, 11, 8, 1, 1, 1),
            Err(FlashLayoutError::HeaderTooSmall)
        );
        assert_eq!(
            FlashLayout::new(0x080C_0000, 0x080C_0000, 10, 11, PAGE_HEADER_LEN, 1, 1, 1),
            Err(FlashLayoutError::SectorAlias)
        );
        assert_eq!(
            FlashLayout::new(
                0x080C_0000,
                0x080E_0000,
                10,
                11,
                PAGE_HEADER_LEN,
                FlashLayout::SECTOR_BYTES,
                1,
                1
            ),
            Err(FlashLayoutError::PageOverflow)
        );
    }

    #[test]
    fn flash_header_page_validation_rejects_crc_and_length_mismatch() {
        let layout = FlashLayout::STM32F405_TS_KV.unwrap();
        let fuel = [0xA5; FUEL_PAGE_LEN];
        let ign = [0x5A; IGN_PAGE_LEN];
        let angles = [0x11; ANGLES_PAGE_LEN];
        let hdr = committed_header(7, &fuel, &ign, &angles);

        assert!(validate_page_image(layout, hdr, FlashPageKey::Fuel, &fuel));

        let mut corrupt = fuel;
        corrupt[0] ^= 0x01;
        assert!(!validate_page_image(
            layout,
            hdr,
            FlashPageKey::Fuel,
            &corrupt
        ));

        let short = &fuel[..FUEL_PAGE_LEN - 1];
        assert!(!validate_page_image(layout, hdr, FlashPageKey::Fuel, short));
    }

    #[test]
    fn flash_sequence_selection_handles_wrap_and_invalid_sector() {
        let empty = [0u8; FUEL_PAGE_LEN];
        let hdr_a = committed_header(10, &empty, &[0u8; IGN_PAGE_LEN], &[0u8; ANGLES_PAGE_LEN]);
        let hdr_b = committed_header(12, &empty, &[0u8; IGN_PAGE_LEN], &[0u8; ANGLES_PAGE_LEN]);
        assert_eq!(newest_sector(hdr_a, hdr_b), Some(ActiveSector::B));
        assert_eq!(target_sector_for_next_write(hdr_a, hdr_b), ActiveSector::A);

        let wrapped_old = committed_header(
            0xFFFE,
            &empty,
            &[0u8; IGN_PAGE_LEN],
            &[0u8; ANGLES_PAGE_LEN],
        );
        let wrapped_new =
            committed_header(1, &empty, &[0u8; IGN_PAGE_LEN], &[0u8; ANGLES_PAGE_LEN]);
        assert_eq!(
            newest_sector(wrapped_old, wrapped_new),
            Some(ActiveSector::B)
        );

        let invalid = KvHeader {
            ok: false,
            seq: 0,
            fuel_len: 0,
            ign_len: 0,
            angles_len: 0,
            fuel_crc: 0,
            ign_crc: 0,
            angles_crc: 0,
        };
        assert_eq!(newest_sector(invalid, invalid), None);
        assert_eq!(
            target_sector_for_next_write(invalid, invalid),
            ActiveSector::A
        );
    }

    #[test]
    fn page_store_provider_reuses_single_state_pointer_for_read_and_write_views() {
        let mut state = BoardTsState::new();
        let mut provider = BoardTsPageStoreProvider::new(&mut state);
        let mut page = [0u8; FUEL_PAGE_LEN];
        let mut out = [0u8; FUEL_PAGE_LEN];

        provider.with_pages(|pages| {
            assert_eq!(pages.read_page(PAGE_FUEL, &mut page), Some(FUEL_PAGE_LEN));
        });
        page[0] ^= 0x5A;

        provider.with_pages_mut(|pages| {
            pages
                .write_page(PAGE_FUEL, &page)
                .expect("fuel page write succeeds");
        });

        provider.with_pages(|pages| {
            assert_eq!(pages.read_page(PAGE_FUEL, &mut out), Some(FUEL_PAGE_LEN));
            assert_eq!(out, page);
        });
    }
}
