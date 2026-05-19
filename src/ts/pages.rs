//! Page stores for fuel/ignition tables
use super::server::{PageError, PageStore, PersistError};
use crate::constants::{fuel as fuel_consts, ignition as ign_consts};
use crate::dfco::DfcoConfig;
use crate::diag;
use crate::enrichment::AeConfig;
use crate::persist::{KvError, KvStore, PERSIST_KEY_FUEL, PERSIST_KEY_IGN};
use crate::sensors::{SensorsCal, SensorsLimits};
use crate::telemetry::IsrStats;
use crate::trigger::SyncState;
use crate::units::{Micros, Rpm};

/// Page numbers
pub const PAGE_FUEL: u8 = 1;
pub const PAGE_IGN: u8 = 2;
pub const PAGE_SENSORS: u8 = 3;
pub const PAGE_AE: u8 = 4;
pub const PAGE_DFCO: u8 = 5;
pub const PAGE_LIMITS: u8 = 6; // New page for sensor limits + emergency triggers
pub const PAGE_DIAG: u8 = 7; // Read-only diagnostics summary
pub const PAGE_DIAG_LOG: u8 = 8; // Read-only recent diagnostics events
pub const PAGE_ANGLES: u8 = 9; // Per-cylinder angle + cam timeout config
pub const PAGE_WUE: u8 = 10; // Warmup Enrichment config
pub const PAGE_ASE: u8 = 11; // AfterStart Enrichment config
pub const PAGE_IDLE: u8 = 12; // Idle control (open-loop)
pub const PAGE_FAN: u8 = 13; // Fan control (on/off with hysteresis)
pub const PAGE_CL: u8 = 14; // Closed-loop fuel (target AFR and gains)
pub const PAGE_SNAPSHOT: u8 = 15; // Unified runtime snapshot
pub const PAGE_EXPERT_TRIGGER: u8 = 16; // Expert trigger setup

const TS_PAGE_BYTES: usize = 512;
const TS_SENSORS_BYTES: usize = 128;
pub const TS_DIAG_BYTES: usize = 32;
pub const TS_EXPERT_TRIGGER_BYTES: usize = 48;
const TS_PAGE_COUNT: usize = 16;

/// Descriptor for a TunerStudio page.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct TsPageDescriptor {
    pub page: u8,
    pub len: usize,
    pub writable: bool,
    pub label: &'static str,
}

pub const TS_PAGE_DESCRIPTORS: [TsPageDescriptor; TS_PAGE_COUNT] = [
    TsPageDescriptor {
        page: PAGE_FUEL,
        len: TS_PAGE_BYTES,
        writable: true,
        label: "fuel",
    },
    TsPageDescriptor {
        page: PAGE_IGN,
        len: TS_PAGE_BYTES,
        writable: true,
        label: "ign",
    },
    TsPageDescriptor {
        page: PAGE_SENSORS,
        len: TS_SENSORS_BYTES,
        writable: true,
        label: "sensors",
    },
    TsPageDescriptor {
        page: PAGE_AE,
        len: 16,
        writable: true,
        label: "ae",
    },
    TsPageDescriptor {
        page: PAGE_DFCO,
        len: 16,
        writable: true,
        label: "dfco",
    },
    TsPageDescriptor {
        page: PAGE_LIMITS,
        len: 10,
        writable: true,
        label: "limits",
    },
    TsPageDescriptor {
        page: PAGE_DIAG,
        len: TS_DIAG_BYTES,
        writable: false,
        label: "diag",
    },
    TsPageDescriptor {
        page: PAGE_DIAG_LOG,
        len: 16 * 9,
        writable: false,
        label: "diag_log",
    },
    TsPageDescriptor {
        page: PAGE_ANGLES,
        len: 68,
        writable: true,
        label: "angles",
    },
    TsPageDescriptor {
        page: PAGE_WUE,
        len: 8,
        writable: true,
        label: "wue",
    },
    TsPageDescriptor {
        page: PAGE_ASE,
        len: 8,
        writable: true,
        label: "ase",
    },
    TsPageDescriptor {
        page: PAGE_IDLE,
        len: 6,
        writable: true,
        label: "idle",
    },
    TsPageDescriptor {
        page: PAGE_FAN,
        len: 6,
        writable: true,
        label: "fan",
    },
    TsPageDescriptor {
        page: PAGE_CL,
        len: 8,
        writable: true,
        label: "cl",
    },
    TsPageDescriptor {
        page: PAGE_SNAPSHOT,
        len: 32,
        writable: false,
        label: "snapshot",
    },
    TsPageDescriptor {
        page: PAGE_EXPERT_TRIGGER,
        len: TS_EXPERT_TRIGGER_BYTES,
        writable: true,
        label: "expert_trigger",
    },
];

pub fn ts_page_descriptor(page: u8) -> Option<TsPageDescriptor> {
    TS_PAGE_DESCRIPTORS
        .iter()
        .copied()
        .find(|descriptor| descriptor.page == page)
}

const EXPERT_SCHEMA_VERSION_CURRENT: u16 = 1;
const EXPERT_UNLOCK_LOCKED: u8 = 0;
const EXPERT_UNLOCK_UNLOCKED: u8 = 1;
const TRIGGER_AUTHORITY_EXPERT_MANUAL: u8 = 1;
const TRIGGER_AUTHORITY_CERTIFIED_PROFILE: u8 = 3;
const TRIGGER_PATTERN_MISSING_TOOTH: u8 = 0;
const SECONDARY_TRIGGER_NONE: u8 = 0;
const EXPERT_IGNITION_SEQUENTIAL_COP: u8 = 3;
const EXPERT_INJECTION_SEQUENTIAL: u8 = 4;
const FIXED_TIMING_FIXED: u8 = 1;

const fn default_expert_trigger_record() -> [u8; TS_EXPERT_TRIGGER_BYTES] {
    let mut record = [0u8; TS_EXPERT_TRIGGER_BYTES];
    record[0] = EXPERT_SCHEMA_VERSION_CURRENT as u8;
    record[13] = 60;
    record[14] = 2;
    record[18] = 1;
    record[23] = 2;
    record[25] = 2;
    record[26] = 1;
    record[27] = 1;
    record[30] = 100;
    record
}

const DEFAULT_EXPERT_TRIGGER_RECORD: [u8; TS_EXPERT_TRIGGER_BYTES] =
    default_expert_trigger_record();

/// Root-owned view of the canonical `ecu_calibration::ExpertTriggerCalibration`
/// TS record layout.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExpertTriggerPageState {
    record: [u8; TS_EXPERT_TRIGGER_BYTES],
}

impl ExpertTriggerPageState {
    pub const fn new() -> Self {
        Self {
            record: DEFAULT_EXPERT_TRIGGER_RECORD,
        }
    }

    fn read(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < TS_EXPERT_TRIGGER_BYTES {
            return None;
        }
        out[..TS_EXPERT_TRIGGER_BYTES].copy_from_slice(&self.record);
        Some(TS_EXPERT_TRIGGER_BYTES)
    }

    fn write(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() != TS_EXPERT_TRIGGER_BYTES {
            return Err(PageError::WrongSize);
        }
        let proposed = canonical_expert_trigger_record(data);
        validate_expert_trigger_record(&proposed, &self.record)?;
        self.record = proposed;
        Ok(())
    }

    fn authority_code(&self) -> u8 {
        self.record[3]
    }

    fn profile_identity(&self) -> u32 {
        u32::from_le_bytes([
            self.record[4],
            self.record[5],
            self.record[6],
            self.record[7],
        ])
    }

    fn profile_hash(&self) -> u32 {
        u32::from_le_bytes([
            self.record[8],
            self.record[9],
            self.record[10],
            self.record[11],
        ])
    }
}

impl Default for ExpertTriggerPageState {
    fn default() -> Self {
        Self::new()
    }
}

fn canonical_expert_trigger_record(data: &[u8]) -> [u8; TS_EXPERT_TRIGGER_BYTES] {
    let mut record = [0u8; TS_EXPERT_TRIGGER_BYTES];
    record[..29].copy_from_slice(&data[..29]);
    record[24] = u8::from(data[24] != 0);
    record[30..32].copy_from_slice(&data[30..32]);
    record
}

fn validate_expert_trigger_record(
    proposed: &[u8; TS_EXPERT_TRIGGER_BYTES],
    current: &[u8; TS_EXPERT_TRIGGER_BYTES],
) -> Result<(), PageError> {
    if u16::from_le_bytes([proposed[0], proposed[1]]) != EXPERT_SCHEMA_VERSION_CURRENT {
        return Err(PageError::Invalid);
    }
    if proposed[3] == TRIGGER_AUTHORITY_CERTIFIED_PROFILE {
        return Err(PageError::Invalid);
    }
    if proposed[2] > EXPERT_UNLOCK_UNLOCKED
        || proposed[3] > 4
        || proposed[12] > 3
        || proposed[15] > 1
        || proposed[19] > 1
        || proposed[20] > 1
        || proposed[21] > 5
        || proposed[22] > 1
        || proposed[23] > 3
        || proposed[26] > 3
        || proposed[27] > 4
        || proposed[28] > 1
    {
        return Err(PageError::Invalid);
    }

    if proposed[3] == TRIGGER_AUTHORITY_EXPERT_MANUAL && proposed[2] != EXPERT_UNLOCK_UNLOCKED {
        return Err(PageError::Invalid);
    }
    if proposed[3] == TRIGGER_AUTHORITY_CERTIFIED_PROFILE {
        let profile_identity =
            u32::from_le_bytes([proposed[4], proposed[5], proposed[6], proposed[7]]);
        let profile_hash =
            u32::from_le_bytes([proposed[8], proposed[9], proposed[10], proposed[11]]);
        if profile_identity == 0 || profile_hash == 0 {
            return Err(PageError::Invalid);
        }
        if proposed[2] != EXPERT_UNLOCK_LOCKED {
            return Err(PageError::Invalid);
        }
    }

    let primary_base_teeth = proposed[13];
    let missing_teeth = proposed[14];
    if primary_base_teeth == 0 {
        return Err(PageError::Invalid);
    }
    if proposed[12] == TRIGGER_PATTERN_MISSING_TOOTH {
        if primary_base_teeth < 2 || missing_teeth == 0 || missing_teeth >= primary_base_teeth {
            return Err(PageError::Invalid);
        }
    } else if missing_teeth != 0 {
        return Err(PageError::Invalid);
    }

    let trigger_angle = u16::from_le_bytes([proposed[16], proposed[17]]);
    if trigger_angle > 7200 {
        return Err(PageError::Invalid);
    }
    if proposed[18] == 0 || proposed[18] > 8 {
        return Err(PageError::Invalid);
    }
    if proposed[24] != 0 && proposed[21] == SECONDARY_TRIGGER_NONE {
        return Err(PageError::Invalid);
    }
    if (proposed[26] == EXPERT_IGNITION_SEQUENTIAL_COP
        || proposed[27] == EXPERT_INJECTION_SEQUENTIAL)
        && proposed[21] == SECONDARY_TRIGGER_NONE
    {
        return Err(PageError::Invalid);
    }
    if proposed[25] > 16 {
        return Err(PageError::Invalid);
    }
    let fixed_timing = i16::from_le_bytes([proposed[30], proposed[31]]);
    if proposed[28] == FIXED_TIMING_FIXED && !(-100..=600).contains(&fixed_timing) {
        return Err(PageError::Invalid);
    }

    if current[3] == TRIGGER_AUTHORITY_CERTIFIED_PROFILE {
        if proposed[3] == TRIGGER_AUTHORITY_CERTIFIED_PROFILE && proposed != current {
            return Err(PageError::Invalid);
        }
        if proposed[3] != TRIGGER_AUTHORITY_CERTIFIED_PROFILE
            && proposed[2] != EXPERT_UNLOCK_UNLOCKED
        {
            return Err(PageError::Invalid);
        }
    }

    Ok(())
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SystemSnapshot {
    pub rpm: Rpm,
    pub sync: SyncState,
    pub base_pw: Micros,
    pub enrich_mult_x100: u16,
    pub stft_x10: i16,
    pub fuel_mult_x100: u16,
    pub final_pw: Micros,
    pub last_fault: Option<diag::DiagCode>,
    pub isr_stats: IsrStats,
}

/// Exposes references to fuel and ignition tables as TS pages
pub struct EcuStatePageStore<'a> {
    pub fuel: &'a mut [[u16; 16]; 16],
    pub ign: &'a mut [[i16; 16]; 16],
}

impl<'a> EcuStatePageStore<'a> {
    fn read_fuel(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < TS_PAGE_BYTES {
            return None;
        }
        let mut idx = 0;
        for row in 0..16 {
            for col in 0..16 {
                let v = self.fuel[row][col].to_le_bytes();
                out[idx] = v[0];
                out[idx + 1] = v[1];
                idx += 2;
            }
        }
        Some(TS_PAGE_BYTES)
    }
    fn write_fuel(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() != TS_PAGE_BYTES {
            return Err(PageError::WrongSize);
        }
        let mut idx = 0;
        for row in 0..16 {
            for col in 0..16 {
                let lo = data[idx] as u16;
                let hi = data[idx + 1] as u16;
                self.fuel[row][col] = lo | (hi << 8);
                idx += 2;
            }
        }
        Ok(())
    }
    fn read_ign(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < TS_PAGE_BYTES {
            return None;
        }
        let mut idx = 0;
        for row in 0..16 {
            for col in 0..16 {
                let v = self.ign[row][col].to_le_bytes();
                out[idx] = v[0];
                out[idx + 1] = v[1];
                idx += 2;
            }
        }
        Some(TS_PAGE_BYTES)
    }
    fn write_ign(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() != TS_PAGE_BYTES {
            return Err(PageError::WrongSize);
        }
        let mut idx = 0;
        for row in 0..16 {
            for col in 0..16 {
                let lo = data[idx] as u16;
                let hi = data[idx + 1] as u16;
                self.ign[row][col] = i16::from_le_bytes([lo as u8, hi as u8]);
                idx += 2;
            }
        }
        Ok(())
    }
}

impl<'a> PageStore for EcuStatePageStore<'a> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_FUEL | PAGE_IGN => Some(TS_PAGE_BYTES),
            _ => None,
        }
    }
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_FUEL => self.read_fuel(out),
            PAGE_IGN => self.read_ign(out),
            _ => None,
        }
    }
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_FUEL => self.write_fuel(data),
            PAGE_IGN => self.write_ign(data),
            _ => Err(PageError::Invalid),
        }
    }
    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}

/// Wrapper that adds persistence to an EcuState-backed PageStore using a KvStore
pub struct PersistedPageStore<'a, KV: KvStore> {
    inner: EcuStatePageStore<'a>,
    /// pub(crate) for test access via PersistedPageStore::into_kv().
    pub(crate) kv: KV,
}

impl<'a, KV: KvStore> PersistedPageStore<'a, KV> {
    pub const fn new(inner: EcuStatePageStore<'a>, kv: KV) -> Self {
        Self { inner, kv }
    }

    /// Attempt to load persisted pages into the in-memory tables.
    /// If keys are absent or invalid, leaves current values untouched.
    pub fn try_load(&mut self) {
        // Fuel
        let mut buf = [0u8; TS_PAGE_BYTES];
        if let Ok(n) = self.kv.read(PERSIST_KEY_FUEL, &mut buf) {
            if n == TS_PAGE_BYTES {
                let _ = self.inner.write_fuel(&buf);
            }
        }
        // Ignition
        let mut ibuf = [0u8; TS_PAGE_BYTES];
        if let Ok(n) = self.kv.read(PERSIST_KEY_IGN, &mut ibuf) {
            if n == TS_PAGE_BYTES {
                let _ = self.inner.write_ign(&ibuf);
            }
        }
    }

    /// Reset in-memory tables to safe defaults (does not modify persistent storage until `burn`).
    pub fn factory_reset(&mut self) {
        // Fuel: fill all cells with DEFAULT_PULSE_WIDTH_US
        for row in 0..16 {
            for col in 0..16 {
                self.inner.fuel[row][col] = fuel_consts::DEFAULT_PULSE_WIDTH_US;
            }
        }
        // Ignition: conservative default timing
        for row in 0..16 {
            for col in 0..16 {
                self.inner.ign[row][col] = ign_consts::DEFAULT_TIMING_BTDC;
            }
        }
    }
}

impl<'a, KV: KvStore> PageStore for PersistedPageStore<'a, KV> {
    fn page_len(&self, page: u8) -> Option<usize> {
        self.inner.page_len(page)
    }
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        self.inner.read_page(page, out)
    }
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        self.inner.write_page(page, data)
    }
    fn burn(&mut self) -> Result<(), PersistError> {
        // Serialize both pages and store via KV
        let mut buf = [0u8; TS_PAGE_BYTES];
        if self.inner.read_fuel(&mut buf).unwrap_or(0) == TS_PAGE_BYTES {
            self.kv
                .write(PERSIST_KEY_FUEL, &buf)
                .map_err(|err| match err {
                    KvError::EngineRunning => PersistError::EngineRunning,
                    _ => PersistError::Fail,
                })?;
        }
        let mut ibuf = [0u8; TS_PAGE_BYTES];
        if self.inner.read_ign(&mut ibuf).unwrap_or(0) == TS_PAGE_BYTES {
            self.kv
                .write(PERSIST_KEY_IGN, &ibuf)
                .map_err(|err| match err {
                    KvError::EngineRunning => PersistError::EngineRunning,
                    _ => PersistError::Fail,
                })?;
        }
        Ok(())
    }
}

/// Composite page store including fuel, ignition, and sensors calibration
pub struct EcuPageStore<'a> {
    pub fuel: &'a mut [[u16; 16]; 16],
    pub ign: &'a mut [[i16; 16]; 16],
    pub sens: &'a mut SensorsCal,
    pub ae: &'a mut AeConfig,
    pub dfco: &'a mut DfcoConfig,
    pub wue: &'a mut crate::enrichment::WueConfig,
    pub ase: &'a mut crate::enrichment::AseConfig,
    pub idle: &'a mut crate::actuators::IdleConfig,
    pub fan: &'a mut crate::actuators::FanConfig,
    pub cl: &'a mut crate::actuators::ClConfig,
    pub limits: &'a mut SensorsLimits,
    pub emerg_trig_map: &'a mut bool,
    pub emerg_trig_tps: &'a mut bool,
    pub diag_emergency: &'a bool,
    pub diag_map: &'a diag::DiagState,
    pub diag_tps: &'a diag::DiagState,
    pub diag_cam: &'a diag::DiagState,
    pub diag_log: &'a diag::DiagLog<16>,
    pub isr_stats: &'a IsrStats,
    pub snapshot: &'a SystemSnapshot,
    pub tooth_count: &'a u8,
    pub sync_loss_tracker: &'a crate::safety::SyncLossTracker,
    pub angles_inj: &'a mut [u16; 16],
    pub angles_tdc: &'a mut [u16; 16],
    pub tooth0_angle_x10: &'a mut u16,
    pub cam_timeout_ms: &'a mut u16,
    pub expert_trigger: &'a mut ExpertTriggerPageState,
}

impl<'a> EcuPageStore<'a> {
    fn read_sensors(&self, out: &mut [u8]) -> Option<usize> {
        // Layout (LE):
        // 0: tps_min (u16), 2: tps_max (u16)
        // 4: map_v0_mv (u16), 6: map_kpa0_x10 (u16)
        // 8: map_v1_mv (u16), 10: map_kpa1_x10 (u16)
        // 12..28: clt_degC [8] (i16)
        // 28..44: iat_degC [8] (i16)
        // 44..(44+32): clt_ohms [8] (u32)
        // 76..(76+32): iat_ohms [8] (u32)
        if out.len() < TS_SENSORS_BYTES {
            return None;
        }
        let mut idx = 0;
        let w16 = |buf: &mut [u8], i: &mut usize, v: u16| {
            buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
            *i += 2;
        };
        let w16s = |buf: &mut [u8], i: &mut usize, v: i16| {
            buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
            *i += 2;
        };
        let w32 = |buf: &mut [u8], i: &mut usize, v: u32| {
            buf[*i..*i + 4].copy_from_slice(&v.to_le_bytes());
            *i += 4;
        };
        w16(out, &mut idx, self.sens.tps_min_counts);
        w16(out, &mut idx, self.sens.tps_max_counts);
        w16(out, &mut idx, self.sens.map_v0_mv);
        w16(out, &mut idx, self.sens.map_kpa0_x10);
        w16(out, &mut idx, self.sens.map_v1_mv);
        w16(out, &mut idx, self.sens.map_kpa1_x10);
        for t in self.sens.clt_deg_c.iter() {
            w16s(out, &mut idx, *t);
        }
        for t in self.sens.iat_deg_c.iter() {
            w16s(out, &mut idx, *t);
        }
        for r in self.sens.clt_ohms.iter() {
            w32(out, &mut idx, *r);
        }
        for r in self.sens.iat_ohms.iter() {
            w32(out, &mut idx, *r);
        }
        Some(TS_SENSORS_BYTES)
    }

    fn write_sensors(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() < TS_SENSORS_BYTES {
            return Err(PageError::WrongSize);
        }
        let mut idx = 0;
        let r16 = |d: &[u8], i: &mut usize| -> u16 {
            let v = u16::from_le_bytes([d[*i], d[*i + 1]]);
            *i += 2;
            v
        };
        let r16s = |d: &[u8], i: &mut usize| -> i16 {
            let v = i16::from_le_bytes([d[*i], d[*i + 1]]);
            *i += 2;
            v
        };
        let r32 = |d: &[u8], i: &mut usize| -> u32 {
            let v = u32::from_le_bytes([d[*i], d[*i + 1], d[*i + 2], d[*i + 3]]);
            *i += 4;
            v
        };
        let tps_min = r16(data, &mut idx);
        let tps_max = r16(data, &mut idx);
        if tps_min >= tps_max {
            return Err(PageError::Invalid);
        }
        self.sens.tps_min_counts = tps_min;
        self.sens.tps_max_counts = tps_max;
        let v0 = r16(data, &mut idx);
        let k0 = r16(data, &mut idx);
        let v1 = r16(data, &mut idx);
        let k1 = r16(data, &mut idx);
        if v1 <= v0 || k1 <= k0 {
            return Err(PageError::Invalid);
        }
        self.sens.map_v0_mv = v0;
        self.sens.map_kpa0_x10 = k0;
        self.sens.map_v1_mv = v1;
        self.sens.map_kpa1_x10 = k1;
        for t in self.sens.clt_deg_c.iter_mut() {
            *t = r16s(data, &mut idx);
        }
        for t in self.sens.iat_deg_c.iter_mut() {
            *t = r16s(data, &mut idx);
        }
        for r in self.sens.clt_ohms.iter_mut() {
            *r = r32(data, &mut idx);
        }
        for r in self.sens.iat_ohms.iter_mut() {
            *r = r32(data, &mut idx);
        }
        Ok(())
    }
}

impl<'a> PageStore for EcuPageStore<'a> {
    fn page_len(&self, page: u8) -> Option<usize> {
        ts_page_descriptor(page).map(|descriptor| descriptor.len)
    }
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_FUEL => {
                if out.len() < TS_PAGE_BYTES {
                    return None;
                }
                let mut idx = 0;
                for row in 0..16 {
                    for col in 0..16 {
                        let v = self.fuel[row][col].to_le_bytes();
                        out[idx] = v[0];
                        out[idx + 1] = v[1];
                        idx += 2;
                    }
                }
                Some(TS_PAGE_BYTES)
            }
            PAGE_IGN => {
                if out.len() < TS_PAGE_BYTES {
                    return None;
                }
                let mut idx = 0;
                for row in 0..16 {
                    for col in 0..16 {
                        let v = self.ign[row][col].to_le_bytes();
                        out[idx] = v[0];
                        out[idx + 1] = v[1];
                        idx += 2;
                    }
                }
                Some(TS_PAGE_BYTES)
            }
            PAGE_SENSORS => self.read_sensors(out),
            PAGE_LIMITS => {
                if out.len() < 10 {
                    return None;
                }
                let mut idx = 0;
                let w16 = |buf: &mut [u8], i: &mut usize, v: u16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                w16(out, &mut idx, self.limits.map_min_kpa_x10);
                w16(out, &mut idx, self.limits.map_max_kpa_x10);
                out[idx] = self.limits.tps_min_percent;
                idx += 1;
                out[idx] = self.limits.tps_max_percent;
                idx += 1;
                w16(out, &mut idx, self.limits.clear_time_s);
                let mut trig: u8 = 0;
                if *self.emerg_trig_map {
                    trig |= 1;
                }
                if *self.emerg_trig_tps {
                    trig |= 2;
                }
                out[idx] = trig;
                idx += 1;
                out[idx] = 0; /* reserved */
                Some(10)
            }
            PAGE_DIAG => {
                if out.len() < TS_DIAG_BYTES {
                    return None;
                }
                let mut idx = 0usize;
                out[idx] = *self.tooth_count;
                idx += 1;
                out[idx] = u8::from(matches!(
                    self.snapshot.sync,
                    SyncState::Locked { cam_ref: true }
                ));
                idx += 1;
                out[idx] = match self.snapshot.sync {
                    SyncState::Unsynced => 0,
                    SyncState::Provisional => 1,
                    SyncState::Locked { .. } => 2,
                };
                idx += 1;
                out[idx] = match self.snapshot.sync {
                    SyncState::Unsynced => 0,
                    SyncState::Provisional => 1,
                    SyncState::Locked { cam_ref: false } => 2,
                    SyncState::Locked { cam_ref: true } => 3,
                };
                idx += 1;
                out[idx] = self.expert_trigger.authority_code();
                idx += 1;
                out[idx] = 0; // trigger angle source placeholder
                idx += 1;
                out[idx] = 0; // output gating reason placeholder
                idx += 1;
                out[idx] = 0; // last sync-loss reason placeholder
                idx += 1;
                let w16 = |buf: &mut [u8], i: &mut usize, v: u16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                let w32 = |buf: &mut [u8], i: &mut usize, v: u32| {
                    buf[*i..*i + 4].copy_from_slice(&v.to_le_bytes());
                    *i += 4;
                };
                w16(out, &mut idx, self.snapshot.rpm.raw());
                w16(out, &mut idx, 0); // detected gap ratio placeholder
                w16(out, &mut idx, self.sync_loss_tracker.total_losses);
                w16(out, &mut idx, 0); // M50 pin-map identity placeholder
                w32(out, &mut idx, self.expert_trigger.profile_identity());
                w32(out, &mut idx, self.expert_trigger.profile_hash());
                out[idx..TS_DIAG_BYTES].fill(0);
                Some(TS_DIAG_BYTES)
            }
            PAGE_AE => {
                // See layout in write/read helper below
                if out.len() < 16 {
                    return None;
                }
                let mut idx = 0;
                let w16s = |buf: &mut [u8], i: &mut usize, v: i16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                let w32 = |buf: &mut [u8], i: &mut usize, v: u32| {
                    buf[*i..*i + 4].copy_from_slice(&v.to_le_bytes());
                    *i += 4;
                };
                w16s(out, &mut idx, self.ae.tpsdot_thresh_pct_s);
                w16s(out, &mut idx, self.ae.mapdot_thresh_kpa_s);
                out[idx] = self.ae.percent_gain;
                idx += 1;
                out[idx] = 0;
                idx += 1;
                w32(out, &mut idx, self.ae.decay_time_ms);
                w32(out, &mut idx, self.ae.lockout_ms);
                Some(16)
            }
            PAGE_DIAG_LOG => {
                // Layout per entry (9 bytes):
                // 0 code (u8): 1=MAP,2=TPS,3=CAM
                // 1..4 start_us (u32 LE)
                // 5..8 end_us (u32 LE)
                if out.len() < 16 * 9 {
                    return None;
                }
                let mut idx = 0usize;
                for slot in self.diag_log.events.iter() {
                    let (code, start, end) = match slot {
                        Some(ev) => {
                            let c = match ev.code {
                                diag::DiagCode::MapRange => 1u8,
                                diag::DiagCode::TpsRange => 2u8,
                                diag::DiagCode::CamMissing => 3u8,
                                diag::DiagCode::LowVoltage => 4u8,
                                diag::DiagCode::Overvoltage => 5u8,
                                diag::DiagCode::MapFailureHighLoad => 6u8,
                                diag::DiagCode::TpsMapPlausibility => 7u8,
                                diag::DiagCode::KnockDetected => 8u8,
                            };
                            (c, ev.start_us, ev.end_us)
                        }
                        None => (0u8, 0u32, 0u32),
                    };
                    out[idx] = code;
                    idx += 1;
                    out[idx..idx + 4].copy_from_slice(&start.to_le_bytes());
                    idx += 4;
                    out[idx..idx + 4].copy_from_slice(&end.to_le_bytes());
                    idx += 4;
                }
                Some(16 * 9)
            }
            PAGE_ANGLES => {
                if out.len() < 68 {
                    return None;
                }
                let mut idx = 0usize;
                for v in self.angles_inj.iter() {
                    let b = v.to_le_bytes();
                    out[idx] = b[0];
                    out[idx + 1] = b[1];
                    idx += 2;
                }
                for v in self.angles_tdc.iter() {
                    let b = v.to_le_bytes();
                    out[idx] = b[0];
                    out[idx + 1] = b[1];
                    idx += 2;
                }
                let b = self.tooth0_angle_x10.to_le_bytes();
                out[idx] = b[0];
                out[idx + 1] = b[1];
                idx += 2;
                // cam missing timeout (ms)
                let b = self.cam_timeout_ms.to_le_bytes();
                out[idx] = b[0];
                out[idx + 1] = b[1];
                Some(68)
            }
            PAGE_DFCO => {
                if out.len() < 16 {
                    return None;
                }
                let mut idx = 0;
                out[idx] = self.dfco.tps_max_pct;
                idx += 1;
                out[idx] = 0;
                idx += 1;
                let w16 = |buf: &mut [u8], i: &mut usize, v: u16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                let w32 = |buf: &mut [u8], i: &mut usize, v: u32| {
                    buf[*i..*i + 4].copy_from_slice(&v.to_le_bytes());
                    *i += 4;
                };
                w16(out, &mut idx, self.dfco.map_max_kpa);
                w16(out, &mut idx, self.dfco.rpm_min);
                w16(out, &mut idx, self.dfco.rpm_max);
                w32(out, &mut idx, self.dfco.delay_ms);
                w32(out, &mut idx, self.dfco.resume_hyst_ms);
                Some(16)
            }
            PAGE_WUE => {
                if out.len() < 8 {
                    return None;
                }
                let mut idx = 0usize;
                let w16s = |buf: &mut [u8], i: &mut usize, v: i16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                out[idx] = self.wue.max_percent;
                idx += 1;
                out[idx] = self.wue.min_percent;
                idx += 1;
                w16s(out, &mut idx, self.wue.start_c);
                w16s(out, &mut idx, self.wue.end_c);
                out[idx] = 0;
                idx += 1; // reserved
                out[idx] = 0; // reserved
                Some(8)
            }
            PAGE_ASE => {
                if out.len() < 8 {
                    return None;
                }
                let mut idx = 0usize;
                out[idx] = self.ase.percent;
                idx += 2; // +1 reserved
                let w32 = |buf: &mut [u8], i: &mut usize, v: u32| {
                    buf[*i..*i + 4].copy_from_slice(&v.to_le_bytes());
                    *i += 4;
                };
                w32(out, &mut idx, self.ase.taper_time_ms);
                let lock = (self.ase.lockout_ms as u16).to_le_bytes();
                out[idx] = lock[0];
                out[idx + 1] = lock[1];
                Some(8)
            }
            PAGE_IDLE => {
                if out.len() < 6 {
                    return None;
                }
                out[0] = self.idle.enable as u8;
                out[1..3].copy_from_slice(&self.idle.duty_x10.to_le_bytes());
                out[3..5].copy_from_slice(&self.idle.freq_hz.to_le_bytes());
                out[5] = 0;
                Some(6)
            }
            PAGE_FAN => {
                if out.len() < 6 {
                    return None;
                }
                out[0] = self.fan.enable as u8;
                out[1..3].copy_from_slice(&self.fan.on_c.to_le_bytes());
                out[3..5].copy_from_slice(&self.fan.off_c.to_le_bytes());
                out[5] = 0;
                Some(6)
            }
            PAGE_CL => {
                if out.len() < 8 {
                    return None;
                }
                out[0] = self.cl.enable as u8;
                out[1] = 0;
                out[2..4].copy_from_slice(&self.cl.target_afr_x10.to_le_bytes());
                out[4..6].copy_from_slice(&self.cl.kp_i.to_le_bytes());
                out[6..8].copy_from_slice(&self.cl.ki_i.to_le_bytes());
                Some(8)
            }
            PAGE_SNAPSHOT => {
                if out.len() < 32 {
                    return None;
                }
                let mut idx = 0usize;
                let w16 = |buf: &mut [u8], i: &mut usize, v: u16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                let w16s = |buf: &mut [u8], i: &mut usize, v: i16| {
                    buf[*i..*i + 2].copy_from_slice(&v.to_le_bytes());
                    *i += 2;
                };
                let w32 = |buf: &mut [u8], i: &mut usize, v: u32| {
                    buf[*i..*i + 4].copy_from_slice(&v.to_le_bytes());
                    *i += 4;
                };
                w16(out, &mut idx, self.snapshot.rpm.raw());
                out[idx] = match self.snapshot.sync {
                    SyncState::Unsynced => 0,
                    SyncState::Provisional => 1,
                    SyncState::Locked { .. } => 2,
                };
                idx += 1;
                out[idx] = 0;
                idx += 1;
                w32(out, &mut idx, self.snapshot.base_pw.raw());
                w16(out, &mut idx, self.snapshot.enrich_mult_x100);
                w16s(out, &mut idx, self.snapshot.stft_x10);
                w16(out, &mut idx, self.snapshot.fuel_mult_x100);
                w32(out, &mut idx, self.snapshot.final_pw.raw());
                out[idx] = match self.snapshot.last_fault {
                    None => 0,
                    Some(diag::DiagCode::MapRange) => 1,
                    Some(diag::DiagCode::TpsRange) => 2,
                    Some(diag::DiagCode::CamMissing) => 3,
                    Some(diag::DiagCode::LowVoltage) => 4,
                    Some(diag::DiagCode::Overvoltage) => 5,
                    Some(diag::DiagCode::MapFailureHighLoad) => 6,
                    Some(diag::DiagCode::TpsMapPlausibility) => 7,
                    Some(diag::DiagCode::KnockDetected) => 8,
                };
                idx += 1;
                out[idx] = 0;
                idx += 1;
                w32(out, &mut idx, self.snapshot.isr_stats.count);
                w32(out, &mut idx, self.snapshot.isr_stats.max_us);
                w32(out, &mut idx, self.snapshot.isr_stats.avg_us);
                Some(32)
            }
            PAGE_EXPERT_TRIGGER => self.expert_trigger.read(out),
            _ => None,
        }
    }
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_FUEL => EcuStatePageStore {
                fuel: self.fuel,
                ign: self.ign,
            }
            .write_fuel(data),
            PAGE_IGN => EcuStatePageStore {
                fuel: self.fuel,
                ign: self.ign,
            }
            .write_ign(data),
            PAGE_SENSORS => self.write_sensors(data),
            PAGE_LIMITS => {
                if data.len() < 10 {
                    return Err(PageError::WrongSize);
                }
                let mut idx = 0;
                let r16 = |d: &[u8], i: &mut usize| -> u16 {
                    let v = u16::from_le_bytes([d[*i], d[*i + 1]]);
                    *i += 2;
                    v
                };
                let map_min = r16(data, &mut idx);
                let map_max = r16(data, &mut idx);
                let tps_min = data[idx];
                idx += 1;
                let tps_max = data[idx];
                idx += 1;
                let clear_s = r16(data, &mut idx);
                let trig = data[idx]; // next byte reserved
                if map_min == 0 || map_max <= map_min {
                    return Err(PageError::Invalid);
                }
                if tps_max > 100 || tps_max <= tps_min {
                    return Err(PageError::Invalid);
                }
                if clear_s == 0 {
                    return Err(PageError::Invalid);
                }
                self.limits.map_min_kpa_x10 = map_min;
                self.limits.map_max_kpa_x10 = map_max;
                self.limits.tps_min_percent = tps_min;
                self.limits.tps_max_percent = tps_max;
                self.limits.clear_time_s = clear_s;
                *self.emerg_trig_map = (trig & 1) != 0;
                *self.emerg_trig_tps = (trig & 2) != 0;
                Ok(())
            }
            PAGE_AE => {
                if data.len() < 16 {
                    return Err(PageError::WrongSize);
                }
                let mut idx = 0;
                let r16s = |d: &[u8], i: &mut usize| -> i16 {
                    let v = i16::from_le_bytes([d[*i], d[*i + 1]]);
                    *i += 2;
                    v
                };
                let r32 = |d: &[u8], i: &mut usize| -> u32 {
                    let v = u32::from_le_bytes([d[*i], d[*i + 1], d[*i + 2], d[*i + 3]]);
                    *i += 4;
                    v
                };
                let tpsdot = r16s(data, &mut idx);
                let mapdot = r16s(data, &mut idx);
                let percent_gain = data[idx];
                idx += 2;
                let decay = r32(data, &mut idx);
                let lockout = r32(data, &mut idx);
                if percent_gain > 100 || decay == 0 {
                    return Err(PageError::Invalid);
                }
                self.ae.tpsdot_thresh_pct_s = tpsdot;
                self.ae.mapdot_thresh_kpa_s = mapdot;
                self.ae.percent_gain = percent_gain;
                self.ae.decay_time_ms = decay;
                self.ae.lockout_ms = lockout;
                Ok(())
            }
            PAGE_DFCO => {
                if data.len() < 16 {
                    return Err(PageError::WrongSize);
                }
                let mut idx = 0;
                let r16 = |d: &[u8], i: &mut usize| -> u16 {
                    let v = u16::from_le_bytes([d[*i], d[*i + 1]]);
                    *i += 2;
                    v
                };
                let r32 = |d: &[u8], i: &mut usize| -> u32 {
                    let v = u32::from_le_bytes([d[*i], d[*i + 1], d[*i + 2], d[*i + 3]]);
                    *i += 4;
                    v
                };
                let tps_max = data[idx];
                idx += 2;
                let map_max = r16(data, &mut idx);
                let rpm_min = r16(data, &mut idx);
                let rpm_max = r16(data, &mut idx);
                let delay = r32(data, &mut idx);
                let hyst = r32(data, &mut idx);
                if tps_max > 100 || rpm_min == 0 || rpm_max < rpm_min {
                    return Err(PageError::Invalid);
                }
                self.dfco.tps_max_pct = tps_max;
                self.dfco.map_max_kpa = map_max;
                self.dfco.rpm_min = rpm_min;
                self.dfco.rpm_max = rpm_max;
                self.dfco.delay_ms = delay;
                self.dfco.resume_hyst_ms = hyst;
                Ok(())
            }
            PAGE_WUE => {
                if data.len() < 8 {
                    return Err(PageError::WrongSize);
                }
                let mut idx = 0usize;
                let maxp = data[idx];
                idx += 1;
                let minp = data[idx];
                idx += 1;
                let r16s = |d: &[u8], i: &mut usize| -> i16 {
                    let v = i16::from_le_bytes([d[*i], d[*i + 1]]);
                    *i += 2;
                    v
                };
                let start_c = r16s(data, &mut idx);
                let end_c = r16s(data, &mut idx);
                if maxp > 100 || minp > 100 || start_c >= end_c {
                    return Err(PageError::Invalid);
                }
                self.wue.max_percent = maxp;
                self.wue.min_percent = minp;
                self.wue.start_c = start_c;
                self.wue.end_c = end_c;
                Ok(())
            }
            PAGE_ASE => {
                if data.len() < 8 {
                    return Err(PageError::WrongSize);
                }
                let mut idx = 0usize;
                let pct = data[idx];
                idx += 2; // skip reserved
                let r32 = |d: &[u8], i: &mut usize| -> u32 {
                    let v = u32::from_le_bytes([d[*i], d[*i + 1], d[*i + 2], d[*i + 3]]);
                    *i += 4;
                    v
                };
                let taper = r32(data, &mut idx);
                let lock = u16::from_le_bytes([data[idx], data[idx + 1]]) as u32;
                if pct > 100 || taper == 0 {
                    return Err(PageError::Invalid);
                }
                self.ase.percent = pct;
                self.ase.taper_time_ms = taper;
                self.ase.lockout_ms = lock;
                Ok(())
            }
            PAGE_IDLE => {
                if data.len() < 6 {
                    return Err(PageError::WrongSize);
                }
                let en = data[0] != 0;
                let duty = u16::from_le_bytes([data[1], data[2]]);
                let freq = u16::from_le_bytes([data[3], data[4]]);
                if duty > 1000 || freq == 0 {
                    return Err(PageError::Invalid);
                }
                self.idle.enable = en;
                self.idle.duty_x10 = duty;
                self.idle.freq_hz = freq;
                Ok(())
            }
            PAGE_FAN => {
                if data.len() < 6 {
                    return Err(PageError::WrongSize);
                }
                let en = data[0] != 0;
                let on = i16::from_le_bytes([data[1], data[2]]);
                let off = i16::from_le_bytes([data[3], data[4]]);
                if on <= off {
                    return Err(PageError::Invalid);
                }
                self.fan.enable = en;
                self.fan.on_c = on;
                self.fan.off_c = off;
                Ok(())
            }
            PAGE_CL => {
                if data.len() < 8 {
                    return Err(PageError::WrongSize);
                }
                let en = data[0] != 0;
                let target = u16::from_le_bytes([data[2], data[3]]);
                let kp = u16::from_le_bytes([data[4], data[5]]);
                let ki = u16::from_le_bytes([data[6], data[7]]);
                if !(100..=220).contains(&target) {
                    return Err(PageError::Invalid);
                }
                self.cl.enable = en;
                self.cl.target_afr_x10 = target;
                self.cl.kp_i = kp;
                self.cl.ki_i = ki;
                Ok(())
            }
            PAGE_DIAG => Err(PageError::Invalid),
            PAGE_EXPERT_TRIGGER => self.expert_trigger.write(data),
            PAGE_ANGLES => {
                if data.len() < 68 {
                    return Err(PageError::WrongSize);
                }
                let mut idx = 0usize;
                let r16 = |d: &[u8], i: &mut usize| -> u16 {
                    let v = u16::from_le_bytes([d[*i], d[*i + 1]]);
                    *i += 2;
                    v
                };
                // inj angles: 0..=3600 (deg*10 BTDC)
                for v in self.angles_inj.iter_mut() {
                    let a = r16(data, &mut idx);
                    if a > 3600 {
                        return Err(PageError::Invalid);
                    }
                    *v = a;
                }
                // per-cylinder TDC angles: 0..=7200 (deg*10 within 720° domain)
                for v in self.angles_tdc.iter_mut() {
                    let a = r16(data, &mut idx);
                    if a > 7200 {
                        return Err(PageError::Invalid);
                    }
                    *v = a;
                }
                // tooth0 reference angle in 0..=3600
                let t0 = r16(data, &mut idx);
                if t0 > 3600 {
                    return Err(PageError::Invalid);
                }
                *self.tooth0_angle_x10 = t0;
                *self.cam_timeout_ms = r16(data, &mut idx);
                Ok(())
            }
            _ => Err(PageError::Invalid),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ts::PageStore;
    use crate::EcuState;

    #[test]
    fn descriptor_registry_covers_all_known_pages() {
        const EXPECTED: [TsPageDescriptor; TS_PAGE_COUNT] = [
            TsPageDescriptor {
                page: PAGE_FUEL,
                len: TS_PAGE_BYTES,
                writable: true,
                label: "fuel",
            },
            TsPageDescriptor {
                page: PAGE_IGN,
                len: TS_PAGE_BYTES,
                writable: true,
                label: "ign",
            },
            TsPageDescriptor {
                page: PAGE_SENSORS,
                len: TS_SENSORS_BYTES,
                writable: true,
                label: "sensors",
            },
            TsPageDescriptor {
                page: PAGE_AE,
                len: 16,
                writable: true,
                label: "ae",
            },
            TsPageDescriptor {
                page: PAGE_DFCO,
                len: 16,
                writable: true,
                label: "dfco",
            },
            TsPageDescriptor {
                page: PAGE_LIMITS,
                len: 10,
                writable: true,
                label: "limits",
            },
            TsPageDescriptor {
                page: PAGE_DIAG,
                len: TS_DIAG_BYTES,
                writable: false,
                label: "diag",
            },
            TsPageDescriptor {
                page: PAGE_DIAG_LOG,
                len: 16 * 9,
                writable: false,
                label: "diag_log",
            },
            TsPageDescriptor {
                page: PAGE_ANGLES,
                len: 68,
                writable: true,
                label: "angles",
            },
            TsPageDescriptor {
                page: PAGE_WUE,
                len: 8,
                writable: true,
                label: "wue",
            },
            TsPageDescriptor {
                page: PAGE_ASE,
                len: 8,
                writable: true,
                label: "ase",
            },
            TsPageDescriptor {
                page: PAGE_IDLE,
                len: 6,
                writable: true,
                label: "idle",
            },
            TsPageDescriptor {
                page: PAGE_FAN,
                len: 6,
                writable: true,
                label: "fan",
            },
            TsPageDescriptor {
                page: PAGE_CL,
                len: 8,
                writable: true,
                label: "cl",
            },
            TsPageDescriptor {
                page: PAGE_SNAPSHOT,
                len: 32,
                writable: false,
                label: "snapshot",
            },
            TsPageDescriptor {
                page: PAGE_EXPERT_TRIGGER,
                len: TS_EXPERT_TRIGGER_BYTES,
                writable: true,
                label: "expert_trigger",
            },
        ];

        assert_eq!(TS_PAGE_DESCRIPTORS, EXPECTED);
        for descriptor in TS_PAGE_DESCRIPTORS {
            assert_eq!(
                ts_page_descriptor(descriptor.page),
                Some(descriptor),
                "descriptor lookup must be stable for page {}",
                descriptor.page
            );
        }
        assert!(ts_page_descriptor(99).is_none());
    }

    #[test]
    fn descriptor_lengths_match_read_paths() {
        let mut state = EcuState::new();
        let store = state.page_store();

        for descriptor in TS_PAGE_DESCRIPTORS {
            assert_eq!(store.page_len(descriptor.page), Some(descriptor.len));
            let mut out = vec![0u8; descriptor.len];
            assert_eq!(
                store.read_page(descriptor.page, &mut out),
                Some(descriptor.len),
                "page {} ({}): read size drift",
                descriptor.page,
                descriptor.label
            );
        }
    }
}
