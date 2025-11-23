//! Page stores for fuel/ignition tables
use super::server::{PageError, PageStore, PersistError};
use crate::dfco::DfcoConfig;
use crate::enrichment::AeConfig;
use crate::persist::KvStore;
use crate::constants::{fuel as fuel_consts, ignition as ign_consts};
use crate::sensors::{SensorsCal, SensorsLimits};
use crate::diag;

/// Page numbers
pub const PAGE_FUEL: u8 = 1;
pub const PAGE_IGN: u8 = 2;
pub const PAGE_SENSORS: u8 = 3;
pub const PAGE_AE: u8 = 4;
pub const PAGE_DFCO: u8 = 5;
pub const PAGE_LIMITS: u8 = 6; // New page for sensor limits + emergency triggers
pub const PAGE_DIAG: u8 = 7;   // Read-only diagnostics summary
pub const PAGE_DIAG_LOG: u8 = 8; // Read-only recent diagnostics events
pub const PAGE_ANGLES: u8 = 9; // Per-cylinder angle + cam timeout config

/// Exposes references to fuel and ignition tables as TS pages
pub struct EcuStatePageStore<'a> {
    pub fuel: &'a mut [[u16; 16]; 16],
    pub ign: &'a mut [[i16; 16]; 16],
}

impl<'a> EcuStatePageStore<'a> {
    fn read_fuel(&self, out: &mut [u8]) -> Option<usize> {
        if out.len() < 512 {
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
        Some(512)
    }
    fn write_fuel(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() != 512 {
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
        if out.len() < 512 {
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
        Some(512)
    }
    fn write_ign(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() != 512 {
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
            PAGE_FUEL | PAGE_IGN => Some(512),
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

/// Keys used for persistence
const KEY_FUEL: &[u8] = b"fuel";
const KEY_IGN: &[u8] = b"ign";

/// Wrapper that adds persistence to an EcuState-backed PageStore using a KvStore
pub struct PersistedPageStore<'a, KV: KvStore> {
    inner: EcuStatePageStore<'a>,
    kv: KV,
}

impl<'a, KV: KvStore> PersistedPageStore<'a, KV> {
    pub const fn new(inner: EcuStatePageStore<'a>, kv: KV) -> Self {
        Self { inner, kv }
    }

    /// Attempt to load persisted pages into the in-memory tables.
    /// If keys are absent or invalid, leaves current values untouched.
    pub fn try_load(&mut self) {
        // Fuel
        let mut buf = [0u8; 512];
        if let Ok(n) = self.kv.read(KEY_FUEL, &mut buf) {
            if n == 512 {
                let _ = self.inner.write_fuel(&buf);
            }
        }
        // Ignition
        let mut ibuf = [0u8; 512];
        if let Ok(n) = self.kv.read(KEY_IGN, &mut ibuf) {
            if n == 512 {
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
        let mut buf = [0u8; 512];
        if self.inner.read_fuel(&mut buf).unwrap_or(0) == 512 {
            self.kv
                .write(KEY_FUEL, &buf)
                .map_err(|_| PersistError::Fail)?;
        }
        let mut ibuf = [0u8; 512];
        if self.inner.read_ign(&mut ibuf).unwrap_or(0) == 512 {
            self.kv
                .write(KEY_IGN, &ibuf)
                .map_err(|_| PersistError::Fail)?;
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
    pub limits: &'a mut SensorsLimits,
    pub emerg_trig_map: &'a mut bool,
    pub emerg_trig_tps: &'a mut bool,
    pub diag_emergency: &'a bool,
    pub diag_map: &'a diag::DiagState,
    pub diag_tps: &'a diag::DiagState,
    pub diag_cam: &'a diag::DiagState,
    pub diag_log: &'a diag::DiagLog<16>,
    pub angles_inj: &'a mut [u16; 16],
    pub angles_tdc: &'a mut [u16; 16],
    pub tooth0_angle_x10: &'a mut u16,
    pub cam_timeout_ms: &'a mut u16,
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
        if out.len() < 128 {
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
        Some(128)
    }

    fn write_sensors(&mut self, data: &[u8]) -> Result<(), PageError> {
        if data.len() < 128 {
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
        match page {
            PAGE_FUEL | PAGE_IGN => Some(512),
            PAGE_LIMITS => Some(10),
            PAGE_DIAG => Some(4),
            PAGE_DIAG_LOG => Some(16 * 9),
            PAGE_ANGLES => Some(68),
            PAGE_SENSORS => Some(128),
            PAGE_AE | PAGE_DFCO => Some(16),
            _ => None,
        }
    }
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_FUEL => {
                if out.len() < 512 {
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
                Some(512)
            }
            PAGE_IGN => {
                if out.len() < 512 {
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
                Some(512)
            }
            PAGE_SENSORS => self.read_sensors(out),
            PAGE_LIMITS => {
                if out.len() < 10 { return None; }
                let mut idx = 0;
                let w16 = |buf: &mut [u8], i: &mut usize, v: u16| { buf[*i..*i+2].copy_from_slice(&v.to_le_bytes()); *i += 2; };
                w16(out, &mut idx, self.limits.map_min_kpa_x10);
                w16(out, &mut idx, self.limits.map_max_kpa_x10);
                out[idx] = self.limits.tps_min_percent; idx += 1;
                out[idx] = self.limits.tps_max_percent; idx += 1;
                w16(out, &mut idx, self.limits.clear_time_s);
                let mut trig: u8 = 0;
                if *self.emerg_trig_map { trig |= 1; }
                if *self.emerg_trig_tps { trig |= 2; }
                out[idx] = trig; idx += 1; out[idx] = 0; /* reserved */
                Some(10)
            }
            PAGE_DIAG => {
                if out.len() < 4 { return None; }
                let mut flags: u16 = 0;
                if self.diag_map.active { flags |= 1 << 0; }
                if self.diag_tps.active { flags |= 1 << 1; }
                if self.diag_cam.active { flags |= 1 << 2; }
                if *self.diag_emergency { flags |= 1 << 3; }
                out[0..2].copy_from_slice(&flags.to_le_bytes());
                out[2] = 0; out[3] = 0; // reserved
                Some(4)
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
                if out.len() < 68 { return None; }
                let mut idx = 0usize;
                for v in self.angles_inj.iter() {
                    let b = v.to_le_bytes(); out[idx]=b[0]; out[idx+1]=b[1]; idx+=2;
                }
                for v in self.angles_tdc.iter() {
                    let b = v.to_le_bytes(); out[idx]=b[0]; out[idx+1]=b[1]; idx+=2;
                }
                let b = self.tooth0_angle_x10.to_le_bytes(); out[idx]=b[0]; out[idx+1]=b[1]; idx+=2;
                // cam missing timeout (ms)
                let b = self.cam_timeout_ms.to_le_bytes(); out[idx]=b[0]; out[idx+1]=b[1];
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
                if data.len() < 10 { return Err(PageError::WrongSize); }
                let mut idx = 0;
                let r16 = |d: &[u8], i: &mut usize| -> u16 { let v = u16::from_le_bytes([d[*i], d[*i+1]]); *i += 2; v };
                let map_min = r16(data, &mut idx);
                let map_max = r16(data, &mut idx);
                let tps_min = data[idx]; idx += 1;
                let tps_max = data[idx]; idx += 1;
                let clear_s = r16(data, &mut idx);
                let trig = data[idx]; // next byte reserved
                if map_min == 0 || map_max <= map_min { return Err(PageError::Invalid); }
                if tps_max > 100 || tps_max <= tps_min { return Err(PageError::Invalid); }
                if clear_s == 0 { return Err(PageError::Invalid); }
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
            PAGE_DIAG => Err(PageError::Invalid),
            PAGE_ANGLES => {
                if data.len() < 68 { return Err(PageError::WrongSize); }
                let mut idx = 0usize;
                let r16 = |d: &[u8], i: &mut usize| -> u16 { let v = u16::from_le_bytes([d[*i], d[*i+1]]); *i+=2; v };
                // inj angles: 0..=3600 (deg*10 BTDC)
                for v in self.angles_inj.iter_mut() {
                    let a = r16(data, &mut idx);
                    if a > 3600 { return Err(PageError::Invalid); }
                    *v = a;
                }
                // per-cylinder TDC angles: 0..=7200 (deg*10 within 720° domain)
                for v in self.angles_tdc.iter_mut() {
                    let a = r16(data, &mut idx);
                    if a > 7200 { return Err(PageError::Invalid); }
                    *v = a;
                }
                // tooth0 reference angle in 0..=3600
                let t0 = r16(data, &mut idx);
                if t0 > 3600 { return Err(PageError::Invalid); }
                *self.tooth0_angle_x10 = t0;
                *self.cam_timeout_ms = r16(data, &mut idx);
                Ok(())
            }
            _ => Err(PageError::Invalid),
        }
    }
}
