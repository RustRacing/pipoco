use ecu_core::persist::KvStore;
use ecu_core::ts::pages::{EcuPageStore, PAGE_FUEL, PAGE_IGN};
use ecu_core::ts::server::{PageError, PageStore, PersistError};
use ecu_core::EcuState;

/// Combined page store that exposes all ECU pages (fuel/ign/sensors/AE/DFCO)
/// and persists only fuel/ignition via a KvStore.
///
/// This avoids multiple overlapping borrows by holding a single EcuPageStore
/// and performing persistence by reading/writing the two 512-byte tables.
pub struct PersistedEcuPageStore<'a, KV: KvStore> {
    inner: EcuPageStore<'a>,
    kv: KV,
}

impl<'a, KV: KvStore> PersistedEcuPageStore<'a, KV> {
    pub fn new(state: &'a mut EcuState, kv: KV) -> Self {
        let inner = EcuPageStore {
            fuel: &mut state.ipw_table,
            ign: &mut state.ignition_table,
            sens: &mut state.sensors_cal,
            ae: &mut state.ae_config,
            dfco: &mut state.dfco_config,
            idle: &mut state.idle_config,
            fan: &mut state.fan_config,
            cl: &mut state.cl_config,
            wue: &mut state.wue_config,
            ase: &mut state.ase_config,
            limits: &mut state.sensors_limits,
            emerg_trig_map: &mut state.emergency_trigger_map_oob,
            emerg_trig_tps: &mut state.emergency_trigger_tps_oob,
            diag_emergency: &state.emergency_mode,
            diag_map: &state.diag_map,
            diag_tps: &state.diag_tps,
            diag_cam: &state.diag_cam,
            diag_log: &state.diag_log,
            angles_inj: &mut state.inj_angle_btdc_x10,
            angles_tdc: &mut state.tdc_per_cyl_x10,
            tooth0_angle_x10: &mut state.tooth0_angle_x10,
            cam_timeout_ms: &mut state.cam_missing_timeout_ms,
        };
        Self { inner, kv }
    }

    /// Attempt to load persisted pages (fuel/ign) into the in-memory tables.
    pub fn try_load(&mut self) {
        let mut buf = [0u8; 512];
        if let Ok(n) = self.kv.read(b"fuel", &mut buf) {
            if n == 512 { let _ = self.inner.write_page(PAGE_FUEL, &buf); }
        }
        if let Ok(n) = self.kv.read(b"ign", &mut buf) {
            if n == 512 { let _ = self.inner.write_page(PAGE_IGN, &buf); }
        }
        // Angles (68 bytes)
        let mut abuf = [0u8; 68];
        if let Ok(n) = self.kv.read(b"angles", &mut abuf) { if n == 68 { let _ = self.inner.write_page(ecu_core::ts::pages::PAGE_ANGLES, &abuf); } }
    }

    /// Reset persisted pages in memory to safe defaults (does not write KV until burn)
    pub fn factory_reset(&mut self) {
        // Fuel defaults
        let _ = self.inner.write_page(PAGE_FUEL, &[0u8; 512]);
        // Ign defaults
        let _ = self.inner.write_page(PAGE_IGN, &[0u8; 512]);
        // Angles defaults (zeros, and default cam timeout)
        let mut angles = [0u8; 68];
        // tooth0_angle_x10=0 at [64..66], cam_timeout_ms default=500 at [66..68]
        angles[66..68].copy_from_slice(&500u16.to_le_bytes());
        let _ = self.inner.write_page(ecu_core::ts::pages::PAGE_ANGLES, &angles);
    }
}

impl<'a, KV: KvStore> PageStore for PersistedEcuPageStore<'a, KV> {
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
        if self.inner.read_page(PAGE_FUEL, &mut buf).unwrap_or(0) == 512 {
            self.kv
                .write(b"fuel", &buf)
                .map_err(|_| PersistError::Fail)?;
        }
        if self.inner.read_page(PAGE_IGN, &mut buf).unwrap_or(0) == 512 {
            self.kv
                .write(b"ign", &buf)
                .map_err(|_| PersistError::Fail)?;
        }
        // Write angles
        let mut angles = [0u8; 68];
        if self
            .inner
            .read_page(ecu_core::ts::pages::PAGE_ANGLES, &mut angles)
            .unwrap_or(0)
            == 68
        {
            self.kv
                .write(b"angles", &angles)
                .map_err(|_| PersistError::Fail)?;
        }
        Ok(())
    }
}
