use ecu_calibration::DfcoConfig;

/// Decel Fuel Cut (DFCO) runtime state.
#[derive(Copy, Clone)]
pub struct DecelFuelCutState {
    pub active: bool,
    enter_ts: u32,
    last_change_ts: u32,
    armed: bool,
}

impl DecelFuelCutState {
    pub const fn new() -> Self {
        Self {
            active: false,
            enter_ts: 0,
            last_change_ts: 0,
            armed: false,
        }
    }

    pub fn update(
        &mut self,
        now_us: u32,
        rpm: u16,
        tps_pct: u8,
        map_kpa: u16,
        cfg: &DfcoConfig,
    ) -> bool {
        let allow = tps_pct <= cfg.tps_max_pct
            && map_kpa <= cfg.map_max_kpa
            && rpm >= cfg.rpm_min
            && rpm <= cfg.rpm_max;
        if allow {
            if !self.active {
                if !self.armed {
                    self.enter_ts = now_us;
                    self.armed = true;
                }
                let delay_us = cfg.delay_ms * 1000;
                if now_us.wrapping_sub(self.enter_ts) >= delay_us {
                    self.active = true;
                    self.last_change_ts = now_us;
                }
            }
        } else if self.active {
            let hyst_us = cfg.resume_hyst_ms * 1000;
            if now_us.wrapping_sub(self.last_change_ts) >= hyst_us {
                self.active = false;
                self.enter_ts = 0;
                self.armed = false;
                self.last_change_ts = now_us;
            }
        } else {
            self.enter_ts = 0;
            self.armed = false;
        }
        self.active
    }
}

impl Default for DecelFuelCutState {
    fn default() -> Self {
        Self::new()
    }
}
