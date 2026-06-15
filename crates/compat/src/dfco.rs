//! Decel Fuel Cut (DFCO) detection

pub use ecu_calibration::configs::DfcoConfig;

#[derive(Copy, Clone)]
pub struct DfcoState {
    pub active: bool,
    enter_ts: u32,
    last_change_ts: u32,
    armed: bool,
}

impl DfcoState {
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
                // ensure delay elapsed
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
        } else {
            // resume with hysteresis
            if self.active {
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
        }
        self.active
    }
}

impl Default for DfcoState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dfco_engage_and_resume() {
        let cfg = DfcoConfig::DEFAULT;
        let mut st = DfcoState::new();
        // At time 0 conditions allow DFCO but delay applies
        assert!(!st.update(0, 2000, 0, 20, &cfg));
        // After delay
        assert!(st.update(cfg.delay_ms * 1000 + 1, 2000, 0, 20, &cfg));
        // Conditions break (TPS blip)
        assert!(st.update(cfg.delay_ms * 1000 + 50_000, 2000, 10, 20, &cfg)); // not yet resumed due to hysteresis
        assert!(!st.update(
            cfg.delay_ms * 1000 + cfg.resume_hyst_ms * 1000 + 2,
            2000,
            10,
            20,
            &cfg
        ));
    }
}
