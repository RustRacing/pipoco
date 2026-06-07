//! Acceleration Enrichment (AE) based on TPSdot/MAPdot with decay and lockout
#![allow(clippy::manual_range_contains)]

pub use ecu_calibration::configs::AeConfig;

#[derive(Copy, Clone)]
pub struct AeState {
    pub active: bool,
    pub current_percent: u8,
    last_trigger_us: u32,
}

impl AeState {
    pub const fn new() -> Self {
        Self {
            active: false,
            current_percent: 0,
            last_trigger_us: 0,
        }
    }

    pub fn update(
        &mut self,
        now_us: u32,
        tpsdot_pct_s: i16,
        mapdot_kpa_s: i16,
        cfg: &AeConfig,
    ) -> u8 {
        // Trigger if either derivative exceeds threshold and lockout passed
        let since = now_us.wrapping_sub(self.last_trigger_us);
        let lockout_us = cfg.lockout_ms * 1000;
        let first_time = self.last_trigger_us == 0 && !self.active;
        if (tpsdot_pct_s >= cfg.tpsdot_thresh_pct_s || mapdot_kpa_s >= cfg.mapdot_thresh_kpa_s)
            && (first_time || since >= lockout_us)
        {
            self.active = true;
            self.current_percent = cfg.percent_gain;
            self.last_trigger_us = now_us;
            return self.current_percent;
        }

        // Decay if active
        if self.active {
            let decay_us = cfg.decay_time_ms * 1000;
            let elapsed = now_us.wrapping_sub(self.last_trigger_us) as u64;
            if elapsed >= decay_us as u64 {
                self.current_percent = 0;
                self.active = false;
            } else {
                // Linear decay
                let remain = decay_us as u64 - elapsed;
                let pct = (cfg.percent_gain as u64 * remain) / (decay_us as u64);
                self.current_percent = pct as u8;
            }
        }
        self.current_percent
    }
}

impl Default for AeState {
    fn default() -> Self {
        Self::new()
    }
}

// Warmup Enrichment (WUE): linear percent vs CLT
pub use ecu_calibration::configs::WueConfig;

// AfterStart Enrichment (ASE): trigger on exit from cranking, taper to 0
pub use ecu_calibration::configs::AseConfig;

#[derive(Copy, Clone)]
pub struct AseState {
    pub active: bool,
    start_us: u32,
    last_trigger_us: u32,
}

impl AseState {
    pub const fn new() -> Self {
        Self {
            active: false,
            start_us: 0,
            last_trigger_us: 0,
        }
    }
    pub fn update(&mut self, now_us: u32, just_started: bool, cfg: &AseConfig) -> u8 {
        if just_started {
            let since = now_us.wrapping_sub(self.last_trigger_us);
            if self.last_trigger_us == 0 || since >= cfg.lockout_ms * 1000 {
                self.active = true;
                self.start_us = now_us;
                self.last_trigger_us = now_us;
            }
        }
        if !self.active {
            return 0;
        }
        let elapsed = now_us.wrapping_sub(self.start_us) as u64;
        let dur = (cfg.taper_time_ms as u64) * 1000;
        if elapsed >= dur {
            self.active = false;
            return 0;
        }
        let remaining = dur - elapsed;
        let pct = (cfg.percent as u64) * remaining / dur;
        pct as u8
    }
}

impl Default for AseState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ae_trigger_and_decay() {
        let cfg = AeConfig::DEFAULT;
        let mut st = AeState::new();
        // Trigger at t=0
        let p = st.update(0, 200, 0, &cfg);
        assert!(st.active && p == cfg.percent_gain);
        // Halfway decay
        let p2 = st.update(cfg.decay_time_ms * 500, 0, 0, &cfg);
        assert!(p2 > 0 && p2 < cfg.percent_gain);
        // After decay time
        let p3 = st.update(cfg.decay_time_ms * 1000 + 1, 0, 0, &cfg);
        assert_eq!(p3, 0);
        assert!(!st.active);
    }

    #[test]
    fn test_ae_lockout() {
        let cfg = AeConfig {
            lockout_ms: 200,
            ..AeConfig::DEFAULT
        };
        let mut st = AeState::new();
        let _ = st.update(0, 200, 0, &cfg);
        // Within lockout
        let p = st.update(100_000, 200, 0, &cfg);
        assert!(p <= cfg.percent_gain);
        // After lockout, can retrigger
        let p2 = st.update(250_000, 200, 0, &cfg);
        assert_eq!(p2, cfg.percent_gain);
    }

    #[test]
    fn test_wue_linear_profile() {
        let w = WueConfig {
            start_c: -20,
            end_c: 60,
            max_percent: 40,
            min_percent: 0,
        };
        assert_eq!(w.compute_percent(-30), 40); // below start
        assert_eq!(w.compute_percent(60), 0); // at end
        assert!(w.compute_percent(20) > 0 && w.compute_percent(20) < 40);
    }

    #[test]
    fn test_ase_trigger_and_taper() {
        let cfg = AseConfig {
            percent: 20,
            taper_time_ms: 1000,
            lockout_ms: 500,
        };
        let mut st = AseState::new();
        // Trigger on just_started
        let p0 = st.update(0, true, &cfg);
        assert!(st.active);
        assert!(p0 <= cfg.percent);
        // Halfway
        let p1 = st.update(500_000, false, &cfg);
        assert!(p1 > 0 && p1 < cfg.percent);
        // After taper
        let p2 = st.update(1_000_000 + 1, false, &cfg);
        assert_eq!(p2, 0);
        assert!(!st.active);
        // Retrigger after sufficient time (beyond lockout)
        let _ = st.update(1_100_000, true, &cfg);
        assert!(st.active);
    }
}
