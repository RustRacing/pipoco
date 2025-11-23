//! Acceleration Enrichment (AE) based on TPSdot/MAPdot with decay and lockout
#![allow(clippy::manual_range_contains)]

#[derive(Copy, Clone)]
pub struct AeConfig {
    pub tpsdot_thresh_pct_s: i16, // %/s threshold
    pub mapdot_thresh_kpa_s: i16, // kPa/s threshold
    pub percent_gain: u8,         // added fuel % at trigger
    pub decay_time_ms: u32,       // time to decay back to 0
    pub lockout_ms: u32,          // minimum time between triggers
}

impl AeConfig {
    pub const DEFAULT: Self = Self {
        tpsdot_thresh_pct_s: 150,
        mapdot_thresh_kpa_s: 80,
        percent_gain: 15,
        decay_time_ms: 400,
        lockout_ms: 150,
    };
}

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
}
