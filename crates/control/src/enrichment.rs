use ecu_domain::{Micros, PulseWidthUs};

use crate::types::FuelWarmupTemperatureMode;

/// Configurable startup enrichment behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StartupConfig {
    pub percent_x100: u16,
    pub taper_time_ms: u32,
}

impl StartupConfig {
    pub const DEFAULT: Self = Self {
        percent_x100: 150,
        taper_time_ms: 3000,
    };
}

impl Default for StartupConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Configurable warmup enrichment behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WarmupConfig {
    pub start_c: i16,
    pub end_c: i16,
    pub max_percent_x100: u16,
    pub min_percent_x100: u16,
}

impl WarmupConfig {
    pub const DEFAULT: Self = Self {
        start_c: -20,
        end_c: 60,
        max_percent_x100: 140,
        min_percent_x100: 100,
    };

    pub fn compute_percent_x100(&self, clt_c: i16) -> u16 {
        let min = self.min_percent_x100.min(self.max_percent_x100) as i32;
        let max = self.min_percent_x100.max(self.max_percent_x100) as i32;

        if self.start_c >= self.end_c {
            return 100_i32.clamp(min, max) as u16;
        }
        if clt_c <= self.start_c {
            return (self.max_percent_x100 as i32).clamp(min, max) as u16;
        }
        if clt_c >= self.end_c {
            return (self.min_percent_x100 as i32).clamp(min, max) as u16;
        }

        let span = (self.end_c - self.start_c) as i32;
        let pos = (clt_c - self.start_c) as i32;
        let configured_max = self.max_percent_x100 as i32;
        let configured_min = self.min_percent_x100 as i32;
        let value = configured_max - (configured_max - configured_min) * pos / span;
        value.clamp(min, max) as u16
    }

    pub fn temperature_mode(&self, clt_c: i16) -> FuelWarmupTemperatureMode {
        if self.start_c >= self.end_c {
            return FuelWarmupTemperatureMode::NeutralFallback;
        }
        if clt_c <= self.start_c {
            return FuelWarmupTemperatureMode::ColdClamp;
        }
        if clt_c >= self.end_c {
            return FuelWarmupTemperatureMode::HotClamp;
        }
        FuelWarmupTemperatureMode::Interpolating
    }
}

impl Default for WarmupConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Configurable after-start enrichment behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AfterStartConfig {
    pub percent_x100: u16,
    pub taper_time_ms: u32,
    pub lockout_ms: u32,
}

impl AfterStartConfig {
    pub const DEFAULT: Self = Self {
        percent_x100: 120,
        taper_time_ms: 5000,
        lockout_ms: 2000,
    };
}

impl Default for AfterStartConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Configurable acceleration enrichment behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AccelerationConfig {
    pub tpsdot_thresh_pct_s: i16,
    pub mapdot_thresh_kpa_s: i16,
    pub percent_x100: u16,
    pub decay_time_ms: u32,
    pub lockout_ms: u32,
}

impl AccelerationConfig {
    pub const DEFAULT: Self = Self {
        tpsdot_thresh_pct_s: 150,
        mapdot_thresh_kpa_s: 80,
        percent_x100: 115,
        decay_time_ms: 400,
        lockout_ms: 150,
    };
}

impl Default for AccelerationConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Inputs required to evaluate enrichments.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnrichmentInputs {
    pub now_us: Micros,
    pub clt_c: i16,
    pub cranking: bool,
    pub just_started: bool,
    pub tpsdot_pct_s: i16,
    pub mapdot_kpa_s: i16,
}

/// Combined enrichment output, expressed as x100 multipliers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnrichmentResult {
    pub startup_x100: u16,
    pub warmup_x100: u16,
    pub after_start_x100: u16,
    pub acceleration_x100: u16,
}

impl EnrichmentResult {
    pub const fn new(
        startup_x100: u16,
        warmup_x100: u16,
        after_start_x100: u16,
        acceleration_x100: u16,
    ) -> Self {
        Self {
            startup_x100,
            warmup_x100,
            after_start_x100,
            acceleration_x100,
        }
    }

    pub fn total_x100(self) -> u16 {
        let mut total = self.startup_x100 as u32;
        total = (total * self.warmup_x100 as u32) / 100;
        total = (total * self.after_start_x100 as u32) / 100;
        total = (total * self.acceleration_x100 as u32) / 100;
        total as u16
    }

    pub fn apply_to(self, base: PulseWidthUs) -> PulseWidthUs {
        PulseWidthUs::new((base.get() * self.total_x100() as u32) / 100)
    }
}

/// Startup enrichment state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StartupState {
    active: bool,
    start_us: Micros,
}

impl StartupState {
    pub const fn new() -> Self {
        Self {
            active: false,
            start_us: Micros::new(0),
        }
    }

    pub fn update(&mut self, now_us: Micros, cranking: bool, cfg: &StartupConfig) -> u16 {
        if cranking {
            self.active = true;
            self.start_us = now_us;
            return cfg.percent_x100;
        }

        if !self.active {
            return 100;
        }

        let elapsed = now_us.get().wrapping_sub(self.start_us.get());
        let taper_us = cfg.taper_time_ms.saturating_mul(1000);
        if elapsed >= taper_us {
            self.active = false;
            return 100;
        }

        let remain = taper_us - elapsed;
        let extra = cfg.percent_x100.saturating_sub(100) as u32;
        let pct = 100 + (extra * remain) / taper_us.max(1);
        pct as u16
    }

    pub const fn active(&self) -> bool {
        self.active
    }

    pub fn remaining_ms(&self, now_us: Micros, cfg: &StartupConfig) -> u16 {
        if !self.active {
            return 0;
        }

        let taper_us = cfg.taper_time_ms.saturating_mul(1000);
        let elapsed = now_us.get().wrapping_sub(self.start_us.get());
        if elapsed >= taper_us {
            return 0;
        }

        let remain_ms = (taper_us - elapsed) / 1000;
        remain_ms.min(u32::from(u16::MAX)) as u16
    }
}

/// After-start enrichment state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AfterStartState {
    active: bool,
    start_us: Micros,
    last_trigger_us: Micros,
}

impl AfterStartState {
    pub const fn new() -> Self {
        Self {
            active: false,
            start_us: Micros::new(0),
            last_trigger_us: Micros::new(0),
        }
    }

    pub fn update(&mut self, now_us: Micros, just_started: bool, cfg: &AfterStartConfig) -> u16 {
        if just_started {
            let since = now_us.get().wrapping_sub(self.last_trigger_us.get());
            if self.last_trigger_us.get() == 0 || since >= cfg.lockout_ms.saturating_mul(1000) {
                self.active = true;
                self.start_us = now_us;
                self.last_trigger_us = now_us;
            }
        }

        if !self.active {
            return 100;
        }

        let elapsed = now_us.get().wrapping_sub(self.start_us.get());
        let dur = cfg.taper_time_ms.saturating_mul(1000);
        if elapsed >= dur {
            self.active = false;
            return 100;
        }

        let remain = dur - elapsed;
        let extra = cfg.percent_x100.saturating_sub(100) as u32;
        let pct = 100 + (extra * remain) / dur.max(1);
        pct as u16
    }

    pub const fn active(&self) -> bool {
        self.active
    }

    pub fn remaining_ms(&self, now_us: Micros, cfg: &AfterStartConfig) -> u16 {
        if !self.active {
            return 0;
        }

        let dur_us = cfg.taper_time_ms.saturating_mul(1000);
        let elapsed = now_us.get().wrapping_sub(self.start_us.get());
        if elapsed >= dur_us {
            return 0;
        }

        let remain_ms = (dur_us - elapsed) / 1000;
        remain_ms.min(u32::from(u16::MAX)) as u16
    }
}

/// Acceleration enrichment state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AccelerationState {
    active: bool,
    current_percent_x100: u16,
    last_trigger_us: Micros,
}

impl AccelerationState {
    pub const fn new() -> Self {
        Self {
            active: false,
            current_percent_x100: 100,
            last_trigger_us: Micros::new(0),
        }
    }

    pub fn update(
        &mut self,
        now_us: Micros,
        tpsdot_pct_s: i16,
        mapdot_kpa_s: i16,
        cfg: &AccelerationConfig,
    ) -> u16 {
        let since = now_us.get().wrapping_sub(self.last_trigger_us.get());
        let lockout_us = cfg.lockout_ms.saturating_mul(1000);
        let first_time = self.last_trigger_us.get() == 0 && !self.active;
        if (tpsdot_pct_s >= cfg.tpsdot_thresh_pct_s || mapdot_kpa_s >= cfg.mapdot_thresh_kpa_s)
            && (first_time || since >= lockout_us)
        {
            self.active = true;
            self.current_percent_x100 = cfg.percent_x100;
            self.last_trigger_us = now_us;
            return self.current_percent_x100;
        }

        if self.active {
            let decay_us = cfg.decay_time_ms.saturating_mul(1000);
            let elapsed = now_us.get().wrapping_sub(self.last_trigger_us.get()) as u64;
            if elapsed >= decay_us as u64 {
                self.current_percent_x100 = 100;
                self.active = false;
            } else {
                let remain = decay_us as u64 - elapsed;
                let extra = cfg.percent_x100.saturating_sub(100) as u64;
                let pct = 100 + (extra * remain) / (decay_us.max(1) as u64);
                self.current_percent_x100 = pct as u16;
            }
        }
        self.current_percent_x100
    }
}

/// Combined enrichment controller.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EnrichmentController {
    pub startup: StartupState,
    pub after_start: AfterStartState,
    pub acceleration: AccelerationState,
}

impl EnrichmentController {
    pub const fn new() -> Self {
        Self {
            startup: StartupState::new(),
            after_start: AfterStartState::new(),
            acceleration: AccelerationState::new(),
        }
    }

    pub fn update(
        &mut self,
        inputs: EnrichmentInputs,
        startup_cfg: &StartupConfig,
        warmup_cfg: &WarmupConfig,
        after_start_cfg: &AfterStartConfig,
        acceleration_cfg: &AccelerationConfig,
    ) -> EnrichmentResult {
        let startup_x100 = self
            .startup
            .update(inputs.now_us, inputs.cranking, startup_cfg);
        let warmup_x100 = warmup_cfg.compute_percent_x100(inputs.clt_c);
        let after_start_x100 =
            self.after_start
                .update(inputs.now_us, inputs.just_started, after_start_cfg);
        let acceleration_x100 = self.acceleration.update(
            inputs.now_us,
            inputs.tpsdot_pct_s,
            inputs.mapdot_kpa_s,
            acceleration_cfg,
        );

        EnrichmentResult::new(
            startup_x100,
            warmup_x100,
            after_start_x100,
            acceleration_x100,
        )
    }
}
