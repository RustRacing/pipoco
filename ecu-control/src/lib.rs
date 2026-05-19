#![cfg_attr(not(test), no_std)]

use ecu_domain::{Degrees10, DwellUs, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm};

/// First-pass base fuel model backed by an IPW table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaseFuelModel {
    rpm_bins: [Rpm; 16],
    load_bins: [Kpa10; 16],
    pulse_widths: [[PulseWidthUs; 16]; 16],
}

impl BaseFuelModel {
    pub const fn new(
        rpm_bins: [Rpm; 16],
        load_bins: [Kpa10; 16],
        pulse_widths: [[PulseWidthUs; 16]; 16],
    ) -> Self {
        Self {
            rpm_bins,
            load_bins,
            pulse_widths,
        }
    }

    pub const fn rpm_bins(self) -> [Rpm; 16] {
        self.rpm_bins
    }

    pub const fn load_bins(self) -> [Kpa10; 16] {
        self.load_bins
    }

    pub const fn pulse_widths(self) -> [[PulseWidthUs; 16]; 16] {
        self.pulse_widths
    }

    fn nearest_index_rpm(&self, value: Rpm) -> usize {
        nearest_index_rpm(&self.rpm_bins, value)
    }

    fn nearest_index_load(&self, value: Kpa10) -> usize {
        nearest_index_kpa10(&self.load_bins, value)
    }
}

impl Default for BaseFuelModel {
    fn default() -> Self {
        Self {
            rpm_bins: [Rpm::new(0); 16],
            load_bins: [Kpa10::new(0); 16],
            pulse_widths: [[PulseWidthUs::new(0); 16]; 16],
        }
    }
}

/// Base fuel calculation interface.
pub trait FuelBaseCalculator {
    fn calculate_base_fuel(&self, rpm: Rpm, load: Kpa10) -> PulseWidthUs;
}

impl FuelBaseCalculator for BaseFuelModel {
    fn calculate_base_fuel(&self, rpm: Rpm, load: Kpa10) -> PulseWidthUs {
        let rpm_idx = self.nearest_index_rpm(rpm);
        let load_idx = self.nearest_index_load(load);
        self.pulse_widths[load_idx][rpm_idx]
    }
}

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
        if self.start_c >= self.end_c {
            return 100;
        }
        if clt_c <= self.start_c {
            return self.max_percent_x100;
        }
        if clt_c >= self.end_c {
            return self.min_percent_x100;
        }

        let span = (self.end_c - self.start_c) as i32;
        let pos = (clt_c - self.start_c) as i32;
        let max = self.max_percent_x100 as i32;
        let min = self.min_percent_x100 as i32;
        let value = max - (max - min) * pos / span;
        value.clamp(100, 200) as u16
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
        let scaled = (base.get() as u32 * self.total_x100() as u32) / 100;
        let clamped = if scaled > u16::MAX as u32 {
            u16::MAX
        } else {
            scaled as u16
        };
        PulseWidthUs::new(clamped)
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
}

/// Warmup enrichment state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct WarmupState {
    last_percent_x100: u16,
}

impl WarmupState {
    pub const fn new() -> Self {
        Self {
            last_percent_x100: 100,
        }
    }

    pub fn update(&mut self, clt_c: i16, cfg: &WarmupConfig) -> u16 {
        self.last_percent_x100 = cfg.compute_percent_x100(clt_c);
        self.last_percent_x100
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
    pub warmup: WarmupState,
    pub after_start: AfterStartState,
    pub acceleration: AccelerationState,
}

impl EnrichmentController {
    pub const fn new() -> Self {
        Self {
            startup: StartupState::new(),
            warmup: WarmupState::new(),
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
        let warmup_x100 = self.warmup.update(inputs.clt_c, warmup_cfg);
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

/// Lambda control operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LambdaMode {
    #[default]
    OpenLoop,
    ClosedLoop,
}

/// Configuration for the first-pass lambda trim planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LambdaTrimConfig {
    pub open_loop_target: Lambda100,
    pub closed_loop_target: Lambda100,
    pub enable_clt_c: i16,
    pub disable_clt_c: i16,
    pub min_trim_x100: i16,
    pub max_trim_x100: i16,
    pub gain_x10: u8,
}

impl LambdaTrimConfig {
    pub const DEFAULT: Self = Self {
        open_loop_target: Lambda100::new(100),
        closed_loop_target: Lambda100::new(100),
        enable_clt_c: 40,
        disable_clt_c: 30,
        min_trim_x100: 85,
        max_trim_x100: 115,
        gain_x10: 4,
    };
}

impl Default for LambdaTrimConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Inputs required to compute lambda trim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LambdaTrimInputs {
    pub clt_c: i16,
    pub lambda_valid: bool,
    pub measured_lambda100: Lambda100,
    pub requested_open_loop: bool,
}

/// Typed lambda trim result for downstream consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LambdaTrimResult {
    pub mode: LambdaMode,
    pub active: bool,
    pub target_lambda100: Lambda100,
    pub measured_lambda100: Lambda100,
    pub trim_x100: i16,
}

impl LambdaTrimResult {
    pub const fn new(
        mode: LambdaMode,
        active: bool,
        target_lambda100: Lambda100,
        measured_lambda100: Lambda100,
        trim_x100: i16,
    ) -> Self {
        Self {
            mode,
            active,
            target_lambda100,
            measured_lambda100,
            trim_x100,
        }
    }

    pub const fn identity(target_lambda100: Lambda100, measured_lambda100: Lambda100) -> Self {
        Self::new(
            LambdaMode::OpenLoop,
            false,
            target_lambda100,
            measured_lambda100,
            100,
        )
    }
}

/// First-pass lambda trim planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LambdaTrimPlanner {
    last_mode: LambdaMode,
    last_trim_x100: i16,
}

impl LambdaTrimPlanner {
    pub const fn new() -> Self {
        Self {
            last_mode: LambdaMode::OpenLoop,
            last_trim_x100: 100,
        }
    }

    pub fn update(&mut self, inputs: LambdaTrimInputs, cfg: &LambdaTrimConfig) -> LambdaTrimResult {
        let closed_loop_enabled = inputs.lambda_valid
            && !inputs.requested_open_loop
            && inputs.clt_c >= cfg.enable_clt_c
            && (self.last_mode == LambdaMode::ClosedLoop || inputs.clt_c >= cfg.disable_clt_c);

        if !closed_loop_enabled {
            self.last_mode = LambdaMode::OpenLoop;
            self.last_trim_x100 = 100;
            return LambdaTrimResult::new(
                LambdaMode::OpenLoop,
                false,
                cfg.open_loop_target,
                inputs.measured_lambda100,
                100,
            );
        }

        let target = cfg.closed_loop_target.get() as i16;
        let measured = inputs.measured_lambda100.get() as i16;
        let error = target - measured;
        let mut trim = 100 + (error * cfg.gain_x10 as i16) / 10;
        trim = trim.clamp(cfg.min_trim_x100, cfg.max_trim_x100);

        self.last_mode = LambdaMode::ClosedLoop;
        self.last_trim_x100 = trim;
        LambdaTrimResult::new(
            LambdaMode::ClosedLoop,
            true,
            cfg.closed_loop_target,
            inputs.measured_lambda100,
            trim,
        )
    }
}

/// Reason a product torque command is limited.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TorqueLimitReason {
    #[default]
    None,
    Idle,
    Driver,
    RevLimiter,
    Knock,
    LimpMode,
}

/// First-pass product torque inputs, expressed as x100 percent of nominal
/// torque.
///
/// This is the runtime control surface, not the frozen x1000 semantic oracle
/// torque pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TorqueInputs {
    pub driver_request_x100: u16,
    pub idle_request_x100: u16,
    pub rev_limit_x100: u16,
    pub knock_limit_x100: u16,
    pub limp_limit_x100: u16,
}

impl TorqueInputs {
    pub const fn new(
        driver_request_x100: u16,
        idle_request_x100: u16,
        rev_limit_x100: u16,
        knock_limit_x100: u16,
        limp_limit_x100: u16,
    ) -> Self {
        Self {
            driver_request_x100,
            idle_request_x100,
            rev_limit_x100,
            knock_limit_x100,
            limp_limit_x100,
        }
    }
}

/// Allowed torque result for downstream planners.
///
/// The runtime planner consumes x100 torque demand; it does not expose the
/// semantic-oracle x1000 observability surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AllowedTorque {
    pub requested_x100: u16,
    pub allowed_x100: u16,
    pub reason: TorqueLimitReason,
}

impl AllowedTorque {
    pub const fn new(requested_x100: u16, allowed_x100: u16, reason: TorqueLimitReason) -> Self {
        Self {
            requested_x100,
            allowed_x100,
            reason,
        }
    }
}

/// Torque arbiter that resolves demand against runtime safety caps.
///
/// This is the product torque limiter path. The frozen spec torque pipeline is
/// mirrored separately in `ecu-runtime` test scaffolding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TorqueArbiter;

impl TorqueArbiter {
    pub const fn new() -> Self {
        Self
    }

    pub fn evaluate(&self, inputs: TorqueInputs) -> AllowedTorque {
        let requested_x100 = inputs.driver_request_x100.max(inputs.idle_request_x100);
        let mut allowed_x100 = requested_x100;
        let mut reason = if inputs.idle_request_x100 >= inputs.driver_request_x100 {
            TorqueLimitReason::Idle
        } else {
            TorqueLimitReason::Driver
        };

        let caps = [
            (inputs.rev_limit_x100, TorqueLimitReason::RevLimiter),
            (inputs.knock_limit_x100, TorqueLimitReason::Knock),
            (inputs.limp_limit_x100, TorqueLimitReason::LimpMode),
        ];
        for (cap, cap_reason) in caps {
            if cap < allowed_x100 {
                allowed_x100 = cap;
                reason = cap_reason;
            }
        }

        if allowed_x100 >= requested_x100 {
            reason = TorqueLimitReason::None;
        }

        AllowedTorque::new(requested_x100, allowed_x100, reason)
    }
}

/// Reason ignition timing was limited.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IgnitionLimitReason {
    #[default]
    None,
    Knock,
    Torque,
    RevLimiter,
}

/// First-pass ignition planning inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IgnitionInputs {
    pub base_advance_deg10: Degrees10,
    pub timing_correction_deg10: i16,
    pub knock_retard_deg10: i16,
    pub torque_retard_deg10: i16,
    pub rev_limit_active: bool,
    pub rpm: Rpm,
}

impl IgnitionInputs {
    pub const fn new(
        base_advance_deg10: Degrees10,
        timing_correction_deg10: i16,
        knock_retard_deg10: i16,
        torque_retard_deg10: i16,
        rev_limit_active: bool,
        rpm: Rpm,
    ) -> Self {
        Self {
            base_advance_deg10,
            timing_correction_deg10,
            knock_retard_deg10,
            torque_retard_deg10,
            rev_limit_active,
            rpm,
        }
    }
}

/// Configuration for dwell planning.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DwellConfig {
    pub base_dwell_us: u16,
    pub min_dwell_us: u16,
    pub max_dwell_us: u16,
    pub rpm_dwell_trim_us: u16,
    pub rpm_trim_start: Rpm,
    pub rpm_trim_end: Rpm,
}

impl DwellConfig {
    pub const DEFAULT: Self = Self {
        base_dwell_us: 2500,
        min_dwell_us: 1500,
        max_dwell_us: 3500,
        rpm_dwell_trim_us: 500,
        rpm_trim_start: Rpm::new(2000),
        rpm_trim_end: Rpm::new(7000),
    };

    pub fn compute_dwell_us(&self, rpm: Rpm) -> DwellUs {
        if rpm <= self.rpm_trim_start {
            return DwellUs::new(self.max_dwell_us);
        }
        if rpm >= self.rpm_trim_end {
            return DwellUs::new(self.min_dwell_us);
        }

        let span = (self.rpm_trim_end.get() - self.rpm_trim_start.get()) as u32;
        let pos = (rpm.get() - self.rpm_trim_start.get()) as u32;
        let trim = self.rpm_dwell_trim_us as u32 * pos / span.max(1);
        let dwell = self.base_dwell_us.saturating_sub(trim as u16);
        DwellUs::new(dwell.clamp(self.min_dwell_us, self.max_dwell_us))
    }
}

impl Default for DwellConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Fully planned ignition output for downstream scheduler use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IgnitionPlan {
    pub advance_deg10: Degrees10,
    pub dwell_us: DwellUs,
    pub limit_reason: IgnitionLimitReason,
}

impl IgnitionPlan {
    pub const fn new(
        advance_deg10: Degrees10,
        dwell_us: DwellUs,
        limit_reason: IgnitionLimitReason,
    ) -> Self {
        Self {
            advance_deg10,
            dwell_us,
            limit_reason,
        }
    }
}

/// First-pass ignition planner.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IgnitionPlanner;

impl IgnitionPlanner {
    pub const fn new() -> Self {
        Self
    }

    pub fn plan(&self, inputs: IgnitionInputs, dwell_cfg: &DwellConfig) -> IgnitionPlan {
        let mut advance = inputs.base_advance_deg10.get() as i32
            + inputs.timing_correction_deg10 as i32
            - inputs.knock_retard_deg10 as i32;
        let mut limit_reason = if inputs.knock_retard_deg10 > 0 {
            IgnitionLimitReason::Knock
        } else {
            IgnitionLimitReason::None
        };

        if inputs.torque_retard_deg10 > 0 {
            advance -= inputs.torque_retard_deg10 as i32;
            limit_reason = IgnitionLimitReason::Torque;
        }

        if inputs.rev_limit_active {
            advance -= 80;
            limit_reason = IgnitionLimitReason::RevLimiter;
        }

        let advance = advance.clamp(-200, 600) as i16;
        IgnitionPlan::new(
            Degrees10::new(advance),
            dwell_cfg.compute_dwell_us(inputs.rpm),
            limit_reason,
        )
    }
}

fn nearest_index_rpm(bins: &[Rpm; 16], value: Rpm) -> usize {
    let mut idx = 0usize;
    while idx < 15 {
        if value.get() < bins[idx + 1].get() {
            return idx;
        }
        idx += 1;
    }
    15
}

fn nearest_index_kpa10(bins: &[Kpa10; 16], value: Kpa10) -> usize {
    let mut idx = 0usize;
    while idx < 15 {
        if value.get() < bins[idx + 1].get() {
            return idx;
        }
        idx += 1;
    }
    15
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_model() -> BaseFuelModel {
        let rpm_bins = [
            Rpm::new(500),
            Rpm::new(1000),
            Rpm::new(1500),
            Rpm::new(2000),
            Rpm::new(2500),
            Rpm::new(3000),
            Rpm::new(3500),
            Rpm::new(4000),
            Rpm::new(4500),
            Rpm::new(5000),
            Rpm::new(5500),
            Rpm::new(6000),
            Rpm::new(6500),
            Rpm::new(7000),
            Rpm::new(7500),
            Rpm::new(8000),
        ];
        let load_bins = [
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1000),
            Kpa10::new(1100),
            Kpa10::new(1200),
            Kpa10::new(1300),
            Kpa10::new(1400),
            Kpa10::new(1500),
            Kpa10::new(1600),
            Kpa10::new(1700),
        ];
        let mut pulse_widths = [[PulseWidthUs::new(0); 16]; 16];
        pulse_widths[0][0] = PulseWidthUs::new(1000);
        pulse_widths[5][5] = PulseWidthUs::new(2500);
        pulse_widths[15][15] = PulseWidthUs::new(4000);
        BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
    }

    #[test]
    fn base_fuel_uses_nearest_lower_bin() {
        let model = test_model();

        assert_eq!(
            model
                .calculate_base_fuel(Rpm::new(750), Kpa10::new(250))
                .get(),
            1000
        );
    }

    #[test]
    fn base_fuel_selects_matching_mid_table_cell() {
        let model = test_model();

        assert_eq!(
            model
                .calculate_base_fuel(Rpm::new(3000), Kpa10::new(700))
                .get(),
            2500
        );
    }

    #[test]
    fn base_fuel_clamps_to_upper_edge() {
        let model = test_model();

        assert_eq!(
            model
                .calculate_base_fuel(Rpm::new(9000), Kpa10::new(1900))
                .get(),
            4000
        );
    }

    #[test]
    fn model_accessors_round_trip() {
        let model = test_model();

        assert_eq!(model.rpm_bins()[0].get(), 500);
        assert_eq!(model.load_bins()[0].get(), 200);
        assert_eq!(model.pulse_widths()[5][5].get(), 2500);
    }

    #[test]
    fn startup_tapers_after_cranking() {
        let cfg = StartupConfig::DEFAULT;
        let mut st = StartupState::new();

        assert_eq!(st.update(Micros::new(0), true, &cfg), cfg.percent_x100);
        let halfway = st.update(Micros::new(cfg.taper_time_ms * 500), false, &cfg);
        assert!(halfway > 100 && halfway < cfg.percent_x100);
        assert_eq!(
            st.update(Micros::new(cfg.taper_time_ms * 1000 + 1), false, &cfg),
            100
        );
    }

    #[test]
    fn warmup_interpolates_by_coolant_temp() {
        let cfg = WarmupConfig::DEFAULT;
        let mut st = WarmupState::new();

        assert_eq!(st.update(-30, &cfg), cfg.max_percent_x100);
        assert_eq!(st.update(60, &cfg), cfg.min_percent_x100);
        let mid = st.update(20, &cfg);
        assert!(mid > cfg.min_percent_x100 && mid < cfg.max_percent_x100);
    }

    #[test]
    fn after_start_triggers_and_decays_with_lockout() {
        let cfg = AfterStartConfig::DEFAULT;
        let mut st = AfterStartState::new();

        assert_eq!(st.update(Micros::new(0), true, &cfg), cfg.percent_x100);
        let halfway = st.update(Micros::new(cfg.taper_time_ms * 500), false, &cfg);
        assert!(halfway > 100 && halfway < cfg.percent_x100);
        assert_eq!(
            st.update(Micros::new(cfg.taper_time_ms * 1000 + 1), false, &cfg),
            100
        );
    }

    #[test]
    fn acceleration_enrichment_triggers_and_decays() {
        let cfg = AccelerationConfig::DEFAULT;
        let mut st = AccelerationState::new();

        assert_eq!(st.update(Micros::new(0), 200, 0, &cfg), cfg.percent_x100);
        let halfway = st.update(Micros::new(cfg.decay_time_ms * 500), 0, 0, &cfg);
        assert!(halfway > 100 && halfway < cfg.percent_x100);
        assert_eq!(
            st.update(Micros::new(cfg.decay_time_ms * 1000 + 1), 0, 0, &cfg),
            100
        );
    }

    #[test]
    fn enrichment_controller_returns_combined_result() {
        let mut ctrl = EnrichmentController::new();
        let result = ctrl.update(
            EnrichmentInputs {
                now_us: Micros::new(0),
                clt_c: -20,
                cranking: true,
                just_started: true,
                tpsdot_pct_s: 200,
                mapdot_kpa_s: 0,
            },
            &StartupConfig::DEFAULT,
            &WarmupConfig::DEFAULT,
            &AfterStartConfig::DEFAULT,
            &AccelerationConfig::DEFAULT,
        );

        assert_eq!(result.startup_x100, StartupConfig::DEFAULT.percent_x100);
        assert_eq!(result.warmup_x100, WarmupConfig::DEFAULT.max_percent_x100);
        assert_eq!(
            result.total_x100(),
            result.apply_to(PulseWidthUs::new(100)).get()
        );
    }

    #[test]
    fn lambda_planner_stays_open_loop_when_disabled() {
        let mut planner = LambdaTrimPlanner::new();
        let result = planner.update(
            LambdaTrimInputs {
                clt_c: 20,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(105),
                requested_open_loop: true,
            },
            &LambdaTrimConfig::DEFAULT,
        );

        assert_eq!(result.mode, LambdaMode::OpenLoop);
        assert!(!result.active);
        assert_eq!(result.trim_x100, 100);
        assert_eq!(
            result.target_lambda100,
            LambdaTrimConfig::DEFAULT.open_loop_target
        );
    }

    #[test]
    fn lambda_planner_enters_closed_loop_and_clamps_trim() {
        let mut planner = LambdaTrimPlanner::new();
        let cfg = LambdaTrimConfig::DEFAULT;

        let result = planner.update(
            LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(90),
                requested_open_loop: false,
            },
            &cfg,
        );

        assert_eq!(result.mode, LambdaMode::ClosedLoop);
        assert!(result.active);
        assert_eq!(result.target_lambda100, cfg.closed_loop_target);
        assert!(result.trim_x100 >= cfg.min_trim_x100);
        assert!(result.trim_x100 <= cfg.max_trim_x100);
    }

    #[test]
    fn torque_arbiter_prefers_requested_torque_when_unlimited() {
        let arbiter = TorqueArbiter::new();
        let result = arbiter.evaluate(TorqueInputs::new(90, 80, 120, 130, 140));

        assert_eq!(result.requested_x100, 90);
        assert_eq!(result.allowed_x100, 90);
        assert_eq!(result.reason, TorqueLimitReason::None);
    }

    #[test]
    fn torque_arbiter_applies_limp_cap_over_other_limits() {
        let arbiter = TorqueArbiter::new();
        let result = arbiter.evaluate(TorqueInputs::new(120, 100, 110, 105, 70));

        assert_eq!(result.requested_x100, 120);
        assert_eq!(result.allowed_x100, 70);
        assert_eq!(result.reason, TorqueLimitReason::LimpMode);
    }

    #[test]
    fn torque_arbiter_uses_idle_request_when_higher() {
        let arbiter = TorqueArbiter::new();
        let result = arbiter.evaluate(TorqueInputs::new(60, 85, 120, 120, 120));

        assert_eq!(result.requested_x100, 85);
        assert_eq!(result.allowed_x100, 85);
        assert_eq!(result.reason, TorqueLimitReason::None);
    }

    #[test]
    fn ignition_planner_combines_base_and_corrections() {
        let planner = IgnitionPlanner::new();
        let plan = planner.plan(
            IgnitionInputs::new(Degrees10::new(120), 10, 5, 0, false, Rpm::new(2500)),
            &DwellConfig::DEFAULT,
        );

        assert_eq!(plan.advance_deg10.get(), 125);
        assert_eq!(plan.limit_reason, IgnitionLimitReason::Knock);
        assert!(plan.dwell_us.get() >= DwellConfig::DEFAULT.min_dwell_us);
        assert!(plan.dwell_us.get() <= DwellConfig::DEFAULT.max_dwell_us);
    }

    #[test]
    fn ignition_planner_applies_rev_limit_and_lower_dwell_at_high_rpm() {
        let planner = IgnitionPlanner::new();
        let low_rpm = planner.plan(
            IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, Rpm::new(2000)),
            &DwellConfig::DEFAULT,
        );
        let high_rpm = planner.plan(
            IgnitionInputs::new(Degrees10::new(120), 0, 0, 10, true, Rpm::new(7000)),
            &DwellConfig::DEFAULT,
        );

        assert_eq!(high_rpm.limit_reason, IgnitionLimitReason::RevLimiter);
        assert!(high_rpm.advance_deg10.get() < low_rpm.advance_deg10.get());
        assert!(high_rpm.dwell_us.get() <= low_rpm.dwell_us.get());
    }

    #[test]
    fn control_planners_compose_purely() {
        let model = test_model();
        let base_pw = model.calculate_base_fuel(Rpm::new(3000), Kpa10::new(700));

        let mut enrichment = EnrichmentController::new();
        let enrich = enrichment.update(
            EnrichmentInputs {
                now_us: Micros::new(0),
                clt_c: 10,
                cranking: true,
                just_started: true,
                tpsdot_pct_s: 180,
                mapdot_kpa_s: 90,
            },
            &StartupConfig::DEFAULT,
            &WarmupConfig::DEFAULT,
            &AfterStartConfig::DEFAULT,
            &AccelerationConfig::DEFAULT,
        );

        let mut lambda = LambdaTrimPlanner::new();
        let lambda_result = lambda.update(
            LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(96),
                requested_open_loop: false,
            },
            &LambdaTrimConfig::DEFAULT,
        );

        let torque = TorqueArbiter::new().evaluate(TorqueInputs::new(92, 80, 120, 118, 110));
        let ignition = IgnitionPlanner::new().plan(
            IgnitionInputs::new(Degrees10::new(110), 8, 2, 4, false, Rpm::new(2800)),
            &DwellConfig::DEFAULT,
        );

        assert_eq!(base_pw.get(), 2500);
        assert!(enrich.total_x100() >= StartupConfig::DEFAULT.percent_x100);
        assert_eq!(lambda_result.mode, LambdaMode::ClosedLoop);
        assert_eq!(torque.allowed_x100, 92);
        assert_eq!(ignition.advance_deg10.get(), 112);
        assert!(ignition.dwell_us.get() >= DwellConfig::DEFAULT.min_dwell_us);
        assert!(ignition.dwell_us.get() <= DwellConfig::DEFAULT.max_dwell_us);
        assert_eq!(
            enrich.apply_to(base_pw).get(),
            ((base_pw.get() as u32 * enrich.total_x100() as u32) / 100) as u16
        );
    }
}
