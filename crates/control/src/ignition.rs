use ecu_domain::{Degrees10, DwellUs, Rpm};

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
