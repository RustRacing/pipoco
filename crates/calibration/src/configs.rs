//! Persisted tune/threshold calibration data extracted from ecu-compat.
//!
//! These are pure calibration structs (integer fields, no heap, no panics).
//! The associated state machines and logic remain in ecu-compat/ecu-control.

#![allow(clippy::manual_range_contains)]

/// Acceleration Enrichment (AE) configuration.
#[derive(Copy, Clone)]
pub struct AeConfig {
    pub tpsdot_thresh_pct_s: i16,
    pub mapdot_thresh_kpa_s: i16,
    pub percent_gain: u8,
    pub decay_time_ms: u32,
    pub lockout_ms: u32,
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

/// Warmup Enrichment (WUE) configuration: linear percent vs CLT.
#[derive(Copy, Clone)]
pub struct WueConfig {
    pub start_c: i16,
    pub end_c: i16,
    pub max_percent: u8,
    pub min_percent: u8,
}

impl WueConfig {
    pub const DEFAULT: Self = Self {
        start_c: -20,
        end_c: 60,
        max_percent: 40,
        min_percent: 0,
    };

    pub fn compute_percent(&self, clt_c: i16) -> u8 {
        if self.start_c >= self.end_c {
            return 0;
        }
        if clt_c <= self.start_c {
            return self.max_percent;
        }
        if clt_c >= self.end_c {
            return self.min_percent;
        }
        let span = (self.end_c - self.start_c) as i32;
        let pos = (clt_c - self.start_c) as i32;
        let max = self.max_percent as i32;
        let min = self.min_percent as i32;
        let val = max - (max - min) * pos / span;
        val.clamp(0, 100) as u8
    }
}

/// AfterStart Enrichment (ASE) configuration.
#[derive(Copy, Clone)]
pub struct AseConfig {
    pub percent: u8,
    pub taper_time_ms: u32,
    pub lockout_ms: u32,
}

impl AseConfig {
    pub const DEFAULT: Self = Self {
        percent: 20,
        taper_time_ms: 5000,
        lockout_ms: 2000,
    };
}

/// Decel Fuel Cut (DFCO) configuration.
#[derive(Copy, Clone)]
pub struct DfcoConfig {
    pub tps_max_pct: u8,
    pub map_max_kpa: u16,
    pub rpm_min: u16,
    pub rpm_max: u16,
    pub delay_ms: u32,
    pub resume_hyst_ms: u32,
}

impl DfcoConfig {
    pub const DEFAULT: Self = Self {
        tps_max_pct: 2,
        map_max_kpa: 30,
        rpm_min: 1500,
        rpm_max: 7000,
        delay_ms: 200,
        resume_hyst_ms: 300,
    };
}

/// Open-loop idle PWM configuration.
#[derive(Copy, Clone)]
pub struct IdleConfig {
    pub enable: bool,
    pub duty_x10: u16,
    pub freq_hz: u16,
}

impl IdleConfig {
    pub const DEFAULT: Self = Self {
        enable: false,
        duty_x10: 0,
        freq_hz: 100,
    };
}

/// Cooling-fan control configuration.
#[derive(Copy, Clone)]
pub struct FanConfig {
    pub enable: bool,
    pub on_c: i16,
    pub off_c: i16,
}

impl FanConfig {
    pub const DEFAULT: Self = Self {
        enable: false,
        on_c: 95,
        off_c: 90,
    };
}

/// Closed-loop actuator stub configuration.
#[derive(Copy, Clone)]
pub struct ClConfig {
    pub enable: bool,
    pub target_afr_x10: u16,
    pub kp_i: u16,
    pub ki_i: u16,
}

impl ClConfig {
    pub const DEFAULT: Self = Self {
        enable: false,
        target_afr_x10: 147,
        kp_i: 0,
        ki_i: 0,
    };
}

/// Runtime sensor limits and clear timing.
#[derive(Copy, Clone)]
pub struct SensorsLimits {
    pub map_min_kpa_x10: u16,
    pub map_max_kpa_x10: u16,
    pub tps_min_percent: u8,
    pub tps_max_percent: u8,
    /// Seconds in-range before clearing an active diag
    pub clear_time_s: u16,
}

impl SensorsLimits {
    pub const fn default() -> Self {
        Self {
            map_min_kpa_x10: 100,
            map_max_kpa_x10: 3000,
            tps_min_percent: 0,
            tps_max_percent: 100,
            clear_time_s: 3,
        }
    }
}

/// Configuration for load failure detection.
#[derive(Debug, Clone, Copy)]
pub struct LoadFailureConfig {
    pub enable: bool,
    pub rpm_threshold: u16,
    pub limp_rpm_limit: u16,
    pub recovery_time_us: u32,
    pub debounce_time_us: u32,
}

impl LoadFailureConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        rpm_threshold: 4000,
        limp_rpm_limit: 3000,
        recovery_time_us: 2_000_000,
        debounce_time_us: 100_000,
    };
}

impl Default for LoadFailureConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Configuration for sensor plausibility checks.
#[derive(Debug, Clone, Copy)]
pub struct PlausibilityConfig {
    pub enable: bool,
    pub tps_high_threshold: u8,
    pub map_low_threshold_x10: u16,
    pub tps_low_threshold: u8,
    pub map_high_threshold_x10: u16,
    pub min_rpm: u16,
    pub debounce_time_us: u32,
}

impl PlausibilityConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        tps_high_threshold: 80,
        map_low_threshold_x10: 300,
        tps_low_threshold: 10,
        map_high_threshold_x10: 950,
        min_rpm: 1000,
        debounce_time_us: 500_000,
    };
}

impl Default for PlausibilityConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Configuration for rate-of-change validation.
#[derive(Debug, Clone, Copy)]
pub struct RateConfig {
    pub enable: bool,
    pub max_tps_rate_per_sec: u16,
    pub max_map_rate_per_sec: u16,
    pub min_sample_interval_us: u32,
}

impl RateConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        max_tps_rate_per_sec: 500,
        max_map_rate_per_sec: 2000,
        min_sample_interval_us: 1000,
    };
}

impl Default for RateConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// O2 sensor type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum O2SensorType {
    /// Narrowband (0-1V, rich/lean only).
    #[default]
    Narrowband,
    /// Wideband (0-5V = 10-20 AFR typical).
    Wideband,
}

/// Configuration for the lambda controller.
#[derive(Debug, Clone, Copy)]
pub struct LambdaConfig {
    /// Enable closed-loop control.
    pub enable: bool,
    /// Target AFR x10 (e.g., 147 = 14.7:1 stoichiometric).
    pub target_afr_x10: u16,
    /// Proportional gain x100 (e.g., 50 = 0.50).
    pub kp_x100: u16,
    /// Integral gain x100 (e.g., 10 = 0.10).
    pub ki_x100: u16,
    /// Maximum authority (percent x10, e.g., 200 = 20%).
    pub authority_max_x10: i16,
    /// Narrowband O2 sensor threshold (millivolts).
    pub narrowband_threshold_mv: u16,
    /// Deadband around target (millivolts).
    pub deadband_mv: u16,
    /// Minimum coolant temperature for closed-loop (Celsius).
    pub min_clt_c: i16,
    /// Maximum TPS for closed-loop (percent).
    /// Above this, run open-loop (WOT enrichment).
    pub max_tps_percent: u8,
    /// Minimum RPM for closed-loop.
    pub min_rpm: u16,
    /// Update interval (microseconds).
    pub update_interval_us: u32,
}

impl LambdaConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        target_afr_x10: 147,
        kp_x100: 50,
        ki_x100: 10,
        authority_max_x10: 200,
        narrowband_threshold_mv: 450,
        deadband_mv: 20,
        min_clt_c: 60,
        max_tps_percent: 80,
        min_rpm: 1200,
        update_interval_us: 100_000,
    };
}

impl Default for LambdaConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Configuration for LTFT learning.
#[derive(Debug, Clone, Copy)]
pub struct LtftConfig {
    /// Enable LTFT learning.
    pub enable: bool,
    /// Learning rate (0-255, lower = slower learning).
    pub learn_rate: u8,
    /// Maximum trim authority (percent x10).
    pub max_trim_x10: i16,
    /// Minimum coolant temperature for learning (Celsius).
    pub min_clt_c: i16,
    /// STFT must be below this threshold to learn (percent x10).
    pub stft_threshold_x10: i16,
    /// Time at steady-state before learning (microseconds).
    pub steady_state_time_us: u32,
}

impl LtftConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        learn_rate: 4,
        max_trim_x10: 100,
        min_clt_c: 70,
        stft_threshold_x10: 30,
        steady_state_time_us: 2_000_000,
    };
}

impl Default for LtftConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Rev limiter strategy.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LimiterStrategy {
    /// Hard cut: completely disable fuel/ignition
    HardCut,
    /// Soft cut: gradually reduce fuel
    SoftCut,
    /// Ignition retard: reduce power by retarding timing
    IgnitionRetard,
    /// Combined: ignition retard + fuel cut
    Combined,
}

/// Rev limiter configuration: persisted RPM-protection thresholds.
#[derive(Debug, Clone, Copy)]
pub struct RevLimiterConfig {
    /// Maximum RPM before limiting starts
    pub max_rpm: u16,

    /// RPM below max_rpm where soft limiting begins (soft cut only)
    pub soft_limit_start_rpm: u16,

    /// Strategy to use
    pub strategy: LimiterStrategy,

    /// Number of cylinders to cut in hard cut mode (1-4)
    pub cut_cylinders: u8,

    /// Ignition retard amount (degrees) for retard strategy
    pub retard_amount: i16,

    /// Hysteresis: RPM must drop this much below limit before re-enabling
    pub hysteresis_rpm: u16,
}

impl RevLimiterConfig {
    /// Conservative default configuration (street safe)
    pub const DEFAULT: Self = Self {
        max_rpm: 7000,
        soft_limit_start_rpm: 6500,
        strategy: LimiterStrategy::SoftCut,
        cut_cylinders: 2,  // Cut 2 cylinders (50% reduction)
        retard_amount: 15, // 15° retard
        hysteresis_rpm: 200,
    };

    /// Aggressive race configuration (hard cut)
    pub const RACE: Self = Self {
        max_rpm: 8000,
        soft_limit_start_rpm: 7800,
        strategy: LimiterStrategy::HardCut,
        cut_cylinders: 4, // Cut all cylinders (full cut)
        retard_amount: 0,
        hysteresis_rpm: 200,
    };

    /// Launch control configuration (smooth power limiting)
    pub const LAUNCH: Self = Self {
        max_rpm: 4000, // Launch RPM limit
        soft_limit_start_rpm: 3900,
        strategy: LimiterStrategy::Combined,
        cut_cylinders: 1,
        retard_amount: 10,
        hysteresis_rpm: 100,
    };
}

/// Global ECU state configuration.
pub struct EcuConfig {
    pub ipw_table: [[u16; 16]; 16],
    pub ve_table: [[u16; 16]; 16],
    pub afr_table: [[u16; 16]; 16],
    pub required_fuel_us: u16,
    pub injector_deadtime_us: u16,
    pub ve_load_source: u8, // 0=MAP, 1=TPS
    pub ignition_table: [[i16; 16]; 16],
    pub sensors_cal: crate::sensors::SensorsCal,
    pub sensors_limits: SensorsLimits,
    pub ae_config: AeConfig,
    pub wue_config: WueConfig,
    pub ase_config: AseConfig,
    pub dfco_config: DfcoConfig,
    pub idle_config: IdleConfig,
    pub fan_config: FanConfig,
    pub cl_config: ClConfig,
    pub load_failure_config: LoadFailureConfig,
    pub plausibility_config: PlausibilityConfig,
    pub rate_config: RateConfig,
    pub lambda_config: LambdaConfig,
    pub rev_limiter_config: RevLimiterConfig,
    pub inj_angle_btdc_x10: [u16; 16],
    pub tdc_per_cyl_x10: [u16; 16],
    pub tooth0_angle_x10: u16,
    pub cam_missing_timeout_ms: u16,
}
