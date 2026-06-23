use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm, SyncState};

/// Fuel load-source selector before VE lookup.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FuelLoadSource {
    #[default]
    Map,
    Tps,
}

/// Fuel strategy operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FuelEngineMode {
    Off,
    Cranking,
    Running,
    Shutdown,
}

/// Optional AFR override for strategy evaluation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FuelAfrOverride {
    None,
    Some(u16),
}

/// Strategy-facing fuel inputs. Richer than direct IPW lookup inputs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelInputSnapshot {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub load_kpa10: Kpa10,
    pub tps_x100: u16,
    pub knock_intensity_x100: u16,
    pub maf_x100: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub baro_kpa10: Kpa10,
    pub vbatt_mv: u16,
    pub maf_valid: bool,
    pub lambda_valid: bool,
    pub lambda_measured: Lambda100,
    pub requested_open_loop: bool,
    pub baro_valid: bool,
    pub sync: SyncState,
    pub mode: FuelEngineMode,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub fuel_cut_request: bool,
    pub spark_cut_request: bool,
    pub target_afr_override_x100: FuelAfrOverride,
}

/// Diagnostic observations from the selected fuel strategy.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FuelStartupWindowMode {
    #[default]
    Inactive = 0,
    Milliseconds = 1,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FuelAfterstartWindowMode {
    #[default]
    Inactive = 0,
    Milliseconds = 1,
    Cycles = 2,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FuelWarmupTemperatureMode {
    #[default]
    Inactive = 0,
    ColdClamp = 1,
    Interpolating = 2,
    HotClamp = 3,
    NeutralFallback = 4,
}

/// Diagnostic observations from the selected fuel strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelObservations {
    pub ve_pct_x100: Option<u16>,
    pub target_afr_x100: Option<u16>,
    pub pw_base_us: u16,
    pub pw_air_us: Option<u16>,
    pub pw_corr_us: u16,
    pub warmup_active: bool,
    pub warmup_correction_x100: u16,
    pub warmup_temperature_mode: FuelWarmupTemperatureMode,
    pub startup_active: bool,
    pub startup_window_remaining: u16,
    pub startup_window_mode: FuelStartupWindowMode,
    pub afterstart_active: bool,
    pub afterstart_window_remaining: u16,
    pub afterstart_window_mode: FuelAfterstartWindowMode,
    pub lambda_ae_freeze_active: bool,
    pub ae_pulse_us: u16,
    pub ae_decay_steps_remaining: u16,
    pub lambda_correction_x1000: u16,
    pub lambda_integrator_acc: i32,
    pub lambda_integrator_min_acc: i32,
    pub lambda_integrator_max_acc: i32,
    pub lambda_integrator_frozen: bool,
    pub idle_active: bool,
    pub idle_duty_x1000: u16,
    pub idle_integrator_acc: i32,
    pub idle_integrator_min_acc: i32,
    pub idle_integrator_max_acc: i32,
    pub idle_integrator_frozen: bool,
    pub advance_deg10_trim: i16,
    pub strategy_is_direct_pw: bool,
}

/// Scheduler-facing bounded fuel intent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelIntent {
    pub pulse_width_us: PulseWidthUs,
    pub target_angle_deg10: Degrees10,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub observations: FuelObservations,
}
