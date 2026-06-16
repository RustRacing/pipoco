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
    pub lambda_valid: bool,
    pub lambda_measured: Lambda100,
    pub sync: SyncState,
    pub mode: FuelEngineMode,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub fuel_cut_request: bool,
    pub spark_cut_request: bool,
    pub target_afr_override_x100: FuelAfrOverride,
}

/// Diagnostic observations from the selected fuel strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelObservations {
    pub ve_pct_x100: Option<u16>,
    pub target_afr_x100: Option<u16>,
    pub pw_base_us: u16,
    pub pw_corr_us: u16,
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
