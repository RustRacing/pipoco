use super::calibration::AfrOverride;
use super::state::{EngineMode, SyncState};
use super::units::{Kpa10, Micros, Millivolts, Rpm, TempC10};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct InputSnapshot {
    pub t_us: Micros,
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub load_kpa10: Kpa10,
    pub tps_x100: u16,
    pub clt_c10: TempC10,
    pub iat_c10: TempC10,
    pub baro_kpa10: Kpa10,
    pub vbatt_mv: Millivolts,
    pub knock_intensity_x100: u16,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub sync: SyncState,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub mode: EngineMode,
    pub target_afr_override_x100: AfrOverride,
}
