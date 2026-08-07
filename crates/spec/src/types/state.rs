use super::calibration::DiagnosticCode;
use super::events::EventBatch;
use super::units::{Kpa10, RatioX1000, TempC10};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SyncState {
    #[default]
    Unsynced,
    Synced,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum EngineMode {
    #[default]
    Off,
    Cranking,
    Running,
    Shutdown,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct KnockState {
    pub retard_deg10: i16,
    pub recovery_counter: u16,
    pub detected: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MathState {
    pub last_valid_map_kpa10: Kpa10,
    pub last_valid_load_kpa10: Kpa10,
    pub last_valid_clt_c10: TempC10,
    pub last_valid_iat_c10: TempC10,
    pub last_valid_baro_kpa10: Kpa10,
    pub trim_ratio_x1000: RatioX1000,
}

impl Default for MathState {
    fn default() -> Self {
        Self {
            last_valid_map_kpa10: Kpa10::default(),
            last_valid_load_kpa10: Kpa10::default(),
            last_valid_clt_c10: TempC10::default(),
            last_valid_iat_c10: TempC10::default(),
            last_valid_baro_kpa10: Kpa10::default(),
            trim_ratio_x1000: RatioX1000::new(1000),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SchedulerState {
    pub pending: EventBatch,
    pub last_cycle_epoch: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct AeState {
    pub active: bool,
    pub pulse_us: u32,
    pub decay_steps_remaining: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PiIntegratorState {
    pub acc: i32,
    pub min_acc: i32,
    pub max_acc: i32,
    pub frozen: bool,
}

impl PiIntegratorState {
    pub const fn zero() -> Self {
        Self {
            acc: 0,
            min_acc: -2000,
            max_acc: 2000,
            frozen: false,
        }
    }
}

impl Default for PiIntegratorState {
    fn default() -> Self {
        Self::zero()
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct DiagState {
    pub current: DiagnosticCode,
    pub unsynced: bool,
    pub fuel_cut_active: bool,
    pub spark_cut_active: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LogicalState {
    pub math: MathState,
    pub scheduler: SchedulerState,
    pub ae: AeState,
    pub idle_integrator_state: PiIntegratorState,
    pub idle_timing_integrator_state: PiIntegratorState,
    pub idle_duty_x1000: u16,
    pub lambda_integrator_state: PiIntegratorState,
    pub lambda_correction_x1000: u16,
    pub dfco_active: bool,
    pub dfco_qualify_counter: u16,
    pub rev_soft_active: bool,
    pub rev_hard_active: bool,
    pub launch_active: bool,
    pub launch_cut_cycle_count: u16,
    pub flat_shift_active: bool,
    pub flat_shift_cut_cycle_count: u16,
    pub knock_state: KnockState,
    pub safety_latched: bool,
    pub knock_intensity_x100: u16,
    pub sensor_plausibility_state: crate::sensors::plausibility::SensorPlausibilityState,
    pub sensor_slew_state: crate::sensors::slew::SensorSlewState,
    pub diag: DiagState,
}

impl Default for LogicalState {
    fn default() -> Self {
        Self {
            math: MathState::default(),
            scheduler: SchedulerState::default(),
            ae: AeState::default(),
            idle_integrator_state: PiIntegratorState::zero(),
            idle_timing_integrator_state: PiIntegratorState::zero(),
            idle_duty_x1000: 0,
            lambda_integrator_state: PiIntegratorState::zero(),
            lambda_correction_x1000: 1000,
            dfco_active: false,
            dfco_qualify_counter: 0,
            rev_soft_active: false,
            rev_hard_active: false,
            launch_active: false,
            launch_cut_cycle_count: 0,
            flat_shift_active: false,
            flat_shift_cut_cycle_count: 0,
            knock_state: KnockState::default(),
            safety_latched: false,
            knock_intensity_x100: 0,
            sensor_plausibility_state:
                crate::sensors::plausibility::SensorPlausibilityState::default(),
            sensor_slew_state: crate::sensors::slew::SensorSlewState::default(),
            diag: DiagState::default(),
        }
    }
}
