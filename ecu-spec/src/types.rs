#![allow(dead_code)]

pub const MAX_RPM_BINS: usize = 16;
pub const MAX_LOAD_BINS: usize = 16;
pub const MAX_CURVE_POINTS: usize = 16;
pub const MAX_CYLINDERS: usize = 8;
pub const MAX_CYLINDER_STORAGE: usize = 16;
pub const MAX_EVENTS_PER_STEP: usize = 64;
pub const MAX_PENDING_EVENTS: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rpm(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Micros(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Kpa10(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TempC10(pub i16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Millivolts(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Degrees10(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignedDegrees10(pub i16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PulseWidthUs(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AfrX100(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VePctX100(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RatioX1000(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderId(pub u8);

macro_rules! impl_newtype_api {
    ($name:ident, $raw:ty) => {
        impl $name {
            pub const fn new(value: $raw) -> Self {
                Self(value)
            }

            pub const fn get(self) -> $raw {
                self.0
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self(0)
            }
        }
    };
}

impl_newtype_api!(Rpm, u16);
impl_newtype_api!(Micros, u32);
impl_newtype_api!(Kpa10, u16);
impl_newtype_api!(TempC10, i16);
impl_newtype_api!(Millivolts, u16);
impl_newtype_api!(Degrees10, u16);
impl_newtype_api!(SignedDegrees10, i16);
impl_newtype_api!(PulseWidthUs, u32);
impl_newtype_api!(AfrX100, u16);
impl_newtype_api!(VePctX100, u16);
impl_newtype_api!(RatioX1000, u16);
impl_newtype_api!(CylinderId, u8);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Axis16 {
    pub len: u8,
    pub values: [u16; MAX_CURVE_POINTS],
}

impl Default for Axis16 {
    fn default() -> Self {
        Self {
            len: 0,
            values: [0; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Curve16 {
    pub axis: Axis16,
    pub values: [u16; MAX_CURVE_POINTS],
}

impl Default for Curve16 {
    fn default() -> Self {
        Self {
            axis: Axis16::default(),
            values: [0; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignedCurve16 {
    pub axis: Axis16,
    pub values: [i16; MAX_CURVE_POINTS],
}

impl Default for SignedCurve16 {
    fn default() -> Self {
        Self {
            axis: Axis16::default(),
            values: [0; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Table2D16<T> {
    pub rpm_axis: Axis16,
    pub load_axis: Axis16,
    pub values: [[T; MAX_CURVE_POINTS]; MAX_CURVE_POINTS],
}

impl<T: Copy + Default> Default for Table2D16<T> {
    fn default() -> Self {
        Self {
            rpm_axis: Axis16::default(),
            load_axis: Axis16::default(),
            values: [[T::default(); MAX_CURVE_POINTS]; MAX_CURVE_POINTS],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderArrayU16 {
    pub count: u8,
    pub values: [u16; MAX_CYLINDER_STORAGE],
}

impl Default for CylinderArrayU16 {
    fn default() -> Self {
        Self {
            count: 0,
            values: [0; MAX_CYLINDER_STORAGE],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderArrayI16 {
    pub count: u8,
    pub values: [i16; MAX_CYLINDER_STORAGE],
}

impl Default for CylinderArrayI16 {
    fn default() -> Self {
        Self {
            count: 0,
            values: [0; MAX_CYLINDER_STORAGE],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventKind {
    InjectionOpen,
    InjectionClose,
    CoilChargeStart,
    CoilFire,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SemanticEvent {
    pub kind: EventKind,
    pub cylinder: CylinderId,
    pub angle_deg10: Degrees10,
}

impl Default for SemanticEvent {
    fn default() -> Self {
        Self {
            kind: EventKind::InjectionOpen,
            cylinder: CylinderId::default(),
            angle_deg10: Degrees10::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventBatch {
    pub len: u8,
    pub events: [SemanticEvent; MAX_EVENTS_PER_STEP],
}

impl Default for EventBatch {
    fn default() -> Self {
        Self {
            len: 0,
            events: [SemanticEvent::default(); MAX_EVENTS_PER_STEP],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventBatchFull {
    Full,
}

impl EventBatch {
    pub fn push(&mut self, event: SemanticEvent) -> Result<(), EventBatchFull> {
        let len = self.len as usize;
        if len >= MAX_EVENTS_PER_STEP {
            return Err(EventBatchFull::Full);
        }
        self.events[len] = event;
        self.len += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_batch_push_overflow_is_observable_and_len_is_stable() {
        let mut batch = EventBatch::default();
        let event = SemanticEvent::default();
        for _ in 0..MAX_EVENTS_PER_STEP {
            assert_eq!(batch.push(event), Ok(()));
        }
        let len_before = batch.len;
        assert_eq!(len_before as usize, MAX_EVENTS_PER_STEP);
        assert_eq!(batch.push(event), Err(EventBatchFull::Full));
        assert_eq!(batch.len, len_before);
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FuelModel {
    SpeedDensityRequiredFuel,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrimPolicy {
    Identity,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PwMaxPolicy {
    Fixed,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum O2SensorMode {
    #[default]
    WidebandLinear,
    NarrowbandSwitch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InjectionAngleMode {
    StartOfInjection,
    EndOfInjection,
}

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
pub enum DiagnosticCode {
    #[default]
    None,
    FuelCutActive,
    SparkCutActive,
    Unsynced,
    SensorPlausibilityFault,
    CalibrationInvalid,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct KnockState {
    pub retard_deg10: i16,
    pub recovery_counter: u16,
    pub detected: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Calibration {
    pub fuel_model: FuelModel,
    pub ve_table: Table2D16<u16>,
    pub afr_target_table: Table2D16<u16>,
    pub spark_advance_table_deg10: Table2D16<i16>,
    pub dwell_table_us: Table2D16<u32>,
    pub injection_target_table_deg10: Table2D16<u16>,
    pub deadtime_table_us: Table2D16<u16>,
    pub cranking_curve: Curve16,
    pub afterstart_table: Table2D16<u16>,
    pub afterstart_window_cycles: u16,
    pub warmup_curve: Curve16,
    pub ae_tps_threshold_curve: Curve16,
    pub ae_map_threshold_curve: Curve16,
    pub ae_shot_curve_us: Curve16,
    pub ae_decay_steps_curve: Curve16,
    pub ae_decay_ratio_curve_x1000: Curve16,
    pub dfco_entry_rpm: Rpm,
    pub dfco_exit_rpm: Rpm,
    pub dfco_entry_tps_x100: u16,
    pub dfco_exit_tps_x100: u16,
    pub dfco_entry_map_kpa10: Kpa10,
    pub dfco_delay_cycles: u16,
    pub soft_rev_rpm: Rpm,
    pub hard_rev_rpm: Rpm,
    pub rev_hysteresis_rpm: Rpm,
    pub soft_retard_max_deg10: u16,
    pub launch_rpm_limit: Rpm,
    pub launch_cut_cycles: u16,
    pub flat_shift_rpm_min: Rpm,
    pub flat_shift_cut_cycles: u16,
    pub knock_threshold_x100: u16,
    pub knock_retard_step_deg10: u16,
    pub knock_retard_max_deg10: u16,
    pub knock_recovery_step_deg10: u16,
    pub knock_recovery_delay_cycles: u16,
    pub tps_adc_min_counts: u16,
    pub tps_adc_max_counts: u16,
    pub idle_target_rpm: Rpm,
    pub idle_base_duty_x1000: u16,
    pub idle_kp_x1000: u16,
    pub idle_ki_x1000: u16,
    pub idle_timing_enabled: bool,
    pub idle_timing_pid_enabled: bool,
    pub idle_timing_rpm_max: Rpm,
    pub idle_timing_tps_max_x100: u16,
    pub idle_advance_curve_deg10: SignedCurve16,
    pub idle_timing_kp_x1000: u16,
    pub idle_timing_ki_x1000: u16,
    pub idle_timing_min_trim_deg10: i16,
    pub idle_timing_max_trim_deg10: i16,
    pub clt_timing_corr_curve_deg10: SignedCurve16,
    pub iat_timing_corr_curve_deg10: SignedCurve16,
    pub lambda_kp_x1000: u16,
    pub lambda_ki_x1000: u16,
    pub o2_sensor_mode: O2SensorMode,
    pub o2_wideband_afr_min_x100: u16,
    pub o2_wideband_afr_max_x100: u16,
    pub o2_narrowband_threshold_counts: u16,
    pub o2_narrowband_hysteresis_counts: u16,
    pub o2_narrowband_rich_afr_x100: u16,
    pub o2_narrowband_lean_afr_x100: u16,
    pub clt_corr_curve: Curve16,
    pub iat_corr_curve: Curve16,
    pub baro_corr_curve: Curve16,
    pub vbat_corr_curve: Curve16,
    pub required_fuel_us: u32,
    pub pref_kpa10: u16,
    pub stoich_afr_x100: u16,
    pub trim_policy: TrimPolicy,
    pub pw_max_policy: PwMaxPolicy,
    pub pw_max_us: u32,
    pub injection_angle_mode: InjectionAngleMode,
    pub cylinder_phase_deg10: CylinderArrayU16,
}

impl Default for Calibration {
    fn default() -> Self {
        Self {
            fuel_model: FuelModel::SpeedDensityRequiredFuel,
            ve_table: Table2D16::default(),
            afr_target_table: Table2D16::default(),
            spark_advance_table_deg10: Table2D16::default(),
            dwell_table_us: Table2D16::default(),
            injection_target_table_deg10: Table2D16::default(),
            deadtime_table_us: Table2D16::default(),
            cranking_curve: Curve16::default(),
            afterstart_table: Table2D16::default(),
            afterstart_window_cycles: 0,
            warmup_curve: Curve16::default(),
            ae_tps_threshold_curve: Curve16::default(),
            ae_map_threshold_curve: Curve16::default(),
            ae_shot_curve_us: Curve16::default(),
            ae_decay_steps_curve: Curve16::default(),
            ae_decay_ratio_curve_x1000: Curve16::default(),
            dfco_entry_rpm: Rpm::default(),
            dfco_exit_rpm: Rpm::default(),
            dfco_entry_tps_x100: 0,
            dfco_exit_tps_x100: 0,
            dfco_entry_map_kpa10: Kpa10::default(),
            dfco_delay_cycles: 0,
            soft_rev_rpm: Rpm::default(),
            hard_rev_rpm: Rpm::default(),
            rev_hysteresis_rpm: Rpm::default(),
            soft_retard_max_deg10: 0,
            launch_rpm_limit: Rpm::default(),
            launch_cut_cycles: 0,
            flat_shift_rpm_min: Rpm::default(),
            flat_shift_cut_cycles: 0,
            knock_threshold_x100: 10000,
            knock_retard_step_deg10: 0,
            knock_retard_max_deg10: 0,
            knock_recovery_step_deg10: 0,
            knock_recovery_delay_cycles: 0,
            tps_adc_min_counts: 0,
            tps_adc_max_counts: 4095,
            idle_target_rpm: Rpm::default(),
            idle_base_duty_x1000: 0,
            idle_kp_x1000: 0,
            idle_ki_x1000: 0,
            idle_timing_enabled: false,
            idle_timing_pid_enabled: false,
            idle_timing_rpm_max: Rpm::default(),
            idle_timing_tps_max_x100: 0,
            idle_advance_curve_deg10: SignedCurve16::default(),
            idle_timing_kp_x1000: 0,
            idle_timing_ki_x1000: 0,
            idle_timing_min_trim_deg10: 0,
            idle_timing_max_trim_deg10: 0,
            clt_timing_corr_curve_deg10: SignedCurve16::default(),
            iat_timing_corr_curve_deg10: SignedCurve16::default(),
            lambda_kp_x1000: 0,
            lambda_ki_x1000: 0,
            o2_sensor_mode: O2SensorMode::WidebandLinear,
            o2_wideband_afr_min_x100: 500,
            o2_wideband_afr_max_x100: 3000,
            o2_narrowband_threshold_counts: 2048,
            o2_narrowband_hysteresis_counts: 64,
            o2_narrowband_rich_afr_x100: 1400,
            o2_narrowband_lean_afr_x100: 1550,
            clt_corr_curve: Curve16::default(),
            iat_corr_curve: Curve16::default(),
            baro_corr_curve: Curve16::default(),
            vbat_corr_curve: Curve16::default(),
            required_fuel_us: 0,
            pref_kpa10: 0,
            stoich_afr_x100: 0,
            trim_policy: TrimPolicy::Identity,
            pw_max_policy: PwMaxPolicy::Fixed,
            pw_max_us: 0,
            injection_angle_mode: InjectionAngleMode::EndOfInjection,
            cylinder_phase_deg10: CylinderArrayU16::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum AfrOverride {
    #[default]
    None,
    Some(AfrX100),
}

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
            trim_ratio_x1000: RatioX1000(1000),
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

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ObservableOutput {
    pub ve_pct_x100: VePctX100,
    pub target_afr_x100: AfrX100,
    pub pw_base_us: PulseWidthUs,
    pub pw_air_us: PulseWidthUs,
    pub pw_corr_us: PulseWidthUs,
    pub lambda_correction_x1000: u16,
    pub idle_duty_x1000: u16,
    pub torque_request_x1000: u16,
    pub torque_allowed_x1000: u16,
    pub torque_actuated_x1000: u16,
    pub cut_reason_code: u8,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub advance_deg10_trim: i16,
    pub knock_intensity_x100: u16,
    pub spark_advance_deg10: SignedDegrees10,
    pub dwell_us: PulseWidthUs,
    pub soi_deg10: CylinderArrayU16,
    pub eoi_deg10: CylinderArrayU16,
    pub spark_deg10: CylinderArrayU16,
    pub dwell_start_deg10: CylinderArrayU16,
    pub events: EventBatch,
    pub diagnostic: DiagnosticCode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct StepResult {
    pub next_state: LogicalState,
    pub output: ObservableOutput,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ValidatedCalibration(pub Calibration);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValidationError {
    AxisTooShort,
    AxisNotStrictlyIncreasing,
    TableDimensionMismatch,
    CurveDimensionMismatch,
    CylinderCountZero,
    CylinderCountTooLarge,
    AngleOutOfRange,
    RequiredFuelZero,
    ReferencePressureZero,
    PwMaxZero,
    CorrectionBelowZero,
    CorrectionAboveLimit,
    VeOutOfRange,
    AfrOutOfRange,
    DwellOutOfRange,
    SparkAdvanceOutOfRange,
    InjectionTargetOutOfRange,
    TargetAfrOverrideOutOfRange,
    DfcoConfigInvalid,
    RevLimitConfigInvalid,
    KnockConfigInvalid,
    LaunchConfigInvalid,
    FlatShiftConfigInvalid,
    IdleConfigInvalid,
    TpsConfigInvalid,
    O2ConfigInvalid,
}
