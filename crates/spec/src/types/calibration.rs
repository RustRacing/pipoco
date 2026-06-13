use super::collections::{Curve16, CylinderArrayU16, SignedCurve16, Table2D16};
use super::units::{AfrX100, Kpa10, Rpm};

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
pub enum DiagnosticCode {
    #[default]
    None,
    FuelCutActive,
    SparkCutActive,
    Unsynced,
    SensorPlausibilityFault,
    CalibrationInvalid,
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
