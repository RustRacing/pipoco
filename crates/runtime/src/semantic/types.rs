use crate::{support::RuntimeAdapterContract, RuntimeFuelStrategy};
use ecu_calibration::{FuelRuntimeTune, FUEL_RUNTIME_LOAD_BINS, FUEL_RUNTIME_RPM_BINS};
use ecu_domain::{Kpa10, Micros, Rpm, SyncState};

// ---------------------------------------------------------------------------
// v9 Runtime Semantic Fuel/Cut Evaluator Types
// ---------------------------------------------------------------------------

/// Fixed table length for v9 semantic types.
pub const RUNTIME_SEMANTIC_TABLE_LEN: usize = 16;

/// An axis of a runtime semantic table or curve with up to 16 entries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticAxis16 {
    /// Number of valid entries (must be in 2..=16 for valid interpolation).
    pub len: u8,
    /// Axis values. Only the first `len` entries are valid.
    pub values: [u16; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// A 2D table of u16 values used for VE and AFR lookups.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTable2dU16 {
    /// RPM axis values.
    pub rpm_axis: RuntimeSemanticAxis16,
    /// Load axis values.
    pub load_axis: RuntimeSemanticAxis16,
    /// Table values indexed by [load_index][rpm_index].
    pub values: [[u16; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// A 1D curve of u16 values used for single-axis corrections.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticCurve16U16 {
    /// Axis values.
    pub axis: RuntimeSemanticAxis16,
    /// Curve values indexed by axis position.
    pub values: [u16; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// AFR override for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticAfrOverride {
    /// No override — use table lookup.
    None,
    /// Override with this fixed AFR × 100 value.
    Some(u16),
}

/// Engine operating mode for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticEngineMode {
    Off,
    Cranking,
    Running,
    Shutdown,
}

/// Full calibration surface required by the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticCalibration {
    pub ve_table: RuntimeSemanticTable2dU16,
    pub afr_target_table: RuntimeSemanticTable2dU16,
    pub deadtime_table_us: RuntimeSemanticTable2dU16,
    pub clt_corr_curve: RuntimeSemanticCurve16U16,
    pub iat_corr_curve: RuntimeSemanticCurve16U16,
    pub baro_corr_curve: RuntimeSemanticCurve16U16,
    pub vbat_corr_curve: RuntimeSemanticCurve16U16,
    pub cranking_curve: RuntimeSemanticCurve16U16,
    pub afterstart_table: RuntimeSemanticTable2dU16,
    pub warmup_curve: RuntimeSemanticCurve16U16,
    pub ae_tps_threshold_curve: RuntimeSemanticCurve16U16,
    pub ae_map_threshold_curve: RuntimeSemanticCurve16U16,
    pub ae_shot_curve_us: RuntimeSemanticCurve16U16,
    pub ae_decay_steps_curve: RuntimeSemanticCurve16U16,
    pub ae_decay_ratio_curve_x1000: RuntimeSemanticCurve16U16,
    pub required_fuel_us: u32,
    pub pref_kpa10: u16,
    pub stoich_afr_x100: u16,
    pub pw_max_us: u32,
    pub afterstart_window_cycles: u16,
    pub dfco_entry_rpm: u16,
    pub dfco_exit_rpm: u16,
    pub dfco_entry_tps_x100: u16,
    pub dfco_exit_tps_x100: u16,
    pub dfco_entry_map_kpa10: u16,
    pub dfco_delay_cycles: u16,
    pub soft_rev_rpm: u16,
    pub hard_rev_rpm: u16,
    pub rev_hysteresis_rpm: u16,
    pub soft_retard_max_deg10: u16,
    pub launch_rpm_limit: u16,
    pub launch_cut_cycles: u16,
    pub flat_shift_rpm_min: u16,
    pub flat_shift_cut_cycles: u16,
    pub knock_threshold_x100: u16,
    pub knock_retard_step_deg10: u16,
    pub knock_retard_max_deg10: u16,
    pub knock_recovery_step_deg10: u16,
    pub knock_recovery_delay_cycles: u16,
    /// Proportional gain for lambda closed-loop correction (x1000).
    pub lambda_kp_x1000: u16,
    /// Integral gain for lambda closed-loop correction (x1000).
    pub lambda_ki_x1000: u16,
}

pub fn runtime_semantic_calibration_from_fuel_tune(
    tune: &FuelRuntimeTune,
) -> RuntimeSemanticCalibration {
    let rpm_axis = RuntimeSemanticAxis16 {
        len: FUEL_RUNTIME_RPM_BINS.len() as u8,
        values: FUEL_RUNTIME_RPM_BINS,
    };
    let load_axis = RuntimeSemanticAxis16 {
        len: FUEL_RUNTIME_LOAD_BINS.len() as u8,
        values: FUEL_RUNTIME_LOAD_BINS,
    };
    let flat100 = RuntimeSemanticCurve16U16 {
        axis: RuntimeSemanticAxis16 {
            len: 2,
            values: [0, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        values: [100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    let flat1000 = RuntimeSemanticCurve16U16 {
        axis: RuntimeSemanticAxis16 {
            len: 2,
            values: [0, 2000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        values: [1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    RuntimeSemanticCalibration {
        ve_table: RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values: tune.ve_table,
        },
        afr_target_table: RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values: tune.afr_table,
        },
        deadtime_table_us: RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values: [[tune.injector_deadtime_us; 16]; 16],
        },
        clt_corr_curve: flat100,
        iat_corr_curve: flat100,
        baro_corr_curve: flat100,
        vbat_corr_curve: flat100,
        cranking_curve: flat100,
        afterstart_table: RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values: [[100; 16]; 16],
        },
        warmup_curve: flat100,
        ae_tps_threshold_curve: flat100,
        ae_map_threshold_curve: flat100,
        ae_shot_curve_us: RuntimeSemanticCurve16U16 {
            axis: RuntimeSemanticAxis16 {
                len: 2,
                values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            },
            values: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        ae_decay_steps_curve: RuntimeSemanticCurve16U16 {
            axis: RuntimeSemanticAxis16 {
                len: 2,
                values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            },
            values: [1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        ae_decay_ratio_curve_x1000: flat1000,
        required_fuel_us: tune.required_fuel_us as u32,
        pref_kpa10: 1013,
        stoich_afr_x100: 1470,
        pw_max_us: 20_000,
        afterstart_window_cycles: 0,
        dfco_entry_rpm: 65_000,
        dfco_exit_rpm: 64_000,
        dfco_entry_tps_x100: 0,
        dfco_exit_tps_x100: 0,
        dfco_entry_map_kpa10: 0,
        dfco_delay_cycles: 0,
        soft_rev_rpm: 20_000,
        hard_rev_rpm: 20_500,
        rev_hysteresis_rpm: 100,
        soft_retard_max_deg10: 0,
        launch_rpm_limit: 0,
        launch_cut_cycles: 0,
        flat_shift_rpm_min: 0,
        flat_shift_cut_cycles: 0,
        knock_threshold_x100: 10_000,
        knock_retard_step_deg10: 0,
        knock_retard_max_deg10: 0,
        knock_recovery_step_deg10: 0,
        knock_recovery_delay_cycles: 0,
        lambda_kp_x1000: 0,
        lambda_ki_x1000: 0,
    }
}

pub fn runtime_fuel_strategy_from_fuel_tune(tune: &FuelRuntimeTune) -> RuntimeFuelStrategy {
    let calibration = runtime_semantic_calibration_from_fuel_tune(tune);
    let state = RuntimeSemanticState::default();
    match tune.ve_load_source {
        1 => RuntimeFuelStrategy::AlphaN { calibration, state },
        _ => RuntimeFuelStrategy::SpeedDensityVe { calibration, state },
    }
}

/// Input snapshot for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticInputSnapshot {
    pub t_us: Micros,
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub load_kpa10: Kpa10,
    pub tps_x100: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub baro_kpa10: Kpa10,
    pub vbatt_mv: u16,
    pub knock_intensity_x100: u16,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub sync: SyncState,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub mode: RuntimeSemanticEngineMode,
    pub target_afr_override_x100: RuntimeSemanticAfrOverride,
}

// ---------------------------------------------------------------------------
// v11 Runtime Semantic Lambda PI Types
// ---------------------------------------------------------------------------

/// PI integrator state for the semantic lambda closed-loop evaluator.
/// This is the runtime-owned equivalent of `ecu_spec::PiIntegratorState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticPiIntegratorState {
    /// Accumulator value.
    pub acc: i32,
    /// Minimum accumulator value.
    pub min_acc: i32,
    /// Maximum accumulator value.
    pub max_acc: i32,
    /// True if the integrator is frozen (not accumulating).
    pub frozen: bool,
}

impl Default for RuntimeSemanticPiIntegratorState {
    fn default() -> Self {
        Self {
            acc: 0,
            min_acc: RUNTIME_SEMANTIC_LAMBDA_MIN_ACC,
            max_acc: RUNTIME_SEMANTIC_LAMBDA_MAX_ACC,
            frozen: false,
        }
    }
}

/// Mutable state fragment for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeSemanticState {
    pub afterstart_cycle_count: u32,
    pub last_valid_load_kpa10: u16,
    pub last_valid_map_kpa10: u16,
    pub ae_active: bool,
    pub ae_pulse_us: u32,
    pub ae_decay_steps_remaining: u16,
    pub lambda_integrator_acc: i32,
    pub dfco_active: bool,
    pub dfco_qualify_counter: u16,
    pub rev_soft_active: bool,
    pub rev_hard_active: bool,
    pub safety_latched: bool,
    pub sensor_plausibility_latched: bool,
    pub launch_active: bool,
    pub launch_cut_cycle_count: u16,
    pub flat_shift_active: bool,
    pub flat_shift_cut_cycle_count: u16,
    pub knock_retard_deg10: i16,
    pub knock_recovery_counter: u16,
}

/// Observable fuel and cut outputs from the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticFuelObservations {
    pub ve_pct_x100: u16,
    pub target_afr_x100: u16,
    pub pw_base_us: u32,
    pub pw_air_us: u32,
    pub pw_corr_us: u32,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    /// Lambda closed-loop correction factor (x1000, e.g. 1000 = 1.000).
    pub lambda_correction_x1000: u16,
    /// Lambda PI integrator state after this step.
    pub lambda_integrator_state: RuntimeSemanticPiIntegratorState,
    /// Final ignition advance trim in deg10 from runtime semantic cut/knock logic.
    pub advance_deg10_trim: i16,
}

/// Errors from the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticFuelError {
    AxisLenInvalid,
    AxisNotStrictlyIncreasing,
    ZeroPrefKpa,
    ZeroTargetAfr,
    CorrectionRangeInvalid,
}

// ---------------------------------------------------------------------------
// v10 Runtime Semantic Schedule Types — runtime-owned, no_std, Verus-friendly
// ---------------------------------------------------------------------------

/// A 2D table of i16 values used for spark advance lookups.
/// Indexed by [load_index][rpm_index].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTable2dI16 {
    /// RPM axis values.
    pub rpm_axis: RuntimeSemanticAxis16,
    /// Load axis values.
    pub load_axis: RuntimeSemanticAxis16,
    /// Table values indexed by [load_index][rpm_index].
    pub values: [[i16; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// A 2D table of u32 values used for dwell time lookups.
/// Indexed by [load_index][rpm_index].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTable2dU32 {
    /// RPM axis values.
    pub rpm_axis: RuntimeSemanticAxis16,
    /// Load axis values.
    pub load_axis: RuntimeSemanticAxis16,
    /// Table values indexed by [load_index][rpm_index].
    pub values: [[u32; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// Cylinder phase array for schedule events.
/// Valid count is 1..=8. Every live phase value must be < 7200.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticCylinderArrayU16 {
    /// Number of active cylinders (1..=8).
    pub count: u8,
    /// Phase values in crank-angle units (0.1 degrees). Only first `count` are valid.
    pub values: [u16; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// Injection angle specification mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticInjectionAngleMode {
    /// Start-of-injection mode.
    StartOfInjection,
    /// End-of-injection mode.
    EndOfInjection,
}

/// Full calibration surface required by the v10 semantic schedule evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleCalibration {
    /// Spark advance table in deg10 units.
    pub spark_advance_table_deg10: RuntimeSemanticTable2dI16,
    /// Dwell time table in microseconds.
    pub dwell_table_us: RuntimeSemanticTable2dU32,
    /// Injection target table in deg10 units.
    pub injection_target_table_deg10: RuntimeSemanticTable2dU16,
    /// Injection angle mode (SOI or EOI).
    pub injection_angle_mode: RuntimeSemanticInjectionAngleMode,
    /// Cylinder phase array in deg10 units.
    pub cylinder_phase_deg10: RuntimeSemanticCylinderArrayU16,
}

/// Diagnostic codes from the schedule semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticScheduleDiagnostic {
    /// No diagnostic active.
    None,
    /// Fuel cut is active.
    FuelCutActive,
    /// Spark cut is active.
    SparkCutActive,
    /// Scheduler is unsynchronized.
    Unsynced,
    /// Calibration data is invalid.
    CalibrationInvalid,
}

/// Kind of schedule event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticScheduleEventKind {
    /// Injector opens.
    InjectionOpen,
    /// Injector closes.
    InjectionClose,
    /// Coil begins charging.
    CoilChargeStart,
    /// Coil fires (spark event).
    CoilFire,
}

/// A single scheduled event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleEvent {
    /// Kind of event.
    pub kind: RuntimeSemanticScheduleEventKind,
    /// Cylinder index (0-based).
    pub cylinder: u8,
    /// Event angle in deg10 units (0..7199).
    pub angle_deg10: u16,
}

/// A batch of scheduled events (up to 64).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleEventBatch {
    /// Number of valid events in the batch.
    pub len: u8,
    /// Event array (up to 64 events).
    pub events: [RuntimeSemanticScheduleEvent; 64],
}

/// Output observations from the schedule semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleObservations {
    /// Injection target angle in deg10 units.
    pub injection_target_deg10: u16,
    /// Spark advance in deg10 units (signed).
    pub spark_advance_deg10: i16,
    /// Dwell time in microseconds.
    pub dwell_us: u32,
    /// Injection duration in deg10 units.
    pub injection_duration_deg10: u16,
    /// Dwell duration in deg10 units.
    pub dwell_duration_deg10: u16,
    /// Start-of-injection angles per cylinder in deg10.
    pub soi_deg10: RuntimeSemanticCylinderArrayU16,
    /// End-of-injection angles per cylinder in deg10.
    pub eoi_deg10: RuntimeSemanticCylinderArrayU16,
    /// Spark event angles per cylinder in deg10.
    pub spark_deg10: RuntimeSemanticCylinderArrayU16,
    /// Dwell start angles per cylinder in deg10.
    pub dwell_start_deg10: RuntimeSemanticCylinderArrayU16,
    /// Scheduled event batch.
    pub events: RuntimeSemanticScheduleEventBatch,
    /// Schedule diagnostic.
    pub diagnostic: RuntimeSemanticScheduleDiagnostic,
}

/// Errors from the v10 semantic schedule evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticScheduleError {
    /// Table axis length is invalid (< 2 for interpolation).
    AxisLenInvalid,
    /// Table axis values are not strictly increasing.
    AxisNotStrictlyIncreasing,
    /// Cylinder count is out of range.
    CylinderCountInvalid,
    /// A cylinder phase value is >= 7200.
    CylinderPhaseInvalid,
    /// Computed duration overflowed the target type.
    DurationOverflow,
    /// Event batch capacity (64) was exceeded.
    EventBatchFull,
}

// ---------------------------------------------------------------------------
// v11 Runtime Semantic Lambda PI Constants
// ---------------------------------------------------------------------------

/// Lambda closed-loop deadband in x1000 units (error whose absolute value is
/// within this is treated as zero).
pub(super) const RUNTIME_SEMANTIC_LAMBDA_DEADBAND_X1000: i32 = 10;
/// Minimum PI integrator accumulator value.
pub(super) const RUNTIME_SEMANTIC_LAMBDA_MIN_ACC: i32 = -2000;
/// Maximum PI integrator accumulator value.
pub(super) const RUNTIME_SEMANTIC_LAMBDA_MAX_ACC: i32 = 2000;
/// Minimum lambda correction factor in x1000 units (0.750).
pub(crate) const RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000: u16 = 750;
/// Maximum lambda correction factor in x1000 units (1.250).
pub(crate) const RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000: u16 = 1250;
/// Fixed lambda error for the v11 frozen oracle path (always zero).
pub(super) const RUNTIME_SEMANTIC_LAMBDA_ERROR_X1000: i32 = 0;

/// Field-by-field conformance status for runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeConformanceStatus {
    Covered,
    AdapterContract(RuntimeAdapterContract),
}
