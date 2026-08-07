#![no_std]
#![forbid(unsafe_code)]

//! Semantic oracle crate for the formal methods plan.
//!
//! This crate exposes the frozen v1 semantic types. Later stories will add the
//! numeric helpers, validation logic, table interpolation, fuel semantics, and
//! proof-facing modules.

#[cfg(kani)]
extern crate kani;

mod ae;
mod afterstart;
mod air_load;
mod arbiter;
mod baro;
mod cranking;
mod deadtime;
mod dfco;
mod fault;
mod flatshift;
mod fuel;
mod idle;
mod idle_timing;
mod interp;
mod knock;
mod lambda_cl;
mod launch;
mod numeric;
mod persist_spec;
mod phase;
mod policies;
mod rev_limit;
mod safety;
mod schedule;
mod sensors;
mod spec_oracle;
mod torque;
mod trigger;
mod ts_spec;
mod types;
mod validation;
mod vbat;
mod vvt;
mod warmup;

#[cfg(kani)]
#[path = "../proofs/kani.rs"]
pub(crate) mod kani_proofs;

#[cfg(verus)]
#[path = "../proofs/verus.rs"]
pub(crate) mod verus_proofs;

pub use ae::{ae_step, ae_step_with_deltas, AeCurves, AeStepResult};
pub use afterstart::{afterstart_corr_x1000, apply_afterstart_pw};
pub use air_load::{air_load_select, AirLoad, AirLoadConfig, AirLoadInput, AirLoadSource};
pub use arbiter::{arbiter_step, ArbiterInputs, ArbiterResult};
pub use baro::{
    baro_correction, select_baro_source, BaroSource, BaroSourceConfig, BaroSourceInput,
    BaroSourceReading,
};
pub use cranking::{apply_cranking_pw, cranking_corr_x1000, spark_selected_for_mode};
pub use deadtime::deadtime_lookup;
pub use dfco::{dfco_step, DfcoResult};
pub use fault::{
    fault_event_for_clear, fault_event_from_state, SpecCancelReason, SpecFaultAction,
    SpecFaultCode, SpecFaultEvent, SpecFaultPersistence, SpecFaultSeverity, SpecFaultState,
};
pub use flatshift::{flat_shift_step, FlatShiftResult};
pub use fuel::{
    compute_afr_corr_x1000, compute_pw_air_us, compute_pw_base_us, compute_pw_corr_us,
    lookup_afterstart_corr_x1000, lookup_baro_corr_x1000, lookup_clt_corr_x1000,
    lookup_cranking_corr_x1000, lookup_deadtime_us, lookup_iat_corr_x1000, lookup_target_afr,
    lookup_vbat_corr_x1000, lookup_ve, lookup_warmup_corr_x1000, FuelParts,
};
pub use idle::{idle_step, IdleResult};
pub use idle_timing::{
    idle_timing_active, idle_timing_step, idle_timing_step_with_base, lookup_idle_advance_deg10,
    select_idle_or_running_advance_deg10, IdleTimingResult,
};
pub use interp::{bilerp_i16, bilerp_u16, find_segment, lerp_i16, lerp_u16};
pub use knock::{knock_step, KnockResult};
pub use lambda_cl::{lambda_step, lambda_step_with_error, LambdaResult};
pub use launch::{launch_step, LaunchResult};
pub use numeric::{
    clamp_i32, clamp_u16, clamp_u32, cyc7200_distance, duration_us_to_deg10, mul_div_floor_i32,
    mul_div_floor_u32, mul_ratio_x1000, norm7200,
};
pub use persist_spec::{
    factory_reset, persist_decode, persist_encode, persist_migrate, EncodedPersistRecord,
    PersistDecodeError, PersistEncodeError, PersistMigrationError, PersistPage, PersistPageId,
    PERSIST_ANGLES_PAGE_BYTES, PERSIST_FUEL_PAGE_BYTES, PERSIST_IGNITION_PAGE_BYTES,
    PERSIST_MAX_PAYLOAD_BYTES, PERSIST_RECORD_MAX_BYTES, PERSIST_SCHEMA_VERSION_CURRENT,
};
pub use phase::{
    cam_phase_step, cam_phase_step_with_config, CamEdgeAction, CamPhase, CamPhaseConfig,
    CamPhaseState, CamPhaseStepResult, CamTooth,
};
pub use rev_limit::{rev_limit_step, RevLimitResult};
pub use safety::{safety_step, SafetyResult};
pub use schedule::{
    compute_dwell_us, compute_injection_target_deg10, compute_spark_advance_deg10,
    schedule_all_cylinders, schedule_all_cylinders_with_advance_trim, schedule_cylinder,
    schedule_cylinder_with_advance_trim, CylinderSchedule, FuelOutput, ScheduleOutput,
};
pub use sensors::baro::baro_from_counts;
pub use sensors::clt::clt_from_counts;
pub use sensors::iat::iat_from_counts;
pub use sensors::knock::knock_from_window;
pub use sensors::maf::maf_from_counts;
pub use sensors::map::map_from_counts;
pub use sensors::o2::{o2_from_counts, O2SensorReading};
pub use sensors::plausibility::{
    sensor_plausibility_step, SensorPlausibilityInput, SensorPlausibilityResult,
    SensorPlausibilityState,
};
pub use sensors::slew::{sensor_slew_step, SensorSlewInput, SensorSlewResult, SensorSlewState};
pub use sensors::tps::tps_from_counts;
pub use sensors::vbat::vbat_from_counts;
pub use spec_oracle::{evaluate_fuel, evaluate_schedule, evaluate_tables, step, update_state};
pub use torque::{
    derive_torque_request_x1000, mode_limiter_ceiling_x1000, torque_pipeline_step,
    TorquePipelineInputs, TorquePipelineResult,
};
pub use trigger::{trigger_60_2_step, TriggerState, TriggerStepResult, TriggerSyncState};
pub use ts_spec::{
    burn_page, committed_page_record, decode_outpc, encode_outpc, encode_ts_diag_log_oldest_first,
    page_meta, save_all, ts_diag_log_pop_oldest, ts_diag_log_push, ts_dispatch_step, write_page,
    OutpcCodecError, OutpcFrame, TsBurnSaveError, TsBurnSaveStore, TsCommandDecodeError,
    TsDecodeError, TsDiagLogEncoded, TsDiagLogEntry, TsDiagLogRing, TsDispatchError,
    TsDispatchResult, TsDispatchState, TsEffect, TsPageId, TsPageMeta, TsPageMetaError,
    TS_DIAG_LOG_CAPACITY, TS_DIAG_LOG_ENTRY_BYTES, TS_DIAG_LOG_MAX_ENCODED_BYTES,
    TS_DIAG_SOURCE_CONTEXT_PRESENT, TS_OUTPC_PAGE_BYTES, TS_PROTO_MAGIC,
};
pub use types::{
    AeState, AfrOverride, AfrX100, Axis16, Calibration, Curve16, CylinderArrayI16,
    CylinderArrayU16, CylinderId, Degrees10, DiagState, DiagnosticCode, EngineMode, EventBatch,
    EventBatchFull, EventKind, FuelModel, InjectionAngleMode, InputSnapshot, KnockState, Kpa10,
    LogicalState, MathState, Micros, Millivolts, O2SensorMode, ObservableOutput, PiIntegratorState,
    PulseWidthUs, PwMaxPolicy, RatioX1000, Rpm, SchedulerState, SemanticEvent, SignedCurve16,
    SignedDegrees10, StepResult, SyncState, Table2D16, TempC10, TrimPolicy, ValidatedCalibration,
    ValidationError, VePctX100,
};
pub use validation::{
    validate_axis, validate_calibration, validate_curve, validate_signed_curve, validate_table_i16,
    validate_table_u16, SignedValueRange, ValueRange,
};
pub use vbat::vbat_correction;
pub use vvt::{
    vvt_step, VvtCam, VvtChannelId, VvtConfig, VvtInput, VvtLoadSource, VvtMode, VvtState,
    VvtStepResult,
};
pub use warmup::{apply_warmup_pw, warmup_correction};

pub fn default_reference_calibration() -> ValidatedCalibration {
    use crate::types::{
        Axis16, Calibration, Curve16, CylinderArrayU16, FuelModel, InjectionAngleMode, PwMaxPolicy,
        SignedCurve16, Table2D16, TrimPolicy,
    };

    fn axis(values: &[u16]) -> Axis16 {
        let mut axis = Axis16 {
            len: values.len() as u8,
            ..Axis16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            axis.values[idx] = values[idx];
            idx += 1;
        }
        axis
    }

    fn table_u16(value: u16) -> Table2D16<u16> {
        let mut table = Table2D16 {
            rpm_axis: axis(&[500, 1000]),
            load_axis: axis(&[500, 1000]),
            ..Table2D16::default()
        };
        table.values[0][0] = value;
        table.values[0][1] = value;
        table.values[1][0] = value;
        table.values[1][1] = value;
        table
    }

    fn table_i16(value: i16) -> Table2D16<i16> {
        let mut table = Table2D16 {
            rpm_axis: axis(&[500, 1000]),
            load_axis: axis(&[500, 1000]),
            ..Table2D16::default()
        };
        table.values[0][0] = value;
        table.values[0][1] = value;
        table.values[1][0] = value;
        table.values[1][1] = value;
        table
    }

    fn table_u32(value: u32) -> Table2D16<u32> {
        let mut table = Table2D16 {
            rpm_axis: axis(&[500, 1000]),
            load_axis: axis(&[500, 1000]),
            ..Table2D16::default()
        };
        table.values[0][0] = value;
        table.values[0][1] = value;
        table.values[1][0] = value;
        table.values[1][1] = value;
        table
    }

    fn signed_curve(value: i16) -> SignedCurve16 {
        let mut curve = SignedCurve16 {
            axis: axis(&[500, 1000]),
            ..SignedCurve16::default()
        };
        curve.values[0] = value;
        curve.values[1] = value;
        curve
    }

    ValidatedCalibration(Calibration {
        fuel_model: FuelModel::SpeedDensityRequiredFuel,
        ve_table: table_u16(8000),
        afr_target_table: table_u16(1470),
        spark_advance_table_deg10: table_i16(150),
        dwell_table_us: table_u32(2500),
        injection_target_table_deg10: table_u16(360),
        deadtime_table_us: {
            let mut table = Table2D16 {
                rpm_axis: axis(&[1000, 13000]),
                load_axis: axis(&[1000, 3000]),
                ..Table2D16::default()
            };
            table.values[0][0] = 800;
            table.values[0][1] = 800;
            table.values[1][0] = 800;
            table.values[1][1] = 800;
            table
        },
        clt_corr_curve: {
            let mut curve = Curve16 {
                axis: axis(&[0, 100]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        iat_corr_curve: {
            let mut curve = Curve16 {
                axis: axis(&[0, 100]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        baro_corr_curve: {
            let mut curve = Curve16 {
                axis: axis(&[0, 100]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        vbat_corr_curve: {
            let mut curve = Curve16 {
                axis: axis(&[8000, 16000]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        cranking_curve: {
            let mut curve = Curve16 {
                axis: axis(&[0, 100]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        afterstart_table: {
            let mut table = Table2D16 {
                rpm_axis: axis(&[0, 1]),
                load_axis: axis(&[0, 100]),
                ..Table2D16::default()
            };
            table.values[0][0] = 1000;
            table.values[0][1] = 1000;
            table.values[1][0] = 1000;
            table.values[1][1] = 1000;
            table
        },
        afterstart_window_cycles: 0,
        warmup_curve: {
            let mut curve = Curve16 {
                axis: axis(&[0, 100]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        ae_tps_threshold_curve: {
            let mut curve = Curve16 {
                axis: axis(&[500, 7000]),
                ..Curve16::default()
            };
            curve.values[0] = 20000;
            curve.values[1] = 20000;
            curve
        },
        ae_map_threshold_curve: {
            let mut curve = Curve16 {
                axis: axis(&[100, 3000]),
                ..Curve16::default()
            };
            curve.values[0] = 20000;
            curve.values[1] = 20000;
            curve
        },
        ae_shot_curve_us: {
            let mut curve = Curve16 {
                axis: axis(&[100, 3000]),
                ..Curve16::default()
            };
            curve.values[0] = 0;
            curve.values[1] = 0;
            curve
        },
        ae_decay_steps_curve: {
            let mut curve = Curve16 {
                axis: axis(&[100, 3000]),
                ..Curve16::default()
            };
            curve.values[0] = 0;
            curve.values[1] = 0;
            curve
        },
        ae_decay_ratio_curve_x1000: {
            let mut curve = Curve16 {
                axis: axis(&[0, 16]),
                ..Curve16::default()
            };
            curve.values[0] = 1000;
            curve.values[1] = 1000;
            curve
        },
        dfco_entry_rpm: Rpm::new(20_000),
        dfco_exit_rpm: Rpm::new(19_000),
        dfco_entry_tps_x100: 0,
        dfco_exit_tps_x100: 100,
        dfco_entry_map_kpa10: Kpa10::new(0),
        dfco_delay_cycles: 0,
        soft_rev_rpm: Rpm::new(19_500),
        hard_rev_rpm: Rpm::new(20_000),
        rev_hysteresis_rpm: Rpm::new(100),
        soft_retard_max_deg10: 0,
        launch_rpm_limit: Rpm::new(20_000),
        launch_cut_cycles: 0,
        flat_shift_rpm_min: Rpm::new(20_000),
        flat_shift_cut_cycles: 0,
        knock_threshold_x100: 500,
        knock_retard_step_deg10: 20,
        knock_retard_max_deg10: 200,
        knock_recovery_step_deg10: 10,
        knock_recovery_delay_cycles: 2,
        tps_adc_min_counts: 0,
        tps_adc_max_counts: 4095,
        idle_target_rpm: Rpm::new(900),
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
        idle_timing_enabled: false,
        idle_timing_pid_enabled: false,
        idle_timing_rpm_max: Rpm::new(1200),
        idle_timing_tps_max_x100: 200,
        idle_advance_curve_deg10: signed_curve(0),
        idle_timing_kp_x1000: 0,
        idle_timing_ki_x1000: 0,
        idle_timing_min_trim_deg10: -300,
        idle_timing_max_trim_deg10: 300,
        clt_timing_corr_curve_deg10: signed_curve(0),
        iat_timing_corr_curve_deg10: signed_curve(0),
        lambda_kp_x1000: 0,
        lambda_ki_x1000: 0,
        o2_sensor_mode: O2SensorMode::WidebandLinear,
        o2_wideband_afr_min_x100: 500,
        o2_wideband_afr_max_x100: 3000,
        o2_narrowband_threshold_counts: 2048,
        o2_narrowband_hysteresis_counts: 64,
        o2_narrowband_rich_afr_x100: 1400,
        o2_narrowband_lean_afr_x100: 1550,
        required_fuel_us: 3000,
        pref_kpa10: 1000,
        stoich_afr_x100: 1470,
        trim_policy: TrimPolicy::Identity,
        pw_max_policy: PwMaxPolicy::Fixed,
        pw_max_us: 25000,
        injection_angle_mode: InjectionAngleMode::EndOfInjection,
        cylinder_phase_deg10: CylinderArrayU16 {
            count: 4,
            values: [0, 1800, 3600, 5400, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
    })
}
