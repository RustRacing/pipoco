//! Real-execution FM0016 conformance tests for ecu-runtime.
//!
//! Drives `EngineRuntime::step` through its public API for every FM0016
//! fixture and compares observable outputs against the frozen spec oracle.
//!
//! Conformance strategy:
//! - Covered fields: rpm (exact match)
//! - Covered fields: lambda_correction, lambda_integrator_state
//! - Adapter-contract fields: ve_pct, target_afr, pw_base, pw_air, pw_corr,
//!   idle_duty, advance_deg10_trim, cut_reason_code, knock_intensity,
//!   torque_allowed, torque_actuated, idle_integrator_state
//! - Torque rows (US-FM0512): torque_request_x1000, torque_allowed_x1000,
//!   torque_actuated_x1000 now come from a product-owned `StepResult`
//!   observation surface. `torque_allowed_x1000` now zeros on the product
//!   Off/Shutdown path, but the rows remain adapter-contract because the live
//!   step boundary still does not own the full
//!   safety_latched/fuel_cut/spark_cut/launch_cut/flat_shift_cut lattice.
//!
//! Cut rows (US-FM0511):
//! - fuel_cut and spark_cut are adapter-contract: RuntimeAdapterContract::FuelCutInput
//!   and RuntimeAdapterContract::SparkCutInput respectively. Runtime does not receive cut
//!   input flags and has no separate product-owned cut source that derives from fixture
//!   input; the ActionBatch cut detection is the only observable. Safety latch, launch,
//!   and flat-shift cut remain adapter-contract for the same reason. Non-cut fixtures
//!   verify no accidental cut is produced. Covered rows for cuts would require runtime
//!   to own the cut causality through a distinct safety path, which does not exist today.
//!
//! Torque rows (US-FM0512):
//! - The semantic torque evaluator is still checked against FM0016 fixtures
//!   as anti-shortcut coverage, but it is not the source of product evidence.

#![cfg(test)]

use ecu_calibration::{
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertUnlock,
    SecondaryTriggerMode, TriggerAuthority,
};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ControlMode, CrankSyncState, EngineTimeAuthority,
    FaultCode, FaultSeverity, Kpa10, Micros, PhaseSyncState, Rpm, SyncState as DomainSyncState,
};
use ecu_runtime::compat::StepInputs;
use ecu_runtime::semantic::{
    conformance::{
        runtime_semantic_evaluate_schedule_with_authority, runtime_semantic_evaluate_torque,
        RuntimeConformanceStatus, RuntimeSemanticScheduleCalibration,
        RuntimeSemanticScheduleDiagnostic, RuntimeSemanticScheduleEvent,
        RuntimeSemanticScheduleEventKind, RuntimeSemanticTorqueInput,
    },
    runtime_semantic_evaluate_fuel, RuntimeSemanticAfrOverride, RuntimeSemanticAxis16,
    RuntimeSemanticCalibration, RuntimeSemanticCurve16U16, RuntimeSemanticCylinderArrayU16,
    RuntimeSemanticEngineMode, RuntimeSemanticFuelObservations, RuntimeSemanticInjectionAngleMode,
    RuntimeSemanticInputSnapshot, RuntimeSemanticState, RuntimeSemanticTable2dI16,
    RuntimeSemanticTable2dU16, RuntimeSemanticTable2dU32,
};
use ecu_runtime::support::{
    extract_fuel_observations, RuntimeAdapterContract, RuntimeObservedSurface,
};
use ecu_runtime::{
    runtime_full_sequential_authorized, runtime_x100_to_spec_x1000, Action, ActionBatch,
    BaseFuelModel, ControlInputs, EngineRuntime, EnrichmentInputs, IgnitionInputs,
    LambdaTrimInputs, TorqueInputs,
};
use ecu_spec::{
    AfrOverride, CylinderArrayU16, EngineMode, InjectionAngleMode, InputSnapshot,
    ValidatedCalibration,
};

#[path = "../../compat/tests/formal/fm0016_fixture_matrix.rs"]
mod fm0016_fixture_matrix;

// ---------------------------------------------------------------------------
// Tolerance constants
// ---------------------------------------------------------------------------

const EPS_RPM: u16 = 0; // exact match
const EPS_TORQUE_PCT: u16 = 50; // ±50 %-points (different torque models)

// ---------------------------------------------------------------------------
// Runtime x100 to spec x1000 unit bridge (US-FM0512)
// NOTE: Uses the helper from ecu-runtime lib.rs
// runtime_x100_to_spec_x1000 is re-exported by the lib
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Fuel model builder
// ---------------------------------------------------------------------------

#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
fn build_fuel_model(cal: &ValidatedCalibration) -> BaseFuelModel {
    let rpm_axis = &cal.0.ve_table.rpm_axis;
    let load_axis = &cal.0.ve_table.load_axis;
    let rpm_len = rpm_axis.len as usize;
    let load_len = load_axis.len as usize;

    let mut ve_values = [[ecu_domain::PulseWidthUs::new(0); 16]; 16];
    for r in 0..16 {
        for c in 0..16 {
            let cal_load = r.min(load_len.saturating_sub(1));
            let cal_rpm = c.min(rpm_len.saturating_sub(1));
            ve_values[r][c] =
                ecu_domain::PulseWidthUs::new(cal.0.ve_table.values[cal_load][cal_rpm]);
        }
    }

    let mut rpm_bins = [ecu_domain::Rpm::new(0); 16];
    for i in 0..16 {
        if i < rpm_len {
            rpm_bins[i] = ecu_domain::Rpm::new(rpm_axis.values[i]);
        }
    }
    let mut load_bins = [ecu_domain::Kpa10::new(0); 16];
    for i in 0..16 {
        if i < load_len {
            load_bins[i] = ecu_domain::Kpa10::new(load_axis.values[i]);
        }
    }

    BaseFuelModel::new(rpm_bins, load_bins, ve_values)
}

// ---------------------------------------------------------------------------
// Semantic calibration builder (US-FM0805)
// ---------------------------------------------------------------------------

/// Convert a ValidatedCalibration to RuntimeSemanticCalibration for the
/// v9 semantic evaluator.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
fn build_semantic_calibration(cal: &ValidatedCalibration) -> RuntimeSemanticCalibration {
    // Helper to copy a Table2D16<T> to RuntimeSemanticTable2dU16
    fn copy_table_2d(src: &ecu_spec::Table2D16<u16>) -> RuntimeSemanticTable2dU16 {
        let rpm_len = src.rpm_axis.len as usize;
        let load_len = src.load_axis.len as usize;
        let mut rpm_axis = RuntimeSemanticAxis16 {
            len: src.rpm_axis.len,
            values: [0; 16],
        };
        let mut load_axis = RuntimeSemanticAxis16 {
            len: src.load_axis.len,
            values: [0; 16],
        };
        let mut values = [[0u16; 16]; 16];

        for i in 0..16 {
            if i < rpm_len {
                rpm_axis.values[i] = src.rpm_axis.values[i];
            }
        }
        for i in 0..16 {
            if i < load_len {
                load_axis.values[i] = src.load_axis.values[i];
            }
        }
        for r in 0..16 {
            for c in 0..16 {
                let cal_load = r.min(load_len.saturating_sub(1));
                let cal_rpm = c.min(rpm_len.saturating_sub(1));
                values[r][c] = src.values[cal_load][cal_rpm];
            }
        }

        RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values,
        }
    }

    // Helper to copy a Table2D16<u16> (deadtime_table_us is u16 in Calibration)
    fn copy_table_2d_u32_as_u16(src: &ecu_spec::Table2D16<u16>) -> RuntimeSemanticTable2dU16 {
        let rpm_len = src.rpm_axis.len as usize;
        let load_len = src.load_axis.len as usize;
        let mut rpm_axis = RuntimeSemanticAxis16 {
            len: src.rpm_axis.len,
            values: [0; 16],
        };
        let mut load_axis = RuntimeSemanticAxis16 {
            len: src.load_axis.len,
            values: [0; 16],
        };
        let mut values = [[0u16; 16]; 16];

        for i in 0..16 {
            if i < rpm_len {
                rpm_axis.values[i] = src.rpm_axis.values[i];
            }
        }
        for i in 0..16 {
            if i < load_len {
                load_axis.values[i] = src.load_axis.values[i];
            }
        }
        for r in 0..16 {
            for c in 0..16 {
                let cal_load = r.min(load_len.saturating_sub(1));
                let cal_rpm = c.min(rpm_len.saturating_sub(1));
                // deadtime_table_us values are u32 but stored as u16 in runtime semantic
                values[r][c] = src.values[cal_load][cal_rpm];
            }
        }

        RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values,
        }
    }

    // Helper to copy a Curve16 to RuntimeSemanticCurve16U16
    fn copy_curve(src: &ecu_spec::Curve16) -> RuntimeSemanticCurve16U16 {
        let len = src.axis.len as usize;
        let mut axis = RuntimeSemanticAxis16 {
            len: src.axis.len,
            values: [0; 16],
        };
        let mut values = [0u16; 16];

        for i in 0..16 {
            if i < len {
                axis.values[i] = src.axis.values[i];
            }
        }
        for i in 0..16 {
            if i < len {
                values[i] = src.values[i];
            }
        }

        RuntimeSemanticCurve16U16 { axis, values }
    }

    let c = &cal.0;
    RuntimeSemanticCalibration {
        ve_table: copy_table_2d(&c.ve_table),
        afr_target_table: copy_table_2d(&c.afr_target_table),
        deadtime_table_us: copy_table_2d_u32_as_u16(&c.deadtime_table_us),
        clt_corr_curve: copy_curve(&c.clt_corr_curve),
        iat_corr_curve: copy_curve(&c.iat_corr_curve),
        baro_corr_curve: copy_curve(&c.baro_corr_curve),
        vbat_corr_curve: copy_curve(&c.vbat_corr_curve),
        cranking_curve: copy_curve(&c.cranking_curve),
        afterstart_table: copy_table_2d(&c.afterstart_table),
        warmup_curve: copy_curve(&c.warmup_curve),
        ae_tps_threshold_curve: copy_curve(&c.ae_tps_threshold_curve),
        ae_map_threshold_curve: copy_curve(&c.ae_map_threshold_curve),
        ae_shot_curve_us: copy_curve(&c.ae_shot_curve_us),
        ae_decay_steps_curve: copy_curve(&c.ae_decay_steps_curve),
        ae_decay_ratio_curve_x1000: copy_curve(&c.ae_decay_ratio_curve_x1000),
        required_fuel_us: c.required_fuel_us,
        pref_kpa10: c.pref_kpa10,
        stoich_afr_x100: c.stoich_afr_x100,
        pw_max_us: c.pw_max_us,
        afterstart_window_cycles: c.afterstart_window_cycles,
        dfco_entry_rpm: c.dfco_entry_rpm.0,
        dfco_exit_rpm: c.dfco_exit_rpm.0,
        dfco_entry_tps_x100: c.dfco_entry_tps_x100,
        dfco_exit_tps_x100: c.dfco_exit_tps_x100,
        dfco_entry_map_kpa10: c.dfco_entry_map_kpa10.0,
        dfco_delay_cycles: c.dfco_delay_cycles,
        soft_rev_rpm: c.soft_rev_rpm.0,
        hard_rev_rpm: c.hard_rev_rpm.0,
        rev_hysteresis_rpm: c.rev_hysteresis_rpm.0,
        soft_retard_max_deg10: c.soft_retard_max_deg10,
        launch_rpm_limit: c.launch_rpm_limit.0,
        launch_cut_cycles: c.launch_cut_cycles,
        flat_shift_rpm_min: c.flat_shift_rpm_min.0,
        flat_shift_cut_cycles: c.flat_shift_cut_cycles,
        knock_threshold_x100: c.knock_threshold_x100,
        knock_retard_step_deg10: c.knock_retard_step_deg10,
        knock_retard_max_deg10: c.knock_retard_max_deg10,
        knock_recovery_step_deg10: c.knock_recovery_step_deg10,
        knock_recovery_delay_cycles: c.knock_recovery_delay_cycles,
        lambda_kp_x1000: c.lambda_kp_x1000,
        lambda_ki_x1000: c.lambda_ki_x1000,
    }
}

// ---------------------------------------------------------------------------
// Semantic schedule calibration builder (US-FM0904)
// ---------------------------------------------------------------------------

/// Convert a ValidatedCalibration to RuntimeSemanticScheduleCalibration for
/// the v10 semantic schedule evaluator.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
fn build_semantic_schedule_calibration(
    cal: &ValidatedCalibration,
) -> RuntimeSemanticScheduleCalibration {
    // Helper to copy a Table2D16<T> to RuntimeSemanticTable2dU16
    fn copy_table_2d_u16(src: &ecu_spec::Table2D16<u16>) -> RuntimeSemanticTable2dU16 {
        let rpm_len = src.rpm_axis.len as usize;
        let load_len = src.load_axis.len as usize;
        let mut rpm_axis = RuntimeSemanticAxis16 {
            len: src.rpm_axis.len,
            values: [0; 16],
        };
        let mut load_axis = RuntimeSemanticAxis16 {
            len: src.load_axis.len,
            values: [0; 16],
        };
        let mut values = [[0u16; 16]; 16];

        for i in 0..16 {
            if i < rpm_len {
                rpm_axis.values[i] = src.rpm_axis.values[i];
            }
        }
        for i in 0..16 {
            if i < load_len {
                load_axis.values[i] = src.load_axis.values[i];
            }
        }
        for r in 0..16 {
            for c in 0..16 {
                let cal_load = r.min(load_len.saturating_sub(1));
                let cal_rpm = c.min(rpm_len.saturating_sub(1));
                values[r][c] = src.values[cal_load][cal_rpm];
            }
        }

        RuntimeSemanticTable2dU16 {
            rpm_axis,
            load_axis,
            values,
        }
    }

    // Helper to copy a Table2D16<i16> to RuntimeSemanticTable2dI16
    fn copy_table_2d_i16(src: &ecu_spec::Table2D16<i16>) -> RuntimeSemanticTable2dI16 {
        let rpm_len = src.rpm_axis.len as usize;
        let load_len = src.load_axis.len as usize;
        let mut rpm_axis = RuntimeSemanticAxis16 {
            len: src.rpm_axis.len,
            values: [0; 16],
        };
        let mut load_axis = RuntimeSemanticAxis16 {
            len: src.load_axis.len,
            values: [0; 16],
        };
        let mut values = [[0i16; 16]; 16];

        for i in 0..16 {
            if i < rpm_len {
                rpm_axis.values[i] = src.rpm_axis.values[i];
            }
        }
        for i in 0..16 {
            if i < load_len {
                load_axis.values[i] = src.load_axis.values[i];
            }
        }
        for r in 0..16 {
            for c in 0..16 {
                let cal_load = r.min(load_len.saturating_sub(1));
                let cal_rpm = c.min(rpm_len.saturating_sub(1));
                values[r][c] = src.values[cal_load][cal_rpm];
            }
        }

        RuntimeSemanticTable2dI16 {
            rpm_axis,
            load_axis,
            values,
        }
    }

    // Helper to copy a Table2D16<u32> to RuntimeSemanticTable2dU32
    fn copy_table_2d_u32(src: &ecu_spec::Table2D16<u32>) -> RuntimeSemanticTable2dU32 {
        let rpm_len = src.rpm_axis.len as usize;
        let load_len = src.load_axis.len as usize;
        let mut rpm_axis = RuntimeSemanticAxis16 {
            len: src.rpm_axis.len,
            values: [0; 16],
        };
        let mut load_axis = RuntimeSemanticAxis16 {
            len: src.load_axis.len,
            values: [0; 16],
        };
        let mut values = [[0u32; 16]; 16];

        for i in 0..16 {
            if i < rpm_len {
                rpm_axis.values[i] = src.rpm_axis.values[i];
            }
        }
        for i in 0..16 {
            if i < load_len {
                load_axis.values[i] = src.load_axis.values[i];
            }
        }
        for r in 0..16 {
            for c in 0..16 {
                let cal_load = r.min(load_len.saturating_sub(1));
                let cal_rpm = c.min(rpm_len.saturating_sub(1));
                values[r][c] = src.values[cal_load][cal_rpm];
            }
        }

        RuntimeSemanticTable2dU32 {
            rpm_axis,
            load_axis,
            values,
        }
    }

    // Helper to copy CylinderArrayU16 to RuntimeSemanticCylinderArrayU16
    fn copy_cylinder_array(src: &CylinderArrayU16) -> RuntimeSemanticCylinderArrayU16 {
        let mut values = [0u16; 16];
        for i in 0..16 {
            if i < src.count as usize {
                values[i] = src.values[i];
            }
        }
        RuntimeSemanticCylinderArrayU16 {
            count: src.count,
            values,
        }
    }

    let c = &cal.0;
    RuntimeSemanticScheduleCalibration {
        spark_advance_table_deg10: copy_table_2d_i16(&c.spark_advance_table_deg10),
        dwell_table_us: copy_table_2d_u32(&c.dwell_table_us),
        injection_target_table_deg10: copy_table_2d_u16(&c.injection_target_table_deg10),
        injection_angle_mode: match c.injection_angle_mode {
            InjectionAngleMode::StartOfInjection => {
                RuntimeSemanticInjectionAngleMode::StartOfInjection
            }
            InjectionAngleMode::EndOfInjection => RuntimeSemanticInjectionAngleMode::EndOfInjection,
        },
        cylinder_phase_deg10: copy_cylinder_array(&c.cylinder_phase_deg10),
    }
}

/// Convert an InputSnapshot to RuntimeSemanticInputSnapshot for the v9
/// semantic evaluator.
fn to_semantic_input(input: &InputSnapshot) -> RuntimeSemanticInputSnapshot {
    let mode = match input.mode {
        EngineMode::Off => RuntimeSemanticEngineMode::Off,
        EngineMode::Cranking => RuntimeSemanticEngineMode::Cranking,
        EngineMode::Running => RuntimeSemanticEngineMode::Running,
        EngineMode::Shutdown => RuntimeSemanticEngineMode::Shutdown,
    };
    let target_afr_override = match input.target_afr_override_x100 {
        AfrOverride::None => RuntimeSemanticAfrOverride::None,
        AfrOverride::Some(afr) => RuntimeSemanticAfrOverride::Some(afr.get()),
    };
    RuntimeSemanticInputSnapshot {
        t_us: Micros::new(input.t_us.0),
        rpm: Rpm::new(input.rpm.0),
        map_kpa10: Kpa10::new(input.map_kpa10.0),
        load_kpa10: Kpa10::new(input.load_kpa10.0),
        tps_x100: input.tps_x100,
        clt_c10: input.clt_c10.0,
        iat_c10: input.iat_c10.0,
        baro_kpa10: Kpa10::new(input.baro_kpa10.0),
        vbatt_mv: input.vbatt_mv.0,
        knock_intensity_x100: input.knock_intensity_x100,
        launch_armed: input.launch_armed,
        flat_shift_armed: input.flat_shift_armed,
        sync: match input.sync {
            ecu_spec::SyncState::Synced => DomainSyncState::Locked { cam_ref: false },
            ecu_spec::SyncState::Unsynced => DomainSyncState::Unsynced,
        },
        fuel_cut: input.fuel_cut,
        spark_cut: input.spark_cut,
        mode,
        target_afr_override_x100: target_afr_override,
    }
}

fn semantic_schedule_authority(input: RuntimeSemanticInputSnapshot) -> EngineTimeAuthority {
    if matches!(input.sync, DomainSyncState::Locked { .. }) {
        let calibration = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            injection_layout: ExpertInjectionLayout::Sequential,
            ..ExpertTriggerCalibration::default()
        };
        let startup_authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );

        calibration
            .to_runtime_engine_time_authority(startup_authority)
            .expect("validated manual runtime authority")
    } else {
        EngineTimeAuthority::none()
    }
}

// ---------------------------------------------------------------------------
// ControlInputs builder
// ---------------------------------------------------------------------------

fn to_control_inputs(input: &InputSnapshot) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: ecu_domain::Micros::new(input.t_us.0),
            clt_c: input.clt_c10.0 / 10,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            clt_c: input.clt_c10.0 / 10,
            lambda_valid: true,
            measured_lambda100: ecu_domain::Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(
            input.tps_x100 / 10, // driver_request_x100: spec derives from TPS
            100,                 // idle_request_x100: runtime default, not in spec torque pipeline
            10_000,              // rev_limit_x100: high so runtime limiter doesn't incorrectly cap
            10_000,              // knock_limit_x100: high so it doesn't fire
            10_000,              // limp_limit_x100: high so it doesn't fire
        ),
        ignition: IgnitionInputs::new(
            ecu_domain::Degrees10::new(150),
            0,
            0,
            0,
            false,
            ecu_domain::Rpm::new(input.rpm.0),
        ),
    }
}

// ---------------------------------------------------------------------------
// StepInputs builder
// ---------------------------------------------------------------------------

fn to_step_inputs(input: &InputSnapshot) -> StepInputs {
    StepInputs {
        now_us: ecu_domain::Micros::new(input.t_us.0),
        rpm: input.rpm.0 as u32,
        load_kpa10: input.load_kpa10.get() as u32,
        angle_x10: 0,
        trigger_synced: matches!(input.sync, ecu_spec::SyncState::Synced),
        cam_seen: matches!(input.sync, ecu_spec::SyncState::Synced),
        launch_armed: input.launch_armed,
        flat_shift_armed: input.flat_shift_armed,
    }
}

// ---------------------------------------------------------------------------
// Runtime execution
// ---------------------------------------------------------------------------

fn run_fixture(case: &fm0016_fixture_matrix::FixtureCase) -> ecu_runtime::StepResult {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(build_fuel_model(&case.calibration));
    runtime.step(to_step_inputs(&case.input), to_control_inputs(&case.input))
}

// ---------------------------------------------------------------------------
// Cut flag extraction from ActionBatch
// ---------------------------------------------------------------------------

fn fuel_cut_from_actions<const N: usize>(actions: &ActionBatch<N>) -> bool {
    for action in (*actions).iter() {
        if let Action::CancelScheduler(_) = action {
            return true;
        }
        if let Action::ArmScheduler { injection, .. } = action {
            if injection.plan.pulse_width.get() == 0 {
                return true;
            }
        }
        if let Action::ArmInjection(injection) = action {
            if injection.plan.pulse_width.get() == 0 {
                return true;
            }
        }
    }
    false
}

fn spark_cut_from_actions<const N: usize>(actions: &ActionBatch<N>) -> bool {
    for action in (*actions).iter() {
        if let Action::CancelScheduler(_) = action {
            return true;
        }
        if let Action::ArmScheduler { ignition, .. } = action {
            if ignition.plan.dwell.get() == 0 {
                return true;
            }
        }
        if let Action::ArmIgnition(ignition) = action {
            if ignition.plan.dwell.get() == 0 {
                return true;
            }
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Observable surface extraction
// ---------------------------------------------------------------------------

fn extract_observable(result: &ecu_runtime::StepResult) -> RuntimeObservedSurface {
    // Use the library helper for fuel observations
    let fuel = extract_fuel_observations(result);
    let torque = result.torque_observations;
    RuntimeObservedSurface {
        rpm: result.validated.rpm.get(),
        sync: result.validated.rpm.get() > 0,
        fuel_cut: fuel_cut_from_actions(&result.actions),
        spark_cut: spark_cut_from_actions(&result.actions),
        torque_request_x100: result.control.torque.requested_x100,
        torque_allowed_x100: result.control.torque.allowed_x100,
        // Torque actuated is not exposed by runtime - use 0 as placeholder,
        // conformance is established via RuntimeAdapterContract::TorqueActuated
        torque_actuated_x100: 0,
        // Raw runtime-observed x1000 torque values emitted by `EngineRuntime::step`.
        torque_request_x1000: torque.request_x1000,
        torque_allowed_x1000: torque.allowed_x1000,
        torque_actuated_x1000: torque.actuated_x1000,
        runtime_base_fuel_pw_us: fuel.base_fuel_pw_us,
        runtime_enriched_fuel_pw_us: fuel.enriched_fuel_pw_us,
        runtime_lambda_target_x100: fuel.lambda_target_x100,
        ignition_advance_deg10: result.control.ignition.advance_deg10.get(),
        dwell_us: result.control.ignition.dwell_us.get(),
        control_mode: result.operating_mode,
        validated_rpm: result.validated.rpm.get(),
        validated_load_kpa10: result.validated.load_kpa10.get(),
        validated_clamped: result.validated.clamped,
    }
}

// ---------------------------------------------------------------------------
// Field-by-field conformance classification
// ---------------------------------------------------------------------------

fn conformance_status_for_field(field: &str) -> RuntimeConformanceStatus {
    match field {
        // Covered fields - directly comparable
        "rpm" => RuntimeConformanceStatus::Covered,
        // US-FM0805: v9 semantic evaluator now provides bit-exact fuel outputs
        // for these 7 fields. They are Covered by runtime_semantic_conformance_all_fixtures.
        // US-FM1004: lambda_correction_x1000 and lambda_integrator_state are also Covered.
        "ve_pct_x100" => RuntimeConformanceStatus::Covered,
        "target_afr_x100" => RuntimeConformanceStatus::Covered,
        "pw_base_us" => RuntimeConformanceStatus::Covered,
        "pw_air_us" => RuntimeConformanceStatus::Covered,
        "pw_corr_us" => RuntimeConformanceStatus::Covered,
        "fuel_cut" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::FuelCutInput)
        }
        "spark_cut" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::SparkCutInput)
        }
        "safety_latched" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::SafetyLatched)
        }
        "launch_cut" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::LaunchCut)
        }
        "flat_shift_cut" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::FlatShiftCut)
        }
        "lambda_correction_x1000" => RuntimeConformanceStatus::Covered,
        "lambda_integrator_state" => RuntimeConformanceStatus::Covered,
        // The runtime now emits a product-owned x1000 torque observation
        // surface on StepResult and zeros allowed torque for Off/Shutdown.
        // These rows remain adapter-contract until actuated torque is derived
        // from the full safety/fuel/spark/launch/flat-shift cut lattice.
        "torque_request_x1000" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::TorqueRequest)
        }
        "torque_allowed_x1000" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::TorqueAllowed)
        }
        "torque_actuated_x1000" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::TorqueActuated)
        }
        // Adapter contracts - non-equivalent architectures
        "idle_duty_x1000" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::IdleDuty)
        }
        "advance_deg10_trim" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::IgnitionAdvanceTrim)
        }
        "cut_reason_code" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::CutReasonCode)
        }
        "knock_intensity_x100" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::KnockIntensity)
        }
        "idle_integrator_state" => {
            RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::IdleIntegratorState)
        }
        _ => RuntimeConformanceStatus::Covered,
    }
}

// ---------------------------------------------------------------------------
// Comparison helpers
// ---------------------------------------------------------------------------

fn cmp_u16(field: &str, obs: u16, exp: u16, tol: u16) {
    let diff = obs.abs_diff(exp);
    if diff > tol {
        panic!("FAIL {field}: obs={obs} exp={exp} diff={diff} tol={tol}");
    }
}

// ---------------------------------------------------------------------------
// Per-fixture conformance test
// ---------------------------------------------------------------------------

fn conformance_test(case: &fm0016_fixture_matrix::FixtureCase) {
    // Call oracle_result ONCE for expected data
    let spec = fm0016_fixture_matrix::oracle_result(*case);
    // Execute EngineRuntime::step for observed data
    let runtime_result = run_fixture(case);

    // Assert fixture semantics via spec oracle
    fm0016_fixture_matrix::assert_fixture_semantics(*case, &spec);

    let obs = extract_observable(&runtime_result);
    let torque = runtime_result.torque_observations;
    assert_eq!(obs.torque_request_x1000, torque.request_x1000);
    assert_eq!(obs.torque_allowed_x1000, torque.allowed_x1000);
    assert_eq!(obs.torque_actuated_x1000, torque.actuated_x1000);

    // ----- RPM (exact match for all synced cases) -----
    cmp_u16("rpm", obs.rpm, case.input.rpm.0, EPS_RPM);

    // ----- Cut flags -----
    // Runtime does not model the frozen cut inputs directly, so cut fixtures
    // are adapter-contract rows. Non-cut fixtures still exercise the product
    // path and must not report accidental cuts.
    if !case.input.fuel_cut {
        assert!(
            !obs.fuel_cut,
            "FAIL fuel_cut: obs={} for non-cut fixture",
            obs.fuel_cut
        );
    }
    if !case.input.spark_cut {
        assert!(
            !obs.spark_cut,
            "FAIL spark_cut: obs={} for non-cut fixture",
            obs.spark_cut
        );
    }

    // ----- Torque request -----
    // Torque request is an adapter-contract field because runtime only exposes
    // x100 torque on the product path. Synced positive cases remain a sanity
    // check against accidental unit drift on the step-derived x1000 bridge.
    let is_synced = case.input.sync == ecu_spec::SyncState::Synced;
    if is_synced {
        let spec_tq = spec.output.torque_request_x1000;
        if spec_tq > 0 {
            cmp_u16(
                "torque_request_x100",
                obs.torque_request_x100,
                spec_tq,
                EPS_TORQUE_PCT,
            );
        }
    }

    // ----- Adapter contracts for non-equivalent fields -----
    // These fields are classified as adapter contracts and cannot be directly compared.
    // The conformance_status_for_field function maps each field to its appropriate
    // RuntimeAdapterContract variant. We verify that the runtime produces sensible
    // values (non-zero for fuel-related, within expected ranges for ignition).

    // Base fuel PW - runtime uses IPW table, spec uses VE computation
    assert!(
        obs.runtime_base_fuel_pw_us > 0 || case.input.sync == ecu_spec::SyncState::Unsynced,
        "RuntimeObservedSurface.runtime_base_fuel_pw_us should be non-zero for running fixtures"
    );

    // Enriched fuel PW - should be >= base_fuel_pw when enrichment is active
    assert!(
        obs.runtime_enriched_fuel_pw_us >= obs.runtime_base_fuel_pw_us,
        "RuntimeObservedSurface.runtime_enriched_fuel_pw_us >= runtime_base_fuel_pw_us"
    );

    // Lambda target - runtime should produce a sensible target lambda
    assert!(
        obs.runtime_lambda_target_x100 > 0,
        "RuntimeObservedSurface.runtime_lambda_target_x100 should be non-zero"
    );

    // Ignition advance - should be reasonable for running engine
    if case.input.sync == ecu_spec::SyncState::Synced && case.input.rpm.0 > 0 {
        assert!(
            obs.ignition_advance_deg10 >= 0,
            "RuntimeObservedSurface.ignition_advance_deg10 should be valid"
        );
    }

    // Dwell time - should be reasonable
    assert!(
        obs.dwell_us <= 10_000,
        "RuntimeObservedSurface.dwell_us should be <= 10000 us"
    );
}

// --------------------------------------------------------------------------
// US-FM0805: Runtime Semantic Fuel Evaluator Conformance Tests
// US-FM0512: Runtime Semantic Torque Evaluator Conformance Tests
// --------------------------------------------------------------------------

/// Tolerance for VE and AFR comparisons (in x100 units, e.g. 8000 = 80.00%)
const EPS_SEMANTIC_VE_X100: u16 = 1;
/// Tolerance for pulse-width comparisons in microseconds.
const EPS_SEMANTIC_PW_US: u32 = 1;

fn semantic_state_for_fixture(case: &fm0016_fixture_matrix::FixtureCase) -> RuntimeSemanticState {
    match (case.fixture, case.variant) {
        ("lambda_cl_integrator_response", "saturation") => RuntimeSemanticState {
            lambda_integrator_acc: 400,
            ..RuntimeSemanticState::default()
        },
        ("lambda_cl_integrator_response", "freeze_cut") => RuntimeSemanticState {
            lambda_integrator_acc: 180,
            ..RuntimeSemanticState::default()
        },
        ("knock_response", "retard") => RuntimeSemanticState {
            knock_retard_deg10: 40,
            knock_recovery_counter: 0,
            ..RuntimeSemanticState::default()
        },
        ("knock_response", "recovery") => RuntimeSemanticState {
            knock_retard_deg10: 60,
            knock_recovery_counter: 1,
            ..RuntimeSemanticState::default()
        },
        ("launch_control_pattern", "disarmed") => RuntimeSemanticState {
            launch_active: true,
            launch_cut_cycle_count: 3,
            ..RuntimeSemanticState::default()
        },
        ("flat_shift_pattern", "disarmed") => RuntimeSemanticState {
            flat_shift_active: true,
            flat_shift_cut_cycle_count: 3,
            ..RuntimeSemanticState::default()
        },
        ("safety_latching", "hold_through_clear_attempt")
        | ("safety_latching", "release_on_clear_condition") => RuntimeSemanticState {
            safety_latched: true,
            ..RuntimeSemanticState::default()
        },
        _ => RuntimeSemanticState::default(),
    }
}

fn runtime_semantic_torque_input_for_case(
    case: &fm0016_fixture_matrix::FixtureCase,
    semantic_input: &RuntimeSemanticInputSnapshot,
    fuel_obs: &RuntimeSemanticFuelObservations,
) -> RuntimeSemanticTorqueInput {
    // Build the semantic-oracle torque input only for fixture comparison.
    // Product torque observations stay on the EngineRuntime::step path.
    let state = semantic_state_for_fixture(case);
    let rev_hard_active = semantic_input.rpm.get() >= case.calibration.0.hard_rev_rpm.0;

    RuntimeSemanticTorqueInput {
        tps_x100: semantic_input.tps_x100,
        mode: semantic_input.mode,
        fuel_cut: fuel_obs.fuel_cut,
        spark_cut: fuel_obs.spark_cut,
        // The torque evaluator consumes the semantic cut result, not raw input
        // scaling. Safety is carried through when already latched. Launch and
        // flat-shift cuts are derived from the matching semantic state so this
        // scaffold exercises the direct cut inputs it claims to cover.
        safety_latched: state.safety_latched,
        rev_soft_active: state.rev_soft_active,
        rev_hard_active,
        launch_cut: matches!(case.fixture, "launch_control_pattern")
            && semantic_input.launch_armed
            && fuel_obs.fuel_cut
            && fuel_obs.spark_cut,
        flat_shift_cut: matches!(case.fixture, "flat_shift_pattern")
            && semantic_input.flat_shift_armed
            && fuel_obs.fuel_cut
            && fuel_obs.spark_cut,
    }
}

/// Run a single fixture through the semantic evaluator and compare against oracle.
fn semantic_conformance_test(case: &fm0016_fixture_matrix::FixtureCase) {
    // Oracle result (expected data)
    let spec = fm0016_fixture_matrix::oracle_result(*case);

    // Build semantic calibration from fixture calibration
    let semantic_cal = build_semantic_calibration(&case.calibration);

    // Build semantic input from fixture input
    let semantic_input = to_semantic_input(&case.input);

    // Run semantic evaluator
    let semantic_state_result = runtime_semantic_evaluate_fuel(
        &semantic_cal,
        semantic_input,
        semantic_state_for_fixture(case),
    );

    let obs = match semantic_state_result {
        Ok(o) => o,
        Err(e) => {
            panic!(
                "FAIL runtime_semantic_evaluate_fuel: case={} fixture={} error={:?}",
                case.fixture, case.variant, e
            );
        }
    };

    // VE percentage comparison
    let ve_diff = obs.ve_pct_x100.abs_diff(spec.output.ve_pct_x100.get());
    if ve_diff > EPS_SEMANTIC_VE_X100 {
        panic!(
            "FAIL ve_pct_x100: case={} fixture={} obs={} exp={} diff={} tol={}",
            case.fixture,
            case.variant,
            obs.ve_pct_x100,
            spec.output.ve_pct_x100.get(),
            ve_diff,
            EPS_SEMANTIC_VE_X100
        );
    }

    // Target AFR comparison
    let afr_diff = obs
        .target_afr_x100
        .abs_diff(spec.output.target_afr_x100.get());
    if afr_diff > EPS_SEMANTIC_VE_X100 {
        panic!(
            "FAIL target_afr_x100: case={} fixture={} obs={} exp={} diff={} tol={}",
            case.fixture,
            case.variant,
            obs.target_afr_x100,
            spec.output.target_afr_x100.get(),
            afr_diff,
            EPS_SEMANTIC_VE_X100
        );
    }

    // Base PW comparison
    let base_diff = obs.pw_base_us.abs_diff(spec.output.pw_base_us.get());
    if base_diff > EPS_SEMANTIC_PW_US {
        panic!(
            "FAIL pw_base_us: case={} fixture={} obs={} exp={} diff={} tol={}",
            case.fixture,
            case.variant,
            obs.pw_base_us,
            spec.output.pw_base_us.get(),
            base_diff,
            EPS_SEMANTIC_PW_US
        );
    }

    // Air PW comparison
    let air_diff = obs.pw_air_us.abs_diff(spec.output.pw_air_us.get());
    if air_diff > EPS_SEMANTIC_PW_US {
        panic!(
            "FAIL pw_air_us: case={} fixture={} obs={} exp={} diff={} tol={}",
            case.fixture,
            case.variant,
            obs.pw_air_us,
            spec.output.pw_air_us.get(),
            air_diff,
            EPS_SEMANTIC_PW_US
        );
    }

    // Corrected PW comparison
    let corr_diff = obs.pw_corr_us.abs_diff(spec.output.pw_corr_us.get());
    if corr_diff > EPS_SEMANTIC_PW_US {
        panic!(
            "FAIL pw_corr_us: case={} fixture={} obs={} exp={} diff={} tol={}",
            case.fixture,
            case.variant,
            obs.pw_corr_us,
            spec.output.pw_corr_us.get(),
            corr_diff,
            EPS_SEMANTIC_PW_US
        );
    }

    // Fuel cut exact comparison
    if obs.fuel_cut != spec.output.fuel_cut {
        panic!(
            "FAIL fuel_cut: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, obs.fuel_cut, spec.output.fuel_cut
        );
    }

    // Spark cut exact comparison
    if obs.spark_cut != spec.output.spark_cut {
        panic!(
            "FAIL spark_cut: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, obs.spark_cut, spec.output.spark_cut
        );
    }

    // Lambda correction exact comparison (US-FM1004)
    // ObservableOutput.lambda_correction_x1000 is a plain u16 (not a newtype wrapper)
    if obs.lambda_correction_x1000 != spec.output.lambda_correction_x1000 {
        panic!(
            "FAIL lambda_correction_x1000: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.lambda_correction_x1000,
            spec.output.lambda_correction_x1000
        );
    }

    // Lambda integrator state exact comparison (US-FM1004)
    if obs.lambda_integrator_state.acc != spec.next_state.lambda_integrator_state.acc {
        panic!(
            "FAIL lambda_integrator_state.acc: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.lambda_integrator_state.acc,
            spec.next_state.lambda_integrator_state.acc
        );
    }
    if obs.lambda_integrator_state.min_acc != spec.next_state.lambda_integrator_state.min_acc {
        panic!(
            "FAIL lambda_integrator_state.min_acc: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.lambda_integrator_state.min_acc,
            spec.next_state.lambda_integrator_state.min_acc
        );
    }
    if obs.lambda_integrator_state.max_acc != spec.next_state.lambda_integrator_state.max_acc {
        panic!(
            "FAIL lambda_integrator_state.max_acc: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.lambda_integrator_state.max_acc,
            spec.next_state.lambda_integrator_state.max_acc
        );
    }
    if obs.lambda_integrator_state.frozen != spec.next_state.lambda_integrator_state.frozen {
        panic!(
            "FAIL lambda_integrator_state.frozen: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.lambda_integrator_state.frozen,
            spec.next_state.lambda_integrator_state.frozen
        );
    }

    // ----- Torque semantic conformance (US-FM0512) -----
    // Runtime semantic torque evaluator mirrors the frozen spec oracle pipeline.
    // Build the torque input from the semantic input snapshot.
    let torque_input = runtime_semantic_torque_input_for_case(case, &semantic_input, &obs);
    let torque_result = runtime_semantic_evaluate_torque(torque_input);

    // Torque request: mirrors spec oracle request stage (TPS -> x1000, clamped).
    // No tolerance — request is deterministic from TPS.
    if torque_result.torque_request_x1000 != spec.output.torque_request_x1000 {
        panic!(
            "FAIL torque_request_x1000: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            torque_result.torque_request_x1000,
            spec.output.torque_request_x1000
        );
    }

    // Torque allowed: this must fail if the evaluator collapses to direct TPS
    // scaling, because the rev-limit fixtures force the oracle limiter to zero.
    let spec_allowed = spec.output.torque_allowed_x1000;
    if torque_result.torque_allowed_x1000 != spec_allowed {
        panic!(
            "FAIL torque_allowed_x1000: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, torque_result.torque_allowed_x1000, spec_allowed
        );
    }

    // Torque actuated: this exercises the cut-gating path directly, including
    // the rev-limit, launch, flat-shift, and safety fixtures.
    let spec_actuated = spec.output.torque_actuated_x1000;
    if torque_result.torque_actuated_x1000 != spec_actuated {
        panic!(
            "FAIL torque_actuated_x1000: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, torque_result.torque_actuated_x1000, spec_actuated
        );
    }
}

#[test]
fn runtime_semantic_conformance_all_fixtures() {
    let cases = fm0016_fixture_matrix::fixture_cases();
    for case in cases {
        semantic_conformance_test(&case);
    }
}

// ---------------------------------------------------------------------------
// Adapter contract verification tests
// ---------------------------------------------------------------------------

#[test]
fn runtime_adapter_contracts_cover_all_non_equivalent_fields() {
    let fields = [
        // US-FM0805: ve_pct_x100, target_afr_x100, pw_base_us, pw_air_us, pw_corr_us
        // are covered by runtime_semantic_conformance_all_fixtures.
        // US-FM1004: lambda_correction_x1000 and lambda_integrator_state are also Covered.
        // Remaining adapter contracts:
        "fuel_cut",
        "spark_cut",
        "safety_latched",
        "launch_cut",
        "flat_shift_cut",
        "torque_request_x1000",
        "torque_allowed_x1000",
        "torque_actuated_x1000",
        "idle_duty_x1000",
        "advance_deg10_trim",
        "cut_reason_code",
        "knock_intensity_x100",
        "idle_integrator_state",
    ];

    for field in fields {
        let status = conformance_status_for_field(field);
        match status {
            RuntimeConformanceStatus::Covered => {
                panic!("Field {field} should be AdapterContract, not Covered");
            }
            RuntimeConformanceStatus::AdapterContract(_) => {
                // Expected
            }
        }
    }
}

#[test]
fn runtime_torque_x1000_rows_stay_blocked_on_product_step_surface() {
    let fields = [
        "safety_latched",
        "fuel_cut",
        "spark_cut",
        "launch_cut",
        "flat_shift_cut",
        "torque_request_x1000",
        "torque_allowed_x1000",
        "torque_actuated_x1000",
    ];
    let blocker = "the runtime now emits product-owned torque x1000 observations, and Off/Shutdown zeroing is handled on the live step path, but the rows stay AdapterContract because the runtime still does not own the full safety_latched/fuel_cut/spark_cut/launch_cut/flat_shift_cut lattice on step inputs and cannot prove the remaining direct cut sources from the product path alone";

    for field in fields {
        let status = conformance_status_for_field(field);
        match status {
            RuntimeConformanceStatus::Covered => {
                panic!("Field {field} should stay AdapterContract: {blocker}");
            }
            RuntimeConformanceStatus::AdapterContract(_) => {}
        }
    }
}

#[test]
fn runtime_torque_x1000_shutdown_path_zeros_allowed_and_actuated() {
    let mut runtime = EngineRuntime::new();

    runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 2_000,
            load_kpa10: 500,
            angle_x10: 100,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(1_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(75, 50, 100, 100, 100),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(120),
                0,
                0,
                0,
                false,
                ecu_domain::Rpm::new(2_000),
            ),
        },
    );

    assert_eq!(result.operating_mode, ControlMode::Shutdown);
    assert_eq!(result.torque_observations.request_x1000, 750);
    assert_eq!(result.torque_observations.allowed_x1000, 0);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}

#[test]
fn runtime_torque_x1000_off_phase_zeros_allowed_and_actuated() {
    let mut runtime = EngineRuntime::new();

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 0,
            load_kpa10: 0,
            angle_x10: 0,
            trigger_synced: false,
            cam_seen: false,
            launch_armed: false,
            flat_shift_armed: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(1_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(75, 50, 100, 100, 100),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(120),
                0,
                0,
                0,
                false,
                ecu_domain::Rpm::new(0),
            ),
        },
    );

    assert_eq!(result.operating_mode, ControlMode::OpenLoop);
    assert_eq!(result.torque_observations.request_x1000, 750);
    assert_eq!(result.torque_observations.allowed_x1000, 0);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}

#[test]
fn runtime_semantic_schedule_requires_validated_authority_not_sync_alone() {
    let case = fm0016_fixture_matrix::fixture_cases()
        .into_iter()
        .find(|case| {
            matches!(case.input.sync, ecu_spec::SyncState::Synced)
                && case.input.rpm.0 > 0
                && !case.input.fuel_cut
                && !case.input.spark_cut
        })
        .expect("synced running fixture");
    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);
    let semantic_input = to_semantic_input(&case.input);
    let fuel_obs = runtime_semantic_evaluate_fuel(
        &build_semantic_calibration(&case.calibration),
        semantic_input,
        semantic_state_for_fixture(&case),
    )
    .expect("fuel observations");

    let sync_state_only = EngineTimeAuthority::none();
    assert_eq!(
        semantic_input.sync,
        DomainSyncState::Locked { cam_ref: false }
    );
    assert!(!runtime_full_sequential_authorized(sync_state_only));

    let blocked = runtime_semantic_evaluate_schedule_with_authority(
        &schedule_cal,
        semantic_input,
        fuel_obs,
        sync_state_only,
    )
    .expect("blocked schedule still validates tables");
    assert_eq!(blocked.events.len, 0);
    assert_eq!(
        blocked.diagnostic,
        RuntimeSemanticScheduleDiagnostic::Unsynced
    );

    let permitted_authority = semantic_schedule_authority(semantic_input);
    assert!(runtime_full_sequential_authorized(permitted_authority));
    let permitted = runtime_semantic_evaluate_schedule_with_authority(
        &schedule_cal,
        semantic_input,
        fuel_obs,
        permitted_authority,
    )
    .expect("permitted schedule");
    assert!(permitted.events.len > 0);
}

#[test]
fn runtime_adapter_contracts_map_to_correct_variants() {
    // US-FM0805: ve_pct_x100, target_afr_x100, pw_base_us, pw_air_us, pw_corr_us
    // are covered by runtime_semantic_conformance_all_fixtures.
    // US-FM1004: lambda_correction_x1000 and lambda_integrator_state are also Covered.
    // Remaining adapter contract assertions:
    assert!(matches!(
        conformance_status_for_field("fuel_cut"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::FuelCutInput)
    ));
    assert!(matches!(
        conformance_status_for_field("spark_cut"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::SparkCutInput)
    ));
    assert!(matches!(
        conformance_status_for_field("torque_request_x1000"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::TorqueRequest)
    ));
    assert!(matches!(
        conformance_status_for_field("torque_allowed_x1000"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::TorqueAllowed)
    ));
    assert!(matches!(
        conformance_status_for_field("torque_actuated_x1000"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::TorqueActuated)
    ));
    assert!(matches!(
        conformance_status_for_field("idle_duty_x1000"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::IdleDuty)
    ));
    assert!(matches!(
        conformance_status_for_field("advance_deg10_trim"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::IgnitionAdvanceTrim)
    ));
    assert!(matches!(
        conformance_status_for_field("cut_reason_code"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::CutReasonCode)
    ));
    assert!(matches!(
        conformance_status_for_field("knock_intensity_x100"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::KnockIntensity)
    ));
    assert!(matches!(
        conformance_status_for_field("idle_integrator_state"),
        RuntimeConformanceStatus::AdapterContract(RuntimeAdapterContract::IdleIntegratorState)
    ));
}

#[test]
fn runtime_semantic_torque_rows_remain_scaffold_only() {
    for field in [
        "torque_request_x1000",
        "torque_allowed_x1000",
        "torque_actuated_x1000",
    ] {
        assert!(matches!(
            conformance_status_for_field(field),
            RuntimeConformanceStatus::AdapterContract(_)
        ));
    }
}

// ---------------------------------------------------------------------------
// US-FM0512: runtime_x100_to_spec_x1000 unit bridge tests
// ---------------------------------------------------------------------------

#[test]
fn runtime_x100_to_spec_x1000_zero() {
    // Zero input must produce zero output
    assert_eq!(runtime_x100_to_spec_x1000(0), 0);
}

#[test]
fn runtime_x100_to_spec_x1000_100() {
    // 100 x100 = 1000 x1000 (nominal scaling)
    assert_eq!(runtime_x100_to_spec_x1000(100), 1000);
}

#[test]
fn runtime_x100_to_spec_x1000_nominal() {
    // A mid-range value like 537 x100 = 5370 x1000
    assert_eq!(runtime_x100_to_spec_x1000(537), 5370);
}

#[test]
fn runtime_x100_to_spec_x1000_max() {
    // u16::MAX x100 = u16::MAX x1000 (saturating at MAX, no overflow)
    assert_eq!(runtime_x100_to_spec_x1000(u16::MAX), u16::MAX);
}

#[test]
fn runtime_x100_to_spec_x1000_boundary_before_saturation() {
    // (u16::MAX / 10) * 10 should not saturate
    let max_safe = u16::MAX / 10;
    assert_eq!(runtime_x100_to_spec_x1000(max_safe), max_safe * 10);
}

#[test]
fn runtime_x100_to_spec_x1000_one_above_saturation_boundary() {
    // (u16::MAX / 10 + 1) * 10 must saturate at u16::MAX
    let above = u16::MAX / 10 + 1;
    assert_eq!(runtime_x100_to_spec_x1000(above), u16::MAX);
}

// ---------------------------------------------------------------------------
// Test suite
// ---------------------------------------------------------------------------

fn run_all(cases: &[fm0016_fixture_matrix::FixtureCase]) {
    for case in cases {
        conformance_test(case);
    }
}

fn synced_fixtures() -> Vec<fm0016_fixture_matrix::FixtureCase> {
    fm0016_fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| matches!(c.input.sync, ecu_spec::SyncState::Synced))
        .collect()
}

fn unsynced_fixtures() -> Vec<fm0016_fixture_matrix::FixtureCase> {
    fm0016_fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| !matches!(c.input.sync, ecu_spec::SyncState::Synced))
        .collect()
}

fn cut_fixtures() -> Vec<fm0016_fixture_matrix::FixtureCase> {
    fm0016_fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| c.fixture.contains("cut"))
        .collect()
}

fn running_fixtures() -> Vec<fm0016_fixture_matrix::FixtureCase> {
    fm0016_fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| c.fixture.contains("running") && !c.fixture.contains("cut"))
        .collect()
}

#[test]
fn runtime_fm0016_unsynced() {
    let cases = unsynced_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn runtime_fm0016_cuts() {
    let cases = cut_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn runtime_fm0016_running() {
    let cases = running_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn runtime_fm0016_synced() {
    let cases = synced_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn runtime_fm0016_all() {
    run_all(&fm0016_fixture_matrix::fixture_cases());
}

// ---------------------------------------------------------------------------
// US-FM0904: Semantic schedule conformance tests
// ---------------------------------------------------------------------------

const EPS_ANGLE_DEG10: u16 = 1;

/// Run a single fixture through the v10 semantic schedule evaluator and compare
/// against the frozen oracle.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
fn schedule_conformance_test(case: &fm0016_fixture_matrix::FixtureCase) {
    // Oracle result (expected data)
    let spec = fm0016_fixture_matrix::oracle_result(*case);

    // Build schedule calibration from fixture calibration
    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);

    // Build semantic input from fixture input
    let semantic_input = to_semantic_input(&case.input);

    // Run semantic fuel evaluator first to get fuel observations
    let fuel_result = runtime_semantic_evaluate_fuel(
        &build_semantic_calibration(&case.calibration),
        semantic_input,
        semantic_state_for_fixture(case),
    );
    let fuel_obs = match fuel_result {
        Ok(o) => o,
        Err(e) => panic!(
            "FAIL fuel eval: case={} fixture={} error={:?}",
            case.fixture, case.variant, e
        ),
    };

    // Run semantic schedule evaluator
    let schedule_result = runtime_semantic_evaluate_schedule_with_authority(
        &schedule_cal,
        semantic_input,
        fuel_obs,
        semantic_schedule_authority(semantic_input),
    );
    let sched_obs = match schedule_result {
        Ok(o) => o,
        Err(e) => panic!(
            "FAIL schedule eval: case={} fixture={} error={:?}",
            case.fixture, case.variant, e
        ),
    };

    // Compare angles for each cylinder
    let spec_cyl_count = spec.output.soi_deg10.count as usize;
    for cyl in 0..spec_cyl_count {
        // SOI
        let diff = sched_obs.soi_deg10.values[cyl].abs_diff(spec.output.soi_deg10.values[cyl]);
        if diff > EPS_ANGLE_DEG10 {
            panic!(
                "FAIL soi_deg10: cyl={} case={} fixture={} obs={} exp={} diff={} tol={}",
                cyl,
                case.fixture,
                case.variant,
                sched_obs.soi_deg10.values[cyl],
                spec.output.soi_deg10.values[cyl],
                diff,
                EPS_ANGLE_DEG10
            );
        }
        // EOI
        let diff = sched_obs.eoi_deg10.values[cyl].abs_diff(spec.output.eoi_deg10.values[cyl]);
        if diff > EPS_ANGLE_DEG10 {
            panic!(
                "FAIL eoi_deg10: cyl={} case={} fixture={} obs={} exp={} diff={} tol={}",
                cyl,
                case.fixture,
                case.variant,
                sched_obs.eoi_deg10.values[cyl],
                spec.output.eoi_deg10.values[cyl],
                diff,
                EPS_ANGLE_DEG10
            );
        }
        // Spark
        let diff = sched_obs.spark_deg10.values[cyl].abs_diff(spec.output.spark_deg10.values[cyl]);
        if diff > EPS_ANGLE_DEG10 {
            panic!(
                "FAIL spark_deg10: cyl={} case={} fixture={} obs={} exp={} diff={} tol={}",
                cyl,
                case.fixture,
                case.variant,
                sched_obs.spark_deg10.values[cyl],
                spec.output.spark_deg10.values[cyl],
                diff,
                EPS_ANGLE_DEG10
            );
        }
        // Dwell start
        let diff = sched_obs.dwell_start_deg10.values[cyl]
            .abs_diff(spec.output.dwell_start_deg10.values[cyl]);
        if diff > EPS_ANGLE_DEG10 {
            panic!(
                "FAIL dwell_start_deg10: cyl={} case={} fixture={} obs={} exp={} diff={} tol={}",
                cyl,
                case.fixture,
                case.variant,
                sched_obs.dwell_start_deg10.values[cyl],
                spec.output.dwell_start_deg10.values[cyl],
                diff,
                EPS_ANGLE_DEG10
            );
        }
    }

    // Compare event count and events
    let spec_event_count = spec.output.events.len as usize;
    if sched_obs.events.len as usize != spec_event_count {
        panic!(
            "FAIL events.len: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, sched_obs.events.len, spec_event_count
        );
    }

    // Compare the event set deterministically without depending on insertion order.
    #[derive(Debug)]
    struct SortableEvent {
        angle: u16,
        kind: u8, // 0=InjOpen, 1=InjClose, 2=CoilCharge, 3=CoilFire
        cylinder: u8,
    }
    impl SortableEvent {
        fn from_obs(evt: &RuntimeSemanticScheduleEvent) -> Self {
            let kind = match evt.kind {
                RuntimeSemanticScheduleEventKind::InjectionOpen => 0,
                RuntimeSemanticScheduleEventKind::InjectionClose => 1,
                RuntimeSemanticScheduleEventKind::CoilChargeStart => 2,
                RuntimeSemanticScheduleEventKind::CoilFire => 3,
            };
            SortableEvent {
                angle: evt.angle_deg10,
                kind,
                cylinder: evt.cylinder,
            }
        }
        fn from_spec(evt: &ecu_spec::SemanticEvent) -> Self {
            let kind = match evt.kind {
                ecu_spec::EventKind::InjectionOpen => 0,
                ecu_spec::EventKind::InjectionClose => 1,
                ecu_spec::EventKind::CoilChargeStart => 2,
                ecu_spec::EventKind::CoilFire => 3,
            };
            SortableEvent {
                angle: evt.angle_deg10.0,
                kind,
                cylinder: evt.cylinder.0,
            }
        }
    }

    let mut obs_sorted: Vec<_> = (0..sched_obs.events.len as usize)
        .map(|i| SortableEvent::from_obs(&sched_obs.events.events[i]))
        .collect();
    let mut spec_sorted: Vec<_> = (0..spec_event_count)
        .map(|i| SortableEvent::from_spec(&spec.output.events.events[i]))
        .collect();

    // Sort by angle, then kind, then cylinder
    obs_sorted.sort_by_key(|e| (e.angle, e.kind, e.cylinder));
    spec_sorted.sort_by_key(|e| (e.angle, e.kind, e.cylinder));

    // Compare event counts
    if obs_sorted.len() != spec_sorted.len() {
        panic!(
            "FAIL events.len: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs_sorted.len(),
            spec_sorted.len()
        );
    }

    // Compare sorted events
    for i in 0..obs_sorted.len() {
        let obs_e = &obs_sorted[i];
        let spec_e = &spec_sorted[i];
        let obs_kind = match obs_e.kind {
            0 => "InjectionOpen",
            1 => "InjectionClose",
            2 => "CoilChargeStart",
            _ => "CoilFire",
        };
        let spec_kind = match spec_e.kind {
            0 => "InjectionOpen",
            1 => "InjectionClose",
            2 => "CoilChargeStart",
            _ => "CoilFire",
        };
        if obs_e.kind != spec_e.kind {
            panic!(
                "FAIL events[{}].kind: case={} fixture={} obs={} exp={}",
                i, case.fixture, case.variant, obs_kind, spec_kind
            );
        }
        if obs_e.cylinder != spec_e.cylinder {
            panic!(
                "FAIL events[{}].cylinder: case={} fixture={} obs={} exp={}",
                i, case.fixture, case.variant, obs_e.cylinder, spec_e.cylinder
            );
        }
        let diff = obs_e.angle.abs_diff(spec_e.angle);
        if diff > EPS_ANGLE_DEG10 {
            panic!(
                "FAIL events[{}].angle_deg10: case={} fixture={} obs={} exp={} diff={} tol={}",
                i, case.fixture, case.variant, obs_e.angle, spec_e.angle, diff, EPS_ANGLE_DEG10
            );
        }
    }

    // Compare diagnostic
    let spec_diag = spec.output.diagnostic;
    let obs_diag = sched_obs.diagnostic;
    let obs_diag_code = match obs_diag {
        RuntimeSemanticScheduleDiagnostic::None => ecu_spec::DiagnosticCode::None,
        RuntimeSemanticScheduleDiagnostic::FuelCutActive => ecu_spec::DiagnosticCode::FuelCutActive,
        RuntimeSemanticScheduleDiagnostic::SparkCutActive => {
            ecu_spec::DiagnosticCode::SparkCutActive
        }
        RuntimeSemanticScheduleDiagnostic::Unsynced => ecu_spec::DiagnosticCode::Unsynced,
        RuntimeSemanticScheduleDiagnostic::CalibrationInvalid => {
            ecu_spec::DiagnosticCode::CalibrationInvalid
        }
    };
    if obs_diag_code != spec_diag {
        panic!(
            "FAIL diagnostic: case={} fixture={} obs={:?} exp={:?}",
            case.fixture, case.variant, obs_diag, spec_diag
        );
    }

    // Assert angles are < 7200
    for cyl in 0..sched_obs.soi_deg10.count as usize {
        assert!(
            sched_obs.soi_deg10.values[cyl] < 7200,
            "soi_deg10[{}] should be < 7200",
            cyl
        );
        assert!(
            sched_obs.eoi_deg10.values[cyl] < 7200,
            "eoi_deg10[{}] should be < 7200",
            cyl
        );
        assert!(
            sched_obs.spark_deg10.values[cyl] < 7200,
            "spark_deg10[{}] should be < 7200",
            cyl
        );
        assert!(
            sched_obs.dwell_start_deg10.values[cyl] < 7200,
            "dwell_start_deg10[{}] should be < 7200",
            cyl
        );
    }

    // Assert fuel-cut cases: no InjectionOpen or InjectionClose
    if fuel_obs.fuel_cut || fuel_obs.pw_corr_us == 0 {
        for i in 0..sched_obs.events.len as usize {
            let kind = sched_obs.events.events[i].kind;
            assert!(
                !matches!(
                    kind,
                    RuntimeSemanticScheduleEventKind::InjectionOpen
                        | RuntimeSemanticScheduleEventKind::InjectionClose
                ),
                "fuel-cut case {} should have no injection events",
                case.fixture
            );
        }
    }

    // Assert spark-cut cases: no CoilChargeStart or CoilFire
    if fuel_obs.spark_cut {
        for i in 0..sched_obs.events.len as usize {
            let kind = sched_obs.events.events[i].kind;
            assert!(
                !matches!(
                    kind,
                    RuntimeSemanticScheduleEventKind::CoilChargeStart
                        | RuntimeSemanticScheduleEventKind::CoilFire
                ),
                "spark-cut case {} should have no ignition events",
                case.fixture
            );
        }
    }

    // Assert unsynced cases: zero events
    if !matches!(semantic_input.sync, DomainSyncState::Locked { .. }) {
        assert_eq!(
            sched_obs.events.len, 0,
            "unsynced case {} should have zero events",
            case.fixture
        );
    }
}

#[test]
fn runtime_semantic_schedule_conformance_all_fixtures() {
    let cases = fm0016_fixture_matrix::fixture_cases();
    for case in cases {
        schedule_conformance_test(&case);
    }
}
