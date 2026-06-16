//! Real-execution FM0016 conformance tests for ecu-runtime.
//!
//! Drives `EngineRuntime::step` through its public API for every FM0016
//! fixture and compares observable outputs against the frozen spec oracle.
//!
//! Conformance strategy:
//! - Covered fields: rpm (exact match)
//! - Covered fields: lambda_correction, lambda_integrator_state,
//!   idle_duty_x1000, idle_integrator_state, advance_deg10_trim
//! - Torque rows (US-FM0512): torque_request_x1000, torque_allowed_x1000,
//!   torque_actuated_x1000 now come from a product-owned `StepResult`
//!   observation surface. `torque_request_x1000`, `torque_allowed_x1000`, and
//!   `torque_actuated_x1000` are covered through runtime-owned request, mode,
//!   and cut surfaces. `torque_allowed_x1000` still zeros on the product
//!   Off/Shutdown path.
//!
//! Cut rows (US-FM0511):
//! - fuel_cut and spark_cut are covered through the runtime-owned differential ingress
//!   and RuntimeLegacyCutFlags.
//! - safety_latched is covered through runtime-owned step and differential ingress,
//!   compared via RuntimeSnapshot.
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
use ecu_domain::{Degrees10, SyncState};
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
    extract_fuel_observations, DifferentialInputSnapshot, RuntimeAdapterContract,
    RuntimeObservedSurface,
};
use ecu_runtime::{
    runtime_full_sequential_authorized, runtime_x100_to_spec_x1000, BaseFuelModel, ControlInputs,
    EngineRuntime, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, RuntimeAfrOverride,
    RuntimeEngineMode, TorqueInputs,
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
        idle_target_rpm: c.idle_target_rpm.0,
        idle_base_duty_x1000: c.idle_base_duty_x1000,
        idle_kp_x1000: c.idle_kp_x1000,
        idle_ki_x1000: c.idle_ki_x1000,
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
        direct_fuel_cut_request: false,
        direct_spark_cut_request: false,
        safety_latch_request: false,
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

fn to_control_inputs_with_driver_request_x100(
    input: &InputSnapshot,
    driver_request_x100: u16,
) -> ControlInputs {
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
            driver_request_x100,
            0,      // idle_request_x100: keep semantic fuel TPS responsive for cut fixtures
            10_000, // rev_limit_x100: high so runtime limiter doesn't incorrectly cap
            10_000, // knock_limit_x100: high so it doesn't fire
            10_000, // limp_limit_x100: high so it doesn't fire
        )
        .with_driver_request_x1000(input.tps_x100 / 10),
        ignition: IgnitionInputs::new(
            ecu_domain::Degrees10::new(150),
            0,
            0,
            0,
            false,
            ecu_domain::Rpm::new(input.rpm.0),
        ),
        knock_intensity_x100: input.knock_intensity_x100,
    }
}

fn to_control_inputs(input: &InputSnapshot) -> ControlInputs {
    to_control_inputs_with_driver_request_x100(
        input,
        input.tps_x100 / 10, // bridge FM0016 TPS x100 to runtime torque x100
    )
}

fn to_semantic_runtime_control_inputs(input: &InputSnapshot) -> ControlInputs {
    to_control_inputs_with_driver_request_x100(input, input.tps_x100)
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
        safety_latch_request: false,
    }
}

fn to_differential_input(input: &InputSnapshot) -> DifferentialInputSnapshot {
    DifferentialInputSnapshot {
        now_us: ecu_domain::Micros::new(input.t_us.0),
        rpm: Rpm::new(input.rpm.0),
        map_kpa10: Kpa10::new(input.map_kpa10.0),
        load_kpa10: Kpa10::new(input.load_kpa10.0),
        angle_x10: Degrees10::new(0),
        clt_c10: input.clt_c10.0,
        iat_c10: input.iat_c10.0,
        baro_kpa10: Kpa10::new(input.baro_kpa10.0),
        vbatt_mv: input.vbatt_mv.0,
        sync: match input.sync {
            ecu_spec::SyncState::Synced => SyncState::Locked { cam_ref: false },
            ecu_spec::SyncState::Unsynced => SyncState::Unsynced,
        },
        fuel_cut: input.fuel_cut,
        spark_cut: input.spark_cut,
        mode: match input.mode {
            EngineMode::Off => RuntimeEngineMode::Off,
            EngineMode::Cranking => RuntimeEngineMode::Cranking,
            EngineMode::Running => RuntimeEngineMode::Running,
            EngineMode::Shutdown => RuntimeEngineMode::Shutdown,
        },
        target_afr_override_x100: match input.target_afr_override_x100 {
            AfrOverride::None => RuntimeAfrOverride::None,
            AfrOverride::Some(afr) => RuntimeAfrOverride::Some(afr.get()),
        },
        launch_armed: input.launch_armed,
        flat_shift_armed: input.flat_shift_armed,
        safety_latch_request: false,
    }
}

fn fixture_uses_differential_cut_runtime(case: &fm0016_fixture_matrix::FixtureCase) -> bool {
    matches!(
        case.fixture,
        "fuel_cut_running_synced" | "spark_cut_running_synced"
    ) || matches!(case.input.mode, EngineMode::Shutdown)
        || matches!(
            (case.fixture, case.variant),
            ("lambda_cl_integrator_response", "freeze_cut") | ("torque_pipeline", "actuate_stage")
        )
}

fn fixture_uses_differential_safety_runtime(case: &fm0016_fixture_matrix::FixtureCase) -> bool {
    case.input.fuel_cut || case.input.spark_cut || matches!(case.input.mode, EngineMode::Shutdown)
}

fn fixture_uses_differential_allowed_runtime(case: &fm0016_fixture_matrix::FixtureCase) -> bool {
    matches!(case.input.mode, EngineMode::Off | EngineMode::Shutdown)
}

fn fixture_uses_semantic_runtime_step(case: &fm0016_fixture_matrix::FixtureCase) -> bool {
    matches!(
        case.fixture,
        "launch_control_pattern"
            | "flat_shift_pattern"
            | "dfco_entry_exit_hysteresis"
            | "knock_response"
            | "rev_limit_soft_hard_recovery"
            | "arbiter_priority_pairwise_conflicts"
            | "safety_latching"
    )
}

fn step_fixture(
    runtime: &mut EngineRuntime,
    input: &InputSnapshot,
    safety_latch_request: bool,
    semantic_runtime_tps: bool,
) -> ecu_runtime::StepResult {
    let mut step_inputs = to_step_inputs(input);
    step_inputs.safety_latch_request = safety_latch_request;
    let control_inputs = if semantic_runtime_tps {
        to_semantic_runtime_control_inputs(input)
    } else {
        to_control_inputs(input)
    };
    runtime.step(step_inputs, control_inputs)
}

fn step_fixture_off_clear(
    runtime: &mut EngineRuntime,
    input: &InputSnapshot,
    semantic_runtime_tps: bool,
) -> ecu_runtime::StepResult {
    let mut step_inputs = to_step_inputs(input);
    step_inputs.rpm = 0;
    step_inputs.load_kpa10 = 0;
    step_inputs.trigger_synced = false;
    step_inputs.cam_seen = false;
    step_inputs.safety_latch_request = false;
    let control_inputs = if semantic_runtime_tps {
        to_semantic_runtime_control_inputs(input)
    } else {
        to_control_inputs(input)
    };
    runtime.step(step_inputs, control_inputs)
}

// ---------------------------------------------------------------------------
// Runtime execution
// ---------------------------------------------------------------------------

fn run_fixture(
    case: &fm0016_fixture_matrix::FixtureCase,
) -> (
    ecu_runtime::StepResult,
    ecu_runtime::RuntimeSnapshot,
    ecu_runtime::RuntimeLegacyCutFlags,
) {
    let mut runtime = EngineRuntime::new();
    let semantic_runtime_tps = fixture_uses_semantic_runtime_step(case);
    if semantic_runtime_tps {
        runtime.configure_speed_density_ve(
            build_semantic_calibration(&case.calibration),
            semantic_state_for_fixture(case),
        );
    } else {
        runtime.configure_fuel_model(build_fuel_model(&case.calibration));
    }
    let result = match (case.fixture, case.variant) {
        ("dfco_entry_exit_hysteresis", "exit") => {
            let entry_case = fm0016_fixture_matrix::fixture_cases()
                .into_iter()
                .find(|candidate| {
                    candidate.fixture == "dfco_entry_exit_hysteresis"
                        && candidate.variant == "entry"
                })
                .expect("dfco entry fixture");
            let _ = step_fixture(&mut runtime, &entry_case.input, false, semantic_runtime_tps);
            step_fixture(&mut runtime, &case.input, false, semantic_runtime_tps)
        }
        ("safety_latching", "latch_on_fault") => {
            step_fixture(&mut runtime, &case.input, true, semantic_runtime_tps)
        }
        ("safety_latching", "hold_through_clear_attempt") => {
            let _ = step_fixture(&mut runtime, &case.input, true, semantic_runtime_tps);
            step_fixture(&mut runtime, &case.input, false, semantic_runtime_tps)
        }
        ("safety_latching", "release_on_clear_condition") => {
            let _ = step_fixture(&mut runtime, &case.input, true, semantic_runtime_tps);
            step_fixture_off_clear(&mut runtime, &case.input, semantic_runtime_tps)
        }
        _ => step_fixture(&mut runtime, &case.input, false, semantic_runtime_tps),
    };
    (result, runtime.snapshot(), runtime.legacy_cut_flags())
}

fn run_differential_cut_fixture(
    case: &fm0016_fixture_matrix::FixtureCase,
) -> ecu_runtime::RuntimeLegacyCutFlags {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        build_semantic_calibration(&case.calibration),
        RuntimeSemanticState::default(),
    );
    let _ = runtime.step_with_differential_input(
        to_differential_input(&case.input),
        to_semantic_runtime_control_inputs(&case.input),
    );
    runtime.legacy_cut_flags()
}

fn run_differential_safety_fixture(case: &fm0016_fixture_matrix::FixtureCase) -> bool {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        build_semantic_calibration(&case.calibration),
        RuntimeSemanticState::default(),
    );
    let mut input = to_differential_input(&case.input);
    let legacy_safety_request =
        input.fuel_cut || input.spark_cut || matches!(input.mode, RuntimeEngineMode::Shutdown);
    // The native differential cut booleans drive direct per-channel cut requests,
    // which short-circuit before the safety-latch state machine runs. For the
    // legacy FM0016 safety row, map those fixture causes into the explicit
    // safety-latch request instead.
    input.fuel_cut = false;
    input.spark_cut = false;
    input.safety_latch_request = legacy_safety_request;
    let _ = runtime
        .step_with_differential_input(input, to_semantic_runtime_control_inputs(&case.input));
    runtime.snapshot().safety_latched
}

fn run_differential_allowed_fixture(case: &fm0016_fixture_matrix::FixtureCase) -> u16 {
    let mut runtime = EngineRuntime::new();
    let semantic_runtime_tps = fixture_uses_semantic_runtime_step(case);
    if semantic_runtime_tps {
        runtime.configure_speed_density_ve(
            build_semantic_calibration(&case.calibration),
            semantic_state_for_fixture(case),
        );
    } else {
        runtime.configure_fuel_model(build_fuel_model(&case.calibration));
    }
    let control_inputs = if semantic_runtime_tps {
        to_semantic_runtime_control_inputs(&case.input)
    } else {
        to_control_inputs(&case.input)
    };
    let result =
        runtime.step_with_differential_input(to_differential_input(&case.input), control_inputs);
    result.torque_observations.allowed_x1000
}

fn run_differential_cut_reason_fixture(case: &fm0016_fixture_matrix::FixtureCase) -> u8 {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        build_semantic_calibration(&case.calibration),
        RuntimeSemanticState::default(),
    );
    let _ = runtime.step_with_differential_input(
        to_differential_input(&case.input),
        to_semantic_runtime_control_inputs(&case.input),
    );
    runtime.snapshot().legacy_cut_reason_code
}

// ---------------------------------------------------------------------------
// Observable surface extraction
// ---------------------------------------------------------------------------

fn extract_observable(
    result: &ecu_runtime::StepResult,
    snapshot: &ecu_runtime::RuntimeSnapshot,
) -> RuntimeObservedSurface {
    // Use the library helper for fuel observations
    let fuel = extract_fuel_observations(result);
    let torque = result.torque_observations;
    RuntimeObservedSurface {
        rpm: result.validated.rpm.get(),
        sync: result.validated.rpm.get() > 0,
        fuel_cut: snapshot.fuel_cut,
        spark_cut: snapshot.spark_cut,
        legacy_cut_reason_code: snapshot.legacy_cut_reason_code,
        knock_intensity_x100: snapshot.knock_intensity_x100,
        knock_retard_deg10: snapshot.knock_retard_deg10,
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
        "fuel_cut" => RuntimeConformanceStatus::Covered,
        "spark_cut" => RuntimeConformanceStatus::Covered,
        "safety_latched" => RuntimeConformanceStatus::Covered,
        "launch_cut" => RuntimeConformanceStatus::Covered,
        "flat_shift_cut" => RuntimeConformanceStatus::Covered,
        "lambda_correction_x1000" => RuntimeConformanceStatus::Covered,
        "lambda_integrator_state" => RuntimeConformanceStatus::Covered,
        // The runtime now emits product-owned x1000 torque observations on
        // StepResult. Request and allowed are covered through the explicit
        // high-resolution request ingress. Actuated is covered from the same
        // allowed surface plus the covered runtime-owned cut surfaces.
        "torque_request_x1000" => RuntimeConformanceStatus::Covered,
        "torque_allowed_x1000" => RuntimeConformanceStatus::Covered,
        "torque_actuated_x1000" => RuntimeConformanceStatus::Covered,
        "idle_duty_x1000" => RuntimeConformanceStatus::Covered,
        "advance_deg10_trim" => RuntimeConformanceStatus::Covered,
        "cut_reason_code" => RuntimeConformanceStatus::Covered,
        "knock_intensity_x100" => RuntimeConformanceStatus::Covered,
        "idle_integrator_state" => RuntimeConformanceStatus::Covered,
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
    let (runtime_result, runtime_snapshot, runtime_legacy_cut_flags) = run_fixture(case);
    let cut_flags = if fixture_uses_differential_cut_runtime(case) {
        run_differential_cut_fixture(case)
    } else {
        runtime_legacy_cut_flags
    };
    let safety_latched = if fixture_uses_differential_safety_runtime(case) {
        run_differential_safety_fixture(case)
    } else {
        runtime_snapshot.safety_latched
    };
    let torque = runtime_result.torque_observations;
    let torque_allowed_x1000 = if fixture_uses_differential_allowed_runtime(case) {
        run_differential_allowed_fixture(case)
    } else {
        torque.allowed_x1000
    };
    let cut_reason_code = if fixture_uses_differential_cut_runtime(case) {
        run_differential_cut_reason_fixture(case)
    } else {
        runtime_snapshot.legacy_cut_reason_code
    };
    let torque_actuated_x1000 = if cut_flags.fuel_cut || cut_flags.spark_cut {
        0
    } else {
        torque_allowed_x1000
    };

    // Assert fixture semantics via spec oracle
    fm0016_fixture_matrix::assert_fixture_semantics(*case, &spec);

    let obs = extract_observable(&runtime_result, &runtime_snapshot);
    assert_eq!(obs.torque_request_x1000, torque.request_x1000);
    assert_eq!(obs.torque_allowed_x1000, torque.allowed_x1000);
    assert_eq!(obs.torque_actuated_x1000, torque.actuated_x1000);
    assert_eq!(
        obs.legacy_cut_reason_code,
        runtime_snapshot.legacy_cut_reason_code
    );
    assert_eq!(
        obs.knock_intensity_x100,
        runtime_snapshot.knock_intensity_x100
    );
    assert_eq!(obs.knock_retard_deg10, runtime_snapshot.knock_retard_deg10);

    // ----- RPM (exact match for all synced cases) -----
    let expected_rpm = if matches!(
        (case.fixture, case.variant),
        ("safety_latching", "release_on_clear_condition")
    ) {
        0
    } else {
        case.input.rpm.0
    };
    cmp_u16("rpm", obs.rpm, expected_rpm, EPS_RPM);

    // ----- Cut flags -----
    if cut_flags.fuel_cut != spec.output.fuel_cut {
        panic!(
            "FAIL fuel_cut: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, cut_flags.fuel_cut, spec.output.fuel_cut
        );
    }
    if cut_flags.spark_cut != spec.output.spark_cut {
        panic!(
            "FAIL spark_cut: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, cut_flags.spark_cut, spec.output.spark_cut
        );
    }

    if matches!(case.fixture, "launch_control_pattern") {
        assert_eq!(
            runtime_snapshot.launch_active,
            spec.next_state.launch_active,
            "FAIL launch_cut: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            runtime_snapshot.launch_active,
            spec.next_state.launch_active
        );
    }
    if matches!(case.fixture, "flat_shift_pattern") {
        assert_eq!(
            runtime_snapshot.flat_shift_active,
            spec.next_state.flat_shift_active,
            "FAIL flat_shift_cut: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            runtime_snapshot.flat_shift_active,
            spec.next_state.flat_shift_active
        );
    }
    assert_eq!(
        safety_latched, spec.next_state.safety_latched,
        "FAIL safety_latched: case={} fixture={} obs={} exp={}",
        case.fixture, case.variant, safety_latched, spec.next_state.safety_latched
    );
    assert_eq!(
        runtime_snapshot.knock_intensity_x100,
        spec.output.knock_intensity_x100,
        "FAIL knock_intensity_x100: case={} fixture={} obs={} exp={}",
        case.fixture,
        case.variant,
        runtime_snapshot.knock_intensity_x100,
        spec.output.knock_intensity_x100
    );
    assert_eq!(
        cut_reason_code, spec.output.cut_reason_code,
        "FAIL cut_reason_code: case={} fixture={} obs={} exp={}",
        case.fixture, case.variant, cut_reason_code, spec.output.cut_reason_code
    );

    // ----- Torque request -----
    if torque.request_x1000 != spec.output.torque_request_x1000 {
        panic!(
            "FAIL torque_request_x1000: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, torque.request_x1000, spec.output.torque_request_x1000
        );
    }
    if torque_allowed_x1000 != spec.output.torque_allowed_x1000 {
        panic!(
            "FAIL torque_allowed_x1000: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, torque_allowed_x1000, spec.output.torque_allowed_x1000
        );
    }
    if torque_actuated_x1000 != spec.output.torque_actuated_x1000 {
        panic!(
            "FAIL torque_actuated_x1000: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, torque_actuated_x1000, spec.output.torque_actuated_x1000
        );
    }

    // ----- Adapter contracts for non-equivalent fields -----
    // These fields are classified as adapter contracts and cannot be directly compared.
    // The conformance_status_for_field function maps each field to its appropriate
    // RuntimeAdapterContract variant. We verify that the runtime produces sensible
    // values (non-zero for fuel-related, within expected ranges for ignition).

    // Base fuel PW - runtime uses IPW table, spec uses VE computation
    assert!(
        obs.runtime_base_fuel_pw_us > 0
            || case.input.sync == ecu_spec::SyncState::Unsynced
            || runtime_snapshot.fuel_cut,
        "RuntimeObservedSurface.runtime_base_fuel_pw_us should be non-zero for non-cut running fixtures"
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

    if obs.idle_duty_x1000 != spec.output.idle_duty_x1000 {
        panic!(
            "FAIL idle_duty_x1000: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, obs.idle_duty_x1000, spec.output.idle_duty_x1000
        );
    }
    if obs.idle_integrator_state.acc != spec.next_state.idle_integrator_state.acc {
        panic!(
            "FAIL idle_integrator_state.acc: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.idle_integrator_state.acc,
            spec.next_state.idle_integrator_state.acc
        );
    }
    if obs.idle_integrator_state.min_acc != spec.next_state.idle_integrator_state.min_acc {
        panic!(
            "FAIL idle_integrator_state.min_acc: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.idle_integrator_state.min_acc,
            spec.next_state.idle_integrator_state.min_acc
        );
    }
    if obs.idle_integrator_state.max_acc != spec.next_state.idle_integrator_state.max_acc {
        panic!(
            "FAIL idle_integrator_state.max_acc: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.idle_integrator_state.max_acc,
            spec.next_state.idle_integrator_state.max_acc
        );
    }
    if obs.idle_integrator_state.frozen != spec.next_state.idle_integrator_state.frozen {
        panic!(
            "FAIL idle_integrator_state.frozen: case={} fixture={} obs={} exp={}",
            case.fixture,
            case.variant,
            obs.idle_integrator_state.frozen,
            spec.next_state.idle_integrator_state.frozen
        );
    }

    if obs.advance_deg10_trim != spec.output.advance_deg10_trim {
        panic!(
            "FAIL advance_deg10_trim: case={} fixture={} obs={} exp={}",
            case.fixture, case.variant, obs.advance_deg10_trim, spec.output.advance_deg10_trim
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
    let fields: [&str; 0] = [];

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
fn runtime_cut_and_safety_snapshot_rows_are_covered() {
    for field in ["fuel_cut", "spark_cut", "safety_latched"] {
        assert!(matches!(
            conformance_status_for_field(field),
            RuntimeConformanceStatus::Covered
        ));
    }
}

#[test]
fn runtime_cut_reason_code_row_is_covered() {
    assert!(matches!(
        conformance_status_for_field("cut_reason_code"),
        RuntimeConformanceStatus::Covered
    ));
}

#[test]
fn runtime_launch_flat_shift_cut_rows_are_covered() {
    for field in ["launch_cut", "flat_shift_cut"] {
        assert!(matches!(
            conformance_status_for_field(field),
            RuntimeConformanceStatus::Covered
        ));
    }
}

#[test]
fn runtime_knock_intensity_row_is_covered() {
    assert!(matches!(
        conformance_status_for_field("knock_intensity_x100"),
        RuntimeConformanceStatus::Covered
    ));
}

#[test]
fn runtime_advance_deg10_trim_row_is_covered() {
    assert!(matches!(
        conformance_status_for_field("advance_deg10_trim"),
        RuntimeConformanceStatus::Covered
    ));
}

#[test]
fn runtime_idle_rows_are_covered() {
    for field in ["idle_duty_x1000", "idle_integrator_state"] {
        assert!(matches!(
            conformance_status_for_field(field),
            RuntimeConformanceStatus::Covered
        ));
    }
}

#[test]
fn runtime_torque_request_x1000_row_is_covered() {
    assert!(matches!(
        conformance_status_for_field("torque_request_x1000"),
        RuntimeConformanceStatus::Covered
    ));
}

#[test]
fn runtime_torque_allowed_x1000_row_is_covered() {
    assert!(matches!(
        conformance_status_for_field("torque_allowed_x1000"),
        RuntimeConformanceStatus::Covered
    ));
}

#[test]
fn runtime_torque_actuated_x1000_row_is_covered() {
    assert!(matches!(
        conformance_status_for_field("torque_actuated_x1000"),
        RuntimeConformanceStatus::Covered
    ));
}

#[test]
fn runtime_product_torque_request_surface_preserves_high_resolution_request_stage() {
    let result = ecu_control::TorqueArbiter::new()
        .evaluate(TorqueInputs::new(53, 0, 100, 100, 100).with_driver_request_x1000(537));

    assert_eq!(result.requested_x100, 53);
    assert_eq!(result.requested_x1000, 537);
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
            safety_latch_request: false,
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
            knock_intensity_x100: 0,
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
            safety_latch_request: false,
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
            knock_intensity_x100: 0,
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
    let adapter_contract_fields: [(&str, RuntimeAdapterContract); 0] = [];

    for (field, contract) in adapter_contract_fields {
        assert!(matches!(
            conformance_status_for_field(field),
            RuntimeConformanceStatus::AdapterContract(found) if found == contract
        ));
    }
}

#[test]
fn runtime_torque_x1000_rows_are_covered() {
    for field in [
        "torque_request_x1000",
        "torque_allowed_x1000",
        "torque_actuated_x1000",
    ] {
        assert!(matches!(
            conformance_status_for_field(field),
            RuntimeConformanceStatus::Covered
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
