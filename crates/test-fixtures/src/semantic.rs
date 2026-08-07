//! Semantic calibration builders shared by the ecu-runtime and ecu-scheduler
//! FM0016 conformance tests (single canonical copy).

use ecu_domain::{Kpa10, Lambda100, Micros, Rpm, SyncState as DomainSyncState};
use ecu_runtime::semantic::{
    RuntimeSemanticAfrOverride, RuntimeSemanticAxis16, RuntimeSemanticCalibration,
    RuntimeSemanticCurve16U16, RuntimeSemanticCylinderArrayU16, RuntimeSemanticDeadtimeTableU16,
    RuntimeSemanticEngineMode, RuntimeSemanticInjectionAngleMode, RuntimeSemanticInputSnapshot,
    RuntimeSemanticScheduleCalibration, RuntimeSemanticState, RuntimeSemanticTable2dI16,
    RuntimeSemanticTable2dU16, RuntimeSemanticTable2dU32,
};
use ecu_spec::{
    AfrOverride, CylinderArrayU16, EngineMode, InjectionAngleMode, InputSnapshot, SyncState,
    ValidatedCalibration,
};

use crate::fixture_matrix::FixtureCase;

/// Convert a ValidatedCalibration to RuntimeSemanticCalibration for the
/// v9 semantic evaluator.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
pub fn build_semantic_calibration(cal: &ValidatedCalibration) -> RuntimeSemanticCalibration {
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
    fn copy_deadtime_table(src: &ecu_spec::Table2D16<u16>) -> RuntimeSemanticDeadtimeTableU16 {
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

        RuntimeSemanticDeadtimeTableU16 {
            vbat_mv_axis: rpm_axis,
            pressure_kpa10_axis: load_axis,
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
        deadtime_table_us: copy_deadtime_table(&c.deadtime_table_us),
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
        dfco_entry_rpm: c.dfco_entry_rpm.get(),
        dfco_exit_rpm: c.dfco_exit_rpm.get(),
        dfco_entry_tps_x100: c.dfco_entry_tps_x100,
        dfco_exit_tps_x100: c.dfco_exit_tps_x100,
        dfco_entry_map_kpa10: c.dfco_entry_map_kpa10.get(),
        dfco_delay_cycles: c.dfco_delay_cycles,
        soft_rev_rpm: c.soft_rev_rpm.get(),
        hard_rev_rpm: c.hard_rev_rpm.get(),
        rev_hysteresis_rpm: c.rev_hysteresis_rpm.get(),
        soft_retard_max_deg10: c.soft_retard_max_deg10,
        idle_target_rpm: c.idle_target_rpm.get(),
        idle_base_duty_x1000: c.idle_base_duty_x1000,
        idle_kp_x1000: c.idle_kp_x1000,
        idle_ki_x1000: c.idle_ki_x1000,
        launch_rpm_limit: c.launch_rpm_limit.get(),
        launch_cut_cycles: c.launch_cut_cycles,
        flat_shift_rpm_min: c.flat_shift_rpm_min.get(),
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

/// Convert a ValidatedCalibration to RuntimeSemanticScheduleCalibration for
/// the v10 semantic schedule evaluator.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
pub fn build_semantic_schedule_calibration(
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
pub fn to_semantic_input(input: &InputSnapshot) -> RuntimeSemanticInputSnapshot {
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
        t_us: Micros::new(input.t_us.get()),
        rpm: Rpm::new(input.rpm.get()),
        map_kpa10: Kpa10::new(input.map_kpa10.get()),
        load_kpa10: Kpa10::new(input.load_kpa10.get()),
        tps_x100: input.tps_x100,
        clt_c10: input.clt_c10.get(),
        iat_c10: input.iat_c10.get(),
        baro_kpa10: Kpa10::new(input.baro_kpa10.get()),
        vbatt_mv: input.vbatt_mv.get(),
        lambda_valid: true,
        lambda_measured: Lambda100::new(100),
        requested_open_loop: false,
        knock_intensity_x100: input.knock_intensity_x100,
        launch_armed: input.launch_armed,
        flat_shift_armed: input.flat_shift_armed,
        sync: match input.sync {
            SyncState::Synced => DomainSyncState::Locked { cam_ref: false },
            SyncState::Unsynced => DomainSyncState::Unsynced,
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

/// Fixture-specific semantic runtime state (mirrors `fixture_state` on the
/// spec side so both evaluators start from the same latch/accumulator state).
pub fn semantic_state_for_fixture(case: &FixtureCase) -> RuntimeSemanticState {
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
