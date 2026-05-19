//! Real-execution FM0016 conformance tests for ecu-scheduler.
//!
//! Drives `SchedulerState` through its public API for every FM0016 fixture
//! and verifies that the scheduler is driven with the correct product runtime
//! fuel and schedule observations.
//!
//! Key insight: the scheduler owns the deadline/channel/event-order logic.
//! It does NOT own the angle computation - that comes from runtime's fuel/ignition
//! planning. The product runtime semantic evaluators (runtime_semantic_evaluate_fuel
//! and runtime_semantic_evaluate_schedule) provide the authoritative product observations.
//!
//! This test:
//!   1. Builds product fuel observations via runtime_semantic_evaluate_fuel
//!   2. Builds product schedule observations via runtime_semantic_evaluate_schedule
//!   3. Drives SchedulerState with the product events
//!   4. Verifies scheduler counts and active groups match product event presence
//!   5. Keeps cancellation/suspend assertions

#![cfg(test)]

use ecu_domain::{Degrees10, DwellUs, Micros, PulseWidthUs, Rpm};
use ecu_scheduler::{
    ChannelId, ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup, SchedulerMode,
    SchedulerObservedSurface, SchedulerState,
};

mod fm0016_fixture_matrix {
    #![allow(dead_code)]
    include!("../../tests/formal/fm0016_fixture_matrix.rs");
}

// Re-export runtime semantic types needed for product observation building
use ecu_runtime::{
    runtime_semantic_evaluate_fuel, runtime_semantic_evaluate_schedule, RuntimeSemanticAfrOverride,
    RuntimeSemanticAxis16, RuntimeSemanticCalibration, RuntimeSemanticCurve16U16,
    RuntimeSemanticCylinderArrayU16, RuntimeSemanticEngineMode, RuntimeSemanticFuelObservations,
    RuntimeSemanticInjectionAngleMode, RuntimeSemanticInputSnapshot,
    RuntimeSemanticScheduleCalibration, RuntimeSemanticScheduleEventKind, RuntimeSemanticState,
    RuntimeSemanticTable2dI16, RuntimeSemanticTable2dU16, RuntimeSemanticTable2dU32,
};

// --------------------------------------------------------------------------
// Calibration builders (copied from ecu-runtime/tests/fm0016_runtime_conformance.rs)
// --------------------------------------------------------------------------

/// Convert a ValidatedCalibration to RuntimeSemanticCalibration for the
/// v9 semantic evaluator.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
fn build_semantic_calibration(cal: &ecu_spec::ValidatedCalibration) -> RuntimeSemanticCalibration {
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
        deadtime_table_us: copy_table_2d(&c.deadtime_table_us),
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

/// Convert a ValidatedCalibration to RuntimeSemanticScheduleCalibration for
/// the v10 semantic schedule evaluator.
#[allow(clippy::indexing_slicing, clippy::needless_range_loop)]
fn build_semantic_schedule_calibration(
    cal: &ecu_spec::ValidatedCalibration,
) -> RuntimeSemanticScheduleCalibration {
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

    fn copy_cylinder_array(src: &ecu_spec::CylinderArrayU16) -> RuntimeSemanticCylinderArrayU16 {
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
            ecu_spec::InjectionAngleMode::StartOfInjection => {
                RuntimeSemanticInjectionAngleMode::StartOfInjection
            }
            ecu_spec::InjectionAngleMode::EndOfInjection => {
                RuntimeSemanticInjectionAngleMode::EndOfInjection
            }
        },
        cylinder_phase_deg10: copy_cylinder_array(&c.cylinder_phase_deg10),
    }
}

/// Convert an InputSnapshot to RuntimeSemanticInputSnapshot for the v9
/// semantic evaluator.
fn to_semantic_input(input: &ecu_spec::InputSnapshot) -> RuntimeSemanticInputSnapshot {
    use ecu_domain::Kpa10;
    use ecu_spec::EngineMode;

    let mode = match input.mode {
        EngineMode::Off => RuntimeSemanticEngineMode::Off,
        EngineMode::Cranking => RuntimeSemanticEngineMode::Cranking,
        EngineMode::Running => RuntimeSemanticEngineMode::Running,
        EngineMode::Shutdown => RuntimeSemanticEngineMode::Shutdown,
    };
    let target_afr_override = match input.target_afr_override_x100 {
        ecu_spec::AfrOverride::None => RuntimeSemanticAfrOverride::None,
        ecu_spec::AfrOverride::Some(afr) => RuntimeSemanticAfrOverride::Some(afr.get()),
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
            ecu_spec::SyncState::Synced => ecu_domain::SyncState::Synced,
            ecu_spec::SyncState::Unsynced => ecu_domain::SyncState::Unsynced,
        },
        fuel_cut: input.fuel_cut,
        spark_cut: input.spark_cut,
        mode,
        target_afr_override_x100: target_afr_override,
    }
}

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

// --------------------------------------------------------------------------
// Product observation helpers
// --------------------------------------------------------------------------

/// Count injection events for a given cylinder in the schedule observations.
fn count_injection_events_for_cylinder(
    sched_obs: &ecu_runtime::RuntimeSemanticScheduleObservations,
    cylinder: u8,
) -> usize {
    let mut count = 0;
    for i in 0..sched_obs.events.len as usize {
        let evt = &sched_obs.events.events[i];
        if evt.cylinder == cylinder
            && matches!(evt.kind, RuntimeSemanticScheduleEventKind::InjectionOpen)
        {
            count += 1;
        }
    }
    count
}

/// Count ignition events for a given cylinder in the schedule observations.
fn count_ignition_events_for_cylinder(
    sched_obs: &ecu_runtime::RuntimeSemanticScheduleObservations,
    cylinder: u8,
) -> usize {
    let mut count = 0;
    for i in 0..sched_obs.events.len as usize {
        let evt = &sched_obs.events.events[i];
        if evt.cylinder == cylinder
            && matches!(evt.kind, RuntimeSemanticScheduleEventKind::CoilFire)
        {
            count += 1;
        }
    }
    count
}

/// Run scheduler for a fixture case using product APIs only.
/// Fuel and schedule observations come from the runtime semantic evaluators,
/// not from the spec oracle.
fn run_scheduler_for_case(case: &fm0016_fixture_matrix::FixtureCase) -> SchedulerObservedSurface {
    let mut state = SchedulerState::new();
    let input = case.input;

    // Build product fuel observations via runtime_semantic_evaluate_fuel
    let semantic_cal = build_semantic_calibration(&case.calibration);
    let semantic_input = to_semantic_input(&case.input);
    let fuel_obs: RuntimeSemanticFuelObservations = runtime_semantic_evaluate_fuel(
        &semantic_cal,
        semantic_input,
        semantic_state_for_fixture(case),
    )
    .expect("fuel evaluation should succeed");

    // Build product schedule observations via runtime_semantic_evaluate_schedule
    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);
    let sched_obs = runtime_semantic_evaluate_schedule(&schedule_cal, semantic_input, fuel_obs)
        .expect("schedule evaluation should succeed");

    // Drive scheduler using product events
    // Use timestamps from input
    let now = Micros::new(input.t_us.0);
    let start = Micros::new(input.t_us.0.saturating_add(100));

    // For each cylinder with injection events, schedule an injection
    // For each cylinder with ignition events, schedule an ignition
    let cyl_count = sched_obs.soi_deg10.count as usize;
    for cyl in 0..cyl_count {
        // Check if we have injection events for this cylinder
        let inj_count = count_injection_events_for_cylinder(&sched_obs, cyl as u8);
        if inj_count > 0 {
            // schedule injection using pw_corr_us
            let duration = fuel_obs.pw_corr_us.max(1);
            let end = Micros::new(start.get().saturating_add(duration));
            let inj_plan = InjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(cyl as u8 + 1)),
                pulse_width: PulseWidthUs::new(duration as u16),
            };
            let _ = state.schedule_injection(now, start, end, inj_plan);
        }

        // Check if we have ignition events for this cylinder
        let ign_count = count_ignition_events_for_cylinder(&sched_obs, cyl as u8);
        if ign_count > 0 {
            let duration = sched_obs.dwell_us.max(1) as u16;
            let end = Micros::new(start.get().saturating_add(duration as u32));
            let ign_plan = IgnitionPlan {
                output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(cyl as u8 + 1)),
                dwell: DwellUs::new(duration),
                advance: Degrees10::new(sched_obs.spark_advance_deg10 as i16),
            };
            let _ = state.schedule_ignition(now, start, end, ign_plan);
        }
    }

    ecu_scheduler::observe_scheduler(&state)
}

// --------------------------------------------------------------------------
// Conformance test
// --------------------------------------------------------------------------

fn conformance_test(case: &fm0016_fixture_matrix::FixtureCase) {
    // Verify fixture semantics via spec oracle (ONE call for semantic verification only)
    let spec = fm0016_fixture_matrix::oracle_result(*case);
    fm0016_fixture_matrix::assert_fixture_semantics(*case, &spec);

    // Run product scheduler to get observed data
    let obs = run_scheduler_for_case(case);

    // Build product observations for comparison
    let semantic_cal = build_semantic_calibration(&case.calibration);
    let semantic_input = to_semantic_input(&case.input);
    let fuel_obs = runtime_semantic_evaluate_fuel(
        &semantic_cal,
        semantic_input,
        semantic_state_for_fixture(case),
    )
    .expect("fuel evaluation should succeed");

    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);
    let sched_obs = runtime_semantic_evaluate_schedule(&schedule_cal, semantic_input, fuel_obs)
        .expect("schedule evaluation should succeed");

    // Determine expected counts from product observations
    let cyl_count = sched_obs.soi_deg10.count as usize;
    let mut expected_injection_count = 0usize;
    let mut expected_ignition_count = 0usize;

    for cyl in 0..cyl_count {
        expected_injection_count += count_injection_events_for_cylinder(&sched_obs, cyl as u8);
        expected_ignition_count += count_ignition_events_for_cylinder(&sched_obs, cyl as u8);
    }

    let has_fuel = fuel_obs.pw_corr_us > 0 && expected_injection_count > 0;
    let has_ignition = expected_ignition_count > 0;
    let has_any_event = has_fuel || has_ignition;

    // Scheduler mode should be Armed after scheduling (only if any event to schedule)
    if has_any_event {
        assert_eq!(
            obs.mode,
            SchedulerMode::Armed,
            "scheduler should be armed when any event is scheduled"
        );

        // Active groups should include Injector and Ignition (if present)
        if expected_injection_count > 0 {
            assert!(
                obs.active_groups & OutputGroup::Injector.mask() != 0,
                "Injector group should be active when injection events present"
            );
        }
        if has_ignition {
            assert!(
                obs.active_groups & OutputGroup::Ignition.mask() != 0,
                "Ignition group should be active when ignition events present"
            );
        }

        // Reserved channels should be set for active outputs
        if expected_injection_count > 0 {
            assert!(
                obs.reserved_channels[0] != 0,
                "Injector channel should be reserved"
            );
        }
        if has_ignition {
            assert!(
                obs.reserved_channels[1] != 0,
                "Ignition channel should be reserved"
            );
        }

        // Injection and ignition counts should match product events
        assert_eq!(
            obs.injection_count as u32, expected_injection_count as u32,
            "injection_count should match product injection events"
        );
        assert_eq!(
            obs.ignition_count as u32, expected_ignition_count as u32,
            "ignition_count should match product ignition events"
        );
    } else {
        // No events scheduled - scheduler should be Idle
        assert_eq!(
            obs.mode,
            SchedulerMode::Idle,
            "scheduler should be idle when no events are scheduled"
        );
    }

    // --- Cancellation/suspend observability ---
    // Verify cancel_group, cancel_all, suspend, on_sync_loss are observable
    let mut cancel_state = SchedulerState::new();
    let test_inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1000),
    };
    let test_ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
        dwell: DwellUs::new(2000),
        advance: Degrees10::new(100),
    };
    let now = Micros::new(1000);
    let start = Micros::new(1100);
    let end = Micros::new(2100);

    let _ = cancel_state.schedule_injection(now, start, end, test_inj);
    let _ = cancel_state.schedule_ignition(now, start, end, test_ign);
    assert_eq!(cancel_state.mode(), SchedulerMode::Armed);

    // cancel_group should clear the group
    cancel_state.cancel_group(OutputGroup::Injector);
    assert_eq!(
        cancel_state.active_groups() & OutputGroup::Injector.mask(),
        0
    );

    // cancel_all should clear everything
    cancel_state.cancel_all();
    assert_eq!(cancel_state.mode(), SchedulerMode::Idle);
    assert_eq!(cancel_state.active_groups(), 0);

    // suspend should transition to Suspended mode
    let _ = cancel_state.schedule_injection(now, start, end, test_inj);
    cancel_state.suspend();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);

    // on_sync_loss should suspend
    cancel_state.on_sync_loss();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);

    // on_hard_safety_shutdown should suspend
    cancel_state.on_hard_safety_shutdown();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);
}

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
fn scheduler_fm0016_unsynced() {
    let cases = unsynced_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_cuts() {
    let cases = cut_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_running() {
    let cases = running_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_synced() {
    let cases = synced_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_all() {
    run_all(&fm0016_fixture_matrix::fixture_cases());
}
