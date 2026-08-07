use super::*;

#[test]
fn runtime_semantic_calibration_from_fuel_tune_preserves_fuel_values() {
    let mut ve_table = [[100; 16]; 16];
    let mut afr_table = [[147; 16]; 16];
    ve_table[2][3] = 81;
    ve_table[13][11] = 123;
    afr_table[4][5] = 132;
    afr_table[14][8] = 155;

    let tune = FuelRuntimeTune::new(ve_table, afr_table, 3210, 654, 0);
    let cal = runtime_semantic_calibration_from_fuel_tune(&tune);

    assert_eq!(cal.ve_table.rpm_axis.values, FUEL_RUNTIME_RPM_BINS);
    assert_eq!(cal.ve_table.load_axis.values, FUEL_RUNTIME_LOAD_BINS);
    assert_eq!(cal.afr_target_table.rpm_axis.values, FUEL_RUNTIME_RPM_BINS);
    assert_eq!(
        cal.afr_target_table.load_axis.values,
        FUEL_RUNTIME_LOAD_BINS
    );
    assert_eq!(cal.ve_table.values[2][3], 81);
    assert_eq!(cal.ve_table.values[13][11], 123);
    assert_eq!(cal.afr_target_table.values[4][5], 132);
    assert_eq!(cal.afr_target_table.values[14][8], 155);
    assert_eq!(cal.required_fuel_us, 3210);
    assert_eq!(cal.deadtime_table_us.vbat_mv_axis.values[0], 0);
    assert_eq!(cal.deadtime_table_us.vbat_mv_axis.values[1], 20_000);
    assert_eq!(cal.deadtime_table_us.pressure_kpa10_axis.values[0], 0);
    assert_eq!(cal.deadtime_table_us.pressure_kpa10_axis.values[1], 2_000);
    assert_eq!(cal.deadtime_table_us.values[0][0], 654);
    assert_eq!(cal.deadtime_table_us.values[15][15], 654);
    assert_eq!(cal.clt_corr_curve.values[0], 1000);
    assert_eq!(cal.iat_corr_curve.values[0], 1000);
    assert_eq!(cal.baro_corr_curve.values[0], 1000);
    assert_eq!(cal.vbat_corr_curve.values[0], 1000);
    assert_eq!(cal.cranking_curve.values[0], 1000);
    assert_eq!(cal.warmup_curve.values[0], 1000);
    assert_eq!(cal.afterstart_table.values[0][0], 1000);
    assert_eq!(cal.pw_max_us, 20_000);
}
#[test]
fn runtime_semantic_calibration_from_fuel_tune_strategy_respects_load_source() {
    let speed_density = FuelRuntimeTune::new([[100; 16]; 16], [[147; 16]; 16], 2200, 800, 0);
    assert!(matches!(
        runtime_fuel_strategy_from_fuel_tune(&speed_density),
        RuntimeFuelStrategy::SpeedDensityVe { .. }
    ));

    let alpha_n = FuelRuntimeTune::new([[100; 16]; 16], [[147; 16]; 16], 2200, 800, 1);
    assert!(matches!(
        runtime_fuel_strategy_from_fuel_tune(&alpha_n),
        RuntimeFuelStrategy::AlphaN { .. }
    ));
}
#[test]
fn runtime_semantic_schedule_preserves_u32_dwell_values() {
    let out = runtime_semantic_evaluate_schedule(
        &semantic_schedule_calibration(70_000),
        semantic_schedule_input(100, SyncState::Locked { cam_ref: false }),
        semantic_fuel_observations(1000, false, false),
    )
    .expect("valid schedule");

    assert_eq!(out.dwell_us, 70_000);
    assert_eq!(out.dwell_duration_deg10, 420);
}
#[test]
fn runtime_semantic_schedule_rejects_dwell_duration_overflow() {
    let err = runtime_semantic_evaluate_schedule(
        &semantic_schedule_calibration(2_000_000),
        semantic_schedule_input(8000, SyncState::Locked { cam_ref: false }),
        semantic_fuel_observations(0, false, false),
    )
    .expect_err("dwell duration should overflow u16 deg10");

    assert_eq!(err, RuntimeSemanticScheduleError::DurationOverflow);
}
#[test]
fn runtime_semantic_schedule_cut_diagnostic_precedes_unsynced() {
    let out = runtime_semantic_evaluate_schedule(
        &semantic_schedule_calibration(2500),
        semantic_schedule_input(1000, SyncState::Unsynced),
        semantic_fuel_observations(0, true, true),
    )
    .expect("valid schedule");

    assert_eq!(
        out.diagnostic,
        RuntimeSemanticScheduleDiagnostic::FuelCutActive
    );
    assert_eq!(out.events.len, 0);
}
#[test]
fn direct_pw_strategy_emits_scheduler_fuel_intent_without_semantic_observations() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );
    assert!(
        result
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );
    assert_eq!(result.control.fuel_intent.observations.ve_pct_x100, None);
    assert_eq!(
        result.control.base_fuel,
        result.control.fuel_intent.pulse_width_us
    );
}
#[test]
fn direct_pw_strategy_owns_model_when_switching_to_ve_and_back() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model_with_base_pw(1200));

    let first_direct = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );
    assert_eq!(first_direct.control.base_fuel, PulseWidthUs::new(1200));
    assert!(
        first_direct
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );

    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    let ve_result = runtime.step(
        running_step_inputs(2_000, 3000, true, true),
        running_control_inputs(2_000, 3000),
    );
    assert!(
        !ve_result
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );

    runtime.configure_runtime_fuel_model(RuntimeFuelStrategy::DirectPulseWidthTable(
        test_fuel_model_with_base_pw(3200),
    ));
    let second_direct = runtime.step(
        running_step_inputs(3_000, 3000, true, true),
        running_control_inputs(3_000, 3000),
    );

    assert_eq!(second_direct.control.base_fuel, PulseWidthUs::new(3200));
    assert!(
        second_direct
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );
}
#[test]
fn semantic_strategy_does_not_apply_legacy_enrichment_twice() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    runtime.configure_warmup_enrichment(WarmupConfig {
        start_c: 0,
        end_c: 100,
        max_percent_x100: 200,
        min_percent_x100: 100,
    });

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );

    assert!(
        !result
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );
    assert!(result.control.enrichment.total_x100() > 100);
    assert_eq!(
        result.control.base_fuel,
        result.control.fuel_intent.pulse_width_us
    );
    assert_eq!(result.control.enriched_fuel, result.control.base_fuel);
}
#[test]
fn direct_pulse_width_acceleration_enrichment_reports_transient_and_freezes_lambda() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model_with_base_pw(2500));
    runtime.configure_acceleration_enrichment(AccelerationConfig {
        tpsdot_thresh_pct_s: 10,
        mapdot_thresh_kpa_s: 10,
        percent_x100: 120,
        decay_time_ms: 400,
        lockout_ms: 0,
    });

    let mut control = running_control_inputs(1_000, 3_000);
    control.enrichment.tpsdot_pct_s = 25;
    let result = runtime.step(running_step_inputs(1_000, 3_000, true, true), control);
    let observed = extract_runtime_observed_surface(&result, &runtime.snapshot());

    assert!(observed.runtime_fuel.transient_enrichment_active);
    assert_eq!(observed.runtime_fuel.transient_enrichment_pulse_us, 600);
    assert_eq!(
        observed
            .runtime_fuel
            .transient_enrichment_decay_steps_remaining,
        0
    );
    assert_eq!(
        observed.runtime_lambda.disable_reason,
        LambdaDisableReason::AccelerationEnrichment
    );
    assert!(!observed.runtime_lambda.active);
}
#[test]
fn direct_pulse_width_acceleration_pulse_reports_delta_after_other_enrichment() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model_with_base_pw(2500));
    runtime.configure_warmup_enrichment(WarmupConfig {
        start_c: 0,
        end_c: 100,
        max_percent_x100: 150,
        min_percent_x100: 100,
    });
    runtime.configure_acceleration_enrichment(AccelerationConfig {
        tpsdot_thresh_pct_s: 10,
        mapdot_thresh_kpa_s: 10,
        percent_x100: 120,
        decay_time_ms: 400,
        lockout_ms: 0,
    });

    let mut control = running_control_inputs(1_000, 3_000);
    control.enrichment.clt_c = -10;
    control.enrichment.tpsdot_pct_s = 25;
    let result = runtime.step(running_step_inputs(1_000, 3_000, true, true), control);
    let observed = extract_runtime_observed_surface(&result, &runtime.snapshot());

    assert_eq!(observed.runtime_fuel.transient_enrichment_pulse_us, 750);
    assert_eq!(observed.runtime_fuel.enriched_fuel_pw_us, 4500);
}
#[test]
fn direct_pulse_width_acceleration_freeze_holds_previous_lambda_trim() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model_with_base_pw(2500));
    runtime.configure_acceleration_enrichment(AccelerationConfig {
        tpsdot_thresh_pct_s: 10,
        mapdot_thresh_kpa_s: 10,
        percent_x100: 120,
        decay_time_ms: 400,
        lockout_ms: 0,
    });

    let mut active_control = running_control_inputs(1_000, 3_000);
    active_control.lambda.measured_lambda100 = Lambda100::new(90);
    let active = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        active_control,
    );

    let mut frozen_control = running_control_inputs(2_000, 3_000);
    frozen_control.lambda.measured_lambda100 = Lambda100::new(50);
    frozen_control.enrichment.tpsdot_pct_s = 25;
    let frozen = runtime.step(
        running_step_inputs(2_000, 3_000, true, true),
        frozen_control,
    );

    assert_eq!(
        frozen.control.lambda.trim_x100,
        active.control.lambda.trim_x100
    );
    assert_eq!(
        frozen.control.lambda.disable_reason,
        LambdaDisableReason::AccelerationEnrichment
    );
}
#[test]
fn direct_pw_fuel_cut_request_suppresses_injector_outputs_but_keeps_ignition() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_direct_cut_requests(true, false);

    let result = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );

    assert!(result.control.fuel_cut);
    assert!(!result.control.spark_cut);
    assert_eq!(
        result.control.fuel_intent.pulse_width_us,
        PulseWidthUs::new(0)
    );
    assert!(runtime.snapshot().fuel_cut);
    assert!(!runtime.snapshot().spark_cut);

    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(result.actions, &mut outputs, &mut aux)
        .expect("direct pw fuel-cut actions should lower");

    assert_eq!(status.scheduled_output_transitions, 2);
    assert!(outputs
        .as_slice()
        .iter()
        .all(|transition| matches!(transition.output, EcuOutput::Ignition(_))));
}
#[test]
fn direct_pw_spark_cut_request_suppresses_ignition_outputs_but_keeps_injection() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_direct_cut_requests(false, true);

    let result = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );

    assert!(!result.control.fuel_cut);
    assert!(result.control.spark_cut);
    assert!(result.control.fuel_intent.pulse_width_us.get() > 0);
    assert!(!runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);

    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(result.actions, &mut outputs, &mut aux)
        .expect("direct pw spark-cut actions should lower");

    assert_eq!(status.scheduled_output_transitions, 2);
    assert!(outputs
        .as_slice()
        .iter()
        .all(|transition| matches!(transition.output, EcuOutput::Injector(_))));
}
#[test]
fn direct_pw_safety_latch_holds_until_off_clear() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let latched = runtime.step(
        StepInputs {
            safety_latch_request: true,
            ..running_step_inputs(1_000, 3000, true, true)
        },
        running_control_inputs(1_000, 3000),
    );
    assert!(runtime.snapshot().safety_latched);
    assert!(latched.control.fuel_cut);
    assert!(latched.control.spark_cut);

    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(latched.actions, &mut outputs, &mut aux)
        .expect("latched direct pw actions should lower");
    assert_eq!(status.scheduled_output_transitions, 0);

    let held = runtime.step(
        running_step_inputs(2_000, 3000, true, true),
        running_control_inputs(2_000, 3000),
    );
    assert!(runtime.snapshot().safety_latched);
    assert!(held.control.fuel_cut);
    assert!(held.control.spark_cut);

    let _ = runtime.step(
        StepInputs {
            now_us: Micros::new(3_000),
            rpm: 0,
            load_kpa10: 0,
            angle_x10: 0,
            trigger_synced: false,
            cam_seen: false,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(3_000, 0),
    );
    let cleared = runtime.step(
        running_step_inputs(4_000, 3000, true, true),
        running_control_inputs(4_000, 3000),
    );
    assert!(!runtime.snapshot().safety_latched);
    assert!(!cleared.control.fuel_cut);
    assert!(!cleared.control.spark_cut);
}
#[test]
fn direct_pw_sync_loss_sets_paired_cut_and_cancels_outputs() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let armed = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );
    assert!(!armed.control.fuel_cut);
    assert!(!armed.control.spark_cut);
    assert!(armed.actions.iter().any(|action| {
        matches!(
            action,
            Action::ArmScheduler { .. } | Action::ArmInjection(_) | Action::ArmIgnition(_)
        )
    }));

    let lost = runtime.step(
        running_step_inputs(2_000, 3000, false, false),
        running_control_inputs(2_000, 3000),
    );

    assert_eq!(
        runtime.engine.engine_time_authority.crank,
        CrankSyncState::SyncLost
    );
    assert!(lost.control.fuel_cut);
    assert!(lost.control.spark_cut);
    assert_eq!(
        lost.control.fuel_intent.pulse_width_us,
        PulseWidthUs::new(0)
    );
    assert!(runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);

    let mut actions = lost.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SyncLoss))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(arm_scheduler_count(lost.actions), 0);
    assert_eq!(arm_injection_count(lost.actions), 0);
    assert_eq!(arm_ignition_count(lost.actions), 0);
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Suspended
    );

    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(lost.actions, &mut outputs, &mut aux)
        .expect("sync-loss direct pw actions should lower");
    assert_eq!(status.scheduled_output_transitions, 0);
    assert!(status.cancel_scheduled_outputs());
    assert!(outputs.is_empty());
}
#[test]
fn speed_density_vs_alpha_n_strategy_switches_map_vs_tps_lookup() {
    let mut runtime = EngineRuntime::new();
    let calibration = semantic_fuel_calibration_with_ve_cells(5000, 10000);
    let state = RuntimeSemanticState::default();

    runtime.configure_speed_density_ve(calibration, state);
    let map_result = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );

    runtime.configure_alpha_n(calibration, state);
    let tps_result = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );

    assert!(
        !map_result
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );
    assert!(
        !tps_result
            .control
            .fuel_intent
            .observations
            .strategy_is_direct_pw
    );
    assert_ne!(
        map_result.control.fuel_intent.pulse_width_us,
        tps_result.control.fuel_intent.pulse_width_us
    );
}
#[test]
fn typed_fuel_input_propagates_mode_and_afr_override_into_semantic_input() {
    let input = FuelInputSnapshot {
        now_us: Micros::new(123),
        rpm: Rpm::new(1500),
        map_kpa10: Kpa10::new(980),
        load_kpa10: Kpa10::new(980),
        tps_x100: 250,
        knock_intensity_x100: 0,
        maf_x100: 420,
        clt_c10: 700,
        iat_c10: 300,
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: 12100,
        maf_valid: true,
        lambda_valid: true,
        lambda_measured: Lambda100::new(97),
        requested_open_loop: false,
        baro_valid: true,
        sync: SyncState::Locked { cam_ref: false },
        mode: FuelEngineMode::Running,
        launch_armed: true,
        flat_shift_armed: false,
        fuel_cut_request: false,
        spark_cut_request: true,
        target_afr_override_x100: FuelAfrOverride::Some(1320),
    };
    let semantic = EngineRuntime::semantic_input_for_strategy(
        input,
        FuelLoadSource::Map,
        input.sync,
        input.mode,
        true,
    );

    assert_eq!(semantic.mode, RuntimeSemanticEngineMode::Running);
    assert!(semantic.launch_armed);
    assert!(!semantic.flat_shift_armed);
    assert!(semantic.lambda_valid);
    assert_eq!(semantic.lambda_measured, Lambda100::new(97));
    assert!(!semantic.requested_open_loop);
    assert!(!semantic.fuel_cut);
    assert!(!semantic.spark_cut);
    assert!(!semantic.direct_fuel_cut_request);
    assert!(semantic.direct_spark_cut_request);
    assert!(semantic.safety_latch_request);
    assert_eq!(
        semantic.target_afr_override_x100,
        RuntimeSemanticAfrOverride::Some(1320)
    );
}
#[test]
fn maf_strategy_uses_maf_signal_instead_of_map_load() {
    let mut runtime = EngineRuntime::new();
    let calibration = semantic_fuel_calibration_with_ve_cells(4000, 9000);
    let state = RuntimeSemanticState::default();

    runtime.configure_speed_density_ve(calibration, state);
    let map_result = runtime.step(
        running_step_inputs(1_000, 3000, true, true),
        running_control_inputs(1_000, 3000),
    );

    runtime.configure_maf(calibration, state);
    runtime.engine.load_kpa10 = Kpa10::new(400);
    let mut maf_inputs = running_control_inputs(1_000, 3000);
    maf_inputs.fuel_sensors = FuelSensorInputs {
        maf_valid: true,
        maf_x100: 100,
        iat_c10: 250,
        vbatt_mv: 12_000,
        baro_valid: true,
        baro_kpa10: Kpa10::new(1_010),
    };
    let maf_result = runtime.step(running_step_inputs(1_000, 3000, true, true), maf_inputs);

    // MAF path must use the explicit control-side sensor ingress, not a placeholder zero or MAP load.
    assert_ne!(
        map_result.control.fuel_intent.pulse_width_us,
        maf_result.control.fuel_intent.pulse_width_us
    );
}
#[test]
fn maf_strategy_invalid_signal_fails_closed_instead_of_using_placeholder_load() {
    let mut runtime = EngineRuntime::new();
    let calibration = semantic_fuel_calibration_with_ve_cells(4000, 9000);
    runtime.configure_maf(calibration, RuntimeSemanticState::default());
    let mut control_inputs = running_control_inputs(1_000, 3000);
    control_inputs.fuel_sensors = FuelSensorInputs {
        maf_valid: false,
        maf_x100: 100,
        iat_c10: 250,
        vbatt_mv: 12_000,
        baro_valid: true,
        baro_kpa10: Kpa10::new(1_010),
    };

    let result = runtime.step(running_step_inputs(1_000, 3000, true, true), control_inputs);

    assert!(result.control.fuel_cut);
    assert!(result.control.spark_cut);
    assert_eq!(
        result.control.fuel_intent.pulse_width_us,
        PulseWidthUs::new(0)
    );
    assert_eq!(result.control.fuel_intent.observations.ve_pct_x100, None);
    assert_eq!(result.control.fuel_intent.observations.pw_corr_us, 0);
    assert!(runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);
    assert_eq!(arm_injection_count(result.actions), 0);
    assert_eq!(arm_ignition_count(result.actions), 0);

    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(result.actions, &mut outputs, &mut aux)
        .expect("invalid MAF cut actions should lower");
    assert_eq!(status.scheduled_output_transitions, 0);
    assert!(outputs.is_empty());
    assert!(aux.is_empty());
}
#[test]
fn runtime_step_uses_explicit_fuel_sensor_corrections_in_semantic_path() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.iat_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.iat_corr_curve.values = [1050, 900, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    calibration.baro_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [900, 1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.baro_corr_curve.values = [900, 1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    calibration.vbat_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [11_000, 14_000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.vbat_corr_curve.values = [900, 1100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    let run_with_sensors = |fuel_sensors| {
        let mut runtime = EngineRuntime::new();
        runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());
        let mut inputs = running_control_inputs(1_000, 3000);
        inputs.fuel_sensors = fuel_sensors;
        runtime.step(running_step_inputs(1_000, 3000, true, true), inputs)
    };

    let unfavorable = run_with_sensors(FuelSensorInputs {
        maf_valid: false,
        maf_x100: 0,
        iat_c10: 900,
        vbatt_mv: 11_000,
        baro_valid: true,
        baro_kpa10: Kpa10::new(900),
    });
    let favorable = run_with_sensors(FuelSensorInputs {
        maf_valid: false,
        maf_x100: 0,
        iat_c10: 100,
        vbatt_mv: 14_000,
        baro_valid: true,
        baro_kpa10: Kpa10::new(1_100),
    });

    assert!(
        favorable.control.fuel_intent.pulse_width_us
            > unfavorable.control.fuel_intent.pulse_width_us
    );
}
#[test]
fn runtime_step_ignores_invalid_baro_value_in_semantic_path() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.baro_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [500, 1_500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.baro_corr_curve.values = [500, 1_500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

    let run_with_baro = |baro_valid, baro_kpa10| {
        let mut runtime = EngineRuntime::new();
        runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());
        let mut inputs = running_control_inputs(1_000, 3000);
        inputs.fuel_sensors = FuelSensorInputs {
            baro_valid,
            baro_kpa10,
            ..FuelSensorInputs::default()
        };
        runtime.step(running_step_inputs(1_000, 3000, true, true), inputs)
    };

    let invalid_low = run_with_baro(false, Kpa10::new(500));
    let invalid_high = run_with_baro(false, Kpa10::new(1_500));
    let valid_low = run_with_baro(true, Kpa10::new(500));
    let valid_high = run_with_baro(true, Kpa10::new(1_500));

    assert_eq!(
        invalid_low.control.fuel_intent.pulse_width_us,
        invalid_high.control.fuel_intent.pulse_width_us
    );
    assert!(
        valid_high.control.fuel_intent.pulse_width_us
            > valid_low.control.fuel_intent.pulse_width_us
    );
}
#[test]
fn speed_density_map_increase_raises_pulse_width() {
    let calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    let low = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            map_kpa10: Kpa10::new(600),
            load_kpa10: Kpa10::new(600),
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("low-map eval");
    let high = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            map_kpa10: Kpa10::new(1200),
            load_kpa10: Kpa10::new(1200),
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("high-map eval");
    assert!(high.pw_corr_us > low.pw_corr_us);
}
#[test]
fn colder_clt_applies_more_fuel_when_curve_demands_enrichment() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.clt_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.clt_corr_curve.values = [1300, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let cold = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            clt_c10: 100,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("cold eval");
    let hot = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            clt_c10: 900,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("hot eval");
    assert!(cold.pw_corr_us > hot.pw_corr_us);
}
#[test]
fn hotter_iat_reduces_fuel_when_curve_demands_reduction() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.iat_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.iat_corr_curve.values = [1050, 900, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let cool = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            iat_c10: 100,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("cool eval");
    let hot = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            iat_c10: 900,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("hot eval");
    assert!(hot.pw_corr_us < cool.pw_corr_us);
}
#[test]
fn fuel_cut_forces_zero_or_clamped_min_pulse_path() {
    let calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            fuel_cut: true,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("fuel-cut eval");
    assert!(out.fuel_cut);
    assert_eq!(out.pw_corr_us, 0);
}
#[test]
fn runtime_semantic_idle_integrates_to_expected_duty() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.idle_target_rpm = 1000;
    calibration.idle_base_duty_x1000 = 350;
    calibration.idle_kp_x1000 = 300;
    calibration.idle_ki_x1000 = 500;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            rpm: Rpm::new(700),
            clt_c10: 800,
            ..semantic_schedule_input(700, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("idle integrate eval");

    assert_eq!(out.idle_duty_x1000, 590);
    assert_eq!(out.idle_integrator_state.acc, 150);
    assert_eq!(out.idle_integrator_state.min_acc, -2000);
    assert_eq!(out.idle_integrator_state.max_acc, 2000);
    assert!(!out.idle_integrator_state.frozen);
}
#[test]
fn runtime_semantic_idle_freezes_when_cold() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.idle_target_rpm = 1000;
    calibration.idle_base_duty_x1000 = 350;
    calibration.idle_kp_x1000 = 300;
    calibration.idle_ki_x1000 = 500;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            rpm: Rpm::new(700),
            clt_c10: 650,
            ..semantic_schedule_input(700, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState {
            idle_integrator_acc: 123,
            ..RuntimeSemanticState::default()
        },
    )
    .expect("idle cold eval");

    assert_eq!(out.idle_duty_x1000, 563);
    assert_eq!(out.idle_integrator_state.acc, 123);
    assert!(out.idle_integrator_state.frozen);
}
#[test]
fn runtime_semantic_idle_anti_windup_freezes_saturated() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.idle_target_rpm = 3000;
    calibration.idle_base_duty_x1000 = 1000;
    calibration.idle_kp_x1000 = 0;
    calibration.idle_ki_x1000 = 1000;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            rpm: Rpm::new(2500),
            clt_c10: 800,
            ..semantic_schedule_input(2500, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState {
            idle_integrator_acc: 200,
            ..RuntimeSemanticState::default()
        },
    )
    .expect("idle saturated eval");

    assert_eq!(out.idle_duty_x1000, 1000);
    assert_eq!(out.idle_integrator_state.acc, 200);
    assert!(out.idle_integrator_state.frozen);
}
#[test]
fn runtime_semantic_idle_freezes_on_post_arbiter_shutdown_cut() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.idle_target_rpm = 1000;
    calibration.idle_base_duty_x1000 = 350;
    calibration.idle_kp_x1000 = 300;
    calibration.idle_ki_x1000 = 500;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            rpm: Rpm::new(700),
            clt_c10: 800,
            mode: RuntimeSemanticEngineMode::Shutdown,
            ..semantic_schedule_input(700, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState {
            idle_integrator_acc: 123,
            ..RuntimeSemanticState::default()
        },
    )
    .expect("idle shutdown eval");

    assert!(out.fuel_cut);
    assert!(out.spark_cut);
    assert_eq!(out.pw_corr_us, 0);
    assert_eq!(out.idle_duty_x1000, 563);
    assert_eq!(out.idle_integrator_state.acc, 123);
    assert!(out.idle_integrator_state.frozen);
}
#[test]
fn runtime_semantic_idle_freezes_when_ae_was_active_before_step() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.idle_target_rpm = 1000;
    calibration.idle_base_duty_x1000 = 350;
    calibration.idle_kp_x1000 = 300;
    calibration.idle_ki_x1000 = 500;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            rpm: Rpm::new(700),
            clt_c10: 800,
            ..semantic_schedule_input(700, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState {
            ae_active: true,
            idle_integrator_acc: 123,
            ..RuntimeSemanticState::default()
        },
    )
    .expect("idle pre-ae-freeze eval");

    assert_eq!(out.idle_duty_x1000, 563);
    assert_eq!(out.idle_integrator_state.acc, 123);
    assert!(out.idle_integrator_state.frozen);
    assert!(!out.lambda_integrator_state.frozen);
}
#[test]
fn semantic_fuel_advance_trim_defaults_to_zero_without_soft_rev_or_knock() {
    let calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        semantic_schedule_input(3_000, SyncState::Locked { cam_ref: false }),
        RuntimeSemanticState::default(),
    )
    .expect("baseline semantic eval");

    assert_eq!(out.advance_deg10_trim, 0);
}
#[test]
fn semantic_fuel_advance_trim_applies_soft_rev_retard() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.soft_rev_rpm = 2_500;
    calibration.rev_hysteresis_rpm = 100;
    calibration.soft_retard_max_deg10 = 120;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        semantic_schedule_input(3_000, SyncState::Locked { cam_ref: false }),
        RuntimeSemanticState::default(),
    )
    .expect("soft-rev semantic eval");

    assert_eq!(out.advance_deg10_trim, -120);
}
#[test]
fn semantic_fuel_advance_trim_applies_knock_retard() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.knock_threshold_x100 = 500;
    calibration.knock_retard_step_deg10 = 40;
    calibration.knock_retard_max_deg10 = 120;
    calibration.knock_recovery_step_deg10 = 20;
    calibration.knock_recovery_delay_cycles = 1;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            knock_intensity_x100: 600,
            ..semantic_schedule_input(3_000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("knock semantic eval");

    assert_eq!(out.advance_deg10_trim, -40);
}
#[test]
fn semantic_fuel_advance_trim_combines_soft_rev_and_knock_retard() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.soft_rev_rpm = 2_500;
    calibration.rev_hysteresis_rpm = 100;
    calibration.soft_retard_max_deg10 = 120;
    calibration.knock_threshold_x100 = 500;
    calibration.knock_retard_step_deg10 = 40;
    calibration.knock_retard_max_deg10 = 120;
    calibration.knock_recovery_step_deg10 = 20;
    calibration.knock_recovery_delay_cycles = 1;

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            knock_intensity_x100: 600,
            ..semantic_schedule_input(3_000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("combined trim semantic eval");

    assert_eq!(out.advance_deg10_trim, -160);
}
#[test]
fn semantic_fuel_deadtime_uses_voltage_and_pressure_axes() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(0, 0);
    calibration.required_fuel_us = 0;
    let mut values = [[0u16; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN];
    values[0][0] = 100;
    values[0][1] = 200;
    values[1][0] = 300;
    values[1][1] = 500;
    calibration.deadtime_table_us = RuntimeSemanticDeadtimeTableU16 {
        vbat_mv_axis: RuntimeSemanticAxis16 {
            len: 2,
            values: [10_000, 14_000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        pressure_kpa10_axis: RuntimeSemanticAxis16 {
            len: 2,
            values: [800, 1200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        },
        values,
    };

    let out = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            rpm: Rpm::new(6500),
            map_kpa10: Kpa10::new(1000),
            load_kpa10: Kpa10::new(100),
            baro_kpa10: Kpa10::new(1000),
            vbatt_mv: 12_000,
            ..semantic_schedule_input(3_000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("semantic deadtime eval");

    assert_eq!(out.pw_base_us, 0);
    assert_eq!(out.pw_corr_us, 275);
}
#[test]
fn semantic_fuel_correction_order_matches_spec_pipeline() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(8000, 8000);
    calibration.required_fuel_us = 2000;
    calibration.pref_kpa10 = 1000;
    calibration.clt_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.clt_corr_curve.values = [1200, 1200, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    calibration.iat_corr_curve.axis = RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    };
    calibration.iat_corr_curve.values = [900, 900, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    calibration.baro_corr_curve = semantic_curve_u16(1100);
    calibration.vbat_corr_curve = semantic_curve_u16(950);
    calibration.deadtime_table_us = semantic_deadtime_table_u16(300);
    calibration.ae_shot_curve_us = semantic_curve_u16(200);
    calibration.ae_decay_steps_curve = semantic_curve_u16(1);
    calibration.ae_decay_ratio_curve_x1000 = semantic_curve_u16(1000);

    let input = RuntimeSemanticInputSnapshot {
        t_us: Micros::new(1_000),
        rpm: Rpm::new(3000),
        map_kpa10: Kpa10::new(1000),
        load_kpa10: Kpa10::new(1000),
        tps_x100: 0,
        clt_c10: 500,
        iat_c10: 500,
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: 12_000,
        lambda_valid: true,
        lambda_measured: Lambda100::new(100),
        requested_open_loop: false,
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: SyncState::Locked { cam_ref: false },
        fuel_cut: false,
        spark_cut: false,
        direct_fuel_cut_request: false,
        direct_spark_cut_request: false,
        safety_latch_request: false,
        mode: RuntimeSemanticEngineMode::Running,
        target_afr_override_x100: RuntimeSemanticAfrOverride::None,
    };

    let out = runtime_semantic_evaluate_fuel(&calibration, input, RuntimeSemanticState::default())
        .expect("semantic eval");

    // Expected order:
    // pw_base = required_fuel * VE / 10000 = 2000 * 8000 / 10000 = 1600
    // then *clt(1200) *iat(900) *baro(1100) *vbat(950) with floor division by 1000 each stage:
    // 1600 -> 1920 -> 1728 -> 1900 -> 1805
    // + deadtime 300 + AE shot 200 = 2305
    assert_eq!(out.pw_base_us, 1600);
    assert_eq!(out.pw_corr_us, 2305);
}
#[test]
fn lambda_integrator_freezes_when_engine_is_cold() {
    let cal = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    let (corr, integ) = runtime_semantic_lambda_step(
        &cal,
        &RuntimeSemanticInputSnapshot {
            clt_c10: 650,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        1470,
        false,
        false,
        123,
        false,
    );
    assert!(corr >= RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000);
    assert!(corr <= RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000);
    assert_eq!(integ.acc, 123);
    assert!(integ.frozen);
}
#[test]
fn lambda_integrator_freezes_when_fuel_cut_is_active() {
    let cal = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    let (corr, integ) = runtime_semantic_lambda_step(
        &cal,
        &semantic_schedule_input(3000, SyncState::Locked { cam_ref: false }),
        1470,
        true,
        false,
        77,
        false,
    );
    assert!(corr >= RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000);
    assert!(corr <= RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000);
    assert_eq!(integ.acc, 77);
    assert!(integ.frozen);
}
#[test]
fn lambda_integrator_updates_when_not_frozen_and_ki_enabled() {
    let mut cal = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    cal.lambda_kp_x1000 = 0;
    cal.lambda_ki_x1000 = 1000;
    let (corr, integ) = runtime_semantic_lambda_step(
        &cal,
        &RuntimeSemanticInputSnapshot {
            clt_c10: 900,
            lambda_measured: Lambda100::new(95),
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        1470,
        false,
        false,
        0,
        false,
    );
    assert_eq!(corr, 1050);
    assert_eq!(integ.acc, 50);
    assert!(!integ.frozen);
}
#[test]
fn lambda_integrator_freezes_when_requested_open_loop() {
    let mut cal = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    cal.lambda_kp_x1000 = 1000;
    cal.lambda_ki_x1000 = 1000;
    let (corr, integ) = runtime_semantic_lambda_step(
        &cal,
        &RuntimeSemanticInputSnapshot {
            clt_c10: 900,
            lambda_measured: Lambda100::new(95),
            requested_open_loop: true,
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        1470,
        false,
        false,
        123,
        false,
    );
    assert_eq!(corr, 1000);
    assert_eq!(integ.acc, 123);
    assert!(integ.frozen);
}
#[test]
fn semantic_lambda_correction_changes_pulse_width_when_sensor_is_valid() {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.lambda_kp_x1000 = 1000;

    let rich = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            lambda_valid: true,
            lambda_measured: Lambda100::new(95),
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("rich lambda semantic eval");
    let lean = runtime_semantic_evaluate_fuel(
        &calibration,
        RuntimeSemanticInputSnapshot {
            lambda_valid: true,
            lambda_measured: Lambda100::new(105),
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        RuntimeSemanticState::default(),
    )
    .expect("lean lambda semantic eval");

    assert!(rich.pw_corr_us > lean.pw_corr_us);
    assert_eq!(rich.lambda_correction_x1000, 1050);
    assert_eq!(lean.lambda_correction_x1000, 950);
}
