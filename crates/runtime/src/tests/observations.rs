use super::*;

#[test]
fn differential_input_snapshot_represents_fm0016_fields() {
    let snapshot = DifferentialInputSnapshot {
        now_us: Micros::new(10_000),
        rpm: Rpm::new(1200),
        map_kpa10: Kpa10::new(920),
        load_kpa10: Kpa10::new(870),
        angle_x10: Degrees10::new(2000),
        clt_c10: -350,
        iat_c10: -120,
        baro_kpa10: Kpa10::new(980),
        vbatt_mv: 11_800,
        sync: SyncState::Unsynced,
        fuel_cut: true,
        spark_cut: true,
        mode: RuntimeEngineMode::Shutdown,
        target_afr_override_x100: RuntimeAfrOverride::Some(4000),
        launch_armed: false,
        flat_shift_armed: false,
        safety_latch_request: false,
    };

    let mapped = snapshot.to_step_inputs();
    assert_eq!(mapped.now_us, Micros::new(10_000));
    assert_eq!(mapped.rpm, 1200);
    assert_eq!(mapped.load_kpa10, 870);
    assert_eq!(mapped.angle_x10, 2000);
    assert!(!mapped.trigger_synced);
    assert!(!mapped.cam_seen);
}
#[test]
fn extract_fuel_observations_preserves_direct_fuel_state_shells() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_warmup_enrichment(WarmupConfig {
        start_c: 0,
        end_c: 100,
        max_percent_x100: 150,
        min_percent_x100: 100,
    });

    let mut control = running_control_inputs(1_000, 3_000);
    control.enrichment.clt_c = -10;
    control.enrichment.cranking = true;
    control.enrichment.just_started = true;
    let result = runtime.step(running_step_inputs(1_000, 3_000, true, true), control);

    let fuel = extract_fuel_observations(&result);

    assert_eq!(fuel.base_fuel_pw_us, result.control.base_fuel.get() as u16);
    assert_eq!(
        fuel.enriched_fuel_pw_us,
        result.control.enriched_fuel.get() as u16
    );
    assert_eq!(
        fuel.lambda_target_x100,
        result.control.lambda.target_lambda100.get()
    );
    assert!(fuel.startup_active);
    assert_eq!(fuel.startup_window_remaining, 3000);
    assert_eq!(
        fuel.startup_window_mode,
        ecu_control::FuelStartupWindowMode::Milliseconds
    );
    assert!(fuel.warmup_active);
    assert_eq!(fuel.warmup_correction_x100, 150);
    assert_eq!(
        fuel.warmup_temperature_mode,
        ecu_control::FuelWarmupTemperatureMode::ColdClamp
    );
    assert!(fuel.afterstart_active);
    assert_eq!(fuel.afterstart_window_remaining, 5000);
    assert_eq!(
        fuel.afterstart_window_mode,
        ecu_control::FuelAfterstartWindowMode::Milliseconds
    );
    assert!(!fuel.transient_enrichment_active);
    assert_eq!(fuel.transient_enrichment_pulse_us, 0);
    assert_eq!(fuel.transient_enrichment_decay_steps_remaining, 0);

    let lambda = extract_lambda_observations(&result);
    assert_eq!(lambda.mode, result.control.lambda.mode);
    assert_eq!(lambda.active, result.control.lambda.active);
    assert_eq!(
        lambda.target_lambda_x100,
        result.control.lambda.target_lambda100.get()
    );
    assert_eq!(
        lambda.measured_lambda_x100,
        result.control.lambda.measured_lambda100.get()
    );
    assert_eq!(lambda.trim_x100, result.control.lambda.trim_x100);
    assert_eq!(lambda.disable_reason, result.control.lambda.disable_reason);
}
#[test]
fn extract_runtime_observed_surface_preserves_semantic_fuel_state_shells() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_state_calibration(),
        RuntimeSemanticState::default(),
    );

    let mut control = running_control_inputs(1_000, 3_000);
    control.enrichment.clt_c = -10;
    control.enrichment.tpsdot_pct_s = 200;
    let result = runtime.step(running_step_inputs(1_000, 3_000, true, true), control);
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_base_fuel_pw_us,
        result.control.base_fuel.get() as u16
    );
    assert_eq!(
        observed.runtime_enriched_fuel_pw_us,
        result.control.enriched_fuel.get() as u16
    );
    assert_eq!(
        observed.runtime_lambda_target_x100,
        result.control.lambda.target_lambda100.get()
    );
    assert_eq!(observed.runtime_lambda.mode, result.control.lambda.mode);
    assert_eq!(observed.runtime_lambda.active, result.control.lambda.active);
    assert_eq!(
        observed.runtime_lambda.target_lambda_x100,
        observed.runtime_lambda_target_x100
    );
    assert_eq!(
        observed.runtime_lambda.measured_lambda_x100,
        result.control.lambda.measured_lambda100.get()
    );
    assert_eq!(
        observed.runtime_lambda.trim_x100,
        result.control.lambda.trim_x100
    );
    assert_eq!(
        observed.runtime_lambda.disable_reason,
        result.control.lambda.disable_reason
    );
    assert_eq!(
        observed.runtime_lambda_correction,
        extract_lambda_correction_observations(&result)
    );
    assert_eq!(
        observed.runtime_fuel_core,
        extract_fuel_core_observations(&result)
    );
    assert_eq!(
        observed.runtime_ignition_trim,
        extract_ignition_trim_observations(&result)
    );
    assert_eq!(
        observed.runtime_authority,
        extract_authority_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_engine,
        extract_engine_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_control,
        extract_control_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_calibration,
        extract_calibration_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_calibration_identity,
        extract_calibration_identity_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_scheduler,
        extract_scheduler_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_output_profile,
        extract_output_profile_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_fuel_strategy,
        extract_fuel_strategy_observations(&snapshot)
    );
    assert_eq!(observed.runtime_idle, extract_idle_observations(&result));
    assert_eq!(
        observed.runtime_actions,
        extract_action_observations(&result)
    );
    assert_eq!(
        observed.runtime_transitions,
        extract_transition_observations(&result)
    );
    assert_eq!(
        observed.runtime_validated,
        extract_validated_observations(&result)
    );
    assert_eq!(observed.runtime_cut.reason, RuntimeCutReason::None);
    assert!(!observed.runtime_cut.fuel_cut);
    assert!(!observed.runtime_cut.spark_cut);
    assert_eq!(
        observed.runtime_protection,
        RuntimeProtectionObservations::default()
    );
    assert_eq!(observed.runtime_fault, RuntimeFaultObservations::default());
    assert_eq!(
        observed.runtime_torque,
        extract_torque_observations(&result)
    );
    assert_eq!(
        observed.runtime_ignition,
        extract_ignition_observations(&result)
    );
    assert_eq!(
        observed.runtime_knock,
        extract_knock_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_fuel.base_fuel_pw_us,
        observed.runtime_base_fuel_pw_us
    );
    assert_eq!(
        observed.runtime_fuel.enriched_fuel_pw_us,
        observed.runtime_enriched_fuel_pw_us
    );
    assert_eq!(
        observed.runtime_fuel.lambda_target_x100,
        observed.runtime_lambda_target_x100
    );
    assert!(!observed.runtime_fuel.startup_active);
    assert!(observed.runtime_fuel.warmup_active);
    assert_eq!(observed.runtime_fuel.warmup_correction_x100, 120);
    assert_eq!(
        observed.runtime_fuel.warmup_temperature_mode,
        ecu_control::FuelWarmupTemperatureMode::ColdClamp
    );
    assert!(observed.runtime_fuel.afterstart_active);
    assert_eq!(observed.runtime_fuel.afterstart_window_remaining, 2);
    assert_eq!(
        observed.runtime_fuel.afterstart_window_mode,
        ecu_control::FuelAfterstartWindowMode::Cycles
    );
    assert!(observed.runtime_fuel.transient_enrichment_active);
    assert_eq!(observed.runtime_fuel.transient_enrichment_pulse_us, 250);
    assert_eq!(
        observed
            .runtime_fuel
            .transient_enrichment_decay_steps_remaining,
        2
    );
}
#[test]
fn extract_control_observations_preserve_normal_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_control, snapshot.control);
}
#[test]
fn extract_control_observations_preserve_updated_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_warmup_enrichment(WarmupConfig {
        start_c: -20,
        end_c: 20,
        max_percent_x100: 180,
        min_percent_x100: 100,
    });
    runtime.configure_lambda_trim(LambdaTrimConfig {
        closed_loop_target: Lambda100::new(105),
        min_trim_x100: 90,
        max_trim_x100: 130,
        gain_x10: 10,
        ..LambdaTrimConfig::DEFAULT
    });
    runtime.configure_dwell(DwellConfig {
        base_dwell_us: 3200,
        min_dwell_us: 1200,
        max_dwell_us: 4200,
        rpm_dwell_trim_us: 0,
        rpm_trim_start: Rpm::new(1000),
        rpm_trim_end: Rpm::new(8000),
    });

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 3000,
            load_kpa10: 700,
            angle_x10: 1200,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_control, snapshot.control);
    assert_eq!(observed.runtime_control.lambda_target, Lambda100::new(105));
    assert_eq!(observed.runtime_control.dwell, DwellUs::new(3200));
}
#[test]
fn extract_calibration_observations_preserve_default_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_calibration, snapshot.calibration);
    assert_eq!(observed.runtime_calibration, CalibrationState::default());
    assert_eq!(
        observed.runtime_calibration_identity,
        CalibrationPackageIdentity::from_snapshot(snapshot.calibration.active)
    );
    assert_eq!(
        observed.runtime_calibration_identity.active_revision,
        CalibrationRevision::default()
    );
    assert_eq!(
        observed.runtime_calibration_identity.staged_base_revision,
        CalibrationRevision::default()
    );
    assert_eq!(
        observed.runtime_calibration_identity.staged_revision,
        CalibrationRevision::default()
    );
    assert!(!observed.runtime_calibration_identity.staged_dirty);
    assert_ne!(observed.runtime_calibration_identity.checksum.get(), 0);
}
#[test]
fn extract_calibration_identity_observations_use_runtime_dirty_flag() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let calibration_snapshot = CalibrationSnapshot {
        active: ActiveCalibration::new(CalibrationRevision::new(11), Calibration::default()),
        staged: StagedCalibration::new(CalibrationRevision::new(11), Calibration::default()),
    };

    runtime.calibration.active = calibration_snapshot;
    runtime.set_staged_dirty(true);

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert!(observed.runtime_calibration.staged_dirty);
    assert_eq!(
        observed.runtime_calibration_identity.active_revision,
        CalibrationRevision::new(11)
    );
    assert_eq!(
        observed.runtime_calibration_identity.staged_base_revision,
        CalibrationRevision::new(11)
    );
    assert_eq!(
        observed.runtime_calibration_identity.staged_revision,
        CalibrationRevision::new(11)
    );
    assert!(observed.runtime_calibration_identity.staged_dirty);
    assert_eq!(
        observed.runtime_calibration_identity,
        CalibrationPackageIdentity::from_snapshot_with_staged_dirty(calibration_snapshot, true)
    );
    assert_ne!(
        observed.runtime_calibration_identity.checksum,
        CalibrationPackageIdentity::from_snapshot(calibration_snapshot).checksum
    );
}
#[test]
fn extract_calibration_observations_preserve_updated_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let active = ActiveCalibration::new(CalibrationRevision::new(7), Calibration::default());
    let mut staged = StagedCalibration::new(CalibrationRevision::new(7), Calibration::default());
    staged.mark_dirty();
    let calibration_snapshot = CalibrationSnapshot { active, staged };

    runtime.calibration.active = calibration_snapshot;
    runtime.set_staged_dirty(true);

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_calibration, snapshot.calibration);
    assert_eq!(observed.runtime_calibration.active, calibration_snapshot);
    assert!(observed.runtime_calibration.staged_dirty);
    assert_eq!(
        observed.runtime_calibration_identity,
        CalibrationPackageIdentity::from_snapshot(calibration_snapshot)
    );
    assert_eq!(
        observed.runtime_calibration_identity.active_revision,
        CalibrationRevision::new(7)
    );
    assert_eq!(
        observed.runtime_calibration_identity.staged_base_revision,
        CalibrationRevision::new(7)
    );
    assert_eq!(
        observed.runtime_calibration_identity.staged_revision,
        CalibrationRevision::new(8)
    );
    assert!(observed.runtime_calibration_identity.staged_dirty);
    assert_eq!(runtime.calibration_snapshot(), calibration_snapshot);
}
#[test]
fn extract_action_observations_preserve_idle_publish_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
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
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(3_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(3_000),
                clt_c: 80,
                just_started: false,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(100),
                0,
                0,
                0,
                false,
                Rpm::new(0),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_actions,
        extract_action_observations(&result)
    );
    assert_eq!(observed.runtime_actions.total_action_count, 2);
    assert_eq!(observed.runtime_actions.idle_count, 1);
    assert!(observed.runtime_actions.publish_snapshot);
    assert_eq!(observed.runtime_actions.publish_snapshot_count, 1);
    assert!(!observed.runtime_actions.persist_calibration);
    assert!(!observed.runtime_actions.cancel_scheduler);
}
#[test]
fn extract_action_observations_preserve_mixed_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(validated_expert_authority());
    runtime.calibration.staged_dirty = true;
    runtime.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_actions,
        extract_action_observations(&result)
    );
    assert_eq!(observed.runtime_actions.total_action_count, 15);
    assert_eq!(observed.runtime_actions.arm_scheduler_count, 0);
    assert_eq!(observed.runtime_actions.arm_injection_count, 6);
    assert_eq!(observed.runtime_actions.arm_ignition_count, 6);
    assert_eq!(observed.runtime_actions.apply_aux_count, 1);
    assert_eq!(observed.runtime_actions.apply_aux_command_count, 4);
    assert_eq!(observed.runtime_actions.persist_calibration_count, 1);
    assert!(observed.runtime_actions.persist_calibration);
    assert_eq!(observed.runtime_actions.publish_snapshot_count, 1);
    assert!(observed.runtime_actions.publish_snapshot);
    assert!(!observed.runtime_actions.cancel_scheduler);
    assert_eq!(observed.runtime_actions.idle_count, 0);
}
#[test]
fn extract_action_observations_preserve_cancel_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_actions,
        extract_action_observations(&result)
    );
    assert_eq!(observed.runtime_actions.total_action_count, 2);
    assert!(observed.runtime_actions.cancel_scheduler);
    assert_eq!(
        observed.runtime_actions.cancel_reason,
        CancelReason::SafetyShutdown
    );
    assert_eq!(observed.runtime_actions.cancel_scheduler_count, 1);
    assert!(!observed.runtime_actions.multiple_cancel_reasons);
    assert!(observed.runtime_actions.publish_snapshot);
    assert_eq!(observed.runtime_actions.publish_snapshot_count, 1);
    assert_eq!(observed.runtime_actions.idle_count, 0);
}
#[test]
fn extract_transition_observations_preserve_single_point_injection_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_single_point_injection();
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, false),
        running_control_inputs(5_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);
    let injection = result
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmInjection(injection) => Some(injection),
            _ => None,
        })
        .expect("single-point injection action");

    assert_eq!(
        observed.runtime_transitions,
        extract_transition_observations(&result)
    );
    assert_eq!(observed.runtime_transitions.total_transition_count, 2);
    assert_eq!(observed.runtime_transitions.injector_transition_count, 2);
    assert_eq!(observed.runtime_transitions.ignition_transition_count, 0);
    assert_eq!(
        observed.runtime_transitions.earliest_transition_at_us,
        Some(injection.start_at)
    );
    assert_eq!(
        observed.runtime_transitions.latest_transition_at_us,
        Some(injection.end_at)
    );
    assert_eq!(
        observed
            .runtime_transitions
            .earliest_injector_transition_at_us,
        Some(injection.start_at)
    );
    assert_eq!(
        observed
            .runtime_transitions
            .latest_injector_transition_at_us,
        Some(injection.end_at)
    );
    assert_eq!(
        observed
            .runtime_transitions
            .earliest_ignition_transition_at_us,
        None
    );
    assert_eq!(
        observed
            .runtime_transitions
            .latest_ignition_transition_at_us,
        None
    );
    assert!(!observed.runtime_transitions.export_error);
}
#[test]
fn extract_transition_observations_preserve_full_ecu_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(validated_expert_authority());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);
    let mut earliest_transition: Option<Micros> = None;
    let mut latest_transition: Option<Micros> = None;
    let mut earliest_injector: Option<Micros> = None;
    let mut latest_injector: Option<Micros> = None;
    let mut earliest_ignition: Option<Micros> = None;
    let mut latest_ignition: Option<Micros> = None;
    let mut injector_transition_count = 0u8;
    let mut ignition_transition_count = 0u8;

    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(injection) => {
                injector_transition_count = injector_transition_count.saturating_add(2);
                earliest_transition = Some(match earliest_transition {
                    Some(current) if current.get() <= injection.start_at.get() => current,
                    _ => injection.start_at,
                });
                earliest_transition = Some(match earliest_transition {
                    Some(current) if current.get() <= injection.end_at.get() => current,
                    _ => injection.end_at,
                });
                latest_transition = Some(match latest_transition {
                    Some(current) if current.get() >= injection.start_at.get() => current,
                    _ => injection.start_at,
                });
                latest_transition = Some(match latest_transition {
                    Some(current) if current.get() >= injection.end_at.get() => current,
                    _ => injection.end_at,
                });
                earliest_injector = Some(match earliest_injector {
                    Some(current) if current.get() <= injection.start_at.get() => current,
                    _ => injection.start_at,
                });
                earliest_injector = Some(match earliest_injector {
                    Some(current) if current.get() <= injection.end_at.get() => current,
                    _ => injection.end_at,
                });
                latest_injector = Some(match latest_injector {
                    Some(current) if current.get() >= injection.start_at.get() => current,
                    _ => injection.start_at,
                });
                latest_injector = Some(match latest_injector {
                    Some(current) if current.get() >= injection.end_at.get() => current,
                    _ => injection.end_at,
                });
            }
            Action::ArmIgnition(ignition) => {
                ignition_transition_count = ignition_transition_count.saturating_add(2);
                earliest_transition = Some(match earliest_transition {
                    Some(current) if current.get() <= ignition.start_at.get() => current,
                    _ => ignition.start_at,
                });
                earliest_transition = Some(match earliest_transition {
                    Some(current) if current.get() <= ignition.end_at.get() => current,
                    _ => ignition.end_at,
                });
                latest_transition = Some(match latest_transition {
                    Some(current) if current.get() >= ignition.start_at.get() => current,
                    _ => ignition.start_at,
                });
                latest_transition = Some(match latest_transition {
                    Some(current) if current.get() >= ignition.end_at.get() => current,
                    _ => ignition.end_at,
                });
                earliest_ignition = Some(match earliest_ignition {
                    Some(current) if current.get() <= ignition.start_at.get() => current,
                    _ => ignition.start_at,
                });
                earliest_ignition = Some(match earliest_ignition {
                    Some(current) if current.get() <= ignition.end_at.get() => current,
                    _ => ignition.end_at,
                });
                latest_ignition = Some(match latest_ignition {
                    Some(current) if current.get() >= ignition.start_at.get() => current,
                    _ => ignition.start_at,
                });
                latest_ignition = Some(match latest_ignition {
                    Some(current) if current.get() >= ignition.end_at.get() => current,
                    _ => ignition.end_at,
                });
            }
            _ => {}
        }
    }

    assert_eq!(
        observed.runtime_transitions,
        extract_transition_observations(&result)
    );
    assert_eq!(observed.runtime_transitions.total_transition_count, 24);
    assert_eq!(
        observed.runtime_transitions.injector_transition_count,
        injector_transition_count
    );
    assert_eq!(
        observed.runtime_transitions.ignition_transition_count,
        ignition_transition_count
    );
    assert_eq!(
        observed.runtime_transitions.earliest_transition_at_us,
        earliest_transition
    );
    assert_eq!(
        observed.runtime_transitions.latest_transition_at_us,
        latest_transition
    );
    assert_eq!(
        observed
            .runtime_transitions
            .earliest_injector_transition_at_us,
        earliest_injector
    );
    assert_eq!(
        observed
            .runtime_transitions
            .latest_injector_transition_at_us,
        latest_injector
    );
    assert_eq!(
        observed
            .runtime_transitions
            .earliest_ignition_transition_at_us,
        earliest_ignition
    );
    assert_eq!(
        observed
            .runtime_transitions
            .latest_ignition_transition_at_us,
        latest_ignition
    );
    assert!(!observed.runtime_transitions.export_error);
}
#[test]
fn extract_scheduler_observations_preserve_idle_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
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
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_scheduler,
        extract_scheduler_observations(&snapshot)
    );
    assert_eq!(observed.runtime_scheduler, runtime.scheduler_state());
    assert_eq!(
        observed.runtime_scheduler.mode(),
        ecu_scheduler::SchedulerMode::Idle
    );
    assert!(!observed.runtime_scheduler.is_armed());
    assert_eq!(observed.runtime_scheduler.injection_count(), 0);
    assert_eq!(observed.runtime_scheduler.ignition_count(), 0);
}
#[test]
fn extract_scheduler_observations_preserve_armed_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_batch_injection(4);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, false),
        running_control_inputs(5_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_scheduler,
        extract_scheduler_observations(&snapshot)
    );
    assert_eq!(observed.runtime_scheduler, runtime.scheduler_state());
    assert_eq!(
        observed.runtime_scheduler.mode(),
        ecu_scheduler::SchedulerMode::Armed
    );
    assert!(observed.runtime_scheduler.is_armed());
    assert_eq!(observed.runtime_scheduler.injection_count(), 4);
    assert_eq!(observed.runtime_scheduler.ignition_count(), 0);
    assert!(observed.runtime_scheduler.last_injection_start().is_some());
    assert!(observed.runtime_scheduler.last_injection_end().is_some());
    assert_eq!(observed.runtime_scheduler.last_ignition_start(), None);
    assert_eq!(observed.runtime_scheduler.last_ignition_end(), None);
}
#[test]
fn extract_scheduler_observations_preserve_suspended_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_batch_injection(4);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let _ = runtime.step(
        running_step_inputs(1_000, 3_000, true, false),
        running_control_inputs(1_000, 3_000),
    );

    let result = runtime.step(
        running_step_inputs(2_000, 0, false, false),
        running_control_inputs(2_000, 0),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_scheduler,
        extract_scheduler_observations(&snapshot)
    );
    assert_eq!(observed.runtime_scheduler, runtime.scheduler_state());
    assert_eq!(
        observed.runtime_scheduler.mode(),
        ecu_scheduler::SchedulerMode::Suspended
    );
    assert!(!observed.runtime_scheduler.is_armed());
    assert_eq!(
        observed.runtime_scheduler.active_stop_reason(),
        ecu_board_api::frontier::TimingIslandStopReason::SyncLost
    );
    assert_eq!(observed.runtime_scheduler.injection_count(), 0);
    assert_eq!(observed.runtime_scheduler.ignition_count(), 0);
    assert!(observed.runtime_scheduler.last_injection_start().is_some());
    assert!(observed.runtime_scheduler.last_injection_end().is_some());
}
#[test]
fn extract_output_profile_observations_preserve_default_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
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
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_output_profile,
        extract_output_profile_observations(&snapshot)
    );
    assert_eq!(observed.runtime_output_profile, runtime.output_profile());
    assert_eq!(
        observed.runtime_output_profile,
        ecu_board_api::legacy::single_channel_runtime_output_profile()
    );
}
#[test]
fn extract_output_profile_observations_preserve_updated_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    let profile = inline_sequential_cop_profile();
    runtime.configure_full_ecu(profile);
    runtime.set_engine_time_authority(validated_expert_authority());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_output_profile,
        extract_output_profile_observations(&snapshot)
    );
    assert_eq!(observed.runtime_output_profile, runtime.output_profile());
    assert_eq!(
        observed.runtime_output_profile,
        RuntimeOutputProfile::full_ecu(profile)
    );
}
#[test]
fn extract_fuel_strategy_observations_preserve_default_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(4_000),
            rpm: 0,
            load_kpa10: 0,
            angle_x10: 0,
            trigger_synced: false,
            cam_seen: false,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(4_000, 0),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fuel_strategy,
        extract_fuel_strategy_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_fuel_strategy,
        RuntimeFuelStrategyMode::DirectPulseWidthTable
    );
}
#[test]
fn extract_fuel_strategy_observations_preserve_updated_runtime_state() {
    let mut runtime = EngineRuntime::new();
    let calibration = runtime_semantic_calibration_from_fuel_tune(&FuelRuntimeTune::new(
        [[100; 16]; 16],
        [[147; 16]; 16],
        2_400,
        800,
        0,
    ));
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let result = runtime.step(
        running_step_inputs(2_500, 3_000, true, true),
        running_control_inputs(2_500, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fuel_strategy,
        extract_fuel_strategy_observations(&snapshot)
    );
    assert_eq!(
        observed.runtime_fuel_strategy,
        RuntimeFuelStrategyMode::SpeedDensityVe
    );
}
#[test]
fn extract_idle_observations_preserve_direct_path_as_inactive() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(4_500, 3_000, true, true),
        running_control_inputs(4_500, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_idle, extract_idle_observations(&result));
    assert!(!observed.runtime_idle.active);
    assert_eq!(observed.runtime_idle.duty_x1000, 0);
    assert_eq!(observed.runtime_idle.integrator_acc, 0);
    assert_eq!(observed.runtime_idle.integrator_min_acc, 0);
    assert_eq!(observed.runtime_idle.integrator_max_acc, 0);
    assert!(!observed.runtime_idle.integrator_frozen);
}
#[test]
fn extract_idle_observations_preserve_semantic_idle_state() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.idle_target_rpm = 1000;
    calibration.idle_base_duty_x1000 = 350;
    calibration.idle_kp_x1000 = 300;
    calibration.idle_ki_x1000 = 500;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let mut inputs = running_control_inputs(4_500, 700);
    inputs.enrichment.clt_c = 80;
    let result = runtime.step(running_step_inputs(4_500, 700, true, true), inputs);
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_idle, extract_idle_observations(&result));
    assert!(observed.runtime_idle.active);
    assert_eq!(observed.runtime_idle.duty_x1000, 590);
    assert_eq!(observed.runtime_idle.integrator_acc, 150);
    assert_eq!(observed.runtime_idle.integrator_min_acc, -2000);
    assert_eq!(observed.runtime_idle.integrator_max_acc, 2000);
    assert!(!observed.runtime_idle.integrator_frozen);
}
#[test]
fn extract_lambda_correction_observations_preserve_direct_path_as_inactive() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let mut inputs = running_control_inputs(4_750, 3_000);
    inputs.lambda.requested_open_loop = true;
    let result = runtime.step(running_step_inputs(4_750, 3_000, true, true), inputs);
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_lambda_correction,
        extract_lambda_correction_observations(&result)
    );
    assert!(!observed.runtime_lambda_correction.active);
    assert_eq!(observed.runtime_lambda_correction.correction_x1000, 1000);
    assert!(!observed.runtime_lambda_correction.integrator_available);
    assert_eq!(observed.runtime_lambda_correction.integrator_acc, 0);
    assert_eq!(observed.runtime_lambda_correction.integrator_min_acc, 0);
    assert_eq!(observed.runtime_lambda_correction.integrator_max_acc, 0);
    assert!(!observed.runtime_lambda_correction.integrator_frozen);
}
#[test]
fn extract_lambda_correction_observations_preserve_semantic_correction_state() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.lambda_kp_x1000 = 0;
    calibration.lambda_ki_x1000 = 1000;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let mut inputs = running_control_inputs(4_750, 3_000);
    inputs.enrichment.clt_c = 90;
    inputs.lambda.clt_c = 90;
    inputs.lambda.measured_lambda100 = Lambda100::new(95);
    let result = runtime.step(running_step_inputs(4_750, 3_000, true, true), inputs);
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_lambda_correction,
        extract_lambda_correction_observations(&result)
    );
    assert!(observed.runtime_lambda_correction.active);
    assert_eq!(observed.runtime_lambda_correction.correction_x1000, 1050);
    assert!(observed.runtime_lambda_correction.integrator_available);
    assert_eq!(observed.runtime_lambda_correction.integrator_acc, 50);
    assert_eq!(observed.runtime_lambda_correction.integrator_min_acc, -2000);
    assert_eq!(observed.runtime_lambda_correction.integrator_max_acc, 2000);
    assert!(!observed.runtime_lambda_correction.integrator_frozen);
}
#[test]
fn extract_ignition_trim_observations_preserve_direct_path_as_inactive() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_ignition_trim,
        extract_ignition_trim_observations(&result)
    );
    assert!(!observed.runtime_ignition_trim.active);
    assert_eq!(observed.runtime_ignition_trim.trim_deg10, 0);
}
#[test]
fn extract_ignition_trim_observations_preserve_semantic_trim_state() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.soft_rev_rpm = 2_500;
    calibration.soft_retard_max_deg10 = 120;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_ignition_trim,
        extract_ignition_trim_observations(&result)
    );
    assert!(observed.runtime_ignition_trim.active);
    assert_eq!(observed.runtime_ignition_trim.trim_deg10, -120);
}
#[test]
fn extract_fuel_core_observations_preserve_direct_path_fallback() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(5_250, 3_000, true, true),
        running_control_inputs(5_250, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fuel_core,
        extract_fuel_core_observations(&result)
    );
    assert!(!observed.runtime_fuel_core.semantic_available);
    assert_eq!(observed.runtime_fuel_core.ve_pct_x100, None);
    assert_eq!(observed.runtime_fuel_core.target_afr_x100, None);
    assert_eq!(observed.runtime_fuel_core.pw_air_us, None);
    assert_eq!(
        observed.runtime_fuel_core.pw_base_us,
        result.control.fuel_intent.observations.pw_base_us
    );
    assert_eq!(
        observed.runtime_fuel_core.pw_corr_us,
        result.control.fuel_intent.observations.pw_corr_us
    );
}
#[test]
fn extract_fuel_core_observations_preserve_semantic_core_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 7000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step(
        running_step_inputs(5_250, 3_000, true, true),
        running_control_inputs(5_250, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fuel_core,
        extract_fuel_core_observations(&result)
    );
    assert!(observed.runtime_fuel_core.semantic_available);
    assert_eq!(observed.runtime_fuel_core.ve_pct_x100, Some(7000));
    assert_eq!(observed.runtime_fuel_core.target_afr_x100, Some(1470));
    assert_eq!(observed.runtime_fuel_core.pw_base_us, 700);
    assert_eq!(observed.runtime_fuel_core.pw_air_us, Some(490));
    assert_eq!(observed.runtime_fuel_core.pw_corr_us, 490);
}
#[test]
fn extract_runtime_observed_surface_preserves_direct_cut_state_shell() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_direct_cut_requests(true, false);

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert!(snapshot.direct_fuel_cut_request);
    assert!(!snapshot.direct_spark_cut_request);
    assert_eq!(observed.runtime_cut.reason, RuntimeCutReason::DirectRequest);
    assert_eq!(observed.runtime_cut.fuel_cut, snapshot.fuel_cut);
    assert_eq!(observed.runtime_cut.spark_cut, snapshot.spark_cut);
    assert!(observed.runtime_cut.fuel_cut);
    assert!(!observed.runtime_cut.spark_cut);
    assert_eq!(observed.runtime_torque, result.torque_observations);
    assert_eq!(observed.runtime_torque.actuated_x1000, 0);
}
#[test]
fn extract_ignition_observations_preserve_normal_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_ignition, result.control.ignition);
}
#[test]
fn extract_ignition_observations_preserve_limited_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let mut control = running_control_inputs(1_000, 3_000);
    control.ignition = IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, true, Rpm::new(3_000));
    let result = runtime.step(running_step_inputs(1_000, 3_000, true, true), control);
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_ignition, result.control.ignition);
    assert_eq!(
        observed.runtime_ignition.limit_reason,
        IgnitionLimitReason::RevLimiter
    );
}
#[test]
fn extract_knock_observations_preserve_inactive_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_knock,
        RuntimeKnockObservations {
            intensity_x100: 0,
            retard_deg10: 0,
        }
    );
}
#[test]
fn extract_knock_observations_preserve_active_runtime_state() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.knock_threshold_x100 = 500;
    calibration.knock_retard_step_deg10 = 40;
    calibration.knock_retard_max_deg10 = 120;
    calibration.knock_recovery_step_deg10 = 40;
    calibration.knock_recovery_delay_cycles = 0;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let mut inputs = running_control_inputs(1_800, 3_000);
    inputs.knock_intensity_x100 = 600;
    let result = runtime.step(running_step_inputs(1_800, 3_000, true, true), inputs);
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_knock,
        RuntimeKnockObservations {
            intensity_x100: 600,
            retard_deg10: 40,
        }
    );
}
#[test]
fn extract_runtime_observed_surface_preserves_semantic_hard_rev_cut_state_shell() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.hard_rev_rpm = 2_500;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let result = runtime.step(
        running_step_inputs(1_500, 3_000, true, true),
        running_control_inputs(1_500, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert!(snapshot.rev_hard_active);
    assert_eq!(observed.runtime_cut.reason, RuntimeCutReason::HardRev);
    assert_eq!(observed.runtime_cut.fuel_cut, snapshot.fuel_cut);
    assert_eq!(observed.runtime_cut.spark_cut, snapshot.spark_cut);
    assert!(observed.runtime_cut.fuel_cut || observed.runtime_cut.spark_cut);
}
#[test]
fn extract_cut_observations_preserves_knock_retard_reason_without_active_cuts() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.knock_threshold_x100 = 500;
    calibration.knock_retard_step_deg10 = 40;
    calibration.knock_retard_max_deg10 = 120;
    calibration.knock_recovery_step_deg10 = 40;
    calibration.knock_recovery_delay_cycles = 0;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let mut inputs = running_control_inputs(1_800, 3_000);
    inputs.knock_intensity_x100 = 600;
    let _ = runtime.step(running_step_inputs(1_800, 3_000, true, true), inputs);
    let snapshot = runtime.snapshot();

    let cut = extract_cut_observations(&snapshot);

    assert_eq!(cut.reason, RuntimeCutReason::KnockRetard);
    assert!(!cut.fuel_cut);
    assert!(!cut.spark_cut);
}
#[test]
fn extract_engine_observations_preserve_normal_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_engine, snapshot.engine);
}
#[test]
fn extract_engine_observations_preserve_unsynced_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(1_000),
            3_000,
            700,
            2_000,
            EngineTimeAuthority::none(),
            false,
            false,
            false,
        ),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(observed.runtime_engine, snapshot.engine);
    assert_eq!(observed.runtime_engine.sync, SyncState::Unsynced);
}
#[test]
fn extract_protection_observations_is_inactive_for_normal_running_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);
    let fault = extract_fault_observations(&snapshot);
    let protection = extract_protection_observations(&snapshot);

    assert_eq!(fault, RuntimeFaultObservations::default());
    assert_eq!(observed.runtime_fault, fault);
    assert_eq!(protection, RuntimeProtectionObservations::default());
    assert_eq!(observed.runtime_protection, protection);
}
#[test]
fn extract_validated_observations_preserve_unclamped_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_validated,
        ValidatedInputs {
            rpm: Rpm::new(3_000),
            load_kpa10: Kpa10::new(700),
            angle_x10: Degrees10::new(2_000),
            clamped: false,
        }
    );
}
#[test]
fn extract_validated_observations_preserve_clamped_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 50_000,
            load_kpa10: 5_000,
            angle_x10: 8_000,
            trigger_synced: false,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_validated,
        ValidatedInputs {
            rpm: Rpm::new(9_000),
            load_kpa10: Kpa10::new(2_000),
            angle_x10: Degrees10::new(7_200),
            clamped: true,
        }
    );
}
#[test]
fn extract_authority_observations_preserve_unsynced_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(1_000),
            3_000,
            700,
            2_000,
            EngineTimeAuthority::none(),
            false,
            false,
            false,
        ),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_authority,
        RuntimeAuthorityObservations {
            authority: EngineTimeAuthority::none(),
            summary: SyncState::Unsynced,
            phase: snapshot.engine.phase,
            full_sequential_authorized: false,
        }
    );
}
#[test]
fn extract_authority_observations_preserve_crank_only_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    let authority = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    );

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(2_000),
            3_000,
            700,
            2_000,
            authority,
            false,
            false,
            false,
        ),
        running_control_inputs(2_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_authority,
        RuntimeAuthorityObservations {
            authority,
            summary: SyncState::Locked { cam_ref: false },
            phase: snapshot.engine.phase,
            full_sequential_authorized: false,
        }
    );
}
#[test]
fn extract_authority_observations_preserve_validated_runtime_state() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    let authority = validated_expert_authority();

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(3_000),
            3_000,
            700,
            2_000,
            authority,
            false,
            false,
            false,
        ),
        running_control_inputs(3_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_authority,
        RuntimeAuthorityObservations {
            authority,
            summary: SyncState::Locked { cam_ref: false },
            phase: snapshot.engine.phase,
            full_sequential_authorized: true,
        }
    );
}
#[test]
fn extract_runtime_fault_observations_preserve_warning_fault_as_limp_home() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fault,
        RuntimeFaultObservations {
            active: true,
            fault: FaultCode::SensorOutOfRange,
            severity: FaultSeverity::Warning,
            cancel_reason: CancelReason::Manual,
            action: RuntimeFaultAction::LimpHome,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        observed.runtime_protection,
        RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::Degraded,
            source: RuntimeProtectionSource::RuntimeFault,
            action: RuntimeProtectionAction::LimpHome,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
}
#[test]
fn extract_runtime_fault_observations_preserve_critical_fault_as_shutdown() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fault,
        RuntimeFaultObservations {
            active: true,
            fault: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
            action: RuntimeFaultAction::Shutdown,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        observed.runtime_protection,
        RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::ShutdownDriving,
            source: RuntimeProtectionSource::RuntimeFault,
            action: RuntimeProtectionAction::Shutdown,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
}
#[test]
fn extract_runtime_fault_observations_preserve_safety_cut_as_shutdown_regardless_of_severity() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Info,
        CancelReason::Manual,
    );

    let result = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_fault,
        RuntimeFaultObservations {
            active: true,
            fault: FaultCode::SafetyCut,
            severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            action: RuntimeFaultAction::Shutdown,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        observed.runtime_protection,
        RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::ShutdownDriving,
            source: RuntimeProtectionSource::RuntimeFault,
            action: RuntimeProtectionAction::Shutdown,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
}
#[test]
fn extract_protection_observations_preserves_safety_latch_as_output_suppressed() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        StepInputs {
            safety_latch_request: true,
            ..running_step_inputs(1_000, 3_000, true, true)
        },
        running_control_inputs(1_000, 3_000),
    );
    let snapshot = runtime.snapshot();

    let observed = extract_runtime_observed_surface(&result, &snapshot);

    assert_eq!(
        observed.runtime_protection,
        RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::ShutdownDriving,
            source: RuntimeProtectionSource::SafetyLatch,
            action: RuntimeProtectionAction::OutputSuppressed,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        }
    );
}
#[test]
fn torque_observations_zero_allowed_when_engine_is_off() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(500),
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
                now_us: Micros::new(500),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(500),
                clt_c: 80,
                just_started: false,
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
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    assert_eq!(result.operating_mode, ControlMode::OpenLoop);
    assert_eq!(result.torque_observations.request_x1000, 750);
    assert_eq!(result.torque_observations.allowed_x1000, 0);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn differential_input_snapshot_preserves_all_fixture_fields() {
    use crate::{RuntimeAfrOverride, RuntimeEngineMode};

    let cases = [
        // Fields: now_us, rpm, map_kpa10, load_kpa10, angle_x10, clt_c10, iat_c10,
        //         baro_kpa10, vbatt_mv, sync, fuel_cut, spark_cut, mode, target_afr_override_x100
        (
            DifferentialInputSnapshot {
                now_us: Micros::new(1_000_000),
                rpm: Rpm::new(1500),
                map_kpa10: Kpa10::new(950),
                load_kpa10: Kpa10::new(1000),
                angle_x10: Degrees10::new(2000),
                clt_c10: 800,
                iat_c10: 250,
                baro_kpa10: Kpa10::new(1013),
                vbatt_mv: 12_100,
                sync: SyncState::Locked { cam_ref: false },
                fuel_cut: false,
                spark_cut: false,
                mode: RuntimeEngineMode::Running,
                target_afr_override_x100: RuntimeAfrOverride::None,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            "running synced",
        ),
        (
            DifferentialInputSnapshot {
                now_us: Micros::new(2_000_000),
                rpm: Rpm::new(0),
                map_kpa10: Kpa10::new(0),
                load_kpa10: Kpa10::new(0),
                angle_x10: Degrees10::new(0),
                clt_c10: -120,
                iat_c10: -80,
                baro_kpa10: Kpa10::new(950),
                vbatt_mv: 11_500,
                sync: SyncState::Unsynced,
                fuel_cut: true,
                spark_cut: true,
                mode: RuntimeEngineMode::Off,
                target_afr_override_x100: RuntimeAfrOverride::Some(1470),
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            "cut with negative temps and AFR override",
        ),
        (
            DifferentialInputSnapshot {
                now_us: Micros::new(3_000_000),
                rpm: Rpm::new(500),
                map_kpa10: Kpa10::new(300),
                load_kpa10: Kpa10::new(400),
                angle_x10: Degrees10::new(1000),
                clt_c10: -300,
                iat_c10: -400,
                baro_kpa10: Kpa10::new(850),
                vbatt_mv: 13_500,
                sync: SyncState::Provisional,
                fuel_cut: false,
                spark_cut: false,
                mode: RuntimeEngineMode::Cranking,
                target_afr_override_x100: RuntimeAfrOverride::None,
                launch_armed: false,
                flat_shift_armed: false,
                safety_latch_request: false,
            },
            "cranking with cold temps and syncing",
        ),
    ];

    for (snap, label) in cases {
        // to_step_inputs preserves all fields needed for runtime step
        let step = snap.to_step_inputs();
        assert_eq!(step.now_us, snap.now_us, "now_us for {label}");
        assert_eq!(step.rpm, snap.rpm.get() as u32, "rpm for {label}");
        assert_eq!(
            step.load_kpa10,
            snap.load_kpa10.get() as u32,
            "load_kpa10 for {label}"
        );
        assert_eq!(
            step.angle_x10,
            snap.angle_x10.get() as i32,
            "angle_x10 for {label}"
        );
        assert_eq!(
            step.trigger_synced,
            snap.sync == SyncState::Locked { cam_ref: false },
            "trigger_synced for {label}"
        );
        assert_eq!(
            step.cam_seen,
            snap.sync == SyncState::Locked { cam_ref: false },
            "cam_seen for {label}"
        );
    }
}
