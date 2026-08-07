use super::*;

#[test]
fn runtime_snapshot_cut_state_follows_last_control_plan() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.pref_kpa10 = 0;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let result = runtime.step(
        running_step_inputs(2_000, 3000, true, true),
        running_control_inputs(2_000, 3000),
    );
    let snapshot = runtime.snapshot();

    assert!(result.control.fuel_cut);
    assert!(result.control.spark_cut);
    assert_eq!(snapshot.fuel_cut, result.control.fuel_cut);
    assert_eq!(snapshot.spark_cut, result.control.spark_cut);
}
#[test]
fn semantic_step_direct_fuel_cut_request_sets_snapshot_and_zeroes_actuated_torque() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    runtime.set_direct_cut_requests(true, false);

    let result = runtime.step(
        running_step_inputs(1_250, 3_000, true, true),
        running_control_inputs(1_250, 3_000),
    );

    assert!(runtime.snapshot().fuel_cut);
    assert!(!runtime.snapshot().spark_cut);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn semantic_step_direct_spark_cut_request_sets_snapshot_and_zeroes_actuated_torque() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    runtime.set_direct_cut_requests(false, true);

    let result = runtime.step(
        running_step_inputs(1_500, 3_000, true, true),
        running_control_inputs(1_500, 3_000),
    );

    assert!(!runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn differential_step_off_mode_produces_off_phase_snapshot() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_differential_input(
        DifferentialInputSnapshot {
            mode: RuntimeEngineMode::Off,
            rpm: Rpm::new(3_000),
            sync: SyncState::Locked { cam_ref: false },
            ..differential_running_input(RuntimeEngineMode::Running)
        },
        running_control_inputs(1_000, 3_000),
    );

    assert_eq!(result.operating_mode, ControlMode::OpenLoop);
    assert_eq!(runtime.snapshot().engine.phase, EnginePhase::Off);
}
#[test]
fn differential_step_shutdown_mode_emits_cancel_and_snapshot() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_differential_input(
        DifferentialInputSnapshot {
            mode: RuntimeEngineMode::Shutdown,
            ..differential_running_input(RuntimeEngineMode::Running)
        },
        running_control_inputs(1_000, 3_000),
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SafetyShutdown))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(result.operating_mode, ControlMode::Shutdown);
    assert_eq!(runtime.snapshot().faults.fault, FaultCode::SafetyCut);
    assert_eq!(runtime.snapshot().faults.severity, FaultSeverity::Critical);
    assert!(runtime.legacy_cut_flags().fuel_cut);
    assert!(runtime.legacy_cut_flags().spark_cut);
    assert_eq!(runtime.snapshot().legacy_cut_reason_code, 1);
}
#[test]
fn differential_step_fuel_cut_sets_snapshot_without_forcing_spark_cut() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_differential_input(
        DifferentialInputSnapshot {
            fuel_cut: true,
            ..differential_running_input(RuntimeEngineMode::Running)
        },
        running_control_inputs(1_250, 3_000),
    );

    assert!(runtime.snapshot().fuel_cut);
    assert!(!runtime.snapshot().spark_cut);
    assert!(runtime.legacy_cut_flags().fuel_cut);
    assert!(runtime.legacy_cut_flags().spark_cut);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
    assert_eq!(runtime.snapshot().legacy_cut_reason_code, 1);
}
#[test]
fn differential_step_spark_cut_sets_snapshot_without_forcing_fuel_cut() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_differential_input(
        DifferentialInputSnapshot {
            spark_cut: true,
            ..differential_running_input(RuntimeEngineMode::Running)
        },
        running_control_inputs(1_500, 3_000),
    );

    assert!(!runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);
    assert!(runtime.legacy_cut_flags().fuel_cut);
    assert!(runtime.legacy_cut_flags().spark_cut);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn differential_step_safety_latch_request_sets_snapshot_and_legacy_cut_flags() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_differential_input(
        DifferentialInputSnapshot {
            safety_latch_request: true,
            ..differential_running_input(RuntimeEngineMode::Running)
        },
        running_control_inputs(1_750, 3_000),
    );

    assert!(runtime.snapshot().safety_latched);
    assert!(runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);
    assert!(runtime.legacy_cut_flags().fuel_cut);
    assert!(runtime.legacy_cut_flags().spark_cut);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn semantic_step_clearing_direct_cut_requests_removes_cut_on_next_step() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    runtime.set_direct_cut_requests(true, false);

    let _ = runtime.step(
        running_step_inputs(1_750, 3_000, true, true),
        running_control_inputs(1_750, 3_000),
    );

    runtime.set_direct_cut_requests(false, false);

    let result = runtime.step(
        running_step_inputs(2_000, 3_000, true, true),
        running_control_inputs(2_000, 3_000),
    );

    assert!(!result.control.fuel_cut);
    assert!(!result.control.spark_cut);
    assert!(!runtime.snapshot().fuel_cut);
    assert!(!runtime.snapshot().spark_cut);
}
#[test]
fn semantic_step_safety_latch_request_sets_snapshot_and_zeroes_actuated_torque() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step(
        StepInputs {
            safety_latch_request: true,
            ..running_step_inputs(1_250, 3_000, true, true)
        },
        running_control_inputs(1_250, 3_000),
    );

    assert!(runtime.snapshot().safety_latched);
    assert!(runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn semantic_step_safety_latch_holds_after_clear_attempt_while_running() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let _ = runtime.step(
        StepInputs {
            safety_latch_request: true,
            ..running_step_inputs(1_250, 3_000, true, true)
        },
        running_control_inputs(1_250, 3_000),
    );

    let held = runtime.step(
        StepInputs {
            safety_latch_request: false,
            ..running_step_inputs(1_500, 3_000, true, true)
        },
        running_control_inputs(1_500, 3_000),
    );

    assert!(runtime.snapshot().safety_latched);
    assert!(held.control.fuel_cut);
    assert!(held.control.spark_cut);
}
#[test]
fn semantic_step_safety_latch_releases_after_off_clear() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let _ = runtime.step(
        StepInputs {
            safety_latch_request: true,
            ..running_step_inputs(1_250, 3_000, true, true)
        },
        running_control_inputs(1_250, 3_000),
    );

    let _ = runtime.step(
        StepInputs {
            safety_latch_request: false,
            ..running_step_inputs(1_750, 0, false, false)
        },
        running_control_inputs(1_750, 0),
    );

    let cleared = runtime.step(
        StepInputs {
            safety_latch_request: false,
            ..running_step_inputs(2_000, 3_000, true, true)
        },
        running_control_inputs(2_000, 3_000),
    );

    assert!(!runtime.snapshot().safety_latched);
    assert!(!cleared.control.fuel_cut);
    assert!(!cleared.control.spark_cut);
}
#[test]
fn semantic_hard_rev_sets_snapshot_flag_and_zeroes_allowed_torque() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.hard_rev_rpm = 2_500;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let result = runtime.step(
        running_step_inputs(1_500, 3_000, true, true),
        running_control_inputs(1_500, 3_000),
    );
    let snapshot = runtime.snapshot();

    assert!(snapshot.rev_hard_active);
    assert!(!snapshot.rev_soft_active);
    assert_eq!(result.torque_observations.request_x1000, 900);
    assert_eq!(result.torque_observations.allowed_x1000, 0);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn semantic_soft_rev_sets_snapshot_flag_and_zeroes_actuated_torque() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.soft_rev_rpm = 2_500;
    calibration.hard_rev_rpm = 10_000;
    calibration.soft_retard_max_deg10 = 150;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let result = runtime.step(
        running_step_inputs(1_750, 3_000, true, true),
        running_control_inputs(1_750, 3_000),
    );
    let snapshot = runtime.snapshot();

    assert!(snapshot.rev_soft_active);
    assert!(!snapshot.rev_hard_active);
    assert_eq!(result.torque_observations.request_x1000, 900);
    assert_eq!(result.torque_observations.allowed_x1000, 900);
    assert_eq!(result.torque_observations.actuated_x1000, 0);
}
#[test]
fn semantic_step_publishes_knock_snapshot_fields() {
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

    assert_eq!(snapshot.knock_intensity_x100, 600);
    assert_eq!(snapshot.knock_retard_deg10, 40);
    assert_eq!(snapshot.legacy_cut_reason_code, 7);
    assert!(!snapshot.fuel_cut);
    assert!(!snapshot.spark_cut);
}
#[test]
fn semantic_step_detects_knock_legacy_reason_without_retard_accumulation() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.knock_threshold_x100 = 500;
    calibration.knock_retard_step_deg10 = 0;
    calibration.knock_retard_max_deg10 = 120;
    calibration.knock_recovery_step_deg10 = 40;
    calibration.knock_recovery_delay_cycles = 0;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let mut inputs = running_control_inputs(1_850, 3_000);
    inputs.knock_intensity_x100 = 600;

    let _ = runtime.step(running_step_inputs(1_850, 3_000, true, true), inputs);
    let snapshot = runtime.snapshot();

    assert_eq!(snapshot.knock_intensity_x100, 600);
    assert_eq!(snapshot.knock_retard_deg10, 0);
    assert_eq!(snapshot.legacy_cut_reason_code, 7);
}
#[test]
fn semantic_step_below_threshold_does_not_emit_knock_legacy_reason() {
    let mut runtime = EngineRuntime::new();
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 9000);
    calibration.knock_threshold_x100 = 500;
    calibration.knock_retard_step_deg10 = 40;
    calibration.knock_retard_max_deg10 = 120;
    calibration.knock_recovery_step_deg10 = 40;
    calibration.knock_recovery_delay_cycles = 0;
    runtime.configure_speed_density_ve(calibration, RuntimeSemanticState::default());

    let mut inputs = running_control_inputs(1_860, 3_000);
    inputs.knock_intensity_x100 = 400;

    let _ = runtime.step(running_step_inputs(1_860, 3_000, true, true), inputs);
    let snapshot = runtime.snapshot();

    assert_eq!(snapshot.knock_intensity_x100, 400);
    assert_eq!(snapshot.knock_retard_deg10, 0);
    assert_eq!(snapshot.legacy_cut_reason_code, 0);
}
#[test]
fn runtime_step_preserves_explicit_high_resolution_torque_request_observation() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step(
        running_step_inputs(1_900, 3_000, true, true),
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(1_900),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(1_900),
                clt_c: 80,
                just_started: false,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(53, 0, 100, 100, 100).with_driver_request_x1000(537),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(100),
                0,
                0,
                0,
                false,
                Rpm::new(3_000),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    assert_eq!(result.control.torque.requested_x100, 53);
    assert_eq!(result.control.torque.requested_x1000, 537);
    assert_eq!(result.torque_observations.request_x1000, 537);
    assert_eq!(result.control.torque.allowed_x1000, 537);
    assert_eq!(result.torque_observations.allowed_x1000, 537);
}
#[test]
fn authority_step_launch_arming_triggers_semantic_cut() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_launch_and_flat_shift_calibration(),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(1_000),
            3_000,
            700,
            2_000,
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            ),
            true,
            false,
            false,
        ),
        running_control_inputs(1_000, 3_000),
    );

    assert!(result.control.fuel_intent.fuel_cut);
    assert!(result.control.fuel_intent.spark_cut);
    assert!(runtime.snapshot().launch_active);
    assert!(!runtime.snapshot().flat_shift_active);
}
#[test]
fn authority_step_flat_shift_arming_triggers_semantic_cut() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_launch_and_flat_shift_calibration(),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(2_000),
            3_000,
            700,
            2_000,
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            ),
            false,
            true,
            false,
        ),
        running_control_inputs(2_000, 3_000),
    );

    assert!(result.control.fuel_intent.fuel_cut);
    assert!(result.control.fuel_intent.spark_cut);
    assert!(runtime.snapshot().flat_shift_active);
    assert!(!runtime.snapshot().launch_active);
}
#[test]
fn authority_step_without_shift_arming_leaves_semantic_cuts_inactive() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_speed_density_ve(
        semantic_launch_and_flat_shift_calibration(),
        RuntimeSemanticState::default(),
    );

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(3_000),
            3_000,
            700,
            2_000,
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            ),
            false,
            false,
            false,
        ),
        running_control_inputs(3_000, 3_000),
    );

    assert!(!result.control.fuel_intent.fuel_cut);
    assert!(!result.control.fuel_intent.spark_cut);
    assert!(!runtime.snapshot().launch_active);
    assert!(!runtime.snapshot().flat_shift_active);
}
#[test]
fn runtime_differential_mapping_matches_oracle_fuel_and_state() {
    let input = canonical_runtime_input();
    let oracle = spec_step(
        &default_reference_calibration(),
        input,
        &LogicalState::default(),
    );
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model({
        let rpm_bins = [
            Rpm::new(500),
            Rpm::new(1000),
            Rpm::new(1500),
            Rpm::new(2000),
            Rpm::new(2500),
            Rpm::new(3000),
            Rpm::new(3500),
            Rpm::new(4000),
            Rpm::new(4500),
            Rpm::new(5000),
            Rpm::new(5500),
            Rpm::new(6000),
            Rpm::new(6500),
            Rpm::new(7000),
            Rpm::new(7500),
            Rpm::new(8000),
        ];
        let load_bins = [
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1000),
            Kpa10::new(1100),
            Kpa10::new(1200),
            Kpa10::new(1300),
            Kpa10::new(1400),
            Kpa10::new(1500),
            Kpa10::new(1600),
            Kpa10::new(1700),
        ];
        let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(2500); 16]; 16];
        pulse_widths[0][0] = ecu_domain::PulseWidthUs::new(2500);
        BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
    });

    let result = runtime.step(
        StepInputs {
            now_us: ecu_domain::Micros::new(10_000),
            rpm: input.rpm.get() as u32,
            load_kpa10: input.load_kpa10.get() as u32,
            angle_x10: 2_000,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: ecu_domain::Micros::new(10_000),
                clt_c: 4,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: ecu_domain::Micros::new(10_000),
                clt_c: 80,
                just_started: false,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(100, 100, 100, 100, 100),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(150),
                0,
                0,
                0,
                false,
                ecu_domain::Rpm::new(1000),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    assert_eq!(runtime.engine.rpm.get(), input.rpm.get());
    assert_eq!(runtime.engine.load_kpa10.get(), input.load_kpa10.get());
    assert_eq!(
        runtime.engine.sync,
        ecu_domain::SyncState::Locked { cam_ref: false }
    );
    assert_eq!(runtime.engine.mode, ecu_domain::ControlMode::ClosedLoop);
    assert_eq!(result.control.ignition.advance_deg10.get(), 150);
    assert_within(
        "pw_corr_us",
        result.control.enriched_fuel.get() as u32,
        oracle.output.pw_corr_us.get(),
        1,
        &input,
        "canonical_reference_calibration",
    );
    assert_eq!(oracle.output.diagnostic, ecu_spec::DiagnosticCode::None);
    assert_eq!(
        oracle.next_state.diag.current,
        ecu_spec::DiagnosticCode::None
    );
}
#[test]
fn engine_runtime_layout_defaults_cleanly() {
    let runtime = EngineRuntime::new();

    assert_eq!(runtime.engine.sync, SyncState::Unsynced);
    assert_eq!(runtime.engine_time_authority(), EngineTimeAuthority::none());
    assert_eq!(runtime.engine.phase, EnginePhase::Off);
    assert_eq!(runtime.engine.angle_x10.get(), 0);
    assert_eq!(runtime.control.lambda_target.get(), 100);
    assert_eq!(runtime.faults.severity, FaultSeverity::Info);
    assert!(!runtime.calibration.staged_dirty);
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
    assert_eq!(runtime.runtime_snapshot.engine.phase, EnginePhase::Off);
    assert_eq!(runtime.runtime_snapshot.calibration, runtime.calibration);
    assert_eq!(runtime.runtime_snapshot.scheduler, runtime.scheduler);
    assert_eq!(
        runtime.runtime_snapshot.output_profile,
        runtime.output_profile()
    );
    assert_eq!(
        runtime.runtime_snapshot.fuel_strategy_mode,
        RuntimeFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(runtime.calibration_snapshot, CalibrationSnapshot::default());
    assert_eq!(
        runtime.output_profile(),
        ecu_board_api::legacy::single_channel_runtime_output_profile()
    );
}
#[test]
fn decoder_observations_do_not_let_cam_seen_certify_sync_by_itself() {
    let mut runtime = EngineRuntime::new();

    runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        angle_x10: Degrees10::new(45),
        synced: false,
    }));

    assert_eq!(runtime.engine.sync, SyncState::Unsynced);
    assert_eq!(runtime.engine.phase, EnginePhase::Cranking);
    assert_eq!(runtime.engine.rpm.get(), 1200);
    assert_eq!(runtime.engine.angle_x10.get(), 45);
    assert_eq!(
        runtime.engine_time_authority().crank,
        CrankSyncState::PrimarySearching
    );

    runtime.apply_decoder_observation(DecoderObservation::Cam(CamObservation {
        at_us: Micros::new(20),
        cam_seen: true,
    }));

    assert_eq!(runtime.engine.sync, SyncState::Unsynced);
    assert_eq!(runtime.engine.phase, EnginePhase::Cranking);
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));

    runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
        at_us: Micros::new(30),
        rpm: Rpm::new(1200),
        angle_x10: Degrees10::new(90),
        synced: true,
    }));

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(runtime.engine.phase, EnginePhase::Running);
    assert_eq!(
        runtime.runtime_snapshot.engine.sync,
        SyncState::Locked { cam_ref: false }
    );
    assert_eq!(
        runtime.engine_time_authority().phase,
        PhaseSyncState::CrankOnly360
    );
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
}
#[test]
fn set_engine_time_authority_sanitizes_invalid_snapshot() {
    let mut runtime = EngineRuntime::new();
    runtime.set_engine_time_authority(authority(
        CrankSyncState::NoSignal,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::ExpertManual,
    ));

    assert_eq!(runtime.engine_time_authority(), EngineTimeAuthority::none());
    assert_eq!(runtime.engine.sync, SyncState::Unsynced);
    assert_eq!(runtime.engine.phase, EnginePhase::Off);
}
#[test]
fn try_set_engine_time_authority_rejects_invalid_snapshot_without_mutating_state() {
    let mut runtime = EngineRuntime::new();
    let original = validated_expert_authority();
    runtime
        .try_set_engine_time_authority(original)
        .expect("valid authority");
    let invalid = authority(
        CrankSyncState::NoSignal,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::ExpertManual,
    );

    let err = runtime
        .try_set_engine_time_authority(invalid)
        .expect_err("invalid authority must be surfaced");

    assert_eq!(err.authority, invalid);
    assert_eq!(
        err.reason,
        ecu_domain::EngineTimeAuthorityError::AbsoluteTimingWithoutPrimaryLock
    );
    assert_eq!(runtime.engine_time_authority(), original);
}
#[test]
fn step_with_authority_keeps_structured_baseline_when_inputs_match() {
    let mut runtime = EngineRuntime::new();

    let authority = validated_expert_authority();
    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(10),
            3000,
            500,
            100,
            authority,
            false,
            false,
            false,
        ),
        running_control_inputs(10, 3000),
    );

    assert_eq!(runtime.engine_time_authority(), authority);
    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(
        runtime.runtime_snapshot.engine.engine_time_authority,
        authority
    );
    assert_eq!(result.validated.rpm, Rpm::new(3000));
}
#[test]
fn runtime_step_validates_inputs_and_orders_derivation() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.faults.severity = FaultSeverity::Warning;
    runtime.calibration.staged_dirty = true;

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
            rpm: 3_000,
            load_kpa10: 700,
            angle_x10: 8_000,
            trigger_synced: false,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(0),
                clt_c: 20,
                cranking: true,
                just_started: true,
                tpsdot_pct_s: 200,
                mapdot_kpa_s: 90,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(0),
                clt_c: 80,
                just_started: true,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(96),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(92, 80, 120, 118, 110),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(110),
                8,
                2,
                4,
                false,
                Rpm::new(2800),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    assert_eq!(result.validated.rpm.get(), 3000);
    assert_eq!(result.validated.load_kpa10.get(), 700);
    assert_eq!(result.validated.angle_x10.get(), 7200);
    assert!(result.validated.clamped);
    assert_eq!(runtime.engine.sync, SyncState::Unsynced);
    assert_eq!(runtime.engine.phase, EnginePhase::Cranking);
    assert_eq!(result.operating_mode, ControlMode::LimpHome);
    assert_eq!(runtime.engine.mode, ControlMode::LimpHome);
    assert_eq!(runtime.runtime_snapshot.engine.mode, ControlMode::LimpHome);
    assert_eq!(result.control.base_fuel.get(), 2500);
    assert!(result.control.enriched_fuel.get() >= result.control.base_fuel.get());
    assert_eq!(
        runtime.control.fuel_pulse_width,
        result.control.enriched_fuel
    );
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
    let mut actions = result.actions.iter();
    assert_eq!(actions.next(), Some(Action::Idle));
    match actions.next() {
        Some(Action::ApplyAux(batch)) => {
            assert_eq!(
                batch.as_slice(),
                &[AuxCommand::new(
                    AuxOutput::SafetyRelay(1),
                    AuxValue::Level(OutputLevel::High)
                )]
            );
        }
        other => panic!("expected aux batch, got {other:?}"),
    }
    assert_eq!(actions.next(), Some(Action::PersistCalibration));
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
}
#[test]
fn runtime_unsynced_path_emits_idle_and_snapshot() {
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

    let mut actions = result.actions.iter();
    assert_eq!(actions.next(), Some(Action::Idle));
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
}
#[test]
fn runtime_shutdown_path_emits_cancel_and_snapshot() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(4_000),
            rpm: 3_000,
            load_kpa10: 700,
            angle_x10: 2_000,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(4_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(4_000),
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
                Rpm::new(3000),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SafetyShutdown))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(result.operating_mode, ControlMode::Shutdown);
    assert!(result.control.fuel_cut);
    assert!(result.control.spark_cut);
    assert_eq!(
        result.control.fuel_intent.pulse_width_us,
        PulseWidthUs::new(0)
    );
    assert!(runtime.snapshot().fuel_cut);
    assert!(runtime.snapshot().spark_cut);

    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(result.actions, &mut outputs, &mut aux)
        .expect("shutdown direct pw actions should lower");
    assert_eq!(status.scheduled_output_transitions, 0);
    assert!(status.cancel_scheduled_outputs());
    assert!(outputs.is_empty());
}
#[test]
fn runtime_snapshot_matches_state_after_step() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(2_000),
            rpm: 3_000,
            load_kpa10: 700,
            angle_x10: 2_000,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(2_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(2_000),
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
                Rpm::new(3000),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    assert_eq!(runtime.runtime_snapshot.engine, runtime.engine);
    assert_eq!(runtime.runtime_snapshot.control, runtime.control);
    assert_eq!(runtime.runtime_snapshot.faults, runtime.faults);
    assert_eq!(result.control.base_fuel.get(), 2500);
}
