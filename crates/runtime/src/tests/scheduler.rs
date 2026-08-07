use super::*;

#[test]
fn lower_action_arm_scheduler_preserves_transition_order_levels_ticks_and_channels() {
    let injection = test_timed_injection(2, 100, 140);
    let ignition = test_timed_ignition(5, 70, 95);
    let mut outputs = OutputTransitionBatch::<4>::new();
    let mut aux = AuxCommandBatch::<0>::new();

    let status = lower_action_to_board_batches(
        Action::ArmScheduler {
            injection,
            ignition,
        },
        &mut outputs,
        &mut aux,
    )
    .expect("lower arm scheduler");

    assert_eq!(status.scheduled_output_transitions, 4);
    assert_eq!(status.applied_aux_commands, 0);
    assert_eq!(
        outputs.as_slice(),
        &[
            OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(2)),
                OutputLevel::High,
                Ticks::new(100)
            ),
            OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(2)),
                OutputLevel::Low,
                Ticks::new(140)
            ),
            OutputTransition::new(
                EcuOutput::Ignition(ChannelId::new(5)),
                OutputLevel::High,
                Ticks::new(70)
            ),
            OutputTransition::new(
                EcuOutput::Ignition(ChannelId::new(5)),
                OutputLevel::Low,
                Ticks::new(95)
            ),
        ]
    );
    assert!(aux.is_empty());
}
#[test]
fn lower_action_arm_injection_emits_only_injector_pair() {
    let mut outputs = OutputTransitionBatch::<2>::new();
    let mut aux = AuxCommandBatch::<0>::new();

    let status = lower_action_to_board_batches(
        Action::ArmInjection(test_timed_injection(4, 250, 310)),
        &mut outputs,
        &mut aux,
    )
    .expect("lower injection");

    assert_eq!(status.scheduled_output_transitions, 2);
    assert_eq!(
        outputs.as_slice(),
        &[
            OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(4)),
                OutputLevel::High,
                Ticks::new(250)
            ),
            OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(4)),
                OutputLevel::Low,
                Ticks::new(310)
            ),
        ]
    );
    assert!(aux.is_empty());
}
#[test]
fn lower_action_arm_ignition_emits_only_ignition_pair() {
    let mut outputs = OutputTransitionBatch::<2>::new();
    let mut aux = AuxCommandBatch::<0>::new();

    let status = lower_action_to_board_batches(
        Action::ArmIgnition(test_timed_ignition(1, 400, 460)),
        &mut outputs,
        &mut aux,
    )
    .expect("lower ignition");

    assert_eq!(status.scheduled_output_transitions, 2);
    assert_eq!(
        outputs.as_slice(),
        &[
            OutputTransition::new(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::High,
                Ticks::new(400)
            ),
            OutputTransition::new(
                EcuOutput::Ignition(ChannelId::new(1)),
                OutputLevel::Low,
                Ticks::new(460)
            ),
        ]
    );
    assert!(aux.is_empty());
}
#[test]
fn lower_action_apply_aux_copies_commands_in_order() {
    let mut commands = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    commands
        .push(AuxCommand::new(
            AuxOutput::SafetyRelay(0),
            AuxValue::Level(OutputLevel::High),
        ))
        .expect("push fuel pump");
    commands
        .push(AuxCommand::new(AuxOutput::Indicator(0), AuxValue::Off))
        .expect("push cel");
    commands
        .push(AuxCommand::new(
            AuxOutput::Pwm(ChannelId::new(7)),
            AuxValue::Level(OutputLevel::Low),
        ))
        .expect("push pwm");
    let mut outputs = OutputTransitionBatch::<0>::new();
    let mut aux = AuxCommandBatch::<3>::new();

    let status = lower_action_to_board_batches(Action::ApplyAux(commands), &mut outputs, &mut aux)
        .expect("lower aux");

    assert_eq!(status.applied_aux_commands, 3);
    assert!(outputs.is_empty());
    assert_eq!(aux.as_slice(), commands.as_slice());
}
#[test]
fn lower_action_apply_aux_maps_fan_levels_without_legacy_action() {
    let mut outputs = OutputTransitionBatch::<0>::new();
    let mut aux = AuxCommandBatch::<2>::new();
    let mut commands = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    commands
        .push(AuxCommand::new(
            AuxOutput::SafetyRelay(1),
            AuxValue::Level(OutputLevel::High),
        ))
        .expect("push fan high");
    commands
        .push(AuxCommand::new(
            AuxOutput::SafetyRelay(1),
            AuxValue::Level(OutputLevel::Low),
        ))
        .expect("push fan low");

    let status = lower_action_to_board_batches(Action::ApplyAux(commands), &mut outputs, &mut aux)
        .expect("lower fan aux");

    assert_eq!(status.applied_aux_commands, 2);
    assert!(outputs.is_empty());
    assert_eq!(
        aux.as_slice(),
        &[
            AuxCommand::new(
                AuxOutput::SafetyRelay(1),
                AuxValue::Level(OutputLevel::High)
            ),
            AuxCommand::new(AuxOutput::SafetyRelay(1), AuxValue::Level(OutputLevel::Low)),
        ]
    );
}
#[test]
fn lower_action_batch_reports_non_output_status_flags() {
    let mut actions = ActionBatch::<4>::new();
    assert!(actions.push(Action::CancelScheduler(CancelReason::SyncLoss)));
    assert!(actions.push(Action::PersistCalibration));
    assert!(actions.push(Action::PublishSnapshot));
    assert!(actions.push(Action::Idle));
    let mut outputs = OutputTransitionBatch::<0>::new();
    let mut aux = AuxCommandBatch::<0>::new();

    let status = lower_action_batch_to_board_batches(actions, &mut outputs, &mut aux)
        .expect("lower status-only batch");

    assert_eq!(status.scheduled_output_transitions, 0);
    assert_eq!(status.applied_aux_commands, 0);
    assert_eq!(status.cancel_scheduler, Some(CancelReason::SyncLoss));
    assert!(status.cancel_scheduled_outputs());
    assert!(status.persist_calibration);
    assert!(status.publish_snapshot);
    assert!(status.idle_seen);
    assert!(outputs.is_empty());
    assert!(aux.is_empty());
}
#[test]
fn lower_action_output_overflow_is_atomic_for_one_action() {
    let existing = OutputTransition::new(
        EcuOutput::Ignition(ChannelId::new(9)),
        OutputLevel::Low,
        Ticks::new(1),
    );
    let mut outputs = OutputTransitionBatch::<4>::new();
    outputs.push(existing).expect("push existing transition");
    let mut aux = AuxCommandBatch::<0>::new();

    let err = lower_action_to_board_batches(
        Action::ArmScheduler {
            injection: test_timed_injection(2, 100, 140),
            ignition: test_timed_ignition(5, 70, 95),
        },
        &mut outputs,
        &mut aux,
    )
    .expect_err("arm scheduler should not fit");

    assert_eq!(err, ActionLoweringError::OutputBatchFull);
    assert_eq!(outputs.as_slice(), &[existing]);
    assert!(aux.is_empty());
}
#[test]
fn lower_action_aux_overflow_is_atomic_for_one_action() {
    let existing = AuxCommand::new(AuxOutput::SafetyRelay(0), AuxValue::Off);
    let mut commands = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    commands
        .push(AuxCommand::new(
            AuxOutput::SafetyRelay(1),
            AuxValue::Level(OutputLevel::High),
        ))
        .expect("push fan");
    commands
        .push(AuxCommand::new(AuxOutput::Indicator(0), AuxValue::Off))
        .expect("push cel");
    let mut outputs = OutputTransitionBatch::<0>::new();
    let mut aux = AuxCommandBatch::<2>::new();
    aux.push(existing).expect("push existing aux");

    let err = lower_action_to_board_batches(Action::ApplyAux(commands), &mut outputs, &mut aux)
        .expect_err("aux commands should not fit");

    assert_eq!(err, ActionLoweringError::AuxBatchFull);
    assert!(outputs.is_empty());
    assert_eq!(aux.as_slice(), &[existing]);
}
#[test]
fn lower_action_batch_output_overflow_leaves_batches_unchanged() {
    let existing_output = OutputTransition::new(
        EcuOutput::Ignition(ChannelId::new(9)),
        OutputLevel::Low,
        Ticks::new(1),
    );
    let existing_aux = AuxCommand::new(AuxOutput::SafetyRelay(0), AuxValue::Off);
    let mut actions = ActionBatch::<2>::new();
    assert!(actions.push(Action::ArmInjection(test_timed_injection(1, 10, 20))));
    assert!(actions.push(Action::ArmIgnition(test_timed_ignition(2, 30, 40))));
    let mut outputs = OutputTransitionBatch::<4>::new();
    outputs
        .push(existing_output)
        .expect("push existing transition");
    let mut aux = AuxCommandBatch::<1>::new();
    aux.push(existing_aux).expect("push existing aux");

    let err = lower_action_batch_to_board_batches(actions, &mut outputs, &mut aux)
        .expect_err("batch output transitions should not fit");

    assert_eq!(err, ActionLoweringError::OutputBatchFull);
    assert_eq!(outputs.as_slice(), &[existing_output]);
    assert_eq!(aux.as_slice(), &[existing_aux]);
}
#[test]
fn lower_action_batch_aux_overflow_leaves_batches_unchanged() {
    let existing_output = OutputTransition::new(
        EcuOutput::Ignition(ChannelId::new(9)),
        OutputLevel::Low,
        Ticks::new(1),
    );
    let existing_aux = AuxCommand::new(AuxOutput::SafetyRelay(0), AuxValue::Off);
    let mut commands = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    commands
        .push(AuxCommand::new(AuxOutput::Indicator(0), AuxValue::Off))
        .expect("push cel");
    let mut actions = ActionBatch::<3>::new();
    assert!(actions.push(Action::ArmInjection(test_timed_injection(1, 10, 20))));
    assert!(actions.push(Action::ApplyAux(commands)));
    let mut commands = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    commands
        .push(AuxCommand::new(
            AuxOutput::SafetyRelay(1),
            AuxValue::Level(OutputLevel::High),
        ))
        .expect("push fan");
    assert!(actions.push(Action::ApplyAux(commands)));
    let mut outputs = OutputTransitionBatch::<3>::new();
    outputs
        .push(existing_output)
        .expect("push existing transition");
    let mut aux = AuxCommandBatch::<2>::new();
    aux.push(existing_aux).expect("push existing aux");

    let err = lower_action_batch_to_board_batches(actions, &mut outputs, &mut aux)
        .expect_err("batch aux commands should not fit");

    assert_eq!(err, ActionLoweringError::AuxBatchFull);
    assert_eq!(outputs.as_slice(), &[existing_output]);
    assert_eq!(aux.as_slice(), &[existing_aux]);
}
#[test]
fn board_api_batch_executor_rejects_batch_overflow_without_partial_writes() {
    let mut executor = BoardApiBatchExecutor::<3, RUNTIME_AUX_COMMAND_CAP>::new();
    executor
        .execute(Action::ArmInjection(test_timed_injection(1, 10, 20)))
        .expect("seed output transitions lower");
    let before = executor;

    let mut batch = ActionBatch::<1>::new();
    assert!(batch.push(Action::ArmIgnition(test_timed_ignition(2, 100, 130))));

    assert_eq!(
        executor.execute_batch(batch),
        Err(ActionLoweringError::OutputBatchFull)
    );
    assert_eq!(
        executor, before,
        "overflow must not append a partial board-api batch"
    );
}
#[test]
fn lower_action_batch_to_timing_island_maps_outputs_cancel_aux_and_status() {
    let mut aux_commands = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    aux_commands
        .push(AuxCommand::new(
            AuxOutput::SafetyRelay(0),
            AuxValue::Level(OutputLevel::High),
        ))
        .expect("push fuel pump");
    aux_commands
        .push(AuxCommand::new(AuxOutput::Indicator(0), AuxValue::Off))
        .expect("push cel");
    let mut actions = ActionBatch::<6>::new();
    assert!(actions.push(Action::ArmInjection(test_timed_injection(4, 250, 310))));
    assert!(actions.push(Action::CancelScheduler(CancelReason::SyncLoss)));
    assert!(actions.push(Action::ApplyAux(aux_commands)));
    assert!(actions.push(Action::PersistCalibration));
    assert!(actions.push(Action::PublishSnapshot));
    assert!(actions.push(Action::Idle));
    let mut commands = TimingIslandCommandBatch::<5>::new();

    let status = lower_action_batch_to_timing_island(actions, &mut commands)
        .expect("lower timing island batch");

    assert_eq!(status.scheduled_output_transitions, 2);
    assert_eq!(status.applied_aux_commands, 2);
    assert_eq!(status.cancel_scheduler, Some(CancelReason::SyncLoss));
    assert!(status.persist_calibration);
    assert!(status.publish_snapshot);
    assert!(status.idle_seen);
    assert_eq!(
        commands.as_slice(),
        &[
            TimingIslandCommand::ArmOutput(OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(4)),
                OutputLevel::High,
                Ticks::new(250)
            )),
            TimingIslandCommand::ArmOutput(OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(4)),
                OutputLevel::Low,
                Ticks::new(310)
            )),
            TimingIslandCommand::CancelAll(CancelReason::SyncLoss),
            TimingIslandCommand::ApplyAux(AuxCommand::new(
                AuxOutput::SafetyRelay(0),
                AuxValue::Level(OutputLevel::High)
            )),
            TimingIslandCommand::ApplyAux(AuxCommand::new(AuxOutput::Indicator(0), AuxValue::Off)),
        ]
    );
}
#[test]
fn lower_action_to_timing_island_persist_is_status_only() {
    let mut commands = TimingIslandCommandBatch::<0>::new();

    let status = lower_action_to_timing_island(Action::PersistCalibration, &mut commands)
        .expect("lower persist");

    assert!(status.persist_calibration);
    assert_eq!(status.scheduled_output_transitions, 0);
    assert_eq!(status.applied_aux_commands, 0);
    assert!(commands.is_empty());
}
#[test]
fn lower_action_batch_to_timing_island_overflow_leaves_commands_unchanged() {
    let existing = TimingIslandCommand::CancelAll(CancelReason::Manual);
    let mut actions = ActionBatch::<2>::new();
    assert!(actions.push(Action::ArmInjection(test_timed_injection(1, 10, 20))));
    assert!(actions.push(Action::ArmIgnition(test_timed_ignition(2, 30, 40))));
    let mut commands = TimingIslandCommandBatch::<4>::new();
    commands.push(existing).expect("push existing command");

    let err = lower_action_batch_to_timing_island(actions, &mut commands)
        .expect_err("batch timing-island commands should not fit");

    assert_eq!(err, ActionLoweringError::TimingIslandBatchFull);
    assert_eq!(commands.as_slice(), &[existing]);
}
#[test]
fn engine_runtime_pending_output_count_reflects_scheduler_state() {
    let mut runtime = EngineRuntime::new();

    assert_eq!(runtime.pending_output_count(), 0);

    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_batch_injection(4);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    runtime.step(
        running_step_inputs(5_000, 3_000, true, false),
        running_control_inputs(5_000, 3_000),
    );

    assert!(runtime.pending_output_count() > 0);
}
#[test]
fn schedule_depends_on_fuel_intent_not_ve_observation_fields() {
    let cal = semantic_schedule_calibration(2500);
    let input = semantic_schedule_input(3000, SyncState::Locked { cam_ref: false });

    let mut fuel_a = semantic_fuel_observations(1800, false, false);
    fuel_a.ve_pct_x100 = 5000;
    fuel_a.target_afr_x100 = 1470;

    let mut fuel_b = semantic_fuel_observations(1800, false, false);
    fuel_b.ve_pct_x100 = 12000;
    fuel_b.target_afr_x100 = 1250;

    let out_a = runtime_semantic_evaluate_schedule(&cal, input, fuel_a).expect("schedule A");
    let out_b = runtime_semantic_evaluate_schedule(&cal, input, fuel_b).expect("schedule B");

    assert_eq!(
        out_a.injection_duration_deg10,
        out_b.injection_duration_deg10
    );
    assert_eq!(out_a.injection_target_deg10, out_b.injection_target_deg10);
    assert_eq!(out_a.spark_advance_deg10, out_b.spark_advance_deg10);
    assert_eq!(out_a.events.len, out_b.events.len);
}
#[test]
fn fast_events_coalesce_by_kind() {
    let mut queues: RuntimeQueues<2, 2> = RuntimeQueues::new();

    assert_eq!(
        queues.push(Event::Fast(FastEvent::SensorSample {
            rpm: Rpm::new(1000),
            load_kpa10: Kpa10::new(300),
        })),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Fast(FastEvent::SensorSample {
            rpm: Rpm::new(1500),
            load_kpa10: Kpa10::new(450),
        })),
        Ok(QueueResult::Coalesced)
    );

    match queues.pop_fast() {
        Some(Event::Fast(FastEvent::SensorSample { rpm, load_kpa10 })) => {
            assert_eq!(rpm.get(), 1500);
            assert_eq!(load_kpa10.get(), 450);
        }
        other => panic!("unexpected event: {:?}", other),
    }
}
#[test]
fn slow_events_fifo_and_overflow() {
    let mut queues: RuntimeQueues<1, 1> = RuntimeQueues::new();

    assert_eq!(
        queues.push(Event::Slow(SlowEvent::SnapshotRequested)),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Slow(SlowEvent::PersistRequested)),
        Err(QueueOverflow::SlowFull)
    );
    assert_eq!(
        queues.pop_slow(),
        Some(Event::Slow(SlowEvent::SnapshotRequested))
    );
}
#[test]
fn runtime_queue_split_prioritizes_fast_and_reports_overflow() {
    let mut queues: RuntimeQueues<2, 1> = RuntimeQueues::new();

    assert_eq!(
        queues.push(Event::Slow(SlowEvent::SnapshotRequested)),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Fast(FastEvent::TriggerEdge {
            at_us: Micros::new(1),
        })),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Fast(FastEvent::TriggerEdge {
            at_us: Micros::new(2),
        })),
        Ok(QueueResult::Coalesced)
    );
    assert_eq!(
        queues.push(Event::Slow(SlowEvent::PersistRequested)),
        Err(QueueOverflow::SlowFull)
    );
    assert!(matches!(
        queues.pop_fast(),
        Some(Event::Fast(FastEvent::TriggerEdge { .. }))
    ));
    assert!(matches!(
        queues.pop_slow(),
        Some(Event::Slow(SlowEvent::SnapshotRequested))
    ));
}
#[test]
fn fast_and_slow_lanes_are_independent() {
    let mut queues: RuntimeQueues<1, 1> = RuntimeQueues::new();

    assert_eq!(
        queues.push(Event::Fast(FastEvent::TriggerEdge {
            at_us: Micros::new(10),
        })),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Slow(SlowEvent::CalibrationCommitted)),
        Ok(QueueResult::Enqueued)
    );

    assert!(matches!(
        queues.pop_fast(),
        Some(Event::Fast(FastEvent::TriggerEdge { .. }))
    ));
    assert!(matches!(
        queues.pop_slow(),
        Some(Event::Slow(SlowEvent::CalibrationCommitted))
    ));
}
#[test]
fn queue_pressure_and_degraded_authority_keep_runtime_output_suppressed() {
    let mut queues: RuntimeQueues<1, 1> = RuntimeQueues::new();
    assert_eq!(
        queues.push(Event::Fast(FastEvent::TriggerEdge {
            at_us: Micros::new(10),
        })),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Fast(FastEvent::SensorSample {
            rpm: Rpm::new(2500),
            load_kpa10: Kpa10::new(700),
        })),
        Err(QueueOverflow::FastFull)
    );
    assert_eq!(
        queues.push(Event::Slow(SlowEvent::SnapshotRequested)),
        Ok(QueueResult::Enqueued)
    );
    assert_eq!(
        queues.push(Event::Slow(SlowEvent::PersistRequested)),
        Err(QueueOverflow::SlowFull)
    );

    let mut runtime = EngineRuntime::new();
    runtime.configure_full_ecu(inline_sequential_cop_profile());

    let result = runtime.step_with_authority(
        AuthorityStepInputs::new(
            Micros::new(10),
            2500,
            700,
            120,
            authority(
                CrankSyncState::PrimarySearching,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::None,
            ),
            false,
            false,
            false,
        ),
        running_control_inputs(10, 2500),
    );

    assert_eq!(result.operating_mode, ControlMode::OpenLoop);
    assert_eq!(
        runtime.engine_time_authority().crank,
        CrankSyncState::PrimarySearching
    );
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
    assert_eq!(arm_injection_count(result.actions), 0);
    assert_eq!(arm_ignition_count(result.actions), 0);
    assert_eq!(
        runtime.snapshot().engine.engine_time_authority,
        runtime.engine_time_authority()
    );
    assert!(!runtime.snapshot().fuel_cut);
    assert!(!runtime.snapshot().spark_cut);
}
#[test]
fn full_ecu_live_path_enforces_fuel_and_spark_cuts_independently() {
    for (fuel_cut, spark_cut, expected_injection, expected_ignition) in [
        (false, false, 6, 6),
        (true, false, 0, 6),
        (false, true, 6, 0),
        (true, true, 0, 0),
    ] {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
        runtime.configure_full_ecu(inline_sequential_cop_profile());
        runtime.set_direct_cut_requests(fuel_cut, spark_cut);

        let result = runtime.step_with_authority(
            AuthorityStepInputs::new(
                Micros::new(10),
                3000,
                700,
                120,
                validated_expert_authority(),
                false,
                false,
                false,
            ),
            running_control_inputs(10, 3000),
        );

        assert_eq!(result.control.fuel_cut, fuel_cut);
        assert_eq!(result.control.spark_cut, spark_cut);
        assert_eq!(arm_injection_count(result.actions), expected_injection);
        assert_eq!(arm_ignition_count(result.actions), expected_ignition);
        if fuel_cut && spark_cut {
            assert_eq!(result.actions.iter().count(), 2);
            assert!(matches!(result.actions.iter().next(), Some(Action::Idle)));
            assert!(result
                .actions
                .iter()
                .any(|action| matches!(action, Action::PublishSnapshot)));
        }
    }
}
#[test]
fn runtime_full_sequential_gate_requires_validated_phase_and_absolute_authority() {
    let crank_only = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    );
    let cam_observed_expert = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamObserved720,
        AbsoluteTimeAuthority::ExpertManual,
    );
    let cam_validated_geometry = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
    );
    let cam_validated_unknown = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::None,
    );
    let cam_validated_expert = validated_expert_authority();
    let cam_validated_certified = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::CertifiedProfile,
    );

    assert_eq!(
        crank_only.compatibility_summary(),
        SyncState::Locked { cam_ref: false }
    );
    assert!(!runtime_full_sequential_authorized(crank_only));
    assert!(!runtime_full_sequential_authorized(cam_observed_expert));
    assert!(!runtime_full_sequential_authorized(cam_validated_geometry));
    assert!(!runtime_full_sequential_authorized(cam_validated_unknown));
    assert!(runtime_full_sequential_authorized(cam_validated_expert));
    assert!(runtime_full_sequential_authorized(cam_validated_certified));
    assert_ne!(
        cam_validated_certified.absolute,
        cam_validated_expert.absolute
    );
    assert!(matches!(
        cam_validated_certified.absolute,
        ecu_domain::AbsoluteTimeAuthority::CertifiedProfile
    ));
}
#[test]
fn runtime_full_ecu_cop_blocks_unknown_absolute_even_with_validated_720_phase() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    let profile_absolute_authority = AbsoluteTimeAuthority::None;
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        profile_absolute_authority,
    ));

    let result = runtime.step(
        running_step_inputs(6_000, 3_000, true, true),
        running_control_inputs(6_000, 3_000),
    );

    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
}
#[test]
fn runtime_full_ecu_limp_home_emits_fan_and_profile_aux_commands() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
    runtime.set_engine_time_authority(validated_expert_authority());
    runtime.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(5_000),
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
                now_us: Micros::new(5_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(5_000),
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

    let mut injection_count = 0usize;
    let mut ignition_count = 0usize;
    let mut aux_seen = false;
    let mut publish_seen = false;

    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(_) => injection_count += 1,
            Action::ArmIgnition(_) => ignition_count += 1,
            Action::ApplyAux(batch) => {
                aux_seen = true;
                assert_eq!(
                    batch.as_slice(),
                    &[
                        AuxCommand::new(
                            AuxOutput::SafetyRelay(1),
                            AuxValue::Level(OutputLevel::High)
                        ),
                        AuxCommand::new(AuxOutput::Pwm(ChannelId::new(0)), AuxValue::Off),
                        AuxCommand::new(AuxOutput::Digital(ChannelId::new(0)), AuxValue::Off),
                        AuxCommand::new(AuxOutput::Digital(ChannelId::new(1)), AuxValue::Off),
                    ]
                );
            }
            Action::PublishSnapshot => publish_seen = true,
            other => panic!("unexpected action in full-ecu limp-home path: {other:?}"),
        }
    }

    assert_eq!(injection_count, 6);
    assert_eq!(ignition_count, 6);
    assert!(aux_seen);
    assert!(publish_seen);
}
#[test]
fn runtime_action_capacity_covers_max_full_ecu_aux_persist_snapshot_step() {
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

    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(arm_injection_count(result.actions), 6);
    assert_eq!(arm_ignition_count(result.actions), 6);
    assert_eq!(result.actions.len(), 15);
    assert!(RUNTIME_ACTION_CAP >= result.actions.len());
    assert_eq!(
        result
            .actions
            .iter()
            .filter(|action| matches!(action, Action::ApplyAux(_)))
            .count(),
        1
    );
    assert_eq!(
        result
            .actions
            .iter()
            .filter(|action| matches!(action, Action::PersistCalibration))
            .count(),
        1
    );
    assert_eq!(
        result
            .actions
            .iter()
            .filter(|action| matches!(action, Action::PublishSnapshot))
            .count(),
        1
    );
}
#[test]
fn runtime_full_ecu_board_output_capacity_covers_max_cop_step_exactly() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(validated_expert_authority());

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );

    assert_eq!(arm_injection_count(result.actions), 6);
    assert_eq!(arm_ignition_count(result.actions), 6);

    let mut exact = BoardApiBatchExecutor::<24, RUNTIME_AUX_COMMAND_CAP>::new();
    exact
        .execute_batch(result.actions)
        .expect("max six-cylinder COP output batch should fit exact transition capacity");
    assert_eq!(exact.status().scheduled_output_transitions, 24);
    assert_eq!(exact.output_transitions().len(), 24);
    assert_eq!(exact.aux_commands().len(), 0);
    assert_eq!(
        exact
            .output_transitions()
            .as_slice()
            .iter()
            .filter(|transition| matches!(transition.output, EcuOutput::Injector(_)))
            .count(),
        12
    );
    assert_eq!(
        exact
            .output_transitions()
            .as_slice()
            .iter()
            .filter(|transition| matches!(transition.output, EcuOutput::Ignition(_)))
            .count(),
        12
    );

    let mut short = BoardApiBatchExecutor::<23, RUNTIME_AUX_COMMAND_CAP>::new();
    assert_eq!(
        short.execute_batch(result.actions),
        Err(ActionLoweringError::OutputBatchFull)
    );
    assert!(short.output_transitions().is_empty());
    assert!(short.aux_commands().is_empty());
}
#[test]
fn runtime_full_ecu_wasted_spark_profile_blocks_crank_only_primary_lock() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, false),
        running_control_inputs(5_000, 3_000),
    );

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
}
#[test]
fn runtime_ignition_only_wasted_spark_accepts_crank_only_authority() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_crank_only_wasted_spark(6);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, false),
        spark_only_control_inputs(5_000, 3_000),
    );

    let mut channels = [u8::MAX; 3];
    let mut seen = 0usize;
    let mut publish_seen = false;
    for action in result.actions.iter() {
        match action {
            Action::ArmIgnition(ignition) => {
                channels[seen] = ignition.plan.output.channel().get();
                assert!(ignition.plan.dwell.get() > 0);
                assert!(ignition.end_at.get() > ignition.start_at.get());
                seen += 1;
            }
            Action::PublishSnapshot => publish_seen = true,
            other => panic!("unexpected action in ignition-only path: {other:?}"),
        }
    }

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(seen, 3);
    assert_eq!(&channels[..seen], &[0, 1, 2]);
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert!(publish_seen);
}
#[test]
fn runtime_ignition_only_single_coil_uses_one_channel() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_crank_only_single_coil(4);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(5_000, 2_000, true, false),
        spark_only_control_inputs(5_000, 2_000),
    );

    let mut seen = 0usize;
    for action in result.actions.iter() {
        match action {
            Action::ArmIgnition(ignition) => {
                assert_eq!(ignition.plan.output.channel(), ChannelId::new(0));
                seen += 1;
            }
            Action::PublishSnapshot => {}
            other => panic!("unexpected action in single-coil path: {other:?}"),
        }
    }

    assert_eq!(seen, 2);
    assert_eq!(arm_scheduler_count(result.actions), 0);
}
#[test]
fn runtime_injection_only_batch_emits_only_injector_actions() {
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

    let mut channels = [u8::MAX; 4];
    let mut seen = 0usize;
    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(injection) => {
                channels[seen] = injection.plan.output.channel().get();
                assert!(injection.plan.pulse_width.get() > 0);
                assert!(injection.start_at.get() > 5_000);
                assert!(injection.end_at.get() > injection.start_at.get());
                seen += 1;
            }
            Action::PublishSnapshot => {}
            other => panic!("unexpected action in injection-only path: {other:?}"),
        }
    }

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(seen, 4);
    assert_eq!(&channels[..seen], &[0, 1, 2, 3]);
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(arm_injection_count(result.actions), 4);
    assert_eq!(arm_ignition_count(result.actions), 0);
}
#[test]
fn runtime_injection_only_single_point_uses_one_injector_channel() {
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

    let mut seen = 0usize;
    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(injection) => {
                assert_eq!(injection.plan.output.channel(), ChannelId::new(0));
                seen += 1;
            }
            Action::PublishSnapshot => {}
            other => panic!("unexpected action in single-point injection path: {other:?}"),
        }
    }

    assert_eq!(seen, 1);
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(arm_ignition_count(result.actions), 0);
}
#[test]
fn runtime_injection_only_zero_fuel_idles_without_ignition() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_batch_injection(4);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(5_000, 2_500, true, false),
        running_control_inputs(5_000, 2_500),
    );

    let mut actions = result.actions.iter();
    assert_eq!(actions.next(), Some(Action::Idle));
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(arm_injection_count(result.actions), 0);
    assert_eq!(arm_ignition_count(result.actions), 0);
}
#[test]
fn runtime_injection_only_sync_loss_cancels_pending_outputs() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_batch_injection(4);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let first = runtime.step(
        running_step_inputs(1_000, 3_000, true, false),
        running_control_inputs(1_000, 3_000),
    );

    assert_eq!(arm_injection_count(first.actions), 4);
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Armed
    );

    let result = runtime.step(
        running_step_inputs(2_000, 0, false, false),
        running_control_inputs(2_000, 0),
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SyncLoss))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Suspended
    );
}
#[test]
fn runtime_ignition_only_sync_loss_cancels_pending_outputs() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_crank_only_wasted_spark(6);
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let first = runtime.step(
        running_step_inputs(1_000, 3_000, true, false),
        spark_only_control_inputs(1_000, 3_000),
    );

    assert_eq!(arm_ignition_count(first.actions), 3);
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Armed
    );

    let result = runtime.step(
        running_step_inputs(2_000, 0, false, false),
        spark_only_control_inputs(2_000, 0),
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SyncLoss))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Suspended
    );
}
#[test]
fn runtime_invalid_full_ecu_profiles_do_not_emit_outputs() {
    let zero_topology = FullEcuOutputProfile::new(
        IgnitionOutputProfile::wasted_spark(0, 0),
        InjectionOutputProfile::sequential([], 0, true),
        AuxSafetyProfile::none(),
        OutputAuthorityRequirement::FullSequential720,
    );
    let mismatched_event_counts = FullEcuOutputProfile::new(
        IgnitionOutputProfile::wasted_spark(4, 2),
        InjectionOutputProfile::sequential(INLINE_SIX_FIRING_ORDER, 6, true),
        AuxSafetyProfile::none(),
        OutputAuthorityRequirement::FullSequential720,
    );
    let zero_firing_slot = FullEcuOutputProfile::new(
        IgnitionOutputProfile::wasted_spark(6, 3),
        InjectionOutputProfile::sequential(
            [
                CylinderId::new(1),
                CylinderId::new(5),
                CylinderId::new(0),
                CylinderId::new(6),
                CylinderId::new(2),
                CylinderId::new(4),
            ],
            6,
            true,
        ),
        AuxSafetyProfile::none(),
        OutputAuthorityRequirement::FullSequential720,
    );

    assert_invalid_full_ecu_profile_is_inert(zero_topology);
    assert_invalid_full_ecu_profile_is_inert(mismatched_event_counts);
    assert_invalid_full_ecu_profile_is_inert(zero_firing_slot);
}
#[test]
fn runtime_full_ecu_validated_authority_emits_six_injectors_and_wasted_spark() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    let profile = inline_sequential_wasted_spark_profile();
    assert!(profile.is_valid());
    assert_eq!(profile.event_count(), 6);
    runtime.configure_full_ecu(profile);
    runtime.set_engine_time_authority(validated_expert_authority());
    runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
        at_us: Micros::new(1_000),
        rpm: Rpm::new(3_000),
        angle_x10: Degrees10::new(120),
        synced: true,
    }));

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(5_000),
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
                now_us: Micros::new(5_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(5_000),
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

    let mut injector_channels = [u8::MAX; 6];
    let mut ignition_channels = [u8::MAX; 6];
    let mut injection_start_times = [0u32; 6];
    let mut ignition_fire_times = [0u32; 6];
    let mut injection_seen = 0usize;
    let mut ignition_seen = 0usize;
    let mut publish_seen = false;
    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(injection) => {
                injection_start_times[injection_seen] = injection.start_at.get();
                injector_channels[injection_seen] = injection.plan.output.channel().get();
                injection_seen += 1;
            }
            Action::ArmIgnition(ignition) => {
                ignition_fire_times[ignition_seen] = ignition.end_at.get();
                ignition_channels[ignition_seen] = ignition.plan.output.channel().get();
                ignition_seen += 1;
            }
            Action::PublishSnapshot => publish_seen = true,
            other => panic!("unexpected action in full-ecu wasted-spark profile: {other:?}"),
        }
    }

    assert_eq!(injection_seen, 6);
    assert_eq!(ignition_seen, 6);
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(injector_channels, [0, 1, 2, 3, 4, 5]);
    assert_eq!(&ignition_channels[..ignition_seen], &[0, 1, 2, 0, 1, 2]);
    assert!(injection_start_times[..injection_seen]
        .iter()
        .all(|at_us| *at_us >= 5_000));
    assert!(ignition_fire_times[..ignition_seen]
        .iter()
        .all(|at_us| *at_us >= 5_000));
    assert!(publish_seen);
}
#[test]
fn runtime_full_ecu_injection_timing_changes_with_current_angle() {
    let make_runtime = || {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
        runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
        runtime.set_engine_time_authority(validated_expert_authority());
        runtime
    };

    let mut early_angle_runtime = make_runtime();
    let mut late_angle_runtime = make_runtime();

    let early = early_angle_runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );
    let late = late_angle_runtime.step(
        StepInputs {
            now_us: Micros::new(5_000),
            rpm: 3_000,
            load_kpa10: 700,
            angle_x10: 2_600,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(5_000, 3_000),
    );

    let early_first_injection = early
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmInjection(injection) => Some(injection.start_at.get()),
            _ => None,
        })
        .expect("full ecu should emit at least one injector action");
    let late_first_injection = late
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmInjection(injection) => Some(injection.start_at.get()),
            _ => None,
        })
        .expect("full ecu should emit at least one injector action");

    assert_ne!(early_first_injection, late_first_injection);
}
#[test]
fn runtime_full_ecu_ignition_timing_changes_with_current_angle() {
    let make_runtime = || {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
        runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
        runtime.set_engine_time_authority(validated_expert_authority());
        runtime
    };

    let mut early_angle_runtime = make_runtime();
    let mut late_angle_runtime = make_runtime();

    let early = early_angle_runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );
    let late = late_angle_runtime.step(
        StepInputs {
            now_us: Micros::new(5_000),
            rpm: 3_000,
            load_kpa10: 700,
            angle_x10: 2_600,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(5_000, 3_000),
    );

    let early_first_fire = early
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmIgnition(ignition) => Some(ignition.end_at.get()),
            _ => None,
        })
        .expect("full ecu should emit at least one ignition action");
    let late_first_fire = late
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmIgnition(ignition) => Some(ignition.end_at.get()),
            _ => None,
        })
        .expect("full ecu should emit at least one ignition action");

    assert_ne!(early_first_fire, late_first_fire);
}
#[test]
fn runtime_full_ecu_ignition_timing_changes_with_advance() {
    let make_runtime = || {
        let mut runtime = EngineRuntime::new();
        runtime.configure_fuel_model(test_fuel_model());
        runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
        runtime.set_engine_time_authority(validated_expert_authority());
        runtime
    };

    let mut base_runtime = make_runtime();
    let mut advanced_runtime = make_runtime();

    let baseline = base_runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );
    let advanced = advanced_runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        ControlInputs {
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(200),
                0,
                0,
                0,
                false,
                Rpm::new(3000),
            ),
            ..running_control_inputs(5_000, 3_000)
        },
    );

    let baseline_first_fire = baseline
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmIgnition(ignition) => Some(ignition.end_at.get()),
            _ => None,
        })
        .expect("full ecu should emit at least one ignition action");
    let advanced_first_fire = advanced
        .actions
        .iter()
        .find_map(|action| match action {
            Action::ArmIgnition(ignition) => Some(ignition.end_at.get()),
            _ => None,
        })
        .expect("full ecu should emit at least one ignition action");

    assert_ne!(baseline_first_fire, advanced_first_fire);
}
#[test]
fn runtime_full_ecu_fuel_cut_lowers_only_ignition_transitions() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    runtime.set_direct_cut_requests(true, false);
    runtime.set_engine_time_authority(validated_expert_authority());

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );

    let mut outputs = OutputTransitionBatch::<16>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(result.actions, &mut outputs, &mut aux)
        .expect("full ecu fuel-cut actions should lower");

    assert_eq!(status.scheduled_output_transitions, 12);
    assert!(outputs
        .as_slice()
        .iter()
        .all(|transition| matches!(transition.output, EcuOutput::Ignition(_))));
}
#[test]
fn runtime_full_ecu_spark_cut_lowers_only_injector_transitions() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_full_ecu(inline_sequential_wasted_spark_profile());
    runtime.configure_speed_density_ve(
        semantic_fuel_calibration_with_ve_cells(7000, 9000),
        RuntimeSemanticState::default(),
    );
    runtime.set_direct_cut_requests(false, true);
    runtime.set_engine_time_authority(validated_expert_authority());

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );

    let mut outputs = OutputTransitionBatch::<16>::new();
    let mut aux = AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    let status = lower_action_batch_to_board_batches(result.actions, &mut outputs, &mut aux)
        .expect("full ecu spark-cut actions should lower");

    assert_eq!(status.scheduled_output_transitions, 12);
    assert!(outputs
        .as_slice()
        .iter()
        .all(|transition| matches!(transition.output, EcuOutput::Injector(_))));
}
#[test]
fn runtime_full_ecu_cop_profile_with_validated_authority_emits_ignition_channel_five() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(validated_expert_authority());
    runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
        at_us: Micros::new(1_000),
        rpm: Rpm::new(3_000),
        angle_x10: Degrees10::new(120),
        synced: true,
    }));

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(6_000),
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
                now_us: Micros::new(6_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(6_000),
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

    let mut seen = 0usize;
    let mut last_ignition_channel = None;
    let mut publish_seen = false;
    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(_) => {}
            Action::ArmIgnition(ignition) => {
                last_ignition_channel = Some(ignition.plan.output.channel().get());
                seen += 1;
            }
            Action::PublishSnapshot => publish_seen = true,
            other => panic!("unexpected action in full-ecu cop profile: {other:?}"),
        }
    }

    assert_eq!(seen, 6);
    assert_eq!(last_ignition_channel, Some(5));
    assert!(publish_seen);
}
#[test]
fn runtime_full_ecu_cop_blocks_crank_only_primary_lock() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    ));

    let result = runtime.step(
        running_step_inputs(6_000, 3_000, true, false),
        running_control_inputs(6_000, 3_000),
    );

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
}
#[test]
fn runtime_full_ecu_cop_blocks_cam_observed_but_not_validated() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamObserved720,
        AbsoluteTimeAuthority::ExpertManual,
    ));

    let result = runtime.step(
        running_step_inputs(6_000, 3_000, true, true),
        running_control_inputs(6_000, 3_000),
    );

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(
        runtime.engine_time_authority().phase,
        PhaseSyncState::CamObserved720
    );
    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
}
#[test]
fn sync_loss_cancels_pending_outputs() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());

    let _ = runtime.step(
        StepInputs {
            now_us: Micros::new(1_000),
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
                now_us: Micros::new(1_000),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us: Micros::new(1_000),
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

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(2_000),
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
                Rpm::new(0),
            ),
            fuel_sensors: FuelSensorInputs::default(),
            knock_intensity_x100: 0,
        },
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SyncLoss))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Suspended
    );
}
#[test]
fn runtime_full_ecu_cop_sync_loss_cancels_pending_outputs() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(inline_sequential_cop_profile());
    runtime.set_engine_time_authority(validated_expert_authority());

    let first = runtime.step(
        running_step_inputs(1_000, 3_000, true, true),
        running_control_inputs(1_000, 3_000),
    );

    assert_eq!(arm_scheduler_count(first.actions), 0);
    assert_eq!(arm_injection_count(first.actions), 6);
    assert_eq!(arm_ignition_count(first.actions), 6);
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Armed
    );

    let result = runtime.step(
        StepInputs {
            now_us: Micros::new(2_000),
            rpm: 0,
            load_kpa10: 0,
            angle_x10: 0,
            trigger_synced: false,
            cam_seen: false,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        },
        running_control_inputs(2_000, 0),
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SyncLoss))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
    assert_eq!(
        runtime.scheduler.mode(),
        ecu_scheduler::SchedulerMode::Suspended
    );
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));
}
