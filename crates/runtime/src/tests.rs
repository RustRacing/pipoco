use super::*;
use crate::support::DifferentialInputSnapshot;
use ecu_board_api::{
    AuxCommand, AuxCommandBatch, AuxOutput, AuxValue, EcuOutput, OutputLevel, OutputTransition,
    OutputTransitionBatch, TimingIslandCommand, TimingIslandCommandBatch,
};
#[cfg(test)]
use ecu_calibration::{
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertUnlock,
    SecondaryTriggerMode, TriggerAuthority,
};
use ecu_domain::{ChannelId, CylinderId, Ticks};
use ecu_spec::{
    default_reference_calibration, step as spec_step, InputSnapshot as SpecInputSnapshot,
    LogicalState,
};

const INLINE_SIX_FIRING_ORDER: [CylinderId; 6] = [
    CylinderId::new(1),
    CylinderId::new(5),
    CylinderId::new(3),
    CylinderId::new(6),
    CylinderId::new(2),
    CylinderId::new(4),
];

const INLINE_FULL_ECU_AUX_SAFETY: AuxSafetyProfile = AuxSafetyProfile::off_on_limp([
    AuxOutput::Pwm(ChannelId::new(0)),
    AuxOutput::Digital(ChannelId::new(0)),
    AuxOutput::Digital(ChannelId::new(1)),
]);

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
    assert_eq!(cal.deadtime_table_us.values[0][0], 654);
    assert_eq!(cal.deadtime_table_us.values[15][15], 654);
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

const fn inline_sequential_wasted_spark_profile() -> FullEcuOutputProfile {
    FullEcuOutputProfile::sequential_wasted_spark(
        INLINE_SIX_FIRING_ORDER,
        6,
        3,
        INLINE_FULL_ECU_AUX_SAFETY,
        OutputAuthorityRequirement::FullSequential720,
    )
}

const fn inline_sequential_cop_profile() -> FullEcuOutputProfile {
    FullEcuOutputProfile::sequential_coil_on_plug(
        INLINE_SIX_FIRING_ORDER,
        6,
        6,
        INLINE_FULL_ECU_AUX_SAFETY,
        OutputAuthorityRequirement::FullSequential720,
    )
}

fn test_fuel_model() -> BaseFuelModel {
    test_fuel_model_with_base_pw(2500)
}

fn test_fuel_model_with_base_pw(pw_us: u16) -> BaseFuelModel {
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
    let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(0); 16]; 16];
    pulse_widths[5][5] = ecu_domain::PulseWidthUs::new(pw_us);
    BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
}

fn authority(
    crank: CrankSyncState,
    phase: PhaseSyncState,
    absolute: AbsoluteTimeAuthority,
) -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        crank,
        phase,
        absolute,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    )
}

fn validated_expert_authority() -> EngineTimeAuthority {
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
    let startup_authority = authority(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
    );

    calibration
        .to_runtime_engine_time_authority(startup_authority)
        .expect("validated expert authority")
}

fn test_timed_injection(channel: u8, start_at: u32, end_at: u32) -> TimedInjectionPlan {
    TimedInjectionPlan {
        plan: InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(channel)),
            pulse_width: PulseWidthUs::new(end_at.saturating_sub(start_at) as u16),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}

fn test_timed_ignition(channel: u8, start_at: u32, end_at: u32) -> TimedIgnitionPlan {
    TimedIgnitionPlan {
        plan: ecu_scheduler::IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(channel)),
            dwell: DwellUs::new(end_at.saturating_sub(start_at) as u16),
            advance: Degrees10::new(120),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}

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

fn running_step_inputs(now_us: u32, rpm: u32, trigger_synced: bool, cam_seen: bool) -> StepInputs {
    StepInputs {
        now_us: Micros::new(now_us),
        rpm,
        load_kpa10: 700,
        angle_x10: 2_000,
        trigger_synced,
        cam_seen,
        launch_armed: false,
        flat_shift_armed: false,
    }
}

fn running_control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(now_us),
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
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(
            ecu_domain::Degrees10::new(100),
            0,
            0,
            0,
            false,
            Rpm::new(rpm),
        ),
    }
}

fn spark_only_control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
    ControlInputs::spark_only(
        Micros::new(now_us),
        IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(rpm)),
    )
}

fn arm_scheduler_count<const N: usize>(actions: ActionBatch<N>) -> usize {
    actions
        .iter()
        .filter(|action| matches!(*action, Action::ArmScheduler { .. }))
        .count()
}

fn arm_ignition_count<const N: usize>(actions: ActionBatch<N>) -> usize {
    actions
        .iter()
        .filter(|action| matches!(*action, Action::ArmIgnition(_)))
        .count()
}

fn arm_injection_count<const N: usize>(actions: ActionBatch<N>) -> usize {
    actions
        .iter()
        .filter(|action| matches!(*action, Action::ArmInjection(_)))
        .count()
}

fn canonical_runtime_input() -> SpecInputSnapshot {
    SpecInputSnapshot {
        t_us: ecu_spec::Micros(10_000),
        rpm: ecu_spec::Rpm(1000),
        map_kpa10: ecu_spec::Kpa10(1000),
        load_kpa10: ecu_spec::Kpa10(1000),
        tps_x100: 0,
        clt_c10: ecu_spec::TempC10(4),
        iat_c10: ecu_spec::TempC10(20),
        baro_kpa10: ecu_spec::Kpa10(1000),
        vbatt_mv: ecu_spec::Millivolts(12_000),
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: ecu_spec::SyncState::Synced,
        fuel_cut: false,
        spark_cut: false,
        mode: ecu_spec::EngineMode::Running,
        target_afr_override_x100: ecu_spec::AfrOverride::None,
    }
}

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
    };

    let mapped = snapshot.to_step_inputs();
    assert_eq!(mapped.now_us, Micros::new(10_000));
    assert_eq!(mapped.rpm, 1200);
    assert_eq!(mapped.load_kpa10, 870);
    assert_eq!(mapped.angle_x10, 2000);
    assert!(!mapped.trigger_synced);
    assert!(!mapped.cam_seen);
}

fn assert_within(
    field: &str,
    runtime: u32,
    oracle: u32,
    tolerance: u32,
    input: &SpecInputSnapshot,
    fixture: &str,
) {
    let difference = runtime.abs_diff(oracle);
    assert!(
            difference <= tolerance,
            "input_snapshot={input:?}\ncalibration_fixture={fixture}\nruntime_output={runtime}\noracle_output={oracle}\nfield={field}\ndifference={difference}\ntolerance={tolerance}"
        );
}

fn semantic_axis2() -> RuntimeSemanticAxis16 {
    let mut values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    values[0] = 100;
    values[1] = 200;
    RuntimeSemanticAxis16 { len: 2, values }
}

fn semantic_table_u16(value: u16) -> RuntimeSemanticTable2dU16 {
    RuntimeSemanticTable2dU16 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_table_i16(value: i16) -> RuntimeSemanticTable2dI16 {
    RuntimeSemanticTable2dI16 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_table_u32(value: u32) -> RuntimeSemanticTable2dU32 {
    RuntimeSemanticTable2dU32 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_schedule_calibration(dwell_us: u32) -> RuntimeSemanticScheduleCalibration {
    let mut phases = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    phases[0] = 0;
    RuntimeSemanticScheduleCalibration {
        spark_advance_table_deg10: semantic_table_i16(150),
        dwell_table_us: semantic_table_u32(dwell_us),
        injection_target_table_deg10: semantic_table_u16(360),
        injection_angle_mode: RuntimeSemanticInjectionAngleMode::EndOfInjection,
        cylinder_phase_deg10: RuntimeSemanticCylinderArrayU16 {
            count: 1,
            values: phases,
        },
    }
}

fn semantic_schedule_input(rpm: u16, sync: SyncState) -> RuntimeSemanticInputSnapshot {
    RuntimeSemanticInputSnapshot {
        t_us: Micros::new(0),
        rpm: Rpm::new(rpm),
        map_kpa10: Kpa10::new(100),
        load_kpa10: Kpa10::new(100),
        tps_x100: 0,
        clt_c10: 800,
        iat_c10: 250,
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: 12_000,
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync,
        fuel_cut: false,
        spark_cut: false,
        mode: RuntimeSemanticEngineMode::Running,
        target_afr_override_x100: RuntimeSemanticAfrOverride::None,
    }
}

fn semantic_curve_u16(value: u16) -> RuntimeSemanticCurve16U16 {
    RuntimeSemanticCurve16U16 {
        axis: semantic_axis2(),
        values: [value; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_fuel_calibration_with_ve_cells(
    map_cell: u16,
    tps_cell: u16,
) -> RuntimeSemanticCalibration {
    let mut ve_values = [[map_cell; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN];
    ve_values[0][1] = tps_cell;

    RuntimeSemanticCalibration {
        ve_table: RuntimeSemanticTable2dU16 {
            rpm_axis: semantic_axis2(),
            load_axis: semantic_axis2(),
            values: ve_values,
        },
        afr_target_table: semantic_table_u16(1470),
        deadtime_table_us: semantic_table_u16(0),
        clt_corr_curve: semantic_curve_u16(1000),
        iat_corr_curve: semantic_curve_u16(1000),
        baro_corr_curve: semantic_curve_u16(1000),
        vbat_corr_curve: semantic_curve_u16(1000),
        cranking_curve: semantic_curve_u16(1000),
        afterstart_table: semantic_table_u16(1000),
        warmup_curve: semantic_curve_u16(1000),
        ae_tps_threshold_curve: semantic_curve_u16(1000),
        ae_map_threshold_curve: semantic_curve_u16(1000),
        ae_shot_curve_us: semantic_curve_u16(0),
        ae_decay_steps_curve: semantic_curve_u16(1),
        ae_decay_ratio_curve_x1000: semantic_curve_u16(1000),
        required_fuel_us: 1000,
        pref_kpa10: 1000,
        stoich_afr_x100: 1470,
        pw_max_us: 20_000,
        afterstart_window_cycles: 0,
        dfco_entry_rpm: 9_000,
        dfco_exit_rpm: 8_900,
        dfco_entry_tps_x100: 1,
        dfco_exit_tps_x100: 2,
        dfco_entry_map_kpa10: 20,
        dfco_delay_cycles: 1,
        soft_rev_rpm: 9_000,
        hard_rev_rpm: 10_000,
        rev_hysteresis_rpm: 100,
        soft_retard_max_deg10: 0,
        launch_rpm_limit: 9_000,
        launch_cut_cycles: 0,
        flat_shift_rpm_min: 9_000,
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

fn semantic_fuel_observations(
    pw_corr_us: u32,
    fuel_cut: bool,
    spark_cut: bool,
) -> RuntimeSemanticFuelObservations {
    RuntimeSemanticFuelObservations {
        ve_pct_x100: 8000,
        target_afr_x100: 1470,
        pw_base_us: 1000,
        pw_air_us: 1000,
        pw_corr_us,
        fuel_cut,
        spark_cut,
        lambda_correction_x1000: 1000,
        lambda_integrator_state: RuntimeSemanticPiIntegratorState::default(),
        advance_deg10_trim: 0,
    }
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
        maf_x100: 420,
        clt_c10: 700,
        iat_c10: 300,
        baro_kpa10: Kpa10::new(1000),
        vbatt_mv: 12100,
        lambda_valid: true,
        lambda_measured: Lambda100::new(97),
        sync: SyncState::Locked { cam_ref: false },
        mode: FuelEngineMode::Running,
        fuel_cut_request: false,
        target_afr_override_x100: FuelAfrOverride::Some(1320),
    };
    let semantic = EngineRuntime::semantic_input_for_strategy(
        input,
        FuelLoadSource::Map,
        input.sync,
        input.mode,
    );

    assert_eq!(semantic.mode, RuntimeSemanticEngineMode::Running);
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
    let maf_inputs = running_control_inputs(1_000, 3000);
    // Keep control inputs stable; runtime FuelInputSnapshot now carries maf_x100.
    let maf_result = runtime.step(running_step_inputs(1_000, 3000, true, true), maf_inputs);

    // MAF path maps maf_x100 into semantic load and must not collapse to the MAP path value.
    assert_ne!(
        map_result.control.fuel_intent.pulse_width_us,
        maf_result.control.fuel_intent.pulse_width_us
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
    calibration.deadtime_table_us = semantic_table_u16(300);
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
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: SyncState::Locked { cam_ref: false },
        fuel_cut: false,
        spark_cut: false,
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
            ..semantic_schedule_input(3000, SyncState::Locked { cam_ref: false })
        },
        false,
        false,
        0,
        false,
    );
    assert_eq!(corr, 1000);
    assert_eq!(integ.acc, 0);
    assert!(!integ.frozen);
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
            rpm: input.rpm.0 as u32,
            load_kpa10: input.load_kpa10.0 as u32,
            angle_x10: 2_000,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
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
                clt_c: 80,
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
        },
    );

    assert_eq!(runtime.engine.rpm.get(), input.rpm.0);
    assert_eq!(runtime.engine.load_kpa10.get(), input.load_kpa10.0);
    assert_eq!(
        runtime.engine.sync,
        ecu_domain::SyncState::Locked { cam_ref: false }
    );
    assert_eq!(runtime.engine.mode, ecu_domain::ControlMode::ClosedLoop);
    assert_eq!(result.control.ignition.advance_deg10.get(), 150);
    assert_within(
        "pw_corr_us",
        result.control.enriched_fuel.get() as u32,
        oracle.output.pw_corr_us.0,
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
    assert_eq!(runtime.calibration_snapshot, CalibrationSnapshot::default());
    assert_eq!(
        runtime.output_profile(),
        RuntimeOutputProfile::legacy_single_channel()
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
        running_step_inputs(10, 3000, true, true),
        running_control_inputs(10, 3000),
        authority,
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
                clt_c: 80,
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
                clt_c: 80,
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
                clt_c: 80,
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
                clt_c: 80,
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
        },
    );

    let mut actions = result.actions.iter();
    assert_eq!(
        actions.next(),
        Some(Action::CancelScheduler(CancelReason::SafetyShutdown))
    );
    assert_eq!(actions.next(), Some(Action::PublishSnapshot));
    assert!(actions.next().is_none());
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
    assert_eq!(channels, [0, 1, 2]);
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
                assert_eq!(injection.start_at, Micros::new(5_000));
                assert!(injection.end_at.get() > injection.start_at.get());
                seen += 1;
            }
            Action::PublishSnapshot => {}
            other => panic!("unexpected action in injection-only path: {other:?}"),
        }
    }

    assert_eq!(runtime.engine.sync, SyncState::Locked { cam_ref: false });
    assert_eq!(seen, 4);
    assert_eq!(channels, [0, 1, 2, 3]);
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

fn assert_invalid_full_ecu_profile_is_inert(profile: FullEcuOutputProfile) {
    assert!(!profile.is_valid());
    assert_eq!(profile.event_count(), 0);

    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(test_fuel_model());
    runtime.configure_full_ecu(profile);
    runtime.set_engine_time_authority(validated_expert_authority());
    runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
        at_us: Micros::new(1_000),
        rpm: Rpm::new(3_000),
        angle_x10: Degrees10::new(120),
        synced: true,
    }));

    let result = runtime.step(
        running_step_inputs(5_000, 3_000, true, true),
        running_control_inputs(5_000, 3_000),
    );

    assert_eq!(arm_scheduler_count(result.actions), 0);
    assert_eq!(arm_injection_count(result.actions), 0);
    assert_eq!(arm_ignition_count(result.actions), 0);
    assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
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
                clt_c: 80,
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
        },
    );

    let mut injector_channels = [u8::MAX; 6];
    let mut ignition_channels = [u8::MAX; 6];
    let mut start_times = [0u32; 6];
    let mut injection_seen = 0usize;
    let mut ignition_seen = 0usize;
    let mut publish_seen = false;
    for action in result.actions.iter() {
        match action {
            Action::ArmInjection(injection) => {
                start_times[injection_seen] = injection.start_at.get();
                injector_channels[injection_seen] = injection.plan.output.channel().get();
                injection_seen += 1;
            }
            Action::ArmIgnition(ignition) => {
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
    assert!(start_times
        .windows(2)
        .all(|window| window[1] - window[0] == 6_666));
    assert!(start_times[5] - start_times[0] < 40_000);
    assert_eq!(injector_channels, [0, 1, 2, 3, 4, 5]);
    assert_eq!(ignition_channels, [0, 1, 2, 0, 1, 2]);
    assert!(ignition_channels.iter().all(|channel| *channel <= 2));
    assert!(publish_seen);
}

#[test]
fn runtime_full_ecu_cycle_slot_spacing_is_sixth_of_720_degree_cycle() {
    let profile = inline_sequential_wasted_spark_profile();

    assert_eq!(profile.cycle_slot_us(Rpm::new(0)), 0);
    assert_eq!(profile.cycle_slot_us(Rpm::new(3_000)), 6_666);
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
                clt_c: 80,
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
                clt_c: 80,
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
        },
    );

    assert_eq!(runtime.runtime_snapshot.engine, runtime.engine);
    assert_eq!(runtime.runtime_snapshot.control, runtime.control);
    assert_eq!(runtime.runtime_snapshot.faults, runtime.faults);
    assert_eq!(result.control.base_fuel.get(), 2500);
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
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(100),
                0,
                0,
                0,
                false,
                Rpm::new(3000),
            ),
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
                clt_c: 80,
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
