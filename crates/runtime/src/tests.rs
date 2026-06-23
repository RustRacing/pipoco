use super::*;
use crate::compat::StepInputs;
use crate::semantic::runtime_semantic_evaluate_fuel;
use crate::support::DifferentialInputSnapshot;
use crate::support::{
    extract_action_observations, extract_authority_observations,
    extract_calibration_identity_observations, extract_calibration_observations,
    extract_control_observations, extract_cut_observations, extract_engine_observations,
    extract_fault_observations, extract_fuel_core_observations, extract_fuel_observations,
    extract_fuel_strategy_observations, extract_idle_observations, extract_ignition_observations,
    extract_ignition_trim_observations, extract_knock_observations,
    extract_lambda_correction_observations, extract_lambda_observations,
    extract_output_profile_observations, extract_protection_observations,
    extract_runtime_observed_surface, extract_scheduler_observations, extract_torque_observations,
    extract_transition_observations, extract_validated_observations,
};
use ecu_board_api::{
    AuxCommand, AuxCommandBatch, AuxOutput, AuxValue, EcuOutput, OutputLevel, OutputTransition,
    OutputTransitionBatch, TimingIslandCommand, TimingIslandCommandBatch,
};
#[cfg(test)]
use ecu_calibration::{
    ActiveCalibration, Calibration, CalibrationPackageIdentity, CalibrationRevision,
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertUnlock,
    SecondaryTriggerMode, StagedCalibration, TriggerAuthority,
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
        safety_latch_request: false,
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
            now_us: Micros::new(now_us),
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
            Rpm::new(rpm),
        ),
        fuel_sensors: FuelSensorInputs::default(),
        knock_intensity_x100: 0,
    }
}

fn spark_only_control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
    ControlInputs::spark_only(
        Micros::new(now_us),
        IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(rpm)),
    )
}

fn differential_running_input(mode: RuntimeEngineMode) -> DifferentialInputSnapshot {
    DifferentialInputSnapshot {
        now_us: Micros::new(1_000),
        rpm: Rpm::new(3_000),
        map_kpa10: Kpa10::new(700),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(2_000),
        clt_c10: 200,
        iat_c10: 250,
        baro_kpa10: Kpa10::new(1_010),
        vbatt_mv: 12_000,
        sync: SyncState::Locked { cam_ref: false },
        fuel_cut: false,
        spark_cut: false,
        mode,
        target_afr_override_x100: RuntimeAfrOverride::None,
        launch_armed: false,
        flat_shift_armed: false,
        safety_latch_request: false,
    }
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

fn semantic_deadtime_table_u16(value: u16) -> RuntimeSemanticDeadtimeTableU16 {
    RuntimeSemanticDeadtimeTableU16 {
        vbat_mv_axis: semantic_axis2(),
        pressure_kpa10_axis: semantic_axis2(),
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
        lambda_valid: true,
        lambda_measured: Lambda100::new(100),
        requested_open_loop: false,
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync,
        fuel_cut: false,
        spark_cut: false,
        direct_fuel_cut_request: false,
        direct_spark_cut_request: false,
        safety_latch_request: false,
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
        deadtime_table_us: semantic_deadtime_table_u16(0),
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
        idle_target_rpm: 0,
        idle_base_duty_x1000: 0,
        idle_kp_x1000: 0,
        idle_ki_x1000: 0,
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
        warmup_corr_x1000: 1000,
        fuel_cut,
        spark_cut,
        lambda_correction_x1000: 1000,
        lambda_integrator_state: RuntimeSemanticPiIntegratorState::default(),
        idle_duty_x1000: 0,
        idle_integrator_state: RuntimeSemanticPiIntegratorState {
            acc: 0,
            min_acc: -2000,
            max_acc: 2000,
            frozen: false,
        },
        advance_deg10_trim: 0,
    }
}

fn semantic_fuel_state_calibration() -> RuntimeSemanticCalibration {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(100, 100);
    calibration.warmup_curve = semantic_curve_u16(1200);
    calibration.afterstart_table = semantic_table_u16(1200);
    calibration.afterstart_window_cycles = 3;
    calibration.ae_tps_threshold_curve = semantic_curve_u16(1);
    calibration.ae_map_threshold_curve = semantic_curve_u16(1);
    calibration.ae_shot_curve_us = semantic_curve_u16(250);
    calibration.ae_decay_steps_curve = semantic_curve_u16(2);
    calibration.ae_decay_ratio_curve_x1000 = semantic_curve_u16(1000);
    calibration
}

fn semantic_launch_and_flat_shift_calibration() -> RuntimeSemanticCalibration {
    let mut calibration = semantic_fuel_calibration_with_ve_cells(7000, 7000);
    calibration.launch_rpm_limit = 2_500;
    calibration.launch_cut_cycles = 0;
    calibration.flat_shift_rpm_min = 2_500;
    calibration.flat_shift_cut_cycles = 0;
    calibration
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

    assert_eq!(fuel.base_fuel_pw_us, result.control.base_fuel.get());
    assert_eq!(fuel.enriched_fuel_pw_us, result.control.enriched_fuel.get());
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
        result.control.base_fuel.get()
    );
    assert_eq!(
        observed.runtime_enriched_fuel_pw_us,
        result.control.enriched_fuel.get()
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
