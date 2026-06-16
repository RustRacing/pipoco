use super::*;
use ecu_board_api::frontier::{TimingIslandPermitMask, TimingIslandStopReason};
use ecu_board_api::{EcuOutput, OutputLevel, OutputTransition, OutputTransitionBatch};
use ecu_domain::Ticks;
use ecu_runtime::{
    Action, ActionBatch, ActionExecutor, ActionOutputBatchAdapter, RUNTIME_AUX_COMMAND_CAP,
};
use ecu_scheduler::{
    ChannelId, ExclusiveChannel, IgnitionPlan as SchedulerIgnitionPlan,
    InjectionPlan as SchedulerInjectionPlan, Micros, OutputGroup, ScheduleError, ScheduledLevel,
    ScheduledTimingMetrics, ScheduledTransition, ScheduledTransitionKind, ScheduledTransitionQueue,
    SchedulerMode, TimedIgnitionPlan, TimedInjectionPlan, TransitionDrainBuffer,
};

#[derive(Default, Copy, Clone, PartialEq, Eq)]
struct RecordingPin {
    high_count: u8,
    low_count: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct PinFailure;

#[derive(Default)]
struct FailingPin {
    high_count: u8,
    low_count: u8,
}

fn board_transition(output: EcuOutput, level: OutputLevel, at_us: u32) -> OutputTransition {
    OutputTransition::new(output, level, Ticks::new(at_us))
}

impl embedded_hal::digital::v2::OutputPin for RecordingPin {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.low_count = self.low_count.saturating_add(1);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.high_count = self.high_count.saturating_add(1);
        Ok(())
    }
}

impl embedded_hal::digital::v2::OutputPin for FailingPin {
    type Error = PinFailure;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.low_count = self.low_count.saturating_add(1);
        Err(PinFailure)
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.high_count = self.high_count.saturating_add(1);
        Err(PinFailure)
    }
}

#[derive(Default)]
struct RecordingHal1Pin {
    high_count: u8,
    low_count: u8,
}

impl embedded_hal_1::digital::ErrorType for RecordingHal1Pin {
    type Error = core::convert::Infallible;
}

impl embedded_hal_1::digital::OutputPin for RecordingHal1Pin {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.low_count = self.low_count.saturating_add(1);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.high_count = self.high_count.saturating_add(1);
        Ok(())
    }
}

fn transition(
    kind: ScheduledTransitionKind,
    channel: u8,
    level: ScheduledLevel,
) -> ScheduledTransition {
    ScheduledTransition {
        at_us: Micros::new(100),
        kind,
        channel: ChannelId::new(channel),
        level,
    }
}

#[test]
fn apply_transition_routes_injector_and_ignition_outputs() {
    let mut inj0 = RecordingPin::default();
    let mut inj1 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut ign1 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 2] = [&mut inj0, &mut inj1];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 2] = [&mut ign0, &mut ign1];

    apply_transition(
        transition(ScheduledTransitionKind::Injector, 1, ScheduledLevel::High),
        &mut injectors,
        &mut ignition,
    )
    .expect("injector transition applies");
    apply_transition(
        transition(ScheduledTransitionKind::Ignition, 0, ScheduledLevel::Low),
        &mut injectors,
        &mut ignition,
    )
    .expect("ignition transition applies");

    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj1.high_count, 1);
    assert_eq!(ign0.low_count, 1);
    assert_eq!(ign1.low_count, 0);
}

#[test]
fn apply_transition_reports_hal_pin_write_failure() {
    let mut inj0 = FailingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];

    assert_eq!(
        apply_transition(
            transition(ScheduledTransitionKind::Injector, 0, ScheduledLevel::High),
            &mut injectors,
            &mut ignition,
        ),
        Err(TransitionApplyError::PinWrite {
            kind: ScheduledTransitionKind::Injector,
            channel: ChannelId::new(0),
            level: ScheduledLevel::High,
            error: ScheduledOutputPinError::SetHigh,
        })
    );
    assert_eq!(inj0.high_count, 1);
    assert_eq!(ign0.high_count, 0);
}

#[test]
fn apply_transition_reports_channel_range_errors() {
    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];

    assert_eq!(
        apply_transition(
            transition(ScheduledTransitionKind::Injector, 1, ScheduledLevel::High),
            &mut injectors,
            &mut ignition,
        ),
        Err(TransitionApplyError::InjectorChannelOutOfRange(
            ChannelId::new(1)
        ))
    );
    assert_eq!(
        apply_transition(
            transition(ScheduledTransitionKind::Ignition, 1, ScheduledLevel::High),
            &mut injectors,
            &mut ignition,
        ),
        Err(TransitionApplyError::IgnitionChannelOutOfRange(
            ChannelId::new(1)
        ))
    );
}

#[test]
fn apply_drained_transitions_applies_valid_len_only() {
    let mut drained = TransitionDrainBuffer::<4> {
        len: 2,
        transitions: [None; 4],
    };
    drained.transitions[0] = Some(transition(
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::High,
    ));
    drained.transitions[1] = Some(transition(
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::Low,
    ));
    drained.transitions[3] = Some(transition(
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::High,
    ));

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];

    let applied = apply_drained_transitions(&drained, &mut injectors, &mut ignition)
        .expect("drained transitions apply");
    assert_eq!(applied, 2);
    assert_eq!(inj0.high_count, 1);
    assert_eq!(inj0.low_count, 1);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn apply_drained_transitions_rejects_invalid_batch_without_partial_pin_writes() {
    let mut drained = TransitionDrainBuffer::<4> {
        len: 2,
        transitions: [None; 4],
    };
    drained.transitions[0] = Some(transition(
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::High,
    ));
    drained.transitions[1] = Some(transition(
        ScheduledTransitionKind::Ignition,
        1,
        ScheduledLevel::High,
    ));

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];

    assert_eq!(
        apply_drained_transitions(&drained, &mut injectors, &mut ignition),
        Err(TransitionApplyError::IgnitionChannelOutOfRange(
            ChannelId::new(1)
        ))
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn apply_drained_transitions_reports_pin_write_failure() {
    let mut drained = TransitionDrainBuffer::<2> {
        len: 1,
        transitions: [None; 2],
    };
    drained.transitions[0] = Some(transition(
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::Low,
    ));
    let mut inj0 = FailingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];

    assert_eq!(
        apply_drained_transitions(&drained, &mut injectors, &mut ignition),
        Err(TransitionApplyError::PinWrite {
            kind: ScheduledTransitionKind::Injector,
            channel: ChannelId::new(0),
            level: ScheduledLevel::Low,
            error: ScheduledOutputPinError::SetLow,
        })
    );
    assert_eq!(inj0.low_count, 1);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn scheduler_queue_drain_applies_to_target_pins() {
    let mut queue = ScheduledTransitionQueue::<4>::new();
    queue
        .enqueue_transition(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::High,
        ))
        .expect("injector open fits");
    queue
        .enqueue_transition(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::Low,
        ))
        .expect("injector close fits");

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(queue.drain_due(Micros::new(200), &mut drained), 2);

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];

    assert_eq!(
        apply_drained_transitions(&drained, &mut injectors, &mut ignition),
        Ok(2)
    );
    assert_eq!(inj0.high_count, 1);
    assert_eq!(inj0.low_count, 1);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn scheduled_action_executor_exposes_queue_timing_metrics() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    assert_eq!(
        executor.timing_metrics(),
        ScheduledTimingMetrics {
            late_event_count: 0,
            max_lateness_us: None,
            queue_high_water_mark: 4,
            last_drain_count: 0,
        }
    );

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(executor.drain_due(Micros::new(1_000), &mut drained), 4);

    assert_eq!(
        executor.timing_metrics(),
        ScheduledTimingMetrics {
            late_event_count: 4,
            max_lateness_us: Some(Micros::new(900)),
            queue_high_water_mark: 4,
            last_drain_count: 4,
        }
    );
}

fn timed_injection(start_at: u32, end_at: u32) -> TimedInjectionPlan {
    timed_injection_on(0, start_at, end_at)
}

fn timed_injection_on(channel: u8, start_at: u32, end_at: u32) -> TimedInjectionPlan {
    TimedInjectionPlan {
        plan: SchedulerInjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(channel)),
            pulse_width: ecu_scheduler::PulseWidthUs::new((end_at - start_at) as u16),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}

fn timed_ignition(start_at: u32, end_at: u32) -> TimedIgnitionPlan {
    timed_ignition_on(0, start_at, end_at)
}

fn timed_ignition_on(channel: u8, start_at: u32, end_at: u32) -> TimedIgnitionPlan {
    TimedIgnitionPlan {
        plan: SchedulerIgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(channel)),
            dwell: ecu_scheduler::DwellUs::new((end_at - start_at) as u16),
            advance: ecu_scheduler::Degrees10::new(100),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}

fn assert_drained_transition<const N: usize>(
    drained: &TransitionDrainBuffer<N>,
    index: usize,
    kind: ScheduledTransitionKind,
    channel: u8,
    level: ScheduledLevel,
    at_us: u32,
) {
    assert_eq!(
        drained.transitions[index].expect("transition present"),
        ScheduledTransition {
            at_us: Micros::new(at_us),
            kind,
            channel: ChannelId::new(channel),
            level,
        }
    );
}

#[test]
fn scheduled_action_executor_enqueues_runtime_scheduler_action() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    assert_eq!(executor.frontier().mode(), SchedulerMode::Armed);
    assert_eq!(executor.frontier().active_groups(), 0b11);
    assert_eq!(executor.frontier().injection_count(), 1);
    assert_eq!(executor.frontier().ignition_count(), 1);
    assert_eq!(executor.queue().active_count(), 4);

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(executor.drain_due(Micros::new(300), &mut drained), 4);
    assert_drained_transition(
        &drained,
        0,
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::High,
        100,
    );
    assert_drained_transition(
        &drained,
        1,
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::Low,
        120,
    );
    assert_drained_transition(
        &drained,
        2,
        ScheduledTransitionKind::Ignition,
        0,
        ScheduledLevel::High,
        200,
    );
    assert_drained_transition(
        &drained,
        3,
        ScheduledTransitionKind::Ignition,
        0,
        ScheduledLevel::Low,
        230,
    );
}

#[test]
fn board_api_batch_executor_lowers_runtime_batch_to_board_api_batches() {
    let mut executor =
        ActionOutputBatchAdapter::<ACTION_OUTPUT_SCRATCH_CAP, RUNTIME_AUX_COMMAND_CAP>::new();
    let mut batch = ActionBatch::<2>::new();
    assert!(batch.push(Action::ArmInjection(timed_injection(100, 120))));
    let mut fan_commands = ecu_board_api::AuxCommandBatch::<RUNTIME_AUX_COMMAND_CAP>::new();
    fan_commands
        .push(ecu_board_api::AuxCommand::new(
            ecu_board_api::AuxOutput::SafetyRelay(1),
            ecu_board_api::AuxValue::Level(ecu_board_api::OutputLevel::High),
        ))
        .expect("push fan command");
    assert!(batch.push(Action::ApplyAux(fan_commands)));

    executor
        .execute_batch(batch)
        .expect("runtime batch lowers to board api batches");

    assert_eq!(executor.output_transitions().len(), 2);
    assert_eq!(executor.aux_commands().len(), 1);
    assert_eq!(executor.status().scheduled_output_transitions, 2);
    assert_eq!(executor.status().applied_aux_commands, 1);
    assert_eq!(
        executor.output_transitions().as_slice()[0],
        board_transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            100
        )
    );
    assert_eq!(
        executor.output_transitions().as_slice()[1],
        board_transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            120
        )
    );
}

#[test]
fn scheduler_queue_adapter_rejects_output_batch_overflow_without_partial_queue_writes() {
    let mut scheduler = ScheduledQueueAdapter::<2>::new();
    scheduler
        .queue_mut()
        .enqueue_transition(transition(
            ScheduledTransitionKind::Injector,
            0,
            ScheduledLevel::High,
        ))
        .expect("seed transition queues");
    let before = scheduler.queue().snapshot();

    let mut outputs = OutputTransitionBatch::<2>::new();
    outputs
        .push(board_transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::High,
            100,
        ))
        .expect("first output transition fits batch");
    outputs
        .push(board_transition(
            EcuOutput::Injector(ChannelId::new(0)),
            OutputLevel::Low,
            120,
        ))
        .expect("second output transition fits batch");

    assert_eq!(
        scheduler.schedule_output_batch(&outputs),
        Err(ScheduleError::QueueFull)
    );
    assert_eq!(
        scheduler.queue().snapshot(),
        before,
        "oversized output batch must not partially enqueue"
    );
}

#[test]
fn scheduled_action_executor_enqueues_ignition_only_action() {
    let mut executor = ScheduledActionExecutor::<2>::new();
    executor
        .execute(Action::ArmIgnition(timed_ignition(200, 230)))
        .expect("ignition-only action queues");

    assert_eq!(executor.queue().active_count(), 2);

    let mut drained = TransitionDrainBuffer::<2>::new();
    assert_eq!(executor.drain_due(Micros::new(300), &mut drained), 2);
    assert_drained_transition(
        &drained,
        0,
        ScheduledTransitionKind::Ignition,
        0,
        ScheduledLevel::High,
        200,
    );
    assert_drained_transition(
        &drained,
        1,
        ScheduledTransitionKind::Ignition,
        0,
        ScheduledLevel::Low,
        230,
    );
}

#[test]
fn scheduled_action_executor_enqueues_injection_only_action() {
    let mut executor = ScheduledActionExecutor::<2>::new();
    executor
        .execute(Action::ArmInjection(timed_injection(100, 120)))
        .expect("injection-only action queues");

    assert_eq!(executor.queue().active_count(), 2);

    let mut drained = TransitionDrainBuffer::<2>::new();
    assert_eq!(executor.drain_due(Micros::new(300), &mut drained), 2);
    assert_drained_transition(
        &drained,
        0,
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::High,
        100,
    );
    assert_drained_transition(
        &drained,
        1,
        ScheduledTransitionKind::Injector,
        0,
        ScheduledLevel::Low,
        120,
    );
}

#[test]
fn scheduled_action_executor_cancel_clears_queued_transitions() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    executor
        .execute(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss))
        .expect("cancel clears queue");

    assert_eq!(executor.queue().active_count(), 0);
}

#[test]
fn scheduled_action_executor_ignores_apply_aux() {
    let mut executor = ScheduledActionExecutor::<4>::new();

    executor
        .execute(Action::ApplyAux(Default::default()))
        .expect("aux action is ignored");

    assert_eq!(executor.queue().active_count(), 0);
}

#[test]
fn scheduled_action_executor_rejects_atomic_overflow() {
    let mut executor = ScheduledActionExecutor::<3>::new();
    assert_eq!(
        executor.execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        }),
        Err(ScheduleError::QueueFull)
    );
    assert_eq!(
        executor.queue().active_count(),
        0,
        "overflow must not partially enqueue"
    );
}

#[test]
fn scheduled_action_executor_batch_overflow_preserves_live_queue() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmInjection(timed_injection(10, 20)))
        .expect("seed transition queues");
    let before = executor.queue().snapshot();

    let mut batch = ActionBatch::<2>::new();
    assert!(batch.push(Action::ArmInjection(timed_injection_on(1, 100, 120))));
    assert!(batch.push(Action::ArmIgnition(timed_ignition_on(1, 200, 230))));

    assert_eq!(executor.execute_batch(batch), Err(ScheduleError::QueueFull));
    assert_eq!(
        executor.queue().snapshot(),
        before,
        "batch overflow must not commit earlier actions"
    );
}

#[test]
fn scheduled_action_executor_batch_cancel_order_keeps_post_cancel_actions() {
    let mut executor = ScheduledActionExecutor::<4>::new();

    let mut batch = ActionBatch::<3>::new();
    assert!(batch.push(Action::ArmInjection(timed_injection(10, 20))));
    assert!(batch.push(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss)));
    assert!(batch.push(Action::ArmIgnition(timed_ignition(200, 230))));

    executor
        .execute_batch(batch)
        .expect("cancel then post-cancel action queues");

    assert_eq!(executor.queue().active_count(), 2);

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(executor.drain_due(Micros::new(300), &mut drained), 2);
    assert_eq!(
        drained.transitions[0].expect("coil charge").kind,
        ScheduledTransitionKind::Ignition
    );
    assert_eq!(
        drained.transitions[1].expect("coil fire").kind,
        ScheduledTransitionKind::Ignition
    );
}

#[test]
fn scheduled_action_executor_batch_cancel_first_keeps_later_actions() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmInjection(timed_injection(10, 20)))
        .expect("seed transition queues");

    let mut batch = ActionBatch::<2>::new();
    assert!(batch.push(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss)));
    assert!(batch.push(Action::ArmIgnition(timed_ignition(200, 230))));

    executor
        .execute_batch(batch)
        .expect("post-cancel action queues");

    assert_eq!(executor.queue().active_count(), 2);

    let mut drained = TransitionDrainBuffer::<2>::new();
    assert_eq!(executor.drain_due(Micros::new(300), &mut drained), 2);
    assert_drained_transition(
        &drained,
        0,
        ScheduledTransitionKind::Ignition,
        0,
        ScheduledLevel::High,
        200,
    );
    assert_drained_transition(
        &drained,
        1,
        ScheduledTransitionKind::Ignition,
        0,
        ScheduledLevel::Low,
        230,
    );
}

#[test]
fn scheduled_action_executor_batch_cancel_last_clears_earlier_actions() {
    let mut executor = ScheduledActionExecutor::<4>::new();

    let mut batch = ActionBatch::<2>::new();
    assert!(batch.push(Action::ArmInjection(timed_injection(100, 120))));
    assert!(batch.push(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss)));

    executor
        .execute_batch(batch)
        .expect("cancel clears scratch queue");

    assert_eq!(executor.queue().active_count(), 0);
}

#[test]
fn scheduled_action_executor_batch_cancel_then_overflow_preserves_live_queue() {
    let mut executor = ScheduledActionExecutor::<3>::new();
    executor
        .execute(Action::ArmInjection(timed_injection(10, 20)))
        .expect("seed transition queues");
    let before = executor.queue().snapshot();

    let mut batch = ActionBatch::<2>::new();
    assert!(batch.push(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss)));
    assert!(batch.push(Action::ArmScheduler {
        injection: timed_injection(100, 120),
        ignition: timed_ignition(200, 230),
    }));

    assert_eq!(executor.execute_batch(batch), Err(ScheduleError::QueueFull));
    assert_eq!(
        executor.queue().snapshot(),
        before,
        "cancel before overflow must not commit scratch queue changes"
    );
}

#[test]
fn scheduled_action_executor_drain_and_apply_due_is_board_tick_ready() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        executor.drain_and_apply_due(
            Micros::new(240),
            &mut drained,
            &mut injectors,
            &mut ignition
        ),
        Ok(4)
    );
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(inj0.high_count, 1);
    assert_eq!(inj0.low_count, 1);
    assert_eq!(ign0.high_count, 1);
    assert_eq!(ign0.low_count, 1);
}

#[test]
fn scheduled_action_executor_frontier_commit_and_expiry_gate_drain() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    let permit_mask = TimingIslandPermitMask::new(
        TimingIslandPermitMask::IGNITION | TimingIslandPermitMask::INJECTOR,
    );
    assert!(executor.commit_frontier_horizon(
        21,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        permit_mask,
    ));

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        executor.drain_due_with_frontier(Micros::new(140), &mut drained),
        2
    );
    assert_eq!(executor.queue().active_count(), 2);
    assert_eq!(executor.frontier().active_horizon_id(), Some(21));
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::None
    );
    assert_eq!(executor.frontier().active_permit_mask(), permit_mask);
}

#[test]
fn scheduled_action_executor_frontier_heartbeat_expiry_clears_queue() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    assert!(executor.commit_frontier_horizon(
        22,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        TimingIslandPermitMask::ALL,
    ));

    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        executor.drain_due_with_frontier(Micros::new(151), &mut drained),
        0
    );
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::HeartbeatExpired
    );
    assert_eq!(
        executor.frontier().active_permit_mask(),
        TimingIslandPermitMask::NONE
    );
    assert_eq!(executor.frontier().active_horizon_id(), Some(22));

    assert_eq!(
        executor.drain_due_with_frontier(Micros::new(261), &mut drained),
        0
    );
    assert_eq!(executor.frontier().active_horizon_id(), None);
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::HeartbeatExpired
    );
}

#[test]
fn scheduled_action_executor_drain_and_apply_due_stays_frontier_aware_after_expiry() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    assert!(executor.commit_frontier_horizon(
        23,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        TimingIslandPermitMask::ALL,
    ));
    executor.expire_frontier(Micros::new(261));

    let mut inj0 = RecordingPin::default();
    let mut inj1 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut ign1 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 2] = [&mut inj0, &mut inj1];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 2] = [&mut ign0, &mut ign1];
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        executor.drain_and_apply_due(
            Micros::new(300),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Ok(0)
    );
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(inj1.high_count, 0);
    assert_eq!(inj1.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
    assert_eq!(ign1.high_count, 0);
    assert_eq!(ign1.low_count, 0);
}

#[test]
fn scheduled_action_executor_frontier_reset_paths_clear_live_state() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    assert!(executor.commit_frontier_horizon(
        23,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        TimingIslandPermitMask::ALL,
    ));
    executor.on_sync_loss();
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(executor.frontier().active_horizon_id(), None);
    assert_eq!(
        executor.frontier().active_permit_mask(),
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::SyncLost
    );

    assert!(executor.commit_frontier_horizon(
        24,
        Micros::new(300),
        Micros::new(420),
        Micros::new(340),
        TimingIslandPermitMask::ALL,
    ));
    executor.on_hard_safety_shutdown();
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(executor.frontier().active_horizon_id(), None);
    assert_eq!(
        executor.frontier().active_permit_mask(),
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::TimingFault
    );
}

#[test]
fn scheduled_action_executor_runtime_cancel_preserves_frontier_stop_reason() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    assert!(executor.commit_frontier_horizon(
        30,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        TimingIslandPermitMask::ALL,
    ));

    executor
        .execute(Action::CancelScheduler(ecu_domain::CancelReason::SyncLoss))
        .expect("sync loss cancel clears queue");
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(executor.frontier().active_horizon_id(), None);
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::SyncLost
    );

    assert!(executor.commit_frontier_horizon(
        31,
        Micros::new(300),
        Micros::new(520),
        Micros::new(340),
        TimingIslandPermitMask::ALL,
    ));
    executor
        .execute(Action::CancelScheduler(
            ecu_domain::CancelReason::SafetyShutdown,
        ))
        .expect("safety shutdown cancel clears queue");
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(executor.frontier().active_horizon_id(), None);
    assert_eq!(
        executor.frontier().active_stop_reason(),
        TimingIslandStopReason::TimingFault
    );
}

#[test]
fn scheduled_action_executor_drain_and_apply_due_respects_active_frontier_permissions() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");

    assert!(executor.commit_frontier_horizon(
        40,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        TimingIslandPermitMask::NONE,
    ));

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        executor.drain_and_apply_due(
            Micros::new(140),
            &mut drained,
            &mut injectors,
            &mut ignition
        ),
        Ok(0)
    );
    assert_eq!(executor.queue().active_count(), 0);
    assert_eq!(executor.frontier().active_horizon_id(), Some(40));
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn scheduled_outputs4_drains_and_applies_due_transitions() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");
    assert!(executor.commit_frontier_horizon(
        24,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));

    let inj0 = RecordingPin::default();
    let inj1 = RecordingPin::default();
    let ign0 = RecordingPin::default();
    let ign1 = RecordingPin::default();
    let mut outputs = ScheduledOutputs4::new(inj0, inj1, ign0, ign1);
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        outputs.drain_and_apply_due(&mut executor, Micros::new(240), &mut drained),
        Ok(4)
    );

    let (inj0, inj1, ign0, ign1) = outputs.into_inner();
    assert_eq!(inj0.high_count, 1);
    assert_eq!(inj0.low_count, 1);
    assert_eq!(inj1.high_count, 0);
    assert_eq!(inj1.low_count, 0);
    assert_eq!(ign0.high_count, 1);
    assert_eq!(ign0.low_count, 1);
    assert_eq!(ign1.high_count, 0);
    assert_eq!(ign1.low_count, 0);
}

#[test]
fn scheduled_outputs4_accepts_embedded_hal_1_wrapped_pins() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");
    assert!(executor.commit_frontier_horizon(
        25,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));

    let inj0 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
    let inj1 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
    let ign0 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
    let ign1 = Hal1ScheduledOut::new(RecordingHal1Pin::default());
    let mut outputs = ScheduledOutputs4::new(inj0, inj1, ign0, ign1);
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        outputs.drain_and_apply_due(&mut executor, Micros::new(240), &mut drained),
        Ok(4)
    );

    let (inj0, inj1, ign0, ign1) = outputs.into_inner();
    let inj0 = inj0.into_inner();
    let inj1 = inj1.into_inner();
    let ign0 = ign0.into_inner();
    let ign1 = ign1.into_inner();
    assert_eq!(inj0.high_count, 1);
    assert_eq!(inj0.low_count, 1);
    assert_eq!(inj1.high_count, 0);
    assert_eq!(inj1.low_count, 0);
    assert_eq!(ign0.high_count, 1);
    assert_eq!(ign0.low_count, 1);
    assert_eq!(ign1.high_count, 0);
    assert_eq!(ign1.low_count, 0);
}

#[test]
fn scheduled_outputs_generic_supports_six_injectors_and_three_ignition_channels() {
    let mut executor = ScheduledActionExecutor::<4>::new();
    executor
        .queue_mut()
        .enqueue_transition(ScheduledTransition {
            at_us: Micros::new(100),
            kind: ScheduledTransitionKind::Injector,
            channel: ecu_domain::ChannelId::new(5),
            level: ScheduledLevel::High,
        })
        .expect("injector transition fits");
    executor
        .queue_mut()
        .enqueue_transition(ScheduledTransition {
            at_us: Micros::new(100),
            kind: ScheduledTransitionKind::Ignition,
            channel: ecu_domain::ChannelId::new(2),
            level: ScheduledLevel::Low,
        })
        .expect("ignition transition fits");

    let mut outputs = ScheduledOutputs::<6, 3, RecordingPin, RecordingPin>::new(
        [RecordingPin::default(); 6],
        [RecordingPin::default(); 3],
    );
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        outputs.drain_and_apply_due(&mut executor, Micros::new(200), &mut drained),
        Ok(2)
    );

    let (injectors, ignition) = outputs.into_inner();
    assert_eq!(injectors[5].high_count, 1);
    assert_eq!(injectors[5].low_count, 0);
    assert_eq!(ignition[2].high_count, 0);
    assert_eq!(ignition[2].low_count, 1);
}
