use ecu_board_api::frontier::{
    TimingIslandHorizonSequenceId, TimingIslandPermitMask, TimingIslandStopReason,
};
use ecu_board_api::{
    EcuOutput, OutputLevel as BoardOutputLevel, OutputScheduler, OutputTransition,
    OutputTransitionBatch,
};
use ecu_runtime::{
    Action, ActionBatch, ActionExecutor, ActionLoweringError, ActionOutputBatchAdapter,
    RUNTIME_AUX_COMMAND_CAP,
};
use ecu_scheduler::{
    ExclusiveChannel, ScheduleError, ScheduledLevel, ScheduledTimingMetrics, ScheduledTransition,
    ScheduledTransitionKind, ScheduledTransitionQueue, SchedulerState, TransitionDrainBuffer,
};

use super::pins::{apply_drained_transitions, RawScheduledOutputPin, TransitionApplyError};

pub const ACTION_OUTPUT_SCRATCH_CAP: usize = 4;
/// Adapter that queues logical board output batches on the split scheduler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledQueueAdapter<const N: usize> {
    queue: ScheduledTransitionQueue<N>,
    frontier: SchedulerState,
}

impl<const N: usize> ScheduledQueueAdapter<N> {
    pub const fn new() -> Self {
        Self {
            queue: ScheduledTransitionQueue::new(),
            frontier: SchedulerState::new(),
        }
    }

    pub const fn queue(&self) -> &ScheduledTransitionQueue<N> {
        &self.queue
    }

    pub const fn timing_metrics(&self) -> ScheduledTimingMetrics {
        self.queue.timing_metrics()
    }

    pub fn queue_mut(&mut self) -> &mut ScheduledTransitionQueue<N> {
        &mut self.queue
    }

    pub const fn frontier(&self) -> &SchedulerState {
        &self.frontier
    }

    pub fn frontier_mut(&mut self) -> &mut SchedulerState {
        &mut self.frontier
    }

    pub fn commit_frontier_horizon(
        &mut self,
        horizon_id: TimingIslandHorizonSequenceId,
        horizon_start_us: ecu_scheduler::Micros,
        horizon_end_us: ecu_scheduler::Micros,
        heartbeat_deadline_us: ecu_scheduler::Micros,
        permit_mask: TimingIslandPermitMask,
    ) -> bool {
        self.frontier.commit_horizon(
            horizon_id,
            horizon_start_us,
            horizon_end_us,
            heartbeat_deadline_us,
            permit_mask,
        )
    }

    pub fn note_frontier_heartbeat(&mut self, now: ecu_scheduler::Micros) {
        self.frontier.note_heartbeat(now);
    }

    pub fn expire_frontier(&mut self, now: ecu_scheduler::Micros) {
        self.frontier.expire_frontier(now);
    }

    pub fn on_sync_loss(&mut self) {
        self.frontier.on_sync_loss();
        self.queue.on_sync_loss();
    }

    pub fn on_hard_safety_shutdown(&mut self) {
        self.frontier.on_hard_safety_shutdown();
        self.queue.on_hard_safety_shutdown();
    }

    pub fn into_inner(self) -> ScheduledTransitionQueue<N> {
        self.queue
    }

    pub fn cancel_all(&mut self) {
        self.cancel_all_with_reason(TimingIslandStopReason::PermitDenied);
    }

    fn has_live_frontier(&self) -> bool {
        self.frontier.active_horizon_id().is_some()
    }

    fn cancel_all_with_reason(&mut self, reason: TimingIslandStopReason) {
        self.frontier.clear_scheduled_ownership();
        self.frontier.clear_live_frontier_state(reason);
        self.queue.cancel_all();
    }

    pub fn drain_due<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        let drained = self.queue.drain_due(now, out);
        self.note_drained_ownership(out);
        drained
    }

    pub fn drain_due_with_frontier<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        let drained = self
            .queue
            .drain_due_with_frontier(now, &mut self.frontier, out);
        if drained == 0
            && (self.frontier.active_horizon_id().is_none()
                || self.frontier.active_permit_mask().is_empty())
        {
            self.frontier.clear_scheduled_ownership();
        }
        self.note_drained_ownership(out);
        drained
    }

    pub fn schedule_output_batch<const M: usize>(
        &mut self,
        batch: &OutputTransitionBatch<M>,
    ) -> Result<(), ScheduleError> {
        if self.queue.free_slots() < batch.len() {
            if self.has_live_frontier() {
                self.cancel_all_with_reason(TimingIslandStopReason::AdmittedEventRejected);
            }
            return Err(ScheduleError::QueueFull);
        }

        let mut scratch = *self;
        if let Err(error) = scratch.note_output_batch_ownership(batch) {
            if self.has_live_frontier() {
                self.cancel_all_with_reason(TimingIslandStopReason::AdmittedEventRejected);
            }
            return Err(error);
        }
        for transition in batch.iter() {
            if let Err(error) = scratch
                .queue
                .enqueue_transition(scheduled_transition(*transition))
            {
                if self.has_live_frontier() {
                    self.cancel_all_with_reason(TimingIslandStopReason::AdmittedEventRejected);
                }
                return Err(error);
            }
        }
        *self = scratch;
        Ok(())
    }

    pub fn apply_lowered<const OUT: usize, const AUX: usize>(
        &mut self,
        lowered: &ActionOutputBatchAdapter<OUT, AUX>,
    ) -> Result<(), ScheduleError> {
        let mut scratch = *self;
        if let Some(reason) = lowered.status().cancel_scheduler {
            scratch.cancel_all_with_reason(map_cancel_reason(reason));
        }
        if let Err(error) = scratch.schedule_output_batch(lowered.output_transitions()) {
            if self.has_live_frontier() {
                self.cancel_all_with_reason(TimingIslandStopReason::AdmittedEventRejected);
            }
            return Err(error);
        }
        *self = scratch;
        Ok(())
    }
}

impl<const N: usize> ScheduledQueueAdapter<N> {
    fn note_output_batch_ownership<const M: usize>(
        &mut self,
        batch: &OutputTransitionBatch<M>,
    ) -> Result<(), ScheduleError> {
        let mut unique_outputs = [None; M];
        let mut unique_count = 0usize;

        for transition in batch.iter() {
            if unique_outputs[..unique_count]
                .iter()
                .flatten()
                .any(|output| *output == transition.output)
            {
                continue;
            }
            unique_outputs[unique_count] = Some(transition.output);
            unique_count += 1;
        }

        for output in unique_outputs[..unique_count].iter().flatten().copied() {
            let mut starts = [ecu_scheduler::Micros::new(0); M];
            let mut ends = [ecu_scheduler::Micros::new(0); M];
            let mut start_count = 0usize;
            let mut end_count = 0usize;

            for transition in batch
                .iter()
                .filter(|transition| transition.output == output)
            {
                match transition.level {
                    BoardOutputLevel::High => {
                        starts[start_count] = ecu_scheduler::Micros::new(transition.at.get());
                        start_count += 1;
                    }
                    BoardOutputLevel::Low => {
                        ends[end_count] = ecu_scheduler::Micros::new(transition.at.get());
                        end_count += 1;
                    }
                }
            }

            if start_count != end_count {
                return Err(ScheduleError::ImpossibleDeadline);
            }

            starts[..start_count].sort_unstable_by_key(|at| at.get());
            ends[..end_count].sort_unstable_by_key(|at| at.get());

            let output = scheduled_output(output);
            for idx in 0..start_count {
                self.frontier
                    .reserve_window(output, starts[idx], ends[idx])?;
            }
        }

        Ok(())
    }

    fn note_drained_ownership<const M: usize>(&mut self, out: &TransitionDrainBuffer<M>) {
        for transition in out.as_slice().iter().flatten().copied() {
            self.frontier.note_drained_transition(transition);
        }
    }
}

impl<const N: usize> Default for ScheduledQueueAdapter<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const Q: usize, const N: usize> OutputScheduler<N> for ScheduledQueueAdapter<Q> {
    type Error = ScheduleError;

    fn schedule(&mut self, batch: &OutputTransitionBatch<N>) -> Result<(), Self::Error> {
        self.schedule_output_batch(batch)
    }

    fn cancel_all(&mut self) -> Result<(), Self::Error> {
        ScheduledQueueAdapter::cancel_all(self);
        Ok(())
    }

    fn force_safe_state(&mut self) {
        self.cancel_all();
    }
}

/// Action executor that preserves the older split-scheduler compatibility
/// surface while delegating lowering and queueing to the smaller seams above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledActionExecutor<const N: usize> {
    scheduler: ScheduledQueueAdapter<N>,
}

impl<const N: usize> ScheduledActionExecutor<N> {
    pub const fn new() -> Self {
        Self {
            scheduler: ScheduledQueueAdapter::new(),
        }
    }

    pub const fn queue(&self) -> &ScheduledTransitionQueue<N> {
        self.scheduler.queue()
    }

    pub const fn timing_metrics(&self) -> ScheduledTimingMetrics {
        self.scheduler.timing_metrics()
    }

    pub fn queue_mut(&mut self) -> &mut ScheduledTransitionQueue<N> {
        self.scheduler.queue_mut()
    }

    pub const fn frontier(&self) -> &SchedulerState {
        self.scheduler.frontier()
    }

    pub fn frontier_mut(&mut self) -> &mut SchedulerState {
        self.scheduler.frontier_mut()
    }

    pub fn commit_frontier_horizon(
        &mut self,
        horizon_id: TimingIslandHorizonSequenceId,
        horizon_start_us: ecu_scheduler::Micros,
        horizon_end_us: ecu_scheduler::Micros,
        heartbeat_deadline_us: ecu_scheduler::Micros,
        permit_mask: TimingIslandPermitMask,
    ) -> bool {
        self.scheduler.commit_frontier_horizon(
            horizon_id,
            horizon_start_us,
            horizon_end_us,
            heartbeat_deadline_us,
            permit_mask,
        )
    }

    pub fn note_frontier_heartbeat(&mut self, now: ecu_scheduler::Micros) {
        self.scheduler.note_frontier_heartbeat(now);
    }

    pub fn expire_frontier(&mut self, now: ecu_scheduler::Micros) {
        self.scheduler.expire_frontier(now);
    }

    pub fn on_sync_loss(&mut self) {
        self.scheduler.on_sync_loss();
    }

    pub fn on_hard_safety_shutdown(&mut self) {
        self.scheduler.on_hard_safety_shutdown();
    }

    pub fn drain_due_with_frontier<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        self.scheduler.drain_due_with_frontier(now, out)
    }

    pub fn drain_due<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        self.scheduler.drain_due(now, out)
    }

    pub fn drain_and_apply_due<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
        injectors: &mut [&mut dyn RawScheduledOutputPin],
        ignition: &mut [&mut dyn RawScheduledOutputPin],
    ) -> Result<usize, TransitionApplyError> {
        let had_live_frontier = self.scheduler.frontier().active_horizon_id().is_some();
        if self
            .scheduler
            .frontier()
            .last_accepted_horizon_id()
            .is_some()
        {
            self.drain_due_with_frontier(now, out);
        } else {
            self.drain_due(now, out);
        }
        match apply_drained_transitions(out, injectors, ignition) {
            Ok(applied) => Ok(applied),
            Err(error) => {
                if had_live_frontier {
                    self.scheduler
                        .cancel_all_with_reason(TimingIslandStopReason::BoardOutputFault);
                }
                Err(error)
            }
        }
    }
}

fn map_action_lowering_error(error: ActionLoweringError) -> ScheduleError {
    match error {
        ActionLoweringError::OutputBatchFull
        | ActionLoweringError::AuxBatchFull
        | ActionLoweringError::TimingIslandBatchFull => ScheduleError::QueueFull,
    }
}

fn map_cancel_reason(reason: ecu_domain::CancelReason) -> TimingIslandStopReason {
    match reason {
        ecu_domain::CancelReason::Manual => TimingIslandStopReason::PermitDenied,
        ecu_domain::CancelReason::SyncLoss => TimingIslandStopReason::SyncLost,
        ecu_domain::CancelReason::SafetyShutdown => TimingIslandStopReason::TimingFault,
        ecu_domain::CancelReason::Commit => TimingIslandStopReason::HorizonExpired,
        ecu_domain::CancelReason::Timeout => TimingIslandStopReason::HeartbeatExpired,
    }
}

fn scheduled_level(level: BoardOutputLevel) -> ScheduledLevel {
    match level {
        BoardOutputLevel::Low => ScheduledLevel::Low,
        BoardOutputLevel::High => ScheduledLevel::High,
    }
}

fn scheduled_transition(transition: OutputTransition) -> ScheduledTransition {
    let output = scheduled_output(transition.output);
    let kind = match output.group() {
        ecu_scheduler::OutputGroup::Injector => ScheduledTransitionKind::Injector,
        ecu_scheduler::OutputGroup::Ignition => ScheduledTransitionKind::Ignition,
        ecu_scheduler::OutputGroup::Idle | ecu_scheduler::OutputGroup::Fan => unreachable!(),
    };

    ScheduledTransition {
        at_us: ecu_scheduler::Micros::new(transition.at.get()),
        kind,
        channel: output.channel(),
        level: scheduled_level(transition.level),
    }
}

fn scheduled_output(output: EcuOutput) -> ExclusiveChannel {
    match output {
        EcuOutput::Injector(channel) => {
            ExclusiveChannel::new(ecu_scheduler::OutputGroup::Injector, channel)
        }
        EcuOutput::Ignition(channel) => {
            ExclusiveChannel::new(ecu_scheduler::OutputGroup::Ignition, channel)
        }
    }
}

fn execute_action_on_scheduler<const N: usize>(
    scheduler: &mut ScheduledQueueAdapter<N>,
    action: Action,
) -> Result<(), ScheduleError> {
    let mut lowered =
        ActionOutputBatchAdapter::<ACTION_OUTPUT_SCRATCH_CAP, RUNTIME_AUX_COMMAND_CAP>::new();
    lowered.execute(action).map_err(map_action_lowering_error)?;
    scheduler.apply_lowered(&lowered)
}

impl<const N: usize> Default for ScheduledActionExecutor<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ActionExecutor for ScheduledActionExecutor<N> {
    type Error = ScheduleError;

    fn execute(&mut self, action: Action) -> Result<(), Self::Error> {
        execute_action_on_scheduler(&mut self.scheduler, action)
    }

    fn execute_batch<const M: usize>(&mut self, batch: ActionBatch<M>) -> Result<(), Self::Error> {
        let mut scheduler = self.scheduler;
        for action in batch.iter() {
            execute_action_on_scheduler(&mut scheduler, action)?;
        }
        self.scheduler = scheduler;
        Ok(())
    }
}
