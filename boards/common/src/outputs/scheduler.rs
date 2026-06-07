use ecu_board_api::{
    EcuOutput, OutputLevel as BoardOutputLevel, OutputScheduler, OutputTransition,
    OutputTransitionBatch,
};
use ecu_runtime::{
    Action, ActionBatch, ActionExecutor, ActionLoweringError, ActionOutputBatchAdapter,
    RUNTIME_AUX_COMMAND_CAP,
};
use ecu_scheduler::{
    ScheduleError, ScheduledLevel, ScheduledTransition, ScheduledTransitionKind,
    ScheduledTransitionQueue, TransitionDrainBuffer,
};

use super::pins::{apply_drained_transitions, RawScheduledOutputPin, TransitionApplyError};

pub const ACTION_OUTPUT_SCRATCH_CAP: usize = 4;
/// Adapter that queues logical board output batches on the split scheduler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledQueueAdapter<const N: usize> {
    queue: ScheduledTransitionQueue<N>,
}

impl<const N: usize> ScheduledQueueAdapter<N> {
    pub const fn new() -> Self {
        Self {
            queue: ScheduledTransitionQueue::new(),
        }
    }

    pub const fn queue(&self) -> &ScheduledTransitionQueue<N> {
        &self.queue
    }

    pub fn queue_mut(&mut self) -> &mut ScheduledTransitionQueue<N> {
        &mut self.queue
    }

    pub fn into_inner(self) -> ScheduledTransitionQueue<N> {
        self.queue
    }

    pub fn cancel_all(&mut self) {
        self.queue.cancel_all();
    }

    pub fn drain_due<const M: usize>(
        &mut self,
        now: ecu_scheduler::Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        self.queue.drain_due(now, out)
    }

    pub fn schedule_output_batch<const M: usize>(
        &mut self,
        batch: &OutputTransitionBatch<M>,
    ) -> Result<(), ScheduleError> {
        if self.queue.free_slots() < batch.len() {
            return Err(ScheduleError::QueueFull);
        }

        for transition in batch.iter() {
            self.queue
                .enqueue_transition(scheduled_transition(*transition))?;
        }
        Ok(())
    }

    pub fn apply_lowered<const OUT: usize, const AUX: usize>(
        &mut self,
        lowered: &ActionOutputBatchAdapter<OUT, AUX>,
    ) -> Result<(), ScheduleError> {
        let mut scratch = *self;
        if lowered.status().cancel_scheduled_outputs() {
            scratch.cancel_all();
        }
        scratch.schedule_output_batch(lowered.output_transitions())?;
        *self = scratch;
        Ok(())
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

    pub fn queue_mut(&mut self) -> &mut ScheduledTransitionQueue<N> {
        self.scheduler.queue_mut()
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
        self.drain_due(now, out);
        apply_drained_transitions(out, injectors, ignition)
    }
}

fn map_action_lowering_error(error: ActionLoweringError) -> ScheduleError {
    match error {
        ActionLoweringError::OutputBatchFull
        | ActionLoweringError::AuxBatchFull
        | ActionLoweringError::TimingIslandBatchFull => ScheduleError::QueueFull,
    }
}

fn scheduled_level(level: BoardOutputLevel) -> ScheduledLevel {
    match level {
        BoardOutputLevel::Low => ScheduledLevel::Low,
        BoardOutputLevel::High => ScheduledLevel::High,
    }
}

fn scheduled_transition(transition: OutputTransition) -> ScheduledTransition {
    let (kind, channel) = match transition.output {
        EcuOutput::Injector(channel) => (ScheduledTransitionKind::Injector, channel),
        EcuOutput::Ignition(channel) => (ScheduledTransitionKind::Ignition, channel),
    };

    ScheduledTransition {
        at_us: ecu_scheduler::Micros::new(transition.at.get()),
        kind,
        channel,
        level: scheduled_level(transition.level),
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
