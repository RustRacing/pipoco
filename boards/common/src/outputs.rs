mod pins;
mod scheduler;

pub use pins::{
    apply_drained_transitions, apply_transition, Hal1ScheduledOut, HalOut, Outputs4,
    RawScheduledOutputBank, RawScheduledOutputPin, ScheduledOutputPinError, ScheduledOutputs,
    ScheduledOutputs4, TransitionApplyError,
};
pub use scheduler::{ScheduledActionExecutor, ScheduledQueueAdapter, ACTION_OUTPUT_SCRATCH_CAP};

/// Adapter that queues logical board output batches on the split scheduler.
pub type ScheduledTransitionQueueAdapter<const N: usize> = scheduler::ScheduledQueueAdapter<N>;

#[cfg(test)]
mod tests;
