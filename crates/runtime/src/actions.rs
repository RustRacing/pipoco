use ecu_board_api::AuxCommandBatch;
use ecu_calibration::PersistedCalibrationBlob;
use ecu_domain::{CancelReason, ChannelId, Micros};
use ecu_scheduler::{TimedIgnitionPlan, TimedInjectionPlan};

use crate::{RuntimeSnapshot, RUNTIME_AUX_COMMAND_CAP};

/// Runtime-facing output kind exported from a scheduled action.
///
/// This is intentionally smaller than the scheduler queue API. Edge crates that
/// only need to observe runtime output events should not depend on scheduler
/// internals.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeScheduledOutputKind {
    Injector,
    Ignition,
}

/// Logic level exported from a scheduled runtime action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeScheduledLevel {
    Low,
    High,
}

/// Scheduler-neutral output transition exported from a runtime action.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeScheduledTransition {
    pub at_us: Micros,
    pub kind: RuntimeScheduledOutputKind,
    pub channel: ChannelId,
    pub level: RuntimeScheduledLevel,
}

/// Fixed-size export buffer for runtime output transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeScheduledTransitionBatch<const N: usize> {
    len: usize,
    transitions: [Option<RuntimeScheduledTransition>; N],
}

impl<const N: usize> RuntimeScheduledTransitionBatch<N> {
    pub const fn new() -> Self {
        Self {
            len: 0,
            transitions: [None; N],
        }
    }

    pub const fn len(self) -> usize {
        self.len
    }

    pub const fn is_empty(self) -> bool {
        self.len == 0
    }

    pub fn iter(self) -> impl Iterator<Item = RuntimeScheduledTransition> {
        self.transitions.into_iter().flatten()
    }

    fn push(&mut self, transition: RuntimeScheduledTransition) -> Result<(), ActionExportError> {
        if self.len == N {
            return Err(ActionExportError::TransitionBatchFull);
        }
        self.transitions[self.len] = Some(transition);
        self.len += 1;
        Ok(())
    }
}

impl<const N: usize> Default for RuntimeScheduledTransitionBatch<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Error returned when a runtime action cannot be exported into the requested
/// fixed-size transition batch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionExportError {
    TransitionBatchFull,
    SchedulerExport,
}

/// Board-facing action emitted by runtime decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    ArmScheduler {
        injection: TimedInjectionPlan,
        ignition: TimedIgnitionPlan,
    },
    ArmInjection(TimedInjectionPlan),
    ArmIgnition(TimedIgnitionPlan),
    CancelScheduler(CancelReason),
    PublishSnapshot,
    PersistCalibration,
    ApplyAux(AuxCommandBatch<RUNTIME_AUX_COMMAND_CAP>),
    Idle,
}

impl Action {
    pub fn export_scheduled_transitions<const N: usize>(
        self,
    ) -> Result<RuntimeScheduledTransitionBatch<N>, ActionExportError> {
        let mut batch = RuntimeScheduledTransitionBatch::new();
        match self {
            Self::ArmScheduler {
                injection,
                ignition,
            } => {
                push_injection_export(&mut batch, injection)?;
                push_ignition_export(&mut batch, ignition)?;
            }
            Self::ArmInjection(injection) => {
                push_injection_export(&mut batch, injection)?;
            }
            Self::ArmIgnition(ignition) => {
                push_ignition_export(&mut batch, ignition)?;
            }
            Self::CancelScheduler(_)
            | Self::PublishSnapshot
            | Self::PersistCalibration
            | Self::ApplyAux(_)
            | Self::Idle => {}
        }
        Ok(batch)
    }
}

fn push_injection_export<const N: usize>(
    batch: &mut RuntimeScheduledTransitionBatch<N>,
    injection: TimedInjectionPlan,
) -> Result<(), ActionExportError> {
    let export = injection
        .export_transitions::<2>()
        .map_err(|_| ActionExportError::SchedulerExport)?;
    for transition in export.transitions.into_iter().flatten() {
        batch.push(RuntimeScheduledTransition {
            at_us: transition.at_us,
            kind: RuntimeScheduledOutputKind::Injector,
            channel: transition.channel,
            level: match transition.level {
                ecu_scheduler::ScheduledLevel::Low => RuntimeScheduledLevel::Low,
                ecu_scheduler::ScheduledLevel::High => RuntimeScheduledLevel::High,
            },
        })?;
    }
    Ok(())
}

fn push_ignition_export<const N: usize>(
    batch: &mut RuntimeScheduledTransitionBatch<N>,
    ignition: TimedIgnitionPlan,
) -> Result<(), ActionExportError> {
    let export = ignition
        .export_transitions::<2>()
        .map_err(|_| ActionExportError::SchedulerExport)?;
    for transition in export.transitions.into_iter().flatten() {
        batch.push(RuntimeScheduledTransition {
            at_us: transition.at_us,
            kind: RuntimeScheduledOutputKind::Ignition,
            channel: transition.channel,
            level: match transition.level {
                ecu_scheduler::ScheduledLevel::Low => RuntimeScheduledLevel::Low,
                ecu_scheduler::ScheduledLevel::High => RuntimeScheduledLevel::High,
            },
        })?;
    }
    Ok(())
}

/// Fixed-size action batch emitted by a runtime step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActionBatch<const N: usize> {
    actions: [Option<Action>; N],
    len: usize,
}

impl<const N: usize> ActionBatch<N> {
    pub const fn new() -> Self {
        Self {
            actions: [None; N],
            len: 0,
        }
    }

    pub fn push(&mut self, action: Action) -> bool {
        if self.len == N {
            return false;
        }
        self.actions[self.len] = Some(action);
        self.len += 1;
        true
    }

    pub fn len(self) -> usize {
        self.len
    }

    pub fn is_empty(self) -> bool {
        self.len == 0
    }

    pub fn iter(self) -> impl Iterator<Item = Action> {
        self.actions.into_iter().flatten()
    }
}

impl<const N: usize> Default for ActionBatch<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Executor for runtime-emitted actions.
pub trait ActionExecutor {
    type Error;

    fn execute(&mut self, action: Action) -> Result<(), Self::Error>;

    fn execute_batch<const N: usize>(&mut self, batch: ActionBatch<N>) -> Result<(), Self::Error> {
        for action in batch.iter() {
            self.execute(action)?;
        }
        Ok(())
    }
}

/// Transport-facing publisher for runtime snapshots and persistence events.
pub trait TransportPublisher {
    type Error;

    fn publish_snapshot(&mut self, snapshot: &RuntimeSnapshot) -> Result<(), Self::Error>;

    fn publish_calibration(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error>;
}
