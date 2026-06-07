use ecu_board_api::{
    AuxCommandBatch, EcuOutput, OutputLevel, OutputTransition, OutputTransitionBatch,
    TimingIslandCommand, TimingIslandCommandBatch,
};
use ecu_domain::{CancelReason, Micros, Ticks};

use crate::{Action, ActionBatch, ActionExecutor};

/// Capacity error while lowering runtime actions to board API batches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionLoweringError {
    OutputBatchFull,
    AuxBatchFull,
    TimingIslandBatchFull,
}

/// Non-output side effects observed while lowering runtime actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ActionLoweringStatus {
    pub scheduled_output_transitions: usize,
    pub applied_aux_commands: usize,
    pub cancel_scheduler: Option<CancelReason>,
    pub persist_calibration: bool,
    pub publish_snapshot: bool,
    pub idle_seen: bool,
}

impl ActionLoweringStatus {
    pub const fn cancel_scheduled_outputs(self) -> bool {
        self.cancel_scheduler.is_some()
    }

    fn merge(&mut self, other: Self) {
        self.scheduled_output_transitions += other.scheduled_output_transitions;
        self.applied_aux_commands += other.applied_aux_commands;
        if other.cancel_scheduler.is_some() {
            self.cancel_scheduler = other.cancel_scheduler;
        }
        self.persist_calibration |= other.persist_calibration;
        self.publish_snapshot |= other.publish_snapshot;
        self.idle_seen |= other.idle_seen;
    }
}

/// Lower one runtime action into logical board output and auxiliary batches.
pub fn lower_action_to_board_batches<const OUT: usize, const AUX: usize>(
    action: Action,
    outputs: &mut OutputTransitionBatch<OUT>,
    aux: &mut AuxCommandBatch<AUX>,
) -> Result<ActionLoweringStatus, ActionLoweringError> {
    let mut status = ActionLoweringStatus::default();

    match action {
        Action::ArmScheduler {
            injection,
            ignition,
        } => {
            ensure_output_capacity(outputs, 4)?;
            push_output_pair(
                outputs,
                EcuOutput::Injector(injection.plan.output.channel()),
                injection.start_at,
                injection.end_at,
            )?;
            push_output_pair(
                outputs,
                EcuOutput::Ignition(ignition.plan.output.channel()),
                ignition.start_at,
                ignition.end_at,
            )?;
            status.scheduled_output_transitions += 4;
        }
        Action::ArmInjection(injection) => {
            ensure_output_capacity(outputs, 2)?;
            push_output_pair(
                outputs,
                EcuOutput::Injector(injection.plan.output.channel()),
                injection.start_at,
                injection.end_at,
            )?;
            status.scheduled_output_transitions += 2;
        }
        Action::ArmIgnition(ignition) => {
            ensure_output_capacity(outputs, 2)?;
            push_output_pair(
                outputs,
                EcuOutput::Ignition(ignition.plan.output.channel()),
                ignition.start_at,
                ignition.end_at,
            )?;
            status.scheduled_output_transitions += 2;
        }
        Action::CancelScheduler(reason) => {
            status.cancel_scheduler = Some(reason);
        }
        Action::ApplyAux(commands) => {
            ensure_aux_capacity(aux, commands.len())?;
            for command in commands.iter() {
                aux.push(*command)
                    .map_err(|_| ActionLoweringError::AuxBatchFull)?;
                status.applied_aux_commands += 1;
            }
        }
        Action::PersistCalibration => {
            status.persist_calibration = true;
        }
        Action::PublishSnapshot => {
            status.publish_snapshot = true;
        }
        Action::Idle => {
            status.idle_seen = true;
        }
    }

    Ok(status)
}

fn ensure_output_capacity<const N: usize>(
    batch: &OutputTransitionBatch<N>,
    needed: usize,
) -> Result<(), ActionLoweringError> {
    if batch.capacity().saturating_sub(batch.len()) < needed {
        return Err(ActionLoweringError::OutputBatchFull);
    }

    Ok(())
}

fn ensure_aux_capacity<const N: usize>(
    batch: &AuxCommandBatch<N>,
    needed: usize,
) -> Result<(), ActionLoweringError> {
    if batch.capacity().saturating_sub(batch.len()) < needed {
        return Err(ActionLoweringError::AuxBatchFull);
    }

    Ok(())
}

/// Lower a fixed runtime action batch into logical board API batches.
pub fn lower_action_batch_to_board_batches<
    const ACTIONS: usize,
    const OUT: usize,
    const AUX: usize,
>(
    actions: ActionBatch<ACTIONS>,
    outputs: &mut OutputTransitionBatch<OUT>,
    aux: &mut AuxCommandBatch<AUX>,
) -> Result<ActionLoweringStatus, ActionLoweringError> {
    let mut output_transitions_needed = 0usize;
    let mut aux_commands_needed = 0usize;
    for action in actions.iter() {
        output_transitions_needed =
            output_transitions_needed.saturating_add(action_output_transition_count(action));
        aux_commands_needed = aux_commands_needed.saturating_add(action_aux_command_count(action));
    }

    ensure_output_capacity(outputs, output_transitions_needed)?;
    ensure_aux_capacity(aux, aux_commands_needed)?;

    let mut status = ActionLoweringStatus::default();
    for action in actions.iter() {
        status.merge(lower_action_to_board_batches(action, outputs, aux)?);
    }
    Ok(status)
}

/// Runtime action executor that lowers actions into logical `ecu-board-api`
/// batches without knowing about board-specific pin mapping or scheduler queues.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardApiBatchExecutor<const OUT: usize, const AUX: usize> {
    output_transitions: OutputTransitionBatch<OUT>,
    aux_commands: AuxCommandBatch<AUX>,
    status: ActionLoweringStatus,
}

/// Semantic action-to-board-output batch seam.
pub type ActionOutputBatchAdapter<const OUT: usize, const AUX: usize> =
    BoardApiBatchExecutor<OUT, AUX>;

impl<const OUT: usize, const AUX: usize> BoardApiBatchExecutor<OUT, AUX> {
    pub const fn new() -> Self {
        Self {
            output_transitions: OutputTransitionBatch::new(),
            aux_commands: AuxCommandBatch::new(),
            status: ActionLoweringStatus {
                scheduled_output_transitions: 0,
                applied_aux_commands: 0,
                cancel_scheduler: None,
                persist_calibration: false,
                publish_snapshot: false,
                idle_seen: false,
            },
        }
    }

    pub const fn output_transitions(&self) -> &OutputTransitionBatch<OUT> {
        &self.output_transitions
    }

    pub const fn aux_commands(&self) -> &AuxCommandBatch<AUX> {
        &self.aux_commands
    }

    pub const fn status(&self) -> ActionLoweringStatus {
        self.status
    }

    pub fn clear(&mut self) {
        self.output_transitions.clear();
        self.aux_commands.clear();
        self.status = ActionLoweringStatus::default();
    }
}

impl<const OUT: usize, const AUX: usize> Default for BoardApiBatchExecutor<OUT, AUX> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const OUT: usize, const AUX: usize> ActionExecutor for BoardApiBatchExecutor<OUT, AUX> {
    type Error = ActionLoweringError;

    fn execute(&mut self, action: Action) -> Result<(), Self::Error> {
        let mut scratch = *self;
        let status = lower_action_to_board_batches(
            action,
            &mut scratch.output_transitions,
            &mut scratch.aux_commands,
        )?;
        scratch.status.merge(status);
        *self = scratch;
        Ok(())
    }

    fn execute_batch<const N: usize>(&mut self, batch: ActionBatch<N>) -> Result<(), Self::Error> {
        let mut scratch = *self;
        let status = lower_action_batch_to_board_batches(
            batch,
            &mut scratch.output_transitions,
            &mut scratch.aux_commands,
        )?;
        scratch.status.merge(status);
        *self = scratch;
        Ok(())
    }
}

/// Lower one runtime action into optional timing-island commands.
#[allow(deprecated)]
pub fn lower_action_to_timing_island<const CMD: usize>(
    action: Action,
    commands: &mut TimingIslandCommandBatch<CMD>,
) -> Result<ActionLoweringStatus, ActionLoweringError> {
    let mut status = ActionLoweringStatus::default();

    match action {
        Action::ArmScheduler {
            injection,
            ignition,
        } => {
            ensure_timing_island_capacity(commands, 4)?;
            push_timing_island_output_pair(
                commands,
                EcuOutput::Injector(injection.plan.output.channel()),
                injection.start_at,
                injection.end_at,
            )?;
            push_timing_island_output_pair(
                commands,
                EcuOutput::Ignition(ignition.plan.output.channel()),
                ignition.start_at,
                ignition.end_at,
            )?;
            status.scheduled_output_transitions += 4;
        }
        Action::ArmInjection(injection) => {
            ensure_timing_island_capacity(commands, 2)?;
            push_timing_island_output_pair(
                commands,
                EcuOutput::Injector(injection.plan.output.channel()),
                injection.start_at,
                injection.end_at,
            )?;
            status.scheduled_output_transitions += 2;
        }
        Action::ArmIgnition(ignition) => {
            ensure_timing_island_capacity(commands, 2)?;
            push_timing_island_output_pair(
                commands,
                EcuOutput::Ignition(ignition.plan.output.channel()),
                ignition.start_at,
                ignition.end_at,
            )?;
            status.scheduled_output_transitions += 2;
        }
        Action::CancelScheduler(reason) => {
            ensure_timing_island_capacity(commands, 1)?;
            push_timing_island_command(commands, TimingIslandCommand::CancelAll(reason))?;
            status.cancel_scheduler = Some(reason);
        }
        Action::ApplyAux(aux_commands) => {
            ensure_timing_island_capacity(commands, aux_commands.len())?;
            for command in aux_commands.iter() {
                push_timing_island_command(commands, TimingIslandCommand::ApplyAux(*command))?;
                status.applied_aux_commands += 1;
            }
        }
        Action::PersistCalibration => {
            status.persist_calibration = true;
        }
        Action::PublishSnapshot => {
            status.publish_snapshot = true;
        }
        Action::Idle => {
            status.idle_seen = true;
        }
    }

    Ok(status)
}

/// Lower a fixed runtime action batch into optional timing-island commands.
pub fn lower_action_batch_to_timing_island<const ACTIONS: usize, const CMD: usize>(
    actions: ActionBatch<ACTIONS>,
    commands: &mut TimingIslandCommandBatch<CMD>,
) -> Result<ActionLoweringStatus, ActionLoweringError> {
    let mut commands_needed = 0usize;
    for action in actions.iter() {
        commands_needed =
            commands_needed.saturating_add(action_timing_island_command_count(action));
    }

    ensure_timing_island_capacity(commands, commands_needed)?;

    let mut status = ActionLoweringStatus::default();
    for action in actions.iter() {
        status.merge(lower_action_to_timing_island(action, commands)?);
    }
    Ok(status)
}

fn action_output_transition_count(action: Action) -> usize {
    match action {
        Action::ArmScheduler { .. } => 4,
        Action::ArmInjection(_) | Action::ArmIgnition(_) => 2,
        _ => 0,
    }
}

fn action_aux_command_count(action: Action) -> usize {
    match action {
        Action::ApplyAux(commands) => commands.len(),
        _ => 0,
    }
}

fn action_timing_island_command_count(action: Action) -> usize {
    match action {
        Action::ArmScheduler { .. } => 4,
        Action::ArmInjection(_) | Action::ArmIgnition(_) => 2,
        Action::CancelScheduler(_) => 1,
        Action::ApplyAux(commands) => commands.len(),
        _ => 0,
    }
}

fn push_output_pair<const N: usize>(
    batch: &mut OutputTransitionBatch<N>,
    output: EcuOutput,
    start_at: Micros,
    end_at: Micros,
) -> Result<(), ActionLoweringError> {
    batch
        .push(OutputTransition::new(
            output,
            OutputLevel::High,
            Ticks::new(start_at.get()),
        ))
        .map_err(|_| ActionLoweringError::OutputBatchFull)?;
    batch
        .push(OutputTransition::new(
            output,
            OutputLevel::Low,
            Ticks::new(end_at.get()),
        ))
        .map_err(|_| ActionLoweringError::OutputBatchFull)?;
    Ok(())
}

fn ensure_timing_island_capacity<const N: usize>(
    batch: &TimingIslandCommandBatch<N>,
    needed: usize,
) -> Result<(), ActionLoweringError> {
    if batch.capacity().saturating_sub(batch.len()) < needed {
        return Err(ActionLoweringError::TimingIslandBatchFull);
    }

    Ok(())
}

fn push_timing_island_output_pair<const N: usize>(
    batch: &mut TimingIslandCommandBatch<N>,
    output: EcuOutput,
    start_at: Micros,
    end_at: Micros,
) -> Result<(), ActionLoweringError> {
    push_timing_island_command(
        batch,
        TimingIslandCommand::ArmOutput(OutputTransition::new(
            output,
            OutputLevel::High,
            Ticks::new(start_at.get()),
        )),
    )?;
    push_timing_island_command(
        batch,
        TimingIslandCommand::ArmOutput(OutputTransition::new(
            output,
            OutputLevel::Low,
            Ticks::new(end_at.get()),
        )),
    )?;
    Ok(())
}

fn push_timing_island_command<const N: usize>(
    batch: &mut TimingIslandCommandBatch<N>,
    command: TimingIslandCommand,
) -> Result<(), ActionLoweringError> {
    batch
        .push(command)
        .map_err(|_| ActionLoweringError::TimingIslandBatchFull)
}
