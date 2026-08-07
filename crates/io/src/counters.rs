//! Shared named assembly counters for the raw IO layer.

use crate::{OutputStage, SignalStage, StageOutcome, TraceId};
use ecu_domain::Micros;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SignalStageSnapshot {
    pub seen: u32,
    pub accepted: u32,
    pub rejected: u32,
    pub dropped: u32,
    pub stale: u32,
    pub overrun: u32,
    pub last_trace_id: TraceId,
    pub last_sequence_or_command_id: u32,
    pub last_timestamp: Micros,
    pub last_reason: u8,
}

impl SignalStageSnapshot {
    pub const fn new() -> Self {
        Self {
            seen: 0,
            accepted: 0,
            rejected: 0,
            dropped: 0,
            stale: 0,
            overrun: 0,
            last_trace_id: TraceId::new(0),
            last_sequence_or_command_id: 0,
            last_timestamp: Micros::new(0),
            last_reason: 0,
        }
    }

    pub fn merge_from(&mut self, other: Self) {
        let previous_seen = self.seen;
        self.seen = self.seen.saturating_add(other.seen);
        self.accepted = self.accepted.saturating_add(other.accepted);
        self.rejected = self.rejected.saturating_add(other.rejected);
        self.dropped = self.dropped.saturating_add(other.dropped);
        self.stale = self.stale.saturating_add(other.stale);
        self.overrun = self.overrun.saturating_add(other.overrun);
        if other.seen > 0
            && (previous_seen == 0 || other.last_timestamp.get() >= self.last_timestamp.get())
        {
            self.last_trace_id = other.last_trace_id;
            self.last_sequence_or_command_id = other.last_sequence_or_command_id;
            self.last_timestamp = other.last_timestamp;
            self.last_reason = other.last_reason;
        }
    }

    pub fn record(
        &mut self,
        outcome: StageOutcome,
        trace_id: TraceId,
        sequence_id: u32,
        timestamp: Micros,
        reason: u8,
    ) {
        self.seen = self.seen.saturating_add(1);
        self.last_trace_id = trace_id;
        self.last_sequence_or_command_id = sequence_id;
        self.last_timestamp = timestamp;
        self.last_reason = reason;

        match outcome {
            StageOutcome::Seen => {}
            StageOutcome::Accepted
            | StageOutcome::Planned
            | StageOutcome::Admitted
            | StageOutcome::Armed
            | StageOutcome::Executed
            | StageOutcome::Completed => {
                self.accepted = self.accepted.saturating_add(1);
            }
            StageOutcome::Rejected | StageOutcome::Cancelled | StageOutcome::BackendFault => {
                self.rejected = self.rejected.saturating_add(1);
            }
            StageOutcome::Dropped => {
                self.dropped = self.dropped.saturating_add(1);
            }
            StageOutcome::Stale => {
                self.stale = self.stale.saturating_add(1);
            }
            StageOutcome::Overrun | StageOutcome::Late => {
                self.overrun = self.overrun.saturating_add(1);
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct OutputStageSnapshot {
    pub seen: u32,
    pub accepted: u32,
    pub rejected: u32,
    pub dropped: u32,
    pub stale: u32,
    pub overrun: u32,
    pub late: u32,
    pub last_trace_id: TraceId,
    pub last_sequence_or_command_id: u32,
    pub last_timestamp: Micros,
    pub last_reason: u8,
}

impl OutputStageSnapshot {
    pub const fn new() -> Self {
        Self {
            seen: 0,
            accepted: 0,
            rejected: 0,
            dropped: 0,
            stale: 0,
            overrun: 0,
            late: 0,
            last_trace_id: TraceId::new(0),
            last_sequence_or_command_id: 0,
            last_timestamp: Micros::new(0),
            last_reason: 0,
        }
    }

    pub fn merge_from(&mut self, other: Self) {
        let previous_seen = self.seen;
        self.seen = self.seen.saturating_add(other.seen);
        self.accepted = self.accepted.saturating_add(other.accepted);
        self.rejected = self.rejected.saturating_add(other.rejected);
        self.dropped = self.dropped.saturating_add(other.dropped);
        self.stale = self.stale.saturating_add(other.stale);
        self.overrun = self.overrun.saturating_add(other.overrun);
        self.late = self.late.saturating_add(other.late);
        if other.seen > 0
            && (previous_seen == 0 || other.last_timestamp.get() >= self.last_timestamp.get())
        {
            self.last_trace_id = other.last_trace_id;
            self.last_sequence_or_command_id = other.last_sequence_or_command_id;
            self.last_timestamp = other.last_timestamp;
            self.last_reason = other.last_reason;
        }
    }

    pub fn record(
        &mut self,
        outcome: StageOutcome,
        trace_id: TraceId,
        command_id: u32,
        timestamp: Micros,
        reason: u8,
    ) {
        self.seen = self.seen.saturating_add(1);
        self.last_trace_id = trace_id;
        self.last_sequence_or_command_id = command_id;
        self.last_timestamp = timestamp;
        self.last_reason = reason;

        match outcome {
            StageOutcome::Seen => {}
            StageOutcome::Accepted
            | StageOutcome::Planned
            | StageOutcome::Admitted
            | StageOutcome::Armed
            | StageOutcome::Executed
            | StageOutcome::Completed => {
                self.accepted = self.accepted.saturating_add(1);
            }
            StageOutcome::Rejected | StageOutcome::Cancelled | StageOutcome::BackendFault => {
                self.rejected = self.rejected.saturating_add(1);
            }
            StageOutcome::Dropped => {
                self.dropped = self.dropped.saturating_add(1);
            }
            StageOutcome::Stale => {
                self.stale = self.stale.saturating_add(1);
            }
            StageOutcome::Overrun => {
                self.overrun = self.overrun.saturating_add(1);
            }
            StageOutcome::Late => {
                self.late = self.late.saturating_add(1);
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SignalAssemblyCounters {
    pub signal_capture: SignalStageSnapshot,
    pub signal_normalizer: SignalStageSnapshot,
    pub observation_validator: SignalStageSnapshot,
    pub observation_publisher: SignalStageSnapshot,
    pub observation_reader: SignalStageSnapshot,
    pub runtime_snapshot_builder: SignalStageSnapshot,
    pub policy_consumer: SignalStageSnapshot,
}

impl SignalAssemblyCounters {
    pub const fn new() -> Self {
        Self {
            signal_capture: SignalStageSnapshot::new(),
            signal_normalizer: SignalStageSnapshot::new(),
            observation_validator: SignalStageSnapshot::new(),
            observation_publisher: SignalStageSnapshot::new(),
            observation_reader: SignalStageSnapshot::new(),
            runtime_snapshot_builder: SignalStageSnapshot::new(),
            policy_consumer: SignalStageSnapshot::new(),
        }
    }

    pub fn merge_from(&mut self, other: Self) {
        self.signal_capture.merge_from(other.signal_capture);
        self.signal_normalizer.merge_from(other.signal_normalizer);
        self.observation_validator
            .merge_from(other.observation_validator);
        self.observation_publisher
            .merge_from(other.observation_publisher);
        self.observation_reader.merge_from(other.observation_reader);
        self.runtime_snapshot_builder
            .merge_from(other.runtime_snapshot_builder);
        self.policy_consumer.merge_from(other.policy_consumer);
    }

    pub fn record(
        &mut self,
        stage: SignalStage,
        outcome: StageOutcome,
        trace_id: TraceId,
        sequence_id: u32,
        timestamp: Micros,
        reason: u8,
    ) {
        self.stage_mut(stage)
            .record(outcome, trace_id, sequence_id, timestamp, reason);
    }

    fn stage_mut(&mut self, stage: SignalStage) -> &mut SignalStageSnapshot {
        match stage {
            SignalStage::SignalCapture => &mut self.signal_capture,
            SignalStage::SignalNormalizer => &mut self.signal_normalizer,
            SignalStage::ObservationValidator => &mut self.observation_validator,
            SignalStage::ObservationPublisher => &mut self.observation_publisher,
            SignalStage::ObservationReader => &mut self.observation_reader,
            SignalStage::RuntimeSnapshotBuilder => &mut self.runtime_snapshot_builder,
            SignalStage::PolicyConsumer => &mut self.policy_consumer,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct OutputAssemblyCounters {
    pub output_intent: OutputStageSnapshot,
    pub output_planner: OutputStageSnapshot,
    pub output_admission: OutputStageSnapshot,
    pub output_armer: OutputStageSnapshot,
    pub output_executor: OutputStageSnapshot,
    pub output_observer: OutputStageSnapshot,
}

impl OutputAssemblyCounters {
    pub const fn new() -> Self {
        Self {
            output_intent: OutputStageSnapshot::new(),
            output_planner: OutputStageSnapshot::new(),
            output_admission: OutputStageSnapshot::new(),
            output_armer: OutputStageSnapshot::new(),
            output_executor: OutputStageSnapshot::new(),
            output_observer: OutputStageSnapshot::new(),
        }
    }

    pub fn merge_from(&mut self, other: Self) {
        self.output_intent.merge_from(other.output_intent);
        self.output_planner.merge_from(other.output_planner);
        self.output_admission.merge_from(other.output_admission);
        self.output_armer.merge_from(other.output_armer);
        self.output_executor.merge_from(other.output_executor);
        self.output_observer.merge_from(other.output_observer);
    }

    pub fn record(
        &mut self,
        stage: OutputStage,
        outcome: StageOutcome,
        trace_id: TraceId,
        command_id: u32,
        timestamp: Micros,
        reason: u8,
    ) {
        self.stage_mut(stage)
            .record(outcome, trace_id, command_id, timestamp, reason);
    }

    fn stage_mut(&mut self, stage: OutputStage) -> &mut OutputStageSnapshot {
        match stage {
            OutputStage::OutputIntent => &mut self.output_intent,
            OutputStage::OutputPlanner => &mut self.output_planner,
            OutputStage::OutputAdmission => &mut self.output_admission,
            OutputStage::OutputArmer => &mut self.output_armer,
            OutputStage::OutputExecutor => &mut self.output_executor,
            OutputStage::OutputObserver => &mut self.output_observer,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signal_stage_snapshot_records_metadata_and_known_buckets() {
        let mut snapshot = SignalStageSnapshot::default();
        let trace_id = TraceId::new(0xfeed_beef);
        let timestamp = Micros::new(101);

        snapshot.record(StageOutcome::Accepted, trace_id, 33, timestamp, 7);
        snapshot.record(StageOutcome::Rejected, trace_id, 34, timestamp, 8);
        snapshot.record(StageOutcome::Dropped, trace_id, 35, timestamp, 9);
        snapshot.record(StageOutcome::Stale, trace_id, 36, timestamp, 10);
        snapshot.record(StageOutcome::Overrun, trace_id, 37, timestamp, 11);

        assert_eq!(snapshot.seen, 5);
        assert_eq!(snapshot.accepted, 1);
        assert_eq!(snapshot.rejected, 1);
        assert_eq!(snapshot.dropped, 1);
        assert_eq!(snapshot.stale, 1);
        assert_eq!(snapshot.overrun, 1);
        assert_eq!(snapshot.last_trace_id, trace_id);
        assert_eq!(snapshot.last_sequence_or_command_id, 37);
        assert_eq!(snapshot.last_timestamp, timestamp);
        assert_eq!(snapshot.last_reason, 11);
    }

    #[test]
    fn output_stage_snapshot_records_late_outcomes_separately() {
        let mut snapshot = OutputStageSnapshot::default();
        let trace_id = TraceId::new(0x1234_5678);
        let timestamp = Micros::new(202);

        snapshot.record(StageOutcome::Planned, trace_id, 44, timestamp, 1);
        snapshot.record(StageOutcome::Late, trace_id, 45, timestamp, 2);

        assert_eq!(snapshot.seen, 2);
        assert_eq!(snapshot.accepted, 1);
        assert_eq!(snapshot.late, 1);
        assert_eq!(snapshot.rejected, 0);
        assert_eq!(snapshot.dropped, 0);
        assert_eq!(snapshot.stale, 0);
        assert_eq!(snapshot.overrun, 0);
        assert_eq!(snapshot.last_trace_id, trace_id);
        assert_eq!(snapshot.last_sequence_or_command_id, 45);
        assert_eq!(snapshot.last_timestamp, timestamp);
        assert_eq!(snapshot.last_reason, 2);
    }

    #[test]
    fn assembly_counters_route_records_by_stage() {
        let mut signal = SignalAssemblyCounters::default();
        let mut output = OutputAssemblyCounters::default();
        let trace_id = TraceId::new(0xaaaa_bbbb);
        let timestamp = Micros::new(303);

        signal.record(
            SignalStage::ObservationPublisher,
            StageOutcome::Accepted,
            trace_id,
            77,
            timestamp,
            12,
        );
        output.record(
            OutputStage::OutputAdmission,
            StageOutcome::Late,
            trace_id,
            88,
            timestamp,
            13,
        );

        assert_eq!(signal.observation_publisher.seen, 1);
        assert_eq!(signal.observation_publisher.accepted, 1);
        assert_eq!(signal.signal_capture.seen, 0);
        assert_eq!(output.output_admission.seen, 1);
        assert_eq!(output.output_admission.late, 1);
        assert_eq!(output.output_admission.last_sequence_or_command_id, 88);
        assert_eq!(output.output_intent.seen, 0);
    }
}
