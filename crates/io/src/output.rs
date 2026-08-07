//! Output transition types and output transition sink trait.
//!
//! Provides fixed-size, simulator-independent IO contracts for output pin events.

use crate::{StageOutcome, TraceId};
use ecu_domain::{ChannelId, Micros};

/// Kind of output transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum OutputTransitionKind {
    Injector,
    Ignition,
    Idle,
    Fan,
}

/// Logic level of an output.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputLevel {
    Low,
    High,
}

/// A single output pin transition event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputTransition {
    /// Timestamp in microseconds.
    pub at_us: Micros,
    /// Kind of output.
    pub kind: OutputTransitionKind,
    /// Output channel identifier.
    pub channel: ChannelId,
    /// New logic level after this transition.
    pub level: OutputLevel,
}

/// Final trace for an output request and its terminal outcome.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputRequestFinalTrace {
    pub trace_id: TraceId,
    pub request: OutputTransition,
    pub final_transition: Option<OutputTransition>,
    pub final_outcome: StageOutcome,
    pub reason: u8,
}

impl OutputRequestFinalTrace {
    /// Construct a completed output request trace.
    pub const fn completed(
        trace_id: TraceId,
        request: OutputTransition,
        final_transition: OutputTransition,
    ) -> Self {
        Self {
            trace_id,
            request,
            final_transition: Some(final_transition),
            final_outcome: StageOutcome::Completed,
            reason: 0,
        }
    }

    /// Construct a rejected output request trace.
    pub const fn rejected(trace_id: TraceId, request: OutputTransition, reason: u8) -> Self {
        Self {
            trace_id,
            request,
            final_transition: None,
            final_outcome: StageOutcome::Rejected,
            reason,
        }
    }
}

/// Sink for output pin transitions (e.g., logic analyzer capture).
pub trait OutputTransitionSink {
    type Error;

    /// Record a single output transition.
    fn push_transition(&mut self, transition: OutputTransition) -> Result<(), Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn completed_trace_sets_completed_outcome_and_transition() {
        let trace_id = TraceId::new(0xfeed_beef);
        let request = OutputTransition {
            at_us: Micros::new(10),
            kind: OutputTransitionKind::Injector,
            channel: ChannelId::new(7),
            level: OutputLevel::High,
        };
        let final_transition = OutputTransition {
            at_us: Micros::new(20),
            kind: OutputTransitionKind::Injector,
            channel: ChannelId::new(7),
            level: OutputLevel::Low,
        };

        let trace = OutputRequestFinalTrace::completed(trace_id, request, final_transition);

        assert_eq!(trace.trace_id, trace_id);
        assert_eq!(trace.request, request);
        assert_eq!(trace.final_transition, Some(final_transition));
        assert_eq!(trace.final_outcome, StageOutcome::Completed);
        assert_eq!(trace.reason, 0);
    }

    #[test]
    fn rejected_trace_sets_rejected_outcome_without_transition() {
        let trace_id = TraceId::new(0x1234_5678);
        let request = OutputTransition {
            at_us: Micros::new(11),
            kind: OutputTransitionKind::Fan,
            channel: ChannelId::new(3),
            level: OutputLevel::Low,
        };

        let trace = OutputRequestFinalTrace::rejected(trace_id, request, 0x2a);

        assert_eq!(trace.trace_id, trace_id);
        assert_eq!(trace.request, request);
        assert_eq!(trace.final_transition, None);
        assert_eq!(trace.final_outcome, StageOutcome::Rejected);
        assert_eq!(trace.reason, 0x2a);
    }
}
