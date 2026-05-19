//! Output transition types and output transition sink trait.
//!
//! Provides fixed-size, simulator-independent IO contracts for output pin events.

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

/// Sink for output pin transitions (e.g., logic analyzer capture).
pub trait OutputTransitionSink {
    type Error;

    /// Record a single output transition.
    fn push_transition(&mut self, transition: OutputTransition) -> Result<(), Self::Error>;
}
