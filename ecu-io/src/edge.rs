//! Edge sample types and edge source trait.
//!
//! Provides fixed-size, simulator-independent IO contracts for crank/cam edge data.

use ecu_domain::{Degrees10, Micros, Rpm};

/// Physical line that generated an edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgeLine {
    Crank,
    Cam,
}

/// Polarity of an edge transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EdgePolarity {
    Rising,
    Falling,
}

/// A single edge sample from a trigger sensor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EdgeSample {
    /// Timestamp in microseconds.
    pub at_us: Micros,
    /// Which line this edge came from.
    pub line: EdgeLine,
    /// Polarity of the edge transition.
    pub polarity: EdgePolarity,
    /// Crank angle at this edge in degrees * 10.
    pub angle_x10: Degrees10,
    /// Current RPM at this edge.
    pub rpm: Rpm,
}

/// Source of edge samples (e.g., input capture hardware).
pub trait EdgeSource {
    type Error;

    /// Get the next edge sample, if available.
    fn next_edge(&mut self) -> Result<Option<EdgeSample>, Self::Error>;
}
