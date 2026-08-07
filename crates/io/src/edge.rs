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

/// An edge sample paired with a trace identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TracedEdgeSample {
    pub trace_id: crate::TraceId,
    pub sample: EdgeSample,
}

impl EdgeSample {
    pub const fn with_trace_id(self, trace_id: crate::TraceId) -> TracedEdgeSample {
        TracedEdgeSample {
            trace_id,
            sample: self,
        }
    }
}

/// Source of edge samples (e.g., input capture hardware).
pub trait EdgeSource {
    type Error;

    /// Get the next edge sample, if available.
    fn next_edge(&mut self) -> Result<Option<EdgeSample>, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn with_trace_id_preserves_edge_sample_and_trace_id() {
        let sample = EdgeSample {
            at_us: Micros::new(123),
            line: EdgeLine::Cam,
            polarity: EdgePolarity::Falling,
            angle_x10: Degrees10::new(456),
            rpm: Rpm::new(789),
        };
        let trace_id = crate::TraceId::new(0xfeed_beef);

        let traced = sample.with_trace_id(trace_id);

        assert_eq!(traced.trace_id, trace_id);
        assert_eq!(traced.sample, sample);
    }
}
