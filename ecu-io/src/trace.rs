//! Trace record types for recording and replaying ECU IO.
//!
//! Provides structured records for edge, sensor, tick, and output events
//! that can be stored and replayed deterministically.

use crate::{EdgeSample, OutputTransition, SensorFrame};
use ecu_domain::{Degrees10, FaultCode, Micros, Rpm};

/// Classification of input that generated a trace record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceInputKind {
    None,
    Edge,
    Sensor,
    Tick,
}

/// Payload of a trace record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TracePayload {
    None,
    Edge(EdgeSample),
    Sensor(SensorFrame),
    Output(OutputTransition),
}

/// A complete trace record for IO replay.
///
/// All fields use integer units. Diagnostic codes use the domain FaultCode;
/// when no fault is present, FaultCode::None is used.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TraceRecord {
    /// Simulation/test step index.
    pub step_index: u32,
    /// Timestamp in microseconds.
    pub time_us: Micros,
    /// Kind of input that generated this record.
    pub input_kind: TraceInputKind,
    /// Payload data for this record.
    pub payload: TracePayload,
    /// RPM at time of record.
    pub rpm: Rpm,
    /// Whether the decoder was synced at this time.
    pub synced: bool,
    /// Current tooth count (trigger decoder state).
    pub tooth: u16,
    /// Crank angle at this time in degrees * 10.
    pub angle_x10: Degrees10,
    /// Diagnostic code at this time.
    pub diagnostic_code: FaultCode,
}
