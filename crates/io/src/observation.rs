//! Observation header and trace metadata for the signal frontier.
//!
//! This module intentionally stays independent from `debug.rs` so it can
//! define the shared observation contract before the debug vocabulary is wired
//! into the crate root.

/// Identifier for a trace correlation thread.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct TraceId(u32);

impl TraceId {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Identifier for an observation source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct SourceId(u16);

impl SourceId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Per-source monotonic observation sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ObservationSequence(u32);

impl ObservationSequence {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Stable schema identifier for an observation payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ObservationSchemaId(u16);

impl ObservationSchemaId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Version of a schema identified by [`ObservationSchemaId`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ObservationSchemaVersion(u16);

impl ObservationSchemaVersion {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Timestamp domain carried with each observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(u8)]
pub enum TimestampDomain {
    #[default]
    BoardMicros = 0,
    EngineTicks = 1,
    EngineAngle = 2,
    SimulatorStep = 3,
}

/// Quality bits attached to an observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ObservationQuality(u16);

impl ObservationQuality {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Latched fault bits attached to an observation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[repr(transparent)]
pub struct ObservationFaultFlags(u32);

impl ObservationFaultFlags {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Header that prefixes each published observation payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct ObservationHeader {
    pub schema_id: ObservationSchemaId,
    pub schema_version: ObservationSchemaVersion,
    pub source_id: SourceId,
    pub sequence: ObservationSequence,
    pub trace_id: TraceId,
    pub timestamp_domain: TimestampDomain,
    pub timestamp: u32,
    pub engine_angle_valid: bool,
    pub engine_angle_deg10: u16,
    pub sample_window_id: u32,
    pub quality: ObservationQuality,
    pub fault_flags: ObservationFaultFlags,
    pub payload_len: u16,
}

/// Minimal per-stage record for the observation pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct ObservationStageRecord {
    pub trace_id: TraceId,
    pub source_id: SourceId,
    pub sequence: ObservationSequence,
    pub timestamp_domain: TimestampDomain,
    pub timestamp: u32,
    pub stage: u8,
    pub outcome: u8,
}

/// Final state for an input trace.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct InputFinalStateTrace {
    pub trace_id: TraceId,
    pub source_id: SourceId,
    pub sequence: ObservationSequence,
    pub timestamp_domain: TimestampDomain,
    pub timestamp: u32,
    pub final_state: u8,
    pub fault_reason: u8,
    pub stale: bool,
}

impl InputFinalStateTrace {
    pub const fn accepted(
        trace_id: TraceId,
        source_id: SourceId,
        sequence: ObservationSequence,
        timestamp_domain: TimestampDomain,
        timestamp: u32,
        final_state: u8,
    ) -> Self {
        Self {
            trace_id,
            source_id,
            sequence,
            timestamp_domain,
            timestamp,
            final_state,
            fault_reason: 0,
            stale: false,
        }
    }

    pub const fn stale(
        trace_id: TraceId,
        source_id: SourceId,
        sequence: ObservationSequence,
        timestamp_domain: TimestampDomain,
        timestamp: u32,
        final_state: u8,
        fault_reason: u8,
    ) -> Self {
        Self {
            trace_id,
            source_id,
            sequence,
            timestamp_domain,
            timestamp,
            final_state,
            fault_reason,
            stale: true,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn newtypes_round_trip_values() {
        let trace_id = TraceId::new(0x1234_5678);
        let source_id = SourceId::new(0x9abc);
        let sequence = ObservationSequence::new(0xdead_beef);
        let schema_id = ObservationSchemaId::new(0x1357);
        let schema_version = ObservationSchemaVersion::new(0x2468);
        let quality = ObservationQuality::new(0x55aa);
        let fault_flags = ObservationFaultFlags::new(0xfeed_cafe);

        assert_eq!(trace_id.get(), 0x1234_5678);
        assert_eq!(source_id.get(), 0x9abc);
        assert_eq!(sequence.get(), 0xdead_beef);
        assert_eq!(schema_id.get(), 0x1357);
        assert_eq!(schema_version.get(), 0x2468);
        assert_eq!(quality.get(), 0x55aa);
        assert_eq!(fault_flags.get(), 0xfeed_cafe);
    }

    #[test]
    fn timestamp_domain_discriminants_stay_stable() {
        assert_eq!(TimestampDomain::BoardMicros as u8, 0);
        assert_eq!(TimestampDomain::EngineTicks as u8, 1);
        assert_eq!(TimestampDomain::EngineAngle as u8, 2);
        assert_eq!(TimestampDomain::SimulatorStep as u8, 3);
        assert_eq!(TimestampDomain::default(), TimestampDomain::BoardMicros);
    }

    #[test]
    fn header_default_is_zeroed_and_harmless() {
        let header = ObservationHeader::default();

        assert_eq!(header.schema_id.get(), 0);
        assert_eq!(header.schema_version.get(), 0);
        assert_eq!(header.source_id.get(), 0);
        assert_eq!(header.sequence.get(), 0);
        assert_eq!(header.trace_id.get(), 0);
        assert_eq!(header.timestamp_domain, TimestampDomain::BoardMicros);
        assert_eq!(header.timestamp, 0);
        assert!(!header.engine_angle_valid);
        assert_eq!(header.engine_angle_deg10, 0);
        assert_eq!(header.sample_window_id, 0);
        assert_eq!(header.quality.get(), 0);
        assert_eq!(header.fault_flags.get(), 0);
        assert_eq!(header.payload_len, 0);
    }

    #[test]
    fn input_final_state_trace_accepted_constructor_sets_expected_fields() {
        let trace_id = TraceId::new(0x1234_5678);
        let source_id = SourceId::new(0x9abc);
        let sequence = ObservationSequence::new(0xdead_beef);
        let trace = InputFinalStateTrace::accepted(
            trace_id,
            source_id,
            sequence,
            TimestampDomain::EngineTicks,
            0xfeed_cafe,
            0x42,
        );

        assert_eq!(trace.trace_id, trace_id);
        assert_eq!(trace.source_id, source_id);
        assert_eq!(trace.sequence, sequence);
        assert_eq!(trace.timestamp_domain, TimestampDomain::EngineTicks);
        assert_eq!(trace.timestamp, 0xfeed_cafe);
        assert_eq!(trace.final_state, 0x42);
        assert_eq!(trace.fault_reason, 0);
        assert!(!trace.stale);
    }

    #[test]
    fn input_final_state_trace_stale_constructor_sets_expected_fields() {
        let trace_id = TraceId::new(0x1234_5678);
        let source_id = SourceId::new(0x9abc);
        let sequence = ObservationSequence::new(0xdead_beef);
        let trace = InputFinalStateTrace::stale(
            trace_id,
            source_id,
            sequence,
            TimestampDomain::SimulatorStep,
            0xfeed_cafe,
            0x42,
            0x7f,
        );

        assert_eq!(trace.trace_id, trace_id);
        assert_eq!(trace.source_id, source_id);
        assert_eq!(trace.sequence, sequence);
        assert_eq!(trace.timestamp_domain, TimestampDomain::SimulatorStep);
        assert_eq!(trace.timestamp, 0xfeed_cafe);
        assert_eq!(trace.final_state, 0x42);
        assert_eq!(trace.fault_reason, 0x7f);
        assert!(trace.stale);
    }
}
