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
    Output,
}

/// Payload of a trace record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TracePayload {
    None,
    Edge(EdgeSample),
    Sensor(SensorFrame),
    Output(OutputTransition),
}

/// Error returned when constructing a trace record from raw kind/payload fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TraceRecordError {
    PayloadKindMismatch {
        input_kind: TraceInputKind,
        payload: TracePayload,
    },
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
    input_kind: TraceInputKind,
    /// Payload data for this record.
    payload: TracePayload,
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

#[allow(clippy::too_many_arguments)]
impl TraceRecord {
    /// Construct a no-op trace marker.
    pub const fn none(
        step_index: u32,
        time_us: Micros,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Self {
        Self::new_unchecked(
            step_index,
            time_us,
            TraceInputKind::None,
            TracePayload::None,
            rpm,
            synced,
            tooth,
            angle_x10,
            diagnostic_code,
        )
    }

    /// Construct an edge trace record.
    pub const fn edge(
        step_index: u32,
        time_us: Micros,
        edge: EdgeSample,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Self {
        Self::new_unchecked(
            step_index,
            time_us,
            TraceInputKind::Edge,
            TracePayload::Edge(edge),
            rpm,
            synced,
            tooth,
            angle_x10,
            diagnostic_code,
        )
    }

    /// Construct a sensor-frame trace record.
    pub const fn sensor(
        step_index: u32,
        time_us: Micros,
        frame: SensorFrame,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Self {
        Self::new_unchecked(
            step_index,
            time_us,
            TraceInputKind::Sensor,
            TracePayload::Sensor(frame),
            rpm,
            synced,
            tooth,
            angle_x10,
            diagnostic_code,
        )
    }

    /// Construct a periodic tick trace record.
    pub const fn tick(
        step_index: u32,
        time_us: Micros,
        frame: SensorFrame,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Self {
        Self::new_unchecked(
            step_index,
            time_us,
            TraceInputKind::Tick,
            TracePayload::Sensor(frame),
            rpm,
            synced,
            tooth,
            angle_x10,
            diagnostic_code,
        )
    }

    /// Construct an output transition trace record.
    pub const fn output(
        step_index: u32,
        time_us: Micros,
        transition: OutputTransition,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Self {
        Self::new_unchecked(
            step_index,
            time_us,
            TraceInputKind::Output,
            TracePayload::Output(transition),
            rpm,
            synced,
            tooth,
            angle_x10,
            diagnostic_code,
        )
    }

    /// Construct a trace record from raw wire-format fields.
    pub const fn try_from_raw(
        step_index: u32,
        time_us: Micros,
        input_kind: TraceInputKind,
        payload: TracePayload,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Result<Self, TraceRecordError> {
        if Self::payload_matches_kind(input_kind, payload) {
            Ok(Self::new_unchecked(
                step_index,
                time_us,
                input_kind,
                payload,
                rpm,
                synced,
                tooth,
                angle_x10,
                diagnostic_code,
            ))
        } else {
            Err(TraceRecordError::PayloadKindMismatch {
                input_kind,
                payload,
            })
        }
    }

    pub const fn input_kind(self) -> TraceInputKind {
        self.input_kind
    }

    pub const fn payload(self) -> TracePayload {
        self.payload
    }

    pub const fn is_consistent(self) -> bool {
        Self::payload_matches_kind(self.input_kind, self.payload)
    }

    const fn payload_matches_kind(input_kind: TraceInputKind, payload: TracePayload) -> bool {
        matches!(
            (input_kind, payload),
            (TraceInputKind::None, TracePayload::None)
                | (TraceInputKind::Edge, TracePayload::Edge(_))
                | (TraceInputKind::Sensor, TracePayload::Sensor(_))
                | (TraceInputKind::Tick, TracePayload::Sensor(_))
                | (TraceInputKind::Output, TracePayload::Output(_))
        )
    }

    const fn new_unchecked(
        step_index: u32,
        time_us: Micros,
        input_kind: TraceInputKind,
        payload: TracePayload,
        rpm: Rpm,
        synced: bool,
        tooth: u16,
        angle_x10: Degrees10,
        diagnostic_code: FaultCode,
    ) -> Self {
        Self {
            step_index,
            time_us,
            input_kind,
            payload,
            rpm,
            synced,
            tooth,
            angle_x10,
            diagnostic_code,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EdgeLine, EdgePolarity, OutputLevel, OutputTransitionKind, SensorValidityFlags};
    use ecu_domain::{
        ChannelId, KnockLevelX100, Kpa10, Lambda100, MassAirFlowX100, VehicleSpeedKph10,
    };

    const EDGE: EdgeSample = EdgeSample {
        at_us: Micros::new(10),
        line: EdgeLine::Crank,
        polarity: EdgePolarity::Rising,
        angle_x10: Degrees10::new(120),
        rpm: Rpm::new(900),
    };

    const SENSOR: SensorFrame = SensorFrame {
        at_us: Micros::new(20),
        rpm: Rpm::new(901),
        map_kpa10: Kpa10::new(990),
        maf_x100: MassAirFlowX100::new(0),
        maf_valid: false,
        knock_x100: KnockLevelX100::new(0),
        knock_valid: false,
        cam_phase_deg10: None,
        angle_x10: Degrees10::new(130),
        tps_x100: 2500,
        clt_c10: 800,
        iat_c10: 250,
        vbatt_mv: 13_800,
        baro_kpa10: Kpa10::new(1010),
        vehicle_speed_kph10: VehicleSpeedKph10::new(0),
        vehicle_speed_valid: false,
        lambda_valid: true,
        lambda_x100: Lambda100::new(100),
    };

    const OUTPUT: OutputTransition = OutputTransition {
        at_us: Micros::new(30),
        kind: OutputTransitionKind::Ignition,
        channel: ChannelId::new(1),
        level: OutputLevel::High,
    };

    #[test]
    fn typed_constructors_create_consistent_records() {
        let records = [
            TraceRecord::none(
                1,
                Micros::new(1),
                Rpm::new(900),
                false,
                0,
                Degrees10::new(0),
                FaultCode::None,
            ),
            TraceRecord::edge(
                2,
                Micros::new(10),
                EDGE,
                Rpm::new(900),
                true,
                1,
                Degrees10::new(120),
                FaultCode::None,
            ),
            TraceRecord::sensor(
                3,
                Micros::new(20),
                SENSOR,
                Rpm::new(901),
                true,
                2,
                Degrees10::new(130),
                FaultCode::None,
            ),
            TraceRecord::tick(
                4,
                Micros::new(25),
                SENSOR,
                Rpm::new(901),
                true,
                2,
                Degrees10::new(130),
                FaultCode::None,
            ),
            TraceRecord::output(
                5,
                Micros::new(30),
                OUTPUT,
                Rpm::new(902),
                true,
                3,
                Degrees10::new(140),
                FaultCode::ActuatorFault,
            ),
        ];

        for record in records {
            assert!(record.is_consistent());
        }
    }

    #[test]
    fn raw_constructor_rejects_payload_kind_mismatch() {
        let err = TraceRecord::try_from_raw(
            1,
            Micros::new(10),
            TraceInputKind::Edge,
            TracePayload::Sensor(SENSOR),
            Rpm::new(900),
            true,
            1,
            Degrees10::new(120),
            FaultCode::None,
        )
        .unwrap_err();

        assert_eq!(
            err,
            TraceRecordError::PayloadKindMismatch {
                input_kind: TraceInputKind::Edge,
                payload: TracePayload::Sensor(SENSOR),
            }
        );
    }

    #[test]
    fn raw_constructor_accepts_tick_sensor_payload_contract() {
        let record = TraceRecord::try_from_raw(
            1,
            Micros::new(20),
            TraceInputKind::Tick,
            TracePayload::Sensor(SENSOR),
            Rpm::new(901),
            true,
            2,
            Degrees10::new(130),
            FaultCode::None,
        )
        .unwrap();

        assert_eq!(record.input_kind(), TraceInputKind::Tick);
        assert_eq!(record.payload(), TracePayload::Sensor(SENSOR));
        assert!(record.is_consistent());
        assert_eq!(
            SensorValidityFlags::from_frame(SENSOR).bits(),
            SensorValidityFlags::LAMBDA
        );
    }
}
