//! Raw electrical and trace IO layer.
//!
//! `ecu-io` is intentionally below the logical board contract in
//! `ecu-board-api`. Keep this crate focused on raw edge samples, pin
//! transitions, capture buffers, sensor frames, and trace/replay records that
//! describe electrical or recorded IO facts.
//!
//! Do not add recipe-level board capability metadata, watchdog, calibration,
//! telemetry, or runtime action-lowering contracts here. Those logical ECU
//! board contracts belong in `ecu-board-api`; runtime lowering belongs in
//! `ecu-runtime`.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod capture;
mod edge;
mod output;
mod sensor;
pub mod trace;

pub use capture::CaptureBuffer;
pub use edge::{EdgeLine, EdgePolarity, EdgeSample, EdgeSource};
pub use output::{OutputLevel, OutputTransition, OutputTransitionKind, OutputTransitionSink};
pub use sensor::{SensorFrame, SensorFrameSource, SensorValidityFlags};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::{TraceInputKind, TracePayload, TraceRecord, TraceRecordError};
    use ecu_domain::{ChannelId, Degrees10, FaultCode, Micros, Rpm};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct TestError;

    struct OneEdge(Option<EdgeSample>);

    impl EdgeSource for OneEdge {
        type Error = TestError;

        fn next_edge(&mut self) -> Result<Option<EdgeSample>, Self::Error> {
            Ok(self.0.take())
        }
    }

    struct OneFrame(Option<SensorFrame>);

    impl SensorFrameSource for OneFrame {
        type Error = TestError;

        fn next_frame(&mut self) -> Result<Option<SensorFrame>, Self::Error> {
            Ok(self.0.take())
        }
    }

    struct TransitionRecorder {
        last: Option<OutputTransition>,
    }

    impl OutputTransitionSink for TransitionRecorder {
        type Error = TestError;

        fn push_transition(&mut self, transition: OutputTransition) -> Result<(), Self::Error> {
            self.last = Some(transition);
            Ok(())
        }
    }

    const EDGE: EdgeSample = EdgeSample {
        at_us: Micros::new(10),
        line: EdgeLine::Cam,
        polarity: EdgePolarity::Falling,
        angle_x10: Degrees10::new(42),
        rpm: Rpm::new(1200),
    };

    const FRAME: SensorFrame = SensorFrame {
        at_us: Micros::new(11),
        rpm: Rpm::new(1201),
        map_kpa10: ecu_domain::Kpa10::new(1000),
        maf_x100: ecu_domain::MassAirFlowX100::new(0),
        maf_valid: false,
        knock_x100: ecu_domain::KnockLevelX100::new(0),
        knock_valid: false,
        cam_phase_deg10: None,
        angle_x10: Degrees10::new(43),
        tps_x100: 1000,
        clt_c10: 700,
        iat_c10: 240,
        vbatt_mv: 12_500,
        baro_kpa10: ecu_domain::Kpa10::new(1010),
        vehicle_speed_kph10: ecu_domain::VehicleSpeedKph10::new(0),
        vehicle_speed_valid: false,
        lambda_valid: false,
        lambda_x100: ecu_domain::Lambda100::new(0),
    };

    const TRANSITION: OutputTransition = OutputTransition {
        at_us: Micros::new(12),
        kind: OutputTransitionKind::Injector,
        channel: ChannelId::new(2),
        level: OutputLevel::High,
    };

    #[test]
    fn io_source_and_sink_traits_preserve_record_values() {
        let mut edges = OneEdge(Some(EDGE));
        assert_eq!(edges.next_edge(), Ok(Some(EDGE)));
        assert_eq!(edges.next_edge(), Ok(None));

        let mut frames = OneFrame(Some(FRAME));
        assert_eq!(frames.next_frame(), Ok(Some(FRAME)));
        assert_eq!(frames.next_frame(), Ok(None));

        let mut transitions = TransitionRecorder { last: None };
        assert_eq!(transitions.push_transition(TRANSITION), Ok(()));
        assert_eq!(transitions.last, Some(TRANSITION));
    }

    #[test]
    fn exported_trace_error_can_describe_raw_record_mismatch() {
        let err = TraceRecord::try_from_raw(
            1,
            Micros::new(10),
            TraceInputKind::Output,
            TracePayload::None,
            Rpm::new(0),
            false,
            0,
            Degrees10::new(0),
            FaultCode::None,
        )
        .unwrap_err();

        assert_eq!(
            err,
            TraceRecordError::PayloadKindMismatch {
                input_kind: TraceInputKind::Output,
                payload: TracePayload::None,
            }
        );
    }
}
