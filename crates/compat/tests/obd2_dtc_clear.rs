#![cfg(feature = "transport-can")]

use ecu_compat::compat::EcuState;
use ecu_compat::diag::{DiagClearSummary, DiagCode, DiagEvent, DiagSource};
use ecu_compat::transport::compose_obd2_dtc_clear;
use ecu_compat::Micros;
use ecu_transport::{CanObd2DtcClearResponseError, CanObd2DtcClearVerdict, Message};

#[test]
fn compose_obd2_dtc_clear_clears_live_state_after_bounded_validation() {
    let mut state = EcuState::new();
    state.set_emergency_trigger_map_oob(true);
    state.set_emergency_mode(true);
    state.diag_map.latch(Micros::new(10));
    state.diag_tps.latch(Micros::new(20));
    state.diag_log_mut().push(DiagEvent {
        code: DiagCode::MapRange,
        timestamp: Micros::new(100),
        source: DiagSource::Sensor,
        context: Some(100),
        start_us: 10,
        end_us: 40,
    });
    state.diag_log_mut().push(DiagEvent {
        code: DiagCode::TpsRange,
        timestamp: Micros::new(200),
        source: DiagSource::Sensor,
        context: Some(50),
        start_us: 20,
        end_us: 60,
    });

    let request = Message::Obd2Request {
        service: 0x04,
        parameter_id: None,
        payload_len: 0,
        payload: [0; 6],
    };

    let surface = compose_obd2_dtc_clear(&mut state, &request).expect("bounded clear request");

    assert_eq!(
        surface.clear_summary,
        DiagClearSummary {
            cleared_active_count: 2,
            cleared_log_entries: 2,
            emergency_cleared: true,
        }
    );
    assert_eq!(
        surface.transport.verdict,
        CanObd2DtcClearVerdict {
            cleared_dtc_count: 4,
            freeze_frame_cleared: true,
            readiness_reset: true,
        }
    );
    assert_eq!(
        surface.transport.response,
        Message::Obd2Response {
            service: 0x44,
            parameter_id: None,
            negative_response_code: None,
            payload_len: 0,
            payload: [0; 6],
        }
    );
    assert!(!state.diag_map.is_active());
    assert!(!state.diag_tps.is_active());
    assert!(!state.diag_cam.is_active());
    assert!(!state.emergency_mode());
    assert!(state.diag_log().events.iter().all(|entry| entry.is_none()));
}

#[test]
fn compose_obd2_dtc_clear_rejects_invalid_request_without_mutation() {
    let mut state = EcuState::new();
    state.set_emergency_trigger_map_oob(true);
    state.set_emergency_mode(true);
    state.diag_map.latch(Micros::new(10));
    state.diag_log_mut().push(DiagEvent {
        code: DiagCode::MapRange,
        timestamp: Micros::new(100),
        source: DiagSource::Sensor,
        context: Some(100),
        start_us: 10,
        end_us: 40,
    });

    let request = Message::Obd2Request {
        service: 0x03,
        parameter_id: None,
        payload_len: 0,
        payload: [0; 6],
    };

    let error = compose_obd2_dtc_clear(&mut state, &request)
        .expect_err("unsupported service should not clear state");

    assert_eq!(
        error,
        CanObd2DtcClearResponseError::UnsupportedService { service_id: 0x03 }
    );
    assert!(state.diag_map.is_active());
    assert!(state.emergency_mode());
    assert_eq!(
        state
            .diag_log()
            .events
            .iter()
            .filter(|entry| entry.is_some())
            .count(),
        1
    );
}
