#![cfg(all(feature = "transport-bbqueue", feature = "transport-can"))]

use ecu_compat::compat::EcuState;
use ecu_compat::diag::{DiagCode, DiagEvent, DiagSource};
use ecu_compat::Micros;
use ecu_domain::diag::DiagClearSummary;
use ecu_target_common::transport_service::{DiagnosticClearOwner, Obd2TransportService};
use ecu_transport::{BbqTransport, Message, Transport};

struct CompatClearOwner {
    state: EcuState,
}

impl DiagnosticClearOwner for CompatClearOwner {
    fn clear_diagnostics(&mut self) -> DiagClearSummary {
        self.state.clear_diagnostics()
    }
}

fn seeded_owner() -> CompatClearOwner {
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
    CompatClearOwner { state }
}

#[test]
fn bbq_transport_clear_request_roundtrips_through_live_owner() {
    let (mut requester, owner_transport) =
        BbqTransport::create_pair().expect("bbqueue transport pair");
    let mut service = Obd2TransportService::new(owner_transport);
    let mut owner = seeded_owner();

    requester
        .send(&Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        })
        .expect("send request");

    let outcome = service
        .pump_once(&mut owner)
        .expect("valid request should succeed")
        .expect("request should be consumed");

    assert!(!owner.state.diag_map.is_active());
    assert!(!owner.state.emergency_mode());
    assert!(owner
        .state
        .diag_log()
        .events
        .iter()
        .all(|entry| entry.is_none()));

    match outcome {
        ecu_target_common::transport_service::Obd2TransportServiceOutcome::DtcClear(surface) => {
            assert_eq!(surface.clear_summary.cleared_active_count, 1);
            assert_eq!(surface.clear_summary.cleared_log_entries, 1);
            assert!(surface.clear_summary.emergency_cleared);
        }
        other => panic!("unexpected outcome: {other:?}"),
    }

    assert_eq!(
        requester.try_receive(),
        Some(Message::Obd2Response {
            service: 0x44,
            parameter_id: None,
            negative_response_code: None,
            payload_len: 0,
            payload: [0; 6],
        })
    );
}
