#![cfg(all(feature = "transport-bbqueue", feature = "transport-can"))]

use ecu_compat::compat::EcuState;
use ecu_compat::diag::{DiagCode, DiagEvent, DiagSource};
use ecu_compat::Micros;
use ecu_domain::diag::DiagClearSummary;
use ecu_target_common::transport_service::{DiagnosticClearOwner, Obd2TransportService};
use ecu_transport::{BbqTransport, CanObd2DtcClearResponseError, Message, Transport};

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
fn bbq_transport_parameterized_clear_request_stays_non_mutating() {
    let (mut requester, owner_transport) =
        BbqTransport::create_pair().expect("bbqueue transport pair");
    let mut service = Obd2TransportService::new(owner_transport);
    let mut owner = seeded_owner();

    requester
        .send(&Message::Obd2Request {
            service: 0x04,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        })
        .expect("send request");

    let error = service
        .pump_once(&mut owner)
        .expect_err("parameterized clear should fail");

    assert_eq!(
        error,
        ecu_target_common::transport_service::Obd2TransportServiceError::Compose(
            CanObd2DtcClearResponseError::UnexpectedParameterId { parameter_id: 0x01 }
        )
    );
    assert!(owner.state.diag_map.is_active());
    assert!(owner.state.emergency_mode());
    assert_eq!(
        owner
            .state
            .diag_log()
            .events
            .iter()
            .filter(|entry| entry.is_some())
            .count(),
        1
    );
    assert_eq!(requester.try_receive(), None);
}
