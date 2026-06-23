#![cfg(feature = "transport-can")]

#[cfg(test)]
use ecu_board_api::{BoardSensorSnapshot, BoardSensorSnapshotCapture, CaptureSample};
use ecu_target_common::transport_service::Obd2MultiServiceTransportService;
use ecu_transport::Transport;

pub fn new_service<T: Transport>(transport: T) -> Obd2MultiServiceTransportService<T> {
    Obd2MultiServiceTransportService::new(transport)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use ecu_domain::{diag::DiagClearSummary, FaultCode, FaultSeverity, Kpa10, Micros};
    use ecu_runtime::FaultState;
    use ecu_target_common::transport_service::{
        obd2_current_data_from_board_inputs, DiagnosticClearOwner, LiveObd2RequestOwner,
        Obd2MultiServiceTransportServiceOutcome, Obd2RetainedDiagnosticHistory,
    };
    use ecu_transport::TransportStats;
    use ecu_transport::{
        CanObd2MultiServiceDispatchVerdict, CanObd2NegativeResponseCode, Message, TransportError,
    };
    use std::collections::VecDeque;
    use std::vec::Vec;

    #[derive(Debug, Default)]
    struct MockTransport {
        rx: VecDeque<Message>,
        tx: Vec<Message>,
        poll_count: u32,
    }

    impl Transport for MockTransport {
        fn send(&mut self, message: &Message) -> Result<(), TransportError> {
            self.tx.push(message.clone());
            Ok(())
        }

        fn try_receive(&mut self) -> Option<Message> {
            self.rx.pop_front()
        }

        fn poll(&mut self) {
            self.poll_count = self.poll_count.saturating_add(1);
        }

        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }

        fn stats(&self) -> TransportStats {
            TransportStats::default()
        }

        fn is_ready(&self) -> bool {
            true
        }
    }

    #[derive(Debug, Clone)]
    struct MockOwner {
        fault_state: FaultState,
        logical_sensor_capture: Option<BoardSensorSnapshotCapture>,
        capture_sample: Option<CaptureSample>,
        retained_history: Obd2RetainedDiagnosticHistory,
        clear_summary: DiagClearSummary,
        clear_count: u8,
        identity_key_lifecycle: Option<ecu_transport::CanObd2IdentityKeyLifecycleStatus>,
        flash_write_fault: Option<ecu_transport::CanObd2FlashWriteFaultStatus>,
    }

    impl Default for MockOwner {
        fn default() -> Self {
            Self {
                fault_state: FaultState {
                    fault: FaultCode::None,
                    severity: FaultSeverity::Info,
                    cancel_reason: ecu_domain::CancelReason::Manual,
                },
                logical_sensor_capture: None,
                capture_sample: None,
                retained_history: Obd2RetainedDiagnosticHistory::new(
                    obd2_current_data_from_board_inputs(None, None),
                ),
                clear_summary: DiagClearSummary::default(),
                clear_count: 0,
                identity_key_lifecycle: None,
                flash_write_fault: None,
            }
        }
    }

    impl DiagnosticClearOwner for MockOwner {
        fn clear_diagnostics(&mut self) -> DiagClearSummary {
            self.clear_count = self.clear_count.saturating_add(1);
            self.fault_state.fault = FaultCode::None;
            self.clear_summary
        }
    }

    impl LiveObd2RequestOwner for MockOwner {
        fn fault_state(&self) -> FaultState {
            self.fault_state
        }

        fn current_obd2_sensor_data(&self) -> Message {
            obd2_current_data_from_board_inputs(self.logical_sensor_capture, self.capture_sample)
        }

        fn obd2_retained_history(&self) -> &Obd2RetainedDiagnosticHistory {
            &self.retained_history
        }

        fn obd2_retained_history_mut(&mut self) -> &mut Obd2RetainedDiagnosticHistory {
            &mut self.retained_history
        }

        fn obd2_identity_key_lifecycle_status(
            &self,
        ) -> Option<ecu_transport::CanObd2IdentityKeyLifecycleStatus> {
            self.identity_key_lifecycle
        }

        fn obd2_flash_write_fault_status(
            &self,
        ) -> Option<ecu_transport::CanObd2FlashWriteFaultStatus> {
            self.flash_write_fault
        }
    }

    fn populated_owner() -> MockOwner {
        MockOwner {
            fault_state: FaultState {
                fault: FaultCode::SyncLoss,
                severity: FaultSeverity::Critical,
                cancel_reason: ecu_domain::CancelReason::SyncLoss,
            },
            logical_sensor_capture: Some(BoardSensorSnapshotCapture {
                at_us: Micros::new(1234),
                angle_x10: ecu_domain::Degrees10::new(90),
                snapshot: BoardSensorSnapshot {
                    rpm: ecu_domain::Rpm::new(2500),
                    map_kpa10: Kpa10::new(950),
                    tps_x100: 2450,
                    clt_c10: 870,
                    iat_c10: 310,
                    vbatt_mv: 13_800,
                    lambda_x100: ecu_domain::Lambda100::new(102),
                    validity: ecu_board_api::BoardSensorValidityFlags::from_channels(
                        false, false, false, true,
                    ),
                    ..BoardSensorSnapshot::default()
                },
            }),
            capture_sample: Some(CaptureSample {
                at_us: Micros::new(1234),
                rpm: ecu_domain::Rpm::new(2500),
                load_kpa10: Kpa10::new(950),
                angle_x10: ecu_domain::Degrees10::new(90),
            }),
            retained_history: Obd2RetainedDiagnosticHistory::new(
                obd2_current_data_from_board_inputs(None, None),
            ),
            clear_summary: DiagClearSummary {
                cleared_active_count: 1,
                cleared_log_entries: 1,
                emergency_cleared: false,
            },
            clear_count: 0,
            identity_key_lifecycle: None,
            flash_write_fault: None,
        }
    }

    #[test]
    fn pump_once_dispatches_mode01_readiness_from_live_owner() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = new_service(transport);
        let mut owner = populated_owner();

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x01 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::ReadinessMonitorPositive
                );
                assert_eq!(
                    dispatch.response,
                    Message::Obd2Response {
                        service: 0x41,
                        parameter_id: Some(0x01),
                        negative_response_code: None,
                        payload_len: 4,
                        payload: [0x81, 0x21, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        assert_eq!(service.transport().poll_count, 1);
        assert_eq!(service.transport().tx.len(), 1);
    }

    #[test]
    fn pump_once_dispatches_stored_dtcs_and_freeze_frame_from_retained_fault() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        transport.rx.push_back(Message::Obd2Request {
            service: 0x02,
            parameter_id: Some(0x05),
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = new_service(transport);
        let mut owner = populated_owner();

        let stored = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("stored-dtc request should be consumed");
        match stored {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::StoredDtcs
                );
                assert_eq!(
                    dispatch.response,
                    Message::Obd2Response {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 2,
                        payload: [0x03, 0x40, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected stored-dtc outcome: {other:?}"),
        }

        let freeze = service
            .pump_once(&mut owner)
            .expect("mode 0x02 request should succeed")
            .expect("freeze-frame request should be consumed");
        match freeze {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::FreezeFrame
                );
                assert_eq!(
                    dispatch.response,
                    Message::Obd2Response {
                        service: 0x42,
                        parameter_id: Some(0x05),
                        negative_response_code: None,
                        payload_len: 1,
                        payload: [127, 0, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected freeze-frame outcome: {other:?}"),
        }
    }

    #[test]
    fn pump_once_dispatches_identity_key_lifecycle_mode09_segments() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(ecu_transport::CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID),
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = new_service(transport);
        let mut owner = MockOwner {
            identity_key_lifecycle: Some(ecu_transport::CanObd2IdentityKeyLifecycleStatus {
                present: true,
                persisted: true,
                request_id: 0x0102_0304,
                authorized: true,
                accepted: true,
                store_failed: false,
                rejected_reason: 0,
                generation: 0x0506_0708,
            }),
            ..MockOwner::default()
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("identity key lifecycle request should dispatch")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::SegmentedDispatch(dispatch) => {
                assert_eq!(
                    dispatch.info_type_id,
                    ecu_transport::CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID
                );
                assert_eq!(dispatch.segment_count, 2);
            }
            other => panic!("unexpected outcome: {other:?}"),
        }
        assert_eq!(
            service.transport().tx,
            [
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(ecu_transport::CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID),
                    sequence_index: 0,
                    segment_count: 2,
                    total_payload_len: ecu_transport::CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
                    segment_len: 6,
                    segment: [0x01, 0x02, 0x03, 0x04, 0xC3, 0],
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(ecu_transport::CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID),
                    sequence_index: 1,
                    segment_count: 2,
                    total_payload_len: ecu_transport::CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
                    segment_len: 4,
                    segment: [0x05, 0x06, 0x07, 0x08, 0, 0],
                },
            ]
        );
    }

    #[test]
    fn pump_once_dispatches_flash_write_fault_mode09_segment() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID),
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = new_service(transport);
        let mut owner = MockOwner {
            flash_write_fault: Some(ecu_transport::CanObd2FlashWriteFaultStatus {
                present: true,
                phase: ecu_transport::CanObd2FlashWriteFaultPhase::Erase,
                sr_bits: 0x0000_00F2,
            }),
            ..MockOwner::default()
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("flash write fault request should dispatch")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::SegmentedDispatch(dispatch) => {
                assert_eq!(
                    dispatch.info_type_id,
                    ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID
                );
                assert_eq!(dispatch.segment_count, 1);
                assert_eq!(dispatch.total_payload_len, 6);
            }
            other => panic!("unexpected outcome: {other:?}"),
        }
        assert_eq!(
            service.transport().tx,
            [Message::Obd2SegmentedResponse {
                service: 0x49,
                parameter_id: Some(ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID),
                sequence_index: 0,
                segment_count: 1,
                total_payload_len: ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN,
                segment_len: 6,
                segment: [0x80, 0x02, 0x00, 0x00, 0x00, 0xF2],
            }]
        );
    }

    #[test]
    fn pump_once_clears_retained_diagnostics_on_mode04() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = new_service(transport);
        let mut owner = populated_owner();

        let clear = service
            .pump_once(&mut owner)
            .expect("mode 0x04 request should succeed")
            .expect("clear request should be consumed");
        match clear {
            Obd2MultiServiceTransportServiceOutcome::DtcClear {
                clear_summary,
                dispatch,
            } => {
                assert_eq!(clear_summary, owner.clear_summary);
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::DtcClearPositive
                );
                assert_eq!(
                    dispatch.response,
                    Message::Obd2Response {
                        service: 0x44,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 0,
                        payload: [0; 6],
                    }
                );
            }
            other => panic!("unexpected clear outcome: {other:?}"),
        }

        let stored = service
            .pump_once(&mut owner)
            .expect("post-clear stored-dtc request should succeed")
            .expect("stored-dtc request should be consumed");
        match stored {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.response,
                    Message::Obd2Response {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 0,
                        payload: [0; 6],
                    }
                );
            }
            other => panic!("unexpected post-clear outcome: {other:?}"),
        }

        assert_eq!(owner.clear_count, 1);
    }

    #[test]
    fn pump_once_returns_negative_for_freeze_frame_without_retained_fault() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x02,
            parameter_id: Some(0x05),
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = new_service(transport);
        let mut owner = MockOwner::default();
        owner.capture_sample = Some(CaptureSample {
            at_us: Micros::new(77),
            rpm: ecu_domain::Rpm::new(0),
            load_kpa10: Kpa10::new(700),
            angle_x10: ecu_domain::Degrees10::new(0),
        });

        let outcome = service
            .pump_once(&mut owner)
            .expect("missing freeze frame should map to bounded negative")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::NegativeResponse(
                        CanObd2NegativeResponseCode::RequestOutOfRange
                    )
                );
                assert_eq!(
                    dispatch.response,
                    Message::Obd2Response {
                        service: 0x7F,
                        parameter_id: Some(0x02),
                        negative_response_code: Some(0x31),
                        payload_len: 0,
                        payload: [0; 6],
                    }
                );
            }
            other => panic!("unexpected negative outcome: {other:?}"),
        }
    }
}
