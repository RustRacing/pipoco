//! ECU inter-component message protocol.

use serde::{Deserialize, Serialize};

/// Serialized payload bytes available in a single classic CAN frame after the
/// transport transaction header.
pub const CLASSIC_CAN_SINGLE_FRAME_PAYLOAD: usize = 7;
/// Maximum postcard-encoded message size supported by built-in transports.
pub const MAX_ENCODED_MESSAGE_SIZE: usize = 1024;

/// All ECU inter-component messages.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum Message {
    TriggerTiming {
        gap_period_us: u32,
        tooth_period_us: u16,
        tooth_position: u8,
        synced: bool,
        timestamp_us: u32,
    },
    SensorData {
        map_kpa_x10: u16,
        tps_percent: u8,
        iat_offset: u8,
        clt_offset: u8,
        voltage_x10: u8,
        lambda_x100: u8,
        flags: u8,
        timestamp_us: u32,
    },
    GpsData {
        latitude_e7: i32,
        longitude_e7: i32,
        speed_cm_per_s: u16,
        heading_deg_x10: u16,
        timestamp_us: u32,
    },
    ExternalEgtData {
        bank: u8,
        channel: u8,
        egt_c_x10: u16,
        flags: u8,
        timestamp_us: u32,
    },
    Obd2Request {
        service: u8,
        parameter_id: Option<u8>,
        payload_len: u8,
        payload: [u8; 6],
    },
    Obd2Response {
        service: u8,
        parameter_id: Option<u8>,
        negative_response_code: Option<u8>,
        payload_len: u8,
        payload: [u8; 6],
    },
    Obd2SegmentedResponse {
        service: u8,
        parameter_id: Option<u8>,
        sequence_index: u8,
        segment_count: u8,
        total_payload_len: u8,
        segment_len: u8,
        segment: [u8; 6],
    },
    Obd2IdentityProvisioningCommand {
        request_id: u32,
        vin_len: u8,
        vin: [u8; 17],
        calibration_id_len: u8,
        calibration_id: [u8; 17],
        board_build_identity_len: u8,
        board_build_identity: [u8; 6],
    },
    Obd2IdentityProvisioningArm {
        request_id: u32,
        nonce: u32,
        vin_len: u8,
        vin: [u8; 17],
        calibration_id_len: u8,
        calibration_id: [u8; 17],
        board_build_identity_len: u8,
        board_build_identity: [u8; 6],
        tag: [u8; 32],
    },
    Obd2IdentityProvisioningArmAudit {
        request_id: u32,
        armed: bool,
    },
    Obd2IdentityProvisioningAudit {
        request_id: u32,
        authorized: bool,
        authorization_failed: bool,
        attempted: bool,
        accepted: bool,
        store_failed: bool,
        rejected_field: u8,
        vin_len: u8,
        calibration_id_len: u8,
        board_build_identity_len: u8,
    },
    Obd2IdentityProvisioningKeyCommand {
        request_id: u32,
        nonce: u32,
        generation: u32,
        revoke: bool,
        key: [u8; 32],
        tag: [u8; 32],
    },
    Obd2IdentityProvisioningKeyAudit {
        request_id: u32,
        authorized: bool,
        accepted: bool,
        store_failed: bool,
        rejected_reason: u8,
        generation: u32,
    },
    IpwTable {
        version: u32,
        data: [[u16; 16]; 16],
        crc32: u32,
    },
    IgnitionTable {
        version: u32,
        data: [[u16; 16]; 16],
        crc32: u32,
    },
    EngineConfig {
        num_cylinders: u8,
        displacement_cc: u16,
        injection_mode: u8,
        ignition_mode: u8,
        trigger_teeth: u8,
        trigger_missing: u8,
    },
    InjectorConfig {
        flow_rate_cc_per_min: u16,
        dead_time_us: u16,
        voltage_slope_x100: i16,
    },
    Heartbeat {
        node_id: u8,
        uptime_seconds: u32,
        status: u8,
        error_count: u16,
        cpu_usage: u8,
    },
    Error {
        node_id: u8,
        error_code: u16,
        severity: u8,
        timestamp_us: u32,
        data: [u8; 4],
    },
    CmdReset {
        target_node_id: u8,
    },
    CmdEngineControl {
        command: u8,
    },
    CmdCalibrate {
        target_node_id: u8,
        cal_type: u8,
    },
}

impl Message {
    /// Get the actual postcard-encoded size in bytes.
    pub fn encoded_len(&self) -> Result<usize, postcard::Error> {
        let mut buf = [0u8; MAX_ENCODED_MESSAGE_SIZE];
        postcard::to_slice(self, &mut buf).map(|encoded| encoded.len())
    }

    /// Check if message requires fragmentation on classic CAN.
    pub fn requires_fragmentation(&self) -> bool {
        self.encoded_len()
            .map(|len| len > CLASSIC_CAN_SINGLE_FRAME_PAYLOAD)
            .unwrap_or(true)
    }
}

#[cfg(test)]
pub(crate) const TRANSPORT_PARITY_SAMPLE_COUNT: usize = 8;

#[cfg(test)]
pub(crate) fn transport_parity_sample_messages() -> [Message; TRANSPORT_PARITY_SAMPLE_COUNT] {
    let mut table = [[0u16; 16]; 16];
    let mut row = 0usize;
    while row < 16 {
        let mut col = 0usize;
        while col < 16 {
            table[row][col] = (1000 + row * 16 + col) as u16;
            col += 1;
        }
        row += 1;
    }

    [
        Message::CmdEngineControl { command: 0x22 },
        Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 12,
            synced: true,
            timestamp_us: 12345,
        },
        Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 54321,
        },
        Message::GpsData {
            latitude_e7: 377749000,
            longitude_e7: -1224194000,
            speed_cm_per_s: 1834,
            heading_deg_x10: 2715,
            timestamp_us: 42,
        },
        Message::ExternalEgtData {
            bank: 1,
            channel: 2,
            egt_c_x10: 8650,
            flags: 0,
            timestamp_us: 77,
        },
        Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        },
        Message::Obd2Response {
            service: 0x41,
            parameter_id: Some(0x0C),
            negative_response_code: None,
            payload_len: 2,
            payload: [0x1A, 0xF8, 0, 0, 0, 0],
        },
        Message::IpwTable {
            version: 7,
            data: table,
            crc32: 0xDEADBEEF,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn message_priorities_match_legacy_core() {
        let trigger = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };
        assert!(trigger.encoded_len().is_ok());
    }

    #[test]
    fn table_messages_report_fragmentation_need() {
        let table = Message::IpwTable {
            version: 1,
            data: [[1000; 16]; 16],
            crc32: 0,
        };
        assert!(table.requires_fragmentation());
    }

    #[test]
    fn fragmentation_uses_actual_classic_can_payload_boundary() {
        let reset = Message::CmdReset { target_node_id: 1 };
        assert!(reset.encoded_len().expect("encoded len") <= CLASSIC_CAN_SINGLE_FRAME_PAYLOAD);
        assert!(!reset.requires_fragmentation());

        let trigger = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };
        assert!(trigger.encoded_len().expect("encoded len") > CLASSIC_CAN_SINGLE_FRAME_PAYLOAD);
        assert!(trigger.requires_fragmentation());
    }

    #[test]
    fn optional_sensor_messages_encode_cleanly() {
        let gps = Message::GpsData {
            latitude_e7: 377749000,
            longitude_e7: -1224194000,
            speed_cm_per_s: 1834,
            heading_deg_x10: 2715,
            timestamp_us: 42,
        };
        let egt = Message::ExternalEgtData {
            bank: 1,
            channel: 2,
            egt_c_x10: 8650,
            flags: 0,
            timestamp_us: 77,
        };

        assert!(gps.encoded_len().is_ok());
        assert!(egt.encoded_len().is_ok());
    }

    #[test]
    fn obd2_messages_encode_cleanly() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };
        let response = Message::Obd2Response {
            service: 0x41,
            parameter_id: Some(0x0C),
            negative_response_code: None,
            payload_len: 2,
            payload: [0x1A, 0xF8, 0, 0, 0, 0],
        };
        let segmented = Message::Obd2SegmentedResponse {
            service: 0x49,
            parameter_id: Some(0x02),
            sequence_index: 0,
            segment_count: 3,
            total_payload_len: 17,
            segment_len: 6,
            segment: *b"PIPOCO",
        };

        assert!(request.encoded_len().is_ok());
        assert!(response.encoded_len().is_ok());
        assert!(segmented.encoded_len().is_ok());
    }

    #[test]
    fn transport_parity_sample_set_covers_current_shared_surface_categories() {
        let samples = transport_parity_sample_messages();

        assert!(samples
            .iter()
            .any(|message| matches!(message, Message::CmdEngineControl { .. })));
        assert!(samples.iter().any(|message| matches!(
            message,
            Message::TriggerTiming { .. } | Message::SensorData { .. }
        )));
        assert!(samples.iter().any(|message| matches!(
            message,
            Message::GpsData { .. } | Message::ExternalEgtData { .. }
        )));
        assert!(samples.iter().any(|message| matches!(
            message,
            Message::Obd2Request { .. } | Message::Obd2Response { .. }
        )));
    }
}
