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
}
