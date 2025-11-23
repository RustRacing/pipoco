//! ECU inter-component message protocol
//!
//! This module defines all messages used for communication between ECU components.
//! Messages are transport-agnostic and serialized using postcard for efficiency.

use serde::{Deserialize, Serialize};

/// Protocol version for forward compatibility
pub const PROTOCOL_VERSION: u16 = 1;

/// All ECU inter-component messages
///
/// This enum is transport-agnostic and used by all modules.
/// Serialized using postcard for efficient no_std encoding.
///
/// # Message Categories
///
/// - **Trigger/Timing**: Real-time trigger wheel data
/// - **Sensors**: Engine sensor readings
/// - **Tables**: Fuel and ignition lookup tables
/// - **Configuration**: Engine and component parameters
/// - **Status/Diagnostics**: Health monitoring
/// - **Commands**: Control commands
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub enum Message {
    // ========== Trigger/Timing Messages ==========
    /// Raw trigger timing data (Injection → Management)
    ///
    /// Sent every engine revolution or 100ms, whichever is slower.
    /// Management engine uses this for accurate RPM calculation.
    TriggerTiming {
        /// Missing tooth gap period (microseconds)
        gap_period_us: u32,

        /// Last normal tooth period (microseconds)
        tooth_period_us: u16,

        /// Current tooth position (1-58 for 60-2 wheel)
        tooth_position: u8,

        /// Sync status
        synced: bool,

        /// Timestamp when captured (microseconds)
        timestamp_us: u32,
    },

    // ========== Sensor Messages ==========
    /// Real-time sensor data (Sensor → Management)
    ///
    /// Sent at 100 Hz for fast response to engine conditions.
    SensorData {
        /// Manifold Absolute Pressure in kPa * 10 (range 0-255.0 kPa)
        map_kpa_x10: u16,

        /// Throttle Position Sensor percentage (0-100%)
        tps_percent: u8,

        /// Intake Air Temperature in °C + 40 (range -40 to +215°C)
        iat_offset: u8,

        /// Coolant Temperature in °C + 40 (range -40 to +215°C)
        clt_offset: u8,

        /// Battery voltage * 10 (range 0-25.5V)
        voltage_x10: u8,

        /// O2 sensor lambda * 100 (range 0-2.55 lambda)
        lambda_x100: u8,

        /// Status flags
        /// Bit 0: MAP sensor valid
        /// Bit 1: TPS sensor valid
        /// Bit 2: IAT sensor valid
        /// Bit 3: CLT sensor valid
        /// Bit 4: O2 sensor valid
        flags: u8,

        /// Timestamp when sampled (microseconds)
        timestamp_us: u32,
    },

    // ========== Fuel/Ignition Tables ==========
    /// Complete IPW table (Management → Injection)
    ///
    /// Contains ready-to-use pulse widths for all operating points.
    /// Table is [load_bins][rpm_bins] with 16x16 = 256 u16 values.
    IpwTable {
        /// Table version number (increments on each update)
        version: u32,

        /// 16x16 table of pulse widths in microseconds
        /// Indexed by [load_index][rpm_index]
        data: [[u16; 16]; 16],

        /// CRC32 checksum for validation
        crc32: u32,
    },

    /// Ignition timing table (Management → Injection)
    ///
    /// Spark advance in degrees BTDC for each operating point.
    IgnitionTable {
        /// Table version
        version: u32,

        /// 16x16 table of advance in degrees * 10
        /// Indexed by [load_index][rpm_index]
        /// Example: 350 = 35.0° BTDC
        data: [[u16; 16]; 16],

        /// CRC32 checksum
        crc32: u32,
    },

    // ========== Configuration Messages ==========
    /// Engine configuration parameters (Management → Injection)
    EngineConfig {
        /// Number of cylinders
        num_cylinders: u8,

        /// Displacement in cc
        displacement_cc: u16,

        /// Injection mode: 0=batch, 1=sequential
        injection_mode: u8,

        /// Ignition mode: 0=wasted spark, 1=sequential
        ignition_mode: u8,

        /// Number of teeth on trigger wheel
        trigger_teeth: u8,

        /// Number of missing teeth
        trigger_missing: u8,
    },

    /// Injector characteristics (Management → Injection)
    InjectorConfig {
        /// Flow rate in cc/min at reference pressure
        flow_rate_cc_per_min: u16,

        /// Dead time at 14V in microseconds
        dead_time_us: u16,

        /// Voltage correction slope * 100
        voltage_slope_x100: i16,
    },

    // ========== Status/Diagnostics ==========
    /// Heartbeat from any module
    ///
    /// Sent every 100ms for watchdog monitoring.
    Heartbeat {
        /// Node ID (1=management, 2=injection, 3=sensor, etc.)
        node_id: u8,

        /// Uptime in seconds
        uptime_seconds: u32,

        /// Status flags
        /// Bit 0: Error present
        /// Bit 1: Warning present
        /// Bit 2: Calibration mode
        status: u8,

        /// Error count since boot
        error_count: u16,

        /// CPU usage percentage (0-100%)
        cpu_usage: u8,
    },

    /// Error report from any module
    Error {
        /// Node ID reporting the error
        node_id: u8,

        /// Error code
        error_code: u16,

        /// Error severity: 0=info, 1=warning, 2=error, 3=critical
        severity: u8,

        /// Timestamp when error occurred (microseconds)
        timestamp_us: u32,

        /// Optional error data (context-specific)
        data: [u8; 4],
    },

    // ========== Commands ==========
    /// Command to reset a module
    CmdReset {
        /// Target node ID (or 0xFF for broadcast)
        target_node_id: u8,
    },

    /// Command to start/stop engine
    CmdEngineControl {
        /// 0=stop, 1=start
        command: u8,
    },

    /// Command to enter calibration mode
    CmdCalibrate {
        /// Target node ID
        target_node_id: u8,

        /// Calibration type
        cal_type: u8,
    },
}

impl Message {
    /// Get message priority (0 = highest)
    ///
    /// Used by transports that support prioritization (e.g., CAN).
    /// BBQueue ignores this as it's FIFO.
    pub fn priority(&self) -> u8 {
        match self {
            Message::TriggerTiming { .. } => 0,    // Highest - time-critical
            Message::CmdEngineControl { .. } => 0, // Highest - safety critical
            Message::SensorData { .. } => 1,       // High - affects calculations
            Message::Error { .. } => 2,            // High - safety monitoring
            Message::IpwTable { .. } => 3,         // Medium - only changes occasionally
            Message::IgnitionTable { .. } => 3,
            Message::EngineConfig { .. } => 4,
            Message::InjectorConfig { .. } => 4,
            Message::CmdCalibrate { .. } => 4,
            Message::CmdReset { .. } => 1,  // High - system control
            Message::Heartbeat { .. } => 5, // Low - can tolerate delays
        }
    }

    /// Get approximate serialized size in bytes
    ///
    /// Used by transports to allocate appropriate buffers.
    /// Estimates are conservative (may be slightly larger than actual).
    pub fn estimated_size(&self) -> usize {
        match self {
            Message::TriggerTiming { .. } => 16,
            Message::SensorData { .. } => 20,
            Message::IpwTable { .. } => 528, // Large! (4 + 512 + 4 + overhead)
            Message::IgnitionTable { .. } => 528, // Large!
            Message::EngineConfig { .. } => 12,
            Message::InjectorConfig { .. } => 10,
            Message::Heartbeat { .. } => 12,
            Message::Error { .. } => 16,
            Message::CmdReset { .. } => 4,
            Message::CmdEngineControl { .. } => 4,
            Message::CmdCalibrate { .. } => 4,
        }
    }

    /// Check if message requires fragmentation on CAN (>8 bytes)
    pub fn requires_fragmentation(&self) -> bool {
        self.estimated_size() > 8
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_priorities() {
        let trigger = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };
        assert_eq!(trigger.priority(), 0);

        let heartbeat = Message::Heartbeat {
            node_id: 1,
            uptime_seconds: 100,
            status: 0,
            error_count: 0,
            cpu_usage: 50,
        };
        assert_eq!(heartbeat.priority(), 5);
    }

    #[test]
    fn test_message_size_estimates() {
        let trigger = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };
        assert!(trigger.estimated_size() < 32);

        let table = Message::IpwTable {
            version: 1,
            data: [[1000; 16]; 16],
            crc32: 0,
        };
        assert!(table.estimated_size() > 500);
        assert!(table.requires_fragmentation());
    }

    #[test]
    fn test_message_equality() {
        let msg1 = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 12345,
        };

        let msg2 = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 12345,
        };

        assert_eq!(msg1, msg2);
    }
}
