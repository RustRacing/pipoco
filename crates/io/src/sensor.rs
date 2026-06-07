//! Sensor frame types and sensor frame source trait.
//!
//! Provides fixed-size, simulator-independent IO contracts for sensor data.

use ecu_domain::{
    CamPhaseDeg10, Degrees10, KnockLevelX100, Kpa10, Lambda100, MassAirFlowX100, Micros, Rpm,
    VehicleSpeedKph10,
};

/// A complete sensor frame at a point in time.
///
/// All fields use integer units at the ECU boundary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorFrame {
    /// Timestamp in microseconds.
    pub at_us: Micros,
    /// Engine speed.
    pub rpm: Rpm,
    /// Manifold absolute pressure * 10.
    pub map_kpa10: Kpa10,
    /// Optional mass air flow reading in source-native units * 100.
    pub maf_x100: MassAirFlowX100,
    /// Whether the MAF/HFM reading is from a wired, valid source.
    pub maf_valid: bool,
    /// Optional normalized knock level * 100.
    pub knock_x100: KnockLevelX100,
    /// Whether the knock level is from a valid knock front-end/window.
    pub knock_valid: bool,
    /// Optional measured cam phase for VVT/cam timing diagnostics and control.
    pub cam_phase_deg10: Option<CamPhaseDeg10>,
    /// Crank angle at sample time in degrees * 10.
    pub angle_x10: Degrees10,
    /// Throttle position * 100 (0-10000 represents 0-100%).
    pub tps_x100: u16,
    /// Coolant temperature in Celsius * 10.
    pub clt_c10: i16,
    /// Intake air temperature in Celsius * 10.
    pub iat_c10: i16,
    /// Battery voltage in millivolts.
    pub vbatt_mv: u16,
    /// Barometric pressure * 10.
    pub baro_kpa10: Kpa10,
    /// Vehicle speed in km/h * 10.
    pub vehicle_speed_kph10: VehicleSpeedKph10,
    /// Whether vehicle speed is from a valid VSS input.
    pub vehicle_speed_valid: bool,
    /// Whether lambda reading is valid.
    pub lambda_valid: bool,
    /// Lambda value * 100 (e.g., 142 = 1.42).
    pub lambda_x100: Lambda100,
}

/// Packed validity flags for optional sensor channels at external boundaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SensorValidityFlags(u8);

impl SensorValidityFlags {
    pub const MAF: u8 = 1 << 0;
    pub const KNOCK: u8 = 1 << 1;
    pub const VEHICLE_SPEED: u8 = 1 << 2;
    pub const LAMBDA: u8 = 1 << 3;

    pub const fn new(bits: u8) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn contains(self, bit: u8) -> bool {
        (self.0 & bit) != 0
    }

    pub const fn from_frame(frame: SensorFrame) -> Self {
        let mut bits = 0;
        if frame.maf_valid {
            bits |= Self::MAF;
        }
        if frame.knock_valid {
            bits |= Self::KNOCK;
        }
        if frame.vehicle_speed_valid {
            bits |= Self::VEHICLE_SPEED;
        }
        if frame.lambda_valid {
            bits |= Self::LAMBDA;
        }
        Self(bits)
    }
}

/// Source of sensor frames (e.g., ADC or CAN sensor bus).
pub trait SensorFrameSource {
    type Error;

    /// Get the next sensor frame, if available.
    fn next_frame(&mut self) -> Result<Option<SensorFrame>, Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE_FRAME: SensorFrame = SensorFrame {
        at_us: Micros::new(0),
        rpm: Rpm::new(0),
        map_kpa10: Kpa10::new(0),
        maf_x100: MassAirFlowX100::new(0),
        maf_valid: false,
        knock_x100: KnockLevelX100::new(0),
        knock_valid: false,
        cam_phase_deg10: None,
        angle_x10: Degrees10::new(0),
        tps_x100: 0,
        clt_c10: 0,
        iat_c10: 0,
        vbatt_mv: 0,
        baro_kpa10: Kpa10::new(0),
        vehicle_speed_kph10: VehicleSpeedKph10::new(0),
        vehicle_speed_valid: false,
        lambda_valid: false,
        lambda_x100: Lambda100::new(0),
    };

    #[test]
    fn validity_flags_pack_optional_sensor_validity() {
        let flags = SensorValidityFlags::from_frame(SensorFrame {
            maf_valid: true,
            knock_valid: true,
            vehicle_speed_valid: true,
            lambda_valid: true,
            ..BASE_FRAME
        });

        assert_eq!(
            flags.bits(),
            SensorValidityFlags::MAF
                | SensorValidityFlags::KNOCK
                | SensorValidityFlags::VEHICLE_SPEED
                | SensorValidityFlags::LAMBDA
        );
        assert!(flags.contains(SensorValidityFlags::MAF));
        assert!(flags.contains(SensorValidityFlags::KNOCK));
        assert!(flags.contains(SensorValidityFlags::VEHICLE_SPEED));
        assert!(flags.contains(SensorValidityFlags::LAMBDA));
    }

    #[test]
    fn validity_flags_stay_clear_for_unwired_optional_sensors() {
        let flags = SensorValidityFlags::from_frame(BASE_FRAME);

        assert_eq!(flags.bits(), 0);
        assert!(!flags.contains(SensorValidityFlags::MAF));
        assert!(!flags.contains(SensorValidityFlags::KNOCK));
        assert!(!flags.contains(SensorValidityFlags::VEHICLE_SPEED));
        assert!(!flags.contains(SensorValidityFlags::LAMBDA));
    }

    #[test]
    fn validity_flags_can_decode_external_bits() {
        let flags = SensorValidityFlags::new(SensorValidityFlags::MAF | SensorValidityFlags::KNOCK);

        assert!(flags.contains(SensorValidityFlags::MAF));
        assert!(flags.contains(SensorValidityFlags::KNOCK));
        assert!(!flags.contains(SensorValidityFlags::VEHICLE_SPEED));
        assert!(!flags.contains(SensorValidityFlags::LAMBDA));
    }
}
