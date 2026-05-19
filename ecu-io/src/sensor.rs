//! Sensor frame types and sensor frame source trait.
//!
//! Provides fixed-size, simulator-independent IO contracts for sensor data.

use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, Rpm};

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
    /// Whether lambda reading is valid.
    pub lambda_valid: bool,
    /// Lambda value * 100 (e.g., 142 = 1.42).
    pub lambda_x100: Lambda100,
}

/// Source of sensor frames (e.g., ADC or CAN sensor bus).
pub trait SensorFrameSource {
    type Error;

    /// Get the next sensor frame, if available.
    fn next_frame(&mut self) -> Result<Option<SensorFrame>, Self::Error>;
}
