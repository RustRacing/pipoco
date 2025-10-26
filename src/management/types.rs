//! Common types for management engine

/// Observation value types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObservationValue {
    /// Integer value (most common for embedded)
    Integer(i32),
    /// Boolean value (switches, flags)
    Boolean(bool),
    /// Unsigned 16-bit (typical sensor reading)
    U16(u16),
    /// Unsigned 8-bit (percentages, small values)
    U8(u8),
}

impl ObservationValue {
    /// Convert to i32 (common internal representation)
    pub fn as_i32(&self) -> Option<i32> {
        match self {
            ObservationValue::Integer(v) => Some(*v),
            ObservationValue::U16(v) => Some(*v as i32),
            ObservationValue::U8(v) => Some(*v as i32),
            ObservationValue::Boolean(v) => Some(if *v { 1 } else { 0 }),
        }
    }

    /// Convert to u16 (common for sensor readings)
    pub fn as_u16(&self) -> Option<u16> {
        match self {
            ObservationValue::U16(v) => Some(*v),
            ObservationValue::Integer(v) if *v >= 0 && *v <= u16::MAX as i32 => Some(*v as u16),
            ObservationValue::U8(v) => Some(*v as u16),
            _ => None,
        }
    }

    /// Convert to u8 (common for percentages)
    pub fn as_u8(&self) -> Option<u8> {
        match self {
            ObservationValue::U8(v) => Some(*v),
            ObservationValue::Integer(v) if *v >= 0 && *v <= u8::MAX as i32 => Some(*v as u8),
            ObservationValue::U16(v) if *v <= u8::MAX as u16 => Some(*v as u8),
            _ => None,
        }
    }

    /// Check if boolean true
    pub fn as_bool(&self) -> Option<bool> {
        match self {
            ObservationValue::Boolean(v) => Some(*v),
            _ => None,
        }
    }
}

/// Sensor types (extensible)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorType {
    /// Manifold Absolute Pressure
    MAP,
    /// Throttle Position Sensor
    TPS,
    /// Coolant Temperature
    CLT,
    /// Intake Air Temperature
    IAT,
    /// Oxygen Sensor (AFR)
    O2,
    /// Battery Voltage
    VBatt,
    /// Vehicle Speed
    VSS,
    /// Knock Sensor
    Knock,
    /// Cam Position
    CamPos,
    /// Custom sensor (user-defined)
    Custom(u8),
}

/// Load estimation method
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LoadMethod {
    /// MAP-based (speed-density)
    MAP,
    /// TPS-based (alpha-N)
    TPS,
    /// Hybrid (combine MAP and TPS)
    Hybrid,
}

/// Fuel calculation mode
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FuelMode {
    /// Open loop (no O2 correction)
    OpenLoop,
    /// Closed loop (O2 feedback active)
    ClosedLoop,
    /// Flood clear mode (no fuel)
    FloodClear,
    /// Cranking enrichment
    Cranking,
}

/// Idle control strategy
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IdleStrategy {
    /// Fixed position
    Fixed,
    /// PID control
    PID,
    /// Table-based
    Table,
}

/// Control algorithm type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum AlgorithmType {
    /// Traditional MAP-based speed-density
    SpeedDensity,
    /// TPS-based alpha-N (no MAP sensor)
    AlphaN,
    /// Mass airflow sensor
    MAF,
    /// Hybrid (multiple methods)
    Hybrid,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observation_value_conversions() {
        let val = ObservationValue::Integer(1000);
        assert_eq!(val.as_i32(), Some(1000));
        assert_eq!(val.as_u16(), Some(1000));

        let val = ObservationValue::U16(5000);
        assert_eq!(val.as_u16(), Some(5000));
        assert_eq!(val.as_i32(), Some(5000));

        let val = ObservationValue::U8(50);
        assert_eq!(val.as_u8(), Some(50));
        assert_eq!(val.as_u16(), Some(50));

        let val = ObservationValue::Boolean(true);
        assert_eq!(val.as_bool(), Some(true));
        assert_eq!(val.as_i32(), Some(1));
    }

    #[test]
    fn test_observation_value_out_of_range() {
        let val = ObservationValue::Integer(-100);
        assert!(val.as_u16().is_none());
        assert_eq!(val.as_i32(), Some(-100));

        let val = ObservationValue::U16(300);
        assert!(val.as_u8().is_none());
        assert_eq!(val.as_u16(), Some(300));
    }
}
