//! OBSERVE - Sensor Input Abstraction
//!
//! This module provides generic sensor input handling with:
//! - Multiple input sources (ADC, CAN, SPI, computed)
//! - Quality tracking (good, degraded, fault)
//! - Validation and filtering
//! - Extensible sensor registry

use super::types::{ObservationValue, SensorType};

/// Observation source identification
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObservationSource {
    /// Direct ADC channel
    DirectADC(u8),
    /// CAN bus message
    CANBus(u32),
    /// SPI device
    SPI(u8),
    /// Computed/derived value
    Computed,
    /// Injected (test/simulation)
    Injected,
}

/// Observation quality indicator
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Quality {
    /// Good quality, within expected range
    Good,
    /// Degraded (noisy, near limits)
    Degraded,
    /// Fault detected (out of range, stuck, missing)
    Fault,
}

/// Single observation with metadata
#[derive(Debug, Clone, Copy)]
pub struct Observation {
    pub sensor: SensorType,
    pub source: ObservationSource,
    pub timestamp_us: u32,
    pub value: ObservationValue,
    pub quality: Quality,
}

impl Observation {
    /// Create new observation
    pub fn new(
        sensor: SensorType,
        source: ObservationSource,
        timestamp_us: u32,
        value: ObservationValue,
    ) -> Self {
        Self {
            sensor,
            source,
            timestamp_us,
            value,
            quality: Quality::Good, // Default, can be updated by validator
        }
    }

    /// Update quality assessment
    pub fn set_quality(&mut self, quality: Quality) {
        self.quality = quality;
    }
}

/// Collection of observations from one cycle
pub struct ObservationSet {
    observations: [Option<Observation>; 32], // Up to 32 sensors
    count: usize,
}

impl ObservationSet {
    /// Create empty observation set
    pub fn new() -> Self {
        Self {
            observations: [None; 32],
            count: 0,
        }
    }

    /// Add observation to set
    pub fn add(&mut self, obs: Observation) -> Result<(), ObserveError> {
        if self.count >= 32 {
            return Err(ObserveError::BufferFull);
        }
        self.observations[self.count] = Some(obs);
        self.count += 1;
        Ok(())
    }

    /// Get observation by sensor type
    pub fn get(&self, sensor: SensorType) -> Option<&Observation> {
        self.observations[..self.count]
            .iter()
            .filter_map(|o| o.as_ref())
            .find(|o| o.sensor == sensor)
    }

    /// Get all observations
    pub fn iter(&self) -> impl Iterator<Item = &Observation> {
        self.observations[..self.count]
            .iter()
            .filter_map(|o| o.as_ref())
    }

    /// Count observations
    pub fn len(&self) -> usize {
        self.count
    }

    /// Check if empty
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
}

impl Default for ObservationSet {
    fn default() -> Self {
        Self::new()
    }
}

/// Observer - manages sensor inputs
pub struct Observer {
    // Sensor validators (range checking)
    validators: [(SensorType, Option<Validator>); 16],
}

impl Observer {
    /// Create new observer
    pub fn new() -> Self {
        Self {
            validators: [
                (SensorType::MAP, Some(Validator::new(20, 250))), // 20-250 kPa
                (SensorType::TPS, Some(Validator::new(0, 100))),  // 0-100%
                (SensorType::CLT, Some(Validator::new(-40, 150))), // -40 to 150°C
                (SensorType::IAT, Some(Validator::new(-40, 100))), // -40 to 100°C
                (SensorType::VBatt, Some(Validator::new(8000, 18000))), // 8-18V (mv)
                (SensorType::O2, None),                           // Variable based on sensor type
                (SensorType::VSS, None),                          // Variable
                (SensorType::Knock, None),
                (SensorType::CamPos, None),
                (SensorType::Custom(0), None),
                (SensorType::Custom(1), None),
                (SensorType::Custom(2), None),
                (SensorType::Custom(3), None),
                (SensorType::Custom(4), None),
                (SensorType::Custom(5), None),
                (SensorType::Custom(6), None),
            ],
        }
    }

    /// Collect all available observations
    ///
    /// In a real implementation, this would poll all registered sources.
    /// For now, it returns an empty set (sources must be explicitly added).
    pub fn collect_all(&self, _current_time_us: u32) -> Result<ObservationSet, ObserveError> {
        // In real implementation:
        // - Poll ADC channels
        // - Check CAN mailboxes
        // - Read SPI sensors
        // - Compute derived values
        Ok(ObservationSet::new())
    }

    /// Add observation (from external source)
    pub fn observe(
        &self,
        mut obs: Observation,
        set: &mut ObservationSet,
    ) -> Result<(), ObserveError> {
        // Validate observation
        if let Some(validator) = self.get_validator(obs.sensor) {
            obs.quality = validator.validate(&obs.value);
        }

        set.add(obs)
    }

    /// Get validator for sensor type
    fn get_validator(&self, sensor: SensorType) -> Option<&Validator> {
        self.validators
            .iter()
            .find(|(s, _)| *s == sensor)
            .and_then(|(_, v)| v.as_ref())
    }
}

impl Default for Observer {
    fn default() -> Self {
        Self::new()
    }
}

/// Validator for range checking
struct Validator {
    min: i32,
    max: i32,
}

impl Validator {
    fn new(min: i32, max: i32) -> Self {
        Self { min, max }
    }

    fn validate(&self, value: &ObservationValue) -> Quality {
        if let Some(v) = value.as_i32() {
            if v < self.min || v > self.max {
                Quality::Fault
            } else if v < self.min + (self.max - self.min) / 10
                || v > self.max - (self.max - self.min) / 10
            {
                Quality::Degraded // Near limits
            } else {
                Quality::Good
            }
        } else {
            Quality::Fault
        }
    }
}

/// Observe errors
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObserveError {
    /// Observation buffer full
    BufferFull,
    /// Sensor timeout
    Timeout,
    /// Invalid source
    InvalidSource,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_observation_creation() {
        let obs = Observation::new(
            SensorType::MAP,
            ObservationSource::DirectADC(0),
            1000,
            ObservationValue::U16(100),
        );

        assert_eq!(obs.sensor, SensorType::MAP);
        assert_eq!(obs.quality, Quality::Good);
    }

    #[test]
    fn test_observation_set() {
        let mut set = ObservationSet::new();

        let obs1 = Observation::new(
            SensorType::MAP,
            ObservationSource::DirectADC(0),
            1000,
            ObservationValue::U16(100),
        );

        let obs2 = Observation::new(
            SensorType::TPS,
            ObservationSource::DirectADC(1),
            1000,
            ObservationValue::U8(50),
        );

        set.add(obs1).unwrap();
        set.add(obs2).unwrap();

        assert_eq!(set.len(), 2);
        assert!(set.get(SensorType::MAP).is_some());
        assert!(set.get(SensorType::TPS).is_some());
        assert!(set.get(SensorType::CLT).is_none());
    }

    #[test]
    fn test_validator() {
        let validator = Validator::new(0, 100);

        assert_eq!(validator.validate(&ObservationValue::U8(50)), Quality::Good);
        assert_eq!(
            validator.validate(&ObservationValue::U8(5)),
            Quality::Degraded
        );
        assert_eq!(
            validator.validate(&ObservationValue::U8(150)),
            Quality::Fault
        );
    }

    #[test]
    fn test_observer_validation() {
        let observer = Observer::new();
        let mut set = ObservationSet::new();

        // Valid MAP reading
        let obs = Observation::new(
            SensorType::MAP,
            ObservationSource::DirectADC(0),
            1000,
            ObservationValue::U16(100),
        );
        observer.observe(obs, &mut set).unwrap();

        assert_eq!(set.get(SensorType::MAP).unwrap().quality, Quality::Good);

        // Invalid MAP reading (too high)
        let mut set2 = ObservationSet::new();
        let obs = Observation::new(
            SensorType::MAP,
            ObservationSource::DirectADC(0),
            1000,
            ObservationValue::U16(300), // > 250 kPa max
        );
        observer.observe(obs, &mut set2).unwrap();

        assert_eq!(set2.get(SensorType::MAP).unwrap().quality, Quality::Fault);
    }
}
