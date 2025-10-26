//! ORIENT - Data Fusion & Context Building
//!
//! Transforms raw observations into meaningful engine context.
//! Combines multiple sensors, detects operating modes, and builds
//! a coherent picture of engine state.

use super::observe::{ObservationSet, Quality};
use super::types::{LoadMethod, SensorType};

/// Operating mode classification
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperatingMode {
    /// Engine shut down
    Shutdown,
    /// Cranking (starter engaged)
    Cranking,
    /// Idle
    Idle,
    /// Cruising (steady state)
    Cruise,
    /// Accelerating
    Acceleration,
    /// Decelerating
    Deceleration,
    /// Wide open throttle
    WideOpenThrottle,
}

/// Load estimate with method tracking
#[derive(Debug, Clone, Copy)]
pub struct LoadEstimate {
    /// MAP value (kPa)
    pub map_kpa: u16,
    /// TPS value (percent)
    pub tps_percent: u8,
    /// Calculated load (0-100%)
    pub calculated_load: u8,
    /// Method used for estimation
    pub method: LoadMethod,
}

/// Engine context (output of Orient phase)
#[derive(Debug, Clone, Copy)]
pub struct EngineContext {
    /// Timestamp when context was built
    pub timestamp_us: u32,
    /// Current operating mode
    pub operating_mode: OperatingMode,
    /// RPM (from trigger system)
    pub rpm: u16,
    /// Load estimate
    pub load: LoadEstimate,
    /// Coolant temperature (°C)
    pub coolant_temp_c: i16,
    /// Intake air temperature (°C)
    pub intake_temp_c: i16,
    /// Battery voltage (millivolts)
    pub battery_voltage_mv: u16,
    /// AFR/Lambda (if available)
    pub afr: Option<u16>,  // AFR * 10 (e.g., 147 = 14.7:1)
    /// Context confidence (0-255, 255 = perfect)
    pub confidence: u8,
}

impl EngineContext {
    /// Create default context (degraded mode)
    pub fn degraded(timestamp_us: u32) -> Self {
        Self {
            timestamp_us,
            operating_mode: OperatingMode::Shutdown,
            rpm: 0,
            load: LoadEstimate {
                map_kpa: 100,
                tps_percent: 0,
                calculated_load: 0,
                method: LoadMethod::Hybrid,
            },
            coolant_temp_c: 20,
            intake_temp_c: 20,
            battery_voltage_mv: 12000,
            afr: None,
            confidence: 0,
        }
    }
}

/// Orienter - builds context from observations
pub struct Orienter {
    /// Previous context (for rate-of-change detection)
    previous_context: Option<EngineContext>,
    /// RPM history for mode detection (last 3 samples)
    rpm_history: [u16; 3],
    rpm_history_index: usize,
}

impl Orienter {
    /// Create new orienter
    pub fn new() -> Self {
        Self {
            previous_context: None,
            rpm_history: [0; 3],
            rpm_history_index: 0,
        }
    }

    /// Build engine context from observations
    pub fn build_context(
        &mut self,
        observations: &ObservationSet,
    ) -> Result<EngineContext, OrientError> {
        if observations.is_empty() {
            return Err(OrientError::NoObservations);
        }

        // Extract sensor values (with fallbacks)
        let timestamp_us = self.extract_timestamp(observations);
        let rpm = self.extract_rpm(observations).unwrap_or(0);
        let map_kpa = self.extract_map(observations).unwrap_or(100);
        let tps_percent = self.extract_tps(observations).unwrap_or(0);
        let coolant_temp_c = self.extract_clt(observations).unwrap_or(20);
        let intake_temp_c = self.extract_iat(observations).unwrap_or(20);
        let battery_voltage_mv = self.extract_vbatt(observations).unwrap_or(12000);
        let afr = self.extract_afr(observations);

        // Update RPM history
        self.rpm_history[self.rpm_history_index] = rpm;
        self.rpm_history_index = (self.rpm_history_index + 1) % 3;

        // Calculate load
        let load = self.calculate_load(map_kpa, tps_percent);

        // Detect operating mode
        let operating_mode = self.detect_operating_mode(rpm, tps_percent, &load);

        // Calculate confidence
        let confidence = self.calculate_confidence(observations);

        let context = EngineContext {
            timestamp_us,
            operating_mode,
            rpm,
            load,
            coolant_temp_c,
            intake_temp_c,
            battery_voltage_mv,
            afr,
            confidence,
        };

        self.previous_context = Some(context);
        Ok(context)
    }

    /// Extract timestamp (use most recent)
    fn extract_timestamp(&self, observations: &ObservationSet) -> u32 {
        observations
            .iter()
            .map(|o| o.timestamp_us)
            .max()
            .unwrap_or(0)
    }

    /// Extract RPM (custom sensor or injected)
    fn extract_rpm(&self, observations: &ObservationSet) -> Option<u16> {
        observations
            .get(SensorType::Custom(0))  // Assume RPM is Custom(0)
            .and_then(|o| o.value.as_u16())
    }

    /// Extract MAP sensor
    fn extract_map(&self, observations: &ObservationSet) -> Option<u16> {
        observations
            .get(SensorType::MAP)
            .filter(|o| o.quality != Quality::Fault)
            .and_then(|o| o.value.as_u16())
    }

    /// Extract TPS sensor
    fn extract_tps(&self, observations: &ObservationSet) -> Option<u8> {
        observations
            .get(SensorType::TPS)
            .filter(|o| o.quality != Quality::Fault)
            .and_then(|o| o.value.as_u8())
    }

    /// Extract coolant temperature
    fn extract_clt(&self, observations: &ObservationSet) -> Option<i16> {
        observations
            .get(SensorType::CLT)
            .filter(|o| o.quality != Quality::Fault)
            .and_then(|o| o.value.as_i32())
            .map(|v| v as i16)
    }

    /// Extract intake air temperature
    fn extract_iat(&self, observations: &ObservationSet) -> Option<i16> {
        observations
            .get(SensorType::IAT)
            .filter(|o| o.quality != Quality::Fault)
            .and_then(|o| o.value.as_i32())
            .map(|v| v as i16)
    }

    /// Extract battery voltage
    fn extract_vbatt(&self, observations: &ObservationSet) -> Option<u16> {
        observations
            .get(SensorType::VBatt)
            .filter(|o| o.quality != Quality::Fault)
            .and_then(|o| o.value.as_u16())
    }

    /// Extract AFR/O2
    fn extract_afr(&self, observations: &ObservationSet) -> Option<u16> {
        observations
            .get(SensorType::O2)
            .filter(|o| o.quality != Quality::Fault)
            .and_then(|o| o.value.as_u16())
    }

    /// Calculate load estimate
    fn calculate_load(&self, map_kpa: u16, tps_percent: u8) -> LoadEstimate {
        // Simple load calculation (can be made more sophisticated)
        // Load = (MAP / atmospheric) * 100
        let atmospheric_kpa = 100;
        let map_based_load = ((map_kpa as u32 * 100) / atmospheric_kpa) as u8;

        // Use TPS as backup if MAP is not available
        let calculated_load = if map_kpa > 0 && map_kpa < 250 {
            map_based_load.min(100)
        } else {
            tps_percent
        };

        let method = if map_kpa > 0 {
            LoadMethod::MAP
        } else {
            LoadMethod::TPS
        };

        LoadEstimate {
            map_kpa,
            tps_percent,
            calculated_load,
            method,
        }
    }

    /// Detect operating mode
    fn detect_operating_mode(
        &self,
        rpm: u16,
        tps_percent: u8,
        _load: &LoadEstimate,
    ) -> OperatingMode {
        if rpm == 0 {
            return OperatingMode::Shutdown;
        }

        if rpm < 500 {
            return OperatingMode::Cranking;
        }

        if tps_percent >= 90 {
            return OperatingMode::WideOpenThrottle;
        }

        if rpm < 1200 && tps_percent < 10 {
            return OperatingMode::Idle;
        }

        // Check for acceleration/deceleration (needs history)
        if let Some(prev) = self.previous_context {
            let rpm_delta = rpm as i32 - prev.rpm as i32;

            if rpm_delta > 200 {
                return OperatingMode::Acceleration;
            } else if rpm_delta < -200 {
                return OperatingMode::Deceleration;
            }
        }

        OperatingMode::Cruise
    }

    /// Calculate confidence in context
    fn calculate_confidence(&self, observations: &ObservationSet) -> u8 {
        let mut confidence = 255u16;

        // Reduce confidence for each fault or missing critical sensor
        let critical_sensors = [
            SensorType::MAP,
            SensorType::TPS,
            SensorType::CLT,
        ];

        for sensor in &critical_sensors {
            if let Some(obs) = observations.get(*sensor) {
                match obs.quality {
                    Quality::Good => {}  // No reduction
                    Quality::Degraded => confidence -= 50,
                    Quality::Fault => confidence -= 100,
                }
            } else {
                confidence -= 80;  // Missing sensor
            }
        }

        confidence.min(255) as u8
    }
}

/// Orient errors
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OrientError {
    /// No observations available
    NoObservations,
    /// Insufficient data quality
    InsufficientQuality,
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::observe::{Observation, ObservationSource};
    use super::super::types::ObservationValue;

    #[test]
    fn test_orienter_creation() {
        let orienter = Orienter::new();
        assert!(orienter.previous_context.is_none());
    }

    #[test]
    fn test_build_context_empty_observations() {
        let mut orienter = Orienter::new();
        let observations = ObservationSet::new();

        let result = orienter.build_context(&observations);
        assert!(result.is_err());
    }

    #[test]
    fn test_build_context_with_observations() {
        let mut orienter = Orienter::new();
        let mut observations = ObservationSet::new();

        // Add some observations
        observations
            .add(Observation::new(
                SensorType::MAP,
                ObservationSource::DirectADC(0),
                1000,
                ObservationValue::U16(100),
            ))
            .unwrap();

        observations
            .add(Observation::new(
                SensorType::TPS,
                ObservationSource::DirectADC(1),
                1000,
                ObservationValue::U8(50),
            ))
            .unwrap();

        let result = orienter.build_context(&observations);
        assert!(result.is_ok());

        let context = result.unwrap();
        assert_eq!(context.load.map_kpa, 100);
        assert_eq!(context.load.tps_percent, 50);
    }

    #[test]
    fn test_operating_mode_detection() {
        let orienter = Orienter::new();
        let load = LoadEstimate {
            map_kpa: 100,
            tps_percent: 50,
            calculated_load: 100,
            method: LoadMethod::MAP,
        };

        // Shutdown
        assert_eq!(
            orienter.detect_operating_mode(0, 0, &load),
            OperatingMode::Shutdown
        );

        // Cranking
        assert_eq!(
            orienter.detect_operating_mode(300, 10, &load),
            OperatingMode::Cranking
        );

        // Idle
        assert_eq!(
            orienter.detect_operating_mode(800, 5, &load),
            OperatingMode::Idle
        );

        // WOT
        assert_eq!(
            orienter.detect_operating_mode(5000, 95, &load),
            OperatingMode::WideOpenThrottle
        );

        // Cruise
        assert_eq!(
            orienter.detect_operating_mode(3000, 40, &load),
            OperatingMode::Cruise
        );
    }

    #[test]
    fn test_load_calculation() {
        let orienter = Orienter::new();

        let load = orienter.calculate_load(100, 50);
        assert_eq!(load.calculated_load, 100);
        assert_eq!(load.method, LoadMethod::MAP);

        let load = orienter.calculate_load(50, 30);
        assert_eq!(load.calculated_load, 50);
    }
}
