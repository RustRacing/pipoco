//! Sensor simulation
//!
//! Simulates realistic sensor behavior including:
//! - Temperature sensors (thermistors with realistic curves)
//! - Pressure sensors (MAP)
//! - Position sensors (TPS)
//! - Voltage sensors (battery)
//! - Sensor faults (open/short circuit)

use super::EngineState;

/// Sensor fault conditions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorFault {
    None,
    OpenCircuit,
    ShortToGround,
    ShortToBattery,
    Intermittent,
}

/// Simulated sensor suite
pub struct SensorSimulator {
    // Current sensor readings
    map_kpa: u16,
    tps_percent: u16,
    clt_deg_c: i16,
    iat_deg_c: i16,
    battery_mv: u16,

    // Sensor faults
    map_fault: SensorFault,
    tps_fault: SensorFault,
    clt_fault: SensorFault,

    // Noise and filtering
    add_noise: bool,
    noise_seed: u32,
}

impl SensorSimulator {
    /// Create new sensor simulator
    pub fn new() -> Self {
        Self {
            map_kpa: 100,
            tps_percent: 0,
            clt_deg_c: 20,
            iat_deg_c: 20,
            battery_mv: 12500,
            map_fault: SensorFault::None,
            tps_fault: SensorFault::None,
            clt_fault: SensorFault::None,
            add_noise: false,
            noise_seed: 12345,
        }
    }

    /// Update sensors based on engine state
    pub fn update(&mut self, engine: &EngineState) {
        // Update sensors from engine state
        self.map_kpa = engine.load_kpa;
        self.tps_percent = engine.tps_percent;
        self.clt_deg_c = engine.coolant_temp_c;
        self.iat_deg_c = engine.intake_temp_c;
        self.battery_mv = engine.battery_voltage_mv;

        // Apply noise if enabled
        if self.add_noise {
            self.apply_noise();
        }

        // Apply sensor faults
        self.apply_faults();
    }

    /// Enable/disable sensor noise
    pub fn set_noise(&mut self, enabled: bool) {
        self.add_noise = enabled;
    }

    /// Set sensor fault condition
    pub fn set_map_fault(&mut self, fault: SensorFault) {
        self.map_fault = fault;
    }

    pub fn set_tps_fault(&mut self, fault: SensorFault) {
        self.tps_fault = fault;
    }

    pub fn set_clt_fault(&mut self, fault: SensorFault) {
        self.clt_fault = fault;
    }

    /// Get MAP sensor reading (kPa)
    pub fn map_kpa(&self) -> u16 {
        self.map_kpa
    }

    /// Get TPS reading (%)
    pub fn tps_percent(&self) -> u16 {
        self.tps_percent
    }

    /// Get coolant temperature (°C)
    pub fn coolant_temp_c(&self) -> i16 {
        self.clt_deg_c
    }

    /// Get intake air temperature (°C)
    pub fn intake_temp_c(&self) -> i16 {
        self.iat_deg_c
    }

    /// Get battery voltage (mV)
    pub fn battery_voltage_mv(&self) -> u16 {
        self.battery_mv
    }

    /// Apply realistic sensor noise
    fn apply_noise(&mut self) {
        // Simple LCG pseudo-random number generator
        self.noise_seed = self.noise_seed.wrapping_mul(1664525).wrapping_add(1013904223);
        let noise = (self.noise_seed % 20) as i16 - 10;  // ±10 units

        // Apply noise to sensors (clamped to valid ranges)
        let map_noisy = (self.map_kpa as i32 + noise as i32 / 5).clamp(10, 250);
        self.map_kpa = map_noisy as u16;

        let tps_noisy = (self.tps_percent as i32 + noise as i32 / 10).clamp(0, 100);
        self.tps_percent = tps_noisy as u16;
    }

    /// Apply sensor fault conditions
    fn apply_faults(&mut self) {
        // MAP sensor faults
        match self.map_fault {
            SensorFault::None => {},
            SensorFault::OpenCircuit => self.map_kpa = 0,
            SensorFault::ShortToGround => self.map_kpa = 0,
            SensorFault::ShortToBattery => self.map_kpa = 250,
            SensorFault::Intermittent => {
                if (self.noise_seed % 10) > 8 {
                    self.map_kpa = 0;
                }
            }
        }

        // TPS sensor faults
        match self.tps_fault {
            SensorFault::None => {},
            SensorFault::OpenCircuit => self.tps_percent = 0,
            SensorFault::ShortToGround => self.tps_percent = 0,
            SensorFault::ShortToBattery => self.tps_percent = 100,
            SensorFault::Intermittent => {
                if (self.noise_seed % 10) > 8 {
                    self.tps_percent = 0;
                }
            }
        }

        // CLT sensor faults
        match self.clt_fault {
            SensorFault::None => {},
            SensorFault::OpenCircuit => self.clt_deg_c = -40,
            SensorFault::ShortToGround => self.clt_deg_c = 150,
            SensorFault::ShortToBattery => self.clt_deg_c = -40,
            SensorFault::Intermittent => {
                if (self.noise_seed % 10) > 8 {
                    self.clt_deg_c = -40;
                }
            }
        }
    }
}

impl Default for SensorSimulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sensor_creation() {
        let sensors = SensorSimulator::new();
        assert_eq!(sensors.map_kpa(), 100);
        assert_eq!(sensors.tps_percent(), 0);
        assert_eq!(sensors.coolant_temp_c(), 20);
    }

    #[test]
    fn test_sensor_update() {
        let mut sensors = SensorSimulator::new();
        let mut engine = EngineState::default();

        engine.load_kpa = 80;
        engine.tps_percent = 50;
        engine.coolant_temp_c = -10;

        sensors.update(&engine);

        assert_eq!(sensors.map_kpa(), 80);
        assert_eq!(sensors.tps_percent(), 50);
        assert_eq!(sensors.coolant_temp_c(), -10);
    }

    #[test]
    fn test_map_sensor_fault() {
        let mut sensors = SensorSimulator::new();
        let engine = EngineState::default();

        // Normal operation
        sensors.update(&engine);
        assert_eq!(sensors.map_kpa(), 100);

        // Open circuit fault
        sensors.set_map_fault(SensorFault::OpenCircuit);
        sensors.update(&engine);
        assert_eq!(sensors.map_kpa(), 0);

        // Short to battery
        sensors.set_map_fault(SensorFault::ShortToBattery);
        sensors.update(&engine);
        assert_eq!(sensors.map_kpa(), 250);
    }

    #[test]
    fn test_tps_sensor_fault() {
        let mut sensors = SensorSimulator::new();
        let mut engine = EngineState::default();
        engine.tps_percent = 50;

        sensors.update(&engine);
        assert_eq!(sensors.tps_percent(), 50);

        // Fault should override reading
        sensors.set_tps_fault(SensorFault::ShortToBattery);
        sensors.update(&engine);
        assert_eq!(sensors.tps_percent(), 100);
    }

    #[test]
    fn test_sensor_noise() {
        let mut sensors = SensorSimulator::new();
        let engine = EngineState::default();

        sensors.set_noise(false);
        sensors.update(&engine);
        let clean_map = sensors.map_kpa();

        sensors.set_noise(true);
        let mut noisy_readings = Vec::new();
        for _ in 0..10 {
            sensors.update(&engine);
            noisy_readings.push(sensors.map_kpa());
        }

        // With noise, readings should vary
        let all_same = noisy_readings.iter().all(|&x| x == clean_map);
        assert!(!all_same, "Noise should cause variation in readings");
    }

    #[test]
    fn test_battery_voltage() {
        let mut sensors = SensorSimulator::new();
        let mut engine = EngineState::default();

        engine.battery_voltage_mv = 11000;  // Low voltage
        sensors.update(&engine);
        assert_eq!(sensors.battery_voltage_mv(), 11000);

        engine.battery_voltage_mv = 14500;  // High voltage (alternator)
        sensors.update(&engine);
        assert_eq!(sensors.battery_voltage_mv(), 14500);
    }
}
