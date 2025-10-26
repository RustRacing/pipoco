//! Engine physics simulator
//!
//! Simulates engine behavior including:
//! - RPM changes based on throttle input
//! - Load (MAP) based on throttle and RPM
//! - Cranking behavior
//! - Realistic acceleration/deceleration curves

use super::{EngineConfig, EngineState};

/// Engine physics simulator
pub struct EngineSimulator {
    config: EngineConfig,
    state: EngineState,
    target_rpm: u16,
}

impl EngineSimulator {
    /// Create new engine simulator
    pub fn new(config: EngineConfig) -> Self {
        Self {
            config,
            state: EngineState::default(),
            target_rpm: 0,
        }
    }

    /// Get current engine state
    pub fn state(&self) -> &EngineState {
        &self.state
    }

    /// Get mutable engine state (for direct manipulation in tests)
    pub fn state_mut(&mut self) -> &mut EngineState {
        &mut self.state
    }

    /// Start cranking the engine
    pub fn start_cranking(&mut self) {
        self.state.cranking = true;
        self.target_rpm = 200;  // Typical cranking speed
    }

    /// Stop cranking
    pub fn stop_cranking(&mut self) {
        self.state.cranking = false;
        if !self.state.running {
            self.target_rpm = 0;
        }
    }

    /// Start engine running (fired and staying alive)
    pub fn start_running(&mut self) {
        self.state.running = true;
        self.state.cranking = false;
        self.target_rpm = self.config.idle_rpm;
    }

    /// Stop engine
    pub fn stop(&mut self) {
        self.state.running = false;
        self.state.cranking = false;
        self.target_rpm = 0;
    }

    /// Set throttle position (0-100%)
    pub fn set_throttle(&mut self, tps_percent: u16) {
        self.state.tps_percent = tps_percent.min(100);

        // Update target RPM based on throttle (simplified physics)
        if self.state.running {
            if tps_percent == 0 {
                self.target_rpm = self.config.idle_rpm;
            } else {
                // Linear throttle to RPM mapping (simplified)
                let rpm_range = self.config.max_rpm - self.config.idle_rpm;
                let rpm_delta = (rpm_range as u32 * tps_percent as u32) / 100;
                self.target_rpm = self.config.idle_rpm + rpm_delta as u16;
            }
        }
    }

    /// Set coolant temperature
    pub fn set_coolant_temp(&mut self, temp_c: i16) {
        self.state.coolant_temp_c = temp_c;
    }

    /// Set intake air temperature
    pub fn set_intake_temp(&mut self, temp_c: i16) {
        self.state.intake_temp_c = temp_c;
    }

    /// Set battery voltage
    pub fn set_battery_voltage(&mut self, voltage_mv: u16) {
        self.state.battery_voltage_mv = voltage_mv;
    }

    /// Update engine physics
    ///
    /// Should be called at regular intervals (e.g., every 10us) to simulate
    /// realistic engine behavior.
    ///
    /// # Arguments
    /// * `delta_us` - Time elapsed since last update (microseconds)
    pub fn update(&mut self, delta_us: u32) {
        // Update RPM towards target with realistic acceleration
        let rpm_delta = self.target_rpm as i32 - self.state.rpm as i32;

        if rpm_delta != 0 {
            // Calculate acceleration rate based on engine inertia
            // Higher inertia = slower acceleration
            let accel_rate = if rpm_delta > 0 {
                // Accelerating - limited by torque
                if self.state.cranking {
                    500_000  // Cranking: slow acceleration (500 RPM/sec)
                } else {
                    2_000_000  // Running: faster acceleration (2000 RPM/sec)
                }
            } else {
                // Decelerating - faster than acceleration
                3_000_000  // 3000 RPM/sec deceleration
            };

            // Calculate RPM change for this time step
            let rpm_change = (accel_rate as i64 * delta_us as i64) / 1_000_000;
            let rpm_change = rpm_change.clamp(-1000, 1000) as i32;

            // Apply change
            if rpm_delta.abs() <= rpm_change.abs() {
                // Close enough, snap to target
                self.state.rpm = self.target_rpm;
            } else {
                // Apply change in the direction of target
                let new_rpm = self.state.rpm as i32 + rpm_delta.signum() * rpm_change.abs();
                self.state.rpm = new_rpm.clamp(0, self.config.max_rpm as i32) as u16;
            }
        }

        // Update load (MAP) based on throttle and RPM
        // Simplified: more throttle = more load
        self.update_load();

        // Update crank angle based on RPM
        if self.state.rpm > 0 {
            // degrees per microsecond = (RPM * 360 / 60) / 1,000,000
            // = RPM * 6 / 1,000,000
            let degrees_per_us = (self.state.rpm as u32 * 6) as f64 / 1_000_000.0;
            let degrees = (degrees_per_us * delta_us as f64) as u32;
            self.state.crank_angle_deg =
                ((self.state.crank_angle_deg as u32 + degrees) % 720) as u16;
        }
    }

    /// Update manifold absolute pressure based on throttle and RPM
    fn update_load(&mut self) {
        if self.state.rpm == 0 {
            // Engine stopped - atmospheric pressure
            self.state.load_kpa = 100;
        } else {
            // Simplified model:
            // - Closed throttle at high RPM = low vacuum = ~30 kPa
            // - Open throttle = atmospheric ~100 kPa
            // - WOT = slightly above atmospheric

            let base_vacuum_kpa = 30;
            let atmospheric_kpa = 100;

            if self.state.tps_percent == 0 {
                // Idle/closed throttle - high vacuum
                // Higher RPM = more vacuum
                let vacuum_factor = (self.state.rpm as u32 * 100) / self.config.max_rpm as u32;
                let vacuum_kpa = base_vacuum_kpa + ((atmospheric_kpa - base_vacuum_kpa) * vacuum_factor as u16) / 100;
                self.state.load_kpa = vacuum_kpa.max(base_vacuum_kpa);
            } else {
                // Throttle open - load increases with throttle
                let load_range = atmospheric_kpa - base_vacuum_kpa;
                let load_delta = (load_range as u32 * self.state.tps_percent as u32) / 100;
                self.state.load_kpa = base_vacuum_kpa + load_delta as u16;
            }
        }
    }

    /// Get microseconds per tooth at current RPM for 60-2 wheel
    pub fn tooth_period_us(&self) -> u32 {
        if self.state.rpm == 0 {
            return 0;
        }

        // One revolution = 60,000,000 us / RPM
        // 58 teeth per revolution (60-2)
        // Time per tooth = revolution_time / 58
        let rev_time_us = 60_000_000_u32 / self.state.rpm as u32;
        rev_time_us / 58
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_engine_creation() {
        let config = EngineConfig::default();
        let engine = EngineSimulator::new(config);

        assert_eq!(engine.state().rpm, 0);
        assert!(!engine.state().running);
        assert!(!engine.state().cranking);
    }

    #[test]
    fn test_cranking() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);

        engine.start_cranking();
        assert!(engine.state().cranking);

        // Simulate cranking for 100ms
        for _ in 0..10000 {
            engine.update(10);
        }

        // Should reach cranking speed (~200 RPM)
        assert!(engine.state().rpm >= 150 && engine.state().rpm <= 250);
    }

    #[test]
    fn test_throttle_response() {
        let config = EngineConfig::default();
        let idle_rpm = config.idle_rpm;
        let max_rpm = config.max_rpm;
        let mut engine = EngineSimulator::new(config);

        engine.start_running();

        // Set 50% throttle
        engine.set_throttle(50);

        // Simulate for 1 second
        for _ in 0..100000 {
            engine.update(10);
        }

        // Should be somewhere between idle and max
        assert!(engine.state().rpm > idle_rpm);
        assert!(engine.state().rpm < max_rpm);
    }

    #[test]
    fn test_deceleration() {
        let config = EngineConfig::default();
        let idle_rpm = config.idle_rpm;
        let mut engine = EngineSimulator::new(config);

        engine.start_running();
        engine.set_throttle(100);

        // Rev to high RPM
        for _ in 0..100000 {
            engine.update(10);
        }

        let high_rpm = engine.state().rpm;

        // Close throttle
        engine.set_throttle(0);

        // Simulate for 1 second
        for _ in 0..100000 {
            engine.update(10);
        }

        // Should have decelerated back towards idle
        assert!(engine.state().rpm < high_rpm);
        assert!(engine.state().rpm >= idle_rpm - 200);
    }

    #[test]
    fn test_load_calculation() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);

        engine.start_running();

        // Closed throttle at idle should have low load (high vacuum)
        engine.set_throttle(0);
        engine.update(1000);
        let idle_load = engine.state().load_kpa;
        assert!(idle_load < 60, "Idle load should be <60 kPa (high vacuum)");

        // Wide open throttle should have high load (atmospheric)
        engine.set_throttle(100);
        engine.update(1000);
        let wot_load = engine.state().load_kpa;
        assert!(wot_load > 90, "WOT load should be >90 kPa (near atmospheric)");
    }

    #[test]
    fn test_tooth_period_calculation() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);

        engine.state_mut().rpm = 1000;
        let period = engine.tooth_period_us();

        // At 1000 RPM: 60,000,000 us / 1000 / 58 = ~1034 us
        assert!(period >= 1030 && period <= 1040);
    }

    #[test]
    fn test_engine_stop() {
        let config = EngineConfig::default();
        let mut engine = EngineSimulator::new(config);

        engine.start_running();
        engine.set_throttle(50);

        // Run for a bit
        for _ in 0..10000 {
            engine.update(10);
        }

        assert!(engine.state().rpm > 0);

        // Stop engine
        engine.stop();

        // Simulate for 1 second
        for _ in 0..100000 {
            engine.update(10);
        }

        // Should have stopped
        assert_eq!(engine.state().rpm, 0);
        assert!(!engine.state().running);
    }
}
