#![allow(dead_code)]
//! ECU Simulation Framework
//!
//! This module provides a complete software simulation environment for testing
//! the ECU without physical hardware. It simulates:
//!
//! - Engine physics (RPM, load, acceleration)
//! - Trigger wheel signal generation
//! - Sensor behavior (MAP, TPS, CLT, IAT, O2)
//! - Time progression
//! - Real-world scenarios (cold start, acceleration, etc.)
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────┐
//! │                    Simulation Framework                      │
//! ├─────────────────────────────────────────────────────────────┤
//! │                                                               │
//! │  ┌─────────────┐      ┌──────────────┐      ┌────────────┐ │
//! │  │   Engine    │─────▶│   Trigger    │─────▶│   ECU      │ │
//! │  │  Simulator  │      │  Generator   │      │   Core     │ │
//! │  └─────────────┘      └──────────────┘      └────────────┘ │
//! │        │                                           │         │
//! │        │                                           │         │
//! │        ▼                                           ▼         │
//! │  ┌─────────────┐                           ┌────────────┐  │
//! │  │   Sensor    │◀──────────────────────────│  Output    │  │
//! │  │  Simulator  │                           │  Capture   │  │
//! │  └─────────────┘                           └────────────┘  │
//! │                                                               │
//! └─────────────────────────────────────────────────────────────┘
//! ```
//!
//! ## Usage
//!
//! ```rust
//! use ecu_core::simulation::*;
//!
//! // Create simulated engine
//! let mut engine = EngineSimulator::new(EngineConfig::default());
//!
//! // Create ECU state
//! let mut ecu = EcuState::new();
//!
//! // Create test scenario
//! let mut scenario = ColdStartScenario::new(-10);
//!
//! // Run simulation
//! let result = scenario.run(&mut engine, &mut ecu);
//! assert!(result.started_successfully);
//! ```

use ecu_core::hal::TimeSource;
use ecu_core::{EcuState, TriggerDecoder};

// Re-export submodules
pub mod engine;
pub mod outputs;
pub mod scenarios;
pub mod sensors;
pub mod time;
pub mod trigger;

pub use engine::EngineSimulator;
pub use outputs::OutputCapture;
pub use sensors::SensorSimulator;
pub use time::SimulatedTime;
pub use trigger::TriggerGenerator;

/// Configuration for simulated engine
#[derive(Debug, Clone)]
pub struct EngineConfig {
    /// Number of cylinders
    pub cylinders: u8,
    /// Displacement in cc
    pub displacement_cc: u16,
    /// Maximum RPM
    pub max_rpm: u16,
    /// Idle RPM
    pub idle_rpm: u16,
    /// Engine inertia (affects acceleration rate)
    pub inertia_kg_m2: f32,
    /// Trigger pattern
    pub trigger_pattern: TriggerPattern,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self {
            cylinders: 4,
            displacement_cc: 2000,
            max_rpm: 6500,
            idle_rpm: 850,
            inertia_kg_m2: 0.15, // Typical 4-cylinder
            trigger_pattern: TriggerPattern::SixtyMinusTwo,
        }
    }
}

/// Supported trigger patterns
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerPattern {
    /// 60-2 trigger wheel
    SixtyMinusTwo,
    /// 36-1 trigger wheel (future)
    ThirtySixMinusOne,
    /// 24-1 trigger wheel (future)
    TwentyFourMinusOne,
}

/// Current state of simulated engine
#[derive(Debug, Clone)]
pub struct EngineState {
    /// Current RPM
    pub rpm: u16,
    /// Current load (kPa)
    pub load_kpa: u16,
    /// Current TPS (%)
    pub tps_percent: u16,
    /// Coolant temp (°C)
    pub coolant_temp_c: i16,
    /// Intake air temp (°C)
    pub intake_temp_c: i16,
    /// Battery voltage (mV)
    pub battery_voltage_mv: u16,
    /// Crank angle (0-719 degrees)
    pub crank_angle_deg: u16,
    /// Engine running state
    pub running: bool,
    /// Cranking (starter engaged)
    pub cranking: bool,
}

impl Default for EngineState {
    fn default() -> Self {
        Self {
            rpm: 0,
            load_kpa: 100, // Atmospheric
            tps_percent: 0,
            coolant_temp_c: 20,
            intake_temp_c: 20,
            battery_voltage_mv: 12500,
            crank_angle_deg: 0,
            running: false,
            cranking: false,
        }
    }
}

/// Result of a simulation run
#[derive(Debug)]
pub struct SimulationResult {
    /// Duration of simulation (microseconds)
    pub duration_us: u32,
    /// Number of engine cycles completed
    pub cycles_completed: u32,
    /// Average RPM during simulation
    pub avg_rpm: u16,
    /// Peak RPM during simulation
    pub peak_rpm: u16,
    /// Number of injections fired
    pub injection_count: u32,
    /// Average fuel pulse width (microseconds)
    pub avg_fuel_pw_us: u16,
    /// ECU achieved sync
    pub ecu_synced: bool,
    /// Any errors occurred
    pub errors: Vec<String>,
}

impl SimulationResult {
    pub fn new() -> Self {
        Self {
            duration_us: 0,
            cycles_completed: 0,
            avg_rpm: 0,
            peak_rpm: 0,
            injection_count: 0,
            avg_fuel_pw_us: 0,
            ecu_synced: false,
            errors: Vec::new(),
        }
    }
}

/// Main simulation coordinator
pub struct Simulator {
    engine: EngineSimulator,
    trigger: TriggerGenerator,
    sensors: SensorSimulator,
    time: SimulatedTime,
    outputs: OutputCapture,
}

impl Simulator {
    /// Create new simulator with default configuration
    pub fn new() -> Self {
        Self::with_config(EngineConfig::default())
    }

    /// Create new simulator with custom configuration
    pub fn with_config(config: EngineConfig) -> Self {
        Self {
            engine: EngineSimulator::new(config.clone()),
            trigger: TriggerGenerator::new(config.trigger_pattern),
            sensors: SensorSimulator::new(),
            time: SimulatedTime::new(),
            outputs: OutputCapture::new(),
        }
    }

    /// Get reference to engine simulator
    pub fn engine(&self) -> &EngineSimulator {
        &self.engine
    }

    /// Get mutable reference to engine simulator
    pub fn engine_mut(&mut self) -> &mut EngineSimulator {
        &mut self.engine
    }

    /// Get reference to sensor simulator
    pub fn sensors(&self) -> &SensorSimulator {
        &self.sensors
    }

    /// Get mutable reference to sensor simulator
    pub fn sensors_mut(&mut self) -> &mut SensorSimulator {
        &mut self.sensors
    }

    /// Get reference to time source
    pub fn time(&self) -> &SimulatedTime {
        &self.time
    }

    /// Get reference to output capture
    pub fn outputs(&self) -> &OutputCapture {
        &self.outputs
    }

    /// Get mutable reference to output capture
    pub fn outputs_mut(&mut self) -> &mut OutputCapture {
        &mut self.outputs
    }

    /// Run simulation for specified duration
    ///
    /// Advances simulation by stepping through time in small increments,
    /// generating trigger edges, updating sensors, and capturing outputs.
    ///
    /// # Arguments
    /// * `duration_us` - How long to run simulation in microseconds
    /// * `decoder` - Trigger decoder to feed events to
    /// * `ecu` - ECU state to test
    ///
    /// # Returns
    /// SimulationResult with statistics from the run
    pub fn run(
        &mut self,
        duration_us: u32,
        decoder: &mut TriggerDecoder<&SimulatedTime>,
        ecu: &mut EcuState,
    ) -> SimulationResult {
        let mut result = SimulationResult::new();
        result.duration_us = duration_us;

        let start_time = self.time.micros();
        let end_time = start_time + duration_us;

        // Simulation loop - step by 10us increments
        while self.time.micros() < end_time {
            // Update engine physics
            self.engine.update(10);

            // Generate trigger edges if any
            if let Some(edge_time) = self.trigger.next_edge(&self.engine, self.time.micros()) {
                // Advance time to edge
                self.time.set_micros(edge_time);

                // Process trigger edge through decoder
                decoder.tooth_edge();

                // Update ECU with current sensor values if synced
                if decoder.synced() {
                    let rpm = decoder.rpm();
                    let load = self.sensors.map_kpa();

                    // Calculate fuel
                    let pw = ecu.calculate_fuel(rpm, load);

                    // Record injection
                    self.outputs.record_injection(self.time.micros(), pw);
                    result.injection_count += 1;
                }
            } else {
                // No edge yet, advance time by step
                self.time.advance(10);
            }

            // Update sensors based on engine state
            self.sensors.update(self.engine.state());

            // Track statistics
            let current_rpm = self.engine.state().rpm;
            if current_rpm > result.peak_rpm {
                result.peak_rpm = current_rpm;
            }
        }

        // Calculate averages
        result.ecu_synced = decoder.synced();
        result.avg_rpm = self.engine.state().rpm;
        if result.injection_count > 0 {
            result.avg_fuel_pw_us = self.outputs.average_injection_pw();
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_simulator_creation() {
        let sim = Simulator::new();
        assert_eq!(sim.engine().state().rpm, 0);
        assert!(!sim.engine().state().running);
    }

    #[test]
    fn test_engine_config_default() {
        let config = EngineConfig::default();
        assert_eq!(config.cylinders, 4);
        assert_eq!(config.displacement_cc, 2000);
        assert_eq!(config.max_rpm, 6500);
        assert_eq!(config.idle_rpm, 850);
    }

    #[test]
    fn test_simulation_result_creation() {
        let result = SimulationResult::new();
        assert_eq!(result.duration_us, 0);
        assert_eq!(result.cycles_completed, 0);
        assert!(!result.ecu_synced);
        assert_eq!(result.errors.len(), 0);
    }
}
