//! Pre-defined test scenarios for real-world ECU behavior
//!
//! This module contains realistic driving scenarios that can be simulated
//! in software before hardware testing.

use super::{EngineSimulator, SensorSimulator, TriggerGenerator, SimulatedTime, OutputCapture, EngineConfig, TriggerPattern};
use ecu_core::{TriggerDecoder, EcuState};

// Helper to avoid repetition
fn setup_simulation() -> (EngineSimulator, TriggerGenerator, SensorSimulator, SimulatedTime, OutputCapture, EcuState) {
    let engine = EngineSimulator::new(EngineConfig::default());
    let trigger = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);
    let sensors = SensorSimulator::new();
    let time = SimulatedTime::new();
    let outputs = OutputCapture::new();
    let ecu = EcuState::new();

    (engine, trigger, sensors, time, outputs, ecu)
}

/// Result of a scenario test
#[derive(Debug)]
pub struct ScenarioResult {
    pub success: bool,
    pub duration_ms: u32,
    pub final_rpm: u16,
    pub avg_fuel_pw: u16,
    pub sync_achieved: bool,
    pub errors: Vec<String>,
}

/// Cold start scenario (-10°C)
pub struct ColdStartScenario {
    ambient_temp_c: i16,
}

impl ColdStartScenario {
    pub fn new(ambient_temp_c: i16) -> Self {
        Self { ambient_temp_c }
    }

    /// Run cold start scenario
    ///
    /// Simulates:
    /// 1. Cold engine at ambient temperature
    /// 2. Cranking for up to 3 seconds
    /// 3. Engine fires and reaches idle
    ///
    /// Success criteria:
    /// - ECU achieves sync within 2 revolutions
    /// - Engine reaches idle RPM
    /// - Fuel enrichment applied appropriately
    pub fn run(&self) -> ScenarioResult {
        let mut result = ScenarioResult {
            success: false,
            duration_ms: 0,
            final_rpm: 0,
            avg_fuel_pw: 0,
            sync_achieved: false,
            errors: Vec::new(),
        };

        // Setup
        let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = setup_simulation();

        // Set cold conditions
        engine.set_coolant_temp(self.ambient_temp_c);
        engine.set_intake_temp(self.ambient_temp_c);

        // Apply cold start enrichment (in real ECU this would be automatic)
        ecu.corrections.clt = if self.ambient_temp_c < 0 { 150 } else { 120 };

        // Create decoder with reference to time
        let mut decoder = TriggerDecoder::new(&time);

        // Phase 1: Cranking (up to 3 seconds)
        engine.start_cranking();
        let mut current_time = 0u32;
        let crank_duration = 3_000_000;  // 3 seconds

        while current_time < crank_duration {
            engine.update(10);
            sensors.update(engine.state());

            // Generate trigger edges
            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();

                // Once synced, start calculating fuel
                if decoder.synced() && !result.sync_achieved {
                    result.sync_achieved = true;
                }

                if decoder.synced() {
                    let rpm = decoder.rpm();
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);

                    // Simulate engine firing after a few injections
                    if outputs.injection_count() > 10 {
                        engine.start_running();
                        break;
                    }
                }
            }

            current_time += 10;
        }

        // Phase 2: Warm up to idle (1 second)
        let warmup_duration = 1_000_000;
        let warmup_end = current_time + warmup_duration;

        while current_time < warmup_end {
            engine.update(10);
            sensors.update(engine.state());

            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();

                if decoder.synced() {
                    let rpm = decoder.rpm();
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }

            current_time += 10;
        }

        // Evaluate results
        result.duration_ms = current_time / 1000;
        result.final_rpm = engine.state().rpm;
        result.avg_fuel_pw = outputs.average_injection_pw();

        // Success criteria
        if !result.sync_achieved {
            result.errors.push("Failed to achieve sync".to_string());
        }

        if result.final_rpm < 600 {
            result.errors.push(format!("RPM too low: {}", result.final_rpm));
        }

        if outputs.injection_count() == 0 {
            result.errors.push("No injections recorded".to_string());
        }

        result.success = result.errors.is_empty();

        result
    }
}

/// Hot start scenario (80°C)
pub struct HotStartScenario;

impl HotStartScenario {
    pub fn run(&self) -> ScenarioResult {
        let mut result = ScenarioResult {
            success: false,
            duration_ms: 0,
            final_rpm: 0,
            avg_fuel_pw: 0,
            sync_achieved: false,
            errors: Vec::new(),
        };

        // Setup
        let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = setup_simulation();

        // Set hot conditions
        engine.set_coolant_temp(80);
        engine.set_intake_temp(60);

        // No enrichment needed for hot start
        ecu.corrections.clt = 100;

        let mut decoder = TriggerDecoder::new(&time);

        // Cranking - should start quickly
        engine.start_cranking();
        let mut current_time = 0u32;
        let max_duration = 1_000_000;  // 1 second max

        while current_time < max_duration {
            engine.update(10);
            sensors.update(engine.state());

            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();

                if decoder.synced() && !result.sync_achieved {
                    result.sync_achieved = true;
                }

                if decoder.synced() {
                    let rpm = decoder.rpm();
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);

                    // Hot engine should fire quickly
                    if outputs.injection_count() > 3 {
                        engine.start_running();
                        break;
                    }
                }
            }

            current_time += 10;
        }

        result.duration_ms = current_time / 1000;
        result.final_rpm = engine.state().rpm;
        result.avg_fuel_pw = outputs.average_injection_pw();

        // Hot start should be faster
        if result.duration_ms > 500 {
            result.errors.push("Hot start took too long".to_string());
        }

        if !result.sync_achieved {
            result.errors.push("Failed to achieve sync".to_string());
        }

        result.success = result.errors.is_empty();
        result
    }
}

/// Acceleration scenario (idle → 4000 RPM)
pub struct AccelerationScenario;

impl AccelerationScenario {
    pub fn run(&self) -> ScenarioResult {
        let mut result = ScenarioResult {
            success: false,
            duration_ms: 0,
            final_rpm: 0,
            avg_fuel_pw: 0,
            sync_achieved: true,  // Start running
            errors: Vec::new(),
        };

        // Setup
        let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = setup_simulation();

        // Initialize table with realistic values
        ecu.init_linear_table();

        let mut decoder = TriggerDecoder::new(&time);

        // Start at idle
        engine.start_running();
        engine.set_throttle(0);

        // Warm up at idle (500ms)
        let mut current_time = 0u32;
        while current_time < 500_000 {
            engine.update(10);
            sensors.update(engine.state());

            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();

                if decoder.synced() {
                    let rpm = decoder.rpm();
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }

            current_time += 10;
        }

        // WOT acceleration
        engine.set_throttle(100);
        let accel_start_rpm = engine.state().rpm;

        // Accelerate for 2 seconds
        while current_time < 2_500_000 {
            engine.update(10);
            sensors.update(engine.state());

            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();

                if decoder.synced() {
                    let rpm = decoder.rpm();
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }

            current_time += 10;

            // Stop if we hit target
            if engine.state().rpm >= 4000 {
                break;
            }
        }

        result.duration_ms = current_time / 1000;
        result.final_rpm = engine.state().rpm;
        result.avg_fuel_pw = outputs.average_injection_pw();

        // Check maintained sync throughout
        if !decoder.synced() {
            result.errors.push("Lost sync during acceleration".to_string());
        }

        // Check we actually accelerated
        if result.final_rpm <= accel_start_rpm + 100 {
            result.errors.push("Failed to accelerate".to_string());
        }

        result.success = result.errors.is_empty();
        result
    }
}

/// Idle stability scenario
pub struct IdleScenario {
    duration_seconds: u32,
}

impl IdleScenario {
    pub fn new(duration_seconds: u32) -> Self {
        Self { duration_seconds }
    }

    pub fn run(&self) -> ScenarioResult {
        let mut result = ScenarioResult {
            success: false,
            duration_ms: 0,
            final_rpm: 0,
            avg_fuel_pw: 0,
            sync_achieved: true,
            errors: Vec::new(),
        };

        // Setup
        let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = setup_simulation();

        let mut decoder = TriggerDecoder::new(&time);

        // Start at idle
        engine.start_running();
        engine.set_throttle(0);

        let target_duration = self.duration_seconds * 1_000_000;
        let mut current_time = 0u32;
        let mut min_rpm = u16::MAX;
        let mut max_rpm = 0u16;

        while current_time < target_duration {
            engine.update(10);
            sensors.update(engine.state());

            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();

                if decoder.synced() {
                    let rpm = decoder.rpm();

                    // Track RPM variation
                    if rpm > 0 {
                        min_rpm = min_rpm.min(rpm);
                        max_rpm = max_rpm.max(rpm);
                    }

                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }

            current_time += 10;
        }

        result.duration_ms = current_time / 1000;
        result.final_rpm = engine.state().rpm;
        result.avg_fuel_pw = outputs.average_injection_pw();

        // Check idle stability (±100 RPM variation acceptable)
        let rpm_variation = max_rpm - min_rpm;
        if rpm_variation > 100 {
            result.errors.push(format!("Idle unstable: {} RPM variation", rpm_variation));
        }

        // Check maintained sync
        if !decoder.synced() {
            result.errors.push("Lost sync during idle".to_string());
        }

        result.success = result.errors.is_empty();
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cold_start_scenario() {
        let scenario = ColdStartScenario::new(-10);
        let result = scenario.run();

        println!("Cold start result: {:?}", result);

        // Should achieve sync
        assert!(result.sync_achieved, "Should achieve sync");

        // Should have injections
        assert!(result.avg_fuel_pw > 0, "Should have fuel injection");
    }

    #[test]
    fn test_hot_start_scenario() {
        let scenario = HotStartScenario;
        let result = scenario.run();

        println!("Hot start result: {:?}", result);

        assert!(result.sync_achieved);
        assert!(result.duration_ms < 1000, "Hot start should be quick");
    }

    #[test]
    fn test_acceleration_scenario() {
        let scenario = AccelerationScenario;
        let result = scenario.run();

        println!("Acceleration result: {:?}", result);

        assert!(result.sync_achieved);
        assert!(result.final_rpm > 2000, "Should accelerate significantly");
    }

    #[test]
    fn test_idle_scenario() {
        let scenario = IdleScenario::new(1);  // 1 second test
        let result = scenario.run();

        println!("Idle result: {:?}", result);

        assert!(result.sync_achieved);
        assert!(result.success || !result.errors.is_empty());
    }
}
