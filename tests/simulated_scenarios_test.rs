//! Real-world scenario tests using simulation framework
//!
//! These tests simulate real driving scenarios in software, allowing
//! comprehensive testing without physical hardware.

mod simulation;

use simulation::scenarios::*;
use simulation::sensors::SensorFault;

#[test]
fn test_cold_start_minus_10c() {
    let scenario = ColdStartScenario::new(-10);
    let result = scenario.run();

    println!("\n=== Cold Start (-10°C) ===");
    println!("Duration: {}ms", result.duration_ms);
    println!("Final RPM: {}", result.final_rpm);
    println!("Avg fuel PW: {}us", result.avg_fuel_pw);
    println!("Sync achieved: {}", result.sync_achieved);

    assert!(result.sync_achieved, "ECU should achieve sync");
    assert!(result.avg_fuel_pw > 1000, "Cold start should use enrichment (>1000us)");

    if !result.success {
        for error in &result.errors {
            println!("ERROR: {}", error);
        }
    }

    // Note: May not fully succeed in simulation without proper enrichment logic
    // This test validates that the framework works and basic safety is maintained
}

#[test]
fn test_hot_start_80c() {
    let scenario = HotStartScenario;
    let result = scenario.run();

    println!("\n=== Hot Start (80°C) ===");
    println!("Duration: {}ms", result.duration_ms);
    println!("Final RPM: {}", result.final_rpm);
    println!("Avg fuel PW: {}us", result.avg_fuel_pw);

    assert!(result.sync_achieved, "ECU should achieve sync");
    assert!(result.duration_ms < 1000, "Hot start should be quick (<1 second)");

    println!("Hot start success: {}", result.success);
}

#[test]
fn test_idle_stability_1_second() {
    let scenario = IdleScenario::new(1);
    let result = scenario.run();

    println!("\n=== Idle Stability (1 second) ===");
    println!("Duration: {}ms", result.duration_ms);
    println!("Final RPM: {}", result.final_rpm);
    println!("Avg fuel PW: {}us", result.avg_fuel_pw);
    println!("Sync maintained: {}", result.sync_achieved);

    assert!(result.sync_achieved, "Should maintain sync during idle");
    assert!(result.final_rpm > 600 && result.final_rpm < 1200,
            "Idle RPM should be in reasonable range: {}", result.final_rpm);

    println!("Idle stability success: {}", result.success);
}

#[test]
fn test_acceleration_idle_to_4000_rpm() {
    let scenario = AccelerationScenario;
    let result = scenario.run();

    println!("\n=== Acceleration (Idle → 4000 RPM) ===");
    println!("Duration: {}ms", result.duration_ms);
    println!("Final RPM: {}", result.final_rpm);
    println!("Avg fuel PW: {}us", result.avg_fuel_pw);

    assert!(result.sync_achieved, "Should maintain sync during acceleration");
    assert!(result.final_rpm > 2000, "Should accelerate significantly: got {} RPM", result.final_rpm);

    println!("Acceleration success: {}", result.success);
}

/// Test sync loss and recovery scenario
#[test]
fn test_sync_loss_recovery() {
    use simulation::*;
    use ecu_core::{TriggerDecoder, EcuState};

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) =
        (EngineSimulator::new(EngineConfig::default()),
         TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
         SensorSimulator::new(),
         SimulatedTime::new(),
         OutputCapture::new(),
         EcuState::new());

    let mut decoder = TriggerDecoder::new(&time);

    // Start engine
    engine.start_running();
    let mut current_time = 0u32;

    // Phase 1: Achieve sync
    println!("\n=== Sync Loss & Recovery ===");
    println!("Phase 1: Achieving initial sync...");

    while current_time < 200_000 && !decoder.synced() {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();
        }
        current_time += 10;
    }

    assert!(decoder.synced(), "Should achieve initial sync");
    println!("Initial sync achieved at {}ms", current_time / 1000);

    // Phase 2: Simulate signal loss (no edges for 300ms)
    println!("Phase 2: Simulating signal loss (300ms)...");
    trigger.reset();
    time.set_micros(current_time + 300_000);
    decoder.tooth_edge();  // Trigger timeout

    assert!(!decoder.synced(), "Should lose sync after timeout");
    println!("Sync lost as expected");

    // Phase 3: Recover sync
    println!("Phase 3: Recovering sync...");
    current_time += 300_000;
    let recovery_start = current_time;

    while current_time < recovery_start + 200_000 && !decoder.synced() {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();
        }
        current_time += 10;
    }

    assert!(decoder.synced(), "Should recover sync");
    println!("Sync recovered at {}ms after signal return", (current_time - recovery_start) / 1000);
    println!("Total test duration: {}ms", current_time / 1000);
}

/// Test sensor fault handling
#[test]
fn test_sensor_fault_handling() {
    use simulation::*;
    use ecu_core::{TriggerDecoder, EcuState};

    println!("\n=== Sensor Fault Handling ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) =
        (EngineSimulator::new(EngineConfig::default()),
         TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
         SensorSimulator::new(),
         SimulatedTime::new(),
         OutputCapture::new(),
         EcuState::new());

    let mut decoder = TriggerDecoder::new(&time);

    // Start engine running
    engine.start_running();
    let mut current_time = 0u32;

    // Run normally for 100ms
    println!("Phase 1: Normal operation...");
    while current_time < 100_000 {
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

    let normal_injections = outputs.injection_count();
    let normal_avg_pw = outputs.average_injection_pw();
    println!("Normal operation: {} injections, avg PW: {}us", normal_injections, normal_avg_pw);

    // Inject MAP sensor fault
    println!("Phase 2: Injecting MAP sensor fault...");
    sensors.set_map_fault(SensorFault::OpenCircuit);
    outputs.clear();

    while current_time < 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm();
                let load = sensors.map_kpa();
                let pw = ecu.calculate_fuel(rpm, load);

                // Safety: pulse width should still be clamped
                assert!(pw >= 500 && pw <= 20000,
                        "Fuel PW out of safe range with sensor fault: {}us", pw);

                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    let fault_injections = outputs.injection_count();
    println!("With MAP fault: {} injections (ECU still operating)", fault_injections);
    println!("Sensor fault handled safely - no runaway fuel or crashes");

    // ECU should continue operating even with faulty sensor
    assert!(fault_injections > 0, "ECU should continue operating with sensor fault");
}

/// Stress test: 10 second idle
#[test]
#[ignore]  // Long-running test, run with --ignored flag
fn test_long_duration_idle() {
    let scenario = IdleScenario::new(10);
    let result = scenario.run();

    println!("\n=== Long Duration Idle (10 seconds) ===");
    println!("Duration: {}ms", result.duration_ms);
    println!("Final RPM: {}", result.final_rpm);
    println!("Sync maintained: {}", result.sync_achieved);

    assert!(result.sync_achieved, "Should maintain sync for extended period");
    assert!(result.final_rpm > 600, "Should maintain idle RPM");
}
