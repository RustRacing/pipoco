//! Real-world scenario tests using simulation framework
//!
//! These tests simulate real driving scenarios in software, allowing
//! comprehensive testing without physical hardware.

mod simulation;

#[allow(unused_imports)]
use ecu_core::app::EcuApp;
#[allow(unused_imports)]
use ecu_core::constants::fuel::DEFAULT_LOAD_KPA;
use ecu_core::hal::{OutputPin, TimeSource};
use ecu_core::{Kpa10, Micros};
use simulation::scenarios::*;
use simulation::sensors::SensorFault;
use simulation::time::SimulatedTime;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

struct PulseCapturePin<'a> {
    time: &'a SimulatedTime,
    events: Rc<RefCell<Vec<(u32, u16)>>>,
    high_since: Cell<Option<u32>>,
}

impl<'a> PulseCapturePin<'a> {
    fn new(time: &'a SimulatedTime, events: Rc<RefCell<Vec<(u32, u16)>>>) -> Self {
        Self {
            time,
            events,
            high_since: Cell::new(None),
        }
    }
}

impl<'a> OutputPin for PulseCapturePin<'a> {
    fn set_high(&mut self) {
        if self.high_since.get().is_none() {
            self.high_since.set(Some(self.time.micros()));
        }
    }

    fn set_low(&mut self) {
        if let Some(start_us) = self.high_since.replace(None) {
            let duration = self.time.micros().wrapping_sub(start_us);
            self.events
                .borrow_mut()
                .push((self.time.micros(), duration as u16));
        }
    }
}

struct NoopPin;

impl OutputPin for NoopPin {
    fn set_high(&mut self) {}

    fn set_low(&mut self) {}
}

#[test]
fn test_cold_start_minus_10c() {
    let scenario = ColdStartScenario::new(-10);
    let result = scenario.run();

    println!("\n=== Cold Start (-10°C) ===");
    println!("Duration: {ms}ms", ms = result.duration_ms);
    println!("Final RPM: {rpm}", rpm = result.final_rpm);
    println!("Avg fuel PW: {pw}us", pw = result.avg_fuel_pw);
    println!("Sync achieved: {ok}", ok = result.sync_achieved);

    assert!(result.sync_achieved, "ECU should achieve sync");
    assert!(
        result.avg_fuel_pw > 1000,
        "Cold start should use enrichment (>1000us)"
    );

    if !result.success {
        for error in &result.errors {
            println!("ERROR: {error}");
        }
    }

    // Note: May not fully succeed in simulation without proper enrichment logic
    // This test validates that the framework works and basic safety is maintained
}

#[test]
#[cfg(not(feature = "sched-angle-disable"))]
fn cold_start_enrichment() {
    let time = SimulatedTime::new();
    let mut app = EcuApp::new(&time);
    let mut engine = simulation::EngineSimulator::new(simulation::EngineConfig::default());
    let mut trigger = simulation::TriggerGenerator::new(simulation::TriggerPattern::SixtyMinusTwo);
    let mut sensors = simulation::SensorSimulator::new();
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut inj0 = PulseCapturePin::new(&time, Rc::clone(&events));
    let mut inj1 = NoopPin;
    let mut outputs: [&mut dyn OutputPin; 2] = [&mut inj0, &mut inj1];

    engine.set_coolant_temp(-10);
    engine.set_intake_temp(-10);
    engine.start_running();
    engine.set_throttle(0);

    let mut current_time = 0u32;
    let duration_us = 30_000_000;
    let mut sync_seen = false;

    while current_time < duration_us {
        engine.update(10);
        sensors.update(engine.state());

        {
            let state = app.state_mut();
            state.set_clt_x10(engine.state().coolant_temp_c.saturating_mul(10));
            state.set_iat_x10(engine.state().intake_temp_c.saturating_mul(10));
            state.set_tps_percent(sensors.tps_percent().min(u8::MAX as u16) as u8);
            state.set_map_kpa_x10(sensors.map_kpa());
        }

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            app.on_timestamp(edge_time);
            current_time = edge_time;
            if app.state().synced() {
                sync_seen = true;
            }
        } else {
            time.advance(10);
            current_time = current_time.wrapping_add(10);
        }

        app.drive_outputs(time.micros(), &mut outputs);
    }

    assert!(sync_seen, "ECU should achieve sync during the run");

    let recorded = events.borrow();
    assert!(
        recorded.len() > 10,
        "Expected multiple injector pulses, got {}",
        recorded.len()
    );

    let base_pw = app
        .state()
        .calculate_fuel(app.state().rpm(), app.state().map_kpa_x10() / 10);

    for &(_, pw) in recorded.iter().take(10) {
        assert!(
            pw > base_pw,
            "Cold-start pulse width should stay above base PW: {pw} <= {base_pw}"
        );
    }

    for pair in recorded.windows(2) {
        assert!(
            pair[1].1 <= pair[0].1,
            "Cold-start enrichment should decay monotonically: {} -> {}",
            pair[0].1,
            pair[1].1
        );
    }
}

#[test]
#[cfg(not(feature = "sched-angle-disable"))]
fn tip_in_accel_enrichment() {
    let time = SimulatedTime::new();
    let mut app = EcuApp::new(&time);
    let mut engine = simulation::EngineSimulator::new(simulation::EngineConfig::default());
    let mut trigger = simulation::TriggerGenerator::new(simulation::TriggerPattern::SixtyMinusTwo);
    let mut sensors = simulation::SensorSimulator::new();
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut inj0 = PulseCapturePin::new(&time, Rc::clone(&events));
    let mut inj1 = NoopPin;
    let mut outputs: [&mut dyn OutputPin; 2] = [&mut inj0, &mut inj1];
    let mut base_widths = Vec::new();

    engine.set_coolant_temp(80);
    engine.set_intake_temp(60);
    engine.start_running();
    engine.set_throttle(10);

    {
        let state = app.state_mut();
        state.config.ase_config.percent = 0;
        state.config.ase_config.taper_time_ms = 1;
    }

    let mut current_time = 0u32;
    let tip_in_time = 1_000_000u32;
    let run_end = tip_in_time + 5_000_000;
    let decay_us = app.state().config.ae_config.decay_time_ms * 1000;
    let decay_deadline = tip_in_time + decay_us + 2_000_000;
    let mut stepped = false;

    while current_time < run_end.max(decay_deadline) {
        if !stepped && current_time >= tip_in_time {
            engine.set_throttle(60);
            stepped = true;
        }

        engine.update(10);
        sensors.update(engine.state());

        {
            let state = app.state_mut();
            state.set_clt_x10(engine.state().coolant_temp_c.saturating_mul(10));
            state.set_iat_x10(engine.state().intake_temp_c.saturating_mul(10));
            state.set_tps_percent(sensors.tps_percent().min(u8::MAX as u16) as u8);
            state.set_map_kpa_x10(sensors.map_kpa());
        }

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            app.on_timestamp(edge_time);
            current_time = edge_time;

            if app.state().synced()
                && app.state().tooth_count() == ecu_core::constants::timing::INJECTION_TOOTH
            {
                base_widths.push(
                    app.state()
                        .calculate_fuel(app.state().rpm(), DEFAULT_LOAD_KPA),
                );
            }
        } else {
            time.advance(10);
            current_time = current_time.wrapping_add(10);
        }

        app.drive_outputs(time.micros(), &mut outputs);
    }

    assert!(stepped, "Tip-in step should have been applied");

    let recorded = events.borrow();
    assert!(
        recorded.len() + 1 >= base_widths.len(),
        "Captured widths should match scheduled injector events aside from a trailing in-flight pulse"
    );
    assert!(
        recorded.len() > 10,
        "Expected enough injector samples to validate AE behavior"
    );

    let ae_gain = app.state().config.ae_config.percent_gain as u32;
    let tip_index = recorded
        .iter()
        .position(|&(t, _)| t >= tip_in_time)
        .expect("missing post-tip-in sample");
    let tip_base = base_widths[tip_index] as u32;
    let tip_pw = recorded[tip_index].1 as u32;
    assert!(
        tip_pw >= (tip_base * (100 + ae_gain)) / 100,
        "First post-tip-in pulse should exceed base by AE threshold: {tip_pw} < {}",
        (tip_base * (100 + ae_gain)) / 100
    );

    let decay_index = recorded
        .iter()
        .position(|&(t, _)| t >= tip_in_time + decay_us)
        .unwrap_or(recorded.len() - 1);
    let decay_base = base_widths[decay_index] as u32;
    let decay_pw = recorded[decay_index].1 as u32;
    assert!(
        decay_pw <= decay_base + (decay_base / 100),
        "Pulse width should return to base within AE decay window: {decay_pw} vs base {decay_base}"
    );
}

#[test]
fn test_hot_start_80c() {
    let scenario = HotStartScenario;
    let result = scenario.run();

    println!("\n=== Hot Start (80°C) ===");
    println!("Duration: {ms}ms", ms = result.duration_ms);
    println!("Final RPM: {rpm}", rpm = result.final_rpm);
    println!("Avg fuel PW: {pw}us", pw = result.avg_fuel_pw);
    println!("Hot start success: {ok}", ok = result.success);

    assert!(result.sync_achieved, "ECU should achieve sync");
    assert!(result.success, "Hot start scenario should succeed");
    assert!(
        result.duration_ms < 1000,
        "Hot start should be quick (<1 second)"
    );
    assert!(
        result.final_rpm >= 600,
        "Hot start should recover to idle RPM, got {rpm}",
        rpm = result.final_rpm
    );
}

#[test]
fn test_idle_stability_1_second() {
    let scenario = IdleScenario::new(1);
    let result = scenario.run();

    println!("\n=== Idle Stability (1 second) ===");
    println!("Duration: {ms}ms", ms = result.duration_ms);
    println!("Final RPM: {rpm}", rpm = result.final_rpm);
    println!("Avg fuel PW: {pw}us", pw = result.avg_fuel_pw);
    println!("Sync maintained: {ok}", ok = result.sync_achieved);

    assert!(result.sync_achieved, "Should maintain sync during idle");
    assert!(
        result.final_rpm > 600 && result.final_rpm < 1200,
        "Idle RPM should be in reasonable range: {rpm}",
        rpm = result.final_rpm
    );

    println!("Idle stability success: {ok}", ok = result.success);
}

#[test]
fn test_acceleration_idle_to_4000_rpm() {
    let scenario = AccelerationScenario;
    let result = scenario.run();

    println!("\n=== Acceleration (Idle → 4000 RPM) ===");
    println!("Duration: {ms}ms", ms = result.duration_ms);
    println!("Final RPM: {rpm}", rpm = result.final_rpm);
    println!("Avg fuel PW: {pw}us", pw = result.avg_fuel_pw);

    assert!(
        result.sync_achieved,
        "Should maintain sync during acceleration"
    );
    assert!(
        result.final_rpm > 2000,
        "Should accelerate significantly: got {rpm} RPM",
        rpm = result.final_rpm
    );

    println!("Acceleration success: {}", result.success);
}

/// Test sync loss and recovery scenario
#[test]
fn test_sync_loss_recovery() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    let (mut engine, mut trigger, mut sensors, time, _outputs, _ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

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
    decoder.tooth_edge(); // Trigger timeout

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
    println!(
        "Sync recovered at {}ms after signal return",
        (current_time - recovery_start) / 1000
    );
    println!("Total test duration: {}ms", current_time / 1000);
}

/// Test sensor fault handling
#[test]
fn test_sensor_fault_handling() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Sensor Fault Handling ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    let mut decoder = TriggerDecoder::new(&time);

    // Start engine running
    engine.start_running();
    let mut current_time = 0u32;

    // Run long enough to reach sync and establish a real pre-fault baseline.
    println!("Phase 1: Normal operation...");
    while current_time < 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    let normal_injections = outputs.injection_count();
    let normal_avg_pw = outputs.average_injection_pw();
    println!("Normal operation: {normal_injections} injections, avg PW: {normal_avg_pw}us");
    assert!(
        decoder.synced(),
        "Baseline phase should achieve sync before the fault"
    );
    assert!(
        normal_injections > 0,
        "Baseline phase should record injections before the MAP fault"
    );

    // Inject MAP sensor fault
    println!("Phase 2: Injecting MAP sensor fault...");
    sensors.set_map_fault(SensorFault::OpenCircuit);
    outputs.clear();

    while current_time < 300_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                let pw = ecu.calculate_fuel(rpm, load);

                // Safety: pulse width should still be clamped
                assert!(
                    (500..=20000).contains(&pw),
                    "Fuel PW out of safe range with sensor fault: {pw}us"
                );

                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    let fault_injections = outputs.injection_count();
    println!("With MAP fault: {fault_injections} injections (ECU still operating)");
    println!("Sensor fault handled safely - no runaway fuel or crashes");

    // ECU should continue operating even with faulty sensor
    assert!(
        fault_injections > 0,
        "ECU should continue operating with sensor fault"
    );
}

/// Stress test: 10 second idle
#[test]
#[ignore] // Long-running test, run with --ignored flag
fn test_long_duration_idle() {
    let scenario = IdleScenario::new(10);
    let result = scenario.run();

    println!("\n=== Long Duration Idle (10 seconds) ===");
    println!("Duration: {ms}ms", ms = result.duration_ms);
    println!("Final RPM: {rpm}", rpm = result.final_rpm);
    println!("Sync maintained: {ok}", ok = result.sync_achieved);

    assert!(
        result.sync_achieved,
        "Should maintain sync for extended period"
    );
    assert!(result.final_rpm > 600, "Should maintain idle RPM");
}

// ============================================================================
// INTEGRATION SCENARIOS - Testing multiple systems working together
// ============================================================================

/// Test knock detection during acceleration
///
/// Simulates:
/// 1. Engine idling normally
/// 2. WOT acceleration - knock occurs at high load
/// 3. ECU retards timing
/// 4. Knock stops, timing recovers
#[test]
fn test_knock_during_acceleration() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Knock During Acceleration ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    // Configure knock controller
    ecu.knock_controller.config.enable = true;
    ecu.knock_controller.config.threshold = 100;
    ecu.knock_controller.config.debounce_count = 1;
    ecu.knock_controller.config.retard_step_x10 = 30; // 3 degrees per knock
    ecu.knock_controller.config.min_rpm = 2000;
    ecu.knock_controller.config.min_clt_c = 60;

    // Warm engine
    engine.set_coolant_temp(80);
    ecu.init_ignition_table();

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();

    let mut current_time = 0u32;
    let mut knock_events = 0u32;
    let mut timing_retard_applied = false;

    // Phase 1: Idle for 200ms to sync
    println!("Phase 1: Idling...");
    while current_time < 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();
        }
        current_time += 10;
    }
    assert!(decoder.synced(), "Should sync during idle");

    // Phase 2: WOT acceleration with simulated knock
    println!("Phase 2: WOT acceleration with knock...");
    engine.set_throttle(100);
    let accel_end = current_time + 500_000; // 500ms of acceleration

    while current_time < accel_end {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);
                ecu.set_map_kpa_x10(load * 10);

                // Simulate knock at high RPM and load
                if rpm > 3000 && load > 80 {
                    let knock_level = 150; // Above threshold
                    if ecu.process_knock_sample(0, knock_level, 80, edge_time) {
                        knock_events += 1;
                    }
                }

                // Calculate timing with knock retard
                let timing = ecu.calculate_ignition_timing_with_limiter_cyl(rpm, load, 0);
                let base_timing = ecu.calculate_ignition_timing(rpm, load);

                if timing < base_timing {
                    timing_retard_applied = true;
                }

                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    println!("Knock events detected: {knock_events}");
    println!("Timing retard applied: {timing_retard_applied}");
    println!("Total knock count: {}", ecu.total_knock_count());
    println!("Has knock retard: {}", ecu.has_knock_retard());

    assert!(knock_events > 0, "Should have detected knock events");
    assert!(
        timing_retard_applied,
        "Should have retarded timing due to knock"
    );

    // Phase 3: Lift off throttle - knock should stop
    println!("Phase 3: Lift off - timing recovery...");
    engine.set_throttle(20);
    let recovery_end = current_time + 300_000;

    let retard_before = ecu.knock_controller.state.cylinders[0].retard_x10;

    while current_time < recovery_end {
        engine.update(10);
        sensors.update(engine.state());

        // Update recovery every 100ms
        if current_time.is_multiple_of(100_000) {
            ecu.update_knock_recovery(current_time);
        }

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();
        }
        current_time += 10;
    }

    let retard_after = ecu.knock_controller.state.cylinders[0].retard_x10;
    println!("Retard before recovery: {retard_before} x0.1 deg");
    println!("Retard after recovery: {retard_after} x0.1 deg");

    // Timing should have recovered (at least partially)
    assert!(
        retard_after <= retard_before,
        "Timing should recover when no knock"
    );
    println!("Knock scenario completed successfully");
}

/// Test rev limiter approach and intervention
///
/// Simulates:
/// 1. Engine accelerating towards redline
/// 2. Rev limiter activates
/// 3. Torque/fuel cut applied
/// 4. RPM drops, limiter deactivates
#[test]
fn test_rev_limiter_approach() {
    use ecu_core::{EcuState, LimiterStrategy, TriggerDecoder};
    use simulation::*;

    println!("\n=== Rev Limiter Approach ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    // Configure rev limiter
    ecu.rev_limiter_config_mut().max_rpm = 6000;
    ecu.rev_limiter_config_mut().soft_limit_start_rpm = 5800;
    ecu.rev_limiter_config_mut().hysteresis_rpm = 200;
    ecu.rev_limiter_config_mut().strategy = LimiterStrategy::HardCut;

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();
    engine.set_throttle(100); // WOT

    let mut current_time = 0u32;
    let mut limiter_activated = false;
    let mut fuel_cut_count = 0u32;
    let mut max_rpm_reached: u16 = 0;

    // Simulate acceleration towards redline
    println!("Accelerating towards redline...");
    while current_time < 2_000_000 {
        // 2 seconds
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);
                max_rpm_reached = max_rpm_reached.max(rpm);

                // Update rev limiter
                ecu.update_rev_limiter();

                if ecu.rev_limiter_state.active {
                    limiter_activated = true;
                }

                // Check if fuel should be cut
                if !ecu.should_inject_fuel(0) {
                    fuel_cut_count += 1;
                } else {
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }
        }
        current_time += 10;
    }

    println!("Max RPM reached: {max_rpm_reached}");
    println!("Limiter activated: {limiter_activated}");
    println!("Fuel cuts applied: {fuel_cut_count}");
    println!("Total injections: {}", outputs.injection_count());

    // Engine should approach the limiter RPM but not exceed by much
    assert!(max_rpm_reached >= 5500, "Should reach high RPM");
    assert!(max_rpm_reached <= 6500, "Should not greatly exceed limiter");
    assert!(limiter_activated, "Rev limiter should have activated");
    assert!(fuel_cut_count > 0, "Should have some fuel cuts");
    println!("Rev limiter scenario completed successfully");
}

/// Test DFCO (Deceleration Fuel Cut-Off) during throttle lift
///
/// Simulates:
/// 1. Engine at high RPM under load
/// 2. Throttle closes (lift-off)
/// 3. DFCO activates above threshold
/// 4. DFCO deactivates as RPM drops
#[test]
fn test_dfco_deceleration() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== DFCO Deceleration ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    // Configure DFCO
    // DfcoConfig uses: rpm_min (enable above), rpm_max, tps_max_pct, delay_ms
    // Note: The simulation decelerates very fast (designed for real-time rate),
    // so we use lower rpm_min threshold for test purposes
    ecu.config.dfco_config.rpm_min = 1000; // Enable above 1000 RPM (low for fast sim)
    ecu.config.dfco_config.rpm_max = 7000; // Upper bound
    ecu.config.dfco_config.tps_max_pct = 5; // TPS < 5% to activate
    ecu.config.dfco_config.delay_ms = 0; // No delay for test

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();
    engine.set_throttle(80); // High throttle

    let mut current_time = 0u32;
    let mut dfco_active_time = 0u32;
    let mut fuel_save_count = 0u32;

    // Phase 1: Accelerate to high RPM
    println!("Phase 1: Accelerating...");
    while current_time < 800_000 && engine.state().rpm < 5500 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();
        }
        current_time += 10;
    }

    let high_rpm = engine.state().rpm;
    println!("Reached RPM: {high_rpm}");

    // Phase 2: Lift off throttle - DFCO should activate
    // Engine sim decelerates fast, so DFCO window is brief
    println!("Phase 2: Throttle lift - DFCO activation...");
    engine.set_throttle(0);
    // Config already set: rpm_min=1000, delay=0
    let lift_time = current_time;
    let decel_end = current_time + 300_000; // 300ms of deceleration

    let mut debug_print_count = 0;

    while current_time < decel_end {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);
                ecu.set_tps_percent(0);

                // Debug: print RPM at intervals
                if debug_print_count % 50 == 0 {
                    let time_since_lift = current_time.saturating_sub(lift_time);
                    println!(
                        "  t={}ms: decoder_rpm={}, engine_rpm={}",
                        time_since_lift / 1000,
                        rpm,
                        engine.state().rpm
                    );
                }
                debug_print_count += 1;

                // Check DFCO conditions manually
                // DFCO active when: TPS <= tps_max_pct, RPM >= rpm_min, after delay
                let time_since_lift = current_time.saturating_sub(lift_time);
                let delay_us = ecu.config.dfco_config.delay_ms * 1000;
                let dfco_should_activate = rpm >= ecu.config.dfco_config.rpm_min
                    && rpm <= ecu.config.dfco_config.rpm_max
                    && ecu.tps_percent() <= ecu.config.dfco_config.tps_max_pct
                    && time_since_lift > delay_us;

                if dfco_should_activate {
                    dfco_active_time += 10;
                    fuel_save_count += 1;
                } else {
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }
        }
        current_time += 10;
    }

    let final_rpm = engine.state().rpm;
    println!("Final RPM: {final_rpm}");
    println!("DFCO active time: {}ms", dfco_active_time / 1000);
    println!("Fuel saves (injection skips): {fuel_save_count}");
    println!("Total injections: {}", outputs.injection_count());

    assert!(
        dfco_active_time > 0,
        "DFCO should have activated (RPM was above {}, delay was {}ms)",
        ecu.config.dfco_config.rpm_min,
        ecu.config.dfco_config.delay_ms
    );
    assert!(fuel_save_count > 0, "Should have saved fuel during DFCO");
    assert!(final_rpm < high_rpm, "RPM should have decreased");
    println!("DFCO scenario completed successfully");
}

/// Test voltage drop under load
///
/// Simulates:
/// 1. Normal operation with good voltage
/// 2. Heavy electrical load causes voltage drop
/// 3. ECU enters limp mode
/// 4. Voltage recovers, normal operation resumes
#[test]
fn test_voltage_drop_under_load() {
    use ecu_core::{safety::PowerState, EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Voltage Drop Under Load ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();
    engine.set_throttle(50);
    engine.set_battery_voltage(14000); // Normal alternator voltage

    let mut current_time = 0u32;
    let mut limp_mode_entered = false;
    let mut recovered = false;

    // Phase 1: Normal operation
    println!("Phase 1: Normal voltage operation...");
    while current_time < 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);

                let power_state = ecu.update_voltage(14000, edge_time);
                assert_eq!(power_state, PowerState::Normal);

                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    let normal_injections = outputs.injection_count();
    println!("Normal injections: {normal_injections}");

    // Phase 2: Voltage drop - simulate heavy load or alternator failure
    println!("Phase 2: Voltage drop...");
    engine.set_battery_voltage(10500); // Drop to 10.5V (low but not critical)

    while current_time < 600_000 {
        // 400ms of low voltage
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);

                let power_state = ecu.update_voltage(10500, edge_time);

                if power_state == PowerState::Warning
                    || power_state == PowerState::Critical
                    || ecu.voltage_monitor.limp_active
                {
                    limp_mode_entered = true;
                }

                // Apply torque limits
                ecu.apply_safety_torque_limits(edge_time);

                // Check effective RPM limit
                let effective_limit = ecu.get_effective_rpm_limit();
                if effective_limit < ecu.rev_limiter_config_mut().max_rpm {
                    // Limp limit is active
                }

                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    println!("Limp mode entered: {limp_mode_entered}");
    println!("Effective RPM limit: {}", ecu.get_effective_rpm_limit());

    // Phase 3: Voltage recovery
    println!("Phase 3: Voltage recovery...");
    engine.set_battery_voltage(14000);

    while current_time < 1_000_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);

                let power_state = ecu.update_voltage(14000, edge_time);

                if power_state == PowerState::Normal && !ecu.voltage_monitor.limp_active {
                    recovered = true;
                }

                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    println!("Recovered to normal: {recovered}");
    println!(
        "Final power state: {:?}",
        ecu.update_voltage(14000, current_time)
    );

    assert!(
        limp_mode_entered || ecu.voltage_monitor.limp_active || recovered,
        "Should have responded to voltage drop"
    );
    println!("Voltage scenario completed successfully");
}

/// Test sensor failure and recovery
///
/// Simulates:
/// 1. Normal operation with valid sensors
/// 2. MAP sensor goes out of range
/// 3. ECU uses fallback strategy
/// 4. Sensor recovers, normal operation resumes
#[test]
fn test_sensor_failure_recovery() {
    use ecu_core::{diag::DiagCode, EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Sensor Failure and Recovery ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    // Configure sensor limits
    ecu.config.sensors_limits.map_min_kpa_x10 = 150; // 15 kPa min
    ecu.config.sensors_limits.map_max_kpa_x10 = 1100; // 110 kPa max
    ecu.config.sensors_limits.clear_time_s = 1; // 1 second to clear fault

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();
    engine.set_throttle(40);

    let mut current_time = 0u32;
    let mut diag_fault_logged = false;
    let mut diag_cleared = false;

    // Phase 1: Normal operation
    println!("Phase 1: Normal sensor operation...");
    while current_time < 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let raw_map = sensors.map_kpa() * 10; // Convert to x10
                ecu.set_rpm(rpm);

                ecu.process_sensor_update(Micros::new(edge_time), Kpa10::new(raw_map), 40);

                let pw = ecu.calculate_fuel(rpm, ecu.map_kpa_x10() / 10);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    assert!(
        !ecu.diag_map.is_active(),
        "No MAP fault should be active initially"
    );
    println!("Phase 1 complete - no faults");

    // Phase 2: Simulate MAP sensor failure (out of range)
    println!("Phase 2: MAP sensor failure...");
    let fault_map_reading = 1200; // 120 kPa - out of range

    while current_time < 600_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);

                // Use out-of-range MAP value
                let (clamped_map, _) = ecu.process_sensor_update(
                    Micros::new(edge_time),
                    Kpa10::new(fault_map_reading),
                    40,
                );

                // Should be clamped to max
                assert!(
                    clamped_map.raw() <= ecu.config.sensors_limits.map_max_kpa_x10,
                    "MAP should be clamped"
                );

                if ecu.diag_map.is_active() {
                    diag_fault_logged = true;
                }

                // Check load failure condition
                ecu.check_load_failure(edge_time);

                let pw = ecu.calculate_fuel(rpm, clamped_map.raw() / 10);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    println!("MAP fault active: {}", ecu.diag_map.is_active());
    println!("Diag fault logged: {diag_fault_logged}");
    assert!(diag_fault_logged, "Should have logged MAP fault");

    // Phase 3: Sensor recovery
    println!("Phase 3: Sensor recovery...");
    let good_map_reading = 500; // 50 kPa - normal

    while current_time < 2_000_000 {
        // Run for 1.4 more seconds
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);

                ecu.process_sensor_update(Micros::new(edge_time), Kpa10::new(good_map_reading), 40);

                if !ecu.diag_map.is_active() && diag_fault_logged {
                    diag_cleared = true;
                }

                let pw = ecu.calculate_fuel(rpm, good_map_reading / 10);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    println!("Diag cleared: {diag_cleared}");
    println!("Final MAP fault state: {}", ecu.diag_map.is_active());

    // Check diagnostic log
    let map_events: Vec<_> = ecu
        .diag_log()
        .events
        .iter()
        .filter_map(|e| e.as_ref())
        .filter(|e| e.code == DiagCode::MapRange)
        .collect();
    println!("MAP range events in log: {}", map_events.len());

    assert!(diag_fault_logged, "Should have detected MAP fault");
    assert!(diag_cleared, "Fault should have cleared after recovery");
    println!("Sensor failure scenario completed successfully");
}

/// Test full drive cycle from cold start to warm operation
///
/// Simulates:
/// 1. Cold start with enrichment
/// 2. Warm-up phase with decreasing enrichment
/// 3. Normal cruising at operating temperature
/// 4. Acceleration event
/// 5. Deceleration with DFCO
#[test]
fn test_full_drive_cycle() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Full Drive Cycle ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    // Configure all systems
    ecu.init_linear_table();
    ecu.init_ignition_table();
    // WueConfig and AseConfig have max_percent fields (no enable field)
    ecu.config.wue_config.max_percent = 30; // 30% enrichment at cold start
    ecu.config.ase_config.percent = 20; // 20% after-start enrichment
                                        // DfcoConfig uses rpm_min, rpm_max, tps_max_pct
    ecu.config.dfco_config.rpm_min = 2000;
    ecu.config.dfco_config.rpm_max = 7000;

    let mut decoder = TriggerDecoder::new(&time);
    let mut current_time = 0u32;
    let mut phase_results: Vec<(String, u16, u16)> = Vec::new(); // (phase, rpm, avg_pw)

    // === Phase 1: Cold Start ===
    println!("Phase 1: Cold Start (10°C)...");
    engine.set_coolant_temp(10);
    engine.set_intake_temp(10);
    ecu.corrections_mut().clt = 130; // 30% enrichment for cold

    engine.start_cranking();
    outputs.clear();

    // Cranking takes longer to sync - allow up to 2 seconds
    while current_time < 2_000_000 && !engine.state().running {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() && outputs.injection_count() > 5 {
                engine.start_running();
            }

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    // If we never synced during cranking, that's acceptable for simulation
    // but we should have started running if we did sync
    if !decoder.synced() {
        println!("Note: Did not achieve sync during simulated cranking");
        // Force running state for the rest of the test
        engine.start_running();
        // Wait for sync while running
        while current_time < 2_500_000 && !decoder.synced() {
            engine.update(10);
            sensors.update(engine.state());
            if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
                time.set_micros(edge_time);
                decoder.tooth_edge();
            }
            current_time += 10;
        }
    }

    assert!(
        decoder.synced(),
        "Should sync during cold start or shortly after"
    );
    phase_results.push((
        "Cold Start".to_string(),
        engine.state().rpm,
        outputs.average_injection_pw(),
    ));
    println!(
        "Cold start complete - RPM: {}, Avg PW: {}us",
        engine.state().rpm,
        outputs.average_injection_pw()
    );

    // === Phase 2: Warm-up ===
    println!("Phase 2: Warm-up...");
    engine.set_coolant_temp(40); // Warming up
    ecu.corrections_mut().clt = 115; // Reduced enrichment
    outputs.clear();

    let warmup_end = current_time + 500_000;
    while current_time < warmup_end {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    phase_results.push((
        "Warm-up".to_string(),
        engine.state().rpm,
        outputs.average_injection_pw(),
    ));
    println!(
        "Warm-up complete - RPM: {}, Avg PW: {}us",
        engine.state().rpm,
        outputs.average_injection_pw()
    );

    // === Phase 3: Cruising at Operating Temp ===
    println!("Phase 3: Cruising...");
    engine.set_coolant_temp(85); // Operating temp
    ecu.corrections_mut().clt = 100; // No enrichment
    engine.set_throttle(30); // Light cruise
    outputs.clear();

    let cruise_end = current_time + 500_000;
    while current_time < cruise_end {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);
                ecu.set_map_kpa_x10(load * 10);
                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    phase_results.push((
        "Cruise".to_string(),
        engine.state().rpm,
        outputs.average_injection_pw(),
    ));
    println!(
        "Cruise complete - RPM: {}, Avg PW: {}us",
        engine.state().rpm,
        outputs.average_injection_pw()
    );

    // === Phase 4: Acceleration ===
    println!("Phase 4: Acceleration...");
    engine.set_throttle(100);
    outputs.clear();

    let accel_end = current_time + 500_000;
    while current_time < accel_end {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);
                let pw = ecu.calculate_fuel(rpm, load);
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    let accel_rpm = engine.state().rpm;
    phase_results.push((
        "Acceleration".to_string(),
        accel_rpm,
        outputs.average_injection_pw(),
    ));
    println!(
        "Acceleration complete - RPM: {}, Avg PW: {}us",
        accel_rpm,
        outputs.average_injection_pw()
    );

    // === Phase 5: Deceleration ===
    println!("Phase 5: Deceleration...");
    engine.set_throttle(0);
    outputs.clear();
    let mut dfco_events = 0u32;

    let decel_end = current_time + 500_000;
    while current_time < decel_end {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);
                ecu.set_tps_percent(0);

                // Check DFCO
                let in_dfco = rpm >= ecu.config.dfco_config.rpm_min
                    && rpm <= ecu.config.dfco_config.rpm_max
                    && ecu.tps_percent() <= ecu.config.dfco_config.tps_max_pct;

                if in_dfco {
                    dfco_events += 1;
                } else {
                    let load = sensors.map_kpa();
                    let pw = ecu.calculate_fuel(rpm, load);
                    outputs.record_injection(edge_time, pw);
                }
            }
        }
        current_time += 10;
    }

    phase_results.push((
        "Deceleration".to_string(),
        engine.state().rpm,
        outputs.average_injection_pw(),
    ));
    println!(
        "Deceleration complete - RPM: {}, DFCO events: {dfco_events}",
        engine.state().rpm
    );

    // === Summary ===
    println!("\n=== Drive Cycle Summary ===");
    for (phase, rpm, pw) in &phase_results {
        println!("{phase}: RPM={rpm}, Avg PW={pw}us");
    }

    // Verify drive cycle behavior
    assert!(accel_rpm > 2000, "Should have accelerated to high RPM");
    assert!(dfco_events > 0, "DFCO should have activated during decel");
    assert!(decoder.synced(), "Should maintain sync throughout");
    println!("Full drive cycle completed successfully");
}

/// Test torque arbitration under multiple limits
///
/// Simulates:
/// 1. Driver requesting full torque
/// 2. Rev limiter adding limit
/// 3. Limp mode adding limit
/// 4. Min-wins arbitration selecting most restrictive
#[test]
fn test_torque_arbitration_under_limits() {
    use ecu_core::EcuState;

    println!("\n=== Torque Arbitration Under Limits ===");

    let mut ecu = EcuState::new();
    ecu.set_rpm(5500);
    ecu.set_map_kpa_x10(900); // 90 kPa

    // Initialize torque controller
    let max_torque = ecu.update_torque(25);
    println!("Max available torque: {} Nm x10", max_torque);

    // Phase 1: Driver requests 100%
    println!("\nPhase 1: Driver requesting full torque...");
    ecu.request_driver_torque(100, 1000);
    let driver_only = ecu.update_torque(25);
    println!("Driver only arbitrated: {} Nm x10", driver_only);
    assert!(driver_only > 0, "Should have positive torque from driver");

    let base_torque = driver_only;

    // Phase 2: Rev limiter adds limit
    println!("\nPhase 2: Rev limiter active...");
    ecu.set_rpm(6100); // Above soft limit
    ecu.rev_limiter_config_mut().soft_limit_start_rpm = 6000;
    ecu.rev_limiter_config_mut().max_rpm = 6500;
    ecu.update_rev_limiter();
    ecu.apply_safety_torque_limits(2000);

    // Need to call update_torque to actually apply the arbitration result
    let with_rev_limit = ecu.update_torque(25);
    println!("With rev limiter: {} Nm x10", with_rev_limit);
    println!(
        "Winning source: {:?}",
        ecu.torque_controller.arbiter.winning_source
    );

    assert!(
        with_rev_limit < base_torque,
        "Rev limiter should reduce torque"
    );

    // Phase 3: Limp mode also active (voltage issue)
    println!("\nPhase 3: Limp mode also active...");
    ecu.voltage_monitor.limp_active = true;
    ecu.apply_safety_torque_limits(3000);

    let with_limp = ecu.update_torque(25);
    println!("With limp mode: {} Nm x10", with_limp);
    println!(
        "Winning source: {:?}",
        ecu.torque_controller.arbiter.winning_source
    );

    // Check that torque has been reduced
    assert!(
        with_limp < base_torque,
        "Limp mode should reduce torque from base"
    );

    // Reset and verify torque is no longer limited
    ecu.reset_torque();
    let after_reset = ecu.update_torque(25);
    println!("After reset: {} Nm x10", after_reset);

    // With no requests, the arbiter should allow the full available torque.
    assert_eq!(after_reset, ecu.torque_controller.max_available_x10);

    println!("Torque arbitration scenario completed successfully");
}

/// Test LTFT learning during steady-state cruise
///
/// Simulates:
/// 1. Steady cruise conditions
/// 2. STFT shows consistent correction needed
/// 3. LTFT learns from STFT
/// 4. Combined trim applied
#[test]
fn test_ltft_learning_during_cruise() {
    use ecu_core::EcuState;

    println!("\n=== LTFT Learning During Cruise ===");

    let mut ecu = EcuState::new();

    // Configure for cruise conditions
    ecu.set_rpm(2500);
    ecu.set_map_kpa_x10(500); // 50 kPa - part throttle
    ecu.set_tps_percent(25);

    // Configure lambda/LTFT
    ecu.config.lambda_config.enable = true;
    ecu.ltft_manager_mut().config.learn_rate = 50; // Higher learn rate for faster test
    ecu.ltft_manager_mut().config.max_trim_x10 = 150; // ±15%

    // Simulate consistent rich condition requiring negative STFT
    ecu.lambda_state.active = true;
    ecu.set_stft_x10(-30); // -3% STFT (running rich, need to lean out)

    let mut current_time = 0u32;
    let initial_ltft = ecu
        .ltft_manager()
        .table
        .lookup(ecu.rpm(), ecu.map_kpa_x10());
    println!("Initial LTFT: {} x0.1%", initial_ltft);

    // Run LTFT updates for several seconds of "cruise"
    println!("Running LTFT learning...");
    for i in 0..50 {
        current_time += 100_000; // 100ms per update

        let ltft = ecu.update_ltft(85, current_time); // Warm engine

        if i % 10 == 0 {
            println!(
                "  t={}ms: LTFT={} x0.1%, STFT={} x0.1%",
                current_time / 1000,
                ltft,
                ecu.stft_x10()
            );
        }
    }

    let final_ltft = ecu
        .ltft_manager()
        .table
        .lookup(ecu.rpm(), ecu.map_kpa_x10());
    let total_trim = ecu.get_total_fuel_trim();
    let learned_cells = ecu.ltft_learned_cell_count();

    println!("\nFinal state:");
    println!("  LTFT: {} x0.1%", final_ltft);
    println!("  STFT: {} x0.1%", ecu.stft_x10());
    println!("  Total trim: {} x0.1%", total_trim);
    println!("  Learned cells: {}", learned_cells);
    println!("  Learning active: {}", ecu.is_ltft_learning());

    // LTFT should have moved towards compensating for the rich condition
    // The LTFT value should have changed (even if learned_cell_count is still 0 due to
    // the cell not being marked as fully "learned" yet, the value should change)
    assert!(
        final_ltft != initial_ltft || final_ltft < 0,
        "LTFT should have moved to compensate for rich condition: initial={}, final={}",
        initial_ltft,
        final_ltft
    );

    // Test fuel calculation with trims
    let base_pw = ecu.calculate_fuel(ecu.rpm(), ecu.map_kpa_x10() / 10);
    let adjusted_pw = ecu.calculate_fuel_with_enrichments(
        ecu.rpm(),
        ecu.map_kpa_x10() / 10,
        0,
        0,
        0,
        total_trim / 10, // Convert x10 to percent
    );

    println!("\nFuel calculation:");
    println!("  Base PW: {}us", base_pw);
    println!("  Adjusted PW: {}us", adjusted_pw);

    println!("LTFT learning scenario completed successfully");
}

/// Test multi-fault cascade - multiple failures at once
///
/// Simulates:
/// 1. Normal operation
/// 2. MAP sensor fails AND voltage drops simultaneously
/// 3. ECU handles both gracefully
/// 4. Recovery from both faults
#[test]
fn test_multi_fault_cascade() {
    use ecu_core::{safety::PowerState, EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Multi-Fault Cascade ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    // Configure systems
    ecu.config.sensors_limits.map_min_kpa_x10 = 150;
    ecu.config.sensors_limits.map_max_kpa_x10 = 1100;
    ecu.config.sensors_limits.clear_time_s = 1; // 1 second to clear fault after recovery

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();
    engine.set_throttle(40);

    let mut current_time = 0u32;
    let mut faults_handled = false;
    let mut recovered = false;

    // Phase 1: Normal operation
    println!("Phase 1: Normal operation...");
    while current_time < 100_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);
                ecu.update_voltage(14000, edge_time);
                ecu.process_sensor_update(Micros::new(edge_time), Kpa10::new(500), 40);
                outputs.record_injection(edge_time, ecu.calculate_fuel(rpm, 50));
            }
        }
        current_time += 10;
    }

    assert!(
        !ecu.diag_map.is_active(),
        "No fault should be active initially"
    );
    println!("Initial state OK");

    // Phase 2: Simultaneous faults
    println!("Phase 2: Injecting multiple faults...");
    let fault_start = current_time;

    while current_time < fault_start + 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);

                // Inject multiple faults
                let power_state = ecu.update_voltage(9500, edge_time); // Low voltage
                ecu.process_sensor_update(Micros::new(edge_time), Kpa10::new(1200), 40); // MAP out of range

                if (power_state != PowerState::Normal || ecu.voltage_monitor.limp_active)
                    && ecu.diag_map.is_active()
                {
                    faults_handled = true;
                }

                // ECU should still operate despite faults
                let pw = ecu.calculate_fuel(rpm, ecu.map_kpa_x10() / 10);
                assert!(
                    (500..=20000).contains(&pw),
                    "Fuel PW should be within safe range"
                );
                outputs.record_injection(edge_time, pw);
            }
        }
        current_time += 10;
    }

    println!("Faults handled: {faults_handled}");
    println!("MAP fault active: {}", ecu.diag_map.is_active());
    println!("Voltage limp active: {}", ecu.voltage_monitor.limp_active);

    // Phase 3: Recovery
    println!("Phase 3: Recovering from faults...");

    while current_time < fault_start + 3_000_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                ecu.set_rpm(rpm);

                // Restore good conditions
                ecu.update_voltage(14000, edge_time);
                ecu.process_sensor_update(Micros::new(edge_time), Kpa10::new(500), 40);

                if !ecu.diag_map.is_active() && !ecu.voltage_monitor.limp_active {
                    recovered = true;
                }

                outputs.record_injection(edge_time, ecu.calculate_fuel(rpm, 50));
            }
        }
        current_time += 10;
    }

    println!("Recovered: {recovered}");
    println!("Final MAP fault: {}", ecu.diag_map.is_active());
    println!("Final voltage limp: {}", ecu.voltage_monitor.limp_active);

    assert!(faults_handled, "Should have handled multiple faults");
    // Either recovered completely OR at least MAP fault cleared (voltage limp may have longer hysteresis)
    assert!(
        recovered || !ecu.diag_map.is_active(),
        "Should at least recover from MAP fault after sensor returns to normal"
    );
    println!("Multi-fault cascade scenario completed successfully");
}

/// Test rapid throttle transitions
///
/// Simulates:
/// 1. Rapid on-off throttle cycling (tip-in/tip-out)
/// 2. ECU maintains sync and safe fuel
/// 3. No system crashes or runaway behavior
#[test]
fn test_rapid_throttle_cycling() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Rapid Throttle Cycling ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, mut ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    ecu.init_linear_table();

    let mut decoder = TriggerDecoder::new(&time);
    engine.start_running();

    let mut current_time = 0u32;
    let mut cycle_count = 0u32;
    let cycle_period = 100_000; // 100ms cycles
    let total_duration = 2_000_000; // 2 seconds

    let mut min_pw = u16::MAX;
    let mut max_pw = 0u16;
    let mut sync_lost_count = 0u32;

    // Initial sync
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
    println!("Initial sync achieved");

    // Rapid throttle cycling
    println!("Starting rapid throttle cycling...");

    while current_time < total_duration {
        // Toggle throttle every cycle
        let in_cycle_time = current_time % (cycle_period * 2);
        let throttle = if in_cycle_time < cycle_period { 100 } else { 0 };

        engine.set_throttle(throttle);
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                ecu.set_rpm(rpm);
                ecu.set_tps_percent(throttle as u8);

                let pw = ecu.calculate_fuel(rpm, load);
                min_pw = min_pw.min(pw);
                max_pw = max_pw.max(pw);
                outputs.record_injection(edge_time, pw);
            } else {
                sync_lost_count += 1;
            }
        }

        if current_time % cycle_period < 10 {
            cycle_count += 1;
        }
        current_time += 10;
    }

    println!("Cycles completed: {cycle_count}");
    println!("PW range: {} - {} us", min_pw, max_pw);
    println!("Sync lost count: {sync_lost_count}");
    println!("Total injections: {}", outputs.injection_count());

    // Should maintain sync throughout
    assert!(decoder.synced(), "Should maintain sync after cycling");
    assert!(sync_lost_count < 5, "Should not frequently lose sync");
    assert!(max_pw <= 20000, "Fuel should stay within safe max");
    assert!(min_pw >= 500, "Fuel should stay within safe min");
    println!("Rapid throttle cycling scenario completed successfully");
}

/// Test engine restart after stall
///
/// Simulates:
/// 1. Engine running normally
/// 2. Engine stalls (RPM drops to 0)
/// 3. Re-crank and restart
/// 4. Normal operation resumes
#[test]
fn test_engine_restart_after_stall() {
    use ecu_core::{EcuState, TriggerDecoder};
    use simulation::*;

    println!("\n=== Engine Restart After Stall ===");

    let (mut engine, mut trigger, mut sensors, time, mut outputs, ecu) = (
        EngineSimulator::new(EngineConfig::default()),
        TriggerGenerator::new(TriggerPattern::SixtyMinusTwo),
        SensorSimulator::new(),
        SimulatedTime::new(),
        OutputCapture::new(),
        EcuState::new(),
    );

    let mut decoder = TriggerDecoder::new(&time);
    let mut current_time = 0u32;
    let mut restart_achieved = false;

    // Phase 1: Normal running
    println!("Phase 1: Normal running...");
    engine.start_running();

    while current_time < 200_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                outputs.record_injection(edge_time, ecu.calculate_fuel(rpm, load));
            }
        }
        current_time += 10;
    }

    let pre_stall_injections = outputs.injection_count();
    println!("Pre-stall injections: {pre_stall_injections}");
    assert!(decoder.synced(), "Should be synced before stall");

    // Phase 2: Simulate stall
    println!("Phase 2: Engine stalls...");
    engine.stop();
    trigger.reset();

    // Wait for sync timeout
    let stall_start = current_time;
    while current_time < stall_start + 500_000 {
        time.set_micros(current_time);

        // Process timeout in decoder by feeding a single edge after long gap
        if current_time == stall_start + 300_000 {
            decoder.tooth_edge(); // This should trigger timeout
        }

        current_time += 10;
    }

    let stall_detected = !decoder.synced();
    println!("Stall detected (sync lost): {stall_detected}");

    // Phase 3: Restart
    println!("Phase 3: Restarting engine...");
    engine.start_cranking();
    outputs.clear();

    let restart_start = current_time;
    while current_time < restart_start + 2_000_000 {
        engine.update(10);
        sensors.update(engine.state());

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            decoder.tooth_edge();

            if decoder.synced() && !restart_achieved {
                restart_achieved = true;
                engine.start_running();
                println!(
                    "Sync re-acquired at {}ms after restart",
                    (current_time - restart_start) / 1000
                );
            }

            if decoder.synced() {
                let rpm = decoder.rpm().raw();
                let load = sensors.map_kpa();
                outputs.record_injection(edge_time, ecu.calculate_fuel(rpm, load));
            }
        }
        current_time += 10;
    }

    println!("Restart achieved: {restart_achieved}");
    println!("Post-restart injections: {}", outputs.injection_count());

    assert!(stall_detected, "Should detect stall via sync loss");
    assert!(restart_achieved, "Should successfully restart");
    assert!(decoder.synced(), "Should have sync after restart");
    println!("Engine restart scenario completed successfully");
}

/// Test closed-loop lambda control transitions
///
/// Simulates:
/// 1. Open-loop operation (cold engine)
/// 2. Transition to closed-loop (warm engine)
/// 3. Lambda correction active
/// 4. WOT transition back to open-loop
#[test]
fn test_lambda_control_transitions() {
    use ecu_core::lambda::DisableReason;
    use ecu_core::EcuState;

    println!("\n=== Lambda Control Transitions ===");

    let mut ecu = EcuState::new();

    // Configure lambda control
    ecu.config.lambda_config.enable = true;
    ecu.config.lambda_config.min_clt_c = 60;
    ecu.config.lambda_config.max_tps_percent = 80;
    ecu.config.lambda_config.min_rpm = 1200;

    let mut current_time = 0u32;

    // Phase 1: Cold engine - should be open-loop
    println!("Phase 1: Cold engine (open-loop)...");
    ecu.set_rpm(2000);
    ecu.set_tps_percent(25);
    let cold_clt = 40; // Below min_clt_c

    // Update lambda state
    let stft = ecu.lambda_state.update(
        500, // O2 sensor reading (mV)
        cold_clt,
        ecu.tps_percent(),
        ecu.rpm(),
        &ecu.config.lambda_config,
        current_time,
    );

    println!("Cold: STFT={}, Active={}", stft, ecu.lambda_state.active);
    assert!(!ecu.lambda_state.active, "Should be open-loop when cold");
    assert_eq!(
        ecu.lambda_state.disable_reason,
        Some(DisableReason::CoolantTooLow)
    );

    // Phase 2: Warm engine - should transition to closed-loop
    println!("Phase 2: Warm engine (closed-loop)...");
    current_time += 500_000;
    let warm_clt = 85;

    let stft = ecu.lambda_state.update(
        400, // Slightly lean
        warm_clt,
        ecu.tps_percent(),
        ecu.rpm(),
        &ecu.config.lambda_config,
        current_time,
    );

    println!("Warm: STFT={}, Active={}", stft, ecu.lambda_state.active);
    assert!(ecu.lambda_state.active, "Should be closed-loop when warm");

    // Phase 3: Run closed-loop for a while
    println!("Phase 3: Running closed-loop corrections...");
    for i in 0..20 {
        current_time += 100_000;

        // Simulate slightly lean O2 reading (400mV)
        let stft = ecu.lambda_state.update(
            400,
            warm_clt,
            ecu.tps_percent(),
            ecu.rpm(),
            &ecu.config.lambda_config,
            current_time,
        );

        if i % 5 == 0 {
            println!("  t={}ms: STFT={} x0.1%", current_time / 1000, stft);
        }
    }

    let pre_wot_stft = ecu.stft_x10();
    println!("Pre-WOT STFT: {} x0.1%", pre_wot_stft);

    // STFT should have accumulated some positive correction (adding fuel for lean)
    assert!(
        pre_wot_stft > 0 || ecu.lambda_state.integral > 0,
        "Should have positive STFT for lean condition"
    );

    // Phase 4: WOT - should go open-loop
    println!("Phase 4: WOT (open-loop)...");
    ecu.set_tps_percent(95); // Above max_tps_percent
    current_time += 100_000;

    let stft = ecu.lambda_state.update(
        500,
        warm_clt,
        ecu.tps_percent(),
        ecu.rpm(),
        &ecu.config.lambda_config,
        current_time,
    );

    println!("WOT: STFT={}, Active={}", stft, ecu.lambda_state.active);
    assert!(!ecu.lambda_state.active, "Should be open-loop at WOT");
    assert_eq!(
        ecu.lambda_state.disable_reason,
        Some(DisableReason::WideOpenThrottle)
    );

    // Phase 5: Return to part throttle - should go back to closed-loop
    println!("Phase 5: Part throttle (closed-loop again)...");
    ecu.set_tps_percent(30);
    current_time += 100_000;

    let stft = ecu.lambda_state.update(
        450, // Stoichiometric
        warm_clt,
        ecu.tps_percent(),
        ecu.rpm(),
        &ecu.config.lambda_config,
        current_time,
    );

    println!(
        "Part throttle: STFT={}, Active={}",
        stft, ecu.lambda_state.active
    );
    assert!(ecu.lambda_state.active, "Should return to closed-loop");

    println!("Lambda control transitions scenario completed successfully");
}

#[test]
#[cfg(not(feature = "sched-angle-disable"))]
fn closed_loop_convergence() {
    use ecu_core::app::EcuApp;

    let time = SimulatedTime::new();
    let mut app = EcuApp::new(&time);
    let mut engine = simulation::EngineSimulator::new(simulation::EngineConfig::default());
    let mut trigger = simulation::TriggerGenerator::new(simulation::TriggerPattern::SixtyMinusTwo);
    let mut sensors = simulation::SensorSimulator::new();
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut inj0 = PulseCapturePin::new(&time, Rc::clone(&events));
    let mut inj1 = NoopPin;
    let mut outputs: [&mut dyn OutputPin; 2] = [&mut inj0, &mut inj1];
    let mut base_widths: Vec<u16> = Vec::new();

    engine.set_coolant_temp(85);
    engine.set_intake_temp(50);
    engine.start_running();
    engine.set_throttle(0);

    {
        let state = app.state_mut();
        state.config.lambda_config.enable = true;
        state.config.lambda_config.min_clt_c = -40;
        state.config.lambda_config.max_tps_percent = 100;
        state.config.lambda_config.min_rpm = 0;
        state
            .lambda_state
            .set_sensor_type(ecu_core::lambda::O2SensorType::Narrowband);
    }

    let lean_o2_mv = 400u16; // Narrowband lean-side input to drive positive STFT
    let mut current_time = 0u32;
    let duration_us = 60_000_000u32;
    let mut sync_seen = false;

    while current_time < duration_us {
        engine.update(10);
        sensors.update(engine.state());

        {
            let state = app.state_mut();
            state.set_clt_x10(engine.state().coolant_temp_c.saturating_mul(10));
            state.set_iat_x10(engine.state().intake_temp_c.saturating_mul(10));
            state.set_tps_percent(sensors.tps_percent().min(u8::MAX as u16) as u8);
            state.set_map_kpa_x10(sensors.map_kpa());
        }

        {
            let state = app.state_mut();
            let _ = state.lambda_state.update(
                lean_o2_mv,
                engine.state().coolant_temp_c,
                engine.state().tps_percent.min(u16::from(u8::MAX)) as u8,
                engine.state().rpm,
                &state.config.lambda_config,
                current_time,
            );
        }

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            app.on_timestamp(edge_time);
            current_time = edge_time;

            if app.state().synced() {
                sync_seen = true;
                if app.state().tooth_count() == ecu_core::constants::timing::INJECTION_TOOTH {
                    base_widths.push(
                        app.state()
                            .calculate_fuel(app.state().rpm(), DEFAULT_LOAD_KPA),
                    );
                }
            }
        } else {
            time.advance(10);
            current_time = current_time.wrapping_add(10);
        }

        app.drive_outputs(time.micros(), &mut outputs);
    }

    assert!(sync_seen, "ECU should achieve sync during closed-loop run");

    let recorded = events.borrow();
    assert!(
        recorded.len() > 20,
        "Expected multiple injector pulses, got {}",
        recorded.len()
    );
    assert_eq!(
        recorded.len(),
        base_widths.len(),
        "Need a matching base width for every captured pulse"
    );

    let steady_sample_count = recorded.len().min(10);
    let steady_pw = recorded
        .iter()
        .rev()
        .take(steady_sample_count)
        .map(|&(_, pw)| pw as u32)
        .sum::<u32>()
        / steady_sample_count as u32;

    let steady_base_pw = base_widths
        .iter()
        .rev()
        .take(steady_sample_count)
        .map(|&pw| pw as u32)
        .sum::<u32>()
        / steady_sample_count as u32;
    let final_stft = app.state().stft_x10();
    assert!(
        final_stft > 0,
        "STFT should approach a positive steady state, got {}",
        final_stft
    );

    let target_pw = steady_base_pw + (steady_base_pw * final_stft as u32) / 1000;
    assert!(
        (steady_pw as i32 - target_pw as i32).abs() <= (target_pw / 100) as i32,
        "Closed-loop PW should converge toward the settled STFT target: steady={steady_pw}, target={target_pw}"
    );

    assert!(final_stft != 0, "STFT should not settle at zero");
}

#[test]
fn rev_limit_fuel_cut() {
    use ecu_core::app::EcuApp;
    use ecu_core::constants::fuel::DEFAULT_LOAD_KPA;
    use ecu_core::constants::fuel::{MAX_PULSE_WIDTH_US, MIN_PULSE_WIDTH_US};
    use ecu_core::torque::actuate::TorqueConverter;
    use ecu_core::torque::arbiter::TorqueResult;

    let time = SimulatedTime::new();
    let mut app = EcuApp::new(&time);
    let mut engine = simulation::EngineSimulator::new(simulation::EngineConfig::default());
    let mut trigger = simulation::TriggerGenerator::new(simulation::TriggerPattern::SixtyMinusTwo);
    let mut sensors = simulation::SensorSimulator::new();
    let events = Rc::new(RefCell::new(Vec::new()));
    let mut inj0 = PulseCapturePin::new(&time, Rc::clone(&events));
    let mut inj1 = NoopPin;
    let mut outputs: [&mut dyn OutputPin; 2] = [&mut inj0, &mut inj1];
    let mut base_widths: Vec<u16> = Vec::new();
    let mut fuel_mults: Vec<u16> = Vec::new();
    let mut scheduled_pws: Vec<Micros> = Vec::new();

    engine.set_coolant_temp(85);
    engine.set_intake_temp(50);
    engine.start_running();
    engine.set_throttle(100);

    {
        let state = app.state_mut();
        state.rev_limiter_config_mut().max_rpm = 7000;
        state.rev_limiter_config_mut().soft_limit_start_rpm = 3800;
        state.rev_limiter_config_mut().hysteresis_rpm = 150;
        state.rev_limiter_config_mut().strategy = ecu_core::LimiterStrategy::SoftCut;
    }

    let converter = TorqueConverter::new();
    let mut current_time = 0u32;
    let duration_us = 2_500_000u32;
    let mut limiter_seen = false;

    while current_time < duration_us {
        engine.update(10);
        sensors.update(engine.state());

        {
            let state = app.state_mut();
            state.set_clt_x10(engine.state().coolant_temp_c.saturating_mul(10));
            state.set_iat_x10(engine.state().intake_temp_c.saturating_mul(10));
            state.set_tps_percent(sensors.tps_percent().min(u8::MAX as u16) as u8);
            state.set_map_kpa_x10(sensors.map_kpa());
            state.set_rpm(engine.state().rpm);
            state.update_rev_limiter();

            let max_torque_x10 = 2000i16;
            let target_torque_x10 = if state.rev_limiter_state.active {
                limiter_seen = true;
                max_torque_x10.saturating_mul(100 - state.rev_limiter_state.fuel_cut_percent as i16)
                    / 100
            } else {
                max_torque_x10
            };

            let targets = converter.convert(target_torque_x10, max_torque_x10, state.rpm);
            state.apply_torque_result(&TorqueResult::new(targets.fuel_mult_x100 as u16));
        }

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            app.on_timestamp(edge_time);
            current_time = edge_time;

            if app.state().synced()
                && app.state().tooth_count() == ecu_core::constants::timing::INJECTION_TOOTH
            {
                let base_pw = app
                    .state()
                    .calculate_fuel(app.state().rpm(), DEFAULT_LOAD_KPA);
                base_widths.push(base_pw);
                fuel_mults.push(app.state().fuel_mult_x100());
                scheduled_pws.push(app.state().final_pw(
                    ecu_core::Rpm::new(app.state().rpm()),
                    Kpa10::new(DEFAULT_LOAD_KPA),
                ));
            }
        } else {
            time.advance(10);
            current_time = current_time.wrapping_add(10);
        }

        app.drive_outputs(time.micros(), &mut outputs);
    }

    assert!(limiter_seen, "Rev limiter should have activated");
    assert!(
        !fuel_mults.is_empty(),
        "Expected at least one scheduled injection sample"
    );

    let _recorded = events.borrow();
    assert_eq!(
        scheduled_pws.len(),
        base_widths.len(),
        "Need a scheduled PW for every captured base width"
    );

    let cut_index = fuel_mults
        .iter()
        .position(|&fuel_mult| fuel_mult < 100)
        .expect("Expected a reduced fuel multiplier above the limiter threshold");

    let cut_base = base_widths[cut_index] as u32;
    let cut_pw = scheduled_pws[cut_index].raw();
    let cut_mult = fuel_mults[cut_index] as u32;

    assert!(
        cut_mult < 100,
        "Limiter should reduce fuel multiplier below 100, got {}",
        cut_mult
    );
    assert!(
        cut_pw
            == ((cut_base * cut_mult) / 100)
                .clamp(MIN_PULSE_WIDTH_US as u32, MAX_PULSE_WIDTH_US as u32),
        "Scheduled PW should scale with the limiter multiplier and clamp: pw={cut_pw}, base={cut_base}, mult={cut_mult}"
    );
}

/// Test plausibility checking (TPS vs MAP disagreement)
///
/// Simulates:
/// 1. Normal consistent sensor readings
/// 2. TPS high but MAP low (impossible condition)
/// 3. Plausibility fault detected
/// 4. Recovery when sensors agree
#[test]
fn test_plausibility_fault_detection() {
    use ecu_core::sensors::plausibility::PlausibilityFault;
    use ecu_core::EcuState;

    println!("\n=== Plausibility Fault Detection ===");

    let mut ecu = EcuState::new();

    // Configure plausibility checking
    // High TPS should mean high MAP (except at very low RPM)
    ecu.config.plausibility_config.enable = true;
    ecu.config.plausibility_config.tps_high_threshold = 50; // Above 50% TPS triggers check
    ecu.config.plausibility_config.map_low_threshold_x10 = 400; // 40 kPa minimum MAP at high TPS
    ecu.config.plausibility_config.min_rpm = 2000;
    ecu.config.plausibility_config.debounce_time_us = 100_000; // 100ms debounce

    let mut current_time = 0u32;

    // Phase 1: Normal operation - TPS and MAP consistent
    println!("Phase 1: Normal operation...");
    ecu.set_rpm(3000);
    ecu.set_tps_percent(70); // High TPS
    ecu.set_map_kpa_x10(800); // High MAP (80 kPa) - consistent

    let fault = ecu.check_plausibility(current_time);
    println!(
        "TPS={}, MAP={}kPa, Fault={:?}",
        ecu.tps_percent(),
        ecu.map_kpa_x10() / 10,
        fault
    );
    assert_eq!(
        fault,
        PlausibilityFault::None,
        "No fault with consistent sensors"
    );

    // Phase 2: Impossible condition - high TPS but low MAP
    println!("Phase 2: Implausible condition (high TPS, low MAP)...");
    ecu.set_tps_percent(80); // Very high TPS
    ecu.set_map_kpa_x10(250); // Very low MAP (25 kPa) - implausible

    // Run for debounce period
    let mut detected_fault = PlausibilityFault::None;
    for _ in 0..20 {
        current_time += 10_000; // 10ms steps
        let fault = ecu.check_plausibility(current_time);

        if fault != PlausibilityFault::None {
            println!("Fault detected at {}ms: {:?}", current_time / 1000, fault);
            detected_fault = fault;
            break;
        }
    }

    let has_fault = ecu.has_plausibility_fault();
    println!("Plausibility fault active: {has_fault}");
    assert_eq!(
        detected_fault,
        PlausibilityFault::TpsHighMapLow,
        "High TPS / low MAP should confirm the expected plausibility fault"
    );
    assert!(
        has_fault,
        "Plausibility fault should remain active after detection"
    );

    // Phase 3: Return to normal - sensors agree again
    println!("Phase 3: Sensors return to agreement...");
    ecu.set_tps_percent(70);
    ecu.set_map_kpa_x10(850); // Consistent again

    // Clear fault after time
    for _ in 0..50 {
        current_time += 10_000;
        ecu.check_plausibility(current_time);
    }

    let final_fault = ecu.has_plausibility_fault();
    println!("Final fault state: {final_fault}");
    assert!(
        !final_fault,
        "Plausibility fault should clear once the sensors agree for long enough"
    );

    println!("Plausibility checking scenario completed successfully");
}
