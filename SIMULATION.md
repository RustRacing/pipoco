# ECU Simulation Framework

This document describes the host-side simulation code used to test ECU logic
without target hardware.

## Purpose

The simulation layer is intended for logic validation, regression tests, and
scenario-based checks. It is not a replacement for bench or engine testing.

The framework covers:

- engine speed and load progression
- 60-2 trigger edge generation
- sensor updates and injected sensor faults
- deterministic simulated time
- output capture for injection events

## Layout

Simulation support for the test suite lives under `tests/simulation/`:

- `engine.rs`: simple engine state and RPM/load progression
- `trigger.rs`: trigger pattern and edge generation
- `sensors.rs`: sensor values and fault injection
- `time.rs`: deterministic time source
- `outputs.rs`: captured ECU outputs for assertions
- `scenarios.rs`: reusable end-to-end scenarios

The main integration entry point is `tests/simulated_scenarios_test.rs`.

## Model

The simulation loop is intentionally simple:

1. advance engine state
2. update simulated sensors
3. emit trigger edges when due
4. feed edges into the ECU logic
5. record outputs for assertions

The goal is to validate ECU behavior under controlled conditions, not to model
full engine thermodynamics or electrical behavior.

## Example

```rust
use ecu_compat::{EcuState, TriggerDecoder};
use simulation::{
    EngineConfig, EngineSimulator, OutputCapture, SensorSimulator, SimulatedTime,
    TriggerGenerator, TriggerPattern,
};

let mut engine = EngineSimulator::new(EngineConfig::default());
let mut trigger = TriggerGenerator::new(TriggerPattern::SixtyMinusTwo);
let mut sensors = SensorSimulator::new();
let time = SimulatedTime::new();
let mut outputs = OutputCapture::new();
let ecu = EcuState::new();
let mut decoder = TriggerDecoder::new(&time);

engine.start_running();

let mut current_time = 0u32;
while current_time < 1_000_000 {
    engine.update(10);
    sensors.update(engine.state());

    if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
        time.set_micros(edge_time);
        decoder.tooth_edge();

        if decoder.synced() {
            let pw = ecu.calculate_fuel(decoder.rpm(), sensors.map_kpa());
            outputs.record_injection(edge_time, pw);
        }
    }

    current_time += 10;
}
```

## Scenario Tests

Predefined scenarios in `tests/simulation/scenarios.rs` include:

- cold start
- hot start
- idle stability
- acceleration
- sync loss and recovery
- sensor fault handling

These tests are intended to verify behavioral invariants such as:

- sync is achieved and maintained when expected
- fuel pulse widths remain in safe bounds
- fault conditions are detected and can recover
- the ECU continues operating in degraded modes where appropriate

## Running Tests

Run the scenario suite:

```bash
cargo test --test simulated_scenarios_test
```

Run one scenario with debug output:

```bash
cargo test --test simulated_scenarios_test test_hot_start_80c -- --nocapture
```

Run ignored long-duration cases:

```bash
cargo test --test simulated_scenarios_test --ignored -- --nocapture
```

## Fault Injection

`SensorSimulator` supports explicit fault injection. Current test coverage uses
faults such as:

- `OpenCircuit`
- `ShortToGround`
- `ShortToBattery`
- `Intermittent`

This is useful for validating safety clamps, limp behavior, diagnostics, and
recovery logic.

## Limits

The simulation does not model:

- real-time ISR latency or jitter
- analog noise and EMI behavior
- injector or coil electrical loading
- true thermal behavior
- physical combustion or fuel transport

Use hardware testing for timing accuracy, electrical validation, sensor
characterization, and long-duration reliability checks.

## Recommended Workflow

Use the simulation layer first, then move to hardware:

1. host simulation
2. bench test with a trigger source
3. hardware integration
4. engine or vehicle validation

This ordering keeps logic bugs and regression failures cheap to reproduce before
hardware-specific work begins.
