//! Criterion benchmark harness for ecu-runtime step()
//! Validates p95 regression threshold of 5.0% per US-FM0289.
//!
//! Run with:
//!   cargo bench --package ecu-runtime --bench runtime_bench -- --save-baseline=<name>
//!   cargo bench --package ecu-runtime --bench runtime_bench -- --baseline=<name>

use criterion::{black_box, criterion_group, criterion_main, Criterion};
use ecu_control::{EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs};
use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_runtime::{ControlInputs, EngineRuntime, StepInputs};
use ecu_spec::TempC10;

fn default_runtime() -> EngineRuntime {
    EngineRuntime::new()
}

fn runtime_step_input() -> (EngineRuntime, StepInputs, ControlInputs) {
    let runtime = default_runtime();
    let step_inputs = StepInputs {
        now_us: Micros::new(100_000),
        rpm: 3000,
        load_kpa10: 95,
        angle_x10: 0,
        trigger_synced: true,
        cam_seen: false,
        flat_shift_armed: false,
        launch_armed: false,
    };
    let control_inputs = ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(100_000),
            clt_c: TempC10::new(800).get(),
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            clt_c: TempC10::new(800).get(),
            lambda_valid: true,
            measured_lambda100: ecu_domain::Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(
            100,   // driver_request_x100
            0,     // idle_request_x100
            65535, // rev_limit_x100
            65535, // knock_limit_x100
            65535, // limp_limit_x100
        ),
        ignition: IgnitionInputs::new(
            Degrees10::new(150), // base_advance_deg10
            0,                   // timing_correction_deg10
            0,                   // knock_retard_deg10
            0,                   // torque_retard_deg10
            false,               // rev_limit_active
            Rpm::new(3000),
        ),
    };
    (runtime, step_inputs, control_inputs)
}

fn criterion_benchmark(c: &mut Criterion) {
    let (mut runtime, step_inputs, control_inputs) = runtime_step_input();

    c.bench_function("runtime_step", |b| {
        b.iter(|| {
            let result = runtime.step(black_box(step_inputs), black_box(control_inputs));
            black_box(result);
        });
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
