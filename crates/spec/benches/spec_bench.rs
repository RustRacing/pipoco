#![allow(unused)]
use criterion::{black_box, criterion_group, criterion_main, BatchSize, Criterion};
use ecu_spec::{
    default_reference_calibration, step, EngineMode, InputSnapshot, Kpa10, LogicalState,
    Millivolts, Rpm, SyncState, TempC10,
};

fn input_snapshot(rpm: u16, tps_x100: u16, clt_c10: i16, map_kpa10: u16) -> InputSnapshot {
    InputSnapshot {
        mode: EngineMode::Running,
        rpm: Rpm(rpm),
        tps_x100,
        clt_c10: TempC10(clt_c10),
        map_kpa10: Kpa10(map_kpa10),
        load_kpa10: Kpa10(map_kpa10),
        vbatt_mv: Millivolts(12000),
        iat_c10: TempC10(250),
        baro_kpa10: Kpa10(1010),
        knock_intensity_x100: 0,
        launch_armed: false,
        flat_shift_armed: false,
        sync: SyncState::Synced,
        fuel_cut: false,
        spark_cut: false,
        target_afr_override_x100: ecu_spec::AfrOverride::None,
        t_us: ecu_spec::Micros(100_000),
    }
}

fn logical_state() -> LogicalState {
    LogicalState::default()
}

fn criterion_benchmark(c: &mut criterion::Criterion) {
    let cal = default_reference_calibration();
    let state = logical_state();
    let input = input_snapshot(3000, 100, 800, 95);

    c.bench_function("spec_step", |b| {
        b.iter_batched(
            || (cal, state),
            |(cal, state)| black_box(step(&cal, black_box(input), &state)),
            BatchSize::SmallInput,
        )
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
