use ecu_sim_hifi::{
    converge_open_system_cycles, run_open_system_cycle, CylinderGeometry, GasProperties,
    IntegratorConfig, ManifoldConfig, OpenSystemConfig, ResidualConfig, ThrottleConfig,
    ValveTiming,
};

fn base_config() -> OpenSystemConfig {
    OpenSystemConfig {
        geometry: CylinderGeometry::new(0.086, 0.086, 0.143, 10.0),
        fresh_gas: GasProperties::new(287.0, 718.0),
        burned_gas: GasProperties::new(300.0, 800.0),
        integrator: IntegratorConfig { step_deg: 1.0 },
        manifold: ManifoldConfig {
            volume_m3: 0.002,
            temperature_k: 300.0,
            ambient_pressure_pa: 101_325.0,
        },
        throttle: ThrottleConfig {
            max_area_m2: 2.0e-4,
            discharge_coefficient: 0.8,
        },
        throttle_position: 0.5,
        intake_valve: ValveTiming {
            open_angle_rad: 340.0_f64.to_radians(),
            close_angle_rad: 580.0_f64.to_radians(),
            max_lift_m: 0.008,
            seat_diameter_m: 0.032,
            discharge_coefficient: 0.7,
        },
        exhaust_valve: ValveTiming {
            open_angle_rad: 120.0_f64.to_radians(),
            close_angle_rad: 360.0_f64.to_radians(),
            max_lift_m: 0.007,
            seat_diameter_m: 0.028,
            discharge_coefficient: 0.7,
        },
        residual: ResidualConfig {
            residual_temperature_k: 900.0,
        },
        initial_cylinder_pressure_pa: 101_325.0,
        initial_cylinder_temperature_k: 330.0,
        initial_residual_fraction: 0.08,
        exhaust_backpressure_pa: 110_000.0,
        exhaust_temperature_k: 850.0,
        rpm: 2500.0,
        trapped_mass_tolerance_kg: 1.0e-6,
        residual_tolerance: 1.0e-4,
        max_cycles: 8,
    }
}

#[test]
fn volumetric_efficiency_rises_with_throttle() {
    let mut low = base_config();
    low.throttle_position = 0.25;
    let mut high = base_config();
    high.throttle_position = 0.9;

    let low_cycle = run_open_system_cycle(low).unwrap();
    let high_cycle = run_open_system_cycle(high).unwrap();

    assert!(high_cycle.volumetric_efficiency > low_cycle.volumetric_efficiency);
    assert!(high_cycle.trapped_fresh_mass_kg > low_cycle.trapped_fresh_mass_kg);
}

#[test]
fn pumping_work_magnitude_grows_as_throttle_closes() {
    let mut open = base_config();
    open.throttle_position = 0.9;
    let mut closed = base_config();
    closed.throttle_position = 0.2;

    let open_cycle = run_open_system_cycle(open).unwrap();
    let closed_cycle = run_open_system_cycle(closed).unwrap();

    assert!(closed_cycle.pumping_work_j.abs() > open_cycle.pumping_work_j.abs());
    assert!(closed_cycle.pmep_pa > open_cycle.pmep_pa);
}

#[test]
fn residual_fraction_rises_with_exhaust_backpressure() {
    let mut base = base_config();
    base.exhaust_backpressure_pa = 105_000.0;
    let mut raised = base_config();
    raised.exhaust_backpressure_pa = 135_000.0;

    let base_cycle = run_open_system_cycle(base).unwrap();
    let raised_cycle = run_open_system_cycle(raised).unwrap();

    assert!(raised_cycle.residual_fraction > base_cycle.residual_fraction);
}

#[test]
fn residual_fraction_rises_with_valve_overlap() {
    let mut low_overlap = base_config();
    low_overlap.throttle_position = 0.15;
    low_overlap.exhaust_backpressure_pa = 140_000.0;
    low_overlap.intake_valve.open_angle_rad = 345.0_f64.to_radians();
    low_overlap.exhaust_valve.close_angle_rad = 350.0_f64.to_radians();

    let mut high_overlap = base_config();
    high_overlap.throttle_position = 0.15;
    high_overlap.exhaust_backpressure_pa = 140_000.0;
    high_overlap.intake_valve.open_angle_rad = 345.0_f64.to_radians();
    high_overlap.exhaust_valve.close_angle_rad = 395.0_f64.to_radians();

    let low_cycle = run_open_system_cycle(low_overlap).unwrap();
    let high_cycle = run_open_system_cycle(high_overlap).unwrap();

    assert!(high_cycle.residual_fraction > low_cycle.residual_fraction);
}

#[test]
fn open_system_runner_lands_exactly_on_valve_event_angles() {
    let mut config = base_config();
    config.intake_valve.open_angle_rad = 340.0_f64.to_radians();
    config.intake_valve.close_angle_rad = 580.0_f64.to_radians();
    config.exhaust_valve.open_angle_rad = 120.0_f64.to_radians();
    config.exhaust_valve.close_angle_rad = 360.0_f64.to_radians();

    for step_deg in [0.7, 1.0, 7.0] {
        config.integrator.step_deg = step_deg;
        let cycle = run_open_system_cycle(config).unwrap();
        let angles: Vec<f64> = cycle
            .samples
            .iter()
            .map(|sample| sample.crank_angle_rad)
            .collect();
        let expected_events = [
            340.0_f64.to_radians(),
            580.0_f64.to_radians(),
            (120.0_f64 + 720.0).to_radians(),
            360.0_f64.to_radians(),
        ];
        for target in expected_events {
            assert!(
                angles.iter().any(|angle| (*angle - target).abs() < 1.0e-12),
                "missing exact event angle {target} for step {step_deg}",
            );
        }
        assert!(
            cycle.trapped_fresh_mass_kg > 0.0,
            "ivc mass was not captured for step {step_deg}"
        );
    }
}

#[test]
fn open_system_cycle_converges_within_configured_iteration_budget() {
    let result = converge_open_system_cycles(base_config()).unwrap();

    assert!(result.convergence.iteration_count <= base_config().max_cycles);
}

#[test]
fn open_system_cycle_conserves_total_boundary_mass() {
    let cycle = run_open_system_cycle(base_config()).unwrap();
    let initial = cycle.samples.first().expect("initial sample present");
    let final_state = cycle.samples.last().expect("final sample present");
    let initial_system_mass = initial.cylinder_mass_kg + initial.manifold_mass_kg;
    let final_system_mass = final_state.cylinder_mass_kg + final_state.manifold_mass_kg;
    let system_mass_delta = final_system_mass - initial_system_mass;
    let boundary_delta = cycle.throttle_boundary_mass_kg - cycle.exhaust_boundary_mass_kg;

    assert!(
        (system_mass_delta - boundary_delta).abs() < 1.0e-9,
        "system delta {system_mass_delta:e} should match boundary delta {boundary_delta:e}"
    );
}

#[test]
fn open_system_cycle_tracks_isentropic_compression_when_valves_closed() {
    let mut config = base_config();
    config.throttle_position = 0.0;
    config.intake_valve.open_angle_rad = 540.0_f64.to_radians();
    config.intake_valve.close_angle_rad = 541.0_f64.to_radians();
    config.exhaust_valve.open_angle_rad = 540.0_f64.to_radians();
    config.exhaust_valve.close_angle_rad = 541.0_f64.to_radians();
    config.intake_valve.max_lift_m = 0.0;
    config.exhaust_valve.max_lift_m = 0.0;
    config.integrator.step_deg = 1.0;

    let cycle = run_open_system_cycle(config).unwrap();
    let sample_at_540 = cycle
        .samples
        .iter()
        .find(|sample| sample.crank_angle_rad >= 540.0_f64.to_radians())
        .expect("closed-cycle sample at IVC window");
    let sample_at_720 = cycle
        .samples
        .iter()
        .find(|sample| sample.crank_angle_rad >= 720.0_f64.to_radians())
        .expect("closed-cycle sample at TDC");
    let actual_ratio = sample_at_720.cylinder_pressure_pa / sample_at_540.cylinder_pressure_pa;
    let gamma = 1.0 + (config.fresh_gas.r_j_per_kg_k / config.fresh_gas.cv_j_per_kg_k);
    let expected_ratio = 10.0_f64.powf(gamma);
    let tolerance = 0.05;
    let lower = expected_ratio * (1.0 - tolerance);
    let upper = expected_ratio * (1.0 + tolerance);

    assert!(
        (lower..=upper).contains(&actual_ratio),
        "expected pressure ratio {expected_ratio:.3}±5%, got {actual_ratio:.3}"
    );
}

#[test]
fn open_system_cycle_does_not_create_energy_from_nothing() {
    let mut config = base_config();
    config.throttle_position = 0.7;
    config.integrator.step_deg = 1.0;

    let cycle = run_open_system_cycle(config).unwrap();
    let start_sample = cycle.samples.first().expect("initial sample present");
    let _end_sample = cycle.samples.last().expect("final sample present");

    let initial_cylinder_fresh_fraction = (1.0 - config.initial_residual_fraction).max(0.0);
    let initial_cylinder_burned_fraction = config.initial_residual_fraction.max(0.0);
    let initial_gas = GasProperties::new(
        config.fresh_gas.r_j_per_kg_k * initial_cylinder_fresh_fraction
            + config.burned_gas.r_j_per_kg_k * initial_cylinder_burned_fraction,
        config.fresh_gas.cv_j_per_kg_k * initial_cylinder_fresh_fraction
            + config.burned_gas.cv_j_per_kg_k * initial_cylinder_burned_fraction,
    );
    let cylinder_mass = cycle.cylinder_state.mass_kg.max(1.0e-12);
    let cylinder_fresh_fraction = cycle.cylinder_state.composition.fresh_mass_kg.max(0.0)
        / cycle
            .cylinder_state
            .composition
            .total_mass_kg()
            .max(1.0e-12);
    let cylinder_burned_fraction = 1.0 - cylinder_fresh_fraction;
    let end_cylinder_gas = GasProperties::new(
        config.fresh_gas.r_j_per_kg_k * cylinder_fresh_fraction
            + config.burned_gas.r_j_per_kg_k * cylinder_burned_fraction,
        config.fresh_gas.cv_j_per_kg_k * cylinder_fresh_fraction
            + config.burned_gas.cv_j_per_kg_k * cylinder_burned_fraction,
    );
    let manifold_mass = cycle.manifold_state.mass_kg.max(1.0e-12);
    let manifold_fresh_fraction = cycle.manifold_state.composition.fresh_mass_kg.max(0.0)
        / cycle
            .manifold_state
            .composition
            .total_mass_kg()
            .max(1.0e-12);
    let manifold_burned_fraction = 1.0 - manifold_fresh_fraction;
    let end_manifold_gas = GasProperties::new(
        config.fresh_gas.r_j_per_kg_k * manifold_fresh_fraction
            + config.burned_gas.r_j_per_kg_k * manifold_burned_fraction,
        config.fresh_gas.cv_j_per_kg_k * manifold_fresh_fraction
            + config.burned_gas.cv_j_per_kg_k * manifold_burned_fraction,
    );

    let initial_internal_energy_j = start_sample.cylinder_mass_kg
        * initial_gas.cv_j_per_kg_k
        * start_sample.cylinder_temperature_k
        + start_sample.manifold_mass_kg
            * config.fresh_gas.cv_j_per_kg_k
            * config.manifold.temperature_k;
    let end_internal_energy_j =
        cylinder_mass * end_cylinder_gas.cv_j_per_kg_k * cycle.cylinder_state.temperature_k
            + manifold_mass * end_manifold_gas.cv_j_per_kg_k * cycle.manifold_state.temperature_k;
    let delta_internal_energy_j = end_internal_energy_j - initial_internal_energy_j;

    let expected_balance_j = cycle.boundary_enthalpy_j - cycle.boundary_piston_work_j;
    let relative_error =
        ((delta_internal_energy_j - expected_balance_j).abs()) / expected_balance_j.abs().max(1.0);

    assert!(
        relative_error < 1.0e-3,
        "open-system energy mismatch too large: ΔU={delta_internal_energy_j:.6}, boundary={expected_balance_j:.6}, rel={relative_error:.6}"
    );
}

#[test]
fn open_system_manifold_temperature_stays_near_intended_value_at_steady_throttle() {
    let mut config = base_config();
    config.throttle_position = 0.4;
    config.integrator.step_deg = 1.0;

    let cycle = run_open_system_cycle(config).unwrap();
    let initial_manifold_temp = config.manifold.temperature_k;
    let final_manifold_temp = cycle.manifold_state.temperature_k;
    let manifold_delta = (final_manifold_temp - initial_manifold_temp).abs();

    assert!(
        manifold_delta < 30.0,
        "manifold temperature drift {manifold_delta:.3} K exceeded steady-state band"
    );
}
