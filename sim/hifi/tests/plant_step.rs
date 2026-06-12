use ecu_sim_hifi::{
    advance_plant_step, CylinderCommand, PlantConfig, PlantConfigError, PlantCylinderConfig,
    PlantStepInput,
};

fn base_plant_config() -> PlantConfig {
    let mut cfg = ecu_sim_hifi::default_plant_config();
    cfg.cylinders = vec![
        PlantCylinderConfig {
            phase_offset_rad: 0.0,
        },
        PlantCylinderConfig {
            phase_offset_rad: core::f64::consts::PI,
        },
    ];
    cfg.combustion.spark_angle_rad = 15.0_f64.to_radians();
    cfg
}

fn base_input() -> PlantStepInput {
    PlantStepInput {
        now_s: 0.1,
        window_s: 0.02,
        crank_angle_rad: core::f64::consts::PI,
        rpm: 2500.0,
        throttle_position: 0.5,
        load_torque_nm: 20.0,
        cylinders: vec![
            CylinderCommand {
                fuel_mass_kg: 1.8e-5,
                spark_angle_rad: 15.0_f64.to_radians(),
                dwell_s: 0.002,
            },
            CylinderCommand {
                fuel_mass_kg: 1.8e-5,
                spark_angle_rad: 15.0_f64.to_radians(),
                dwell_s: 0.002,
            },
        ],
    }
}

#[test]
fn plant_step_returns_one_trace_per_cylinder_and_aggregate_means() {
    let output = advance_plant_step(&base_plant_config(), &base_input()).unwrap();

    assert_eq!(output.cylinders.len(), 2);
    let mean_lambda = output
        .cylinders
        .iter()
        .map(|cylinder| cylinder.lambda)
        .sum::<f64>()
        / 2.0;
    let mean_egt = output
        .cylinders
        .iter()
        .map(|cylinder| cylinder.egt_k)
        .sum::<f64>()
        / 2.0;
    let mean_knock = output
        .cylinders
        .iter()
        .map(|cylinder| cylinder.knock_margin)
        .sum::<f64>()
        / 2.0;
    let total_torque = output
        .cylinders
        .iter()
        .map(|cylinder| cylinder.brake_torque_nm)
        .sum::<f64>();

    assert!((output.lambda - mean_lambda).abs() < 1.0e-12);
    assert!((output.egt_k - mean_egt).abs() < 1.0e-12);
    assert!((output.knock_margin - mean_knock).abs() < 1.0e-12);
    assert!((output.brake_torque_nm - total_torque).abs() < 1.0e-12);
}

#[test]
fn plant_step_rpm_drops_as_load_torque_increases() {
    let mut light_load = base_input();
    light_load.load_torque_nm = 5.0;
    let mut heavy_load = base_input();
    heavy_load.load_torque_nm = 80.0;

    let light = advance_plant_step(&base_plant_config(), &light_load).unwrap();
    let heavy = advance_plant_step(&base_plant_config(), &heavy_load).unwrap();

    assert!(light.rpm > heavy.rpm);
}

#[test]
fn plant_step_manifold_pressure_rises_with_throttle() {
    let mut closed = base_input();
    closed.throttle_position = 0.2;

    let mut open = base_input();
    open.throttle_position = 0.9;

    let closed_output = advance_plant_step(&base_plant_config(), &closed).unwrap();
    let open_output = advance_plant_step(&base_plant_config(), &open).unwrap();

    assert!(open_output.manifold_pressure_pa > closed_output.manifold_pressure_pa);
}

#[test]
fn plant_step_rejects_invalid_throttle_position() {
    let mut input = base_input();
    input.throttle_position = 1.2;

    let error = advance_plant_step(&base_plant_config(), &input).unwrap_err();

    assert_eq!(error, PlantConfigError::InvalidThrottlePosition);
}

#[test]
fn trace_warm_idle_step_states() {
    let mut input = base_input();
    input.throttle_position = 0.28;
    input.load_torque_nm = 12.0;
    input.cylinders[0].fuel_mass_kg = 1.2e-5;

    let mut previous_map_kpa = None;
    for step in 0..10 {
        let now_s = (step as f64) * 0.02;
        input.now_s = now_s;
        let output = advance_plant_step(&base_plant_config(), &input).unwrap();

        assert!(output.manifold_pressure_pa > 0.0);
        assert!(output.brake_torque_nm.is_finite());
        if let Some(previous_map_kpa) = previous_map_kpa {
            assert!(previous_map_kpa >= 0.0);
        }
        previous_map_kpa = Some(output.manifold_pressure_pa / 100.0);

        assert!(output.lambda > 0.0);
        assert!(output.brake_torque_nm.is_finite());
    }
}
