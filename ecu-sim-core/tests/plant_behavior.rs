use ecu_sim_core::*;

const CYL: usize = 4;
const EDGES: usize = 64;
const EVENTS: usize = 8;

type TestPlant = Plant<CYL, EDGES, EVENTS>;
type TestInput = PlantStepInput<CYL, EVENTS>;
type TestOutput = PlantStepOutput<CYL, EDGES, EVENTS>;

fn config() -> PlantConfig<CYL> {
    PlantConfig::<CYL>::default_four()
}

fn fueled_sparked_input() -> TestInput {
    let mut input = TestInput::idle(Micros(100_000));
    input.driver.throttle_x1000 = 1000;
    input
        .ecu_outputs
        .injection_events
        .push(InjectionCommand {
            cylinder: CylinderIndex(0),
            mode: InjectionTimingMode::StartOfInjection,
            angle_deg10: CrankDeg10(3600),
            pulse_width_us: Micros(3500),
            injector_flow_ug_per_us: MicrogramsPerMicros(2),
            deadtime_us: Micros(1000),
        })
        .unwrap();
    input
        .ecu_outputs
        .spark_events
        .push(SparkCommand {
            cylinder: CylinderIndex(0),
            spark_angle_deg10: CrankDeg10(180),
            dwell_us: Micros(2000),
            coil_energy_x1000: 1000,
        })
        .unwrap();
    input
}

fn speed_density_fueled_sparked_input() -> TestInput {
    let mut input = TestInput::idle(Micros(100_000));
    input.driver.throttle_x1000 = 1000;
    input
        .ecu_outputs
        .injection_events
        .push(InjectionCommand {
            cylinder: CylinderIndex(0),
            mode: InjectionTimingMode::StartOfInjection,
            angle_deg10: CrankDeg10(3600),
            pulse_width_us: Micros(15_000),
            injector_flow_ug_per_us: MicrogramsPerMicros(3),
            deadtime_us: Micros(1000),
        })
        .unwrap();
    input
        .ecu_outputs
        .spark_events
        .push(SparkCommand {
            cylinder: CylinderIndex(0),
            spark_angle_deg10: CrankDeg10(180),
            dwell_us: Micros(2000),
            coil_energy_x1000: 1000,
        })
        .unwrap();
    input
}

#[test]
fn plant_step_is_deterministic() {
    let mut a = TestPlant::new(config());
    let mut b = TestPlant::new(config());
    a.reset(InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(1200),
        crank_angle_deg10: CrankDeg10(100),
        ..InitialPlantState::new()
    });
    b.reset(InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(1200),
        crank_angle_deg10: CrankDeg10(100),
        ..InitialPlantState::new()
    });
    let input = fueled_sparked_input();
    let mut out_a = TestOutput::empty();
    let mut out_b = TestOutput::empty();

    a.step(&input, &mut out_a).unwrap();
    b.step(&input, &mut out_b).unwrap();

    assert_eq!(a, b);
    assert_eq!(out_a, out_b);
}

#[test]
fn no_fuel_records_no_fuel_misfire() {
    let mut plant = TestPlant::new(config());
    let mut input = TestInput::idle(Micros(10_000));
    input.driver.throttle_x1000 = 1000;
    input
        .ecu_outputs
        .spark_events
        .push(SparkCommand {
            cylinder: CylinderIndex(0),
            spark_angle_deg10: CrankDeg10(180),
            dwell_us: Micros(2000),
            coil_energy_x1000: 1000,
        })
        .unwrap();
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(
        out.combustion.cylinders[0].misfire,
        Some(MisfireReason::NoFuel)
    );
    assert_eq!(out.combustion.cylinders[0].torque_nm_x100, TorqueNmX100(0));
}

#[test]
fn no_spark_records_no_spark_misfire() {
    let mut plant = TestPlant::new(config());
    let mut input = TestInput::idle(Micros(10_000));
    input.driver.throttle_x1000 = 1000;
    input
        .ecu_outputs
        .injection_events
        .push(InjectionCommand {
            cylinder: CylinderIndex(0),
            mode: InjectionTimingMode::EndOfInjection,
            angle_deg10: CrankDeg10(3000),
            pulse_width_us: Micros(3500),
            injector_flow_ug_per_us: MicrogramsPerMicros(2),
            deadtime_us: Micros(1000),
        })
        .unwrap();
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(
        out.combustion.cylinders[0].misfire,
        Some(MisfireReason::NoSpark)
    );
}

#[test]
fn no_cuts_consumes_events_and_allows_combustion() {
    let mut plant = TestPlant::new(config());
    let input = fueled_sparked_input();
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(out.consumed_events.injection_count, 1);
    assert_eq!(out.consumed_events.spark_count, 1);
    assert_eq!(out.consumed_events.ignored_injection_count, 0);
    assert_eq!(out.consumed_events.ignored_spark_count, 0);
    assert_eq!(out.combustion.cylinders[0].misfire, None);
    assert!(out.combustion.cylinders[0].torque_nm_x100.0 > 0);
}

#[test]
fn fuel_and_spark_cuts_suppress_events_and_combustion() {
    let mut plant = TestPlant::new(config());
    let mut input = fueled_sparked_input();
    input.ecu_outputs.fuel_cut = true;
    input.ecu_outputs.spark_cut = true;
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(out.consumed_events.injection_count, 1);
    assert_eq!(out.consumed_events.spark_count, 1);
    assert_eq!(out.consumed_events.ignored_injection_count, 1);
    assert_eq!(out.consumed_events.ignored_spark_count, 1);
    assert_eq!(
        out.combustion.cylinders[0].misfire,
        Some(MisfireReason::FuelCut)
    );
}

#[test]
fn starter_spins_from_zero_rpm() {
    let mut plant = TestPlant::new(config());
    let mut input = TestInput::idle(Micros(100_000));
    input.driver.starter_enabled = true;
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert!(out.sensors.rpm.0 > 0);
    assert!(out.sensors.crank_angle_deg10.0 < 7200);
}

#[test]
fn friction_slows_rpm_without_combustion() {
    let mut plant = TestPlant::new(config());
    plant.reset(InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(0),
        ..InitialPlantState::new()
    });
    let input = TestInput::idle(Micros(100_000));
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert!(out.sensors.rpm.0 < 1000);
}

#[test]
fn trigger_edges_and_faults_are_deterministic() {
    let mut a = TestPlant::new(config());
    let mut b = TestPlant::new(config());
    let initial = InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(0),
        ..InitialPlantState::new()
    };
    a.reset(initial);
    b.reset(initial);
    let mut input = TestInput::idle(Micros(100_000));
    input.faults.duplicate_next_crank_edges = 1;
    input.faults.delay_next_edge_us = Micros(5);
    let mut out_a = TestOutput::empty();
    let mut out_b = TestOutput::empty();

    a.step(&input, &mut out_a).unwrap();
    b.step(&input, &mut out_b).unwrap();

    assert_eq!(out_a.trigger_edges, out_b.trigger_edges);
    assert!(out_a.trigger_edges.len() > 1);
    assert_eq!(
        out_a.trigger_edges.as_slice()[0].channel,
        out_a.trigger_edges.as_slice()[1].channel
    );
    assert_eq!(
        out_a.trigger_edges.as_slice()[0].crank_angle_deg10,
        out_a.trigger_edges.as_slice()[1].crank_angle_deg10
    );
}

#[test]
fn knock_risk_rises_with_load_heat_and_advance() {
    let cfg = config();
    let calm = estimate_knock_risk(
        &cfg,
        Kpa10(700),
        Rpm(1500),
        Degrees10(100),
        950,
        Celsius10(250),
    );
    let risky = estimate_knock_risk(
        &cfg,
        Kpa10(1600),
        Rpm(5000),
        Degrees10(400),
        1150,
        Celsius10(650),
    );

    assert!(risky > calm);
}

#[test]
fn dyno_horsepower_uses_fixed_point_power_formula() {
    let hp = horsepower_x100(TorqueNmX100(20000), Rpm(3000));

    assert!((hp - 8418).abs() <= 2);
}

#[test]
fn zero_dt_returns_current_state_without_consuming_events() {
    let mut plant = TestPlant::new(config());
    plant.reset(InitialPlantState {
        timestamp_us: Micros(99),
        rpm: Rpm(1234),
        crank_angle_deg10: CrankDeg10(4321),
        ..InitialPlantState::new()
    });
    let mut input = fueled_sparked_input();
    input.dt_us = Micros(0);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(out.sensors.timestamp_us, Micros(99));
    assert_eq!(out.sensors.rpm, Rpm(1234));
    assert_eq!(out.sensors.crank_angle_deg10, CrankDeg10(4321));
    assert_eq!(out.consumed_events.injection_count, 0);
    assert_eq!(out.consumed_events.spark_count, 0);
    assert_eq!(out.trigger_edges.len(), 0);
    assert_eq!(out.combustion.cylinders[0].torque_nm_x100, TorqueNmX100(0));
}

#[test]
fn invalid_injector_flow_is_rejected() {
    let mut plant = TestPlant::new(config());
    let mut input = fueled_sparked_input();
    input.ecu_outputs.injection_events.as_mut_slice()[0].injector_flow_ug_per_us =
        MicrogramsPerMicros(0);
    let mut out = TestOutput::empty();

    assert_eq!(
        plant.step(&input, &mut out),
        Err(PlantStepError::InvalidInput)
    );
}

#[test]
fn output_capacity_error_reports_trigger_overflow() {
    type SmallOutput = PlantStepOutput<CYL, 0, EVENTS>;
    type SmallPlant = Plant<CYL, 0, EVENTS>;
    let mut plant = SmallPlant::new(config());
    plant.reset(InitialPlantState {
        rpm: Rpm(6000),
        ..InitialPlantState::new()
    });
    let input = PlantStepInput::<CYL, EVENTS>::idle(Micros(100_000));
    let mut out = SmallOutput::empty();

    assert_eq!(
        plant.step(&input, &mut out),
        Err(PlantStepError::OutputCapacityExceeded)
    );
}

#[test]
fn raw_battery_voltage_is_visible_in_sensors_and_telemetry() {
    let mut plant = TestPlant::new(config());
    let mut input = TestInput::idle(Micros(10_000));
    input.environment.battery_mv = Millivolts(10_000);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(out.sensors.battery_mv, Millivolts(10_000));
    assert_eq!(out.telemetry.battery_mv, Millivolts(10_000));
}

#[test]
fn scenario_runner_steps_fixed_inputs() {
    let mut plant = TestPlant::new(config());
    let steps = [ScenarioStep::new(TestInput::idle(Micros(10_000)))];
    let mut out = TestOutput::empty();

    let result = run_scenario(&mut plant, InitialPlantState::new(), &steps, &mut out).unwrap();

    assert_eq!(result.steps_run, 1);
}

#[test]
fn kinematic_pressure_mode_derives_torque_from_volume_direction() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    let mut expansion = TestPlant::new(cfg);
    let mut compression = TestPlant::new(cfg);
    expansion.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(900),
        ..InitialPlantState::new()
    });
    compression.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(2700),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut expansion_out = TestOutput::empty();
    let mut compression_out = TestOutput::empty();

    expansion.step(&input, &mut expansion_out).unwrap();
    compression.step(&input, &mut compression_out).unwrap();

    assert!(expansion_out.combustion.total_torque_nm_x100.0 > 0);
    assert!(compression_out.combustion.total_torque_nm_x100.0 < 0);
}

#[test]
fn kinematic_pressure_mode_exposes_bmep_in_dyno_and_telemetry() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(900),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert!(out.dyno.indicated_torque_nm_x100.0 > 0);
    assert_eq!(out.dyno.brake_torque_nm_x100, out.dyno.torque_nm_x100);
    assert!(out.dyno.bmep_bar_x100.0 > 0);
    assert_eq!(out.telemetry.bmep_bar_x100, out.dyno.bmep_bar_x100);
}

#[test]
fn substepper_counts_completed_cycle_boundaries() {
    let mut plant = TestPlant::new(config());
    plant.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(7100),
        ..InitialPlantState::new()
    });
    let input = TestInput::idle(Micros(100_000));
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(plant.state.cycle.completed_cycle_count, 1);
}

#[test]
fn polytropic_wiebe_mode_uses_burn_progress_for_torque() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::PolytropicWiebe;
    let mut early = TestPlant::new(cfg);
    let mut later = TestPlant::new(cfg);
    early.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(250),
        ..InitialPlantState::new()
    });
    later.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(650),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut early_out = TestOutput::empty();
    let mut later_out = TestOutput::empty();

    early.step(&input, &mut early_out).unwrap();
    later.step(&input, &mut later_out).unwrap();

    assert!(
        later.state.physics[0].last_burn_fraction_x10000
            > early.state.physics[0].last_burn_fraction_x10000
    );
    assert!(
        later_out.combustion.total_torque_nm_x100.0 > early_out.combustion.total_torque_nm_x100.0
    );
    assert!(later.state.physics[0].pmax_pa.0 > cfg.thermo.initial_cylinder_pressure_pa.0);
    assert!(later.state.physics[0].ca50_deg10.is_some());
}

#[test]
fn single_zone_mode_raises_temperature_and_pressure_from_heat_release() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::SingleZoneIdealGas;
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(650),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert!(plant.state.physics[0].temperature_k10.0 > cfg.thermo.initial_cylinder_temp_k10.0);
    assert!(plant.state.physics[0].pressure_pa.0 > cfg.thermo.p_ref_pa.0);
    assert!(plant.state.physics[0].pmax_pa.0 >= plant.state.physics[0].pressure_pa.0);
    assert_ne!(out.combustion.total_torque_nm_x100, TorqueNmX100(0));
}

#[test]
fn non_synthetic_modes_use_ve_table_for_air_mass() {
    let mut low_cfg = config();
    low_cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    low_cfg.air.ve_table = VeTable::constant(700);
    let mut high_cfg = low_cfg;
    high_cfg.air.ve_table = VeTable::constant(1200);
    let mut low = TestPlant::new(low_cfg);
    let mut high = TestPlant::new(high_cfg);
    low.reset(InitialPlantState {
        rpm: Rpm(2000),
        crank_angle_deg10: CrankDeg10(900),
        ..InitialPlantState::new()
    });
    high.reset(InitialPlantState {
        rpm: Rpm(2000),
        crank_angle_deg10: CrankDeg10(900),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut low_out = TestOutput::empty();
    let mut high_out = TestOutput::empty();

    low.step(&input, &mut low_out).unwrap();
    high.step(&input, &mut high_out).unwrap();

    assert!(high.state.cylinders[0].air_mass_ug.0 > low.state.cylinders[0].air_mass_ug.0);
    assert!(high_out.combustion.total_torque_nm_x100.0 > low_out.combustion.total_torque_nm_x100.0);
}

#[test]
fn wall_film_changes_delivered_fuel_without_hiding_raw_command() {
    let mut cfg = config();
    cfg.fuel.wall_film_enabled = true;
    cfg.fuel.wall_film_deposit_x1000 = 100;
    cfg.fuel.wall_film_tau_ms = 1000;
    let mut plant = TestPlant::new(cfg);
    let input = fueled_sparked_input();
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(
        out.combustion.cylinders[0].misfire, None,
        "small film deposit should preserve combustibility"
    );
    assert_eq!(plant.state.cylinders[0].last_injection_pw_us, Micros(3500));
    assert!(out.consumed_events.injection_fuel_mass_ug[0].0 < 5000);
    assert!(plant.state.physics[0].wall_film_ug.0 > 0);
}

#[test]
fn manifold_filling_mode_drives_map_with_finite_lag() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    cfg.air.manifold_filling_enabled = true;
    cfg.air.manifold_volume_cc = 3000;
    cfg.air.throttle_area_mm2 = 1800;
    cfg.air.throttle_discharge_coeff_x1000 = 700;
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        timestamp_us: Micros(1),
        rpm: Rpm(2500),
        ..InitialPlantState::new()
    });
    plant.state.map_kpa10 = Kpa10(450);
    let mut input = TestInput::idle(Micros(20_000));
    input.driver.throttle_x1000 = 800;
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert!(out.sensors.map_kpa10.0 > 450);
    assert!(out.sensors.map_kpa10.0 < cfg.air.reference_pressure_kpa10.0);
}

#[test]
fn lambda_transport_delays_sensor_observation_from_combustion_lambda() {
    let mut cfg = config();
    cfg.sensors.lambda_transport_enabled = true;
    cfg.sensors.lambda_delay_crank_deg = 720;
    cfg.sensors.lambda_sensor_tau_ms = 0;
    cfg.combustion.min_lambda_x1000 = 400;
    cfg.combustion.max_lambda_x1000 = 2000;
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        rpm: Rpm(1200),
        ..InitialPlantState::new()
    });
    let mut input = fueled_sparked_input();
    input.dt_us = Micros(20_000);
    input.ecu_outputs.injection_events.as_mut_slice()[0].pulse_width_us = Micros(4500);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_ne!(out.combustion.cylinders[0].lambda_x1000, 1000);
    assert_eq!(out.sensors.lambda_x1000, 1000);
}

#[test]
fn residual_overlap_low_load_reduces_stable_torque_in_plant_path() {
    let mut mild_cfg = config();
    mild_cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    mild_cfg.residual.enabled = true;
    mild_cfg.residual.base_fraction_x1000 = 50;
    mild_cfg.combustion.min_lambda_x1000 = 300;
    mild_cfg.combustion.max_lambda_x1000 = 2500;
    mild_cfg.valve_events.ivo_deg_btdc_x10 = 0;
    mild_cfg.valve_events.evc_deg_atdc_x10 = 0;
    let mut overlap_cfg = mild_cfg;
    overlap_cfg.valve_events.ivo_deg_btdc_x10 = 350;
    overlap_cfg.valve_events.evc_deg_atdc_x10 = 250;
    let mut mild = TestPlant::new(mild_cfg);
    let mut overlap = TestPlant::new(overlap_cfg);
    mild.reset(InitialPlantState {
        timestamp_us: Micros(1),
        rpm: Rpm(1200),
        ..InitialPlantState::new()
    });
    overlap.reset(InitialPlantState {
        timestamp_us: Micros(1),
        rpm: Rpm(1200),
        ..InitialPlantState::new()
    });
    mild.state.map_kpa10 = Kpa10(350);
    overlap.state.map_kpa10 = Kpa10(350);
    let mut input = speed_density_fueled_sparked_input();
    input.driver.throttle_x1000 = 200;
    input.dt_us = Micros(10_000);
    let mut mild_out = TestOutput::empty();
    let mut overlap_out = TestOutput::empty();

    mild.step(&input, &mut mild_out).unwrap();
    overlap.step(&input, &mut overlap_out).unwrap();

    assert!(
        overlap_out.telemetry.residual_fraction_x1000[0]
            > mild_out.telemetry.residual_fraction_x1000[0]
    );
    assert!(overlap.state.cylinders[0].air_mass_ug.0 < mild.state.cylinders[0].air_mass_ug.0);
    assert!(
        overlap_out.combustion.cylinders[0].torque_nm_x100.0
            < mild_out.combustion.cylinders[0].torque_nm_x100.0
    );
}

#[test]
fn valve_event_ivc_sweep_changes_low_and_high_rpm_torque_shape() {
    let mut early_cfg = config();
    early_cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    early_cfg.valve_events.ivc_deg_abdc_x10 = 350;
    early_cfg.combustion.min_lambda_x1000 = 300;
    early_cfg.combustion.max_lambda_x1000 = 2500;
    let mut late_cfg = early_cfg;
    late_cfg.valve_events.ivc_deg_abdc_x10 = 700;
    let mut early_low = TestPlant::new(early_cfg);
    let mut late_low = TestPlant::new(late_cfg);
    let mut early_high = TestPlant::new(early_cfg);
    let mut late_high = TestPlant::new(late_cfg);
    for plant in [&mut early_low, &mut late_low] {
        plant.reset(InitialPlantState {
            timestamp_us: Micros(1),
            rpm: Rpm(1800),
            ..InitialPlantState::new()
        });
    }
    for plant in [&mut early_high, &mut late_high] {
        plant.reset(InitialPlantState {
            timestamp_us: Micros(1),
            rpm: Rpm(6000),
            ..InitialPlantState::new()
        });
    }
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut early_low_out = TestOutput::empty();
    let mut late_low_out = TestOutput::empty();
    let mut early_high_out = TestOutput::empty();
    let mut late_high_out = TestOutput::empty();

    early_low.step(&input, &mut early_low_out).unwrap();
    late_low.step(&input, &mut late_low_out).unwrap();
    early_high.step(&input, &mut early_high_out).unwrap();
    late_high.step(&input, &mut late_high_out).unwrap();

    assert!(early_low.state.cylinders[0].air_mass_ug.0 > late_low.state.cylinders[0].air_mass_ug.0);
    assert!(
        late_high.state.cylinders[0].air_mass_ug.0 > early_high.state.cylinders[0].air_mass_ug.0
    );
    assert!(
        early_low_out.combustion.cylinders[0].torque_nm_x100.0
            > late_low_out.combustion.cylinders[0].torque_nm_x100.0
    );
    assert!(
        late_high_out.combustion.cylinders[0].torque_nm_x100.0
            > early_high_out.combustion.cylinders[0].torque_nm_x100.0
    );
}

#[test]
fn exhaust_temperature_model_reports_hotter_retarded_spark() {
    let cfg = config();
    let mut normal = TestPlant::new(cfg);
    let mut retarded = TestPlant::new(cfg);
    normal.reset(InitialPlantState {
        rpm: Rpm(2500),
        ..InitialPlantState::new()
    });
    retarded.reset(InitialPlantState {
        rpm: Rpm(2500),
        ..InitialPlantState::new()
    });
    let normal_input = fueled_sparked_input();
    let mut retarded_input = fueled_sparked_input();
    retarded_input.ecu_outputs.spark_events.as_mut_slice()[0].spark_angle_deg10 = CrankDeg10(60);
    let mut normal_out = TestOutput::empty();
    let mut retarded_out = TestOutput::empty();

    normal.step(&normal_input, &mut normal_out).unwrap();
    retarded.step(&retarded_input, &mut retarded_out).unwrap();

    assert!(retarded_out.telemetry.egt_k_x10 > normal_out.telemetry.egt_k_x10);
}

#[test]
fn dyno_sweep_emits_full_cycle_averaged_point_and_advances_target() {
    let mut cfg = config();
    cfg.dyno.mode = DynoMode::TargetRpmSweep;
    cfg.dyno.sweep_start_rpm = Rpm(1000);
    cfg.dyno.sweep_end_rpm = Rpm(1200);
    cfg.dyno.sweep_step_rpm = Rpm(200);
    cfg.dyno.hold_cycles_before_sample = 1;
    cfg.dyno.sample_cycles = 2;
    cfg.dyno.rpm_error_limit = Rpm(10_000);
    cfg.dyno.target_rpm = Rpm(1000);
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(7100),
        ..InitialPlantState::new()
    });
    let mut input = fueled_sparked_input();
    input.dt_us = Micros(100_000);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();
    assert!(!out.dyno.sweep_point.valid);
    plant.state.cycle.completed_cycle_count += 1;
    plant.step(&input, &mut out).unwrap();
    assert!(!out.dyno.sweep_point.valid);
    plant.state.cycle.completed_cycle_count += 1;
    plant.step(&input, &mut out).unwrap();

    assert!(out.dyno.sweep_point.valid);
    assert_eq!(out.dyno.sweep_point.target_rpm, Rpm(1000));
    assert_eq!(out.dyno.sweep_point.ve_x1000, 1000);
    assert_eq!(out.dyno.sweep_target_rpm, Rpm(1200));
    assert!(!out.dyno.sweep_complete);
}

#[test]
fn pressure_saturation_is_diagnostic_visible() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::KinematicPressurePulse;
    cfg.combustion.torque_scale_x100 = u16::MAX;
    cfg.air.ve_table = VeTable::constant(u16::MAX);
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(900),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    input.ecu_outputs.injection_events.as_mut_slice()[0].pulse_width_us = Micros(4000);
    input.ecu_outputs.injection_events.as_mut_slice()[0].injector_flow_ug_per_us =
        MicrogramsPerMicros(1000);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert!(out
        .diagnostics
        .events
        .as_slice()
        .iter()
        .any(|event| event.kind == DiagnosticKind::PhysicalSaturation));
}

#[test]
fn telemetry_exposes_required_physics_observations() {
    let mut cfg = config();
    cfg.physics_mode = PlantPhysicsMode::PolytropicWiebe;
    let mut plant = TestPlant::new(cfg);
    plant.reset(InitialPlantState {
        rpm: Rpm(1000),
        crank_angle_deg10: CrankDeg10(650),
        ..InitialPlantState::new()
    });
    let mut input = speed_density_fueled_sparked_input();
    input.dt_us = Micros(1000);
    let mut out = TestOutput::empty();

    plant.step(&input, &mut out).unwrap();

    assert_eq!(
        out.telemetry.crank_angle_deg10,
        out.sensors.crank_angle_deg10
    );
    assert_eq!(out.telemetry.tps_x1000, input.driver.throttle_x1000);
    assert_eq!(out.telemetry.ve_x1000, 1000);
    assert_eq!(
        out.telemetry.trapped_air_mass_ug[0],
        plant.state.cylinders[0].air_mass_ug
    );
    assert_eq!(out.telemetry.dwell_us[0], Micros(2000));
    assert!(out.telemetry.soi_deg10[0].is_some());
    assert!(out.telemetry.eoi_deg10[0].is_some());
    assert!(out.telemetry.delivered_fuel_mass_ug[0].0 > 0);
    assert!(out.telemetry.pmax_pa[0].0 > 0);
    assert!(out.telemetry.ca50_deg10[0].is_some());
    assert_eq!(
        out.telemetry.indicated_torque_nm_x100,
        out.dyno.indicated_torque_nm_x100
    );
    assert_eq!(
        out.telemetry.brake_torque_nm_x100,
        out.dyno.brake_torque_nm_x100
    );
    assert_eq!(
        out.telemetry.diagnostic_event_count as usize,
        out.diagnostics.events.len()
    );
}
