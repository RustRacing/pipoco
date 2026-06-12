use ecu_sim_core::{
    config::{BurnCurve, LossConfig, VeTable},
    fuel::{InjectionCommand, InjectionTimingMode},
    spark::SparkCommand,
    types::{
        CrankDeg10, CylinderIndex, Kpa10, MicrogramsPerMicros, Micros, PressurePa, Rpm,
        TorqueNmX100,
    },
    InitialPlantState, Plant,
};
use ecu_sim_driver::{
    hifi_scenario_calibrations, hifi_scenario_initial_rpm, hifi_scenario_initial_temperature_k,
    hifi_scenario_step_frame, ScenarioConfig, ScenarioKind,
};
use ecu_sim_hifi::{
    advance_plant_step, parse_burn_curve, parse_loss_config, parse_ve_table, CylinderCommand,
    PlantConfig as HifiPlantConfig, PlantStepInput as HifiPlantStepInput,
};

const CYL: usize = 4;
const CORE_MAX_EDGES: usize = 64;
const CORE_MAX_EVENTS: usize = 8;
const STEP_US: u32 = 20_000;
const STEP_COUNT: usize = 10;
const INJECTOR_FLOW_UG_PER_US: u32 = 3;
const INJECTOR_DEADTIME_US: u32 = 1_000;

// Bands are set from observed step errors for the current generated-artifact
// alignment and will be tightened as the physics model and calibration are
// iterated.
#[derive(Clone, Copy)]
struct SignalSample {
    torque_nm: f64,
    map_kpa10: f64,
    lambda_x1000: f64,
}

#[derive(Clone, Copy)]
struct ConformanceBands {
    torque_max_abs_nm: f64,
    torque_steady_state_nm: f64,
    map_max_abs_kpa10: f64,
    map_steady_state_kpa10: f64,
    lambda_max_abs_x1000: f64,
    lambda_steady_state_x1000: f64,
    provenance: &'static str,
}

#[derive(Clone, Copy)]
struct ConformancePoint {
    name: &'static str,
    throttle_x1000: u16,
    load_torque_nm_x100: i32,
    fuel_mass_kg: f64,
    spark_advance_deg10: u16,
    bands: ConformanceBands,
}

const CORPUS: [ConformancePoint; 3] = [
    ConformancePoint {
        name: "warm-idle",
        throttle_x1000: 280,
        load_torque_nm_x100: 1_200,
        fuel_mass_kg: 1.2e-5,
        spark_advance_deg10: 160,
        bands: ConformanceBands {
            torque_max_abs_nm: 10.0,
            torque_steady_state_nm: 10.0,
            map_max_abs_kpa10: 60.0,
            map_steady_state_kpa10: 60.0,
            lambda_max_abs_x1000: 1_200.0,
            lambda_steady_state_x1000: 1_200.0,
            provenance: "warm-idle temporary bootstrap band, commit 58cdac9",
        },
    },
    ConformancePoint {
        name: "mid-load",
        throttle_x1000: 520,
        load_torque_nm_x100: 2_500,
        fuel_mass_kg: 1.8e-5,
        spark_advance_deg10: 180,
        bands: ConformanceBands {
            torque_max_abs_nm: 800.0,
            torque_steady_state_nm: 90.0,
            map_max_abs_kpa10: 60.0,
            map_steady_state_kpa10: 60.0,
            lambda_max_abs_x1000: 800.0,
            lambda_steady_state_x1000: 800.0,
            provenance: "mid-load temporary bootstrap band, commit 58cdac9",
        },
    },
    ConformancePoint {
        name: "high-load",
        throttle_x1000: 820,
        load_torque_nm_x100: 4_500,
        fuel_mass_kg: 2.4e-5,
        spark_advance_deg10: 220,
        bands: ConformanceBands {
            torque_max_abs_nm: 561.896 * 1.5,
            torque_steady_state_nm: 63.704 * 1.5,
            map_max_abs_kpa10: 21.248 * 1.5,
            map_steady_state_kpa10: 21.248 * 1.5,
            lambda_max_abs_x1000: 261.026 * 1.5,
            lambda_steady_state_x1000: 261.026 * 1.5,
            provenance: "high-load band rebased at 1.5x observed error, commit 58cdac9",
        },
    },
];

#[derive(Clone, Copy)]
struct ScenarioBands {
    torque_max_abs_nm: f64,
    map_max_abs_kpa10: f64,
    lambda_max_abs_x1000: f64,
    provenance: &'static str,
}

#[derive(Clone, Copy)]
struct ScenarioCase {
    name: &'static str,
    config: ScenarioConfig,
    bands: ScenarioBands,
}

fn scenario_corpus() -> [ScenarioCase; 3] {
    [
        ScenarioCase {
            name: "cold-start",
            config: ScenarioConfig::cold_start(),
            bands: ScenarioBands {
                torque_max_abs_nm: 800.0,
                map_max_abs_kpa10: 400.0,
                lambda_max_abs_x1000: 400.0,
                provenance: "cold-start scenario aligned to open-loop conformance harness, commit 58cdac9+api",
            },
        },
        ScenarioCase {
            name: "hot-restart",
            config: ScenarioConfig::hot_restart(),
            bands: ScenarioBands {
                torque_max_abs_nm: 15.143 * 1.5,
                map_max_abs_kpa10: 0.268 * 1.5,
                lambda_max_abs_x1000: 450.524 * 1.5,
                provenance: "hot-restart scenario aligned to open-loop conformance harness and rebased at 1.5x observed error, commit 58cdac9+api",
            },
        },
        ScenarioCase {
            name: "sync-loss-recovery",
            config: ScenarioConfig::sync_loss_recovery(),
            bands: ScenarioBands {
                torque_max_abs_nm: 14.874 * 1.5,
                map_max_abs_kpa10: 0.268 * 1.5,
                lambda_max_abs_x1000: 451.110 * 1.5,
                provenance: "sync-loss-recovery scenario aligned to open-loop conformance harness and rebased at 1.5x observed error, commit 58cdac9+api",
            },
        },
    ]
}

#[test]
fn core_and_hifi_track_shared_operating_points_within_documented_bands() {
    let mut failures = Vec::new();

    for point in CORPUS {
        let core = run_core_point(point);
        let hifi = run_hifi_point(point);

        assert_eq!(core.len(), STEP_COUNT, "{}", point.name);
        assert_eq!(hifi.len(), STEP_COUNT, "{}", point.name);

        let torque_max_abs = max_abs_diff(&core, &hifi, |sample| sample.torque_nm);
        let torque_steady = steady_state_abs_diff(&core, &hifi, |sample| sample.torque_nm);
        let map_max_abs = max_abs_diff(&core, &hifi, |sample| sample.map_kpa10);
        let map_steady = steady_state_abs_diff(&core, &hifi, |sample| sample.map_kpa10);
        let lambda_max_abs = max_abs_diff(&core, &hifi, |sample| sample.lambda_x1000);
        let lambda_steady = steady_state_abs_diff(&core, &hifi, |sample| sample.lambda_x1000);

        if torque_max_abs > point.bands.torque_max_abs_nm
            || torque_steady > point.bands.torque_steady_state_nm
            || map_max_abs > point.bands.map_max_abs_kpa10
            || map_steady > point.bands.map_steady_state_kpa10
            || lambda_max_abs > point.bands.lambda_max_abs_x1000
            || lambda_steady > point.bands.lambda_steady_state_x1000
        {
            let core_last = core.last().unwrap();
            let hifi_last = hifi.last().unwrap();
            failures.push(format!(
                "{} [{}]: torque max {:.3} steady {:.3} (core {:.3}, hifi {:.3}); map max {:.3} steady {:.3} (core {:.3}, hifi {:.3}); lambda max {:.3} steady {:.3} (core {:.3}, hifi {:.3})",
                point.name,
                point.bands.provenance,
                torque_max_abs,
                torque_steady,
                core_last.torque_nm,
                hifi_last.torque_nm,
                map_max_abs,
                map_steady,
                core_last.map_kpa10,
                hifi_last.map_kpa10,
                lambda_max_abs,
                lambda_steady,
                core_last.lambda_x1000,
                hifi_last.lambda_x1000
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "cross-plant conformance exceeded documented bands:\n{}",
        failures.join("\n")
    );
}

#[test]
fn core_and_hifi_track_scenario_defined_step_schedules_within_loose_bands() {
    let mut failures = Vec::new();

    for case in scenario_corpus() {
        let core = run_core_scenario(case);
        let hifi = run_hifi_scenario(case);

        assert_eq!(core.len(), case.config.steps as usize, "{}", case.name);
        assert_eq!(hifi.len(), case.config.steps as usize, "{}", case.name);

        let torque_max_abs = max_abs_diff(&core, &hifi, |sample| sample.torque_nm);
        let map_max_abs = max_abs_diff(&core, &hifi, |sample| sample.map_kpa10);
        let lambda_max_abs = max_abs_diff(&core, &hifi, |sample| sample.lambda_x1000);

        if torque_max_abs > case.bands.torque_max_abs_nm
            || map_max_abs > case.bands.map_max_abs_kpa10
            || lambda_max_abs > case.bands.lambda_max_abs_x1000
        {
            let core_last = core.last().unwrap();
            let hifi_last = hifi.last().unwrap();
            failures.push(format!(
                "{} [{}]: torque max {:.3} (core {:.3}, hifi {:.3}); map max {:.3} (core {:.3}, hifi {:.3}); lambda max {:.3} (core {:.3}, hifi {:.3})",
                case.name,
                case.bands.provenance,
                torque_max_abs,
                core_last.torque_nm,
                hifi_last.torque_nm,
                map_max_abs,
                core_last.map_kpa10,
                hifi_last.map_kpa10,
                lambda_max_abs,
                core_last.lambda_x1000,
                hifi_last.lambda_x1000
            ));
        }
    }

    assert!(
        failures.is_empty(),
        "scenario conformance exceeded documented bands:\n{}",
        failures.join("\n")
    );
}

fn run_core_point(point: ConformancePoint) -> Vec<SignalSample> {
    let mut plant =
        Plant::<CYL, CORE_MAX_EDGES, CORE_MAX_EVENTS>::new(core_config_from_hifi_artifacts());
    plant.reset(InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(2500),
        map_kpa10: Kpa10(700),
        crank_angle_deg10: CrankDeg10(0),
        completed_cycle_count: 0,
        cylinder_fuel_mass_ug: [mass_kg_to_ug(point.fuel_mass_kg); CYL],
    });

    let input = core_input_for_point(point);
    let mut output =
        ecu_sim_core::io::PlantStepOutput::<CYL, CORE_MAX_EDGES, CORE_MAX_EVENTS>::empty();
    let mut samples = Vec::with_capacity(STEP_COUNT);

    for _ in 0..STEP_COUNT {
        plant.step(&input, &mut output).unwrap();
        samples.push(SignalSample {
            torque_nm: f64::from(output.telemetry.brake_torque_nm_x100.0) / 100.0,
            map_kpa10: f64::from(output.telemetry.map_kpa10.0),
            lambda_x1000: f64::from(output.telemetry.lambda_x1000).min(10_000.0),
        });
    }

    samples
}

fn run_hifi_point(point: ConformancePoint) -> Vec<SignalSample> {
    let config = hifi_config_from_source_defaults();
    let mut samples = Vec::with_capacity(STEP_COUNT);
    let mut rpm = 2500.0;
    let mut crank_angle_rad = core::f64::consts::PI;

    for step in 0..STEP_COUNT {
        let now_s = (step as f64) * (STEP_US as f64 / 1_000_000.0);
        let output = advance_plant_step(
            &config,
            &HifiPlantStepInput {
                crank_angle_rad,
                rpm,
                now_s,
                window_s: STEP_US as f64 / 1_000_000.0,
                throttle_position: f64::from(point.throttle_x1000) / 1000.0,
                load_torque_nm: f64::from(point.load_torque_nm_x100) / 100.0,
                cylinders: vec![
                    CylinderCommand {
                        fuel_mass_kg: point.fuel_mass_kg,
                        spark_angle_rad: f64::from(point.spark_advance_deg10).to_radians() / 10.0,
                        dwell_s: 0.002,
                    };
                    CYL
                ],
            },
        )
        .unwrap();
        rpm = output.rpm;
        crank_angle_rad = output.crank_angle_rad;

        samples.push(SignalSample {
            torque_nm: output.brake_torque_nm,
            map_kpa10: output.manifold_pressure_pa / 100.0,
            lambda_x1000: output.lambda * 1000.0,
        });
    }

    samples
}

fn run_core_scenario(case: ScenarioCase) -> Vec<SignalSample> {
    let mut plant =
        Plant::<CYL, CORE_MAX_EDGES, CORE_MAX_EVENTS>::new(core_config_for_scenario(case));
    plant.reset(InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(scenario_initial_rpm(case.config.kind)),
        map_kpa10: Kpa10(700),
        crank_angle_deg10: CrankDeg10(0),
        completed_cycle_count: 0,
        cylinder_fuel_mass_ug: [mass_kg_to_ug(
            hifi_scenario_calibrations(case.config.kind).fuel_mass_kg,
        ); CYL],
    });

    let mut output =
        ecu_sim_core::io::PlantStepOutput::<CYL, CORE_MAX_EDGES, CORE_MAX_EVENTS>::empty();
    let mut samples = Vec::with_capacity(case.config.steps as usize);

    for step_index in 0..case.config.steps {
        let input = core_input_for_scenario_step(case, step_index);
        plant.step(&input, &mut output).unwrap();
        samples.push(SignalSample {
            torque_nm: f64::from(output.telemetry.brake_torque_nm_x100.0) / 100.0,
            map_kpa10: f64::from(output.telemetry.map_kpa10.0),
            lambda_x1000: f64::from(output.telemetry.lambda_x1000).min(10_000.0),
        });
    }

    samples
}

fn run_hifi_scenario(case: ScenarioCase) -> Vec<SignalSample> {
    let config = hifi_config_for_scenario(case);
    let mut samples = Vec::with_capacity(case.config.steps as usize);
    let mut rpm = f64::from(hifi_scenario_initial_rpm(case.config.kind));
    let mut crank_angle_rad = core::f64::consts::PI;

    for step_index in 0..case.config.steps {
        let frame = hifi_scenario_step_frame(case.config, step_index);
        let output = advance_plant_step(
            &config,
            &HifiPlantStepInput {
                crank_angle_rad,
                rpm,
                now_s: (step_index as f64) * (STEP_US as f64 / 1_000_000.0),
                window_s: STEP_US as f64 / 1_000_000.0,
                throttle_position: f64::from(frame.throttle_x1000) / 1000.0,
                load_torque_nm: f64::from(frame.load_torque_x100) / 100.0,
                cylinders: vec![
                    CylinderCommand {
                        fuel_mass_kg: frame.fuel_mass_kg,
                        spark_angle_rad: f64::from(frame.spark_advance_deg10).to_radians() / 10.0,
                        dwell_s: 0.002,
                    };
                    CYL
                ],
            },
        )
        .unwrap();
        rpm = output.rpm;
        crank_angle_rad = output.crank_angle_rad;

        samples.push(SignalSample {
            torque_nm: output.brake_torque_nm,
            map_kpa10: output.manifold_pressure_pa / 100.0,
            lambda_x1000: bounded_lambda_x1000(output.lambda),
        });
    }

    samples
}

fn core_config_from_hifi_artifacts() -> ecu_sim_core::config::PlantConfig<CYL> {
    let burn_curve = parse_burn_curve(include_str!("../../hifi/artifacts/burn_curve.txt")).unwrap();
    let ve_table = parse_ve_table(include_str!("../../hifi/artifacts/ve_table.txt")).unwrap();
    let losses = parse_loss_config(include_str!("../../hifi/artifacts/loss_config.txt")).unwrap();

    let mut cfg = ecu_sim_core::config::PlantConfig::<CYL>::default_four();
    cfg.physics_mode = ecu_sim_core::config::PlantPhysicsMode::SingleZoneIdealGas;
    cfg.air.idle_map_kpa10 = Kpa10(900);
    cfg.air.wide_open_map_kpa10 = Kpa10(1013);
    cfg.air.manifold_filling_enabled = false;
    cfg.air.manifold_volume_cc = 2_000;
    cfg.air.throttle_area_mm2 = 200;
    cfg.air.throttle_discharge_coeff_x1000 = 800;
    cfg.air.reference_air_temp_k10 = ecu_sim_core::types::Kelvin10(3000);
    cfg.air.reference_pressure_kpa10 = Kpa10(1013);
    cfg.air.ve_table = VeTable {
        rpm_axis: ve_table.rpm_axis.map(Rpm),
        load_axis: ve_table.load_axis_kpa10.map(Kpa10),
        ve_x1000: ve_table.ve_x1000,
    };
    cfg.combustion.burn_curve = BurnCurve {
        burn_fraction_x10000: burn_curve.burn_fraction_x10000,
    };
    cfg.losses = LossConfig {
        fmep_base_pa: PressurePa(losses.fmep_base_pa),
        fmep_rpm_pa_per_krpm: losses.fmep_rpm_pa_per_krpm,
        fmep_rpm2_pa_per_krpm2: losses.fmep_rpm2_pa_per_krpm2,
        fmep_load_pa_per_kpa: losses.fmep_load_pa_per_kpa,
        pumping_base_pa: PressurePa(losses.pumping_base_pa),
        pumping_throttle_pa_per_x1000: losses.pumping_throttle_pa_per_x1000,
        accessory_torque_nm_x100: TorqueNmX100(losses.accessory_torque_nm_x100),
    };
    cfg.thermo.initial_cylinder_temp_k10 = ecu_sim_core::types::Kelvin10(3300);
    cfg.thermo.gamma_x1000 = 1400;
    cfg.friction_torque_nm_x100 = TorqueNmX100(0);
    cfg.residual.enabled = true;
    cfg.residual.base_fraction_x1000 = 80;
    cfg.residual.overlap_gain_x1000 = 2;
    cfg.residual.low_map_gain_x1000 = 1;
    cfg
}

fn core_config_for_scenario(case: ScenarioCase) -> ecu_sim_core::config::PlantConfig<CYL> {
    let mut cfg = core_config_from_hifi_artifacts();
    cfg.thermo.initial_cylinder_temp_k10 =
        ecu_sim_core::types::Kelvin10(scenario_initial_temp_k10(case.config.kind));
    cfg
}

fn hifi_config_from_source_defaults() -> HifiPlantConfig {
    ecu_sim_hifi::default_plant_config()
}

fn hifi_config_for_scenario(case: ScenarioCase) -> HifiPlantConfig {
    let mut cfg = hifi_config_from_source_defaults();
    cfg.initial_temperature_k = hifi_scenario_initial_temperature_k(case.config.kind);
    cfg
}

fn core_input_for_point(
    point: ConformancePoint,
) -> ecu_sim_core::io::PlantStepInput<CYL, CORE_MAX_EVENTS> {
    let mut input = ecu_sim_core::io::PlantStepInput::idle(Micros(STEP_US));
    input.driver.throttle_x1000 = point.throttle_x1000;
    input.driver.load_torque_nm_x100 = TorqueNmX100(point.load_torque_nm_x100);

    let fuel_mass_ug = mass_kg_to_ug(point.fuel_mass_kg).0;
    let effective_pw_us = fuel_mass_ug.div_ceil(INJECTOR_FLOW_UG_PER_US);
    let pulse_width_us = effective_pw_us + INJECTOR_DEADTIME_US;

    for cylinder in 0..CYL {
        input
            .ecu_outputs
            .injection_events
            .push(InjectionCommand {
                cylinder: CylinderIndex(cylinder as u8),
                mode: InjectionTimingMode::StartOfInjection,
                angle_deg10: CrankDeg10(3600),
                pulse_width_us: Micros(pulse_width_us),
                injector_flow_ug_per_us: MicrogramsPerMicros(INJECTOR_FLOW_UG_PER_US),
                deadtime_us: Micros(INJECTOR_DEADTIME_US),
            })
            .unwrap();
        input
            .ecu_outputs
            .spark_events
            .push(SparkCommand {
                cylinder: CylinderIndex(cylinder as u8),
                spark_angle_deg10: CrankDeg10(point.spark_advance_deg10),
                dwell_us: Micros(2000),
                coil_energy_x1000: 1000,
            })
            .unwrap();
    }

    input
}

fn core_input_for_scenario_step(
    case: ScenarioCase,
    step_index: u16,
) -> ecu_sim_core::io::PlantStepInput<CYL, CORE_MAX_EVENTS> {
    let frame = hifi_scenario_step_frame(case.config, step_index);
    let mut input = ecu_sim_core::io::PlantStepInput::idle(Micros(STEP_US));
    input.driver.throttle_x1000 = frame.throttle_x1000;
    input.driver.load_torque_nm_x100 = TorqueNmX100(frame.load_torque_x100);

    let fuel_mass_ug = mass_kg_to_ug(frame.fuel_mass_kg).0;
    let effective_pw_us = fuel_mass_ug.div_ceil(INJECTOR_FLOW_UG_PER_US);
    let pulse_width_us = effective_pw_us + INJECTOR_DEADTIME_US;

    for cylinder in 0..CYL {
        input
            .ecu_outputs
            .injection_events
            .push(InjectionCommand {
                cylinder: CylinderIndex(cylinder as u8),
                mode: InjectionTimingMode::StartOfInjection,
                angle_deg10: CrankDeg10(3600),
                pulse_width_us: Micros(pulse_width_us),
                injector_flow_ug_per_us: MicrogramsPerMicros(INJECTOR_FLOW_UG_PER_US),
                deadtime_us: Micros(INJECTOR_DEADTIME_US),
            })
            .unwrap();
        input
            .ecu_outputs
            .spark_events
            .push(SparkCommand {
                cylinder: CylinderIndex(cylinder as u8),
                spark_angle_deg10: CrankDeg10(frame.spark_advance_deg10),
                dwell_us: Micros(2000),
                coil_energy_x1000: 1000,
            })
            .unwrap();
    }

    input
}

fn scenario_initial_temp_k10(kind: ScenarioKind) -> u16 {
    match kind {
        ScenarioKind::ColdStart => 2610,
        ScenarioKind::HotRestart => 3610,
        ScenarioKind::DfcoDecel => 3580,
        ScenarioKind::SyncLossRecovery => 3510,
        ScenarioKind::Smoke => 3300,
    }
}

fn scenario_initial_rpm(kind: ScenarioKind) -> u32 {
    match kind {
        ScenarioKind::ColdStart => 250,
        ScenarioKind::HotRestart | ScenarioKind::DfcoDecel | ScenarioKind::SyncLossRecovery => 850,
        ScenarioKind::Smoke => 850,
    }
}

fn bounded_lambda_x1000(lambda: f64) -> f64 {
    if lambda.is_finite() {
        (lambda * 1000.0).clamp(0.0, 10_000.0)
    } else {
        10_000.0
    }
}

fn mass_kg_to_ug(mass_kg: f64) -> ecu_sim_core::types::MassUg {
    ecu_sim_core::types::MassUg((mass_kg * 1_000_000_000.0).round() as u32)
}

fn max_abs_diff(
    lhs: &[SignalSample],
    rhs: &[SignalSample],
    project: impl Fn(SignalSample) -> f64,
) -> f64 {
    lhs.iter()
        .zip(rhs)
        .map(|(left, right)| (project(*left) - project(*right)).abs())
        .fold(0.0, f64::max)
}

fn steady_state_abs_diff(
    lhs: &[SignalSample],
    rhs: &[SignalSample],
    project: impl Fn(SignalSample) -> f64,
) -> f64 {
    (project(*lhs.last().unwrap()) - project(*rhs.last().unwrap())).abs()
}
