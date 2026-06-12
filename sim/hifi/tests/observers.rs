use ecu_sim_hifi::{
    estimate_exhaust, estimate_knock, observe_lambda, run_fired_cycle, BurnModel, CombustionConfig,
    CylinderGeometry, FiredCycleConfig, GasProperties, IntegratorConfig, KnockModelConfig,
    LossCorrelationConfig, ThermalBoundary, WoschniConfig,
};

fn base_config() -> FiredCycleConfig {
    FiredCycleConfig {
        geometry: CylinderGeometry::new(0.086, 0.086, 0.143, 10.0),
        wall: ThermalBoundary {
            wall_temperature_k: 420.0,
        },
        gas: GasProperties::new(287.0, 718.0),
        burned_gas: GasProperties::new(300.0, 800.0),
        integrator: IntegratorConfig { step_deg: 0.5 },
        initial_pressure_pa: 101_325.0,
        initial_temperature_k: 330.0,
        initial_mass_kg: None,
        initial_burned_fraction: 0.08,
        rpm: 2500.0,
        closed_cycle_start_rad: 580.0_f64.to_radians(),
        closed_cycle_end_rad: (120.0_f64 + 720.0).to_radians(),
        manifold_pressure_pa: 95_000.0,
        open_system_pmep_pa: 500.0,
        combustion: CombustionConfig {
            burn_model: BurnModel::SingleWiebe {
                a: 5.0,
                m: 2.0,
                duration_rad: 50.0_f64.to_radians(),
                spark_to_soc_delay_rad: 5.0_f64.to_radians(),
            },
            spark_angle_rad: 10.0_f64.to_radians(),
            fuel_mass_kg: 3.4e-5,
            fuel_lhv_j_per_kg: 43_000_000.0,
            combustion_efficiency: 0.95,
            stoich_afr: 14.7,
        },
        woschni: WoschniConfig {
            c: 3.26,
            c1: 2.28,
            c2: 0.00324,
            t_ref_k: 300.0,
            p_ref_pa: 101_325.0,
            v_ref_m3: 5.0e-4,
        },
        losses: LossCorrelationConfig {
            fmep_base_pa: 500.0,
            fmep_rpm_pa_per_krpm: 100.0,
            fmep_rpm2_pa_per_krpm2: 10.0,
            fmep_load_pa_per_kpa: 0.0,
        },
    }
}

fn knock_model() -> KnockModelConfig {
    KnockModelConfig {
        octane_number: 95.0,
        a: 0.01768,
        n1: 3.402,
        n2: -1.7,
        b: 3800.0,
    }
}

#[test]
fn knock_model_distinguishes_knocking_and_mild_conditions() {
    let mut knocking = base_config();
    knocking.geometry = CylinderGeometry::new(0.086, 0.086, 0.143, 12.0);
    knocking.initial_temperature_k = 330.0;
    knocking.combustion.spark_angle_rad = 30.0_f64.to_radians();
    knocking.combustion.fuel_mass_kg = 4.5e-5;
    let mut mild = base_config();
    mild.geometry = CylinderGeometry::new(0.086, 0.086, 0.143, 9.0);
    mild.initial_temperature_k = 290.0;
    mild.combustion.spark_angle_rad = 5.0_f64.to_radians();
    mild.combustion.fuel_mass_kg = 2.4e-5;

    let knocking_cycle = run_fired_cycle(knocking).unwrap();
    let mild_cycle = run_fired_cycle(mild).unwrap();
    let knocking_obs = estimate_knock(
        &knocking_cycle,
        knocking,
        KnockModelConfig {
            octane_number: 85.0,
            ..knock_model()
        },
    );
    let mild_obs = estimate_knock(
        &mild_cycle,
        mild,
        KnockModelConfig {
            octane_number: 100.0,
            ..knock_model()
        },
    );

    assert!(knocking_obs.knock_integral >= 1.0);
    assert!(knocking_obs.predicted_onset_angle_rad.is_some());
    assert!(mild_obs.knock_integral < 0.5);
    assert!(mild_obs.predicted_onset_angle_rad.is_none());
}

#[test]
fn lambda_observer_matches_cycle_lambda() {
    let result = run_fired_cycle(base_config()).unwrap();
    assert_eq!(observe_lambda(&result), result.lambda);
}

#[test]
fn exhaust_temperature_rises_with_spark_retard() {
    let mut advanced = base_config();
    advanced.combustion.spark_angle_rad = 20.0_f64.to_radians();
    let mut retarded = base_config();
    retarded.combustion.spark_angle_rad = 10.0_f64.to_radians();

    let advanced_cycle = run_fired_cycle(advanced).unwrap();
    let retarded_cycle = run_fired_cycle(retarded).unwrap();
    let retarded_obs = estimate_exhaust(&retarded_cycle);
    let advanced_obs = estimate_exhaust(&advanced_cycle);

    assert!(retarded_obs.mean_exhaust_temperature_k > advanced_obs.mean_exhaust_temperature_k);
    assert!(retarded_obs.exhaust_enthalpy_j > advanced_obs.exhaust_enthalpy_j);
}

#[test]
fn observers_do_not_mutate_cycle_results() {
    let config = base_config();
    let result = run_fired_cycle(config).unwrap();
    let snapshot = result.clone();

    let _ = estimate_knock(&result, config, knock_model());
    let _ = observe_lambda(&result);
    let _ = estimate_exhaust(&result);

    assert_eq!(result, snapshot);
}
