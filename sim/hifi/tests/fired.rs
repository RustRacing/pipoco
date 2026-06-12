use ecu_sim_hifi::{
    run_fired_cycle, BurnModel, CombustionConfig, CylinderGeometry, FiredCycleConfig,
    GasProperties, GeometryModel, IntegratorConfig, LossCorrelationConfig, ThermalBoundary,
    WoschniConfig,
};

fn base_config() -> FiredCycleConfig {
    FiredCycleConfig {
        geometry: CylinderGeometry::new(0.086, 0.086, 0.143, 10.0),
        wall: ThermalBoundary {
            wall_temperature_k: 420.0,
        },
        gas: GasProperties::new(287.0, 718.0),
        burned_gas: GasProperties::new(300.0, 800.0),
        integrator: IntegratorConfig { step_deg: 0.1 },
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

#[test]
fn spark_advance_moves_ca50_earlier() {
    let mut retarded = base_config();
    retarded.combustion.spark_angle_rad = 0.0_f64.to_radians();
    let mut advanced = base_config();
    advanced.combustion.spark_angle_rad = 20.0_f64.to_radians();

    let retarded_result = run_fired_cycle(retarded).unwrap();
    let advanced_result = run_fired_cycle(advanced).unwrap();

    assert!(advanced_result.ca50_rad.unwrap() < retarded_result.ca50_rad.unwrap());
}

#[test]
fn trapped_air_and_fuel_raise_brake_torque() {
    let low = run_fired_cycle(base_config()).unwrap();
    let mut high = base_config();
    high.initial_pressure_pa = 112_000.0;
    high.manifold_pressure_pa = 108_000.0;
    high.combustion.fuel_mass_kg = 3.8e-5;

    let high_result = run_fired_cycle(high).unwrap();
    assert!(high_result.brake_torque_nm > low.brake_torque_nm);
}

#[test]
fn enabling_wall_heat_loss_reduces_imep() {
    let mut no_loss = base_config();
    no_loss.woschni.c = 1.0e-12;
    let mut with_loss = base_config();
    with_loss.woschni.c2 = 0.00324;

    let no_loss_result = run_fired_cycle(no_loss).unwrap();
    let with_loss_result = run_fired_cycle(with_loss).unwrap();

    assert!(with_loss_result.wall_heat_j > 0.0);
    assert!(with_loss_result.imep_gross_pa < no_loss_result.imep_gross_pa);
}

#[test]
fn double_wiebe_premixed_fraction_moves_pmax_earlier() {
    let mut single = base_config();
    single.combustion.burn_model = BurnModel::SingleWiebe {
        a: 5.0,
        m: 2.0,
        duration_rad: 50.0_f64.to_radians(),
        spark_to_soc_delay_rad: 5.0_f64.to_radians(),
    };
    let mut double = base_config();
    double.combustion.burn_model = BurnModel::DoubleWiebe {
        premixed_fraction: 0.45,
        premixed_a: 6.0,
        premixed_m: 1.5,
        premixed_duration_rad: 20.0_f64.to_radians(),
        main_a: 5.0,
        main_m: 2.0,
        main_duration_rad: 55.0_f64.to_radians(),
        spark_to_soc_delay_rad: 5.0_f64.to_radians(),
    };

    let single_result = run_fired_cycle(single).unwrap();
    let double_result = run_fired_cycle(double).unwrap();

    assert!(double_result.pmax_angle_rad < single_result.pmax_angle_rad);
}

#[test]
fn fired_cycle_closes_energy_balance_within_reasonable_tolerance() {
    let config = base_config();
    let geometry = GeometryModel::new(config.geometry);
    let cycle_start = core::f64::consts::PI;
    let released_heat = config.combustion.fuel_mass_kg
        * config.combustion.fuel_lhv_j_per_kg
        * config.combustion.combustion_efficiency;
    let result = run_fired_cycle(config).unwrap();
    let start_sample = result.samples.first().expect("fired sample exists");
    let end_sample = result.samples.last().expect("fired sample exists");
    let start_fresh_fraction = 1.0 - config.initial_burned_fraction;
    let start_gas = GasProperties::new(
        config.gas.r_j_per_kg_k * start_fresh_fraction
            + config.burned_gas.r_j_per_kg_k * config.initial_burned_fraction,
        config.gas.cv_j_per_kg_k * start_fresh_fraction
            + config.burned_gas.cv_j_per_kg_k * config.initial_burned_fraction,
    );
    let end_burn_fraction = end_sample.burn_fraction;
    let end_gas = GasProperties::new(
        config.gas.r_j_per_kg_k * (1.0 - end_burn_fraction)
            + config.burned_gas.r_j_per_kg_k * end_burn_fraction,
        config.gas.cv_j_per_kg_k * (1.0 - end_burn_fraction)
            + config.burned_gas.cv_j_per_kg_k * end_burn_fraction,
    );
    let initial_mass_kg = if let Some(initial_mass) = config.initial_mass_kg {
        initial_mass
    } else {
        config.initial_pressure_pa * geometry.volume_m3(cycle_start)
            / (start_gas.r_j_per_kg_k * config.initial_temperature_k)
    };
    let final_mass_kg = end_sample.pressure_pa * geometry.volume_m3(end_sample.crank_angle_rad)
        / (end_gas.r_j_per_kg_k * end_sample.temperature_k);
    let delta_internal_energy = final_mass_kg * end_gas.cv_j_per_kg_k * end_sample.temperature_k
        - initial_mass_kg * start_gas.cv_j_per_kg_k * start_sample.temperature_k;
    let accounted = result.indicated_work_j + result.wall_heat_j + delta_internal_energy;
    let relative_error = ((released_heat - accounted) / released_heat).abs();
    assert!(
        relative_error < 0.02,
        "released heat {released_heat:.3} J should match work + heat + ΔU within 2%, got relative error {relative_error:.6}"
    );
}

#[test]
fn fired_cycle_reports_ordered_burn_phasing_and_peak_pressure() {
    let result = run_fired_cycle(base_config()).unwrap();

    let ca10 = result.ca10_rad.expect("burn phasing should be reported");
    let ca50 = result.ca50_rad.expect("burn phasing should be reported");
    let ca90 = result.ca90_rad.expect("burn phasing should be reported");
    let firing_tdc_rad = 720.0_f64.to_radians();
    let max_temperature_k = result
        .samples
        .iter()
        .map(|sample| sample.temperature_k)
        .fold(f64::NEG_INFINITY, f64::max);
    assert!(ca10 < ca50);
    assert!(ca50 < ca90);
    assert!(ca50 > firing_tdc_rad);
    assert!(ca50 < firing_tdc_rad + 40.0_f64.to_radians());
    assert!((30.0..=80.0).contains(&(result.pmax_pa / 1.0e5)));
    assert!((2000.0..=3000.0).contains(&max_temperature_k));
    assert!(result.brake_torque_nm > 20.0);
    assert!(result.pmax_angle_rad > firing_tdc_rad);
    assert!(result.pmax_angle_rad < firing_tdc_rad + 40.0_f64.to_radians());
}

#[test]
fn woschni_wall_heat_transfer_stays_reasonable_at_high_pressure() {
    let cycle = base_config();

    let combustion_term = cycle.woschni.c2
        * ((cycle.geometry.swept_volume_m3() * cycle.woschni.t_ref_k)
            / (cycle.woschni.p_ref_pa * cycle.woschni.v_ref_m3))
        * (5_000_000.0 - 200_000.0);
    let velocity = (cycle.woschni.c1 * 51.5 + combustion_term).max(1.0e-6_f64);
    let h = cycle.woschni.c
        * (5_000_000.0_f64 / 1000.0).powf(0.8_f64)
        * cycle.geometry.bore_m.powf(-0.2)
        * 2000.0_f64.powf(-0.55)
        * velocity.powf(0.8);
    let _wall_heat_rate_j_per_rad = h
        * GeometryModel::new(cycle.geometry).wall_area_m2(540.0_f64.to_radians())
        * (2000.0 - cycle.wall.wall_temperature_k).max(0.0)
        / 300.0_f64;

    assert!(
        (500.0..5000.0).contains(&h),
        "woschni wall heat should be in expected range, got {h}"
    );
}
