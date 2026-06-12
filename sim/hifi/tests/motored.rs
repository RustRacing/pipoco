use ecu_sim_hifi::{
    run_motored_cycle, CylinderGeometry, EventTable, GasProperties, GeometryModel,
    InitialChargeState, IntegratorConfig, MotoredCylinderConfig, ThermalBoundary,
};

fn config_with_step(step_deg: f64) -> MotoredCylinderConfig {
    MotoredCylinderConfig {
        geometry: CylinderGeometry::new(0.086, 0.086, 0.143, 10.0),
        wall: ThermalBoundary {
            wall_temperature_k: 360.0,
        },
        initial_charge: InitialChargeState {
            pressure_pa: 101_325.0,
            temperature_k: 293.15,
            crank_angle_rad: 180.0_f64.to_radians(),
        },
        gas: GasProperties::new(287.0, 718.0),
        integrator: IntegratorConfig { step_deg },
    }
}

#[test]
fn adiabatic_cycle_closes_to_nearly_zero_work() {
    let result = run_motored_cycle(config_with_step(0.2), &EventTable::empty()).unwrap();
    let swept_volume = GeometryModel::new(config_with_step(0.2).geometry).swept_volume_m3();
    let reference_work = config_with_step(0.2).initial_charge.pressure_pa * swept_volume;

    assert!(result.indicated_work_j.abs() < reference_work * 1.0e-4);
}

#[test]
fn compression_branch_matches_polytropic_invariant_and_converges() {
    let coarse = run_motored_cycle(config_with_step(0.5), &EventTable::empty()).unwrap();
    let fine = run_motored_cycle(config_with_step(0.25), &EventTable::empty()).unwrap();
    let gamma = config_with_step(0.5).gas.gamma();

    let coarse_error = max_invariant_error(&coarse, gamma);
    let fine_error = max_invariant_error(&fine, gamma);

    assert!(coarse_error < 5.0e-3, "coarse error = {coarse_error}");
    assert!(
        fine_error < coarse_error,
        "fine={fine_error} coarse={coarse_error}"
    );
}

fn max_invariant_error(result: &ecu_sim_hifi::MotoredCycleResult, gamma: f64) -> f64 {
    let compression_branch: Vec<_> = result
        .samples
        .iter()
        .filter(|sample| sample.crank_angle_rad <= 360.0_f64.to_radians())
        .collect();
    let baseline = compression_branch
        .first()
        .map(|sample| sample.pressure_pa * sample.volume_m3.powf(gamma))
        .unwrap();

    compression_branch
        .into_iter()
        .map(|sample| {
            let invariant = sample.pressure_pa * sample.volume_m3.powf(gamma);
            ((invariant - baseline) / baseline).abs()
        })
        .fold(0.0, f64::max)
}
