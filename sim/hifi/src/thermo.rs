use crate::{
    geometry::GeometryModel,
    params::{GasProperties, ManifoldConfig, MotoredCylinderConfig},
    state::{ClosedSystemState, CompositionState, CycleConvergence, ManifoldState},
};

pub fn ideal_gas_pressure_pa(state: ClosedSystemState, volume_m3: f64, r_j_per_kg_k: f64) -> f64 {
    state.mass_kg * r_j_per_kg_k * state.temperature_k / volume_m3
}

pub fn initial_closed_system_state(
    config: MotoredCylinderConfig,
    geometry: &GeometryModel,
) -> ClosedSystemState {
    let initial_volume = geometry.volume_m3(config.initial_charge.crank_angle_rad);
    let mass_kg = config.initial_charge.pressure_pa * initial_volume
        / (config.gas.r_j_per_kg_k * config.initial_charge.temperature_k);
    ClosedSystemState {
        mass_kg,
        temperature_k: config.initial_charge.temperature_k,
    }
}

pub fn pressure_pa(
    config: MotoredCylinderConfig,
    geometry: &GeometryModel,
    theta_rad: f64,
    state: ClosedSystemState,
) -> f64 {
    ideal_gas_pressure_pa(
        state,
        geometry.volume_m3(theta_rad),
        config.gas.r_j_per_kg_k,
    )
}

pub fn specific_internal_energy_j_per_kg(temperature_k: f64, cv_j_per_kg_k: f64) -> f64 {
    cv_j_per_kg_k * temperature_k
}

pub fn specific_enthalpy_j_per_kg(temperature_k: f64, gas: GasProperties) -> f64 {
    (gas.cv_j_per_kg_k + gas.r_j_per_kg_k) * temperature_k
}

pub fn manifold_pressure_pa(
    state: ManifoldState,
    config: ManifoldConfig,
    gas: GasProperties,
) -> f64 {
    state.mass_kg * gas.r_j_per_kg_k * state.temperature_k / config.volume_m3
}

pub fn manifold_mass_from_pressure_pa(
    pressure_pa: f64,
    config: ManifoldConfig,
    gas: GasProperties,
) -> f64 {
    pressure_pa * config.volume_m3 / (gas.r_j_per_kg_k * config.temperature_k)
}

pub fn mass_weighted_gas_properties(
    composition: CompositionState,
    fresh: GasProperties,
    burned: GasProperties,
) -> GasProperties {
    let total = composition.total_mass_kg();
    if total <= 0.0 {
        return fresh;
    }
    let fresh_fraction = composition.fresh_mass_kg / total;
    let burned_fraction = composition.burned_mass_kg / total;
    GasProperties {
        r_j_per_kg_k: fresh.r_j_per_kg_k * fresh_fraction + burned.r_j_per_kg_k * burned_fraction,
        cv_j_per_kg_k: fresh.cv_j_per_kg_k * fresh_fraction
            + burned.cv_j_per_kg_k * burned_fraction,
    }
}

pub fn convergence_status(
    iteration_count: usize,
    previous_trapped_mass_kg: f64,
    trapped_mass_kg: f64,
    previous_residual_fraction: f64,
    residual_fraction: f64,
    trapped_mass_tolerance_kg: f64,
    residual_tolerance: f64,
) -> CycleConvergence {
    let trapped_mass_delta_kg = (trapped_mass_kg - previous_trapped_mass_kg).abs();
    let residual_fraction_delta = (residual_fraction - previous_residual_fraction).abs();
    CycleConvergence {
        iteration_count,
        converged: trapped_mass_delta_kg <= trapped_mass_tolerance_kg
            && residual_fraction_delta <= residual_tolerance,
        trapped_mass_delta_kg,
        residual_fraction_delta,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn manifold_pressure_and_mass_are_consistent() {
        let gas = GasProperties::new(287.0, 718.0);
        let config = ManifoldConfig {
            volume_m3: 0.002,
            temperature_k: 300.0,
            ambient_pressure_pa: 101_325.0,
        };
        let state = ManifoldState {
            mass_kg: manifold_mass_from_pressure_pa(120_000.0, config, gas),
            temperature_k: config.temperature_k,
            composition: CompositionState {
                fresh_mass_kg: manifold_mass_from_pressure_pa(120_000.0, config, gas),
                burned_mass_kg: 0.0,
            },
        };

        let pressure_pa = manifold_pressure_pa(state, config, gas);

        assert!((pressure_pa - 120_000.0).abs() < 1.0e-6);
    }

    #[test]
    fn gas_properties_mix_by_mass_fraction() {
        let fresh = GasProperties::new(287.0, 718.0);
        let burned = GasProperties::new(300.0, 800.0);
        let composition = CompositionState {
            fresh_mass_kg: 0.9,
            burned_mass_kg: 0.1,
        };

        let mixed = mass_weighted_gas_properties(composition, fresh, burned);

        assert!((mixed.r_j_per_kg_k - 288.3).abs() < 1.0e-9);
        assert!((mixed.cv_j_per_kg_k - 726.2).abs() < 1.0e-9);
    }

    #[test]
    fn convergence_status_requires_both_mass_and_residual_to_settle() {
        let converged = convergence_status(3, 0.0010, 0.0010005, 0.08, 0.0804, 1.0e-5, 5.0e-4);
        let not_converged = convergence_status(4, 0.0010, 0.00102, 0.08, 0.0804, 1.0e-5, 5.0e-4);

        assert!(converged.converged);
        assert!(!not_converged.converged);
    }
}
