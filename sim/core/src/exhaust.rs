#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExhaustThermalState {
    pub egt_k_x10: u16,
    pub exhaust_manifold_temp_k_x10: u16,
    pub catalyst_temp_k_x10: u16,
}

impl ExhaustThermalState {
    pub const fn ambient() -> Self {
        Self {
            egt_k_x10: 2930,
            exhaust_manifold_temp_k_x10: 2930,
            catalyst_temp_k_x10: 2930,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ExhaustThermalInput {
    pub fuel_energy_j_per_cycle: u32,
    pub lambda_x1000: u16,
    pub spark_retard_deg_x10: i16,
    pub mass_flow_mg_per_s: u32,
    pub dt_ms: u16,
}

pub fn update_exhaust_thermal(
    state: &mut ExhaustThermalState,
    input: ExhaustThermalInput,
) -> ExhaustThermalState {
    let target = target_egt_k_x10(input);
    state.egt_k_x10 = first_order_u16(state.egt_k_x10, target, 180, input.dt_ms);
    state.exhaust_manifold_temp_k_x10 = first_order_u16(
        state.exhaust_manifold_temp_k_x10,
        state.egt_k_x10,
        900,
        input.dt_ms,
    );
    state.catalyst_temp_k_x10 = first_order_u16(
        state.catalyst_temp_k_x10,
        state.exhaust_manifold_temp_k_x10,
        2500,
        input.dt_ms,
    );
    *state
}

fn target_egt_k_x10(input: ExhaustThermalInput) -> u16 {
    let energy_term = (input.fuel_energy_j_per_cycle / 8).min(4500);
    let retard_term = input.spark_retard_deg_x10.max(0) as u32 * 2;
    let rich_term = 1000u16.saturating_sub(input.lambda_x1000) as u32 / 2;
    let lean_term = input.lambda_x1000.saturating_sub(1000) as u32 / 4;
    let flow_cooling = (input.mass_flow_mg_per_s / 25_000).min(1000);
    let target = 5500u32
        .saturating_add(energy_term)
        .saturating_add(retard_term)
        .saturating_add(rich_term)
        .saturating_add(lean_term)
        .saturating_sub(flow_cooling);
    target.clamp(2930, u16::MAX as u32) as u16
}

fn first_order_u16(current: u16, target: u16, tau_ms: u16, dt_ms: u16) -> u16 {
    if tau_ms == 0 {
        return target;
    }
    let gain_x1000 = (dt_ms as u32 * 1000 / tau_ms as u32).clamp(1, 1000) as i32;
    let delta = target as i32 - current as i32;
    (current as i32 + delta * gain_x1000 / 1000).clamp(0, u16::MAX as i32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn exhaust_temperature_rises_with_retarded_spark() {
        let mut normal = ExhaustThermalState::ambient();
        let mut retarded = ExhaustThermalState::ambient();
        let common = ExhaustThermalInput {
            fuel_energy_j_per_cycle: 2500,
            lambda_x1000: 950,
            spark_retard_deg_x10: 0,
            mass_flow_mg_per_s: 80_000,
            dt_ms: 100,
        };

        update_exhaust_thermal(&mut normal, common);
        update_exhaust_thermal(
            &mut retarded,
            ExhaustThermalInput {
                spark_retard_deg_x10: 120,
                ..common
            },
        );

        assert!(retarded.egt_k_x10 > normal.egt_k_x10);
        assert!(retarded.exhaust_manifold_temp_k_x10 >= normal.exhaust_manifold_temp_k_x10);
    }
}
