use crate::{
    combustion::CombustionFrame,
    config::PlantConfig,
    exhaust::{update_exhaust_thermal, ExhaustThermalInput},
    lambda::{update_lambda_transport, LambdaTransportConfig, LambdaTransportState},
    state::PlantState,
    types::*,
};

pub(super) fn update_exhaust_state<const CYL: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    dt_us: Micros,
) {
    let mut fuel_mass_ug = 0u64;
    let mut air_mass_ug = 0u64;
    let mut spark_retard = 0i16;
    let mut counted = 0u16;
    let mut cyl = 0;
    while cyl < config.cylinder_count as usize {
        fuel_mass_ug = fuel_mass_ug.saturating_add(state.cylinders[cyl].fuel_mass_ug.0 as u64);
        air_mass_ug = air_mass_ug.saturating_add(state.cylinders[cyl].air_mass_ug.0 as u64);
        spark_retard = spark_retard.saturating_add(
            config
                .spark
                .mbt_deg10
                .0
                .saturating_sub(state.cylinders[cyl].last_spark_advance_deg10.0)
                .max(0),
        );
        counted = counted.saturating_add(1);
        cyl += 1;
    }
    let spark_retard = if counted == 0 {
        0
    } else {
        spark_retard / counted as i16
    };
    let fuel_energy_j_per_cycle =
        (fuel_mass_ug.saturating_mul(config.fuel.fuel_lhv_j_per_kg as u64) / 1_000_000_000)
            .min(u32::MAX as u64) as u32;
    let dt_ms = dt_ms_u16(dt_us);
    let mass_flow_mg_per_s = if dt_ms == 0 {
        0
    } else {
        (air_mass_ug / 1000)
            .saturating_mul(1000)
            .saturating_div(dt_ms as u64)
            .min(u32::MAX as u64) as u32
    };

    update_exhaust_thermal(
        &mut state.exhaust,
        ExhaustThermalInput {
            fuel_energy_j_per_cycle,
            lambda_x1000: state.last_lambda_x1000,
            spark_retard_deg_x10: spark_retard,
            mass_flow_mg_per_s,
            dt_ms,
        },
    );
}

fn average_lambda<const CYL: usize>(combustion: &CombustionFrame<CYL>) -> u16 {
    let mut sum = 0u32;
    let mut count = 0u32;
    for event in &combustion.cylinders {
        if event.afr_x100 != 0 {
            sum += event.lambda_x1000 as u32;
            count += 1;
        }
    }
    if count == 0 {
        1000
    } else {
        (sum / count) as u16
    }
}

pub(super) fn observed_lambda<const CYL: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    combustion: &CombustionFrame<CYL>,
    dt_us: Micros,
) -> u16 {
    let raw_lambda = average_lambda(combustion);
    if !config.sensors.lambda_transport_enabled {
        return raw_lambda;
    }

    let mut transport = LambdaTransportState::<MAX_CYLINDERS> {
        ring: state.lambda_transport_ring,
        idx: state.lambda_transport_idx,
        filled: state.lambda_transport_filled,
        sensor_lambda_x1000: state.last_lambda_x1000,
    };
    let observed = update_lambda_transport(
        &mut transport,
        LambdaTransportConfig {
            delay_crank_deg: config.sensors.lambda_delay_crank_deg,
            sensor_tau_ms: config.sensors.lambda_sensor_tau_ms,
            exhaust_mixing_x1000: config.sensors.lambda_exhaust_mixing_x1000,
        },
        raw_lambda,
        dt_ms_u16(dt_us),
    );
    state.lambda_transport_ring = transport.ring;
    state.lambda_transport_idx = transport.idx;
    state.lambda_transport_filled = transport.filled;
    observed
}

pub(super) fn dt_ms_u16(dt_us: Micros) -> u16 {
    if dt_us.0 == 0 {
        0
    } else {
        dt_us
            .0
            .saturating_add(999)
            .saturating_div(1000)
            .min(u16::MAX as u32) as u16
    }
}
