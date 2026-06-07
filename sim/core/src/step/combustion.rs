use crate::{
    burn::{
        burn_fraction_at_elapsed_deg10, crank_angle_after_tdc_for_burn_fraction,
        elapsed_burn_angle_deg10,
    },
    combustion::{evaluate_combustion_with_residual, CombustionFrame},
    config::{PlantConfig, PlantPhysicsMode},
    fuel::{injection_window_deg10, AcceptedInjection},
    geometry::{cylinder_volume_mm3, swept_volume_mm3},
    io::{DiagnosticKind, PlantStepInput, PlantStepOutput},
    pressure::{kinematic_pressure_delta_pa, kinematic_pressure_torque_nm_x100},
    residual::{apply_residual_to_fresh_air, estimate_residual_gas, ResidualGasInput},
    spark::AcceptedSpark,
    state::PlantState,
    thermo::{
        fuel_heat_release_micro_j, ideal_gas_pressure_pa, polytropic_pressure_pa,
        temperature_after_heat_release_k10,
    },
    types::*,
};

#[allow(clippy::too_many_arguments)]
pub(super) fn evaluate_all_combustion<
    const CYL: usize,
    const MAX_EDGES: usize,
    const MAX_EVENTS: usize,
>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    input: &PlantStepInput<CYL, MAX_EVENTS>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    timestamp: Micros,
    angle: CrankDeg10,
    air_mass: MassUg,
    accepted_fuel: [Option<AcceptedInjection>; CYL],
    accepted_spark: [Option<AcceptedSpark>; CYL],
    record_diagnostics: bool,
) -> CombustionFrame<CYL> {
    let mut frame = CombustionFrame::empty();
    for cyl in 0..config.cylinder_count as usize {
        let residual_fraction_x1000 = residual_fraction_for_cylinder(config, state, cyl);
        let effective_air_mass = apply_residual_to_fresh_air(air_mass, residual_fraction_x1000);
        let mut event = evaluate_combustion_with_residual(
            config,
            timestamp,
            angle,
            cyl,
            effective_air_mass,
            accepted_fuel[cyl],
            accepted_spark[cyl],
            input.ecu_outputs.fuel_cut,
            input.ecu_outputs.spark_cut,
            residual_fraction_x1000,
        );
        if event.misfire.is_none()
            && matches!(
                config.physics_mode,
                PlantPhysicsMode::KinematicPressurePulse
                    | PlantPhysicsMode::PolytropicWiebe
                    | PlantPhysicsMode::SingleZoneIdealGas
            )
        {
            let local_angle =
                normalize_deg10_i32(angle.0 as i32 - config.cylinder_phase_deg10[cyl].0 as i32);
            let mut pressure = kinematic_pressure_delta_pa(
                effective_air_mass,
                event.quality_x1000,
                config.combustion.torque_scale_x100,
            );
            let mut saturated = pressure.0 == i32::MAX;
            if matches!(
                config.physics_mode,
                PlantPhysicsMode::PolytropicWiebe | PlantPhysicsMode::SingleZoneIdealGas
            ) {
                let volume = cylinder_volume_mm3(config.engine, local_angle);
                let reference_volume = VolumeMm3(
                    crate::geometry::clearance_volume_mm3(config.engine)
                        .0
                        .saturating_add(swept_volume_mm3(config.engine).0),
                );
                let compression_pressure = polytropic_pressure_pa(
                    config.thermo.p_ref_pa,
                    reference_volume,
                    volume,
                    config.thermo.gamma_x1000,
                );
                pressure.0 = pressure.0.saturating_add(
                    compression_pressure
                        .0
                        .saturating_sub(config.thermo.p_ref_pa.0),
                );
                let start_of_combustion = accepted_spark[cyl]
                    .map(|spark| {
                        normalize_deg10(
                            spark.command.spark_angle_deg10.0 as u32
                                + config.spark.ignition_delay_deg10 as u32,
                        )
                    })
                    .unwrap_or(CrankDeg10(0));
                let elapsed = elapsed_burn_angle_deg10(local_angle, start_of_combustion);
                let burn = burn_fraction_at_elapsed_deg10(
                    &config.combustion.burn_curve,
                    elapsed,
                    config.combustion.burn_duration_deg10,
                );
                pressure.0 = (pressure.0 as i64 * burn as i64 / 10000) as i32;
                state.physics[cyl].last_burn_fraction_x10000 = burn;
                state.physics[cyl].ca10_deg10 = crank_angle_after_tdc_for_burn_fraction(
                    &config.combustion.burn_curve,
                    1000,
                    config.combustion.burn_duration_deg10,
                )
                .map(|ca| normalize_deg10_i32(start_of_combustion.0 as i32 + ca.0 as i32));
                state.physics[cyl].ca50_deg10 = crank_angle_after_tdc_for_burn_fraction(
                    &config.combustion.burn_curve,
                    5000,
                    config.combustion.burn_duration_deg10,
                )
                .map(|ca| normalize_deg10_i32(start_of_combustion.0 as i32 + ca.0 as i32));
                state.physics[cyl].ca90_deg10 = crank_angle_after_tdc_for_burn_fraction(
                    &config.combustion.burn_curve,
                    9000,
                    config.combustion.burn_duration_deg10,
                )
                .map(|ca| normalize_deg10_i32(start_of_combustion.0 as i32 + ca.0 as i32));
            }
            if config.physics_mode == PlantPhysicsMode::SingleZoneIdealGas {
                let burned_heat = accepted_fuel[cyl]
                    .map(|fuel| {
                        let heat = fuel_heat_release_micro_j(
                            fuel.fuel_mass_ug,
                            config.fuel.fuel_lhv_j_per_kg,
                            event.quality_x1000,
                        );
                        EnergyMicroJ(
                            heat.0.saturating_mul(
                                state.physics[cyl].last_burn_fraction_x10000 as i64,
                            ) / 10000,
                        )
                    })
                    .unwrap_or(EnergyMicroJ(0));
                let temperature = temperature_after_heat_release_k10(
                    config.thermo.initial_cylinder_temp_k10,
                    burned_heat,
                    effective_air_mass,
                    config.thermo.cv_air_j_per_kg_k,
                );
                let volume = cylinder_volume_mm3(config.engine, local_angle);
                let absolute = ideal_gas_pressure_pa(
                    effective_air_mass,
                    temperature,
                    volume,
                    config.thermo.r_air_j_per_kg_k,
                );
                saturated = saturated || absolute.0 == i32::MAX;
                state.physics[cyl].temperature_k10 = temperature;
                pressure.0 = absolute.0.saturating_sub(config.thermo.p_ref_pa.0);
            }
            let absolute_pressure = PressurePa(
                config
                    .thermo
                    .initial_cylinder_pressure_pa
                    .0
                    .saturating_add(pressure.0),
            );
            state.physics[cyl].pressure_pa = absolute_pressure;
            if absolute_pressure.0 > state.physics[cyl].pmax_pa.0 {
                state.physics[cyl].pmax_pa = absolute_pressure;
                state.physics[cyl].pmax_angle_deg10 = local_angle;
            }
            if record_diagnostics && saturated {
                output.diagnostics.push(
                    timestamp,
                    DiagnosticKind::PhysicalSaturation,
                    Some(CylinderIndex(cyl as u8)),
                );
            }
            event.torque_nm_x100 =
                kinematic_pressure_torque_nm_x100(config.engine, local_angle, pressure);
        }
        if record_diagnostics && event.misfire.is_some() {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::CombustionMisfire,
                Some(CylinderIndex(cyl as u8)),
            );
        }
        frame.total_torque_nm_x100.0 = frame
            .total_torque_nm_x100
            .0
            .saturating_add(event.torque_nm_x100.0);
        frame.cylinders[cyl] = event;
        state.physics[cyl].residual_fraction_x1000 = residual_fraction_x1000;
        state.cylinders[cyl].air_mass_ug = effective_air_mass;
        state.cylinders[cyl].fuel_mass_ug = accepted_fuel[cyl]
            .map(|fuel| fuel.fuel_mass_ug)
            .unwrap_or(MassUg(0));
        state.cylinders[cyl].last_injection_pw_us = accepted_fuel[cyl]
            .map(|fuel| fuel.command.pulse_width_us)
            .unwrap_or(Micros(0));
        let injection_window = accepted_fuel[cyl].and_then(|fuel| {
            injection_window_deg10(
                fuel.command.mode,
                fuel.command.angle_deg10,
                fuel.command.pulse_width_us,
                state.rpm,
            )
        });
        state.cylinders[cyl].last_soi_deg10 = injection_window.map(|window| window.0);
        state.cylinders[cyl].last_eoi_deg10 = injection_window.map(|window| window.1);
        state.cylinders[cyl].last_dwell_us = accepted_spark[cyl]
            .map(|spark| spark.command.dwell_us)
            .unwrap_or(Micros(0));
        state.cylinders[cyl].last_spark_advance_deg10 = accepted_spark[cyl]
            .map(|spark| Degrees10(spark.command.spark_angle_deg10.0 as i16))
            .unwrap_or(Degrees10(0));
        state.cylinders[cyl].combustion_quality_x1000 = event.quality_x1000;
        state.cylinders[cyl].misfire = event.misfire;
    }
    frame
}

fn residual_fraction_for_cylinder<const CYL: usize>(
    config: &PlantConfig<CYL>,
    state: &PlantState<CYL>,
    cyl: usize,
) -> u16 {
    if !config.residual.enabled {
        return 0;
    }
    let previous = state.physics[cyl];
    estimate_residual_gas(ResidualGasInput {
        base_fraction_x1000: config.residual.base_fraction_x1000,
        overlap_gain_x1000: config.residual.overlap_gain_x1000,
        low_map_gain_x1000: config.residual.low_map_gain_x1000,
        exhaust_backpressure_gain_x1000: config.residual.exhaust_backpressure_gain_x1000,
        scavenging_gain_x1000: config.residual.scavenging_gain_x1000,
        valve_events: config.valve_events,
        map_kpa10: state.map_kpa10,
        exhaust_pressure_kpa10: config.air.reference_pressure_kpa10,
        rpm: state.rpm,
        previous_lambda_x1000: state.last_lambda_x1000,
        previous_temp_k_x10: previous.temperature_k10.0,
    })
    .residual_fraction_x1000
}
