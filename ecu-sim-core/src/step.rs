use crate::{
    air::{
        estimate_air_mass_ug, estimate_map_kpa10, estimate_speed_density_air_mass_ug,
        lookup_ve_x1000, update_intake_manifold, update_manifold_map_kpa10,
        valve_event_airflow_modifier_x1000, IntakeManifoldConfig, IntakeManifoldState,
    },
    burn::{
        burn_fraction_at_elapsed_deg10, crank_angle_after_tdc_for_burn_fraction,
        elapsed_burn_angle_deg10,
    },
    combustion::{evaluate_combustion_with_residual, CombustionFrame},
    config::{PlantConfig, PlantPhysicsMode},
    crank::{choose_substep_us, update_crank_with_remainder},
    cycle::accumulate_cycle_work,
    dyno::{dyno_load_torque, dyno_pid_load_torque, horsepower_x100, DynoFrame},
    exhaust::{update_exhaust_thermal, ExhaustThermalInput},
    fuel::{
        fuel_mass, injection_phasing, injection_window_deg10, update_wall_film, AcceptedInjection,
        WallFilmParams, WallFilmState,
    },
    geometry::{cylinder_volume_mm3, swept_volume_mm3},
    io::{DiagnosticKind, PlantStepError, PlantStepInput, PlantStepOutput},
    knock::estimate_knock_risk_with_physics,
    lambda::{update_lambda_transport, LambdaTransportConfig, LambdaTransportState},
    losses::{
        brake_torque_from_indicated, fmep_bar_x100, fmep_pa, pmep_bar_x100, pmep_pa,
        torque_from_mep_nm_x100,
    },
    pressure::{bmep_bar_x100, kinematic_pressure_delta_pa, kinematic_pressure_torque_nm_x100},
    residual::{apply_residual_to_fresh_air, estimate_residual_gas, ResidualGasInput},
    sensors::SensorSnapshot,
    spark::{dwell_quality, spark_phase_quality, AcceptedSpark},
    state::PlantState,
    telemetry::is_misfire,
    thermo::{
        fuel_heat_release_micro_j, ideal_gas_pressure_pa, polytropic_pressure_pa,
        temperature_after_heat_release_k10,
    },
    trigger::generate_edges,
    types::*,
};

pub fn step_plant<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    input: &PlantStepInput<CYL, MAX_EVENTS>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
) -> Result<(), PlantStepError> {
    input.validate(config.cylinder_count)?;
    output.clear_event_buffers();
    output.diagnostics.last_step_us = input.dt_us;
    output.diagnostics.validated_config = true;

    let timestamp = state.timestamp_us;
    let target_map = estimate_map_kpa10(
        config,
        input.driver.throttle_x1000,
        input.ecu_outputs.idle_command_x1000,
    );
    let map = estimate_step_map(config, state, input, target_map);
    if input.dt_us.0 == 0 {
        output.sensors = SensorSnapshot {
            timestamp_us: state.timestamp_us,
            rpm: state.rpm,
            crank_angle_deg10: state.crank_angle_deg10,
            map_kpa10: state.map_kpa10,
            tps_x1000: state.tps_x1000,
            clt_c10: state.clt_c10,
            iat_c10: state.iat_c10,
            lambda_x1000: state.last_lambda_x1000,
            battery_mv: state.battery_mv,
            knock_intensity_x100: 0,
        };
        output.telemetry = crate::telemetry::TelemetryFrame::from_state(
            state.timestamp_us,
            state.rpm,
            output.dyno,
            output.sensors,
            &state.cylinders,
            &state.physics,
            lookup_ve_x1000(&config.air.ve_table, state.rpm, state.map_kpa10),
        );
        return Ok(());
    }
    let air_mass = estimate_step_air_mass(config, state.rpm, map, input.environment.ambient_c10);
    let mut accepted_fuel: [Option<AcceptedInjection>; CYL] = [None; CYL];
    let mut accepted_spark: [Option<AcceptedSpark>; CYL] = [None; CYL];

    ingest_fuel(config, state, input, output, timestamp, &mut accepted_fuel);
    ingest_spark(config, input, output, timestamp, &mut accepted_spark);

    let mut dyno_config = config.dyno;
    if config.dyno.mode == crate::config::DynoMode::TargetRpmSweep {
        if state.dyno_current_target_rpm.0 == 0 {
            state.dyno_current_target_rpm = config.dyno.sweep_start_rpm;
        }
        dyno_config.target_rpm = state.dyno_current_target_rpm;
    }
    let dyno_target = dyno_config.target_rpm.0;
    let dyno_load = if matches!(
        dyno_config.mode,
        crate::config::DynoMode::TargetRpmHold | crate::config::DynoMode::TargetRpmSweep
    ) {
        dyno_pid_load_torque(
            dyno_config,
            state.rpm,
            input.dt_us,
            &mut state.dyno_pid_integral_x100,
            &mut state.dyno_previous_error_rpm,
        )
    } else {
        dyno_load_torque(
            dyno_config.mode,
            dyno_config.fixed_load_torque_nm_x100,
            state.rpm,
            Rpm(dyno_target),
        )
    };
    state.last_dyno_load_torque_nm_x100 = dyno_load;
    let total_load = TorqueNmX100(
        input
            .driver
            .load_torque_nm_x100
            .0
            .saturating_add(dyno_load.0),
    );
    let mut remaining_us = input.dt_us.0;
    let mut substep_timestamp = timestamp;
    let mut rpm = state.rpm;
    let mut angle = state.crank_angle_deg10;
    let mut final_combustion = CombustionFrame::empty();
    while remaining_us > 0 {
        let substep = choose_substep_us(config.crank, rpm, Micros(remaining_us));
        let substep_combustion = evaluate_all_combustion(
            config,
            state,
            input,
            output,
            substep_timestamp,
            angle,
            air_mass,
            accepted_fuel,
            accepted_spark,
            false,
        );
        let crank = update_crank_with_remainder(
            config,
            substep,
            rpm,
            angle,
            substep_combustion.total_torque_nm_x100,
            total_load,
            input.driver.starter_enabled,
            &mut state.rpm_delta_remainder,
        );

        generate_edges(
            config.trigger,
            crank.old_angle_deg10,
            crank.new_angle_deg10,
            substep_timestamp,
            substep,
            input.faults,
            &mut state.trigger_sequence,
            &mut output.trigger_edges,
        )
        .map_err(|_| PlantStepError::OutputCapacityExceeded)?;
        accumulate_cycle_work(
            &mut state.cycle,
            crank.old_angle_deg10,
            crank.new_angle_deg10,
            substep_combustion.total_torque_nm_x100,
            substep_combustion.total_torque_nm_x100,
        );

        final_combustion = substep_combustion;
        remaining_us = remaining_us.saturating_sub(substep.0);
        substep_timestamp.0 = substep_timestamp.0.saturating_add(substep.0);
        rpm = crank.rpm;
        angle = crank.new_angle_deg10;
    }

    state.timestamp_us.0 = state.timestamp_us.0.saturating_add(input.dt_us.0);
    state.rpm = rpm;
    state.crank_angle_deg10 = angle;
    state.map_kpa10 = map;
    state.tps_x1000 = input.driver.throttle_x1000;
    state.iat_c10 = input.environment.ambient_c10;
    state.clt_c10 = input.environment.coolant_c10;
    state.battery_mv = input.environment.battery_mv;

    if input.dt_us.0 != 0 {
        final_combustion = evaluate_all_combustion(
            config,
            state,
            input,
            output,
            state.timestamp_us,
            state.crank_angle_deg10,
            air_mass,
            accepted_fuel,
            accepted_spark,
            true,
        );
    }

    let lambda = observed_lambda(config, state, &final_combustion, input.dt_us);
    state.last_lambda_x1000 = lambda;
    output.combustion = final_combustion;
    update_exhaust_state(config, state, input.dt_us);
    fill_knock(config, state, output, map);
    fill_dyno(config, state, output);
    fill_sensors(state, output, input, map);
    fill_telemetry(config, state, output, map);
    Ok(())
}

fn update_exhaust_state<const CYL: usize>(
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

fn estimate_step_map<const CYL: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    state: &PlantState<CYL>,
    input: &PlantStepInput<CYL, MAX_EVENTS>,
    target_map: Kpa10,
) -> Kpa10 {
    if config.physics_mode == PlantPhysicsMode::SyntheticTorque || state.timestamp_us.0 == 0 {
        return target_map;
    }
    if !config.air.manifold_filling_enabled {
        return update_manifold_map_kpa10(
            config,
            state.map_kpa10,
            input.driver.throttle_x1000,
            input.ecu_outputs.idle_command_x1000,
            state.rpm,
            input.dt_us,
        );
    }

    let temperature_k_x100 =
        ((input.environment.ambient_c10.0 as i32 + 2731).clamp(1, u16::MAX as i32) as u32) * 10;
    let manifold_cfg = IntakeManifoldConfig {
        volume_cc: config.air.manifold_volume_cc,
        throttle_area_mm2: config.air.throttle_area_mm2,
        discharge_coeff_x1000: config.air.throttle_discharge_coeff_x1000,
    };
    let mut manifold = IntakeManifoldState::from_pressure(
        state.map_kpa10.0 as i32 * 100,
        temperature_k_x100,
        manifold_cfg,
    );
    let dt_ms = Millis(input.dt_us.0.saturating_add(999) / 1000);
    let ve_x1000 = lookup_ve_x1000(&config.air.ve_table, state.rpm, state.map_kpa10);
    update_intake_manifold(
        &mut manifold,
        manifold_cfg,
        input.driver.throttle_x1000,
        state.rpm,
        config.engine.displacement_cc,
        ve_x1000,
        config.air.reference_pressure_kpa10.0 as i32 * 100,
        dt_ms,
    );
    Kpa10((manifold.pressure_pa / 100).clamp(0, u16::MAX as i32) as u16)
}

fn estimate_step_air_mass<const CYL: usize>(
    config: &PlantConfig<CYL>,
    rpm: Rpm,
    map: Kpa10,
    iat_c10: Celsius10,
) -> MassUg {
    if config.physics_mode == PlantPhysicsMode::SyntheticTorque {
        return estimate_air_mass_ug(config, map);
    }

    let kelvin10 = Kelvin10((iat_c10.0 as i32 + 2731).clamp(1, u16::MAX as i32) as u16);
    let base = estimate_speed_density_air_mass_ug(config, rpm, map, kelvin10);
    let valve_modifier_x1000 = valve_event_airflow_modifier_x1000(config.valve_events, rpm);
    MassUg((base.0 as u64 * valve_modifier_x1000 as u64 / 1000).min(u32::MAX as u64) as u32)
}

fn ingest_fuel<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    input: &PlantStepInput<CYL, MAX_EVENTS>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    timestamp: Micros,
    accepted: &mut [Option<AcceptedInjection>; CYL],
) {
    for (idx, command) in input
        .ecu_outputs
        .injection_events
        .as_slice()
        .iter()
        .enumerate()
    {
        let cyl = command.cylinder.0 as usize;
        output.consumed_events.injection_count += 1;
        if cyl >= config.cylinder_count as usize {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InvalidCylinderIndex,
                Some(command.cylinder),
            );
            output.consumed_events.ignored_injection_count += 1;
            continue;
        }
        if input.ecu_outputs.fuel_cut || !config.fuel.enabled {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InputEventIgnoredDueToCut,
                Some(command.cylinder),
            );
            output.consumed_events.ignored_injection_count += 1;
            continue;
        }
        let fuel_mass_ug =
            delivered_fuel_mass_with_wall_film(config, state, cyl, *command, input.dt_us);
        let accepted_event = AcceptedInjection {
            command: *command,
            fuel_mass_ug,
            phasing_x1000: injection_phasing(command.mode, command.angle_deg10),
        };
        accepted[cyl] = Some(accepted_event);
        if idx < MAX_EVENTS {
            output.consumed_events.injection_fuel_mass_ug[idx] = accepted_event.fuel_mass_ug;
        }
    }
}

fn delivered_fuel_mass_with_wall_film<const CYL: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    cyl: usize,
    command: crate::fuel::InjectionCommand,
    dt_us: Micros,
) -> MassUg {
    let raw = fuel_mass(command);
    if !config.fuel.wall_film_enabled {
        return raw;
    }

    let mut wall_film = WallFilmState {
        film_fuel_ug: state.physics[cyl].wall_film_ug.0.min(i32::MAX as u32) as i32,
    };
    let delivered = update_wall_film(
        &mut wall_film,
        raw.0.min(i32::MAX as u32) as i32,
        WallFilmParams {
            x_deposit_x1000: config.fuel.wall_film_deposit_x1000,
            tau_ms: config.fuel.wall_film_tau_ms,
        },
        dt_ms_u16(dt_us),
    );
    state.physics[cyl].wall_film_ug = MassUg(wall_film.film_fuel_ug.max(0) as u32);
    MassUg(delivered.max(0) as u32)
}

fn ingest_spark<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    input: &PlantStepInput<CYL, MAX_EVENTS>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    timestamp: Micros,
    accepted: &mut [Option<AcceptedSpark>; CYL],
) {
    for command in input.ecu_outputs.spark_events.as_slice() {
        let cyl = command.cylinder.0 as usize;
        output.consumed_events.spark_count += 1;
        if cyl >= config.cylinder_count as usize {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InvalidCylinderIndex,
                Some(command.cylinder),
            );
            output.consumed_events.ignored_spark_count += 1;
            continue;
        }
        if input.ecu_outputs.spark_cut {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InputEventIgnoredDueToCut,
                Some(command.cylinder),
            );
            output.consumed_events.ignored_spark_count += 1;
            continue;
        }
        accepted[cyl] = Some(AcceptedSpark {
            command: *command,
            dwell_quality_x1000: dwell_quality(command.dwell_us, config.spark.min_dwell_us),
            phase_quality_x1000: spark_phase_quality(
                command.spark_angle_deg10,
                config.spark.mbt_deg10,
                config.spark.max_advance_deg10,
            ),
        });
    }
}

#[allow(clippy::too_many_arguments)]
fn evaluate_all_combustion<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
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

fn observed_lambda<const CYL: usize>(
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

fn dt_ms_u16(dt_us: Micros) -> u16 {
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

fn fill_knock<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    map: Kpa10,
) {
    let mut max_risk = 0u16;
    for cyl in 0..config.cylinder_count as usize {
        let advance = state.cylinders[cyl].last_spark_advance_deg10;
        let risk = estimate_knock_risk_with_physics(
            config,
            map,
            state.rpm,
            advance,
            state.last_lambda_x1000,
            state.iat_c10,
            state.physics[cyl].pmax_pa,
            state.physics[cyl].temperature_k10,
        );
        state.cylinders[cyl].knock_risk_x1000 = risk;
        output.knock.knock_risk_x1000[cyl] = risk;
        output.knock.knock_event[cyl] = risk >= config.knock.risk_threshold_x1000;
        max_risk = max_risk.max(risk);
    }
    output.knock.knock_intensity_x100 = (max_risk / 10).min(100);
}

fn fill_dyno<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
) {
    let indicated = output.combustion.total_torque_nm_x100;
    let (friction, pumping, accessories) =
        if config.physics_mode == PlantPhysicsMode::SyntheticTorque {
            (TorqueNmX100(0), TorqueNmX100(0), TorqueNmX100(0))
        } else {
            (
                torque_from_mep_nm_x100(
                    fmep_pa(config.losses, state.rpm, state.map_kpa10),
                    config.engine.displacement_cc,
                ),
                torque_from_mep_nm_x100(
                    pmep_pa(config.losses, state.tps_x1000),
                    config.engine.displacement_cc,
                ),
                config.losses.accessory_torque_nm_x100,
            )
        };
    let torque = brake_torque_from_indicated(indicated, friction, pumping, accessories);
    let bmep = bmep_bar_x100(torque, config.engine.displacement_cc);
    let mut sweep_point = crate::dyno::DynoSweepPoint::empty();
    let mut sweep_complete = false;
    if config.dyno.mode == crate::config::DynoMode::TargetRpmSweep {
        if state.dyno_current_target_rpm.0 == 0 {
            state.dyno_current_target_rpm = config.dyno.sweep_start_rpm;
        }
        if state.cycle.completed_cycle_count != state.dyno_last_completed_cycle_count {
            state.dyno_last_completed_cycle_count = state.cycle.completed_cycle_count;
            let error = state.rpm.0.abs_diff(state.dyno_current_target_rpm.0);
            if error <= config.dyno.rpm_error_limit.0 {
                if state.dyno_hold_cycle_count < config.dyno.hold_cycles_before_sample {
                    state.dyno_hold_cycle_count = state.dyno_hold_cycle_count.saturating_add(1);
                } else {
                    state.dyno_sample_cycle_count = state.dyno_sample_cycle_count.saturating_add(1);
                    state.dyno_sample_torque_sum =
                        state.dyno_sample_torque_sum.saturating_add(torque.0 as i64);
                    if state.dyno_sample_cycle_count >= config.dyno.sample_cycles.max(1) {
                        let avg = (state.dyno_sample_torque_sum
                            / state.dyno_sample_cycle_count.max(1) as i64)
                            as i32;
                        let avg_torque = TorqueNmX100(avg);
                        sweep_point = crate::dyno::DynoSweepPoint {
                            valid: true,
                            target_rpm: state.dyno_current_target_rpm,
                            measured_rpm: state.rpm,
                            torque_nm_x100: avg_torque,
                            horsepower_x100: horsepower_x100(avg_torque, state.rpm),
                            bmep_bar_x100: bmep_bar_x100(avg_torque, config.engine.displacement_cc),
                            ve_x1000: lookup_ve_x1000(
                                &config.air.ve_table,
                                state.rpm,
                                state.map_kpa10,
                            ),
                            map_kpa10: state.map_kpa10,
                            lambda_x1000: state.last_lambda_x1000,
                            spark_advance_deg10: state.cylinders[0].last_spark_advance_deg10,
                            dyno_load_torque_nm_x100: state.last_dyno_load_torque_nm_x100,
                        };
                        let next = state
                            .dyno_current_target_rpm
                            .0
                            .saturating_add(config.dyno.sweep_step_rpm.0);
                        if config.dyno.sweep_step_rpm.0 == 0 || next > config.dyno.sweep_end_rpm.0 {
                            sweep_complete = true;
                        } else {
                            state.dyno_current_target_rpm = Rpm(next);
                        }
                        state.dyno_hold_cycle_count = 0;
                        state.dyno_sample_cycle_count = 0;
                        state.dyno_sample_torque_sum = 0;
                    }
                }
            } else {
                state.dyno_hold_cycle_count = 0;
                state.dyno_sample_cycle_count = 0;
                state.dyno_sample_torque_sum = 0;
            }
        }
    }

    output.dyno = DynoFrame {
        torque_nm_x100: torque,
        filtered_torque_nm_x100: torque,
        indicated_torque_nm_x100: indicated,
        brake_torque_nm_x100: torque,
        bmep_bar_x100: bmep,
        imep_bar_x100: ImepBarX100(bmep_bar_x100(indicated, config.engine.displacement_cc).0),
        pmep_bar_x100: pmep_bar_x100(pumping, config.engine.displacement_cc),
        fmep_bar_x100: fmep_bar_x100(friction, config.engine.displacement_cc),
        horsepower_x100: horsepower_x100(torque, state.rpm),
        load_mode: config.dyno.mode,
        sweep_target_rpm: state.dyno_current_target_rpm,
        sweep_point,
        sweep_complete,
    };
}

fn fill_sensors<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    state: &PlantState<CYL>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    input: &PlantStepInput<CYL, MAX_EVENTS>,
    map: Kpa10,
) {
    output.sensors = SensorSnapshot {
        timestamp_us: state.timestamp_us,
        rpm: state.rpm,
        crank_angle_deg10: state.crank_angle_deg10,
        map_kpa10: map,
        tps_x1000: input.driver.throttle_x1000,
        clt_c10: input.environment.coolant_c10,
        iat_c10: input.environment.ambient_c10,
        lambda_x1000: state.last_lambda_x1000,
        battery_mv: input.environment.battery_mv,
        knock_intensity_x100: output.knock.knock_intensity_x100,
    };
}

fn fill_telemetry<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
    config: &PlantConfig<CYL>,
    state: &PlantState<CYL>,
    output: &mut PlantStepOutput<CYL, MAX_EDGES, MAX_EVENTS>,
    map: Kpa10,
) {
    output.telemetry.timestamp_us = state.timestamp_us;
    output.telemetry.rpm = state.rpm;
    output.telemetry.torque_nm_x100 = output.dyno.torque_nm_x100.0;
    output.telemetry.horsepower_x100 = output.dyno.horsepower_x100;
    output.telemetry.bmep_bar_x100 = output.dyno.bmep_bar_x100;
    output.telemetry.imep_bar_x100 = output.dyno.imep_bar_x100;
    output.telemetry.pmep_bar_x100 = output.dyno.pmep_bar_x100;
    output.telemetry.fmep_bar_x100 = output.dyno.fmep_bar_x100;
    output.telemetry.indicated_torque_nm_x100 = output.dyno.indicated_torque_nm_x100;
    output.telemetry.brake_torque_nm_x100 = output.dyno.brake_torque_nm_x100;
    output.telemetry.dyno_load_torque_nm_x100 = state.last_dyno_load_torque_nm_x100;
    output.telemetry.lambda_x1000 = state.last_lambda_x1000;
    output.telemetry.map_kpa10 = map;
    output.telemetry.tps_x1000 = state.tps_x1000;
    output.telemetry.crank_angle_deg10 = state.crank_angle_deg10;
    output.telemetry.ve_x1000 = lookup_ve_x1000(&config.air.ve_table, state.rpm, map);
    output.telemetry.battery_mv = state.battery_mv;
    output.telemetry.egt_k_x10 = state.exhaust.egt_k_x10;
    output.telemetry.exhaust_manifold_temp_k_x10 = state.exhaust.exhaust_manifold_temp_k_x10;
    output.telemetry.catalyst_temp_k_x10 = state.exhaust.catalyst_temp_k_x10;
    output.telemetry.afr_x100 = output.combustion.cylinders[0].afr_x100;
    for cyl in 0..CYL {
        output.telemetry.trapped_air_mass_ug[cyl] = state.cylinders[cyl].air_mass_ug;
        output.telemetry.delivered_fuel_mass_ug[cyl] = state.cylinders[cyl].fuel_mass_ug;
        output.telemetry.spark_advance_deg10[cyl] = state.cylinders[cyl].last_spark_advance_deg10.0;
        output.telemetry.dwell_us[cyl] = state.cylinders[cyl].last_dwell_us;
        output.telemetry.injection_pw_us[cyl] = state.cylinders[cyl].last_injection_pw_us;
        output.telemetry.soi_deg10[cyl] = state.cylinders[cyl].last_soi_deg10;
        output.telemetry.eoi_deg10[cyl] = state.cylinders[cyl].last_eoi_deg10;
        output.telemetry.combustion_quality_x1000[cyl] =
            output.combustion.cylinders[cyl].quality_x1000;
        output.telemetry.residual_fraction_x1000[cyl] = state.physics[cyl].residual_fraction_x1000;
        output.telemetry.knock_risk_x1000[cyl] = output.knock.knock_risk_x1000[cyl];
        output.telemetry.misfire_flags[cyl] = is_misfire(output.combustion.cylinders[cyl].misfire);
        output.telemetry.pmax_pa[cyl] = state.physics[cyl].pmax_pa;
        output.telemetry.pmax_angle_deg10[cyl] = state.physics[cyl].pmax_angle_deg10;
        output.telemetry.ca10_deg10[cyl] = state.physics[cyl].ca10_deg10;
        output.telemetry.ca50_deg10[cyl] = state.physics[cyl].ca50_deg10;
        output.telemetry.ca90_deg10[cyl] = state.physics[cyl].ca90_deg10;
    }
    output.telemetry.diagnostic_event_count = output.diagnostics.events.len() as u16;
    output.telemetry.diagnostic_overflow_count = output.diagnostics.overflow_count;
}
