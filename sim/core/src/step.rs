use crate::{
    air::{
        estimate_air_mass_ug, estimate_map_kpa10, estimate_speed_density_air_mass_ug,
        lookup_ve_x1000, update_intake_manifold, update_manifold_map_kpa10,
        valve_event_airflow_modifier_x1000, IntakeManifoldConfig, IntakeManifoldState,
    },
    combustion::CombustionFrame,
    config::{PlantConfig, PlantPhysicsMode},
    crank::{choose_substep_us, update_crank_with_remainder},
    cycle::accumulate_cycle_work,
    dyno::{dyno_load_torque, dyno_pid_load_torque},
    fuel::{
        fuel_mass, injection_phasing, update_wall_film, AcceptedInjection, WallFilmParams,
        WallFilmState,
    },
    io::{DiagnosticKind, PlantStepError, PlantStepInput, PlantStepOutput},
    sensors::SensorSnapshot,
    spark::{dwell_quality, spark_phase_quality, AcceptedSpark},
    state::PlantState,
    trigger::generate_edges,
    types::*,
};

mod combustion;
mod output;
mod thermal;

use combustion::evaluate_all_combustion;
use output::{fill_dyno, fill_knock, fill_sensors, fill_telemetry};
use thermal::{dt_ms_u16, observed_lambda, update_exhaust_state};

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
        output.consumed_events.record_injection_seen();
        if cyl >= config.cylinder_count as usize {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InvalidCylinderIndex,
                Some(command.cylinder),
            );
            output.consumed_events.record_injection_ignored();
            continue;
        }
        if input.ecu_outputs.fuel_cut || !config.fuel.enabled {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InputEventIgnoredDueToCut,
                Some(command.cylinder),
            );
            output.consumed_events.record_injection_ignored();
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
        output
            .consumed_events
            .record_injection_fuel_mass(idx, accepted_event.fuel_mass_ug);
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
        output.consumed_events.record_spark_seen();
        if cyl >= config.cylinder_count as usize {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InvalidCylinderIndex,
                Some(command.cylinder),
            );
            output.consumed_events.record_spark_ignored();
            continue;
        }
        if input.ecu_outputs.spark_cut {
            output.diagnostics.push(
                timestamp,
                DiagnosticKind::InputEventIgnoredDueToCut,
                Some(command.cylinder),
            );
            output.consumed_events.record_spark_ignored();
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        fuel::{InjectionCommand, InjectionTimingMode},
        spark::SparkCommand,
        Plant,
    };

    #[test]
    fn invalid_injection_cylinder_is_counted_and_reported_without_fuel_mass() {
        let mut plant = Plant::<4, 16, 2>::new(PlantConfig::<4>::default_four());
        let mut input = PlantStepInput::<4, 2>::idle(Micros(1000));
        input
            .ecu_outputs
            .injection_events
            .push(InjectionCommand {
                cylinder: CylinderIndex(7),
                mode: InjectionTimingMode::StartOfInjection,
                angle_deg10: CrankDeg10(0),
                pulse_width_us: Micros(2000),
                injector_flow_ug_per_us: MicrogramsPerMicros(5),
                deadtime_us: Micros(1000),
            })
            .unwrap();
        let mut output = PlantStepOutput::<4, 16, 2>::empty();

        plant.step(&input, &mut output).unwrap();

        assert_eq!(output.consumed_events.injection_count, 1);
        assert_eq!(output.consumed_events.ignored_injection_count, 1);
        assert_eq!(output.consumed_events.injection_fuel_mass_ug[0], MassUg(0));
        assert!(output
            .diagnostics
            .events
            .as_slice()
            .iter()
            .any(|event| event.kind == DiagnosticKind::InvalidCylinderIndex));
    }

    #[test]
    fn invalid_event_diagnostics_saturate_without_losing_accounting() {
        const EVENTS: usize = MAX_DIAGNOSTIC_EVENTS_PER_STEP + 4;
        let mut plant = Plant::<4, 16, EVENTS>::new(PlantConfig::<4>::default_four());
        let mut input = PlantStepInput::<4, EVENTS>::idle(Micros(1000));
        for _ in 0..EVENTS {
            input
                .ecu_outputs
                .spark_events
                .push(SparkCommand {
                    cylinder: CylinderIndex(9),
                    spark_angle_deg10: CrankDeg10(0),
                    dwell_us: Micros(1000),
                    coil_energy_x1000: 1000,
                })
                .unwrap();
        }
        let mut output = PlantStepOutput::<4, 16, EVENTS>::empty();

        plant.step(&input, &mut output).unwrap();

        assert_eq!(output.consumed_events.spark_count, EVENTS);
        assert_eq!(output.consumed_events.ignored_spark_count, EVENTS);
        assert_eq!(
            output.diagnostics.events.len(),
            MAX_DIAGNOSTIC_EVENTS_PER_STEP
        );
        assert!(output
            .diagnostics
            .events
            .as_slice()
            .iter()
            .all(|event| event.kind == DiagnosticKind::InvalidCylinderIndex));
        assert_eq!(output.diagnostics.overflow_count, 8);
    }
}
