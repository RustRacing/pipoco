const ENGINE_CYCLE_RAD: f64 = 4.0 * core::f64::consts::PI;
use crate::{
    events::{CycleEvent, CycleEventKind, EventTable},
    flow::{compressible_orifice_flow, throttle_effective_area, valve_effective_area, FlowStation},
    geometry::GeometryModel,
    observers,
    params::{
        BurnModel, CombustionConfig, FiredCycleConfig, FiredCycleConfigError, GasProperties,
        KnockModelConfig, MotoredConfigError, MotoredCylinderConfig, OpenSystemConfig,
        OpenSystemConfigError, PlantConfig, PlantConfigError,
    },
    state::{
        ClosedSystemState, CompositionState, ConvergedOpenSystemResult, CylinderPlantTrace,
        FiredCycleResult, FiredCycleSample, ManifoldState, MotoredCycleResult, MotoredSample,
        OpenSystemCycleResult, OpenSystemCylinderState, OpenSystemSample, PlantStepInput,
        PlantStepOutput,
    },
    thermo,
};

pub fn run_motored_cycle(
    config: MotoredCylinderConfig,
    events: &EventTable,
) -> Result<MotoredCycleResult, MotoredConfigError> {
    run_motored_cycle_with_heat_loss(config, events, |_, _, _| 0.0)
}

pub fn run_motored_cycle_with_heat_loss<F>(
    config: MotoredCylinderConfig,
    events: &EventTable,
    heat_loss_per_rad: F,
) -> Result<MotoredCycleResult, MotoredConfigError>
where
    F: Fn(f64, ClosedSystemState, &GeometryModel) -> f64,
{
    config.validate()?;

    let geometry = GeometryModel::new(config.geometry);
    let cycle_start = config.initial_charge.crank_angle_rad;
    let cycle_end = cycle_start + ENGINE_CYCLE_RAD;
    let mut theta = cycle_start;
    let nominal_step = config.integrator.step_rad();
    let mut state = thermo::initial_closed_system_state(config, &geometry);
    let mut samples = Vec::new();
    let mut indicated_work_j = 0.0;

    samples.push(sample_at(config, &geometry, theta, state));

    while theta < cycle_end - 1.0e-12 {
        let mut step = nominal_step.min(cycle_end - theta);
        let mut next_theta = theta + step;
        if let Some((boundary, _)) = events.next_boundary_after(theta, theta + step, cycle_start) {
            next_theta = boundary;
            step = next_theta - theta;
        }

        let next_state = rk4_step(config, &geometry, theta, step, state, &heat_loss_per_rad);
        let p0 = thermo::pressure_pa(config, &geometry, theta, state);
        let p1 = thermo::pressure_pa(config, &geometry, theta + step, next_state);
        let dv0 = geometry.dvolume_dtheta_m3_per_rad(theta);
        let dv1 = geometry.dvolume_dtheta_m3_per_rad(theta + step);
        indicated_work_j += 0.5 * ((p0 * dv0) + (p1 * dv1)) * step;

        theta = next_theta;
        state = next_state;
        samples.push(sample_at(config, &geometry, theta, state));
    }

    Ok(MotoredCycleResult {
        samples,
        indicated_work_j,
    })
}

pub fn run_open_system_cycle(
    config: OpenSystemConfig,
) -> Result<OpenSystemCycleResult, OpenSystemConfigError> {
    run_open_system_cycle_from_state(config, None, None)
}

fn run_open_system_cycle_from_state(
    config: OpenSystemConfig,
    initial_cylinder_state: Option<OpenSystemCylinderState>,
    initial_manifold_state: Option<ManifoldState>,
) -> Result<OpenSystemCycleResult, OpenSystemConfigError> {
    config.validate()?;

    let geometry = GeometryModel::new(config.geometry);
    let omega_rad_per_s = config.rpm * core::f64::consts::TAU / 60.0;
    let step_rad = config.integrator.step_rad();
    let cycle_start = core::f64::consts::PI;
    let cycle_end = cycle_start + ENGINE_CYCLE_RAD;
    let event_table = EventTable::with_events([
        Some(CycleEvent {
            kind: CycleEventKind::Ivo,
            angle_rad_offset: config.intake_valve.open_angle_rad,
        }),
        Some(CycleEvent {
            kind: CycleEventKind::Ivc,
            angle_rad_offset: config.intake_valve.close_angle_rad,
        }),
        Some(CycleEvent {
            kind: CycleEventKind::Evo,
            angle_rad_offset: config.exhaust_valve.open_angle_rad,
        }),
        Some(CycleEvent {
            kind: CycleEventKind::Evc,
            angle_rad_offset: config.exhaust_valve.close_angle_rad,
        }),
        None,
    ]);

    let initial_manifold_mass = thermo::manifold_mass_from_pressure_pa(
        config.manifold.ambient_pressure_pa,
        config.manifold,
        config.fresh_gas,
    );
    let mut manifold_state = initial_manifold_state.unwrap_or(ManifoldState {
        mass_kg: initial_manifold_mass,
        temperature_k: config.manifold.temperature_k,
        composition: CompositionState {
            fresh_mass_kg: initial_manifold_mass,
            burned_mass_kg: 0.0,
        },
    });

    let mut cylinder_state = initial_cylinder_state.unwrap_or_else(|| {
        let initial_composition = CompositionState {
            fresh_mass_kg: 1.0 - config.initial_residual_fraction,
            burned_mass_kg: config.initial_residual_fraction,
        };
        let initial_gas = mixture_gas(config, initial_composition);
        let initial_volume = geometry.volume_m3(cycle_start);
        let initial_temperature_k = config.initial_cylinder_temperature_k
            * (1.0 - config.initial_residual_fraction)
            + config.residual.residual_temperature_k * config.initial_residual_fraction;
        let total_mass = config.initial_cylinder_pressure_pa * initial_volume
            / (initial_gas.r_j_per_kg_k * initial_temperature_k);
        OpenSystemCylinderState {
            mass_kg: total_mass,
            temperature_k: initial_temperature_k,
            composition: CompositionState {
                fresh_mass_kg: total_mass * (1.0 - config.initial_residual_fraction),
                burned_mass_kg: total_mass * config.initial_residual_fraction,
            },
        }
    });

    let mut theta = cycle_start;
    let mut samples = vec![open_sample(
        config,
        &geometry,
        theta,
        cylinder_state,
        manifold_state,
    )];
    let mut pumping_work_j = 0.0;
    let mut throttle_boundary_mass_kg = 0.0;
    let mut exhaust_boundary_mass_kg = 0.0;
    let mut boundary_enthalpy_j = 0.0;
    let mut boundary_work_j = 0.0;
    let mut exhaust_enthalpy_j = 0.0;
    let mut exhaust_enthalpy_temperature_j = 0.0;
    let mut ivc_fresh_mass_kg = None;
    let mut ivc_residual_fraction = None;

    while theta < cycle_end - 1.0e-12 {
        let mut dtheta = step_rad.min(cycle_end - theta);
        let mut next_theta = theta + dtheta;
        let mut hit_ivc_boundary = false;
        if let Some((boundary, boundary_kind)) =
            event_table.next_boundary_after(theta, theta + dtheta, cycle_start)
        {
            hit_ivc_boundary = matches!(boundary_kind, CycleEventKind::Ivc);
            next_theta = boundary;
            dtheta = next_theta - theta;
        }
        let dt = dtheta / omega_rad_per_s;

        let cylinder_gas = mixture_gas(config, cylinder_state.composition);
        let manifold_gas = mixture_gas(config, manifold_state.composition);
        let manifold_internal_energy_old =
            manifold_state.mass_kg * manifold_gas.cv_j_per_kg_k * manifold_state.temperature_k;
        let cylinder_pressure_pa =
            cylinder_pressure(config, &geometry, theta, cylinder_state, cylinder_gas);
        let manifold_pressure_pa =
            thermo::manifold_pressure_pa(manifold_state, config.manifold, manifold_gas);

        let cylinder_station = FlowStation {
            pressure_pa: cylinder_pressure_pa,
            temperature_k: cylinder_state.temperature_k,
            gamma: cylinder_gas.gamma(),
            gas_constant_j_per_kg_k: cylinder_gas.r_j_per_kg_k,
        };
        let manifold_station = FlowStation {
            pressure_pa: manifold_pressure_pa,
            temperature_k: manifold_state.temperature_k,
            gamma: manifold_gas.gamma(),
            gas_constant_j_per_kg_k: manifold_gas.r_j_per_kg_k,
        };
        let ambient_station = FlowStation {
            pressure_pa: config.manifold.ambient_pressure_pa,
            temperature_k: config.manifold.temperature_k,
            gamma: config.fresh_gas.gamma(),
            gas_constant_j_per_kg_k: config.fresh_gas.r_j_per_kg_k,
        };
        let exhaust_station = FlowStation {
            pressure_pa: config.exhaust_backpressure_pa,
            temperature_k: config.exhaust_temperature_k,
            gamma: config.burned_gas.gamma(),
            gas_constant_j_per_kg_k: config.burned_gas.r_j_per_kg_k,
        };

        let throttle_flow = compressible_orifice_flow(
            ambient_station,
            manifold_station,
            throttle_effective_area(config.throttle, config.throttle_position),
            config.throttle.discharge_coefficient,
        );
        let intake_flow = compressible_orifice_flow(
            manifold_station,
            cylinder_station,
            valve_effective_area(theta, config.intake_valve),
            config.intake_valve.discharge_coefficient,
        );
        let exhaust_flow = compressible_orifice_flow(
            cylinder_station,
            exhaust_station,
            valve_effective_area(theta, config.exhaust_valve),
            config.exhaust_valve.discharge_coefficient,
        );

        let actual_throttle_mass_kg = apply_reservoir_flow_to_manifold(
            &mut manifold_state,
            throttle_flow.mass_flow_kg_per_s,
            dt,
        );
        let cylinder_mass_kg_before = cylinder_state.mass_kg;
        let cylinder_internal_energy_old =
            cylinder_mass_kg_before * cylinder_gas.cv_j_per_kg_k * cylinder_state.temperature_k;

        let actual_intake_mass_kg = apply_bidirectional_transfer(
            &mut manifold_state.composition,
            &mut cylinder_state.composition,
            intake_flow.mass_flow_kg_per_s,
            dt,
        );
        manifold_state.mass_kg = manifold_state.composition.total_mass_kg();
        cylinder_state.mass_kg = cylinder_state.composition.total_mass_kg();

        let actual_exhaust_mass_kg = apply_exhaust_exchange(
            &mut cylinder_state.composition,
            exhaust_flow.mass_flow_kg_per_s,
            dt,
        );
        throttle_boundary_mass_kg += actual_throttle_mass_kg;
        exhaust_boundary_mass_kg += actual_exhaust_mass_kg;
        cylinder_state.mass_kg = cylinder_state.composition.total_mass_kg();

        let intake_enthalpy = signed_transfer_enthalpy(
            actual_intake_mass_kg,
            manifold_state.temperature_k,
            manifold_gas,
            cylinder_state.temperature_k,
            cylinder_gas,
        );
        let exhaust_enthalpy = signed_exhaust_enthalpy(
            actual_exhaust_mass_kg,
            cylinder_state.temperature_k,
            cylinder_gas,
            config.exhaust_temperature_k,
            config.burned_gas,
        );
        if actual_exhaust_mass_kg > 0.0 {
            let outflow_enthalpy_j = actual_exhaust_mass_kg
                * thermo::specific_enthalpy_j_per_kg(cylinder_state.temperature_k, cylinder_gas);
            exhaust_enthalpy_j += outflow_enthalpy_j;
            exhaust_enthalpy_temperature_j += outflow_enthalpy_j * cylinder_state.temperature_k;
        }
        let throttle_enthalpy = throttle_enthalpy(
            actual_throttle_mass_kg,
            manifold_state.temperature_k,
            manifold_gas,
            config,
        );
        boundary_enthalpy_j += throttle_enthalpy - exhaust_enthalpy;

        let manifold_energy = manifold_internal_energy_old + throttle_enthalpy - intake_enthalpy;
        let new_manifold_gas = mixture_gas(config, manifold_state.composition);
        manifold_state.temperature_k = if manifold_state.mass_kg > 1.0e-12 {
            (manifold_energy / (manifold_state.mass_kg * new_manifold_gas.cv_j_per_kg_k)).max(200.0)
        } else {
            config.manifold.temperature_k
        };

        let new_cylinder_gas = mixture_gas(config, cylinder_state.composition);
        let dvolume_m3 = geometry.volume_m3(theta + dtheta) - geometry.volume_m3(theta);
        let net_enthalpy_j = cylinder_internal_energy_old + intake_enthalpy - exhaust_enthalpy;
        let cylinder_mass_kg_after = cylinder_state.mass_kg;
        let pre_work_internal_energy_j = net_enthalpy_j;
        let pre_work_temperature_k = if cylinder_mass_kg_after > 0.0
            && new_cylinder_gas.cv_j_per_kg_k > 0.0
        {
            (pre_work_internal_energy_j / (cylinder_mass_kg_after * new_cylinder_gas.cv_j_per_kg_k))
                .max(200.0)
        } else {
            config.initial_cylinder_temperature_k
        };
        let post_work_pressure_pa = if cylinder_mass_kg_after > 0.0 {
            cylinder_mass_kg_after * new_cylinder_gas.r_j_per_kg_k * pre_work_temperature_k
                / geometry.volume_m3(theta + dtheta)
        } else {
            0.0
        };
        let piston_work_j = 0.5 * (cylinder_pressure_pa + post_work_pressure_pa) * dvolume_m3;
        pumping_work_j += piston_work_j;

        let cylinder_energy = net_enthalpy_j - piston_work_j;
        cylinder_state.temperature_k = if cylinder_state.mass_kg > 0.0 {
            (cylinder_energy / (cylinder_state.mass_kg * new_cylinder_gas.cv_j_per_kg_k)).max(200.0)
        } else {
            config.initial_cylinder_temperature_k
        };
        boundary_work_j += piston_work_j;

        theta = next_theta;
        if hit_ivc_boundary {
            ivc_fresh_mass_kg = Some(cylinder_state.composition.fresh_mass_kg);
            ivc_residual_fraction = Some(cylinder_state.composition.residual_fraction());
        }
        samples.push(open_sample(
            config,
            &geometry,
            theta,
            cylinder_state,
            manifold_state,
        ));
    }

    let trapped_fresh_mass_kg =
        ivc_fresh_mass_kg.unwrap_or(cylinder_state.composition.fresh_mass_kg);
    let reference_density = config.manifold.ambient_pressure_pa
        / (config.fresh_gas.r_j_per_kg_k * config.manifold.temperature_k);
    let normalized_trapped_fresh_mass_kg = trapped_fresh_mass_kg.clamp(0.0, f64::MAX);
    let volumetric_efficiency =
        normalized_trapped_fresh_mass_kg / (reference_density * geometry.swept_volume_m3());
    let residual_fraction =
        ivc_residual_fraction.unwrap_or(cylinder_state.composition.residual_fraction());

    Ok(OpenSystemCycleResult {
        samples,
        pumping_work_j,
        pmep_pa: -pumping_work_j / geometry.swept_volume_m3(),
        throttle_boundary_mass_kg,
        exhaust_boundary_mass_kg,
        trapped_fresh_mass_kg: normalized_trapped_fresh_mass_kg,
        volumetric_efficiency,
        residual_fraction,
        cylinder_state,
        manifold_state,
        boundary_enthalpy_j,
        boundary_piston_work_j: boundary_work_j,
        exhaust_enthalpy_j,
        exhaust_enthalpy_temperature_j,
    })
}

pub fn converge_open_system_cycles(
    config: OpenSystemConfig,
) -> Result<ConvergedOpenSystemResult, OpenSystemConfigError> {
    config.validate()?;

    let mut previous_mass = 0.0;
    let mut previous_residual = 0.0;
    let mut latest = run_open_system_cycle(config)?;
    let mut convergence = thermo::convergence_status(
        1,
        previous_mass,
        latest.trapped_fresh_mass_kg,
        previous_residual,
        latest.residual_fraction,
        config.trapped_mass_tolerance_kg,
        config.residual_tolerance,
    );

    previous_mass = latest.trapped_fresh_mass_kg;
    previous_residual = latest.residual_fraction;

    for iteration in 2..=config.max_cycles {
        latest = run_open_system_cycle_from_state(
            config,
            Some(latest.cylinder_state),
            Some(latest.manifold_state),
        )?;
        convergence = thermo::convergence_status(
            iteration,
            previous_mass,
            latest.trapped_fresh_mass_kg,
            previous_residual,
            latest.residual_fraction,
            config.trapped_mass_tolerance_kg,
            config.residual_tolerance,
        );
        if convergence.converged {
            return Ok(ConvergedOpenSystemResult {
                cycle: latest,
                convergence,
            });
        }
        previous_mass = latest.trapped_fresh_mass_kg;
        previous_residual = latest.residual_fraction;
    }

    Ok(ConvergedOpenSystemResult {
        cycle: latest,
        convergence,
    })
}

pub fn run_fired_cycle(
    config: FiredCycleConfig,
) -> Result<FiredCycleResult, FiredCycleConfigError> {
    config.validate()?;

    let geometry = GeometryModel::new(config.geometry);
    let cycle_start = core::f64::consts::PI;
    let closed_cycle_start_rad = config.closed_cycle_start_rad;
    let closed_cycle_end_rad = config.closed_cycle_end_rad;
    let combustion_start_rad = combustion_start_angle(config, cycle_start);
    let step = config.integrator.step_rad();
    let mean_piston_speed = 2.0 * config.geometry.stroke_m * config.rpm / 60.0;
    let omega_rad_per_s = config.rpm * core::f64::consts::TAU / 60.0;
    let motored_reference = run_motored_cycle(
        MotoredCylinderConfig {
            geometry: config.geometry,
            wall: config.wall,
            initial_charge: crate::params::InitialChargeState {
                pressure_pa: config.initial_pressure_pa,
                temperature_k: config.initial_temperature_k,
                crank_angle_rad: closed_cycle_start_rad,
            },
            gas: config.gas,
            integrator: config.integrator,
        },
        &EventTable::empty(),
    )
    .map_err(|_| FiredCycleConfigError::InvalidInitialState)?;

    let initial_composition = CompositionState {
        fresh_mass_kg: 1.0 - config.initial_burned_fraction,
        burned_mass_kg: config.initial_burned_fraction,
    };
    let initial_gas =
        thermo::mass_weighted_gas_properties(initial_composition, config.gas, config.burned_gas);
    let initial_mass = config
        .initial_mass_kg
        .unwrap_or(
            config.initial_pressure_pa * geometry.volume_m3(closed_cycle_start_rad)
                / (initial_gas.r_j_per_kg_k * config.initial_temperature_k),
        )
        .max(1.0e-12);
    let initial_enthalpy_j = initial_mass
        * thermo::specific_enthalpy_j_per_kg(config.initial_temperature_k, initial_gas);
    let mut state = ClosedSystemState {
        mass_kg: initial_mass,
        temperature_k: config.initial_temperature_k,
    };
    let mut burned_fraction = config.initial_burned_fraction;
    let mut theta = closed_cycle_start_rad;
    let mut samples = Vec::new();
    let mut indicated_work_j = 0.0;
    let mut wall_heat_j = 0.0;
    let mut pmax_pa = 0.0;
    let mut pmax_angle_rad = cycle_start;
    let mut ca10 = None;
    let mut ca50 = None;
    let mut ca90 = None;

    while theta < closed_cycle_end_rad - 1.0e-12 {
        let dtheta = step.min(closed_cycle_end_rad - theta);
        let gas = thermo::mass_weighted_gas_properties(
            CompositionState {
                fresh_mass_kg: 1.0 - burned_fraction,
                burned_mass_kg: burned_fraction,
            },
            config.gas,
            config.burned_gas,
        );
        let current_burn_fraction = burn_fraction(theta, config, combustion_start_rad);
        let pressure_pa =
            state.mass_kg * gas.r_j_per_kg_k * state.temperature_k / geometry.volume_m3(theta);
        let motored_pressure_pa = sample_motored_pressure(&motored_reference, theta);
        let current_heat_release_rate = config.combustion.combustion_efficiency
            * config.combustion.fuel_mass_kg
            * config.combustion.fuel_lhv_j_per_kg
            * wiebe_dx_dtheta(theta, config, combustion_start_rad);
        let current_wall_heat_rate = woschni_wall_heat_rate(&WoschniWallHeatState {
            config: &config,
            geometry: &geometry,
            theta_rad: theta,
            pressure_pa,
            temperature_k: state.temperature_k,
            mean_piston_speed,
            motored_pressure_pa,
            omega_rad_per_s,
        });
        let dtemp_dtheta = |theta_rad: f64, temp: f64, motored_pressure_pa: f64| -> f64 {
            let local_burn_fraction = burn_fraction(theta_rad, config, combustion_start_rad);
            let local_burn_fraction_rate = wiebe_dx_dtheta(theta_rad, config, combustion_start_rad);
            let local_state = ClosedSystemState {
                mass_kg: state.mass_kg,
                temperature_k: temp,
            };
            let local_gas = thermo::mass_weighted_gas_properties(
                CompositionState {
                    fresh_mass_kg: 1.0 - local_burn_fraction,
                    burned_mass_kg: local_burn_fraction,
                },
                config.gas,
                config.burned_gas,
            );
            let local_pressure_pa =
                local_state.mass_kg * local_gas.r_j_per_kg_k * local_state.temperature_k
                    / geometry.volume_m3(theta_rad);
            let heat_release_rate = config.combustion.combustion_efficiency
                * config.combustion.fuel_mass_kg
                * config.combustion.fuel_lhv_j_per_kg
                * wiebe_dx_dtheta(theta_rad, config, combustion_start_rad);
            let wall_heat_rate = woschni_wall_heat_rate(&WoschniWallHeatState {
                config: &config,
                geometry: &geometry,
                theta_rad,
                pressure_pa: local_pressure_pa,
                temperature_k: local_state.temperature_k,
                mean_piston_speed,
                motored_pressure_pa,
                omega_rad_per_s,
            });
            let composition_cv_rate = (config.burned_gas.cv_j_per_kg_k - config.gas.cv_j_per_kg_k)
                * local_burn_fraction_rate;
            (heat_release_rate
                - wall_heat_rate
                - local_pressure_pa * geometry.dvolume_dtheta_m3_per_rad(theta_rad)
                - local_state.mass_kg * local_state.temperature_k * composition_cv_rate)
                / (local_state.mass_kg * local_gas.cv_j_per_kg_k)
        };
        let dtemp1 = dtemp_dtheta(theta, state.temperature_k, motored_pressure_pa);
        let theta_mid = theta + 0.5 * dtheta;
        let dtemp2 = dtemp_dtheta(
            theta_mid,
            state.temperature_k + 0.5 * dtheta * dtemp1,
            sample_motored_pressure(&motored_reference, theta_mid),
        );
        let dtemp3 = dtemp_dtheta(
            theta_mid,
            state.temperature_k + 0.5 * dtheta * dtemp2,
            sample_motored_pressure(&motored_reference, theta_mid),
        );
        let theta_end = theta + dtheta;
        let motored_pressure_pa_end = sample_motored_pressure(&motored_reference, theta_end);
        let next_burn_fraction =
            burn_fraction(theta_end, config, combustion_start_rad).max(burned_fraction);
        let dtemp4 = dtemp_dtheta(
            theta_end,
            state.temperature_k + dtheta * dtemp3,
            motored_pressure_pa_end,
        );
        let dv_dtheta = geometry.dvolume_dtheta_m3_per_rad(theta);
        let next_temperature_k =
            state.temperature_k + (dtheta / 6.0) * (dtemp1 + 2.0 * dtemp2 + 2.0 * dtemp3 + dtemp4);
        let next_state = ClosedSystemState {
            mass_kg: state.mass_kg,
            temperature_k: next_temperature_k,
        };
        let next_gas = thermo::mass_weighted_gas_properties(
            CompositionState {
                fresh_mass_kg: 1.0 - next_burn_fraction,
                burned_mass_kg: next_burn_fraction,
            },
            config.gas,
            config.burned_gas,
        );
        let next_pressure_pa =
            next_state.mass_kg * next_gas.r_j_per_kg_k * next_state.temperature_k
                / geometry.volume_m3(theta + dtheta);

        indicated_work_j += 0.5
            * (pressure_pa * dv_dtheta
                + next_pressure_pa * geometry.dvolume_dtheta_m3_per_rad(theta + dtheta))
            * dtheta;
        let wall_heat_rate_end = woschni_wall_heat_rate(&WoschniWallHeatState {
            config: &config,
            geometry: &geometry,
            theta_rad: theta + dtheta,
            pressure_pa: next_pressure_pa,
            temperature_k: next_state.temperature_k,
            mean_piston_speed,
            motored_pressure_pa: motored_pressure_pa_end,
            omega_rad_per_s,
        });
        wall_heat_j += 0.5 * (current_wall_heat_rate + wall_heat_rate_end) * dtheta;

        if ca10.is_none() && current_burn_fraction >= 0.10 {
            ca10 = Some(theta);
        }
        if ca50.is_none() && current_burn_fraction >= 0.50 {
            ca50 = Some(theta);
        }
        if ca90.is_none() && current_burn_fraction >= 0.90 {
            ca90 = Some(theta);
        }
        if pressure_pa > pmax_pa {
            pmax_pa = pressure_pa;
            pmax_angle_rad = theta;
        }

        samples.push(FiredCycleSample {
            crank_angle_rad: theta,
            pressure_pa,
            temperature_k: state.temperature_k,
            burn_fraction: current_burn_fraction,
            heat_release_rate_j_per_rad: current_heat_release_rate,
            wall_heat_rate_j_per_rad: current_wall_heat_rate,
        });

        theta += dtheta;
        state = next_state;
        burned_fraction = next_burn_fraction;
    }

    let imep_gross_pa = indicated_work_j / geometry.swept_volume_m3();
    let pmep_pa = config.open_system_pmep_pa;
    let load_kpa = config.manifold_pressure_pa / 1000.0;
    let krpm = config.rpm / 1000.0;
    let fmep_pa = (config.losses.fmep_base_pa
        + config.losses.fmep_rpm_pa_per_krpm * krpm
        + config.losses.fmep_rpm2_pa_per_krpm2 * krpm * krpm
        + config.losses.fmep_load_pa_per_kpa * load_kpa)
        .max(0.0);
    let bmep_pa = imep_gross_pa - pmep_pa - fmep_pa;
    let brake_torque_nm = bmep_pa * geometry.swept_volume_m3() / (4.0 * core::f64::consts::PI);
    let air_mass_kg = initial_mass * (1.0 - config.initial_burned_fraction);
    let lambda = if config.combustion.fuel_mass_kg > 0.0 {
        air_mass_kg / (config.combustion.fuel_mass_kg * config.combustion.stoich_afr)
    } else {
        f64::INFINITY
    };
    let final_enthalpy_j = state.mass_kg
        * thermo::specific_enthalpy_j_per_kg(
            state.temperature_k,
            thermo::mass_weighted_gas_properties(
                CompositionState {
                    fresh_mass_kg: 1.0 - burned_fraction,
                    burned_mass_kg: burned_fraction,
                },
                config.gas,
                config.burned_gas,
            ),
        );
    let exhaust_enthalpy_j = final_enthalpy_j - initial_enthalpy_j;

    Ok(FiredCycleResult {
        samples,
        closed_cycle_start_rad,
        closed_cycle_end_rad,
        imep_gross_pa,
        pmep_pa,
        fmep_pa,
        bmep_pa,
        brake_torque_nm,
        lambda,
        ca10_rad: ca10,
        ca50_rad: ca50,
        ca90_rad: ca90,
        pmax_pa,
        pmax_angle_rad,
        indicated_work_j,
        wall_heat_j,
        exhaust_enthalpy_j,
    })
}

pub fn advance_plant_step(
    config: &PlantConfig,
    input: &PlantStepInput,
) -> Result<PlantStepOutput, PlantConfigError> {
    config.validate()?;
    if !(0.0..=1.0).contains(&input.throttle_position) {
        return Err(PlantConfigError::InvalidThrottlePosition);
    }
    if !(input.rpm >= 0.0 && input.rpm.is_finite()) {
        return Err(PlantConfigError::InvalidTimingWindow);
    }
    if !(input.window_s > 0.0 && input.now_s.is_finite() && input.window_s.is_finite()) {
        return Err(PlantConfigError::InvalidTimingWindow);
    }
    if !input.crank_angle_rad.is_finite() {
        return Err(PlantConfigError::InvalidTimingWindow);
    }
    if input.cylinders.len() != config.cylinders.len() {
        return Err(PlantConfigError::InvalidCylinderCount);
    }
    let omega_prev = input.rpm * core::f64::consts::TAU / 60.0;

    let open_cycle = converge_open_system_cycles(OpenSystemConfig {
        geometry: config.geometry,
        fresh_gas: config.gas,
        burned_gas: config.burned_gas,
        integrator: config.integrator,
        manifold: config.manifold,
        throttle: config.throttle,
        throttle_position: input.throttle_position,
        intake_valve: config.intake_valve,
        exhaust_valve: config.exhaust_valve,
        residual: config.residual,
        initial_cylinder_pressure_pa: config.initial_pressure_pa,
        initial_cylinder_temperature_k: config.initial_temperature_k,
        initial_residual_fraction: config.initial_residual_fraction,
        exhaust_backpressure_pa: config.exhaust_backpressure_pa,
        exhaust_temperature_k: config.exhaust_temperature_k,
        rpm: input.rpm,
        trapped_mass_tolerance_kg: 1.0e-6,
        residual_tolerance: 1.0e-4,
        max_cycles: 4,
    })
    .map_err(|_| PlantConfigError::InvalidModelConfig)?;

    let open_cycle = open_cycle.cycle;
    let ivc_angle_rad = config.intake_valve.close_angle_rad;
    let evo_angle_rad = if config.exhaust_valve.open_angle_rad <= ivc_angle_rad {
        config.exhaust_valve.open_angle_rad + ENGINE_CYCLE_RAD
    } else {
        config.exhaust_valve.open_angle_rad
    };
    let ivc_sample = open_cycle
        .samples
        .iter()
        .find(|sample| (sample.crank_angle_rad - ivc_angle_rad).abs() < 1.0e-12)
        .copied()
        .or_else(|| open_cycle.samples.first().copied())
        .ok_or(PlantConfigError::InvalidModelConfig)?;

    let mut traces = Vec::with_capacity(config.cylinders.len());
    let mut total_brake_torque_nm = 0.0;
    let mut total_lambda = 0.0;
    let mut total_egt_k = 0.0;
    let mut total_knock_margin = 0.0;

    for (cylinder_cfg, command) in config.cylinders.iter().zip(&input.cylinders) {
        let fired_config = FiredCycleConfig {
            geometry: config.geometry,
            wall: config.wall,
            gas: config.gas,
            burned_gas: config.burned_gas,
            integrator: config.integrator,
            initial_pressure_pa: ivc_sample.cylinder_pressure_pa,
            initial_temperature_k: ivc_sample.cylinder_temperature_k,
            initial_mass_kg: Some(if open_cycle.residual_fraction < 1.0 {
                (open_cycle.trapped_fresh_mass_kg / (1.0 - open_cycle.residual_fraction))
                    .max(1.0e-12)
            } else {
                1.0e-12
            }),
            initial_burned_fraction: open_cycle.residual_fraction,
            rpm: input.rpm,
            closed_cycle_start_rad: ivc_angle_rad,
            closed_cycle_end_rad: evo_angle_rad,
            manifold_pressure_pa: open_cycle
                .samples
                .last()
                .map(|sample| sample.manifold_pressure_pa)
                .unwrap_or(config.manifold.ambient_pressure_pa),
            open_system_pmep_pa: open_cycle.pmep_pa,
            combustion: CombustionConfig {
                burn_model: config.combustion.burn_model,
                spark_angle_rad: command.spark_angle_rad + cylinder_cfg.phase_offset_rad,
                fuel_mass_kg: command.fuel_mass_kg,
                fuel_lhv_j_per_kg: config.combustion.fuel_lhv_j_per_kg,
                combustion_efficiency: config.combustion.combustion_efficiency,
                stoich_afr: config.combustion.stoich_afr,
            },
            woschni: config.woschni,
            losses: config.losses,
        };
        let fired =
            run_fired_cycle(fired_config).map_err(|_| PlantConfigError::InvalidModelConfig)?;
        let knock = crate::observers::estimate_knock(
            &fired,
            fired_config,
            KnockModelConfig {
                octane_number: 95.0,
                a: 0.01768,
                n1: 3.402,
                n2: -1.7,
                b: 3800.0,
            },
        );
        let exhaust = crate::observers::estimate_exhaust_from_open_cycle(&open_cycle);
        let trace = CylinderPlantTrace {
            brake_torque_nm: fired.brake_torque_nm,
            lambda: fired.lambda,
            knock_margin: knock.knock_margin,
            egt_k: exhaust.mean_exhaust_temperature_k,
        };
        total_brake_torque_nm += trace.brake_torque_nm;
        total_lambda += trace.lambda;
        total_egt_k += trace.egt_k;
        total_knock_margin += trace.knock_margin;
        traces.push(trace);
    }

    let cylinder_count = traces.len().max(1) as f64;
    let net_torque_nm = total_brake_torque_nm - input.load_torque_nm;
    let angular_accel_rad_per_s2 = net_torque_nm / config.crank_inertia_kg_m2;
    let omega = (omega_prev + angular_accel_rad_per_s2 * input.window_s).max(0.0);
    let omega_mean = 0.5 * (omega_prev + omega);
    let crank_angle_rad =
        (input.crank_angle_rad + omega_mean * input.window_s).rem_euclid(ENGINE_CYCLE_RAD);
    let rpm = omega * 60.0 / core::f64::consts::TAU;

    Ok(PlantStepOutput {
        crank_angle_rad,
        rpm,
        manifold_pressure_pa: open_cycle
            .samples
            .last()
            .map(|sample| sample.manifold_pressure_pa)
            .unwrap_or(config.manifold.ambient_pressure_pa),
        lambda: total_lambda / cylinder_count,
        egt_k: total_egt_k / cylinder_count,
        knock_margin: total_knock_margin / cylinder_count,
        brake_torque_nm: total_brake_torque_nm,
        cylinders: traces,
    })
}

fn sample_at(
    config: MotoredCylinderConfig,
    geometry: &GeometryModel,
    theta_rad: f64,
    state: ClosedSystemState,
) -> MotoredSample {
    MotoredSample {
        crank_angle_rad: theta_rad,
        volume_m3: geometry.volume_m3(theta_rad),
        pressure_pa: thermo::pressure_pa(config, geometry, theta_rad, state),
        temperature_k: state.temperature_k,
        wall_area_m2: geometry.wall_area_m2(theta_rad),
    }
}

fn rk4_step<F>(
    config: MotoredCylinderConfig,
    geometry: &GeometryModel,
    theta_rad: f64,
    step_rad: f64,
    state: ClosedSystemState,
    heat_loss_per_rad: &F,
) -> ClosedSystemState
where
    F: Fn(f64, ClosedSystemState, &GeometryModel) -> f64,
{
    let k1 = dtemp_dtheta(config, geometry, theta_rad, state, heat_loss_per_rad);
    let s2 = ClosedSystemState {
        mass_kg: state.mass_kg,
        temperature_k: state.temperature_k + 0.5 * step_rad * k1,
    };
    let k2 = dtemp_dtheta(
        config,
        geometry,
        theta_rad + 0.5 * step_rad,
        s2,
        heat_loss_per_rad,
    );
    let s3 = ClosedSystemState {
        mass_kg: state.mass_kg,
        temperature_k: state.temperature_k + 0.5 * step_rad * k2,
    };
    let k3 = dtemp_dtheta(
        config,
        geometry,
        theta_rad + 0.5 * step_rad,
        s3,
        heat_loss_per_rad,
    );
    let s4 = ClosedSystemState {
        mass_kg: state.mass_kg,
        temperature_k: state.temperature_k + step_rad * k3,
    };
    let k4 = dtemp_dtheta(
        config,
        geometry,
        theta_rad + step_rad,
        s4,
        heat_loss_per_rad,
    );
    ClosedSystemState {
        mass_kg: state.mass_kg,
        temperature_k: state.temperature_k + (step_rad / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4),
    }
}

fn dtemp_dtheta<F>(
    config: MotoredCylinderConfig,
    geometry: &GeometryModel,
    theta_rad: f64,
    state: ClosedSystemState,
    heat_loss_per_rad: &F,
) -> f64
where
    F: Fn(f64, ClosedSystemState, &GeometryModel) -> f64,
{
    let pressure_pa = thermo::pressure_pa(config, geometry, theta_rad, state);
    let dvolume_dtheta = geometry.dvolume_dtheta_m3_per_rad(theta_rad);
    let dq_wall_dtheta = heat_loss_per_rad(theta_rad, state, geometry);
    (-pressure_pa * dvolume_dtheta - dq_wall_dtheta) / (state.mass_kg * config.gas.cv_j_per_kg_k)
}

fn mixture_gas(config: OpenSystemConfig, composition: CompositionState) -> GasProperties {
    thermo::mass_weighted_gas_properties(composition, config.fresh_gas, config.burned_gas)
}

fn cylinder_pressure(
    _config: OpenSystemConfig,
    geometry: &GeometryModel,
    theta_rad: f64,
    state: OpenSystemCylinderState,
    gas: GasProperties,
) -> f64 {
    state.mass_kg * gas.r_j_per_kg_k * state.temperature_k / geometry.volume_m3(theta_rad)
}

fn open_sample(
    config: OpenSystemConfig,
    geometry: &GeometryModel,
    theta_rad: f64,
    cylinder_state: OpenSystemCylinderState,
    manifold_state: ManifoldState,
) -> OpenSystemSample {
    let cylinder_gas = mixture_gas(config, cylinder_state.composition);
    let manifold_gas = mixture_gas(config, manifold_state.composition);
    OpenSystemSample {
        crank_angle_rad: theta_rad,
        cylinder_pressure_pa: cylinder_pressure(
            config,
            geometry,
            theta_rad,
            cylinder_state,
            cylinder_gas,
        ),
        manifold_pressure_pa: thermo::manifold_pressure_pa(
            manifold_state,
            config.manifold,
            manifold_gas,
        ),
        cylinder_temperature_k: cylinder_state.temperature_k,
        manifold_mass_kg: manifold_state.mass_kg,
        cylinder_mass_kg: cylinder_state.mass_kg,
        residual_fraction: cylinder_state.composition.residual_fraction(),
    }
}

fn apply_reservoir_flow_to_manifold(
    manifold: &mut ManifoldState,
    mass_flow_kg_per_s: f64,
    dt: f64,
) -> f64 {
    let delta = mass_flow_kg_per_s * dt;
    if delta >= 0.0 {
        manifold.composition.fresh_mass_kg += delta;
        manifold.mass_kg = manifold.composition.total_mass_kg();
        delta
    } else {
        let removed = take_from_composition(&mut manifold.composition, -delta);
        manifold.mass_kg = manifold.composition.total_mass_kg();
        -removed.total_mass_kg()
    }
}

fn apply_bidirectional_transfer(
    source: &mut CompositionState,
    sink: &mut CompositionState,
    mass_flow_kg_per_s: f64,
    dt: f64,
) -> f64 {
    let delta = mass_flow_kg_per_s * dt;
    if delta > 0.0 {
        let transferred = take_from_composition(source, delta);
        sink.fresh_mass_kg += transferred.fresh_mass_kg;
        sink.burned_mass_kg += transferred.burned_mass_kg;
        transferred.total_mass_kg()
    } else if delta < 0.0 {
        let transferred = take_from_composition(sink, -delta);
        source.fresh_mass_kg += transferred.fresh_mass_kg;
        source.burned_mass_kg += transferred.burned_mass_kg;
        -transferred.total_mass_kg()
    } else {
        0.0
    }
}

fn apply_exhaust_exchange(
    cylinder: &mut CompositionState,
    mass_flow_kg_per_s: f64,
    dt: f64,
) -> f64 {
    let delta = mass_flow_kg_per_s * dt;
    if delta > 0.0 {
        let removed = take_from_composition(cylinder, delta);
        removed.total_mass_kg()
    } else if delta < 0.0 {
        cylinder.burned_mass_kg += -delta;
        delta
    } else {
        0.0
    }
}

fn take_from_composition(
    composition: &mut CompositionState,
    requested_mass_kg: f64,
) -> CompositionState {
    let available = composition.total_mass_kg();
    if available <= 0.0 || requested_mass_kg <= 0.0 {
        return CompositionState {
            fresh_mass_kg: 0.0,
            burned_mass_kg: 0.0,
        };
    }
    let taken = requested_mass_kg.min(available);
    let fresh_fraction = composition.fresh_mass_kg / available;
    let burned_fraction = composition.burned_mass_kg / available;
    let transferred = CompositionState {
        fresh_mass_kg: taken * fresh_fraction,
        burned_mass_kg: taken * burned_fraction,
    };
    composition.fresh_mass_kg -= transferred.fresh_mass_kg;
    composition.burned_mass_kg -= transferred.burned_mass_kg;
    transferred
}

fn signed_transfer_enthalpy(
    transferred_mass_kg: f64,
    source_temperature_k: f64,
    source_gas: GasProperties,
    sink_temperature_k: f64,
    sink_gas: GasProperties,
) -> f64 {
    if transferred_mass_kg >= 0.0 {
        transferred_mass_kg * thermo::specific_enthalpy_j_per_kg(source_temperature_k, source_gas)
    } else {
        transferred_mass_kg * thermo::specific_enthalpy_j_per_kg(sink_temperature_k, sink_gas)
    }
}

fn signed_exhaust_enthalpy(
    transferred_mass_kg: f64,
    cylinder_temperature_k: f64,
    cylinder_gas: GasProperties,
    exhaust_temperature_k: f64,
    exhaust_gas: GasProperties,
) -> f64 {
    if transferred_mass_kg >= 0.0 {
        transferred_mass_kg
            * thermo::specific_enthalpy_j_per_kg(cylinder_temperature_k, cylinder_gas)
    } else {
        transferred_mass_kg * thermo::specific_enthalpy_j_per_kg(exhaust_temperature_k, exhaust_gas)
    }
}

fn throttle_enthalpy(
    transferred_mass_kg: f64,
    manifold_temperature_k: f64,
    manifold_gas: GasProperties,
    config: OpenSystemConfig,
) -> f64 {
    if transferred_mass_kg >= 0.0 {
        transferred_mass_kg
            * thermo::specific_enthalpy_j_per_kg(config.manifold.temperature_k, config.fresh_gas)
    } else {
        transferred_mass_kg
            * thermo::specific_enthalpy_j_per_kg(manifold_temperature_k, manifold_gas)
    }
}

fn burn_fraction(theta_rad: f64, config: FiredCycleConfig, combustion_start_rad: f64) -> f64 {
    if theta_rad <= combustion_start_rad {
        return 0.0;
    }
    match config.combustion.burn_model {
        BurnModel::SingleWiebe {
            a, m, duration_rad, ..
        } => {
            wiebe_fraction_component(theta_rad, combustion_start_rad, duration_rad, a, m)
                / wiebe_normalization(a)
        }
        BurnModel::DoubleWiebe {
            premixed_fraction,
            premixed_a,
            premixed_m,
            premixed_duration_rad,
            main_a,
            main_m,
            main_duration_rad,
            ..
        } => {
            premixed_fraction
                * (wiebe_fraction_component(
                    theta_rad,
                    combustion_start_rad,
                    premixed_duration_rad,
                    premixed_a,
                    premixed_m,
                ) / wiebe_normalization(premixed_a))
                + (1.0 - premixed_fraction)
                    * (wiebe_fraction_component(
                        theta_rad,
                        combustion_start_rad,
                        main_duration_rad,
                        main_a,
                        main_m,
                    ) / wiebe_normalization(main_a))
        }
    }
}

fn wiebe_dx_dtheta(theta_rad: f64, config: FiredCycleConfig, combustion_start_rad: f64) -> f64 {
    if theta_rad <= combustion_start_rad {
        return 0.0;
    }
    match config.combustion.burn_model {
        BurnModel::SingleWiebe {
            a, m, duration_rad, ..
        } => {
            wiebe_derivative_component(theta_rad, combustion_start_rad, duration_rad, a, m)
                / wiebe_normalization(a)
        }
        BurnModel::DoubleWiebe {
            premixed_fraction,
            premixed_a,
            premixed_m,
            premixed_duration_rad,
            main_a,
            main_m,
            main_duration_rad,
            ..
        } => {
            premixed_fraction
                * (wiebe_derivative_component(
                    theta_rad,
                    combustion_start_rad,
                    premixed_duration_rad,
                    premixed_a,
                    premixed_m,
                ) / wiebe_normalization(premixed_a))
                + (1.0 - premixed_fraction)
                    * (wiebe_derivative_component(
                        theta_rad,
                        combustion_start_rad,
                        main_duration_rad,
                        main_a,
                        main_m,
                    ) / wiebe_normalization(main_a))
        }
    }
}

fn combustion_start_angle(config: FiredCycleConfig, cycle_start: f64) -> f64 {
    observers::combustion_start_angle(config, cycle_start)
}

fn wiebe_normalization(a: f64) -> f64 {
    1.0 - (-a).exp()
}

fn wiebe_fraction_component(theta_rad: f64, soc: f64, duration_rad: f64, a: f64, m: f64) -> f64 {
    let x = ((theta_rad - soc) / duration_rad).clamp(0.0, 1.0);
    1.0 - (-a * x.powf(m + 1.0)).exp()
}

fn wiebe_derivative_component(theta_rad: f64, soc: f64, duration_rad: f64, a: f64, m: f64) -> f64 {
    if theta_rad >= soc + duration_rad {
        return 0.0;
    }
    let x = ((theta_rad - soc) / duration_rad).clamp(0.0, 1.0);
    (a * (m + 1.0) / duration_rad) * x.powf(m) * (-a * x.powf(m + 1.0)).exp()
}

fn sample_motored_pressure(reference: &MotoredCycleResult, theta_rad: f64) -> f64 {
    reference
        .samples
        .iter()
        .min_by(|a, b| {
            (a.crank_angle_rad - theta_rad)
                .abs()
                .partial_cmp(&(b.crank_angle_rad - theta_rad).abs())
                .unwrap()
        })
        .map(|sample| sample.pressure_pa)
        .unwrap_or(101_325.0)
}

fn woschni_wall_heat_rate(state: &WoschniWallHeatState<'_>) -> f64 {
    let combustion_term = state.config.woschni.c2
        * ((state.geometry.swept_volume_m3() * state.config.woschni.t_ref_k)
            / (state.config.woschni.p_ref_pa * state.config.woschni.v_ref_m3))
        * (state.pressure_pa - state.motored_pressure_pa);
    let velocity =
        (state.config.woschni.c1 * state.mean_piston_speed + combustion_term).max(1.0e-6);
    let h = state.config.woschni.c
        * (state.pressure_pa / 1000.0).powf(0.8)
        * state.config.geometry.bore_m.powf(-0.2)
        * state.temperature_k.powf(-0.55)
        * velocity.powf(0.8);
    let wall_heat_rate_j_per_s = h
        * state.geometry.wall_area_m2(state.theta_rad)
        * (state.temperature_k - state.config.wall.wall_temperature_k);
    if state.omega_rad_per_s > 0.0 {
        wall_heat_rate_j_per_s / state.omega_rad_per_s
    } else {
        0.0
    }
}

struct WoschniWallHeatState<'a> {
    config: &'a FiredCycleConfig,
    geometry: &'a GeometryModel,
    theta_rad: f64,
    pressure_pa: f64,
    temperature_k: f64,
    mean_piston_speed: f64,
    motored_pressure_pa: f64,
    omega_rad_per_s: f64,
}
