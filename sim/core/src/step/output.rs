use crate::{
    air::lookup_ve_x1000,
    config::{DynoMode, PlantConfig, PlantPhysicsMode},
    dyno::{horsepower_x100, DynoFrame, DynoSweepPoint},
    io::{PlantStepInput, PlantStepOutput},
    knock::estimate_knock_risk_with_physics,
    losses::{
        brake_torque_from_indicated, fmep_bar_x100, fmep_pa, pmep_bar_x100, pmep_pa,
        torque_from_mep_nm_x100,
    },
    pressure::bmep_bar_x100,
    sensors::SensorSnapshot,
    state::PlantState,
    telemetry::is_misfire,
    types::*,
};

pub(super) fn fill_knock<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
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

pub(super) fn fill_dyno<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
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
    let sweep = advance_dyno_sweep(config, state, torque);

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
        sweep_point: sweep.point,
        sweep_complete: sweep.complete,
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct DynoSweepStep {
    point: DynoSweepPoint,
    complete: bool,
}

fn advance_dyno_sweep<const CYL: usize>(
    config: &PlantConfig<CYL>,
    state: &mut PlantState<CYL>,
    torque: TorqueNmX100,
) -> DynoSweepStep {
    if config.dyno.mode != DynoMode::TargetRpmSweep {
        return DynoSweepStep {
            point: DynoSweepPoint::empty(),
            complete: false,
        };
    }
    if state.dyno_current_target_rpm.0 == 0 {
        state.dyno_current_target_rpm = config.dyno.sweep_start_rpm;
    }
    if state.cycle.completed_cycle_count == state.dyno_last_completed_cycle_count {
        return DynoSweepStep {
            point: DynoSweepPoint::empty(),
            complete: false,
        };
    }

    state.dyno_last_completed_cycle_count = state.cycle.completed_cycle_count;
    let error = state.rpm.0.abs_diff(state.dyno_current_target_rpm.0);
    if error > config.dyno.rpm_error_limit.0 {
        reset_dyno_sample_window(state);
        return DynoSweepStep {
            point: DynoSweepPoint::empty(),
            complete: false,
        };
    }
    if state.dyno_hold_cycle_count < config.dyno.hold_cycles_before_sample {
        state.dyno_hold_cycle_count = state.dyno_hold_cycle_count.saturating_add(1);
        return DynoSweepStep {
            point: DynoSweepPoint::empty(),
            complete: false,
        };
    }

    state.dyno_sample_cycle_count = state.dyno_sample_cycle_count.saturating_add(1);
    state.dyno_sample_torque_sum = state.dyno_sample_torque_sum.saturating_add(torque.0 as i64);
    if state.dyno_sample_cycle_count < config.dyno.sample_cycles.max(1) {
        return DynoSweepStep {
            point: DynoSweepPoint::empty(),
            complete: false,
        };
    }

    let sample_count = state.dyno_sample_cycle_count.max(1) as i64;
    let avg_torque = TorqueNmX100((state.dyno_sample_torque_sum / sample_count) as i32);
    let point = DynoSweepPoint {
        valid: true,
        target_rpm: state.dyno_current_target_rpm,
        measured_rpm: state.rpm,
        torque_nm_x100: avg_torque,
        horsepower_x100: horsepower_x100(avg_torque, state.rpm),
        bmep_bar_x100: bmep_bar_x100(avg_torque, config.engine.displacement_cc),
        ve_x1000: lookup_ve_x1000(&config.air.ve_table, state.rpm, state.map_kpa10),
        map_kpa10: state.map_kpa10,
        lambda_x1000: state.last_lambda_x1000,
        spark_advance_deg10: state.cylinders[0].last_spark_advance_deg10,
        dyno_load_torque_nm_x100: state.last_dyno_load_torque_nm_x100,
    };
    let next = state
        .dyno_current_target_rpm
        .0
        .saturating_add(config.dyno.sweep_step_rpm.0);
    let complete = config.dyno.sweep_step_rpm.0 == 0 || next > config.dyno.sweep_end_rpm.0;
    if !complete {
        state.dyno_current_target_rpm = Rpm(next);
    }
    reset_dyno_sample_window(state);

    DynoSweepStep { point, complete }
}

fn reset_dyno_sample_window<const CYL: usize>(state: &mut PlantState<CYL>) {
    state.dyno_hold_cycle_count = 0;
    state.dyno_sample_cycle_count = 0;
    state.dyno_sample_torque_sum = 0;
}

pub(super) fn fill_sensors<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
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

pub(super) fn fill_telemetry<const CYL: usize, const MAX_EDGES: usize, const MAX_EVENTS: usize>(
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

#[cfg(test)]
mod tests {
    use super::*;

    fn sweep_config() -> PlantConfig<4> {
        let mut config = PlantConfig::<4>::default_four();
        config.dyno.mode = DynoMode::TargetRpmSweep;
        config.dyno.sweep_start_rpm = Rpm(2000);
        config.dyno.sweep_end_rpm = Rpm(2200);
        config.dyno.sweep_step_rpm = Rpm(100);
        config.dyno.hold_cycles_before_sample = 1;
        config.dyno.sample_cycles = 2;
        config.dyno.rpm_error_limit = Rpm(25);
        config
    }

    #[test]
    fn dyno_sweep_advances_through_hold_sample_and_next_target() {
        let config = sweep_config();
        let mut state = PlantState::<4>::new();
        state.rpm = Rpm(2000);
        state.map_kpa10 = Kpa10(900);
        state.cycle.completed_cycle_count = 1;

        let hold = advance_dyno_sweep(&config, &mut state, TorqueNmX100(1000));
        assert!(!hold.point.valid);
        assert_eq!(state.dyno_current_target_rpm, Rpm(2000));
        assert_eq!(state.dyno_hold_cycle_count, 1);

        state.cycle.completed_cycle_count = 2;
        let sample_one = advance_dyno_sweep(&config, &mut state, TorqueNmX100(1200));
        assert!(!sample_one.point.valid);
        assert_eq!(state.dyno_sample_cycle_count, 1);
        assert_eq!(state.dyno_sample_torque_sum, 1200);

        state.cycle.completed_cycle_count = 3;
        let sample_two = advance_dyno_sweep(&config, &mut state, TorqueNmX100(1400));
        assert!(sample_two.point.valid);
        assert!(!sample_two.complete);
        assert_eq!(sample_two.point.target_rpm, Rpm(2000));
        assert_eq!(sample_two.point.torque_nm_x100, TorqueNmX100(1300));
        assert_eq!(state.dyno_current_target_rpm, Rpm(2100));
        assert_eq!(state.dyno_hold_cycle_count, 0);
        assert_eq!(state.dyno_sample_cycle_count, 0);
        assert_eq!(state.dyno_sample_torque_sum, 0);
    }

    #[test]
    fn dyno_sweep_resets_window_when_rpm_is_outside_error_limit() {
        let config = sweep_config();
        let mut state = PlantState::<4>::new();
        state.rpm = Rpm(2500);
        state.dyno_current_target_rpm = Rpm(2000);
        state.dyno_hold_cycle_count = 1;
        state.dyno_sample_cycle_count = 1;
        state.dyno_sample_torque_sum = 1000;
        state.cycle.completed_cycle_count = 1;

        let step = advance_dyno_sweep(&config, &mut state, TorqueNmX100(1000));

        assert!(!step.point.valid);
        assert!(!step.complete);
        assert_eq!(state.dyno_hold_cycle_count, 0);
        assert_eq!(state.dyno_sample_cycle_count, 0);
        assert_eq!(state.dyno_sample_torque_sum, 0);
    }

    #[test]
    fn dyno_sweep_reports_complete_at_last_target() {
        let config = sweep_config();
        let mut state = PlantState::<4>::new();
        state.rpm = Rpm(2200);
        state.dyno_current_target_rpm = Rpm(2200);
        state.dyno_hold_cycle_count = config.dyno.hold_cycles_before_sample;
        state.dyno_sample_cycle_count = 1;
        state.dyno_sample_torque_sum = 1500;
        state.cycle.completed_cycle_count = 1;

        let step = advance_dyno_sweep(&config, &mut state, TorqueNmX100(1700));

        assert!(step.point.valid);
        assert!(step.complete);
        assert_eq!(step.point.target_rpm, Rpm(2200));
        assert_eq!(step.point.torque_nm_x100, TorqueNmX100(1600));
        assert_eq!(state.dyno_current_target_rpm, Rpm(2200));
    }
}
