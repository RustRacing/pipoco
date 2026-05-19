use crate::fuel::{
    compute_afr_corr_x1000, compute_pw_air_us, compute_pw_base_us, compute_pw_corr_us,
    lookup_afterstart_corr_x1000, lookup_baro_corr_x1000, lookup_clt_corr_x1000,
    lookup_cranking_corr_x1000, lookup_deadtime_us, lookup_iat_corr_x1000, lookup_target_afr,
    lookup_vbat_corr_x1000, lookup_ve, lookup_warmup_corr_x1000, FuelParts,
};
use crate::schedule::{schedule_all_cylinders_with_advance_trim, FuelOutput, ScheduleOutput};
use crate::{
    ae_step, arbiter_step, compute_spark_advance_deg10, dfco_step, flat_shift_step, idle_step,
    idle_timing_step_with_base, knock_step, lambda_step, launch_step, mode_limiter_ceiling_x1000,
    rev_limit_step, safety_step, torque_pipeline_step, ArbiterInputs, ArbiterResult, DfcoResult,
    DiagnosticCode, FlatShiftResult, IdleResult, IdleTimingResult, InputSnapshot, KnockResult,
    LambdaResult, LaunchResult, LogicalState, ObservableOutput, RevLimitResult,
    SensorPlausibilityInput, SensorPlausibilityState, SensorSlewInput, SensorSlewState,
    SignedDegrees10, StepResult, SyncState, TorquePipelineInputs, ValidatedCalibration,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TableEvaluation {
    pub ve: crate::VePctX100,
    pub target_afr: crate::AfrX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FuelEvaluation {
    pub pw_base_us: crate::PulseWidthUs,
    pub pw_air_us: crate::PulseWidthUs,
    pub pw_corr_us: crate::PulseWidthUs,
    pub ae_next_state: crate::AeState,
    pub lambda: LambdaResult,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct UpdateContext {
    pub ae_next_state: crate::AeState,
    pub idle: IdleResult,
    pub idle_timing: IdleTimingResult,
    pub lambda: LambdaResult,
    pub dfco: DfcoResult,
    pub rev_limit: RevLimitResult,
    pub launch: LaunchResult,
    pub flat_shift: FlatShiftResult,
    pub knock: KnockResult,
    pub safety: crate::SafetyResult,
    pub arbiter: ArbiterResult,
    pub sensor_plausibility_state: SensorPlausibilityState,
    pub sensor_slew_state: SensorSlewState,
    pub diagnostic: DiagnosticCode,
}

pub fn evaluate_tables(cal: &ValidatedCalibration, input: InputSnapshot) -> TableEvaluation {
    TableEvaluation {
        ve: lookup_ve(cal, input),
        target_afr: lookup_target_afr(cal, input),
    }
}

fn evaluate_fuel_details(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
    tables: TableEvaluation,
) -> FuelEvaluation {
    let pw_base_us = compute_pw_base_us(cal, tables.ve);
    let pw_air_us = compute_pw_air_us(cal, pw_base_us, input.map_kpa10);
    let ae_result = ae_step(cal, input, state);
    let lambda = lambda_step(cal, input, state, ae_result.next_state.active);
    let parts = FuelParts {
        pw_air_us,
        ae_pulse_us: ae_result.ae_pulse_us,
        deadtime_us: lookup_deadtime_us(cal, input),
        clt_corr_x1000: lookup_clt_corr_x1000(cal, input),
        iat_corr_x1000: lookup_iat_corr_x1000(cal, input),
        baro_corr_x1000: lookup_baro_corr_x1000(cal, input),
        vbat_corr_x1000: lookup_vbat_corr_x1000(cal, input),
        cranking_corr_x1000: lookup_cranking_corr_x1000(cal, input),
        afterstart_corr_x1000: lookup_afterstart_corr_x1000(cal, state, input),
        warmup_corr_x1000: lookup_warmup_corr_x1000(cal, input),
        afr_corr_x1000: compute_afr_corr_x1000(cal, tables.target_afr),
        lambda_corr_x1000: crate::RatioX1000(lambda.correction_x1000),
    };
    let pw_corr_us = compute_pw_corr_us(cal, state, input, parts);
    FuelEvaluation {
        pw_base_us,
        pw_air_us,
        pw_corr_us,
        ae_next_state: ae_result.next_state,
        lambda,
    }
}

pub fn evaluate_fuel(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
    tables: TableEvaluation,
) -> FuelOutput {
    let detail = evaluate_fuel_details(cal, input, state, tables);
    FuelOutput {
        pw_corr_us: detail.pw_corr_us,
    }
}

pub fn evaluate_schedule(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
) -> ScheduleOutput {
    evaluate_schedule_with_advance_trim(cal, input, fuel, SignedDegrees10(0))
}

pub fn evaluate_schedule_with_advance_trim(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    fuel: FuelOutput,
    advance_trim_deg10: SignedDegrees10,
) -> ScheduleOutput {
    schedule_all_cylinders_with_advance_trim(cal, input, fuel, advance_trim_deg10)
}

pub fn update_state(
    input: InputSnapshot,
    previous: &LogicalState,
    schedule: &ScheduleOutput,
    context: UpdateContext,
) -> LogicalState {
    let mut next_state = *previous;
    next_state.math.last_valid_map_kpa10 = input.map_kpa10;
    next_state.math.last_valid_load_kpa10 = input.load_kpa10;
    next_state.math.last_valid_clt_c10 = input.clt_c10;
    next_state.math.last_valid_iat_c10 = input.iat_c10;
    next_state.math.last_valid_baro_kpa10 = input.baro_kpa10;
    next_state.scheduler.pending = schedule.events;
    if engine_enabled(input) {
        next_state.scheduler.last_cycle_epoch =
            next_state.scheduler.last_cycle_epoch.saturating_add(1);
    }
    next_state.ae = context.ae_next_state;
    next_state.idle_integrator_state = context.idle.integrator_state;
    next_state.idle_timing_integrator_state = context.idle_timing.integrator_state;
    next_state.idle_duty_x1000 = context.idle.duty_x1000;
    next_state.lambda_integrator_state = context.lambda.integrator_state;
    next_state.lambda_correction_x1000 = context.lambda.correction_x1000;
    next_state.dfco_active = context.dfco.dfco_active;
    next_state.dfco_qualify_counter = context.dfco.dfco_qualify_counter;
    next_state.rev_soft_active = context.rev_limit.soft_active;
    next_state.rev_hard_active = context.rev_limit.hard_active;
    next_state.launch_active = context.launch.active;
    next_state.launch_cut_cycle_count = context.launch.cut_cycle_count;
    next_state.flat_shift_active = context.flat_shift.active;
    next_state.flat_shift_cut_cycle_count = context.flat_shift.cut_cycle_count;
    next_state.knock_state = context.knock.next_state;
    next_state.safety_latched = context.safety.safety_latched;
    next_state.knock_intensity_x100 = input.knock_intensity_x100;
    next_state.sensor_plausibility_state = context.sensor_plausibility_state;
    next_state.sensor_slew_state = context.sensor_slew_state;

    let unsynced = input.sync != SyncState::Synced;
    next_state.diag.unsynced = unsynced;
    next_state.diag.fuel_cut_active = context.arbiter.fuel_cut;
    next_state.diag.spark_cut_active = context.arbiter.spark_cut;
    next_state.diag.current = context.diagnostic;
    next_state
}

fn engine_enabled(input: InputSnapshot) -> bool {
    matches!(
        input.mode,
        crate::EngineMode::Cranking | crate::EngineMode::Running
    )
}

pub fn step(cal: &ValidatedCalibration, input: InputSnapshot, state: &LogicalState) -> StepResult {
    let dfco = dfco_step(cal, input, state);
    let rev_limit = rev_limit_step(cal, input, state);
    let launch = launch_step(cal, input, state);
    let flat_shift = flat_shift_step(cal, input, state);
    let knock = knock_step(cal, input, state);
    let safety = safety_step(input, state);
    let arbiter = arbiter_step(ArbiterInputs {
        safety_latched: safety.safety_latched,
        dfco_cut: dfco.fuel_cut,
        rev_limit,
        launch_cut: launch.launch_cut,
        flat_shift,
        knock,
    });
    let mut arb_input = input;
    arb_input.fuel_cut = arbiter.fuel_cut;
    arb_input.spark_cut = arbiter.spark_cut;
    let slew = crate::sensor_slew_step(
        SensorSlewInput {
            t_us: arb_input.t_us,
            clt_c10: arb_input.clt_c10,
            iat_c10: arb_input.iat_c10,
            map_kpa10: arb_input.map_kpa10.get(),
            tps_x100: arb_input.tps_x100,
            maf_x100: if state.sensor_slew_state.initialized {
                state.sensor_slew_state.maf_x100
            } else {
                0
            },
            o2_afr_x100: if state.sensor_slew_state.initialized {
                state.sensor_slew_state.o2_afr_x100
            } else {
                1470
            },
            knock_intensity_x100: arb_input.knock_intensity_x100,
            baro_kpa10: arb_input.baro_kpa10.get(),
            vbat_mv: arb_input.vbatt_mv.get(),
        },
        state.sensor_slew_state,
    );
    arb_input.clt_c10 = slew.limited.clt_c10;
    arb_input.iat_c10 = slew.limited.iat_c10;
    arb_input.map_kpa10 = crate::Kpa10::new(slew.limited.map_kpa10);
    arb_input.tps_x100 = slew.limited.tps_x100;
    arb_input.knock_intensity_x100 = slew.limited.knock_intensity_x100;
    arb_input.baro_kpa10 = crate::Kpa10::new(slew.limited.baro_kpa10);
    arb_input.vbatt_mv = crate::Millivolts::new(slew.limited.vbat_mv);

    let tables = evaluate_tables(cal, arb_input);
    let fuel = evaluate_fuel_details(cal, arb_input, state, tables);
    let idle = idle_step(cal, arb_input, state);
    let base_advance = compute_spark_advance_deg10(cal, arb_input);
    let idle_timing = idle_timing_step_with_base(cal, arb_input, state, base_advance);
    let advance_trim_deg10 = rev_limit
        .soft_retard_deg10
        .saturating_add(knock.advance_trim_deg10)
        .saturating_add(idle_timing.trim_deg10);
    let schedule = evaluate_schedule_with_advance_trim(
        cal,
        arb_input,
        FuelOutput {
            pw_corr_us: fuel.pw_corr_us,
        },
        SignedDegrees10(advance_trim_deg10),
    );
    let plausibility = crate::sensor_plausibility_step(
        SensorPlausibilityInput {
            t_us: arb_input.t_us,
            rpm: arb_input.rpm,
            clt_c10: arb_input.clt_c10,
            iat_c10: arb_input.iat_c10,
            map_kpa10: arb_input.map_kpa10.get(),
            tps_x100: arb_input.tps_x100,
            // MAF/O2 live sensor channels are added in later stories; keep
            // them in-range here so plausibility only trips on represented inputs.
            maf_x100: 0,
            o2_afr_x100: tables.target_afr.get(),
            knock_intensity_x100: arb_input.knock_intensity_x100,
            baro_kpa10: arb_input.baro_kpa10.get(),
            // Voltage diagnostics must see the raw measurement. Slew/table
            // limiting may create an effective voltage for math, but must not
            // hide charging-system failures such as a running 10V event.
            vbat_mv: input.vbatt_mv.get(),
        },
        state.sensor_plausibility_state,
    );
    let diagnostic = match schedule.diagnostic {
        DiagnosticCode::CalibrationInvalid => DiagnosticCode::CalibrationInvalid,
        _ if arbiter.fuel_cut => DiagnosticCode::FuelCutActive,
        _ if arbiter.spark_cut => DiagnosticCode::SparkCutActive,
        _ if plausibility.diagnostic == DiagnosticCode::SensorPlausibilityFault => {
            DiagnosticCode::SensorPlausibilityFault
        }
        _ if slew.diagnostic == DiagnosticCode::SensorPlausibilityFault => {
            DiagnosticCode::SensorPlausibilityFault
        }
        _ if engine_enabled(arb_input) && arb_input.sync != SyncState::Synced => {
            DiagnosticCode::Unsynced
        }
        _ => DiagnosticCode::None,
    };
    let next_state = update_state(
        arb_input,
        state,
        &schedule,
        UpdateContext {
            ae_next_state: fuel.ae_next_state,
            idle,
            idle_timing,
            lambda: fuel.lambda,
            dfco,
            rev_limit,
            launch,
            flat_shift,
            knock,
            safety,
            arbiter,
            sensor_plausibility_state: plausibility.next_state,
            sensor_slew_state: slew.next_state,
            diagnostic,
        },
    );
    let torque = torque_pipeline_step(TorquePipelineInputs {
        request_x1000: crate::derive_torque_request_x1000(arb_input),
        limiter_ceiling_x1000: mode_limiter_ceiling_x1000(arb_input.mode),
        safety_latched: safety.safety_latched,
        fuel_cut: arbiter.fuel_cut,
        spark_cut: arbiter.spark_cut,
        rev_hard_active: rev_limit.hard_active,
        launch_cut: launch.launch_cut,
        flat_shift_cut: flat_shift.flat_shift_cut,
    });
    let output = ObservableOutput {
        ve_pct_x100: tables.ve,
        target_afr_x100: tables.target_afr,
        pw_base_us: fuel.pw_base_us,
        pw_air_us: fuel.pw_air_us,
        pw_corr_us: fuel.pw_corr_us,
        lambda_correction_x1000: fuel.lambda.correction_x1000,
        idle_duty_x1000: idle.duty_x1000,
        torque_request_x1000: torque.torque_request_x1000,
        torque_allowed_x1000: torque.torque_allowed_x1000,
        torque_actuated_x1000: torque.torque_actuated_x1000,
        cut_reason_code: arbiter.cut_reason_code,
        fuel_cut: arbiter.fuel_cut,
        spark_cut: arbiter.spark_cut,
        advance_deg10_trim: advance_trim_deg10,
        knock_intensity_x100: input.knock_intensity_x100,
        spark_advance_deg10: schedule.spark_advance_deg10,
        dwell_us: schedule.dwell_us,
        soi_deg10: schedule.soi_deg10,
        eoi_deg10: schedule.eoi_deg10,
        spark_deg10: schedule.spark_deg10,
        dwell_start_deg10: schedule.dwell_start_deg10,
        events: schedule.events,
        diagnostic: next_state.diag.current,
    };
    StepResult { next_state, output }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        default_reference_calibration, AfrOverride, EngineMode, Kpa10, Micros, Millivolts, Rpm,
        SyncState,
    };

    #[test]
    fn oracle_step_matches_canonical_fixture() {
        let cal = default_reference_calibration();
        let input = InputSnapshot {
            t_us: Micros(0),
            rpm: Rpm(1000),
            map_kpa10: Kpa10(1000),
            load_kpa10: Kpa10(1000),
            tps_x100: 0,
            clt_c10: crate::TempC10(800),
            iat_c10: crate::TempC10(250),
            baro_kpa10: Kpa10(1000),
            vbatt_mv: Millivolts(12_000),
            knock_intensity_x100: 0,
            launch_armed: false,
            flat_shift_armed: false,
            sync: SyncState::Synced,
            fuel_cut: false,
            spark_cut: false,
            mode: EngineMode::Running,
            target_afr_override_x100: AfrOverride::None,
        };
        let result = step(&cal, input, &LogicalState::default());
        assert_eq!(result.output.ve_pct_x100.0, 8000);
        assert_eq!(result.output.target_afr_x100.0, 1470);
        assert_eq!(result.output.pw_base_us.0, 2400);
        assert_eq!(result.output.pw_air_us.0, 2400);
        assert_eq!(result.output.pw_corr_us.0, 3200);
        assert_eq!(result.output.dwell_us.0, 2500);
        assert_eq!(result.output.events.len, 16);
        assert_eq!(result.output.soi_deg10.values[0], 6648);
        assert_eq!(result.output.eoi_deg10.values[0], 6840);
        assert_eq!(result.output.spark_deg10.values[0], 7050);
        assert_eq!(result.output.dwell_start_deg10.values[0], 6900);
        assert_eq!(result.output.soi_deg10.values[1], 1248);
        assert_eq!(result.output.eoi_deg10.values[1], 1440);
        assert_eq!(result.output.spark_deg10.values[1], 1650);
        assert_eq!(result.output.dwell_start_deg10.values[1], 1500);
        assert_eq!(result.output.soi_deg10.values[2], 3048);
        assert_eq!(result.output.eoi_deg10.values[2], 3240);
        assert_eq!(result.output.spark_deg10.values[2], 3450);
        assert_eq!(result.output.dwell_start_deg10.values[2], 3300);
        assert_eq!(result.output.soi_deg10.values[3], 4848);
        assert_eq!(result.output.eoi_deg10.values[3], 5040);
        assert_eq!(result.output.spark_deg10.values[3], 5250);
        assert_eq!(result.output.dwell_start_deg10.values[3], 5100);
        assert_eq!(result.output.diagnostic, DiagnosticCode::None);
        assert_eq!(result.output.cut_reason_code, 0);
        assert!(!result.output.fuel_cut);
        assert!(!result.output.spark_cut);
        assert_eq!(result.next_state.diag.current, DiagnosticCode::None);
        assert_eq!(result.next_state.scheduler.pending.len, 16);
    }

    #[test]
    fn oracle_updates_diagnostics_and_state() {
        let cal = default_reference_calibration();
        let input = InputSnapshot {
            sync: SyncState::Unsynced,
            fuel_cut: true,
            spark_cut: true,
            ..InputSnapshot::default()
        };
        let result = step(&cal, input, &LogicalState::default());
        assert!(result.next_state.diag.unsynced);
        assert!(result.next_state.diag.fuel_cut_active);
        assert!(result.next_state.diag.spark_cut_active);
        assert_eq!(result.output.cut_reason_code, 1);
        assert!(result.output.fuel_cut);
        assert!(result.output.spark_cut);
        assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
        assert_eq!(
            result.next_state.diag.current,
            DiagnosticCode::FuelCutActive
        );
        assert_eq!(result.output.events.len, 0);
    }

    #[test]
    fn oracle_shutdown_feeds_safety_arbiter_and_latches() {
        let cal = default_reference_calibration();
        let input = InputSnapshot {
            mode: EngineMode::Shutdown,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };
        let result = step(&cal, input, &LogicalState::default());
        assert_eq!(result.output.cut_reason_code, 1);
        assert!(result.output.fuel_cut);
        assert!(result.output.spark_cut);
        assert!(result.next_state.safety_latched);
    }

    #[test]
    fn oracle_soft_rev_cut_suppresses_only_spark_events() {
        let mut cal = default_reference_calibration();
        cal.0.soft_rev_rpm = Rpm(1500);
        cal.0.hard_rev_rpm = Rpm(2000);
        cal.0.rev_hysteresis_rpm = Rpm(100);
        let input = InputSnapshot {
            rpm: Rpm(1600),
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };

        let result = step(&cal, input, &LogicalState::default());
        assert_eq!(result.output.cut_reason_code, 6);
        assert_eq!(result.output.diagnostic, DiagnosticCode::SparkCutActive);
        assert!(!result.output.fuel_cut);
        assert!(result.output.spark_cut);
        assert!(result.next_state.diag.spark_cut_active);
        assert!(!result.next_state.diag.fuel_cut_active);
    }

    #[test]
    fn oracle_hard_rev_cut_suppresses_all_events() {
        let mut cal = default_reference_calibration();
        cal.0.soft_rev_rpm = Rpm(1500);
        cal.0.hard_rev_rpm = Rpm(2000);
        cal.0.rev_hysteresis_rpm = Rpm(100);
        let input = InputSnapshot {
            rpm: Rpm(2100),
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            ..InputSnapshot::default()
        };

        let result = step(&cal, input, &LogicalState::default());
        assert_eq!(result.output.cut_reason_code, 2);
        assert_eq!(result.output.diagnostic, DiagnosticCode::FuelCutActive);
        assert!(result.output.fuel_cut);
        assert!(result.output.spark_cut);
        assert!(result.next_state.diag.fuel_cut_active);
        assert!(result.next_state.diag.spark_cut_active);
        assert_eq!(result.output.events.len, 0);
    }

    #[test]
    fn oracle_sensor_plausibility_fault_reports_after_debounce() {
        let cal = default_reference_calibration();
        let input0 = InputSnapshot {
            t_us: Micros(0),
            rpm: Rpm(2000),
            map_kpa10: Kpa10(1000),
            sync: SyncState::Synced,
            mode: EngineMode::Running,
            ..InputSnapshot::default()
        };
        let step0 = step(&cal, input0, &LogicalState::default());
        assert_eq!(step0.output.diagnostic, DiagnosticCode::None);

        let input1 = InputSnapshot {
            t_us: Micros(500_000),
            rpm: Rpm(2000),
            map_kpa10: Kpa10(1000),
            sync: SyncState::Synced,
            mode: EngineMode::Running,
            ..InputSnapshot::default()
        };
        let step1 = step(&cal, input1, &step0.next_state);
        assert_eq!(
            step1.output.diagnostic,
            DiagnosticCode::SensorPlausibilityFault
        );
    }
}
