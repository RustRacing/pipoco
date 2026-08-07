use super::*;
use crate::enrichment::StartupState;
use crate::ignition::IgnitionLimitReason;
use crate::lambda::LambdaMode;
use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm};

fn test_model() -> BaseFuelModel {
    let rpm_bins = [
        Rpm::new(500),
        Rpm::new(1000),
        Rpm::new(1500),
        Rpm::new(2000),
        Rpm::new(2500),
        Rpm::new(3000),
        Rpm::new(3500),
        Rpm::new(4000),
        Rpm::new(4500),
        Rpm::new(5000),
        Rpm::new(5500),
        Rpm::new(6000),
        Rpm::new(6500),
        Rpm::new(7000),
        Rpm::new(7500),
        Rpm::new(8000),
    ];
    let load_bins = [
        Kpa10::new(200),
        Kpa10::new(300),
        Kpa10::new(400),
        Kpa10::new(500),
        Kpa10::new(600),
        Kpa10::new(700),
        Kpa10::new(800),
        Kpa10::new(900),
        Kpa10::new(1000),
        Kpa10::new(1100),
        Kpa10::new(1200),
        Kpa10::new(1300),
        Kpa10::new(1400),
        Kpa10::new(1500),
        Kpa10::new(1600),
        Kpa10::new(1700),
    ];
    let mut pulse_widths = [[PulseWidthUs::new(0); 16]; 16];
    pulse_widths[0][0] = PulseWidthUs::new(1000);
    pulse_widths[5][5] = PulseWidthUs::new(2500);
    pulse_widths[15][15] = PulseWidthUs::new(4000);
    BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
}

#[test]
fn base_fuel_uses_nearest_lower_bin() {
    let model = test_model();

    assert_eq!(
        model
            .calculate_base_fuel(Rpm::new(750), Kpa10::new(250))
            .get(),
        1000
    );
}

#[test]
fn base_fuel_selects_matching_mid_table_cell() {
    let model = test_model();

    assert_eq!(
        model
            .calculate_base_fuel(Rpm::new(3000), Kpa10::new(700))
            .get(),
        2500
    );
}

#[test]
fn base_fuel_clamps_to_upper_edge() {
    let model = test_model();

    assert_eq!(
        model
            .calculate_base_fuel(Rpm::new(9000), Kpa10::new(1900))
            .get(),
        4000
    );
}

#[test]
fn model_accessors_round_trip() {
    let model = test_model();

    assert_eq!(model.rpm_bins()[0].get(), 500);
    assert_eq!(model.load_bins()[0].get(), 200);
    assert_eq!(model.pulse_widths()[5][5].get(), 2500);
}

#[test]
fn startup_tapers_after_cranking() {
    let cfg = StartupConfig::DEFAULT;
    let mut st = StartupState::new();

    assert_eq!(st.update(Micros::new(0), true, &cfg), cfg.percent_x100);
    let halfway = st.update(Micros::new(cfg.taper_time_ms * 500), false, &cfg);
    assert!(halfway > 100 && halfway < cfg.percent_x100);
    assert_eq!(
        st.update(Micros::new(cfg.taper_time_ms * 1000 + 1), false, &cfg),
        100
    );
}

#[test]
fn warmup_interpolates_by_coolant_temp() {
    let cfg = WarmupConfig::DEFAULT;
    assert_eq!(cfg.compute_percent_x100(-30), cfg.max_percent_x100);
    assert_eq!(cfg.compute_percent_x100(60), cfg.min_percent_x100);
    let mid = cfg.compute_percent_x100(20);
    assert!(mid > cfg.min_percent_x100 && mid < cfg.max_percent_x100);
}

#[test]
fn warmup_respects_configured_percent_bounds() {
    let cfg = WarmupConfig {
        start_c: 0,
        end_c: 100,
        max_percent_x100: 260,
        min_percent_x100: 80,
    };

    assert_eq!(cfg.compute_percent_x100(-1), 260);
    assert_eq!(cfg.compute_percent_x100(100), 80);
    assert_eq!(cfg.compute_percent_x100(50), 170);
}

#[test]
fn warmup_reversed_temperature_range_returns_bounded_neutral() {
    let neutral_in_range = WarmupConfig {
        start_c: 80,
        end_c: 20,
        max_percent_x100: 160,
        min_percent_x100: 80,
    };
    let neutral_below_range = WarmupConfig {
        start_c: 80,
        end_c: 20,
        max_percent_x100: 90,
        min_percent_x100: 70,
    };
    let neutral_above_range = WarmupConfig {
        start_c: 80,
        end_c: 20,
        max_percent_x100: 140,
        min_percent_x100: 120,
    };

    assert_eq!(neutral_in_range.compute_percent_x100(40), 100);
    assert_eq!(neutral_below_range.compute_percent_x100(40), 90);
    assert_eq!(neutral_above_range.compute_percent_x100(40), 120);
}

#[test]
fn warmup_handles_reversed_percent_bounds_without_panicking() {
    let cfg = WarmupConfig {
        start_c: 0,
        end_c: 100,
        max_percent_x100: 80,
        min_percent_x100: 160,
    };

    assert_eq!(cfg.compute_percent_x100(-1), 80);
    assert_eq!(cfg.compute_percent_x100(100), 160);
    assert_eq!(cfg.compute_percent_x100(50), 120);
}

#[test]
fn after_start_triggers_and_decays_with_lockout() {
    let cfg = AfterStartConfig::DEFAULT;
    let mut st = AfterStartState::new();

    assert_eq!(st.update(Micros::new(0), true, &cfg), cfg.percent_x100);
    let halfway = st.update(Micros::new(cfg.taper_time_ms * 500), false, &cfg);
    assert!(halfway > 100 && halfway < cfg.percent_x100);
    assert_eq!(
        st.update(Micros::new(cfg.taper_time_ms * 1000 + 1), false, &cfg),
        100
    );
}

#[test]
fn acceleration_enrichment_triggers_and_decays() {
    let cfg = AccelerationConfig::DEFAULT;
    let mut st = AccelerationState::new();

    assert_eq!(st.update(Micros::new(0), 200, 0, &cfg), cfg.percent_x100);
    let halfway = st.update(Micros::new(cfg.decay_time_ms * 500), 0, 0, &cfg);
    assert!(halfway > 100 && halfway < cfg.percent_x100);
    assert_eq!(
        st.update(Micros::new(cfg.decay_time_ms * 1000 + 1), 0, 0, &cfg),
        100
    );
}

#[test]
fn enrichment_controller_returns_combined_result() {
    let mut ctrl = EnrichmentController::new();
    let result = ctrl.update(
        EnrichmentInputs {
            now_us: Micros::new(0),
            clt_c: -20,
            cranking: true,
            just_started: true,
            tpsdot_pct_s: 200,
            mapdot_kpa_s: 0,
        },
        &StartupConfig::DEFAULT,
        &WarmupConfig::DEFAULT,
        &AfterStartConfig::DEFAULT,
        &AccelerationConfig::DEFAULT,
    );

    assert_eq!(result.startup_x100, StartupConfig::DEFAULT.percent_x100);
    assert_eq!(result.warmup_x100, WarmupConfig::DEFAULT.max_percent_x100);
    assert_eq!(
        result.total_x100(),
        result.apply_to(PulseWidthUs::new(100)).get() as u16
    );
}

#[test]
fn lambda_planner_stays_open_loop_when_disabled() {
    let mut planner = LambdaTrimPlanner::new();
    let result = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 20,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(105),
            requested_open_loop: true,
        },
        &LambdaTrimConfig::DEFAULT,
        Kpa10::new(500),
        false,
        false,
    );

    assert_eq!(result.mode, LambdaMode::OpenLoop);
    assert!(!result.active);
    assert_eq!(result.trim_x100, 100);
    assert_eq!(
        result.disable_reason,
        LambdaDisableReason::RequestedOpenLoop
    );
    assert_eq!(
        result.target_lambda100,
        LambdaTrimConfig::DEFAULT.open_loop_target
    );
}

#[test]
fn lambda_planner_enters_closed_loop_and_clamps_trim() {
    let mut planner = LambdaTrimPlanner::new();
    let cfg = LambdaTrimConfig::DEFAULT;

    let result = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(90),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        false,
        false,
    );

    assert_eq!(result.mode, LambdaMode::ClosedLoop);
    assert!(result.active);
    assert_eq!(result.disable_reason, LambdaDisableReason::None);
    assert_eq!(result.target_lambda100, cfg.closed_loop_target);
    assert!(result.trim_x100 >= cfg.min_trim_x100);
    assert!(result.trim_x100 <= cfg.max_trim_x100);
}

#[test]
fn lambda_planner_holds_open_loop_during_startup_delay() {
    let mut planner = LambdaTrimPlanner::new();
    let cfg = LambdaTrimConfig {
        startup_delay_us: Micros::new(2_000_000),
        ..LambdaTrimConfig::DEFAULT
    };

    let delayed = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(1_000_000),
            clt_c: 80,
            just_started: true,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(98),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        false,
        false,
    );

    assert_eq!(delayed.mode, LambdaMode::OpenLoop);
    assert!(!delayed.active);
    assert_eq!(delayed.disable_reason, LambdaDisableReason::StartupDelay);

    let active = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(3_000_000),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(98),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        false,
        false,
    );

    assert_eq!(active.mode, LambdaMode::ClosedLoop);
    assert!(active.active);
    assert_eq!(active.disable_reason, LambdaDisableReason::None);
}

#[test]
fn torque_arbiter_prefers_requested_torque_when_unlimited() {
    let arbiter = TorqueArbiter::new();
    let result = arbiter.evaluate(TorqueInputs::new(90, 80, 120, 130, 140));

    assert_eq!(result.requested_x100, 90);
    assert_eq!(result.allowed_x100, 90);
    assert_eq!(result.allowed_x1000, 900);
    assert_eq!(result.reason, TorqueLimitReason::None);
}

#[test]
fn lambda_planner_marks_power_reduction_cut_as_frozen_closed_loop() {
    let mut planner = LambdaTrimPlanner::new();
    let cfg = LambdaTrimConfig::DEFAULT;

    let frozen = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(95),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        false,
        true,
    );

    assert_eq!(frozen.mode, LambdaMode::ClosedLoop);
    assert!(!frozen.active);
    assert_eq!(
        frozen.disable_reason,
        LambdaDisableReason::PowerReductionCut
    );
    assert_eq!(frozen.target_lambda100, cfg.closed_loop_target);
    assert!(frozen.trim_x100 >= cfg.min_trim_x100);
    assert!(frozen.trim_x100 <= cfg.max_trim_x100);
}

#[test]
fn lambda_planner_marks_acceleration_enrichment_as_frozen_closed_loop() {
    let mut planner = LambdaTrimPlanner::new();
    let cfg = LambdaTrimConfig::DEFAULT;

    let frozen = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(95),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        true,
        false,
    );

    assert_eq!(frozen.mode, LambdaMode::ClosedLoop);
    assert!(!frozen.active);
    assert_eq!(
        frozen.disable_reason,
        LambdaDisableReason::AccelerationEnrichment
    );
    assert_eq!(frozen.target_lambda100, cfg.closed_loop_target);
    assert!(frozen.trim_x100 >= cfg.min_trim_x100);
    assert!(frozen.trim_x100 <= cfg.max_trim_x100);
}

#[test]
fn lambda_planner_holds_last_trim_during_acceleration_enrichment_freeze() {
    let mut planner = LambdaTrimPlanner::new();
    let cfg = LambdaTrimConfig::DEFAULT;

    let active = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(90),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        false,
        false,
    );
    let frozen = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(10_000),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(50),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(500),
        true,
        false,
    );

    assert!(active.active);
    assert_eq!(
        frozen.disable_reason,
        LambdaDisableReason::AccelerationEnrichment
    );
    assert_eq!(frozen.trim_x100, active.trim_x100);
}

#[test]
fn lambda_planner_holds_open_loop_at_low_load_with_hysteresis() {
    let mut planner = LambdaTrimPlanner::new();
    let cfg = LambdaTrimConfig {
        enable_load_kpa10: Kpa10::new(300),
        disable_load_kpa10: Kpa10::new(250),
        ..LambdaTrimConfig::DEFAULT
    };

    let low = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(98),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(200),
        false,
        false,
    );

    assert_eq!(low.mode, LambdaMode::OpenLoop);
    assert!(!low.active);
    assert_eq!(low.disable_reason, LambdaDisableReason::LowLoadGate);

    let active = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(1),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(98),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(350),
        false,
        false,
    );

    assert_eq!(active.mode, LambdaMode::ClosedLoop);
    assert!(active.active);
    assert_eq!(active.disable_reason, LambdaDisableReason::None);

    let hysteresis = planner.update(
        LambdaTrimInputs {
            now_us: Micros::new(2),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(98),
            requested_open_loop: false,
        },
        &cfg,
        Kpa10::new(260),
        false,
        false,
    );

    assert_eq!(hysteresis.mode, LambdaMode::ClosedLoop);
    assert!(hysteresis.active);
    assert_eq!(hysteresis.disable_reason, LambdaDisableReason::None);
}

#[test]
fn torque_arbiter_applies_limp_cap_over_other_limits() {
    let arbiter = TorqueArbiter::new();
    let result = arbiter.evaluate(TorqueInputs::new(120, 100, 110, 105, 70));

    assert_eq!(result.requested_x100, 120);
    assert_eq!(result.allowed_x100, 70);
    assert_eq!(result.allowed_x1000, 700);
    assert_eq!(result.reason, TorqueLimitReason::LimpMode);
}

#[test]
fn torque_arbiter_uses_idle_request_when_higher() {
    let arbiter = TorqueArbiter::new();
    let result = arbiter.evaluate(TorqueInputs::new(60, 85, 120, 120, 120));

    assert_eq!(result.requested_x100, 85);
    assert_eq!(result.allowed_x100, 85);
    assert_eq!(result.allowed_x1000, 850);
    assert_eq!(result.reason, TorqueLimitReason::None);
}

#[test]
fn torque_arbiter_preserves_explicit_high_resolution_request() {
    let arbiter = TorqueArbiter::new();
    let result =
        arbiter.evaluate(TorqueInputs::new(53, 0, 120, 120, 120).with_driver_request_x1000(537));

    assert_eq!(result.requested_x100, 53);
    assert_eq!(result.requested_x1000, 537);
    assert_eq!(result.allowed_x100, 53);
    assert_eq!(result.allowed_x1000, 537);
    assert_eq!(result.reason, TorqueLimitReason::None);
}

#[test]
fn ignition_planner_combines_base_and_corrections() {
    let planner = IgnitionPlanner::new();
    let plan = planner.plan(
        IgnitionInputs::new(Degrees10::new(120), 10, 5, 0, false, Rpm::new(2500)),
        &DwellConfig::DEFAULT,
    );

    assert_eq!(plan.advance_deg10.get(), 125);
    assert_eq!(plan.limit_reason, IgnitionLimitReason::Knock);
    assert!(plan.dwell_us.get() >= DwellConfig::DEFAULT.min_dwell_us);
    assert!(plan.dwell_us.get() <= DwellConfig::DEFAULT.max_dwell_us);
}

#[test]
fn ignition_planner_applies_rev_limit_and_lower_dwell_at_high_rpm() {
    let planner = IgnitionPlanner::new();
    let low_rpm = planner.plan(
        IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, Rpm::new(2000)),
        &DwellConfig::DEFAULT,
    );
    let high_rpm = planner.plan(
        IgnitionInputs::new(Degrees10::new(120), 0, 0, 10, true, Rpm::new(7000)),
        &DwellConfig::DEFAULT,
    );

    assert_eq!(high_rpm.limit_reason, IgnitionLimitReason::RevLimiter);
    assert!(high_rpm.advance_deg10.get() < low_rpm.advance_deg10.get());
    assert!(high_rpm.dwell_us.get() <= low_rpm.dwell_us.get());
}

#[test]
fn control_planners_compose_purely() {
    let model = test_model();
    let base_pw = model.calculate_base_fuel(Rpm::new(3000), Kpa10::new(700));

    let mut enrichment = EnrichmentController::new();
    let enrich = enrichment.update(
        EnrichmentInputs {
            now_us: Micros::new(0),
            clt_c: 10,
            cranking: true,
            just_started: true,
            tpsdot_pct_s: 180,
            mapdot_kpa_s: 90,
        },
        &StartupConfig::DEFAULT,
        &WarmupConfig::DEFAULT,
        &AfterStartConfig::DEFAULT,
        &AccelerationConfig::DEFAULT,
    );

    let mut lambda = LambdaTrimPlanner::new();
    let lambda_result = lambda.update(
        LambdaTrimInputs {
            now_us: Micros::new(0),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(96),
            requested_open_loop: false,
        },
        &LambdaTrimConfig::DEFAULT,
        Kpa10::new(500),
        false,
        false,
    );

    let torque = TorqueArbiter::new().evaluate(TorqueInputs::new(92, 80, 120, 118, 110));
    let ignition = IgnitionPlanner::new().plan(
        IgnitionInputs::new(Degrees10::new(110), 8, 2, 4, false, Rpm::new(2800)),
        &DwellConfig::DEFAULT,
    );

    assert_eq!(base_pw.get(), 2500);
    assert!(enrich.total_x100() >= StartupConfig::DEFAULT.percent_x100);
    assert_eq!(lambda_result.mode, LambdaMode::ClosedLoop);
    assert_eq!(torque.allowed_x100, 92);
    assert_eq!(ignition.advance_deg10.get(), 112);
    assert!(ignition.dwell_us.get() >= DwellConfig::DEFAULT.min_dwell_us);
    assert!(ignition.dwell_us.get() <= DwellConfig::DEFAULT.max_dwell_us);
    assert_eq!(
        enrich.apply_to(base_pw).get(),
        (base_pw.get() as u32 * enrich.total_x100() as u32) / 100
    );
}

#[test]
fn decel_fuel_cut_engage_and_resume_parity() {
    use crate::DecelFuelCutState;
    use ecu_calibration::DfcoConfig;

    let cfg = DfcoConfig::DEFAULT;
    let mut st = DecelFuelCutState::new();
    assert!(!st.update(0, 2000, 0, 20, &cfg));
    assert!(st.update(cfg.delay_ms * 1000 + 1, 2000, 0, 20, &cfg));
    assert!(st.update(cfg.delay_ms * 1000 + 50_000, 2000, 10, 20, &cfg));
    assert!(!st.update(
        cfg.delay_ms * 1000 + cfg.resume_hyst_ms * 1000 + 2,
        2000,
        10,
        20,
        &cfg
    ));
}

#[test]
fn cranking_gate_hysteresis_parity() {
    use crate::{CrankingGate, CRANKING_EXIT_RPM, CRANKING_RPM_THRESHOLD};

    let mut gate = CrankingGate::new();
    assert!(!gate.is_cranking());

    assert!(gate.update(CRANKING_RPM_THRESHOLD - 1));
    assert!(gate.is_cranking());

    assert!(gate.update(CRANKING_EXIT_RPM - 1));
    assert!(gate.is_cranking());

    assert!(!gate.update(CRANKING_EXIT_RPM));
    assert!(!gate.is_cranking());

    assert!(!gate.update(CRANKING_RPM_THRESHOLD));
    assert!(!gate.is_cranking());
}
