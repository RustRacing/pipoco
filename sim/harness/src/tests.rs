use super::*;
use ecu_domain::{FaultCode, FaultSeverity, Lambda100, PulseWidthUs};
use ecu_runtime::{
    BaseFuelModel, EngineRuntime, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
    RuntimeFuelStrategy, RuntimeSemanticAxis16, RuntimeSemanticCalibration,
    RuntimeSemanticCurve16U16, RuntimeSemanticDeadtimeTableU16, RuntimeSemanticState,
    RuntimeSemanticTable2dU16, TorqueInputs, RUNTIME_SEMANTIC_TABLE_LEN,
};

fn control_inputs() -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(1_000),
            clt_c: 40,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 10,
            mapdot_kpa_s: 10,
        },
        lambda: LambdaTrimInputs {
            now_us: Micros::new(1_000),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(3000)),
        fuel_sensors: ecu_runtime::FuelSensorInputs::default(),
        knock_intensity_x100: 0,
    }
}

fn nonzero_fuel_model() -> BaseFuelModel {
    BaseFuelModel::new(
        [Rpm::new(1800); 16],
        [Kpa10::new(600); 16],
        [[PulseWidthUs::new(2500); 16]; 16],
    )
}

fn semantic_axis2() -> RuntimeSemanticAxis16 {
    RuntimeSemanticAxis16 {
        len: 2,
        values: [0, 2_000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    }
}

fn semantic_curve_u16(value: u16) -> RuntimeSemanticCurve16U16 {
    RuntimeSemanticCurve16U16 {
        axis: semantic_axis2(),
        values: [value; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_table_u16(value: u16) -> RuntimeSemanticTable2dU16 {
    RuntimeSemanticTable2dU16 {
        rpm_axis: semantic_axis2(),
        load_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_deadtime_table_u16(value: u16) -> RuntimeSemanticDeadtimeTableU16 {
    RuntimeSemanticDeadtimeTableU16 {
        vbat_mv_axis: semantic_axis2(),
        pressure_kpa10_axis: semantic_axis2(),
        values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
    }
}

fn semantic_shift_strategy(launch_rpm_limit: u16, flat_shift_rpm_min: u16) -> RuntimeFuelStrategy {
    RuntimeFuelStrategy::SpeedDensityVe {
        calibration: RuntimeSemanticCalibration {
            ve_table: semantic_table_u16(7_000),
            afr_target_table: semantic_table_u16(1_470),
            deadtime_table_us: semantic_deadtime_table_u16(0),
            clt_corr_curve: semantic_curve_u16(1_000),
            iat_corr_curve: semantic_curve_u16(1_000),
            baro_corr_curve: semantic_curve_u16(1_000),
            vbat_corr_curve: semantic_curve_u16(1_000),
            cranking_curve: semantic_curve_u16(1_000),
            afterstart_table: semantic_table_u16(1_000),
            warmup_curve: semantic_curve_u16(1_000),
            ae_tps_threshold_curve: semantic_curve_u16(1_000),
            ae_map_threshold_curve: semantic_curve_u16(1_000),
            ae_shot_curve_us: semantic_curve_u16(0),
            ae_decay_steps_curve: semantic_curve_u16(1),
            ae_decay_ratio_curve_x1000: semantic_curve_u16(1_000),
            required_fuel_us: 1_000,
            pref_kpa10: 1_000,
            stoich_afr_x100: 1_470,
            pw_max_us: 20_000,
            afterstart_window_cycles: 0,
            dfco_entry_rpm: 9_000,
            dfco_exit_rpm: 8_900,
            dfco_entry_tps_x100: 1,
            dfco_exit_tps_x100: 2,
            dfco_entry_map_kpa10: 20,
            dfco_delay_cycles: 1,
            soft_rev_rpm: 9_000,
            hard_rev_rpm: 10_000,
            rev_hysteresis_rpm: 100,
            soft_retard_max_deg10: 0,
            idle_target_rpm: 0,
            idle_base_duty_x1000: 0,
            idle_kp_x1000: 0,
            idle_ki_x1000: 0,
            launch_rpm_limit,
            launch_cut_cycles: 0,
            flat_shift_rpm_min,
            flat_shift_cut_cycles: 0,
            knock_threshold_x100: 10_000,
            knock_retard_step_deg10: 0,
            knock_retard_max_deg10: 0,
            knock_recovery_step_deg10: 0,
            knock_recovery_delay_cycles: 0,
            lambda_kp_x1000: 0,
            lambda_ki_x1000: 0,
        },
        state: RuntimeSemanticState::default(),
    }
}

fn semantic_shift_harness(
    launch_rpm_limit: u16,
    flat_shift_rpm_min: u16,
) -> SimulationHarness<4, 4> {
    let mut sim = SimulationHarness::default();
    sim.configure_runtime_fuel_strategy(semantic_shift_strategy(
        launch_rpm_limit,
        flat_shift_rpm_min,
    ));
    sim
}

#[test]
fn harness_applies_board_like_events_to_runtime() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

    assert!(matches!(
        sim.trigger_edge(Micros::new(10), Rpm::new(1200), Degrees10::new(45), false),
        Ok(QueueResult::Enqueued)
    ));
    assert!(matches!(
        sim.cam_edge(Micros::new(12), true),
        Ok(QueueResult::Enqueued)
    ));
    assert!(matches!(
        sim.sensor_frame(
            Micros::new(14),
            Rpm::new(1800),
            Kpa10::new(600),
            Degrees10::new(60)
        ),
        Ok(QueueResult::Enqueued)
    ));
    assert!(matches!(
        sim.tick(Micros::new(20), control_inputs()),
        Ok(QueueResult::Enqueued)
    ));

    assert_eq!(sim.drain_until_idle(), 4);
    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.rpm.get(), 1800);
    assert_eq!(snapshot.engine.load_kpa10.get(), 600);
    assert_eq!(snapshot.engine.angle_x10.get(), 60);
    assert!(sim.last_result().is_some());
}

#[test]
fn default_shift_arming_leaves_semantic_cuts_inactive() {
    let mut sim = semantic_shift_harness(2_500, 9_000);

    sim.trigger_edge(Micros::new(10), Rpm::new(3_000), Degrees10::new(12), true)
        .unwrap();
    sim.sensor_frame(
        Micros::new(12),
        Rpm::new(3_000),
        Kpa10::new(450),
        Degrees10::new(12),
    )
    .unwrap();
    sim.tick(Micros::new(20), control_inputs()).unwrap();
    sim.drain_until_idle();

    let result = sim.last_result().expect("tick result");
    assert!(!result.control.fuel_intent.fuel_cut);
    assert!(!result.control.fuel_intent.spark_cut);
}

#[test]
fn fast_events_coalesce_by_kind() {
    let mut sim: SimulationHarness<2, 1> = SimulationHarness::default();

    assert!(matches!(
        sim.trigger_edge(Micros::new(1), Rpm::new(1000), Degrees10::new(20), false),
        Ok(QueueResult::Enqueued)
    ));
    assert!(matches!(
        sim.trigger_edge(Micros::new(2), Rpm::new(1500), Degrees10::new(40), true),
        Ok(QueueResult::Coalesced)
    ));
    assert_eq!(sim.drain_until_idle(), 1);
    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.rpm.get(), 1500);
    assert_eq!(snapshot.engine.angle_x10.get(), 40);
}

#[test]
fn launch_shift_arming_can_trigger_semantic_cut() {
    let mut sim = semantic_shift_harness(2_500, 9_000);

    sim.trigger_edge(Micros::new(10), Rpm::new(3_000), Degrees10::new(12), true)
        .unwrap();
    sim.sensor_frame(
        Micros::new(12),
        Rpm::new(3_000),
        Kpa10::new(450),
        Degrees10::new(12),
    )
    .unwrap();
    sim.set_shift_arming(true, false);
    sim.tick(Micros::new(20), control_inputs()).unwrap();
    sim.drain_until_idle();

    let result = sim.last_result().expect("tick result");
    assert!(result.control.fuel_intent.fuel_cut);
    assert!(result.control.fuel_intent.spark_cut);
}

#[test]
fn flat_shift_arming_can_trigger_semantic_cut() {
    let mut sim = semantic_shift_harness(9_000, 2_500);

    sim.trigger_edge(Micros::new(10), Rpm::new(3_000), Degrees10::new(12), true)
        .unwrap();
    sim.sensor_frame(
        Micros::new(12),
        Rpm::new(3_000),
        Kpa10::new(450),
        Degrees10::new(12),
    )
    .unwrap();
    sim.set_shift_arming(false, true);
    sim.tick(Micros::new(20), control_inputs()).unwrap();
    sim.drain_until_idle();

    let result = sim.last_result().expect("tick result");
    assert!(result.control.fuel_intent.fuel_cut);
    assert!(result.control.fuel_intent.spark_cut);
}

#[test]
fn cold_start_scenario_runs_through_public_runtime_api() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

    sim.trigger_edge(Micros::new(5), Rpm::new(650), Degrees10::new(10), false)
        .unwrap();
    sim.cam_edge(Micros::new(6), false).unwrap();
    sim.sensor_frame(
        Micros::new(8),
        Rpm::new(650),
        Kpa10::new(250),
        Degrees10::new(10),
    )
    .unwrap();
    sim.tick(Micros::new(10), control_inputs()).unwrap();

    sim.drain_until_idle();

    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.phase, ecu_domain::EnginePhase::Cranking);
    assert_eq!(snapshot.engine.sync, ecu_domain::SyncState::Unsynced);
    assert!(matches!(
        sim.last_result().unwrap().actions.iter().next(),
        Some(ecu_runtime::Action::Idle)
    ));
}

#[test]
fn hot_start_scenario_arms_scheduler_and_reaches_closed_loop() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

    sim.trigger_edge(Micros::new(20), Rpm::new(1800), Degrees10::new(30), true)
        .unwrap();
    sim.cam_edge(Micros::new(21), true).unwrap();
    sim.sensor_frame(
        Micros::new(22),
        Rpm::new(1800),
        Kpa10::new(600),
        Degrees10::new(30),
    )
    .unwrap();
    sim.tick(Micros::new(24), control_inputs()).unwrap();

    sim.drain_until_idle();

    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.phase, ecu_domain::EnginePhase::Running);
    assert_eq!(snapshot.engine.mode, ecu_domain::ControlMode::ClosedLoop);
    assert!(sim.last_result().unwrap().actions.iter().any(|action| {
        matches!(
            action,
            ecu_runtime::Action::ArmScheduler { .. }
                | ecu_runtime::Action::ArmInjection(_)
                | ecu_runtime::Action::ArmIgnition(_)
        )
    }));
}

#[test]
fn simulator_can_run_injection_only_product_without_arm_scheduler() {
    let runtime = EngineRuntime::new();
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::new(runtime);
    sim.configure_batch_injection(2);
    sim.configure_fuel_model(nonzero_fuel_model());

    sim.trigger_edge(Micros::new(20), Rpm::new(1800), Degrees10::new(30), true)
        .unwrap();
    sim.sensor_frame(
        Micros::new(22),
        Rpm::new(1800),
        Kpa10::new(600),
        Degrees10::new(30),
    )
    .unwrap();
    sim.tick(Micros::new(24), control_inputs()).unwrap();
    sim.drain_until_idle();

    let actions = sim.last_result().unwrap().actions;
    assert!(actions.iter().any(|action| matches!(
        action,
        ecu_runtime::Action::ArmInjection(_) | ecu_runtime::Action::Idle
    )));
    assert!(!actions
        .iter()
        .any(|action| matches!(action, ecu_runtime::Action::ArmIgnition(_))));
    assert!(!actions
        .iter()
        .any(|action| matches!(action, ecu_runtime::Action::ArmScheduler { .. })));
}

#[test]
fn acceleration_scenario_updates_runtime_through_ticks() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

    sim.trigger_edge(Micros::new(30), Rpm::new(1200), Degrees10::new(15), true)
        .unwrap();
    sim.sensor_frame(
        Micros::new(31),
        Rpm::new(1200),
        Kpa10::new(350),
        Degrees10::new(15),
    )
    .unwrap();
    sim.tick(Micros::new(32), control_inputs()).unwrap();
    sim.sensor_frame(
        Micros::new(40),
        Rpm::new(2200),
        Kpa10::new(520),
        Degrees10::new(18),
    )
    .unwrap();
    sim.tick(Micros::new(42), control_inputs()).unwrap();

    sim.drain_until_idle();

    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.rpm.get(), 2200);
    assert_eq!(snapshot.engine.load_kpa10.get(), 520);
    assert_eq!(snapshot.engine.angle_x10.get(), 18);
    assert!(sim.last_result().is_some());
}

#[test]
fn sync_loss_and_recovery_scenario_cancels_then_rearms_outputs() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

    sim.trigger_edge(Micros::new(50), Rpm::new(1500), Degrees10::new(20), true)
        .unwrap();
    sim.cam_edge(Micros::new(51), true).unwrap();
    sim.sensor_frame(
        Micros::new(52),
        Rpm::new(1500),
        Kpa10::new(450),
        Degrees10::new(20),
    )
    .unwrap();
    sim.tick(Micros::new(54), control_inputs()).unwrap();
    sim.drain_until_idle();
    assert!(sim.last_result().unwrap().actions.iter().any(|action| {
        matches!(
            action,
            ecu_runtime::Action::ArmScheduler { .. }
                | ecu_runtime::Action::ArmInjection(_)
                | ecu_runtime::Action::ArmIgnition(_)
        )
    }));

    sim.trigger_edge(Micros::new(60), Rpm::new(0), Degrees10::new(20), false)
        .unwrap();
    sim.cam_edge(Micros::new(61), false).unwrap();
    sim.sensor_frame(
        Micros::new(62),
        Rpm::new(0),
        Kpa10::new(0),
        Degrees10::new(20),
    )
    .unwrap();
    sim.tick(Micros::new(64), control_inputs()).unwrap();
    sim.drain_until_idle();
    assert!(sim
        .last_result()
        .unwrap()
        .actions
        .iter()
        .any(|action| matches!(action, ecu_runtime::Action::CancelScheduler(_))));
    assert_ne!(
        sim.runtime().scheduler_state().mode(),
        ecu_scheduler::SchedulerMode::Armed
    );

    sim.trigger_edge(Micros::new(70), Rpm::new(1500), Degrees10::new(20), true)
        .unwrap();
    sim.cam_edge(Micros::new(71), true).unwrap();
    sim.sensor_frame(
        Micros::new(72),
        Rpm::new(1500),
        Kpa10::new(450),
        Degrees10::new(20),
    )
    .unwrap();
    sim.tick(Micros::new(74), control_inputs()).unwrap();
    sim.drain_until_idle();
    assert!(sim
        .last_result()
        .unwrap()
        .actions
        .iter()
        .any(|action| matches!(
            action,
            ecu_runtime::Action::ArmScheduler { .. }
                | ecu_runtime::Action::ArmInjection(_)
                | ecu_runtime::Action::ArmIgnition(_)
        )));
}

#[test]
fn sensor_fault_scenario_surfaces_limp_home_state() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    sim.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        ecu_domain::CancelReason::Manual,
    );

    sim.trigger_edge(Micros::new(90), Rpm::new(900), Degrees10::new(5), true)
        .unwrap();
    sim.cam_edge(Micros::new(91), true).unwrap();
    sim.sensor_frame(
        Micros::new(92),
        Rpm::new(900),
        Kpa10::new(300),
        Degrees10::new(5),
    )
    .unwrap();
    sim.tick(Micros::new(94), control_inputs()).unwrap();
    sim.drain_until_idle();

    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.faults.fault, FaultCode::SensorOutOfRange);
    assert_eq!(snapshot.faults.severity, FaultSeverity::Warning);
    assert_eq!(snapshot.engine.mode, ecu_domain::ControlMode::LimpHome);
    assert!(sim
        .last_result()
        .unwrap()
        .actions
        .iter()
        .any(|action| match action {
            ecu_runtime::Action::ApplyAux(commands) => commands.len() == 1,
            _ => false,
        }));
}

#[test]
fn sync_loss_cancels_pending_outputs_through_scheduler() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

    sim.trigger_edge(Micros::new(100), Rpm::new(1600), Degrees10::new(22), true)
        .unwrap();
    sim.cam_edge(Micros::new(101), true).unwrap();
    sim.sensor_frame(
        Micros::new(102),
        Rpm::new(1600),
        Kpa10::new(500),
        Degrees10::new(22),
    )
    .unwrap();
    sim.tick(Micros::new(104), control_inputs()).unwrap();
    sim.drain_until_idle();
    assert_eq!(
        sim.runtime().scheduler_state().mode(),
        ecu_scheduler::SchedulerMode::Armed
    );

    sim.trigger_edge(Micros::new(110), Rpm::new(0), Degrees10::new(22), false)
        .unwrap();
    sim.cam_edge(Micros::new(111), false).unwrap();
    sim.sensor_frame(
        Micros::new(112),
        Rpm::new(0),
        Kpa10::new(0),
        Degrees10::new(22),
    )
    .unwrap();
    sim.tick(Micros::new(114), control_inputs()).unwrap();
    sim.drain_until_idle();

    assert_ne!(
        sim.runtime().scheduler_state().mode(),
        ecu_scheduler::SchedulerMode::Armed
    );
    assert!(sim
        .last_result()
        .unwrap()
        .actions
        .iter()
        .any(|action| matches!(action, ecu_runtime::Action::CancelScheduler(_))));
}

#[test]
fn degraded_and_substituted_inputs_remain_visible_in_snapshot_and_faults() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    sim.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        ecu_domain::CancelReason::Manual,
    );

    sim.trigger_edge(Micros::new(200), Rpm::new(750), Degrees10::new(12), false)
        .unwrap();
    sim.cam_edge(Micros::new(201), false).unwrap();
    sim.sensor_frame(
        Micros::new(202),
        Rpm::new(775),
        Kpa10::new(345),
        Degrees10::new(14),
    )
    .unwrap();
    sim.tick(Micros::new(204), control_inputs()).unwrap();
    sim.drain_until_idle();

    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.faults.fault, FaultCode::SensorOutOfRange);
    assert_eq!(snapshot.faults.severity, FaultSeverity::Warning);
    assert_eq!(snapshot.engine.rpm.get(), 775);
    assert_eq!(snapshot.engine.load_kpa10.get(), 345);
    assert_eq!(snapshot.engine.angle_x10.get(), 14);
    assert_eq!(snapshot.engine.rpm.get(), 775);
    assert_eq!(snapshot.engine.load_kpa10.get(), 345);
}
