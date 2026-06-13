use super::*;
use ecu_domain::{FaultCode, FaultSeverity, Lambda100, PulseWidthUs};
use ecu_runtime::{
    BaseFuelModel, EngineRuntime, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs,
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
            clt_c: 80,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(3000)),
    }
}

fn nonzero_fuel_model() -> BaseFuelModel {
    BaseFuelModel::new(
        [Rpm::new(1800); 16],
        [Kpa10::new(600); 16],
        [[PulseWidthUs::new(2500); 16]; 16],
    )
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
    assert!(matches!(
        sim.last_result().unwrap().actions.iter().next(),
        Some(ecu_runtime::Action::ArmScheduler { .. })
    ));
}

#[test]
fn simulator_can_run_injection_only_product_without_arm_scheduler() {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(nonzero_fuel_model());
    runtime.configure_batch_injection(2);
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::new(runtime);

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
    assert!(matches!(
        sim.last_result().unwrap().actions.iter().next(),
        Some(ecu_runtime::Action::ArmScheduler { .. })
    ));

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
            ecu_runtime::Action::ArmScheduler { .. } | ecu_runtime::Action::ArmInjection(_)
        )));
}

#[test]
fn sensor_fault_scenario_surfaces_limp_home_state() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    sim.runtime.set_fault_state(
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
    sim.runtime.set_fault_state(
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
