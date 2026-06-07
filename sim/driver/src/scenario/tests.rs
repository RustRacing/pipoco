use super::execution::{
    run_default_headless_smoke, run_default_headless_smoke_twice, run_headless_smoke,
};
use super::output::{OutputLevelTracker, PendingOutputQueue};
use super::{DriverRunReport, ScenarioConfig};
use crate::DriverError;
use ecu_domain::Micros;
use ecu_io::OutputTransitionKind;
use ecu_sim::plant::{ClosedLoopPlant, FixedPlantProfile, InjectorModel};

use crate::trace::FixedDriverTrace;

fn require_report(result: Result<DriverRunReport, DriverError>) -> DriverRunReport {
    let Ok(report) = result else {
        panic!("scenario should succeed in this unit test: {result:?}")
    };
    report
}

#[test]
fn headless_smoke_syncs_and_emits_outputs() {
    let report = require_report(run_default_headless_smoke());
    assert_eq!(report.ecu_snapshot.synced, 1, "ECU should be synced");
    assert!(
        report.injection_outputs > 0,
        "should emit injection outputs"
    );
    assert!(report.ignition_outputs > 0, "should emit ignition outputs");
}

#[test]
fn headless_smoke_outputs_drive_plant_combustion() {
    let report = require_report(run_default_headless_smoke());
    assert!(
        report.combustion_events > 0,
        "plant should have combustion events"
    );
}

#[test]
fn headless_smoke_is_deterministic_across_repeated_runs() {
    let Ok((r1, r2)) = run_default_headless_smoke_twice() else {
        unreachable!("determinism run should succeed")
    };
    assert_eq!(r1, r2, "two smoke runs should produce identical reports");
}

#[test]
fn invalid_scenario_config_is_rejected() {
    let bad_crank = ScenarioConfig {
        crank_period_us: 0,
        ..Default::default()
    };
    let result = run_headless_smoke(bad_crank);
    assert!(result.is_err(), "zero crank_period should be rejected");

    let bad_tick = ScenarioConfig {
        tick_period_us: 0,
        ..Default::default()
    };
    let result_tick = run_headless_smoke(bad_tick);
    assert!(result_tick.is_err(), "zero tick_period should be rejected");

    let bad_steps = ScenarioConfig {
        steps: 10,
        ..Default::default()
    };
    let result_steps = run_headless_smoke(bad_steps);
    assert!(result_steps.is_err(), "steps < 12 should be rejected");

    let bad_events = ScenarioConfig {
        max_events_per_step: ecu_sim_ffi::ECU_SIM_MAX_EVENTS + 1,
        ..Default::default()
    };
    let result_events = run_headless_smoke(bad_events);
    assert!(
        result_events.is_err(),
        "max_events_per_step above the ABI limit should be rejected"
    );
}

#[test]
fn timestamp_overflow_is_rejected() {
    let config = ScenarioConfig {
        start_us: u32::MAX - 8,
        tick_period_us: 2,
        steps: 12,
        ..Default::default()
    };

    assert_eq!(
        run_headless_smoke(config),
        Err(DriverError::FfiStatus(
            ecu_sim_ffi::EcuSimStatus::ErrInvalid
        )),
        "scenario timestamp overflow must be explicit instead of wrapping"
    );
}

#[test]
fn pending_output_queue_orders_and_drains_due_events() {
    let mut queue: PendingOutputQueue<8> = PendingOutputQueue::new();
    let mut plant = ClosedLoopPlant::new(
        FixedPlantProfile::inline_four(),
        InjectorModel::gasoline(240),
    );
    let mut trace = FixedDriverTrace::<8>::new();
    let mut levels = OutputLevelTracker::new();

    // Insert events out of order, including a duplicate unchanged transition.
    let e1 = ecu_io::OutputTransition {
        at_us: Micros::new(300),
        kind: OutputTransitionKind::Ignition,
        channel: ecu_domain::ChannelId::new(0),
        level: ecu_io::OutputLevel::High,
    };
    let e2 = ecu_io::OutputTransition {
        at_us: Micros::new(100),
        kind: OutputTransitionKind::Injector,
        channel: ecu_domain::ChannelId::new(0),
        level: ecu_io::OutputLevel::High,
    };
    let e3 = ecu_io::OutputTransition {
        at_us: Micros::new(100),
        kind: OutputTransitionKind::Injector,
        channel: ecu_domain::ChannelId::new(0),
        level: ecu_io::OutputLevel::High,
    };
    let e4 = ecu_io::OutputTransition {
        at_us: Micros::new(400),
        kind: OutputTransitionKind::Ignition,
        channel: ecu_domain::ChannelId::new(0),
        level: ecu_io::OutputLevel::Low,
    };

    assert_eq!(queue.push_sorted(e1), Ok(()));
    assert_eq!(queue.push_sorted(e2), Ok(()));
    assert_eq!(queue.push_sorted(e3), Ok(()));
    assert_eq!(queue.push_sorted(e4), Ok(()));

    // Drain at t=150 - both 100us events should fire and the second one should
    // remain a duplicate unchanged transition.
    assert_eq!(
        queue.drain_due(150, &mut plant, &mut trace, &mut levels),
        Ok(())
    );
    assert_eq!(trace.len(), 2, "should have two trace records after drain");
    assert_eq!(trace.get(0).map(|record| record.at_us), Some(100));
    assert_eq!(trace.get(1).map(|record| record.at_us), Some(100));
    assert_eq!(trace.get(0).map(|record| record.status), Some(0));
    assert_eq!(trace.get(1).map(|record| record.status), Some(1));
    assert_eq!(queue.len, 2, "future events should remain queued");
    assert_eq!(queue.events[0].map(|event| event.at_us.get()), Some(300));
    assert_eq!(queue.events[1].map(|event| event.at_us.get()), Some(400));

    // Drain at t=350 - should apply the 300us event and retain the future one.
    assert_eq!(
        queue.drain_due(350, &mut plant, &mut trace, &mut levels),
        Ok(())
    );
    assert_eq!(trace.len(), 3);
    assert_eq!(trace.get(2).map(|record| record.at_us), Some(300));
    assert_eq!(queue.len, 1);
    assert_eq!(queue.events[0].map(|event| event.at_us.get()), Some(400));

    // Drain at t=450 - should apply the final future event.
    assert_eq!(
        queue.drain_due(450, &mut plant, &mut trace, &mut levels),
        Ok(())
    );
    assert_eq!(trace.len(), 4);
    assert_eq!(queue.len, 0, "queue should be empty");
}

#[test]
fn pending_output_queue_rejects_invalid_channel_before_plant_apply() {
    let mut queue: PendingOutputQueue<2> = PendingOutputQueue::new();
    let mut plant = ClosedLoopPlant::new(
        FixedPlantProfile::inline_four(),
        InjectorModel::gasoline(240),
    );
    let mut trace = FixedDriverTrace::<2>::new();
    let mut levels = OutputLevelTracker::new();
    let invalid = ecu_io::OutputTransition {
        at_us: Micros::new(100),
        kind: OutputTransitionKind::Injector,
        channel: ecu_domain::ChannelId::new(4),
        level: ecu_io::OutputLevel::High,
    };

    assert_eq!(queue.push_sorted(invalid), Ok(()));
    assert_eq!(
        queue.drain_due(100, &mut plant, &mut trace, &mut levels),
        Err(DriverError::InvalidOutputChannel {
            kind: OutputTransitionKind::Injector,
            channel: 4,
        })
    );
    assert_eq!(trace.len(), 0);
}

#[test]
fn crank_period_changes_crank_edge_cadence() {
    let default_report = require_report(run_default_headless_smoke());
    let slower_crank_report = require_report(run_headless_smoke(ScenarioConfig {
        crank_period_us: 1_000,
        ..Default::default()
    }));

    let mut default_crank_edges = 0usize;
    for i in 0..default_report.trace.len() {
        if let Some(rec) = default_report.trace.get(i) {
            if rec.kind == crate::trace::DriverTraceKind::CrankEdge {
                default_crank_edges += 1;
            }
        }
    }

    let mut slower_crank_edges = 0usize;
    for i in 0..slower_crank_report.trace.len() {
        if let Some(rec) = slower_crank_report.trace.get(i) {
            if rec.kind == crate::trace::DriverTraceKind::CrankEdge {
                slower_crank_edges += 1;
            }
        }
    }

    assert!(
        slower_crank_edges < default_crank_edges,
        "slower crank period must produce fewer crank-edge records: default={}, slower={}",
        default_crank_edges,
        slower_crank_edges
    );
}
