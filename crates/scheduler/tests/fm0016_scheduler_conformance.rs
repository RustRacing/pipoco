//! Real-execution FM0016 conformance tests for ecu-scheduler.
//!
//! Drives `SchedulerState` through its public API for every FM0016 fixture
//! and verifies that the scheduler is driven with the correct product runtime
//! fuel and schedule observations.
//!
//! Key insight: the scheduler owns the deadline/channel/event-order logic.
//! It does NOT own the angle computation - that comes from runtime's fuel/ignition
//! planning. The product runtime semantic evaluators (runtime_semantic_evaluate_fuel
//! and runtime_semantic_evaluate_schedule) provide the authoritative product observations.
//!
//! This test:
//!   1. Builds product fuel observations via runtime_semantic_evaluate_fuel
//!   2. Builds product schedule observations via runtime_semantic_evaluate_schedule
//!   3. Drives SchedulerState with the product events
//!   4. Verifies scheduler counts and active groups match product event presence
//!   5. Keeps cancellation/suspend assertions

#![cfg(test)]

use ecu_domain::{Degrees10, DwellUs, Micros, PulseWidthUs};
use ecu_runtime::semantic::{
    conformance::runtime_semantic_evaluate_schedule, runtime_semantic_evaluate_fuel,
    RuntimeSemanticFuelObservations, RuntimeSemanticScheduleEventKind,
};
use ecu_scheduler::test_support::{observe_scheduler, SchedulerObservedSurface};
use ecu_scheduler::{
    ChannelId, ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup, SchedulerMode,
    SchedulerState,
};
use ecu_test_fixtures::semantic::{
    build_semantic_calibration, build_semantic_schedule_calibration, semantic_state_for_fixture,
    to_semantic_input,
};

// --------------------------------------------------------------------------
// Product observation helpers
// --------------------------------------------------------------------------

/// Count injection events for a given cylinder in the schedule observations.
fn count_injection_events_for_cylinder(
    sched_obs: &ecu_runtime::semantic::RuntimeSemanticScheduleObservations,
    cylinder: u8,
) -> usize {
    let mut count = 0;
    for i in 0..sched_obs.events.len as usize {
        let evt = &sched_obs.events.events[i];
        if evt.cylinder == cylinder
            && matches!(evt.kind, RuntimeSemanticScheduleEventKind::InjectionOpen)
        {
            count += 1;
        }
    }
    count
}

/// Count ignition events for a given cylinder in the schedule observations.
fn count_ignition_events_for_cylinder(
    sched_obs: &ecu_runtime::semantic::RuntimeSemanticScheduleObservations,
    cylinder: u8,
) -> usize {
    let mut count = 0;
    for i in 0..sched_obs.events.len as usize {
        let evt = &sched_obs.events.events[i];
        if evt.cylinder == cylinder
            && matches!(evt.kind, RuntimeSemanticScheduleEventKind::CoilFire)
        {
            count += 1;
        }
    }
    count
}

/// Run scheduler for a fixture case using product APIs only.
/// Fuel and schedule observations come from the runtime semantic evaluators,
/// not from the spec oracle.
fn run_scheduler_for_case(
    case: &ecu_test_fixtures::fixture_matrix::FixtureCase,
) -> SchedulerObservedSurface {
    let mut state = SchedulerState::new();
    let input = case.input;

    // Build product fuel observations via runtime_semantic_evaluate_fuel
    let semantic_cal = build_semantic_calibration(&case.calibration);
    let semantic_input = to_semantic_input(&case.input);
    let fuel_obs: RuntimeSemanticFuelObservations = runtime_semantic_evaluate_fuel(
        &semantic_cal,
        semantic_input,
        semantic_state_for_fixture(case),
    )
    .expect("fuel evaluation should succeed");

    // Build product schedule observations via runtime_semantic_evaluate_schedule
    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);
    let sched_obs = runtime_semantic_evaluate_schedule(&schedule_cal, semantic_input, fuel_obs)
        .expect("schedule evaluation should succeed");

    // Drive scheduler using product events
    // Use timestamps from input
    let now = Micros::new(input.t_us.get());
    let start = Micros::new(input.t_us.get().saturating_add(100));

    // For each cylinder with injection events, schedule an injection
    // For each cylinder with ignition events, schedule an ignition
    let cyl_count = sched_obs.soi_deg10.count as usize;
    for cyl in 0..cyl_count {
        // Check if we have injection events for this cylinder
        let inj_count = count_injection_events_for_cylinder(&sched_obs, cyl as u8);
        if inj_count > 0 {
            // schedule injection using pw_corr_us
            let duration = fuel_obs.pw_corr_us.max(1);
            let end = Micros::new(start.get().saturating_add(duration));
            let inj_plan = InjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(cyl as u8 + 1)),
                pulse_width: PulseWidthUs::new(duration),
            };
            let _ = state.schedule_injection(now, start, end, inj_plan);
        }

        // Check if we have ignition events for this cylinder
        let ign_count = count_ignition_events_for_cylinder(&sched_obs, cyl as u8);
        if ign_count > 0 {
            let duration = sched_obs.dwell_us.max(1) as u16;
            let end = Micros::new(start.get().saturating_add(duration as u32));
            let ign_plan = IgnitionPlan {
                output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(cyl as u8 + 1)),
                dwell: DwellUs::new(duration),
                advance: Degrees10::new(sched_obs.spark_advance_deg10 as i16),
            };
            let _ = state.schedule_ignition(now, start, end, ign_plan);
        }
    }

    observe_scheduler(&state)
}

// --------------------------------------------------------------------------
// Conformance test
// --------------------------------------------------------------------------

fn conformance_test(case: &ecu_test_fixtures::fixture_matrix::FixtureCase) {
    // Verify fixture semantics via spec oracle (ONE call for semantic verification only)
    let spec = ecu_test_fixtures::fixture_matrix::oracle_result(*case);
    ecu_test_fixtures::fixture_matrix::assert_fixture_semantics(*case, &spec);

    // Run product scheduler to get observed data
    let obs = run_scheduler_for_case(case);

    // Build product observations for comparison
    let semantic_cal = build_semantic_calibration(&case.calibration);
    let semantic_input = to_semantic_input(&case.input);
    let fuel_obs = runtime_semantic_evaluate_fuel(
        &semantic_cal,
        semantic_input,
        semantic_state_for_fixture(case),
    )
    .expect("fuel evaluation should succeed");

    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);
    let sched_obs = runtime_semantic_evaluate_schedule(&schedule_cal, semantic_input, fuel_obs)
        .expect("schedule evaluation should succeed");

    // Determine expected counts from product observations
    let cyl_count = sched_obs.soi_deg10.count as usize;
    let mut expected_injection_count = 0usize;
    let mut expected_ignition_count = 0usize;

    for cyl in 0..cyl_count {
        expected_injection_count += count_injection_events_for_cylinder(&sched_obs, cyl as u8);
        expected_ignition_count += count_ignition_events_for_cylinder(&sched_obs, cyl as u8);
    }

    let has_fuel = fuel_obs.pw_corr_us > 0 && expected_injection_count > 0;
    let has_ignition = expected_ignition_count > 0;
    let has_any_event = has_fuel || has_ignition;

    // Scheduler mode should be Armed after scheduling (only if any event to schedule)
    if has_any_event {
        assert_eq!(
            obs.mode,
            SchedulerMode::Armed,
            "scheduler should be armed when any event is scheduled"
        );

        // Active groups should include Injector and Ignition (if present)
        if expected_injection_count > 0 {
            assert!(
                obs.active_groups & OutputGroup::Injector.mask() != 0,
                "Injector group should be active when injection events present"
            );
        }
        if has_ignition {
            assert!(
                obs.active_groups & OutputGroup::Ignition.mask() != 0,
                "Ignition group should be active when ignition events present"
            );
        }

        // Reserved channels should be set for active outputs
        if expected_injection_count > 0 {
            assert!(
                obs.reserved_channels[0] != 0,
                "Injector channel should be reserved"
            );
        }
        if has_ignition {
            assert!(
                obs.reserved_channels[1] != 0,
                "Ignition channel should be reserved"
            );
        }

        // Injection and ignition counts should match product events
        assert_eq!(
            obs.injection_count as u32, expected_injection_count as u32,
            "injection_count should match product injection events"
        );
        assert_eq!(
            obs.ignition_count as u32, expected_ignition_count as u32,
            "ignition_count should match product ignition events"
        );
    } else {
        // No events scheduled - scheduler should be Idle
        assert_eq!(
            obs.mode,
            SchedulerMode::Idle,
            "scheduler should be idle when no events are scheduled"
        );
    }

    // --- Cancellation/suspend observability ---
    // Verify cancel_group, cancel_all, suspend, on_sync_loss are observable
    let mut cancel_state = SchedulerState::new();
    let test_inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(1000),
    };
    let test_ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
        dwell: DwellUs::new(2000),
        advance: Degrees10::new(100),
    };
    let now = Micros::new(1000);
    let start = Micros::new(1100);
    let end = Micros::new(2100);

    let _ = cancel_state.schedule_injection(now, start, end, test_inj);
    let _ = cancel_state.schedule_ignition(now, start, end, test_ign);
    assert_eq!(cancel_state.mode(), SchedulerMode::Armed);

    // cancel_group should clear the group
    cancel_state.cancel_group(OutputGroup::Injector);
    assert_eq!(
        cancel_state.active_groups() & OutputGroup::Injector.mask(),
        0
    );

    // cancel_all should clear everything
    cancel_state.cancel_all();
    assert_eq!(cancel_state.mode(), SchedulerMode::Idle);
    assert_eq!(cancel_state.active_groups(), 0);

    // suspend should transition to Suspended mode
    let _ = cancel_state.schedule_injection(now, start, end, test_inj);
    cancel_state.suspend();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);

    // on_sync_loss should suspend
    cancel_state.on_sync_loss();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);

    // on_hard_safety_shutdown should suspend
    cancel_state.on_hard_safety_shutdown();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);
}

fn run_all(cases: &[ecu_test_fixtures::fixture_matrix::FixtureCase]) {
    for case in cases {
        conformance_test(case);
    }
}

fn synced_fixtures() -> Vec<ecu_test_fixtures::fixture_matrix::FixtureCase> {
    ecu_test_fixtures::fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| matches!(c.input.sync, ecu_spec::SyncState::Synced))
        .collect()
}

fn unsynced_fixtures() -> Vec<ecu_test_fixtures::fixture_matrix::FixtureCase> {
    ecu_test_fixtures::fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| !matches!(c.input.sync, ecu_spec::SyncState::Synced))
        .collect()
}

fn cut_fixtures() -> Vec<ecu_test_fixtures::fixture_matrix::FixtureCase> {
    ecu_test_fixtures::fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| c.fixture.contains("cut"))
        .collect()
}

fn running_fixtures() -> Vec<ecu_test_fixtures::fixture_matrix::FixtureCase> {
    ecu_test_fixtures::fixture_matrix::fixture_cases()
        .into_iter()
        .filter(|c| c.fixture.contains("running") && !c.fixture.contains("cut"))
        .collect()
}

#[test]
fn scheduler_fm0016_unsynced() {
    let cases = unsynced_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_cuts() {
    let cases = cut_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_running() {
    let cases = running_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_synced() {
    let cases = synced_fixtures();
    assert!(!cases.is_empty());
    run_all(&cases);
}

#[test]
fn scheduler_fm0016_all() {
    run_all(&ecu_test_fixtures::fixture_matrix::fixture_cases());
}
