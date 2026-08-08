//! Real-execution FM0016 conformance tests for ecu-scheduler.
//!
//! Drives `SchedulerState` through its public API for every FM0016 fixture
//! using the product runtime semantic evaluators, and checks the resulting
//! scheduler surface against the INDEPENDENT spec oracle
//! (`ecu_test_fixtures::fixture_matrix::oracle_result`).
//!
//! The scheduler owns the deadline/channel/event-order logic; it does not own
//! the angle computation. So the drive path is the product runtime, but every
//! expectation - counts, window durations, advance, and the angle/event data
//! the scheduler is fed - is derived from the spec oracle. A semantic
//! regression in the runtime evaluators therefore fails here instead of being
//! silently mirrored into the expectations.

#![cfg(test)]

use ecu_calibration::{
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertUnlock,
    SecondaryTriggerMode, TriggerAuthority,
};
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, DwellUs, EngineTimeAuthority, Micros,
    PhaseSyncState, PulseWidthUs, SyncState as DomainSyncState,
};
use ecu_runtime::semantic::{
    conformance::runtime_semantic_evaluate_schedule_with_authority, runtime_semantic_evaluate_fuel,
    RuntimeSemanticFuelObservations, RuntimeSemanticInputSnapshot,
    RuntimeSemanticScheduleEventKind, RuntimeSemanticScheduleObservations,
};
use ecu_scheduler::test_support::{observe_scheduler, SchedulerObservedSurface};
use ecu_scheduler::{
    ChannelId, ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup, SchedulerMode,
    SchedulerState,
};
use ecu_spec::EventKind;
use ecu_test_fixtures::fixture_matrix::EPS_ANGLE_DEG10;
use ecu_test_fixtures::semantic::{
    build_semantic_calibration, build_semantic_schedule_calibration, semantic_state_for_fixture,
    to_semantic_input,
};

// --------------------------------------------------------------------------
// Oracle expectations (independent of the runtime evaluators)
// --------------------------------------------------------------------------

/// What the spec oracle says the scheduler must end up holding for a fixture.
struct OracleExpectation {
    /// Number of injector channels the oracle expects to be reserved.
    injection_channels: usize,
    /// Number of ignition channels the oracle expects to be reserved.
    ignition_channels: usize,
    /// Injector pulse width in microseconds.
    pw_corr_us: u32,
    /// Coil dwell in microseconds.
    dwell_us: u32,
    /// Spark advance in deg10.
    spark_advance_deg10: i16,
}

fn oracle_expectation(spec: &ecu_spec::StepResult) -> OracleExpectation {
    let mut injector_cylinders: u64 = 0;
    let mut ignition_cylinders: u64 = 0;
    for idx in 0..spec.output.events.len as usize {
        let evt = spec.output.events.events[idx];
        let bit = 1u64 << (evt.cylinder.get() as u32 % 64);
        match evt.kind {
            EventKind::InjectionOpen => injector_cylinders |= bit,
            EventKind::CoilFire => ignition_cylinders |= bit,
            _ => {}
        }
    }
    OracleExpectation {
        injection_channels: injector_cylinders.count_ones() as usize,
        ignition_channels: ignition_cylinders.count_ones() as usize,
        pw_corr_us: spec.output.pw_corr_us.get(),
        dwell_us: spec.output.dwell_us.get(),
        spark_advance_deg10: spec.output.spark_advance_deg10.get(),
    }
}

/// Assert that the runtime observations used to drive the scheduler agree with
/// the oracle. Without this, the scheduler test is blind to any semantic drift
/// in its own drive path.
fn assert_drive_matches_oracle(
    case: &ecu_test_fixtures::fixture_matrix::FixtureCase,
    spec: &ecu_spec::StepResult,
    fuel_obs: &RuntimeSemanticFuelObservations,
    sched_obs: &RuntimeSemanticScheduleObservations,
) {
    let label = |what: &str| format!("{} [{}/{}]", what, case.fixture, case.variant);

    assert_eq!(
        fuel_obs.pw_corr_us,
        spec.output.pw_corr_us.get(),
        "{}",
        label("pw_corr_us driving the scheduler must match the oracle")
    );
    assert_eq!(
        sched_obs.dwell_us,
        spec.output.dwell_us.get(),
        "{}",
        label("dwell_us driving the scheduler must match the oracle")
    );
    assert_eq!(
        sched_obs.spark_advance_deg10,
        spec.output.spark_advance_deg10.get(),
        "{}",
        label("spark_advance_deg10 driving the scheduler must match the oracle")
    );
    // The oracle fills per-cylinder value arrays but leaves `count` at its
    // default, so the live cylinder count comes from the calibration-derived
    // runtime observation.
    for cyl in 0..sched_obs.soi_deg10.count as usize {
        for (what, observed, expected) in [
            (
                "soi_deg10",
                sched_obs.soi_deg10.values[cyl],
                spec.output.soi_deg10.values[cyl],
            ),
            (
                "eoi_deg10",
                sched_obs.eoi_deg10.values[cyl],
                spec.output.eoi_deg10.values[cyl],
            ),
            (
                "spark_deg10",
                sched_obs.spark_deg10.values[cyl],
                spec.output.spark_deg10.values[cyl],
            ),
            (
                "dwell_start_deg10",
                sched_obs.dwell_start_deg10.values[cyl],
                spec.output.dwell_start_deg10.values[cyl],
            ),
        ] {
            assert!(
                observed.abs_diff(expected) <= EPS_ANGLE_DEG10,
                "{}: cyl={cyl} observed={observed} expected={expected}",
                label(what)
            );
        }
    }

    for (kind, spec_kind) in [
        (
            RuntimeSemanticScheduleEventKind::InjectionOpen,
            EventKind::InjectionOpen,
        ),
        (
            RuntimeSemanticScheduleEventKind::InjectionClose,
            EventKind::InjectionClose,
        ),
        (
            RuntimeSemanticScheduleEventKind::CoilChargeStart,
            EventKind::CoilChargeStart,
        ),
        (
            RuntimeSemanticScheduleEventKind::CoilFire,
            EventKind::CoilFire,
        ),
    ] {
        let observed = (0..sched_obs.events.len as usize)
            .filter(|idx| sched_obs.events.events[*idx].kind == kind)
            .count();
        let expected = (0..spec.output.events.len as usize)
            .filter(|idx| spec.output.events.events[*idx].kind == spec_kind)
            .count();
        assert_eq!(
            observed,
            expected,
            "{}: {:?}",
            label("event count must match the oracle"),
            kind
        );
    }
}

// --------------------------------------------------------------------------
// Product drive path
// --------------------------------------------------------------------------

/// Result of driving the scheduler with the product runtime observations.
struct DrivenScheduler {
    surface: SchedulerObservedSurface,
    fuel_obs: RuntimeSemanticFuelObservations,
    sched_obs: RuntimeSemanticScheduleObservations,
    /// Advance carried by the last accepted ignition plan, as the scheduler
    /// returned it.
    accepted_ignition_advance: Option<i16>,
}

/// Sequential-authority stand-in matching the ecu-runtime FM0016 harness, so
/// the scheduler is driven with the same event set the runtime conformance
/// suite validates.
fn semantic_schedule_authority(input: RuntimeSemanticInputSnapshot) -> EngineTimeAuthority {
    if matches!(input.sync, DomainSyncState::Locked { .. }) {
        let calibration = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D35_3054,
            profile_hash: 0xA5A5_1234,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            injection_layout: ExpertInjectionLayout::Sequential,
            ..ExpertTriggerCalibration::default()
        };
        let startup_authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );

        calibration
            .to_runtime_engine_time_authority(startup_authority)
            .expect("validated manual runtime authority")
    } else {
        EngineTimeAuthority::none()
    }
}

/// Count injection events for a given cylinder in the schedule observations.
fn count_injection_events_for_cylinder(
    sched_obs: &RuntimeSemanticScheduleObservations,
    cylinder: u8,
) -> usize {
    (0..sched_obs.events.len as usize)
        .filter(|idx| {
            let evt = &sched_obs.events.events[*idx];
            evt.cylinder == cylinder
                && matches!(evt.kind, RuntimeSemanticScheduleEventKind::InjectionOpen)
        })
        .count()
}

/// Count ignition events for a given cylinder in the schedule observations.
fn count_ignition_events_for_cylinder(
    sched_obs: &RuntimeSemanticScheduleObservations,
    cylinder: u8,
) -> usize {
    (0..sched_obs.events.len as usize)
        .filter(|idx| {
            let evt = &sched_obs.events.events[*idx];
            evt.cylinder == cylinder
                && matches!(evt.kind, RuntimeSemanticScheduleEventKind::CoilFire)
        })
        .count()
}

/// Run the scheduler for a fixture case using product APIs only.
fn run_scheduler_for_case(
    case: &ecu_test_fixtures::fixture_matrix::FixtureCase,
) -> DrivenScheduler {
    let mut state = SchedulerState::new();
    let input = case.input;

    let semantic_cal = build_semantic_calibration(&case.calibration);
    let semantic_input = to_semantic_input(&case.input);
    let fuel_obs: RuntimeSemanticFuelObservations = runtime_semantic_evaluate_fuel(
        &semantic_cal,
        semantic_input,
        semantic_state_for_fixture(case),
    )
    .expect("fuel evaluation should succeed");

    let schedule_cal = build_semantic_schedule_calibration(&case.calibration);
    let sched_obs = runtime_semantic_evaluate_schedule_with_authority(
        &schedule_cal,
        semantic_input,
        fuel_obs,
        semantic_schedule_authority(semantic_input),
    )
    .expect("schedule evaluation should succeed");

    let now = Micros::new(input.t_us.get());
    let start = Micros::new(input.t_us.get().saturating_add(100));
    let mut accepted_ignition_advance = None;

    let cyl_count = sched_obs.soi_deg10.count as usize;
    for cyl in 0..cyl_count {
        if count_injection_events_for_cylinder(&sched_obs, cyl as u8) > 0 {
            let duration = fuel_obs.pw_corr_us.max(1);
            let end = Micros::new(start.get().saturating_add(duration));
            let inj_plan = InjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(cyl as u8 + 1)),
                pulse_width: PulseWidthUs::new(duration),
            };
            let _ = state.schedule_injection(now, start, end, inj_plan);
        }

        if count_ignition_events_for_cylinder(&sched_obs, cyl as u8) > 0 {
            let duration = sched_obs.dwell_us.max(1) as u16;
            let end = Micros::new(start.get().saturating_add(duration as u32));
            let ign_plan = IgnitionPlan {
                output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(cyl as u8 + 1)),
                dwell: DwellUs::new(duration),
                advance: Degrees10::new(sched_obs.spark_advance_deg10),
            };
            if let Ok(timed) = state.schedule_ignition(now, start, end, ign_plan) {
                accepted_ignition_advance = Some(timed.plan.advance.get());
            }
        }
    }

    DrivenScheduler {
        surface: observe_scheduler(&state),
        fuel_obs,
        sched_obs,
        accepted_ignition_advance,
    }
}

// --------------------------------------------------------------------------
// Conformance test
// --------------------------------------------------------------------------

fn conformance_test(case: &ecu_test_fixtures::fixture_matrix::FixtureCase) {
    let spec = ecu_test_fixtures::fixture_matrix::oracle_result(*case);
    ecu_test_fixtures::fixture_matrix::assert_fixture_semantics(*case, &spec);

    let driven = run_scheduler_for_case(case);
    let obs = driven.surface;

    assert_drive_matches_oracle(case, &spec, &driven.fuel_obs, &driven.sched_obs);

    let expected = oracle_expectation(&spec);
    let has_fuel = expected.pw_corr_us > 0 && expected.injection_channels > 0;
    let has_ignition = expected.ignition_channels > 0;

    if has_fuel || has_ignition {
        assert_eq!(
            obs.mode,
            SchedulerMode::Armed,
            "scheduler should be armed when the oracle expects any event"
        );

        if expected.injection_channels > 0 {
            assert!(
                obs.active_groups & OutputGroup::Injector.mask() != 0,
                "Injector group should be active when the oracle expects injection events"
            );
            assert!(
                obs.reserved_channels[0] != 0,
                "Injector channel should be reserved"
            );
        }
        if has_ignition {
            assert!(
                obs.active_groups & OutputGroup::Ignition.mask() != 0,
                "Ignition group should be active when the oracle expects ignition events"
            );
            assert!(
                obs.reserved_channels[1] != 0,
                "Ignition channel should be reserved"
            );
        }

        assert_eq!(
            obs.injection_count as usize, expected.injection_channels,
            "injection_count should match the oracle injection channel count"
        );
        assert_eq!(
            obs.ignition_count as usize, expected.ignition_channels,
            "ignition_count should match the oracle ignition channel count"
        );

        // Reserved windows must carry the oracle's durations.
        if expected.injection_channels > 0 {
            let start = obs
                .last_injection_start
                .expect("injection start should be recorded");
            let end = obs
                .last_injection_end
                .expect("injection end should be recorded");
            assert_eq!(
                end.get() - start.get(),
                expected.pw_corr_us.max(1),
                "injection window duration should equal the oracle pulse width"
            );
        }
        if has_ignition {
            let start = obs
                .last_ignition_start
                .expect("ignition start should be recorded");
            let end = obs
                .last_ignition_end
                .expect("ignition end should be recorded");
            assert_eq!(
                end.get() - start.get(),
                expected.dwell_us.max(1),
                "ignition window duration should equal the oracle dwell"
            );
            assert_eq!(
                driven.accepted_ignition_advance,
                Some(expected.spark_advance_deg10),
                "accepted ignition plan should carry the oracle spark advance"
            );
        }
    } else {
        assert_eq!(
            obs.mode,
            SchedulerMode::Idle,
            "scheduler should be idle when the oracle expects no events"
        );
        assert_eq!(obs.injection_count, 0);
        assert_eq!(obs.ignition_count, 0);
    }

    // --- Cancellation/suspend observability ---
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

    cancel_state.cancel_group(OutputGroup::Injector);
    assert_eq!(
        cancel_state.active_groups() & OutputGroup::Injector.mask(),
        0
    );

    cancel_state.cancel_all();
    assert_eq!(cancel_state.mode(), SchedulerMode::Idle);
    assert_eq!(cancel_state.active_groups(), 0);

    let _ = cancel_state.schedule_injection(now, start, end, test_inj);
    cancel_state.suspend();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);

    cancel_state.on_sync_loss();
    assert_eq!(cancel_state.mode(), SchedulerMode::Suspended);

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
