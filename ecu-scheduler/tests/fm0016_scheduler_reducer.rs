use std::collections::BTreeSet;

#[path = "../../tests/formal/fm0016_fixture_matrix.rs"]
mod fm0016_fixture_matrix;

use ecu_domain::{ChannelId, Degrees10, DwellUs, Micros, PulseWidthUs};
use ecu_scheduler::{ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup, SchedulerState};
use fm0016_fixture_matrix::{
    assert_fixture_semantics, fixture_cases, oracle_result, required_fixture_names,
    EPS_ANGLE_DEG10, EPS_PW_US, EPS_VE_X100,
};

#[test]
fn fm0016_scheduler_reducer_has_required_fixture_names() {
    let cases = fixture_cases();
    let observed: BTreeSet<&'static str> = cases.iter().map(|case| case.fixture).collect();
    let required: BTreeSet<&'static str> = required_fixture_names().into_iter().collect();
    assert_eq!(observed, required);
}

#[test]
fn fm0016_scheduler_reducer_uses_plan_tolerances() {
    assert_eq!(EPS_VE_X100, 1);
    assert_eq!(EPS_PW_US, 1);
    assert_eq!(EPS_ANGLE_DEG10, 1);
}

#[test]
fn fm0016_scheduler_reducer_oracle_replay_asserts_fixture_semantics() {
    for case in fixture_cases() {
        let result = oracle_result(case);
        assert_fixture_semantics(case, &result);
    }
}

#[test]
fn fm0016_scheduler_single_window_api_still_works() {
    let mut state = SchedulerState::new();
    let inj = InjectionPlan {
        output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
        pulse_width: PulseWidthUs::new(3200),
    };
    let ign = IgnitionPlan {
        output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
        dwell: DwellUs::new(2500),
        advance: Degrees10::new(150),
    };
    assert!(state
        .schedule_injection(Micros::new(0), Micros::new(6648), Micros::new(6840), inj)
        .is_ok());
    assert!(state
        .schedule_ignition(Micros::new(0), Micros::new(6900), Micros::new(7050), ign)
        .is_ok());
}
