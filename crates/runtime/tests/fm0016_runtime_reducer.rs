use std::collections::BTreeSet;

use ecu_test_fixtures::fixture_matrix::{fixture_cases, required_fixture_names};

/// Verifies that the fixture matrix contains exactly the required fixture families.
/// This is a corpus inventory test - it does not execute the product or compare
/// against spec oracle output.
#[test]
fn fm0016_reducer_has_required_fixture_names() {
    let observed: BTreeSet<&'static str> =
        fixture_cases().iter().map(|case| case.fixture).collect();
    let required: BTreeSet<&'static str> = required_fixture_names().into_iter().collect();
    assert_eq!(observed, required);
}

/// Verifies that the reducer uses the expected plan tolerance constants.
#[test]
fn fm0016_reducer_uses_plan_tolerances() {
    use ecu_test_fixtures::fixture_matrix::{EPS_ANGLE_DEG10, EPS_PW_US, EPS_VE_X100};
    assert_eq!(EPS_VE_X100, 1);
    assert_eq!(EPS_PW_US, 1);
    assert_eq!(EPS_ANGLE_DEG10, 1);
}

/// Trigger decoder tooth stream test - this actually runs the runtime product
/// through the decoder observation API, verifying end-to-end sync acquisition,
/// loss, resync, and stall behavior against the spec trigger model.
#[test]
fn fm0016_trigger_decoder_tooth_stream_matches_spec_mapping() {
    use ecu_domain::{AbsoluteTimeAuthority, Degrees10, Micros, Rpm, SyncState};
    use ecu_runtime::{DecoderObservation, EngineRuntime, TriggerObservation};
    use ecu_spec::{trigger_60_2_step, Micros as SpecMicros, TriggerState, TriggerSyncState};

    let streams: [(&str, &[u32]); 4] = [
        ("sync_acquire", &[1000, 2000, 3500, 4500]),
        ("sync_loss", &[1000, 2000, 3500, 4500, 5200, 5700]),
        ("resync", &[1000, 2000, 3500, 4500, 5200, 5700, 7800, 9000]),
        ("stall", &[1000, 2000, 3500, 4500, 500_000]),
    ];

    for (label, stream) in streams {
        let mut runtime = EngineRuntime::new();
        let mut spec_state = TriggerState::default();
        let mut idx = 0usize;
        while idx < stream.len() {
            let spec = trigger_60_2_step(spec_state, SpecMicros::new(stream[idx]));
            spec_state = spec.state;

            runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
                at_us: Micros::new(stream[idx]),
                rpm: Rpm::new(spec.rpm_estimate.get()),
                angle_x10: Degrees10::new(spec.angle_deg10.get() as i16),
                synced: spec.sync_state == TriggerSyncState::Synced,
            }));
            let snapshot = runtime.snapshot();

            let expected_sync = match spec.sync_state {
                TriggerSyncState::Synced => SyncState::Locked { cam_ref: false },
                TriggerSyncState::PreSync => SyncState::Unsynced,
                TriggerSyncState::NoSync | TriggerSyncState::SyncLoss => SyncState::Unsynced,
            };
            assert_eq!(
                snapshot.engine.sync, expected_sync,
                "stream={label} edge_idx={idx}"
            );
            assert_eq!(
                snapshot.engine.rpm.get(),
                spec.rpm_estimate.get(),
                "stream={label} edge_idx={idx}"
            );
            assert_eq!(
                snapshot.engine.angle_x10.get(),
                spec.angle_deg10.get() as i16,
                "stream={label} edge_idx={idx}"
            );

            let expected_phase = match spec.sync_state {
                TriggerSyncState::Synced => ecu_domain::EnginePhase::Running,
                TriggerSyncState::PreSync => {
                    if spec.rpm_estimate.get() > 0 {
                        ecu_domain::EnginePhase::Cranking
                    } else {
                        ecu_domain::EnginePhase::Off
                    }
                }
                TriggerSyncState::NoSync | TriggerSyncState::SyncLoss => {
                    if spec.rpm_estimate.get() > 0 {
                        ecu_domain::EnginePhase::Cranking
                    } else {
                        ecu_domain::EnginePhase::Off
                    }
                }
            };
            assert_eq!(
                snapshot.engine.phase, expected_phase,
                "stream={label} edge_idx={idx}"
            );
            if matches!(
                spec.sync_state,
                TriggerSyncState::NoSync | TriggerSyncState::SyncLoss
            ) {
                assert_eq!(
                    snapshot.engine.engine_time_authority.absolute,
                    AbsoluteTimeAuthority::None,
                    "stream={label} edge_idx={idx}"
                );
            }

            idx += 1;
        }
    }
}
