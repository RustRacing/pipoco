use ecu_board_api::{SensorSnapshot, TriggerEdge};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, CrankSyncState, EnginePhase, Kpa10, Lambda100, Micros,
    Percent, PhaseSyncState, Rpm, SyncState,
};
use ecu_runtime::Action;
use ecu_sim_driver::trigger_replay::{
    m50_dry_crank_fixture_placeholder, replay_frame_time_us, TriggerReplayFixture,
    TriggerReplayFrame,
};
use ecu_sim_driver::x86_runtime_board::{run_x86_runtime_tick, X86RuntimeBoard};
use ecu_trigger::{MissingToothDecoderEvent, SyncLossReason};

#[test]
fn sixty_minus_two_replay_is_deterministic_and_locks_primary() {
    let fixture = TriggerReplayFixture::sixty_minus_two_with_cam();
    let first = fixture.replay().unwrap();
    let second = fixture.replay().unwrap();

    assert_eq!(first.frames, second.frames);
    assert_eq!(
        first.authority_transitions(),
        second.authority_transitions()
    );
    assert!(first.frames.iter().any(|frame| matches!(
        frame.decoder_event,
        Some(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    )));
    assert_eq!(first.final_authority().crank, CrankSyncState::PrimaryLocked);
    assert_eq!(
        first.final_authority().phase,
        PhaseSyncState::CamValidated720
    );
    assert_eq!(
        first.final_authority().absolute,
        AbsoluteTimeAuthority::ExpertManual
    );
    assert_eq!(first.unsupported_cam_edges, 0);
    assert!(first.final_diagnostics.observation.cam_seen);
    assert!(first.reached_full_sequential_authority());
}

#[test]
fn wrong_edge_or_wrong_tooth_count_never_reaches_certified_authority() {
    for fixture in [
        TriggerReplayFixture::wrong_primary_edge_with_certified_profile(),
        TriggerReplayFixture::wrong_tooth_count_after_lock(),
        TriggerReplayFixture::false_gap_after_primary_lock(),
    ] {
        let replay = fixture.replay().unwrap();
        assert!(
            !replay.reached_certified_authority(),
            "{} reached certified authority: {:?}",
            fixture.name,
            replay.authority_transitions()
        );
    }

    let wrong_primary = TriggerReplayFixture::wrong_primary_edge_with_certified_profile()
        .replay()
        .unwrap();
    assert!(!wrong_primary.sync_loss_reasons().is_empty());
    assert!(wrong_primary
        .sync_loss_reasons()
        .iter()
        .all(|reason| *reason == SyncLossReason::UnexpectedPrimaryEdge));
    assert_eq!(
        wrong_primary.final_authority().crank,
        CrankSyncState::PrimarySearching
    );
    assert_eq!(
        wrong_primary.final_authority().phase,
        PhaseSyncState::Unknown
    );

    let wrong_tooth = TriggerReplayFixture::wrong_tooth_count_after_lock()
        .replay()
        .unwrap();
    assert_eq!(
        wrong_tooth.sync_loss_reasons(),
        vec![SyncLossReason::WrongToothCount]
    );
    assert_eq!(
        wrong_tooth.final_authority().crank,
        CrankSyncState::SyncLost
    );

    let false_gap = TriggerReplayFixture::false_gap_after_primary_lock()
        .replay()
        .unwrap();
    let false_gap_frame = false_gap
        .frames
        .iter()
        .find(|frame| frame.sync_loss == Some(SyncLossReason::WrongToothCount))
        .expect("early false gap should drop primary sync");
    assert_eq!(
        false_gap_frame.decoder_event, None,
        "false gap is rejected as sync loss rather than accepted as a valid gap"
    );
    assert_eq!(false_gap_frame.authority().crank, CrankSyncState::SyncLost);
    assert_eq!(
        false_gap.final_diagnostics.last_sync_loss,
        Some(SyncLossReason::WrongToothCount)
    );
    assert!(!false_gap.reached_certified_authority());
}

#[test]
fn false_edge_noise_is_logged_and_does_not_break_primary_lock() {
    let replay = TriggerReplayFixture::false_edge_noise_after_lock()
        .replay()
        .unwrap();
    let ignored = replay
        .frames
        .iter()
        .find(|frame| frame.decoder_event == Some(MissingToothDecoderEvent::IgnoredByFilter))
        .expect("noise edge should be ignored by the decoder filter");

    assert_eq!(replay_frame_time_us(ignored), Micros::new(4_049));
    assert!(replay.sync_loss_reasons().is_empty());
    assert_eq!(
        replay.final_authority().crank,
        CrankSyncState::PrimaryLocked
    );
    assert!(!replay.reached_certified_authority());
}

#[test]
fn missing_cam_never_allows_full_sequential_authority() {
    let replay = TriggerReplayFixture::missing_cam().replay().unwrap();

    assert_eq!(replay.unsupported_cam_edges, 0);
    assert_eq!(
        replay.final_authority().crank,
        CrankSyncState::PrimaryLocked
    );
    assert_eq!(replay.final_authority().phase, PhaseSyncState::CrankOnly360);
    assert_eq!(
        replay.final_authority().absolute,
        AbsoluteTimeAuthority::None
    );
    assert_eq!(
        replay.final_diagnostics.last_sync_loss,
        Some(SyncLossReason::SecondaryTimeout)
    );
    assert!(!replay.reached_full_sequential_authority());
}

#[test]
fn trigger_replay_frame_view_schema_is_stable() {
    assert_eq!(TriggerReplayFrame::VIEWER_SCHEMA_VERSION, 1);
    assert_eq!(
        TriggerReplayFrame::VIEWER_FIELD_NAMES,
        [
            "index",
            "edge",
            "decoder_event",
            "sync_loss",
            "diagnostics",
            "unsupported_cam_edge",
        ]
    );
}

#[test]
fn wrong_cam_level_fixture_is_recorded_without_phase_promotion() {
    let replay = TriggerReplayFixture::wrong_cam_level().replay().unwrap();

    assert_eq!(replay.unsupported_cam_edges, 0);
    assert!(!replay.frames.iter().any(|frame| frame.unsupported_cam_edge));
    assert!(replay
        .sync_loss_reasons()
        .contains(&SyncLossReason::PhaseMismatch));
    assert_eq!(replay.final_authority().phase, PhaseSyncState::CrankOnly360);
    assert_eq!(
        replay.final_authority().absolute,
        AbsoluteTimeAuthority::None
    );
    assert_eq!(
        replay.final_diagnostics.last_sync_loss,
        Some(SyncLossReason::SecondaryTimeout)
    );
    assert!(!replay.reached_full_sequential_authority());
}

#[test]
fn batch_6_software_trigger_fixture_coverage_is_explicit() {
    let expected = [
        "60-2-with-cam",
        "wrong-tooth-count-after-lock",
        "false-gap-after-primary-lock",
        "missing-cam",
        "wrong-cam-level",
        "sync-loss-after-primary-lock",
    ];
    let fixtures = [
        TriggerReplayFixture::sixty_minus_two_with_cam(),
        TriggerReplayFixture::wrong_tooth_count_after_lock(),
        TriggerReplayFixture::false_gap_after_primary_lock(),
        TriggerReplayFixture::missing_cam(),
        TriggerReplayFixture::wrong_cam_level(),
        TriggerReplayFixture::sync_loss_after_primary_lock(),
    ];

    assert_eq!(fixtures.each_ref().map(|fixture| fixture.name), expected);

    let sixty_minus_two = fixtures[0].replay().unwrap();
    assert!(sixty_minus_two.reached_full_sequential_authority());

    let wrong_tooth = fixtures[1].replay().unwrap();
    assert_eq!(
        wrong_tooth.sync_loss_reasons(),
        vec![SyncLossReason::WrongToothCount]
    );

    let false_gap = fixtures[2].replay().unwrap();
    assert_eq!(
        false_gap.sync_loss_reasons(),
        vec![SyncLossReason::WrongToothCount]
    );
    assert!(!false_gap.reached_certified_authority());

    let missing_cam = fixtures[3].replay().unwrap();
    assert_eq!(
        missing_cam.final_diagnostics.last_sync_loss,
        Some(SyncLossReason::SecondaryTimeout)
    );

    let wrong_cam = fixtures[4].replay().unwrap();
    assert!(wrong_cam
        .sync_loss_reasons()
        .contains(&SyncLossReason::PhaseMismatch));

    let sync_loss = fixtures[5].replay().unwrap();
    assert_eq!(sync_loss.final_authority().crank, CrankSyncState::SyncLost);
}

#[test]
fn replay_authority_gates_x86_outputs_before_sync_loss_cancels() {
    let replay = TriggerReplayFixture::sixty_minus_two_with_cam()
        .replay()
        .unwrap();
    let blocked_frame = replay
        .frames
        .iter()
        .find(|frame| {
            frame.authority().has_primary_lock() && !frame.allows_full_sequential_authority()
        })
        .expect("fixture should produce a blocked authority frame");
    let validated_frame = replay
        .frames
        .iter()
        .find(|frame| frame.allows_full_sequential_authority())
        .expect("fixture should produce a validated authority frame");

    let mut board = X86RuntimeBoard::new();
    board.configure_full_cop();

    board.set_clock(replay_frame_time_us(blocked_frame));
    board.set_sensor_snapshot(SensorSnapshot::new_with_engine_time_authority(
        replay_frame_time_us(blocked_frame),
        Rpm::new(3_000),
        Kpa10::new(450),
        Percent::new(12),
        840,
        550,
        12_500,
        Lambda100::new(100),
        blocked_frame.authority(),
        EnginePhase::Running,
    ));
    board
        .set_trigger_edges(&[TriggerEdge::new(
            blocked_frame.edge.kind,
            blocked_frame.edge.at,
        )])
        .unwrap();

    let blocked = run_x86_runtime_tick(&mut board).unwrap();
    assert!(blocked.scheduled_outputs.is_empty());
    assert!(!board.diagnostics().engine_time.full_sequential_authorized);
    assert!(blocked
        .step_result
        .actions
        .iter()
        .any(|action| matches!(action, Action::Idle)));

    board.set_clock(replay_frame_time_us(validated_frame));
    board.set_sensor_snapshot(SensorSnapshot::new_with_engine_time_authority(
        replay_frame_time_us(validated_frame),
        Rpm::new(3_000),
        Kpa10::new(450),
        Percent::new(12),
        840,
        550,
        12_500,
        Lambda100::new(100),
        validated_frame.authority(),
        EnginePhase::Running,
    ));
    board
        .set_trigger_edges(&[TriggerEdge::new(
            validated_frame.edge.kind,
            validated_frame.edge.at,
        )])
        .unwrap();

    let validated = run_x86_runtime_tick(&mut board).unwrap();
    let bridge = ecu_sim_driver::x86_runtime_board::bridge_output_transitions_to_core_frame::<6, 12>(
        &validated.scheduled_outputs,
    );
    assert!(board.diagnostics().engine_time.full_sequential_authorized);
    assert_eq!(bridge.ecu_outputs.injection_events.len(), 6);
    assert_eq!(bridge.ecu_outputs.spark_events.len(), 6);

    let sync_loss_replay = TriggerReplayFixture::sync_loss_after_primary_lock()
        .replay()
        .unwrap();
    let sync_loss_frame = sync_loss_replay
        .frames
        .iter()
        .find(|frame| frame.sync_loss == Some(SyncLossReason::WrongToothCount))
        .expect("fixture should drop sync");
    assert_eq!(
        sync_loss_frame.authority().compatibility_summary(),
        SyncState::Unsynced
    );

    board.set_clock(replay_frame_time_us(sync_loss_frame));
    board.set_sensor_snapshot(SensorSnapshot::new_with_engine_time_authority(
        replay_frame_time_us(sync_loss_frame),
        Rpm::new(0),
        Kpa10::new(0),
        Percent::new(0),
        840,
        550,
        12_500,
        Lambda100::new(100),
        sync_loss_frame.authority(),
        EnginePhase::Off,
    ));
    board.set_trigger_edges(&[]).unwrap();

    let cancel_count_before = board.diagnostics().cancel_all_count;
    let result = run_x86_runtime_tick(&mut board).unwrap();

    assert_eq!(result.sensor_snapshot.sync_state, SyncState::Unsynced);
    assert!(board.diagnostics().cancel_all_count > cancel_count_before);
    assert_eq!(
        board.diagnostics().last_cancel_reason,
        Some(CancelReason::SyncLoss)
    );
    assert_eq!(
        board.diagnostics().scheduled_transition_count,
        validated.scheduled_outputs.len()
    );
    assert!(result
        .step_result
        .actions
        .iter()
        .any(|action| matches!(action, Action::CancelScheduler(CancelReason::SyncLoss))));
}

#[test]
fn m50_dry_crank_placeholder_has_stable_log_format_metadata() {
    let placeholder = m50_dry_crank_fixture_placeholder();

    assert_eq!(placeholder.format_version, 1);
    assert_eq!(placeholder.profile_name, "M50B25TU_FULL_COP");
    assert_eq!(placeholder.primary_pattern, "60-2 crank");
    assert!(placeholder.edges.is_empty());
}
