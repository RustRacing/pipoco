use super::*;
use crate::adapter::{
    BoardAdapter, CommonObservabilityRecord, CommonObservabilityRecordKind,
    CommonObservabilityRecordTraceOverflow, CommonObservabilityTraceOverflow,
    FixedCommonObservabilityTrace, FixedCommonObservabilityTracePair,
};
use crate::noop::{NoopStore, NoopTransport, NoopWatchdog};
use crate::sensor_sample::FixedLoadSensor;
use ecu_board_api::CaptureSampleSource;
use ecu_domain::{Kpa10, PhaseSyncState};
use ecu_runtime::{Action, ActionExecutor};

#[derive(Debug, Clone, Copy)]
struct MockTime;

impl EcuClock for MockTime {
    fn now_us(&self) -> Micros {
        Micros::new(0)
    }
}

#[test]
fn trigger_adapter_keeps_repeated_unsynced_edges_below_crank_synced() {
    let mut adapter = SplitTriggerAdapter::new(MockTime);

    let first = adapter.on_trigger_edge(1_000);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);
    assert!(matches!(
        first,
        BoardEvent::TriggerEdge { synced: false, .. }
    ));

    let second = adapter.on_trigger_edge(2_000);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);
    assert!(matches!(
        second,
        BoardEvent::TriggerEdge { synced: false, .. }
    ));

    let third = adapter.on_trigger_edge(3_000);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);
    assert!(matches!(
        third,
        BoardEvent::TriggerEdge { synced: false, .. }
    ));

    let fourth = adapter.on_trigger_edge(4_000);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);
    assert!(matches!(
        fourth,
        BoardEvent::TriggerEdge { synced: false, .. }
    ));
}

#[test]
fn trigger_adapter_rejects_invalid_profile_without_panicking() {
    let mut profile = default_trigger_profile();
    profile.pattern = TriggerPattern::MissingTooth {
        nominal_teeth: 2,
        missing_teeth: 2,
    };

    assert!(matches!(
        SplitTriggerAdapter::with_profile(MockTime, profile),
        Err(TriggerValidationError::MissingTeethNotLessThanNominal)
    ));
}

#[test]
fn trigger_adapter_reset_sync_clears_state_and_allows_reacquisition() {
    let mut adapter = SplitTriggerAdapter::new(MockTime);

    let _ = adapter.on_trigger_edge(10_000);
    let _ = adapter.on_trigger_edge(11_000);
    let event = adapter.on_trigger_edge(13_000);

    assert_eq!(
        event,
        BoardEvent::TriggerEdge {
            at_us: Micros::new(13_000),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: adapter.authority(),
            synced: true,
        }
    );
    assert_eq!(adapter.rpm(), Rpm::new(1_000));
    assert!(adapter.synced());
    assert_eq!(adapter.sync_state(), SplitSyncState::CrankSynced);
    adapter.reset_sync();

    assert_eq!(adapter.rpm(), Rpm::new(0));
    assert!(!adapter.synced());
    assert_eq!(adapter.sync_state(), SplitSyncState::NoSignal);

    let _ = adapter.on_trigger_edge(20_000);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);

    let _ = adapter.on_trigger_edge(21_000);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);

    let reacquired = adapter.on_trigger_edge(23_000);
    assert_eq!(
        reacquired,
        BoardEvent::TriggerEdge {
            at_us: Micros::new(23_000),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: adapter.authority(),
            synced: true,
        }
    );
    assert!(adapter.synced());
    assert_eq!(adapter.sync_state(), SplitSyncState::CrankSynced);
}

#[test]
fn trigger_adapter_angle_advances_from_decoder_estimate() {
    let mut adapter = SplitTriggerAdapter::new(MockTime);

    let _ = adapter.on_trigger_edge(1_000);
    let _ = adapter.on_trigger_edge(2_000);
    let _ = adapter.on_trigger_edge(4_000);
    let angle = adapter.angle_x10(18_500);

    assert_eq!(angle, Degrees10::new(900));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockActions;

impl ActionExecutor for MockActions {
    type Error = core::convert::Infallible;

    fn execute(&mut self, _action: Action) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_updates_sensor_live_state_before_runtime_event() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        MockActions,
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();
    let sample = board.sensor().sample().unwrap();

    assert_eq!(sample.rpm, Rpm::new(1_000));
    assert_eq!(sample.angle_x10, Degrees10::new(0));
    assert_eq!(sample.load_kpa10, Kpa10::new(700));
    assert!(trigger.synced());
    assert_eq!(
        board.runtime().snapshot().engine.engine_time_authority,
        trigger.authority()
    );
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_record_stores_trigger_record() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    apply_trigger_timestamp_to_runtime_adapter_and_record(
        &mut board,
        &mut trigger,
        1_000,
        &mut trace,
    )
    .unwrap();

    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
    );
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_record_updates_sensor_live_state_before_runtime_event(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    apply_trigger_timestamp_to_runtime_adapter_and_record(
        &mut board,
        &mut trigger,
        1_000,
        &mut trace,
    )
    .unwrap();
    apply_trigger_timestamp_to_runtime_adapter_and_record(
        &mut board,
        &mut trigger,
        2_000,
        &mut trace,
    )
    .unwrap();
    apply_trigger_timestamp_to_runtime_adapter_and_record(
        &mut board,
        &mut trigger,
        4_000,
        &mut trace,
    )
    .unwrap();
    let sample = board.sensor().sample().unwrap();

    assert_eq!(sample.rpm, Rpm::new(1_000));
    assert_eq!(sample.angle_x10, Degrees10::new(0));
    assert_eq!(sample.load_kpa10, Kpa10::new(700));
    assert!(trigger.synced());
    assert_eq!(
        board.runtime().snapshot().engine.engine_time_authority,
        trigger.authority()
    );
    assert_eq!(trace.len(), 3);
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_record_reports_overflow_after_runtime_event_applies(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
        .unwrap();

    let err = apply_trigger_timestamp_to_runtime_adapter_and_record(
        &mut board,
        &mut trigger,
        4_000,
        &mut trace,
    )
    .unwrap_err();

    assert!(matches!(err, ApplyAndRecordError::Record(_)));
    assert_eq!(trace.len(), 1);
    assert!(trigger.synced());
    assert_eq!(
        board.runtime().snapshot().engine.engine_time_authority,
        trigger.authority()
    );
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_pair_stores_matching_pair() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    apply_trigger_timestamp_to_runtime_adapter_and_push_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap();

    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(board.observability_sample()));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
    );
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_pair_updates_sensor_live_state_before_runtime_event(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    apply_trigger_timestamp_to_runtime_adapter_and_push_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap();
    apply_trigger_timestamp_to_runtime_adapter_and_push_pair(
        &mut board,
        &mut trigger,
        2_000,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap();
    apply_trigger_timestamp_to_runtime_adapter_and_push_pair(
        &mut board,
        &mut trigger,
        4_000,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap();
    let sample = board.sensor().sample().unwrap();

    assert_eq!(sample.rpm, Rpm::new(1_000));
    assert_eq!(sample.angle_x10, Degrees10::new(0));
    assert_eq!(sample.load_kpa10, Kpa10::new(700));
    assert!(trigger.synced());
    assert_eq!(
        board.runtime().snapshot().engine.engine_time_authority,
        trigger.authority()
    );
    assert_eq!(sample_trace.len(), 3);
    assert_eq!(record_trace.len(), 3);
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_pair_reports_record_overflow_before_sample_push(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
        .unwrap();

    let err = apply_trigger_timestamp_to_runtime_adapter_and_push_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        TriggerAndPushPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    ));
    assert_eq!(sample_trace.len(), 0);
    assert_eq!(record_trace.len(), 1);
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_pair_reports_sample_overflow_after_record_push(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    sample_trace.push(board.observability_sample()).unwrap();

    let err = apply_trigger_timestamp_to_runtime_adapter_and_push_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        TriggerAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    ));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
    );
    assert_eq!(sample_trace.len(), 1);
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair_stores_matching_pair() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut traces,
    )
    .unwrap();

    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(board.observability_sample()));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
    );
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair_updates_sensor_live_state_before_runtime_event(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut traces,
    )
    .unwrap();
    apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        &mut trigger,
        2_000,
        &mut traces,
    )
    .unwrap();
    apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        &mut trigger,
        4_000,
        &mut traces,
    )
    .unwrap();
    let sample = board.sensor().sample().unwrap();

    assert_eq!(sample.rpm, Rpm::new(1_000));
    assert_eq!(sample.angle_x10, Degrees10::new(0));
    assert_eq!(sample.load_kpa10, Kpa10::new(700));
    assert!(trigger.synced());
    assert_eq!(
        board.runtime().snapshot().engine.engine_time_authority,
        trigger.authority()
    );
    assert_eq!(traces.sample().len(), 3);
    assert_eq!(traces.record().len(), 3);
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair_reports_record_overflow_before_sample_push(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();

    traces
        .record_mut()
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
        .unwrap();

    let err = apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut traces,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        TriggerAndPushPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    ));
    assert_eq!(traces.sample().len(), 0);
    assert_eq!(traces.record().len(), 1);
}

#[test]
fn apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair_reports_sample_overflow_after_record_push(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();

    traces
        .sample_mut()
        .push(board.observability_sample())
        .unwrap();

    let err = apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        &mut trigger,
        1_000,
        &mut traces,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        TriggerAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    ));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
    );
    assert_eq!(traces.sample().len(), 1);
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_record_stores_cam_record() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();

    apply_cam_observation_to_runtime_adapter_and_record(&mut board, 5_000, true, &mut trace)
        .unwrap();

    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: board.observability_sample(),
        })
    );
    assert_eq!(
        board
            .runtime()
            .snapshot()
            .engine
            .engine_time_authority
            .phase,
        PhaseSyncState::CamObserved720
    );
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_record_reports_overflow_after_runtime_event_applies(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut trigger = SplitTriggerAdapter::new(MockTime);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();
    trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
        .unwrap();

    let err =
        apply_cam_observation_to_runtime_adapter_and_record(&mut board, 5_000, true, &mut trace)
            .unwrap_err();

    assert!(matches!(
        err,
        CamObservationAndRecordError::Record(CommonObservabilityRecordTraceOverflow {
            capacity: 1
        })
    ));
    assert_eq!(trace.len(), 1);
    assert_eq!(
        board
            .runtime()
            .snapshot()
            .engine
            .engine_time_authority
            .phase,
        PhaseSyncState::CamObserved720
    );
    assert!(trigger.synced());
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_push_pair_stores_matching_pair() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();

    apply_cam_observation_to_runtime_adapter_and_push_pair(
        &mut board,
        5_000,
        true,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap();

    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(board.observability_sample()));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: board.observability_sample(),
        })
    );
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_push_pair_leaves_sample_trace_untouched_on_record_overflow(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
        .unwrap();

    let err = apply_cam_observation_to_runtime_adapter_and_push_pair(
        &mut board,
        5_000,
        true,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        CamObservationAndPushPairError::Record(CommonObservabilityRecordTraceOverflow {
            capacity: 1
        })
    ));
    assert_eq!(sample_trace.len(), 0);
    assert_eq!(record_trace.len(), 1);
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_push_pair_returns_after_record_entry_is_stored_on_sample_overflow(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();
    sample_trace.push(board.observability_sample()).unwrap();

    let err = apply_cam_observation_to_runtime_adapter_and_push_pair(
        &mut board,
        5_000,
        true,
        &mut sample_trace,
        &mut record_trace,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        CamObservationAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    ));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: board.observability_sample(),
        })
    );
    assert_eq!(sample_trace.len(), 1);
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair_stores_matching_pair() {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();

    apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        5_000,
        true,
        &mut traces,
    )
    .unwrap();

    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(board.observability_sample()));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: board.observability_sample(),
        })
    );
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair_leaves_sample_trace_untouched_on_record_overflow(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();
    traces
        .record_mut()
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: board.observability_sample(),
        })
        .unwrap();

    let err = apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        5_000,
        true,
        &mut traces,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        CamObservationAndPushPairError::Record(CommonObservabilityRecordTraceOverflow {
            capacity: 1
        })
    ));
    assert_eq!(traces.sample().len(), 0);
    assert_eq!(traces.record().len(), 1);
}

#[test]
fn apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair_returns_after_record_entry_is_stored_on_sample_overflow(
) {
    let mut board = BoardAdapter::new(
        FixedLoadSensor::new(MockTime, Kpa10::new(700)),
        crate::noop::NoopCapture,
        ScheduledActionExecutor::<4>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let mut trigger = SplitTriggerAdapter::new(MockTime);

    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 1_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 2_000).unwrap();
    apply_trigger_timestamp_to_runtime_adapter(&mut board, &mut trigger, 4_000).unwrap();
    traces
        .sample_mut()
        .push(board.observability_sample())
        .unwrap();

    let err = apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair(
        &mut board,
        5_000,
        true,
        &mut traces,
    )
    .unwrap_err();

    assert!(matches!(
        err,
        CamObservationAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    ));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: board.observability_sample(),
        })
    );
    assert_eq!(traces.sample().len(), 1);
}
