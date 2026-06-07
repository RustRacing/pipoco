use super::*;
use ecu_domain::{Degrees10, KnockLevelX100, Lambda100, MassAirFlowX100, Rpm, VehicleSpeedKph10};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockTime(u32);

impl EcuClock for MockTime {
    fn now_us(&self) -> Micros {
        Micros::new(self.0)
    }
}

fn live_trigger_event(rpm: Rpm, angle_x10: Degrees10) -> SplitLiveTriggerEvent {
    SplitLiveTriggerEvent::new(rpm, angle_x10, true)
}

#[test]
fn split_capture_sample_combines_live_trigger_and_sensor_load() {
    let mut live = SplitLiveInputs::new();
    live.apply_event(live_trigger_event(Rpm::new(2_400), Degrees10::new(450)));

    let sample = split_capture_sample(
        live,
        SplitSensorSignals {
            at_us: Micros::new(20),
            load_kpa10: Kpa10::new(850),
        },
    );

    assert_eq!(sample.at_us, Micros::new(20));
    assert_eq!(sample.rpm, Rpm::new(2_400));
    assert_eq!(sample.load_kpa10, Kpa10::new(850));
    assert_eq!(sample.angle_x10, Degrees10::new(450));
}

#[test]
fn split_capture_sample_stays_zeroed_without_trigger_input() {
    let sample = split_capture_sample(
        SplitLiveInputs::new(),
        SplitSensorSignals {
            at_us: Micros::new(30),
            load_kpa10: Kpa10::new(700),
        },
    );

    assert_eq!(sample.rpm, Rpm::new(0));
    assert_eq!(sample.angle_x10, Degrees10::new(0));
}

#[test]
fn fixed_load_sensor_uses_time_load_and_live_trigger_inputs() {
    let mut sensor = FixedLoadSensor::new(MockTime(123), Kpa10::new(700));
    sensor.apply_live_event(live_trigger_event(Rpm::new(1_900), Degrees10::new(300)));

    let sample = sensor.sample().unwrap();

    assert_eq!(sample.at_us, Micros::new(123));
    assert_eq!(sample.rpm, Rpm::new(1_900));
    assert_eq!(sample.load_kpa10, Kpa10::new(700));
    assert_eq!(sample.angle_x10, Degrees10::new(300));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockLoad(Kpa10);

impl LoadKpa10Source for MockLoad {
    type Error = core::convert::Infallible;

    fn load_kpa10(&mut self) -> Result<Kpa10, Self::Error> {
        Ok(self.0)
    }
}

#[test]
fn live_load_sensor_uses_dynamic_load_and_live_trigger_inputs() {
    let mut sensor = LiveLoadSensor::new(MockTime(456), MockLoad(Kpa10::new(920)));
    sensor.apply_live_event(live_trigger_event(Rpm::new(2_100), Degrees10::new(510)));

    let sample = sensor.sample().unwrap();

    assert_eq!(sample.at_us, Micros::new(456));
    assert_eq!(sample.rpm, Rpm::new(2_100));
    assert_eq!(sample.load_kpa10, Kpa10::new(920));
    assert_eq!(sample.angle_x10, Degrees10::new(510));
}

#[test]
fn live_load_sensor_provides_logical_snapshot_capture() {
    let mut sensor = LiveLoadSensor::new(MockTime(456), MockLoad(Kpa10::new(920)));
    sensor.apply_live_event(live_trigger_event(Rpm::new(2_100), Degrees10::new(510)));

    let capture = sensor.next_snapshot_capture().unwrap().unwrap();

    assert_eq!(capture.at_us, Micros::new(456));
    assert_eq!(capture.angle_x10, Degrees10::new(510));
    assert_eq!(capture.snapshot.rpm, Rpm::new(2_100));
    assert_eq!(capture.snapshot.map_kpa10, Kpa10::new(920));
}

#[test]
fn board_sensor_snapshot_sample_source_forwards_live_events_to_inner_sensor() {
    let mut source =
        BoardSensorSnapshotSampleSource::new(FixedLoadSensor::new(MockTime(789), Kpa10::new(700)));
    source.apply_live_event(live_trigger_event(Rpm::new(1_950), Degrees10::new(330)));

    let sample = source.sample().unwrap();

    assert_eq!(sample.at_us, Micros::new(789));
    assert_eq!(sample.rpm, Rpm::new(1_950));
    assert_eq!(sample.load_kpa10, Kpa10::new(700));
    assert_eq!(sample.angle_x10, Degrees10::new(330));
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockSnapshotSource {
    capture: Option<BoardSensorSnapshotCapture>,
}

impl BoardSensorSnapshotCaptureSource for MockSnapshotSource {
    type Error = core::convert::Infallible;

    fn next_snapshot_capture(&mut self) -> Result<Option<BoardSensorSnapshotCapture>, Self::Error> {
        Ok(self.capture.take())
    }
}

fn full_sensor_snapshot() -> BoardSensorSnapshot {
    BoardSensorSnapshot {
        rpm: Rpm::new(2_300),
        map_kpa10: Kpa10::new(880),
        tps_x100: 2_500,
        clt_c10: 850,
        iat_c10: 290,
        vbatt_mv: 12_400,
        baro_kpa10: Kpa10::new(1_013),
        maf_x100: MassAirFlowX100::new(3_210),
        knock_x100: KnockLevelX100::new(45),
        vehicle_speed_kph10: VehicleSpeedKph10::new(123),
        cam_phase_deg10: Some(ecu_domain::CamPhaseDeg10::new(-50)),
        lambda_x100: Lambda100::new(101),
        validity: BoardSensorValidityFlags::from_channels(true, true, true, true),
    }
}

fn snapshot_capture(snapshot: BoardSensorSnapshot) -> BoardSensorSnapshotCapture {
    BoardSensorSnapshotCapture {
        at_us: Micros::new(777),
        angle_x10: Degrees10::new(120),
        snapshot,
    }
}

#[test]
fn board_sensor_snapshot_sample_source_projects_runtime_load_and_preserves_snapshot() {
    let snapshot = full_sensor_snapshot();
    let mut source = BoardSensorSnapshotSampleSource::new(MockSnapshotSource {
        capture: Some(snapshot_capture(snapshot)),
    });

    let sample = source.sample().unwrap();

    assert_eq!(sample.at_us, Micros::new(777));
    assert_eq!(sample.rpm, snapshot.rpm);
    assert_eq!(sample.load_kpa10, snapshot.map_kpa10);
    assert_eq!(sample.angle_x10, Degrees10::new(120));
    assert_eq!(source.last_snapshot(), Some(snapshot));
    assert_eq!(
        source.last_snapshot().unwrap().maf_x100,
        MassAirFlowX100::new(3_210)
    );
    assert!(source
        .last_snapshot()
        .unwrap()
        .validity
        .contains(BoardSensorValidityFlags::MAF));
    assert_eq!(
        source.last_snapshot().unwrap().cam_phase_deg10,
        Some(ecu_domain::CamPhaseDeg10::new(-50))
    );
}

#[test]
fn board_sensor_snapshot_sample_source_rejects_maf_runtime_load_until_runtime_supports_it() {
    let snapshot = full_sensor_snapshot();
    let mut source = BoardSensorSnapshotSampleSource::with_runtime_load(
        MockSnapshotSource {
            capture: Some(snapshot_capture(snapshot)),
        },
        BoardSensorRuntimeLoad::MafFlow,
    );

    assert_eq!(
        source.sample(),
        Err(
            BoardSensorSnapshotSampleError::RuntimeLoadSourceUnsupported(
                BoardSensorRuntimeLoad::MafFlow
            )
        )
    );
    assert_eq!(source.last_snapshot(), None);
}

#[test]
fn board_sensor_snapshot_sample_source_rejects_invalid_maf_runtime_load() {
    let snapshot = BoardSensorSnapshot {
        validity: BoardSensorValidityFlags::from_channels(false, true, true, true),
        ..full_sensor_snapshot()
    };
    let mut source = BoardSensorSnapshotSampleSource::with_runtime_load(
        MockSnapshotSource {
            capture: Some(snapshot_capture(snapshot)),
        },
        BoardSensorRuntimeLoad::MafFlow,
    );

    assert_eq!(
        source.sample(),
        Err(BoardSensorSnapshotSampleError::MafLoadInvalid)
    );
    assert_eq!(source.last_snapshot(), None);
}

#[test]
fn board_sensor_snapshot_sample_source_rejects_tps_alpha_n_until_runtime_supports_it() {
    let snapshot = full_sensor_snapshot();
    let mut source = BoardSensorSnapshotSampleSource::with_runtime_load(
        MockSnapshotSource {
            capture: Some(snapshot_capture(snapshot)),
        },
        BoardSensorRuntimeLoad::TpsAlphaN,
    );

    assert_eq!(
        source.sample(),
        Err(
            BoardSensorSnapshotSampleError::RuntimeLoadSourceUnsupported(
                BoardSensorRuntimeLoad::TpsAlphaN
            )
        )
    );
    assert_eq!(source.last_snapshot(), None);
}

#[test]
fn board_sensor_snapshot_sample_source_reports_missing_snapshot_without_fabricating_data() {
    let mut source = BoardSensorSnapshotSampleSource::new(MockSnapshotSource { capture: None });

    assert_eq!(
        source.sample(),
        Err(BoardSensorSnapshotSampleError::NoFrame)
    );
    assert_eq!(source.last_snapshot(), None);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct MockFrameSource {
    frame: Option<SensorFrame>,
}

impl SensorFrameSource for MockFrameSource {
    type Error = core::convert::Infallible;

    fn next_frame(&mut self) -> Result<Option<SensorFrame>, Self::Error> {
        Ok(self.frame.take())
    }
}

fn full_sensor_frame() -> SensorFrame {
    SensorFrame {
        at_us: Micros::new(777),
        rpm: Rpm::new(2_300),
        map_kpa10: Kpa10::new(880),
        maf_x100: MassAirFlowX100::new(3_210),
        maf_valid: true,
        knock_x100: KnockLevelX100::new(45),
        knock_valid: true,
        cam_phase_deg10: Some(ecu_domain::CamPhaseDeg10::new(-50)),
        angle_x10: Degrees10::new(120),
        tps_x100: 2_500,
        clt_c10: 850,
        iat_c10: 290,
        vbatt_mv: 12_400,
        baro_kpa10: Kpa10::new(1_013),
        vehicle_speed_kph10: VehicleSpeedKph10::new(123),
        vehicle_speed_valid: true,
        lambda_valid: true,
        lambda_x100: Lambda100::new(101),
    }
}

#[test]
fn sensor_frame_sample_source_adapts_raw_frame_to_snapshot_compatibility_path() {
    let frame = full_sensor_frame();
    let snapshot = board_sensor_snapshot_from_frame(frame);
    let mut source = SensorFrameSampleSource::new(MockFrameSource { frame: Some(frame) });

    let sample = source.sample().unwrap();

    assert_eq!(sample.at_us, frame.at_us);
    assert_eq!(sample.rpm, snapshot.rpm);
    assert_eq!(sample.load_kpa10, snapshot.map_kpa10);
    assert_eq!(sample.angle_x10, frame.angle_x10);
    assert_eq!(source.last_frame(), Some(frame));
    assert_eq!(source.last_snapshot(), Some(snapshot));
}
