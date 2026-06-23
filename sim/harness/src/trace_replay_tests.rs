use super::*;
use crate::SimulationHarness;
use ecu_domain::{FaultCode, Kpa10};
use ecu_io::{EdgeSample, SensorFrame};

fn sensor_frame(time_us: u32, rpm: u16, map_kpa10: u16, angle_x10: i16) -> SensorFrame {
    SensorFrame {
        at_us: Micros::new(time_us),
        rpm: Rpm::new(rpm),
        map_kpa10: Kpa10::new(map_kpa10),
        maf_x100: ecu_domain::MassAirFlowX100::new(0),
        maf_valid: false,
        knock_x100: ecu_domain::KnockLevelX100::new(0),
        knock_valid: false,
        cam_phase_deg10: None,
        angle_x10: Degrees10::new(angle_x10),
        tps_x100: 1000,
        clt_c10: 800,
        iat_c10: 300,
        vbatt_mv: 12400,
        baro_kpa10: Kpa10::new(1000),
        vehicle_speed_kph10: ecu_domain::VehicleSpeedKph10::new(0),
        vehicle_speed_valid: false,
        lambda_valid: false,
        lambda_x100: ecu_domain::Lambda100::new(100),
    }
}

/// Helper: make a TraceRecord for an edge event.
fn make_edge_record(
    step_index: u32,
    time_us: u32,
    line: EdgeLine,
    polarity: EdgePolarity,
    rpm: u16,
    angle_x10: i16,
    synced: bool,
) -> TraceRecord {
    TraceRecord::edge(
        step_index,
        Micros::new(time_us),
        EdgeSample {
            at_us: Micros::new(time_us),
            line,
            polarity,
            angle_x10: Degrees10::new(angle_x10),
            rpm: Rpm::new(rpm),
        },
        Rpm::new(rpm),
        synced,
        0,
        Degrees10::new(angle_x10),
        FaultCode::None,
    )
}

/// Helper: make a TraceRecord for a sensor frame.
fn make_sensor_record(
    step_index: u32,
    time_us: u32,
    rpm: u16,
    map_kpa10: u16,
    angle_x10: i16,
) -> TraceRecord {
    TraceRecord::sensor(
        step_index,
        Micros::new(time_us),
        sensor_frame(time_us, rpm, map_kpa10, angle_x10),
        Rpm::new(rpm),
        true,
        0,
        Degrees10::new(angle_x10),
        FaultCode::None,
    )
}

/// Helper: make a TraceRecord for a tick event.
fn make_tick_record(step_index: u32, time_us: u32, rpm: u16) -> TraceRecord {
    TraceRecord::tick(
        step_index,
        Micros::new(time_us),
        sensor_frame(time_us, rpm, 600, 0),
        Rpm::new(rpm),
        true,
        0,
        Degrees10::new(0),
        FaultCode::None,
    )
}

/// Helper: make a TraceRecord for a None input.
fn make_none_record(step_index: u32, time_us: u32) -> TraceRecord {
    TraceRecord::none(
        step_index,
        Micros::new(time_us),
        Rpm::new(0),
        false,
        0,
        Degrees10::new(0),
        FaultCode::None,
    )
}

#[test]
fn replay_rejects_nonmonotonic_time() {
    let sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let mut replay = TraceReplay::new(sim);

    let record1 = make_edge_record(
        0,
        100,
        EdgeLine::Crank,
        EdgePolarity::Rising,
        1200,
        0,
        false,
    );
    let record2 = make_edge_record(
        1,
        50, // Time going backwards!
        EdgeLine::Crank,
        EdgePolarity::Rising,
        1200,
        60,
        false,
    );

    assert!(replay.replay_one(&record1).is_ok());
    let result = replay.replay_one(&record2);
    assert!(matches!(result, Err(ReplayError::NonMonotonicTime)));
}

#[test]
fn replay_validates_record_consistency() {
    let sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let mut replay = TraceReplay::new(sim);

    let invalid_record = TraceRecord::output(
        0,
        Micros::new(100),
        ecu_io::OutputTransition {
            at_us: Micros::new(100),
            kind: ecu_io::OutputTransitionKind::Injector,
            channel: ecu_domain::ChannelId::new(0),
            level: ecu_io::OutputLevel::High,
        },
        Rpm::new(1200),
        false,
        0,
        Degrees10::new(0),
        FaultCode::None,
    );

    let result = replay.replay_one(&invalid_record);
    assert!(matches!(result, Err(ReplayError::UnsupportedRecord)));
}

#[test]
fn replay_ignores_none_input_with_none_payload() {
    let sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let mut replay = TraceReplay::new(sim);

    let record = make_none_record(0, 100);
    assert!(replay.replay_one(&record).is_ok());
}

#[test]
fn replay_feeds_edge_records_as_edges() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let mut replay = TraceReplay::new(&mut sim);

    let crank_record = make_edge_record(
        0,
        100,
        EdgeLine::Crank,
        EdgePolarity::Rising,
        1200,
        0,
        false,
    );
    let cam_record = make_edge_record(1, 102, EdgeLine::Cam, EdgePolarity::Rising, 1200, 0, true);

    assert!(replay.replay_one(&crank_record).is_ok());
    assert!(replay.replay_one(&cam_record).is_ok());

    // Drain and verify
    let count = sim.drain_until_idle();
    assert_eq!(count, 2);
}

#[test]
fn replay_feeds_sensor_records_as_sensor_frames() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let mut replay = TraceReplay::new(&mut sim);

    let record = make_sensor_record(0, 100, 1800, 600, 0);
    assert!(replay.replay_one(&record).is_ok());

    sim.drain_until_idle();
    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.rpm.get(), 1800);
}

#[test]
fn replay_target_rejects_map_values_that_would_truncate() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let err = ReplayTarget::sensor_frame(
        &mut sim,
        Micros::new(100),
        Rpm::new(1800),
        u16::MAX as u32 + 1,
        Degrees10::new(0),
    )
    .unwrap_err();

    assert_eq!(
        err,
        ReplayError::MapOutOfRange {
            load_kpa10: u16::MAX as u32 + 1,
        }
    );
}

#[test]
fn replay_feeds_tick_records_as_ticks() {
    let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
    let mut replay = TraceReplay::new(&mut sim);

    let record = make_tick_record(0, 100, 1200);
    assert!(replay.replay_one(&record).is_ok());

    sim.drain_until_idle();
    let result = sim.last_result();
    assert!(result.is_some());
}

#[test]
fn replay_same_trace_twice_produces_identical_results() {
    // Create a simple trace
    let trace: &[TraceRecord] = &[
        make_edge_record(
            0,
            100,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            0,
            false,
        ),
        make_edge_record(
            1,
            160,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            60,
            false,
        ),
        make_sensor_record(2, 170, 1200, 600, 60),
        make_tick_record(3, 200, 1200),
    ];

    // First replay
    let mut sim1: SimulationHarness<8, 4> = SimulationHarness::default();
    let mut replay1 = TraceReplay::new(&mut sim1);
    replay1.replay_all(trace).unwrap();
    sim1.drain_until_idle();

    // Get first result
    let result1 = sim1.last_result();

    // Second replay
    let mut sim2: SimulationHarness<8, 4> = SimulationHarness::default();
    let mut replay2 = TraceReplay::new(&mut sim2);
    replay2.replay_all(trace).unwrap();
    sim2.drain_until_idle();

    // Get second result
    let result2 = sim2.last_result();

    // Results should be identical
    assert_eq!(result1, result2);
}

#[test]
fn golden_trace_replay_matches_expected_runtime_snapshot() {
    let golden_trace: &[TraceRecord] = &[
        make_edge_record(
            0,
            100,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            0,
            false,
        ),
        make_sensor_record(1, 150, 1800, 650, 120),
        make_tick_record(2, 200, 1800),
    ];
    let mut sim: SimulationHarness<8, 4> = SimulationHarness::default();
    let mut replay = TraceReplay::new(&mut sim);

    replay.replay_all(golden_trace).unwrap();
    sim.drain_until_idle();

    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.rpm, Rpm::new(1800));
    assert_eq!(snapshot.engine.load_kpa10, Kpa10::new(650));
    assert_eq!(snapshot.engine.angle_x10, Degrees10::new(120));
    assert!(sim.last_result().is_some());
}

#[test]
fn unsupported_record_combinations_return_unsupported_record() {
    // Test: Edge input with None payload
    {
        assert!(TraceRecord::try_from_raw(
            0,
            Micros::new(100),
            TraceInputKind::Edge,
            TracePayload::None,
            Rpm::new(0),
            false,
            0,
            Degrees10::new(0),
            FaultCode::None,
        )
        .is_err());
    }

    // Test: Sensor input with Edge payload
    {
        assert!(TraceRecord::try_from_raw(
            0,
            Micros::new(100),
            TraceInputKind::Sensor,
            TracePayload::Edge(EdgeSample {
                at_us: Micros::new(100),
                line: EdgeLine::Crank,
                polarity: EdgePolarity::Rising,
                angle_x10: Degrees10::new(0),
                rpm: Rpm::new(1200),
            }),
            Rpm::new(1200),
            false,
            0,
            Degrees10::new(0),
            FaultCode::None,
        )
        .is_err());
    }

    // Test: Tick input with None payload
    {
        assert!(TraceRecord::try_from_raw(
            0,
            Micros::new(100),
            TraceInputKind::Tick,
            TracePayload::None,
            Rpm::new(0),
            false,
            0,
            Degrees10::new(0),
            FaultCode::None,
        )
        .is_err());
    }
}

#[test]
fn validate_record_accepts_valid_combinations() {
    // None input + None payload
    {
        let record = make_none_record(0, 100);
        assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
    }

    // Edge input + Edge payload
    {
        let record = make_edge_record(
            0,
            100,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            0,
            false,
        );
        assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
    }

    // Sensor input + Sensor payload
    {
        let record = make_sensor_record(0, 100, 1800, 600, 0);
        assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
    }

    // Tick input + Sensor payload
    {
        let record = make_tick_record(0, 100, 1200);
        assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
    }
}

#[test]
fn replay_preserves_order_for_equal_timestamps() {
    let mut sim: SimulationHarness<8, 4> = SimulationHarness::default();
    let mut replay = TraceReplay::new(&mut sim);

    // Create multiple records with same timestamp
    let records = &[
        make_edge_record(
            0,
            100,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            0,
            false,
        ),
        make_edge_record(1, 100, EdgeLine::Cam, EdgePolarity::Rising, 1200, 0, true),
        make_sensor_record(2, 100, 1200, 600, 0),
    ];

    // All should succeed - order is preserved
    for record in records {
        assert!(replay.replay_one(record).is_ok());
    }

    // All events should be in the queue
    let count = sim.drain_until_idle();
    assert_eq!(count, 3);
}

#[test]
fn sixty_minus_two_generated_trace_replays_into_runtime_harness() {
    use crate::trigger_pattern::{MissingToothEdgeGenerator, MissingToothPattern};

    // Generate a 60-2 trace for exactly one revolution.
    // The 60-2 pattern produces 60 emitted edges per revolution:
    // 1 cam edge (at cam_phase_deg10=0) + 59 crank edges.
    // Using a 60-element array ensures we stop at the revolution boundary
    // before any GAP_EXIT re-emits tooth 0 at time=0 (which would violate
    // monotonic time ordering in the trace).
    let pattern = MissingToothPattern::sixty_minus_two();
    let mut gen = MissingToothEdgeGenerator::new(pattern).unwrap();
    gen.set_rpm(Rpm::new(1000)).unwrap();

    let mut trace = smallvec::SmallVec::<[TraceRecord; 64]>::new();
    let edges = core::array::from_fn::<_, 60, _>(|_| gen.next_edge().unwrap());

    for (i, edge) in edges.iter().copied().enumerate() {
        let record = TraceRecord::edge(
            i as u32,
            edge.at_us,
            edge,
            edge.rpm,
            false,
            0,
            edge.angle_x10,
            FaultCode::None,
        );
        trace.push(record);
    }

    // Replay into runtime harness
    let mut sim: SimulationHarness<128, 16> = SimulationHarness::default();
    let mut replay = TraceReplay::new(&mut sim);

    // Replay one by one to find which edge fails
    for (i, record) in trace.iter().enumerate() {
        if replay.replay_one(record).is_err() {
            panic!("Replay failed at edge {}", i);
        }
    }

    // Check queue state after replay

    // All 60 records should have been processed
    let processed = sim.drain_until_idle();
    assert_eq!(processed, 2, "Coalescing queue stores 2 coalesced events");

    // Runtime should have processed edges (RPM should be set)
    let snapshot = sim.runtime().snapshot();
    assert_eq!(snapshot.engine.rpm.get(), 1000);
}
