use super::*;
use ecu_domain::{ChannelId, Micros};
use ecu_io::OutputTransitionKind;
use ecu_io::{OutputLevel, OutputTransition};

#[test]
fn same_timestamp_trigger_edges_keep_deterministic_order() {
    let mut buffer: FixedTriggerEdgeBuffer<4> = FixedTriggerEdgeBuffer::new();

    buffer
        .push_sorted(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Cam,
            SimEdgePolarity::Falling,
            300,
        ))
        .unwrap();
    buffer
        .push_sorted(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Crank,
            SimEdgePolarity::Falling,
            200,
        ))
        .unwrap();
    buffer
        .push_sorted(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Crank,
            SimEdgePolarity::Rising,
            250,
        ))
        .unwrap();
    buffer
        .push_sorted(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Crank,
            SimEdgePolarity::Rising,
            100,
        ))
        .unwrap();

    assert_eq!(
        buffer.get(0),
        Some(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Crank,
            SimEdgePolarity::Rising,
            100,
        ))
    );
    assert_eq!(
        buffer.get(1),
        Some(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Crank,
            SimEdgePolarity::Rising,
            250,
        ))
    );
    assert_eq!(
        buffer.get(2),
        Some(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Crank,
            SimEdgePolarity::Falling,
            200,
        ))
    );
    assert_eq!(
        buffer.get(3),
        Some(SimTriggerEdge::new(
            1_000,
            SimTriggerLine::Cam,
            SimEdgePolarity::Falling,
            300,
        ))
    );
}

#[test]
fn fixed_buffer_overflow_is_explicit() {
    let mut trace: FixedTraceBuffer<1> = FixedTraceBuffer::new();
    assert_eq!(trace.overflow_count(), 0);

    assert!(trace
        .push(SimBoardTraceRecord::new(
            100,
            SimBoardTraceKind::Tick,
            0,
            1,
            0
        ))
        .is_ok());

    let overflow = trace
        .push(SimBoardTraceRecord::new(
            101,
            SimBoardTraceKind::Note,
            1,
            2,
            3,
        ))
        .unwrap_err();

    assert_eq!(overflow, SimBufferOverflow::new(1));
    assert_eq!(trace.overflow_count(), 1);
    assert_eq!(trace.len(), 1);
}

#[test]
fn output_transitions_sort_same_timestamp_deterministically() {
    let mut outputs: FixedOutputQueue<3> = FixedOutputQueue::new();
    for transition in [
        OutputTransition {
            at_us: Micros::new(1_000),
            kind: OutputTransitionKind::Fan,
            channel: ChannelId::new(3),
            level: OutputLevel::High,
        },
        OutputTransition {
            at_us: Micros::new(1_000),
            kind: OutputTransitionKind::Injector,
            channel: ChannelId::new(2),
            level: OutputLevel::High,
        },
        OutputTransition {
            at_us: Micros::new(1_000),
            kind: OutputTransitionKind::Injector,
            channel: ChannelId::new(1),
            level: OutputLevel::Low,
        },
    ] {
        outputs.push_sorted(transition).unwrap();
    }

    assert_eq!(
        outputs.get(0).map(|event| event.kind),
        Some(OutputTransitionKind::Injector)
    );
    assert_eq!(outputs.get(0).map(|event| event.channel.get()), Some(1));
    assert_eq!(
        outputs.get(0).map(|event| event.level),
        Some(OutputLevel::Low)
    );
    assert_eq!(
        outputs.get(1).map(|event| event.kind),
        Some(OutputTransitionKind::Injector)
    );
    assert_eq!(outputs.get(1).map(|event| event.channel.get()), Some(2));
    assert_eq!(
        outputs.get(1).map(|event| event.level),
        Some(OutputLevel::High)
    );
    assert_eq!(
        outputs.get(2).map(|event| event.kind),
        Some(OutputTransitionKind::Fan)
    );
}
