use super::*;
use crate::safety::{SafetyGateInput, SafetyGateReason, SafetyGateStatus, SafetyPermitMask};
use crate::timing_island::{
    EdgeBatch, TimingIslandCommand, TimingIslandEvent, TimingIslandFaultStatus,
    TimingIslandRejectReason, TriggerEdge,
};
use crate::wire::{
    decode_safety_gate_input, decode_safety_gate_status, decode_timing_island_command,
    decode_timing_island_event, encode_safety_gate_input, encode_safety_gate_status,
    encode_timing_island_command, encode_timing_island_event, TimingIslandCodecError,
    SAFETY_GATE_INPUT_WIRE_LEN, SAFETY_GATE_STATUS_WIRE_LEN, TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT,
    TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG, TIMING_ISLAND_COMMAND_WIRE_LEN,
    TIMING_ISLAND_EVENT_TAG_REJECTED, TIMING_ISLAND_EVENT_WIRE_LEN, TIMING_ISLAND_WIRE_VERSION,
};
use core::mem::needs_drop;
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, ControlMode, CrankSyncState, Degrees10,
    DwellUs, EnginePhase, EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10, Lambda100, Micros,
    Percent, PhaseSyncState, PulseWidthUs, Rpm, SyncState, Ticks,
};

#[test]
fn edge_batch_push_until_full_then_rejects() {
    let mut batch = EdgeBatch::<4>::new();

    for idx in 0..4 {
        assert!(batch
            .push(TriggerEdge::new(EdgeKind::Rising, Ticks::new(idx)))
            .is_ok());
    }

    assert_eq!(batch.len(), 4);
    assert_eq!(
        batch.push(TriggerEdge::new(EdgeKind::Falling, Ticks::new(99))),
        Err(TriggerEdge::new(EdgeKind::Falling, Ticks::new(99)))
    );

    let mut seen = 0;
    for (idx, edge) in batch.iter().enumerate() {
        assert_eq!(edge.kind, EdgeKind::Rising);
        assert_eq!(edge.at, Ticks::new(idx as u32));
        seen += 1;
    }

    assert_eq!(seen, 4);
}

#[test]
fn output_transition_batch_clear_resets_len_but_preserves_capacity() {
    let mut batch = OutputTransitionBatch::<2>::new();

    assert!(batch
        .push(OutputTransition::new(
            EcuOutput::Injector(ChannelId::new(1)),
            OutputLevel::High,
            Ticks::new(10),
        ))
        .is_ok());
    assert!(batch
        .push(OutputTransition::new(
            EcuOutput::Ignition(ChannelId::new(2)),
            OutputLevel::Low,
            Ticks::new(11),
        ))
        .is_ok());

    assert!(batch.push(OutputTransition::EMPTY).is_err());
    batch.clear();

    assert!(batch.is_empty());
    assert_eq!(batch.capacity(), 2);

    assert!(batch
        .push(OutputTransition::new(
            EcuOutput::Injector(ChannelId::new(3)),
            OutputLevel::High,
            Ticks::new(12),
        ))
        .is_ok());
    assert_eq!(batch.len(), 1);
    assert_eq!(batch.as_slice()[0].at, Ticks::new(12));
}

#[test]
fn core_types_are_copy_and_do_not_need_drop() {
    fn assert_copy<T: Copy>() {}

    assert_copy::<TriggerEdge>();
    assert_copy::<OutputTransition>();
    assert_copy::<AuxCommand>();
    assert_copy::<EngineTimeAuthorityTelemetry>();
    assert_copy::<SensorSnapshot>();
    assert_copy::<TelemetryFrame>();
    assert_copy::<EdgeBatch<4>>();
    assert_copy::<OutputTransitionBatch<4>>();
    assert_copy::<AuxCommandBatch<4>>();

    assert!(!needs_drop::<TriggerEdge>());
    assert!(!needs_drop::<OutputTransition>());
    assert!(!needs_drop::<AuxCommand>());
    assert!(!needs_drop::<EngineTimeAuthorityTelemetry>());
    assert!(!needs_drop::<SensorSnapshot>());
    assert!(!needs_drop::<ProfileId>());
    assert!(!needs_drop::<IgnitionProfileId>());
    assert!(!needs_drop::<PinMapId>());
    assert!(!needs_drop::<RuntimeBuildId>());
    assert!(!needs_drop::<IgnitionProfileMode>());
    assert!(!needs_drop::<TelemetryFrame>());
    assert!(!needs_drop::<EdgeBatch<4>>());
    assert!(!needs_drop::<OutputTransitionBatch<4>>());
    assert!(!needs_drop::<AuxCommandBatch<4>>());
}

#[test]
fn aux_output_generic_variants_construct_and_round_trip() {
    let variants = [
        AuxOutput::SafetyRelay(0),
        AuxOutput::SafetyRelay(1),
        AuxOutput::SafetyRelay(7),
        AuxOutput::Indicator(0),
        AuxOutput::Indicator(2),
        AuxOutput::FrequencyOut(0),
        AuxOutput::FrequencyOut(3),
        AuxOutput::Digital(ChannelId::new(7)),
        AuxOutput::Digital(ChannelId::new(8)),
        AuxOutput::Pwm(ChannelId::new(8)),
        AuxOutput::Digital(ChannelId::new(9)),
        AuxOutput::Pwm(ChannelId::new(10)),
    ];

    let mut batch = AuxCommandBatch::<12>::new();
    for (idx, output) in variants.into_iter().enumerate() {
        assert!(batch
            .push(AuxCommand::new(
                output,
                if idx % 2 == 0 {
                    AuxValue::Off
                } else {
                    AuxValue::Level(OutputLevel::High)
                },
            ))
            .is_ok());
    }

    assert_eq!(batch.len(), 12);
    assert_eq!(
        batch.as_slice()[9].output,
        AuxOutput::Pwm(ChannelId::new(8))
    );
    assert_eq!(
        batch.as_slice()[10].output,
        AuxOutput::Digital(ChannelId::new(9))
    );
    assert_eq!(
        batch.as_slice()[11].output,
        AuxOutput::Pwm(ChannelId::new(10))
    );
}

#[test]
fn telemetry_frame_identity_fields_round_trip() {
    let snapshot = SensorSnapshot::new(
        Micros::new(123),
        Rpm::new(456),
        Kpa10::new(789),
        Percent::new(12),
        34,
        56,
        12_345,
        Lambda100::new(98),
        SyncState::Locked { cam_ref: false },
        EnginePhase::Running,
    );

    let frame = TelemetryFrame::new(
        snapshot,
        ProfileId::new(11),
        IgnitionProfileId::new(22),
        IgnitionProfileMode::SequentialCop,
        PinMapId::new(33),
        RuntimeBuildId::new(44),
        ControlMode::ClosedLoop,
        FaultCode::default(),
        FaultSeverity::default(),
        Degrees10::new(15),
        DwellUs::new(2500),
        PulseWidthUs::new(3700),
    );

    assert_eq!(frame.snapshot, snapshot);
    assert_eq!(frame.profile_id, ProfileId::new(11));
    assert_eq!(frame.ignition_profile_id, IgnitionProfileId::new(22));
    assert_eq!(
        frame.ignition_profile_mode,
        IgnitionProfileMode::SequentialCop
    );
    assert_eq!(frame.pin_map_id, PinMapId::new(33));
    assert_eq!(frame.runtime_build_id, RuntimeBuildId::new(44));
    assert_eq!(frame.control_mode, ControlMode::ClosedLoop);
    assert_eq!(
        frame.snapshot.engine_time.source(),
        AbsoluteTimeAuthority::GeometryOnly
    );
    assert!(!frame.snapshot.engine_time.full_sequential_authorized);
}

#[test]
fn telemetry_reports_explicit_engine_time_authority_source_and_gate() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::ExpertManual,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    let snapshot = SensorSnapshot::new_with_engine_time_authority(
        Micros::new(123),
        Rpm::new(456),
        Kpa10::new(789),
        Percent::new(12),
        34,
        56,
        12_345,
        Lambda100::new(98),
        authority,
        EnginePhase::Running,
    );

    assert_eq!(snapshot.sync_state, SyncState::Locked { cam_ref: false });
    assert_eq!(
        snapshot.engine_time.summary,
        SyncState::Locked { cam_ref: false }
    );
    assert_eq!(
        snapshot.engine_time.source(),
        AbsoluteTimeAuthority::ExpertManual
    );
    assert!(snapshot.engine_time.full_sequential_authorized);
}

#[test]
fn geometry_only_authority_does_not_authorize_full_sequential() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    let telemetry = EngineTimeAuthorityTelemetry::new(authority);

    assert_eq!(telemetry.summary, SyncState::Locked { cam_ref: false });
    assert_eq!(telemetry.source(), AbsoluteTimeAuthority::GeometryOnly);
    assert!(!telemetry.full_sequential_authorized);
}

#[test]
fn timing_island_command_wire_round_trips_all_variants() {
    let commands = [
        TimingIslandCommand::ArmOutput(OutputTransition::new(
            EcuOutput::Injector(ChannelId::new(1)),
            OutputLevel::High,
            Ticks::new(100),
        )),
        TimingIslandCommand::ApplyAux(AuxCommand::new(
            AuxOutput::Pwm(ChannelId::new(2)),
            AuxValue::Duty(Percent::new(45)),
        )),
        TimingIslandCommand::CancelAll(CancelReason::SafetyShutdown),
        TimingIslandCommand::FeedWatchdog,
        TimingIslandCommand::UpdatePermitMask(SafetyPermitMask::new(0x55aa_00ff)),
    ];

    for command in commands {
        let mut bytes = [0u8; TIMING_ISLAND_COMMAND_WIRE_LEN];

        assert_eq!(
            encode_timing_island_command(command, &mut bytes),
            Ok(TIMING_ISLAND_COMMAND_WIRE_LEN)
        );
        assert_eq!(decode_timing_island_command(&bytes), Ok(command));
    }
}

#[test]
fn timing_island_event_wire_round_trips_representative_variants() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::CertifiedProfile,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        3,
    );
    let transition = OutputTransition::new(
        EcuOutput::Ignition(ChannelId::new(4)),
        OutputLevel::Low,
        Ticks::new(2_000),
    );
    let events = [
        TimingIslandEvent::OutputArmed(transition),
        TimingIslandEvent::OutputCompleted(transition),
        TimingIslandEvent::AuxApplied(AuxCommand::new(
            AuxOutput::SafetyRelay(0),
            AuxValue::Level(OutputLevel::High),
        )),
        TimingIslandEvent::Cancelled(CancelReason::SyncLoss),
        TimingIslandEvent::Rejected(TimingIslandRejectReason::BackendFault),
        TimingIslandEvent::CrankEdge(TriggerEdge::new(EdgeKind::Rising, Ticks::new(30))),
        TimingIslandEvent::CamEdge(TriggerEdge::new(EdgeKind::Falling, Ticks::new(60))),
        TimingIslandEvent::SyncStatus(EngineTimeAuthorityTelemetry::new(authority)),
        TimingIslandEvent::FaultStatus(TimingIslandFaultStatus::new(
            FaultCode::SafetyCut,
            FaultSeverity::Critical,
            Ticks::new(90),
        )),
    ];

    for event in events {
        let mut bytes = [0u8; TIMING_ISLAND_EVENT_WIRE_LEN];

        assert_eq!(
            encode_timing_island_event(event, &mut bytes),
            Ok(TIMING_ISLAND_EVENT_WIRE_LEN)
        );
        assert_eq!(decode_timing_island_event(&bytes), Ok(event));
    }
}

#[test]
fn safety_gate_wire_round_trips_input_and_status() {
    let input = SafetyGateInput::new(
        Micros::new(1234),
        true,
        true,
        false,
        true,
        false,
        true,
        0x11,
        SafetyPermitMask::new(0x33),
    );
    let status = SafetyGateStatus::denied(
        SafetyGateReason::WatchdogTimeout,
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        0x11,
        Micros::new(1234),
    );
    let mut input_bytes = [0u8; SAFETY_GATE_INPUT_WIRE_LEN];
    let mut status_bytes = [0u8; SAFETY_GATE_STATUS_WIRE_LEN];

    assert_eq!(
        encode_safety_gate_input(input, &mut input_bytes),
        Ok(SAFETY_GATE_INPUT_WIRE_LEN)
    );
    assert_eq!(decode_safety_gate_input(&input_bytes), Ok(input));
    assert_eq!(
        encode_safety_gate_status(status, &mut status_bytes),
        Ok(SAFETY_GATE_STATUS_WIRE_LEN)
    );
    assert_eq!(decode_safety_gate_status(&status_bytes), Ok(status));
}

#[test]
fn wire_codecs_reject_malformed_frames() {
    let mut command = [0u8; TIMING_ISLAND_COMMAND_WIRE_LEN];
    let mut event = [0u8; TIMING_ISLAND_EVENT_WIRE_LEN];
    let mut safety_input = [0u8; SAFETY_GATE_INPUT_WIRE_LEN];
    let mut safety_status = [0u8; SAFETY_GATE_STATUS_WIRE_LEN];

    assert_eq!(
        encode_timing_island_command(TimingIslandCommand::FeedWatchdog, &mut command[..3]),
        Err(TimingIslandCodecError::ShortBuffer)
    );
    assert_eq!(
        decode_timing_island_command(&command[..3]),
        Err(TimingIslandCodecError::ShortBuffer)
    );

    command[0] = TIMING_ISLAND_WIRE_VERSION + 1;
    command[1] = TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG;
    assert_eq!(
        decode_timing_island_command(&command),
        Err(TimingIslandCodecError::UnsupportedVersion(
            TIMING_ISLAND_WIRE_VERSION + 1
        ))
    );

    command[0] = TIMING_ISLAND_WIRE_VERSION;
    command[1] = 0xff;
    assert_eq!(
        decode_timing_island_command(&command),
        Err(TimingIslandCodecError::UnknownTag(0xff))
    );

    command[1] = TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT;
    command[2] = 0xff;
    assert_eq!(
        decode_timing_island_command(&command),
        Err(TimingIslandCodecError::InvalidField)
    );

    event[0] = TIMING_ISLAND_WIRE_VERSION;
    event[1] = TIMING_ISLAND_EVENT_TAG_REJECTED;
    event[2] = 0xff;
    assert_eq!(
        decode_timing_island_event(&event),
        Err(TimingIslandCodecError::InvalidField)
    );

    safety_input[0] = TIMING_ISLAND_WIRE_VERSION;
    safety_input[1] = 0b1100_0000;
    assert_eq!(
        decode_safety_gate_input(&safety_input),
        Err(TimingIslandCodecError::InvalidField)
    );

    safety_status[0] = TIMING_ISLAND_WIRE_VERSION;
    safety_status[1] = 0xff;
    assert_eq!(
        decode_safety_gate_status(&safety_status),
        Err(TimingIslandCodecError::InvalidField)
    );
}

#[test]
fn output_scheduler_safe_state_has_fallible_observable_hook() {
    #[derive(Default)]
    struct Scheduler {
        safe_state_forced: bool,
    }

    impl OutputScheduler<1> for Scheduler {
        type Error = ();

        fn schedule(&mut self, _batch: &OutputTransitionBatch<1>) -> Result<(), Self::Error> {
            Ok(())
        }

        fn cancel_all(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn force_safe_state(&mut self) {
            self.safe_state_forced = true;
        }
    }

    #[derive(Default)]
    struct FallibleScheduler;

    impl OutputScheduler<1> for FallibleScheduler {
        type Error = &'static str;

        fn schedule(&mut self, _batch: &OutputTransitionBatch<1>) -> Result<(), Self::Error> {
            Ok(())
        }

        fn cancel_all(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }

        fn force_safe_state(&mut self) {}

        fn try_force_safe_state(&mut self) -> Result<(), Self::Error> {
            Err("driver fault")
        }
    }

    let mut scheduler = Scheduler::default();
    assert_eq!(scheduler.try_force_safe_state(), Ok(()));
    assert!(scheduler.safe_state_forced);

    let mut fallible = FallibleScheduler;
    assert_eq!(fallible.try_force_safe_state(), Err("driver fault"));
}
