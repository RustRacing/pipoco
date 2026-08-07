//! Wire-level byte codecs for timing island and safety-gate contracts.

use crate::{
    safety::{
        SafetyGateInput, SafetyGatePermit, SafetyGateReason, SafetyGateStatus, SafetyPermitMask,
    },
    telemetry::EngineTimeAuthorityTelemetry,
    timing_island::{
        AuxCommand, AuxOutput, AuxValue, EcuOutput, EdgeKind, OutputLevel, OutputTransition,
        TimingIslandCommand, TimingIslandEvent, TimingIslandFaultStatus, TimingIslandRejectReason,
        TriggerEdge,
    },
};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, CrankSyncState, EngineTimeAuthority, FaultCode,
    FaultSeverity, Micros, Percent, PhaseSyncState, SyncState, Ticks,
};

pub const TIMING_ISLAND_WIRE_VERSION: u8 = 1;
pub const TIMING_ISLAND_COMMAND_WIRE_LEN: usize = 16;
pub const TIMING_ISLAND_EVENT_WIRE_LEN: usize = 16;
pub const SAFETY_GATE_INPUT_WIRE_LEN: usize = 16;
pub const SAFETY_GATE_STATUS_WIRE_LEN: usize = 20;

pub const TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT: u8 = 1;
pub const TIMING_ISLAND_COMMAND_TAG_APPLY_AUX: u8 = 2;
pub const TIMING_ISLAND_COMMAND_TAG_CANCEL_ALL: u8 = 3;
pub const TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG: u8 = 4;
pub const TIMING_ISLAND_COMMAND_TAG_UPDATE_PERMIT_MASK: u8 = 5;

pub const TIMING_ISLAND_EVENT_TAG_OUTPUT_ARMED: u8 = 1;
pub const TIMING_ISLAND_EVENT_TAG_OUTPUT_COMPLETED: u8 = 2;
pub const TIMING_ISLAND_EVENT_TAG_AUX_APPLIED: u8 = 3;
pub const TIMING_ISLAND_EVENT_TAG_CANCELLED: u8 = 4;
pub const TIMING_ISLAND_EVENT_TAG_REJECTED: u8 = 5;
pub const TIMING_ISLAND_EVENT_TAG_CRANK_EDGE: u8 = 6;
pub const TIMING_ISLAND_EVENT_TAG_CAM_EDGE: u8 = 7;
pub const TIMING_ISLAND_EVENT_TAG_SYNC_STATUS: u8 = 8;
pub const TIMING_ISLAND_EVENT_TAG_FAULT_STATUS: u8 = 9;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimingIslandCodecError {
    ShortBuffer,
    UnsupportedVersion(u8),
    UnknownTag(u8),
    InvalidField,
}

pub fn encode_timing_island_command(
    command: TimingIslandCommand,
    out: &mut [u8],
) -> Result<usize, TimingIslandCodecError> {
    let out = fixed_out::<TIMING_ISLAND_COMMAND_WIRE_LEN>(out)?;
    out.fill(0);
    out[0] = TIMING_ISLAND_WIRE_VERSION;
    match command {
        TimingIslandCommand::ArmOutput(transition) => {
            out[1] = TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT;
            encode_output_transition(transition, &mut out[2..10]);
        }
        TimingIslandCommand::ApplyAux(command) => {
            out[1] = TIMING_ISLAND_COMMAND_TAG_APPLY_AUX;
            encode_aux_command(command, &mut out[2..6]);
        }
        TimingIslandCommand::CancelAll(reason) => {
            out[1] = TIMING_ISLAND_COMMAND_TAG_CANCEL_ALL;
            out[2] = encode_cancel_reason(reason);
        }
        TimingIslandCommand::FeedWatchdog => {
            out[1] = TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG;
        }
        TimingIslandCommand::UpdatePermitMask(mask) => {
            out[1] = TIMING_ISLAND_COMMAND_TAG_UPDATE_PERMIT_MASK;
            put_u32(&mut out[2..6], mask.bits());
        }
    }
    Ok(TIMING_ISLAND_COMMAND_WIRE_LEN)
}

pub fn decode_timing_island_command(
    bytes: &[u8],
) -> Result<TimingIslandCommand, TimingIslandCodecError> {
    let bytes = fixed_in::<TIMING_ISLAND_COMMAND_WIRE_LEN>(bytes)?;
    expect_version(bytes[0])?;
    match bytes[1] {
        TIMING_ISLAND_COMMAND_TAG_ARM_OUTPUT => Ok(TimingIslandCommand::ArmOutput(
            decode_output_transition(&bytes[2..10])?,
        )),
        TIMING_ISLAND_COMMAND_TAG_APPLY_AUX => Ok(TimingIslandCommand::ApplyAux(
            decode_aux_command(&bytes[2..6])?,
        )),
        TIMING_ISLAND_COMMAND_TAG_CANCEL_ALL => Ok(TimingIslandCommand::CancelAll(
            decode_cancel_reason(bytes[2])?,
        )),
        TIMING_ISLAND_COMMAND_TAG_FEED_WATCHDOG => Ok(TimingIslandCommand::FeedWatchdog),
        TIMING_ISLAND_COMMAND_TAG_UPDATE_PERMIT_MASK => Ok(TimingIslandCommand::UpdatePermitMask(
            SafetyPermitMask::new(get_u32(&bytes[2..6])),
        )),
        tag => Err(TimingIslandCodecError::UnknownTag(tag)),
    }
}

pub fn encode_timing_island_event(
    event: TimingIslandEvent,
    out: &mut [u8],
) -> Result<usize, TimingIslandCodecError> {
    let out = fixed_out::<TIMING_ISLAND_EVENT_WIRE_LEN>(out)?;
    out.fill(0);
    out[0] = TIMING_ISLAND_WIRE_VERSION;
    match event {
        TimingIslandEvent::OutputArmed(transition) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_OUTPUT_ARMED;
            encode_output_transition(transition, &mut out[2..10]);
        }
        TimingIslandEvent::OutputCompleted(transition) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_OUTPUT_COMPLETED;
            encode_output_transition(transition, &mut out[2..10]);
        }
        TimingIslandEvent::AuxApplied(command) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_AUX_APPLIED;
            encode_aux_command(command, &mut out[2..6]);
        }
        TimingIslandEvent::Cancelled(reason) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_CANCELLED;
            out[2] = encode_cancel_reason(reason);
        }
        TimingIslandEvent::Rejected(reason) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_REJECTED;
            out[2] = encode_timing_island_reject_reason(reason);
        }
        TimingIslandEvent::CrankEdge(edge) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_CRANK_EDGE;
            encode_trigger_edge(edge, &mut out[2..7]);
        }
        TimingIslandEvent::CamEdge(edge) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_CAM_EDGE;
            encode_trigger_edge(edge, &mut out[2..7]);
        }
        TimingIslandEvent::SyncStatus(status) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_SYNC_STATUS;
            encode_engine_time_authority_telemetry(status, &mut out[2..11]);
        }
        TimingIslandEvent::FaultStatus(status) => {
            out[1] = TIMING_ISLAND_EVENT_TAG_FAULT_STATUS;
            encode_timing_island_fault_status(status, &mut out[2..8]);
        }
    }
    Ok(TIMING_ISLAND_EVENT_WIRE_LEN)
}

pub fn decode_timing_island_event(
    bytes: &[u8],
) -> Result<TimingIslandEvent, TimingIslandCodecError> {
    let bytes = fixed_in::<TIMING_ISLAND_EVENT_WIRE_LEN>(bytes)?;
    expect_version(bytes[0])?;
    match bytes[1] {
        TIMING_ISLAND_EVENT_TAG_OUTPUT_ARMED => Ok(TimingIslandEvent::OutputArmed(
            decode_output_transition(&bytes[2..10])?,
        )),
        TIMING_ISLAND_EVENT_TAG_OUTPUT_COMPLETED => Ok(TimingIslandEvent::OutputCompleted(
            decode_output_transition(&bytes[2..10])?,
        )),
        TIMING_ISLAND_EVENT_TAG_AUX_APPLIED => Ok(TimingIslandEvent::AuxApplied(
            decode_aux_command(&bytes[2..6])?,
        )),
        TIMING_ISLAND_EVENT_TAG_CANCELLED => Ok(TimingIslandEvent::Cancelled(
            decode_cancel_reason(bytes[2])?,
        )),
        TIMING_ISLAND_EVENT_TAG_REJECTED => Ok(TimingIslandEvent::Rejected(
            decode_timing_island_reject_reason(bytes[2])?,
        )),
        TIMING_ISLAND_EVENT_TAG_CRANK_EDGE => Ok(TimingIslandEvent::CrankEdge(
            decode_trigger_edge(&bytes[2..7])?,
        )),
        TIMING_ISLAND_EVENT_TAG_CAM_EDGE => Ok(TimingIslandEvent::CamEdge(decode_trigger_edge(
            &bytes[2..7],
        )?)),
        TIMING_ISLAND_EVENT_TAG_SYNC_STATUS => Ok(TimingIslandEvent::SyncStatus(
            decode_engine_time_authority_telemetry(&bytes[2..11])?,
        )),
        TIMING_ISLAND_EVENT_TAG_FAULT_STATUS => Ok(TimingIslandEvent::FaultStatus(
            decode_timing_island_fault_status(&bytes[2..8])?,
        )),
        tag => Err(TimingIslandCodecError::UnknownTag(tag)),
    }
}

pub fn encode_safety_gate_input(
    input: SafetyGateInput,
    out: &mut [u8],
) -> Result<usize, TimingIslandCodecError> {
    let out = fixed_out::<SAFETY_GATE_INPUT_WIRE_LEN>(out)?;
    out.fill(0);
    out[0] = TIMING_ISLAND_WIRE_VERSION;
    out[1] = encode_safety_gate_input_flags(input);
    put_u32(&mut out[2..6], input.now_us.get());
    put_u32(&mut out[6..10], input.driver_faults);
    put_u32(&mut out[10..14], input.requested_permit_mask.bits());
    Ok(SAFETY_GATE_INPUT_WIRE_LEN)
}

pub fn decode_safety_gate_input(bytes: &[u8]) -> Result<SafetyGateInput, TimingIslandCodecError> {
    let bytes = fixed_in::<SAFETY_GATE_INPUT_WIRE_LEN>(bytes)?;
    expect_version(bytes[0])?;
    let flags = bytes[1];
    if flags & !0b0011_1111 != 0 {
        return Err(TimingIslandCodecError::InvalidField);
    }
    Ok(SafetyGateInput {
        now_us: Micros::new(get_u32(&bytes[2..6])),
        kill_n: flags & (1 << 0) != 0,
        power_good: flags & (1 << 1) != 0,
        watchdog_ok: flags & (1 << 2) != 0,
        timing_backend_alive: flags & (1 << 3) != 0,
        backend_alive: flags & (1 << 4) != 0,
        sync_authority_ok: flags & (1 << 5) != 0,
        driver_faults: get_u32(&bytes[6..10]),
        requested_permit_mask: SafetyPermitMask::new(get_u32(&bytes[10..14])),
    })
}

pub fn encode_safety_gate_status(
    status: SafetyGateStatus,
    out: &mut [u8],
) -> Result<usize, TimingIslandCodecError> {
    let out = fixed_out::<SAFETY_GATE_STATUS_WIRE_LEN>(out)?;
    out.fill(0);
    out[0] = TIMING_ISLAND_WIRE_VERSION;
    out[1] = encode_safety_gate_permit(status.permit);
    out[2] = encode_safety_gate_reason(status.reason);
    out[3] = encode_fault_code(status.fault_code);
    out[4] = encode_fault_severity(status.fault_severity);
    put_u32(&mut out[5..9], status.driver_faults);
    put_u32(&mut out[9..13], status.last_checked_us.get());
    put_u32(&mut out[13..17], status.effective_permit_mask.bits());
    Ok(SAFETY_GATE_STATUS_WIRE_LEN)
}

pub fn decode_safety_gate_status(bytes: &[u8]) -> Result<SafetyGateStatus, TimingIslandCodecError> {
    let bytes = fixed_in::<SAFETY_GATE_STATUS_WIRE_LEN>(bytes)?;
    expect_version(bytes[0])?;
    Ok(SafetyGateStatus {
        permit: decode_safety_gate_permit(bytes[1])?,
        reason: decode_safety_gate_reason(bytes[2])?,
        fault_code: decode_fault_code(bytes[3])?,
        fault_severity: decode_fault_severity(bytes[4])?,
        driver_faults: get_u32(&bytes[5..9]),
        last_checked_us: Micros::new(get_u32(&bytes[9..13])),
        effective_permit_mask: SafetyPermitMask::new(get_u32(&bytes[13..17])),
    })
}

fn fixed_out<const N: usize>(out: &mut [u8]) -> Result<&mut [u8], TimingIslandCodecError> {
    out.get_mut(..N).ok_or(TimingIslandCodecError::ShortBuffer)
}

fn fixed_in<const N: usize>(bytes: &[u8]) -> Result<&[u8], TimingIslandCodecError> {
    bytes.get(..N).ok_or(TimingIslandCodecError::ShortBuffer)
}

fn expect_version(version: u8) -> Result<(), TimingIslandCodecError> {
    if version == TIMING_ISLAND_WIRE_VERSION {
        Ok(())
    } else {
        Err(TimingIslandCodecError::UnsupportedVersion(version))
    }
}

fn put_u32(out: &mut [u8], value: u32) {
    out[..4].copy_from_slice(&value.to_le_bytes());
}

fn get_u32(bytes: &[u8]) -> u32 {
    u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]])
}

fn put_u16(out: &mut [u8], value: u16) {
    out[..2].copy_from_slice(&value.to_le_bytes());
}

fn get_u16(bytes: &[u8]) -> u16 {
    u16::from_le_bytes([bytes[0], bytes[1]])
}

fn encode_trigger_edge(edge: TriggerEdge, out: &mut [u8]) {
    out[0] = encode_edge_kind(edge.kind);
    put_u32(&mut out[1..5], edge.at.get());
}

fn decode_trigger_edge(bytes: &[u8]) -> Result<TriggerEdge, TimingIslandCodecError> {
    Ok(TriggerEdge::new(
        decode_edge_kind(bytes[0])?,
        Ticks::new(get_u32(&bytes[1..5])),
    ))
}

fn encode_edge_kind(kind: EdgeKind) -> u8 {
    match kind {
        EdgeKind::Rising => 0,
        EdgeKind::Falling => 1,
    }
}

fn decode_edge_kind(value: u8) -> Result<EdgeKind, TimingIslandCodecError> {
    match value {
        0 => Ok(EdgeKind::Rising),
        1 => Ok(EdgeKind::Falling),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_output_transition(transition: OutputTransition, out: &mut [u8]) {
    let (kind, channel) = match transition.output {
        EcuOutput::Injector(channel) => (1, channel.get()),
        EcuOutput::Ignition(channel) => (2, channel.get()),
    };
    out[0] = kind;
    out[1] = channel;
    out[2] = encode_output_level(transition.level);
    put_u32(&mut out[4..8], transition.at.get());
}

fn decode_output_transition(bytes: &[u8]) -> Result<OutputTransition, TimingIslandCodecError> {
    let output = match bytes[0] {
        1 => EcuOutput::Injector(ChannelId::new(bytes[1])),
        2 => EcuOutput::Ignition(ChannelId::new(bytes[1])),
        _ => return Err(TimingIslandCodecError::InvalidField),
    };
    Ok(OutputTransition::new(
        output,
        decode_output_level(bytes[2])?,
        Ticks::new(get_u32(&bytes[4..8])),
    ))
}

fn encode_aux_command(command: AuxCommand, out: &mut [u8]) {
    let (output_kind, output_arg) = encode_aux_output(command.output);
    let (value_kind, value_arg) = encode_aux_value(command.value);
    out[0] = output_kind;
    out[1] = output_arg;
    out[2] = value_kind;
    out[3] = value_arg;
}

fn decode_aux_command(bytes: &[u8]) -> Result<AuxCommand, TimingIslandCodecError> {
    Ok(AuxCommand::new(
        decode_aux_output(bytes[0], bytes[1])?,
        decode_aux_value(bytes[2], bytes[3])?,
    ))
}

fn encode_aux_output(output: AuxOutput) -> (u8, u8) {
    match output {
        AuxOutput::SafetyRelay(id) => (1, id),
        AuxOutput::Indicator(id) => (2, id),
        AuxOutput::FrequencyOut(id) => (3, id),
        AuxOutput::Digital(channel) => (4, channel.get()),
        AuxOutput::Pwm(channel) => (5, channel.get()),
    }
}

fn decode_aux_output(kind: u8, arg: u8) -> Result<AuxOutput, TimingIslandCodecError> {
    match kind {
        1 => Ok(AuxOutput::SafetyRelay(arg)),
        2 => Ok(AuxOutput::Indicator(arg)),
        3 => Ok(AuxOutput::FrequencyOut(arg)),
        4 => Ok(AuxOutput::Digital(ChannelId::new(arg))),
        5 => Ok(AuxOutput::Pwm(ChannelId::new(arg))),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_aux_value(value: AuxValue) -> (u8, u8) {
    match value {
        AuxValue::Off => (0, 0),
        AuxValue::Duty(percent) => (1, percent.get()),
        AuxValue::Level(level) => (2, encode_output_level(level)),
    }
}

fn decode_aux_value(kind: u8, arg: u8) -> Result<AuxValue, TimingIslandCodecError> {
    match kind {
        0 => Ok(AuxValue::Off),
        1 => Ok(AuxValue::Duty(Percent::new(arg))),
        2 => Ok(AuxValue::Level(decode_output_level(arg)?)),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_output_level(level: OutputLevel) -> u8 {
    match level {
        OutputLevel::Low => 0,
        OutputLevel::High => 1,
    }
}

fn decode_output_level(value: u8) -> Result<OutputLevel, TimingIslandCodecError> {
    match value {
        0 => Ok(OutputLevel::Low),
        1 => Ok(OutputLevel::High),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_cancel_reason(reason: CancelReason) -> u8 {
    match reason {
        CancelReason::Manual => 0,
        CancelReason::SyncLoss => 1,
        CancelReason::SafetyShutdown => 2,
        CancelReason::Commit => 3,
        CancelReason::Timeout => 4,
    }
}

fn decode_cancel_reason(value: u8) -> Result<CancelReason, TimingIslandCodecError> {
    match value {
        0 => Ok(CancelReason::Manual),
        1 => Ok(CancelReason::SyncLoss),
        2 => Ok(CancelReason::SafetyShutdown),
        3 => Ok(CancelReason::Commit),
        4 => Ok(CancelReason::Timeout),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_timing_island_reject_reason(reason: TimingIslandRejectReason) -> u8 {
    match reason {
        TimingIslandRejectReason::CommandQueueFull => 0,
        TimingIslandRejectReason::PermitDenied => 1,
        TimingIslandRejectReason::BackendFault => 2,
        TimingIslandRejectReason::StaleCommand => 3,
    }
}

fn decode_timing_island_reject_reason(
    value: u8,
) -> Result<TimingIslandRejectReason, TimingIslandCodecError> {
    match value {
        0 => Ok(TimingIslandRejectReason::CommandQueueFull),
        1 => Ok(TimingIslandRejectReason::PermitDenied),
        2 => Ok(TimingIslandRejectReason::BackendFault),
        3 => Ok(TimingIslandRejectReason::StaleCommand),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_engine_time_authority_telemetry(status: EngineTimeAuthorityTelemetry, out: &mut [u8]) {
    out[0] = encode_crank_sync_state(status.authority.crank);
    out[1] = encode_phase_sync_state(status.authority.phase);
    out[2] = encode_absolute_time_authority(status.authority.absolute);
    out[3] = encode_sync_state(status.summary);
    out[4] = u8::from(status.full_sequential_authorized);
    put_u16(&mut out[5..7], status.authority.confidence_x1000);
    put_u16(&mut out[7..9], status.authority.sync_loss_count);
}

fn decode_engine_time_authority_telemetry(
    bytes: &[u8],
) -> Result<EngineTimeAuthorityTelemetry, TimingIslandCodecError> {
    let authority = EngineTimeAuthority::new(
        decode_crank_sync_state(bytes[0])?,
        decode_phase_sync_state(bytes[1])?,
        decode_absolute_time_authority(bytes[2])?,
        get_u16(&bytes[5..7]),
        get_u16(&bytes[7..9]),
    );
    let status = EngineTimeAuthorityTelemetry::new(authority);

    if decode_sync_state(bytes[3])? != status.summary || (bytes[4] != 0 && bytes[4] != 1) {
        return Err(TimingIslandCodecError::InvalidField);
    }
    if (bytes[4] != 0) != status.full_sequential_authorized {
        return Err(TimingIslandCodecError::InvalidField);
    }

    Ok(status)
}

fn encode_crank_sync_state(state: CrankSyncState) -> u8 {
    match state {
        CrankSyncState::NoSignal => 0,
        CrankSyncState::PrimarySearching => 1,
        CrankSyncState::PrimaryLocked => 2,
        CrankSyncState::SyncLost => 3,
    }
}

fn decode_crank_sync_state(value: u8) -> Result<CrankSyncState, TimingIslandCodecError> {
    match value {
        0 => Ok(CrankSyncState::NoSignal),
        1 => Ok(CrankSyncState::PrimarySearching),
        2 => Ok(CrankSyncState::PrimaryLocked),
        3 => Ok(CrankSyncState::SyncLost),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_phase_sync_state(state: PhaseSyncState) -> u8 {
    match state {
        PhaseSyncState::Unknown => 0,
        PhaseSyncState::CrankOnly360 => 1,
        PhaseSyncState::CamObserved720 => 2,
        PhaseSyncState::CamValidated720 => 3,
    }
}

fn decode_phase_sync_state(value: u8) -> Result<PhaseSyncState, TimingIslandCodecError> {
    match value {
        0 => Ok(PhaseSyncState::Unknown),
        1 => Ok(PhaseSyncState::CrankOnly360),
        2 => Ok(PhaseSyncState::CamObserved720),
        3 => Ok(PhaseSyncState::CamValidated720),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_absolute_time_authority(authority: AbsoluteTimeAuthority) -> u8 {
    match authority {
        AbsoluteTimeAuthority::None => 0,
        AbsoluteTimeAuthority::GeometryOnly => 1,
        AbsoluteTimeAuthority::ExpertManual => 2,
        AbsoluteTimeAuthority::CommunityProfile => 3,
        AbsoluteTimeAuthority::CertifiedProfile => 4,
        AbsoluteTimeAuthority::BenchLearned => 5,
    }
}

fn decode_absolute_time_authority(
    value: u8,
) -> Result<AbsoluteTimeAuthority, TimingIslandCodecError> {
    match value {
        0 => Ok(AbsoluteTimeAuthority::None),
        1 => Ok(AbsoluteTimeAuthority::GeometryOnly),
        2 => Ok(AbsoluteTimeAuthority::ExpertManual),
        3 => Ok(AbsoluteTimeAuthority::CommunityProfile),
        4 => Ok(AbsoluteTimeAuthority::CertifiedProfile),
        5 => Ok(AbsoluteTimeAuthority::BenchLearned),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_sync_state(state: SyncState) -> u8 {
    match state {
        SyncState::Unsynced => 0,
        SyncState::Provisional => 1,
        SyncState::Locked { .. } => 2,
    }
}

fn decode_sync_state(value: u8) -> Result<SyncState, TimingIslandCodecError> {
    match value {
        0 => Ok(SyncState::Unsynced),
        1 => Ok(SyncState::Provisional),
        2 => Ok(SyncState::Locked { cam_ref: false }),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_timing_island_fault_status(status: TimingIslandFaultStatus, out: &mut [u8]) {
    out[0] = encode_fault_code(status.code);
    out[1] = encode_fault_severity(status.severity);
    put_u32(&mut out[2..6], status.at.get());
}

fn decode_timing_island_fault_status(
    bytes: &[u8],
) -> Result<TimingIslandFaultStatus, TimingIslandCodecError> {
    Ok(TimingIslandFaultStatus::new(
        decode_fault_code(bytes[0])?,
        decode_fault_severity(bytes[1])?,
        Ticks::new(get_u32(&bytes[2..6])),
    ))
}

fn encode_safety_gate_input_flags(input: SafetyGateInput) -> u8 {
    u8::from(input.kill_n)
        | (u8::from(input.power_good) << 1)
        | (u8::from(input.watchdog_ok) << 2)
        | (u8::from(input.timing_backend_alive) << 3)
        | (u8::from(input.backend_alive) << 4)
        | (u8::from(input.sync_authority_ok) << 5)
}

fn encode_safety_gate_permit(permit: SafetyGatePermit) -> u8 {
    match permit {
        SafetyGatePermit::Denied => 0,
        SafetyGatePermit::Allowed => 1,
    }
}

fn decode_safety_gate_permit(value: u8) -> Result<SafetyGatePermit, TimingIslandCodecError> {
    match value {
        0 => Ok(SafetyGatePermit::Denied),
        1 => Ok(SafetyGatePermit::Allowed),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_safety_gate_reason(reason: SafetyGateReason) -> u8 {
    match reason {
        SafetyGateReason::None => 0,
        SafetyGateReason::SyncNotAuthorized => 1,
        SafetyGateReason::KillAsserted => 5,
        SafetyGateReason::PowerNotGood => 6,
        SafetyGateReason::WatchdogTimeout => 7,
        SafetyGateReason::TimingBackendNotAlive => 8,
        SafetyGateReason::BackendNotAlive => 9,
        SafetyGateReason::DriverFault => 10,
    }
}

fn decode_safety_gate_reason(value: u8) -> Result<SafetyGateReason, TimingIslandCodecError> {
    match value {
        0 => Ok(SafetyGateReason::None),
        1 => Ok(SafetyGateReason::SyncNotAuthorized),
        5 => Ok(SafetyGateReason::KillAsserted),
        6 => Ok(SafetyGateReason::PowerNotGood),
        7 => Ok(SafetyGateReason::WatchdogTimeout),
        8 => Ok(SafetyGateReason::TimingBackendNotAlive),
        9 => Ok(SafetyGateReason::BackendNotAlive),
        10 => Ok(SafetyGateReason::DriverFault),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_fault_code(code: FaultCode) -> u8 {
    match code {
        FaultCode::None => 0,
        FaultCode::SyncLoss => 1,
        FaultCode::SensorOutOfRange => 2,
        FaultCode::CalibrationInvalid => 3,
        FaultCode::SafetyCut => 4,
        FaultCode::ActuatorFault => 5,
    }
}

fn decode_fault_code(value: u8) -> Result<FaultCode, TimingIslandCodecError> {
    match value {
        0 => Ok(FaultCode::None),
        1 => Ok(FaultCode::SyncLoss),
        2 => Ok(FaultCode::SensorOutOfRange),
        3 => Ok(FaultCode::CalibrationInvalid),
        4 => Ok(FaultCode::SafetyCut),
        5 => Ok(FaultCode::ActuatorFault),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}

fn encode_fault_severity(severity: FaultSeverity) -> u8 {
    match severity {
        FaultSeverity::Info => 0,
        FaultSeverity::Warning => 1,
        FaultSeverity::Critical => 2,
    }
}

fn decode_fault_severity(value: u8) -> Result<FaultSeverity, TimingIslandCodecError> {
    match value {
        0 => Ok(FaultSeverity::Info),
        1 => Ok(FaultSeverity::Warning),
        2 => Ok(FaultSeverity::Critical),
        _ => Err(TimingIslandCodecError::InvalidField),
    }
}
