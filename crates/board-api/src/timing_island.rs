//! Typed timing island commands/events and fixed-capacity batch helpers.

use crate::telemetry::EngineTimeAuthorityTelemetry;
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, CrankSyncState, EngineTimeAuthority, Percent,
    PhaseSyncState, SyncState, Ticks,
};

use ecu_domain::{FaultCode, FaultSeverity};

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum EdgeKind {
    #[default]
    Rising,
    Falling,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum OutputLevel {
    #[default]
    Low,
    High,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum AuxValue {
    #[default]
    Off,
    Duty(Percent),
    Level(OutputLevel),
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EcuOutput {
    Injector(ChannelId),
    Ignition(ChannelId),
}

impl Default for EcuOutput {
    fn default() -> Self {
        Self::Injector(ChannelId::new(0))
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AuxOutput {
    SafetyRelay(u8),
    Indicator(u8),
    FrequencyOut(u8),
    Digital(ChannelId),
    Pwm(ChannelId),
}

impl Default for AuxOutput {
    fn default() -> Self {
        Self::Digital(ChannelId::new(0))
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct TriggerEdge {
    pub kind: EdgeKind,
    pub at: Ticks,
}

impl TriggerEdge {
    pub const EMPTY: Self = Self {
        kind: EdgeKind::Rising,
        at: Ticks::new(0),
    };

    pub const fn new(kind: EdgeKind, at: Ticks) -> Self {
        Self { kind, at }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct OutputTransition {
    pub output: EcuOutput,
    pub level: OutputLevel,
    pub at: Ticks,
}

impl OutputTransition {
    pub const EMPTY: Self = Self {
        output: EcuOutput::Injector(ChannelId::new(0)),
        level: OutputLevel::Low,
        at: Ticks::new(0),
    };

    pub const fn new(output: EcuOutput, level: OutputLevel, at: Ticks) -> Self {
        Self { output, level, at }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AuxCommand {
    pub output: AuxOutput,
    pub value: AuxValue,
}

impl AuxCommand {
    pub const EMPTY: Self = Self {
        output: AuxOutput::Digital(ChannelId::new(0)),
        value: AuxValue::Off,
    };

    pub const fn new(output: AuxOutput, value: AuxValue) -> Self {
        Self { output, value }
    }
}

/// Typed command for an optional timing island.
///
/// The contract is expressed in logical board terms. A board implementation may
/// execute it with any board-provided timing or simulator backend.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimingIslandCommand {
    ArmOutput(OutputTransition),
    ApplyAux(AuxCommand),
    CancelAll(CancelReason),
    FeedWatchdog,
    UpdatePermitMask(crate::safety::SafetyPermitMask),
}

impl TimingIslandCommand {
    pub const EMPTY: Self = Self::CancelAll(CancelReason::Manual);
}

impl Default for TimingIslandCommand {
    fn default() -> Self {
        Self::EMPTY
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum TimingIslandRejectReason {
    #[default]
    CommandQueueFull,
    PermitDenied,
    BackendFault,
    StaleCommand,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TimingIslandEvent {
    OutputArmed(OutputTransition),
    OutputCompleted(OutputTransition),
    AuxApplied(AuxCommand),
    Cancelled(CancelReason),
    Rejected(TimingIslandRejectReason),
    CrankEdge(TriggerEdge),
    CamEdge(TriggerEdge),
    SyncStatus(EngineTimeAuthorityTelemetry),
    FaultStatus(TimingIslandFaultStatus),
}

impl TimingIslandEvent {
    pub const EMPTY: Self = Self::Cancelled(CancelReason::Manual);
}

impl Default for TimingIslandEvent {
    fn default() -> Self {
        Self::EMPTY
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct TimingIslandFaultStatus {
    pub code: FaultCode,
    pub severity: FaultSeverity,
    pub at: Ticks,
}

impl TimingIslandFaultStatus {
    pub const fn new(code: FaultCode, severity: FaultSeverity, at: Ticks) -> Self {
        Self { code, severity, at }
    }
}

pub(crate) const fn absolute_time_authorizes_full_sequential(
    source: AbsoluteTimeAuthority,
) -> bool {
    matches!(
        source,
        AbsoluteTimeAuthority::ExpertManual
            | AbsoluteTimeAuthority::CommunityProfile
            | AbsoluteTimeAuthority::CertifiedProfile
            | AbsoluteTimeAuthority::BenchLearned
    )
}

pub const fn engine_time_authorizes_full_sequential(authority: EngineTimeAuthority) -> bool {
    authority.confidence_x1000 <= EngineTimeAuthority::MAX_CONFIDENCE_X1000
        && matches!(authority.crank, CrankSyncState::PrimaryLocked)
        && matches!(authority.phase, PhaseSyncState::CamValidated720)
        && absolute_time_authorizes_full_sequential(authority.absolute)
}

pub const fn legacy_sync_state_authority(sync_state: ecu_domain::SyncState) -> EngineTimeAuthority {
    match sync_state {
        SyncState::Unsynced => EngineTimeAuthority::none(),
        SyncState::Provisional => EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::Unknown,
            AbsoluteTimeAuthority::None,
            0,
            0,
        ),
        SyncState::Locked { .. } => EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        ),
    }
}

macro_rules! fixed_batch {
    ($name:ident, $item:ty, $empty:expr) => {
        #[repr(C)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct $name<const N: usize> {
            len: usize,
            items: [$item; N],
        }

        impl<const N: usize> $name<N> {
            pub const fn new() -> Self {
                Self {
                    len: 0,
                    items: [$empty; N],
                }
            }

            pub const fn capacity(&self) -> usize {
                N
            }

            pub const fn len(&self) -> usize {
                self.len
            }

            pub const fn is_empty(&self) -> bool {
                self.len == 0
            }

            pub fn clear(&mut self) {
                self.len = 0;
            }

            pub fn push(&mut self, item: $item) -> Result<(), $item> {
                if self.len == N {
                    return Err(item);
                }

                self.items[self.len] = item;
                self.len += 1;
                Ok(())
            }

            pub fn iter(&self) -> core::slice::Iter<'_, $item> {
                self.items[..self.len].iter()
            }

            pub fn as_slice(&self) -> &[$item] {
                &self.items[..self.len]
            }
        }

        impl<const N: usize> Default for $name<N> {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

fixed_batch!(EdgeBatch, TriggerEdge, TriggerEdge::EMPTY);
fixed_batch!(
    OutputTransitionBatch,
    OutputTransition,
    OutputTransition::EMPTY
);
fixed_batch!(AuxCommandBatch, AuxCommand, AuxCommand::EMPTY);
fixed_batch!(
    TimingIslandCommandBatch,
    TimingIslandCommand,
    TimingIslandCommand::EMPTY
);
