use ecu_domain::Rpm;

use crate::{ChannelId, Degrees10, DwellUs, EngineTimeAuthority, Micros, PulseWidthUs};

/// Output group used to express exclusivity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum OutputGroup {
    #[default]
    Injector,
    Ignition,
    Idle,
    Fan,
}

/// Output channel keyed by group and domain-local identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ExclusiveChannel {
    group: OutputGroup,
    channel: ChannelId,
}

impl ExclusiveChannel {
    pub const fn new(group: OutputGroup, channel: ChannelId) -> Self {
        Self { group, channel }
    }

    pub const fn group(self) -> OutputGroup {
        self.group
    }

    pub const fn channel(self) -> ChannelId {
        self.channel
    }
}

/// First-pass injection intent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct InjectionPlan {
    pub output: ExclusiveChannel,
    pub pulse_width: PulseWidthUs,
}

/// First-pass ignition intent.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IgnitionPlan {
    pub output: ExclusiveChannel,
    pub dwell: DwellUs,
    pub advance: Degrees10,
}

/// Scheduler rejection reasons for invalid plan conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleError {
    Suspended,
    StaleDeadline,
    ImpossibleDeadline,
    ConflictingChannel,
    InvalidChannel,
    QueueFull,
}

/// Injection plan with explicit deadlines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedInjectionPlan {
    pub plan: InjectionPlan,
    pub start_at: Micros,
    pub end_at: Micros,
}

/// Ignition plan with explicit deadlines.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TimedIgnitionPlan {
    pub plan: IgnitionPlan,
    pub start_at: Micros,
    pub end_at: Micros,
}

/// Engine-position snapshot used by domain schedulers.
///
/// Capture hardware and trigger decoding update this value. Ignition and
/// injection schedulers consume it, but actuator test paths do not need it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CrankSnapshot {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub angle_deg10: Degrees10,
    pub authority: EngineTimeAuthority,
}

impl CrankSnapshot {
    pub const fn new(
        now_us: Micros,
        rpm: Rpm,
        angle_deg10: Degrees10,
        authority: EngineTimeAuthority,
    ) -> Self {
        Self {
            now_us,
            rpm,
            angle_deg10,
            authority,
        }
    }
}

impl Default for CrankSnapshot {
    fn default() -> Self {
        Self {
            now_us: Micros::new(0),
            rpm: Rpm::new(0),
            angle_deg10: Degrees10::new(0),
            authority: EngineTimeAuthority::none(),
        }
    }
}
