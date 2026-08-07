#![cfg_attr(not(test), no_std)]

mod conformance;
mod planner;
mod profiles;
mod queue;
mod state;
mod types;

#[cfg(test)]
mod tests;

pub use ecu_domain::ChannelId;
pub use ecu_domain::{Degrees10, DwellUs, EngineTimeAuthority, Micros, Percent, PulseWidthUs};

/// Single source of truth for the `scheduler.tla` `MaxPending` constant.
///
/// `MaxPending` bounds the per-cylinder event window the formal model schedules
/// before any are fired: two injection transitions (open + close) plus two
/// ignition transitions (coil charge + fire). The implementation mirrors this
/// as the `export_transitions` array width. The model `.tla`/`.cfg` value and
/// this constant are kept in lockstep by the drift check in
/// `tools/verify_formal.sh`.
pub const MODEL_MAX_PENDING: usize = 4;

/// Single source of truth for the `scheduler.tla` `MaxOutputs` constant.
///
/// `MaxOutputs` bounds the distinct fired-output set the formal model tracks
/// (`InjectionOpen`, `InjectionClose`, `CoilChargeStart`, `CoilFire`). The
/// model `.tla`/`.cfg` value and this constant are kept in lockstep by the
/// drift check in `tools/verify_formal.sh`.
pub const MODEL_MAX_OUTPUTS: usize = 4;

pub use planner::{
    FullEcuIgnitionScheduler, FullEcuInjectionScheduler, IgnitionScheduler, InjectionScheduler,
};
pub use profiles::{
    FuelOutputMode, FuelOutputProfile, FuelPlan, IgnitionOutputProfile, InjectionOutputProfile,
    SparkOutputMode, SparkOutputProfile, SparkPlan,
};
pub use queue::{
    ScheduleExport, ScheduledLevel, ScheduledTimingMetrics, ScheduledTransition,
    ScheduledTransitionKind, ScheduledTransitionQueue, ScheduledTransitionQueueSnapshot,
    TransitionDrainBuffer,
};
pub use state::{SchedulerMode, SchedulerState};
pub use types::{
    CrankSnapshot, ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup, ScheduleError,
    TimedIgnitionPlan, TimedInjectionPlan,
};

/// Explicit conformance/test support surface.
///
/// Product code should use the scheduler state and planner APIs directly.
pub mod test_support {
    pub use crate::conformance::{
        observe_scheduler, observe_scheduler_queue, SchedulerAdapterContract,
        SchedulerConformanceStatus, SchedulerEventKind, SchedulerEventSnapshot,
        SchedulerObservedSurface,
    };
}
