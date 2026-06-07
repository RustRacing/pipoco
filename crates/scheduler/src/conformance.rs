use crate::{
    ChannelId, Micros, SchedulerMode, SchedulerState, TimedIgnitionPlan, TimedInjectionPlan,
};

/// Observable surface for scheduler conformance.
/// Fields read from SchedulerState and product scheduler APIs only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SchedulerObservedSurface {
    pub mode: SchedulerMode,
    pub active_groups: u8,
    pub reserved_channels: [u128; 4],
    pub last_injection_start: Option<Micros>,
    pub last_injection_end: Option<Micros>,
    pub last_ignition_start: Option<Micros>,
    pub last_ignition_end: Option<Micros>,
    pub injection_count: u8,
    pub ignition_count: u8,
}

/// Adapter contracts for scheduler fields that cannot be directly compared.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerAdapterContract {
    /// SOI angle is computed upstream by runtime fuel pipeline, not by scheduler.
    SoiAngleComputedUpstream,
    /// EOI angle is computed upstream by runtime fuel pipeline, not by scheduler.
    EoiAngleComputedUpstream,
    /// Spark angle is computed upstream by runtime ignition planner, not by scheduler.
    SparkAngleComputedUpstream,
    /// Dwell start angle is computed upstream by runtime ignition planner.
    DwellStartAngleComputedUpstream,
    /// Event kinds are derived from upstream runtime plan, not scheduler.
    EventKindFromRuntime,
    /// Scheduler diagnostic surface is independent of spec diagnostic codes.
    DiagnosticSurfaceIncomparable,
    /// Event count depends on upstream injection/ignition planning.
    EventCountFromRuntimePlanning,
}

/// Field-by-field conformance status for scheduler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulerConformanceStatus {
    Covered,
    AdapterContract(SchedulerAdapterContract),
}

// ---------------------------------------------------------------------------
// Scheduler event snapshot (US-FM0517)
// ---------------------------------------------------------------------------

/// Snapshot of a scheduled event produced by the scheduler.
///
/// This is a read-only view of an event that the scheduler has either
/// scheduled (injection/ignition) or cancelled/suspended. It is not
/// stored internally - callers receive it as the result of observation APIs.
///
/// This type is fixed-size and no_alloc-friendly.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SchedulerEventSnapshot {
    /// The kind of event.
    pub kind: SchedulerEventKind,
    /// Which output channel this event targets.
    pub channel: ChannelId,
    /// Scheduled deadline in microseconds.
    pub deadline_us: Micros,
    /// Whether this event is currently active (scheduled and not cancelled).
    pub active: bool,
    /// Whether this event was explicitly cancelled.
    pub cancelled: bool,
    /// Order index in the scheduler's event queue (if applicable).
    /// None if the scheduler does not maintain ordering.
    pub order_index: Option<u8>,
}

/// Kinds of scheduler-owned events.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum SchedulerEventKind {
    Injection,
    Ignition,
}

impl SchedulerEventSnapshot {
    /// Create an injection event snapshot from a TimedInjectionPlan.
    #[inline]
    pub fn from_injection_plan(plan: &TimedInjectionPlan, order_index: u8) -> Self {
        Self {
            kind: SchedulerEventKind::Injection,
            channel: plan.plan.output.channel(),
            deadline_us: plan.end_at,
            active: true,
            cancelled: false,
            order_index: Some(order_index),
        }
    }

    /// Create an ignition event snapshot from a TimedIgnitionPlan.
    #[inline]
    pub fn from_ignition_plan(plan: &TimedIgnitionPlan, order_index: u8) -> Self {
        Self {
            kind: SchedulerEventKind::Ignition,
            channel: plan.plan.output.channel(),
            deadline_us: plan.end_at,
            active: true,
            cancelled: false,
            order_index: Some(order_index),
        }
    }

    /// Mark this snapshot as cancelled.
    #[inline]
    pub fn with_cancelled(mut self) -> Self {
        self.active = false;
        self.cancelled = true;
        self
    }
}

/// Extract an observation snapshot from the current scheduler state.
///
/// This is a read-only observation - it does not mutate the scheduler.
/// Returns the current scheduler mode, active groups, channel reservations,
/// active event counts. The returned `last_*` timestamps are the most recent
/// deadlines accepted via the `schedule_injection`/`schedule_ignition` APIs.
#[inline]
pub fn observe_scheduler(state: &SchedulerState) -> SchedulerObservedSurface {
    SchedulerObservedSurface {
        mode: state.mode(),
        active_groups: state.active_groups(),
        reserved_channels: state.reserved_channels(),
        last_injection_start: state.last_injection_start(),
        last_injection_end: state.last_injection_end(),
        last_ignition_start: state.last_ignition_start(),
        last_ignition_end: state.last_ignition_end(),
        injection_count: state.injection_count(),
        ignition_count: state.ignition_count(),
    }
}
