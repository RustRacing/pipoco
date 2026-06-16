use crate::types::{
    FrontierAdmissionReport, FrontierMetricSnapshot, FRONTIER_HEARTBEAT_EXPIRY_US,
    FRONTIER_HORIZON_SEQUENCE_BITS, FRONTIER_MAX_HORIZON_US,
};
use crate::{
    ChannelId, Micros, ScheduledTransitionQueue, SchedulerMode, SchedulerState, TimedIgnitionPlan,
    TimedInjectionPlan,
};
use ecu_board_api::frontier::{
    TimingIslandHorizonSequenceId, TimingIslandPermitMask, TimingIslandStopReason,
    TimingIslandSyncLossReason,
};
use ecu_domain::SyncState;

/// Observable surface for scheduler conformance.
/// Fields read from SchedulerState and product scheduler APIs only.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SchedulerObservedSurface {
    pub mode: SchedulerMode,
    pub active_groups: u8,
    pub reserved_channels: [u128; 4],
    pub last_accepted_horizon_id: Option<TimingIslandHorizonSequenceId>,
    pub active_horizon_id: Option<TimingIslandHorizonSequenceId>,
    pub horizon_start_us: Option<Micros>,
    pub horizon_end_us: Option<Micros>,
    pub heartbeat_deadline_us: Option<Micros>,
    pub active_permit_mask: TimingIslandPermitMask,
    pub active_stop_reason: TimingIslandStopReason,
    pub last_injection_start: Option<Micros>,
    pub last_injection_end: Option<Micros>,
    pub last_ignition_start: Option<Micros>,
    pub last_ignition_end: Option<Micros>,
    pub injection_count: u8,
    pub ignition_count: u8,
    pub late_event_count: u32,
    pub max_lateness_us: Option<Micros>,
    pub queue_high_water_mark: u8,
    pub last_drain_count: u8,
}

/// Canonical frontier contract surface for scheduler conformance checks.
#[allow(dead_code)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) struct FrontierContractSurface {
    pub horizon_sequence_bits: u8,
    pub heartbeat_expiry_us: Micros,
    pub max_horizon_us: Micros,
    pub horizon_sequence_id: TimingIslandHorizonSequenceId,
    pub default_permit_mask: TimingIslandPermitMask,
    pub default_stop_reason: TimingIslandStopReason,
    pub admission_report: FrontierAdmissionReport,
    pub metric_snapshot: FrontierMetricSnapshot,
}

#[allow(dead_code)]
pub(crate) const fn frontier_contract_surface() -> FrontierContractSurface {
    FrontierContractSurface {
        horizon_sequence_bits: FRONTIER_HORIZON_SEQUENCE_BITS,
        heartbeat_expiry_us: FRONTIER_HEARTBEAT_EXPIRY_US,
        max_horizon_us: FRONTIER_MAX_HORIZON_US,
        horizon_sequence_id: 0,
        default_permit_mask: TimingIslandPermitMask::NONE,
        default_stop_reason: TimingIslandStopReason::None,
        admission_report: FrontierAdmissionReport::new(
            0,
            false,
            TimingIslandStopReason::None,
            TimingIslandPermitMask::NONE,
            Micros::new(0),
            Micros::new(0),
        ),
        metric_snapshot: FrontierMetricSnapshot::new(
            SyncState::Unsynced,
            TimingIslandSyncLossReason::None,
            false,
            None,
            None,
            None,
            TimingIslandPermitMask::NONE,
            TimingIslandStopReason::None,
            0,
            0,
        ),
    }
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
        last_accepted_horizon_id: state.last_accepted_horizon_id(),
        active_horizon_id: state.active_horizon_id(),
        horizon_start_us: state.horizon_start_us(),
        horizon_end_us: state.horizon_end_us(),
        heartbeat_deadline_us: state.heartbeat_deadline_us(),
        active_permit_mask: state.active_permit_mask(),
        active_stop_reason: state.active_stop_reason(),
        last_injection_start: state.last_injection_start(),
        last_injection_end: state.last_injection_end(),
        last_ignition_start: state.last_ignition_start(),
        last_ignition_end: state.last_ignition_end(),
        injection_count: state.injection_count(),
        ignition_count: state.ignition_count(),
        late_event_count: 0,
        max_lateness_us: None,
        queue_high_water_mark: 0,
        last_drain_count: 0,
    }
}

/// Extract queue timing metrics into the shared observation surface.
#[inline]
pub fn observe_scheduler_queue<const N: usize>(
    queue: &ScheduledTransitionQueue<N>,
) -> SchedulerObservedSurface {
    let snapshot = queue.snapshot();
    SchedulerObservedSurface {
        late_event_count: snapshot.late_event_count,
        max_lateness_us: snapshot.max_lateness_us,
        queue_high_water_mark: snapshot.queue_high_water_mark,
        last_drain_count: snapshot.last_drain_count,
        ..SchedulerObservedSurface::default()
    }
}
