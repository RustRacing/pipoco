#![cfg_attr(not(test), no_std)]

// Re-export domain types for use in tests and downstream crates.
pub use ecu_domain::ChannelId;
pub use ecu_domain::{Degrees10, DwellUs, Micros, Percent, PulseWidthUs};

use ecu_domain::Rpm;

const DIFF_CYLINDER_STORAGE: usize = 16;
const DIFF_EVENTS_PER_STEP: usize = 64;

/// Differential event kind for FM0016 output mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum DifferentialEventKind {
    #[default]
    InjectionOpen,
    InjectionClose,
    CoilChargeStart,
    CoilFire,
}

/// Differential event carrying semantic kind and angle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DifferentialEvent {
    pub kind: DifferentialEventKind,
    pub angle_deg10: Degrees10,
}

/// Fixed-size differential event batch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DifferentialEventBatch {
    pub len: u8,
    pub events: [DifferentialEvent; DIFF_EVENTS_PER_STEP],
}

impl Default for DifferentialEventBatch {
    fn default() -> Self {
        Self {
            len: 0,
            events: [DifferentialEvent::default(); DIFF_EVENTS_PER_STEP],
        }
    }
}

/// Differential diagnostic code for FM0016 output mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum DifferentialDiagnosticCode {
    #[default]
    None,
    Unsynced,
    FuelCutActive,
    SparkCutActive,
    CalibrationInvalid,
    SensorPlausibilityFault,
}

/// Additive full-field observable output surface for FM0016 conformance
/// comparisons.
///
/// This is an observation surface, not a second scheduler oracle. The
/// deadline/transition export path remains intentionally narrow.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DifferentialOutputSurface {
    pub ve_pct_x100: u16,
    pub target_afr_x100: u16,
    pub pw_base_us: u32,
    pub pw_air_us: u32,
    pub pw_corr_us: u32,
    pub soi_deg10: [u16; DIFF_CYLINDER_STORAGE],
    pub eoi_deg10: [u16; DIFF_CYLINDER_STORAGE],
    pub spark_deg10: [u16; DIFF_CYLINDER_STORAGE],
    pub dwell_start_deg10: [u16; DIFF_CYLINDER_STORAGE],
    pub events: DifferentialEventBatch,
    pub diagnostic: DifferentialDiagnosticCode,
}

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

/// First-pass idle command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IdleCommand {
    pub enabled: bool,
    pub target_rpm: Rpm,
    pub duty: Percent,
}

/// First-pass fan command.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FanCommand {
    pub enabled: bool,
    pub target_c: i16,
}

/// Scheduler rejection reasons for invalid plan conversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduleError {
    Suspended,
    StaleDeadline,
    ImpossibleDeadline,
    ConflictingChannel,
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

/// Kind of scheduled output transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ScheduledTransitionKind {
    Injector,
    Ignition,
}

/// Logic level of a scheduled transition.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScheduledLevel {
    Low,
    High,
}

/// A pin-level output transition exported from a scheduled plan.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledTransition {
    /// Timestamp in microseconds.
    pub at_us: Micros,
    /// Kind of transition.
    pub kind: ScheduledTransitionKind,
    /// Output channel.
    pub channel: ChannelId,
    /// New logic level.
    pub level: ScheduledLevel,
}

/// Export of scheduled transitions from a timed plan.
///
/// The scheduler exports deadlines and levels that were already planned
/// upstream; it does not re-derive the semantic angle pipeline here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduleExport<const N: usize> {
    /// Number of valid transitions in the array.
    pub len: u8,
    /// Fixed-size array of transitions.
    pub transitions: [Option<ScheduledTransition>; N],
}

/// Fixed-size buffer filled by `ScheduledTransitionQueue::drain_due`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TransitionDrainBuffer<const N: usize> {
    pub len: u8,
    pub transitions: [Option<ScheduledTransition>; N],
}

impl<const N: usize> TransitionDrainBuffer<N> {
    pub const fn new() -> Self {
        Self {
            len: 0,
            transitions: [None; N],
        }
    }

    pub fn clear(&mut self) {
        self.len = 0;
        self.transitions = [None; N];
    }

    fn push(&mut self, transition: ScheduledTransition) -> Result<(), ScheduleError> {
        let idx = self.len as usize;
        if idx >= N {
            return Err(ScheduleError::QueueFull);
        }
        self.transitions[idx] = Some(transition);
        self.len = self.len.saturating_add(1);
        Ok(())
    }

    pub fn as_slice(&self) -> &[Option<ScheduledTransition>] {
        &self.transitions[..self.len as usize]
    }
}

impl<const N: usize> Default for TransitionDrainBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Read-only queue observation for diagnostics and tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledTransitionQueueSnapshot<const N: usize> {
    pub len: u8,
    pub transitions: [Option<ScheduledTransition>; N],
}

/// Fixed-capacity transition queue for live scheduler execution.
///
/// The queue owns deadline ordering and cancellation, but it never drives pins.
/// Targets drain due transitions and apply the `ScheduledLevel` elsewhere.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledTransitionQueue<const N: usize> {
    transitions: [Option<ScheduledTransition>; N],
}

impl<const N: usize> ScheduledTransitionQueue<N> {
    pub const fn new() -> Self {
        Self {
            transitions: [None; N],
        }
    }

    pub fn enqueue_transition(
        &mut self,
        transition: ScheduledTransition,
    ) -> Result<(), ScheduleError> {
        for slot in &mut self.transitions {
            if slot.is_none() {
                *slot = Some(transition);
                return Ok(());
            }
        }
        Err(ScheduleError::QueueFull)
    }

    pub fn enqueue_export<const M: usize>(
        &mut self,
        export: &ScheduleExport<M>,
    ) -> Result<(), ScheduleError> {
        if self.free_slots() < export.len as usize {
            return Err(ScheduleError::QueueFull);
        }
        let mut idx = 0;
        while idx < export.len as usize {
            if let Some(transition) = export.transitions[idx] {
                self.enqueue_transition(transition)?;
            }
            idx += 1;
        }
        Ok(())
    }

    pub fn drain_due<const M: usize>(
        &mut self,
        now: Micros,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        out.clear();
        let mut drained = 0;
        loop {
            if out.len as usize >= M {
                break;
            }
            let Some(idx) = self.next_due_index(now) else {
                break;
            };
            if let Some(transition) = self.transitions[idx].take() {
                // Capacity was checked at the top of the loop.
                let _ = out.push(transition);
                drained += 1;
            }
        }
        drained
    }

    pub fn cancel_channel_after(&mut self, channel: ChannelId, cutoff: Micros) {
        for slot in &mut self.transitions {
            if let Some(transition) = slot {
                if transition.channel == channel && is_at_or_after(transition.at_us, cutoff) {
                    *slot = None;
                }
            }
        }
    }

    pub fn cancel_channel_all(&mut self, channel: ChannelId) {
        for slot in &mut self.transitions {
            if slot.is_some_and(|transition| transition.channel == channel) {
                *slot = None;
            }
        }
    }

    pub fn cancel_kind(&mut self, kind: ScheduledTransitionKind) {
        for slot in &mut self.transitions {
            if slot.is_some_and(|transition| transition.kind == kind) {
                *slot = None;
            }
        }
    }

    pub fn cancel_group(&mut self, group: OutputGroup) {
        match group {
            OutputGroup::Injector => self.cancel_kind(ScheduledTransitionKind::Injector),
            OutputGroup::Ignition => self.cancel_kind(ScheduledTransitionKind::Ignition),
            OutputGroup::Idle | OutputGroup::Fan => {}
        }
    }

    pub fn cancel_all(&mut self) {
        self.transitions = [None; N];
    }

    pub fn on_sync_loss(&mut self) {
        self.cancel_all();
    }

    pub fn on_hard_safety_shutdown(&mut self) {
        self.cancel_all();
    }

    pub fn active_count(&self) -> usize {
        self.transitions
            .iter()
            .filter(|transition| transition.is_some())
            .count()
    }

    pub fn capacity(&self) -> usize {
        N
    }

    pub fn free_slots(&self) -> usize {
        N.saturating_sub(self.active_count())
    }

    pub fn is_full(&self) -> bool {
        self.active_count() == N
    }

    pub fn snapshot(&self) -> ScheduledTransitionQueueSnapshot<N> {
        ScheduledTransitionQueueSnapshot {
            len: self.active_count() as u8,
            transitions: self.transitions,
        }
    }

    fn next_due_index(&self, now: Micros) -> Option<usize> {
        let mut selected: Option<(usize, ScheduledTransition)> = None;
        for (idx, slot) in self.transitions.iter().enumerate() {
            let Some(transition) = *slot else {
                continue;
            };
            if !is_due(now, transition.at_us) {
                continue;
            }
            match selected {
                None => selected = Some((idx, transition)),
                Some((_, current)) => {
                    if is_before(transition.at_us, current.at_us) {
                        selected = Some((idx, transition));
                    }
                }
            }
        }
        selected.map(|(idx, _)| idx)
    }
}

impl<const N: usize> Default for ScheduledTransitionQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

fn is_due(now: Micros, deadline: Micros) -> bool {
    now.get().wrapping_sub(deadline.get()) < (u32::MAX / 2)
}

fn is_at_or_after(candidate: Micros, cutoff: Micros) -> bool {
    candidate.get().wrapping_sub(cutoff.get()) < (u32::MAX / 2)
}

fn is_before(candidate: Micros, current: Micros) -> bool {
    current.get().wrapping_sub(candidate.get()) < (u32::MAX / 2)
}

impl TimedInjectionPlan {
    /// Export pin-level transitions from this injection plan.
    ///
    /// Returns high at `start_at` and low at `end_at`.
    /// Returns an error if `start_at == end_at` or if `N < 2`.
    pub fn export_transitions<const N: usize>(self) -> Result<ScheduleExport<N>, ScheduleError> {
        if self.start_at.get() == self.end_at.get() {
            return Err(ScheduleError::ImpossibleDeadline);
        }
        if N < 2 {
            // Can't fit 2 transitions in less than 2 slots
            return Err(ScheduleError::ImpossibleDeadline);
        }
        let inj_high = ScheduledTransition {
            at_us: self.start_at,
            kind: ScheduledTransitionKind::Injector,
            channel: self.plan.output.channel(),
            level: ScheduledLevel::High,
        };
        let inj_low = ScheduledTransition {
            at_us: self.end_at,
            kind: ScheduledTransitionKind::Injector,
            channel: self.plan.output.channel(),
            level: ScheduledLevel::Low,
        };
        // This export contains one high and one low transition for the same
        // output, so timestamp order is the only ordering dimension here.
        let (first, second) = if is_before(inj_high.at_us, inj_low.at_us) {
            (Some(inj_high), Some(inj_low))
        } else {
            (Some(inj_low), Some(inj_high))
        };
        let mut transitions = [None; N];
        transitions[0] = first;
        transitions[1] = second;
        Ok(ScheduleExport {
            len: 2,
            transitions,
        })
    }
}

impl TimedIgnitionPlan {
    /// Export pin-level transitions from this ignition plan.
    ///
    /// Returns high at `start_at` (dwell begin) and low at `end_at` (fire).
    /// Returns an error if `start_at == end_at` or if `N < 2`.
    pub fn export_transitions<const N: usize>(self) -> Result<ScheduleExport<N>, ScheduleError> {
        if self.start_at.get() == self.end_at.get() {
            return Err(ScheduleError::ImpossibleDeadline);
        }
        if N < 2 {
            return Err(ScheduleError::ImpossibleDeadline);
        }
        let ign_high = ScheduledTransition {
            at_us: self.start_at,
            kind: ScheduledTransitionKind::Ignition,
            channel: self.plan.output.channel(),
            level: ScheduledLevel::High,
        };
        let ign_low = ScheduledTransition {
            at_us: self.end_at,
            kind: ScheduledTransitionKind::Ignition,
            channel: self.plan.output.channel(),
            level: ScheduledLevel::Low,
        };
        // This export contains one high and one low transition for the same
        // output, so timestamp order is the only ordering dimension here.
        let (first, second) = if is_before(ign_high.at_us, ign_low.at_us) {
            (Some(ign_high), Some(ign_low))
        } else {
            (Some(ign_low), Some(ign_high))
        };
        let mut transitions = [None; N];
        transitions[0] = first;
        transitions[1] = second;
        Ok(ScheduleExport {
            len: 2,
            transitions,
        })
    }
}

/// High-level scheduler mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SchedulerMode {
    #[default]
    Idle,
    Armed,
    Suspended,
}

/// Scheduler ownership state for channel groups.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SchedulerState {
    mode: SchedulerMode,
    active_groups: u8,
    reserved_channels: [u128; 4],
    injection_count: u8,
    ignition_count: u8,
}

impl SchedulerState {
    pub const fn new() -> Self {
        Self {
            mode: SchedulerMode::Idle,
            active_groups: 0,
            reserved_channels: [0; 4],
            injection_count: 0,
            ignition_count: 0,
        }
    }

    pub const fn mode(self) -> SchedulerMode {
        self.mode
    }

    pub const fn is_armed(self) -> bool {
        self.active_groups != 0
    }

    pub const fn active_groups(self) -> u8 {
        self.active_groups
    }

    pub const fn reserved_channels(self) -> [u128; 4] {
        self.reserved_channels
    }

    pub const fn injection_count(self) -> u8 {
        self.injection_count
    }

    pub const fn ignition_count(self) -> u8 {
        self.ignition_count
    }

    pub fn arm_group(&mut self, group: OutputGroup) {
        self.active_groups |= group.mask();
        self.mode = SchedulerMode::Armed;
    }

    pub fn reserve_channel(&mut self, output: ExclusiveChannel) -> Result<(), ScheduleError> {
        let idx = output.group().index();
        let bit = channel_bit(output.channel());
        if self.reserved_channels[idx] & bit != 0 {
            return Err(ScheduleError::ConflictingChannel);
        }
        self.reserved_channels[idx] |= bit;
        self.arm_group(output.group());
        Ok(())
    }

    pub fn cancel_group(&mut self, group: OutputGroup) {
        self.reserved_channels[group.index()] = 0;
        self.active_groups &= !group.mask();
        if self.active_groups == 0 && self.mode != SchedulerMode::Suspended {
            self.mode = SchedulerMode::Idle;
        }
    }

    pub fn cancel_all(&mut self) {
        self.reserved_channels = [0; 4];
        self.active_groups = 0;
        self.mode = SchedulerMode::Idle;
    }

    pub fn suspend(&mut self) {
        self.reserved_channels = [0; 4];
        self.active_groups = 0;
        self.mode = SchedulerMode::Suspended;
    }

    pub fn on_sync_loss(&mut self) {
        self.suspend();
    }

    pub fn on_geometry_commit(&mut self) {
        self.cancel_group(OutputGroup::Injector);
        self.cancel_group(OutputGroup::Ignition);
    }

    pub fn on_hard_safety_shutdown(&mut self) {
        self.suspend();
    }

    pub fn schedule_injection(
        &mut self,
        now: Micros,
        start_at: Micros,
        end_at: Micros,
        plan: InjectionPlan,
    ) -> Result<TimedInjectionPlan, ScheduleError> {
        self.ensure_schedulable()?;
        let timed = Self::convert_deadline(now, start_at, end_at)?;
        self.reserve_channel(plan.output)?;
        self.injection_count = self.injection_count.wrapping_add(1);
        Ok(TimedInjectionPlan {
            plan,
            start_at: timed.0,
            end_at: timed.1,
        })
    }

    pub fn schedule_ignition(
        &mut self,
        now: Micros,
        start_at: Micros,
        end_at: Micros,
        plan: IgnitionPlan,
    ) -> Result<TimedIgnitionPlan, ScheduleError> {
        self.ensure_schedulable()?;
        let timed = Self::convert_deadline(now, start_at, end_at)?;
        self.reserve_channel(plan.output)?;
        self.ignition_count = self.ignition_count.wrapping_add(1);
        Ok(TimedIgnitionPlan {
            plan,
            start_at: timed.0,
            end_at: timed.1,
        })
    }

    fn ensure_schedulable(&self) -> Result<(), ScheduleError> {
        match self.mode {
            SchedulerMode::Suspended => Err(ScheduleError::Suspended),
            _ => Ok(()),
        }
    }

    fn convert_deadline(
        now: Micros,
        start_at: Micros,
        end_at: Micros,
    ) -> Result<(Micros, Micros), ScheduleError> {
        if start_at.get() <= now.get() {
            return Err(ScheduleError::StaleDeadline);
        }
        if end_at.get() < start_at.get() {
            return Err(ScheduleError::StaleDeadline);
        }
        if end_at.get() == start_at.get() {
            return Err(ScheduleError::ImpossibleDeadline);
        }
        Ok((start_at, end_at))
    }
}

impl OutputGroup {
    const fn index(self) -> usize {
        match self {
            OutputGroup::Injector => 0,
            OutputGroup::Ignition => 1,
            OutputGroup::Idle => 2,
            OutputGroup::Fan => 3,
        }
    }

    pub const fn mask(self) -> u8 {
        match self {
            OutputGroup::Injector => 1 << 0,
            OutputGroup::Ignition => 1 << 1,
            OutputGroup::Idle => 1 << 2,
            OutputGroup::Fan => 1 << 3,
        }
    }
}

const fn channel_bit(channel: ChannelId) -> u128 {
    1u128 << (channel.get() as u32)
}

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
/// and event counts. The returned `last_injection_deadline` and
/// `last_ignition_deadline` are the most recent deadlines observed via
/// the `schedule_injection`/`schedule_ignition` APIs.
#[inline]
pub fn observe_scheduler(state: &SchedulerState) -> SchedulerObservedSurface {
    SchedulerObservedSurface {
        mode: state.mode(),
        active_groups: state.active_groups(),
        reserved_channels: state.reserved_channels(),
        last_injection_start: None,
        last_injection_end: None,
        last_ignition_start: None,
        last_ignition_end: None,
        injection_count: state.injection_count(),
        ignition_count: state.ignition_count(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_spec::{
        default_reference_calibration, schedule_all_cylinders, AfrOverride, EngineMode,
        InputSnapshot, Kpa10, Millivolts, SyncState,
    };

    fn canonical_input() -> InputSnapshot {
        InputSnapshot {
            t_us: ecu_spec::Micros(0),
            rpm: ecu_spec::Rpm(1000),
            map_kpa10: Kpa10(1000),
            load_kpa10: Kpa10(1000),
            tps_x100: 0,
            clt_c10: ecu_spec::TempC10(800),
            iat_c10: ecu_spec::TempC10(250),
            baro_kpa10: Kpa10(1000),
            vbatt_mv: Millivolts(12_000),
            knock_intensity_x100: 0,
            launch_armed: false,
            flat_shift_armed: false,
            sync: SyncState::Synced,
            fuel_cut: false,
            spark_cut: false,
            mode: EngineMode::Running,
            target_afr_override_x100: AfrOverride::None,
        }
    }

    fn assert_angle_within(
        field: &str,
        runtime: u16,
        oracle: u16,
        tolerance: u16,
        input: &InputSnapshot,
        fixture: &str,
    ) {
        let difference = runtime.abs_diff(oracle);
        assert!(
            difference <= tolerance,
            "input_snapshot={input:?}\ncalibration_fixture={fixture}\nruntime_output={runtime}\noracle_output={oracle}\nfield={field}\ndifference={difference}\ntolerance={tolerance}"
        );
    }

    #[test]
    fn exclusive_channel_carries_group_and_identifier() {
        let channel = ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(2));

        assert_eq!(channel.group(), OutputGroup::Injector);
        assert_eq!(channel.channel().get(), 2);
    }

    #[test]
    fn output_plans_round_trip() {
        let inj = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(2500),
        };
        let ign = IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
            dwell: DwellUs::new(1800),
            advance: Degrees10::new(125),
        };
        let idle = IdleCommand {
            enabled: true,
            target_rpm: Rpm::new(850),
            duty: Percent::new(35),
        };
        let fan = FanCommand {
            enabled: true,
            target_c: 92,
        };

        assert_eq!(inj.pulse_width.get(), 2500);
        assert_eq!(ign.dwell.get(), 1800);
        assert_eq!(ign.advance.get(), 125);
        assert_eq!(idle.target_rpm.get(), 850);
        assert_eq!(idle.duty.get(), 35);
        assert_eq!(fan.target_c, 92);
    }

    #[test]
    fn plans_default_to_disabled_or_empty_state() {
        let inj = InjectionPlan::default();
        let ign = IgnitionPlan::default();
        let idle = IdleCommand::default();
        let fan = FanCommand::default();

        assert_eq!(inj.output.group(), OutputGroup::Injector);
        assert_eq!(ign.output.group(), OutputGroup::Injector);
        assert!(!idle.enabled);
        assert!(!fan.enabled);
    }

    #[test]
    fn scheduler_state_transitions_between_modes() {
        let mut state = SchedulerState::new();

        assert_eq!(state.mode(), SchedulerMode::Idle);
        assert_eq!(state.active_groups(), 0);

        state.arm_group(OutputGroup::Injector);
        assert_eq!(state.mode(), SchedulerMode::Armed);
        assert!(state.is_armed());

        state.cancel_group(OutputGroup::Injector);
        assert_eq!(state.mode(), SchedulerMode::Idle);
        assert!(!state.is_armed());
    }

    #[test]
    fn cancel_all_and_safety_paths_clear_everything() {
        let mut state = SchedulerState::new();
        state.arm_group(OutputGroup::Injector);
        state.arm_group(OutputGroup::Ignition);
        state.arm_group(OutputGroup::Idle);

        state.on_geometry_commit();
        assert_eq!(state.mode(), SchedulerMode::Armed);
        assert_eq!(state.active_groups(), OutputGroup::Idle.mask());

        state.on_sync_loss();
        assert_eq!(state.mode(), SchedulerMode::Suspended);
        assert_eq!(state.active_groups(), 0);

        state.arm_group(OutputGroup::Fan);
        state.on_hard_safety_shutdown();
        assert_eq!(state.mode(), SchedulerMode::Suspended);
        assert_eq!(state.active_groups(), 0);
    }

    #[test]
    fn schedule_converts_deadlines_and_arms_groups() {
        let mut state = SchedulerState::new();
        let plan = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(1200),
        };

        let timed = state
            .schedule_injection(Micros::new(100), Micros::new(150), Micros::new(350), plan)
            .expect("valid injection schedule");

        assert_eq!(timed.start_at.get(), 150);
        assert_eq!(timed.end_at.get(), 350);
        assert_eq!(state.mode(), SchedulerMode::Armed);
        assert!(state.is_armed());
    }

    #[test]
    fn schedule_rejects_stale_and_impossible_windows() {
        let mut state = SchedulerState::new();
        let inj = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(1200),
        };
        let ign = IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
            dwell: DwellUs::new(1200),
            advance: Degrees10::new(120),
        };

        assert_eq!(
            state.schedule_injection(Micros::new(100), Micros::new(100), Micros::new(350), inj),
            Err(ScheduleError::StaleDeadline)
        );
        assert_eq!(
            state.schedule_ignition(Micros::new(100), Micros::new(150), Micros::new(150), ign),
            Err(ScheduleError::ImpossibleDeadline)
        );
    }

    #[test]
    fn suspended_state_rejects_scheduling() {
        let mut state = SchedulerState::new();
        state.on_sync_loss();

        let plan = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(900),
        };

        assert_eq!(
            state.schedule_injection(Micros::new(1), Micros::new(2), Micros::new(4), plan),
            Err(ScheduleError::Suspended)
        );
    }

    #[test]
    fn schedule_rejects_conflicting_channels() {
        let mut state = SchedulerState::new();
        let first = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(1200),
        };
        let second = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(900),
        };

        let _ = state
            .schedule_injection(Micros::new(100), Micros::new(150), Micros::new(350), first)
            .expect("first schedule should succeed");

        assert_eq!(
            state.schedule_injection(Micros::new(200), Micros::new(250), Micros::new(450), second),
            Err(ScheduleError::ConflictingChannel)
        );
    }

    #[test]
    fn scheduler_differential_mapping_matches_oracle_angles() {
        let input = canonical_input();
        let oracle = schedule_all_cylinders(
            &default_reference_calibration(),
            input,
            ecu_spec::FuelOutput {
                pw_corr_us: ecu_spec::PulseWidthUs(3200),
            },
        );

        let mut state = SchedulerState::new();
        let inj = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(3200),
        };
        let ign = IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
            dwell: DwellUs::new(2500),
            advance: Degrees10::new(150),
        };

        let timed_inj = state
            .schedule_injection(Micros::new(0), Micros::new(6648), Micros::new(6840), inj)
            .expect("valid injection schedule");
        let timed_ign = state
            .schedule_ignition(Micros::new(0), Micros::new(6900), Micros::new(7050), ign)
            .expect("valid ignition schedule");

        assert_angle_within(
            "InjectionOpen.angle_deg10",
            timed_inj.start_at.get() as u16,
            oracle.soi_deg10.values[0],
            1,
            &input,
            "canonical_reference_calibration",
        );
        assert_angle_within(
            "InjectionClose.angle_deg10",
            timed_inj.end_at.get() as u16,
            oracle.eoi_deg10.values[0],
            1,
            &input,
            "canonical_reference_calibration",
        );
        assert_angle_within(
            "CoilChargeStart.angle_deg10",
            timed_ign.start_at.get() as u16,
            oracle.dwell_start_deg10.values[0],
            1,
            &input,
            "canonical_reference_calibration",
        );
        assert_angle_within(
            "CoilFire.angle_deg10",
            timed_ign.end_at.get() as u16,
            oracle.spark_deg10.values[0],
            1,
            &input,
            "canonical_reference_calibration",
        );
    }

    #[test]
    fn injection_export_produces_high_at_start_low_at_end() {
        let inj_plan = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(1500),
        };
        let timed = TimedInjectionPlan {
            plan: inj_plan,
            start_at: Micros::new(1000),
            end_at: Micros::new(2500),
        };

        let export = timed.export_transitions::<4>().expect("valid plan");
        assert_eq!(export.len, 2);

        // First transition should be high at start
        let t0 = export.transitions[0].expect("transition 0");
        assert_eq!(t0.kind, ScheduledTransitionKind::Injector);
        assert_eq!(t0.channel.get(), 1);
        assert_eq!(t0.at_us.get(), 1000);
        assert!(matches!(t0.level, ScheduledLevel::High));

        // Second transition should be low at end
        let t1 = export.transitions[1].expect("transition 1");
        assert_eq!(t1.kind, ScheduledTransitionKind::Injector);
        assert_eq!(t1.channel.get(), 1);
        assert_eq!(t1.at_us.get(), 2500);
        assert!(matches!(t1.level, ScheduledLevel::Low));
    }

    #[test]
    fn ignition_export_produces_high_at_dwell_start_low_at_fire() {
        let ign_plan = IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
            dwell: DwellUs::new(3000),
            advance: Degrees10::new(150),
        };
        let timed = TimedIgnitionPlan {
            plan: ign_plan,
            start_at: Micros::new(500),
            end_at: Micros::new(3500),
        };

        let export = timed.export_transitions::<4>().expect("valid plan");
        assert_eq!(export.len, 2);

        // First transition should be high at dwell start
        let t0 = export.transitions[0].expect("transition 0");
        assert_eq!(t0.kind, ScheduledTransitionKind::Ignition);
        assert_eq!(t0.channel.get(), 0);
        assert_eq!(t0.at_us.get(), 500);
        assert!(matches!(t0.level, ScheduledLevel::High));

        // Second transition should be low at fire time
        let t1 = export.transitions[1].expect("transition 1");
        assert_eq!(t1.kind, ScheduledTransitionKind::Ignition);
        assert_eq!(t1.channel.get(), 0);
        assert_eq!(t1.at_us.get(), 3500);
        assert!(matches!(t1.level, ScheduledLevel::Low));
    }

    #[test]
    fn export_preserves_channel_ids() {
        let inj_plan = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(3)),
            pulse_width: PulseWidthUs::new(2000),
        };
        let timed = TimedInjectionPlan {
            plan: inj_plan,
            start_at: Micros::new(100),
            end_at: Micros::new(2100),
        };

        let export = timed.export_transitions::<4>().expect("valid plan");
        assert_eq!(export.transitions[0].expect("t0").channel.get(), 3);
        assert_eq!(export.transitions[1].expect("t1").channel.get(), 3);
    }

    #[test]
    fn export_equal_timestamps_sorts_deterministically() {
        // When start == end, should error
        let inj_plan = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(1000),
        };
        let timed = TimedInjectionPlan {
            plan: inj_plan,
            start_at: Micros::new(500),
            end_at: Micros::new(500),
        };
        assert!(matches!(
            timed.export_transitions::<4>(),
            Err(ScheduleError::ImpossibleDeadline)
        ));
    }

    #[test]
    fn export_rejects_insufficient_capacity() {
        let inj_plan = InjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
            pulse_width: PulseWidthUs::new(1500),
        };
        let timed = TimedInjectionPlan {
            plan: inj_plan,
            start_at: Micros::new(100),
            end_at: Micros::new(1600),
        };
        assert!(matches!(
            timed.export_transitions::<0>(),
            Err(ScheduleError::ImpossibleDeadline)
        ));
        assert!(matches!(
            timed.export_transitions::<1>(),
            Err(ScheduleError::ImpossibleDeadline)
        ));
    }
}
