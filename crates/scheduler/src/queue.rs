use crate::state::SchedulerState;
use crate::{ChannelId, Micros, OutputGroup, ScheduleError, TimedIgnitionPlan, TimedInjectionPlan};

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

impl Ord for ScheduledTransition {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.at_us
            .get()
            .cmp(&other.at_us.get())
            .then_with(|| self.kind.cmp(&other.kind))
            .then_with(|| self.channel.get().cmp(&other.channel.get()))
            .then_with(|| level_rank(self.level).cmp(&level_rank(other.level)))
    }
}

impl PartialOrd for ScheduledTransition {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
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
    pub const MAX_METADATA_CAPACITY: usize = u8::MAX as usize;

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
        if !Self::metadata_capacity_supported() {
            return Err(ScheduleError::QueueFull);
        }
        let idx = self.len as usize;
        if idx >= N {
            return Err(ScheduleError::QueueFull);
        }
        self.transitions[idx] = Some(transition);
        self.len += 1;
        Ok(())
    }

    pub fn as_slice(&self) -> &[Option<ScheduledTransition>] {
        &self.transitions[..self.len as usize]
    }

    pub const fn metadata_capacity_supported() -> bool {
        N <= Self::MAX_METADATA_CAPACITY
    }
}

impl<const N: usize> Default for TransitionDrainBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Queue-owned timing metrics recorded by scheduler queue operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledTimingMetrics {
    pub late_event_count: u32,
    pub max_lateness_us: Option<Micros>,
    pub queue_high_water_mark: u8,
    pub last_drain_count: u8,
}

impl ScheduledTimingMetrics {
    pub const fn new() -> Self {
        Self {
            late_event_count: 0,
            max_lateness_us: None,
            queue_high_water_mark: 0,
            last_drain_count: 0,
        }
    }
}

impl Default for ScheduledTimingMetrics {
    fn default() -> Self {
        Self::new()
    }
}

/// Read-only queue observation for diagnostics and tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledTransitionQueueSnapshot<const N: usize> {
    pub len: u8,
    pub transitions: [Option<ScheduledTransition>; N],
    pub late_event_count: u32,
    pub max_lateness_us: Option<Micros>,
    pub queue_high_water_mark: u8,
    pub last_drain_count: u8,
}

/// Fixed-capacity transition queue for live scheduler execution.
///
/// The queue owns deadline ordering and cancellation, but it never drives pins.
/// Targets drain due transitions and apply the `ScheduledLevel` elsewhere.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ScheduledTransitionQueue<const N: usize> {
    transitions: [Option<ScheduledTransition>; N],
    metrics: ScheduledTimingMetrics,
}

impl<const N: usize> ScheduledTransitionQueue<N> {
    pub const MAX_METADATA_CAPACITY: usize = u8::MAX as usize;

    pub const fn new() -> Self {
        Self {
            transitions: [None; N],
            metrics: ScheduledTimingMetrics::new(),
        }
    }

    pub fn enqueue_transition(
        &mut self,
        transition: ScheduledTransition,
    ) -> Result<(), ScheduleError> {
        if !Self::metadata_capacity_supported() {
            return Err(ScheduleError::QueueFull);
        }
        for slot in &mut self.transitions {
            if slot.is_none() {
                *slot = Some(transition);
                self.record_enqueue_metrics();
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
        if !TransitionDrainBuffer::<M>::metadata_capacity_supported() {
            self.metrics.last_drain_count = 0;
            return 0;
        }
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
                self.record_drained_transition(now, transition);
                let _ = out.push(transition);
                drained += 1;
            }
        }
        self.metrics.last_drain_count = drained as u8;
        drained
    }

    pub fn drain_due_with_frontier<const M: usize>(
        &mut self,
        now: Micros,
        frontier: &mut SchedulerState,
        out: &mut TransitionDrainBuffer<M>,
    ) -> usize {
        frontier.expire_frontier(now);
        if frontier.active_horizon_id().is_none() || frontier.active_permit_mask().is_empty() {
            self.cancel_all();
            out.clear();
            self.metrics.last_drain_count = 0;
            return 0;
        }

        self.drain_due(now, out)
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
        debug_assert!(Self::metadata_capacity_supported());
        ScheduledTransitionQueueSnapshot {
            len: self.active_count() as u8,
            transitions: self.transitions,
            late_event_count: self.metrics.late_event_count,
            max_lateness_us: self.metrics.max_lateness_us,
            queue_high_water_mark: self.metrics.queue_high_water_mark,
            last_drain_count: self.metrics.last_drain_count,
        }
    }

    pub const fn timing_metrics(&self) -> ScheduledTimingMetrics {
        self.metrics
    }

    pub const fn metadata_capacity_supported() -> bool {
        N <= Self::MAX_METADATA_CAPACITY
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
                    if is_before(transition.at_us, current.at_us)
                        || (transition.at_us == current.at_us && transition < current)
                    {
                        selected = Some((idx, transition));
                    }
                }
            }
        }
        selected.map(|(idx, _)| idx)
    }

    fn record_enqueue_metrics(&mut self) {
        let active_count = self.active_count() as u8;
        if active_count > self.metrics.queue_high_water_mark {
            self.metrics.queue_high_water_mark = active_count;
        }
    }

    fn record_drained_transition(&mut self, now: Micros, transition: ScheduledTransition) {
        let lateness_us = now.get().wrapping_sub(transition.at_us.get());
        if lateness_us == 0 {
            return;
        }

        self.metrics.late_event_count = self.metrics.late_event_count.saturating_add(1);
        let should_update = self
            .metrics
            .max_lateness_us
            .is_none_or(|max_lateness| lateness_us > max_lateness.get());
        if should_update {
            self.metrics.max_lateness_us = Some(Micros::new(lateness_us));
        }
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

const fn level_rank(level: ScheduledLevel) -> u8 {
    match level {
        ScheduledLevel::High => 0,
        ScheduledLevel::Low => 1,
    }
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
