use crate::{
    ChannelId, ExclusiveChannel, IgnitionPlan, InjectionPlan, Micros, OutputGroup, ScheduleError,
    ScheduledLevel, ScheduledTransition, ScheduledTransitionKind, TimedIgnitionPlan,
    TimedInjectionPlan,
};
use ecu_board_api::frontier::{
    TimingIslandHorizonSequenceId as FrontierHorizonSequenceId,
    TimingIslandPermitMask as FrontierPermitMask, TimingIslandStopReason as FrontierStopReason,
    HEARTBEAT_EXPIRY_US as FRONTIER_HEARTBEAT_EXPIRY_US, MAX_HORIZON_US as FRONTIER_MAX_HORIZON_US,
};

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
    scheduled_windows: [Option<ScheduledWindow>; MAX_SCHEDULED_WINDOWS],
    last_accepted_horizon_id: Option<FrontierHorizonSequenceId>,
    last_accepted_horizon_start_us: Option<Micros>,
    last_accepted_horizon_end_us: Option<Micros>,
    active_horizon_id: Option<FrontierHorizonSequenceId>,
    horizon_start_us: Option<Micros>,
    horizon_end_us: Option<Micros>,
    heartbeat_deadline_us: Option<Micros>,
    active_permit_mask: FrontierPermitMask,
    active_stop_reason: FrontierStopReason,
    injection_count: u8,
    ignition_count: u8,
    last_injection_start: Option<Micros>,
    last_injection_end: Option<Micros>,
    last_ignition_start: Option<Micros>,
    last_ignition_end: Option<Micros>,
}

impl SchedulerState {
    pub const fn new() -> Self {
        Self {
            mode: SchedulerMode::Idle,
            active_groups: 0,
            reserved_channels: [0; 4],
            scheduled_windows: [None; MAX_SCHEDULED_WINDOWS],
            last_accepted_horizon_id: None,
            last_accepted_horizon_start_us: None,
            last_accepted_horizon_end_us: None,
            active_horizon_id: None,
            horizon_start_us: None,
            horizon_end_us: None,
            heartbeat_deadline_us: None,
            active_permit_mask: FrontierPermitMask::NONE,
            active_stop_reason: FrontierStopReason::None,
            injection_count: 0,
            ignition_count: 0,
            last_injection_start: None,
            last_injection_end: None,
            last_ignition_start: None,
            last_ignition_end: None,
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

    pub const fn last_accepted_horizon_id(self) -> Option<FrontierHorizonSequenceId> {
        self.last_accepted_horizon_id
    }

    pub const fn last_accepted_horizon_start_us(self) -> Option<Micros> {
        self.last_accepted_horizon_start_us
    }

    pub const fn last_accepted_horizon_end_us(self) -> Option<Micros> {
        self.last_accepted_horizon_end_us
    }

    pub const fn active_horizon_id(self) -> Option<FrontierHorizonSequenceId> {
        self.active_horizon_id
    }

    pub const fn horizon_start_us(self) -> Option<Micros> {
        self.horizon_start_us
    }

    pub const fn horizon_end_us(self) -> Option<Micros> {
        self.horizon_end_us
    }

    pub const fn heartbeat_deadline_us(self) -> Option<Micros> {
        self.heartbeat_deadline_us
    }

    pub const fn active_permit_mask(self) -> FrontierPermitMask {
        self.active_permit_mask
    }

    pub const fn active_stop_reason(self) -> FrontierStopReason {
        self.active_stop_reason
    }

    pub const fn injection_count(self) -> u8 {
        self.injection_count
    }

    pub const fn ignition_count(self) -> u8 {
        self.ignition_count
    }

    pub const fn last_injection_start(self) -> Option<Micros> {
        self.last_injection_start
    }

    pub const fn last_injection_end(self) -> Option<Micros> {
        self.last_injection_end
    }

    pub const fn last_ignition_start(self) -> Option<Micros> {
        self.last_ignition_start
    }

    pub const fn last_ignition_end(self) -> Option<Micros> {
        self.last_ignition_end
    }

    pub fn arm_group(&mut self, group: OutputGroup) {
        self.active_groups |= group.mask();
        self.mode = SchedulerMode::Armed;
    }

    pub fn reserve_window(
        &mut self,
        output: ExclusiveChannel,
        start_at: Micros,
        end_at: Micros,
    ) -> Result<(), ScheduleError> {
        if end_at.get() <= start_at.get() {
            return Err(ScheduleError::ImpossibleDeadline);
        }

        channel_bit(output.channel())?;
        for existing in self.scheduled_windows.iter().flatten() {
            if existing.output == output
                && windows_overlap(start_at, end_at, existing.start_at, existing.end_at)
            {
                return Err(ScheduleError::ConflictingChannel);
            }
        }

        let Some(slot) = self
            .scheduled_windows
            .iter_mut()
            .find(|slot| slot.is_none())
        else {
            return Err(ScheduleError::QueueFull);
        };
        *slot = Some(ScheduledWindow {
            output,
            start_at,
            end_at,
        });
        self.arm_group(output.group());
        self.refresh_reserved_channels();
        self.refresh_counts();
        Ok(())
    }

    pub fn cancel_group(&mut self, group: OutputGroup) {
        self.scheduled_windows.iter_mut().for_each(|slot| {
            if slot.is_some_and(|window| window.output.group() == group) {
                *slot = None;
            }
        });
        self.reserved_channels[group.index()] = 0;
        self.active_groups &= !group.mask();
        if self.active_groups == 0 && self.mode != SchedulerMode::Suspended {
            self.mode = SchedulerMode::Idle;
        }
        self.refresh_counts();
    }

    pub fn cancel_all(&mut self) {
        self.clear_scheduled_ownership();
        self.clear_live_frontier_state(FrontierStopReason::PermitDenied);
    }

    pub fn suspend(&mut self) {
        self.clear_scheduled_ownership();
        self.mode = SchedulerMode::Suspended;
    }

    pub fn on_sync_loss(&mut self) {
        self.clear_live_frontier_state(FrontierStopReason::SyncLost);
        self.suspend();
    }

    pub fn on_sync_recovered(&mut self) {
        if self.mode == SchedulerMode::Suspended {
            self.mode = SchedulerMode::Idle;
        }
    }

    pub fn on_geometry_commit(&mut self) {
        self.cancel_group(OutputGroup::Injector);
        self.cancel_group(OutputGroup::Ignition);
    }

    pub fn on_hard_safety_shutdown(&mut self) {
        self.clear_live_frontier_state(FrontierStopReason::TimingFault);
        self.suspend();
    }

    pub fn commit_horizon(
        &mut self,
        horizon_id: FrontierHorizonSequenceId,
        horizon_start_us: Micros,
        horizon_end_us: Micros,
        heartbeat_deadline_us: Micros,
        permit_mask: FrontierPermitMask,
    ) -> bool {
        if horizon_end_us.get() <= horizon_start_us.get() {
            return false;
        }
        if horizon_end_us.get().saturating_sub(horizon_start_us.get())
            > FRONTIER_MAX_HORIZON_US.get()
        {
            return false;
        }
        if self
            .last_accepted_horizon_id
            .is_some_and(|last| horizon_id <= last)
        {
            return false;
        }

        self.last_accepted_horizon_id = Some(horizon_id);
        self.last_accepted_horizon_start_us = Some(horizon_start_us);
        self.last_accepted_horizon_end_us = Some(horizon_end_us);
        self.active_horizon_id = Some(horizon_id);
        self.horizon_start_us = Some(horizon_start_us);
        self.horizon_end_us = Some(horizon_end_us);
        self.heartbeat_deadline_us = Some(heartbeat_deadline_us);
        self.active_permit_mask = permit_mask;
        self.active_stop_reason = FrontierStopReason::None;
        true
    }

    pub fn note_heartbeat(&mut self, now: Micros) {
        if self.active_horizon_id.is_some() {
            self.heartbeat_deadline_us = Some(Micros::new(
                now.get().saturating_add(FRONTIER_HEARTBEAT_EXPIRY_US.get()),
            ));
        }
    }

    pub fn expire_frontier(&mut self, now: Micros) {
        self.expire_heartbeat(now);
        self.expire_horizon(now);
    }

    pub fn clear_live_frontier_state(&mut self, stop_reason: FrontierStopReason) {
        self.active_horizon_id = None;
        self.horizon_start_us = None;
        self.horizon_end_us = None;
        self.heartbeat_deadline_us = None;
        self.active_permit_mask = FrontierPermitMask::NONE;
        self.active_stop_reason = stop_reason;
    }

    pub fn expire_heartbeat(&mut self, now: Micros) -> bool {
        let Some(deadline) = self.heartbeat_deadline_us else {
            return false;
        };
        if now.get() <= deadline.get() {
            return false;
        }

        self.active_permit_mask = FrontierPermitMask::NONE;
        if self.active_stop_reason == FrontierStopReason::None {
            self.active_stop_reason = FrontierStopReason::HeartbeatExpired;
        }
        true
    }

    pub fn expire_horizon(&mut self, now: Micros) -> bool {
        let Some(end_us) = self.horizon_end_us else {
            return false;
        };
        if now.get() <= end_us.get() {
            return false;
        }

        self.clear_live_frontier_state(match self.active_stop_reason {
            FrontierStopReason::None => FrontierStopReason::HorizonExpired,
            reason => reason,
        });
        true
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
        self.reserve_window(plan.output, timed.0, timed.1)?;
        self.last_injection_start = Some(timed.0);
        self.last_injection_end = Some(timed.1);
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
        self.reserve_window(plan.output, timed.0, timed.1)?;
        self.last_ignition_start = Some(timed.0);
        self.last_ignition_end = Some(timed.1);
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

    pub fn note_drained_transition(&mut self, transition: ScheduledTransition) {
        if transition.level != ScheduledLevel::Low {
            return;
        }

        let group = match transition.kind {
            ScheduledTransitionKind::Injector => OutputGroup::Injector,
            ScheduledTransitionKind::Ignition => OutputGroup::Ignition,
        };
        let output = ExclusiveChannel::new(group, transition.channel);
        let mut released = false;
        for slot in &mut self.scheduled_windows {
            if slot
                .is_some_and(|window| window.output == output && window.end_at == transition.at_us)
            {
                *slot = None;
                released = true;
                break;
            }
        }
        if !released {
            return;
        }

        self.refresh_reserved_channels();
        self.sync_scheduled_group(OutputGroup::Injector);
        self.sync_scheduled_group(OutputGroup::Ignition);
        self.refresh_counts();
        self.refresh_mode_from_groups();
    }

    pub fn clear_scheduled_ownership(&mut self) {
        self.scheduled_windows = [None; MAX_SCHEDULED_WINDOWS];
        self.reserved_channels = [0; 4];
        self.active_groups = 0;
        self.refresh_counts();
        self.refresh_mode_from_groups();
    }

    fn refresh_reserved_channels(&mut self) {
        self.reserved_channels = [0; 4];
        for window in self.scheduled_windows.iter().flatten() {
            let idx = window.output.group().index();
            let Ok(bit) = channel_bit(window.output.channel()) else {
                continue;
            };
            self.reserved_channels[idx] |= bit;
        }
    }

    fn sync_scheduled_group(&mut self, group: OutputGroup) {
        if self.reserved_channels[group.index()] == 0 {
            self.active_groups &= !group.mask();
        } else {
            self.active_groups |= group.mask();
        }
    }

    fn refresh_mode_from_groups(&mut self) {
        if self.active_groups == 0 && self.mode != SchedulerMode::Suspended {
            self.mode = SchedulerMode::Idle;
        } else if self.active_groups != 0 && self.mode != SchedulerMode::Suspended {
            self.mode = SchedulerMode::Armed;
        }
    }

    fn refresh_counts(&mut self) {
        self.injection_count =
            self.reserved_channels[OutputGroup::Injector.index()].count_ones() as u8;
        self.ignition_count =
            self.reserved_channels[OutputGroup::Ignition.index()].count_ones() as u8;
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

const CHANNEL_BIT_WIDTH: u8 = u128::BITS as u8;
const MAX_SCHEDULED_WINDOWS: usize = ecu_board_api::FULL_ECU_MAX_CYLINDERS * 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct ScheduledWindow {
    output: ExclusiveChannel,
    start_at: Micros,
    end_at: Micros,
}

const fn channel_bit(channel: ChannelId) -> Result<u128, ScheduleError> {
    if channel.get() >= CHANNEL_BIT_WIDTH {
        return Err(ScheduleError::InvalidChannel);
    }
    Ok(1u128 << (channel.get() as u32))
}

const fn windows_overlap(
    candidate_start: Micros,
    candidate_end: Micros,
    existing_start: Micros,
    existing_end: Micros,
) -> bool {
    candidate_start.get() < existing_end.get() && existing_start.get() < candidate_end.get()
}
