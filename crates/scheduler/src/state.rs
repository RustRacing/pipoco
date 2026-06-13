use crate::{
    ChannelId, ExclusiveChannel, IgnitionPlan, InjectionPlan, Micros, OutputGroup, ScheduleError,
    TimedIgnitionPlan, TimedInjectionPlan,
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

    pub fn reserve_channel(&mut self, output: ExclusiveChannel) -> Result<(), ScheduleError> {
        let idx = output.group().index();
        let bit = channel_bit(output.channel())?;
        if self.reserved_channels[idx] & bit != 0 {
            return Err(ScheduleError::ConflictingChannel);
        }
        self.reserved_channels[idx] |= bit;
        self.arm_group(output.group());
        self.refresh_counts();
        Ok(())
    }

    pub fn cancel_group(&mut self, group: OutputGroup) {
        self.reserved_channels[group.index()] = 0;
        self.active_groups &= !group.mask();
        if self.active_groups == 0 && self.mode != SchedulerMode::Suspended {
            self.mode = SchedulerMode::Idle;
        }
        self.refresh_counts();
    }

    pub fn cancel_all(&mut self) {
        self.reserved_channels = [0; 4];
        self.active_groups = 0;
        self.mode = SchedulerMode::Idle;
        self.refresh_counts();
    }

    pub fn suspend(&mut self) {
        self.reserved_channels = [0; 4];
        self.active_groups = 0;
        self.mode = SchedulerMode::Suspended;
        self.refresh_counts();
    }

    pub fn on_sync_loss(&mut self) {
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
        self.reserve_channel(plan.output)?;
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

const fn channel_bit(channel: ChannelId) -> Result<u128, ScheduleError> {
    if channel.get() >= CHANNEL_BIT_WIDTH {
        return Err(ScheduleError::InvalidChannel);
    }
    Ok(1u128 << (channel.get() as u32))
}
