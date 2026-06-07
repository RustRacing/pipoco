use crate::DriverError;
use ecu_io::{OutputLevel, OutputTransitionKind};
use ecu_sim::plant::{ClosedLoopPlant, FixedPlantProfile};

use crate::output_validation::validate_scenario_output_channel;

use super::{ScenarioConfig, ScenarioKind};

/// Pending output queue for sorting and due-time dispatch.
pub(super) struct PendingOutputQueue<const N: usize> {
    pub(super) events: [Option<ecu_io::OutputTransition>; N],
    pub(super) len: usize,
    overflow_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct OutputLevelTracker {
    injector: [OutputLevel; 16],
    ignition: [OutputLevel; 16],
    idle: [OutputLevel; 16],
    fan: [OutputLevel; 16],
}

impl OutputLevelTracker {
    pub(super) const fn new() -> Self {
        Self {
            injector: [OutputLevel::Low; 16],
            ignition: [OutputLevel::Low; 16],
            idle: [OutputLevel::Low; 16],
            fan: [OutputLevel::Low; 16],
        }
    }

    fn update(&mut self, event: ecu_io::OutputTransition) -> bool {
        let idx = event.channel.get() as usize;
        if idx >= 16 {
            return false;
        }
        let level = match event.kind {
            OutputTransitionKind::Injector => &mut self.injector[idx],
            OutputTransitionKind::Ignition => &mut self.ignition[idx],
            OutputTransitionKind::Idle => &mut self.idle[idx],
            OutputTransitionKind::Fan => &mut self.fan[idx],
        };
        if *level == event.level {
            false
        } else {
            *level = event.level;
            true
        }
    }
}

impl Default for OutputLevelTracker {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> PendingOutputQueue<N> {
    pub(super) fn new() -> Self {
        Self {
            events: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub(super) fn push_sorted(
        &mut self,
        event: ecu_io::OutputTransition,
    ) -> Result<(), DriverError> {
        if self.len >= N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(DriverError::PendingOutputOverflow);
        }

        // Insert in sorted order: (at_us, kind, channel, level)
        let mut idx = self.len;
        while idx > 0 {
            let Some(prev) = self.events[idx - 1] else {
                break;
            };
            if (
                event.at_us.get(),
                sort_key_kind(event.kind),
                event.channel.get(),
                sort_level(&event.level),
            ) >= (
                prev.at_us.get(),
                sort_key_kind(prev.kind),
                prev.channel.get(),
                sort_level(&prev.level),
            ) {
                break;
            }
            idx -= 1;
        }
        // Shift elements to make room
        let mut i = self.len;
        while i > idx {
            self.events[i] = self.events[i - 1];
            i -= 1;
        }
        self.events[idx] = Some(event);
        self.len += 1;
        Ok(())
    }

    pub(super) fn drain_due<const M: usize>(
        &mut self,
        now_us: u32,
        plant: &mut ClosedLoopPlant<FixedPlantProfile>,
        trace: &mut crate::trace::FixedDriverTrace<M>,
        levels: &mut OutputLevelTracker,
    ) -> Result<(), DriverError> {
        let profile = FixedPlantProfile::inline_four();
        let mut i = 0;
        while i < self.len {
            let Some(event) = self.events[i] else {
                i += 1;
                continue;
            };
            if event.at_us.get() <= now_us {
                validate_scenario_output_channel(profile, event)?;
                let changed = levels.update(event);
                if changed {
                    plant.apply_transition(event).map_err(DriverError::Plant)?;
                }
                trace.push(output_trace_record(event, if changed { 0 } else { 1 }))?;
                // Remove from queue
                let mut j = i;
                while j < self.len - 1 {
                    self.events[j] = self.events[j + 1];
                    j += 1;
                }
                self.events[self.len - 1] = None;
                self.len -= 1;
            } else {
                i += 1;
            }
        }
        Ok(())
    }

    pub(super) fn pending_overflow(&self) -> u32 {
        self.overflow_count
    }
}

impl<const N: usize> Default for PendingOutputQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

fn sort_key_kind(kind: OutputTransitionKind) -> i32 {
    match kind {
        OutputTransitionKind::Injector => 0,
        OutputTransitionKind::Ignition => 1,
        OutputTransitionKind::Idle => 2,
        OutputTransitionKind::Fan => 3,
    }
}

fn sort_level(level: &ecu_io::OutputLevel) -> i32 {
    match level {
        ecu_io::OutputLevel::Low => 0,
        ecu_io::OutputLevel::High => 1,
    }
}

pub(super) fn output_trace_record(
    event: ecu_io::OutputTransition,
    status: i32,
) -> crate::trace::DriverTraceRecord {
    crate::trace::DriverTraceRecord {
        at_us: event.at_us.get(),
        kind: crate::trace::DriverTraceKind::Output,
        status,
        rpm: 0,
        map_kpa10: 0,
        angle_x10: 0,
        output_kind: match event.kind {
            OutputTransitionKind::Injector => 0,
            OutputTransitionKind::Ignition => 1,
            OutputTransitionKind::Idle => 2,
            OutputTransitionKind::Fan => 3,
        },
        channel: event.channel.get(),
        high: match event.level {
            ecu_io::OutputLevel::Low => 0,
            ecu_io::OutputLevel::High => 1,
        },
        combustion_events: 0,
        synced: 0,
        tooth: 0,
        diagnostic_code: 0,
        fault_severity: 0,
        cancel_reason: 0,
        control_mode: 0,
        observability: crate::trace::DriverObservability::default(),
    }
}

pub(super) fn suppress_output_to_plant(config: ScenarioConfig, kind: OutputTransitionKind) -> bool {
    match kind {
        OutputTransitionKind::Injector => config.suppress_injection_to_plant,
        OutputTransitionKind::Ignition => config.suppress_ignition_to_plant,
        OutputTransitionKind::Idle | OutputTransitionKind::Fan => false,
    }
}

pub(super) fn scenario_initial_rpm(kind: ScenarioKind) -> u16 {
    match kind {
        ScenarioKind::ColdStart => 250,
        ScenarioKind::HotRestart | ScenarioKind::DfcoDecel | ScenarioKind::SyncLossRecovery => 850,
        ScenarioKind::Smoke => 850,
    }
}

pub(super) fn scenario_initial_clt_c10(kind: ScenarioKind) -> i16 {
    match kind {
        ScenarioKind::ColdStart => -120,
        ScenarioKind::HotRestart => 880,
        ScenarioKind::DfcoDecel => 820,
        ScenarioKind::SyncLossRecovery => 780,
        ScenarioKind::Smoke => 800,
    }
}

pub(super) fn scenario_initial_iat_c10(kind: ScenarioKind) -> i16 {
    match kind {
        ScenarioKind::ColdStart => -90,
        ScenarioKind::HotRestart => 600,
        ScenarioKind::DfcoDecel => 320,
        ScenarioKind::SyncLossRecovery => 280,
        ScenarioKind::Smoke => 250,
    }
}

pub(super) fn scenario_restart_step(kind: ScenarioKind) -> Option<u16> {
    match kind {
        ScenarioKind::HotRestart => Some(10),
        _ => None,
    }
}

pub(super) fn scenario_dfco_decel_step(kind: ScenarioKind) -> Option<u16> {
    match kind {
        ScenarioKind::DfcoDecel => Some(12),
        _ => None,
    }
}

pub(super) fn scenario_sync_loss_gap_step(kind: ScenarioKind) -> Option<u16> {
    match kind {
        ScenarioKind::SyncLossRecovery => Some(14),
        _ => None,
    }
}

pub(super) fn scenario_sync_loss_gap_us(kind: ScenarioKind) -> Option<u32> {
    match kind {
        ScenarioKind::SyncLossRecovery => Some(5_000),
        _ => None,
    }
}

pub(super) fn scenario_starter_on(
    config: ScenarioConfig,
    step_index: u16,
    restart_step: Option<u16>,
) -> bool {
    if step_index < config.starter_steps {
        return true;
    }
    match restart_step {
        Some(restart_step) => {
            let restart_end = restart_step.saturating_add(config.starter_steps);
            step_index >= restart_step && step_index < restart_end
        }
        None => false,
    }
}
