use ecu_domain::{Kpa10, PulseWidthUs, Rpm};
use ecu_runtime::{BaseFuelModel, EngineRuntime, RuntimeScheduledOutputKind};
use ecu_sim::SimulationHarness;

use crate::{
    EcuSimInitCfg, EcuSimOutputEvent, EcuSimSensorFrame, EcuSimStatus, CRANK_TEETH_PER_REV,
    ECU_SIM_MAX_CHANNELS, ECU_SIM_MAX_EVENTS, ECU_SIM_MAX_FIRING_ORDER, FAST_QUEUE_CAP,
    SLOW_QUEUE_CAP,
};
#[cfg(any(test, feature = "test-support"))]
use ecu_domain::{CancelReason, FaultCode, FaultSeverity};

pub(super) const MIN_SYNC_LOSS_TIMEOUT_US: u32 = 4_000;
pub(super) const SYNC_LOSS_PERIOD_MULTIPLIER: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RuntimeConfig {
    pub(super) has_cam: bool,
    inj_count: u8,
    inj_channels: [u8; ECU_SIM_MAX_CHANNELS],
    ign_count: u8,
    ign_channels: [u8; ECU_SIM_MAX_CHANNELS],
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            has_cam: true,
            inj_count: 1,
            inj_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ign_count: 1,
            ign_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
    }
}

impl RuntimeConfig {
    pub(crate) fn from_ffi(cfg: EcuSimInitCfg) -> Result<Self, EcuSimStatus> {
        if cfg.has_cam > 1 {
            return Err(EcuSimStatus::ErrInvalid);
        }
        validate_mode(
            cfg.inj_mode,
            crate::EcuSimInjMode::Batch as i32,
            crate::EcuSimInjMode::Sequential as i32,
        )?;
        validate_mode(
            cfg.ign_mode,
            crate::EcuSimIgnMode::Wasted as i32,
            crate::EcuSimIgnMode::Sequential as i32,
        )?;

        if cfg.cylinders == 0 || usize::from(cfg.cylinders) > ECU_SIM_MAX_FIRING_ORDER {
            return Err(EcuSimStatus::ErrInvalid);
        }
        if cfg.firing_len != cfg.cylinders {
            return Err(EcuSimStatus::ErrInvalid);
        }
        if usize::from(cfg.inj_count) > ECU_SIM_MAX_CHANNELS
            || usize::from(cfg.ign_count) > ECU_SIM_MAX_CHANNELS
            || cfg.inj_count == 0
            || cfg.ign_count == 0
        {
            return Err(EcuSimStatus::ErrInvalid);
        }

        let mut seen_cylinders = [false; ECU_SIM_MAX_FIRING_ORDER + 1];
        let mut idx = 0usize;
        while idx < usize::from(cfg.firing_len) {
            let cyl = cfg.firing_order[idx];
            if cyl == 0 || cyl > cfg.cylinders {
                return Err(EcuSimStatus::ErrInvalid);
            }
            let cyl_index = usize::from(cyl);
            if seen_cylinders[cyl_index] {
                return Err(EcuSimStatus::ErrInvalid);
            }
            seen_cylinders[cyl_index] = true;
            idx += 1;
        }
        validate_channel_map(&cfg.inj_channels, cfg.inj_count)?;
        validate_channel_map(&cfg.ign_channels, cfg.ign_count)?;

        Ok(Self {
            has_cam: cfg.has_cam != 0,
            inj_count: cfg.inj_count,
            inj_channels: cfg.inj_channels,
            ign_count: cfg.ign_count,
            ign_channels: cfg.ign_channels,
        })
    }

    pub(super) fn map_channel(self, kind: RuntimeScheduledOutputKind, runtime_channel: u8) -> u8 {
        let index = usize::from(runtime_channel.saturating_sub(1));
        match kind {
            RuntimeScheduledOutputKind::Injector if index < usize::from(self.inj_count) => {
                self.inj_channels[index]
            }
            RuntimeScheduledOutputKind::Ignition if index < usize::from(self.ign_count) => {
                self.ign_channels[index]
            }
            _ => runtime_channel,
        }
    }
}

fn validate_channel_map(
    channels: &[u8; ECU_SIM_MAX_CHANNELS],
    count: u8,
) -> Result<(), EcuSimStatus> {
    let mut seen = [false; u8::MAX as usize + 1];
    let mut idx = 0usize;
    while idx < usize::from(count) {
        let channel = channels[idx];
        if channel == 0 {
            return Err(EcuSimStatus::ErrInvalid);
        }
        let channel_index = usize::from(channel);
        if seen[channel_index] {
            return Err(EcuSimStatus::ErrInvalid);
        }
        seen[channel_index] = true;
        idx += 1;
    }
    Ok(())
}

fn validate_mode(value: i32, low: i32, high: i32) -> Result<(), EcuSimStatus> {
    if value < low || value > high {
        Err(EcuSimStatus::ErrInvalid)
    } else {
        Ok(())
    }
}

#[derive(Debug)]
pub(super) struct OutputEventQueue<const N: usize> {
    events: [EcuSimOutputEvent; N],
    len: usize,
    pub(super) overflow_count: u32,
}

impl<const N: usize> OutputEventQueue<N> {
    const fn new() -> Self {
        Self {
            events: [EcuSimOutputEvent::ZERO; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub(super) fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub(super) fn push_sorted(&mut self, event: EcuSimOutputEvent) -> Result<(), ()> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(());
        }

        let mut index = self.len;
        while index > 0 && event_less(event, self.events[index - 1]) {
            self.events[index] = self.events[index - 1];
            index -= 1;
        }
        self.events[index] = event;
        self.len += 1;
        Ok(())
    }

    pub(super) fn pop_front(&mut self) -> Option<EcuSimOutputEvent> {
        if self.len == 0 {
            return None;
        }
        let event = self.events[0];
        let mut index = 1usize;
        while index < self.len {
            self.events[index - 1] = self.events[index];
            index += 1;
        }
        self.len -= 1;
        Some(event)
    }
}

impl<const N: usize> Default for OutputEventQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) fn event_less(left: EcuSimOutputEvent, right: EcuSimOutputEvent) -> bool {
    (left.time_us, left.kind, left.channel, left.high)
        < (right.time_us, right.kind, right.channel, right.high)
}

#[derive(Debug)]
pub(crate) struct EcuSimHandle {
    pub(super) initialized: bool,
    pub(super) sim: SimulationHarness<FAST_QUEUE_CAP, SLOW_QUEUE_CAP>,
    pub(super) outputs: OutputEventQueue<ECU_SIM_MAX_EVENTS>,
    pub(super) cfg: RuntimeConfig,
    pub(super) now_us: u32,
    pub(super) sensors: EcuSimSensorFrame,
    pub(super) rpm: u16,
    pub(super) tooth: u8,
    pub(super) angle_x10: i16,
    pub(super) synced: bool,
    pub(super) last_crank_edge_us: Option<u32>,
    pub(super) last_crank_period_us: Option<u32>,
    pub(super) overflow_latched: bool,
    #[cfg(any(test, feature = "test-support"))]
    pub(super) fault_override: Option<(FaultCode, FaultSeverity, CancelReason)>,
}

impl EcuSimHandle {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn init(&mut self, cfg: RuntimeConfig) {
        *self = Self {
            initialized: true,
            sim: new_sim_harness(),
            outputs: OutputEventQueue::new(),
            cfg,
            now_us: 0,
            sensors: EcuSimSensorFrame::default(),
            rpm: 0,
            tooth: 0,
            angle_x10: 0,
            synced: false,
            last_crank_edge_us: None,
            last_crank_period_us: None,
            overflow_latched: false,
            #[cfg(any(test, feature = "test-support"))]
            fault_override: None,
        };
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    #[cfg(any(test, feature = "test-support"))]
    pub(crate) fn initialized(&self) -> bool {
        self.initialized
    }

    pub(crate) fn require_init(&self) -> Result<(), EcuSimStatus> {
        if self.initialized {
            Ok(())
        } else {
            Err(EcuSimStatus::ErrNotInit)
        }
    }
}

impl Default for EcuSimHandle {
    fn default() -> Self {
        Self {
            initialized: false,
            sim: new_sim_harness(),
            outputs: OutputEventQueue::new(),
            cfg: RuntimeConfig::default(),
            now_us: 0,
            sensors: EcuSimSensorFrame::default(),
            rpm: 0,
            tooth: 0,
            angle_x10: 0,
            synced: false,
            last_crank_edge_us: None,
            last_crank_period_us: None,
            overflow_latched: false,
            #[cfg(any(test, feature = "test-support"))]
            fault_override: None,
        }
    }
}

fn new_sim_harness() -> SimulationHarness<FAST_QUEUE_CAP, SLOW_QUEUE_CAP> {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(default_ffi_fuel_model());
    SimulationHarness::new(runtime)
}

fn default_ffi_fuel_model() -> BaseFuelModel {
    BaseFuelModel::new(
        [
            Rpm::new(0),
            Rpm::new(500),
            Rpm::new(1_000),
            Rpm::new(1_500),
            Rpm::new(2_000),
            Rpm::new(2_500),
            Rpm::new(3_000),
            Rpm::new(3_500),
            Rpm::new(4_000),
            Rpm::new(4_500),
            Rpm::new(5_000),
            Rpm::new(5_500),
            Rpm::new(6_000),
            Rpm::new(6_500),
            Rpm::new(7_000),
            Rpm::new(7_500),
        ],
        [
            Kpa10::new(0),
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1_000),
            Kpa10::new(1_100),
            Kpa10::new(1_200),
            Kpa10::new(1_300),
            Kpa10::new(1_400),
            Kpa10::new(1_500),
            Kpa10::new(1_600),
        ],
        [[PulseWidthUs::new(2_500); 16]; 16],
    )
}

pub(super) fn rpm_from_tooth_period(period_us: u32) -> u16 {
    if period_us == 0 {
        return 0;
    }
    let rpm = 60_000_000u32 / period_us / CRANK_TEETH_PER_REV;
    rpm.min(u32::from(u16::MAX)) as u16
}
