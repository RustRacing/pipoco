use ecu_board_api::{AuxCommandBatch, OutputTransition, OutputTransitionBatch, TelemetryFrame};
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, EngineTimeAuthority, Kpa10, Lambda100,
    Micros, Percent, PhaseSyncState, Rpm,
};

use crate::profile::Atmega2560DigitalPin;
use crate::profile::{MAX_AUX_COMMANDS, MAX_OUTPUT_TRANSITIONS};

/// Inputs already sampled by board-specific register glue.
///
/// This crate intentionally does not read ATmega registers. It is the portable
/// runtime-to-board boundary that a future HAL crate can call after sampling
/// timers, ADCs, trigger sync state, and cam state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atmega2560StepInput {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub crank_angle_x10: Degrees10,
    pub throttle: Percent,
    pub coolant_temp_c10: i16,
    pub intake_temp_c10: i16,
    pub battery_mv: u16,
    pub lambda: Lambda100,
    pub engine_time_authority: EngineTimeAuthority,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
}

impl Atmega2560StepInput {
    pub const fn expert_manual_authority() -> EngineTimeAuthority {
        EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::ExpertManual,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        )
    }

    pub const fn bench_synced(now_us: Micros, rpm: Rpm, load_kpa10: Kpa10) -> Self {
        Self {
            now_us,
            rpm,
            load_kpa10,
            crank_angle_x10: Degrees10::new(0),
            throttle: Percent::new(50),
            coolant_temp_c10: 850,
            intake_temp_c10: 300,
            battery_mv: 13_800,
            lambda: Lambda100::new(100),
            engine_time_authority: Self::expert_manual_authority(),
            launch_armed: false,
            flat_shift_armed: false,
        }
    }

    pub const fn with_engine_time_authority(self, authority: EngineTimeAuthority) -> Self {
        Self {
            now_us: self.now_us,
            rpm: self.rpm,
            load_kpa10: self.load_kpa10,
            crank_angle_x10: self.crank_angle_x10,
            throttle: self.throttle,
            coolant_temp_c10: self.coolant_temp_c10,
            intake_temp_c10: self.intake_temp_c10,
            battery_mv: self.battery_mv,
            lambda: self.lambda,
            engine_time_authority: authority,
            launch_armed: self.launch_armed,
            flat_shift_armed: self.flat_shift_armed,
        }
    }

    pub const fn with_launch_armed(self, launch_armed: bool) -> Self {
        Self {
            now_us: self.now_us,
            rpm: self.rpm,
            load_kpa10: self.load_kpa10,
            crank_angle_x10: self.crank_angle_x10,
            throttle: self.throttle,
            coolant_temp_c10: self.coolant_temp_c10,
            intake_temp_c10: self.intake_temp_c10,
            battery_mv: self.battery_mv,
            lambda: self.lambda,
            engine_time_authority: self.engine_time_authority,
            launch_armed,
            flat_shift_armed: self.flat_shift_armed,
        }
    }

    pub const fn with_flat_shift_armed(self, flat_shift_armed: bool) -> Self {
        Self {
            now_us: self.now_us,
            rpm: self.rpm,
            load_kpa10: self.load_kpa10,
            crank_angle_x10: self.crank_angle_x10,
            throttle: self.throttle,
            coolant_temp_c10: self.coolant_temp_c10,
            intake_temp_c10: self.intake_temp_c10,
            battery_mv: self.battery_mv,
            lambda: self.lambda,
            engine_time_authority: self.engine_time_authority,
            launch_armed: self.launch_armed,
            flat_shift_armed,
        }
    }
}

/// Fixed-capacity board command set emitted by one runtime step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atmega2560StepOutput {
    pub outputs: OutputTransitionBatch<MAX_OUTPUT_TRANSITIONS>,
    pub aux: AuxCommandBatch<MAX_AUX_COMMANDS>,
    pub telemetry: TelemetryFrame,
    pub cancel_scheduled_outputs: bool,
    pub persist_calibration: bool,
}

impl Atmega2560StepOutput {
    pub const fn new(telemetry: TelemetryFrame) -> Self {
        Self {
            outputs: OutputTransitionBatch::new(),
            aux: AuxCommandBatch::new(),
            telemetry,
            cancel_scheduled_outputs: false,
            persist_calibration: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Atmega2560MappedOutputTransition {
    pub transition: OutputTransition,
    pub pin: Atmega2560DigitalPin,
}

impl Atmega2560MappedOutputTransition {
    pub const EMPTY: Self = Self {
        transition: OutputTransition::EMPTY,
        pin: Atmega2560DigitalPin::new(0),
    };
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Atmega2560MappedOutputBatch<const N: usize> {
    len: usize,
    items: [Atmega2560MappedOutputTransition; N],
}

impl<const N: usize> Atmega2560MappedOutputBatch<N> {
    pub const fn new() -> Self {
        Self {
            len: 0,
            items: [Atmega2560MappedOutputTransition::EMPTY; N],
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn push(&mut self, item: Atmega2560MappedOutputTransition) -> Result<(), ()> {
        if self.len == N {
            return Err(());
        }

        self.items[self.len] = item;
        self.len += 1;
        Ok(())
    }

    pub fn iter(&self) -> core::slice::Iter<'_, Atmega2560MappedOutputTransition> {
        self.items[..self.len].iter()
    }

    pub fn as_slice(&self) -> &[Atmega2560MappedOutputTransition] {
        &self.items[..self.len]
    }
}

impl<const N: usize> Default for Atmega2560MappedOutputBatch<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Atmega2560MappedStepOutput {
    pub mapped_outputs: Atmega2560MappedOutputBatch<MAX_OUTPUT_TRANSITIONS>,
    pub step: Atmega2560StepOutput,
}
