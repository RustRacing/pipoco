//! History/Telemetry System
//!
//! Configurable data logging with memory-aware buffering.

use super::orient::EngineContext;
use super::decide::ControlDecisions;

/// History channel selection
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HistoryChannel {
    RPM,
    MAP,
    TPS,
    CLT,
    IAT,
    AFR,
    FuelPW,
    IgnitionTiming,
    LoadPercent,
}

/// History configuration
pub struct HistoryConfig {
    pub enabled: bool,
    pub sample_rate_divider: u16,  // Sample every Nth cycle
}

impl HistoryConfig {
    pub fn tier_1() -> Self {
        Self {
            enabled: false,
            sample_rate_divider: 1,
        }
    }

    pub fn tier_2() -> Self {
        Self {
            enabled: true,
            sample_rate_divider: 10,  // Sample every 10th cycle
        }
    }
}

/// Circular history buffer
pub struct History<const N: usize> {
    // Simple ring buffers for key parameters
    rpm: [u16; N],
    map: [u16; N],
    tps: [u8; N],
    index: usize,
    wrapped: bool,
    sample_counter: u16,
    sample_rate_divider: u16,
}

impl<const N: usize> History<N> {
    /// Create new history buffer
    pub fn new() -> Self {
        Self {
            rpm: [0; N],
            map: [0; N],
            tps: [0; N],
            index: 0,
            wrapped: false,
            sample_counter: 0,
            sample_rate_divider: 10,
        }
    }

    /// Record context and decisions
    pub fn record(&mut self, ctx: &EngineContext, _decisions: &ControlDecisions) {
        self.sample_counter += 1;
        if self.sample_counter < self.sample_rate_divider {
            return;
        }
        self.sample_counter = 0;

        self.rpm[self.index] = ctx.rpm;
        self.map[self.index] = ctx.load.map_kpa;
        self.tps[self.index] = ctx.load.tps_percent;

        self.index += 1;
        if self.index >= N {
            self.index = 0;
            self.wrapped = true;
        }
    }

    /// Get latest value for channel
    pub fn get_latest(&self, channel: HistoryChannel) -> Option<u16> {
        if self.index == 0 && !self.wrapped {
            return None;
        }

        let latest_idx = if self.index > 0 {
            self.index - 1
        } else {
            N - 1
        };

        match channel {
            HistoryChannel::RPM => Some(self.rpm[latest_idx]),
            HistoryChannel::MAP => Some(self.map[latest_idx]),
            HistoryChannel::TPS => Some(self.tps[latest_idx] as u16),
            _ => None,
        }
    }

    /// Get sample count
    pub fn sample_count(&self) -> usize {
        if self.wrapped {
            N
        } else {
            self.index
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::orient::{EngineContext, OperatingMode, LoadEstimate};
    use super::super::types::LoadMethod;
    use super::super::decide::{ControlDecisions, FuelCommand, IgnitionCommand};
    use crate::{Corrections, ignition::IgnitionCorrections};

    #[test]
    fn test_history_creation() {
        let history = History::<100>::new();
        assert_eq!(history.sample_count(), 0);
    }

    #[test]
    fn test_history_recording() {
        let mut history = History::<10>::new();

        let ctx = EngineContext {
            timestamp_us: 1000,
            operating_mode: OperatingMode::Cruise,
            rpm: 3000,
            load: LoadEstimate {
                map_kpa: 100,
                tps_percent: 50,
                calculated_load: 100,
                method: LoadMethod::MAP,
            },
            coolant_temp_c: 80,
            intake_temp_c: 25,
            battery_voltage_mv: 13500,
            afr: None,
            confidence: 255,
        };

        let decisions = ControlDecisions {
            fuel: FuelCommand {
                ipw_table: [[1000; 16]; 16],
                corrections: Corrections::DEFAULT,
            },
            ignition: IgnitionCommand {
                timing_table: [[15; 16]; 16],
                corrections: IgnitionCorrections::DEFAULT,
                dwell_us: 3000,
            },
        };

        // Record samples (accounting for divider)
        for _ in 0..30 {  // 30 cycles = 3 samples (divider = 10)
            history.record(&ctx, &decisions);
        }

        assert_eq!(history.sample_count(), 3);
        assert_eq!(history.get_latest(HistoryChannel::RPM), Some(3000));
    }
}
