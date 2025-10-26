//! DECIDE - Control Strategy & Algorithm Execution
//!
//! Determines optimal control actions based on engine context.

use super::orient::EngineContext;
use crate::{Corrections, ignition::IgnitionCorrections};

/// Fuel command output
#[derive(Debug, Clone, Copy)]
pub struct FuelCommand {
    /// IPW table to send to injection module
    pub ipw_table: [[u16; 16]; 16],
    /// Correction multipliers
    pub corrections: Corrections,
}

/// Ignition command output
#[derive(Debug, Clone, Copy)]
pub struct IgnitionCommand {
    /// Timing table (degrees BTDC)
    pub timing_table: [[i16; 16]; 16],
    /// Ignition corrections
    pub corrections: IgnitionCorrections,
    /// Dwell time (microseconds)
    pub dwell_us: u32,
}

/// Control decisions (output of Decide phase)
pub struct ControlDecisions {
    pub fuel: FuelCommand,
    pub ignition: IgnitionCommand,
}

/// Control strategy trait (extensible)
pub trait ControlStrategy {
    fn calculate_fuel(&self, ctx: &EngineContext) -> FuelCommand;
    fn calculate_ignition(&self, ctx: &EngineContext) -> IgnitionCommand;
}

/// Decider - executes control algorithms
pub struct Decider {
    // VE table for speed-density
    ve_table: [[u16; 16]; 16],
    // Ignition timing table
    timing_table: [[i16; 16]; 16],
}

impl Decider {
    /// Create new decider with default tables
    pub fn new() -> Self {
        Self {
            ve_table: [[80; 16]; 16],  // 80% VE default
            timing_table: [[15; 16]; 16],  // 15° BTDC default
        }
    }

    /// Compute control decisions
    pub fn compute_control(
        &self,
        ctx: &EngineContext,
    ) -> Result<ControlDecisions, DecideError> {
        let fuel = self.calculate_fuel(ctx);
        let ignition = self.calculate_ignition(ctx);

        Ok(ControlDecisions { fuel, ignition })
    }

    /// Calculate fuel command
    fn calculate_fuel(&self, ctx: &EngineContext) -> FuelCommand {
        // Simple VE → IPW conversion (placeholder for full algorithm)
        // Real implementation would use injector characterization
        let mut ipw_table = [[1000u16; 16]; 16];

        // Scale VE by load
        for row in 0..16 {
            for col in 0..16 {
                let ve_percent = self.ve_table[row][col];
                let base_pw = (ve_percent as u32 * 10) as u16;  // Simplified
                ipw_table[row][col] = base_pw;
            }
        }

        // Calculate corrections
        let clt_correction = self.calculate_clt_correction(ctx.coolant_temp_c);
        let iat_correction = self.calculate_iat_correction(ctx.intake_temp_c);
        let vbatt_correction = self.calculate_vbatt_correction(ctx.battery_voltage_mv);

        FuelCommand {
            ipw_table,
            corrections: Corrections::new(clt_correction, iat_correction, vbatt_correction),
        }
    }

    /// Calculate ignition command
    fn calculate_ignition(&self, ctx: &EngineContext) -> IgnitionCommand {
        // Calculate dwell based on battery voltage
        let dwell_us = crate::ignition::calculate_dwell(ctx.battery_voltage_mv);

        // Ignition corrections
        let clt_correction = self.calculate_timing_clt_correction(ctx.coolant_temp_c);

        IgnitionCommand {
            timing_table: self.timing_table,
            corrections: IgnitionCorrections {
                clt_correction,
                iat_correction: 0,
                knock_retard: 0,
            },
            dwell_us,
        }
    }

    /// CLT correction for fuel (100 = 1.0x)
    fn calculate_clt_correction(&self, clt_c: i16) -> u8 {
        if clt_c < 0 {
            150  // 1.5x enrichment when cold
        } else if clt_c < 60 {
            120  // 1.2x enrichment
        } else {
            100  // No correction when warm
        }
    }

    /// IAT correction for fuel
    fn calculate_iat_correction(&self, iat_c: i16) -> u8 {
        if iat_c > 40 {
            95  // Slight reduction when hot
        } else {
            100
        }
    }

    /// Battery voltage correction
    fn calculate_vbatt_correction(&self, vbatt_mv: u16) -> u8 {
        if vbatt_mv < 11000 {
            110  // More fuel at low voltage
        } else if vbatt_mv > 14000 {
            95   // Less fuel at high voltage
        } else {
            100
        }
    }

    /// CLT correction for timing (degrees)
    fn calculate_timing_clt_correction(&self, clt_c: i16) -> i16 {
        if clt_c < 60 {
            -5  // Retard 5° when cold
        } else {
            0
        }
    }
}

/// Decide errors
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecideError {
    /// Invalid context
    InvalidContext,
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::orient::{EngineContext, OperatingMode, LoadEstimate};
    use super::super::types::LoadMethod;

    fn create_test_context() -> EngineContext {
        EngineContext {
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
        }
    }

    #[test]
    fn test_decider_creation() {
        let decider = Decider::new();
        assert_eq!(decider.ve_table[0][0], 80);
    }

    #[test]
    fn test_compute_control() {
        let decider = Decider::new();
        let ctx = create_test_context();

        let result = decider.compute_control(&ctx);
        assert!(result.is_ok());

        let decisions = result.unwrap();
        assert_eq!(decisions.fuel.corrections.clt, 100);  // Warm engine
    }

    #[test]
    fn test_clt_correction_cold() {
        let decider = Decider::new();
        assert_eq!(decider.calculate_clt_correction(-10), 150);
        assert_eq!(decider.calculate_clt_correction(30), 120);
        assert_eq!(decider.calculate_clt_correction(80), 100);
    }
}
