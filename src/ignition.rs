//! Ignition timing and dwell control
//!
//! This module handles spark timing and coil dwell (charge time) calculations.
//! Uses a 2D table for base timing with corrections for temperature and knock.
//!
//! # Timing Convention
//!
//! - Timing is in degrees **Before Top Dead Center (BTDC)**
//! - Positive values = advance (spark before TDC)
//! - Negative values = retard (spark after TDC)
//! - Example: 15° BTDC means spark fires 15° before piston reaches TDC
//!
//! # Dwell Control
//!
//! Dwell is the time the coil is charged before firing. Too short = weak spark,
//! too long = coil overheating. Dwell is compensated for battery voltage:
//! - Low voltage = longer dwell (more time to build energy)
//! - High voltage = shorter dwell (builds energy faster)

use crate::constants::ignition::*;
#[cfg(feature = "interp-bilinear")]
use crate::ve_engine::interpolation::{bilinear_interpolate_i16, find_bin_interpolation};

/// Ignition timing table (16x16 grid)
///
/// Values are timing in degrees BTDC (Before Top Dead Center).
/// Table is indexed by [load_idx][rpm_idx].
pub struct IgnitionTable {
    pub rpm_bins: [u16; 16],
    pub load_bins: [u16; 16],
    pub values: [[i16; 16]; 16], // Degrees BTDC (can be negative for retard)
}

impl IgnitionTable {
    /// Create new ignition table with default values
    pub const fn new() -> Self {
        Self {
            rpm_bins: crate::constants::fuel::RPM_BINS, // Reuse RPM bins
            load_bins: crate::constants::fuel::LOAD_BINS, // Reuse load bins
            values: [[DEFAULT_TIMING_BTDC; 16]; 16],
        }
    }

    /// Lookup ignition timing (no interpolation for MVP)
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Timing in degrees BTDC (positive = advance, negative = retard)
    pub fn lookup(&self, rpm: u16, load: u16) -> i16 {
        #[cfg(feature = "interp-bilinear")]
        {
            let (rx0, rx1, fx) = find_bin_interpolation(&self.rpm_bins, rpm);
            let (ly0, ly1, fy) = find_bin_interpolation(&self.load_bins, load);
            let v00 = self.values[ly0][rx0];
            let v01 = self.values[ly0][rx1];
            let v10 = self.values[ly1][rx0];
            let v11 = self.values[ly1][rx1];
            bilinear_interpolate_i16(v00, v01, v10, v11, fx, fy)
        }
        #[cfg(not(feature = "interp-bilinear"))]
        {
            let rpm_idx = self.find_index(&self.rpm_bins, rpm);
            let load_idx = self.find_index(&self.load_bins, load);
            self.values[load_idx][rpm_idx]
        }
    }

    #[cfg(not(feature = "interp-bilinear"))]
    fn find_index(&self, bins: &[u16; 16], value: u16) -> usize {
        if value < bins[0] {
            return 0;
        }
        for i in 0..15 {
            if value < bins[i + 1] {
                return i;
            }
        }
        15
    }
}

impl Default for IgnitionTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Ignition corrections (in degrees)
///
/// All corrections are additive/subtractive to base timing.
/// Positive = more advance, negative = more retard.
#[derive(Debug, Clone, Copy)]
pub struct IgnitionCorrections {
    /// Coolant temperature correction (degrees)
    /// Cold engine typically needs less advance
    pub clt_correction: i16,

    /// Intake air temperature correction (degrees)
    /// Hot air needs slightly less advance
    pub iat_correction: i16,

    /// Knock retard (degrees)
    /// Reduces timing when knock is detected
    pub knock_retard: i16,
}

impl IgnitionCorrections {
    /// Default corrections (no adjustment)
    pub const DEFAULT: Self = Self {
        clt_correction: 0,
        iat_correction: 0,
        knock_retard: 0,
    };

    /// Create new corrections
    pub const fn new() -> Self {
        Self::DEFAULT
    }
}

impl Default for IgnitionCorrections {
    fn default() -> Self {
        Self::new()
    }
}

/// Calculate final ignition timing with corrections
///
/// # Arguments
/// * `base_timing` - Base timing from table (degrees BTDC)
/// * `corrections` - Temperature and knock corrections
///
/// # Returns
/// Final timing in degrees BTDC, clamped to safe limits
pub fn calculate_timing(base_timing: i16, corrections: &IgnitionCorrections) -> i16 {
    let mut timing = base_timing;

    // Apply corrections
    timing += corrections.clt_correction;
    timing += corrections.iat_correction;
    timing -= corrections.knock_retard; // Subtract because retard is positive

    // Clamp to safe limits
    timing = timing.clamp(MIN_TIMING_BTDC, MAX_TIMING_BTDC);

    timing
}

/// Calculate coil dwell time based on battery voltage
///
/// Lower voltage requires longer dwell to build same magnetic field energy.
/// Higher voltage charges coil faster, so less dwell needed.
///
/// # Arguments
/// * `battery_voltage_mv` - Battery voltage in millivolts (e.g., 12500 = 12.5V)
///
/// # Returns
/// Dwell time in microseconds
pub fn calculate_dwell(battery_voltage_mv: u16) -> u32 {
    // Base dwell at 13.5V (typical running voltage)
    const BASE_VOLTAGE_MV: u32 = 13500;
    const BASE_DWELL_US: u32 = 3000; // 3ms at 13.5V

    // Guard against zero/very low voltage (sensor failure)
    if battery_voltage_mv < 5000 {
        // < 5V is unrealistic, likely sensor failure
        return MAX_DWELL_US; // Safe default
    }

    // Calculate voltage factor (fixed-point math, scaled by 1000)
    // dwell = base_dwell * (base_voltage / actual_voltage)
    let voltage_factor = (BASE_VOLTAGE_MV * 1000) / battery_voltage_mv as u32;
    let dwell_us = (BASE_DWELL_US * voltage_factor) / 1000;

    // Clamp to safe limits
    dwell_us.clamp(MIN_DWELL_US, MAX_DWELL_US)
}

/// Initialize ignition table with safe conservative values
///
/// Creates a basic ignition map suitable for initial engine testing.
/// Conservative timing (less advance) is safer for unknown engines.
///
/// Typical strategy:
/// - Low RPM: Moderate advance (10-15°)
/// - Mid RPM: More advance (20-30°)
/// - High RPM: Less advance (15-25°)
/// - Low load: More advance
/// - High load: Less advance (prevent knock)
pub fn init_conservative_table(table: &mut IgnitionTable) {
    for load_idx in 0..16 {
        for rpm_idx in 0..16 {
            let load = table.load_bins[load_idx];
            let rpm = table.rpm_bins[rpm_idx];

            // Base timing calculation (conservative)
            let base = if rpm < 1500 {
                10 // Low RPM: 10° BTDC
            } else if rpm < 3000 {
                15 // Mid-low RPM: 15° BTDC
            } else if rpm < 5000 {
                20 // Mid RPM: 20° BTDC
            } else {
                18 // High RPM: 18° BTDC
            };

            // Reduce timing at high load (prevent knock)
            let load_correction = if load > 100 {
                -5 // High load: reduce 5°
            } else if load > 80 {
                -2 // Medium-high load: reduce 2°
            } else {
                2 // Low load: add 2°
            };

            table.values[load_idx][rpm_idx] = base + load_correction;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ignition_table_creation() {
        let table = IgnitionTable::new();
        assert_eq!(table.rpm_bins.len(), 16);
        assert_eq!(table.load_bins.len(), 16);
        assert_eq!(table.values.len(), 16);
    }

    #[test]
    fn test_ignition_table_lookup() {
        let mut table = IgnitionTable::new();
        // Default everywhere
        for rpm in [500, 2000, 4000, 8000] {
            for load in [20, 60, 100, 170] {
                let timing = table.lookup(rpm, load);
                assert_eq!(timing, DEFAULT_TIMING_BTDC);
            }
        }
        #[cfg(feature = "interp-bilinear")]
        {
            // Distinct corners in first 2x2 to test bilinear at midpoint
            table.values[0][0] = 10;
            table.values[0][1] = 20;
            table.values[1][0] = 30;
            table.values[1][1] = 40;
            let t = table.lookup(750, 25);
            assert!((24..=26).contains(&t), "t={t}");
        }
    }

    #[test]
    fn test_timing_calculation_with_corrections() {
        let base = 20; // 20° BTDC
        let corrections = IgnitionCorrections {
            clt_correction: -5, // Cold engine, reduce advance
            iat_correction: -2, // Hot air, reduce advance
            knock_retard: 3,    // Knock detected, retard 3°
        };

        let final_timing = calculate_timing(base, &corrections);
        // 20 - 5 - 2 - 3 = 10°
        assert_eq!(final_timing, 10);
    }

    #[test]
    fn test_timing_min_clamp() {
        let base = 0;
        let corrections = IgnitionCorrections {
            clt_correction: -20,
            iat_correction: -10,
            knock_retard: 10,
        };

        let final_timing = calculate_timing(base, &corrections);
        // Should clamp to MIN_TIMING_BTDC
        assert_eq!(final_timing, MIN_TIMING_BTDC);
    }

    #[test]
    fn test_timing_max_clamp() {
        let base = 40;
        let corrections = IgnitionCorrections {
            clt_correction: 10,
            iat_correction: 10,
            knock_retard: 0,
        };

        let final_timing = calculate_timing(base, &corrections);
        // Should clamp to MAX_TIMING_BTDC
        assert_eq!(final_timing, MAX_TIMING_BTDC);
    }

    #[test]
    fn test_dwell_calculation() {
        // Nominal voltage (13.5V)
        let dwell_nominal = calculate_dwell(13500);
        assert_eq!(dwell_nominal, 3000); // 3ms

        // Low voltage (11V) - should increase dwell
        let dwell_low = calculate_dwell(11000);
        assert!(dwell_low > 3000, "Low voltage should increase dwell");

        // High voltage (14.5V) - should decrease dwell
        let dwell_high = calculate_dwell(14500);
        assert!(dwell_high < 3000, "High voltage should decrease dwell");
    }

    #[test]
    fn test_dwell_clamping() {
        // Very low voltage - should clamp to max
        let dwell_very_low = calculate_dwell(6000); // 6V
        assert_eq!(dwell_very_low, MAX_DWELL_US);

        // Very high voltage - should be at or near min
        let dwell_high = calculate_dwell(20000); // 20V
        assert!(
            dwell_high <= 2100,
            "Dwell at 20V should be <=2100us, got {dwell_high}"
        );

        // Extremely high voltage - definitely hits min
        let dwell_extreme = calculate_dwell(30000); // 30V (unrealistic but tests clamping)
        assert_eq!(dwell_extreme, MIN_DWELL_US);

        // Verify clamping at extremes
        assert_eq!(calculate_dwell(5000), MAX_DWELL_US); // 5V hits max
        assert_eq!(calculate_dwell(50000), MIN_DWELL_US); // 50V hits min
    }

    #[test]
    fn test_conservative_table_initialization() {
        let mut table = IgnitionTable::new();
        init_conservative_table(&mut table);

        // Verify some known points
        // Low RPM, low load should have moderate advance
        let timing_low = table.lookup(1000, 40);
        assert!((8..=15).contains(&timing_low));

        // High RPM, high load should have less advance
        let timing_high = table.lookup(6000, 150);
        assert!((10..=20).contains(&timing_high));

        // All values should be within safe range
        for row in 0..16 {
            for col in 0..16 {
                let timing = table.values[row][col];
                assert!(
                    (MIN_TIMING_BTDC..=MAX_TIMING_BTDC).contains(&timing),
                    "Timing out of range at [{row},{col}]: {timing}"
                );
            }
        }
    }

    #[test]
    fn test_knock_retard_reduces_timing() {
        let base = 25;
        let no_knock = IgnitionCorrections {
            knock_retard: 0,
            ..IgnitionCorrections::DEFAULT
        };
        let with_knock = IgnitionCorrections {
            knock_retard: 5,
            ..IgnitionCorrections::DEFAULT
        };

        let timing_no_knock = calculate_timing(base, &no_knock);
        let timing_with_knock = calculate_timing(base, &with_knock);

        assert_eq!(timing_no_knock, 25);
        assert_eq!(timing_with_knock, 20);
        assert!(timing_with_knock < timing_no_knock);
    }
}
