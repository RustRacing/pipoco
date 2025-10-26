//! ECU Core Library
//!
//! Minimal viable ECU (Engine Control Unit) implementation in Rust.
//! Designed for no_std embedded environments with zero dependencies.
//!
//! # Architecture
//!
//! This library uses an IPW (Injector Pulse Width) table approach instead of
//! traditional VE (Volumetric Efficiency) calculations. This eliminates complex
//! math from the embedded module - all calculations are pre-computed and stored
//! in lookup tables.
//!
//! ## Modules
//!
//! - `trigger`: 60-2 trigger wheel decoder for position and RPM
//! - `tables`: IPW table lookup (no interpolation)
//! - `scheduler`: Event scheduling for injection and ignition
//! - `hal`: Hardware abstraction traits
//! - `constants`: System-wide configuration constants
//! - `transport`: Transport-agnostic inter-component communication
//!
//! ## Design Principles
//!
//! - **Integer-only arithmetic**: No floating-point operations
//! - **Static memory**: No heap allocation, all state in static variables
//! - **Wrapping arithmetic**: Correctly handles timer overflow
//! - **Minimal dependencies**: Zero external dependencies in core library

#![cfg_attr(not(test), no_std)]

pub mod hal;
pub mod trigger;
pub mod tables;
pub mod scheduler;
pub mod constants;
pub mod transport;

pub use trigger::{TriggerDecoder, TriggerTiming};
pub use tables::IpwTable;
pub use scheduler::{Scheduler, Channel, Event};
pub use transport::{Transport, TransportError, TransportStats, Message};

#[cfg(feature = "transport-bbqueue")]
pub use transport::BbqTransport;

use constants::corrections::*;
use constants::fuel::*;

/// Fixed-point math helper (no floats!)
///
/// Multiplies value by (multiplier / 100) using integer arithmetic only.
/// Uses saturating multiplication to prevent overflow.
///
/// # Arguments
/// * `value` - Base value (e.g., pulse width in microseconds)
/// * `multiplier` - Correction factor scaled by 100 (e.g., 150 = 1.5x, 80 = 0.8x)
///
/// # Returns
/// Corrected value, saturated at u16::MAX if overflow would occur
///
/// # Example
/// ```
/// use ecu_core::scale_u16;
///
/// assert_eq!(scale_u16(1000, 150), 1500);  // 1.5x
/// assert_eq!(scale_u16(1000, 80), 800);    // 0.8x
/// assert_eq!(scale_u16(1000, 100), 1000);  // 1.0x (no change)
/// ```
pub fn scale_u16(value: u16, multiplier: u8) -> u16 {
    // Use saturating multiply to prevent overflow
    let intermediate = (value as u32).saturating_mul(multiplier as u32);
    let result = intermediate / 100;

    // Clamp to u16::MAX
    if result > u16::MAX as u32 {
        u16::MAX
    } else {
        result as u16
    }
}

/// Correction multipliers (100 = 1.0x)
///
/// All corrections are represented as integers scaled by 100 to avoid
/// floating-point operations. A value of 100 means no correction (1.0x).
///
/// # Examples
/// - 150 = 1.5x (add 50% fuel)
/// - 80 = 0.8x (reduce fuel by 20%)
/// - 100 = 1.0x (no change)
#[derive(Debug, Clone, Copy)]
pub struct Corrections {
    /// Coolant temperature correction
    pub clt: u8,
    /// Intake air temperature correction
    pub iat: u8,
    /// Battery voltage correction (compensates for injector opening time)
    pub vbatt: u8,
}

impl Corrections {
    /// Default corrections (1.0x all - no corrections applied)
    pub const DEFAULT: Self = Self {
        clt: UNITY_CORRECTION,
        iat: UNITY_CORRECTION,
        vbatt: UNITY_CORRECTION,
    };

    /// Create new corrections with specified values
    pub const fn new(clt: u8, iat: u8, vbatt: u8) -> Self {
        Self { clt, iat, vbatt }
    }
}

/// Global ECU state
///
/// Contains all state needed for ECU operation. Designed to be stored
/// in a static variable for access from ISR context.
pub struct EcuState {
    pub rpm: u16,
    pub synced: bool,
    pub tooth_count: u8,
    pub ipw_table: [[u16; 16]; 16],
    pub corrections: Corrections,
}

impl EcuState {
    /// Create new ECU state with defaults
    pub const fn new() -> Self {
        Self {
            rpm: 0,
            synced: false,
            tooth_count: 0,
            ipw_table: [[DEFAULT_PULSE_WIDTH_US; 16]; 16],
            corrections: Corrections::DEFAULT,
        }
    }

    /// Calculate fuel pulse width with corrections
    ///
    /// Performs the complete fuel calculation:
    /// 1. Table lookup for base pulse width
    /// 2. Apply temperature and voltage corrections
    /// 3. Clamp to valid range
    ///
    /// Uses integer-only arithmetic with saturating operations to prevent overflow.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa (or TPS %)
    ///
    /// # Returns
    /// Final pulse width in microseconds, clamped to MIN/MAX limits
    pub fn calculate_fuel(&self, rpm: u16, load: u16) -> u16 {
        let table = IpwTable {
            rpm_bins: RPM_BINS,
            load_bins: LOAD_BINS,
            values: self.ipw_table,
        };

        // 1. Base lookup
        let mut pw = table.lookup(rpm, load);

        // 2. Apply corrections sequentially with saturation
        pw = scale_u16(pw, self.corrections.clt);
        pw = scale_u16(pw, self.corrections.iat);
        pw = scale_u16(pw, self.corrections.vbatt);

        // 3. Clamp to reasonable range
        if pw < MIN_PULSE_WIDTH_US {
            pw = MIN_PULSE_WIDTH_US;
        }
        if pw > MAX_PULSE_WIDTH_US {
            pw = MAX_PULSE_WIDTH_US;
        }

        pw
    }

    /// Initialize IPW table with linear test values
    ///
    /// Creates a simple linear fuel map for initial testing.
    /// More fuel at higher load, slightly less at higher RPM.
    ///
    /// This is a helper method for hardware testing. Real tuning data
    /// should be loaded from external storage or CAN.
    pub fn init_linear_table(&mut self) {
        for row in 0..16 {
            for col in 0..16 {
                let base = DEFAULT_PULSE_WIDTH_US;
                let load_factor = (row as u16).saturating_mul(50);  // 0-750us
                let rpm_factor = (col as u16).saturating_mul(10);   // 0-150us

                // More fuel at higher load, slightly less at higher RPM
                self.ipw_table[row][col] = base
                    .saturating_add(load_factor)
                    .saturating_sub(rpm_factor);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_u16_normal() {
        assert_eq!(scale_u16(1000, 150), 1500);  // 1.5x
        assert_eq!(scale_u16(1000, 80), 800);    // 0.8x
        assert_eq!(scale_u16(1000, 100), 1000);  // 1.0x
        assert_eq!(scale_u16(500, 200), 1000);   // 2.0x
    }

    #[test]
    fn test_scale_u16_saturation() {
        // Test overflow protection
        assert_eq!(scale_u16(u16::MAX, 200), u16::MAX);  // Would overflow
        assert_eq!(scale_u16(50000, 200), u16::MAX);     // Would overflow
    }

    #[test]
    fn test_fuel_calculation_clamping() {
        let mut state = EcuState::new();

        // Test minimum clamping (with very low correction)
        state.corrections.clt = 10;  // 0.1x (very low)
        let pw = state.calculate_fuel(3000, 60);
        assert_eq!(pw, MIN_PULSE_WIDTH_US);

        // Test maximum clamping (with very high base value and correction)
        // First set a high base value in the table
        // 3000 RPM maps to RPM bin index 5, 60 kPa maps to load bin index 4
        // Table is [load_idx][rpm_idx]
        state.ipw_table[4][5] = 15000;  // 15ms base
        state.corrections.clt = 255;  // 2.55x (very high)
        state.corrections.iat = 255;
        state.corrections.vbatt = 255;
        // This should result in: 15000 * 2.55 * 2.55 * 2.55 = 249,146 which exceeds MAX
        let pw = state.calculate_fuel(3000, 60);  // Maps to bin [4][5]
        assert_eq!(pw, MAX_PULSE_WIDTH_US);
    }

    #[test]
    fn test_fuel_calculation_normal() {
        let state = EcuState::new();

        // With default corrections (1.0x), should return table value
        let pw = state.calculate_fuel(3000, 60);
        assert_eq!(pw, DEFAULT_PULSE_WIDTH_US);
    }

    #[test]
    fn test_linear_table_initialization() {
        let mut state = EcuState::new();
        state.init_linear_table();

        // Verify table has been populated
        // First cell should be base + 0 - 0
        assert_eq!(state.ipw_table[0][0], DEFAULT_PULSE_WIDTH_US);

        // Last cell should be base + 750 - 150
        let expected = DEFAULT_PULSE_WIDTH_US + 750 - 150;
        assert_eq!(state.ipw_table[15][15], expected);

        // Verify middle cell has reasonable value
        assert!(state.ipw_table[8][8] > DEFAULT_PULSE_WIDTH_US);
    }
}
