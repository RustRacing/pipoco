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
pub mod ignition;
pub mod rev_limiter;
pub mod safety;
pub mod management;
pub mod ve_engine;

pub use trigger::{TriggerDecoder, TriggerTiming};
pub use tables::IpwTable;
pub use scheduler::{Scheduler, Channel, Event};
pub use transport::{Transport, TransportError, TransportStats, Message};
pub use ignition::{IgnitionTable, IgnitionCorrections, calculate_timing, calculate_dwell};
pub use rev_limiter::{RevLimiterConfig, RevLimiterState, LimiterStrategy, update_limiter, should_inject, apply_limiter_retard};
pub use safety::{FloodClearState, SyncLossTracker, update_flood_clear, should_allow_injection};

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
    pub ignition_table: [[i16; 16]; 16],
    pub corrections: Corrections,
    pub ignition_corrections: ignition::IgnitionCorrections,
    pub battery_voltage_mv: u16,
    pub rev_limiter_config: rev_limiter::RevLimiterConfig,
    pub rev_limiter_state: rev_limiter::RevLimiterState,
    pub tps_percent: u8,  // Throttle position (0-100%)
    pub flood_clear_state: safety::FloodClearState,
    pub sync_loss_tracker: safety::SyncLossTracker,
}

impl EcuState {
    /// Create new ECU state with defaults
    pub const fn new() -> Self {
        Self {
            rpm: 0,
            synced: false,
            tooth_count: 0,
            ipw_table: [[DEFAULT_PULSE_WIDTH_US; 16]; 16],
            ignition_table: [[constants::ignition::DEFAULT_TIMING_BTDC; 16]; 16],
            corrections: Corrections::DEFAULT,
            ignition_corrections: ignition::IgnitionCorrections::DEFAULT,
            battery_voltage_mv: 12500,  // 12.5V nominal
            rev_limiter_config: rev_limiter::RevLimiterConfig::DEFAULT,
            rev_limiter_state: rev_limiter::RevLimiterState::new(),
            tps_percent: 0,  // Throttle closed
            flood_clear_state: safety::FloodClearState::new(),
            sync_loss_tracker: safety::SyncLossTracker::new(),
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

    /// Calculate ignition timing with corrections
    ///
    /// Performs the complete ignition timing calculation:
    /// 1. Table lookup for base timing
    /// 2. Apply temperature and knock corrections
    /// 3. Clamp to safe limits
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC (positive = advance, negative = retard)
    pub fn calculate_ignition_timing(&self, rpm: u16, load: u16) -> i16 {
        let table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.ignition_table,
        };

        // 1. Base lookup
        let base_timing = table.lookup(rpm, load);

        // 2. Apply corrections and clamp
        ignition::calculate_timing(base_timing, &self.ignition_corrections)
    }

    /// Calculate coil dwell time based on battery voltage
    ///
    /// # Returns
    /// Dwell time in microseconds
    pub fn calculate_dwell(&self) -> u32 {
        ignition::calculate_dwell(self.battery_voltage_mv)
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

    /// Initialize ignition table with conservative values
    ///
    /// Creates a conservative ignition map safe for initial testing.
    /// Should be replaced with properly tuned values for production.
    pub fn init_ignition_table(&mut self) {
        let mut table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.ignition_table,
        };

        ignition::init_conservative_table(&mut table);
        self.ignition_table = table.values;
    }

    /// Update rev limiter state based on current RPM
    ///
    /// Should be called every engine cycle or in main loop.
    /// Updates internal limiter state which affects fuel and ignition.
    pub fn update_rev_limiter(&mut self) {
        rev_limiter::update_limiter(self.rpm, &self.rev_limiter_config, &mut self.rev_limiter_state);
    }

    /// Check if fuel injection should proceed (considers rev limiter)
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should occur, `false` if limiter is cutting fuel
    pub fn should_inject_fuel(&self, cylinder: u8) -> bool {
        rev_limiter::should_inject(&self.rev_limiter_state, cylinder)
    }

    /// Calculate ignition timing with all corrections (including rev limiter)
    ///
    /// This is the main method to use - applies ignition corrections AND rev limiter retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter(&self, rpm: u16, load: u16) -> i16 {
        // Get base timing with normal corrections
        let base_timing = self.calculate_ignition_timing(rpm, load);

        // Apply rev limiter retard
        rev_limiter::apply_limiter_retard(base_timing, &self.rev_limiter_state)
    }

    /// Update flood clear state based on current conditions
    ///
    /// Should be called every engine cycle or main loop iteration.
    ///
    /// # Returns
    /// `true` if flood clear is active (fuel should be cut)
    pub fn update_flood_clear(&mut self) -> bool {
        safety::update_flood_clear(self.rpm, self.tps_percent, &mut self.flood_clear_state)
    }

    /// Record a sync loss event
    ///
    /// Call this when trigger sync is lost. The tracker will determine
    /// if this is an ESD glitch (recoverable) or real failure (shutdown).
    ///
    /// # Arguments
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if engine should shut down, `false` if should attempt recovery
    pub fn record_sync_loss(&mut self, current_time_us: u32) -> bool {
        self.synced = false;
        self.sync_loss_tracker.record_sync_loss(current_time_us)
    }

    /// Record successful sync recovery
    ///
    /// Call this when sync is successfully re-established after a loss.
    pub fn record_sync_recovery(&mut self) {
        self.synced = true;
        self.sync_loss_tracker.record_recovery();
    }

    /// Reset sync loss window after sustained good operation
    ///
    /// Call this periodically (e.g., every 10 seconds) when sync is stable.
    /// This allows the system to recover from old ESD events.
    pub fn reset_sync_loss_window(&mut self) {
        self.sync_loss_tracker.reset_window();
    }

    /// Check if fuel injection should proceed considering ALL safety features
    ///
    /// This is the master safety check. Returns `true` only if:
    /// - Not in flood clear mode
    /// - Not shut down due to sync loss
    /// - Rev limiter allows injection
    /// - Engine is synced
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should proceed, `false` otherwise
    pub fn should_inject_with_all_safety(&self, cylinder: u8) -> bool {
        // Must be synced
        if !self.synced {
            return false;
        }

        // Check flood clear and shutdown
        if !safety::should_allow_injection(
            self.flood_clear_state.active,
            self.sync_loss_tracker.is_shutdown(),
        ) {
            return false;
        }

        // Check rev limiter
        if !self.should_inject_fuel(cylinder) {
            return false;
        }

        true
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
