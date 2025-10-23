//! Trigger wheel decoder for 60-2 pattern
//!
//! This module decodes signals from a 60-2 trigger wheel (60 teeth with 2 missing).
//! The missing tooth gap is used for synchronization and position reference.
//!
//! # 60-2 Wheel Pattern
//!
//! ```text
//! Tooth:  1  2  3 ... 56 57 58  [GAP]  1  2  3 ...
//!         |  |  |     |  |  |    |  |   |  |  |
//! Period: 1ms 1ms 1ms ... 1ms 2ms (missing tooth gap)
//! ```
//!
//! The decoder:
//! - Detects the missing tooth gap (period > 1.5x normal)
//! - Synchronizes tooth count to position 1 after the gap
//! - Calculates RPM from the gap period
//! - Detects loss of sync if no teeth seen for 200ms

use crate::hal::TimeSource;
use crate::constants::trigger::*;
use crate::constants::rpm::*;

/// Raw trigger timing data for management engine
///
/// This structure contains precise timing measurements that the management
/// engine can use for accurate RPM calculation and analysis.
///
/// The injection module uses approximate RPM for table lookup, but exports
/// raw timing data so the management engine can calculate exact RPM:
/// ```ignore
/// exact_rpm = 60_000_000 / (gap_period_us * 29)
/// ```
#[derive(Copy, Clone, Debug)]
pub struct TriggerTiming {
    /// Period of the missing tooth gap in microseconds
    ///
    /// For a 60-2 wheel, this represents the time for 2 teeth worth of rotation.
    /// Management engine can calculate exact RPM from this.
    pub gap_period_us: u32,

    /// Period of the last normal tooth in microseconds
    ///
    /// Useful for detecting acceleration/deceleration and validating sync.
    pub tooth_period_us: u32,

    /// Current tooth position (1-58)
    pub tooth_position: u8,

    /// Trigger sync status
    pub synced: bool,

    /// Timestamp when this data was captured (microseconds)
    pub timestamp_us: u32,
}

impl TriggerTiming {
    const fn new() -> Self {
        Self {
            gap_period_us: 0,
            tooth_period_us: 0,
            tooth_position: 0,
            synced: false,
            timestamp_us: 0,
        }
    }

    /// Calculate exact RPM from gap period
    ///
    /// This is what the management engine should use for accurate RPM.
    /// Returns RPM with proper integer division (no 3% error).
    ///
    /// Formula: For 60-2 wheel, gap represents 2 teeth, so:
    /// - Full revolution = gap_period * 29 (58 teeth / 2)
    /// - RPM = 60,000,000 us/min / (gap_period * 29)
    /// - RPM = 2,068,966 / gap_period
    pub fn exact_rpm(&self) -> u16 {
        if self.gap_period_us == 0 || !self.synced {
            return 0;
        }

        // Exact calculation: 60,000,000 / (gap_period * 29)
        // Simplified: 2,068,966 / gap_period
        let exact_numerator = 2_068_966_u32;
        (exact_numerator / self.gap_period_us) as u16
    }

    /// Calculate acceleration in RPM/second
    ///
    /// Compare with previous timing data to determine acceleration.
    /// Positive = accelerating, negative = decelerating
    pub fn acceleration_rpm_per_sec(&self, previous: &TriggerTiming) -> i32 {
        if !self.synced || !previous.synced {
            return 0;
        }

        let current_rpm = self.exact_rpm() as i32;
        let previous_rpm = previous.exact_rpm() as i32;
        let time_delta_us = self.timestamp_us.wrapping_sub(previous.timestamp_us);

        if time_delta_us == 0 {
            return 0;
        }

        // RPM change per second = (delta_rpm * 1_000_000) / time_delta_us
        let rpm_change = current_rpm - previous_rpm;
        (rpm_change * 1_000_000) / time_delta_us as i32
    }
}

/// Trigger decoder for 60-2 pattern only (MVP)
///
/// Uses integer arithmetic only. Handles timer overflow with wrapping subtraction.
pub struct TriggerDecoder<T: TimeSource> {
    time_source: T,  // Private - use getter methods
    last_tooth_time: u32,
    last_period: u32,
    last_gap_period: u32,  // Period of last missing tooth gap
    tooth_count: u8,
    synced: bool,
    rpm: u16,  // Approximate for table lookup
    timing_data: TriggerTiming,  // Raw data for management engine
}

impl<T: TimeSource> TriggerDecoder<T> {
    /// Create new trigger decoder
    pub fn new(time_source: T) -> Self {
        Self {
            time_source,
            last_tooth_time: 0,
            last_period: 0,
            last_gap_period: 0,
            tooth_count: 0,
            synced: false,
            rpm: 0,
            timing_data: TriggerTiming::new(),
        }
    }

    /// Called from ISR on every tooth edge
    ///
    /// This function must be called for every rising edge of the trigger signal.
    /// It detects the missing tooth gap, maintains sync, and calculates RPM.
    pub fn tooth_edge(&mut self) {
        let now = self.time_source.micros();

        // Use wrapping subtraction to handle timer overflow
        let period = now.wrapping_sub(self.last_tooth_time);

        // Check for loss of sync due to timeout
        if self.synced && period > SYNC_TIMEOUT_US {
            self.synced = false;
            self.rpm = 0;
            self.tooth_count = 0;
            self.timing_data.synced = false;
        }

        // Missing tooth detection (60-2 specific)
        // Missing tooth gap is ~2x normal tooth period
        // Require last_period to be valid (non-zero) and current period significantly longer
        // Using integer multiply to avoid float: threshold = last_period * 3 / 2 (i.e., 1.5x)
        let threshold = self.last_period
            .saturating_mul(MISSING_TOOTH_THRESHOLD_NUM)
            .wrapping_div(MISSING_TOOTH_THRESHOLD_DEN);

        // Only attempt sync if period is reasonable (not a timeout period)
        // Missing tooth should be 2-3x normal period, not 100x
        let max_valid_gap = self.last_period.saturating_mul(10);  // Max 10x normal period

        if self.last_period > MIN_VALID_PERIOD_US
            && period > threshold
            && period < max_valid_gap {
            // Found missing tooth gap - sync!
            self.tooth_count = 1;
            self.synced = true;
            self.last_gap_period = period;

            // Calculate approximate RPM for table lookup (fast)
            self.rpm = calculate_rpm_from_period(period);

            // Update timing data for management engine (accurate)
            self.timing_data = TriggerTiming {
                gap_period_us: period,
                tooth_period_us: self.last_period,
                tooth_position: 1,
                synced: true,
                timestamp_us: now,
            };
        } else if self.synced {
            // Normal tooth - increment count
            self.tooth_count += 1;
            if self.tooth_count > TEETH_PER_REV {
                self.tooth_count = 1;
            }

            // Update tooth position in timing data
            self.timing_data.tooth_position = self.tooth_count;
            self.timing_data.tooth_period_us = period;
            self.timing_data.timestamp_us = now;
        }

        self.last_tooth_time = now;
        self.last_period = period;
    }

    /// Get current RPM
    pub fn rpm(&self) -> u16 {
        self.rpm
    }

    /// Check if decoder is synced
    ///
    /// Returns `false` if:
    /// - Decoder has never seen a missing tooth gap
    /// - No tooth seen for SYNC_TIMEOUT_US microseconds
    pub fn synced(&self) -> bool {
        self.synced
    }

    /// Get current tooth number (1-58)
    ///
    /// Only valid when `synced()` returns true.
    pub fn tooth(&self) -> u8 {
        self.tooth_count
    }

    /// Get reference to time source
    ///
    /// Needed for STM32 implementation to access timer from ISR.
    pub fn time_source(&self) -> &T {
        &self.time_source
    }

    /// Get raw timing data for management engine
    ///
    /// Returns precise timing measurements that the management engine can use
    /// for accurate RPM calculation, acceleration detection, and analysis.
    ///
    /// The management engine should use `timing_data.exact_rpm()` for display
    /// and calculations, while the injection module uses the approximate `rpm()`
    /// for fast table lookup.
    ///
    /// # Example
    /// ```ignore
    /// // In management engine (via CAN):
    /// let timing = decoder.timing_data();
    /// let exact_rpm = timing.exact_rpm();  // Accurate, no 3% error
    /// let accel = timing.acceleration_rpm_per_sec(&previous_timing);
    ///
    /// // In injection module (local):
    /// let approx_rpm = decoder.rpm();  // Fast, good enough for table lookup
    /// ```
    pub fn timing_data(&self) -> TriggerTiming {
        self.timing_data
    }

    /// Force loss of sync (for testing or error recovery)
    pub fn reset_sync(&mut self) {
        self.synced = false;
        self.rpm = 0;
        self.tooth_count = 0;
        self.timing_data.synced = false;
    }
}

/// Calculate approximate RPM for fast table lookup (injection module use)
///
/// This is a FAST approximation used only by the injection module for table lookup.
/// Management engine should use `TriggerTiming::exact_rpm()` for accurate RPM.
///
/// Uses integer-only arithmetic to avoid floating point operations.
///
/// # Derivation
///
/// For a 60-2 trigger wheel:
/// - Total teeth: 60
/// - Missing teeth: 2
/// - Actual teeth: 58
/// - Missing tooth gap represents 2 teeth worth of angular distance
///
/// If the missing tooth gap takes `period` microseconds:
/// - 2 teeth = `period` microseconds
/// - 58 teeth = `period * 29` microseconds (one full revolution)
/// - Time per revolution = `period * 29` microseconds
/// - RPM = 60,000,000 µs/min ÷ (period × 29)
/// - RPM = 2,068,966 ÷ period (EXACT)
///
/// For faster computation, we use 2,000,000 ÷ period (introduces ~3.3% error).
/// This is acceptable for table lookup since bins are 500 RPM apart.
///
/// # Arguments
/// * `period` - Missing tooth gap duration in microseconds
///
/// # Returns
/// Approximate RPM as u16, or 0 if period is invalid
fn calculate_rpm_from_period(period: u32) -> u16 {
    // Guard against division by zero
    if period == 0 {
        return 0;
    }

    // Periods longer than MAX_PERIOD_FOR_CALC_US indicate < 1000 RPM
    // Treat as 0 to avoid numerical issues
    if period > MAX_PERIOD_FOR_CALC_US {
        return 0;
    }

    // Integer division: RPM ≈ 2,000,000 / period
    // This is safe because we've checked period != 0
    (RPM_CALC_NUMERATOR / period) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;  // Available in test mode

    #[test]
    fn test_rpm_calculation_bounds() {
        // Test valid RPM range (approximate calculation)
        assert_eq!(calculate_rpm_from_period(1000), 2000);  // 2000 RPM
        assert_eq!(calculate_rpm_from_period(2000), 1000);  // 1000 RPM

        // Test boundary conditions
        assert_eq!(calculate_rpm_from_period(0), 0);      // Invalid
        assert_eq!(calculate_rpm_from_period(100000), 0); // Too slow
    }

    #[test]
    fn test_exact_rpm_calculation() {
        // Test exact RPM calculation in TriggerTiming
        let timing = TriggerTiming {
            gap_period_us: 2000,  // 2ms gap
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };

        let exact_rpm = timing.exact_rpm();
        // 2,068,966 / 2000 = 1034 RPM (exact)
        assert_eq!(exact_rpm, 1034);

        // Compare with approximate
        let approx_rpm = calculate_rpm_from_period(2000);
        // 2,000,000 / 2000 = 1000 RPM (3.3% error)
        assert_eq!(approx_rpm, 1000);

        // Verify the difference is indeed ~3%
        let error_percent = ((exact_rpm as i32 - approx_rpm as i32).abs() as f32 / exact_rpm as f32) * 100.0;
        assert!(error_percent < 4.0, "Error is {}%, expected < 4%", error_percent);
    }

    #[test]
    fn test_acceleration_calculation() {
        let timing1 = TriggerTiming {
            gap_period_us: 2000,  // 1034 RPM
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };

        let timing2 = TriggerTiming {
            gap_period_us: 1800,  // 1149 RPM
            tooth_period_us: 900,
            tooth_position: 1,
            synced: true,
            timestamp_us: 100_000,  // 100ms later
        };

        let accel = timing2.acceleration_rpm_per_sec(&timing1);
        // RPM change: 1149 - 1034 = 115 RPM
        // Time: 100ms = 0.1s
        // Acceleration: 115 / 0.1 = 1150 RPM/s
        assert!(accel >= 1100 && accel <= 1200, "Accel was {}, expected ~1150", accel);
    }

    #[test]
    fn test_timing_data_updated() {
        struct MockTime {
            time: Cell<u32>,
        }

        impl MockTime {
            fn new(t: u32) -> Self {
                Self { time: Cell::new(t) }
            }
            fn set(&self, t: u32) {
                self.time.set(t);
            }
        }

        impl TimeSource for MockTime {
            fn micros(&self) -> u32 {
                self.time.get()
            }
        }

        let time = MockTime::new(0);
        let mut decoder = TriggerDecoder::new(time);

        // Simulate sync
        for i in 0..57 {
            decoder.time_source().set(i * 1000);
            decoder.tooth_edge();
        }

        decoder.time_source().set(57 * 1000);
        decoder.tooth_edge();
        decoder.time_source().set(59 * 1000);
        decoder.tooth_edge();

        assert!(decoder.synced());

        // Check timing data was populated
        let timing = decoder.timing_data();
        assert!(timing.synced);
        assert_eq!(timing.gap_period_us, 2000);  // 2ms gap
        assert_eq!(timing.tooth_period_us, 1000);  // 1ms normal tooth
        assert_eq!(timing.tooth_position, 1);

        // Check exact RPM is more accurate than approximate
        let exact = timing.exact_rpm();
        let approx = decoder.rpm();
        assert!(exact > approx, "Exact ({}) should be > approx ({})", exact, approx);
    }

    #[test]
    fn test_constants_consistency() {
        // Verify our constants make sense
        assert!(TEETH_PER_REV > 0);
        assert!(MISSING_TEETH > 0);
        assert!(MIN_VALID_PERIOD_US > 0);
        assert!(SYNC_TIMEOUT_US > MIN_VALID_PERIOD_US);
    }
}
