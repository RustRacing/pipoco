//! Trigger wheel decoder for 60-2 pattern.
//!
//! ISR entry points:
//! - `tooth_edge`
//! - `tooth_edge_with_timestamp`
//!
//! The `rpm`, `synced`, and `angle_x10_base` fields are main-loop-only reads
//! and must be protected with `cortex_m::interrupt::free` or an equivalent
//! critical section when accessed outside the ISR context.
//!
//! The decoder is not re-entrant.
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

use crate::constants::rpm::*;
use crate::constants::trigger::*;
use crate::hal::TimeSource;
use crate::units::{DegX10, Micros, Rpm};
#[cfg(debug_assertions)]
use core::cell::Cell;
use core::marker::PhantomData;

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

pub use ecu_domain::SyncState;

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
        let exact_numerator = ecu_domain::RPM_CALC_NUMERATOR_EXACT;
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
    time_source: T, // Private - use getter methods
    last_tooth_time: u32,
    last_period: u32,
    last_gap_period: u32, // Period of last missing tooth gap
    tooth_count: u8,
    sync: SyncState,
    rpm: u16,                   // Approximate for table lookup
    timing_data: TriggerTiming, // Raw data for management engine
    // Angle/time tracking (Phase 2)
    rev_us_est: u32,           // Estimated revolution period (us)
    angle_x10_base: u16,       // Angle at last update (deg*10, 0..3599)
    last_angle_update_us: u32, // Timestamp of last angle base update
    #[cfg(debug_assertions)]
    reentry_guard: Cell<bool>,
    _not_sync: PhantomData<*const ()>,
}

#[cfg(debug_assertions)]
struct ReentryGuard(*const Cell<bool>);

#[cfg(debug_assertions)]
impl Drop for ReentryGuard {
    fn drop(&mut self) {
        unsafe {
            (*self.0).set(false);
        }
    }
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
            sync: SyncState::Unsynced,
            rpm: 0,
            timing_data: TriggerTiming::new(),
            rev_us_est: 0,
            angle_x10_base: 0,
            last_angle_update_us: 0,
            #[cfg(debug_assertions)]
            reentry_guard: Cell::new(false),
            _not_sync: PhantomData,
        }
    }

    /// Called from ISR on every tooth edge
    ///
    /// This function must be called for every rising edge of the trigger signal.
    /// It detects the missing tooth gap, maintains sync, and calculates RPM.
    pub fn tooth_edge(&mut self) {
        #[cfg(debug_assertions)]
        let _guard = Self::enter_guard(&self.reentry_guard);
        let now = self.time_source.micros();
        self.tooth_edge_with_timestamp_inner(now);
    }

    /// Variant of `tooth_edge` that accepts a captured timestamp.
    /// Use this with hardware input-capture to avoid reading the timer here.
    pub fn tooth_edge_with_timestamp(&mut self, now: u32) {
        #[cfg(debug_assertions)]
        let _guard = Self::enter_guard(&self.reentry_guard);
        self.tooth_edge_with_timestamp_inner(now);
    }

    fn tooth_edge_with_timestamp_inner(&mut self, now: u32) {
        // Use wrapping subtraction to handle timer overflow
        let period = now.wrapping_sub(self.last_tooth_time);

        // Check for loss of sync due to timeout
        if matches!(self.sync, SyncState::Locked { .. } | SyncState::Provisional)
            && period > SYNC_TIMEOUT_US
        {
            self.sync = SyncState::Unsynced;
            self.rpm = 0;
            self.tooth_count = 0;
            self.timing_data.synced = false;
        }

        // Missing tooth detection (60-2 specific)
        // Missing tooth gap is ~2x normal tooth period
        // Require last_period to be valid (non-zero) and current period significantly longer
        // Using integer multiply to avoid float: threshold = last_period * 3 / 2 (i.e., 1.5x)
        let threshold = self
            .last_period
            .saturating_mul(MISSING_TOOTH_THRESHOLD_NUM)
            .wrapping_div(MISSING_TOOTH_THRESHOLD_DEN);

        // Only attempt sync if period is reasonable (not a timeout period)
        // Missing tooth should be 2-3x normal period, not 100x
        let max_valid_gap = self.last_period.saturating_mul(10); // Max 10x normal period

        if self.last_period > MIN_VALID_PERIOD_US && period > threshold && period < max_valid_gap {
            // Found missing tooth gap - sync!
            self.tooth_count = 1;
            self.sync = SyncState::Provisional;
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
            // Initialize angle tracking on sync
            let rev = period.saturating_mul(29);
            self.update_rev_estimate(rev);
            self.angle_x10_base = 0;
            self.last_angle_update_us = now;
        } else if matches!(self.sync, SyncState::Locked { .. } | SyncState::Provisional) {
            // Normal tooth - increment count
            if matches!(self.sync, SyncState::Provisional) {
                self.sync = SyncState::Locked { cam_ref: false };
            }
            self.tooth_count += 1;
            if self.tooth_count > TEETH_PER_REV {
                self.tooth_count = 1;
            }

            // Update tooth position in timing data
            self.timing_data.tooth_position = self.tooth_count;
            self.timing_data.tooth_period_us = period;
            self.timing_data.timestamp_us = now;
            // Advance angle and refine revolution estimate from normal tooth period (~6°)
            self.advance_angle_to(now);
            let rev = period.saturating_mul(60);
            self.update_rev_estimate(rev);
        }

        self.last_tooth_time = now;
        self.last_period = period;
    }

    /// Get current RPM
    pub fn rpm(&self) -> Rpm {
        Rpm::new(self.rpm)
    }

    /// Check if decoder is synced
    ///
    /// Returns `false` if:
    /// - Decoder has never seen a missing tooth gap
    /// - No tooth seen for SYNC_TIMEOUT_US microseconds
    pub fn synced(&self) -> bool {
        matches!(self.sync, SyncState::Locked { .. } | SyncState::Provisional)
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
    /// let approx_rpm = decoder.rpm().raw();  // Fast, good enough for table lookup
    /// ```
    pub fn timing_data(&self) -> TriggerTiming {
        self.timing_data
    }

    /// Estimated revolution period in microseconds (single 360°)
    pub fn period_us(&self) -> Micros {
        Micros::new(self.rev_us_est)
    }

    /// Backward-compatible raw period accessor for internal code during migration.
    pub fn rev_period_us(&self) -> u32 {
        self.period_us().raw()
    }

    fn update_rev_estimate(&mut self, new_rev_us: u32) {
        if new_rev_us == 0 {
            return;
        }
        if self.rev_us_est == 0 {
            self.rev_us_est = new_rev_us;
        } else {
            // EMA with 1/8 weight to new measurement
            let est = (self.rev_us_est as u64 * 7 + new_rev_us as u64) >> 3;
            self.rev_us_est = est as u32;
        }
    }

    fn advance_angle_to(&mut self, now: u32) {
        if self.rev_us_est == 0 {
            self.last_angle_update_us = now;
            return;
        }
        let dt = now.wrapping_sub(self.last_angle_update_us) as u64;
        if dt == 0 {
            return;
        }
        let delta = (dt * 3600u64) / (self.rev_us_est as u64);
        if delta == 0 {
            // Not enough time to advance by 0.1°
            return;
        }
        let curr = (self.angle_x10_base as u64 + (delta % 3600)) % 3600;
        self.angle_x10_base = curr as u16;
        self.last_angle_update_us = now;
    }

    /// Current crank angle (deg*10, 0..3599) based on last sync and time elapsed
    pub fn angle_x10(&self, now: u32) -> DegX10 {
        if self.rev_us_est == 0 {
            return DegX10::new(0);
        }
        let dt = now.wrapping_sub(self.last_angle_update_us) as u64;
        let delta = (dt * 3600u64) / (self.rev_us_est as u64);
        DegX10::new(((self.angle_x10_base as u64 + delta) % 3600) as i16)
    }

    /// Backward-compatible raw angle accessor for internal code during migration.
    pub fn current_angle_x10(&self, now: u32) -> u16 {
        self.angle_x10(now).raw() as u16
    }

    /// Compute an absolute timestamp (microseconds) when the crank reaches `target_angle_x10`.
    /// `modulo_x10` controls wrap domain: 3600 for 360°, 7200 for 720°.
    /// Returns None if revolution estimate is not available yet.
    pub fn time_for_target_angle(
        &self,
        now_us: u32,
        target_angle_x10: u16,
        modulo_x10: u16,
    ) -> Option<u32> {
        let rev = self.rev_us_est;
        if rev == 0 {
            return None;
        }
        let curr = self.current_angle_x10(now_us) as u32;
        let target = target_angle_x10 as u32;
        let modulo = modulo_x10 as u32;
        let delta = if target >= curr {
            (target - curr) % modulo
        } else {
            modulo - ((curr - target) % modulo)
        };
        // time offset = delta/360° * rev_us; angles are in deg*10, so divide by 3600
        let offs = delta.saturating_mul(rev) / 3600;
        Some(now_us.wrapping_add(offs))
    }

    /// Force loss of sync (for testing or error recovery)
    pub fn reset_sync(&mut self) {
        self.sync = SyncState::Unsynced;
        self.rpm = 0;
        self.tooth_count = 0;
        self.timing_data.synced = false;
        self.rev_us_est = 0;
        self.angle_x10_base = 0;
        self.last_angle_update_us = 0;
    }

    #[cfg(debug_assertions)]
    fn enter_guard(flag: &Cell<bool>) -> ReentryGuard {
        let was_set = flag.replace(true);
        debug_assert!(!was_set, "TriggerDecoder re-entrancy detected");
        ReentryGuard(flag as *const Cell<bool>)
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
    use std::cell::Cell; // Available in test mode

    #[test]
    fn test_rpm_calculation_bounds() {
        // Test valid RPM range (approximate calculation)
        assert_eq!(calculate_rpm_from_period(1000), 2000); // 2000 RPM
        assert_eq!(calculate_rpm_from_period(2000), 1000); // 1000 RPM

        // Test boundary conditions
        assert_eq!(calculate_rpm_from_period(0), 0); // Invalid
        assert_eq!(calculate_rpm_from_period(100000), 0); // Too slow
    }

    #[test]
    fn test_exact_rpm_calculation() {
        // Test exact RPM calculation in TriggerTiming
        let timing = TriggerTiming {
            gap_period_us: 2000, // 2ms gap
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
        let error_percent =
            ((exact_rpm as i32 - approx_rpm as i32).abs() as f32 / exact_rpm as f32) * 100.0;
        assert!(
            error_percent < 4.0,
            "Error is {error_percent}%, expected < 4%"
        );
    }

    #[test]
    fn test_acceleration_calculation() {
        let timing1 = TriggerTiming {
            gap_period_us: 2000, // 1034 RPM
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };

        let timing2 = TriggerTiming {
            gap_period_us: 1800, // 1149 RPM
            tooth_period_us: 900,
            tooth_position: 1,
            synced: true,
            timestamp_us: 100_000, // 100ms later
        };

        let accel = timing2.acceleration_rpm_per_sec(&timing1);
        // RPM change: 1149 - 1034 = 115 RPM
        // Time: 100ms = 0.1s
        // Acceleration: 115 / 0.1 = 1150 RPM/s
        assert!(
            (1100..=1200).contains(&accel),
            "Accel was {accel}, expected ~1150"
        );
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
        let timing = decoder.timing_data();
        assert!(timing.synced);
        assert_eq!(timing.gap_period_us, 2000);
        assert_eq!(timing.tooth_period_us, 1000);
        assert_eq!(timing.tooth_position, 1);
        let exact = timing.exact_rpm();
        let approx = decoder.rpm().raw();
        assert!(
            exact > approx,
            "Exact ({exact}) should be > approx ({approx})"
        );
    }

    #[test]
    fn test_angle_tracking_and_target_time() {
        struct MockTime(Cell<u32>);
        impl TimeSource for MockTime {
            fn micros(&self) -> u32 {
                self.0.get()
            }
        }
        let ts = MockTime(Cell::new(0));
        let mut dec = TriggerDecoder::new(ts);

        dec.tooth_edge_with_timestamp(1000);
        dec.tooth_edge_with_timestamp(3000);
        assert!(dec.synced());
        assert_eq!(dec.period_us().raw(), 58_000);
        assert_eq!(dec.angle_x10(3000).raw(), 0);
        assert_eq!(dec.angle_x10(3000 + 29_000).raw(), 1800);
        let t = dec.time_for_target_angle(3000, 900, 3600).unwrap();
        assert_eq!(t, 3000 + 14_500);
        let t2 = dec.time_for_target_angle(3000, 4500, 7200).unwrap();
        assert_eq!(t2, 3000 + 72_500);
    }

    #[test]
    fn test_target_time_wraps_next_cycle() {
        struct MockTime(Cell<u32>);
        impl TimeSource for MockTime {
            fn micros(&self) -> u32 {
                self.0.get()
            }
        }
        let ts = MockTime(Cell::new(0));
        let mut dec = TriggerDecoder::new(ts);
        // Sync with gap: 1000->3000 us, rev = 58_000 us
        dec.tooth_edge_with_timestamp(1000);
        dec.tooth_edge_with_timestamp(3000);
        assert_eq!(dec.period_us().raw(), 58_000);
        // Advance time to near end of 360°: choose now so current angle ≈ 3590 (deg*10)
        // angle delta = dt * 3600 / rev => dt ≈ 3589/3600 * 58_000 ≈ 57_849 us
        let now = 3000 + 57_840;
        let t = dec.time_for_target_angle(now, 50, 3600).unwrap(); // target 5.0°
                                                                   // Expected small forward offset: delta = (3600 - (3590 - 50)) = 60 => 60/3600 of rev = 966 us
        assert!(
            t > now && t <= now + 2_000,
            "wrap to next cycle within ~1ms window"
        );
    }

    // Removed constant assertions that are always true (clippy)
}
