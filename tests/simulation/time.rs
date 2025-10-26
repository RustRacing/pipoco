//! Simulated time source
//!
//! Provides a controllable time source for deterministic testing.

use ecu_core::hal::TimeSource;
use std::cell::Cell;

/// Simulated time source with microsecond precision
///
/// Unlike real hardware timers, this can be advanced arbitrarily for testing.
/// Supports wrapping at u32::MAX like real hardware timers.
pub struct SimulatedTime {
    micros: Cell<u32>,
}

impl SimulatedTime {
    /// Create new simulated time starting at 0
    pub fn new() -> Self {
        Self::with_start(0)
    }

    /// Create new simulated time starting at specified value
    pub fn with_start(start_micros: u32) -> Self {
        Self {
            micros: Cell::new(start_micros),
        }
    }

    /// Advance time by specified microseconds
    ///
    /// Uses wrapping addition to simulate hardware timer overflow.
    pub fn advance(&self, delta_us: u32) {
        let current = self.micros.get();
        self.micros.set(current.wrapping_add(delta_us));
    }

    /// Set absolute time
    pub fn set_micros(&self, micros: u32) {
        self.micros.set(micros);
    }

    /// Get elapsed time since last reset
    pub fn elapsed_since(&self, start: u32) -> u32 {
        self.micros.get().wrapping_sub(start)
    }
}

impl TimeSource for SimulatedTime {
    fn micros(&self) -> u32 {
        self.micros.get()
    }
}

// Also implement for references (needed for TriggerDecoder)
impl TimeSource for &SimulatedTime {
    fn micros(&self) -> u32 {
        self.micros.get()
    }
}

impl Default for SimulatedTime {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_starts_at_zero() {
        let time = SimulatedTime::new();
        assert_eq!(time.micros(), 0);
    }

    #[test]
    fn test_time_advance() {
        let time = SimulatedTime::new();
        time.advance(1000);
        assert_eq!(time.micros(), 1000);
        time.advance(500);
        assert_eq!(time.micros(), 1500);
    }

    #[test]
    fn test_time_wrapping() {
        let time = SimulatedTime::with_start(u32::MAX - 100);
        time.advance(200);
        assert_eq!(time.micros(), 99);  // Wrapped around
    }

    #[test]
    fn test_elapsed_since() {
        let time = SimulatedTime::new();
        let start = time.micros();
        time.advance(1000);
        assert_eq!(time.elapsed_since(start), 1000);
    }

    #[test]
    fn test_elapsed_with_wrapping() {
        let time = SimulatedTime::with_start(u32::MAX - 100);
        let start = time.micros();
        time.advance(200);
        assert_eq!(time.elapsed_since(start), 200);
    }
}
