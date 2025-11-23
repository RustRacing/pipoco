//! Hardware Abstraction Layer (HAL) traits
//!
//! Defines minimal trait interfaces for hardware peripherals needed by the ECU core.
//! These traits allow the core logic to be platform-independent and testable.
//!
//! # Design Philosophy
//!
//! - **Minimal**: Only essential operations, no unnecessary complexity
//! - **no_std compatible**: Works in bare-metal embedded environments
//! - **Zero-cost**: Traits compile to direct function calls with optimization
//!
//! # Timer Overflow
//!
//! The `TimeSource::micros()` method returns a u32, which wraps around after
//! approximately 71 minutes (2^32 microseconds). Code using this trait must
//! handle wraparound correctly using wrapping arithmetic.

/// Time source trait for microsecond timing
///
/// Provides monotonic time in microseconds since system boot.
/// Time wraps around after ~71 minutes (u32::MAX microseconds).
pub trait TimeSource {
    /// Get current time in microseconds since boot
    ///
    /// # Wraparound Behavior
    ///
    /// This returns a u32, which wraps from 0xFFFFFFFF to 0x00000000
    /// after approximately 71.5 minutes of continuous operation.
    ///
    /// Code must use wrapping arithmetic for time comparisons:
    /// ```ignore
    /// let elapsed = now.wrapping_sub(start_time);
    /// if elapsed < timeout {
    ///     // Event is within timeout
    /// }
    /// ```
    fn micros(&self) -> u32;

    /// Native tick counter (default: microseconds)
    ///
    /// Implementors may override to return a high-resolution hardware timer.
    /// Defaults to `micros()` to avoid breaking existing targets.
    fn ticks(&self) -> u32 {
        self.micros()
    }

    /// Native tick frequency in Hz (default: 1_000_000 for micros)
    fn freq_hz(&self) -> u32 {
        1_000_000
    }
}

/// Digital output pin trait
///
/// Minimal interface for controlling a digital output pin.
/// No error handling for MVP simplicity - assumes operations always succeed.
pub trait OutputPin {
    /// Set pin to logic high level
    ///
    /// The actual voltage level depends on the hardware (typically 3.3V or 5V).
    fn set_high(&mut self);

    /// Set pin to logic low level
    ///
    /// The actual voltage level is typically 0V (ground).
    fn set_low(&mut self);
}

/// Optional timestamp source for captured input edges
///
/// Implemented by targets that use hardware input-capture (with or without DMA)
/// to enqueue trigger edge timestamps for low-jitter processing.
pub trait EdgeTimestampSource {
    /// Pop the next captured timestamp in microseconds, if available
    fn try_pop(&mut self) -> Option<u32>;
}

/// Independent watchdog control
pub trait Watchdog {
    /// Start/enable the watchdog with a timeout in milliseconds
    fn start(&mut self, timeout_ms: u32);

    /// Pet/kick the watchdog to prevent a reset
    fn pet(&mut self);
}

/// System reset reason and control
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ResetReason {
    PowerOn,
    PinReset,
    Software,
    IndependentWatchdog,
    WindowWatchdog,
    BrownOut,
    LowPower,
    Unknown,
}

pub trait ResetController {
    /// Read the last reset reason from hardware flags
    fn reason(&self) -> ResetReason;
    /// Clear latched reset flags
    fn clear(&mut self);
}
