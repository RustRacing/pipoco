//! Event scheduler for timed output control
//!
//! Manages a fixed-size queue of timed events for controlling injector and ignition outputs.
//! Uses integer arithmetic and wrapping time comparisons to handle timer overflow.

use crate::constants::scheduler::*;
use crate::hal::OutputPin;

/// Type-safe channel identifier for outputs
///
/// Prevents accidentally passing invalid channel numbers to the scheduler.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Channel(u8);

impl Channel {
    pub const INJ1: Self = Self(CHANNEL_INJ1);
    pub const INJ2: Self = Self(CHANNEL_INJ2);
    pub const IGN1: Self = Self(CHANNEL_IGN1);
    pub const IGN2: Self = Self(CHANNEL_IGN2);

    /// Create a new channel (private, use constants)
    const fn new(value: u8) -> Self {
        Self(value)
    }

    /// Create a channel from a raw index (0..MAX_CHANNELS)
    /// Caller must ensure `index < MAX_CHANNELS`.
    pub const fn from_index(index: u8) -> Self {
        Self(index)
    }

    /// Get the raw channel number
    pub const fn as_u8(&self) -> u8 {
        self.0
    }

    /// Check if channel is valid
    pub const fn is_valid(&self) -> bool {
        self.0 < MAX_CHANNELS
    }
}

/// Scheduled event with private fields for encapsulation
#[derive(Copy, Clone, Debug)]
pub struct Event {
    time: u32,
    channel: Channel,
    state: bool, // true = high, false = low
    active: bool,
}

impl Event {
    /// Create a new inactive event (for initialization)
    const fn new_inactive() -> Self {
        Self {
            time: 0,
            channel: Channel::new(0),
            state: false,
            active: false,
        }
    }

    /// Create a new active event
    fn new_active(time: u32, channel: Channel, state: bool) -> Self {
        Self {
            time,
            channel,
            state,
            active: true,
        }
    }

    /// Check if event is active
    pub const fn is_active(&self) -> bool {
        self.active
    }

    /// Get event time
    pub const fn time(&self) -> u32 {
        self.time
    }

    /// Get event channel
    pub const fn channel(&self) -> Channel {
        self.channel
    }

    /// Get event state
    pub const fn state(&self) -> bool {
        self.state
    }

    /// Deactivate this event
    fn deactivate(&mut self) {
        self.active = false;
    }
}

/// Fixed-size event scheduler (no heap allocation)
///
/// Manages up to MAX_EVENTS scheduled events. Uses wrapping arithmetic
/// to correctly handle timer overflow after 71 minutes of runtime.
pub struct Scheduler {
    events: [Event; MAX_EVENTS],
}

impl Scheduler {
    /// Create new scheduler with all events inactive
    pub const fn new() -> Self {
        Self {
            events: [Event::new_inactive(); MAX_EVENTS],
        }
    }

    /// Schedule a new event
    ///
    /// Returns `true` if event was scheduled successfully, `false` if scheduler is full.
    ///
    /// # Arguments
    /// * `time` - Absolute time (in microseconds) when event should fire
    /// * `channel` - Output channel to control
    /// * `state` - Desired state (true = high, false = low)
    ///
    /// # Safety
    /// This function can be called from ISR context. Ensure proper synchronization
    /// if also called from main loop.
    pub fn schedule(&mut self, time: u32, channel: Channel, state: bool) -> bool {
        debug_assert!(channel.is_valid(), "Invalid channel: {}", channel.as_u8());

        // Find free slot
        for event in &mut self.events {
            if !event.active {
                *event = Event::new_active(time, channel, state);
                return true;
            }
        }

        // No free slots - scheduler is full!
        false
    }

    /// Schedule a new event using native tick timebase.
    /// Identical semantics to `schedule`, but the time unit is target-specific ticks.
    pub fn schedule_ticks(&mut self, ticks: u32, channel: Channel, state: bool) -> bool {
        self.schedule(ticks, channel, state)
    }

    /// Check for due events and execute them
    ///
    /// Uses wrapping arithmetic to correctly handle timer overflow.
    /// An event is considered due if: (now - event.time) < u32::MAX/2
    ///
    /// # Arguments
    /// * `now` - Current time in microseconds
    /// * `outputs` - Array of output pins to control
    ///
    /// # Safety
    /// This function assumes `outputs` array has at least MAX_CHANNELS elements.
    pub fn check_and_execute(&mut self, now: u32, outputs: &mut [&mut dyn OutputPin]) {
        for event in &mut self.events {
            if event.active {
                // Use wrapping subtraction to handle timer overflow correctly
                // Event is due if (now - event.time) is a small positive number
                let elapsed = now.wrapping_sub(event.time);

                // If elapsed < half of u32 range, event is due
                // This handles overflow: if now=10 and event.time=0xFFFFFFF0,
                // elapsed = 10 - 0xFFFFFFF0 (wrapping) = 0x20, which is < u32::MAX/2
                if elapsed < (u32::MAX / 2) {
                    let channel_idx = event.channel.as_u8() as usize;

                    if channel_idx < outputs.len() {
                        if event.state {
                            outputs[channel_idx].set_high();
                        } else {
                            outputs[channel_idx].set_low();
                        }
                    }

                    event.deactivate();
                }
            }
        }
    }

    /// Tick-based variant of `check_and_execute` using the native timer domain.
    pub fn check_and_execute_ticks(&mut self, now_ticks: u32, outputs: &mut [&mut dyn OutputPin]) {
        self.check_and_execute(now_ticks, outputs)
    }

    /// Clear all events
    ///
    /// Useful for emergency shutdown or reset.
    pub fn clear(&mut self) {
        for event in &mut self.events {
            event.deactivate();
        }
    }

    /// Get number of active events (for debugging/monitoring)
    pub fn active_count(&self) -> usize {
        self.events.iter().filter(|e| e.is_active()).count()
    }

    /// Check if scheduler is full
    pub fn is_full(&self) -> bool {
        self.events.iter().all(|e| e.is_active())
    }

    /// Get reference to events array (for manual control in ISR)
    ///
    /// # Safety
    /// Direct access to events array. Caller must ensure proper synchronization.
    pub fn events_mut(&mut self) -> &mut [Event; MAX_EVENTS] {
        &mut self.events
    }

    /// Deactivate all events for `channel` scheduled at or after `cutoff` (tick/micro domain consistent with schedule calls).
    pub fn deactivate_channel_after(&mut self, cutoff: u32, channel: Channel) {
        for e in &mut self.events {
            if e.active && e.channel == channel {
                // event is in the future relative to cutoff if (event.time - cutoff) < MAX/2
                let is_future = e.time.wrapping_sub(cutoff) < (u32::MAX / 2);
                if is_future {
                    e.deactivate();
                }
            }
        }
    }

    /// Deactivate all events for the given channel, regardless of time.
    pub fn deactivate_channel_all(&mut self, channel: Channel) {
        for e in &mut self.events {
            if e.active && e.channel == channel {
                e.deactivate();
            }
        }
    }
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}
