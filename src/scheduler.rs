//! Event scheduler for timed output control.
//!
//! This module assumes a single-writer context for all mutable state.
//! Callers must ensure mutual exclusion between ISR and main-loop access
//! with `cortex_m::interrupt::free` or an equivalent critical section.
//!
//! ISR-facing entry points:
//! - `schedule`
//! - `check_and_execute`
//! - `deactivate_all`
//!
//! Uses integer arithmetic and wrapping time comparisons to handle timer overflow.

use crate::constants::scheduler::*;
use crate::hal::OutputPin;
use crate::units::Micros;
#[cfg(debug_assertions)]
use core::cell::Cell;
use core::marker::PhantomData;

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
    state_kind: EventState,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum EventState {
    Idle,
    Armed { channel: Channel, deadline: Micros },
    Fired { at: Micros },
}

impl Event {
    /// Create a new inactive event (for initialization)
    const fn new_inactive() -> Self {
        Self {
            time: 0,
            channel: Channel::new(0),
            state: false,
            state_kind: EventState::Idle,
        }
    }

    /// Create a new active event
    fn new_active(time: u32, channel: Channel, state: bool) -> Self {
        Self {
            time,
            channel,
            state,
            state_kind: EventState::Armed {
                channel,
                deadline: Micros::new(time),
            },
        }
    }

    /// Check if event is active
    pub const fn is_active(&self) -> bool {
        !matches!(self.state_kind, EventState::Idle)
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
        self.state_kind = EventState::Idle;
    }
}

/// Fixed-size event scheduler (no heap allocation)
///
/// Manages up to MAX_EVENTS scheduled events. Uses wrapping arithmetic
/// to correctly handle timer overflow after 71 minutes of runtime.
pub struct Scheduler {
    events: [Event; MAX_EVENTS],
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

impl Scheduler {
    /// Create new scheduler with all events inactive
    pub const fn new() -> Self {
        Self {
            events: [Event::new_inactive(); MAX_EVENTS],
            #[cfg(debug_assertions)]
            reentry_guard: Cell::new(false),
            _not_sync: PhantomData,
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
    pub fn schedule(&mut self, time: Micros, channel: Channel, state: bool) -> bool {
        self.schedule_raw(time.raw(), channel, state)
    }

    fn schedule_raw(&mut self, time: u32, channel: Channel, state: bool) -> bool {
        #[cfg(debug_assertions)]
        let _guard = Self::enter_guard(&self.reentry_guard);
        debug_assert!(channel.is_valid(), "Invalid channel: {}", channel.as_u8());

        // Find free slot
        for event in &mut self.events {
            if !event.is_active() {
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
        self.schedule_raw(ticks, channel, state)
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
    pub fn check_and_execute(&mut self, now: Micros, outputs: &mut [&mut dyn OutputPin]) {
        self.check_and_execute_raw(now.raw(), outputs)
    }

    fn check_and_execute_raw(&mut self, now: u32, outputs: &mut [&mut dyn OutputPin]) {
        #[cfg(debug_assertions)]
        let _guard = Self::enter_guard(&self.reentry_guard);
        for event in &mut self.events {
            if event.is_active() {
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

                    event.state_kind = EventState::Fired {
                        at: Micros::new(now),
                    };
                    event.deactivate();
                }
            }
        }
    }

    /// Tick-based variant of `check_and_execute` using the native timer domain.
    pub fn check_and_execute_ticks(&mut self, now_ticks: u32, outputs: &mut [&mut dyn OutputPin]) {
        self.check_and_execute_raw(now_ticks, outputs)
    }

    /// Clear all events
    ///
    /// Useful for emergency shutdown or reset.
    pub fn clear(&mut self) {
        self.deactivate_all();
    }

    /// Deactivate all scheduled events.
    pub fn deactivate_all(&mut self) {
        #[cfg(debug_assertions)]
        let _guard = Self::enter_guard(&self.reentry_guard);
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
    pub fn deactivate_channel_after(&mut self, cutoff: Micros, channel: Channel) {
        self.deactivate_channel_after_raw(cutoff.raw(), channel)
    }

    fn deactivate_channel_after_raw(&mut self, cutoff: u32, channel: Channel) {
        for e in &mut self.events {
            if e.is_active() && e.channel == channel {
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
            if e.is_active() && e.channel == channel {
                e.deactivate();
            }
        }
    }

    #[cfg(debug_assertions)]
    fn enter_guard(flag: &Cell<bool>) -> ReentryGuard {
        let was_set = flag.replace(true);
        debug_assert!(!was_set, "Scheduler re-entrancy detected");
        ReentryGuard(flag as *const Cell<bool>)
    }
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hal::OutputPin;

    struct MockPin {
        high_count: usize,
        low_count: usize,
    }

    impl MockPin {
        fn new() -> Self {
            Self {
                high_count: 0,
                low_count: 0,
            }
        }
    }

    impl OutputPin for MockPin {
        fn set_high(&mut self) {
            self.high_count += 1;
        }

        fn set_low(&mut self) {
            self.low_count += 1;
        }
    }

    #[test]
    fn deactivate_all() {
        let mut scheduler = Scheduler::new();
        assert!(scheduler.schedule(Micros::new(100), Channel::INJ1, true));
        assert!(scheduler.schedule(Micros::new(110), Channel::INJ1, false));
        assert!(scheduler.schedule(Micros::new(120), Channel::INJ2, true));
        assert!(scheduler.schedule(Micros::new(130), Channel::IGN1, true));

        scheduler.deactivate_all();

        let mut pin0 = MockPin::new();
        let mut pin1 = MockPin::new();
        let mut pin2 = MockPin::new();
        let mut outputs: [&mut dyn OutputPin; 3] = [&mut pin0, &mut pin1, &mut pin2];

        scheduler.check_and_execute(Micros::new(200), &mut outputs);

        assert_eq!(pin0.high_count, 0);
        assert_eq!(pin0.low_count, 0);
        assert_eq!(pin1.high_count, 0);
        assert_eq!(pin1.low_count, 0);
        assert_eq!(pin2.high_count, 0);
        assert_eq!(pin2.low_count, 0);
        assert_eq!(scheduler.active_count(), 0);
    }
}
