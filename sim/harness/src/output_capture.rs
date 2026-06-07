//! Fixed-size output transition capture buffer.
//!
//! A no_std-compatible ring buffer for capturing pin-level output transitions
//! without heap allocation.

use ecu_io::{OutputTransition, OutputTransitionSink};

/// Error when capture buffer is full.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CaptureError {
    /// The capture buffer has overflowed.
    Full,
}

/// Fixed-size output transition buffer with overflow tracking.
///
/// This buffer stores up to N `OutputTransition` records and implements
/// `OutputTransitionSink` for use in test harnesses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedTransitionBuffer<const N: usize> {
    /// Storage for transitions.
    buf: [Option<OutputTransition>; N],
    /// Next write position.
    head: usize,
    /// Number of valid entries.
    len: usize,
    /// Number of times overflow has occurred.
    overflow_count: u32,
}

impl<const N: usize> FixedTransitionBuffer<N> {
    /// Create a new empty capture buffer.
    pub const fn new() -> Self {
        Self {
            buf: [None; N],
            head: 0,
            len: 0,
            overflow_count: 0,
        }
    }

    /// Returns the number of transitions currently stored.
    pub fn len(&self) -> usize {
        self.len
    }

    /// Returns true if no transitions are stored.
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns the maximum capacity of this buffer.
    pub fn capacity(&self) -> usize {
        N
    }

    /// Returns the number of times the buffer has overflowed.
    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    /// Clears all entries from the buffer.
    ///
    /// Does not reset `overflow_count`.
    pub fn clear(&mut self) {
        self.buf = [None; N];
        self.head = 0;
        self.len = 0;
    }

    /// Get a transition by index, if available.
    pub fn get(&self, index: usize) -> Option<OutputTransition> {
        if index >= self.len || N == 0 {
            return None;
        }
        let oldest = if self.len == N { self.head } else { 0 };
        let pos = (oldest + index) % N;
        self.buf[pos]
    }

    /// Push a new transition into the buffer.
    ///
    /// If the buffer is full, returns `Err(CaptureError::Full)` and increments
    /// `overflow_count`. The oldest entry is overwritten.
    pub fn push(&mut self, transition: OutputTransition) -> Result<(), CaptureError> {
        if self.len == N {
            self.overflow_count = self.overflow_count.wrapping_add(1);
            return Err(CaptureError::Full);
        }
        let pos = self.head;
        self.buf[pos] = Some(transition);
        self.head = (self.head + 1) % N;
        self.len += 1;
        Ok(())
    }
}

impl<const N: usize> Default for FixedTransitionBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> OutputTransitionSink for FixedTransitionBuffer<N> {
    type Error = CaptureError;

    fn push_transition(&mut self, transition: OutputTransition) -> Result<(), Self::Error> {
        self.push(transition)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::{ChannelId, Micros};
    use ecu_io::{OutputLevel, OutputTransitionKind};

    fn make_transition(at_us: u32, channel: u8) -> OutputTransition {
        OutputTransition {
            at_us: Micros::new(at_us),
            kind: OutputTransitionKind::Injector,
            channel: ChannelId::new(channel),
            level: OutputLevel::High,
        }
    }

    #[test]
    fn empty_buffer_reports_length_zero() {
        let buf = FixedTransitionBuffer::<8>::new();
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
    }

    #[test]
    fn push_preserves_insertion_order() {
        let mut buf = FixedTransitionBuffer::<4>::new();
        buf.push(make_transition(100, 1)).unwrap();
        buf.push(make_transition(200, 2)).unwrap();

        assert_eq!(buf.len(), 2);
        assert_eq!(buf.get(0).unwrap().at_us.get(), 100);
        assert_eq!(buf.get(1).unwrap().at_us.get(), 200);
    }

    #[test]
    fn full_buffer_returns_full_and_increments_overflow() {
        let mut buf = FixedTransitionBuffer::<2>::new();
        buf.push(make_transition(100, 1)).unwrap();
        buf.push(make_transition(200, 2)).unwrap();
        assert_eq!(buf.overflow_count(), 0);

        // Third push should fail and increment overflow
        let result = buf.push(make_transition(300, 3));
        assert!(matches!(result, Err(CaptureError::Full)));
        assert_eq!(buf.overflow_count(), 1);
    }

    #[test]
    fn clear_allows_reuse() {
        let mut buf = FixedTransitionBuffer::<4>::new();
        buf.push(make_transition(100, 1)).unwrap();
        buf.push(make_transition(200, 2)).unwrap();
        assert_eq!(buf.len(), 2);

        buf.clear();
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
        // Overflow count should NOT be reset
        assert_eq!(buf.overflow_count(), 0);
    }

    #[test]
    fn overflow_count_increments_on_each_overflow() {
        let mut buf = FixedTransitionBuffer::<2>::new();
        buf.push(make_transition(100, 1)).unwrap();
        buf.push(make_transition(200, 2)).unwrap();

        for i in 0..5 {
            let _ = buf.push(make_transition(300 + i, 3));
        }
        assert_eq!(buf.overflow_count(), 5);
    }

    #[test]
    fn overflow_count_survives_clear_and_reuse() {
        let mut buf = FixedTransitionBuffer::<2>::new();
        buf.push(make_transition(100, 1)).unwrap();
        buf.push(make_transition(200, 2)).unwrap();

        assert!(matches!(
            buf.push(make_transition(300, 3)),
            Err(CaptureError::Full)
        ));
        assert_eq!(buf.overflow_count(), 1);

        buf.clear();
        assert_eq!(buf.len(), 0);
        assert!(buf.is_empty());
        assert_eq!(buf.overflow_count(), 1);

        buf.push(make_transition(400, 4)).unwrap();
        assert_eq!(buf.len(), 1);
        assert_eq!(buf.overflow_count(), 1);
        assert_eq!(buf.get(0).unwrap().at_us.get(), 400);
    }

    #[test]
    fn zero_capacity_buffer_handled_without_panic() {
        let mut buf = FixedTransitionBuffer::<0>::new();
        assert_eq!(buf.capacity(), 0);
        assert!(buf.is_empty());

        // Push should always fail
        let result = buf.push(make_transition(100, 1));
        assert!(matches!(result, Err(CaptureError::Full)));
        assert_eq!(buf.overflow_count(), 1);

        // Clear should work
        buf.clear();
        assert_eq!(buf.overflow_count(), 1); // Not reset
    }

    #[test]
    fn ring_buffer_fills_and_returns_full() {
        let mut buf = FixedTransitionBuffer::<4>::new();
        // Fill the buffer
        for i in 0..4 {
            buf.push(make_transition((i + 1) * 100, i as u8)).unwrap();
        }
        assert_eq!(buf.len(), 4);

        // Buffer is full - push should return Full
        let result = buf.push(make_transition(500, 10));
        assert!(matches!(result, Err(CaptureError::Full)));

        // Entries should still be the original 4
        assert_eq!(buf.get(0).unwrap().at_us.get(), 100);
        assert_eq!(buf.get(3).unwrap().at_us.get(), 400);
    }

    #[test]
    fn full_buffer_reads_from_wrapped_head_without_underflow() {
        let mut buf = FixedTransitionBuffer::<2>::new();
        buf.push(make_transition(100, 1)).unwrap();
        buf.push(make_transition(200, 2)).unwrap();

        assert_eq!(buf.len(), 2);
        assert_eq!(buf.get(0).unwrap().at_us.get(), 100);
        assert_eq!(buf.get(1).unwrap().at_us.get(), 200);
        assert!(buf.get(2).is_none());
    }

    #[test]
    fn output_transition_sink_trait_impl() {
        use ecu_io::OutputTransitionSink;

        let mut buf = FixedTransitionBuffer::<3>::new();
        let t = make_transition(100, 1);
        OutputTransitionSink::push_transition(&mut buf, t).unwrap();
        assert_eq!(buf.len(), 1);
    }
}
