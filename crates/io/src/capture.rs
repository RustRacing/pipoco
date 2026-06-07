//! Lightweight board-facing capture buffers.

/// Simple timestamp ring buffer for ISR-to-main communication.
///
/// The buffer uses an empty-slot ring layout so `head == tail` can represent
/// empty without an extra full flag. Its effective capacity is therefore
/// `N.saturating_sub(1)`. When full, new timestamps are dropped and the
/// existing FIFO contents are preserved.
#[derive(Debug)]
pub struct CaptureBuffer<const N: usize> {
    buf: [u32; N],
    head: usize,
    tail: usize,
}

impl<const N: usize> CaptureBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            head: 0,
            tail: 0,
        }
    }

    /// Number of timestamps this buffer can hold before dropping new pushes.
    pub const fn capacity(&self) -> usize {
        Self::CAPACITY
    }

    /// Number of timestamps this buffer can hold before dropping new pushes.
    pub const CAPACITY: usize = N.saturating_sub(1);

    /// Returns true when another `push` would be dropped.
    pub fn is_full(&self) -> bool {
        if N == 0 {
            return true;
        }

        let next = (self.head + 1) % N;
        next == self.tail
    }

    /// Push a timestamp if space is available.
    ///
    /// Returns `true` when the timestamp was stored and `false` when it was
    /// dropped because the buffer was full or zero-sized.
    pub fn push(&mut self, ts: u32) -> bool {
        if !self.is_full() {
            self.buf[self.head] = ts;
            self.head = (self.head + 1) % N;
            true
        } else {
            false
        }
    }

    /// Pop a timestamp if available.
    pub fn try_pop(&mut self) -> Option<u32> {
        if self.tail == self.head {
            None
        } else {
            let v = self.buf[self.tail];
            self.tail = (self.tail + 1) % N;
            Some(v)
        }
    }
}

impl<const N: usize> Default for CaptureBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_buffer_push_pop_fifo_order() {
        let mut buf = CaptureBuffer::<4>::new();
        assert!(buf.try_pop().is_none());

        assert_eq!(buf.capacity(), 3);
        assert!(buf.push(100));
        assert!(buf.push(200));

        assert_eq!(buf.try_pop(), Some(100));
        assert_eq!(buf.try_pop(), Some(200));
        assert!(buf.try_pop().is_none());
    }

    #[test]
    fn capture_buffer_drops_on_overflow_without_indexing_past_capacity() {
        let mut buf = CaptureBuffer::<4>::new();

        assert_eq!(CaptureBuffer::<4>::CAPACITY, 3);
        assert!(!buf.is_full());
        assert!(buf.push(100));
        assert!(buf.push(200));
        assert!(buf.push(300));
        assert!(buf.is_full());
        assert!(!buf.push(400));
        assert!(!buf.push(500));

        assert_eq!(buf.try_pop(), Some(100));
        assert_eq!(buf.try_pop(), Some(200));
        assert_eq!(buf.try_pop(), Some(300));
        assert!(buf.try_pop().is_none());
    }

    #[test]
    fn zero_capacity_capture_buffer_drops_everything() {
        let mut buf = CaptureBuffer::<0>::new();

        assert_eq!(buf.capacity(), 0);
        assert!(buf.is_full());
        assert!(!buf.push(100));

        assert!(buf.try_pop().is_none());
    }

    #[test]
    fn one_slot_capture_buffer_has_no_effective_capacity() {
        let mut buf = CaptureBuffer::<1>::new();

        assert_eq!(buf.capacity(), 0);
        assert!(buf.is_full());
        assert!(!buf.push(100));
        assert!(buf.try_pop().is_none());
    }

    #[test]
    fn capture_buffer_can_reuse_space_after_wraparound() {
        let mut buf = CaptureBuffer::<3>::new();

        assert_eq!(buf.capacity(), 2);
        assert!(buf.push(10));
        assert!(buf.push(20));
        assert!(!buf.push(30));
        assert_eq!(buf.try_pop(), Some(10));
        assert!(buf.push(40));

        assert_eq!(buf.try_pop(), Some(20));
        assert_eq!(buf.try_pop(), Some(40));
        assert!(buf.try_pop().is_none());
    }
}
