//! Lightweight lock-free ring buffer for timestamp capture (u32)
//!
//! Intended for passing trigger edge timestamps from ISR to main loop.

#[derive(Debug)]
pub struct CaptureBuffer<const N: usize> {
    buf: [u32; N],
    head: u8,
    tail: u8,
}

impl<const N: usize> CaptureBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buf: [0; N],
            head: 0,
            tail: 0,
        }
    }

    /// Push a timestamp if space is available (drops on overflow)
    pub fn push(&mut self, ts: u32) {
        let next = self.head.wrapping_add(1);
        if next != self.tail {
            self.buf[self.head as usize] = ts;
            self.head = next;
        }
    }

    /// Pop a timestamp if available
    pub fn try_pop(&mut self) -> Option<u32> {
        if self.tail == self.head {
            None
        } else {
            let v = self.buf[self.tail as usize];
            self.tail = self.tail.wrapping_add(1);
            Some(v)
        }
    }
}

impl<const N: usize> Default for CaptureBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}
