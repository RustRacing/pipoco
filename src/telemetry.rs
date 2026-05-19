//! Lightweight runtime telemetry for ISR/cycle timing
//!
//! Tracks count, max, and simple moving average of durations (microseconds).

#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct IsrStats {
    pub count: u32,
    pub max_us: u32,
    pub avg_us: u32,
}

impl IsrStats {
    pub const fn new() -> Self {
        Self {
            count: 0,
            max_us: 0,
            avg_us: 0,
        }
    }

    /// Update stats with a new duration sample (microseconds)
    pub fn update(&mut self, duration_us: u32) {
        self.count = self.count.saturating_add(1);
        if duration_us > self.max_us {
            self.max_us = duration_us;
        }

        // Simple running average with limited precision to keep it cheap
        // avg_{n+1} = avg_n + (x - avg_n) / 8  (EMA-like, power-of-two division)
        let delta = duration_us.wrapping_sub(self.avg_us);
        self.avg_us = self.avg_us.wrapping_add(delta >> 3);
    }
}
