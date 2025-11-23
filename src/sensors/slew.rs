//! Stateless interface wrapper for rate limiting using internal state.
//! Provides a small `SlewLimiter` that clamps value changes to a maximum rate per second.

/// Simple per-channel slew limiter.
/// Stores last value and timestamp; clamps new values to `max_rate_per_s`.
#[derive(Copy, Clone)]
pub struct SlewLimiter {
    last_value: i32,
    last_ts_us: u32,
    max_rate_per_s: i32,
    initialized: bool,
}

impl SlewLimiter {
    pub const fn new(max_rate_per_s: i32) -> Self {
        Self { last_value: 0, last_ts_us: 0, max_rate_per_s, initialized: false }
    }

    /// Apply slew limiting to `value` at time `now_us`.
    /// Returns clamped value; updates internal state.
    pub fn apply(&mut self, now_us: u32, value: i32) -> i32 {
        if !self.initialized {
            self.initialized = true;
            self.last_value = value;
            self.last_ts_us = now_us;
            return value;
        }
        let dt_us = now_us.wrapping_sub(self.last_ts_us);
        if dt_us == 0 || self.max_rate_per_s <= 0 {
            self.last_value = value;
            self.last_ts_us = now_us;
            return value;
        }
        // Maximum allowed delta = rate * dt
        let max_delta = ((self.max_rate_per_s as i64) * (dt_us as i64) / 1_000_000) as i32;
        let delta = value.saturating_sub(self.last_value);
        let clamped_delta = if delta > max_delta { max_delta } else if delta < -max_delta { -max_delta } else { delta };
        let new_val = self.last_value.saturating_add(clamped_delta);
        self.last_value = new_val;
        self.last_ts_us = now_us;
        new_val
    }
}

#[cfg(test)]
mod tests {
    use super::SlewLimiter;

    #[test]
    fn clamps_large_step_based_on_dt() {
        // 100 kPa/s
        let mut sl = SlewLimiter::new(100);
        let mut now = 0u32;
        assert_eq!(sl.apply(now, 1000), 1000);
        now += 100_000; // 0.1s -> max delta 10
        // request big jump: +200 -> expect +10
        let v = sl.apply(now, 1200);
        assert_eq!(v, 1010);
        now += 900_000; // 0.9s -> max delta 90
        let v2 = sl.apply(now, 2000);
        assert_eq!(v2, 1100);
    }

    #[test]
    fn allows_small_changes() {
        let mut sl = SlewLimiter::new(1000); // large rate
        let mut now = 0u32;
        let _ = sl.apply(now, 50);
        now += 10_000; // 10ms -> max delta 10
        let v = sl.apply(now, 55);
        assert_eq!(v, 55);
    }
}

