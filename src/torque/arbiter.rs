//! Torque Arbitration
//!
//! Coordinates multiple torque requests and selects the final target.
//! Uses "min-wins" logic: the most restrictive request wins for safety.

use super::request::{TorqueRequest, TorqueSource, priority};

/// Maximum number of torque sources
pub const MAX_SOURCES: usize = 8;

/// Torque arbiter - coordinates multiple torque requests
#[derive(Debug, Clone, Copy)]
pub struct TorqueArbiter {
    /// Requests from each source
    pub requests: [TorqueRequest; MAX_SOURCES],
    /// Which source won the last arbitration
    pub winning_source: Option<TorqueSource>,
    /// Last arbitrated torque (Nm x10)
    pub last_result_x10: i16,
}

impl TorqueArbiter {
    pub const fn new() -> Self {
        Self {
            requests: [
                TorqueRequest::new(TorqueSource::Driver),
                TorqueRequest::new(TorqueSource::Idle),
                TorqueRequest::new(TorqueSource::RevLimiter),
                TorqueRequest::new(TorqueSource::Traction),
                TorqueRequest::new(TorqueSource::Limp),
                TorqueRequest::new(TorqueSource::External),
                TorqueRequest::new(TorqueSource::AntiStall),
                TorqueRequest::new(TorqueSource::Knock),
            ],
            winning_source: None,
            last_result_x10: 0,
        }
    }

    /// Get the slot index for a torque source
    fn source_index(source: TorqueSource) -> usize {
        match source {
            TorqueSource::Driver => 0,
            TorqueSource::Idle => 1,
            TorqueSource::RevLimiter => 2,
            TorqueSource::Traction => 3,
            TorqueSource::Limp => 4,
            TorqueSource::External => 5,
            TorqueSource::AntiStall => 6,
            TorqueSource::Knock => 7,
        }
    }

    /// Submit a torque request
    ///
    /// Overwrites any previous request from the same source.
    pub fn request(&mut self, req: TorqueRequest) {
        let idx = Self::source_index(req.source);
        self.requests[idx] = req;
    }

    /// Clear requests from a specific source
    pub fn clear(&mut self, source: TorqueSource) {
        let idx = Self::source_index(source);
        self.requests[idx].active = false;
        self.requests[idx].torque_nm_x10 = i16::MAX; // Inactive = unlimited
    }

    /// Arbitrate among all active requests
    ///
    /// Uses "min-wins" logic: the lowest (most restrictive) torque request wins.
    /// High-priority requests (limp, rev limiter) can override others.
    ///
    /// # Arguments
    /// * `max_available_x10` - Maximum available torque from the engine
    ///
    /// # Returns
    /// The arbitrated torque target (Nm x10)
    pub fn arbitrate(&mut self, max_available_x10: i16) -> i16 {
        let mut min_torque = max_available_x10;
        let mut winning: Option<TorqueSource> = None;
        let mut highest_priority = 0u8;

        for req in &self.requests {
            if !req.active {
                continue;
            }

            // For high-priority sources, they always win if active
            if req.priority >= priority::REV_LIMITER && req.torque_nm_x10 < i16::MAX {
                if req.priority > highest_priority {
                    highest_priority = req.priority;
                    min_torque = req.torque_nm_x10;
                    winning = Some(req.source);
                }
            }
        }

        // If a high-priority source is active, it wins
        if highest_priority >= priority::REV_LIMITER {
            self.winning_source = winning;
            self.last_result_x10 = min_torque.min(max_available_x10).max(0);
            return self.last_result_x10;
        }

        // Otherwise, use min-wins among normal priority requests
        // Start with max available, reduce based on requests
        min_torque = max_available_x10;
        let mut any_active = false;

        for req in &self.requests {
            if !req.active {
                continue;
            }
            any_active = true;

            // Clamp request to max available, then apply min-wins
            let effective_request = if req.torque_nm_x10 >= i16::MAX {
                max_available_x10 // Unlimited request = max available
            } else {
                req.torque_nm_x10.min(max_available_x10)
            };

            if effective_request < min_torque {
                min_torque = effective_request;
                winning = Some(req.source);
            } else if winning.is_none() {
                // First active request - use it as baseline
                min_torque = effective_request;
                winning = Some(req.source);
            }
        }

        // If no active requests, default to 0
        if !any_active {
            winning = Some(TorqueSource::Driver);
            min_torque = 0;
        }

        self.winning_source = winning;
        // Clamp to valid range
        self.last_result_x10 = min_torque.clamp(0, max_available_x10);
        self.last_result_x10
    }

    /// Get the source that won the last arbitration
    pub fn get_winning_source(&self) -> Option<TorqueSource> {
        self.winning_source
    }

    /// Check if a specific source is limiting torque
    pub fn is_source_limiting(&self, source: TorqueSource) -> bool {
        self.winning_source == Some(source)
    }

    /// Reset all requests
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Get the request from a specific source
    pub fn get_request(&self, source: TorqueSource) -> &TorqueRequest {
        &self.requests[Self::source_index(source)]
    }

    /// Count active requests
    pub fn active_count(&self) -> u8 {
        let mut count = 0u8;
        for req in &self.requests {
            if req.active {
                count = count.saturating_add(1);
            }
        }
        count
    }
}

impl Default for TorqueArbiter {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::super::request::*;

    #[test]
    fn test_arbiter_new() {
        let arbiter = TorqueArbiter::new();
        assert_eq!(arbiter.active_count(), 0);
        assert_eq!(arbiter.winning_source, None);
    }

    #[test]
    fn test_arbiter_single_request() {
        let mut arbiter = TorqueArbiter::new();

        let req = driver_torque_request(50, 3000, 2000, 0);
        arbiter.request(req);

        let result = arbiter.arbitrate(2000);
        assert_eq!(result, 1000); // 50% of 2000
        assert_eq!(arbiter.winning_source, Some(TorqueSource::Driver));
    }

    #[test]
    fn test_arbiter_min_wins() {
        let mut arbiter = TorqueArbiter::new();

        // Driver wants 100%
        arbiter.request(driver_torque_request(100, 3000, 2000, 0));
        // Idle wants less
        arbiter.request(idle_torque_request(800, 900, 50, 0)); // Above target, negative

        let result = arbiter.arbitrate(2000);
        // Idle request is negative (below zero), should win
        assert!(result < 2000);
    }

    #[test]
    fn test_arbiter_rev_limiter_wins() {
        let mut arbiter = TorqueArbiter::new();

        // Driver wants 100%
        arbiter.request(driver_torque_request(100, 3000, 2000, 0));
        // Rev limiter active
        arbiter.request(rev_limiter_torque_request(true, 0));

        let result = arbiter.arbitrate(2000);
        assert_eq!(result, 0); // Rev limiter = 0 torque
        assert_eq!(arbiter.winning_source, Some(TorqueSource::RevLimiter));
    }

    #[test]
    fn test_arbiter_limp_mode_wins() {
        let mut arbiter = TorqueArbiter::new();

        // Driver wants 100%
        arbiter.request(driver_torque_request(100, 3000, 2000, 0));
        // Limp mode active
        arbiter.request(limp_torque_request(true, 2000, 0));

        let result = arbiter.arbitrate(2000);
        assert_eq!(result, 600); // Limp = 30% of max
        assert_eq!(arbiter.winning_source, Some(TorqueSource::Limp));
    }

    #[test]
    fn test_arbiter_limp_beats_rev_limiter() {
        let mut arbiter = TorqueArbiter::new();

        // Both rev limiter and limp active
        arbiter.request(rev_limiter_torque_request(true, 0));
        arbiter.request(limp_torque_request(true, 2000, 0));

        let result = arbiter.arbitrate(2000);
        // Limp has higher priority but rev limiter has lower torque
        // In our implementation, limp wins because higher priority
        assert_eq!(arbiter.winning_source, Some(TorqueSource::Limp));
    }

    #[test]
    fn test_arbiter_clear_source() {
        let mut arbiter = TorqueArbiter::new();

        arbiter.request(driver_torque_request(100, 3000, 2000, 0));
        assert_eq!(arbiter.active_count(), 1);

        arbiter.clear(TorqueSource::Driver);
        assert_eq!(arbiter.active_count(), 0);
    }

    #[test]
    fn test_arbiter_reset() {
        let mut arbiter = TorqueArbiter::new();

        arbiter.request(driver_torque_request(100, 3000, 2000, 0));
        arbiter.request(limp_torque_request(true, 2000, 0));
        arbiter.arbitrate(2000);

        arbiter.reset();

        assert_eq!(arbiter.active_count(), 0);
        assert_eq!(arbiter.winning_source, None);
    }

    #[test]
    fn test_arbiter_clamps_to_max() {
        let mut arbiter = TorqueArbiter::new();

        // Request more than available
        arbiter.request(TorqueRequest::active(
            TorqueSource::Driver,
            5000, // Request 500 Nm
            priority::DRIVER,
            0,
        ));

        let result = arbiter.arbitrate(2000); // Only 200 Nm available
        assert_eq!(result, 2000);
    }

    #[test]
    fn test_arbiter_clamps_to_zero() {
        let mut arbiter = TorqueArbiter::new();

        // Request negative torque
        arbiter.request(TorqueRequest::active(
            TorqueSource::Idle,
            -100,
            priority::IDLE,
            0,
        ));

        let result = arbiter.arbitrate(2000);
        assert_eq!(result, 0); // Can't go below 0
    }

    #[test]
    fn test_arbiter_no_active_requests() {
        let mut arbiter = TorqueArbiter::new();

        let result = arbiter.arbitrate(2000);
        assert_eq!(result, 0); // Default to 0 with no requests
    }

    #[test]
    fn test_arbiter_knock_reduction() {
        let mut arbiter = TorqueArbiter::new();

        // Driver wants 100%
        arbiter.request(driver_torque_request(100, 3000, 2000, 0));
        // Knock causes 10 degrees retard = 20% reduction
        arbiter.request(knock_torque_request(100, 2000, 0));

        let result = arbiter.arbitrate(2000);
        assert!(result < 2000);
    }
}
