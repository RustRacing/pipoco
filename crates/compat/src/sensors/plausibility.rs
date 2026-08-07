//! Sensor plausibility checks
//!
//! Detects implausible sensor combinations that indicate sensor failure.
//! For example, high TPS with low MAP (or vice versa) shouldn't occur
//! in normal operation.

/// Plausibility fault type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PlausibilityFault {
    /// No fault detected
    #[default]
    None,
    /// TPS high but MAP low - stuck TPS high or MAP stuck low
    TpsHighMapLow,
    /// TPS low but MAP high - stuck TPS low or MAP stuck high
    TpsLowMapHigh,
}

pub use ecu_calibration::configs::PlausibilityConfig;

/// State for plausibility checking
#[derive(Debug, Clone, Copy)]
pub struct PlausibilityState {
    /// Currently detected fault (before debounce)
    pub pending_fault: PlausibilityFault,
    /// Confirmed fault (after debounce)
    pub confirmed_fault: PlausibilityFault,
    /// Timestamp when pending fault was first detected
    pub fault_start_us: u32,
    /// Is the fault currently confirmed?
    pub fault_confirmed: bool,
    /// Timestamp when fault cleared (for recovery tracking)
    pub clear_start_us: u32,
    /// Last sample timestamp (canonical accumulator debounce)
    initialized: bool,
    last_t_us: u32,
    /// Accumulated fault persistence (canonical debounce counters)
    assert_counter_us: u32,
    clear_counter_us: u32,
}

impl PlausibilityState {
    /// Create new plausibility state
    pub const fn new() -> Self {
        Self {
            pending_fault: PlausibilityFault::None,
            confirmed_fault: PlausibilityFault::None,
            fault_start_us: 0,
            fault_confirmed: false,
            clear_start_us: 0,
            initialized: false,
            last_t_us: 0,
            assert_counter_us: 0,
            clear_counter_us: 0,
        }
    }

    /// Check TPS vs MAP plausibility
    ///
    /// # Arguments
    /// * `tps_percent` - Throttle position (0-100%)
    /// * `map_kpa_x10` - Manifold pressure (kPa x10)
    /// * `rpm` - Current engine RPM
    /// * `config` - Plausibility configuration
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current confirmed fault (None if no fault)
    ///
    /// Uses the canonical accumulator debounce from
    /// `ecu_control::sensors::plausibility` (review 011): the fault must
    /// persist `debounce_time_us` of *sampled* time (dt accumulates per
    /// step) before it confirms, matching the spec oracle verdict stream.
    pub fn check(
        &mut self,
        tps_percent: u8,
        map_kpa_x10: u16,
        rpm: u16,
        config: &PlausibilityConfig,
        now_us: u32,
    ) -> PlausibilityFault {
        if !config.enable {
            return PlausibilityFault::None;
        }

        let dt_us = if self.initialized {
            now_us.wrapping_sub(self.last_t_us)
        } else {
            0
        };
        self.initialized = true;
        self.last_t_us = now_us;

        // Don't check at low RPM (idle, cranking) - hold current verdict.
        if rpm < config.min_rpm {
            return self.confirmed_fault;
        }

        // Detect fault conditions
        let current_fault = self.detect_fault(tps_percent, map_kpa_x10, config);
        let prev_pending = self.pending_fault;
        self.pending_fault = current_fault;

        if current_fault != PlausibilityFault::None {
            // Fault detected: accumulate persistence; clear counter resets.
            if prev_pending != current_fault {
                self.fault_start_us = now_us;
            }
            self.assert_counter_us = self.assert_counter_us.saturating_add(dt_us);
            if self.assert_counter_us > config.debounce_time_us {
                self.assert_counter_us = config.debounce_time_us;
            }
            self.clear_counter_us = 0;
            self.clear_start_us = 0;
            if self.assert_counter_us >= config.debounce_time_us && !self.fault_confirmed {
                self.fault_confirmed = true;
                self.confirmed_fault = current_fault;
            }
        } else {
            // No fault: clear counter accumulates; assert resets.
            self.clear_counter_us = self.clear_counter_us.saturating_add(dt_us);
            if self.clear_counter_us > config.debounce_time_us {
                self.clear_counter_us = config.debounce_time_us;
            }
            self.assert_counter_us = 0;
            if self.clear_start_us == 0 {
                self.clear_start_us = now_us;
            }
            if self.clear_counter_us >= config.debounce_time_us {
                self.fault_confirmed = false;
                self.confirmed_fault = PlausibilityFault::None;
                self.clear_start_us = 0;
            }
        }

        self.confirmed_fault
    }

    /// Detect instantaneous fault condition
    fn detect_fault(
        &self,
        tps_percent: u8,
        map_kpa_x10: u16,
        config: &PlausibilityConfig,
    ) -> PlausibilityFault {
        // High TPS with low MAP is implausible
        // (throttle open should mean higher manifold pressure)
        if tps_percent >= config.tps_high_threshold && map_kpa_x10 <= config.map_low_threshold_x10 {
            return PlausibilityFault::TpsHighMapLow;
        }

        // Low TPS with high MAP is implausible
        // (throttle closed should mean vacuum, not near-atmospheric)
        if tps_percent <= config.tps_low_threshold && map_kpa_x10 >= config.map_high_threshold_x10 {
            return PlausibilityFault::TpsLowMapHigh;
        }

        PlausibilityFault::None
    }

    /// Check if a fault is currently confirmed
    pub fn has_fault(&self) -> bool {
        self.fault_confirmed
    }

    /// Get the current confirmed fault
    pub fn get_fault(&self) -> PlausibilityFault {
        self.confirmed_fault
    }

    /// Reset the plausibility state
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for PlausibilityState {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Rate-of-Change Validation
// ============================================================================

pub use ecu_calibration::configs::RateConfig;

/// Rate validator for a single sensor.
///
/// Delegates to the canonical `ecu_control::sensors::slew` clamp math
/// (delta-window per second, review 011): a sample that moves faster than
/// `max_rate_per_sec` is clamped to the rate limit instead of rejected.
#[derive(Debug, Clone, Copy)]
pub struct RateValidator {
    /// Last valid value
    pub last_value: u16,
    /// Last update timestamp
    pub last_time_us: u32,
    /// Was last reading clamped (rate exceeded)?
    pub last_rejected: bool,
    /// Count of consecutive clamps
    pub reject_count: u8,
    /// Has the validator been initialized with at least one sample?
    pub initialized: bool,
}

impl RateValidator {
    /// Create new rate validator
    pub const fn new(initial_value: u16) -> Self {
        Self {
            last_value: initial_value,
            last_time_us: 0,
            last_rejected: false,
            reject_count: 0,
            initialized: false,
        }
    }

    /// Validate a new sensor reading
    ///
    /// # Arguments
    /// * `value` - New sensor value
    /// * `now_us` - Current timestamp in microseconds
    /// * `max_rate_per_sec` - Maximum allowed change per second
    /// * `min_interval_us` - Minimum time between samples
    ///
    /// # Returns
    /// Rate-limited value (clamped toward the candidate when the rate is
    /// exceeded; the previous value is held for samples inside the minimum
    /// interval, matching the canonical slew short-dt rule).
    pub fn validate(
        &mut self,
        value: u16,
        now_us: u32,
        max_rate_per_sec: u16,
        min_interval_us: u32,
    ) -> u16 {
        // First sample ever - accept unconditionally
        if !self.initialized {
            self.last_value = value;
            self.last_time_us = now_us;
            self.last_rejected = false;
            self.initialized = true;
            return value;
        }

        let elapsed = now_us.wrapping_sub(self.last_time_us);

        // Too soon since last sample - hold the last accepted value.
        if elapsed < min_interval_us {
            self.last_time_us = now_us;
            self.last_rejected = false;
            return self.last_value;
        }

        // Canonical delta-window clamp: max delta = rate * elapsed / 1e6.
        let delta = ecu_control::sensors::slew::max_delta(max_rate_per_sec as u32, elapsed);
        let limited = ecu_control::sensors::slew::limit_u16(self.last_value, value, delta);
        self.last_rejected = limited != value;
        if self.last_rejected {
            self.reject_count = self.reject_count.saturating_add(1);
        } else {
            self.reject_count = 0;
        }
        self.last_value = limited;
        self.last_time_us = now_us;
        limited
    }

    /// Check if the last reading was rejected
    pub fn was_rejected(&self) -> bool {
        self.last_rejected
    }

    /// Get the current validated value
    pub fn get_value(&self) -> u16 {
        self.last_value
    }

    /// Reset the validator with a new initial value
    pub fn reset(&mut self, value: u16, now_us: u32) {
        self.last_value = value;
        self.last_time_us = now_us;
        self.last_rejected = false;
        self.reject_count = 0;
        self.initialized = true;
    }
}

impl Default for RateValidator {
    fn default() -> Self {
        Self::new(0)
    }
}

/// Combined rate validation state for TPS and MAP
#[derive(Debug, Clone, Copy)]
pub struct RateValidationState {
    pub tps_validator: RateValidator,
    pub map_validator: RateValidator,
}

impl RateValidationState {
    /// Create new rate validation state
    pub const fn new() -> Self {
        Self {
            tps_validator: RateValidator::new(0),
            map_validator: RateValidator::new(1000), // ~100 kPa default
        }
    }

    /// Validate TPS and MAP readings
    ///
    /// # Returns
    /// (validated_tps, validated_map, tps_rejected, map_rejected)
    pub fn validate(
        &mut self,
        tps_percent: u8,
        map_kpa_x10: u16,
        config: &RateConfig,
        now_us: u32,
    ) -> (u8, u16, bool, bool) {
        if !config.enable {
            return (tps_percent, map_kpa_x10, false, false);
        }

        let validated_tps = self.tps_validator.validate(
            tps_percent as u16,
            now_us,
            config.max_tps_rate_per_sec,
            config.min_sample_interval_us,
        ) as u8;

        let validated_map = self.map_validator.validate(
            map_kpa_x10,
            now_us,
            config.max_map_rate_per_sec,
            config.min_sample_interval_us,
        );

        let tps_rejected = self.tps_validator.was_rejected();
        let map_rejected = self.map_validator.was_rejected();

        (validated_tps, validated_map, tps_rejected, map_rejected)
    }

    /// Check if any sensor was rejected
    pub fn any_rejected(&self) -> bool {
        self.tps_validator.was_rejected() || self.map_validator.was_rejected()
    }

    /// Reset validators
    pub fn reset(&mut self, tps: u8, map: u16, now_us: u32) {
        self.tps_validator.reset(tps as u16, now_us);
        self.map_validator.reset(map, now_us);
    }
}

impl Default for RateValidationState {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "plausibility_tests.rs"]
mod tests;
