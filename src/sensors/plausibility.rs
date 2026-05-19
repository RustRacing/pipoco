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

/// Configuration for plausibility checks
#[derive(Debug, Clone, Copy)]
pub struct PlausibilityConfig {
    /// Enable plausibility checking
    pub enable: bool,
    /// TPS threshold above which MAP should not be low (percent)
    pub tps_high_threshold: u8,
    /// MAP threshold below which is considered low (kPa x10)
    pub map_low_threshold_x10: u16,
    /// TPS threshold below which MAP should not be high (percent)
    pub tps_low_threshold: u8,
    /// MAP threshold above which is considered high (kPa x10)
    /// For naturally aspirated engines, ~95 kPa is max
    pub map_high_threshold_x10: u16,
    /// Minimum RPM for plausibility check (don't check at idle/cranking)
    pub min_rpm: u16,
    /// Debounce time before confirming fault (microseconds)
    pub debounce_time_us: u32,
}

impl PlausibilityConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        tps_high_threshold: 80,      // 80% TPS
        map_low_threshold_x10: 300,  // 30 kPa
        tps_low_threshold: 10,       // 10% TPS
        map_high_threshold_x10: 950, // 95 kPa (NA engine)
        min_rpm: 1000,               // Don't check below 1000 RPM
        debounce_time_us: 500_000,   // 500ms debounce
    };
}

impl Default for PlausibilityConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

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

        // Don't check at low RPM (idle, cranking)
        if rpm < config.min_rpm {
            // Clear any pending fault
            self.pending_fault = PlausibilityFault::None;
            return self.confirmed_fault;
        }

        // Detect fault conditions
        let current_fault = self.detect_fault(tps_percent, map_kpa_x10, config);

        if current_fault != PlausibilityFault::None {
            // Fault detected
            self.clear_start_us = 0;

            if self.pending_fault != current_fault {
                // New fault type, start debounce
                self.pending_fault = current_fault;
                self.fault_start_us = now_us;
            } else {
                // Same fault, check debounce
                let elapsed = now_us.wrapping_sub(self.fault_start_us);
                if elapsed >= config.debounce_time_us && !self.fault_confirmed {
                    self.fault_confirmed = true;
                    self.confirmed_fault = current_fault;
                }
            }
        } else {
            // No fault - check for recovery
            self.pending_fault = PlausibilityFault::None;

            if self.fault_confirmed {
                // Start recovery timer
                if self.clear_start_us == 0 {
                    self.clear_start_us = now_us;
                }

                // Require same debounce time for recovery
                let clear_elapsed = now_us.wrapping_sub(self.clear_start_us);
                if clear_elapsed >= config.debounce_time_us {
                    self.fault_confirmed = false;
                    self.confirmed_fault = PlausibilityFault::None;
                    self.clear_start_us = 0;
                }
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

/// Configuration for rate-of-change validation
#[derive(Debug, Clone, Copy)]
pub struct RateConfig {
    /// Enable rate validation
    pub enable: bool,
    /// Maximum TPS change per second (percent/s)
    /// Typical: 500%/s allows 0-100% in 200ms
    pub max_tps_rate_per_sec: u16,
    /// Maximum MAP change per second (kPa x10 per second)
    /// Typical: 2000 = 200 kPa/s
    pub max_map_rate_per_sec: u16,
    /// Minimum time between samples for rate calculation (microseconds)
    /// Prevents division by very small numbers
    pub min_sample_interval_us: u32,
}

impl RateConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        max_tps_rate_per_sec: 500,    // 0-100% in 200ms
        max_map_rate_per_sec: 2000,   // 200 kPa/s
        min_sample_interval_us: 1000, // 1ms minimum
    };
}

impl Default for RateConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Rate validator for a single sensor
#[derive(Debug, Clone, Copy)]
pub struct RateValidator {
    /// Last valid value
    pub last_value: u16,
    /// Last update timestamp
    pub last_time_us: u32,
    /// Was last reading rejected?
    pub last_rejected: bool,
    /// Count of consecutive rejections
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
    /// Validated value (either new value if OK, or last-known-good if rejected)
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

        // Too soon since last sample - accept without rate check
        if elapsed < min_interval_us {
            self.last_value = value;
            self.last_time_us = now_us;
            self.last_rejected = false;
            return value;
        }

        // Calculate rate of change
        let delta = value.abs_diff(self.last_value);

        // Convert to rate per second
        // rate = delta / (elapsed_us / 1_000_000) = delta * 1_000_000 / elapsed_us
        let rate_per_sec = (delta as u32).saturating_mul(1_000_000) / elapsed;

        if rate_per_sec > max_rate_per_sec as u32 {
            // Rate exceeded - reject this sample
            self.last_rejected = true;
            self.reject_count = self.reject_count.saturating_add(1);

            // If we've rejected too many samples, accept anyway (sensor may be real)
            // This prevents getting stuck on a wrong value forever
            if self.reject_count >= 5 {
                self.last_value = value;
                self.last_time_us = now_us;
                self.reject_count = 0;
            }

            self.last_value // Return last-known-good
        } else {
            // Rate OK - accept
            self.last_value = value;
            self.last_time_us = now_us;
            self.last_rejected = false;
            self.reject_count = 0;
            value
        }
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
mod tests {
    use super::*;

    #[test]
    fn test_no_fault_normal_conditions() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Normal: 50% TPS, 70 kPa
        let fault = state.check(50, 700, 3000, &config, 0);
        assert_eq!(fault, PlausibilityFault::None);
        assert!(!state.has_fault());
    }

    #[test]
    fn test_no_fault_at_low_rpm() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Implausible values but at low RPM - should not fault
        let fault = state.check(90, 200, 500, &config, 0);
        assert_eq!(fault, PlausibilityFault::None);
    }

    #[test]
    fn test_tps_high_map_low_fault() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // TPS high (85%), MAP low (25 kPa) at running RPM
        // First check - starts debounce
        let fault = state.check(85, 250, 3000, &config, 0);
        assert_eq!(fault, PlausibilityFault::None); // Not confirmed yet
        assert_eq!(state.pending_fault, PlausibilityFault::TpsHighMapLow);

        // Before debounce time
        let fault = state.check(85, 250, 3000, &config, 400_000);
        assert_eq!(fault, PlausibilityFault::None);

        // After debounce time - fault confirmed
        let fault = state.check(85, 250, 3000, &config, 600_000);
        assert_eq!(fault, PlausibilityFault::TpsHighMapLow);
        assert!(state.has_fault());
    }

    #[test]
    fn test_tps_low_map_high_fault() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // TPS low (5%), MAP high (96 kPa) at running RPM
        state.check(5, 960, 3000, &config, 0);
        state.check(5, 960, 3000, &config, 600_000);

        assert_eq!(state.confirmed_fault, PlausibilityFault::TpsLowMapHigh);
    }

    #[test]
    fn test_fault_clears_with_debounce() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Create fault
        state.check(85, 250, 3000, &config, 0);
        state.check(85, 250, 3000, &config, 600_000);
        assert!(state.has_fault());

        // Condition clears - start recovery
        state.check(50, 600, 3000, &config, 700_000);
        assert!(state.has_fault()); // Still faulted

        // After recovery debounce
        let fault = state.check(50, 600, 3000, &config, 1_300_000);
        assert_eq!(fault, PlausibilityFault::None);
        assert!(!state.has_fault());
    }

    #[test]
    fn test_debounce_resets_on_condition_change() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Start one fault
        state.check(85, 250, 3000, &config, 0);
        assert_eq!(state.pending_fault, PlausibilityFault::TpsHighMapLow);

        // Condition clears before debounce
        state.check(50, 600, 3000, &config, 400_000);
        assert_eq!(state.pending_fault, PlausibilityFault::None);
        assert!(!state.has_fault());

        // New fault starts fresh debounce
        state.check(85, 250, 3000, &config, 500_000);
        assert_eq!(state.fault_start_us, 500_000);
    }

    #[test]
    fn test_disabled_config() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig {
            enable: false,
            ..PlausibilityConfig::DEFAULT
        };

        // Implausible condition but checking disabled
        state.check(85, 250, 3000, &config, 0);
        state.check(85, 250, 3000, &config, 600_000);
        assert!(!state.has_fault());
    }

    #[test]
    fn test_reset() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Create confirmed fault
        state.check(85, 250, 3000, &config, 0);
        state.check(85, 250, 3000, &config, 600_000);
        assert!(state.has_fault());

        // Reset
        state.reset();
        assert!(!state.has_fault());
        assert_eq!(state.confirmed_fault, PlausibilityFault::None);
    }

    #[test]
    fn test_edge_cases_at_thresholds() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Exactly at TPS high threshold, exactly at MAP low threshold
        // (should trigger fault - >= and <=)
        state.check(80, 300, 3000, &config, 0);
        state.check(80, 300, 3000, &config, 600_000);
        assert_eq!(state.confirmed_fault, PlausibilityFault::TpsHighMapLow);
    }

    #[test]
    fn test_wot_with_good_map() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // WOT with good MAP (high pressure) - should be fine
        let fault = state.check(100, 950, 5000, &config, 0);
        assert_eq!(fault, PlausibilityFault::None);
    }

    #[test]
    fn test_closed_throttle_with_vacuum() {
        let mut state = PlausibilityState::new();
        let config = PlausibilityConfig::DEFAULT;

        // Closed throttle with vacuum - should be fine
        let fault = state.check(0, 300, 3000, &config, 0);
        assert_eq!(fault, PlausibilityFault::None);
    }

    // =========================================================================
    // Rate Validator Tests
    // =========================================================================

    #[test]
    fn test_rate_validator_accepts_first_sample() {
        let mut validator = RateValidator::new(50);

        let result = validator.validate(75, 10_000, 500, 1000);
        assert_eq!(result, 75);
        assert!(!validator.was_rejected());
    }

    #[test]
    fn test_rate_validator_accepts_normal_change() {
        let mut validator = RateValidator::new(50);

        // Initialize with first sample
        validator.validate(50, 0, 500, 1000);

        // Change of 25 in 100ms = 250/s, which is below 500/s limit
        let result = validator.validate(75, 100_000, 500, 1000);
        assert_eq!(result, 75);
        assert!(!validator.was_rejected());
    }

    #[test]
    fn test_rate_validator_rejects_spike() {
        let mut validator = RateValidator::new(50);

        // Initialize with first sample
        validator.validate(50, 0, 500, 1000);

        // Change of 80 in 10ms = 8000/s, way above 500/s limit
        let result = validator.validate(130, 10_000, 500, 1000);
        assert_eq!(result, 50); // Returns last-known-good
        assert!(validator.was_rejected());
    }

    #[test]
    fn test_rate_validator_accepts_after_5_rejections() {
        let mut validator = RateValidator::new(50);

        // Initialize with first sample
        validator.validate(50, 0, 500, 1000);

        // 5 consecutive rejections should force acceptance
        for i in 0..4 {
            let result = validator.validate(130, (i + 1) * 10_000, 500, 1000);
            assert_eq!(result, 50); // Still rejecting
        }

        // 5th rejection - should accept
        let result = validator.validate(130, 50_000, 500, 1000);
        assert_eq!(result, 130);
    }

    #[test]
    fn test_rate_validator_reset_clears_state() {
        let mut validator = RateValidator::new(50);

        // Initialize with first sample
        validator.validate(50, 0, 500, 1000);

        // Reject a sample
        validator.validate(130, 10_000, 500, 1000);
        assert!(validator.was_rejected());

        // Reset
        validator.reset(75, 20_000);

        assert!(!validator.was_rejected());
        assert_eq!(validator.get_value(), 75);
        assert_eq!(validator.reject_count, 0);
    }

    #[test]
    fn test_rate_validator_ignores_samples_within_min_interval() {
        let mut validator = RateValidator::new(50);

        // First sample
        validator.validate(50, 0, 500, 1000);

        // Sample within min interval - should accept without rate check
        let result = validator.validate(130, 500, 500, 1000);
        assert_eq!(result, 130); // Accepted even though rate would exceed
        assert!(!validator.was_rejected());
    }

    #[test]
    fn test_rate_validation_state_tps_and_map() {
        let mut state = RateValidationState::new();
        let config = RateConfig::DEFAULT;

        // Initialize
        state.validate(50, 800, &config, 0);

        // Normal change
        let (tps, map, tps_rej, map_rej) = state.validate(55, 820, &config, 100_000);
        assert_eq!(tps, 55);
        assert_eq!(map, 820);
        assert!(!tps_rej);
        assert!(!map_rej);
    }

    #[test]
    fn test_rate_validation_state_rejects_tps_spike() {
        let mut state = RateValidationState::new();
        let config = RateConfig::DEFAULT;

        // Initialize
        state.validate(50, 800, &config, 0);

        // TPS spike (0 to 100 in 10ms = 10000/s)
        let (tps, map, tps_rej, map_rej) = state.validate(100, 820, &config, 10_000);
        assert_eq!(tps, 50); // Rejected, returns last-good
        assert_eq!(map, 820); // MAP OK
        assert!(tps_rej);
        assert!(!map_rej);
        assert!(state.any_rejected());
    }

    #[test]
    fn test_rate_validation_disabled() {
        let mut state = RateValidationState::new();
        let config = RateConfig {
            enable: false,
            ..RateConfig::DEFAULT
        };

        state.validate(50, 800, &config, 0);

        // Spike should be passed through when disabled
        let (tps, map, tps_rej, map_rej) = state.validate(100, 2000, &config, 10_000);
        assert_eq!(tps, 100);
        assert_eq!(map, 2000);
        assert!(!tps_rej);
        assert!(!map_rej);
    }
}
