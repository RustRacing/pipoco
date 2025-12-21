//! Knock Detection and Retard Controller
//!
//! Provides infrastructure for knock-based timing retard with:
//! - Per-cylinder knock tracking
//! - Configurable retard step on knock detection
//! - Gradual timing recovery when no knock
//! - Knock window timing (only listen during combustion)

/// Knock controller configuration
#[derive(Debug, Clone, Copy)]
pub struct KnockConfig {
    /// Enable knock control
    pub enable: bool,
    /// Knock detection threshold (ADC counts or normalized level)
    pub threshold: u16,
    /// Retard step per knock event (degrees x10)
    pub retard_step_x10: u16,
    /// Maximum retard allowed (degrees x10)
    pub retard_max_x10: u16,
    /// Recovery rate per second (degrees x10) when no knock
    pub recovery_rate_x10: u16,
    /// Knock window start (degrees BTDC)
    pub window_start_btdc: i16,
    /// Knock window end (degrees BTDC, typically negative = ATDC)
    pub window_end_btdc: i16,
    /// Minimum RPM for knock detection
    pub min_rpm: u16,
    /// Minimum coolant temperature for knock detection (Celsius)
    pub min_clt_c: i16,
    /// Number of consecutive knock events before applying retard
    pub debounce_count: u8,
    /// Maximum cylinder index (0-7)
    pub max_cylinders: u8,
}

impl KnockConfig {
    /// Default configuration for a 4-cylinder engine
    pub const DEFAULT: Self = Self {
        enable: true,
        threshold: 100,           // Baseline threshold
        retard_step_x10: 20,      // 2.0 degrees retard per knock
        retard_max_x10: 150,      // 15.0 degrees max retard
        recovery_rate_x10: 10,    // 1.0 degree per second recovery
        window_start_btdc: 20,    // Start listening at 20° BTDC
        window_end_btdc: -30,     // Stop at 30° ATDC
        min_rpm: 1500,
        min_clt_c: 60,
        debounce_count: 2,
        max_cylinders: 4,
    };

    /// Configuration for an 8-cylinder engine
    pub const DEFAULT_V8: Self = Self {
        max_cylinders: 8,
        ..Self::DEFAULT
    };
}

impl Default for KnockConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Per-cylinder knock state
#[derive(Debug, Clone, Copy, Default)]
pub struct CylinderKnockState {
    /// Current retard for this cylinder (degrees x10)
    pub retard_x10: u16,
    /// Total knock count for this cylinder
    pub knock_count: u16,
    /// Consecutive knock events (for debouncing)
    pub consecutive_knocks: u8,
    /// Time of last knock event (microseconds)
    pub last_knock_us: u32,
}

impl CylinderKnockState {
    pub const fn new() -> Self {
        Self {
            retard_x10: 0,
            knock_count: 0,
            consecutive_knocks: 0,
            last_knock_us: 0,
        }
    }
}

/// Overall knock controller state
#[derive(Debug, Clone, Copy)]
pub struct KnockState {
    /// Per-cylinder knock state (up to 8 cylinders)
    pub cylinders: [CylinderKnockState; 8],
    /// Global retard applied to all cylinders (degrees x10)
    pub global_retard_x10: u16,
    /// Last recovery update timestamp
    pub last_recovery_us: u32,
    /// Is knock detection currently active?
    pub active: bool,
    /// Total knock events across all cylinders
    pub total_knock_count: u32,
    /// Peak knock level seen (for diagnostics)
    pub peak_knock_level: u16,
    /// Current knock window active?
    pub in_window: bool,
    /// Current cylinder being monitored
    pub current_cylinder: u8,
}

impl KnockState {
    pub const fn new() -> Self {
        Self {
            cylinders: [CylinderKnockState::new(); 8],
            global_retard_x10: 0,
            last_recovery_us: 0,
            active: false,
            total_knock_count: 0,
            peak_knock_level: 0,
            in_window: false,
            current_cylinder: 0,
        }
    }

    /// Check if knock detection should be enabled
    pub fn should_enable(
        &self,
        config: &KnockConfig,
        rpm: u16,
        clt_c: i16,
    ) -> bool {
        if !config.enable {
            return false;
        }
        if rpm < config.min_rpm {
            return false;
        }
        if clt_c < config.min_clt_c {
            return false;
        }
        true
    }

    /// Set the current knock window state
    ///
    /// Call this based on crank angle to enable/disable knock listening.
    ///
    /// # Arguments
    /// * `in_window` - true if currently in the knock window
    /// * `cylinder` - current cylinder being monitored
    pub fn set_window(&mut self, in_window: bool, cylinder: u8) {
        self.in_window = in_window;
        self.current_cylinder = cylinder;
    }

    /// Process a knock sensor sample
    ///
    /// Call this during the knock window with the current sensor reading.
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder index (0-7)
    /// * `level` - Knock sensor reading (ADC or normalized)
    /// * `config` - Knock configuration
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if knock was detected
    pub fn process_sample(
        &mut self,
        cylinder: u8,
        level: u16,
        config: &KnockConfig,
        now_us: u32,
    ) -> bool {
        if !config.enable || cylinder >= config.max_cylinders {
            return false;
        }

        // Update peak level for diagnostics
        if level > self.peak_knock_level {
            self.peak_knock_level = level;
        }

        let cyl_idx = cylinder as usize;
        let cyl = &mut self.cylinders[cyl_idx];

        // Check if this sample exceeds threshold
        if level >= config.threshold {
            cyl.consecutive_knocks = cyl.consecutive_knocks.saturating_add(1);

            // Check debounce
            if cyl.consecutive_knocks >= config.debounce_count {
                // Confirmed knock!
                cyl.knock_count = cyl.knock_count.saturating_add(1);
                cyl.last_knock_us = now_us;
                self.total_knock_count = self.total_knock_count.saturating_add(1);

                // Apply retard
                cyl.retard_x10 = cyl.retard_x10.saturating_add(config.retard_step_x10);
                if cyl.retard_x10 > config.retard_max_x10 {
                    cyl.retard_x10 = config.retard_max_x10;
                }

                // Reset consecutive counter after applying retard
                cyl.consecutive_knocks = 0;
                return true;
            }
        } else {
            // No knock - reset consecutive counter
            cyl.consecutive_knocks = 0;
        }

        false
    }

    /// Get total retard for a cylinder (per-cylinder + global)
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder index (0-7)
    ///
    /// # Returns
    /// Total retard in degrees x10
    pub fn get_retard(&self, cylinder: u8) -> u16 {
        if cylinder >= 8 {
            return self.global_retard_x10;
        }

        let cyl_retard = self.cylinders[cylinder as usize].retard_x10;
        cyl_retard.saturating_add(self.global_retard_x10)
    }

    /// Get total retard for a cylinder in degrees (i16 for ignition calculation)
    ///
    /// Returns negative value since retard reduces timing.
    pub fn get_retard_degrees(&self, cylinder: u8) -> i16 {
        let retard_x10 = self.get_retard(cylinder);
        -((retard_x10 / 10) as i16)
    }

    /// Update timing recovery (call periodically)
    ///
    /// Gradually reduces retard when no knock is detected.
    ///
    /// # Arguments
    /// * `config` - Knock configuration
    /// * `now_us` - Current timestamp in microseconds
    pub fn update_recovery(&mut self, config: &KnockConfig, now_us: u32) {
        if !config.enable {
            return;
        }

        // Calculate elapsed time
        let elapsed_us = now_us.wrapping_sub(self.last_recovery_us);
        self.last_recovery_us = now_us;

        // Calculate recovery amount: rate * dt
        // recovery_rate_x10 is degrees x10 per second
        // dt is in microseconds
        let recovery_amount = (config.recovery_rate_x10 as u32 * elapsed_us) / 1_000_000;

        if recovery_amount == 0 {
            return;
        }

        let recovery = recovery_amount as u16;

        // Apply recovery to all cylinders
        for cyl in &mut self.cylinders[..config.max_cylinders as usize] {
            if cyl.retard_x10 > 0 {
                cyl.retard_x10 = cyl.retard_x10.saturating_sub(recovery);
            }
        }

        // Also recover global retard
        if self.global_retard_x10 > 0 {
            self.global_retard_x10 = self.global_retard_x10.saturating_sub(recovery);
        }
    }

    /// Apply global retard to all cylinders
    ///
    /// Use this for severe knock conditions.
    pub fn apply_global_retard(&mut self, retard_x10: u16, config: &KnockConfig) {
        self.global_retard_x10 = retard_x10.min(config.retard_max_x10);
    }

    /// Reset all knock state
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Reset knock counts (but keep retard)
    pub fn reset_counts(&mut self) {
        for cyl in &mut self.cylinders {
            cyl.knock_count = 0;
            cyl.consecutive_knocks = 0;
        }
        self.total_knock_count = 0;
        self.peak_knock_level = 0;
    }

    /// Get the cylinder with the highest knock count
    pub fn get_worst_cylinder(&self, config: &KnockConfig) -> u8 {
        let mut worst = 0u8;
        let mut max_count = 0u16;

        for i in 0..config.max_cylinders as usize {
            if self.cylinders[i].knock_count > max_count {
                max_count = self.cylinders[i].knock_count;
                worst = i as u8;
            }
        }

        worst
    }

    /// Check if any cylinder has significant retard
    pub fn has_retard(&self, config: &KnockConfig) -> bool {
        if self.global_retard_x10 > 0 {
            return true;
        }
        for i in 0..config.max_cylinders as usize {
            if self.cylinders[i].retard_x10 > 0 {
                return true;
            }
        }
        false
    }

    /// Simulate a knock event for testing
    #[cfg(any(test, feature = "simulation"))]
    pub fn inject_knock(&mut self, cylinder: u8, level: u16, config: &KnockConfig, now_us: u32) {
        // Bypass debounce and directly trigger knock
        if cylinder >= config.max_cylinders {
            return;
        }

        let cyl_idx = cylinder as usize;
        let cyl = &mut self.cylinders[cyl_idx];

        if level > self.peak_knock_level {
            self.peak_knock_level = level;
        }

        cyl.knock_count = cyl.knock_count.saturating_add(1);
        cyl.last_knock_us = now_us;
        self.total_knock_count = self.total_knock_count.saturating_add(1);

        cyl.retard_x10 = cyl.retard_x10.saturating_add(config.retard_step_x10);
        if cyl.retard_x10 > config.retard_max_x10 {
            cyl.retard_x10 = config.retard_max_x10;
        }
    }
}

impl Default for KnockState {
    fn default() -> Self {
        Self::new()
    }
}

/// Combined knock controller
#[derive(Debug, Clone, Copy)]
pub struct KnockController {
    pub state: KnockState,
    pub config: KnockConfig,
}

impl KnockController {
    pub const fn new() -> Self {
        Self {
            state: KnockState::new(),
            config: KnockConfig::DEFAULT,
        }
    }

    /// Process a knock sample and update state
    ///
    /// # Returns
    /// `true` if knock was detected
    pub fn process(
        &mut self,
        cylinder: u8,
        level: u16,
        rpm: u16,
        clt_c: i16,
        now_us: u32,
    ) -> bool {
        self.state.active = self.state.should_enable(&self.config, rpm, clt_c);

        if !self.state.active {
            return false;
        }

        self.state.process_sample(cylinder, level, &self.config, now_us)
    }

    /// Update recovery (call periodically)
    pub fn update_recovery(&mut self, now_us: u32) {
        self.state.update_recovery(&self.config, now_us);
    }

    /// Get retard for timing calculation
    pub fn get_retard_degrees(&self, cylinder: u8) -> i16 {
        if !self.config.enable {
            return 0;
        }
        self.state.get_retard_degrees(cylinder)
    }

    /// Reset all state
    pub fn reset(&mut self) {
        self.state.reset();
    }
}

impl Default for KnockController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_knock_config_default() {
        let config = KnockConfig::DEFAULT;
        assert!(config.enable);
        assert_eq!(config.max_cylinders, 4);
        assert_eq!(config.retard_step_x10, 20); // 2.0 degrees
        assert_eq!(config.retard_max_x10, 150); // 15.0 degrees
    }

    #[test]
    fn test_knock_state_new() {
        let state = KnockState::new();
        assert_eq!(state.global_retard_x10, 0);
        assert_eq!(state.total_knock_count, 0);
        assert!(!state.active);

        for cyl in &state.cylinders {
            assert_eq!(cyl.retard_x10, 0);
            assert_eq!(cyl.knock_count, 0);
        }
    }

    #[test]
    fn test_knock_detection_below_threshold() {
        let mut state = KnockState::new();
        let config = KnockConfig::DEFAULT;

        // Level below threshold should not trigger
        let detected = state.process_sample(0, 50, &config, 0);
        assert!(!detected);
        assert_eq!(state.cylinders[0].knock_count, 0);
        assert_eq!(state.cylinders[0].retard_x10, 0);
    }

    #[test]
    fn test_knock_detection_with_debounce() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            debounce_count: 2,
            ..KnockConfig::DEFAULT
        };

        // First knock - not enough for debounce
        let detected = state.process_sample(0, 150, &config, 0);
        assert!(!detected);
        assert_eq!(state.cylinders[0].consecutive_knocks, 1);

        // Second knock - should trigger
        let detected = state.process_sample(0, 150, &config, 1000);
        assert!(detected);
        assert_eq!(state.cylinders[0].knock_count, 1);
        assert!(state.cylinders[0].retard_x10 > 0);
    }

    #[test]
    fn test_knock_retard_application() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            debounce_count: 1, // Instant trigger
            retard_step_x10: 20, // 2.0 degrees
            ..KnockConfig::DEFAULT
        };

        // Trigger knock
        state.process_sample(0, 150, &config, 0);
        assert_eq!(state.cylinders[0].retard_x10, 20);

        // Another knock
        state.process_sample(0, 150, &config, 1000);
        assert_eq!(state.cylinders[0].retard_x10, 40);
    }

    #[test]
    fn test_knock_retard_max_clamping() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            debounce_count: 1,
            retard_step_x10: 50,  // 5.0 degrees
            retard_max_x10: 100,  // 10.0 degrees max
            ..KnockConfig::DEFAULT
        };

        // Trigger many knocks
        for i in 0..10 {
            state.process_sample(0, 150, &config, i * 1000);
        }

        // Should be clamped to max
        assert_eq!(state.cylinders[0].retard_x10, 100);
    }

    #[test]
    fn test_knock_recovery() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            debounce_count: 1,
            retard_step_x10: 50,    // 5.0 degrees
            recovery_rate_x10: 10,  // 1.0 degree per second
            ..KnockConfig::DEFAULT
        };

        // Trigger knock at t=0
        state.process_sample(0, 150, &config, 0);
        assert_eq!(state.cylinders[0].retard_x10, 50);

        // Set last_recovery_us to start recovery
        state.last_recovery_us = 0;

        // After 1 second, should recover 1 degree (10 x10)
        state.update_recovery(&config, 1_000_000);
        assert_eq!(state.cylinders[0].retard_x10, 40);

        // After 5 seconds total
        state.update_recovery(&config, 5_000_000);
        assert_eq!(state.cylinders[0].retard_x10, 0); // Fully recovered
    }

    #[test]
    fn test_knock_per_cylinder_tracking() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            debounce_count: 1,
            max_cylinders: 4,
            ..KnockConfig::DEFAULT
        };

        // Knock on cylinder 0
        state.process_sample(0, 150, &config, 0);
        // Knock on cylinder 2
        state.process_sample(2, 150, &config, 1000);

        assert_eq!(state.cylinders[0].knock_count, 1);
        assert_eq!(state.cylinders[1].knock_count, 0);
        assert_eq!(state.cylinders[2].knock_count, 1);
        assert_eq!(state.cylinders[3].knock_count, 0);

        assert!(state.cylinders[0].retard_x10 > 0);
        assert_eq!(state.cylinders[1].retard_x10, 0);
        assert!(state.cylinders[2].retard_x10 > 0);
    }

    #[test]
    fn test_knock_get_retard() {
        let mut state = KnockState::new();
        state.cylinders[0].retard_x10 = 30;
        state.global_retard_x10 = 10;

        // Per-cylinder + global
        assert_eq!(state.get_retard(0), 40);
        // Just global for other cylinders
        assert_eq!(state.get_retard(1), 10);
    }

    #[test]
    fn test_knock_get_retard_degrees() {
        let mut state = KnockState::new();
        state.cylinders[0].retard_x10 = 35; // 3.5 degrees

        let retard = state.get_retard_degrees(0);
        assert_eq!(retard, -3); // Negative for retard, truncated
    }

    #[test]
    fn test_knock_disabled_below_min_rpm() {
        let state = KnockState::new();
        let config = KnockConfig {
            min_rpm: 1500,
            ..KnockConfig::DEFAULT
        };

        assert!(!state.should_enable(&config, 1000, 80));
        assert!(state.should_enable(&config, 2000, 80));
    }

    #[test]
    fn test_knock_disabled_when_cold() {
        let state = KnockState::new();
        let config = KnockConfig {
            min_clt_c: 60,
            ..KnockConfig::DEFAULT
        };

        assert!(!state.should_enable(&config, 2000, 40));
        assert!(state.should_enable(&config, 2000, 70));
    }

    #[test]
    fn test_knock_reset() {
        let mut state = KnockState::new();
        let config = KnockConfig::DEFAULT;

        state.cylinders[0].retard_x10 = 50;
        state.cylinders[0].knock_count = 10;
        state.total_knock_count = 10;
        state.global_retard_x10 = 20;

        state.reset();

        assert_eq!(state.cylinders[0].retard_x10, 0);
        assert_eq!(state.cylinders[0].knock_count, 0);
        assert_eq!(state.total_knock_count, 0);
        assert_eq!(state.global_retard_x10, 0);
    }

    #[test]
    fn test_knock_worst_cylinder() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            max_cylinders: 4,
            ..KnockConfig::DEFAULT
        };

        state.cylinders[0].knock_count = 5;
        state.cylinders[1].knock_count = 10;
        state.cylinders[2].knock_count = 3;
        state.cylinders[3].knock_count = 8;

        assert_eq!(state.get_worst_cylinder(&config), 1);
    }

    #[test]
    fn test_knock_inject_simulation() {
        let mut state = KnockState::new();
        let config = KnockConfig::DEFAULT;

        state.inject_knock(0, 200, &config, 0);

        assert_eq!(state.cylinders[0].knock_count, 1);
        assert!(state.cylinders[0].retard_x10 > 0);
        assert_eq!(state.peak_knock_level, 200);
    }

    #[test]
    fn test_knock_controller_integration() {
        let mut controller = KnockController::new();
        controller.config.debounce_count = 1;

        // Cold engine - should not process
        let detected = controller.process(0, 150, 2000, 40, 0);
        assert!(!detected);
        assert!(!controller.state.active);

        // Warm engine - should process
        let detected = controller.process(0, 150, 2000, 70, 1000);
        assert!(detected);
        assert!(controller.state.active);

        let retard = controller.get_retard_degrees(0);
        assert!(retard < 0);
    }

    #[test]
    fn test_knock_debounce_reset_on_no_knock() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            debounce_count: 3,
            ..KnockConfig::DEFAULT
        };

        // Two knocks - not enough
        state.process_sample(0, 150, &config, 0);
        state.process_sample(0, 150, &config, 1000);
        assert_eq!(state.cylinders[0].consecutive_knocks, 2);

        // Below threshold - resets counter
        state.process_sample(0, 50, &config, 2000);
        assert_eq!(state.cylinders[0].consecutive_knocks, 0);

        // Need 3 consecutive again
        state.process_sample(0, 150, &config, 3000);
        assert_eq!(state.cylinders[0].consecutive_knocks, 1);
    }

    #[test]
    fn test_knock_global_retard() {
        let mut state = KnockState::new();
        let config = KnockConfig {
            retard_max_x10: 100,
            ..KnockConfig::DEFAULT
        };

        state.apply_global_retard(50, &config);
        assert_eq!(state.global_retard_x10, 50);

        // Check it's added to per-cylinder
        state.cylinders[0].retard_x10 = 20;
        assert_eq!(state.get_retard(0), 70);

        // Clamp to max
        state.apply_global_retard(200, &config);
        assert_eq!(state.global_retard_x10, 100);
    }
}
