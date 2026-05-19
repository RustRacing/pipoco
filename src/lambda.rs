//! Lambda (Air-Fuel Ratio) Closed-Loop Controller
//!
//! Implements a PI controller for closed-loop fuel control based on O2 sensor feedback.
//! Supports both narrowband (rich/lean) and wideband (exact AFR) sensors.
//!
//! ## Long-Term Fuel Trim (LTFT)
//!
//! LTFT learns from STFT corrections over time to improve base calibration.
//! Uses a 4x4 grid (RPM x Load) for coarse learning with slow update rates.

/// Configuration for the lambda controller
#[derive(Debug, Clone, Copy)]
pub struct LambdaConfig {
    /// Enable closed-loop control
    pub enable: bool,
    /// Target AFR x10 (e.g., 147 = 14.7:1 stoichiometric)
    pub target_afr_x10: u16,
    /// Proportional gain x100 (e.g., 50 = 0.50)
    pub kp_x100: u16,
    /// Integral gain x100 (e.g., 10 = 0.10)
    pub ki_x100: u16,
    /// Maximum authority (percent x10, e.g., 200 = 20%)
    pub authority_max_x10: i16,
    /// Narrowband O2 sensor threshold (millivolts)
    /// Above this = rich, below = lean (typically ~450mV)
    pub narrowband_threshold_mv: u16,
    /// Deadband around threshold (millivolts)
    /// No correction within threshold ± deadband
    pub deadband_mv: u16,
    /// Minimum coolant temperature for closed-loop (Celsius)
    pub min_clt_c: i16,
    /// Maximum TPS for closed-loop (percent)
    /// Above this, run open-loop (WOT enrichment)
    pub max_tps_percent: u8,
    /// Minimum RPM for closed-loop
    pub min_rpm: u16,
    /// Update interval (microseconds)
    pub update_interval_us: u32,
}

impl LambdaConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        target_afr_x10: 147,    // Stoichiometric
        kp_x100: 50,            // 0.50 proportional gain
        ki_x100: 10,            // 0.10 integral gain
        authority_max_x10: 200, // ±20%
        narrowband_threshold_mv: 450,
        deadband_mv: 20,
        min_clt_c: 60,       // 60°C minimum coolant temp
        max_tps_percent: 80, // Disable at WOT
        min_rpm: 1200,
        update_interval_us: 100_000, // 100ms (10Hz)
    };
}

impl Default for LambdaConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// O2 sensor type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum O2SensorType {
    /// Narrowband (0-1V, rich/lean only)
    #[default]
    Narrowband,
    /// Wideband (0-5V = 10-20 AFR typical)
    Wideband,
}

/// Lambda controller state
#[derive(Debug, Clone, Copy)]
pub struct LambdaState {
    /// Is closed-loop currently active?
    pub active: bool,
    /// Current short-term fuel trim x10 (-200 to +200 = -20% to +20%)
    pub stft_x10: i16,
    /// Integral accumulator (scaled)
    pub integral: i32,
    /// Last O2 sensor reading (millivolts)
    pub last_o2_mv: u16,
    /// Last calculated AFR x10 (for wideband)
    pub last_afr_x10: u16,
    /// Last update timestamp (microseconds)
    pub last_update_us: u32,
    /// Is sensor currently in deadband?
    pub in_deadband: bool,
    /// Reason for being disabled
    pub disable_reason: Option<DisableReason>,
    /// O2 sensor type in use
    pub sensor_type: O2SensorType,
}

/// Reason closed-loop is disabled
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DisableReason {
    /// Feature disabled in config
    ConfigDisabled,
    /// Engine too cold
    CoolantTooLow,
    /// At wide-open throttle
    WideOpenThrottle,
    /// RPM too low
    RpmTooLow,
    /// Engine not running / no O2 signal
    NoSignal,
    /// Manual override
    ManualDisable,
}

impl LambdaState {
    /// Create new lambda state
    pub const fn new() -> Self {
        Self {
            active: false,
            stft_x10: 0,
            integral: 0,
            last_o2_mv: 450, // Mid-point
            last_afr_x10: 147,
            last_update_us: 0,
            in_deadband: false,
            disable_reason: Some(DisableReason::NoSignal),
            sensor_type: O2SensorType::Narrowband,
        }
    }

    /// Update the PI controller with new O2 reading
    ///
    /// # Arguments
    /// * `o2_mv` - O2 sensor reading in millivolts
    /// * `clt_c` - Coolant temperature in Celsius
    /// * `tps_percent` - Throttle position (0-100%)
    /// * `rpm` - Engine RPM
    /// * `config` - Lambda configuration
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Short-term fuel trim x10 (-200 to +200 = -20% to +20%)
    pub fn update(
        &mut self,
        o2_mv: u16,
        clt_c: i16,
        tps_percent: u8,
        rpm: u16,
        config: &LambdaConfig,
        now_us: u32,
    ) -> i16 {
        self.last_o2_mv = o2_mv;

        // Check enable conditions
        if !config.enable {
            self.deactivate(DisableReason::ConfigDisabled);
            return 0;
        }

        if clt_c < config.min_clt_c {
            self.deactivate(DisableReason::CoolantTooLow);
            return 0;
        }

        if tps_percent > config.max_tps_percent {
            self.deactivate(DisableReason::WideOpenThrottle);
            return 0;
        }

        if rpm < config.min_rpm {
            self.deactivate(DisableReason::RpmTooLow);
            return 0;
        }

        // Check update interval
        let elapsed = now_us.wrapping_sub(self.last_update_us);
        if self.last_update_us != 0 && elapsed < config.update_interval_us {
            return self.stft_x10; // Return current value, don't update yet
        }

        self.last_update_us = now_us;
        self.active = true;
        self.disable_reason = None;

        // Calculate error based on sensor type
        let error = self.calculate_error(o2_mv, config);

        // Apply deadband
        if error.abs() <= config.deadband_mv as i32 {
            self.in_deadband = true;
            // In deadband, don't accumulate integral, but keep current correction
            return self.stft_x10;
        }
        self.in_deadband = false;

        // PI control
        // P term: error * Kp
        let p_term = (error * config.kp_x100 as i32) / 100;

        // I term: integral of error * Ki
        // Scale elapsed time to seconds for integral
        let dt_sec_x1000 = (elapsed / 1000) as i32; // ms
        self.integral += (error * config.ki_x100 as i32 * dt_sec_x1000) / 100_000;

        // Anti-windup: limit integral
        let max_integral = config.authority_max_x10 as i32 * 10;
        self.integral = self.integral.clamp(-max_integral, max_integral);

        // Calculate total correction
        let correction = p_term + (self.integral / 10);

        // Apply authority limits
        self.stft_x10 =
            (correction as i16).clamp(-config.authority_max_x10, config.authority_max_x10);

        self.stft_x10
    }

    /// Calculate error from O2 reading
    fn calculate_error(&mut self, o2_mv: u16, config: &LambdaConfig) -> i32 {
        match self.sensor_type {
            O2SensorType::Narrowband => {
                // Narrowband: simple rich/lean detection
                // Above threshold = rich (need to lean out, negative error)
                // Below threshold = lean (need to richen, positive error)
                let threshold = config.narrowband_threshold_mv as i32;
                let reading = o2_mv as i32;

                // Scale to make error magnitude reasonable
                // 100mV deviation = ~50 units of error
                (threshold - reading) / 2
            }
            O2SensorType::Wideband => {
                // Wideband: Linear 0-5V = 10-20 AFR typical
                // Convert mV to AFR x10
                // 0mV = 10.0 AFR (100), 5000mV = 20.0 AFR (200)
                let afr_x10 = 100 + ((o2_mv as u32 * 100) / 5000) as u16;
                self.last_afr_x10 = afr_x10;

                // Error = target - actual
                // Positive error = too lean, need more fuel
                let target = config.target_afr_x10 as i32;
                let actual = afr_x10 as i32;

                (target - actual) * 5 // Scale for similar magnitude to narrowband
            }
        }
    }

    /// Deactivate closed-loop with reason
    fn deactivate(&mut self, reason: DisableReason) {
        if self.active {
            // Don't immediately zero the integral - gradual decay
            self.integral = (self.integral * 9) / 10;
        }
        self.active = false;
        self.disable_reason = Some(reason);
        // Keep stft_x10 at current value for smooth transition
    }

    /// Check if closed-loop is currently active
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Get current short-term fuel trim
    pub fn get_stft(&self) -> i16 {
        self.stft_x10
    }

    /// Get current short-term fuel trim as a percentage x10
    pub fn get_stft_percent_x10(&self) -> i16 {
        self.stft_x10
    }

    /// Set sensor type
    pub fn set_sensor_type(&mut self, sensor_type: O2SensorType) {
        self.sensor_type = sensor_type;
    }

    /// Reset controller state
    pub fn reset(&mut self) {
        self.stft_x10 = 0;
        self.integral = 0;
        self.in_deadband = false;
    }

    /// Force disable closed-loop
    pub fn manual_disable(&mut self) {
        self.deactivate(DisableReason::ManualDisable);
    }

    /// Re-enable after manual disable
    pub fn manual_enable(&mut self) {
        if self.disable_reason == Some(DisableReason::ManualDisable) {
            self.disable_reason = None;
        }
    }
}

impl Default for LambdaState {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Long-Term Fuel Trim (LTFT)
// ============================================================================

/// LTFT constants
pub mod ltft_constants {
    /// Default RPM bins for 4x4 LTFT table
    pub const RPM_BINS: [u16; 4] = [1000, 2000, 3500, 5500];
    /// Default load bins (kPa x10) for 4x4 LTFT table
    pub const LOAD_BINS: [u16; 4] = [300, 600, 900, 1200];
    /// Maximum LTFT trim (±10%)
    pub const MAX_TRIM_X10: i16 = 100;
    /// Minimum samples before cell is considered learned
    pub const MIN_SAMPLES: u16 = 50;
    /// Default learning rate (0-255, where 255 = instant)
    pub const DEFAULT_LEARN_RATE: u8 = 4;
    /// STFT threshold for learning (only learn if STFT is stable)
    pub const STFT_THRESHOLD_X10: i16 = 30; // 3%
    /// Minimum time at steady-state before learning (microseconds)
    pub const STEADY_STATE_TIME_US: u32 = 2_000_000; // 2 seconds
    /// Maximum RPM deviation for steady-state detection
    pub const STEADY_STATE_RPM_DEV: u16 = 200;
    /// Maximum load deviation for steady-state detection (kPa x10)
    pub const STEADY_STATE_LOAD_DEV: u16 = 50;
    /// Key for persisting LTFT table
    pub const PERSIST_KEY: &[u8] = b"ltft";
}

/// Configuration for LTFT learning
#[derive(Debug, Clone, Copy)]
pub struct LtftConfig {
    /// Enable LTFT learning
    pub enable: bool,
    /// Learning rate (0-255, lower = slower learning)
    pub learn_rate: u8,
    /// Maximum trim authority (percent x10)
    pub max_trim_x10: i16,
    /// Minimum coolant temperature for learning (Celsius)
    pub min_clt_c: i16,
    /// STFT must be below this threshold to learn (percent x10)
    pub stft_threshold_x10: i16,
    /// Time at steady-state before learning (microseconds)
    pub steady_state_time_us: u32,
}

impl LtftConfig {
    pub const DEFAULT: Self = Self {
        enable: true,
        learn_rate: ltft_constants::DEFAULT_LEARN_RATE,
        max_trim_x10: ltft_constants::MAX_TRIM_X10,
        min_clt_c: 70, // Fully warmed up
        stft_threshold_x10: ltft_constants::STFT_THRESHOLD_X10,
        steady_state_time_us: ltft_constants::STEADY_STATE_TIME_US,
    };
}

impl Default for LtftConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Single cell in the LTFT table
#[derive(Debug, Clone, Copy, Default)]
pub struct LtftCell {
    /// Learned trim value (percent x10, -100 to +100 = ±10%)
    pub trim_x10: i16,
    /// Number of samples used for learning (confidence metric)
    pub sample_count: u16,
}

impl LtftCell {
    pub const fn new() -> Self {
        Self {
            trim_x10: 0,
            sample_count: 0,
        }
    }

    /// Check if this cell has enough samples to be considered learned
    pub fn is_learned(&self) -> bool {
        self.sample_count >= ltft_constants::MIN_SAMPLES
    }
}

/// 4x4 LTFT learning table
#[derive(Debug, Clone, Copy)]
pub struct LtftTable {
    /// 4x4 grid of cells [load_idx][rpm_idx]
    pub cells: [[LtftCell; 4]; 4],
    /// RPM bins
    pub rpm_bins: [u16; 4],
    /// Load bins (kPa x10)
    pub load_bins: [u16; 4],
}

impl LtftTable {
    pub const fn new() -> Self {
        Self {
            cells: [[LtftCell::new(); 4]; 4],
            rpm_bins: ltft_constants::RPM_BINS,
            load_bins: ltft_constants::LOAD_BINS,
        }
    }

    /// Find the bin index for a given value
    fn find_bin(value: u16, bins: &[u16; 4]) -> usize {
        for i in (0..4).rev() {
            if value >= bins[i] {
                return i;
            }
        }
        0
    }

    /// Learn from current STFT at the given operating point
    ///
    /// Uses exponential moving average for smooth learning.
    ///
    /// # Arguments
    /// * `rpm` - Current RPM
    /// * `load` - Current load (kPa x10)
    /// * `stft` - Current short-term fuel trim (percent x10)
    /// * `rate` - Learning rate (0-255)
    pub fn learn(&mut self, rpm: u16, load: u16, stft: i16, rate: u8, max_trim: i16) {
        let rpm_idx = Self::find_bin(rpm, &self.rpm_bins);
        let load_idx = Self::find_bin(load, &self.load_bins);
        let cell = &mut self.cells[load_idx][rpm_idx];

        // Exponential moving average: new = old + rate/256 * (stft - old)
        // This provides slow, stable learning
        let rate_i32 = rate as i32;
        let current = cell.trim_x10 as i32;
        let target = stft as i32;
        let delta = ((target - current) * rate_i32) / 256;

        let new_trim = (current + delta) as i16;
        cell.trim_x10 = new_trim.clamp(-max_trim, max_trim);
        cell.sample_count = cell.sample_count.saturating_add(1);
    }

    /// Lookup LTFT for current conditions
    ///
    /// Returns the learned trim value for the nearest bin.
    ///
    /// # Arguments
    /// * `rpm` - Current RPM
    /// * `load` - Current load (kPa x10)
    ///
    /// # Returns
    /// LTFT trim value (percent x10)
    pub fn lookup(&self, rpm: u16, load: u16) -> i16 {
        let rpm_idx = Self::find_bin(rpm, &self.rpm_bins);
        let load_idx = Self::find_bin(load, &self.load_bins);
        self.cells[load_idx][rpm_idx].trim_x10
    }

    /// Check if the cell for given conditions is learned
    pub fn is_cell_learned(&self, rpm: u16, load: u16) -> bool {
        let rpm_idx = Self::find_bin(rpm, &self.rpm_bins);
        let load_idx = Self::find_bin(load, &self.load_bins);
        self.cells[load_idx][rpm_idx].is_learned()
    }

    /// Reset all learning
    pub fn reset(&mut self) {
        for row in &mut self.cells {
            for cell in row {
                *cell = LtftCell::new();
            }
        }
    }

    /// Count how many cells have been learned
    pub fn learned_cell_count(&self) -> u8 {
        let mut count = 0u8;
        for row in &self.cells {
            for cell in row {
                if cell.is_learned() {
                    count = count.saturating_add(1);
                }
            }
        }
        count
    }

    /// Serialize to bytes for persistence (64 bytes)
    /// Format: 16 cells * 4 bytes (2 trim + 2 count) = 64 bytes
    pub fn to_bytes(&self) -> [u8; 64] {
        let mut out = [0u8; 64];
        let mut idx = 0;
        for row in &self.cells {
            for cell in row {
                let trim_bytes = cell.trim_x10.to_le_bytes();
                let count_bytes = cell.sample_count.to_le_bytes();
                out[idx] = trim_bytes[0];
                out[idx + 1] = trim_bytes[1];
                out[idx + 2] = count_bytes[0];
                out[idx + 3] = count_bytes[1];
                idx += 4;
            }
        }
        out
    }

    /// Deserialize from bytes
    pub fn from_bytes(data: &[u8; 64]) -> Self {
        let mut table = Self::new();
        let mut idx = 0;
        for row in &mut table.cells {
            for cell in row {
                cell.trim_x10 = i16::from_le_bytes([data[idx], data[idx + 1]]);
                cell.sample_count = u16::from_le_bytes([data[idx + 2], data[idx + 3]]);
                idx += 4;
            }
        }
        table
    }
}

impl Default for LtftTable {
    fn default() -> Self {
        Self::new()
    }
}

/// LTFT learning state machine
#[derive(Debug, Clone, Copy)]
pub struct LtftState {
    /// Is learning currently active?
    pub learning_active: bool,
    /// Last RPM reading (for steady-state detection)
    pub last_rpm: u16,
    /// Last load reading (for steady-state detection)
    pub last_load: u16,
    /// Time when steady-state began (microseconds)
    pub steady_since_us: u32,
    /// Is currently in steady-state?
    pub in_steady_state: bool,
    /// Last update timestamp
    pub last_update_us: u32,
    /// Total learning events
    pub learn_count: u32,
}

impl LtftState {
    pub const fn new() -> Self {
        Self {
            learning_active: false,
            last_rpm: 0,
            last_load: 0,
            steady_since_us: 0,
            in_steady_state: false,
            last_update_us: 0,
            learn_count: 0,
        }
    }

    /// Check if conditions allow learning and update state
    ///
    /// # Arguments
    /// * `rpm` - Current RPM
    /// * `load` - Current load (kPa x10)
    /// * `clt_c` - Coolant temperature (Celsius)
    /// * `stft` - Current STFT (percent x10)
    /// * `lambda_active` - Is lambda closed-loop active?
    /// * `config` - LTFT configuration
    /// * `now_us` - Current timestamp
    ///
    /// # Returns
    /// `true` if learning should occur this cycle
    #[allow(clippy::too_many_arguments)]
    pub fn should_learn(
        &mut self,
        rpm: u16,
        load: u16,
        clt_c: i16,
        stft: i16,
        lambda_active: bool,
        config: &LtftConfig,
        now_us: u32,
    ) -> bool {
        // Basic enable checks
        if !config.enable || !lambda_active {
            self.learning_active = false;
            return false;
        }

        // Temperature check
        if clt_c < config.min_clt_c {
            self.learning_active = false;
            self.in_steady_state = false;
            return false;
        }

        // STFT stability check - don't learn during large corrections
        if stft.abs() > config.stft_threshold_x10 {
            // Large STFT - we should learn, but reset steady-state timer
            self.in_steady_state = false;
        }

        // Steady-state detection
        let rpm_stable = (rpm as i32 - self.last_rpm as i32).abs()
            <= ltft_constants::STEADY_STATE_RPM_DEV as i32;
        let load_stable = (load as i32 - self.last_load as i32).abs()
            <= ltft_constants::STEADY_STATE_LOAD_DEV as i32;

        if rpm_stable && load_stable {
            if !self.in_steady_state {
                self.steady_since_us = now_us;
                self.in_steady_state = true;
            }
        } else {
            self.in_steady_state = false;
        }

        self.last_rpm = rpm;
        self.last_load = load;
        self.last_update_us = now_us;

        // Check if we've been in steady-state long enough
        if self.in_steady_state {
            let elapsed = now_us.wrapping_sub(self.steady_since_us);
            if elapsed >= config.steady_state_time_us {
                self.learning_active = true;
                return true;
            }
        }

        self.learning_active = false;
        false
    }

    /// Record a learning event
    pub fn record_learn(&mut self) {
        self.learn_count = self.learn_count.saturating_add(1);
    }

    /// Reset learning state
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for LtftState {
    fn default() -> Self {
        Self::new()
    }
}

/// Combined LTFT manager
#[derive(Debug, Clone, Copy)]
pub struct LtftManager {
    pub table: LtftTable,
    pub state: LtftState,
    pub config: LtftConfig,
}

impl LtftManager {
    pub const fn new() -> Self {
        Self {
            table: LtftTable::new(),
            state: LtftState::new(),
            config: LtftConfig::DEFAULT,
        }
    }

    /// Update LTFT learning with current conditions
    ///
    /// Call this periodically (e.g., 10Hz) during normal operation.
    ///
    /// # Returns
    /// Current LTFT value for the operating point (percent x10)
    pub fn update(
        &mut self,
        rpm: u16,
        load: u16,
        clt_c: i16,
        stft: i16,
        lambda_active: bool,
        now_us: u32,
    ) -> i16 {
        // Check if we should learn
        let should_learn =
            self.state
                .should_learn(rpm, load, clt_c, stft, lambda_active, &self.config, now_us);

        if should_learn {
            self.table.learn(
                rpm,
                load,
                stft,
                self.config.learn_rate,
                self.config.max_trim_x10,
            );
            self.state.record_learn();
        }

        // Always return the current LTFT lookup
        self.table.lookup(rpm, load)
    }

    /// Get total fuel trim (STFT + LTFT)
    ///
    /// # Arguments
    /// * `stft` - Current short-term fuel trim (percent x10)
    /// * `rpm` - Current RPM
    /// * `load` - Current load (kPa x10)
    ///
    /// # Returns
    /// Combined fuel trim (percent x10), clamped to authority limits
    pub fn get_total_trim(&self, stft: i16, rpm: u16, load: u16) -> i16 {
        let ltft = self.table.lookup(rpm, load);
        // Combined trim, clamped to reasonable range
        (stft + ltft).clamp(-200, 200) // ±20% max combined
    }

    /// Reset all learning
    pub fn reset(&mut self) {
        self.table.reset();
        self.state.reset();
    }

    /// Load table from persistence
    pub fn load_from_bytes(&mut self, data: &[u8; 64]) {
        self.table = LtftTable::from_bytes(data);
    }

    /// Save table for persistence
    pub fn save_to_bytes(&self) -> [u8; 64] {
        self.table.to_bytes()
    }

    /// Check if any learning has occurred
    pub fn has_learned(&self) -> bool {
        self.table.learned_cell_count() > 0
    }
}

impl Default for LtftManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn running_conditions() -> (i16, u8, u16) {
        // CLT, TPS, RPM for normal running
        (70, 30, 2500)
    }

    #[test]
    fn test_lambda_disabled_when_config_disabled() {
        let mut state = LambdaState::new();
        let config = LambdaConfig {
            enable: false,
            ..LambdaConfig::DEFAULT
        };

        let (clt, tps, rpm) = running_conditions();
        let stft = state.update(450, clt, tps, rpm, &config, 0);

        assert_eq!(stft, 0);
        assert!(!state.is_active());
        assert_eq!(state.disable_reason, Some(DisableReason::ConfigDisabled));
    }

    #[test]
    fn test_lambda_disabled_when_cold() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;

        let stft = state.update(450, 40, 30, 2500, &config, 0); // 40°C < 60°C min

        assert_eq!(stft, 0);
        assert!(!state.is_active());
        assert_eq!(state.disable_reason, Some(DisableReason::CoolantTooLow));
    }

    #[test]
    fn test_lambda_disabled_at_wot() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;

        let stft = state.update(450, 70, 90, 2500, &config, 0); // 90% > 80% max

        assert_eq!(stft, 0);
        assert!(!state.is_active());
        assert_eq!(state.disable_reason, Some(DisableReason::WideOpenThrottle));
    }

    #[test]
    fn test_lambda_disabled_at_low_rpm() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;

        let stft = state.update(450, 70, 30, 800, &config, 0); // 800 < 1200 min

        assert_eq!(stft, 0);
        assert!(!state.is_active());
        assert_eq!(state.disable_reason, Some(DisableReason::RpmTooLow));
    }

    #[test]
    fn test_lambda_activates_in_normal_conditions() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        state.update(450, clt, tps, rpm, &config, 0);

        assert!(state.is_active());
        assert_eq!(state.disable_reason, None);
    }

    #[test]
    fn test_lambda_leans_on_rich_signal() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // Rich signal (high voltage, above 450mV threshold)
        let stft = state.update(800, clt, tps, rpm, &config, 0);

        // Should lean out (negative STFT)
        assert!(stft < 0);
    }

    #[test]
    fn test_lambda_richens_on_lean_signal() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // Lean signal (low voltage, below 450mV threshold)
        let stft = state.update(100, clt, tps, rpm, &config, 0);

        // Should richen (positive STFT)
        assert!(stft > 0);
    }

    #[test]
    fn test_lambda_deadband() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // Exactly at threshold - within deadband
        state.update(450, clt, tps, rpm, &config, 0);

        assert!(state.in_deadband);
    }

    #[test]
    fn test_lambda_authority_limits() {
        let mut state = LambdaState::new();
        let config = LambdaConfig {
            authority_max_x10: 100, // ±10%
            ..LambdaConfig::DEFAULT
        };
        let (clt, tps, rpm) = running_conditions();

        // Very lean signal - should hit authority limit
        // Run multiple updates to accumulate integral
        for i in 0..20 {
            state.update(100, clt, tps, rpm, &config, i * 200_000);
        }

        // Should be clamped to +10%
        assert!(state.stft_x10 <= 100);
        assert!(state.stft_x10 >= -100);
    }

    #[test]
    fn test_lambda_integral_accumulates() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // Persistent lean condition
        state.update(200, clt, tps, rpm, &config, 0);
        let stft1 = state.stft_x10;

        state.update(200, clt, tps, rpm, &config, 200_000);
        let stft2 = state.stft_x10;

        // Integral should increase correction over time
        assert!(stft2 >= stft1);
    }

    #[test]
    fn test_lambda_reset() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // Build up some correction
        for i in 0..5 {
            state.update(200, clt, tps, rpm, &config, i * 200_000);
        }
        assert!(state.stft_x10 > 0);
        assert!(state.integral > 0);

        // Reset
        state.reset();

        assert_eq!(state.stft_x10, 0);
        assert_eq!(state.integral, 0);
    }

    #[test]
    fn test_lambda_wideband_sensor() {
        let mut state = LambdaState::new();
        state.set_sensor_type(O2SensorType::Wideband);
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // 2500mV = 15.0 AFR (lean for stoich target)
        state.update(2500, clt, tps, rpm, &config, 0);

        // Should be calculating AFR
        assert!(state.last_afr_x10 > 100);
    }

    #[test]
    fn test_lambda_respects_update_interval() {
        let mut state = LambdaState::new();
        let config = LambdaConfig::DEFAULT;
        let (clt, tps, rpm) = running_conditions();

        // First update
        state.update(200, clt, tps, rpm, &config, 0);
        let stft1 = state.stft_x10;

        // Update too soon - should return same value
        let stft2 = state.update(200, clt, tps, rpm, &config, 50_000); // 50ms < 100ms interval

        assert_eq!(stft1, stft2);
    }

    // ========================================================================
    // LTFT Tests
    // ========================================================================

    #[test]
    fn test_ltft_cell_new() {
        let cell = LtftCell::new();
        assert_eq!(cell.trim_x10, 0);
        assert_eq!(cell.sample_count, 0);
        assert!(!cell.is_learned());
    }

    #[test]
    fn test_ltft_cell_is_learned() {
        let mut cell = LtftCell::new();
        assert!(!cell.is_learned());

        cell.sample_count = ltft_constants::MIN_SAMPLES - 1;
        assert!(!cell.is_learned());

        cell.sample_count = ltft_constants::MIN_SAMPLES;
        assert!(cell.is_learned());
    }

    #[test]
    fn test_ltft_table_find_bin() {
        // RPM bins: [1000, 2000, 3500, 5500]
        assert_eq!(LtftTable::find_bin(500, &ltft_constants::RPM_BINS), 0);
        assert_eq!(LtftTable::find_bin(1000, &ltft_constants::RPM_BINS), 0);
        assert_eq!(LtftTable::find_bin(1500, &ltft_constants::RPM_BINS), 0);
        assert_eq!(LtftTable::find_bin(2000, &ltft_constants::RPM_BINS), 1);
        assert_eq!(LtftTable::find_bin(3000, &ltft_constants::RPM_BINS), 1);
        assert_eq!(LtftTable::find_bin(3500, &ltft_constants::RPM_BINS), 2);
        assert_eq!(LtftTable::find_bin(5000, &ltft_constants::RPM_BINS), 2);
        assert_eq!(LtftTable::find_bin(5500, &ltft_constants::RPM_BINS), 3);
        assert_eq!(LtftTable::find_bin(7000, &ltft_constants::RPM_BINS), 3);
    }

    #[test]
    fn test_ltft_table_learn_basic() {
        let mut table = LtftTable::new();

        // Learn with positive STFT (lean condition)
        table.learn(2500, 600, 50, 64, 100); // STFT = 5%

        // Should have learned something
        assert!(table.cells[1][1].trim_x10 > 0);
        assert_eq!(table.cells[1][1].sample_count, 1);
    }

    #[test]
    fn test_ltft_table_learn_convergence() {
        let mut table = LtftTable::new();
        let rate = 64u8; // Faster rate for test
        let stft = 50i16; // Target 5% LTFT

        // Learn many times
        for _ in 0..100 {
            table.learn(2500, 600, stft, rate, 100);
        }

        // Should have converged close to STFT
        let learned = table.cells[1][1].trim_x10;
        assert!(
            learned > 40 && learned < 60,
            "Expected ~50, got {}",
            learned
        );
    }

    #[test]
    fn test_ltft_table_learn_clamping() {
        let mut table = LtftTable::new();

        // Try to learn extreme value
        for _ in 0..200 {
            table.learn(2500, 600, 500, 64, 100); // Way above max
        }

        // Should be clamped to max
        assert!(table.cells[1][1].trim_x10 <= 100);
    }

    #[test]
    fn test_ltft_table_lookup() {
        let mut table = LtftTable::new();
        // RPM bins: [1000, 2000, 3500, 5500], Load bins: [300, 600, 900, 1200]
        // 2500 RPM -> bin 1, 700 kPa x10 -> bin 1
        table.cells[1][1].trim_x10 = 35;

        // Lookup should return the value for the correct bin
        let ltft = table.lookup(2500, 700);
        assert_eq!(ltft, 35);
    }

    #[test]
    fn test_ltft_table_reset() {
        let mut table = LtftTable::new();
        table.learn(2500, 600, 50, 64, 100);
        assert!(table.cells[1][1].sample_count > 0);

        table.reset();

        // All cells should be reset
        for row in &table.cells {
            for cell in row {
                assert_eq!(cell.trim_x10, 0);
                assert_eq!(cell.sample_count, 0);
            }
        }
    }

    #[test]
    fn test_ltft_table_serialization() {
        let mut table = LtftTable::new();
        table.cells[0][0].trim_x10 = 25;
        table.cells[0][0].sample_count = 100;
        table.cells[1][2].trim_x10 = -15;
        table.cells[1][2].sample_count = 50;

        let bytes = table.to_bytes();
        let restored = LtftTable::from_bytes(&bytes);

        assert_eq!(restored.cells[0][0].trim_x10, 25);
        assert_eq!(restored.cells[0][0].sample_count, 100);
        assert_eq!(restored.cells[1][2].trim_x10, -15);
        assert_eq!(restored.cells[1][2].sample_count, 50);
    }

    #[test]
    fn test_ltft_table_learned_cell_count() {
        let mut table = LtftTable::new();
        assert_eq!(table.learned_cell_count(), 0);

        table.cells[0][0].sample_count = ltft_constants::MIN_SAMPLES;
        assert_eq!(table.learned_cell_count(), 1);

        table.cells[1][1].sample_count = ltft_constants::MIN_SAMPLES;
        table.cells[2][2].sample_count = ltft_constants::MIN_SAMPLES;
        assert_eq!(table.learned_cell_count(), 3);
    }

    #[test]
    fn test_ltft_state_disabled_when_config_off() {
        let mut state = LtftState::new();
        let config = LtftConfig {
            enable: false,
            ..LtftConfig::DEFAULT
        };

        let should = state.should_learn(2500, 600, 80, 10, true, &config, 0);
        assert!(!should);
        assert!(!state.learning_active);
    }

    #[test]
    fn test_ltft_state_disabled_when_cold() {
        let mut state = LtftState::new();
        let config = LtftConfig::DEFAULT;

        // Cold engine (below 70°C)
        let should = state.should_learn(2500, 600, 50, 10, true, &config, 0);
        assert!(!should);
    }

    #[test]
    fn test_ltft_state_disabled_when_lambda_inactive() {
        let mut state = LtftState::new();
        let config = LtftConfig::DEFAULT;

        let should = state.should_learn(2500, 600, 80, 10, false, &config, 0);
        assert!(!should);
    }

    #[test]
    fn test_ltft_state_steady_state_detection() {
        let mut state = LtftState::new();
        let config = LtftConfig::DEFAULT;

        // First update - not in steady state yet
        state.should_learn(2500, 600, 80, 10, true, &config, 0);
        assert!(!state.learning_active);

        // Same conditions - now entering steady state
        state.should_learn(2500, 600, 80, 10, true, &config, 1_000_000);
        assert!(state.in_steady_state);
        assert!(!state.learning_active); // Not enough time yet

        // After steady state time elapsed
        let should = state.should_learn(2500, 600, 80, 10, true, &config, 3_000_000);
        assert!(should);
        assert!(state.learning_active);
    }

    #[test]
    fn test_ltft_state_steady_state_broken_by_rpm_change() {
        let mut state = LtftState::new();
        let config = LtftConfig::DEFAULT;

        // Enter steady state
        state.should_learn(2500, 600, 80, 10, true, &config, 0);
        state.should_learn(2500, 600, 80, 10, true, &config, 1_000_000);
        assert!(state.in_steady_state);

        // Large RPM change breaks steady state
        state.should_learn(3000, 600, 80, 10, true, &config, 1_500_000);
        assert!(!state.in_steady_state);
    }

    #[test]
    fn test_ltft_manager_update() {
        let mut manager = LtftManager::new();
        manager.config.learn_rate = 64; // Faster for test
        manager.config.steady_state_time_us = 0; // Instant for test

        // Update with STFT
        let ltft = manager.update(2500, 600, 80, 20, true, 1_000_000);
        assert_eq!(ltft, 0); // First update, no learning yet

        // Second update should learn
        manager.update(2500, 600, 80, 20, true, 2_000_000);
        let ltft = manager.table.lookup(2500, 600);
        assert!(ltft > 0);
    }

    #[test]
    fn test_ltft_manager_total_trim() {
        let mut manager = LtftManager::new();
        manager.table.cells[1][1].trim_x10 = 30; // 3% LTFT

        let stft = 20i16; // 2% STFT
        let total = manager.get_total_trim(stft, 2500, 600);

        assert_eq!(total, 50); // 5% total
    }

    #[test]
    fn test_ltft_manager_total_trim_clamping() {
        let mut manager = LtftManager::new();
        manager.table.cells[1][1].trim_x10 = 100; // 10% LTFT

        let stft = 150i16; // 15% STFT
        let total = manager.get_total_trim(stft, 2500, 600);

        // Should be clamped to 20%
        assert_eq!(total, 200);
    }

    #[test]
    fn test_ltft_manager_reset() {
        let mut manager = LtftManager::new();
        manager.config.steady_state_time_us = 0;
        manager.update(2500, 600, 80, 20, true, 1_000_000);
        manager.update(2500, 600, 80, 20, true, 2_000_000);
        assert!(manager.table.cells[1][1].sample_count > 0);

        manager.reset();

        assert_eq!(manager.table.cells[1][1].sample_count, 0);
        assert_eq!(manager.table.cells[1][1].trim_x10, 0);
        assert_eq!(manager.state.learn_count, 0);
    }

    #[test]
    fn test_ltft_manager_persistence() {
        let mut manager = LtftManager::new();
        manager.table.cells[0][0].trim_x10 = 45;
        manager.table.cells[0][0].sample_count = 100;

        let bytes = manager.save_to_bytes();

        let mut manager2 = LtftManager::new();
        manager2.load_from_bytes(&bytes);

        assert_eq!(manager2.table.cells[0][0].trim_x10, 45);
        assert_eq!(manager2.table.cells[0][0].sample_count, 100);
    }
}
