use super::config::{ltft_constants, LtftConfig};

/// Single cell in the LTFT table.
#[derive(Debug, Clone, Copy, Default)]
pub struct LtftCell {
    /// Learned trim value (percent x10, -100 to +100 = ±10%).
    pub trim_x10: i16,
    /// Number of samples used for learning (confidence metric).
    pub sample_count: u16,
}

impl LtftCell {
    pub const fn new() -> Self {
        Self {
            trim_x10: 0,
            sample_count: 0,
        }
    }

    /// Check if this cell has enough samples to be considered learned.
    pub fn is_learned(&self) -> bool {
        self.sample_count >= ltft_constants::MIN_SAMPLES
    }
}

/// 4x4 LTFT learning table.
#[derive(Debug, Clone, Copy)]
pub struct LtftTable {
    /// 4x4 grid of cells [load_idx][rpm_idx].
    pub cells: [[LtftCell; 4]; 4],
    /// RPM bins.
    pub rpm_bins: [u16; 4],
    /// Load bins (kPa x10).
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

    /// Find the bin index for a given value.
    pub(crate) fn find_bin(value: u16, bins: &[u16; 4]) -> usize {
        for i in (0..4).rev() {
            if value >= bins[i] {
                return i;
            }
        }
        0
    }

    /// Learn from current STFT at the given operating point.
    ///
    /// Uses exponential moving average for smooth learning.
    ///
    /// # Arguments
    /// * `rpm` - Current RPM.
    /// * `load` - Current load (kPa x10).
    /// * `stft` - Current short-term fuel trim (percent x10).
    /// * `rate` - Learning rate (0-255).
    pub fn learn(&mut self, rpm: u16, load: u16, stft: i16, rate: u8, max_trim: i16) {
        let rpm_idx = Self::find_bin(rpm, &self.rpm_bins);
        let load_idx = Self::find_bin(load, &self.load_bins);
        let cell = &mut self.cells[load_idx][rpm_idx];

        // Exponential moving average: new = old + rate/256 * (stft - old)
        // This provides slow, stable learning.
        let rate_i32 = rate as i32;
        let current = cell.trim_x10 as i32;
        let target = stft as i32;
        let delta = ((target - current) * rate_i32) / 256;

        let new_trim = (current + delta) as i16;
        cell.trim_x10 = new_trim.clamp(-max_trim, max_trim);
        cell.sample_count = cell.sample_count.saturating_add(1);
    }

    /// Lookup LTFT for current conditions.
    ///
    /// Returns the learned trim value for the nearest bin.
    ///
    /// # Arguments
    /// * `rpm` - Current RPM.
    /// * `load` - Current load (kPa x10).
    ///
    /// # Returns
    /// LTFT trim value (percent x10).
    pub fn lookup(&self, rpm: u16, load: u16) -> i16 {
        let rpm_idx = Self::find_bin(rpm, &self.rpm_bins);
        let load_idx = Self::find_bin(load, &self.load_bins);
        self.cells[load_idx][rpm_idx].trim_x10
    }

    /// Check if the cell for given conditions is learned.
    pub fn is_cell_learned(&self, rpm: u16, load: u16) -> bool {
        let rpm_idx = Self::find_bin(rpm, &self.rpm_bins);
        let load_idx = Self::find_bin(load, &self.load_bins);
        self.cells[load_idx][rpm_idx].is_learned()
    }

    /// Reset all learning.
    pub fn reset(&mut self) {
        for row in &mut self.cells {
            for cell in row {
                *cell = LtftCell::new();
            }
        }
    }

    /// Count how many cells have been learned.
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

    /// Serialize to bytes for persistence (64 bytes).
    /// Format: 16 cells * 4 bytes (2 trim + 2 count) = 64 bytes.
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

    /// Deserialize from bytes.
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

/// LTFT learning state machine.
#[derive(Debug, Clone, Copy)]
pub struct LtftState {
    /// Is learning currently active?
    pub learning_active: bool,
    /// Last RPM reading (for steady-state detection).
    pub last_rpm: u16,
    /// Last load reading (for steady-state detection).
    pub last_load: u16,
    /// Time when steady-state began (microseconds).
    pub steady_since_us: u32,
    /// Is currently in steady-state?
    pub in_steady_state: bool,
    /// Last update timestamp.
    pub last_update_us: u32,
    /// Total learning events.
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

    /// Check if conditions allow learning and update state.
    ///
    /// # Arguments
    /// * `rpm` - Current RPM.
    /// * `load` - Current load (kPa x10).
    /// * `clt_c` - Coolant temperature (Celsius).
    /// * `stft` - Current STFT (percent x10).
    /// * `lambda_active` - Is lambda closed-loop active?
    /// * `config` - LTFT configuration.
    /// * `now_us` - Current timestamp.
    ///
    /// # Returns
    /// `true` if learning should occur this cycle.
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
        // Basic enable checks.
        if !config.enable || !lambda_active {
            self.learning_active = false;
            return false;
        }

        // Temperature check.
        if clt_c < config.min_clt_c {
            self.learning_active = false;
            self.in_steady_state = false;
            return false;
        }

        // STFT stability check - don't learn during large corrections.
        if stft.abs() > config.stft_threshold_x10 {
            // Large STFT - we should learn, but reset steady-state timer.
            self.in_steady_state = false;
        }

        // Steady-state detection.
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

        // Check if we've been in steady-state long enough.
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

    /// Record a learning event.
    pub fn record_learn(&mut self) {
        self.learn_count = self.learn_count.saturating_add(1);
    }

    /// Reset learning state.
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for LtftState {
    fn default() -> Self {
        Self::new()
    }
}

/// Combined LTFT manager.
#[derive(Debug, Clone, Copy)]
pub struct LtftManager {
    /// Learned correction table.
    pub table: LtftTable,
    /// Current learning state.
    pub state: LtftState,
    /// Runtime configuration.
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

    /// Update LTFT learning with current conditions.
    ///
    /// Call this periodically (e.g., 10Hz) during normal operation.
    ///
    /// # Returns
    /// Current LTFT value for the operating point (percent x10).
    pub fn update(
        &mut self,
        rpm: u16,
        load: u16,
        clt_c: i16,
        stft: i16,
        lambda_active: bool,
        now_us: u32,
    ) -> i16 {
        // Check if we should learn.
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

        // Always return the current LTFT lookup.
        self.table.lookup(rpm, load)
    }

    /// Get total fuel trim (STFT + LTFT).
    ///
    /// # Arguments
    /// * `stft` - Current short-term fuel trim (percent x10).
    /// * `rpm` - Current RPM.
    /// * `load` - Current load (kPa x10).
    ///
    /// # Returns
    /// Combined fuel trim (percent x10), clamped to authority limits.
    pub fn get_total_trim(&self, stft: i16, rpm: u16, load: u16) -> i16 {
        let ltft = self.table.lookup(rpm, load);
        // Combined trim, clamped to reasonable range.
        (stft + ltft).clamp(-200, 200) // ±20% max combined.
    }

    /// Reset all learning.
    pub fn reset(&mut self) {
        self.table.reset();
        self.state.reset();
    }

    /// Load table from persistence.
    pub fn load_from_bytes(&mut self, data: &[u8; 64]) {
        self.table = LtftTable::from_bytes(data);
    }

    /// Save table for persistence.
    pub fn save_to_bytes(&self) -> [u8; 64] {
        self.table.to_bytes()
    }

    /// Check if any learning has occurred.
    pub fn has_learned(&self) -> bool {
        self.table.learned_cell_count() > 0
    }
}

impl Default for LtftManager {
    fn default() -> Self {
        Self::new()
    }
}
