//! Safety Features for Engine Protection
//!
//! This module implements critical safety features to prevent engine damage
//! and handle fault conditions gracefully.
//!
//! # Features
//!
//! 1. **Flood Clear Mode** - Prevents flooding during cranking
//!    - Activated when cranking with WOT (Wide Open Throttle)
//!    - Cuts fuel delivery to clear flooded cylinders
//!    - Allows air flow to dry out spark plugs
//!
//! 2. **Sync Loss Management** - ESD-resistant shutdown
//!    - Detects loss of trigger sync
//!    - Attempts recovery before shutdown (ESD protection)
//!    - Tracks sync loss history to distinguish ESD from real failures
//!    - Only shuts down after multiple failures in short window
//!
//! # Safety Philosophy
//!
//! **Flood Clear**: A flooded engine won't start and may damage spark plugs.
//! Allowing WOT during cranking signals "clear the flood" to the ECU.
//!
//! **ESD Protection**: Electrostatic discharge can cause brief signal glitches.
//! We must distinguish between:
//! - **Transient ESD**: Brief glitch, recover immediately
//! - **Real Failure**: Persistent loss, shut down for safety

use crate::constants::safety::*;

/// Flood clear state
#[derive(Debug, Clone, Copy)]
pub struct FloodClearState {
    /// Is flood clear mode currently active?
    pub active: bool,

    /// How many engine cycles flood clear has been active
    pub active_cycles: u16,
}

impl FloodClearState {
    /// Create new flood clear state (inactive)
    pub const fn new() -> Self {
        Self {
            active: false,
            active_cycles: 0,
        }
    }

    /// Reset flood clear state
    pub fn reset(&mut self) {
        self.active = false;
        self.active_cycles = 0;
    }
}

impl Default for FloodClearState {
    fn default() -> Self {
        Self::new()
    }
}

/// Sync loss recovery tracking
///
/// Tracks sync loss events to distinguish ESD glitches from real failures.
/// ESD events are typically isolated, real failures are persistent.
#[derive(Debug, Clone, Copy)]
pub struct SyncLossTracker {
    /// Number of sync loss events in current window
    loss_count: u8,

    /// Timestamp of first sync loss in current window (microseconds)
    window_start_us: u32,

    /// Is engine currently shut down due to sync loss?
    pub shutdown: bool,

    /// Total number of sync losses (for diagnostics)
    pub total_losses: u16,

    /// Total number of successful recoveries (for diagnostics)
    pub successful_recoveries: u16,
}

impl SyncLossTracker {
    /// Create new sync loss tracker
    pub const fn new() -> Self {
        Self {
            loss_count: 0,
            window_start_us: 0,
            shutdown: false,
            total_losses: 0,
            successful_recoveries: 0,
        }
    }

    /// Record a sync loss event
    ///
    /// # Arguments
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if engine should shut down, `false` if should attempt recovery
    pub fn record_sync_loss(&mut self, current_time_us: u32) -> bool {
        self.total_losses = self.total_losses.saturating_add(1);

        // Check if we're in a new window
        let time_since_window_start = current_time_us.wrapping_sub(self.window_start_us);

        if self.loss_count == 0 || time_since_window_start > SYNC_RECOVERY_WINDOW_US {
            // Start new window
            self.window_start_us = current_time_us;
            self.loss_count = 1;
            return false; // First loss in window, attempt recovery
        }

        // We're within the same window
        self.loss_count = self.loss_count.saturating_add(1);

        if self.loss_count >= SYNC_RECOVERY_ATTEMPTS {
            // Too many losses in short window - shut down
            self.shutdown = true;
            true
        } else {
            // Still have recovery attempts left
            false
        }
    }

    /// Record successful sync recovery
    ///
    /// Call this when sync is re-established after a loss.
    /// This helps distinguish ESD (recovers quickly) from real failures.
    pub fn record_recovery(&mut self) {
        self.successful_recoveries = self.successful_recoveries.saturating_add(1);

        // Don't reset loss_count immediately - we're still in the window
        // This allows us to catch rapid repeated losses (not just ESD)
    }

    /// Reset tracker after successful long-term operation
    ///
    /// Call this periodically (e.g., every 10 seconds of good sync)
    /// to clear the loss count and allow fresh recovery attempts.
    pub fn reset_window(&mut self) {
        self.loss_count = 0;
        self.window_start_us = 0;
    }

    /// Check if engine is shut down
    pub fn is_shutdown(&self) -> bool {
        self.shutdown
    }

    /// Manually reset shutdown state
    ///
    /// Should only be called by explicit user action (e.g., key cycle).
    /// NOT for automatic recovery.
    pub fn clear_shutdown(&mut self) {
        self.shutdown = false;
        self.loss_count = 0;
        self.window_start_us = 0;
    }
}

impl Default for SyncLossTracker {
    fn default() -> Self {
        Self::new()
    }
}

/// Check if flood clear mode should be active
///
/// Flood clear is activated when:
/// 1. Engine is cranking (low RPM)
/// 2. Throttle is at WOT (Wide Open Throttle)
///
/// # Arguments
/// * `rpm` - Current engine RPM
/// * `tps_percent` - Throttle position (0-100%)
/// * `state` - Current flood clear state (will be updated)
///
/// # Returns
/// `true` if flood clear is active (cut fuel), `false` otherwise
pub fn update_flood_clear(rpm: u16, tps_percent: u8, state: &mut FloodClearState) -> bool {
    let is_cranking = rpm < CRANKING_RPM_THRESHOLD;
    let is_wot = tps_percent >= WOT_TPS_THRESHOLD;

    if is_cranking && is_wot {
        // Activate flood clear
        state.active = true;
        state.active_cycles = state.active_cycles.saturating_add(1);
        true
    } else {
        // Deactivate flood clear
        if state.active {
            state.reset();
        }
        false
    }
}

/// Check if fuel injection should proceed considering all safety features
///
/// This is the main safety check that combines:
/// - Flood clear mode
/// - Sync loss shutdown
/// - Rev limiter (handled separately in rev_limiter module)
///
/// # Arguments
/// * `flood_clear_active` - Is flood clear mode active?
/// * `sync_shutdown` - Is engine shut down due to sync loss?
///
/// # Returns
/// `true` if injection should proceed, `false` if it should be inhibited
pub fn should_allow_injection(flood_clear_active: bool, sync_shutdown: bool) -> bool {
    // Don't inject if flood clear is active
    if flood_clear_active {
        return false;
    }

    // Don't inject if shut down due to sync loss
    if sync_shutdown {
        return false;
    }

    true
}

/// Cranking gate with simple hysteresis to avoid flapping around the threshold.
#[derive(Debug, Clone, Copy)]
pub struct CrankingGate {
    cranking: bool,
}

impl CrankingGate {
    pub const fn new() -> Self {
        Self { cranking: false }
    }
    /// Update internal state based on current RPM and return `true` if cranking.
    pub fn update(&mut self, rpm: u16) -> bool {
        if self.cranking {
            // stay cranking until safely above exit threshold
            if rpm >= CRANKING_EXIT_RPM {
                self.cranking = false;
            }
        } else if rpm < CRANKING_RPM_THRESHOLD {
            self.cranking = true;
        }
        self.cranking
    }
    pub fn is_cranking(&self) -> bool {
        self.cranking
    }
}

impl Default for CrankingGate {
    fn default() -> Self {
        Self::new()
    }
}

/// Output latch and helpers for fail-safe states
#[derive(Debug, Clone, Copy)]
pub struct OutputLatch {
    latched_off: bool,
}

impl OutputLatch {
    pub const fn new() -> Self {
        Self { latched_off: false }
    }
    pub fn latch_off(&mut self) {
        self.latched_off = true;
    }
    pub fn clear(&mut self) {
        self.latched_off = false;
    }
    pub fn is_latched(&self) -> bool {
        self.latched_off
    }
}

impl Default for OutputLatch {
    fn default() -> Self {
        Self::new()
    }
}

/// Force all outputs to safe state (logic-low)
pub fn apply_safe_state(outputs: &mut [&mut dyn crate::hal::OutputPin]) {
    for o in outputs.iter_mut() {
        o.set_low();
    }
}

// ============================================================================
// Voltage Monitoring (Brown-out detection)
// ============================================================================

use crate::constants::voltage::*;

/// Power supply state based on voltage monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PowerState {
    /// Normal operation (voltage OK)
    #[default]
    Normal,
    /// Warning: Low voltage, limp mode active
    Warning,
    /// Critical: Very low voltage, fuel cut active
    Critical,
    /// Overvoltage detected (load dump)
    Overvoltage,
}

/// Voltage monitor for brown-out detection and limp mode
#[derive(Debug, Clone, Copy)]
pub struct VoltageMonitor {
    /// Current power state
    pub state: PowerState,
    /// Last measured voltage (millivolts)
    pub last_voltage_mv: u16,
    /// Timestamp when warning state was entered (microseconds)
    pub warning_start_us: u32,
    /// Timestamp when voltage recovered above threshold
    pub recovery_start_us: u32,
    /// Count of consecutive critical readings (for debounce)
    pub critical_count: u8,
    /// Is fuel cut active due to voltage?
    pub fuel_cut_active: bool,
    /// Is limp mode active due to voltage?
    pub limp_active: bool,
}

impl VoltageMonitor {
    /// Create new voltage monitor in normal state
    pub const fn new() -> Self {
        Self {
            state: PowerState::Normal,
            last_voltage_mv: 12500, // Assume 12.5V nominal
            warning_start_us: 0,
            recovery_start_us: 0,
            critical_count: 0,
            fuel_cut_active: false,
            limp_active: false,
        }
    }

    /// Update voltage monitor with new reading
    ///
    /// # Arguments
    /// * `voltage_mv` - Current battery voltage in millivolts
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// The new power state
    pub fn update(&mut self, voltage_mv: u16, now_us: u32) -> PowerState {
        self.last_voltage_mv = voltage_mv;

        // Check for overvoltage (load dump)
        if voltage_mv > OVERVOLTAGE_MV {
            self.state = PowerState::Overvoltage;
            // Don't cut fuel on overvoltage - just log it
            // Load dumps are transient and engine should keep running
            self.critical_count = 0;
            return self.state;
        }

        // Check for critical low voltage
        if voltage_mv < BROWNOUT_CRITICAL_MV {
            self.critical_count = self.critical_count.saturating_add(1);

            if self.critical_count >= CRITICAL_DEBOUNCE_COUNT {
                self.state = PowerState::Critical;
                self.fuel_cut_active = true;
                self.limp_active = true;
                self.recovery_start_us = 0;
            }
            return self.state;
        }

        // Reset critical count if voltage is above critical
        self.critical_count = 0;

        // Check for warning low voltage
        if voltage_mv < BROWNOUT_WARNING_MV {
            if self.state != PowerState::Warning && self.state != PowerState::Critical {
                self.warning_start_us = now_us;
            }
            self.state = PowerState::Warning;
            self.limp_active = true;
            self.recovery_start_us = 0;
            // Don't cut fuel at warning level - just limit RPM
            if self.state != PowerState::Critical {
                self.fuel_cut_active = false;
            }
            return self.state;
        }

        // Voltage is above warning threshold - check for recovery
        if voltage_mv >= RECOVERY_MV {
            // Track recovery time
            if self.recovery_start_us == 0 && self.state != PowerState::Normal {
                self.recovery_start_us = now_us;
            }

            // Check if recovery time has elapsed
            if self.recovery_start_us != 0 {
                let recovery_duration = now_us.wrapping_sub(self.recovery_start_us);
                if recovery_duration >= RECOVERY_TIME_US {
                    // Recovery complete
                    self.state = PowerState::Normal;
                    self.fuel_cut_active = false;
                    self.limp_active = false;
                    self.recovery_start_us = 0;
                }
            }
        } else {
            // Voltage between warning and recovery - stay in current state
            self.recovery_start_us = 0;
        }

        // If we were in overvoltage and now normal, clear it
        if self.state == PowerState::Overvoltage && voltage_mv <= OVERVOLTAGE_MV {
            self.state = PowerState::Normal;
        }

        self.state
    }

    /// Check if fuel injection should be blocked due to voltage
    pub fn should_block_fuel(&self) -> bool {
        self.fuel_cut_active
    }

    /// Check if RPM should be limited due to voltage
    pub fn should_limit_rpm(&self) -> bool {
        self.limp_active
    }

    /// Get the RPM limit when in limp mode
    pub fn get_rpm_limit(&self) -> Option<u16> {
        if self.limp_active {
            Some(LIMP_RPM_LIMIT)
        } else {
            None
        }
    }

    /// Reset monitor to normal state (for key cycle or manual reset)
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for VoltageMonitor {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// Load Failure Tracking (MAP sensor failure at high load)
// ============================================================================

/// Reason for entering load-failure limp mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoadFailureReason {
    /// MAP sensor failure while at high RPM
    MapFailureHighRpm,
}

pub use ecu_calibration::configs::LoadFailureConfig;

/// State tracker for load failure detection
#[derive(Debug, Clone, Copy)]
pub struct LoadFailureTracker {
    /// Is limp mode active due to load failure?
    pub in_limp: bool,
    /// Reason for entering limp mode
    pub reason: Option<LoadFailureReason>,
    /// Timestamp when limp mode was entered
    pub entered_us: u32,
    /// Timestamp when good signal was first detected (for recovery)
    pub good_since_us: u32,
    /// Timestamp when fault condition was first detected (for debounce)
    pub fault_detected_us: u32,
    /// Is fault condition currently active (before debounce)?
    pub fault_pending: bool,
}

impl LoadFailureTracker {
    /// Create new load failure tracker
    pub const fn new() -> Self {
        Self {
            in_limp: false,
            reason: None,
            entered_us: 0,
            good_since_us: 0,
            fault_detected_us: 0,
            fault_pending: false,
        }
    }

    /// Check for load failure condition and update state
    ///
    /// # Arguments
    /// * `map_fault` - Is MAP sensor currently in fault state?
    /// * `rpm` - Current engine RPM
    /// * `config` - Load failure configuration
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if in limp mode (RPM should be limited)
    pub fn check(
        &mut self,
        map_fault: bool,
        rpm: u16,
        config: &LoadFailureConfig,
        now_us: u32,
    ) -> bool {
        if !config.enable {
            return false;
        }

        // Check if fault condition exists: MAP fault AND high RPM
        let fault_active = map_fault && rpm >= config.rpm_threshold;

        if fault_active {
            // Reset recovery timer
            self.good_since_us = 0;

            if !self.fault_pending {
                // Start debounce timer
                self.fault_pending = true;
                self.fault_detected_us = now_us;
            } else {
                // Check if debounce period has elapsed
                let elapsed = now_us.wrapping_sub(self.fault_detected_us);
                if elapsed >= config.debounce_time_us && !self.in_limp {
                    // Enter limp mode
                    self.in_limp = true;
                    self.reason = Some(LoadFailureReason::MapFailureHighRpm);
                    self.entered_us = now_us;
                }
            }
        } else {
            // No fault condition
            self.fault_pending = false;
            self.fault_detected_us = 0;

            // Check for recovery from limp mode
            if self.in_limp {
                if self.good_since_us == 0 {
                    // Start recovery timer
                    self.good_since_us = now_us;
                } else {
                    // Check if recovery time has elapsed
                    let recovery_elapsed = now_us.wrapping_sub(self.good_since_us);
                    if recovery_elapsed >= config.recovery_time_us {
                        // Exit limp mode
                        self.in_limp = false;
                        self.reason = None;
                        self.good_since_us = 0;
                    }
                }
            }
        }

        self.in_limp
    }

    /// Check if RPM should be limited
    pub fn should_limit_rpm(&self) -> bool {
        self.in_limp
    }

    /// Get the RPM limit when in limp mode
    pub fn get_rpm_limit(&self, config: &LoadFailureConfig) -> Option<u16> {
        if self.in_limp {
            Some(config.limp_rpm_limit)
        } else {
            None
        }
    }

    /// Reset tracker state
    pub fn reset(&mut self) {
        *self = Self::new();
    }
}

impl Default for LoadFailureTracker {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
#[path = "safety_tests.rs"]
mod tests;
