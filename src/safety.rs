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
    pub const fn new() -> Self { Self { cranking: false } }
    /// Update internal state based on current RPM and return `true` if cranking.
    pub fn update(&mut self, rpm: u16) -> bool {
        if self.cranking {
            // stay cranking until safely above exit threshold
            if rpm >= CRANKING_EXIT_RPM { self.cranking = false; }
        } else if rpm < CRANKING_RPM_THRESHOLD { self.cranking = true; }
        self.cranking
    }
    pub fn is_cranking(&self) -> bool { self.cranking }
}

impl Default for CrankingGate {
    fn default() -> Self { Self::new() }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cranking_gate_hysteresis() {
        let mut cg = CrankingGate::new();
        // Below threshold -> cranking
        assert!(cg.update(300));
        // Slightly above threshold but below exit -> still cranking
        assert!(cg.update(CRANKING_RPM_THRESHOLD + 50));
        // Above exit -> not cranking
        assert!(!cg.update(CRANKING_EXIT_RPM));
        // Drop below threshold -> cranking
        assert!(cg.update(CRANKING_RPM_THRESHOLD - 1));
    }

    #[test]
    fn test_flood_clear_inactive_during_normal_running() {
        let mut state = FloodClearState::new();

        // Normal running (2000 RPM, 50% throttle)
        let active = update_flood_clear(2000, 50, &mut state);

        assert!(!active);
        assert!(!state.active);
    }

    #[test]
    fn test_flood_clear_inactive_when_cranking_without_wot() {
        let mut state = FloodClearState::new();

        // Cranking without WOT (300 RPM, 30% throttle)
        let active = update_flood_clear(300, 30, &mut state);

        assert!(!active);
        assert!(!state.active);
    }

    #[test]
    fn test_flood_clear_active_when_cranking_with_wot() {
        let mut state = FloodClearState::new();

        // Cranking with WOT (300 RPM, 95% throttle)
        let active = update_flood_clear(300, 95, &mut state);

        assert!(active);
        assert!(state.active);
    }

    #[test]
    fn test_flood_clear_deactivates_when_engine_starts() {
        let mut state = FloodClearState::new();

        // Cranking with WOT - activates flood clear
        update_flood_clear(300, 95, &mut state);
        assert!(state.active);

        // Engine starts and revs up
        let active = update_flood_clear(800, 95, &mut state);

        assert!(!active);
        assert!(!state.active);
    }

    #[test]
    fn test_flood_clear_counts_active_cycles() {
        let mut state = FloodClearState::new();

        // Multiple cranking cycles with WOT
        for i in 1..=5 {
            update_flood_clear(300, 95, &mut state);
            assert_eq!(state.active_cycles, i);
        }
    }

    #[test]
    fn test_sync_loss_tracker_first_loss_allows_recovery() {
        let mut tracker = SyncLossTracker::new();

        let should_shutdown = tracker.record_sync_loss(1000);

        assert!(!should_shutdown);
        assert_eq!(tracker.loss_count, 1);
        assert_eq!(tracker.total_losses, 1);
    }

    #[test]
    fn test_sync_loss_tracker_shuts_down_after_multiple_losses() {
        let mut tracker = SyncLossTracker::new();

        // First loss - recovery
        let should_shutdown = tracker.record_sync_loss(1000);
        assert!(!should_shutdown);

        // Second loss (within window) - recovery
        let should_shutdown = tracker.record_sync_loss(2000);
        assert!(!should_shutdown);

        // Third loss (within window) - shutdown
        let should_shutdown = tracker.record_sync_loss(3000);
        assert!(should_shutdown);
        assert!(tracker.is_shutdown());
    }

    #[test]
    fn test_sync_loss_tracker_resets_window_after_timeout() {
        let mut tracker = SyncLossTracker::new();

        // First loss
        tracker.record_sync_loss(1000);
        assert_eq!(tracker.loss_count, 1);

        // Second loss after window expires (6 seconds later)
        // Should start new window
        let should_shutdown = tracker.record_sync_loss(6_000_000 + 1000);
        assert!(!should_shutdown);
        assert_eq!(tracker.loss_count, 1); // Reset to 1 in new window
    }

    #[test]
    fn test_sync_loss_tracker_handles_esd_glitches() {
        let mut tracker = SyncLossTracker::new();

        // Simulate isolated ESD events spread over time
        // Loss 1 at T=0
        tracker.record_sync_loss(0);
        tracker.record_recovery();

        // Loss 2 at T=6s (new window)
        tracker.record_sync_loss(6_000_000);
        tracker.record_recovery();

        // Loss 3 at T=12s (new window)
        tracker.record_sync_loss(12_000_000);
        tracker.record_recovery();

        // Should not shut down - losses are spread out (ESD pattern)
        assert!(!tracker.is_shutdown());
        assert_eq!(tracker.total_losses, 3);
        assert_eq!(tracker.successful_recoveries, 3);
    }

    #[test]
    fn test_sync_loss_tracker_detects_real_failure() {
        let mut tracker = SyncLossTracker::new();

        // Simulate rapid repeated losses (real failure pattern)
        // All within 1 second
        tracker.record_sync_loss(0);
        tracker.record_sync_loss(100_000); // 0.1s later
        tracker.record_sync_loss(200_000); // 0.2s later

        // Should shut down - too many losses too quickly
        assert!(tracker.is_shutdown());
    }

    #[test]
    fn test_should_allow_injection_normal() {
        let allow = should_allow_injection(false, false);
        assert!(allow);
    }

    #[test]
    fn test_should_allow_injection_blocks_on_flood_clear() {
        let allow = should_allow_injection(true, false);
        assert!(!allow);
    }

    #[test]
    fn test_should_allow_injection_blocks_on_shutdown() {
        let allow = should_allow_injection(false, true);
        assert!(!allow);
    }

    #[test]
    fn test_should_allow_injection_blocks_on_both() {
        let allow = should_allow_injection(true, true);
        assert!(!allow);
    }

    #[test]
    fn test_clear_shutdown_resets_state() {
        let mut tracker = SyncLossTracker::new();

        // Trigger shutdown
        tracker.record_sync_loss(0);
        tracker.record_sync_loss(1000);
        tracker.record_sync_loss(2000);
        assert!(tracker.is_shutdown());

        // Manual clear (key cycle)
        tracker.clear_shutdown();

        assert!(!tracker.is_shutdown());
        assert_eq!(tracker.loss_count, 0);
    }

    #[test]
    fn test_reset_window_clears_loss_count() {
        let mut tracker = SyncLossTracker::new();

        // Record some losses
        tracker.record_sync_loss(0);
        tracker.record_sync_loss(1000);
        assert_eq!(tracker.loss_count, 2);

        // After sustained good operation, reset window
        tracker.reset_window();

        assert_eq!(tracker.loss_count, 0);
        assert!(!tracker.is_shutdown());
        // Total losses preserved for diagnostics
        assert_eq!(tracker.total_losses, 2);
    }
}
