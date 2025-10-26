//! RPM Limiting (Rev Limiter) for Engine Protection
//!
//! Prevents engine over-rev damage by cutting fuel and/or ignition at maximum RPM.
//! Supports multiple limiting strategies for different applications.
//!
//! # Safety Philosophy
//!
//! Over-revving an engine can cause catastrophic damage:
//! - Valve float (valves don't close properly at high RPM)
//! - Bent valves (piston-to-valve contact)
//! - Thrown connecting rods (inertia forces exceed material strength)
//! - Broken crankshaft
//!
//! The rev limiter is the LAST line of defense. Proper operation is critical.
//!
//! # Limiting Strategies
//!
//! - **Hard Cut**: Completely cut fuel/ignition for X cylinders, then resume
//!   - Pros: Very effective, simple
//!   - Cons: Harsh, can upset vehicle balance
//!   - Use: Race applications, max RPM protection
//!
//! - **Soft Cut**: Gradually reduce fuel as RPM approaches limit
//!   - Pros: Smooth, less harsh
//!   - Cons: More complex, less effective at preventing over-rev
//!   - Use: Street applications, smooth limiting
//!
//! - **Ignition Retard**: Retard ignition timing to reduce power
//!   - Pros: Smooth, maintains some power
//!   - Cons: Hot exhaust (can damage turbo/cat)
//!   - Use: Launch control, turbo applications

use crate::constants::rev_limiter::*;

/// Rev limiter strategy
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LimiterStrategy {
    /// Hard cut: completely disable fuel/ignition
    HardCut,
    /// Soft cut: gradually reduce fuel
    SoftCut,
    /// Ignition retard: reduce power by retarding timing
    IgnitionRetard,
    /// Combined: ignition retard + fuel cut
    Combined,
}

/// Rev limiter configuration
#[derive(Debug, Clone, Copy)]
pub struct RevLimiterConfig {
    /// Maximum RPM before limiting starts
    pub max_rpm: u16,

    /// RPM below max_rpm where soft limiting begins (soft cut only)
    pub soft_limit_start_rpm: u16,

    /// Strategy to use
    pub strategy: LimiterStrategy,

    /// Number of cylinders to cut in hard cut mode (1-4)
    pub cut_cylinders: u8,

    /// Ignition retard amount (degrees) for retard strategy
    pub retard_amount: i16,

    /// Hysteresis: RPM must drop this much below limit before re-enabling
    pub hysteresis_rpm: u16,
}

impl RevLimiterConfig {
    /// Conservative default configuration (street safe)
    pub const DEFAULT: Self = Self {
        max_rpm: DEFAULT_MAX_RPM,
        soft_limit_start_rpm: DEFAULT_SOFT_LIMIT_START_RPM,
        strategy: LimiterStrategy::SoftCut,
        cut_cylinders: 2,  // Cut 2 cylinders (50% reduction)
        retard_amount: 15, // 15° retard
        hysteresis_rpm: HYSTERESIS_RPM,
    };

    /// Aggressive race configuration (hard cut)
    pub const RACE: Self = Self {
        max_rpm: 8000,
        soft_limit_start_rpm: 7800,
        strategy: LimiterStrategy::HardCut,
        cut_cylinders: 4,  // Cut all cylinders (full cut)
        retard_amount: 0,
        hysteresis_rpm: 200,
    };

    /// Launch control configuration (smooth power limiting)
    pub const LAUNCH: Self = Self {
        max_rpm: 4000,  // Launch RPM limit
        soft_limit_start_rpm: 3900,
        strategy: LimiterStrategy::Combined,
        cut_cylinders: 1,
        retard_amount: 10,
        hysteresis_rpm: 100,
    };
}

/// Rev limiter state
#[derive(Debug, Clone, Copy)]
pub struct RevLimiterState {
    /// Is limiting currently active?
    pub active: bool,

    /// Fuel cut percentage (0-100, where 100 = full cut)
    pub fuel_cut_percent: u8,

    /// Ignition retard amount (degrees)
    pub ignition_retard: i16,

    /// Cut pattern counter (for hard cut cycling)
    cut_counter: u8,
}

impl RevLimiterState {
    /// Create new limiter state (inactive)
    pub const fn new() -> Self {
        Self {
            active: false,
            fuel_cut_percent: 0,
            ignition_retard: 0,
            cut_counter: 0,
        }
    }

    /// Reset limiter to inactive state
    pub fn reset(&mut self) {
        self.active = false;
        self.fuel_cut_percent = 0;
        self.ignition_retard = 0;
        self.cut_counter = 0;
    }
}

/// Calculate rev limiter action based on current RPM
///
/// # Arguments
/// * `rpm` - Current engine RPM
/// * `config` - Rev limiter configuration
/// * `state` - Current limiter state (will be updated)
///
/// # Returns
/// Updated state indicating fuel cut percentage and ignition retard
pub fn update_limiter(rpm: u16, config: &RevLimiterConfig, state: &mut RevLimiterState) {
    // Determine activation threshold based on strategy
    let activation_rpm = match config.strategy {
        LimiterStrategy::SoftCut | LimiterStrategy::Combined => config.soft_limit_start_rpm,
        LimiterStrategy::HardCut | LimiterStrategy::IgnitionRetard => config.max_rpm,
    };

    // Check if we should activate limiting
    let should_limit = rpm >= activation_rpm;

    // Check if we should deactivate (hysteresis)
    let deactivation_rpm = activation_rpm.saturating_sub(config.hysteresis_rpm);
    let should_deactivate = state.active && (rpm < deactivation_rpm);

    if should_deactivate {
        state.reset();
        return;
    }

    if !should_limit && !state.active {
        // Below limit, not active - nothing to do
        state.reset();
        return;
    }

    // Limiting is active
    state.active = true;

    match config.strategy {
        LimiterStrategy::HardCut => {
            apply_hard_cut(config, state);
        }
        LimiterStrategy::SoftCut => {
            apply_soft_cut(rpm, config, state);
        }
        LimiterStrategy::IgnitionRetard => {
            apply_ignition_retard(config, state);
        }
        LimiterStrategy::Combined => {
            apply_combined(rpm, config, state);
        }
    }
}

/// Apply hard cut strategy (full fuel/ignition cut for X cylinders)
fn apply_hard_cut(config: &RevLimiterConfig, state: &mut RevLimiterState) {
    // Cut specified number of cylinders
    // This is a simplified version - real implementation would cut specific cylinders
    let cut_percentage = (config.cut_cylinders as u16 * 100 / 4).min(100) as u8;
    state.fuel_cut_percent = cut_percentage;
    state.ignition_retard = 0;

    // Increment counter for cycling (could be used for per-cylinder cutting)
    state.cut_counter = state.cut_counter.wrapping_add(1);
}

/// Apply soft cut strategy (gradual fuel reduction)
fn apply_soft_cut(rpm: u16, config: &RevLimiterConfig, state: &mut RevLimiterState) {
    if rpm < config.soft_limit_start_rpm {
        // Below soft limit start - no cutting
        state.fuel_cut_percent = 0;
        state.ignition_retard = 0;
    } else if rpm >= config.max_rpm {
        // Above max RPM - full cut
        state.fuel_cut_percent = 100;
        state.ignition_retard = 0;
    } else {
        // Between soft start and max - linear interpolation
        let rpm_range = config.max_rpm - config.soft_limit_start_rpm;
        let rpm_above_start = rpm - config.soft_limit_start_rpm;
        let cut_percentage = ((rpm_above_start as u32 * 100) / rpm_range as u32) as u8;
        state.fuel_cut_percent = cut_percentage.min(100);
        state.ignition_retard = 0;
    }
}

/// Apply ignition retard strategy
fn apply_ignition_retard(config: &RevLimiterConfig, state: &mut RevLimiterState) {
    state.fuel_cut_percent = 0;
    state.ignition_retard = config.retard_amount;
}

/// Apply combined strategy (retard + fuel cut)
fn apply_combined(rpm: u16, config: &RevLimiterConfig, state: &mut RevLimiterState) {
    if rpm < config.soft_limit_start_rpm {
        // Below soft limit - no action
        state.fuel_cut_percent = 0;
        state.ignition_retard = 0;
    } else if rpm >= config.max_rpm {
        // Above max - full retard + partial fuel cut
        state.ignition_retard = config.retard_amount;
        state.fuel_cut_percent = 50;  // 50% fuel cut + retard
    } else {
        // Between soft start and max - gradual retard
        let rpm_range = config.max_rpm - config.soft_limit_start_rpm;
        let rpm_above_start = rpm.saturating_sub(config.soft_limit_start_rpm);
        let retard_percent = ((rpm_above_start as u32 * 100) / rpm_range as u32) as u8;
        state.ignition_retard = ((config.retard_amount as i32 * retard_percent as i32) / 100) as i16;
        state.fuel_cut_percent = 0;
    }
}

/// Check if fuel injection should be allowed for this event
///
/// # Arguments
/// * `state` - Current limiter state
/// * `cylinder` - Cylinder number (0-3)
///
/// # Returns
/// `true` if injection should proceed, `false` if it should be cut
pub fn should_inject(state: &RevLimiterState, _cylinder: u8) -> bool {
    if !state.active {
        return true;
    }

    // For simplicity, apply fuel cut percentage globally
    // A more sophisticated implementation would cut specific cylinders
    if state.fuel_cut_percent >= 100 {
        return false;
    }

    if state.fuel_cut_percent == 0 {
        return true;
    }

    // Probabilistic cut based on percentage (simplified)
    // Real implementation would use a deterministic pattern
    // For now, always inject if cut is <50%, never inject if >=50%
    state.fuel_cut_percent < 50
}

/// Apply rev limiter ignition retard to base timing
///
/// # Arguments
/// * `base_timing` - Base ignition timing (degrees BTDC)
/// * `state` - Current limiter state
///
/// # Returns
/// Modified timing with limiter retard applied
pub fn apply_limiter_retard(base_timing: i16, state: &RevLimiterState) -> i16 {
    if !state.active {
        return base_timing;
    }

    base_timing - state.ignition_retard
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_limiter_inactive_below_rpm() {
        let config = RevLimiterConfig::DEFAULT;
        let mut state = RevLimiterState::new();

        update_limiter(5000, &config, &mut state);

        assert!(!state.active);
        assert_eq!(state.fuel_cut_percent, 0);
        assert_eq!(state.ignition_retard, 0);
    }

    #[test]
    fn test_hard_cut_at_max_rpm() {
        let mut config = RevLimiterConfig::DEFAULT;
        config.strategy = LimiterStrategy::HardCut;
        config.max_rpm = 7000;
        config.cut_cylinders = 4;

        let mut state = RevLimiterState::new();

        update_limiter(7000, &config, &mut state);

        assert!(state.active);
        assert_eq!(state.fuel_cut_percent, 100);
    }

    #[test]
    fn test_soft_cut_gradual_increase() {
        let mut config = RevLimiterConfig::DEFAULT;
        config.strategy = LimiterStrategy::SoftCut;
        config.soft_limit_start_rpm = 6500;
        config.max_rpm = 7000;

        let mut state = RevLimiterState::new();

        // Below soft limit
        update_limiter(6400, &config, &mut state);
        assert_eq!(state.fuel_cut_percent, 0);

        // Slightly above soft limit start (need enough RPM for integer division to show)
        update_limiter(6550, &config, &mut state);
        assert!(state.fuel_cut_percent > 0);
        assert!(state.fuel_cut_percent < 100);

        // Halfway between
        update_limiter(6750, &config, &mut state);
        assert!(state.fuel_cut_percent >= 40 && state.fuel_cut_percent <= 60);

        // At max RPM
        update_limiter(7000, &config, &mut state);
        assert_eq!(state.fuel_cut_percent, 100);
    }

    #[test]
    fn test_hysteresis_prevents_oscillation() {
        let mut config = RevLimiterConfig::DEFAULT;
        config.strategy = LimiterStrategy::HardCut;  // Hard cut activates at max_rpm
        config.max_rpm = 7000;
        config.hysteresis_rpm = 200;

        let mut state = RevLimiterState::new();

        // Activate limiter
        update_limiter(7000, &config, &mut state);
        assert!(state.active);

        // Drop slightly below max - should still be active (hysteresis)
        update_limiter(6950, &config, &mut state);
        assert!(state.active);

        // Drop below hysteresis threshold - should deactivate
        update_limiter(6799, &config, &mut state);
        assert!(!state.active);
        assert_eq!(state.fuel_cut_percent, 0);
    }

    #[test]
    fn test_ignition_retard_strategy() {
        let mut config = RevLimiterConfig::DEFAULT;
        config.strategy = LimiterStrategy::IgnitionRetard;
        config.max_rpm = 7000;
        config.retard_amount = 15;

        let mut state = RevLimiterState::new();

        update_limiter(7000, &config, &mut state);

        assert!(state.active);
        assert_eq!(state.fuel_cut_percent, 0);
        assert_eq!(state.ignition_retard, 15);
    }

    #[test]
    fn test_combined_strategy() {
        let mut config = RevLimiterConfig::DEFAULT;
        config.strategy = LimiterStrategy::Combined;
        config.soft_limit_start_rpm = 6500;
        config.max_rpm = 7000;
        config.retard_amount = 20;

        let mut state = RevLimiterState::new();

        // At max RPM - should have both retard and fuel cut
        update_limiter(7000, &config, &mut state);
        assert!(state.active);
        assert_eq!(state.ignition_retard, 20);
        assert_eq!(state.fuel_cut_percent, 50);
    }

    #[test]
    fn test_should_inject() {
        let mut state = RevLimiterState::new();

        // Inactive - always inject
        assert!(should_inject(&state, 0));

        // Active with 0% cut - inject
        state.active = true;
        state.fuel_cut_percent = 0;
        assert!(should_inject(&state, 0));

        // Active with partial cut
        state.fuel_cut_percent = 25;
        assert!(should_inject(&state, 0));

        // Active with 50%+ cut - don't inject
        state.fuel_cut_percent = 50;
        assert!(!should_inject(&state, 0));

        // Active with full cut - don't inject
        state.fuel_cut_percent = 100;
        assert!(!should_inject(&state, 0));
    }

    #[test]
    fn test_apply_limiter_retard() {
        let mut state = RevLimiterState::new();
        let base_timing = 20;

        // Inactive - no change
        assert_eq!(apply_limiter_retard(base_timing, &state), 20);

        // Active with 10° retard
        state.active = true;
        state.ignition_retard = 10;
        assert_eq!(apply_limiter_retard(base_timing, &state), 10);

        // Active with 15° retard
        state.ignition_retard = 15;
        assert_eq!(apply_limiter_retard(base_timing, &state), 5);
    }

    #[test]
    fn test_race_config() {
        let config = RevLimiterConfig::RACE;
        let mut state = RevLimiterState::new();

        update_limiter(8000, &config, &mut state);

        assert!(state.active);
        assert_eq!(state.fuel_cut_percent, 100);  // Full hard cut
    }

    #[test]
    fn test_launch_config() {
        let config = RevLimiterConfig::LAUNCH;
        let mut state = RevLimiterState::new();

        update_limiter(4000, &config, &mut state);

        assert!(state.active);
        // Combined strategy should have both retard and fuel cut
        assert!(state.ignition_retard > 0 || state.fuel_cut_percent > 0);
    }
}
