//! Torque Request Types and Generators
//!
//! Defines torque request sources and provides functions to generate
//! torque requests from various inputs.

use super::TorqueConfig;

/// Source of a torque request
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TorqueSource {
    /// Driver pedal position
    Driver,
    /// Idle speed controller
    Idle,
    /// Rev limiter
    RevLimiter,
    /// Traction control (future)
    Traction,
    /// Limp mode
    Limp,
    /// External request (CAN, cruise control, etc.)
    External,
    /// Anti-stall protection
    AntiStall,
    /// Knock retard torque reduction
    Knock,
}

/// A torque request from any source
#[derive(Debug, Clone, Copy)]
pub struct TorqueRequest {
    /// Source of this request
    pub source: TorqueSource,
    /// Requested torque (Nm x10, signed: positive = driving, negative = braking)
    pub torque_nm_x10: i16,
    /// Priority (higher = more important, 0-255)
    pub priority: u8,
    /// Timestamp when request was made (microseconds)
    pub timestamp_us: u32,
    /// Is this request active?
    pub active: bool,
}

impl TorqueRequest {
    pub const fn new(source: TorqueSource) -> Self {
        Self {
            source,
            torque_nm_x10: 0,
            priority: 0,
            timestamp_us: 0,
            active: false,
        }
    }

    /// Create an active request
    pub const fn active(
        source: TorqueSource,
        torque_nm_x10: i16,
        priority: u8,
        timestamp_us: u32,
    ) -> Self {
        Self {
            source,
            torque_nm_x10,
            priority,
            timestamp_us,
            active: true,
        }
    }
}

impl Default for TorqueRequest {
    fn default() -> Self {
        Self::new(TorqueSource::Driver)
    }
}

/// Priority levels for torque sources
pub mod priority {
    pub const DRIVER: u8 = 50; // Normal driver request
    pub const IDLE: u8 = 40; // Below driver
    pub const EXTERNAL: u8 = 60; // Cruise control, etc.
    pub const TRACTION: u8 = 100; // Safety override
    pub const REV_LIMITER: u8 = 200; // Near-absolute
    pub const KNOCK: u8 = 180; // High priority safety
    pub const LIMP: u8 = 250; // Highest priority
    pub const ANTI_STALL: u8 = 30; // Low priority assist
}

/// Torque estimation from engine conditions
#[derive(Debug, Clone, Copy, Default)]
pub struct TorqueEstimator {
    /// Last estimated max torque
    pub last_max_x10: i16,
}

impl TorqueEstimator {
    pub const fn new() -> Self {
        Self { last_max_x10: 0 }
    }

    /// Estimate maximum available torque based on current conditions
    ///
    /// Uses a simplified model based on MAP (manifold pressure) as a proxy
    /// for air mass flow, which correlates with maximum torque.
    ///
    /// # Arguments
    /// * `rpm` - Engine RPM
    /// * `map_kpa_x10` - MAP reading (kPa x10)
    /// * `iat_c` - Intake air temperature (Celsius)
    /// * `config` - Torque configuration
    ///
    /// # Returns
    /// Maximum available torque (Nm x10)
    pub fn estimate_max_torque(
        &mut self,
        rpm: u16,
        map_kpa_x10: u16,
        iat_c: i16,
        config: &TorqueConfig,
    ) -> i16 {
        // Base torque from MAP (higher MAP = more air = more torque)
        // Assume 100 kPa (1000 x10) = 100% of rated torque
        let map_factor = (map_kpa_x10 as i32 * 100) / 1000; // 0-100+ (can exceed for boost)

        // RPM factor (torque curve approximation)
        // Peak at peak_torque_rpm, reduced at low/high RPM
        let rpm_factor = self.rpm_torque_factor(rpm, config);

        // IAT factor (cold air = denser = more power)
        // Normalize to 25°C = 100%
        let iat_factor = self.iat_torque_factor(iat_c);

        // Calculate max torque
        let max_torque =
            (config.max_torque_nm_x10 as i32 * map_factor * rpm_factor as i32 * iat_factor as i32)
                / 1_000_000; // Scale back (100 * 100 * 100)

        self.last_max_x10 = max_torque.clamp(0, config.max_torque_nm_x10 as i32) as i16;
        self.last_max_x10
    }

    /// RPM-based torque multiplier (percent, 0-100)
    fn rpm_torque_factor(&self, rpm: u16, config: &TorqueConfig) -> u8 {
        if rpm == 0 {
            return 0;
        }

        let peak_rpm = config.peak_torque_rpm as i32;
        let rpm_i32 = rpm as i32;

        // Simplified torque curve:
        // - Rises linearly from 0 to peak_rpm
        // - Drops off above peak_rpm
        if rpm_i32 <= peak_rpm {
            // Rising portion: linear 0-100%
            ((rpm_i32 * 100) / peak_rpm).clamp(0, 100) as u8
        } else {
            // Falling portion: drops ~10% per 1000 RPM above peak
            let over_peak = rpm_i32 - peak_rpm;
            let drop = (over_peak * 10) / 1000;
            (100 - drop).clamp(50, 100) as u8 // Never drops below 50%
        }
    }

    /// IAT-based torque multiplier (percent, 80-110)
    fn iat_torque_factor(&self, iat_c: i16) -> u8 {
        // Cold air is denser, hot air is thinner
        // Baseline: 25°C = 100%
        // Each degree below 25: +0.3% (up to 110% at -10°C)
        // Each degree above 25: -0.3% (down to 85% at 75°C)
        let baseline = 25i16;
        let delta = baseline - iat_c;
        let adjustment = (delta as i32 * 3) / 10;
        (100 + adjustment).clamp(80, 115) as u8
    }
}

/// Generate a driver torque request from pedal position
///
/// # Arguments
/// * `pedal_percent` - Accelerator pedal position (0-100%)
/// * `rpm` - Current engine RPM
/// * `max_torque_x10` - Maximum available torque (Nm x10)
/// * `now_us` - Current timestamp
pub fn driver_torque_request(
    pedal_percent: u8,
    _rpm: u16,
    max_torque_x10: i16,
    now_us: u32,
) -> TorqueRequest {
    // Linear mapping: 0% pedal = 0 torque, 100% = max torque
    let requested = (max_torque_x10 as i32 * pedal_percent as i32) / 100;

    TorqueRequest::active(
        TorqueSource::Driver,
        requested as i16,
        priority::DRIVER,
        now_us,
    )
}

/// Generate an idle controller torque request
///
/// # Arguments
/// * `target_rpm` - Target idle RPM
/// * `actual_rpm` - Current RPM
/// * `reserve_x10` - Torque reserve for idle control (Nm x10)
/// * `now_us` - Current timestamp
pub fn idle_torque_request(
    target_rpm: u16,
    actual_rpm: u16,
    reserve_x10: i16,
    now_us: u32,
) -> TorqueRequest {
    // Simple proportional control: more torque if below target
    let rpm_error = target_rpm as i32 - actual_rpm as i32;

    // Scale: ~1 Nm per 100 RPM error, limited to reserve
    let torque = (rpm_error * reserve_x10 as i32) / 100;
    let torque = torque.clamp(-reserve_x10 as i32, reserve_x10 as i32);

    TorqueRequest::active(TorqueSource::Idle, torque as i16, priority::IDLE, now_us)
}

/// Generate a rev limiter torque request
///
/// When the rev limiter is active, request zero torque.
pub fn rev_limiter_torque_request(limit_active: bool, now_us: u32) -> TorqueRequest {
    TorqueRequest {
        source: TorqueSource::RevLimiter,
        torque_nm_x10: if limit_active { 0 } else { i16::MAX },
        priority: priority::REV_LIMITER,
        timestamp_us: now_us,
        active: limit_active,
    }
}

/// Generate a limp mode torque request
///
/// Limp mode limits torque to a safe level.
pub fn limp_torque_request(limp_active: bool, max_torque_x10: i16, now_us: u32) -> TorqueRequest {
    // Limp mode: limit to 30% of max torque
    let limp_torque = if limp_active {
        (max_torque_x10 as i32 * 30 / 100) as i16
    } else {
        i16::MAX
    };

    TorqueRequest {
        source: TorqueSource::Limp,
        torque_nm_x10: limp_torque,
        priority: priority::LIMP,
        timestamp_us: now_us,
        active: limp_active,
    }
}

/// Generate a knock-based torque reduction request
pub fn knock_torque_request(
    knock_retard_x10: u16,
    max_torque_x10: i16,
    now_us: u32,
) -> TorqueRequest {
    // Each degree of retard reduces torque by ~2%
    let reduction_percent = (knock_retard_x10 as i32 * 2) / 10;
    let reduced_torque = max_torque_x10 as i32 * (100 - reduction_percent) / 100;

    TorqueRequest::active(
        TorqueSource::Knock,
        reduced_torque as i16,
        priority::KNOCK,
        now_us,
    )
}

/// Generate an anti-stall torque request
///
/// Adds torque when RPM drops near stall to prevent engine stall.
/// This is a positive intervention (adds torque) unlike most safety limits.
///
/// # Arguments
/// * `actual_rpm` - Current engine RPM
/// * `stall_rpm` - RPM threshold below which anti-stall activates
/// * `assist_torque_x10` - Maximum assist torque (Nm x10)
/// * `now_us` - Current timestamp
pub fn anti_stall_torque_request(
    actual_rpm: u16,
    stall_rpm: u16,
    assist_torque_x10: i16,
    now_us: u32,
) -> TorqueRequest {
    if actual_rpm >= stall_rpm {
        // Above stall threshold, no assist needed
        return TorqueRequest {
            source: TorqueSource::AntiStall,
            torque_nm_x10: 0,
            priority: priority::ANTI_STALL,
            timestamp_us: now_us,
            active: false,
        };
    }

    // Proportional assist: more torque as RPM drops
    // Full assist at 50% of stall_rpm, none at stall_rpm
    let rpm_margin = stall_rpm.saturating_sub(actual_rpm);
    let half_stall = stall_rpm / 2;
    let assist_factor = (rpm_margin as i32 * 100) / half_stall.max(1) as i32;
    let assist = (assist_torque_x10 as i32 * assist_factor.min(100)) / 100;

    TorqueRequest::active(
        TorqueSource::AntiStall,
        assist as i16,
        priority::ANTI_STALL,
        now_us,
    )
}

/// Generate a traction control torque request
///
/// Reduces torque when wheel slip is detected.
///
/// # Arguments
/// * `slip_percent` - Wheel slip percentage (0 = no slip, 100 = full slip)
/// * `max_torque_x10` - Maximum available torque (Nm x10)
/// * `now_us` - Current timestamp
pub fn traction_torque_request(
    slip_percent: u8,
    max_torque_x10: i16,
    now_us: u32,
) -> TorqueRequest {
    if slip_percent == 0 {
        // No slip, allow full torque
        return TorqueRequest {
            source: TorqueSource::Traction,
            torque_nm_x10: i16::MAX,
            priority: priority::TRACTION,
            timestamp_us: now_us,
            active: false,
        };
    }

    // Reduce torque proportionally to slip
    // 10% slip = 90% torque, 50% slip = 50% torque, etc.
    let reduction = slip_percent.min(100) as i32;
    let limited_torque = max_torque_x10 as i32 * (100 - reduction) / 100;

    TorqueRequest::active(
        TorqueSource::Traction,
        limited_torque as i16,
        priority::TRACTION,
        now_us,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_torque_request_new() {
        let req = TorqueRequest::new(TorqueSource::Driver);
        assert_eq!(req.source, TorqueSource::Driver);
        assert!(!req.active);
        assert_eq!(req.torque_nm_x10, 0);
    }

    #[test]
    fn test_torque_request_active() {
        let req = TorqueRequest::active(TorqueSource::Idle, 100, 50, 1000);
        assert!(req.active);
        assert_eq!(req.torque_nm_x10, 100);
        assert_eq!(req.priority, 50);
    }

    #[test]
    fn test_driver_torque_request() {
        let req = driver_torque_request(50, 3000, 2000, 0);
        assert_eq!(req.source, TorqueSource::Driver);
        assert_eq!(req.torque_nm_x10, 1000); // 50% of 2000
        assert!(req.active);
    }

    #[test]
    fn test_driver_torque_request_full() {
        let req = driver_torque_request(100, 3000, 2000, 0);
        assert_eq!(req.torque_nm_x10, 2000);
    }

    #[test]
    fn test_driver_torque_request_zero() {
        let req = driver_torque_request(0, 3000, 2000, 0);
        assert_eq!(req.torque_nm_x10, 0);
    }

    #[test]
    fn test_idle_torque_request_below_target() {
        let req = idle_torque_request(800, 700, 50, 0);
        assert_eq!(req.source, TorqueSource::Idle);
        // Below target, should request positive torque
        assert!(req.torque_nm_x10 > 0);
    }

    #[test]
    fn test_idle_torque_request_above_target() {
        let req = idle_torque_request(800, 900, 50, 0);
        // Above target, should request negative torque (engine braking)
        assert!(req.torque_nm_x10 < 0);
    }

    #[test]
    fn test_rev_limiter_torque_request() {
        let req_active = rev_limiter_torque_request(true, 0);
        assert!(req_active.active);
        assert_eq!(req_active.torque_nm_x10, 0);

        let req_inactive = rev_limiter_torque_request(false, 0);
        assert!(!req_inactive.active);
        assert_eq!(req_inactive.torque_nm_x10, i16::MAX);
    }

    #[test]
    fn test_limp_torque_request() {
        let req = limp_torque_request(true, 2000, 0);
        assert!(req.active);
        assert_eq!(req.torque_nm_x10, 600); // 30% of 2000
    }

    #[test]
    fn test_knock_torque_request() {
        let req = knock_torque_request(50, 2000, 0); // 5 degrees retard
        assert!(req.active);
        // 5 degrees * 2% = 10% reduction
        assert_eq!(req.torque_nm_x10, 1800); // 2000 - 10%
    }

    #[test]
    fn test_torque_estimator_basic() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        let max = estimator.estimate_max_torque(3000, 800, 25, &config);
        assert!(max > 0);
        assert!(max <= config.max_torque_nm_x10);
    }

    #[test]
    fn test_torque_estimator_map_scaling() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        let low_map = estimator.estimate_max_torque(3000, 400, 25, &config);
        let high_map = estimator.estimate_max_torque(3000, 900, 25, &config);

        assert!(high_map > low_map);
    }

    #[test]
    fn test_torque_estimator_rpm_scaling() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        let low_rpm = estimator.estimate_max_torque(1000, 800, 25, &config);
        let peak_rpm = estimator.estimate_max_torque(4000, 800, 25, &config);

        assert!(peak_rpm > low_rpm);
    }

    #[test]
    fn test_torque_estimator_iat_scaling() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        let cold_iat = estimator.estimate_max_torque(3000, 800, 0, &config);
        let hot_iat = estimator.estimate_max_torque(3000, 800, 50, &config);

        assert!(cold_iat > hot_iat);
    }

    #[test]
    fn test_priority_ordering() {
        let ordered = [
            priority::IDLE,
            priority::DRIVER,
            priority::EXTERNAL,
            priority::TRACTION,
            priority::KNOCK,
            priority::REV_LIMITER,
            priority::LIMP,
        ];
        assert!(ordered.windows(2).all(|w| w[0] < w[1]));
    }

    // --- Anti-stall tests ---

    #[test]
    fn test_anti_stall_request_inactive_above_threshold() {
        let req = anti_stall_torque_request(1000, 800, 50, 0);
        assert!(!req.active);
        assert_eq!(req.source, TorqueSource::AntiStall);
    }

    #[test]
    fn test_anti_stall_request_active_below_threshold() {
        let req = anti_stall_torque_request(600, 800, 50, 0);
        assert!(req.active);
        assert!(
            req.torque_nm_x10 > 0,
            "Should provide positive assist torque"
        );
    }

    #[test]
    fn test_anti_stall_request_proportional() {
        // At 50% of stall RPM, should get full assist
        let full_assist = anti_stall_torque_request(400, 800, 100, 0);
        // At 75% of stall RPM (closer to threshold), should get less assist
        let partial_assist = anti_stall_torque_request(600, 800, 100, 0);

        assert!(
            full_assist.torque_nm_x10 >= partial_assist.torque_nm_x10,
            "Assist should increase as RPM drops"
        );
    }

    // --- Traction control tests ---

    #[test]
    fn test_traction_request_no_slip() {
        let req = traction_torque_request(0, 2000, 0);
        assert!(!req.active);
        assert_eq!(req.source, TorqueSource::Traction);
    }

    #[test]
    fn test_traction_request_moderate_slip() {
        let req = traction_torque_request(20, 2000, 0);
        assert!(req.active);
        // 20% slip = 80% torque = 1600
        assert_eq!(req.torque_nm_x10, 1600);
    }

    #[test]
    fn test_traction_request_heavy_slip() {
        let req = traction_torque_request(50, 2000, 0);
        assert!(req.active);
        // 50% slip = 50% torque = 1000
        assert_eq!(req.torque_nm_x10, 1000);
    }

    #[test]
    fn test_traction_request_full_slip() {
        let req = traction_torque_request(100, 2000, 0);
        assert!(req.active);
        // 100% slip = 0% torque = 0
        assert_eq!(req.torque_nm_x10, 0);
    }

    // --- Torque estimator edge cases ---

    #[test]
    fn test_torque_estimator_zero_rpm() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        let torque = estimator.estimate_max_torque(0, 800, 25, &config);
        assert_eq!(torque, 0, "No torque at 0 RPM");
    }

    #[test]
    fn test_torque_estimator_boosted() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        // Boost = MAP > 100 kPa (1000 x10)
        let natural = estimator.estimate_max_torque(3000, 900, 25, &config);
        let boosted = estimator.estimate_max_torque(3000, 1200, 25, &config); // 120 kPa

        assert!(boosted > natural, "Boost should increase available torque");
    }

    #[test]
    fn test_torque_estimator_very_high_rpm() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        // At very high RPM, torque should still be reasonable (clamped to 50% min)
        let high_rpm = estimator.estimate_max_torque(8000, 800, 25, &config);
        let peak = estimator.estimate_max_torque(config.peak_torque_rpm, 800, 25, &config);

        assert!(high_rpm > 0, "Should still have some torque at high RPM");
        assert!(high_rpm < peak, "Torque should drop above peak RPM");
        assert!(high_rpm >= peak / 2, "Torque should not drop below 50%");
    }

    #[test]
    fn test_torque_estimator_extreme_temps() {
        let mut estimator = TorqueEstimator::new();
        let config = TorqueConfig::DEFAULT;

        // Very cold
        let cold = estimator.estimate_max_torque(3000, 800, -20, &config);
        // Very hot
        let hot = estimator.estimate_max_torque(3000, 800, 80, &config);
        // Normal
        let normal = estimator.estimate_max_torque(3000, 800, 25, &config);

        assert!(cold > normal, "Cold air should increase torque");
        assert!(hot < normal, "Hot air should decrease torque");
    }
}
