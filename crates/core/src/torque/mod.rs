//! Torque-Based Engine Control Model
//!
//! Provides a unified torque abstraction for engine control with:
//! - Multiple torque request sources (driver, idle, rev limiter, etc.)
//! - Min-wins arbitration (most restrictive request wins)
//! - Torque-to-actuator conversion (fuel/timing/throttle)
//!
//! This enables coordinated control and simplifies safety interventions.

pub mod actuate;
pub mod arbiter;
pub mod request;

pub use actuate::{ActuatorTargets, TorqueConverter};
pub use arbiter::TorqueArbiter;
pub use request::{TorqueEstimator, TorqueRequest, TorqueSource};

/// Configuration for the torque model
#[derive(Debug, Clone, Copy)]
pub struct TorqueConfig {
    /// Enable torque-based control
    pub enable: bool,
    /// Nominal engine torque at peak (Nm x10)
    pub max_torque_nm_x10: i16,
    /// RPM at peak torque
    pub peak_torque_rpm: u16,
    /// Minimum engine braking torque (negative, Nm x10)
    pub min_torque_nm_x10: i16,
    /// Idle torque reserve (Nm x10)
    pub idle_torque_reserve_x10: i16,
}

impl TorqueConfig {
    /// Default configuration for a typical 4-cylinder engine
    pub const DEFAULT: Self = Self {
        enable: true,
        max_torque_nm_x10: 2000, // 200 Nm
        peak_torque_rpm: 4000,
        min_torque_nm_x10: -500,     // -50 Nm engine braking
        idle_torque_reserve_x10: 50, // 5 Nm reserve for idle control
    };
}

impl Default for TorqueConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Combined torque controller
#[derive(Debug, Clone, Copy)]
pub struct TorqueController {
    pub config: TorqueConfig,
    pub arbiter: TorqueArbiter,
    pub estimator: TorqueEstimator,
    pub converter: TorqueConverter,
    /// Current arbitrated torque target (Nm x10)
    pub target_torque_x10: i16,
    /// Current maximum available torque (Nm x10)
    pub max_available_x10: i16,
}

impl TorqueController {
    pub const fn new() -> Self {
        Self {
            config: TorqueConfig::DEFAULT,
            arbiter: TorqueArbiter::new(),
            estimator: TorqueEstimator::new(),
            converter: TorqueConverter::new(),
            target_torque_x10: 0,
            max_available_x10: 0,
        }
    }

    /// Update the torque controller with current engine conditions
    ///
    /// # Arguments
    /// * `rpm` - Current engine RPM
    /// * `map_kpa_x10` - MAP sensor reading (kPa x10)
    /// * `iat_c` - Intake air temperature (Celsius)
    ///
    /// # Returns
    /// The arbitrated torque target (Nm x10)
    pub fn update(&mut self, rpm: u16, map_kpa_x10: u16, iat_c: i16) -> i16 {
        // Estimate maximum available torque
        self.max_available_x10 =
            self.estimator
                .estimate_max_torque(rpm, map_kpa_x10, iat_c, &self.config);

        // Arbitrate among all requests
        self.target_torque_x10 = self.arbiter.arbitrate(self.max_available_x10);

        self.target_torque_x10
    }

    /// Get actuator targets for the current torque target
    pub fn get_actuator_targets(&self, rpm: u16) -> ActuatorTargets {
        self.converter
            .convert(self.target_torque_x10, self.max_available_x10, rpm)
    }

    /// Submit a torque request
    pub fn request(&mut self, req: TorqueRequest) {
        self.arbiter.request(req);
    }

    /// Submit a driver torque request based on pedal position
    pub fn request_driver(&mut self, pedal_percent: u8, rpm: u16, now_us: u32) {
        let req =
            request::driver_torque_request(pedal_percent, rpm, self.max_available_x10, now_us);
        self.arbiter.request(req);
    }

    /// Submit an idle controller torque request
    pub fn request_idle(&mut self, target_rpm: u16, actual_rpm: u16, now_us: u32) {
        let req = request::idle_torque_request(
            target_rpm,
            actual_rpm,
            self.config.idle_torque_reserve_x10,
            now_us,
        );
        self.arbiter.request(req);
    }

    /// Submit a rev limiter torque request
    pub fn request_rev_limit(&mut self, limit_active: bool, now_us: u32) {
        let req = request::rev_limiter_torque_request(limit_active, now_us);
        self.arbiter.request(req);
    }

    /// Submit a limp mode torque request
    pub fn request_limp(&mut self, limp_active: bool, now_us: u32) {
        let req = request::limp_torque_request(limp_active, self.max_available_x10, now_us);
        self.arbiter.request(req);
    }

    /// Clear requests from a specific source
    pub fn clear_source(&mut self, source: TorqueSource) {
        self.arbiter.clear(source);
    }

    /// Reset all torque requests
    pub fn reset(&mut self) {
        self.arbiter.reset();
        self.target_torque_x10 = 0;
    }

    /// Check if any torque limiting is active
    pub fn is_limited(&self) -> bool {
        self.target_torque_x10 < self.max_available_x10
    }
}

impl Default for TorqueController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_torque_config_default() {
        let config = TorqueConfig::DEFAULT;
        assert!(config.enable);
        assert_eq!(config.max_torque_nm_x10, 2000);
        assert_eq!(config.peak_torque_rpm, 4000);
    }

    #[test]
    fn test_torque_controller_new() {
        let controller = TorqueController::new();
        assert_eq!(controller.target_torque_x10, 0);
        assert_eq!(controller.max_available_x10, 0);
    }

    #[test]
    fn test_torque_controller_update() {
        let mut controller = TorqueController::new();

        // First update to get max available torque
        controller.update(3000, 800, 25);
        assert!(controller.max_available_x10 > 0);

        // Now add a driver request (uses max_available)
        controller.request_driver(50, 3000, 0);

        // Update again with engine conditions
        let torque = controller.update(3000, 800, 25);

        // Should have a positive torque target (50% of max)
        assert!(torque > 0);
    }

    #[test]
    fn test_torque_limiting() {
        let mut controller = TorqueController::new();

        // Request full power
        controller.request_driver(100, 3000, 0);
        controller.update(3000, 800, 25);

        // Add rev limiter
        controller.request_rev_limit(true, 1000);
        let torque = controller.arbiter.arbitrate(controller.max_available_x10);

        // Rev limiter should override
        assert!(torque < controller.max_available_x10);
        assert!(controller.is_limited());
    }
}
