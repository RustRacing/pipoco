//! Torque-to-Actuator Conversion
//!
//! Converts torque targets to actual engine control actuator commands:
//! fuel multiplier, timing offset, and throttle position.

/// Actuator targets derived from torque request
#[derive(Debug, Clone, Copy, Default)]
pub struct ActuatorTargets {
    /// Fuel multiplier (0-200 where 100 = 1.0x, 0 = fuel cut)
    pub fuel_mult_x100: u8,
    /// Timing offset from table (degrees x10, negative = retard)
    pub timing_offset_x10: i16,
    /// Electronic throttle target (0-100%, future use)
    pub throttle_target: u8,
    /// Is fuel cut active?
    pub fuel_cut: bool,
    /// Is timing being retarded for torque reduction?
    pub timing_reduced: bool,
}

impl ActuatorTargets {
    /// No intervention targets
    pub const fn none() -> Self {
        Self {
            fuel_mult_x100: 100,
            timing_offset_x10: 0,
            throttle_target: 100,
            fuel_cut: false,
            timing_reduced: false,
        }
    }
}

/// Configuration for torque-to-actuator conversion
#[derive(Debug, Clone, Copy)]
pub struct ActuatorConfig {
    /// Prefer timing retard over fuel cut for emissions
    pub prefer_timing: bool,
    /// Maximum timing retard for torque reduction (degrees x10)
    pub max_timing_retard_x10: i16,
    /// Minimum fuel multiplier before cutting fuel
    pub min_fuel_mult_x100: u8,
    /// Enable electronic throttle control
    pub enable_throttle: bool,
}

impl ActuatorConfig {
    pub const DEFAULT: Self = Self {
        prefer_timing: true,
        max_timing_retard_x10: 150, // 15 degrees
        min_fuel_mult_x100: 70,     // 70% minimum before cut
        enable_throttle: false,     // Not implemented yet
    };
}

impl Default for ActuatorConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Torque-to-actuator converter
#[derive(Debug, Clone, Copy, Default)]
pub struct TorqueConverter {
    pub config: ActuatorConfig,
}

impl TorqueConverter {
    pub const fn new() -> Self {
        Self {
            config: ActuatorConfig::DEFAULT,
        }
    }

    /// Convert torque target to actuator commands
    ///
    /// Uses a coordinated strategy:
    /// 1. For small reductions: use timing retard (better for emissions)
    /// 2. For medium reductions: combine timing + fuel reduction
    /// 3. For large reductions: fuel cut
    ///
    /// # Arguments
    /// * `target_x10` - Target torque (Nm x10)
    /// * `max_x10` - Maximum available torque (Nm x10)
    /// * `_rpm` - Current RPM (for future use)
    ///
    /// # Returns
    /// Actuator targets to achieve the requested torque
    pub fn convert(
        &self,
        target_x10: i16,
        max_x10: i16,
        _rpm: u16,
    ) -> ActuatorTargets {
        // Handle edge cases
        if max_x10 <= 0 {
            return ActuatorTargets {
                fuel_mult_x100: 0,
                timing_offset_x10: 0,
                throttle_target: 0,
                fuel_cut: true,
                timing_reduced: false,
            };
        }

        // Calculate required torque reduction
        let reduction_percent = if target_x10 >= max_x10 {
            0i32
        } else if target_x10 <= 0 {
            100i32
        } else {
            ((max_x10 as i32 - target_x10 as i32) * 100) / max_x10 as i32
        };

        // No reduction needed
        if reduction_percent == 0 {
            return ActuatorTargets::none();
        }

        // Full fuel cut for 100% reduction
        if reduction_percent >= 100 {
            return ActuatorTargets {
                fuel_mult_x100: 0,
                timing_offset_x10: 0,
                throttle_target: 0,
                fuel_cut: true,
                timing_reduced: false,
            };
        }

        // Coordinate reduction between timing and fuel
        let mut targets = ActuatorTargets::none();

        if self.config.prefer_timing {
            // Strategy: timing first, then fuel
            // ~2% torque reduction per degree of retard
            let timing_capacity = (self.config.max_timing_retard_x10 as i32 * 2) / 10; // % reduction possible

            if reduction_percent as i32 <= timing_capacity {
                // Timing alone can handle it
                // reduction_percent = timing_retard * 2 / 10
                // timing_retard = reduction_percent * 10 / 2 = reduction_percent * 5
                let timing_retard = (reduction_percent * 5) as i16;
                targets.timing_offset_x10 = -timing_retard.min(self.config.max_timing_retard_x10);
                targets.timing_reduced = true;
            } else {
                // Use max timing retard
                targets.timing_offset_x10 = -self.config.max_timing_retard_x10;
                targets.timing_reduced = true;

                // Remaining reduction via fuel
                let remaining = reduction_percent as i32 - timing_capacity;

                // Fuel multiplier: 100 - remaining, but not below min
                let fuel_mult = (100 - remaining).clamp(
                    self.config.min_fuel_mult_x100 as i32,
                    100,
                ) as u8;
                targets.fuel_mult_x100 = fuel_mult;

                // If still not enough, fuel cut
                if fuel_mult <= self.config.min_fuel_mult_x100 && remaining > 30 {
                    targets.fuel_mult_x100 = 0;
                    targets.fuel_cut = true;
                }
            }
        } else {
            // Strategy: direct fuel reduction
            let fuel_mult = (100 - reduction_percent as i32).clamp(0, 100) as u8;
            targets.fuel_mult_x100 = fuel_mult;

            if fuel_mult == 0 {
                targets.fuel_cut = true;
            }
        }

        targets
    }

    /// Apply actuator targets to fuel pulse width
    pub fn apply_to_fuel(&self, base_pw_us: u16, targets: &ActuatorTargets) -> u16 {
        if targets.fuel_cut || targets.fuel_mult_x100 == 0 {
            return 0;
        }

        let result = (base_pw_us as u32 * targets.fuel_mult_x100 as u32) / 100;
        result.min(u16::MAX as u32) as u16
    }

    /// Apply actuator targets to ignition timing
    pub fn apply_to_timing(&self, base_timing: i16, targets: &ActuatorTargets) -> i16 {
        base_timing + targets.timing_offset_x10 / 10
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_actuator_targets_none() {
        let targets = ActuatorTargets::none();
        assert_eq!(targets.fuel_mult_x100, 100);
        assert_eq!(targets.timing_offset_x10, 0);
        assert!(!targets.fuel_cut);
        assert!(!targets.timing_reduced);
    }

    #[test]
    fn test_converter_no_reduction() {
        let converter = TorqueConverter::new();
        let targets = converter.convert(2000, 2000, 3000);

        assert_eq!(targets.fuel_mult_x100, 100);
        assert_eq!(targets.timing_offset_x10, 0);
        assert!(!targets.fuel_cut);
    }

    #[test]
    fn test_converter_full_reduction() {
        let converter = TorqueConverter::new();
        let targets = converter.convert(0, 2000, 3000);

        assert!(targets.fuel_cut);
        assert_eq!(targets.fuel_mult_x100, 0);
    }

    #[test]
    fn test_converter_small_reduction_timing() {
        let mut converter = TorqueConverter::new();
        converter.config.prefer_timing = true;
        converter.config.max_timing_retard_x10 = 150;

        // 10% reduction (should be timing only)
        let targets = converter.convert(1800, 2000, 3000);

        assert!(targets.timing_reduced);
        assert!(targets.timing_offset_x10 < 0); // Retard (negative)
        assert_eq!(targets.fuel_mult_x100, 100); // No fuel reduction
        assert!(!targets.fuel_cut);
    }

    #[test]
    fn test_converter_large_reduction_combined() {
        let mut converter = TorqueConverter::new();
        converter.config.prefer_timing = true;
        converter.config.max_timing_retard_x10 = 100; // 10 degrees = ~20% capacity

        // 40% reduction (needs timing + fuel)
        let targets = converter.convert(1200, 2000, 3000);

        assert!(targets.timing_reduced);
        assert_eq!(targets.timing_offset_x10, -100); // Max retard
        assert!(targets.fuel_mult_x100 < 100); // Some fuel reduction
    }

    #[test]
    fn test_converter_fuel_only() {
        let mut converter = TorqueConverter::new();
        converter.config.prefer_timing = false;

        // 20% reduction
        let targets = converter.convert(1600, 2000, 3000);

        assert!(!targets.timing_reduced);
        assert_eq!(targets.fuel_mult_x100, 80);
    }

    #[test]
    fn test_apply_to_fuel() {
        let converter = TorqueConverter::new();

        let targets = ActuatorTargets {
            fuel_mult_x100: 80,
            ..ActuatorTargets::none()
        };

        let result = converter.apply_to_fuel(1000, &targets);
        assert_eq!(result, 800);
    }

    #[test]
    fn test_apply_to_fuel_cut() {
        let converter = TorqueConverter::new();

        let targets = ActuatorTargets {
            fuel_mult_x100: 0,
            fuel_cut: true,
            ..ActuatorTargets::none()
        };

        let result = converter.apply_to_fuel(1000, &targets);
        assert_eq!(result, 0);
    }

    #[test]
    fn test_apply_to_timing() {
        let converter = TorqueConverter::new();

        let targets = ActuatorTargets {
            timing_offset_x10: -50, // -5 degrees
            ..ActuatorTargets::none()
        };

        let result = converter.apply_to_timing(20, &targets);
        assert_eq!(result, 15); // 20 - 5 = 15
    }

    #[test]
    fn test_converter_negative_max() {
        let converter = TorqueConverter::new();

        // Edge case: max_x10 <= 0
        let targets = converter.convert(100, 0, 3000);
        assert!(targets.fuel_cut);
    }

    #[test]
    fn test_converter_target_exceeds_max() {
        let converter = TorqueConverter::new();

        // Target > max (shouldn't happen, but handle gracefully)
        let targets = converter.convert(3000, 2000, 3000);

        // Should result in no reduction
        assert_eq!(targets.fuel_mult_x100, 100);
        assert!(!targets.fuel_cut);
    }
}
