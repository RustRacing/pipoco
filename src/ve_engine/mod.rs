//! VE Engine - Volumetric Efficiency to IPW Calculation Module
//!
//! This is a **pure calculation module** that converts VE tables to IPW tables.
//! It maintains a baseline safe map and applies non-destructive transformations
//! commanded by the management engine.
//!
//! # Architecture Philosophy
//!
//! The VE engine is designed as a **stateful but safe** component:
//! - Maintains a baseline VE map (safe default)
//! - Accepts transformation commands (rich/lean/retard)
//! - Transformations are NON-DESTRUCTIVE (can be cleared)
//! - Always produces valid IPW output
//!
//! # Safety Model
//!
//! ```text
//! Baseline VE Map (80% VE, safe for all engines)
//!          ↓
//!   [Transformation Layer]
//!     - Emergency Rich: +20% fuel
//!     - Limp Mode: -10 degrees timing, -20% fuel
//!     - Cold Start: +50% fuel
//!     - Tuner Adjustment: Fine-tune specific cells
//!          ↓
//!   Final IPW Table → Injection Module
//! ```
//!
//! # Example Usage
//!
//! ```rust
//! use ecu_core::ve_engine::{VeEngine, VeCommand, SensorData, types::{InjectorConfig, VeTable, AfrTable}};
//!
//! let config = InjectorConfig { engine_displacement_cc: 2000, num_cylinders: 4, flow_rate_cc_min: 440, reference_pressure_kpa: 300, fuel_density_mg_cc: 750, dead_time_curve: [(90,1500),(100,1300),(110,1150),(120,1000),(130,900),(140,800),(150,750),(160,700)] };
//! let ve_tbl = VeTable::default_safe();
//! let afr_tbl = AfrTable::default_gasoline();
//! let mut ve = VeEngine::new_with(config, ve_tbl, afr_tbl);
//! let sensors = SensorData {
//!     timestamp_us: 1000,
//!     iat_celsius: 20,
//!     clt_celsius: 80,
//!     battery_voltage_mv: 13500,
//! };
//!
//! // Normal operation
//! let ipw_table = ve.calculate_ipw_table(&sensors);
//!
//! // Emergency: run rich to get to parking spot
//! ve.apply_command(VeCommand::EmergencyRich { percent: 20 });
//! let safe_ipw = ve.calculate_ipw_table(&sensors);
//!
//! // Clear emergency mode when safe
//! ve.clear_transformations();
//! ```

pub mod corrections;
pub mod injector;
pub mod interpolation;
pub mod speed_density;
pub mod types;

pub use types::*;

use crate::constants::fuel::{LOAD_BINS, RPM_BINS};

/// VE Engine - Main calculation engine with transformation support
///
/// Maintains a baseline VE map and applies non-destructive transformations.
pub struct VeEngine {
    /// Baseline VE map (80% VE - safe for most engines)
    baseline_ve: VeTable,

    /// Active transformation stack (applied in order)
    transformations: TransformationStack,

    /// Injector configuration
    injector_config: InjectorConfig,

    /// Target AFR table
    afr_table: AfrTable,

    /// Statistics
    calculation_count: u32,
}

impl VeEngine {
    /// Create a VE engine with explicit configuration (no defaults)
    pub fn new_with(config: InjectorConfig, baseline: VeTable, afr: AfrTable) -> Self {
        Self {
            baseline_ve: baseline,
            transformations: TransformationStack::new(),
            injector_config: config,
            afr_table: afr,
            calculation_count: 0,
        }
    }

    /// Calculate complete IPW table from VE table + transformations
    ///
    /// This is the main function - applies all active transformations
    /// to the baseline VE map and calculates IPW for each cell.
    ///
    /// # Arguments
    /// * `sensors` - Current sensor readings
    ///
    /// # Returns
    /// Complete 16x16 IPW table ready for injection module
    ///
    /// # Performance
    /// Target: <1ms on STM32F4 (168MHz)
    pub fn calculate_ipw_table(&mut self, sensors: &SensorData) -> IpwTable {
        let mut ipw_table = IpwTable {
            rpm_bins: RPM_BINS,
            load_bins: LOAD_BINS,
            values: [[0; 16]; 16],
        };

        // Calculate each cell
        for (load_idx, &load) in LOAD_BINS.iter().enumerate() {
            for (rpm_idx, &rpm) in RPM_BINS.iter().enumerate() {
                // Get base VE value
                let base_ve = self.baseline_ve.values[load_idx][rpm_idx];

                // Apply transformations
                let transformed_ve = self.transformations.apply_ve(base_ve, rpm, load);

                // Get target AFR
                let target_afr = self.afr_table.values[load_idx][rpm_idx];
                let transformed_afr = self.transformations.apply_afr(target_afr, rpm, load);

                // Calculate IPW for this cell
                ipw_table.values[load_idx][rpm_idx] =
                    self.calculate_ipw_cell(rpm, load, transformed_ve, transformed_afr, sensors);
            }
        }

        self.calculation_count += 1;
        ipw_table
    }

    /// Calculate IPW for a single cell
    ///
    /// This is the core calculation: VE% + Sensors → IPW (microseconds)
    ///
    /// # Steps
    /// 1. Calculate air mass using speed-density
    /// 2. Calculate fuel mass from AFR
    /// 3. Convert to pulse width using injector flow
    /// 4. Add injector dead time
    /// 5. Apply corrections (CLT, IAT, voltage)
    ///
    /// # Arguments
    /// * `rpm` - Engine speed
    /// * `load` - Engine load (kPa)
    /// * `ve_percent` - Volumetric efficiency (0-255%)
    /// * `target_afr` - Target AFR scaled by 10 (e.g., 147 = 14.7:1)
    /// * `sensors` - Current sensor readings
    ///
    /// # Returns
    /// Pulse width in microseconds
    pub fn calculate_ipw_cell(
        &self,
        _rpm: u16,
        load: u16,
        ve_percent: u8,
        target_afr: u16,
        sensors: &SensorData,
    ) -> u16 {
        // 1. Calculate air mass per cylinder per cycle (mg)
        let air_mass_mg = speed_density::calculate_air_mass(
            self.injector_config.engine_displacement_cc,
            self.injector_config.num_cylinders,
            load,
            ve_percent,
            sensors.iat_celsius,
        );

        // 2. Calculate required fuel mass (mg)
        let fuel_mass_mg = (air_mass_mg * 10) / target_afr as u32;

        // 3. Convert to pulse width
        let base_pw_us = injector::calculate_pulse_width(
            fuel_mass_mg,
            self.injector_config.flow_rate_cc_min,
            self.injector_config.fuel_density_mg_cc,
        );

        // 4. Add injector dead time
        let dead_time_us =
            injector::calculate_dead_time(sensors.battery_voltage_mv, &self.injector_config);
        let pw_with_deadtime = base_pw_us.saturating_add(dead_time_us as u32);

        // 5. Apply corrections
        let pw_corrected = corrections::apply_all_corrections(pw_with_deadtime, sensors);

        // 6. Clamp to valid range
        pw_corrected.clamp(500, 20000) as u16
    }

    /// Apply a transformation command from management engine
    ///
    /// Transformations are NON-DESTRUCTIVE and can be cleared.
    ///
    /// # Arguments
    /// * `command` - Transformation command to apply
    ///
    /// # Example
    /// ```rust
    /// use ecu_core::ve_engine::{VeEngine, VeCommand, types::{InjectorConfig, VeTable, AfrTable}};
    /// let config = InjectorConfig { engine_displacement_cc: 2000, num_cylinders: 4, flow_rate_cc_min: 440, reference_pressure_kpa: 300, fuel_density_mg_cc: 750, dead_time_curve: [(90,1500),(100,1300),(110,1150),(120,1000),(130,900),(140,800),(150,750),(160,700)] };
    /// let mut ve = VeEngine::new_with(config, VeTable::default_safe(), AfrTable::default_gasoline());
    /// ve.apply_command(VeCommand::EmergencyRich { percent: 20 });
    /// ve.apply_command(VeCommand::LimpMode);
    /// ```
    pub fn apply_command(&mut self, command: VeCommand) {
        self.transformations.push(command);
    }

    /// Clear all transformations (return to baseline map)
    pub fn clear_transformations(&mut self) {
        self.transformations.clear();
    }

    /// Clear specific transformation type
    pub fn clear_transformation_type(&mut self, trans_type: TransformationType) {
        self.transformations.remove_type(trans_type);
    }

    /// Update baseline VE table (from tuning software)
    ///
    /// This replaces the baseline map. Use carefully!
    pub fn update_ve_table(&mut self, new_ve: VeTable) {
        self.baseline_ve = new_ve;
    }

    /// Update injector configuration
    pub fn update_injector_config(&mut self, config: InjectorConfig) {
        self.injector_config = config;
    }

    /// Update AFR target table
    pub fn update_afr_table(&mut self, table: AfrTable) {
        self.afr_table = table;
    }

    /// Get current baseline VE table (read-only)
    pub fn baseline_ve(&self) -> &VeTable {
        &self.baseline_ve
    }

    /// Get active transformations (for diagnostics)
    pub fn active_transformations(&self) -> &[VeCommand] {
        self.transformations.list()
    }

    /// Get calculation statistics
    pub fn stats(&self) -> VeEngineStats {
        VeEngineStats {
            calculation_count: self.calculation_count,
            active_transformation_count: self.transformations.count() as u8,
        }
    }
}

/// Transformation stack - manages non-destructive modifications
///
/// Transformations are applied in FIFO order.
struct TransformationStack {
    stack: [Option<VeCommand>; 8], // Max 8 simultaneous transformations
    count: usize,
}

impl TransformationStack {
    fn new() -> Self {
        Self {
            stack: [None; 8],
            count: 0,
        }
    }

    /// Push a new transformation (replaces same type if exists)
    fn push(&mut self, command: VeCommand) {
        // Check if this type already exists
        let trans_type = command.transformation_type();

        // Remove old instance of same type
        self.remove_type(trans_type);

        // Add new transformation
        if self.count < 8 {
            self.stack[self.count] = Some(command);
            self.count += 1;
        }
    }

    /// Remove all transformations of a specific type
    fn remove_type(&mut self, trans_type: TransformationType) {
        let mut write_idx = 0;
        for read_idx in 0..self.count {
            if let Some(cmd) = self.stack[read_idx] {
                if cmd.transformation_type() != trans_type {
                    self.stack[write_idx] = Some(cmd);
                    write_idx += 1;
                }
            }
        }

        // Clear remaining slots
        for i in write_idx..self.count {
            self.stack[i] = None;
        }

        self.count = write_idx;
    }

    /// Clear all transformations
    fn clear(&mut self) {
        for i in 0..self.count {
            self.stack[i] = None;
        }
        self.count = 0;
    }

    /// Apply all transformations to VE value
    fn apply_ve(&self, mut base_ve: u8, rpm: u16, load: u16) -> u8 {
        for i in 0..self.count {
            if let Some(cmd) = self.stack[i] {
                base_ve = cmd.transform_ve(base_ve, rpm, load);
            }
        }
        base_ve
    }

    /// Apply all transformations to AFR value
    fn apply_afr(&self, mut base_afr: u16, rpm: u16, load: u16) -> u16 {
        for i in 0..self.count {
            if let Some(cmd) = self.stack[i] {
                base_afr = cmd.transform_afr(base_afr, rpm, load);
            }
        }
        base_afr
    }

    /// Get list of active transformations
    fn list(&self) -> &[VeCommand] {
        // This is a workaround since we can't return &[Option<VeCommand>]
        // In real implementation, we'd filter out Nones
        unsafe { core::slice::from_raw_parts(self.stack.as_ptr() as *const VeCommand, self.count) }
    }

    /// Count active transformations
    fn count(&self) -> usize {
        self.count
    }
}

/// VE engine statistics
#[derive(Debug, Clone, Copy)]
pub struct VeEngineStats {
    pub calculation_count: u32,
    pub active_transformation_count: u8,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ve_engine_creation() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let ve = VeEngine::new_with(
            config,
            VeTable::default_safe(),
            AfrTable::default_gasoline(),
        );
        assert_eq!(ve.calculation_count, 0);
        assert_eq!(ve.baseline_ve.values[0][0], 80); // Default 80% VE
    }

    #[test]
    fn test_transformation_stack() {
        let mut stack = TransformationStack::new();

        // Add emergency rich
        stack.push(VeCommand::EmergencyRich { percent: 20 });
        assert_eq!(stack.count(), 1);

        // Apply to VE value
        let base_ve = 80;
        let transformed = stack.apply_ve(base_ve, 3000, 100);
        assert!(transformed > base_ve); // Should be richer

        // Clear
        stack.clear();
        assert_eq!(stack.count(), 0);
    }

    #[test]
    fn test_transformation_replacement() {
        let mut stack = TransformationStack::new();

        // Add emergency rich 20%
        stack.push(VeCommand::EmergencyRich { percent: 20 });
        assert_eq!(stack.count(), 1);

        // Add another emergency rich 30% (should replace)
        stack.push(VeCommand::EmergencyRich { percent: 30 });
        assert_eq!(stack.count(), 1); // Still only 1
    }

    #[test]
    fn test_ipw_calculation_reasonable() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let ve = VeEngine::new_with(
            config,
            VeTable::default_safe(),
            AfrTable::default_gasoline(),
        );
        let sensors = SensorData {
            timestamp_us: 1000,
            iat_celsius: 20,
            clt_celsius: 80,
            battery_voltage_mv: 13500,
        };

        // Calculate IPW at 3000 RPM, 100 kPa
        let ipw = ve.calculate_ipw_cell(3000, 100, 80, 147, &sensors);

        // Should be reasonable value (500-20000 us)
        assert!(ipw >= 500);
        assert!(ipw <= 20000);

        // Typical value for this condition is around 2-4ms
        assert!(ipw > 1000);
        assert!(ipw < 5000);
    }

    #[test]
    fn test_emergency_rich_increases_fuel() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let mut ve = VeEngine::new_with(
            config,
            VeTable::default_safe(),
            AfrTable::default_gasoline(),
        );
        let sensors = SensorData {
            timestamp_us: 1000,
            iat_celsius: 20,
            clt_celsius: 80,
            battery_voltage_mv: 13500,
        };

        // Normal calculation - get table and check a cell
        let normal_table = ve.calculate_ipw_table(&sensors);
        let normal_ipw = normal_table.values[6][5]; // Arbitrary cell

        // Apply emergency rich
        ve.apply_command(VeCommand::EmergencyRich { percent: 20 });
        let rich_table = ve.calculate_ipw_table(&sensors);
        let rich_ipw = rich_table.values[6][5]; // Same cell

        // Rich should give more fuel
        assert!(
            rich_ipw > normal_ipw,
            "rich: {rich_ipw}, normal: {normal_ipw}"
        );
    }

    #[test]
    fn test_clear_transformations() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let mut ve = VeEngine::new_with(
            config,
            VeTable::default_safe(),
            AfrTable::default_gasoline(),
        );
        let sensors = SensorData {
            timestamp_us: 1000,
            iat_celsius: 20,
            clt_celsius: 80,
            battery_voltage_mv: 13500,
        };

        let normal_table = ve.calculate_ipw_table(&sensors);
        let normal_ipw = normal_table.values[6][5];

        // Apply transformation
        ve.apply_command(VeCommand::EmergencyRich { percent: 20 });
        let transformed_table = ve.calculate_ipw_table(&sensors);
        let transformed_ipw = transformed_table.values[6][5];
        assert_ne!(
            normal_ipw, transformed_ipw,
            "normal: {normal_ipw}, transformed: {transformed_ipw}"
        );

        // Clear and verify back to normal
        ve.clear_transformations();
        let cleared_table = ve.calculate_ipw_table(&sensors);
        let cleared_ipw = cleared_table.values[6][5];
        assert_eq!(
            normal_ipw, cleared_ipw,
            "normal: {normal_ipw}, cleared: {cleared_ipw}"
        );
    }
}
