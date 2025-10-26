//! VE Engine Types
//!
//! Data structures for VE→IPW calculations and transformations.

#![cfg_attr(not(test), no_std)]

/// VE Table - Volumetric Efficiency map
///
/// Values represent how efficiently the engine fills cylinders with air.
/// VE% = (actual air mass / theoretical air mass) * 100
#[derive(Debug, Clone, Copy)]
pub struct VeTable {
    /// RPM bins (x-axis)
    pub rpm_bins: [u16; 16],
    /// Load bins (y-axis) in kPa
    pub load_bins: [u16; 16],
    /// VE values (0-255%) - [load_idx][rpm_idx]
    pub values: [[u8; 16]; 16],
}

impl VeTable {
    /// Create a safe default VE table (80% VE across the board)
    ///
    /// This is conservative and will run on most engines without damage.
    /// Should be replaced with properly tuned values.
    pub fn default_safe() -> Self {
        Self {
            rpm_bins: [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
                      4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000],
            load_bins: [20, 30, 40, 50, 60, 70, 80, 90,
                       100, 110, 120, 130, 140, 150, 160, 170],
            values: [[80; 16]; 16],  // 80% VE everywhere
        }
    }

    /// Lookup VE value (no interpolation for now)
    pub fn lookup(&self, rpm: u16, load_kpa: u16) -> u8 {
        let rpm_idx = self.find_rpm_bin(rpm);
        let load_idx = self.find_load_bin(load_kpa);
        self.values[load_idx][rpm_idx]
    }

    fn find_rpm_bin(&self, rpm: u16) -> usize {
        for i in 0..15 {
            if rpm >= self.rpm_bins[i] && rpm < self.rpm_bins[i + 1] {
                return i;
            }
        }
        15  // Last bin
    }

    fn find_load_bin(&self, load: u16) -> usize {
        for i in 0..15 {
            if load >= self.load_bins[i] && load < self.load_bins[i + 1] {
                return i;
            }
        }
        15  // Last bin
    }
}

/// AFR Target Table - desired air-fuel ratio map
///
/// AFR varies by operating condition:
/// - Rich at high load (12.5:1) for power
/// - Stoich at cruise (14.7:1) for efficiency
/// - Lean at light load (15.5:1) for economy
#[derive(Debug, Clone, Copy)]
pub struct AfrTable {
    /// RPM bins (same as VE table)
    pub rpm_bins: [u16; 16],
    /// Load bins (same as VE table)
    pub load_bins: [u16; 16],
    /// AFR values scaled by 10 - [load_idx][rpm_idx]
    /// e.g., 147 = 14.7:1
    pub values: [[u16; 16]; 16],
}

impl AfrTable {
    /// Create typical AFR table for gasoline
    pub fn default_gasoline() -> Self {
        let rpm_bins = [500, 1000, 1500, 2000, 2500, 3000, 3500, 4000,
                       4500, 5000, 5500, 6000, 6500, 7000, 7500, 8000];
        let load_bins = [20, 30, 40, 50, 60, 70, 80, 90,
                        100, 110, 120, 130, 140, 150, 160, 170];

        let mut values = [[147u16; 16]; 16];  // Default to stoich

        // Set AFR based on load
        for (load_idx, &load) in load_bins.iter().enumerate() {
            for rpm_idx in 0..16 {
                values[load_idx][rpm_idx] = if load >= 120 {
                    125  // Rich for power (12.5:1)
                } else if load >= 60 {
                    147  // Stoich (14.7:1)
                } else {
                    155  // Lean for economy (15.5:1)
                };
            }
        }

        Self {
            rpm_bins,
            load_bins,
            values,
        }
    }
}

/// IPW Table - Injector Pulse Width output table
///
/// This is the final output sent to the injection module.
#[derive(Debug, Clone, Copy)]
pub struct IpwTable {
    /// RPM bins
    pub rpm_bins: [u16; 16],
    /// Load bins
    pub load_bins: [u16; 16],
    /// Pulse width values in microseconds - [load_idx][rpm_idx]
    pub values: [[u16; 16]; 16],
}

/// Injector configuration
///
/// Characterizes injector flow and dead time.
#[derive(Debug, Clone, Copy)]
pub struct InjectorConfig {
    /// Engine displacement in cc
    pub engine_displacement_cc: u16,
    /// Number of cylinders
    pub num_cylinders: u8,
    /// Injector flow rate at reference pressure (cc/min)
    pub flow_rate_cc_min: u16,
    /// Reference fuel pressure (kPa)
    pub reference_pressure_kpa: u16,
    /// Fuel density (mg/cc) - gasoline ~750 mg/cc
    pub fuel_density_mg_cc: u16,
    /// Dead time vs voltage curve (voltage*10, time_us)
    pub dead_time_curve: [(u8, u16); 8],
}

impl InjectorConfig {
    /// Generic injector configuration (440cc @ 3 bar, 4-cyl 2.0L)
    pub fn default_generic() -> Self {
        Self {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),   // 9V: 1.5ms
                (100, 1300),  // 10V: 1.3ms
                (110, 1150),  // 11V: 1.15ms
                (120, 1000),  // 12V: 1.0ms
                (130, 900),   // 13V: 0.9ms
                (140, 800),   // 14V: 0.8ms
                (150, 750),   // 15V: 0.75ms
                (160, 700),   // 16V: 0.7ms
            ],
        }
    }
}

/// Sensor data input for calculations
#[derive(Debug, Clone, Copy)]
pub struct SensorData {
    /// Timestamp (microseconds)
    pub timestamp_us: u32,
    /// Intake air temperature (°C)
    pub iat_celsius: i16,
    /// Coolant temperature (°C)
    pub clt_celsius: i16,
    /// Battery voltage (millivolts)
    pub battery_voltage_mv: u16,
}

/// VE Command - transformation instructions from management engine
///
/// These commands modify the VE engine behavior WITHOUT destroying the baseline map.
/// All transformations are reversible and can be cleared.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum VeCommand {
    /// Emergency rich mode - add fuel percentage
    ///
    /// Use when: Engine overheating, knock detected, sensor failure
    /// Effect: Increases fuel delivery to cool engine/prevent damage
    ///
    /// # Example
    /// ```
    /// use ecu_core::ve_engine::VeCommand;
    ///
    /// let cmd = VeCommand::EmergencyRich { percent: 20 };  // Add 20% fuel
    /// ```
    EmergencyRich {
        percent: u8,  // 0-100% additional fuel
    },

    /// Emergency lean mode - reduce fuel percentage
    ///
    /// Use when: Fuel pressure too high, need to conserve fuel
    /// Effect: Reduces fuel delivery
    EmergencyLean {
        percent: u8,  // 0-100% fuel reduction
    },

    /// Limp mode - run engine at minimal safe level
    ///
    /// Use when: Critical sensor failure, getting to parking spot
    /// Effect: Retards timing, reduces fuel to minimum safe operation
    LimpMode,

    /// Cold start enrichment override
    ///
    /// Use when: Additional cold start enrichment needed
    /// Effect: Adds fuel at low temperatures
    ColdStart {
        enrichment_percent: u8,
        max_clt_celsius: i16,  // Apply only below this temperature
    },

    /// Global fuel trim (fine tuning)
    ///
    /// Use when: Tuner wants global adjustment without changing VE map
    /// Effect: Multiplies all fuel by this factor
    GlobalFuelTrim {
        multiplier_percent: u8,  // 100 = 1.0x (no change)
    },

    /// Regional fuel trim (specific RPM/load range)
    ///
    /// Use when: Need to adjust specific area of map
    /// Effect: Adjusts fuel only in specified region
    RegionalTrim {
        rpm_min: u16,
        rpm_max: u16,
        load_min: u16,
        load_max: u16,
        trim_percent: i8,  // -50 to +50%
    },

    /// Altitude compensation override
    ///
    /// Use when: Rapid altitude changes, barometric sensor failure
    /// Effect: Adjusts fuel for altitude
    AltitudeCompensation {
        altitude_m: i16,  // Meters above sea level
    },

    /// Fuel cut (complete shutdown)
    ///
    /// Use when: Overrev, critical failure
    /// Effect: Zero fuel delivery
    FuelCut,
}

impl VeCommand {
    /// Get transformation type (for deduplication)
    pub fn transformation_type(&self) -> TransformationType {
        match self {
            VeCommand::EmergencyRich { .. } => TransformationType::EmergencyRich,
            VeCommand::EmergencyLean { .. } => TransformationType::EmergencyLean,
            VeCommand::LimpMode => TransformationType::LimpMode,
            VeCommand::ColdStart { .. } => TransformationType::ColdStart,
            VeCommand::GlobalFuelTrim { .. } => TransformationType::GlobalTrim,
            VeCommand::RegionalTrim { .. } => TransformationType::RegionalTrim,
            VeCommand::AltitudeCompensation { .. } => TransformationType::AltitudeCompensation,
            VeCommand::FuelCut => TransformationType::FuelCut,
        }
    }

    /// Transform VE value
    pub fn transform_ve(&self, base_ve: u8, rpm: u16, load: u16) -> u8 {
        match self {
            VeCommand::EmergencyRich { percent } => {
                // Increase VE (more air = more fuel)
                let increase = (base_ve as u16 * *percent as u16) / 100;
                base_ve.saturating_add(increase as u8)
            }

            VeCommand::EmergencyLean { percent } => {
                // Decrease VE
                let decrease = (base_ve as u16 * *percent as u16) / 100;
                base_ve.saturating_sub(decrease as u8)
            }

            VeCommand::LimpMode => {
                // Reduce VE to 60% of normal (very conservative)
                ((base_ve as u16 * 60) / 100) as u8
            }

            VeCommand::ColdStart { enrichment_percent, .. } => {
                // Increase VE for cold start
                let increase = (base_ve as u16 * *enrichment_percent as u16) / 100;
                base_ve.saturating_add(increase as u8)
            }

            VeCommand::GlobalFuelTrim { multiplier_percent } => {
                // Apply global multiplier
                ((base_ve as u16 * *multiplier_percent as u16) / 100) as u8
            }

            VeCommand::RegionalTrim { rpm_min, rpm_max, load_min, load_max, trim_percent } => {
                // Check if in region
                if rpm >= *rpm_min && rpm <= *rpm_max && load >= *load_min && load <= *load_max {
                    if *trim_percent >= 0 {
                        let increase = (base_ve as u16 * *trim_percent as u16) / 100;
                        base_ve.saturating_add(increase as u8)
                    } else {
                        let decrease = (base_ve as u16 * (-*trim_percent) as u16) / 100;
                        base_ve.saturating_sub(decrease as u8)
                    }
                } else {
                    base_ve
                }
            }

            VeCommand::AltitudeCompensation { altitude_m } => {
                // Reduce VE at altitude (less air density)
                // Rule of thumb: -3% VE per 1000m
                let reduction_percent = (*altitude_m / 1000) * 3;
                let reduction = (base_ve as u16 * reduction_percent.max(0) as u16) / 100;
                base_ve.saturating_sub(reduction as u8)
            }

            VeCommand::FuelCut => {
                // Zero fuel
                0
            }
        }
    }

    /// Transform AFR value (some commands affect AFR directly)
    pub fn transform_afr(&self, base_afr: u16, _rpm: u16, _load: u16) -> u16 {
        match self {
            VeCommand::EmergencyRich { percent } => {
                // Lower AFR (richer)
                let decrease = (base_afr as u32 * *percent as u32) / 100;
                base_afr.saturating_sub(decrease as u16)
            }

            VeCommand::EmergencyLean { percent } => {
                // Raise AFR (leaner)
                let increase = (base_afr as u32 * *percent as u32) / 100;
                base_afr.saturating_add(increase as u16)
            }

            VeCommand::LimpMode => {
                // Run slightly rich in limp mode (safer)
                135  // 13.5:1 AFR
            }

            VeCommand::FuelCut => {
                // Infinite AFR (no fuel)
                u16::MAX
            }

            _ => base_afr,  // Other commands don't affect AFR
        }
    }
}

/// Transformation type (for deduplication)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TransformationType {
    EmergencyRich,
    EmergencyLean,
    LimpMode,
    ColdStart,
    GlobalTrim,
    RegionalTrim,
    AltitudeCompensation,
    FuelCut,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ve_table_default() {
        let table = VeTable::default_safe();
        assert_eq!(table.values[0][0], 80);
        assert_eq!(table.values[15][15], 80);
    }

    #[test]
    fn test_ve_table_lookup() {
        let table = VeTable::default_safe();
        let ve = table.lookup(3000, 100);
        assert_eq!(ve, 80);
    }

    #[test]
    fn test_afr_table_default() {
        let table = AfrTable::default_gasoline();

        // Light load: lean
        assert_eq!(table.values[0][5], 155);  // 15.5:1

        // Medium load: stoich
        assert_eq!(table.values[5][5], 147);  // 14.7:1

        // High load: rich
        assert_eq!(table.values[10][5], 125);  // 12.5:1
    }

    #[test]
    fn test_emergency_rich_transform() {
        let cmd = VeCommand::EmergencyRich { percent: 20 };
        let base_ve = 80;
        let transformed = cmd.transform_ve(base_ve, 3000, 100);

        // Should add 20% fuel
        assert_eq!(transformed, 96);  // 80 + 16
    }

    #[test]
    fn test_limp_mode_transform() {
        let cmd = VeCommand::LimpMode;
        let base_ve = 100;
        let transformed = cmd.transform_ve(base_ve, 3000, 100);

        // Should be 60% of original
        assert_eq!(transformed, 60);
    }

    #[test]
    fn test_fuel_cut_transform() {
        let cmd = VeCommand::FuelCut;
        let base_ve = 80;
        let transformed = cmd.transform_ve(base_ve, 3000, 100);

        // Should be zero
        assert_eq!(transformed, 0);
    }

    #[test]
    fn test_regional_trim_in_range() {
        let cmd = VeCommand::RegionalTrim {
            rpm_min: 2000,
            rpm_max: 4000,
            load_min: 80,
            load_max: 120,
            trim_percent: 10,
        };

        let base_ve = 80;

        // Inside range
        let transformed = cmd.transform_ve(base_ve, 3000, 100);
        assert_eq!(transformed, 88);  // +10%

        // Outside range
        let transformed = cmd.transform_ve(base_ve, 1000, 100);
        assert_eq!(transformed, 80);  // No change
    }

    #[test]
    fn test_altitude_compensation() {
        let cmd = VeCommand::AltitudeCompensation { altitude_m: 2000 };
        let base_ve = 100;
        let transformed = cmd.transform_ve(base_ve, 3000, 100);

        // Should reduce by ~6% (3% per 1000m)
        assert_eq!(transformed, 94);
    }
}
