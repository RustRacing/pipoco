use super::EcuState;
use crate::constants::fuel::DEFAULT_PULSE_WIDTH_US;
use crate::{constants, ignition, rev_limiter};

impl EcuState {
    pub fn calculate_ignition_timing(&self, rpm: u16, load: u16) -> i16 {
        let table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.config.ignition_table,
        };

        // 1. Base lookup
        let base_timing = table.lookup(rpm, load);

        // 2. Apply corrections and clamp
        ignition::calculate_timing(base_timing, self.ignition_corrections())
    }

    /// Calculate coil dwell time based on battery voltage
    ///
    /// # Returns
    /// Dwell time in microseconds
    pub fn calculate_dwell(&self) -> u32 {
        ignition::calculate_dwell(self.battery_voltage_mv())
    }

    /// Initialize IPW table with linear test values
    ///
    /// Creates a simple linear fuel map for initial testing.
    /// More fuel at higher load, slightly less at higher RPM.
    ///
    /// This is a helper method for hardware testing. Real tuning data
    /// should be loaded from external storage or CAN.
    pub fn init_linear_table(&mut self) {
        for row in 0..16 {
            for col in 0..16 {
                let base = DEFAULT_PULSE_WIDTH_US;
                let load_factor = (row as u16).saturating_mul(50); // 0-750us
                let rpm_factor = (col as u16).saturating_mul(10); // 0-150us

                // More fuel at higher load, slightly less at higher RPM
                self.config.ipw_table[row][col] =
                    base.saturating_add(load_factor).saturating_sub(rpm_factor);
            }
        }
    }

    /// Initialize ignition table with conservative values
    ///
    /// Creates a conservative ignition map safe for initial testing.
    /// Should be replaced with properly tuned values for production.
    pub fn init_ignition_table(&mut self) {
        let mut table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.config.ignition_table,
        };

        ignition::init_conservative_table(&mut table);
        self.config.ignition_table = table.values;
    }

    /// Update rev limiter state based on current RPM
    ///
    /// Should be called every engine cycle or in main loop.
    /// Updates internal limiter state which affects fuel and ignition.
    pub fn update_rev_limiter(&mut self) {
        let config = *self.rev_limiter_config();
        rev_limiter::update_limiter(
            self.trigger_inputs().rpm,
            &config,
            &mut self.rev_limiter_state,
        );
    }

    /// Check if fuel injection should proceed (considers rev limiter)
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should occur, `false` if limiter is cutting fuel
    pub fn should_inject_fuel(&self, cylinder: u8) -> bool {
        rev_limiter::should_inject(&self.rev_limiter_state, cylinder)
    }

    /// Calculate ignition timing with all corrections (including rev limiter)
    ///
    /// This is the main method to use - applies ignition corrections AND rev limiter retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter(&self, rpm: u16, load: u16) -> i16 {
        self.calculate_ignition_timing_with_limiter_cyl(rpm, load, 0)
    }

    /// Calculate ignition timing with all corrections (including rev limiter and knock)
    ///
    /// This is the main method to use - applies ignition corrections, rev limiter retard,
    /// and per-cylinder knock retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    /// * `cylinder` - Cylinder number (0-7) for per-cylinder knock retard
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter_cyl(
        &self,
        rpm: u16,
        load: u16,
        cylinder: u8,
    ) -> i16 {
        // Get base timing with normal corrections
        let base_timing = self.calculate_ignition_timing(rpm, load);

        // Apply rev limiter retard
        let with_limiter = rev_limiter::apply_limiter_retard(base_timing, &self.rev_limiter_state);

        // Apply knock retard (returns negative value)
        let knock_retard = self.knock_controller.get_retard_degrees(cylinder);
        (with_limiter + knock_retard).max(constants::ignition::MIN_TIMING_BTDC)
    }
}
