use super::EcuState;
use crate::units::Micros;
use crate::{diag, torque};

impl EcuState {
    pub fn update_ltft(&mut self, clt_c: i16, now_us: u32) -> i16 {
        let trigger_inputs = self.trigger_inputs();
        let map_kpa_x10 = self.map_kpa_x10();
        let stft_x10 = self.stft_x10();
        let lambda_active = self.lambda_state.active;
        self.ltft_manager_mut().update(
            trigger_inputs.rpm,
            map_kpa_x10,
            clt_c,
            stft_x10,
            lambda_active,
            now_us,
        )
    }

    /// Get combined fuel trim (STFT + LTFT)
    ///
    /// # Returns
    /// Combined fuel trim (percent x10), clamped to ±20%
    pub fn get_total_fuel_trim(&self) -> i16 {
        self.ltft_manager().get_total_trim(
            self.stft_x10(),
            self.trigger_inputs().rpm,
            self.map_kpa_x10(),
        )
    }

    /// Reset LTFT learning
    ///
    /// Clears all learned values. Use via TunerStudio command or after
    /// major engine changes that invalidate learned data.
    pub fn reset_ltft(&mut self) {
        self.ltft_manager_mut().reset();
    }

    /// Check if LTFT learning is currently active
    pub fn is_ltft_learning(&self) -> bool {
        self.ltft_manager().state.learning_active
    }

    /// Get number of LTFT cells that have been learned
    pub fn ltft_learned_cell_count(&self) -> u8 {
        self.ltft_manager().table.learned_cell_count()
    }

    /// Process a knock sensor sample
    ///
    /// Call this during the knock window with the current sensor reading.
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder index (0-7)
    /// * `level` - Knock sensor reading
    /// * `clt_c` - Coolant temperature in Celsius
    /// * `now_us` - Current timestamp
    ///
    /// # Returns
    /// `true` if knock was detected
    pub fn process_knock_sample(
        &mut self,
        cylinder: u8,
        level: u16,
        clt_c: i16,
        now_us: u32,
    ) -> bool {
        let detected = self.knock_controller.process(
            cylinder,
            level,
            self.trigger_inputs().rpm,
            clt_c,
            now_us,
        );

        // Log knock event to diagnostics
        if detected {
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::KnockDetected,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Sensor,
                context: Some(cylinder as u32),
                start_us: now_us,
                end_us: 0,
            });
        }

        detected
    }

    /// Update knock timing recovery
    ///
    /// Call this periodically (e.g., every 100ms) to allow timing recovery.
    pub fn update_knock_recovery(&mut self, now_us: u32) {
        self.knock_controller.update_recovery(now_us);
    }

    /// Reset knock controller state
    pub fn reset_knock(&mut self) {
        self.knock_controller.reset();
    }

    /// Check if any knock retard is active
    pub fn has_knock_retard(&self) -> bool {
        self.knock_controller
            .state
            .has_retard(&self.knock_controller.config)
    }

    /// Get total knock count across all cylinders
    pub fn total_knock_count(&self) -> u32 {
        self.knock_controller.state.total_knock_count
    }

    /// Update torque controller with current conditions
    ///
    /// Call this periodically (e.g., in main loop) to update torque arbitration.
    ///
    /// # Arguments
    /// * `iat_c` - Intake air temperature in Celsius
    ///
    /// # Returns
    /// Arbitrated torque target (Nm x10)
    pub fn update_torque(&mut self, iat_c: i16) -> i16 {
        self.torque_controller
            .update(self.trigger_inputs().rpm, self.map_kpa_x10(), iat_c)
    }

    /// Submit a driver torque request based on pedal position
    ///
    /// # Arguments
    /// * `pedal_percent` - Accelerator pedal position (0-100%)
    /// * `now_us` - Current timestamp
    pub fn request_driver_torque(&mut self, pedal_percent: u8, now_us: u32) {
        self.torque_controller
            .request_driver(pedal_percent, self.trigger_inputs().rpm, now_us);
    }

    /// Submit an idle controller torque request
    ///
    /// # Arguments
    /// * `target_rpm` - Target idle RPM
    /// * `now_us` - Current timestamp
    pub fn request_idle_torque(&mut self, target_rpm: u16, now_us: u32) {
        self.torque_controller
            .request_idle(target_rpm, self.trigger_inputs().rpm, now_us);
    }

    /// Submit torque limits based on current safety states
    ///
    /// Call this after updating safety monitors to apply torque limits.
    pub fn apply_safety_torque_limits(&mut self, now_us: u32) {
        // Rev limiter
        let rev_limited = self.rev_limiter_state.active;
        self.torque_controller
            .request_rev_limit(rev_limited, now_us);

        // Limp mode from voltage or load failure
        let limp_active = self.voltage_monitor.limp_active || self.load_failure_tracker.in_limp;
        self.torque_controller.request_limp(limp_active, now_us);
    }

    /// Get actuator targets from torque controller
    ///
    /// Returns fuel/timing modifications to achieve torque target.
    pub fn get_torque_actuators(&self) -> torque::ActuatorTargets {
        self.torque_controller
            .get_actuator_targets(self.trigger_inputs().rpm)
    }

    /// Check if torque is being limited
    pub fn is_torque_limited(&self) -> bool {
        self.torque_controller.is_limited()
    }

    /// Reset torque controller
    pub fn reset_torque(&mut self) {
        self.torque_controller.reset();
    }
}
