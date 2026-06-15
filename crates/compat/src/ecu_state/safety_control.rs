use super::EcuState;
use crate::units::Micros;
use crate::{diag, safety, sensors};

impl EcuState {
    pub fn update_flood_clear(&mut self) -> bool {
        safety::update_flood_clear(
            self.trigger_inputs().rpm,
            self.tps_percent,
            &mut self.flood_clear_state,
        )
    }

    /// Record a sync loss event
    ///
    /// Call this when trigger sync is lost. The tracker will determine
    /// if this is an ESD glitch (recoverable) or real failure (shutdown).
    ///
    /// # Arguments
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if engine should shut down, `false` if should attempt recovery
    pub fn record_sync_loss(&mut self, current_time_us: u32) -> bool {
        self.synced = false;
        self.inputs.synced = false;
        self.sync_loss_tracker.record_sync_loss(current_time_us)
    }

    /// Record successful sync recovery
    ///
    /// Call this when sync is successfully re-established after a loss.
    pub fn record_sync_recovery(&mut self) {
        self.synced = true;
        self.inputs.synced = true;
        self.sync_loss_tracker.record_recovery();
    }

    /// Reset sync loss window after sustained good operation
    ///
    /// Call this periodically (e.g., every 10 seconds) when sync is stable.
    /// This allows the system to recover from old ESD events.
    pub fn reset_sync_loss_window(&mut self) {
        self.sync_loss_tracker.reset_window();
    }

    /// Check if fuel injection should proceed considering ALL safety features
    ///
    /// This is the master safety check. Returns `true` only if:
    /// - Not in flood clear mode
    /// - Not shut down due to sync loss
    /// - Rev limiter allows injection
    /// - Engine is synced
    /// - Voltage is not critically low
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should proceed, `false` otherwise
    pub fn should_inject_with_all_safety(&self, cylinder: u8) -> bool {
        let trigger_inputs = self.trigger_inputs();
        // Must be synced
        if !trigger_inputs.synced {
            return false;
        }
        // Emergency mode blocks fuel
        if self.emergency_mode() {
            return false;
        }

        // Check voltage - critical low voltage blocks fuel
        if self.voltage_monitor.should_block_fuel() {
            return false;
        }

        // Check flood clear and shutdown
        if !safety::should_allow_injection(
            self.flood_clear_state.active,
            self.sync_loss_tracker.is_shutdown(),
        ) {
            return false;
        }

        // Check rev limiter
        if !self.should_inject_fuel(cylinder) {
            return false;
        }

        true
    }

    /// Update voltage monitor with current battery reading
    ///
    /// Should be called periodically (e.g., every 10-100ms) with ADC reading.
    ///
    /// # Arguments
    /// * `voltage_mv` - Battery voltage in millivolts
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current power state
    pub fn update_voltage(&mut self, voltage_mv: u16, now_us: u32) -> safety::PowerState {
        let state = self.voltage_monitor.update(voltage_mv, now_us);

        // Update battery_voltage_mv for other calculations (injector dead time, dwell)
        self.set_battery_voltage_mv(voltage_mv);

        // Log diagnostic events on state transitions
        if state == safety::PowerState::Critical && !self.diag_map.is_active() {
            // Log low voltage event (reusing diag infrastructure)
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::LowVoltage,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Sensor,
                context: Some(voltage_mv as u32),
                start_us: now_us,
                end_us: 0, // Will be updated when recovered
            });
        }

        state
    }

    /// Get effective RPM limit considering all sources
    ///
    /// Returns the most restrictive RPM limit from:
    /// - Rev limiter config
    /// - Voltage limp mode
    /// - Load failure limp mode
    pub fn get_effective_rpm_limit(&self) -> u16 {
        let mut limit = self.rev_limiter_config().max_rpm;
        let load_failure_config = *self.load_failure_config();

        // Apply voltage limp limit if active
        if let Some(voltage_limit) = self.voltage_monitor.get_rpm_limit() {
            limit = limit.min(voltage_limit);
        }

        // Apply load failure limp limit if active
        if let Some(load_limit) = self
            .load_failure_tracker
            .get_rpm_limit(&load_failure_config)
        {
            limit = limit.min(load_limit);
        }

        limit
    }

    /// Check for load failure condition (MAP fault at high RPM)
    ///
    /// Should be called after process_sensor_update to check if MAP fault
    /// combined with high RPM requires limp mode activation.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if in load-failure limp mode
    pub fn check_load_failure(&mut self, now_us: u32) -> bool {
        let map_fault = self.diag_map.is_active();
        let load_failure_config = *self.load_failure_config();
        let trigger_inputs = self.trigger_inputs();
        let in_limp = self.load_failure_tracker.check(
            map_fault,
            trigger_inputs.rpm,
            &load_failure_config,
            now_us,
        );

        // Log event when entering limp mode
        if in_limp && self.load_failure_tracker.entered_us == now_us {
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::MapFailureHighLoad,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Safety,
                context: Some(trigger_inputs.rpm as u32),
                start_us: now_us,
                end_us: 0,
            });
        }

        in_limp
    }

    /// Check TPS vs MAP sensor plausibility
    ///
    /// Detects implausible sensor combinations that indicate sensor failure.
    /// Should be called after process_sensor_update.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// The confirmed plausibility fault (if any)
    pub fn check_plausibility(&mut self, now_us: u32) -> sensors::plausibility::PlausibilityFault {
        let old_has_fault = self.plausibility_state.has_fault();
        let plausibility_config = *self.plausibility_config();

        let fault = self.plausibility_state.check(
            self.tps_percent,
            self.map_kpa_x10,
            self.trigger_inputs().rpm,
            &plausibility_config,
            now_us,
        );

        // Log event when fault is first confirmed
        if self.plausibility_state.has_fault() && !old_has_fault {
            let tps_percent = self.tps_percent as u32;
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::TpsMapPlausibility,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Safety,
                context: Some(tps_percent),
                start_us: now_us,
                end_us: 0,
            });
        }

        fault
    }

    /// Check if there's a plausibility fault active
    pub fn has_plausibility_fault(&self) -> bool {
        self.plausibility_state.has_fault()
    }

    /// Validate sensor rate-of-change
    ///
    /// Filters out impossible sensor spikes that indicate noise or failure.
    /// Should be called before process_sensor_update for best filtering.
    ///
    /// # Arguments
    /// * `tps_percent` - Raw TPS reading
    /// * `map_kpa_x10` - Raw MAP reading
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// (validated_tps, validated_map) - Filtered values
    pub fn validate_sensor_rates(
        &mut self,
        tps_percent: u8,
        map_kpa_x10: u16,
        now_us: u32,
    ) -> (u8, u16) {
        let rate_config = *self.rate_config();
        let (validated_tps, validated_map, _, _) =
            self.rate_state
                .validate(tps_percent, map_kpa_x10, &rate_config, now_us);

        (validated_tps, validated_map)
    }

    /// Check if any sensor rate was rejected in the last update
    pub fn any_rate_rejected(&self) -> bool {
        self.rate_state.any_rejected()
    }
}
