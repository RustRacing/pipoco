use super::{config::DisableReason, config::LambdaConfig, config::O2SensorType};

#[derive(Debug, Clone, Copy)]
pub struct LambdaState {
    /// Is closed-loop currently active?
    pub active: bool,
    /// Current short-term fuel trim x10 (-200 to +200 = -20% to +20%).
    pub stft_x10: i16,
    /// Integral accumulator (scaled).
    pub integral: i32,
    /// Last O2 sensor reading (millivolts).
    pub last_o2_mv: u16,
    /// Last calculated AFR x10 (for wideband).
    pub last_afr_x10: u16,
    /// Last update timestamp (microseconds).
    pub last_update_us: u32,
    /// Is sensor currently in deadband?
    pub in_deadband: bool,
    /// Reason for being disabled.
    pub disable_reason: Option<DisableReason>,
    /// O2 sensor type in use.
    pub sensor_type: O2SensorType,
}

impl LambdaState {
    /// Create new lambda state.
    pub const fn new() -> Self {
        Self {
            active: false,
            stft_x10: 0,
            integral: 0,
            last_o2_mv: 450, // Mid-point
            last_afr_x10: 147,
            last_update_us: 0,
            in_deadband: false,
            disable_reason: Some(DisableReason::NoSignal),
            sensor_type: O2SensorType::Narrowband,
        }
    }

    /// Check if closed-loop is currently active.
    pub fn is_active(&self) -> bool {
        self.active
    }

    /// Get current short-term fuel trim.
    pub fn get_stft(&self) -> i16 {
        self.stft_x10
    }

    /// Get current short-term fuel trim as a percentage x10.
    pub fn get_stft_percent_x10(&self) -> i16 {
        self.stft_x10
    }

    /// Set sensor type.
    pub fn set_sensor_type(&mut self, sensor_type: O2SensorType) {
        self.sensor_type = sensor_type;
    }

    /// Reset controller state.
    pub fn reset(&mut self) {
        self.stft_x10 = 0;
        self.integral = 0;
        self.in_deadband = false;
    }

    /// Force disable closed-loop.
    pub fn manual_disable(&mut self) {
        self.deactivate(DisableReason::ManualDisable);
    }

    /// Re-enable after manual disable.
    pub fn manual_enable(&mut self) {
        if self.disable_reason == Some(DisableReason::ManualDisable) {
            self.disable_reason = None;
        }
    }

    /// Update the PI controller with new O2 reading.
    ///
    /// # Arguments
    /// * `o2_mv` - O2 sensor reading in millivolts.
    /// * `clt_c` - Coolant temperature in Celsius.
    /// * `tps_percent` - Throttle position (0-100%).
    /// * `rpm` - Engine RPM.
    /// * `config` - Lambda configuration.
    /// * `now_us` - Current timestamp in microseconds.
    ///
    /// # Returns
    /// Short-term fuel trim x10 (-200 to +200 = -20% to +20%).
    pub fn update(
        &mut self,
        o2_mv: u16,
        clt_c: i16,
        tps_percent: u8,
        rpm: u16,
        config: &LambdaConfig,
        now_us: u32,
    ) -> i16 {
        crate::lambda::controller::update(self, o2_mv, clt_c, tps_percent, rpm, config, now_us)
    }
}

impl Default for LambdaState {
    fn default() -> Self {
        Self::new()
    }
}
