use super::{
    config::{DisableReason, LambdaConfig},
    state::LambdaState,
};

pub(super) fn update(
    state: &mut LambdaState,
    o2_mv: u16,
    clt_c: i16,
    tps_percent: u8,
    rpm: u16,
    config: &LambdaConfig,
    now_us: u32,
) -> i16 {
    state.last_o2_mv = o2_mv;

    // Check enable conditions
    if !config.enable {
        state.deactivate(DisableReason::ConfigDisabled);
        return 0;
    }

    if clt_c < config.min_clt_c {
        state.deactivate(DisableReason::CoolantTooLow);
        return 0;
    }

    if tps_percent > config.max_tps_percent {
        state.deactivate(DisableReason::WideOpenThrottle);
        return 0;
    }

    if rpm < config.min_rpm {
        state.deactivate(DisableReason::RpmTooLow);
        return 0;
    }

    // Check update interval
    let elapsed = now_us.wrapping_sub(state.last_update_us);
    if state.last_update_us != 0 && elapsed < config.update_interval_us {
        return state.stft_x10; // Return current value, don't update yet.
    }

    state.last_update_us = now_us;
    state.active = true;
    state.disable_reason = None;

    // Calculate error based on sensor type.
    let error = calculate_error(state, o2_mv, config);

    // Apply deadband.
    if error.abs() <= config.deadband_mv as i32 {
        state.in_deadband = true;
        // In deadband, don't accumulate integral, but keep current correction.
        return state.stft_x10;
    }
    state.in_deadband = false;

    // PI control.
    // P term: error * Kp
    let p_term = (error * config.kp_x100 as i32) / 100;

    // I term: integral of error * Ki.
    // Scale elapsed time to seconds for integral.
    let dt_sec_x1000 = (elapsed / 1000) as i32; // ms
    state.integral += (error * config.ki_x100 as i32 * dt_sec_x1000) / 100_000;

    // Anti-windup: limit integral.
    let max_integral = config.authority_max_x10 as i32 * 10;
    state.integral = state.integral.clamp(-max_integral, max_integral);

    // Calculate total correction.
    let correction = p_term + (state.integral / 10);

    // Apply authority limits.
    state.stft_x10 = (correction as i16).clamp(-config.authority_max_x10, config.authority_max_x10);

    state.stft_x10
}

pub(super) fn calculate_error(state: &mut LambdaState, o2_mv: u16, config: &LambdaConfig) -> i32 {
    match state.sensor_type {
        super::O2SensorType::Narrowband => {
            // Narrowband: simple rich/lean detection.
            // Above threshold = rich (need to lean out, negative error).
            // Below threshold = lean (need to richen, positive error).
            let threshold = config.narrowband_threshold_mv as i32;
            let reading = o2_mv as i32;

            // Scale to make error magnitude reasonable.
            // 100mV deviation = ~50 units of error.
            (threshold - reading) / 2
        }
        super::O2SensorType::Wideband => {
            // Wideband: Linear 0-5V = 10-20 AFR typical.
            // Convert mV to AFR x10.
            // 0mV = 10.0 AFR (100), 5000mV = 20.0 AFR (200).
            let afr_x10 = 100 + ((o2_mv as u32 * 100) / 5000) as u16;
            state.last_afr_x10 = afr_x10;

            // Error = target - actual.
            // Positive error = too lean, need more fuel.
            let target = config.target_afr_x10 as i32;
            let actual = afr_x10 as i32;

            (target - actual) * 5 // Scale for similar magnitude to narrowband.
        }
    }
}

pub(super) fn deactivate(state: &mut LambdaState, reason: DisableReason) {
    if state.active {
        // Don't immediately zero the integral - gradual decay.
        state.integral = (state.integral * 9) / 10;
    }
    state.active = false;
    state.disable_reason = Some(reason);
    // Keep stft_x10 at current value for smooth transition.
}

impl LambdaState {
    pub(super) fn deactivate(&mut self, reason: DisableReason) {
        deactivate(self, reason)
    }
}
