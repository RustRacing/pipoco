use core::num::NonZeroI32;

use crate::numeric::{clamp_i32, mul_div_floor_i32};
use crate::{
    EngineMode, InputSnapshot, LogicalState, PiIntegratorState, Rpm, SignedDegrees10, SyncState,
    ValidatedCalibration,
};

const IDLE_TIMING_MIN_ACC: i32 = -7200;
const IDLE_TIMING_MAX_ACC: i32 = 7200;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdleTimingResult {
    pub trim_deg10: i16,
    pub integrator_state: PiIntegratorState,
    pub active: bool,
}

pub fn idle_timing_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> IdleTimingResult {
    idle_timing_step_with_base(cal, input, state, SignedDegrees10::new(0))
}

pub fn idle_timing_step_with_base(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
    base_advance_deg10: SignedDegrees10,
) -> IdleTimingResult {
    let active = idle_timing_active(cal, input);
    let selected_base = select_idle_or_running_advance_deg10(cal, input, base_advance_deg10).get();
    let replacement_trim = selected_base as i32 - base_advance_deg10.get() as i32;
    let clt_c10 = if input.clt_c10.get() < 0 {
        0
    } else {
        input.clt_c10.get() as u16
    };
    let iat_c10 = if input.iat_c10.get() < 0 {
        0
    } else {
        input.iat_c10.get() as u16
    };
    let clt_timing = lookup_signed_curve(&cal.0.clt_timing_corr_curve_deg10, clt_c10) as i32;
    let iat_timing = lookup_signed_curve(&cal.0.iat_timing_corr_curve_deg10, iat_c10) as i32;
    let rpm_error = cal.0.idle_target_rpm.get() as i32 - input.rpm.get() as i32;
    let p_term = if active && cal.0.idle_timing_pid_enabled {
        apply_gain(rpm_error, cal.0.idle_timing_kp_x1000)
    } else {
        0
    };
    let i_step = if active && cal.0.idle_timing_pid_enabled {
        apply_gain(rpm_error, cal.0.idle_timing_ki_x1000)
    } else {
        0
    };

    let acc = state.idle_timing_integrator_state.acc;
    let raw_pre = replacement_trim + clt_timing + iat_timing + p_term + acc;
    let anti_windup_freeze = saturation_freeze(cal, raw_pre, i_step);
    let freeze = !active || !cal.0.idle_timing_pid_enabled || anti_windup_freeze;
    let acc_next = if freeze {
        acc
    } else {
        clamp_i32(
            acc.saturating_add(i_step),
            IDLE_TIMING_MIN_ACC,
            IDLE_TIMING_MAX_ACC,
        )
    };
    let pid_integral_term = if active && cal.0.idle_timing_pid_enabled {
        acc_next
    } else {
        0
    };
    let raw = replacement_trim + clt_timing + iat_timing + p_term + pid_integral_term;
    let trim = clamp_i32(
        raw,
        cal.0.idle_timing_min_trim_deg10 as i32,
        cal.0.idle_timing_max_trim_deg10 as i32,
    ) as i16;

    IdleTimingResult {
        trim_deg10: trim,
        integrator_state: PiIntegratorState {
            acc: acc_next,
            min_acc: IDLE_TIMING_MIN_ACC,
            max_acc: IDLE_TIMING_MAX_ACC,
            frozen: freeze,
        },
        active,
    }
}

pub fn select_idle_or_running_advance_deg10(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    running_advance_deg10: SignedDegrees10,
) -> SignedDegrees10 {
    if !idle_timing_active(cal, input) {
        return running_advance_deg10;
    }

    let idle_advance = lookup_idle_advance_deg10(cal, input.rpm).get();
    let threshold = cal.0.idle_timing_tps_max_x100;
    let full_idle_threshold = threshold / 2;
    if input.tps_x100 <= full_idle_threshold {
        return SignedDegrees10::new(idle_advance);
    }
    if input.tps_x100 >= threshold {
        return running_advance_deg10;
    }

    SignedDegrees10::new(lerp_i16_by_u16(
        full_idle_threshold,
        threshold,
        idle_advance,
        running_advance_deg10.get(),
        input.tps_x100,
    ))
}

pub fn idle_timing_active(cal: &ValidatedCalibration, input: InputSnapshot) -> bool {
    cal.0.idle_timing_enabled
        && input.mode == EngineMode::Running
        && input.sync == SyncState::Synced
        && !input.fuel_cut
        && !input.spark_cut
        && input.rpm.get() > 0
        && input.rpm.get() <= cal.0.idle_timing_rpm_max.get()
        && input.tps_x100 <= cal.0.idle_timing_tps_max_x100
}

pub fn lookup_idle_advance_deg10(cal: &ValidatedCalibration, rpm: Rpm) -> SignedDegrees10 {
    SignedDegrees10::new(lookup_signed_curve(
        &cal.0.idle_advance_curve_deg10,
        rpm.get(),
    ))
}

fn lookup_signed_curve(curve: &crate::SignedCurve16, x: u16) -> i16 {
    let len = curve.axis.len as usize;
    let x = crate::numeric::clamp_u16(x, curve.axis.values[0], curve.axis.values[len - 1]);
    let idx = crate::interp::find_segment(&curve.axis, x);
    crate::interp::lerp_i16(
        curve.axis.values[idx],
        curve.axis.values[idx + 1],
        curve.values[idx],
        curve.values[idx + 1],
        x,
    )
}

fn lerp_i16_by_u16(x0: u16, x1: u16, y0: i16, y1: i16, x: u16) -> i16 {
    if x1 <= x0 {
        return y0;
    }
    let num = (x - x0) as i32;
    let den = (x1 - x0) as i32;
    let delta = y1 as i32 - y0 as i32;
    (y0 as i32 + delta * num / den) as i16
}

fn apply_gain(error: i32, gain_x1000: u16) -> i32 {
    let divisor = NonZeroI32::new(1000).unwrap_or(NonZeroI32::MIN);
    mul_div_floor_i32(error, gain_x1000 as i32, divisor)
}

fn saturation_freeze(cal: &ValidatedCalibration, raw_pre: i32, i_step: i32) -> bool {
    (raw_pre <= cal.0.idle_timing_min_trim_deg10 as i32 && i_step < 0)
        || (raw_pre >= cal.0.idle_timing_max_trim_deg10 as i32 && i_step > 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{default_reference_calibration, Axis16, SignedCurve16, TempC10};

    fn idle_curve(low: i16, high: i16) -> SignedCurve16 {
        let mut curve = SignedCurve16 {
            axis: Axis16 {
                len: 2,
                values: [500, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            },
            ..SignedCurve16::default()
        };
        curve.values[0] = low;
        curve.values[1] = high;
        curve
    }

    #[test]
    fn disabled_idle_timing_has_zero_trim() {
        let cal = default_reference_calibration();
        let input = InputSnapshot {
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            rpm: Rpm::new(900),
            clt_c10: TempC10::new(800),
            ..InputSnapshot::default()
        };
        let result = idle_timing_step(&cal, input, &LogicalState::default());
        assert_eq!(result.trim_deg10, 0);
        assert!(!result.active);
    }

    #[test]
    fn enabled_idle_timing_uses_advance_curve() {
        let mut cal = default_reference_calibration();
        cal.0.idle_timing_enabled = true;
        cal.0.idle_timing_rpm_max = Rpm::new(1200);
        cal.0.idle_timing_tps_max_x100 = 200;
        cal.0.idle_advance_curve_deg10 = idle_curve(100, 200);
        cal.0.idle_timing_min_trim_deg10 = -100;
        cal.0.idle_timing_max_trim_deg10 = 300;
        let input = InputSnapshot {
            mode: EngineMode::Running,
            sync: SyncState::Synced,
            rpm: Rpm::new(750),
            tps_x100: 0,
            ..InputSnapshot::default()
        };
        let result = idle_timing_step_with_base(
            &cal,
            input,
            &LogicalState::default(),
            SignedDegrees10::new(100),
        );
        assert_eq!(result.trim_deg10, 50);
        assert!(result.active);
    }
}
