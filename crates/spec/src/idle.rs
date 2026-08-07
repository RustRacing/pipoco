use core::num::NonZeroI32;

use crate::numeric::{clamp_i32, clamp_u16, mul_div_floor_i32};
use crate::{InputSnapshot, LogicalState, PiIntegratorState, Rpm, ValidatedCalibration};

const IDLE_DEADBAND_RPM: i32 = 20;
const IDLE_MIN_ACC: i32 = -2000;
const IDLE_MAX_ACC: i32 = 2000;
const IDLE_DUTY_MIN_X1000: u16 = 0;
const IDLE_DUTY_MAX_X1000: u16 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IdleResult {
    pub duty_x1000: u16,
    pub integrator_state: PiIntegratorState,
}

pub fn idle_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> IdleResult {
    let rpm_error = effective_rpm_error(cal.0.idle_target_rpm, input.rpm);
    let p_term = apply_gain(rpm_error, cal.0.idle_kp_x1000);
    let i_step = apply_gain(rpm_error, cal.0.idle_ki_x1000);
    let acc = state.idle_integrator_state.acc;
    let base = cal.0.idle_base_duty_x1000 as i32;
    let u_pre = base + p_term + acc;

    let freeze_gate =
        input.clt_c10.get() < 700 || state.ae.active || input.fuel_cut || input.spark_cut;
    let anti_windup_freeze = saturation_freeze(u_pre, i_step);
    let freeze = freeze_gate || anti_windup_freeze;
    let acc_next = if freeze {
        acc
    } else {
        clamp_i32(acc.saturating_add(i_step), IDLE_MIN_ACC, IDLE_MAX_ACC)
    };

    let duty_pre = base + p_term + acc_next;
    let duty_x1000 = clamp_u16(
        if duty_pre < 0 {
            0
        } else if duty_pre > u16::MAX as i32 {
            u16::MAX
        } else {
            duty_pre as u16
        },
        IDLE_DUTY_MIN_X1000,
        IDLE_DUTY_MAX_X1000,
    );

    IdleResult {
        duty_x1000,
        integrator_state: PiIntegratorState {
            acc: acc_next,
            min_acc: IDLE_MIN_ACC,
            max_acc: IDLE_MAX_ACC,
            frozen: freeze,
        },
    }
}

fn effective_rpm_error(target: Rpm, measured: Rpm) -> i32 {
    let raw = target.get() as i32 - measured.get() as i32;
    if raw.abs() <= IDLE_DEADBAND_RPM {
        0
    } else {
        raw
    }
}

fn apply_gain(error: i32, gain_x1000: u16) -> i32 {
    let divisor = NonZeroI32::new(1000).unwrap_or(NonZeroI32::MIN);
    mul_div_floor_i32(error, gain_x1000 as i32, divisor)
}

fn saturation_freeze(u_pre: i32, i_step: i32) -> bool {
    (u_pre <= IDLE_DUTY_MIN_X1000 as i32 && i_step < 0)
        || (u_pre >= IDLE_DUTY_MAX_X1000 as i32 && i_step > 0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{default_reference_calibration, EngineMode, TempC10};

    #[test]
    fn deadband_zeroes_p_and_i_terms() {
        let mut cal = default_reference_calibration();
        cal.0.idle_target_rpm = Rpm::new(1000);
        cal.0.idle_base_duty_x1000 = 350;
        cal.0.idle_kp_x1000 = 300;
        cal.0.idle_ki_x1000 = 500;
        let input = InputSnapshot {
            rpm: Rpm::new(1010),
            mode: EngineMode::Running,
            clt_c10: TempC10::new(800),
            ..InputSnapshot::default()
        };
        let state = LogicalState::default();
        let result = idle_step(&cal, input, &state);
        assert_eq!(result.duty_x1000, 350);
        assert_eq!(result.integrator_state.acc, 0);
    }

    #[test]
    fn anti_windup_freezes_when_saturated_and_i_pushes_farther() {
        let mut cal = default_reference_calibration();
        cal.0.idle_target_rpm = Rpm::new(3000);
        cal.0.idle_base_duty_x1000 = 1000;
        cal.0.idle_kp_x1000 = 0;
        cal.0.idle_ki_x1000 = 1000;
        let input = InputSnapshot {
            rpm: Rpm::new(2500),
            clt_c10: TempC10::new(800),
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            idle_integrator_state: PiIntegratorState {
                acc: 200,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let result = idle_step(&cal, input, &state);
        assert_eq!(result.integrator_state.acc, 200);
        assert!(result.integrator_state.frozen);
        assert_eq!(result.duty_x1000, 1000);
    }

    #[test]
    fn freeze_gate_holds_integrator_when_engine_is_cold() {
        let mut cal = default_reference_calibration();
        cal.0.idle_target_rpm = Rpm::new(1200);
        cal.0.idle_base_duty_x1000 = 300;
        cal.0.idle_kp_x1000 = 0;
        cal.0.idle_ki_x1000 = 1000;
        let input = InputSnapshot {
            rpm: Rpm::new(1000),
            clt_c10: TempC10::new(650),
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            idle_integrator_state: PiIntegratorState {
                acc: 123,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let result = idle_step(&cal, input, &state);
        assert_eq!(result.integrator_state.acc, 123);
        assert!(result.integrator_state.frozen);
    }
}
