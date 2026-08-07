use core::num::NonZeroI32;

use crate::numeric::{clamp_i32, clamp_u16, mul_div_floor_i32};
use crate::{LogicalState, PiIntegratorState, ValidatedCalibration};

const LAMBDA_DEADBAND_X1000: i32 = 10;
const LAMBDA_MIN_ACC: i32 = -2000;
const LAMBDA_MAX_ACC: i32 = 2000;
const LAMBDA_CORR_MIN_X1000: u16 = 750;
const LAMBDA_CORR_MAX_X1000: u16 = 1250;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LambdaResult {
    pub correction_x1000: u16,
    pub integrator_state: PiIntegratorState,
}

pub fn lambda_step(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
    state: &LogicalState,
    ae_active: bool,
) -> LambdaResult {
    lambda_step_with_error(cal, input, state, ae_active, 0)
}

pub fn lambda_step_with_error(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
    state: &LogicalState,
    ae_active: bool,
    lambda_error_x1000: i32,
) -> LambdaResult {
    let error = effective_lambda_error(lambda_error_x1000);
    let p_term = apply_gain(error, cal.0.lambda_kp_x1000);
    let i_step = apply_gain(error, cal.0.lambda_ki_x1000);
    let acc = state.lambda_integrator_state.acc;
    let corr_pre = 1000 + p_term + acc;

    let freeze_gate = input.clt_c10.get() < 700 || ae_active || input.fuel_cut || input.spark_cut;
    let anti_windup_freeze = saturation_freeze(corr_pre, i_step);
    let freeze = freeze_gate || anti_windup_freeze;
    let acc_next = if freeze {
        acc
    } else {
        clamp_i32(acc.saturating_add(i_step), LAMBDA_MIN_ACC, LAMBDA_MAX_ACC)
    };

    let correction_pre = 1000 + p_term + acc_next;
    let correction_x1000 = clamp_u16(
        i32_to_u16_saturating(correction_pre),
        LAMBDA_CORR_MIN_X1000,
        LAMBDA_CORR_MAX_X1000,
    );

    LambdaResult {
        correction_x1000,
        integrator_state: PiIntegratorState {
            acc: acc_next,
            min_acc: LAMBDA_MIN_ACC,
            max_acc: LAMBDA_MAX_ACC,
            frozen: freeze,
        },
    }
}

fn effective_lambda_error(raw: i32) -> i32 {
    if raw.abs() <= LAMBDA_DEADBAND_X1000 {
        0
    } else {
        raw
    }
}

fn apply_gain(error: i32, gain_x1000: u16) -> i32 {
    let divisor = NonZeroI32::new(1000).unwrap_or(NonZeroI32::MIN);
    mul_div_floor_i32(error, gain_x1000 as i32, divisor)
}

fn saturation_freeze(corr_pre: i32, i_step: i32) -> bool {
    (corr_pre <= LAMBDA_CORR_MIN_X1000 as i32 && i_step < 0)
        || (corr_pre >= LAMBDA_CORR_MAX_X1000 as i32 && i_step > 0)
}

fn i32_to_u16_saturating(value: i32) -> u16 {
    if value < 0 {
        0
    } else if value > u16::MAX as i32 {
        u16::MAX
    } else {
        value as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{default_reference_calibration, InputSnapshot, TempC10};

    #[test]
    fn deadband_zeroes_p_and_i_terms() {
        let mut cal = default_reference_calibration();
        cal.0.lambda_kp_x1000 = 500;
        cal.0.lambda_ki_x1000 = 500;
        let input = InputSnapshot {
            clt_c10: TempC10::new(800),
            ..InputSnapshot::default()
        };
        let state = LogicalState::default();
        let result = lambda_step_with_error(&cal, input, &state, false, 10);
        assert_eq!(result.correction_x1000, 1000);
        assert_eq!(result.integrator_state.acc, 0);
    }

    #[test]
    fn anti_windup_freezes_when_saturated_and_i_pushes_farther() {
        let mut cal = default_reference_calibration();
        cal.0.lambda_kp_x1000 = 0;
        cal.0.lambda_ki_x1000 = 1000;
        let input = InputSnapshot {
            clt_c10: TempC10::new(800),
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            lambda_integrator_state: PiIntegratorState {
                acc: 300,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let result = lambda_step_with_error(&cal, input, &state, false, 500);
        assert_eq!(result.integrator_state.acc, 300);
        assert!(result.integrator_state.frozen);
        assert_eq!(result.correction_x1000, 1250);
    }

    #[test]
    fn freeze_gate_holds_integrator_when_ae_active() {
        let mut cal = default_reference_calibration();
        cal.0.lambda_kp_x1000 = 0;
        cal.0.lambda_ki_x1000 = 1000;
        let input = InputSnapshot {
            clt_c10: TempC10::new(800),
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            lambda_integrator_state: PiIntegratorState {
                acc: 123,
                ..PiIntegratorState::zero()
            },
            ..LogicalState::default()
        };
        let result = lambda_step_with_error(&cal, input, &state, true, 400);
        assert_eq!(result.integrator_state.acc, 123);
        assert!(result.integrator_state.frozen);
    }
}
