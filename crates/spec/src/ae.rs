use crate::interp::{find_segment, lerp_u16};
use crate::numeric::{clamp_i32, clamp_u16, mul_ratio_x1000};
use crate::{AeState, Curve16, InputSnapshot, LogicalState, PulseWidthUs, ValidatedCalibration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AeStepResult {
    pub ae_pulse_us: PulseWidthUs,
    pub next_state: AeState,
}

#[derive(Clone, Copy)]
pub struct AeCurves<'a> {
    pub tps_threshold_curve: &'a Curve16,
    pub map_threshold_curve: &'a Curve16,
    pub shot_curve_us: &'a Curve16,
    pub decay_steps_curve: &'a Curve16,
    pub decay_ratio_curve_x1000: &'a Curve16,
}

pub fn ae_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> AeStepResult {
    let curves = AeCurves {
        tps_threshold_curve: &cal.0.ae_tps_threshold_curve,
        map_threshold_curve: &cal.0.ae_map_threshold_curve,
        shot_curve_us: &cal.0.ae_shot_curve_us,
        decay_steps_curve: &cal.0.ae_decay_steps_curve,
        decay_ratio_curve_x1000: &cal.0.ae_decay_ratio_curve_x1000,
    };
    let tps_delta_x100 = delta_i16(
        input.load_kpa10.get(),
        state.math.last_valid_load_kpa10.get(),
    );
    let map_delta_kpa10 = delta_i16(input.map_kpa10.get(), state.math.last_valid_map_kpa10.get());
    ae_step_with_deltas(curves, input, state.ae, tps_delta_x100, map_delta_kpa10)
}

pub fn ae_step_with_deltas(
    curves: AeCurves<'_>,
    input: InputSnapshot,
    previous: AeState,
    tps_delta_x100: i16,
    map_delta_kpa10: i16,
) -> AeStepResult {
    let tps_threshold = lookup_curve_u16(curves.tps_threshold_curve, input.rpm.get());
    let map_threshold = lookup_curve_u16(curves.map_threshold_curve, input.load_kpa10.get());
    let triggered = abs_i16_to_u16(tps_delta_x100) >= tps_threshold
        || abs_i16_to_u16(map_delta_kpa10) >= map_threshold;

    if triggered {
        let shot = lookup_curve_u16(curves.shot_curve_us, input.load_kpa10.get()) as u32;
        let decay_steps = lookup_curve_u16(curves.decay_steps_curve, input.load_kpa10.get());
        return AeStepResult {
            ae_pulse_us: PulseWidthUs::new(shot),
            next_state: AeState {
                active: shot > 0,
                pulse_us: shot,
                decay_steps_remaining: decay_steps,
            },
        };
    }

    if previous.decay_steps_remaining > 0 {
        let decay_index = previous.decay_steps_remaining.saturating_sub(1);
        let decay_ratio_x1000 = lookup_curve_u16(curves.decay_ratio_curve_x1000, decay_index);
        let next_pulse =
            mul_ratio_x1000(previous.pulse_us, crate::RatioX1000::new(decay_ratio_x1000));
        return AeStepResult {
            ae_pulse_us: PulseWidthUs::new(next_pulse),
            next_state: AeState {
                active: next_pulse > 0,
                pulse_us: next_pulse,
                decay_steps_remaining: previous.decay_steps_remaining.saturating_sub(1),
            },
        };
    }

    AeStepResult {
        ae_pulse_us: PulseWidthUs::new(0),
        next_state: AeState::default(),
    }
}

fn lookup_curve_u16(curve: &Curve16, x: u16) -> u16 {
    let len = curve.axis.len as usize;
    let clipped = clamp_u16(x, curve.axis.values[0], curve.axis.values[len - 1]);
    let seg = find_segment(&curve.axis, clipped);
    lerp_u16(
        curve.axis.values[seg],
        curve.axis.values[seg + 1],
        curve.values[seg],
        curve.values[seg + 1],
        clipped,
    )
}

fn abs_i16_to_u16(v: i16) -> u16 {
    if v >= 0 {
        v as u16
    } else {
        v.saturating_neg() as u16
    }
}

fn delta_i16(current: u16, previous: u16) -> i16 {
    let diff = current as i32 - previous as i32;
    clamp_i32(diff, i16::MIN as i32, i16::MAX as i32) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Axis16, Curve16, Kpa10, LogicalState, Rpm};

    fn axis(values: &[u16]) -> Axis16 {
        let mut axis = Axis16 {
            len: values.len() as u8,
            ..Axis16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            axis.values[idx] = values[idx];
            idx += 1;
        }
        axis
    }

    fn curve(axis_values: &[u16], values: &[u16]) -> Curve16 {
        let mut curve = Curve16 {
            axis: axis(axis_values),
            ..Curve16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            curve.values[idx] = values[idx];
            idx += 1;
        }
        curve
    }

    #[test]
    fn triggers_shot_on_threshold_cross() {
        let input = InputSnapshot {
            rpm: Rpm::new(2000),
            load_kpa10: Kpa10::new(1000),
            tps_x100: 0,
            map_kpa10: Kpa10::new(1000),
            ..InputSnapshot::default()
        };
        let result = ae_step_with_deltas(
            AeCurves {
                tps_threshold_curve: &curve(&[1000, 3000], &[50, 50]),
                map_threshold_curve: &curve(&[500, 1500], &[50, 50]),
                shot_curve_us: &curve(&[500, 1500], &[400, 800]),
                decay_steps_curve: &curve(&[500, 1500], &[3, 3]),
                decay_ratio_curve_x1000: &curve(&[0, 3], &[500, 500]),
            },
            input,
            AeState::default(),
            80,
            0,
        );

        assert_eq!(result.ae_pulse_us.get(), 600);
        assert!(result.next_state.active);
        assert_eq!(result.next_state.decay_steps_remaining, 3);
    }

    #[test]
    fn decays_when_active_without_retrigger() {
        let input = InputSnapshot {
            rpm: Rpm::new(2000),
            load_kpa10: Kpa10::new(1000),
            tps_x100: 0,
            map_kpa10: Kpa10::new(1000),
            ..InputSnapshot::default()
        };
        let result = ae_step_with_deltas(
            AeCurves {
                tps_threshold_curve: &curve(&[1000, 3000], &[500, 500]),
                map_threshold_curve: &curve(&[500, 1500], &[500, 500]),
                shot_curve_us: &curve(&[500, 1500], &[0, 0]),
                decay_steps_curve: &curve(&[500, 1500], &[0, 0]),
                decay_ratio_curve_x1000: &curve(&[0, 3], &[800, 800]),
            },
            input,
            AeState {
                active: true,
                pulse_us: 1000,
                decay_steps_remaining: 2,
            },
            0,
            0,
        );

        assert_eq!(result.ae_pulse_us.get(), 800);
        assert!(result.next_state.active);
        assert_eq!(result.next_state.decay_steps_remaining, 1);
    }

    #[test]
    fn resets_to_zero_when_not_triggered_and_no_decay_left() {
        let input = InputSnapshot {
            rpm: Rpm::new(2000),
            load_kpa10: Kpa10::new(1000),
            tps_x100: 0,
            map_kpa10: Kpa10::new(1000),
            ..InputSnapshot::default()
        };
        let result = ae_step_with_deltas(
            AeCurves {
                tps_threshold_curve: &curve(&[1000, 3000], &[500, 500]),
                map_threshold_curve: &curve(&[500, 1500], &[500, 500]),
                shot_curve_us: &curve(&[500, 1500], &[0, 0]),
                decay_steps_curve: &curve(&[500, 1500], &[0, 0]),
                decay_ratio_curve_x1000: &curve(&[0, 3], &[800, 800]),
            },
            input,
            AeState {
                active: true,
                pulse_us: 50,
                decay_steps_remaining: 0,
            },
            0,
            0,
        );

        assert_eq!(result.ae_pulse_us.get(), 0);
        assert_eq!(result.next_state, AeState::default());
    }

    #[test]
    fn top_level_step_uses_state_deltas() {
        let cal = crate::default_reference_calibration();
        let mut state = LogicalState::default();
        state.math.last_valid_map_kpa10 = Kpa10::new(1000);
        state.math.last_valid_load_kpa10 = Kpa10::new(1000);

        let input = InputSnapshot {
            rpm: Rpm::new(1000),
            load_kpa10: Kpa10::new(1600),
            tps_x100: 0,
            map_kpa10: Kpa10::new(1000),
            ..InputSnapshot::default()
        };

        let result = ae_step(&cal, input, &state);
        assert_eq!(result.ae_pulse_us.get(), 0);
    }
}
