use crate::{InputSnapshot, KnockState, LogicalState, ValidatedCalibration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KnockResult {
    pub advance_trim_deg10: i16,
    pub knock_active: bool,
    pub next_state: KnockState,
}

pub fn knock_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> KnockResult {
    let mut retard = state.knock_state.retard_deg10;
    let mut recovery_counter = state.knock_state.recovery_counter;
    let detected = input.knock_intensity_x100 >= cal.0.knock_threshold_x100;

    if detected {
        let step = cal.0.knock_retard_step_deg10 as i16;
        let max = cal.0.knock_retard_max_deg10 as i16;
        retard = retard.saturating_add(step);
        if retard > max {
            retard = max;
        }
        recovery_counter = 0;
    } else if retard > 0 {
        let delay = cal.0.knock_recovery_delay_cycles;
        if recovery_counter >= delay {
            let step = cal.0.knock_recovery_step_deg10 as i16;
            retard = retard.saturating_sub(step);
            if retard < 0 {
                retard = 0;
            }
            recovery_counter = 0;
        } else {
            recovery_counter = recovery_counter.saturating_add(1);
        }
    } else {
        recovery_counter = 0;
    }

    let knock_active = retard > 0;
    KnockResult {
        advance_trim_deg10: -retard,
        knock_active,
        next_state: KnockState {
            retard_deg10: retard,
            recovery_counter,
            detected,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::default_reference_calibration;

    #[test]
    fn detect_increments_retard_until_maximum() {
        let mut cal = default_reference_calibration();
        cal.0.knock_threshold_x100 = 200;
        cal.0.knock_retard_step_deg10 = 30;
        cal.0.knock_retard_max_deg10 = 80;

        let input = InputSnapshot {
            knock_intensity_x100: 200,
            ..InputSnapshot::default()
        };

        let first = knock_step(&cal, input, &LogicalState::default());
        assert_eq!(first.next_state.retard_deg10, 30);
        assert_eq!(first.advance_trim_deg10, -30);
        assert!(first.knock_active);
        assert!(first.next_state.detected);

        let mut state = LogicalState {
            knock_state: first.next_state,
            ..LogicalState::default()
        };
        let second = knock_step(&cal, input, &state);
        assert_eq!(second.next_state.retard_deg10, 60);

        state.knock_state = second.next_state;
        let third = knock_step(&cal, input, &state);
        assert_eq!(third.next_state.retard_deg10, 80);
        assert_eq!(third.advance_trim_deg10, -80);
    }

    #[test]
    fn clear_path_recovery_uses_counter_delay() {
        let mut cal = default_reference_calibration();
        cal.0.knock_threshold_x100 = 500;
        cal.0.knock_recovery_step_deg10 = 20;
        cal.0.knock_recovery_delay_cycles = 2;

        let input = InputSnapshot {
            knock_intensity_x100: 100,
            ..InputSnapshot::default()
        };
        let mut state = LogicalState {
            knock_state: KnockState {
                retard_deg10: 60,
                recovery_counter: 0,
                detected: true,
            },
            ..LogicalState::default()
        };

        let first = knock_step(&cal, input, &state);
        assert_eq!(first.next_state.retard_deg10, 60);
        assert_eq!(first.next_state.recovery_counter, 1);
        assert!(!first.next_state.detected);

        state.knock_state = first.next_state;
        let second = knock_step(&cal, input, &state);
        assert_eq!(second.next_state.retard_deg10, 60);
        assert_eq!(second.next_state.recovery_counter, 2);

        state.knock_state = second.next_state;
        let third = knock_step(&cal, input, &state);
        assert_eq!(third.next_state.retard_deg10, 40);
        assert_eq!(third.next_state.recovery_counter, 0);
        assert_eq!(third.advance_trim_deg10, -40);
    }
}
