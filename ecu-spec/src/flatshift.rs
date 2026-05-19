use crate::{InputSnapshot, LogicalState, ValidatedCalibration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FlatShiftResult {
    pub flat_shift_cut: bool,
    pub active: bool,
    pub cut_cycle_count: u16,
}

pub fn flat_shift_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> FlatShiftResult {
    let active = input.flat_shift_armed && input.rpm.0 >= cal.0.flat_shift_rpm_min.0;
    if !active {
        return FlatShiftResult {
            flat_shift_cut: false,
            active: false,
            cut_cycle_count: 0,
        };
    }

    if cal.0.flat_shift_cut_cycles == 0 {
        return FlatShiftResult {
            flat_shift_cut: true,
            active: true,
            cut_cycle_count: 0,
        };
    }

    let phase = state.flat_shift_cut_cycle_count % (cal.0.flat_shift_cut_cycles.saturating_add(1));
    let flat_shift_cut = phase < cal.0.flat_shift_cut_cycles;
    let cut_cycle_count = if phase >= cal.0.flat_shift_cut_cycles {
        0
    } else {
        state.flat_shift_cut_cycle_count.saturating_add(1)
    };

    FlatShiftResult {
        flat_shift_cut,
        active: true,
        cut_cycle_count,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{default_reference_calibration, InputSnapshot, LogicalState, Rpm};

    #[test]
    fn disarmed_or_below_rpm_resets_state_and_does_not_cut() {
        let mut cal = default_reference_calibration();
        cal.0.flat_shift_rpm_min = Rpm(5000);
        let state = LogicalState {
            flat_shift_active: true,
            flat_shift_cut_cycle_count: 3,
            ..LogicalState::default()
        };

        let disarmed = InputSnapshot {
            flat_shift_armed: false,
            rpm: Rpm(7000),
            ..InputSnapshot::default()
        };
        let result = flat_shift_step(&cal, disarmed, &state);
        assert!(!result.flat_shift_cut);
        assert!(!result.active);
        assert_eq!(result.cut_cycle_count, 0);

        let below_rpm = InputSnapshot {
            flat_shift_armed: true,
            rpm: Rpm(4500),
            ..InputSnapshot::default()
        };
        let result = flat_shift_step(&cal, below_rpm, &state);
        assert!(!result.flat_shift_cut);
        assert!(!result.active);
        assert_eq!(result.cut_cycle_count, 0);
    }

    #[test]
    fn active_step_uses_cycle_bounded_cut_pattern() {
        let mut cal = default_reference_calibration();
        cal.0.flat_shift_rpm_min = Rpm(5000);
        cal.0.flat_shift_cut_cycles = 2;
        let input = InputSnapshot {
            flat_shift_armed: true,
            rpm: Rpm(6000),
            ..InputSnapshot::default()
        };
        let mut state = LogicalState::default();

        let s0 = flat_shift_step(&cal, input, &state);
        assert!(s0.active);
        assert!(s0.flat_shift_cut);
        assert_eq!(s0.cut_cycle_count, 1);

        state.flat_shift_cut_cycle_count = s0.cut_cycle_count;
        let s1 = flat_shift_step(&cal, input, &state);
        assert!(s1.active);
        assert!(s1.flat_shift_cut);
        assert_eq!(s1.cut_cycle_count, 2);

        state.flat_shift_cut_cycle_count = s1.cut_cycle_count;
        let s2 = flat_shift_step(&cal, input, &state);
        assert!(s2.active);
        assert!(!s2.flat_shift_cut);
        assert_eq!(s2.cut_cycle_count, 0);
    }
}
