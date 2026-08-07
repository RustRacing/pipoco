use crate::{InputSnapshot, LogicalState, ValidatedCalibration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LaunchResult {
    pub launch_cut: bool,
    pub active: bool,
    pub cut_cycle_count: u16,
}

pub fn launch_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> LaunchResult {
    let active = input.launch_armed;
    if !active {
        return LaunchResult {
            launch_cut: false,
            active: false,
            cut_cycle_count: 0,
        };
    }

    let rpm_gate = input.rpm.get() >= cal.0.launch_rpm_limit.get();
    if !rpm_gate {
        return LaunchResult {
            launch_cut: false,
            active: true,
            cut_cycle_count: 0,
        };
    }

    if cal.0.launch_cut_cycles == 0 {
        return LaunchResult {
            launch_cut: true,
            active: true,
            cut_cycle_count: 0,
        };
    }

    let phase = state.launch_cut_cycle_count % (cal.0.launch_cut_cycles.saturating_add(1));
    let launch_cut = phase < cal.0.launch_cut_cycles;
    let cut_cycle_count = if phase >= cal.0.launch_cut_cycles {
        0
    } else {
        state.launch_cut_cycle_count.saturating_add(1)
    };

    LaunchResult {
        launch_cut,
        active: true,
        cut_cycle_count,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{default_reference_calibration, InputSnapshot, LogicalState, Rpm};

    #[test]
    fn disarmed_launch_resets_state_and_never_cuts() {
        let cal = default_reference_calibration();
        let input = InputSnapshot {
            launch_armed: false,
            rpm: Rpm::new(8000),
            ..InputSnapshot::default()
        };
        let state = LogicalState {
            launch_active: true,
            launch_cut_cycle_count: 3,
            ..LogicalState::default()
        };

        let result = launch_step(&cal, input, &state);
        assert!(!result.launch_cut);
        assert!(!result.active);
        assert_eq!(result.cut_cycle_count, 0);
    }

    #[test]
    fn armed_below_rpm_limit_does_not_cut() {
        let mut cal = default_reference_calibration();
        cal.0.launch_rpm_limit = Rpm::new(5000);
        let input = InputSnapshot {
            launch_armed: true,
            rpm: Rpm::new(4500),
            ..InputSnapshot::default()
        };

        let result = launch_step(&cal, input, &LogicalState::default());
        assert!(!result.launch_cut);
        assert!(result.active);
        assert_eq!(result.cut_cycle_count, 0);
    }

    #[test]
    fn armed_above_limit_uses_cycle_bounded_cut_pattern() {
        let mut cal = default_reference_calibration();
        cal.0.launch_rpm_limit = Rpm::new(5000);
        cal.0.launch_cut_cycles = 2;

        let input = InputSnapshot {
            launch_armed: true,
            rpm: Rpm::new(5500),
            ..InputSnapshot::default()
        };
        let mut state = LogicalState::default();

        let s0 = launch_step(&cal, input, &state);
        assert!(s0.launch_cut);
        assert_eq!(s0.cut_cycle_count, 1);

        state.launch_cut_cycle_count = s0.cut_cycle_count;
        let s1 = launch_step(&cal, input, &state);
        assert!(s1.launch_cut);
        assert_eq!(s1.cut_cycle_count, 2);

        state.launch_cut_cycle_count = s1.cut_cycle_count;
        let s2 = launch_step(&cal, input, &state);
        assert!(!s2.launch_cut);
        assert_eq!(s2.cut_cycle_count, 0);
    }
}
