use crate::{InputSnapshot, LogicalState, Rpm, ValidatedCalibration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RevLimitResult {
    pub soft_rev_spark_cut: bool,
    pub hard_rev_fuel_cut: bool,
    pub soft_retard_deg10: i16,
    pub soft_active: bool,
    pub hard_active: bool,
}

pub fn rev_limit_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> RevLimitResult {
    let hard_active = latch_with_hysteresis(
        state.rev_hard_active,
        input.rpm,
        cal.0.hard_rev_rpm,
        cal.0.rev_hysteresis_rpm,
    );
    let soft_active = latch_with_hysteresis(
        state.rev_soft_active,
        input.rpm,
        cal.0.soft_rev_rpm,
        cal.0.rev_hysteresis_rpm,
    );

    RevLimitResult {
        soft_rev_spark_cut: soft_active && !hard_active,
        hard_rev_fuel_cut: hard_active,
        soft_retard_deg10: if soft_active {
            -(cal.0.soft_retard_max_deg10 as i16)
        } else {
            0
        },
        soft_active,
        hard_active,
    }
}

fn latch_with_hysteresis(active: bool, rpm: Rpm, threshold: Rpm, hysteresis: Rpm) -> bool {
    let release = threshold.get().saturating_sub(hysteresis.get());
    if active {
        rpm.get() > release
    } else {
        rpm.get() >= threshold.get()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::default_reference_calibration;

    #[test]
    fn hard_limit_latches_and_releases_with_hysteresis() {
        let mut cal = default_reference_calibration();
        cal.0.soft_rev_rpm = Rpm::new(5000);
        cal.0.hard_rev_rpm = Rpm::new(6000);
        cal.0.rev_hysteresis_rpm = Rpm::new(200);

        let mut state = LogicalState::default();
        let mut input = InputSnapshot {
            rpm: Rpm::new(6100),
            ..InputSnapshot::default()
        };

        let at_hard = rev_limit_step(&cal, input, &state);
        assert!(at_hard.hard_rev_fuel_cut);
        state.rev_hard_active = at_hard.hard_active;

        input.rpm = Rpm::new(5900);
        let latched = rev_limit_step(&cal, input, &state);
        assert!(latched.hard_rev_fuel_cut);
        state.rev_hard_active = latched.hard_active;

        input.rpm = Rpm::new(5800);
        let released = rev_limit_step(&cal, input, &state);
        assert!(!released.hard_rev_fuel_cut);
    }

    #[test]
    fn soft_limit_sets_spark_cut_without_hard_cut() {
        let mut cal = default_reference_calibration();
        cal.0.soft_rev_rpm = Rpm::new(4500);
        cal.0.hard_rev_rpm = Rpm::new(6000);
        cal.0.rev_hysteresis_rpm = Rpm::new(100);
        cal.0.soft_retard_max_deg10 = 120;

        let input = InputSnapshot {
            rpm: Rpm::new(4600),
            ..InputSnapshot::default()
        };
        let result = rev_limit_step(&cal, input, &LogicalState::default());

        assert!(result.soft_rev_spark_cut);
        assert!(!result.hard_rev_fuel_cut);
        assert_eq!(result.soft_retard_deg10, -120);
    }
}
