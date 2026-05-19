use crate::{FlatShiftResult, KnockResult, RevLimitResult};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArbiterInputs {
    pub safety_latched: bool,
    pub dfco_cut: bool,
    pub rev_limit: RevLimitResult,
    pub launch_cut: bool,
    pub flat_shift: FlatShiftResult,
    pub knock: KnockResult,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ArbiterResult {
    pub cut_reason_code: u8,
    pub fuel_cut: bool,
    pub spark_cut: bool,
}

pub fn arbiter_step(input: ArbiterInputs) -> ArbiterResult {
    if input.safety_latched {
        return ArbiterResult {
            cut_reason_code: 1,
            fuel_cut: true,
            spark_cut: true,
        };
    }

    if input.rev_limit.hard_rev_fuel_cut {
        return ArbiterResult {
            cut_reason_code: 2,
            fuel_cut: true,
            spark_cut: true,
        };
    }

    if input.launch_cut {
        return ArbiterResult {
            cut_reason_code: 3,
            fuel_cut: true,
            spark_cut: true,
        };
    }

    if input.flat_shift.flat_shift_cut {
        return ArbiterResult {
            cut_reason_code: 4,
            fuel_cut: true,
            spark_cut: true,
        };
    }

    if input.dfco_cut {
        return ArbiterResult {
            cut_reason_code: 5,
            fuel_cut: true,
            spark_cut: false,
        };
    }

    if input.rev_limit.soft_rev_spark_cut {
        return ArbiterResult {
            cut_reason_code: 6,
            fuel_cut: false,
            spark_cut: true,
        };
    }

    if input.knock.knock_active {
        return ArbiterResult {
            cut_reason_code: 7,
            fuel_cut: false,
            spark_cut: false,
        };
    }

    ArbiterResult {
        cut_reason_code: 0,
        fuel_cut: false,
        spark_cut: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{default_reference_calibration, flat_shift_step, knock_step, launch_step};
    use crate::{dfco_step, rev_limit_step, InputSnapshot, LogicalState, Rpm};

    fn base_inputs() -> ArbiterInputs {
        let mut cal = default_reference_calibration();
        cal.0.dfco_entry_rpm = Rpm(2000);
        cal.0.dfco_exit_rpm = Rpm(1800);
        cal.0.dfco_entry_tps_x100 = 0;
        cal.0.dfco_exit_tps_x100 = 0;
        cal.0.dfco_entry_map_kpa10 = crate::Kpa10(0);
        cal.0.dfco_delay_cycles = 0;
        cal.0.soft_rev_rpm = Rpm(3000);
        cal.0.hard_rev_rpm = Rpm(4000);
        cal.0.rev_hysteresis_rpm = Rpm(100);
        cal.0.launch_rpm_limit = Rpm(3500);
        cal.0.launch_cut_cycles = 0;
        cal.0.flat_shift_rpm_min = Rpm(3500);
        cal.0.flat_shift_cut_cycles = 0;
        cal.0.knock_threshold_x100 = 100;
        cal.0.knock_retard_step_deg10 = 50;
        cal.0.knock_retard_max_deg10 = 200;

        let input = InputSnapshot {
            rpm: Rpm(5000),
            tps_x100: 0,
            map_kpa10: crate::Kpa10(0),
            launch_armed: true,
            flat_shift_armed: true,
            knock_intensity_x100: 100,
            ..InputSnapshot::default()
        };
        let state = LogicalState::default();
        ArbiterInputs {
            safety_latched: false,
            dfco_cut: dfco_step(&cal, input, &state).fuel_cut,
            rev_limit: rev_limit_step(&cal, input, &state),
            launch_cut: launch_step(&cal, input, &state).launch_cut,
            flat_shift: flat_shift_step(&cal, input, &state),
            knock: knock_step(&cal, input, &state),
        }
    }

    #[test]
    fn safety_has_highest_priority() {
        let mut input = base_inputs();
        input.safety_latched = true;
        let out = arbiter_step(input);
        assert_eq!(out.cut_reason_code, 1);
        assert!(out.fuel_cut);
        assert!(out.spark_cut);
    }

    #[test]
    fn hard_rev_beats_launch_flatshift_dfco_soft_and_knock() {
        let out = arbiter_step(base_inputs());
        assert_eq!(out.cut_reason_code, 2);
        assert!(out.fuel_cut);
        assert!(out.spark_cut);
    }

    #[test]
    fn none_is_returned_when_no_input_is_active() {
        let out = arbiter_step(ArbiterInputs {
            safety_latched: false,
            dfco_cut: false,
            rev_limit: RevLimitResult {
                soft_rev_spark_cut: false,
                hard_rev_fuel_cut: false,
                soft_retard_deg10: 0,
                soft_active: false,
                hard_active: false,
            },
            launch_cut: false,
            flat_shift: FlatShiftResult {
                flat_shift_cut: false,
                active: false,
                cut_cycle_count: 0,
            },
            knock: KnockResult {
                advance_trim_deg10: 0,
                knock_active: false,
                next_state: crate::KnockState::default(),
            },
        });
        assert_eq!(out.cut_reason_code, 0);
        assert!(!out.fuel_cut);
        assert!(!out.spark_cut);
    }
}
