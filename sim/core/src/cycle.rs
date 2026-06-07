use crate::{state::CycleAccumulator, types::*};

pub fn crossed_cycle_boundary(start: CrankDeg10, end: CrankDeg10) -> bool {
    end.0 < start.0
}

pub fn angle_travel_deg10(start: CrankDeg10, end: CrankDeg10) -> u16 {
    if end.0 >= start.0 {
        end.0 - start.0
    } else {
        (CYCLE_DEG10 - start.0 as u32 + end.0 as u32) as u16
    }
}

pub fn torque_angle_work_micro_j(torque: TorqueNmX100, travel_deg10: u16) -> EnergyMicroJ {
    let work = torque.0 as i128 * travel_deg10 as i128 * 355 * 1_000_000 / (113 * 100 * 10 * 180);
    EnergyMicroJ(work.clamp(i64::MIN as i128, i64::MAX as i128) as i64)
}

pub fn accumulate_cycle_work<const CYL: usize>(
    cycle: &mut CycleAccumulator<CYL>,
    start: CrankDeg10,
    end: CrankDeg10,
    indicated_torque: TorqueNmX100,
    brake_torque: TorqueNmX100,
) {
    let travel = angle_travel_deg10(start, end);
    cycle.indicated_work_micro_j.0 = cycle
        .indicated_work_micro_j
        .0
        .saturating_add(torque_angle_work_micro_j(indicated_torque, travel).0);
    cycle.brake_work_micro_j.0 = cycle
        .brake_work_micro_j
        .0
        .saturating_add(torque_angle_work_micro_j(brake_torque, travel).0);

    if crossed_cycle_boundary(start, end) {
        cycle.completed_cycle_count = cycle.completed_cycle_count.saturating_add(1);
        cycle.reset_current_cycle(end);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cycle_boundary_detects_wrap_only() {
        assert!(!crossed_cycle_boundary(CrankDeg10(100), CrankDeg10(200)));
        assert!(crossed_cycle_boundary(CrankDeg10(7100), CrankDeg10(20)));
    }

    #[test]
    fn angle_travel_wraps_over_cycle_boundary() {
        assert_eq!(angle_travel_deg10(CrankDeg10(100), CrankDeg10(250)), 150);
        assert_eq!(angle_travel_deg10(CrankDeg10(7100), CrankDeg10(100)), 200);
    }

    #[test]
    fn torque_angle_work_preserves_sign() {
        assert!(torque_angle_work_micro_j(TorqueNmX100(10000), 900).0 > 0);
        assert!(torque_angle_work_micro_j(TorqueNmX100(-10000), 900).0 < 0);
    }

    #[test]
    fn accumulator_counts_completed_cycles() {
        let mut cycle = CycleAccumulator::<4>::new();

        accumulate_cycle_work(
            &mut cycle,
            CrankDeg10(7100),
            CrankDeg10(100),
            TorqueNmX100(1000),
            TorqueNmX100(800),
        );

        assert_eq!(cycle.completed_cycle_count, 1);
        assert_eq!(cycle.cycle_start_angle_deg10, CrankDeg10(100));
    }
}
