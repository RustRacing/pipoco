use crate::{config::CrankConfig, config::PlantConfig, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrankUpdate {
    pub old_angle_deg10: CrankDeg10,
    pub new_angle_deg10: CrankDeg10,
    pub rpm: Rpm,
}

#[allow(clippy::too_many_arguments)]
pub fn update_crank_with_remainder<const CYL: usize>(
    config: &PlantConfig<CYL>,
    dt_us: Micros,
    rpm: Rpm,
    angle: CrankDeg10,
    combustion_torque: TorqueNmX100,
    external_load_torque: TorqueNmX100,
    starter_enabled: bool,
    rpm_delta_remainder: &mut i64,
) -> CrankUpdate {
    if dt_us.0 == 0 {
        return CrankUpdate {
            old_angle_deg10: angle,
            new_angle_deg10: angle,
            rpm,
        };
    }
    update_crank_inner(
        config,
        dt_us,
        rpm,
        angle,
        combustion_torque,
        external_load_torque,
        starter_enabled,
        rpm_delta_remainder,
    )
}

#[allow(clippy::too_many_arguments)]
fn update_crank_inner<const CYL: usize>(
    config: &PlantConfig<CYL>,
    dt_us: Micros,
    rpm: Rpm,
    angle: CrankDeg10,
    combustion_torque: TorqueNmX100,
    external_load_torque: TorqueNmX100,
    starter_enabled: bool,
    rpm_delta_remainder: &mut i64,
) -> CrankUpdate {
    let starter = if starter_enabled {
        config.starter_torque_nm_x100.0
    } else {
        0
    };
    let friction = if rpm.0 > 0 || starter_enabled {
        config.friction_torque_nm_x100.0
    } else {
        0
    };
    let net = starter
        .saturating_add(combustion_torque.0)
        .saturating_sub(friction)
        .saturating_sub(external_load_torque.0);
    let denom = (config.crank_inertia_x1000 as i64)
        .saturating_mul(100_000)
        .max(1);
    let numerator = (net as i64)
        .saturating_mul(dt_us.0 as i64)
        .saturating_add(*rpm_delta_remainder);
    let delta_rpm = numerator / denom;
    *rpm_delta_remainder = numerator % denom;
    let new_rpm = (rpm.0 as i64 + delta_rpm).max(0) as u32;
    if new_rpm == 0 && delta_rpm < 0 {
        *rpm_delta_remainder = 0;
    }
    let delta_angle = (new_rpm as u64)
        .saturating_mul(dt_us.0 as u64)
        .saturating_mul(3600)
        / 60_000_000;

    CrankUpdate {
        old_angle_deg10: angle,
        new_angle_deg10: normalize_deg10(angle.0 as u32 + delta_angle as u32),
        rpm: Rpm(new_rpm),
    }
}

pub fn choose_substep_us(config: CrankConfig, rpm: Rpm, remaining: Micros) -> Micros {
    if remaining.0 <= config.min_substep_us.0 {
        return remaining;
    }

    let by_time = remaining.0.min(config.max_substep_us.0);
    if rpm.0 == 0 {
        return Micros(by_time.max(config.min_substep_us.0).min(remaining.0));
    }

    let by_angle = ((config.max_substep_deg10 as u64).saturating_mul(60_000_000)
        / (rpm.0 as u64).saturating_mul(3600))
    .max(config.min_substep_us.0 as u64)
    .min(u32::MAX as u64) as u32;

    Micros(
        by_time
            .min(by_angle)
            .max(config.min_substep_us.0)
            .min(remaining.0),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    #[test]
    fn substep_is_bounded_by_time_at_zero_rpm() {
        let cfg = PlantConfig::<4>::default_four().crank;

        assert_eq!(
            choose_substep_us(cfg, Rpm(0), Micros(10_000)),
            cfg.max_substep_us
        );
    }

    #[test]
    fn substep_shrinks_at_high_rpm_to_honor_angle_limit() {
        let cfg = PlantConfig::<4>::default_four().crank;
        let low = choose_substep_us(cfg, Rpm(1000), Micros(10_000));
        let high = choose_substep_us(cfg, Rpm(6000), Micros(10_000));

        assert!(high.0 < low.0);
        assert!(high.0 >= cfg.min_substep_us.0);
    }

    #[test]
    fn remainder_preserves_small_substep_acceleration() {
        let cfg = PlantConfig::<4>::default_four();
        let mut rpm = Rpm(0);
        let mut angle = CrankDeg10(0);
        let mut remainder = 0;
        for _ in 0..100 {
            let next = update_crank_with_remainder(
                &cfg,
                Micros(1000),
                rpm,
                angle,
                TorqueNmX100(0),
                TorqueNmX100(0),
                true,
                &mut remainder,
            );
            rpm = next.rpm;
            angle = next.new_angle_deg10;
        }

        assert!(rpm.0 > 0);
    }
}
