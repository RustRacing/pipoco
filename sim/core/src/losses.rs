use crate::{config::LossConfig, pressure::bmep_bar_x100, types::*};

pub fn fmep_pa(config: LossConfig, rpm: Rpm, load: Kpa10) -> PressurePa {
    let rpm_term = config.fmep_rpm_pa_per_krpm as i64 * rpm.0 as i64 / 1000;
    let rpm2_term = config.fmep_rpm2_pa_per_krpm2 as i64 * rpm.0 as i64 * rpm.0 as i64 / 1_000_000;
    let load_term = config.fmep_load_pa_per_kpa as i64 * load.0 as i64 / 10;
    let value = config.fmep_base_pa.0 as i64 + rpm_term + rpm2_term + load_term;

    PressurePa(value.clamp(0, i32::MAX as i64) as i32)
}

pub fn pmep_pa(config: LossConfig, throttle_x1000: u16) -> PressurePa {
    let closed_throttle = 1000i64.saturating_sub(throttle_x1000.min(1000) as i64);
    let throttle_term = config.pumping_throttle_pa_per_x1000 as i64 * closed_throttle / 1000;
    let value = config.pumping_base_pa.0 as i64 + throttle_term;

    PressurePa(value.clamp(0, i32::MAX as i64) as i32)
}

pub fn torque_from_mep_nm_x100(pressure: PressurePa, displacement_cc: u32) -> TorqueNmX100 {
    if displacement_cc == 0 || pressure.0 <= 0 {
        return TorqueNmX100(0);
    }

    let pressure_bar_x100 = pressure.0 as i128 / 1000;
    let numerator = pressure_bar_x100 * displacement_cc as i128 * 113;
    let denominator = 40 * 355;

    TorqueNmX100((numerator / denominator) as i32)
}

pub fn brake_torque_from_indicated(
    indicated: TorqueNmX100,
    friction: TorqueNmX100,
    pumping: TorqueNmX100,
    accessories: TorqueNmX100,
) -> TorqueNmX100 {
    TorqueNmX100(
        indicated
            .0
            .saturating_sub(friction.0)
            .saturating_sub(pumping.0)
            .saturating_sub(accessories.0),
    )
}

pub fn fmep_bar_x100(friction: TorqueNmX100, displacement_cc: u32) -> FmepBarX100 {
    FmepBarX100(bmep_bar_x100(friction, displacement_cc).0)
}

pub fn pmep_bar_x100(pumping: TorqueNmX100, displacement_cc: u32) -> PmepBarX100 {
    PmepBarX100(bmep_bar_x100(pumping, displacement_cc).0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    #[test]
    fn fmep_rises_with_rpm_for_positive_coefficients() {
        let losses = PlantConfig::<4>::default_four().losses;

        assert!(
            fmep_pa(losses, Rpm(4000), Kpa10(1000)).0 > fmep_pa(losses, Rpm(1000), Kpa10(1000)).0
        );
    }

    #[test]
    fn pumping_loss_rises_as_throttle_closes_when_configured() {
        let mut losses = PlantConfig::<4>::default_four().losses;
        losses.pumping_throttle_pa_per_x1000 = 10_000;

        assert!(pmep_pa(losses, 0).0 > pmep_pa(losses, 1000).0);
    }

    #[test]
    fn mep_torque_round_trips_to_bmep_scale() {
        let torque = torque_from_mep_nm_x100(PressurePa(1_000_000), 2000);
        let bmep = bmep_bar_x100(torque, 2000);

        assert!((bmep.0 - 1000).abs() <= 2);
    }

    #[test]
    fn brake_torque_subtracts_explicit_losses() {
        assert_eq!(
            brake_torque_from_indicated(
                TorqueNmX100(10000),
                TorqueNmX100(1000),
                TorqueNmX100(2000),
                TorqueNmX100(500),
            ),
            TorqueNmX100(6500)
        );
    }
}
