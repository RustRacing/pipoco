use crate::{
    config::EngineGeometryConfig,
    geometry::{dvolume_dtheta_mm3_per_rad_q16, pressure_delta_to_torque_nm_x100},
    types::*,
};

pub fn kinematic_pressure_delta_pa(
    air_mass: MassUg,
    quality_x1000: u16,
    torque_scale_x100: u16,
) -> PressurePa {
    let pressure = air_mass.0 as u128 * quality_x1000 as u128 * torque_scale_x100 as u128 / 1000;
    PressurePa(pressure.min(i32::MAX as u128) as i32)
}

pub fn kinematic_pressure_torque_nm_x100(
    geometry: EngineGeometryConfig,
    local_angle: CrankDeg10,
    delta_pressure: PressurePa,
) -> TorqueNmX100 {
    pressure_delta_to_torque_nm_x100(
        delta_pressure,
        dvolume_dtheta_mm3_per_rad_q16(geometry, local_angle),
    )
}

pub fn bmep_bar_x100(torque: TorqueNmX100, displacement_cc: u32) -> BmepBarX100 {
    if displacement_cc == 0 {
        return BmepBarX100(0);
    }

    let numerator = torque.0 as i128 * 40 * 355;
    let denominator = 113 * displacement_cc as i128;
    BmepBarX100((numerator / denominator) as i32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    #[test]
    fn pressure_pulse_torque_sign_follows_volume_derivative() {
        let cfg = PlantConfig::<4>::default_four();
        let pressure = PressurePa(250_000);
        let expansion = kinematic_pressure_torque_nm_x100(cfg.engine, CrankDeg10(900), pressure);
        let compression = kinematic_pressure_torque_nm_x100(cfg.engine, CrankDeg10(2700), pressure);

        assert!(expansion.0 > 0);
        assert!(compression.0 < 0);
    }

    #[test]
    fn bmep_reports_plausible_bar_value_for_known_torque() {
        let bmep = bmep_bar_x100(TorqueNmX100(20000), 2000);

        assert!((bmep.0 - 1256).abs() <= 1);
    }

    #[test]
    fn kinematic_pressure_delta_saturates_instead_of_overflowing() {
        let pressure = kinematic_pressure_delta_pa(MassUg(u32::MAX), u16::MAX, u16::MAX);

        assert_eq!(pressure, PressurePa(i32::MAX));
    }
}
