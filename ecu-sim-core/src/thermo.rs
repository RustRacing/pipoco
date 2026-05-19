use crate::types::*;

pub fn ideal_gas_pressure_pa(
    air_mass: MassUg,
    temperature: Kelvin10,
    volume: VolumeMm3,
    r_air_j_per_kg_k: u16,
) -> PressurePa {
    if air_mass.0 == 0 || temperature.0 == 0 || volume.0 == 0 {
        return PressurePa(0);
    }

    let pressure = air_mass.0 as u128 * r_air_j_per_kg_k as u128 * temperature.0 as u128
        / (10 * volume.0 as u128);
    PressurePa(pressure.min(i32::MAX as u128) as i32)
}

pub fn fuel_heat_release_micro_j(
    fuel_mass: MassUg,
    fuel_lhv_j_per_kg: u32,
    efficiency_x1000: u16,
) -> EnergyMicroJ {
    let heat =
        fuel_mass.0 as u128 * fuel_lhv_j_per_kg as u128 * efficiency_x1000 as u128 / 1_000_000;
    EnergyMicroJ(heat.min(i64::MAX as u128) as i64)
}

pub fn pressure_volume_work_micro_j(pressure: PressurePa, delta_volume: VolumeMm3) -> EnergyMicroJ {
    EnergyMicroJ(pressure.0 as i64 * delta_volume.0 as i64 / 1000)
}

pub fn temperature_after_heat_release_k10(
    initial: Kelvin10,
    heat: EnergyMicroJ,
    air_mass: MassUg,
    cv_air_j_per_kg_k: u16,
) -> Kelvin10 {
    if heat.0 <= 0 || air_mass.0 == 0 || cv_air_j_per_kg_k == 0 {
        return initial;
    }

    let delta_k10 = heat.0 as u128 * 10_000 / (air_mass.0 as u128 * cv_air_j_per_kg_k as u128);
    Kelvin10(
        initial
            .0
            .saturating_add(delta_k10.min(u16::MAX as u128) as u16),
    )
}

pub fn polytropic_pressure_pa(
    reference_pressure: PressurePa,
    reference_volume: VolumeMm3,
    current_volume: VolumeMm3,
    gamma_x1000: u16,
) -> PressurePa {
    if reference_pressure.0 <= 0 || reference_volume.0 == 0 || current_volume.0 == 0 {
        return PressurePa(0);
    }

    let ratio_q1000 = reference_volume.0 as i128 * 1000 / current_volume.0 as i128;
    let linear = reference_pressure.0 as i128 * ratio_q1000 / 1000;
    let gamma_extra = gamma_x1000.saturating_sub(1000) as i128;
    let correction = if ratio_q1000 >= 1000 {
        linear * (ratio_q1000 - 1000) * gamma_extra / 1_000_000
    } else {
        -(linear * (1000 - ratio_q1000) * gamma_extra / 1_000_000)
    };
    PressurePa((linear + correction).clamp(0, i32::MAX as i128) as i32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ideal_gas_pressure_tracks_mass_temperature_and_volume() {
        let base = ideal_gas_pressure_pa(MassUg(500_000), Kelvin10(2930), VolumeMm3(500_000), 287);
        let more_mass =
            ideal_gas_pressure_pa(MassUg(750_000), Kelvin10(2930), VolumeMm3(500_000), 287);
        let hotter =
            ideal_gas_pressure_pa(MassUg(500_000), Kelvin10(3500), VolumeMm3(500_000), 287);
        let bigger_volume =
            ideal_gas_pressure_pa(MassUg(500_000), Kelvin10(2930), VolumeMm3(750_000), 287);

        assert!(more_mass.0 > base.0);
        assert!(hotter.0 > base.0);
        assert!(bigger_volume.0 < base.0);
    }

    #[test]
    fn fuel_heat_release_uses_lhv_and_efficiency() {
        let full = fuel_heat_release_micro_j(MassUg(1000), 43_000_000, 1000);
        let half = fuel_heat_release_micro_j(MassUg(1000), 43_000_000, 500);

        assert_eq!(full, EnergyMicroJ(43_000_000));
        assert_eq!(half, EnergyMicroJ(21_500_000));
    }

    #[test]
    fn pressure_volume_work_uses_pa_mm3_to_microj_conversion() {
        assert_eq!(
            pressure_volume_work_micro_j(PressurePa(100_000), VolumeMm3(1000)),
            EnergyMicroJ(100_000)
        );
    }

    #[test]
    fn heat_release_raises_temperature() {
        let initial = Kelvin10(2930);
        let heated = temperature_after_heat_release_k10(
            initial,
            EnergyMicroJ(10_000_000),
            MassUg(500_000),
            718,
        );

        assert!(heated.0 > initial.0);
    }

    #[test]
    fn polytropic_pressure_rises_as_volume_shrinks() {
        let reference = PressurePa(100_000);
        let bdc = VolumeMm3(500_000);
        let tdc = VolumeMm3(100_000);
        let expanded = VolumeMm3(750_000);

        assert!(polytropic_pressure_pa(reference, bdc, tdc, 1350).0 > reference.0);
        assert!(polytropic_pressure_pa(reference, bdc, expanded, 1350).0 < reference.0);
    }
}
