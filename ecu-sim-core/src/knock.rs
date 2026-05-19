use crate::{config::PlantConfig, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct KnockFrame<const CYL: usize> {
    pub knock_risk_x1000: [u16; CYL],
    pub knock_event: [bool; CYL],
    pub knock_intensity_x100: u16,
}

impl<const CYL: usize> KnockFrame<CYL> {
    pub const fn empty() -> Self {
        Self {
            knock_risk_x1000: [0; CYL],
            knock_event: [false; CYL],
            knock_intensity_x100: 0,
        }
    }
}

pub fn estimate_knock_risk<const CYL: usize>(
    config: &PlantConfig<CYL>,
    map_kpa10: Kpa10,
    rpm: Rpm,
    spark_advance: Degrees10,
    lambda_x1000: u16,
    iat_c10: Celsius10,
) -> u16 {
    estimate_knock_risk_with_physics(
        config,
        map_kpa10,
        rpm,
        spark_advance,
        lambda_x1000,
        iat_c10,
        PressurePa(0),
        Kelvin10((iat_c10.0 as i32 + 2731).clamp(1, u16::MAX as i32) as u16),
    )
}

#[allow(clippy::too_many_arguments)]
pub fn estimate_knock_risk_with_physics<const CYL: usize>(
    config: &PlantConfig<CYL>,
    map_kpa10: Kpa10,
    rpm: Rpm,
    spark_advance: Degrees10,
    lambda_x1000: u16,
    iat_c10: Celsius10,
    pmax_pa: PressurePa,
    cylinder_temp_k10: Kelvin10,
) -> u16 {
    let load = map_kpa10.0.saturating_sub(600) as u32;
    let rpm_term = (rpm.0 / 100).min(80) as i64;
    let advance = (spark_advance.0 as i32 - 150).max(0) as i64;
    let lean = lambda_x1000.saturating_sub(1000) as i64;
    let iat = (iat_c10.0 as i32 - 300).max(0) as i64;
    let pmax = (pmax_pa.0 - config.thermo.p_ref_pa.0).max(0) as i64 / 10_000;
    let cylinder_temp = (cylinder_temp_k10.0 as i32 - 3000).max(0) as i64 / 10;
    let compression = config.engine.compression_ratio_x100.saturating_sub(900) as i64;
    let octane = config.knock.fuel_octane_x10.saturating_sub(870) as i64;
    let rich_margin = 1000u16.saturating_sub(lambda_x1000) as i64;
    let weighted_pressure = pmax * config.knock.pmax_weight_x1000 as i64 / 1000;
    let weighted_temp = (cylinder_temp + iat) * config.knock.temp_weight_x1000 as i64 / 1000;
    let weighted_advance = advance * config.knock.advance_weight_x1000 as i64 / 500;
    let weighted_compression = compression * config.knock.compression_weight_x1000 as i64 / 1000;
    let octane_credit = octane * config.knock.octane_credit_x1000 as i64 / 1000;
    let rich_credit = rich_margin * config.knock.rich_margin_credit_x1000 as i64 / 3000;
    let risk = load as i64 / 2
        + rpm_term
        + lean / 3
        + weighted_pressure
        + weighted_temp
        + weighted_advance
        + weighted_compression
        - octane_credit
        - rich_credit;

    risk.clamp(0, 1000) as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    #[test]
    fn physics_knock_risk_rises_with_pressure_temperature_and_advance() {
        let cfg = PlantConfig::<4>::default_four();
        let calm = estimate_knock_risk_with_physics(
            &cfg,
            Kpa10(800),
            Rpm(2500),
            Degrees10(120),
            950,
            Celsius10(250),
            PressurePa(120_000),
            Kelvin10(3100),
        );
        let risky = estimate_knock_risk_with_physics(
            &cfg,
            Kpa10(1400),
            Rpm(5500),
            Degrees10(420),
            1150,
            Celsius10(700),
            PressurePa(900_000),
            Kelvin10(4500),
        );

        assert!(risky > calm);
    }

    #[test]
    fn higher_octane_and_rich_margin_reduce_knock_risk() {
        let mut low_octane = PlantConfig::<4>::default_four();
        low_octane.knock.fuel_octane_x10 = 870;
        let mut high_octane = low_octane;
        high_octane.knock.fuel_octane_x10 = 1050;
        let low = estimate_knock_risk_with_physics(
            &low_octane,
            Kpa10(1200),
            Rpm(4000),
            Degrees10(350),
            1100,
            Celsius10(500),
            PressurePa(700_000),
            Kelvin10(4200),
        );
        let high = estimate_knock_risk_with_physics(
            &high_octane,
            Kpa10(1200),
            Rpm(4000),
            Degrees10(350),
            950,
            Celsius10(500),
            PressurePa(700_000),
            Kelvin10(4200),
        );

        assert!(high < low);
    }
}
