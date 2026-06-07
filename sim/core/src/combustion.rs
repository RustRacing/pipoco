use crate::{
    config::{CombustionConfig, PlantConfig},
    fuel::AcceptedInjection,
    spark::AcceptedSpark,
    state::MisfireReason,
    types::*,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderCombustion {
    pub timestamp_us: Micros,
    pub cylinder: CylinderIndex,
    pub cycle_angle_deg10: CrankDeg10,
    pub torque_nm_x100: TorqueNmX100,
    pub afr_x100: u16,
    pub lambda_x1000: u16,
    pub quality_x1000: u16,
    pub misfire: Option<MisfireReason>,
}

impl CylinderCombustion {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: Micros(0),
            cylinder: CylinderIndex(0),
            cycle_angle_deg10: CrankDeg10(0),
            torque_nm_x100: TorqueNmX100(0),
            afr_x100: 0,
            lambda_x1000: 1000,
            quality_x1000: 0,
            misfire: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CombustionFrame<const CYL: usize> {
    pub cylinders: [CylinderCombustion; CYL],
    pub total_torque_nm_x100: TorqueNmX100,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CombustionPhasingTarget {
    pub ca50_opt_deg_atdc_x10: i16,
    pub pmax_opt_deg_atdc_x10: i16,
    pub ca50_sensitivity_x1000: u16,
    pub pmax_sensitivity_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CombustionPhasingResult {
    pub ca10_deg_x10: i16,
    pub ca50_deg_x10: i16,
    pub ca90_deg_x10: i16,
    pub pmax_deg_x10: i16,
    pub phasing_efficiency_x1000: u16,
}

impl<const CYL: usize> CombustionFrame<CYL> {
    pub const fn empty() -> Self {
        Self {
            cylinders: [CylinderCombustion::empty(); CYL],
            total_torque_nm_x100: TorqueNmX100(0),
        }
    }
}

pub fn combustion_phasing_result_with_delay(
    config: CombustionConfig,
    ignition_delay_deg10: u16,
    spark_advance_deg10: CrankDeg10,
) -> CombustionPhasingResult {
    let target = CombustionPhasingTarget {
        ca50_opt_deg_atdc_x10: config.ca50_target_at_mbt_deg10,
        pmax_opt_deg_atdc_x10: config.pmax_target_at_mbt_deg10,
        ca50_sensitivity_x1000: config.ca50_sensitivity_x1000,
        pmax_sensitivity_x1000: config.pmax_sensitivity_x1000,
    };
    let ca10_offset = crate::burn::crank_angle_after_tdc_for_burn_fraction(
        &config.burn_curve,
        1000,
        config.burn_duration_deg10,
    )
    .unwrap_or(Degrees10(0))
    .0;
    let ca50_offset = crate::burn::crank_angle_after_tdc_for_burn_fraction(
        &config.burn_curve,
        5000,
        config.burn_duration_deg10,
    )
    .unwrap_or(Degrees10(0))
    .0;
    let ca90_offset = crate::burn::crank_angle_after_tdc_for_burn_fraction(
        &config.burn_curve,
        9000,
        config.burn_duration_deg10,
    )
    .unwrap_or(Degrees10(0))
    .0;
    let burn_start_deg10 = ignition_delay_deg10 as i32 - spark_advance_deg10.0 as i32;
    let ca10 = (burn_start_deg10 + ca10_offset as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    let ca50 = (burn_start_deg10 + ca50_offset as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    let ca90 = (burn_start_deg10 + ca90_offset as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    let pmax = ca50 + 60;
    let ca50_error = ca50 - target.ca50_opt_deg_atdc_x10 as i32;
    let pmax_error = pmax - target.pmax_opt_deg_atdc_x10 as i32;
    let penalty = squared_penalty(ca50_error, target.ca50_sensitivity_x1000)
        .saturating_add(squared_penalty(pmax_error, target.pmax_sensitivity_x1000));

    CombustionPhasingResult {
        ca10_deg_x10: ca10 as i16,
        ca50_deg_x10: ca50 as i16,
        ca90_deg_x10: ca90 as i16,
        pmax_deg_x10: pmax.clamp(i16::MIN as i32, i16::MAX as i32) as i16,
        phasing_efficiency_x1000: 1000u32.saturating_sub(penalty).max(250) as u16,
    }
}

fn squared_penalty(error_deg10: i32, sensitivity_x1000: u16) -> u32 {
    let error_deg10 = error_deg10.unsigned_abs();
    error_deg10
        .saturating_mul(error_deg10)
        .saturating_mul(sensitivity_x1000 as u32)
        / 1000
}

#[allow(clippy::too_many_arguments)]
pub fn evaluate_combustion_with_residual<const CYL: usize>(
    config: &PlantConfig<CYL>,
    timestamp_us: Micros,
    angle: CrankDeg10,
    cyl: usize,
    air_mass: MassUg,
    fuel: Option<AcceptedInjection>,
    spark: Option<AcceptedSpark>,
    fuel_cut: bool,
    spark_cut: bool,
    residual_fraction_x1000: u16,
) -> CylinderCombustion {
    let mut out = CylinderCombustion::empty();
    out.timestamp_us = timestamp_us;
    out.cylinder = CylinderIndex(cyl as u8);
    out.cycle_angle_deg10 = angle;

    if fuel_cut {
        out.misfire = Some(MisfireReason::FuelCut);
        return out;
    }
    if spark_cut {
        out.misfire = Some(MisfireReason::SparkCut);
        return out;
    }
    let Some(fuel) = fuel else {
        out.misfire = Some(MisfireReason::NoFuel);
        return out;
    };
    let Some(spark) = spark else {
        out.misfire = Some(MisfireReason::NoSpark);
        return out;
    };
    if air_mass.0 < config.combustion.min_air_mass_ug.0 {
        out.misfire = Some(MisfireReason::AirMassTooLow);
        return out;
    }
    if spark.dwell_quality_x1000 < 1000 {
        out.misfire = Some(MisfireReason::InsufficientDwell);
        return out;
    }
    if spark.phase_quality_x1000 == 0 {
        out.misfire = Some(MisfireReason::BadSparkTiming);
        return out;
    }

    let afr_x100 = if fuel.fuel_mass_ug.0 == 0 {
        u16::MAX
    } else {
        clamp_u16(
            (air_mass.0 as u64 * 100 / fuel.fuel_mass_ug.0 as u64) as u32,
            u16::MAX,
        )
    };
    let lambda = clamp_u16(
        (afr_x100 as u32 * 1000) / config.fuel.stoich_afr_x100.max(1) as u32,
        u16::MAX,
    );
    out.afr_x100 = afr_x100;
    out.lambda_x1000 = lambda;

    if lambda < config.combustion.min_lambda_x1000 {
        out.misfire = Some(MisfireReason::TooRich);
        return out;
    }
    if lambda > config.combustion.max_lambda_x1000 {
        out.misfire = Some(MisfireReason::TooLean);
        return out;
    }

    let lambda_error = lambda.abs_diff(1000) as u32;
    let afr_quality = 1000u32.saturating_sub(lambda_error * 2).max(250);
    let phasing = combustion_phasing_result_with_delay(
        config.combustion,
        config.spark.ignition_delay_deg10,
        spark.command.spark_angle_deg10,
    );
    let quality = afr_quality * fuel.phasing_x1000 as u32 / 1000
        * phasing.phasing_efficiency_x1000 as u32
        / 1000
        * crate::residual::residual_combustion_quality_x1000(residual_fraction_x1000) as u32
        / 1000
        * spark.dwell_quality_x1000 as u32
        / 1000;
    out.quality_x1000 = quality as u16;
    let torque =
        (air_mass.0 as i64 * quality as i64 * config.combustion.torque_scale_x100 as i64) / 100_000;
    out.torque_nm_x100 = TorqueNmX100(torque as i32);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    #[test]
    fn spark_sweep_ca50_phasing_peaks_near_configured_target() {
        let cfg = PlantConfig::<4>::default_four();
        let early = combustion_phasing_result_with_delay(
            cfg.combustion,
            cfg.spark.ignition_delay_deg10,
            CrankDeg10(320),
        );
        let near_mbt = combustion_phasing_result_with_delay(
            cfg.combustion,
            cfg.spark.ignition_delay_deg10,
            CrankDeg10(180),
        );
        let late = combustion_phasing_result_with_delay(
            cfg.combustion,
            cfg.spark.ignition_delay_deg10,
            CrankDeg10(40),
        );

        assert!((80..=130).contains(&near_mbt.ca50_deg_x10));
        assert!(near_mbt.phasing_efficiency_x1000 > early.phasing_efficiency_x1000);
        assert!(near_mbt.phasing_efficiency_x1000 > late.phasing_efficiency_x1000);
    }
}
