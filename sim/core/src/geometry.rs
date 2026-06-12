use crate::{config::EngineGeometryConfig, types::*};

const Q15_ONE: i64 = 32_768;
const Q16_ONE: i128 = 65_536;
const PI_NUM: u128 = 355;
const PI_DEN: u128 = 113;
const UM3_PER_MM3: u128 = 1_000_000_000;

pub const SIN_Q15_QUARTER_DEG10: [i32; 901] = build_sin_q15_quarter_table();

pub fn sin_deg10_q15(angle: CrankDeg10) -> i32 {
    let mut x = (angle.0 as u32) % 3600;
    let negative = x >= 1800;
    if negative {
        x -= 1800;
    }
    if x > 900 {
        x = 1800 - x;
    }

    let magnitude = SIN_Q15_QUARTER_DEG10[x as usize];

    if negative {
        -magnitude
    } else {
        magnitude
    }
}

const fn build_sin_q15_quarter_table() -> [i32; 901] {
    let mut table = [0; 901];
    let mut x = 0;
    while x <= 900 {
        table[x] = sin_q15_quarter_approx(x as u32);
        x += 1;
    }
    table
}

const fn sin_q15_quarter_approx(x: u32) -> i32 {
    if x == 900 {
        return Q15_ONE as i32;
    }
    let numerator = 4u64 * x as u64 * (1800 - x) as u64;
    let denominator = 4_050_000u64 - x as u64 * (1800 - x) as u64;
    ((numerator * Q15_ONE as u64) / denominator) as i32
}

pub fn cos_deg10_q15(angle: CrankDeg10) -> i32 {
    sin_deg10_q15(normalize_deg10(angle.0 as u32 + 900))
}

pub fn piston_area_um2(config: EngineGeometryConfig) -> u64 {
    let bore = config.bore_um as u128;
    ((bore * bore * PI_NUM) / (4 * PI_DEN)) as u64
}

pub fn swept_volume_mm3(config: EngineGeometryConfig) -> VolumeMm3 {
    let volume_um3 = piston_area_um2(config) as u128 * config.stroke_um as u128;
    VolumeMm3((volume_um3 / UM3_PER_MM3) as u64)
}

pub fn clearance_volume_mm3(config: EngineGeometryConfig) -> VolumeMm3 {
    let swept = swept_volume_mm3(config).0 as u128;
    let compression_minus_one = config.compression_ratio_x100.saturating_sub(100) as u128;
    if compression_minus_one == 0 {
        return VolumeMm3(0);
    }
    VolumeMm3(((swept * 100) / compression_minus_one) as u64)
}

pub fn piston_position_um(config: EngineGeometryConfig, local_angle: CrankDeg10) -> u64 {
    let crank_radius = config.stroke_um as u128 / 2;
    let rod_length = config.rod_length_um as u128;
    let sin_q15 = sin_deg10_q15(local_angle) as i128;
    let cos_q15 = cos_deg10_q15(local_angle) as i128;

    let term_one =
        crank_radius.saturating_mul((Q15_ONE as i128 - cos_q15).max(0) as u128) / Q15_ONE as u128;
    let sin_squared = (sin_q15 * sin_q15) as u128;
    let crank_squared = crank_radius.saturating_mul(crank_radius);
    let rod_squared = rod_length.saturating_mul(rod_length);
    let offset = crank_squared.saturating_mul(sin_squared) / (Q15_ONE as u128 * Q15_ONE as u128);
    let under_sqrt = rod_squared.saturating_sub(offset);
    let rod_projection = isqrt_u128(under_sqrt);

    term_one
        .saturating_add(rod_length)
        .saturating_sub(rod_projection) as u64
}

pub fn cylinder_volume_mm3(config: EngineGeometryConfig, local_angle: CrankDeg10) -> VolumeMm3 {
    let clearance = clearance_volume_mm3(config).0 as u128;
    let swept_um3 = (piston_area_um2(config) as u128)
        .saturating_mul(piston_position_um(config, local_angle) as u128);
    VolumeMm3((clearance + swept_um3 / UM3_PER_MM3) as u64)
}

pub fn dvolume_dtheta_mm3_per_rad_q16(
    config: EngineGeometryConfig,
    local_angle: CrankDeg10,
) -> VolumeDerivativeMm3PerRadQ16 {
    let crank_radius = config.stroke_um as i128 / 2;
    let rod_length = config.rod_length_um as u128;
    let sin_q15 = sin_deg10_q15(local_angle) as i128;
    let cos_q15 = cos_deg10_q15(local_angle) as i128;

    let sin_squared = (sin_q15 * sin_q15) as u128;
    let crank_squared_unsigned = (crank_radius as u128).saturating_mul(crank_radius as u128);
    let rod_squared = rod_length.saturating_mul(rod_length);
    let offset =
        crank_squared_unsigned.saturating_mul(sin_squared) / (Q15_ONE as u128 * Q15_ONE as u128);
    let under_sqrt = rod_squared.saturating_sub(offset);
    let denominator = isqrt_u128(under_sqrt).max(1) as i128;

    let term_one_q15 = crank_radius.saturating_mul(sin_q15);
    let term_two_q15 = crank_radius
        .saturating_mul(crank_radius)
        .saturating_mul(sin_q15)
        .saturating_mul(cos_q15)
        / Q15_ONE as i128
        / denominator;
    let dx_dtheta_um_per_rad_q15 = term_one_q15.saturating_add(term_two_q15);
    let volume_um3_per_rad_q15 = piston_area_um2(config) as i128 * dx_dtheta_um_per_rad_q15;
    let volume_mm3_per_rad_q16 = volume_um3_per_rad_q15.saturating_mul(2) / UM3_PER_MM3 as i128;

    VolumeDerivativeMm3PerRadQ16(volume_mm3_per_rad_q16 as i64)
}

pub fn pressure_delta_to_torque_nm_x100(
    delta_pressure_pa: PressurePa,
    dvolume: VolumeDerivativeMm3PerRadQ16,
) -> TorqueNmX100 {
    let numerator = delta_pressure_pa.0 as i128 * dvolume.0 as i128 * 100;
    let denominator = UM3_PER_MM3 as i128 * Q16_ONE;
    TorqueNmX100((numerator / denominator) as i32)
}

pub const fn isqrt_u128(value: u128) -> u128 {
    let mut bit = 1u128 << 126;
    let mut n = value;
    let mut result = 0u128;

    while bit > n {
        bit >>= 2;
    }

    while bit != 0 {
        if n >= result + bit {
            n -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    fn geometry() -> EngineGeometryConfig {
        PlantConfig::<4>::default_four().engine
    }

    #[test]
    fn fixed_point_trig_matches_key_angles() {
        assert_eq!(SIN_Q15_QUARTER_DEG10.len(), 901);
        assert_eq!(SIN_Q15_QUARTER_DEG10[0], 0);
        assert_eq!(SIN_Q15_QUARTER_DEG10[900], 32_768);
        assert_eq!(sin_deg10_q15(CrankDeg10(0)), 0);
        assert_eq!(sin_deg10_q15(CrankDeg10(900)), 32_768);
        assert_eq!(sin_deg10_q15(CrankDeg10(1800)), 0);
        assert_eq!(sin_deg10_q15(CrankDeg10(2700)), -32_768);
        assert!((sin_deg10_q15(CrankDeg10(300)) - 16_384).abs() <= 2);
        assert!((sin_deg10_q15(CrankDeg10(600)) - 28_378).abs() <= 250);
        assert!((sin_deg10_q15(CrankDeg10(450)) - 23_170).abs() <= 300);
    }

    #[test]
    fn integer_sqrt_floors_exactly() {
        assert_eq!(isqrt_u128(0), 0);
        assert_eq!(isqrt_u128(1), 1);
        assert_eq!(isqrt_u128(15), 3);
        assert_eq!(isqrt_u128(16), 4);
        assert_eq!(isqrt_u128(17), 4);
        assert_eq!(isqrt_u128(1_000_000), 1000);
    }

    #[test]
    fn volume_is_minimum_at_tdc_and_maximum_at_bdc() {
        let cfg = geometry();
        let tdc = cylinder_volume_mm3(cfg, CrankDeg10(0));
        let bdc = cylinder_volume_mm3(cfg, CrankDeg10(1800));
        let swept = swept_volume_mm3(cfg);

        assert!(tdc.0 > 0);
        assert!(bdc.0 > tdc.0);
        assert!((bdc.0 - tdc.0).abs_diff(swept.0) <= swept.0 / 100);
    }

    #[test]
    fn dvolume_sign_tracks_compression_and_expansion() {
        let cfg = geometry();

        assert!(dvolume_dtheta_mm3_per_rad_q16(cfg, CrankDeg10(900)).0 > 0);
        assert!(dvolume_dtheta_mm3_per_rad_q16(cfg, CrankDeg10(2700)).0 < 0);
        assert!(dvolume_dtheta_mm3_per_rad_q16(cfg, CrankDeg10(0)).0.abs() < 100);
        assert!(
            dvolume_dtheta_mm3_per_rad_q16(cfg, CrankDeg10(1800))
                .0
                .abs()
                < 100
        );
    }

    #[test]
    fn pressure_to_torque_uses_per_radian_derivative() {
        let torque = pressure_delta_to_torque_nm_x100(
            PressurePa(100_000),
            VolumeDerivativeMm3PerRadQ16(100_000_000),
        );

        assert_eq!(torque, TorqueNmX100(15));
    }

    #[test]
    fn core_geometry_matches_reference_analytic_formula_within_quantization_error() {
        let cfg = geometry();
        let bore_m = cfg.bore_um as f64 / 1_000_000.0;
        let stroke_m = cfg.stroke_um as f64 / 1_000_000.0;
        let rod_length_m = cfg.rod_length_um as f64 / 1_000_000.0;
        let compression_ratio = cfg.compression_ratio_x100 as f64 / 100.0;

        let a = stroke_m / 2.0;
        let piston_area_mm2 = core::f64::consts::PI * bore_m * bore_m / 4.0 * 1_000_000.0;
        let clearance_mm3 = (piston_area_mm2 * stroke_m / (compression_ratio - 1.0)) * 1_000.0;

        for angle_deg10 in (0..7200).step_by(25) {
            let angle = CrankDeg10(angle_deg10);
            let theta_rad = f64::from(angle_deg10).to_radians() / 10.0;
            let sin_theta = theta_rad.sin();
            let cos_theta = theta_rad.cos();
            let root = (rod_length_m * rod_length_m - a * a * sin_theta * sin_theta).sqrt();
            let displacement_m = a * (1.0 - cos_theta) + rod_length_m - root;
            let piston_position_um = (displacement_m * 1_000_000.0).round();
            let expected_volume_mm3 = clearance_mm3 + piston_area_mm2 * piston_position_um / 1000.0;

            let core_volume_mm3 = cylinder_volume_mm3(cfg, angle).0 as f64;
            assert!(
                (core_volume_mm3 - expected_volume_mm3).abs() <= 500.0,
                "volume mismatch at {angle_deg10}: core={core_volume_mm3} analytic={expected_volume_mm3}",
            );

            let core_dv_q16 = dvolume_dtheta_mm3_per_rad_q16(cfg, angle).0 as f64;
            let dx_dtheta_um = (a * 1_000_000.0) * sin_theta
                + ((a * 1_000_000.0) * (a * 1_000_000.0) * sin_theta * cos_theta
                    / (root * 1_000_000.0));
            let piston_area_um2 =
                core::f64::consts::PI * (cfg.bore_um as f64) * (cfg.bore_um as f64) / 4.0;
            let expected_dv_q16 = piston_area_um2 * dx_dtheta_um * 65_536.0 / 1_000_000_000.0;
            assert!(
                (core_dv_q16 - expected_dv_q16).abs() <= 35_000_000.0,
                "dV/dtheta mismatch at {angle_deg10}: core={core_dv_q16} analytic={expected_dv_q16}",
            );
        }
    }
}
