//! Speed-Density Air Mass Calculation
//!
//! Calculates air mass per cylinder per cycle using the speed-density equation.
//! This is a MAF-less approach that uses MAP, RPM, IAT, and VE.
//!
//! # Theory
//!
//! The ideal gas law: PV = nRT
//!
//! Air mass = (MAP × Displacement × VE) / (R × Temperature)
//!
//! Where:
//! - MAP: Manifold Absolute Pressure (kPa)
//! - Displacement: Cylinder volume (cc)
//! - VE: Volumetric Efficiency (%)
//! - R: Gas constant for air
//! - T: Intake air temperature (Kelvin)
//!
//! # Simplifications
//!
//! We use integer arithmetic with careful scaling to avoid floating point.
//! The result is air mass in milligrams (mg).

#![cfg_attr(not(test), no_std)]

/// Calculate air mass per cylinder per cycle
///
/// Uses the speed-density equation with integer-only arithmetic.
///
/// # Arguments
/// * `engine_displacement_cc` - Total engine displacement
/// * `num_cylinders` - Number of cylinders
/// * `map_kpa` - Manifold absolute pressure
/// * `ve_percent` - Volumetric efficiency (0-255%)
/// * `iat_celsius` - Intake air temperature
///
/// # Returns
/// Air mass in milligrams (mg)
///
/// # Example
/// ```
/// use ecu_core::ve_engine::speed_density::calculate_air_mass;
///
/// let air_mg = calculate_air_mass(
///     2000,  // 2.0L engine
///     4,     // 4 cylinders
///     100,   // 100 kPa (atmospheric)
///     85,    // 85% VE
///     20,    // 20°C intake temp
/// );
///
/// // For 2.0L 4-cyl at 100kPa, 85% VE, 20°C
/// // Displacement per cylinder = 500cc, effective = 250cc
/// // Air mass: 250cc * 1.2 mg/cc * 0.85 = 255mg
/// assert!(air_mg > 240 && air_mg < 270);
/// ```
pub fn calculate_air_mass(
    engine_displacement_cc: u16,
    num_cylinders: u8,
    map_kpa: u16,
    ve_percent: u8,
    iat_celsius: i16,
) -> u32 {
    // Calculate displacement per cylinder
    let displacement_per_cyl = engine_displacement_cc as u32 / num_cylinders as u32;

    // For 4-stroke engine: only half the displacement is used per cycle
    let effective_displacement = displacement_per_cyl / 2;

    // Calculate air density factor
    // Standard conditions: 101.325 kPa, 293K (20°C)
    // Air density at standard = ~1.2 mg/cc
    let air_density_factor = calculate_air_density_factor(map_kpa, iat_celsius);

    // Calculate air mass (mg)
    // mass = volume × density × VE
    // density_factor is scaled by 100 (120 = 1.2 mg/cc), VE is %, so divide by 10000
    let air_mass = (effective_displacement * air_density_factor * ve_percent as u32) / 10000;

    air_mass
}

/// Calculate air density factor relative to standard conditions
///
/// Returns density factor scaled by 100 (100 = 1.0x standard density)
///
/// # Arguments
/// * `map_kpa` - Manifold absolute pressure
/// * `iat_celsius` - Intake air temperature
///
/// # Returns
/// Density factor × 100
///
/// # Theory
/// ρ/ρ₀ = (P/P₀) × (T₀/T)
///
/// Where:
/// - ρ = actual air density
/// - ρ₀ = standard air density (at 101.325 kPa, 20°C)
/// - P = actual pressure
/// - P₀ = standard pressure
/// - T = actual temperature (Kelvin)
/// - T₀ = standard temperature (293K)
fn calculate_air_density_factor(map_kpa: u16, iat_celsius: i16) -> u32 {
    const STANDARD_PRESSURE_KPA: u32 = 101;  // Simplified
    const STANDARD_TEMP_K: u32 = 293;  // 20°C
    const AIR_DENSITY_MG_CC: u32 = 120;  // 1.2 mg/cc scaled by 100

    // Convert IAT to Kelvin
    let iat_kelvin = (iat_celsius + 273) as u32;

    // Pressure ratio (scaled by 100)
    let pressure_ratio = (map_kpa as u32 * 100) / STANDARD_PRESSURE_KPA;

    // Temperature ratio (scaled by 100)
    let temp_ratio = (STANDARD_TEMP_K * 100) / iat_kelvin;

    // Density factor = pressure_ratio × temp_ratio × base_density
    // Both ratios are scaled by 100, so divide by 10000
    let density_factor = (pressure_ratio * temp_ratio * AIR_DENSITY_MG_CC) / 10000;

    density_factor
}

/// Calculate air mass flow rate (mg/s)
///
/// This is useful for calculating MAF equivalent from speed-density.
///
/// # Arguments
/// * `air_mass_per_cycle_mg` - Air mass per cylinder per cycle
/// * `rpm` - Engine speed
/// * `num_cylinders` - Number of cylinders
///
/// # Returns
/// Air mass flow in mg/s
///
/// # Example
/// ```
/// use ecu_core::ve_engine::speed_density::calculate_maf;
///
/// let maf_mg_s = calculate_maf(
///     500,   // 500mg per cycle
///     3000,  // 3000 RPM
///     4,     // 4 cylinders
/// );
///
/// // At 3000 RPM, 4-cyl does 1500 power cycles per minute = 25 Hz
/// // 4 cylinders × 25 Hz = 100 injections/sec
/// // 500mg × 100 = 50,000 mg/s = 50 g/s
/// assert!(maf_mg_s > 45000 && maf_mg_s < 55000);
/// ```
pub fn calculate_maf(
    air_mass_per_cycle_mg: u32,
    rpm: u16,
    num_cylinders: u8,
) -> u32 {
    // For 4-stroke: power strokes per second = (RPM / 60) / 2
    // Total firing events per second = power_strokes × num_cylinders
    let firing_events_per_minute = (rpm as u32 * num_cylinders as u32) / 2;
    let firing_events_per_second = firing_events_per_minute / 60;

    // MAF = air_mass_per_cycle × firing_rate
    air_mass_per_cycle_mg * firing_events_per_second
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_air_mass_at_standard_conditions() {
        // 2.0L 4-cyl at 100 kPa, 85% VE, 20°C
        let air_mass = calculate_air_mass(2000, 4, 100, 85, 20);

        // Expected: ~255mg (250cc effective × 1.2 mg/cc × 0.85)
        assert!(air_mass > 240, "air_mass = {}", air_mass);
        assert!(air_mass < 270, "air_mass = {}", air_mass);
    }

    #[test]
    fn test_air_mass_at_boost() {
        // Same engine at 150 kPa (boost)
        let air_mass = calculate_air_mass(2000, 4, 150, 85, 20);

        // Should be ~1.5x more air (~380mg)
        assert!(air_mass > 360, "air_mass = {}", air_mass);
        assert!(air_mass < 400, "air_mass = {}", air_mass);
    }

    #[test]
    fn test_air_mass_cold() {
        // Same engine at 0°C (cold air is dense)
        let air_mass = calculate_air_mass(2000, 4, 100, 85, 0);

        // Should be slightly more than at 20°C
        let warm_air = calculate_air_mass(2000, 4, 100, 85, 20);
        assert!(air_mass > warm_air, "cold: {}, warm: {}", air_mass, warm_air);
    }

    #[test]
    fn test_air_mass_hot() {
        // Same engine at 60°C (hot air is less dense)
        let air_mass = calculate_air_mass(2000, 4, 100, 85, 60);

        // Should be less than at 20°C
        let warm_air = calculate_air_mass(2000, 4, 100, 85, 20);
        assert!(air_mass < warm_air, "hot: {}, warm: {}", air_mass, warm_air);
    }

    #[test]
    fn test_air_mass_scales_with_ve() {
        let ve_80 = calculate_air_mass(2000, 4, 100, 80, 20);
        let ve_100 = calculate_air_mass(2000, 4, 100, 100, 20);

        // 100% VE should give 25% more air than 80% VE
        let ratio = (ve_100 as f32) / (ve_80 as f32);
        assert!(ratio > 1.2 && ratio < 1.3, "ratio = {}", ratio);
    }

    #[test]
    fn test_air_density_factor_standard() {
        let factor = calculate_air_density_factor(100, 20);

        // At standard conditions, factor should be ~120 (1.2 mg/cc)
        assert!(factor > 110 && factor < 130, "factor = {}", factor);
    }

    #[test]
    fn test_maf_calculation() {
        // 500mg per cycle, 3000 RPM, 4 cylinders
        let maf = calculate_maf(500, 3000, 4);

        // 3000 RPM / 2 (4-stroke) = 1500 power strokes/min = 25 Hz
        // 4 cylinders × 25 Hz = 100 events/sec
        // 500mg × 100 = 50,000 mg/s
        assert!(maf > 45000 && maf < 55000, "maf = {}", maf);
    }

    #[test]
    fn test_maf_doubles_with_rpm() {
        let maf_3000 = calculate_maf(500, 3000, 4);
        let maf_6000 = calculate_maf(500, 6000, 4);

        // Double RPM should double MAF
        let ratio = (maf_6000 as f32) / (maf_3000 as f32);
        assert!(ratio > 1.95 && ratio < 2.05, "ratio = {}", ratio);
    }

    #[test]
    fn test_small_engine() {
        // 600cc 2-cyl motorcycle engine
        let air_mass = calculate_air_mass(600, 2, 100, 80, 20);

        // 300cc per cylinder, 150cc effective, 150 * 1.2 * 0.8 = 144mg
        assert!(air_mass > 135 && air_mass < 155, "air_mass = {}", air_mass);
    }

    #[test]
    fn test_large_engine() {
        // 6.2L V8 (GM LS3)
        let air_mass = calculate_air_mass(6200, 8, 100, 90, 20);

        // 775cc per cylinder, 387.5cc effective, 387.5 * 1.2 * 0.9 = 418mg
        assert!(air_mass > 405 && air_mass < 430, "air_mass = {}", air_mass);
    }
}
