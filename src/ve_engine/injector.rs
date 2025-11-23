//! Injector Pulse Width Calculation
//!
//! Converts fuel mass requirement to injector pulse width.
//! Accounts for injector flow rate, fuel density, and dead time.

use super::types::InjectorConfig;

/// Calculate injector pulse width from fuel mass
///
/// # Arguments
/// * `fuel_mass_mg` - Required fuel mass in milligrams
/// * `flow_rate_cc_min` - Injector flow rate at reference pressure (cc/min)
/// * `fuel_density_mg_cc` - Fuel density (mg/cc, gasoline ~750)
///
/// # Returns
/// Base pulse width in microseconds (without dead time)
///
/// # Theory
/// flow_rate_mg_us = (flow_cc_min × density_mg_cc) / (60 × 1,000,000)
/// pulse_width_us = fuel_mass_mg / flow_rate_mg_us
///
/// # Example
/// ```
/// use ecu_core::ve_engine::injector::calculate_pulse_width;
///
/// let pw = calculate_pulse_width(
///     35,    // 35mg fuel needed
///     440,   // 440 cc/min injector
///     750,   // Gasoline density
/// );
///
/// // 440 cc/min = 330,000 mg/min = 5,500 mg/s = 5.5 mg/ms
/// // 35mg / 5.5 mg/ms ≈ 6.4ms = 6400us
/// assert!(pw > 6000 && pw < 7000, "pw = {}", pw);
/// ```
pub fn calculate_pulse_width(
    fuel_mass_mg: u32,
    flow_rate_cc_min: u16,
    fuel_density_mg_cc: u16,
) -> u32 {
    // Convert flow rate to mg/us
    // flow_mg_min = flow_cc_min × density_mg_cc
    let flow_mg_min = flow_rate_cc_min as u32 * fuel_density_mg_cc as u32;

    // flow_mg_us = flow_mg_min / (60 × 1,000,000)
    // Simplify: flow_mg_us × 60,000,000 = flow_mg_min
    // So: pulse_width_us = (fuel_mass_mg × 60,000,000) / flow_mg_min

    if flow_mg_min == 0 {
        return 1000; // Safe default
    }

    (fuel_mass_mg * 60_000_000) / flow_mg_min
}

/// Calculate injector dead time based on battery voltage
///
/// Dead time (latency) is the delay between injector signal and actual fuel flow.
/// It varies with battery voltage due to electromagnetic solenoid response.
///
/// # Arguments
/// * `battery_voltage_mv` - Battery voltage in millivolts
/// * `config` - Injector configuration with dead time curve
///
/// # Returns
/// Dead time in microseconds
///
/// # Example
/// ```
/// use ecu_core::ve_engine::injector::calculate_dead_time;
/// use ecu_core::ve_engine::types::InjectorConfig;
///
/// let config = InjectorConfig {
///     engine_displacement_cc: 2000,
///     num_cylinders: 4,
///     flow_rate_cc_min: 440,
///     reference_pressure_kpa: 300,
///     fuel_density_mg_cc: 750,
///     dead_time_curve: [
///         (90, 1500), (100, 1300), (110, 1150), (120, 1000),
///         (130, 900), (140, 800), (150, 750), (160, 700),
///     ],
/// };
/// let dead_time = calculate_dead_time(13500, &config);
///
/// // At 13.5V, dead time should be ~900us
/// assert!(dead_time > 800 && dead_time < 1000, "dead_time = {}", dead_time);
/// ```
pub fn calculate_dead_time(battery_voltage_mv: u16, config: &InjectorConfig) -> u16 {
    let voltage_x10 = (battery_voltage_mv / 100) as u8; // Convert to voltage × 10

    // Find bounding points in curve
    for i in 0..7 {
        let (v1, t1) = config.dead_time_curve[i];
        let (v2, t2) = config.dead_time_curve[i + 1];

        if v2 == 0 {
            // End of curve, use last valid value
            return t1;
        }

        if voltage_x10 >= v1 && voltage_x10 <= v2 {
            // Linear interpolation
            if v2 == v1 {
                return t1;
            }

            let slope = (t2 as i32 - t1 as i32) / (v2 as i32 - v1 as i32);
            let offset = voltage_x10 as i32 - v1 as i32;
            let result = t1 as i32 + slope * offset;

            return result.max(0) as u16;
        }
    }

    // Out of range - use first or last value
    if voltage_x10 < config.dead_time_curve[0].0 {
        config.dead_time_curve[0].1
    } else {
        // Find last valid entry
        for i in (0..8).rev() {
            if config.dead_time_curve[i].0 != 0 {
                return config.dead_time_curve[i].1;
            }
        }
        1000 // Fallback
    }
}

/// Calculate effective pulse width (base + dead time + corrections)
///
/// This is a convenience function that combines all pulse width calculations.
///
/// # Arguments
/// * `fuel_mass_mg` - Required fuel mass
/// * `battery_voltage_mv` - Battery voltage
/// * `config` - Injector configuration
///
/// # Returns
/// Total pulse width in microseconds
pub fn calculate_effective_pulse_width(
    fuel_mass_mg: u32,
    battery_voltage_mv: u16,
    config: &InjectorConfig,
) -> u32 {
    let base_pw = calculate_pulse_width(
        fuel_mass_mg,
        config.flow_rate_cc_min,
        config.fuel_density_mg_cc,
    );

    let dead_time = calculate_dead_time(battery_voltage_mv, config);

    base_pw.saturating_add(dead_time as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pulse_width_calculation() {
        // 440 cc/min injector, 750 mg/cc fuel, 35mg needed
        let pw = calculate_pulse_width(35, 440, 750);

        // 440 cc/min × 750 mg/cc = 330,000 mg/min
        // 330,000 mg/min = 5,500 mg/s
        // 35 mg / 5.5 mg/ms = 6.36ms ≈ 6360us
        assert!(pw > 6000 && pw < 7000, "pw = {pw}");
    }

    #[test]
    fn test_pulse_width_scales_with_fuel_mass() {
        let pw_20mg = calculate_pulse_width(20, 440, 750);
        let pw_40mg = calculate_pulse_width(40, 440, 750);

        // Double fuel should double pulse width
        let ratio = (pw_40mg as f32) / (pw_20mg as f32);
        assert!(ratio > 1.95 && ratio < 2.05, "ratio = {ratio}");
    }

    #[test]
    fn test_pulse_width_scales_with_flow_rate() {
        let pw_440 = calculate_pulse_width(35, 440, 750);
        let pw_880 = calculate_pulse_width(35, 880, 750);

        // Double flow rate should halve pulse width
        let ratio = (pw_440 as f32) / (pw_880 as f32);
        assert!(ratio > 1.95 && ratio < 2.05, "ratio = {ratio}");
    }

    #[test]
    fn test_dead_time_at_nominal_voltage() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let dead_time = calculate_dead_time(13000, &config);

        // At 13V, should be ~900us
        assert!(
            dead_time > 850 && dead_time < 950,
            "dead_time = {dead_time}"
        );
    }

    #[test]
    fn test_dead_time_at_low_voltage() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let dead_time = calculate_dead_time(10000, &config);

        // At 10V, should be ~1300us (higher)
        assert!(
            dead_time > 1250 && dead_time < 1350,
            "dead_time = {dead_time}"
        );
    }

    #[test]
    fn test_dead_time_at_high_voltage() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let dead_time = calculate_dead_time(15000, &config);

        // At 15V, should be ~750us (lower)
        assert!(
            dead_time > 700 && dead_time < 800,
            "dead_time = {dead_time}"
        );
    }

    #[test]
    fn test_dead_time_interpolation() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };

        // Test midpoint between 12V and 13V
        let dead_12v = calculate_dead_time(12000, &config);
        let dead_13v = calculate_dead_time(13000, &config);
        let dead_12_5v = calculate_dead_time(12500, &config);

        // Should be between the two
        assert!(
            dead_12_5v < dead_12v,
            "12.5V: {dead_12_5v}, 12V: {dead_12v}"
        );
        assert!(
            dead_12_5v > dead_13v,
            "12.5V: {dead_12_5v}, 13V: {dead_13v}"
        );
    }

    #[test]
    fn test_effective_pulse_width() {
        let config = InjectorConfig {
            engine_displacement_cc: 2000,
            num_cylinders: 4,
            flow_rate_cc_min: 440,
            reference_pressure_kpa: 300,
            fuel_density_mg_cc: 750,
            dead_time_curve: [
                (90, 1500),
                (100, 1300),
                (110, 1150),
                (120, 1000),
                (130, 900),
                (140, 800),
                (150, 750),
                (160, 700),
            ],
        };
        let total_pw = calculate_effective_pulse_width(35, 13000, &config);

        // Should be base (~6400us) + dead time (~900us) ≈ 7300us
        assert!(total_pw > 7000 && total_pw < 7600, "total_pw = {total_pw}");
    }

    #[test]
    fn test_zero_flow_safety() {
        // Should not panic with zero flow
        let pw = calculate_pulse_width(35, 0, 750);
        assert_eq!(pw, 1000); // Safe default
    }

    #[test]
    fn test_realistic_idle_scenario() {
        // Idle: 2.0L 4-cyl, 800 RPM, 40 kPa, 70% VE
        // Air mass ≈ 200mg per cycle
        // AFR 14.7:1 → fuel mass ≈ 13.6mg
        let pw = calculate_pulse_width(14, 440, 750);

        // Expected: ~2.5ms
        assert!(pw > 2000 && pw < 3000, "pw = {pw}");
    }

    #[test]
    fn test_realistic_cruise_scenario() {
        // Cruise: 2.0L 4-cyl, 3000 RPM, 100 kPa, 85% VE
        // Air mass ≈ 510mg per cycle
        // AFR 14.7:1 → fuel mass ≈ 35mg
        let pw = calculate_pulse_width(35, 440, 750);

        // Expected: ~6.4ms
        assert!(pw > 6000 && pw < 7000, "pw = {pw}");
    }

    #[test]
    fn test_realistic_wot_scenario() {
        // WOT: 2.0L 4-cyl, 6000 RPM, 100 kPa, 95% VE
        // Air mass ≈ 570mg per cycle
        // AFR 12.5:1 (rich for power) → fuel mass ≈ 46mg
        let pw = calculate_pulse_width(46, 440, 750);

        // Expected: ~8.4ms
        assert!(pw > 8000 && pw < 9000, "pw = {pw}");
    }
}
