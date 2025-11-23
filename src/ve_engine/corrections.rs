//! Fuel Corrections
//!
//! Apply environmental and operational corrections to pulse width.
//! All corrections use integer arithmetic scaled by 100 (100 = 1.0x).

use super::types::SensorData;

/// Apply all corrections to pulse width
///
/// # Arguments
/// * `base_pw_us` - Base pulse width in microseconds
/// * `sensors` - Current sensor readings
///
/// # Returns
/// Corrected pulse width in microseconds
///
/// # Corrections Applied
/// 1. CLT correction (cold start enrichment)
/// 2. IAT correction (air density)
/// 3. Battery voltage correction (injector response)
pub fn apply_all_corrections(base_pw_us: u32, sensors: &SensorData) -> u32 {
    let mut pw = base_pw_us;

    // Apply corrections sequentially
    pw = apply_clt_correction(pw, sensors.clt_celsius);
    pw = apply_iat_correction(pw, sensors.iat_celsius);
    pw = apply_voltage_correction(pw, sensors.battery_voltage_mv);

    pw
}

/// Coolant temperature correction
///
/// Cold engines need more fuel for several reasons:
/// - Fuel condenses on cold cylinder walls
/// - Oil is thicker, increasing friction
/// - Worse fuel atomization
///
/// # Arguments
/// * `base_pw_us` - Base pulse width
/// * `clt_celsius` - Coolant temperature
///
/// # Returns
/// Corrected pulse width
///
/// # Correction Curve
/// - Below -20°C: +100% fuel (2.0x)
/// - -20°C to 0°C: +60% fuel (1.6x)
/// - 0°C to 40°C: +30% fuel (1.3x)
/// - 40°C to 70°C: Gradual reduction to 1.0x
/// - Above 70°C: No correction (1.0x)
pub fn apply_clt_correction(base_pw_us: u32, clt_celsius: i16) -> u32 {
    let correction = calculate_clt_correction(clt_celsius);
    (base_pw_us * correction as u32) / 100
}

/// Calculate CLT correction multiplier
///
/// Returns correction factor scaled by 100 (100 = 1.0x)
pub fn calculate_clt_correction(clt_celsius: i16) -> u8 {
    if clt_celsius < -20 {
        200 // 2.0x - very cold
    } else if clt_celsius < 0 {
        160 // 1.6x - cold
    } else if clt_celsius < 40 {
        130 // 1.3x - warming up
    } else if clt_celsius < 50 {
        120 // 1.2x
    } else if clt_celsius < 60 {
        110 // 1.1x
    } else if clt_celsius < 70 {
        105 // 1.05x
    } else {
        100 // 1.0x - fully warm
    }
}

/// Intake air temperature correction
///
/// Air density changes with temperature:
/// - Cold air is dense (more oxygen) → less fuel needed
/// - Hot air is thin (less oxygen) → more fuel needed
///
/// # Arguments
/// * `base_pw_us` - Base pulse width
/// * `iat_celsius` - Intake air temperature
///
/// # Returns
/// Corrected pulse width
///
/// # Correction Curve
/// - Below -20°C: -15% fuel (0.85x)
/// - -20°C to 20°C: Linear from 0.9x to 1.0x
/// - 20°C to 60°C: Linear from 1.0x to 1.1x
/// - Above 60°C: +15% fuel (1.15x)
pub fn apply_iat_correction(base_pw_us: u32, iat_celsius: i16) -> u32 {
    let correction = calculate_iat_correction(iat_celsius);
    (base_pw_us * correction as u32) / 100
}

/// Calculate IAT correction multiplier
///
/// Returns correction factor scaled by 100 (100 = 1.0x)
pub fn calculate_iat_correction(iat_celsius: i16) -> u8 {
    if iat_celsius < -20 {
        85 // 0.85x - very cold, dense air
    } else if iat_celsius < 0 {
        90 // 0.9x - cold
    } else if iat_celsius < 20 {
        95 // 0.95x
    } else if iat_celsius < 40 {
        100 // 1.0x - normal
    } else if iat_celsius < 60 {
        105 // 1.05x - warm
    } else if iat_celsius < 80 {
        110 // 1.1x - hot
    } else {
        115 // 1.15x - very hot
    }
}

/// Battery voltage correction
///
/// Injector opening time varies with battery voltage:
/// - Low voltage → slower solenoid → effectively longer pulse
/// - High voltage → faster solenoid → effectively shorter pulse
///
/// # Arguments
/// * `base_pw_us` - Base pulse width
/// * `voltage_mv` - Battery voltage in millivolts
///
/// # Returns
/// Corrected pulse width
///
/// # Correction Curve
/// - Below 10V: +20% (1.2x)
/// - 10V to 12V: +10% (1.1x)
/// - 12V to 14V: No correction (1.0x)
/// - Above 14V: -5% (0.95x)
pub fn apply_voltage_correction(base_pw_us: u32, voltage_mv: u16) -> u32 {
    let correction = calculate_voltage_correction(voltage_mv);
    (base_pw_us * correction as u32) / 100
}

/// Calculate voltage correction multiplier
///
/// Returns correction factor scaled by 100 (100 = 1.0x)
pub fn calculate_voltage_correction(voltage_mv: u16) -> u8 {
    if voltage_mv < 10000 {
        120 // 1.2x - very low voltage
    } else if voltage_mv < 11000 {
        110 // 1.1x - low voltage
    } else if voltage_mv < 12000 {
        105 // 1.05x
    } else if voltage_mv < 14000 {
        100 // 1.0x - normal
    } else if voltage_mv < 15000 {
        98 // 0.98x
    } else {
        95 // 0.95x - high voltage
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_clt_correction_cold() {
        let corr = calculate_clt_correction(-30);
        assert_eq!(corr, 200); // 2.0x very cold
    }

    #[test]
    fn test_clt_correction_warm() {
        let corr = calculate_clt_correction(80);
        assert_eq!(corr, 100); // 1.0x fully warm
    }

    #[test]
    fn test_clt_correction_warming() {
        let corr = calculate_clt_correction(50);
        assert_eq!(corr, 110); // 1.1x at 50°C
    }

    #[test]
    fn test_iat_correction_cold() {
        let corr = calculate_iat_correction(-30);
        assert_eq!(corr, 85); // 0.85x cold dense air
    }

    #[test]
    fn test_iat_correction_normal() {
        let corr = calculate_iat_correction(25);
        assert_eq!(corr, 100); // 1.0x normal
    }

    #[test]
    fn test_iat_correction_hot() {
        let corr = calculate_iat_correction(70);
        assert_eq!(corr, 110); // 1.1x hot thin air
    }

    #[test]
    fn test_voltage_correction_low() {
        let corr = calculate_voltage_correction(9500);
        assert_eq!(corr, 120); // 1.2x low voltage
    }

    #[test]
    fn test_voltage_correction_normal() {
        let corr = calculate_voltage_correction(13000);
        assert_eq!(corr, 100); // 1.0x normal
    }

    #[test]
    fn test_voltage_correction_high() {
        let corr = calculate_voltage_correction(15500);
        assert_eq!(corr, 95); // 0.95x high voltage
    }

    #[test]
    fn test_apply_clt_correction() {
        let base_pw = 3000; // 3ms

        // Cold engine
        let cold_pw = apply_clt_correction(base_pw, -10);
        assert_eq!(cold_pw, 4800); // 1.6x

        // Warm engine
        let warm_pw = apply_clt_correction(base_pw, 80);
        assert_eq!(warm_pw, 3000); // 1.0x (no change)
    }

    #[test]
    fn test_apply_all_corrections() {
        let sensors = SensorData {
            timestamp_us: 1000,
            clt_celsius: -10,          // Cold: 1.6x
            iat_celsius: 25,           // Normal: 1.0x
            battery_voltage_mv: 13000, // Normal: 1.0x
        };

        let base_pw = 3000;
        let corrected = apply_all_corrections(base_pw, &sensors);

        // Only CLT correction should apply: 3000 × 1.6 = 4800
        assert_eq!(corrected, 4800);
    }

    #[test]
    fn test_combined_corrections_cold_start() {
        let sensors = SensorData {
            timestamp_us: 1000,
            clt_celsius: -20,          // Cold: 1.6x (not 2.0x, that's below -20)
            iat_celsius: -15,          // Cold air: 0.9x
            battery_voltage_mv: 10500, // Low voltage: 1.1x
        };

        let base_pw = 2000;
        let corrected = apply_all_corrections(base_pw, &sensors);

        // 2000 × 1.6 × 0.9 × 1.1 = 3168
        assert!(
            corrected > 3100 && corrected < 3250,
            "corrected = {corrected}"
        );
    }

    #[test]
    fn test_combined_corrections_normal() {
        let sensors = SensorData {
            timestamp_us: 1000,
            clt_celsius: 80,           // Warm: 1.0x
            iat_celsius: 25,           // Normal: 1.0x
            battery_voltage_mv: 13500, // Normal: 1.0x
        };

        let base_pw = 3000;
        let corrected = apply_all_corrections(base_pw, &sensors);

        // No corrections: should equal base
        assert_eq!(corrected, 3000);
    }

    #[test]
    fn test_combined_corrections_hot_engine() {
        let sensors = SensorData {
            timestamp_us: 1000,
            clt_celsius: 90,           // Hot: 1.0x (no CLT correction above 70°C)
            iat_celsius: 70,           // Hot air: 1.1x
            battery_voltage_mv: 14500, // High voltage: 0.98x
        };

        let base_pw = 3000;
        let corrected = apply_all_corrections(base_pw, &sensors);

        // 3000 × 1.0 × 1.1 × 0.98 = 3234
        assert!(
            corrected > 3200 && corrected < 3300,
            "corrected = {corrected}"
        );
    }

    #[test]
    fn test_corrections_never_zero() {
        let sensors = SensorData {
            timestamp_us: 1000,
            clt_celsius: -50, // Extreme cold
            iat_celsius: -40,
            battery_voltage_mv: 8000, // Very low voltage
        };

        let base_pw = 100;
        let corrected = apply_all_corrections(base_pw, &sensors);

        // Should never be zero
        assert!(corrected > 0);
    }
}
