use crate::{numeric::clamp_u16, Calibration};

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;
const TPS_MIN_X100: u16 = 0;
const TPS_MAX_X100: u16 = 10000;

pub fn tps_from_counts(calibration: &Calibration, adc_counts: u16) -> u16 {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let adc_min = clamp_u16(calibration.tps_adc_min_counts, ADC_MIN, ADC_MAX);
    let adc_max = clamp_u16(calibration.tps_adc_max_counts, ADC_MIN, ADC_MAX);

    if adc_max <= adc_min {
        return TPS_MIN_X100;
    }

    if counts <= adc_min {
        return TPS_MIN_X100;
    }
    if counts >= adc_max {
        return TPS_MAX_X100;
    }

    let span = (adc_max - adc_min) as u32;
    let scaled = ((counts - adc_min) as u32 * TPS_MAX_X100 as u32) / span;
    scaled as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn calibration() -> Calibration {
        Calibration {
            tps_adc_min_counts: 1000,
            tps_adc_max_counts: 3000,
            ..Calibration::default()
        }
    }

    #[test]
    fn clamps_at_calibrated_endpoints() {
        let cal = calibration();
        assert_eq!(tps_from_counts(&cal, 1000), 0);
        assert_eq!(tps_from_counts(&cal, 3000), 10000);
    }

    #[test]
    fn midpoint_uses_floor_linear_interpolation() {
        let cal = calibration();
        // floor(((2000 - 1000) * 10000) / (3000 - 1000)) = 5000
        assert_eq!(tps_from_counts(&cal, 2000), 5000);
    }

    #[test]
    fn is_monotonic_non_decreasing_over_full_adc_domain() {
        let cal = calibration();
        let mut prev = tps_from_counts(&cal, 0);
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = tps_from_counts(&cal, adc);
            assert!(
                current >= prev,
                "adc={} current={} prev={}",
                adc,
                current,
                prev
            );
            prev = current;
            adc += 1;
        }
    }
}
