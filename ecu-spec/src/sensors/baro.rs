use crate::Kpa10;

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;
const BARO_MIN_KPA10: u16 = 500;
const BARO_MAX_KPA10: u16 = 1200;

const fn clamp_u16(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

pub fn baro_from_counts(adc_counts: u16) -> Kpa10 {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let span = (BARO_MAX_KPA10 - BARO_MIN_KPA10) as u32;
    let scaled = (counts as u32 * span) / ADC_MAX as u32;
    Kpa10((BARO_MIN_KPA10 as u32 + scaled) as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_adc_endpoints() {
        assert_eq!(baro_from_counts(0), Kpa10(500));
        assert_eq!(baro_from_counts(4095), Kpa10(1200));
    }

    #[test]
    fn midpoint_uses_floor_linear_interpolation() {
        // floor((2048 * (1200 - 500)) / 4095) + 500 = 850
        assert_eq!(baro_from_counts(2048), Kpa10(850));
    }

    #[test]
    fn is_monotonic_non_decreasing_over_full_adc_domain() {
        let mut prev = baro_from_counts(0).0;
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = baro_from_counts(adc).0;
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
