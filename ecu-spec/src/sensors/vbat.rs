use crate::Millivolts;

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;
const VBAT_MIN_MV: u16 = 6000;
const VBAT_MAX_MV: u16 = 18000;

const fn clamp_u16(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

pub fn vbat_from_counts(adc_counts: u16) -> Millivolts {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let span = (VBAT_MAX_MV - VBAT_MIN_MV) as u32;
    let scaled = (counts as u32 * span) / ADC_MAX as u32;
    Millivolts((VBAT_MIN_MV as u32 + scaled) as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_adc_endpoints() {
        assert_eq!(vbat_from_counts(0), Millivolts(6000));
        assert_eq!(vbat_from_counts(4095), Millivolts(18000));
    }

    #[test]
    fn midpoint_uses_floor_linear_interpolation() {
        // floor((2048 * (18000 - 6000)) / 4095) + 6000 = 12001
        assert_eq!(vbat_from_counts(2048), Millivolts(12001));
    }

    #[test]
    fn is_monotonic_non_decreasing_over_full_adc_domain() {
        let mut prev = vbat_from_counts(0).0;
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = vbat_from_counts(adc).0;
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
