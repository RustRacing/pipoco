use crate::Kpa10;

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;
const MAP_MIN_KPA10: u16 = 100;
const MAP_MAX_KPA10: u16 = 3000;

const fn clamp_u16(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

pub fn map_from_counts(adc_counts: u16) -> Kpa10 {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let span = (MAP_MAX_KPA10 - MAP_MIN_KPA10) as u32;
    let scaled = (counts as u32 * span) / ADC_MAX as u32;
    Kpa10((MAP_MIN_KPA10 as u32 + scaled) as u16)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_adc_endpoints() {
        assert_eq!(map_from_counts(0), Kpa10(100));
        assert_eq!(map_from_counts(4095), Kpa10(3000));
        assert_eq!(map_from_counts(u16::MAX), Kpa10(3000));
    }

    #[test]
    fn midpoint_uses_floor_linear_interpolation() {
        // floor((2048 * (3000 - 100)) / 4095) + 100 = 1550
        assert_eq!(map_from_counts(2048), Kpa10(1550));
    }

    #[test]
    fn is_monotonic_non_decreasing_over_full_adc_domain() {
        let mut prev = map_from_counts(0).0;
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = map_from_counts(adc).0;
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
