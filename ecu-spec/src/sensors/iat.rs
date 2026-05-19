use crate::{interp::lerp_i16, Axis16, TempC10};

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;

const IAT_COUNTS_AXIS: Axis16 = Axis16 {
    len: 16,
    values: [
        0, 256, 512, 768, 1024, 1280, 1536, 1792, 2048, 2304, 2560, 2816, 3072, 3328, 3584, 4095,
    ],
};

const IAT_TEMP_C10: [i16; 16] = [
    1100, 950, 820, 700, 590, 490, 390, 300, 220, 140, 70, 0, -80, -170, -280, -400,
];

const fn clamp_u16(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

fn find_segment(axis: &Axis16, x: u16) -> usize {
    let len = axis.len as usize;
    if len < 2 {
        return 0;
    }

    let clipped = clamp_u16(x, axis.values[0], axis.values[len - 1]);
    let mut idx = 0usize;
    while idx + 1 < len {
        let lo = axis.values[idx];
        let hi = axis.values[idx + 1];
        if clipped >= lo && (clipped < hi || (idx + 1 == len - 1 && clipped == hi)) {
            return idx;
        }
        idx += 1;
    }

    len - 2
}

pub fn iat_from_counts(adc_counts: u16) -> TempC10 {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let seg = find_segment(&IAT_COUNTS_AXIS, counts);

    let x0 = IAT_COUNTS_AXIS.values[seg];
    let x1 = IAT_COUNTS_AXIS.values[seg + 1];
    let y0 = IAT_TEMP_C10[seg];
    let y1 = IAT_TEMP_C10[seg + 1];

    TempC10(lerp_i16(x0, x1, y0, y1, counts))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_adc_endpoints() {
        assert_eq!(iat_from_counts(0), TempC10(1100));
        assert_eq!(iat_from_counts(4095), TempC10(-400));
    }

    #[test]
    fn interpolates_midpoint_between_knots() {
        // Midpoint between 2048->2304 counts where 220->140 c10 yields 180 c10.
        assert_eq!(iat_from_counts(2176), TempC10(180));
    }

    #[test]
    fn is_monotonic_non_increasing_over_full_adc_domain() {
        let mut prev = iat_from_counts(0).0;
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = iat_from_counts(adc).0;
            assert!(
                current <= prev,
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
