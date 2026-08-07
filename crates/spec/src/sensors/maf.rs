use crate::{
    interp::{find_segment, lerp_u16},
    numeric::clamp_u16,
    Axis16,
};

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;

const MAF_COUNTS_AXIS: Axis16 = Axis16 {
    len: 16,
    values: [
        0, 256, 512, 768, 1024, 1280, 1536, 1792, 2048, 2304, 2560, 2816, 3072, 3328, 3584, 4095,
    ],
};

const MAF_FLOW_X100: [u16; 16] = [
    0, 120, 280, 500, 780, 1120, 1520, 1980, 2520, 3150, 3880, 4720, 5680, 6760, 7960, 9300,
];

pub fn maf_from_counts(adc_counts: u16) -> u16 {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let seg = find_segment(&MAF_COUNTS_AXIS, counts);

    let x0 = MAF_COUNTS_AXIS.values[seg];
    let x1 = MAF_COUNTS_AXIS.values[seg + 1];
    let y0 = MAF_FLOW_X100[seg];
    let y1 = MAF_FLOW_X100[seg + 1];

    lerp_u16(x0, x1, y0, y1, counts)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_adc_endpoints() {
        assert_eq!(maf_from_counts(0), 0);
        assert_eq!(maf_from_counts(4095), 9300);
    }

    #[test]
    fn interpolates_midpoint_between_knots() {
        // Midpoint between 2048->2304 counts where 2520->3150 x100 yields 2835 x100.
        assert_eq!(maf_from_counts(2176), 2835);
    }

    #[test]
    fn is_monotonic_non_decreasing_over_full_adc_domain() {
        let mut prev = maf_from_counts(0);
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = maf_from_counts(adc);
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
