use crate::{
    interp::{find_segment, lerp_i16},
    numeric::clamp_u16,
    Axis16, TempC10,
};

const ADC_MIN: u16 = 0;
const ADC_MAX: u16 = 4095;

const CLT_COUNTS_AXIS: Axis16 = Axis16 {
    len: 16,
    values: [
        0, 256, 512, 768, 1024, 1280, 1536, 1792, 2048, 2304, 2560, 2816, 3072, 3328, 3584, 4095,
    ],
};

const CLT_TEMP_C10: [i16; 16] = [
    1200, 1020, 860, 730, 610, 500, 390, 290, 200, 120, 40, -40, -120, -200, -300, -400,
];

pub fn clt_from_counts(adc_counts: u16) -> TempC10 {
    let counts = clamp_u16(adc_counts, ADC_MIN, ADC_MAX);
    let seg = find_segment(&CLT_COUNTS_AXIS, counts);

    let x0 = CLT_COUNTS_AXIS.values[seg];
    let x1 = CLT_COUNTS_AXIS.values[seg + 1];
    let y0 = CLT_TEMP_C10[seg];
    let y1 = CLT_TEMP_C10[seg + 1];

    TempC10::new(lerp_i16(x0, x1, y0, y1, counts))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamps_at_adc_endpoints() {
        assert_eq!(clt_from_counts(0), TempC10::new(1200));
        assert_eq!(clt_from_counts(4095), TempC10::new(-400));
    }

    #[test]
    fn interpolates_midpoint_between_knots() {
        // Midpoint between 2048->2304 counts where 200->120 c10 yields 160 c10.
        assert_eq!(clt_from_counts(2176), TempC10::new(160));
    }

    #[test]
    fn is_monotonic_non_increasing_over_full_adc_domain() {
        let mut prev = clt_from_counts(0).get();
        let mut adc = 1u16;
        while adc <= 4095 {
            let current = clt_from_counts(adc).get();
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
