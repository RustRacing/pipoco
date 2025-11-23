/// Piecewise-linear interpolation for thermistor tables
///
/// Interprets `table` as temperatures in °C at evenly spaced ADC codes across 0..=4095.
/// For N=8, bin width = 4096 / 7 ≈ 585; we use 4096/8=512 for simple index and interpolate across 8 bins.
pub fn interp_temp_c<const N: usize>(adc_counts: u16, table: &[i16; N]) -> i16 {
    if N < 2 {
        return 0;
    }
    let step = (4096 / N) as u16; // e.g., 512 when N=8
    let idx = core::cmp::min((adc_counts / step) as usize, N - 1);
    if idx == N - 1 {
        return table[N - 1];
    }
    let base_code = (idx as u16) * step;
    let frac = (adc_counts.saturating_sub(base_code)) as u32;
    let span = step as u32;
    let t0 = table[idx] as i32;
    let t1 = table[idx + 1] as i32;
    let dt = t1 - t0;
    let interp = t0 + ((dt as i64 * frac as i64) / span as i64) as i32;
    interp as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_interp_temp_monotonic() {
        let tbl = [-40, -20, 0, 20, 40, 60, 80, 100];
        assert_eq!(interp_temp_c(0, &tbl), -40);
        let mid = interp_temp_c(2048, &tbl);
        assert!((0..=40).contains(&mid));
        assert_eq!(interp_temp_c(4095, &tbl), 100);
    }
}
