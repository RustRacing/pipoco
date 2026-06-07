const KNOCK_WINDOW_GAIN_X100: u16 = 100;
const KNOCK_INTENSITY_MAX_X100: u16 = 10_000;

const fn clamp_u16(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

pub fn knock_from_window(window_energy: u16) -> u16 {
    let scaled = (window_energy as u32 * KNOCK_WINDOW_GAIN_X100 as u32) / 100;
    clamp_u16(
        scaled.min(u16::MAX as u32) as u16,
        0,
        KNOCK_INTENSITY_MAX_X100,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn knock_intensity_clamps_to_plausibility_domain() {
        assert_eq!(knock_from_window(0), 0);
        assert_eq!(knock_from_window(u16::MAX), KNOCK_INTENSITY_MAX_X100);
    }

    #[test]
    fn knock_intensity_midpoint_uses_floor_integer_scaling() {
        assert_eq!(knock_from_window(5000), 5000);
    }

    #[test]
    fn knock_intensity_is_monotonic_non_decreasing() {
        let mut prev = knock_from_window(0);
        let mut sample = 1u16;
        loop {
            let current = knock_from_window(sample);
            assert!(
                current >= prev,
                "sample={} current={} prev={}",
                sample,
                current,
                prev
            );
            prev = current;
            if sample == u16::MAX {
                break;
            }
            sample = sample.saturating_add(257);
        }
    }
}
