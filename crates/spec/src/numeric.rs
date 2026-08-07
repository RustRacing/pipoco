use crate::RatioX1000;
use core::num::{NonZeroI32, NonZeroU32};

pub use ecu_domain::{cyc7200_distance, duration_us_to_deg10, norm7200};

pub const fn clamp_u16(value: u16, min: u16, max: u16) -> u16 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub const fn clamp_u32(value: u32, min: u32, max: u32) -> u32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub const fn clamp_i32(value: i32, min: i32, max: i32) -> i32 {
    if value < min {
        min
    } else if value > max {
        max
    } else {
        value
    }
}

pub const fn mul_div_floor_u32(value: u32, mul: u32, div: NonZeroU32) -> u32 {
    let product = (value as u64) * (mul as u64);
    let quotient = product / div.get() as u64;
    if quotient > u32::MAX as u64 {
        u32::MAX
    } else {
        quotient as u32
    }
}

pub const fn mul_div_floor_i32(value: i32, mul: i32, div: NonZeroI32) -> i32 {
    let numerator = (value as i64) * (mul as i64);
    let denominator = div.get() as i64;
    let quotient = numerator.div_euclid(denominator);
    if quotient < i32::MIN as i64 || quotient > i32::MAX as i64 {
        if quotient < i32::MIN as i64 {
            i32::MIN
        } else {
            i32::MAX
        }
    } else {
        quotient as i32
    }
}

pub const fn mul_ratio_x1000(value: u32, ratio: RatioX1000) -> u32 {
    let divisor = match NonZeroU32::new(1000) {
        Some(value) => value,
        None => NonZeroU32::MIN,
    };
    mul_div_floor_u32(value, ratio.get() as u32, divisor)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Degrees10, PulseWidthUs, Rpm};

    #[test]
    fn clamps_values() {
        assert_eq!(clamp_u16(3, 5, 10), 5);
        assert_eq!(clamp_u16(7, 5, 10), 7);
        assert_eq!(clamp_u16(12, 5, 10), 10);
        assert_eq!(clamp_u32(1, 2, 9), 2);
        assert_eq!(clamp_i32(-7, -5, 10), -5);
    }

    #[test]
    fn floors_integer_products() {
        assert_eq!(
            mul_div_floor_u32(7, 10, NonZeroU32::new(3).unwrap_or(NonZeroU32::MIN)),
            23
        );
        assert_eq!(
            mul_div_floor_u32(1_000_000, 3, NonZeroU32::new(2).unwrap_or(NonZeroU32::MIN)),
            1_500_000
        );
        assert_eq!(mul_ratio_x1000(2_500, RatioX1000::new(1_075)), 2_687);
        assert_eq!(
            mul_div_floor_i32(7, 10, NonZeroI32::new(3).unwrap_or(NonZeroI32::MIN)),
            23
        );
        assert_eq!(
            mul_div_floor_i32(-7, 10, NonZeroI32::new(3).unwrap_or(NonZeroI32::MIN)),
            -24
        );
        assert_eq!(
            mul_div_floor_i32(7, -10, NonZeroI32::new(3).unwrap_or(NonZeroI32::MIN)),
            -24
        );
        assert_eq!(
            mul_div_floor_i32(-7, -10, NonZeroI32::new(3).unwrap_or(NonZeroI32::MIN)),
            23
        );
    }

    #[test]
    fn saturates_when_widened_quotient_exceeds_target_type() {
        assert_eq!(
            mul_div_floor_u32(u32::MAX, u32::MAX, NonZeroU32::MIN),
            u32::MAX
        );
        assert_eq!(
            mul_div_floor_i32(
                i32::MAX,
                i32::MAX,
                NonZeroI32::new(1).unwrap_or(NonZeroI32::MIN)
            ),
            i32::MAX
        );
        assert_eq!(
            mul_div_floor_i32(
                i32::MIN,
                i32::MAX,
                NonZeroI32::new(1).unwrap_or(NonZeroI32::MIN)
            ),
            i32::MIN
        );
    }

    #[test]
    fn normalizes_cycle_angles() {
        assert_eq!(norm7200(0), Degrees10::new(0));
        assert_eq!(norm7200(7199), Degrees10::new(7199));
        assert_eq!(norm7200(7200), Degrees10::new(0));
        assert_eq!(norm7200(-1), Degrees10::new(7199));
        assert_eq!(norm7200(-7201), Degrees10::new(7199));
    }

    #[test]
    fn computes_cycle_distance() {
        assert_eq!(cyc7200_distance(Degrees10::new(10), Degrees10::new(20)), 10);
        assert_eq!(cyc7200_distance(Degrees10::new(20), Degrees10::new(10)), 10);
        assert_eq!(cyc7200_distance(Degrees10::new(0), Degrees10::new(7199)), 1);
        assert_eq!(cyc7200_distance(Degrees10::new(7199), Degrees10::new(0)), 1);
    }

    #[test]
    fn converts_duration_to_deg10() {
        assert_eq!(
            duration_us_to_deg10(PulseWidthUs::new(100_000), Rpm::new(1_000)).get(),
            6_000
        );
        assert_eq!(
            duration_us_to_deg10(PulseWidthUs::new(50_000), Rpm::new(2_000)).get(),
            6_000
        );
        assert_eq!(
            duration_us_to_deg10(PulseWidthUs::new(1), Rpm::new(1)).get(),
            0
        );
    }
}
