use crate::interp::{find_segment, lerp_u16};
use crate::numeric::clamp_u16;
use crate::{Curve16, PulseWidthUs, RatioX1000, TempC10};

pub fn warmup_correction(warmup_curve: &Curve16, clt_c10: TempC10) -> RatioX1000 {
    RatioX1000::new(lookup_curve_u16(warmup_curve, temp_curve_input(clt_c10)))
}

pub fn apply_warmup_pw(pw_afterstart_us: PulseWidthUs, corr_x1000: RatioX1000) -> PulseWidthUs {
    PulseWidthUs::new(crate::mul_ratio_x1000(pw_afterstart_us.get(), corr_x1000))
}

fn lookup_curve_u16(curve: &Curve16, x: u16) -> u16 {
    let len = curve.axis.len as usize;
    let clipped = clamp_u16(x, curve.axis.values[0], curve.axis.values[len - 1]);
    let seg = find_segment(&curve.axis, clipped);
    lerp_u16(
        curve.axis.values[seg],
        curve.axis.values[seg + 1],
        curve.values[seg],
        curve.values[seg + 1],
        clipped,
    )
}

fn temp_curve_input(temp: TempC10) -> u16 {
    if temp.get() < 0 {
        0
    } else {
        temp.get() as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Axis16;

    fn warmup_curve() -> Curve16 {
        let mut curve = Curve16 {
            axis: Axis16 {
                len: 3,
                values: [0, 600, 900, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            },
            ..Curve16::default()
        };
        curve.values[0] = 1300;
        curve.values[1] = 1100;
        curve.values[2] = 1000;
        curve
    }

    #[test]
    fn warmup_cold_edge_uses_cold_correction() {
        let corr = warmup_correction(&warmup_curve(), TempC10::new(-400));
        assert_eq!(corr, RatioX1000::new(1300));
    }

    #[test]
    fn warmup_midpoint_interpolates_with_floor() {
        let corr = warmup_correction(&warmup_curve(), TempC10::new(300));
        assert_eq!(corr, RatioX1000::new(1200));
    }

    #[test]
    fn warmup_warm_edge_uses_warm_identity() {
        let corr = warmup_correction(&warmup_curve(), TempC10::new(1000));
        assert_eq!(corr, RatioX1000::new(1000));
    }
}
