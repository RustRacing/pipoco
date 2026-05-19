use crate::interp::{find_segment, lerp_u16};
use crate::{Curve16, Kpa10, RatioX1000};

pub fn baro_correction(baro_corr_curve: &Curve16, baro_kpa10: Kpa10) -> RatioX1000 {
    RatioX1000(lookup_curve_u16(baro_corr_curve, baro_kpa10.0))
}

fn lookup_curve_u16(curve: &Curve16, x: u16) -> u16 {
    let len = curve.axis.len as usize;
    let clipped = x.clamp(curve.axis.values[0], curve.axis.values[len - 1]);
    let seg = find_segment(&curve.axis, clipped);
    lerp_u16(
        curve.axis.values[seg],
        curve.axis.values[seg + 1],
        curve.values[seg],
        curve.values[seg + 1],
        clipped,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Axis16;

    fn axis(values: &[u16]) -> Axis16 {
        let mut axis = Axis16 {
            len: values.len() as u8,
            ..Axis16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            axis.values[idx] = values[idx];
            idx += 1;
        }
        axis
    }

    fn baro_curve() -> Curve16 {
        let mut values = [0u16; 16];
        values[0] = 700;
        values[1] = 850;
        values[2] = 1000;
        Curve16 {
            axis: axis(&[700, 850, 1000]),
            values,
        }
    }

    #[test]
    fn baro_high_altitude_matches_curve_point() {
        let corr = baro_correction(&baro_curve(), Kpa10(700));
        assert_eq!(corr, RatioX1000(700));
    }

    #[test]
    fn baro_sea_level_matches_curve_point() {
        let corr = baro_correction(&baro_curve(), Kpa10(1000));
        assert_eq!(corr, RatioX1000(1000));
    }
}
