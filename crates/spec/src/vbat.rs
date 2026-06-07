use crate::interp::{find_segment, lerp_u16};
use crate::{Curve16, Millivolts, RatioX1000};

pub fn vbat_correction(vbat_corr_curve: &Curve16, vbat_mv: Millivolts) -> RatioX1000 {
    RatioX1000(lookup_curve_u16(vbat_corr_curve, vbat_mv.0))
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

    fn vbat_curve() -> Curve16 {
        let mut values = [0u16; 16];
        values[0] = 1200;
        values[1] = 1000;
        values[2] = 900;
        Curve16 {
            axis: axis(&[8000, 12000, 16000]),
            values,
        }
    }

    #[test]
    fn vbat_low_edge_clamps_to_min_axis() {
        let corr = vbat_correction(&vbat_curve(), Millivolts(7000));
        assert_eq!(corr, RatioX1000(1200));
    }

    #[test]
    fn vbat_nominal_point_matches_curve_cell() {
        let corr = vbat_correction(&vbat_curve(), Millivolts(12_000));
        assert_eq!(corr, RatioX1000(1000));
    }

    #[test]
    fn vbat_high_edge_clamps_to_max_axis() {
        let corr = vbat_correction(&vbat_curve(), Millivolts(17_000));
        assert_eq!(corr, RatioX1000(900));
    }
}
