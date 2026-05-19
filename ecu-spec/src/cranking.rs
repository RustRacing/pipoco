use crate::interp::{find_segment, lerp_u16};
use crate::numeric::mul_ratio_x1000;
use crate::{Curve16, EngineMode, PulseWidthUs, RatioX1000, TempC10};

pub fn cranking_corr_x1000(
    cranking_curve: &Curve16,
    mode: EngineMode,
    clt_c10: TempC10,
) -> RatioX1000 {
    if mode != EngineMode::Cranking {
        return RatioX1000(1000);
    }
    RatioX1000(lookup_curve_u16(cranking_curve, temp_curve_input(clt_c10)))
}

pub fn apply_cranking_pw(pw_vbat_us: PulseWidthUs, corr_x1000: RatioX1000) -> PulseWidthUs {
    PulseWidthUs(mul_ratio_x1000(pw_vbat_us.0, corr_x1000))
}

pub fn spark_selected_for_mode(mode: EngineMode) -> bool {
    matches!(mode, EngineMode::Cranking | EngineMode::Running)
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

fn temp_curve_input(temp: TempC10) -> u16 {
    if temp.0 < 0 {
        0
    } else {
        temp.0 as u16
    }
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

    fn curve() -> Curve16 {
        let mut values = [0u16; 16];
        values[0] = 1800;
        values[1] = 1200;
        values[2] = 1000;
        Curve16 {
            axis: axis(&[0, 400, 800]),
            values,
        }
    }

    #[test]
    fn cranking_correction_only_applies_in_cranking_mode() {
        let c = curve();
        assert_eq!(
            cranking_corr_x1000(&c, EngineMode::Cranking, TempC10(400)),
            RatioX1000(1200)
        );
        assert_eq!(
            cranking_corr_x1000(&c, EngineMode::Running, TempC10(400)),
            RatioX1000(1000)
        );
    }

    #[test]
    fn cranking_pw_uses_floor_ratio_multiply() {
        let pw = apply_cranking_pw(PulseWidthUs(1501), RatioX1000(1333));
        assert_eq!(pw, PulseWidthUs(2000));
    }

    #[test]
    fn spark_is_selected_in_cranking_and_running_only() {
        assert!(spark_selected_for_mode(EngineMode::Cranking));
        assert!(spark_selected_for_mode(EngineMode::Running));
        assert!(!spark_selected_for_mode(EngineMode::Off));
        assert!(!spark_selected_for_mode(EngineMode::Shutdown));
    }
}
