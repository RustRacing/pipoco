use crate::interp::bilerp_u16;
use crate::numeric::mul_ratio_x1000;
use crate::{EngineMode, PulseWidthUs, RatioX1000, Table2D16, TempC10};

pub fn afterstart_corr_x1000(
    afterstart_table: &Table2D16<u16>,
    afterstart_window_cycles: u16,
    mode: EngineMode,
    cycles_since_start: u16,
    clt_c10: TempC10,
) -> RatioX1000 {
    if mode != EngineMode::Running || cycles_since_start > afterstart_window_cycles {
        return RatioX1000::new(1000);
    }
    let clt_input = if clt_c10.get() < 0 {
        0
    } else {
        clt_c10.get() as u16
    };
    RatioX1000::new(bilerp_u16(
        afterstart_table,
        crate::Rpm::new(cycles_since_start),
        crate::Kpa10::new(clt_input),
    ))
}

pub fn apply_afterstart_pw(pw_cranking_us: PulseWidthUs, corr_x1000: RatioX1000) -> PulseWidthUs {
    PulseWidthUs::new(mul_ratio_x1000(pw_cranking_us.get(), corr_x1000))
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

    fn table() -> Table2D16<u16> {
        let mut table = Table2D16 {
            rpm_axis: axis(&[0, 10]),
            load_axis: axis(&[0, 1000]),
            ..Table2D16::default()
        };
        table.values[0][0] = 1300;
        table.values[0][1] = 1000;
        table.values[1][0] = 1200;
        table.values[1][1] = 1000;
        table
    }

    #[test]
    fn afterstart_applies_only_in_running_within_window() {
        let t = table();
        assert_eq!(
            afterstart_corr_x1000(&t, 50, EngineMode::Running, 5, TempC10::new(0)),
            RatioX1000::new(1150)
        );
        assert_eq!(
            afterstart_corr_x1000(&t, 50, EngineMode::Cranking, 5, TempC10::new(0)),
            RatioX1000::new(1000)
        );
        assert_eq!(
            afterstart_corr_x1000(&t, 4, EngineMode::Running, 5, TempC10::new(0)),
            RatioX1000::new(1000)
        );
    }

    #[test]
    fn afterstart_pw_uses_floor_ratio_multiply() {
        let pw = apply_afterstart_pw(PulseWidthUs::new(1501), RatioX1000::new(1333));
        assert_eq!(pw, PulseWidthUs::new(2000));
    }
}
