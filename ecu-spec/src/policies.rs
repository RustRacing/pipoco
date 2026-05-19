use crate::numeric::clamp_u32;
use crate::{AfrOverride, AfrX100, InputSnapshot, PulseWidthUs, RatioX1000, ValidatedCalibration};

pub fn trim_corr_x1000(state: &crate::LogicalState) -> RatioX1000 {
    state.math.trim_ratio_x1000
}

pub fn clamp_target_afr_override(input: InputSnapshot) -> AfrOverride {
    match input.target_afr_override_x100 {
        AfrOverride::Some(value) => AfrOverride::Some(AfrX100(clamp_u16_afr(value.0))),
        AfrOverride::None => AfrOverride::None,
    }
}

const fn clamp_u16_afr(value: u16) -> u16 {
    if value < 500 {
        500
    } else if value > 3000 {
        3000
    } else {
        value
    }
}

pub fn apply_pw_max(pw: u32, cal: &ValidatedCalibration) -> PulseWidthUs {
    PulseWidthUs(clamp_u32(pw, 0, cal.0.pw_max_us))
}
