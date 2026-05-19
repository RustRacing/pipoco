use crate::interp::{bilerp_u16, find_segment, lerp_u16};
use crate::numeric::{clamp_u16, mul_div_floor_u32, mul_ratio_x1000};
use crate::policies::{apply_pw_max, clamp_target_afr_override, trim_corr_x1000};
use crate::{
    afterstart_corr_x1000, baro_correction, cranking_corr_x1000, deadtime_lookup, vbat_correction,
    warmup_correction, AfrOverride, AfrX100, Kpa10, LogicalState, PulseWidthUs, RatioX1000,
    ValidatedCalibration, VePctX100,
};
use core::num::NonZeroU32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FuelParts {
    pub pw_air_us: PulseWidthUs,
    pub ae_pulse_us: PulseWidthUs,
    pub deadtime_us: PulseWidthUs,
    pub clt_corr_x1000: RatioX1000,
    pub iat_corr_x1000: RatioX1000,
    pub baro_corr_x1000: RatioX1000,
    pub vbat_corr_x1000: RatioX1000,
    pub cranking_corr_x1000: RatioX1000,
    pub afterstart_corr_x1000: RatioX1000,
    pub warmup_corr_x1000: RatioX1000,
    pub afr_corr_x1000: RatioX1000,
    pub lambda_corr_x1000: RatioX1000,
}

pub fn lookup_ve(cal: &ValidatedCalibration, input: crate::InputSnapshot) -> VePctX100 {
    VePctX100(bilerp_u16(&cal.0.ve_table, input.rpm, input.load_kpa10))
}

pub fn lookup_target_afr(cal: &ValidatedCalibration, input: crate::InputSnapshot) -> AfrX100 {
    match clamp_target_afr_override(input) {
        AfrOverride::Some(value) => value,
        AfrOverride::None => AfrX100(bilerp_u16(
            &cal.0.afr_target_table,
            input.rpm,
            input.load_kpa10,
        )),
    }
}

pub fn lookup_deadtime_us(cal: &ValidatedCalibration, input: crate::InputSnapshot) -> PulseWidthUs {
    deadtime_lookup(&cal.0.deadtime_table_us, input.vbatt_mv, input.baro_kpa10)
}

pub fn lookup_clt_corr_x1000(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    RatioX1000(lookup_curve_u16(
        &cal.0.clt_corr_curve,
        temp_curve_input(input.clt_c10),
    ))
}

pub fn lookup_iat_corr_x1000(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    RatioX1000(lookup_curve_u16(
        &cal.0.iat_corr_curve,
        temp_curve_input(input.iat_c10),
    ))
}

pub fn lookup_baro_corr_x1000(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    baro_correction(&cal.0.baro_corr_curve, input.baro_kpa10)
}

pub fn lookup_vbat_corr_x1000(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    vbat_correction(&cal.0.vbat_corr_curve, input.vbatt_mv)
}

pub fn lookup_cranking_corr_x1000(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    cranking_corr_x1000(&cal.0.cranking_curve, input.mode, input.clt_c10)
}

pub fn lookup_afterstart_corr_x1000(
    cal: &ValidatedCalibration,
    state: &LogicalState,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    let cycles_since_start =
        core::cmp::min(state.scheduler.last_cycle_epoch, u16::MAX as u32) as u16;
    afterstart_corr_x1000(
        &cal.0.afterstart_table,
        cal.0.afterstart_window_cycles,
        input.mode,
        cycles_since_start,
        input.clt_c10,
    )
}

pub fn lookup_warmup_corr_x1000(
    cal: &ValidatedCalibration,
    input: crate::InputSnapshot,
) -> RatioX1000 {
    warmup_correction(&cal.0.warmup_curve, input.clt_c10)
}

pub fn compute_pw_base_us(cal: &ValidatedCalibration, ve: VePctX100) -> PulseWidthUs {
    PulseWidthUs(mul_div_floor_u32(
        cal.0.required_fuel_us,
        ve.0 as u32,
        NonZeroU32::new(10_000).unwrap_or(NonZeroU32::MIN),
    ))
}

pub fn compute_pw_air_us(
    cal: &ValidatedCalibration,
    base: PulseWidthUs,
    map: Kpa10,
) -> PulseWidthUs {
    let pref_kpa10 = NonZeroU32::new(cal.0.pref_kpa10 as u32).unwrap_or(NonZeroU32::MIN);
    PulseWidthUs(mul_div_floor_u32(base.0, map.0 as u32, pref_kpa10))
}

pub fn compute_afr_corr_x1000(cal: &ValidatedCalibration, target: AfrX100) -> RatioX1000 {
    if target.0 == 0 {
        return RatioX1000(0);
    }
    let target = NonZeroU32::new(target.0 as u32).unwrap_or(NonZeroU32::MIN);
    RatioX1000(mul_div_floor_u32(cal.0.stoich_afr_x100 as u32, 1000, target) as u16)
}

pub fn compute_pw_corr_us(
    cal: &ValidatedCalibration,
    state: &LogicalState,
    input: crate::InputSnapshot,
    parts: FuelParts,
) -> PulseWidthUs {
    let trim_corr_x1000 = trim_corr_x1000(state);
    let mut pw = parts.pw_air_us.0;
    pw = mul_ratio_x1000(pw, parts.cranking_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.afterstart_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.warmup_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.clt_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.iat_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.baro_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.vbat_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.afr_corr_x1000);
    pw = mul_ratio_x1000(pw, parts.lambda_corr_x1000);
    pw = mul_ratio_x1000(pw, trim_corr_x1000);
    pw = pw.saturating_add(parts.ae_pulse_us.0);
    pw = pw.saturating_add(parts.deadtime_us.0);
    if input.fuel_cut {
        return PulseWidthUs(0);
    }
    apply_pw_max(pw, cal)
}

fn lookup_curve_u16(curve: &crate::Curve16, x: u16) -> u16 {
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

fn temp_curve_input(temp: crate::TempC10) -> u16 {
    if temp.0 < 0 {
        0
    } else {
        temp.0 as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{EngineMode, Millivolts};
    use crate::{
        ae_step, AfrOverride, Axis16, Calibration, Curve16, CylinderArrayU16, FuelModel,
        InputSnapshot, LogicalState, PwMaxPolicy, Rpm, SyncState, Table2D16, TempC10, TrimPolicy,
    };

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

    fn curve(values: &[u16]) -> Curve16 {
        let mut curve = Curve16 {
            axis: axis(&[100, 200, 300]),
            ..Curve16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            curve.values[idx] = values[idx];
            idx += 1;
        }
        curve
    }

    fn table(values: [[u16; 16]; 16]) -> Table2D16<u16> {
        Table2D16 {
            rpm_axis: axis(&[100, 200]),
            load_axis: axis(&[10, 20]),
            values,
        }
    }

    fn calibration() -> ValidatedCalibration {
        ValidatedCalibration(Calibration {
            fuel_model: FuelModel::SpeedDensityRequiredFuel,
            ve_table: table({
                let mut values = [[0u16; 16]; 16];
                values[0][0] = 1000;
                values[0][1] = 2000;
                values[1][0] = 3000;
                values[1][1] = 4000;
                values
            }),
            afr_target_table: table({
                let mut values = [[0u16; 16]; 16];
                values[0][0] = 1470;
                values[0][1] = 1470;
                values[1][0] = 1470;
                values[1][1] = 1470;
                values
            }),
            deadtime_table_us: table({
                let mut values = [[0u16; 16]; 16];
                values[0][0] = 100;
                values[0][1] = 100;
                values[1][0] = 100;
                values[1][1] = 100;
                values
            }),
            clt_corr_curve: curve(&[1000, 1000, 1000]),
            iat_corr_curve: curve(&[1000, 1000, 1000]),
            baro_corr_curve: curve(&[1000, 1000, 1000]),
            vbat_corr_curve: curve(&[1000, 1000, 1000]),
            cranking_curve: curve(&[2000, 1500, 1000]),
            afterstart_table: table({
                let mut values = [[0u16; 16]; 16];
                values[0][0] = 1000;
                values[0][1] = 1000;
                values[1][0] = 1000;
                values[1][1] = 1000;
                values
            }),
            afterstart_window_cycles: 0,
            warmup_curve: curve(&[1000, 1000, 1000]),
            ae_tps_threshold_curve: curve(&[20000, 20000, 20000]),
            ae_map_threshold_curve: curve(&[20000, 20000, 20000]),
            ae_shot_curve_us: curve(&[0, 0, 0]),
            ae_decay_steps_curve: curve(&[0, 0, 0]),
            ae_decay_ratio_curve_x1000: curve(&[1000, 1000, 1000]),
            required_fuel_us: 10_000,
            pref_kpa10: 100,
            stoich_afr_x100: 1470,
            trim_policy: TrimPolicy::Identity,
            pw_max_policy: PwMaxPolicy::Fixed,
            pw_max_us: 25_000,
            cylinder_phase_deg10: CylinderArrayU16 {
                count: 4,
                values: [0; 16],
            },
            ..Calibration::default()
        })
    }

    fn make_input() -> InputSnapshot {
        InputSnapshot {
            rpm: Rpm(150),
            load_kpa10: crate::Kpa10(15),
            tps_x100: 0,
            map_kpa10: crate::Kpa10(120),
            clt_c10: TempC10(150),
            iat_c10: TempC10(150),
            baro_kpa10: crate::Kpa10(150),
            vbatt_mv: Millivolts(12_600),
            sync: SyncState::Synced,
            mode: EngineMode::Running,
            ..InputSnapshot::default()
        }
    }

    #[test]
    fn clamps_target_override() {
        let cal = calibration();
        let mut input = make_input();
        input.target_afr_override_x100 = AfrOverride::Some(crate::AfrX100(400));
        assert_eq!(lookup_target_afr(&cal, input).0, 500);
        input.target_afr_override_x100 = AfrOverride::Some(crate::AfrX100(3200));
        assert_eq!(lookup_target_afr(&cal, input).0, 3000);
    }

    #[test]
    fn computes_fuel_pipeline_with_floor_rounding() {
        let cal = calibration();
        let input = make_input();
        let ve = lookup_ve(&cal, input);
        let target = lookup_target_afr(&cal, input);
        let base = compute_pw_base_us(&cal, ve);
        let air = compute_pw_air_us(&cal, base, input.map_kpa10);
        let parts = FuelParts {
            pw_air_us: air,
            ae_pulse_us: ae_step(&cal, input, &LogicalState::default()).ae_pulse_us,
            deadtime_us: lookup_deadtime_us(&cal, input),
            clt_corr_x1000: lookup_clt_corr_x1000(&cal, input),
            iat_corr_x1000: lookup_iat_corr_x1000(&cal, input),
            baro_corr_x1000: lookup_baro_corr_x1000(&cal, input),
            vbat_corr_x1000: lookup_vbat_corr_x1000(&cal, input),
            cranking_corr_x1000: lookup_cranking_corr_x1000(&cal, input),
            afterstart_corr_x1000: lookup_afterstart_corr_x1000(
                &cal,
                &LogicalState::default(),
                input,
            ),
            warmup_corr_x1000: lookup_warmup_corr_x1000(&cal, input),
            afr_corr_x1000: compute_afr_corr_x1000(&cal, target),
            lambda_corr_x1000: RatioX1000(1000),
        };
        let pw = compute_pw_corr_us(&cal, &LogicalState::default(), input, parts);
        assert_eq!(ve.0, 2500);
        assert_eq!(base.0, 2500);
        assert_eq!(air.0, 3000);
        assert_eq!(pw.0, 3100);
    }

    #[test]
    fn respects_fuel_cut_and_pw_max() {
        let mut cal = calibration();
        cal.0.pw_max_us = 1_000;
        let mut input = InputSnapshot {
            fuel_cut: true,
            ..make_input()
        };
        let parts = FuelParts {
            pw_air_us: PulseWidthUs(9999),
            ae_pulse_us: ae_step(&cal, input, &LogicalState::default()).ae_pulse_us,
            deadtime_us: PulseWidthUs(9999),
            clt_corr_x1000: RatioX1000(3000),
            iat_corr_x1000: RatioX1000(3000),
            baro_corr_x1000: RatioX1000(3000),
            vbat_corr_x1000: RatioX1000(3000),
            cranking_corr_x1000: RatioX1000(3000),
            afterstart_corr_x1000: RatioX1000(3000),
            warmup_corr_x1000: RatioX1000(3000),
            afr_corr_x1000: RatioX1000(3000),
            lambda_corr_x1000: RatioX1000(3000),
        };
        assert_eq!(
            compute_pw_corr_us(&cal, &LogicalState::default(), input, parts).0,
            0
        );

        input = InputSnapshot {
            fuel_cut: false,
            ..make_input()
        };
        assert_eq!(
            compute_pw_corr_us(&cal, &LogicalState::default(), input, parts).0,
            1000
        );
    }

    #[test]
    fn negative_temperatures_clip_to_low_curve_endpoint() {
        let cal = calibration();
        let mut input = make_input();
        input.clt_c10 = TempC10(-100);
        input.iat_c10 = TempC10(-100);

        assert_eq!(lookup_clt_corr_x1000(&cal, input), RatioX1000(1000));
        assert_eq!(lookup_iat_corr_x1000(&cal, input), RatioX1000(1000));
    }

    #[test]
    fn cranking_correction_applies_only_while_cranking() {
        let cal = calibration();
        let mut input = make_input();
        input.mode = EngineMode::Cranking;
        input.clt_c10 = TempC10(100);
        assert_eq!(lookup_cranking_corr_x1000(&cal, input), RatioX1000(2000));
        input.mode = EngineMode::Running;
        assert_eq!(lookup_cranking_corr_x1000(&cal, input), RatioX1000(1000));
    }
}
