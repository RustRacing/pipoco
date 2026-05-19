use crate::{DiagnosticCode, Micros, TempC10};

const MIN_SLEW_DT_US: u32 = 1_000;
const US_PER_SEC: u64 = 1_000_000;

const CLT_MAX_RATE_PER_S: u32 = 200;
const IAT_MAX_RATE_PER_S: u32 = 300;
const MAP_MAX_RATE_PER_S: u32 = 2_000;
const TPS_MAX_RATE_PER_S: u32 = 50_000;
const MAF_MAX_RATE_PER_S: u32 = 100_000;
const O2_MAX_RATE_PER_S: u32 = 2_000;
const KNOCK_MAX_RATE_PER_S: u32 = 50_000;
const BARO_MAX_RATE_PER_S: u32 = 50;
const VBAT_MAX_RATE_PER_S: u32 = 5_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SensorSlewInput {
    pub t_us: Micros,
    pub clt_c10: TempC10,
    pub iat_c10: TempC10,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub maf_x100: u16,
    pub o2_afr_x100: u16,
    pub knock_intensity_x100: u16,
    pub baro_kpa10: u16,
    pub vbat_mv: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SensorSlewState {
    pub initialized: bool,
    pub last_t_us: Micros,
    pub clt_c10: TempC10,
    pub iat_c10: TempC10,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub maf_x100: u16,
    pub o2_afr_x100: u16,
    pub knock_intensity_x100: u16,
    pub baro_kpa10: u16,
    pub vbat_mv: u16,
    pub reject_count: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSlewResult {
    pub next_state: SensorSlewState,
    pub limited: SensorSlewInput,
    pub diagnostic: DiagnosticCode,
}

fn max_delta(rate_per_s: u32, dt_us: u32) -> u32 {
    ((rate_per_s as u64 * dt_us as u64) / US_PER_SEC) as u32
}

fn limit_u16(last: u16, candidate: u16, delta: u32) -> u16 {
    let min = last.saturating_sub(delta as u16);
    let max = last.saturating_add(delta as u16);
    candidate.clamp(min, max)
}

fn limit_i16(last: i16, candidate: i16, delta: u32) -> i16 {
    let lo = (last as i32 - delta as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    let hi = (last as i32 + delta as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    (candidate as i32).clamp(lo, hi) as i16
}

pub fn sensor_slew_step(input: SensorSlewInput, previous: SensorSlewState) -> SensorSlewResult {
    if !previous.initialized {
        let next = SensorSlewState {
            initialized: true,
            last_t_us: input.t_us,
            clt_c10: input.clt_c10,
            iat_c10: input.iat_c10,
            map_kpa10: input.map_kpa10,
            tps_x100: input.tps_x100,
            maf_x100: input.maf_x100,
            o2_afr_x100: input.o2_afr_x100,
            knock_intensity_x100: input.knock_intensity_x100,
            baro_kpa10: input.baro_kpa10,
            vbat_mv: input.vbat_mv,
            ..SensorSlewState::default()
        };

        return SensorSlewResult {
            next_state: next,
            limited: input,
            diagnostic: DiagnosticCode::None,
        };
    }

    let dt_us = input.t_us.get().wrapping_sub(previous.last_t_us.get());
    if dt_us < MIN_SLEW_DT_US {
        let limited = SensorSlewInput {
            t_us: input.t_us,
            clt_c10: previous.clt_c10,
            iat_c10: previous.iat_c10,
            map_kpa10: previous.map_kpa10,
            tps_x100: previous.tps_x100,
            maf_x100: previous.maf_x100,
            o2_afr_x100: previous.o2_afr_x100,
            knock_intensity_x100: previous.knock_intensity_x100,
            baro_kpa10: previous.baro_kpa10,
            vbat_mv: previous.vbat_mv,
        };
        let mut next = previous;
        next.last_t_us = input.t_us;

        return SensorSlewResult {
            next_state: next,
            limited,
            diagnostic: DiagnosticCode::None,
        };
    }

    let clt = TempC10::new(limit_i16(
        previous.clt_c10.get(),
        input.clt_c10.get(),
        max_delta(CLT_MAX_RATE_PER_S, dt_us),
    ));
    let iat = TempC10::new(limit_i16(
        previous.iat_c10.get(),
        input.iat_c10.get(),
        max_delta(IAT_MAX_RATE_PER_S, dt_us),
    ));
    let map = limit_u16(
        previous.map_kpa10,
        input.map_kpa10,
        max_delta(MAP_MAX_RATE_PER_S, dt_us),
    );
    let tps = limit_u16(
        previous.tps_x100,
        input.tps_x100,
        max_delta(TPS_MAX_RATE_PER_S, dt_us),
    );
    let maf = limit_u16(
        previous.maf_x100,
        input.maf_x100,
        max_delta(MAF_MAX_RATE_PER_S, dt_us),
    );
    let o2 = limit_u16(
        previous.o2_afr_x100,
        input.o2_afr_x100,
        max_delta(O2_MAX_RATE_PER_S, dt_us),
    );
    let knock = limit_u16(
        previous.knock_intensity_x100,
        input.knock_intensity_x100,
        max_delta(KNOCK_MAX_RATE_PER_S, dt_us),
    );
    let baro = limit_u16(
        previous.baro_kpa10,
        input.baro_kpa10,
        max_delta(BARO_MAX_RATE_PER_S, dt_us),
    );
    let vbat = limit_u16(
        previous.vbat_mv,
        input.vbat_mv,
        max_delta(VBAT_MAX_RATE_PER_S, dt_us),
    );

    let limited = SensorSlewInput {
        t_us: input.t_us,
        clt_c10: clt,
        iat_c10: iat,
        map_kpa10: map,
        tps_x100: tps,
        maf_x100: maf,
        o2_afr_x100: o2,
        knock_intensity_x100: knock,
        baro_kpa10: baro,
        vbat_mv: vbat,
    };

    let exceeded = limited.clt_c10 != input.clt_c10
        || limited.iat_c10 != input.iat_c10
        || limited.map_kpa10 != input.map_kpa10
        || limited.tps_x100 != input.tps_x100
        || limited.maf_x100 != input.maf_x100
        || limited.o2_afr_x100 != input.o2_afr_x100
        || limited.knock_intensity_x100 != input.knock_intensity_x100
        || limited.baro_kpa10 != input.baro_kpa10
        || limited.vbat_mv != input.vbat_mv;

    let mut next = previous;
    next.last_t_us = input.t_us;
    next.clt_c10 = limited.clt_c10;
    next.iat_c10 = limited.iat_c10;
    next.map_kpa10 = limited.map_kpa10;
    next.tps_x100 = limited.tps_x100;
    next.maf_x100 = limited.maf_x100;
    next.o2_afr_x100 = limited.o2_afr_x100;
    next.knock_intensity_x100 = limited.knock_intensity_x100;
    next.baro_kpa10 = limited.baro_kpa10;
    next.vbat_mv = limited.vbat_mv;
    if exceeded {
        next.reject_count = next.reject_count.saturating_add(1);
    } else {
        next.reject_count = 0;
    }

    SensorSlewResult {
        next_state: next,
        limited,
        diagnostic: if exceeded {
            DiagnosticCode::SensorPlausibilityFault
        } else {
            DiagnosticCode::None
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal_input(t_us: u32) -> SensorSlewInput {
        SensorSlewInput {
            t_us: Micros::new(t_us),
            clt_c10: TempC10::new(800),
            iat_c10: TempC10::new(250),
            map_kpa10: 1000,
            tps_x100: 2500,
            maf_x100: 12000,
            o2_afr_x100: 1470,
            knock_intensity_x100: 100,
            baro_kpa10: 1000,
            vbat_mv: 12000,
        }
    }

    #[test]
    fn first_sample_initializes_without_clamp() {
        let input = nominal_input(0);
        let step = sensor_slew_step(input, SensorSlewState::default());
        assert_eq!(step.limited, input);
        assert_eq!(step.diagnostic, DiagnosticCode::None);
        assert_eq!(step.next_state.clt_c10, input.clt_c10);
    }

    #[test]
    fn short_dt_keeps_last_accepted() {
        let input0 = nominal_input(0);
        let step0 = sensor_slew_step(input0, SensorSlewState::default());

        let mut input1 = nominal_input(500);
        input1.tps_x100 = 9000;
        let step1 = sensor_slew_step(input1, step0.next_state);

        assert_eq!(step1.limited.tps_x100, input0.tps_x100);
        assert_eq!(step1.diagnostic, DiagnosticCode::None);
        assert_eq!(step1.next_state.reject_count, 0);
    }

    #[test]
    fn exceeds_rate_is_clamped_and_faulted() {
        let step0 = sensor_slew_step(nominal_input(0), SensorSlewState::default());
        let mut input1 = nominal_input(1_000_000);
        input1.map_kpa10 = 4000;

        let step1 = sensor_slew_step(input1, step0.next_state);

        assert_eq!(step1.limited.map_kpa10, 3000);
        assert_eq!(step1.diagnostic, DiagnosticCode::SensorPlausibilityFault);
        assert_eq!(step1.next_state.reject_count, 1);
    }
}
