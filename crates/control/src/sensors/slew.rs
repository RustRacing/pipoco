//! Canonical sensor slew/rate limiting: clamps each channel to a per-second
//! rate over the elapsed delta window. Ported from the frozen spec oracle
//! (review 011). The `limit_*`/`max_delta` helpers are the shared primitive
//! used by `ecu-compat`'s per-channel validators.

pub const MIN_SLEW_DT_US: u32 = 1_000;
const US_PER_SEC: u64 = 1_000_000;

pub const CLT_MAX_RATE_PER_S: u32 = 200;
pub const IAT_MAX_RATE_PER_S: u32 = 300;
pub const MAP_MAX_RATE_PER_S: u32 = 2_000;
pub const TPS_MAX_RATE_PER_S: u32 = 50_000;
pub const MAF_MAX_RATE_PER_S: u32 = 100_000;
pub const O2_MAX_RATE_PER_S: u32 = 2_000;
pub const KNOCK_MAX_RATE_PER_S: u32 = 50_000;
pub const BARO_MAX_RATE_PER_S: u32 = 50;
pub const VBAT_MAX_RATE_PER_S: u32 = 5_000;

/// Raw sensor snapshot fed to the slew limiter.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SlewInput {
    pub t_us: u32,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub maf_x100: u16,
    pub o2_afr_x100: u16,
    pub knock_intensity_x100: u16,
    pub baro_kpa10: u16,
    pub vbat_mv: u16,
}

/// Slew limiter state (last accepted channel values).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SlewState {
    pub initialized: bool,
    pub last_t_us: u32,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub maf_x100: u16,
    pub o2_afr_x100: u16,
    pub knock_intensity_x100: u16,
    pub baro_kpa10: u16,
    pub vbat_mv: u16,
    pub reject_count: u16,
}

/// Result of one slew step: next state, the rate-limited snapshot, and whether
/// any channel was clamped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SlewResult {
    pub next_state: SlewState,
    pub limited: SlewInput,
    pub exceeded: bool,
}

/// Maximum allowed delta for `rate_per_s` over `dt_us`.
pub fn max_delta(rate_per_s: u32, dt_us: u32) -> u32 {
    ((rate_per_s as u64 * dt_us as u64) / US_PER_SEC) as u32
}

/// Clamp `candidate` to within `delta` of `last`.
pub fn limit_u16(last: u16, candidate: u16, delta: u32) -> u16 {
    let min = last.saturating_sub(delta as u16);
    let max = last.saturating_add(delta as u16);
    candidate.clamp(min, max)
}

/// Clamp `candidate` to within `delta` of `last` (signed).
pub fn limit_i16(last: i16, candidate: i16, delta: u32) -> i16 {
    let lo = (last as i32 - delta as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    let hi = (last as i32 + delta as i32).clamp(i16::MIN as i32, i16::MAX as i32);
    (candidate as i32).clamp(lo, hi) as i16
}

/// Advance the slew limiter with one raw sensor snapshot.
pub fn slew_step(input: SlewInput, previous: SlewState) -> SlewResult {
    if !previous.initialized {
        let next = SlewState {
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
            ..SlewState::default()
        };

        return SlewResult {
            next_state: next,
            limited: input,
            exceeded: false,
        };
    }

    let dt_us = input.t_us.wrapping_sub(previous.last_t_us);
    if dt_us < MIN_SLEW_DT_US {
        let limited = SlewInput {
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

        return SlewResult {
            next_state: next,
            limited,
            exceeded: false,
        };
    }

    let clt = limit_i16(
        previous.clt_c10,
        input.clt_c10,
        max_delta(CLT_MAX_RATE_PER_S, dt_us),
    );
    let iat = limit_i16(
        previous.iat_c10,
        input.iat_c10,
        max_delta(IAT_MAX_RATE_PER_S, dt_us),
    );
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

    let limited = SlewInput {
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

    SlewResult {
        next_state: next,
        limited,
        exceeded,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal_input(t_us: u32) -> SlewInput {
        SlewInput {
            t_us,
            clt_c10: 800,
            iat_c10: 250,
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
        let step = slew_step(input, SlewState::default());
        assert_eq!(step.limited, input);
        assert!(!step.exceeded);
        assert_eq!(step.next_state.clt_c10, input.clt_c10);
    }

    #[test]
    fn short_dt_keeps_last_accepted() {
        let input0 = nominal_input(0);
        let step0 = slew_step(input0, SlewState::default());

        let mut input1 = nominal_input(500);
        input1.tps_x100 = 9000;
        let step1 = slew_step(input1, step0.next_state);

        assert_eq!(step1.limited.tps_x100, input0.tps_x100);
        assert!(!step1.exceeded);
        assert_eq!(step1.next_state.reject_count, 0);
    }

    #[test]
    fn exceeds_rate_is_clamped_and_faulted() {
        let step0 = slew_step(nominal_input(0), SlewState::default());
        let mut input1 = nominal_input(1_000_000);
        input1.map_kpa10 = 4000;

        let step1 = slew_step(input1, step0.next_state);

        assert_eq!(step1.limited.map_kpa10, 3000);
        assert!(step1.exceeded);
        assert_eq!(step1.next_state.reject_count, 1);
    }
}
