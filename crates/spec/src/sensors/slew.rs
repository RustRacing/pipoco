//! Sensor slew/rate limiting: the spec oracle's **independent** definition of
//! the delta-window limiter (review 011 / ADR 0012).
//!
//! This is deliberately NOT a delegation to `ecu_control::sensors::slew`. The
//! differential tests in `ecu-compat` compare that implementation against this
//! one, so routing both sides through the same arithmetic would make the
//! comparison true by construction and unable to catch a shared defect.
//!
//! The two derivations differ structurally on purpose: `ecu-control` computes a
//! `u32` delta and then narrows it to the channel width, while this module
//! fuses the rate and clamp steps in the `i64` domain and narrows only once, at
//! the final clamp. A truncation defect in one cannot mirror itself in the
//! other.
//!
//! The plain-data `SlewInput`/`SlewState` types are shared; only the arithmetic
//! is re-derived. `slew_rates_match_canonical` pins the rate constants so the
//! two definitions cannot drift apart silently.

pub use ecu_control::sensors::slew::{SlewInput, SlewState};

use crate::DiagnosticCode;

/// Minimum elapsed time before a new sample is rate-checked.
pub const MIN_SLEW_DT_US: u32 = 1_000;

const US_PER_SEC: i64 = 1_000_000;

pub const CLT_MAX_RATE_PER_S: u32 = 200;
pub const IAT_MAX_RATE_PER_S: u32 = 300;
pub const MAP_MAX_RATE_PER_S: u32 = 2_000;
pub const TPS_MAX_RATE_PER_S: u32 = 50_000;
pub const MAF_MAX_RATE_PER_S: u32 = 100_000;
pub const O2_MAX_RATE_PER_S: u32 = 2_000;
pub const KNOCK_MAX_RATE_PER_S: u32 = 50_000;
pub const BARO_MAX_RATE_PER_S: u32 = 50;
pub const VBAT_MAX_RATE_PER_S: u32 = 5_000;

/// Spec-compatible alias for the canonical slew input.
pub type SensorSlewInput = SlewInput;

/// Spec-compatible alias for the canonical slew state.
pub type SensorSlewState = SlewState;

/// Result of one slew step: next state, the rate-limited snapshot, and the
/// spec diagnostic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorSlewResult {
    pub next_state: SensorSlewState,
    pub limited: SensorSlewInput,
    pub diagnostic: DiagnosticCode,
}

/// Widest excursion permitted over `dt_us`, in `i64` so no intermediate can
/// wrap regardless of rate or elapsed time.
fn window(rate_per_s: u32, dt_us: u32) -> i64 {
    (i64::from(rate_per_s) * i64::from(dt_us)) / US_PER_SEC
}

/// Clamp `candidate` to within the rate window of `last`, narrowing only once
/// the value is known to be in range.
fn clamp_u16(last: u16, candidate: u16, rate_per_s: u32, dt_us: u32) -> u16 {
    let delta = window(rate_per_s, dt_us);
    let lo = (i64::from(last) - delta).clamp(0, i64::from(u16::MAX));
    let hi = (i64::from(last) + delta).clamp(0, i64::from(u16::MAX));
    i64::from(candidate).clamp(lo, hi) as u16
}

/// Signed counterpart of [`clamp_u16`].
fn clamp_i16(last: i16, candidate: i16, rate_per_s: u32, dt_us: u32) -> i16 {
    let delta = window(rate_per_s, dt_us);
    let lo = (i64::from(last) - delta).clamp(i64::from(i16::MIN), i64::from(i16::MAX));
    let hi = (i64::from(last) + delta).clamp(i64::from(i16::MIN), i64::from(i16::MAX));
    i64::from(candidate).clamp(lo, hi) as i16
}

/// Delta-window slew step, derived independently of `ecu-control`.
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

    let dt_us = input.t_us.wrapping_sub(previous.last_t_us);
    if dt_us < MIN_SLEW_DT_US {
        let mut next = previous;
        next.last_t_us = input.t_us;

        return SensorSlewResult {
            next_state: next,
            limited: SensorSlewInput {
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
            },
            diagnostic: DiagnosticCode::None,
        };
    }

    let limited = SensorSlewInput {
        t_us: input.t_us,
        clt_c10: clamp_i16(previous.clt_c10, input.clt_c10, CLT_MAX_RATE_PER_S, dt_us),
        iat_c10: clamp_i16(previous.iat_c10, input.iat_c10, IAT_MAX_RATE_PER_S, dt_us),
        map_kpa10: clamp_u16(
            previous.map_kpa10,
            input.map_kpa10,
            MAP_MAX_RATE_PER_S,
            dt_us,
        ),
        tps_x100: clamp_u16(previous.tps_x100, input.tps_x100, TPS_MAX_RATE_PER_S, dt_us),
        maf_x100: clamp_u16(previous.maf_x100, input.maf_x100, MAF_MAX_RATE_PER_S, dt_us),
        o2_afr_x100: clamp_u16(
            previous.o2_afr_x100,
            input.o2_afr_x100,
            O2_MAX_RATE_PER_S,
            dt_us,
        ),
        knock_intensity_x100: clamp_u16(
            previous.knock_intensity_x100,
            input.knock_intensity_x100,
            KNOCK_MAX_RATE_PER_S,
            dt_us,
        ),
        baro_kpa10: clamp_u16(
            previous.baro_kpa10,
            input.baro_kpa10,
            BARO_MAX_RATE_PER_S,
            dt_us,
        ),
        vbat_mv: clamp_u16(previous.vbat_mv, input.vbat_mv, VBAT_MAX_RATE_PER_S, dt_us),
    };

    let exceeded = limited != input;

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
    next.reject_count = if exceeded {
        next.reject_count.saturating_add(1)
    } else {
        0
    };

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

    /// The two derivations are independent by design, but they must agree on
    /// the rate constants; drift there would be a spec change, not a bug.
    #[test]
    fn slew_rates_match_canonical() {
        use ecu_control::sensors::slew as canonical;
        assert_eq!(MIN_SLEW_DT_US, canonical::MIN_SLEW_DT_US);
        assert_eq!(CLT_MAX_RATE_PER_S, canonical::CLT_MAX_RATE_PER_S);
        assert_eq!(IAT_MAX_RATE_PER_S, canonical::IAT_MAX_RATE_PER_S);
        assert_eq!(MAP_MAX_RATE_PER_S, canonical::MAP_MAX_RATE_PER_S);
        assert_eq!(TPS_MAX_RATE_PER_S, canonical::TPS_MAX_RATE_PER_S);
        assert_eq!(MAF_MAX_RATE_PER_S, canonical::MAF_MAX_RATE_PER_S);
        assert_eq!(O2_MAX_RATE_PER_S, canonical::O2_MAX_RATE_PER_S);
        assert_eq!(KNOCK_MAX_RATE_PER_S, canonical::KNOCK_MAX_RATE_PER_S);
        assert_eq!(BARO_MAX_RATE_PER_S, canonical::BARO_MAX_RATE_PER_S);
        assert_eq!(VBAT_MAX_RATE_PER_S, canonical::VBAT_MAX_RATE_PER_S);
    }

    /// Wide windows must not truncate into a narrower one. This is the defect
    /// class the independent derivation exists to expose.
    #[test]
    fn wide_window_is_not_truncated() {
        let step0 = sensor_slew_step(nominal_input(0), SensorSlewState::default());
        let mut input1 = nominal_input(655_360);
        input1.maf_x100 = 60_000;

        let step1 = sensor_slew_step(input1, step0.next_state);

        assert_eq!(step1.limited.maf_x100, 60_000);
        assert_eq!(step1.diagnostic, DiagnosticCode::None);
    }
}
