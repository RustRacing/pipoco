//! Sensor plausibility: the spec oracle's **independent** definition of the
//! debounced fault latch (review 011 / ADR 0012).
//!
//! This deliberately does NOT delegate to
//! `ecu_control::sensors::plausibility`. The differential tests in `ecu-compat`
//! compare implementations against this oracle; if the oracle were an alias for
//! one of them the comparison would be true by construction and unable to catch
//! a shared defect (see the slew truncation defect that motivated this).
//!
//! The derivations differ structurally on purpose: `ecu-control` evaluates a
//! boolean chain and keeps two clamped counters, while this module folds the
//! range checks over a table and tracks a single persistence streak whose sign
//! is the current verdict.
//!
//! The plain-data `PlausibilityInput`/`PlausibilityState` types are shared;
//! only the decision logic is re-derived. `plausibility_limits_match_canonical`
//! pins the shared constants so the definitions cannot drift silently.

pub use ecu_control::sensors::plausibility::{PlausibilityInput, PlausibilityState};

use crate::DiagnosticCode;

/// Fault must persist this long (us) before it latches.
pub const PLAUSIBILITY_DEBOUNCE_US: u32 = 500_000;
/// Plausibility gate is only active at or above this RPM.
pub const PLAUSIBILITY_MIN_RPM: u16 = 1000;

/// Spec-compatible alias for the canonical plausibility input.
pub type SensorPlausibilityInput = PlausibilityInput;

/// Spec-compatible alias for the canonical plausibility state.
pub type SensorPlausibilityState = PlausibilityState;

/// Result of one plausibility step: next state plus the spec diagnostic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorPlausibilityResult {
    pub next_state: SensorPlausibilityState,
    pub diagnostic: DiagnosticCode,
}

/// Inclusive acceptance band for one channel, in that channel's raw units.
struct Band {
    value: i32,
    lo: i32,
    hi: i32,
}

fn any_out_of_band(input: SensorPlausibilityInput) -> bool {
    let bands = [
        Band {
            value: input.clt_c10 as i32,
            lo: -400,
            hi: 1500,
        },
        Band {
            value: input.iat_c10 as i32,
            lo: -400,
            hi: 1200,
        },
        Band {
            value: input.map_kpa10 as i32,
            lo: 100,
            hi: 3000,
        },
        Band {
            value: input.tps_x100 as i32,
            lo: 0,
            hi: 10_000,
        },
        Band {
            value: input.maf_x100 as i32,
            lo: 0,
            hi: 60_000,
        },
        Band {
            value: input.o2_afr_x100 as i32,
            lo: 500,
            hi: 3000,
        },
        Band {
            value: input.knock_intensity_x100 as i32,
            lo: 0,
            hi: 10_000,
        },
        Band {
            value: input.baro_kpa10 as i32,
            lo: 500,
            hi: 1200,
        },
        Band {
            value: input.vbat_mv as i32,
            lo: 6000,
            hi: 18_000,
        },
    ];

    bands
        .iter()
        .any(|band| band.value < band.lo || band.value > band.hi)
}

/// Throttle and manifold pressure must not disagree about engine load.
fn load_signals_disagree(input: SensorPlausibilityInput) -> bool {
    let throttle_open_no_vacuum = input.tps_x100 >= 8_000 && input.map_kpa10 <= 300;
    let throttle_shut_no_pumping = input.tps_x100 <= 1_000 && input.map_kpa10 >= 950;
    throttle_open_no_vacuum || throttle_shut_no_pumping
}

/// Every channel repeating its previous reading exactly means the acquisition
/// path has stopped updating.
fn every_channel_repeated(input: SensorPlausibilityInput, state: &SensorPlausibilityState) -> bool {
    state.initialized
        && [
            (state.last_clt_c10 as i32, input.clt_c10 as i32),
            (state.last_iat_c10 as i32, input.iat_c10 as i32),
            (state.last_map_kpa10 as i32, input.map_kpa10 as i32),
            (state.last_tps_x100 as i32, input.tps_x100 as i32),
            (state.last_maf_x100 as i32, input.maf_x100 as i32),
            (state.last_o2_afr_x100 as i32, input.o2_afr_x100 as i32),
            (
                state.last_knock_intensity_x100 as i32,
                input.knock_intensity_x100 as i32,
            ),
            (state.last_baro_kpa10 as i32, input.baro_kpa10 as i32),
            (state.last_vbat_mv as i32, input.vbat_mv as i32),
        ]
        .iter()
        .all(|(last, now)| last == now)
}

fn fault_present(input: SensorPlausibilityInput, state: &SensorPlausibilityState) -> bool {
    any_out_of_band(input) || load_signals_disagree(input) || every_channel_repeated(input, state)
}

/// Persistence accumulator, saturating at the debounce threshold.
fn extend_streak(streak_us: u32, dt_us: u32) -> u32 {
    streak_us
        .saturating_add(dt_us)
        .min(PLAUSIBILITY_DEBOUNCE_US)
}

/// Debounced-latch plausibility step, derived independently of `ecu-control`.
pub fn sensor_plausibility_step(
    input: SensorPlausibilityInput,
    previous: SensorPlausibilityState,
) -> SensorPlausibilityResult {
    let mut next = previous;

    let dt_us = if previous.initialized {
        input.t_us.wrapping_sub(previous.last_t_us)
    } else {
        0
    };

    if input.rpm >= PLAUSIBILITY_MIN_RPM {
        // One streak is live and the other is held at zero; which one depends on
        // the verdict for this sample. The latch flips only once a streak has
        // run for the full debounce window.
        if fault_present(input, &previous) {
            next.assert_counter_us = extend_streak(previous.assert_counter_us, dt_us);
            next.clear_counter_us = 0;
            next.latched = previous.latched || next.assert_counter_us >= PLAUSIBILITY_DEBOUNCE_US;
        } else {
            next.clear_counter_us = extend_streak(previous.clear_counter_us, dt_us);
            next.assert_counter_us = 0;
            next.latched = previous.latched && next.clear_counter_us < PLAUSIBILITY_DEBOUNCE_US;
        }
    }

    next.initialized = true;
    next.last_t_us = input.t_us;
    next.last_clt_c10 = input.clt_c10;
    next.last_iat_c10 = input.iat_c10;
    next.last_map_kpa10 = input.map_kpa10;
    next.last_tps_x100 = input.tps_x100;
    next.last_maf_x100 = input.maf_x100;
    next.last_o2_afr_x100 = input.o2_afr_x100;
    next.last_knock_intensity_x100 = input.knock_intensity_x100;
    next.last_baro_kpa10 = input.baro_kpa10;
    next.last_vbat_mv = input.vbat_mv;

    SensorPlausibilityResult {
        next_state: next,
        diagnostic: if next.latched {
            DiagnosticCode::SensorPlausibilityFault
        } else {
            DiagnosticCode::None
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal_input(t_us: u32) -> SensorPlausibilityInput {
        SensorPlausibilityInput {
            t_us,
            rpm: 2000,
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
    fn out_of_range_fault_latches_after_debounce() {
        let state0 = SensorPlausibilityState::default();
        let mut bad = nominal_input(0);
        bad.map_kpa10 = 50;

        let step0 = sensor_plausibility_step(bad, state0);
        assert_eq!(step0.diagnostic, DiagnosticCode::None);

        let mut bad_late = bad;
        bad_late.t_us = 500_000;
        let step1 = sensor_plausibility_step(bad_late, step0.next_state);
        assert_eq!(step1.diagnostic, DiagnosticCode::SensorPlausibilityFault);
    }

    #[test]
    fn stuck_fault_latches_after_debounce_and_clears_after_debounce() {
        let input0 = nominal_input(0);
        let step0 = sensor_plausibility_step(input0, SensorPlausibilityState::default());
        assert_eq!(step0.diagnostic, DiagnosticCode::None);

        let input1 = nominal_input(500_000);
        let step1 = sensor_plausibility_step(input1, step0.next_state);
        assert_eq!(step1.diagnostic, DiagnosticCode::SensorPlausibilityFault);

        let mut input2 = nominal_input(1_000_000);
        input2.map_kpa10 = 1010;
        let step2 = sensor_plausibility_step(input2, step1.next_state);
        assert_eq!(step2.diagnostic, DiagnosticCode::None);
    }

    #[test]
    fn gate_disabled_below_min_rpm() {
        let mut input = nominal_input(0);
        input.rpm = 900;
        input.map_kpa10 = 50;

        let step = sensor_plausibility_step(input, SensorPlausibilityState::default());
        assert_eq!(step.diagnostic, DiagnosticCode::None);
    }

    /// The derivations are independent by design, but a divergence in the
    /// shared constants would be a spec change rather than a bug.
    #[test]
    fn plausibility_limits_match_canonical() {
        use ecu_control::sensors::plausibility as canonical;
        assert_eq!(
            PLAUSIBILITY_DEBOUNCE_US,
            canonical::PLAUSIBILITY_DEBOUNCE_US
        );
        assert_eq!(PLAUSIBILITY_MIN_RPM, canonical::PLAUSIBILITY_MIN_RPM);
    }

    /// The cross-check threshold is a boundary, so probe both sides of it
    /// rather than only the faulting value.
    #[test]
    fn load_disagreement_boundary_is_exact() {
        let mut just_below = nominal_input(0);
        just_below.tps_x100 = 7_999;
        just_below.map_kpa10 = 300;
        assert!(!super::load_signals_disagree(just_below));

        let mut at_threshold = just_below;
        at_threshold.tps_x100 = 8_000;
        assert!(super::load_signals_disagree(at_threshold));

        let mut shut_below = nominal_input(0);
        shut_below.tps_x100 = 1_000;
        shut_below.map_kpa10 = 949;
        assert!(!super::load_signals_disagree(shut_below));

        let mut shut_at = shut_below;
        shut_at.map_kpa10 = 950;
        assert!(super::load_signals_disagree(shut_at));
    }
}
