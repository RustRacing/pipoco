//! Sensor slew/rate limiting: thin wrapper over the canonical `ecu-control`
//! implementation (review 011 / ADR 0012). The delta-window limiter lives in
//! `ecu_control::sensors::slew`; this module only adapts the primitive
//! exceeded flag to the spec `DiagnosticCode` surface.

pub use ecu_control::sensors::slew::{slew_step, SlewInput, SlewState};

use crate::DiagnosticCode;

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

/// Delta-window slew step (delegates to `ecu-control`).
pub fn sensor_slew_step(input: SensorSlewInput, previous: SensorSlewState) -> SensorSlewResult {
    let result = slew_step(input, previous);
    SensorSlewResult {
        next_state: result.next_state,
        limited: result.limited,
        diagnostic: if result.exceeded {
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
}
