//! Sensor plausibility: thin wrapper over the canonical `ecu-control`
//! implementation (review 011 / ADR 0012). The debounce-latch algorithm lives
//! in `ecu_control::sensors::plausibility`; this module only adapts the
//! primitive verdict to the spec `DiagnosticCode` surface.

pub use ecu_control::sensors::plausibility::{
    plausibility_step, PlausibilityInput, PlausibilityState,
};

use crate::DiagnosticCode;

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

/// Debounced-latch plausibility step (delegates to `ecu-control`).
pub fn sensor_plausibility_step(
    input: SensorPlausibilityInput,
    previous: SensorPlausibilityState,
) -> SensorPlausibilityResult {
    let result = plausibility_step(input, previous);
    SensorPlausibilityResult {
        next_state: result.next_state,
        diagnostic: if result.latched {
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
}
