use crate::{EngineMode, InputSnapshot, LogicalState};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SafetyResult {
    pub safety_latched: bool,
}

#[must_use]
pub fn safety_step(input: InputSnapshot, state: &LogicalState) -> SafetyResult {
    let fault_asserted = latch_fault_asserted(input, state);
    let clear_condition = latch_clear_condition(input, state);

    let safety_latched = if fault_asserted {
        true
    } else if clear_condition {
        false
    } else {
        state.safety_latched
    };

    SafetyResult { safety_latched }
}

fn latch_fault_asserted(input: InputSnapshot, state: &LogicalState) -> bool {
    input.fuel_cut
        || input.spark_cut
        || matches!(input.mode, EngineMode::Shutdown)
        || state.sensor_plausibility_state.latched
}

fn latch_clear_condition(input: InputSnapshot, state: &LogicalState) -> bool {
    matches!(input.mode, EngineMode::Off)
        && !input.fuel_cut
        && !input.spark_cut
        && !state.sensor_plausibility_state.latched
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shutdown_asserts_latch() {
        let input = InputSnapshot {
            mode: EngineMode::Shutdown,
            ..InputSnapshot::default()
        };
        let result = safety_step(input, &LogicalState::default());
        assert!(result.safety_latched);
    }

    #[test]
    fn fuel_cut_asserts_latch_and_off_clears() {
        let input = InputSnapshot {
            fuel_cut: true,
            ..InputSnapshot::default()
        };
        let state = LogicalState::default();
        let asserted = safety_step(input, &state);
        assert!(asserted.safety_latched);

        let cleared_input = InputSnapshot {
            mode: EngineMode::Off,
            ..InputSnapshot::default()
        };
        let asserted_state = LogicalState {
            safety_latched: true,
            ..LogicalState::default()
        };
        let cleared = safety_step(cleared_input, &asserted_state);
        assert!(!cleared.safety_latched);
    }

    #[test]
    fn holds_latch_until_clear_condition() {
        let state = LogicalState {
            safety_latched: true,
            ..LogicalState::default()
        };
        let running = InputSnapshot {
            mode: EngineMode::Running,
            ..InputSnapshot::default()
        };
        let held = safety_step(running, &state);
        assert!(held.safety_latched);
    }
}
