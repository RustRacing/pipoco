//! Legacy `EcuState` to runtime calibration adapters.
//!
//! These helpers intentionally stay in `ecu-compat` because they depend on the
//! legacy `EcuState` layout. Board-common should remain generic runtime/IO
//! plumbing and not depend on `EcuState`.

use crate::compat::EcuState;
use ecu_calibration::FuelRuntimeTune;
use ecu_runtime::{RuntimeFuelStrategy, RuntimeSemanticCalibration};

pub fn runtime_semantic_calibration_from_state(state: &EcuState) -> RuntimeSemanticCalibration {
    let tune = fuel_runtime_tune_from_state(state);
    runtime_semantic_calibration_from_fuel_tune(&tune)
}

pub fn runtime_semantic_calibration_from_fuel_tune(
    tune: &FuelRuntimeTune,
) -> RuntimeSemanticCalibration {
    ecu_runtime::runtime_semantic_calibration_from_fuel_tune(tune)
}

pub fn runtime_fuel_strategy_from_state(state: &EcuState) -> RuntimeFuelStrategy {
    let tune = fuel_runtime_tune_from_state(state);
    runtime_fuel_strategy_from_fuel_tune(&tune)
}

pub fn runtime_fuel_strategy_from_fuel_tune(tune: &FuelRuntimeTune) -> RuntimeFuelStrategy {
    ecu_runtime::runtime_fuel_strategy_from_fuel_tune(tune)
}

fn fuel_runtime_tune_from_state(state: &EcuState) -> FuelRuntimeTune {
    FuelRuntimeTune::new(
        state.config.ve_table,
        state.config.afr_table,
        state.config.required_fuel_us,
        state.config.injector_deadtime_us,
        state.config.ve_load_source,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn semantic_calibration_uses_required_fuel_and_deadtime_from_state() {
        let mut state = EcuState::new();
        state.config.required_fuel_us = 2345;
        state.config.injector_deadtime_us = 975;
        let cal = runtime_semantic_calibration_from_state(&state);
        assert_eq!(cal.required_fuel_us, 2345);
        assert_eq!(cal.deadtime_table_us.values[0][0], 975);
        assert_eq!(cal.deadtime_table_us.values[15][15], 975);
    }

    #[test]
    fn semantic_calibration_from_fuel_tune_matches_state_shim() {
        let mut state = EcuState::new();
        state.config.ve_table[2][3] = 77;
        state.config.ve_table[13][11] = 123;
        state.config.afr_table[4][5] = 132;
        state.config.afr_table[14][8] = 155;
        state.config.required_fuel_us = 3210;
        state.config.injector_deadtime_us = 654;

        let tune = fuel_runtime_tune_from_state(&state);
        let from_state = runtime_semantic_calibration_from_state(&state);
        let from_tune = runtime_semantic_calibration_from_fuel_tune(&tune);

        assert_eq!(from_tune, from_state);
        assert_eq!(from_tune.ve_table.values[2][3], 77);
        assert_eq!(from_tune.ve_table.values[13][11], 123);
        assert_eq!(from_tune.afr_target_table.values[4][5], 132);
        assert_eq!(from_tune.afr_target_table.values[14][8], 155);
        assert_eq!(from_tune.required_fuel_us, 3210);
        assert_eq!(from_tune.deadtime_table_us.values[0][0], 654);
        assert_eq!(from_tune.deadtime_table_us.values[15][15], 654);
    }

    #[test]
    fn runtime_fuel_strategy_selection_respects_load_source() {
        let mut state = EcuState::new();
        state.config.ve_load_source = 0;
        assert!(matches!(
            runtime_fuel_strategy_from_state(&state),
            RuntimeFuelStrategy::SpeedDensityVe { .. }
        ));
        state.config.ve_load_source = 1;
        assert!(matches!(
            runtime_fuel_strategy_from_state(&state),
            RuntimeFuelStrategy::AlphaN { .. }
        ));
    }

    #[test]
    fn runtime_fuel_strategy_from_fuel_tune_matches_state_shim_for_load_sources() {
        let mut state = EcuState::new();
        state.config.ve_load_source = 0;
        let speed_density_tune = fuel_runtime_tune_from_state(&state);
        assert_eq!(
            runtime_fuel_strategy_from_fuel_tune(&speed_density_tune),
            runtime_fuel_strategy_from_state(&state)
        );
        assert!(matches!(
            runtime_fuel_strategy_from_fuel_tune(&speed_density_tune),
            RuntimeFuelStrategy::SpeedDensityVe { .. }
        ));

        state.config.ve_load_source = 1;
        let alpha_n_tune = fuel_runtime_tune_from_state(&state);
        assert_eq!(
            runtime_fuel_strategy_from_fuel_tune(&alpha_n_tune),
            runtime_fuel_strategy_from_state(&state)
        );
        assert!(matches!(
            runtime_fuel_strategy_from_fuel_tune(&alpha_n_tune),
            RuntimeFuelStrategy::AlphaN { .. }
        ));
    }
}
