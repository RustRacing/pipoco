//! Deterministic headless smoke scenario for the ECU driver.

mod config;
mod execution;
mod output;

pub use config::{
    default_hifi_scenario_config, hifi_scenario_calibrations, hifi_scenario_initial_rpm,
    hifi_scenario_initial_temperature_k, hifi_scenario_step_frame, DriverRunReport,
    DriverScenarioSignals, HifiDriverRunReport, ScenarioBackend, ScenarioConfig, ScenarioKind,
};
pub use execution::{
    run_cold_start_scenario, run_cold_start_scenario_with_backend, run_default_headless_scenario,
    run_default_headless_smoke, run_default_headless_smoke_twice, run_dfco_decel_scenario,
    run_dfco_decel_scenario_with_backend, run_headless_hifi_smoke, run_headless_smoke,
    run_hot_restart_scenario, run_hot_restart_scenario_with_backend,
    run_sync_loss_recovery_scenario, run_sync_loss_recovery_scenario_with_backend,
};

#[cfg(test)]
mod tests;
