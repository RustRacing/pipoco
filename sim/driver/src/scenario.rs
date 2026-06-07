//! Deterministic headless smoke scenario for the ECU driver.

mod config;
mod execution;
mod output;

pub use config::{DriverRunReport, DriverScenarioSignals, ScenarioConfig, ScenarioKind};
pub use execution::{
    run_cold_start_scenario, run_default_headless_smoke, run_default_headless_smoke_twice,
    run_dfco_decel_scenario, run_headless_smoke, run_hot_restart_scenario,
    run_sync_loss_recovery_scenario,
};

#[cfg(test)]
mod tests;
