//! Host-only deterministic ECU simulation driver.
//!
//! This crate provides a safe FFI wrapper around `ecu-sim-ffi` for driving
//! the ECU simulation from a host-only context. It does not depend on any
//! external simulator source.

pub mod embedded_loop;
pub mod ffi_client;
pub mod scenario;
pub mod trace;
pub mod trigger_replay;
pub mod x86_board;
pub mod x86_runtime_board;

pub use ffi_client::EcuFfiClient;
pub use scenario::{
    run_cold_start_scenario, run_default_headless_smoke, run_default_headless_smoke_twice,
    run_dfco_decel_scenario, run_headless_smoke, run_hot_restart_scenario,
    run_sync_loss_recovery_scenario, DriverRunReport, DriverScenarioSignals, ScenarioConfig,
    ScenarioKind,
};
pub use trace::{
    DriverDecision, DriverDiagnostics, DriverFreezeFrame, DriverObservability, DriverTraceKind,
    DriverTraceRecord, FixedDriverTrace, DRIVER_TRACE_CAP,
};

/// Driver error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriverError {
    FfiStatus(ecu_sim_ffi::EcuSimStatus),
    Plant(ecu_sim::plant::PlantError),
    EventOverflow,
    PendingOutputOverflow,
    TraceOverflow,
    UnknownOutputKind(i32),
    ScenarioDidNotSync,
    ScenarioDidNotEmitInjection,
    ScenarioDidNotEmitIgnition,
    ScenarioDidNotCombust,
    ScenarioTraceMismatch,
}
