//! Host-only deterministic ECU simulation driver.
//!
//! This crate provides a safe FFI wrapper around `ecu-sim-ffi` for driving
//! the ECU simulation from a host-only context. It does not depend on any
//! external simulator source.

mod embedded_loop;
pub mod ffi_client;
pub mod hifi_bridge;
mod output_validation;
mod plant_bridge;
mod readiness;
mod scenario;
pub mod trace;
mod trigger_replay;
mod x86_runtime_board;

pub use embedded_loop::{
    FixedOutputQueue, FixedTraceBuffer, FixedTriggerEdgeBuffer, SimBoard, SimBoardLoop,
    SimBoardLoopConfig, SimBoardLoopError, SimBoardLoopReport, SimBoardTraceKind,
    SimBoardTraceRecord, SimBufferOverflow, SimControlMode, SimDriverInput, SimEdgePolarity,
    SimEnvironment, SimSensorFrame, SimTriggerEdge, SimTriggerLine, TorqueNmX100,
};
pub use ffi_client::{
    diagnostics_from_snapshot, observability_from_snapshot_and_sensor_frame, sensor_frame_to_ffi,
    EcuFfiClient,
};
pub use hifi_bridge::{
    bridge_output_transitions_to_hifi_input, drive_hifi_board_step, drive_hifi_loop_tick,
    drive_hifi_runtime_board_tick, quantize_hifi_output_to_sensor_frame, run_hifi_adapter_step,
    synthesize_hifi_trigger_edges, X86HifiAdapterStep, X86HifiAdapterStepInput,
    X86HifiBoardStepError, X86HifiCrankReference, X86HifiLoopPlant, X86HifiLoopTickError,
    X86HifiPlantBridgeFrame, X86HifiRuntimeTickError,
};
pub use plant_bridge::X86PlantBridgeDiagnostics;
pub use readiness::{SimulatorReadinessEvidence, SoftwareReadinessReport};
pub use scenario::{
    run_cold_start_scenario, run_cold_start_scenario_with_backend, run_default_headless_scenario,
    run_default_headless_smoke, run_default_headless_smoke_twice, run_dfco_decel_scenario,
    run_dfco_decel_scenario_with_backend, run_headless_hifi_smoke, run_headless_smoke,
    run_hot_restart_scenario, run_hot_restart_scenario_with_backend,
    run_sync_loss_recovery_scenario, run_sync_loss_recovery_scenario_with_backend, DriverRunReport,
    DriverScenarioSignals, HifiDriverRunReport, ScenarioBackend, ScenarioConfig, ScenarioKind,
};
pub use trace::{
    DriverDecision, DriverDiagnostics, DriverFreezeFrame, DriverObservability, DriverTraceKind,
    DriverTraceRecord, FixedDriverTrace, DRIVER_TRACE_CAP,
};
pub use trigger_replay::{
    replay_frame_time_us, replay_trigger_edges, ReplayCamLevel, ReplayTriggerChannel,
    SyntheticTriggerEdge, TriggerReplay, TriggerReplayError, TriggerReplayFixture,
    TriggerReplayFrame, SIXTY_MINUS_TWO_MISSING_TEETH, SIXTY_MINUS_TWO_NOMINAL_TEETH,
    SIXTY_MINUS_TWO_OBSERVED_TEETH,
};
pub use x86_runtime_board::{
    bridge_output_transitions_to_core_frame, run_x86_runtime_tick, X86RuntimeBoard,
    X86RuntimeBoardDiagnostics, X86RuntimeBoardError, X86RuntimePlantBridgeFrame,
    X86RuntimeTickResult,
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
    InvalidOutputChannel {
        kind: ecu_io::OutputTransitionKind,
        channel: u8,
    },
    ScenarioDidNotSync,
    ScenarioDidNotEmitInjection,
    ScenarioDidNotEmitIgnition,
    ScenarioDidNotCombust,
    ScenarioTraceMismatch,
    HifiPlant(ecu_sim_hifi::PlantConfigError),
}
