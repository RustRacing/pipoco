#![cfg_attr(not(test), no_std)]

//! Deterministic runtime orchestration and action emission.
//!
//! # Supported integration surfaces
//!
//! Product callers should prefer the structured, authority-aware stepping path
//! exposed through [`ingress`]. This is the canonical runtime boundary.
//!
//! Raw [`StepInputs`] and related fixture-oriented helpers remain supported for
//! compatibility and support use, but they are not the preferred product
//! ingress. Conformance- and representability-oriented helpers live under
//! [`support`], not the main product-facing surface.
//!
//! See `aidocs/architecture/adr-0010-runtime-compat-boundaries.md`.
//!
mod actions;
mod engine;
mod lowering;
mod observations;
mod outputs;
#[cfg(test)]
mod queues;
pub mod semantic;
pub mod ingress {
    pub use crate::observations::{
        AuthorityStepInputs, ControlInputs, RuntimeAuthorityError, StepResult,
    };
    pub use ecu_domain::EngineTimeAuthority;
}
pub mod compat {
    pub use crate::observations::StepInputs;
}
pub mod support {
    pub use crate::observations::{
        extract_fuel_observations, extract_torque_observations, DifferentialInputSnapshot,
        RuntimeAdapterContract, RuntimeObservedSurface,
    };
}

pub use actions::{
    Action, ActionBatch, ActionExecutor, ActionExportError, RuntimeScheduledLevel,
    RuntimeScheduledOutputKind, RuntimeScheduledTransition, RuntimeScheduledTransitionBatch,
    TransportPublisher,
};
pub use ecu_board_api::{
    AuxSafetyProfile, FullEcuOutputProfile, IgnitionOutputProfile, InjectionOutputProfile,
    OutputAuthorityRequirement, RuntimeOutputProfile, FULL_ECU_MAX_CYLINDERS,
    FULL_ECU_MAX_LIMP_AUX_OUTPUTS,
};
pub use engine::{ControlPlannerState, EngineRuntime, RuntimeFuelStrategy};
pub use lowering::{
    lower_action_batch_to_board_batches, lower_action_batch_to_timing_island,
    lower_action_to_board_batches, lower_action_to_timing_island, ActionLoweringError,
    ActionLoweringStatus, ActionOutputBatchAdapter, BoardApiBatchExecutor,
};
pub use observations::{
    AuthorityStepInputs, CalibrationState, CamObservation, ControlInputs, ControlPlan,
    ControlState, DecoderObservation, EngineState, FaultState, RuntimeAfrOverride,
    RuntimeEngineMode, RuntimeFuelObservations, RuntimeSnapshot, StepResult, TorqueObservations,
    TriggerObservation, ValidatedInputs,
};
pub use outputs::runtime_full_sequential_authorized;
#[cfg(test)]
pub(crate) use queues::{Event, FastEvent, QueueOverflow, QueueResult, RuntimeQueues, SlowEvent};
#[cfg(test)]
pub(crate) use semantic::{
    conformance::{
        runtime_semantic_evaluate_schedule, RuntimeSemanticScheduleCalibration,
        RuntimeSemanticScheduleDiagnostic, RuntimeSemanticScheduleError,
    },
    runtime_semantic_lambda_step, RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000,
    RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000,
};
pub use semantic::{
    runtime_fuel_strategy_from_fuel_tune, runtime_semantic_calibration_from_fuel_tune,
    RuntimeSemanticAfrOverride, RuntimeSemanticAxis16, RuntimeSemanticCalibration,
    RuntimeSemanticCurve16U16, RuntimeSemanticCylinderArrayU16, RuntimeSemanticEngineMode,
    RuntimeSemanticFuelError, RuntimeSemanticFuelObservations, RuntimeSemanticInjectionAngleMode,
    RuntimeSemanticInputSnapshot, RuntimeSemanticPiIntegratorState, RuntimeSemanticState,
    RuntimeSemanticTable2dI16, RuntimeSemanticTable2dU16, RuntimeSemanticTable2dU32,
    RUNTIME_SEMANTIC_TABLE_LEN,
};

pub use ecu_calibration::{CalibrationSnapshot, PersistedCalibrationBlob};
#[cfg(test)]
use ecu_calibration::{FuelRuntimeTune, FUEL_RUNTIME_LOAD_BINS, FUEL_RUNTIME_RPM_BINS};
pub use ecu_control::{
    AccelerationConfig, AccelerationState, AfterStartConfig, AfterStartState, AllowedTorque,
    BaseFuelModel, DwellConfig, EnrichmentController, EnrichmentInputs, EnrichmentResult,
    FuelAfrOverride, FuelEngineMode, FuelInputSnapshot, FuelIntent, FuelLoadSource,
    FuelObservations, IgnitionInputs, IgnitionPlan, IgnitionPlanner, LambdaTrimConfig,
    LambdaTrimInputs, LambdaTrimPlanner, LambdaTrimResult, StartupConfig, TorqueArbiter,
    TorqueInputs, WarmupConfig,
};
pub use ecu_scheduler::{
    FuelOutputMode, FuelOutputProfile, SchedulerState, SparkOutputMode, SparkOutputProfile,
};

#[cfg(test)]
use ecu_domain::{AbsoluteTimeAuthority, CrankSyncState, PhaseSyncState};
#[cfg(test)]
use ecu_domain::{
    CancelReason, ControlMode, Degrees10, DwellUs, EnginePhase, EngineTimeAuthority, FaultCode,
    FaultSeverity, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm, SyncState,
};
#[cfg(test)]
use ecu_scheduler::{
    ExclusiveChannel, InjectionPlan, OutputGroup, TimedIgnitionPlan, TimedInjectionPlan,
};

pub const RUNTIME_AUX_COMMAND_CAP: usize = 16;
pub const RUNTIME_ACTION_CAP: usize = 20;

// ---------------------------------------------------------------------------
// Unit conversion helpers
// ---------------------------------------------------------------------------

/// Converts runtime torque percent-x100 to spec torque percent-x1000.
///
/// The spec uses x1000 units (e.g., 10000 = 100.00%) and runtime uses
/// x100 units (e.g., 10000 = 100.00%). This helper applies x10 scaling
/// only when the two values represent the same percent-style quantity.
///
/// # Saturation
///
/// Uses `saturating_mul(10)` so that `u16::MAX * 10` does not wrap.
/// The result saturates at `u16::MAX` (65535).
///
/// # When NOT to use
///
/// Do NOT use this for torque_allowed unless runtime arbiter semantics
/// have been confirmed as the same limiter stack as the spec oracle.
#[inline]
pub const fn runtime_x100_to_spec_x1000(runtime_x100: u16) -> u16 {
    runtime_x100.saturating_mul(10)
}

#[cfg(test)]
mod tests;
