mod fuel;
mod schedule;
mod torque;
mod types;

pub use fuel::runtime_semantic_evaluate_fuel;
#[cfg(test)]
pub(crate) use fuel::runtime_semantic_lambda_step;
pub use types::*;

/// Explicit conformance/test support surface.
///
/// These evaluators mirror frozen semantic-oracle behavior for proof and
/// differential tests. Product runtime execution should use `EngineRuntime`.
pub mod conformance {
    pub use super::schedule::{
        runtime_semantic_evaluate_schedule, runtime_semantic_evaluate_schedule_with_authority,
    };
    pub use super::torque::{
        runtime_semantic_evaluate_torque, RuntimeSemanticTorqueInput, RuntimeSemanticTorqueResult,
        TORQUE_SCALE_X1000_MAX,
    };
    pub use super::types::{
        RuntimeConformanceStatus, RuntimeSemanticScheduleCalibration,
        RuntimeSemanticScheduleDiagnostic, RuntimeSemanticScheduleError,
        RuntimeSemanticScheduleEvent, RuntimeSemanticScheduleEventBatch,
        RuntimeSemanticScheduleEventKind, RuntimeSemanticScheduleObservations,
    };
}
