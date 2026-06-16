//! Logical ECU board capability contract.
//!
//! `ecu-board-api` is the canonical boundary for what a board can provide to
//! ECU recipes and runtime-independent board code: clocks, decoded trigger
//! edges, logical sensor snapshots, scheduled ECU outputs, aux commands,
//! telemetry, watchdog service, and board capability metadata.
//!
//! Keep this crate above raw electrical details and below runtime policy. It
//! may depend on `ecu-domain`, but it must not depend on `ecu-runtime` or own
//! action lowering. Raw pins, capture buffers, trace records, and electrical
//! samples belong in `ecu-io`.

#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod capabilities;
mod output_profiles;
mod safety;
mod sensors;
mod telemetry;
mod timing_island;
mod traits;
pub mod wire;

pub mod legacy {
    //! Compatibility helpers for migration-era callers.
    //!
    //! New board/runtime code should use the canonical authority-aware timing
    //! and output profile APIs from the crate root instead of these shims.

    pub use crate::capabilities::CalibrationPage;
    pub use crate::output_profiles::legacy::single_channel_runtime_output_profile;
    pub use crate::timing_island::legacy::sync_state_authority;
    pub use crate::traits::CalibrationStore;
}

pub mod frontier {
    pub use crate::safety::SafetyPermitMask;
    pub use crate::timing_island::{
        TimingIslandAdmissionReport, TimingIslandHorizonSequenceId, TimingIslandMetricSnapshot,
        TimingIslandPermitMask, TimingIslandStopReason, TimingIslandSyncLossReason,
        HEARTBEAT_EXPIRY_US, HORIZON_SEQUENCE_BITS, MAX_HORIZON_US,
    };
}

pub use capabilities::{
    BoardCapabilities, BoardResourceLimits, IgnitionProfileId, LoadSourceCapabilities, PinMapId,
    ProfileId, RuntimeBuildId,
};
pub use output_profiles::{
    AuxSafetyProfile, FuelOutputMode, FuelOutputProfile, FullEcuOutputProfile,
    IgnitionOutputProfile, InjectionOutputProfile, OutputAuthorityRequirement,
    RuntimeOutputProfile, SparkOutputMode, SparkOutputProfile, FULL_ECU_MAX_CYLINDERS,
    FULL_ECU_MAX_LIMP_AUX_OUTPUTS,
};
pub use safety::{
    SafetyDriverFaultMask, SafetyGateInput, SafetyGatePermit, SafetyGateReason, SafetyGateStatus,
    SafetyPermitMask,
};
pub use sensors::{
    BoardSensorSnapshot, BoardSensorSnapshotCapture, BoardSensorSnapshotCaptureSource,
    BoardSensorValidityFlags, CaptureSample, CaptureSampleSource, CaptureSink, SensorSnapshot,
};
pub use telemetry::{
    CommonActionTelemetry, CommonCamEdgeTelemetry, CommonControlReasonTelemetry,
    CommonControlTelemetry, CommonDecisionTelemetry, CommonDiagnosticsTelemetry,
    CommonEngineTelemetry, CommonEnrichmentTelemetry, CommonFaultTransitionTelemetry,
    CommonFrontierTelemetry, CommonFuelObservationTelemetry, CommonFuelStrategyMode,
    CommonIgnitionLimitReason, CommonLambdaMode, CommonPendingInputTelemetry, CommonSchedulerMode,
    CommonSchedulerOwnershipTelemetry, CommonSchedulerReservationTelemetry,
    CommonSchedulerStateSummaryTelemetry, CommonSchedulerWindowTelemetry,
    CommonShiftArmingTelemetry, CommonSyncTelemetryState, CommonTorqueLimitReason,
    CommonTorqueTelemetry, CommonTriggerEdgeTelemetry, CommonValidatedInputTelemetry,
    EngineTimeAuthorityTelemetry, IgnitionProfileMode, TelemetryFrame,
};
pub use timing_island::{
    engine_time_authorizes_full_sequential, AuxCommand, AuxCommandBatch, AuxOutput, AuxValue,
    EcuOutput, EdgeBatch, EdgeKind, OutputLevel, OutputTransition, OutputTransitionBatch,
    TimingIslandAdmissionReport, TimingIslandCommand, TimingIslandCommandBatch, TimingIslandEvent,
    TimingIslandFaultStatus, TimingIslandHorizonSequenceId, TimingIslandMetricSnapshot,
    TimingIslandPermitMask, TimingIslandRejectReason, TimingIslandStopReason,
    TimingIslandSyncLossReason, TriggerEdge, HEARTBEAT_EXPIRY_US, HORIZON_SEQUENCE_BITS,
    MAX_HORIZON_US,
};
pub use traits::{
    AuxOutputSink, EcuClock, OutputScheduler, SensorSource, TelemetrySink, TriggerEdgeSource,
    Watchdog,
};

#[cfg(test)]
mod tests;
