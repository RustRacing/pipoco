#![no_std]
#![forbid(unsafe_code)]

pub const ENGINE_CYCLE_DEGREES10: i16 = 7200;

/// Single source of truth for the `trigger.tla` `MaxGood` constant.
///
/// `MaxGood` bounds the consecutive-good-tooth counter the formal model uses to
/// gate sync acquisition. The reference 60-2 profile mirrors this as
/// `StartupSyncPolicy.skip_revolutions`. The model `.tla`/`.cfg` value and this
/// constant are kept in lockstep by the drift check in
/// `tools/verify_formal.sh`.
pub const MODEL_MAX_GOOD: u8 = 2;

/// Single source of truth for the `trigger.tla` `MaxBad` constant.
///
/// `MaxBad` bounds the consecutive-bad-tooth counter the formal model uses to
/// gate sync loss. The model `.tla`/`.cfg` value and this constant are kept in
/// lockstep by the drift check in `tools/verify_formal.sh`.
pub const MODEL_MAX_BAD: u8 = 2;

mod diag;
mod edge_batch;
mod math;
mod missing_tooth;
mod profile;
mod profiled;

pub use diag::{DecoderObservation, SyncLossReason, TriggerDiagnostics, TriggerValidationError};
pub use edge_batch::{PrimaryEdgeBatch, PrimaryEdgeBatchError, PrimaryEdgeSample};
pub use math::normalize_engine_cycle_deg10;
pub use missing_tooth::{
    MissingToothDecoder, MissingToothDecoderConfig, MissingToothDecoderEvent, PrimaryEdgeIngestion,
    DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
};
pub use profile::{
    EngineTimeLatency, PatternId, PollLevelPolarity, ResyncPolicy, RuntimeMissingToothProfile,
    RuntimeSecondaryTriggerMode, RuntimeSecondaryTriggerProfile, SecondaryTriggerMode,
    SecondaryTriggerProfile, StartupSyncPolicy, TriggerAngleAuthority, TriggerEdge, TriggerFilter,
    TriggerLevel, TriggerPattern, TriggerProfile, TriggerSpeed,
};
pub use profiled::ProfiledMissingToothDecoder;

#[cfg(test)]
mod tests;
