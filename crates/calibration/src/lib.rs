#![cfg_attr(not(test), no_std)]

pub mod configs;
pub mod expert_trigger;
pub mod kv;
pub mod model;
pub mod runtime_tune;
pub mod sensors;

pub use configs::{
    AeConfig, AseConfig, ClConfig, DfcoConfig, EcuConfig, FanConfig, IdleConfig, LambdaConfig,
    LimiterStrategy, LoadFailureConfig, LtftConfig, O2SensorType, PlausibilityConfig, RateConfig,
    RevLimiterConfig, SensorsLimits, WueConfig,
};

pub use expert_trigger::{
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertTriggerRecordError,
    ExpertTriggerRuntimeAuthorityError, ExpertTriggerValidationError, ExpertUnlock,
    FixedTimingMode, PollLevelPolarity, PrimaryTriggerSpeed, SecondaryTriggerMode,
    TriggerAuthority, TriggerEdge, TriggerFilter, TriggerPattern, EXPERT_TRIGGER_RECORD_LEN,
};
pub use kv::{KvError, KvStore, PersistError};
pub use model::{
    ActiveCalibration, Calibration, CalibrationClass, CalibrationDiff, CalibrationHardwareTargetId,
    CalibrationPackageChecksum, CalibrationPackageCompareResult, CalibrationPackageCompatibility,
    CalibrationPackageIdentity, CalibrationPackageMigrationResult,
    CalibrationPackageMigrationStatus, CalibrationPackageReview, CalibrationPackageTargetIdentity,
    CalibrationPackageWireError, CalibrationRevision, CalibrationRuntimeBuildId,
    CalibrationSchemaVersion, CalibrationSnapshot, CommitRules, CommitVerdict,
    PersistedCalibrationBlob, PersistedCalibrationPackage, PersistedCalibrationStore,
    StagedCalibration,
};
pub use runtime_tune::{
    FuelRuntimeTable16, FuelRuntimeTune, FUEL_RUNTIME_LOAD_BINS, FUEL_RUNTIME_RPM_BINS,
    FUEL_RUNTIME_TABLE_DIM,
};
