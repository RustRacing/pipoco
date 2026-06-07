#![cfg_attr(not(test), no_std)]

mod compat;
mod model;
pub mod profiles {
    pub mod m50b25tu;
}
pub mod recipe;

pub use compat::{
    check_profile_board_compatibility, conservative_first_start_preset,
    BoardFirstStartCapabilities, BoardFirstStartOutputCapabilities,
    BoardFirstStartSafetyCapabilities, BoardFirstStartSensorCapabilities, FirstStartLoadSource,
    FirstStartPreset, ProfileCompatibilityIssue, ProfileCompatibilityReport,
};
pub use ecu_trigger::{
    EngineTimeLatency, PollLevelPolarity, ResyncPolicy, SecondaryTriggerMode,
    SecondaryTriggerProfile, StartupSyncPolicy, TriggerAngleAuthority, TriggerEdge, TriggerFilter,
    TriggerPattern, TriggerProfile, TriggerSpeed,
};
pub use model::{
    AuxOutputRole, AuxProfile, BaroSensorProfile, BaroSourceRole, CamPhaseEdgeAction, CamProfile,
    CamSensorDefault, EngineBoardProfile, EngineProfile, HardwareMapBinding, HardwareMapOrigin,
    HardwareMapProfile, HardwareMapProvenance, HardwareMappingStyle, IgnitionProfile,
    IgnitionTopology, InjectionProfile, MapSensorModel, MapSensorProfile, MapSensorRole,
    SafetyProfile, SensorInventoryEntry, SensorInventoryProfile, SensorInventoryRole,
    SensorPresence, SensorScaling, SensorScalingProfile, SensorSupport,
};
pub use recipe::{
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BoardSelection, BuildArtifact,
    BuildInvocationError, CargoFeatureSet, FeatureBinding, FirmwareBuildInvocation,
    FirmwareBuildMode, FirmwareBuildPlan, FirmwareRecipe, FirstRunChecklist, FirstRunChecklistItem,
    InputSourceSet, OutputTopology, RecipeFeature, RecipeValidationError, RuntimeProfile,
    SafetyPolicy, SubsystemSet, TunerStudioProfileSelection,
};

#[cfg(test)]
mod tests;
