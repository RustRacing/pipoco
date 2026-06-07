use ecu_board_api::{BoardCapabilities, PinMapId, RuntimeBuildId};

use super::validation::RecipeValidationError;

pub const MAX_CARGO_FEATURES: usize = 8;
pub const MAX_FIRST_RUN_CHECKLIST_ITEMS: usize = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoardId(pub &'static str);

impl BoardId {
    pub const fn new(value: &'static str) -> Self {
        Self(value)
    }

    pub const fn get(self) -> &'static str {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildArtifact {
    CargoBin {
        package: &'static str,
        binary: &'static str,
        target_triple: &'static str,
        required_features: &'static [&'static str],
    },
    BoardOnly {
        board_id: BoardId,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardBuildMetadata {
    pub board_id: BoardId,
    pub artifact: BuildArtifact,
    pub capabilities: BoardCapabilities,
    pub pin_map_id: PinMapId,
    pub runtime_build_id: RuntimeBuildId,
    pub feature_bindings: BoardFeatureBindings,
    pub default_ts_profile: Option<&'static str>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareBuildPlan {
    pub board_id: BoardId,
    pub artifact: BuildArtifact,
    pub cargo_features: CargoFeatureSet,
    pub pin_map_id: PinMapId,
    pub runtime_build_id: RuntimeBuildId,
    pub first_run: FirstRunChecklist,
    pub ts_profile: TunerStudioProfileSelection,
}

impl FirmwareBuildPlan {
    pub fn cargo_build_invocation(self) -> Result<FirmwareBuildInvocation, BuildInvocationError> {
        FirmwareBuildInvocation::from_plan(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirmwareBuildMode {
    Debug,
    Release,
}

impl FirmwareBuildMode {
    pub const fn default_for_recipe_builds() -> Self {
        Self::Release
    }
}

impl Default for FirmwareBuildMode {
    fn default() -> Self {
        Self::default_for_recipe_builds()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirmwareBuildInvocation {
    pub package: &'static str,
    pub binary: &'static str,
    pub target_triple: &'static str,
    /// Recipe-driven board builds default to release artifacts for flashing.
    pub mode: FirmwareBuildMode,
    pub features: CargoFeatureSet,
}

impl FirmwareBuildInvocation {
    pub fn from_plan(plan: FirmwareBuildPlan) -> Result<Self, BuildInvocationError> {
        match plan.artifact {
            BuildArtifact::CargoBin {
                package,
                binary,
                target_triple,
                required_features,
            } => {
                let mut features = plan.cargo_features;
                for feature in required_features {
                    features.push_for_invocation(feature)?;
                }

                Ok(Self {
                    package,
                    binary,
                    target_triple,
                    mode: FirmwareBuildMode::default_for_recipe_builds(),
                    features,
                })
            }
            BuildArtifact::BoardOnly { .. } => Err(BuildInvocationError::NoCargoBinaryArtifact),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildInvocationError {
    NoCargoBinaryArtifact,
    CargoFeatureCapacityExceeded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CargoFeatureSet {
    features: [&'static str; MAX_CARGO_FEATURES],
    len: u8,
}

impl CargoFeatureSet {
    pub const fn new() -> Self {
        Self {
            features: [""; MAX_CARGO_FEATURES],
            len: 0,
        }
    }

    pub const fn len(self) -> usize {
        self.len as usize
    }

    pub const fn is_empty(self) -> bool {
        self.len == 0
    }

    pub fn as_slice(&self) -> &[&'static str] {
        &self.features[..self.len()]
    }

    pub fn contains(&self, feature: &'static str) -> bool {
        self.as_slice().contains(&feature)
    }

    pub(super) fn push(&mut self, feature: &'static str) -> Result<(), RecipeValidationError> {
        if self.contains(feature) {
            return Ok(());
        }

        let index = self.len();
        if index >= MAX_CARGO_FEATURES {
            return Err(RecipeValidationError::CargoFeatureCapacityExceeded);
        }

        self.features[index] = feature;
        self.len += 1;
        Ok(())
    }

    pub(super) fn validate_extra_features(
        self,
        features: &'static [&'static str],
    ) -> Result<(), RecipeValidationError> {
        let mut merged = self;
        for feature in features {
            merged.push(feature)?;
        }
        Ok(())
    }

    fn push_for_invocation(&mut self, feature: &'static str) -> Result<(), BuildInvocationError> {
        if self.contains(feature) {
            return Ok(());
        }

        let index = self.len();
        if index >= MAX_CARGO_FEATURES {
            return Err(BuildInvocationError::CargoFeatureCapacityExceeded);
        }

        self.features[index] = feature;
        self.len += 1;
        Ok(())
    }
}

impl Default for CargoFeatureSet {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FirstRunChecklist {
    items: [Option<FirstRunChecklistItem>; MAX_FIRST_RUN_CHECKLIST_ITEMS],
    len: u8,
}

impl FirstRunChecklist {
    pub const fn new() -> Self {
        Self {
            items: [None; MAX_FIRST_RUN_CHECKLIST_ITEMS],
            len: 0,
        }
    }

    pub const fn len(self) -> usize {
        self.len as usize
    }

    pub const fn is_empty(self) -> bool {
        self.len == 0
    }

    pub fn as_slice(&self) -> &[Option<FirstRunChecklistItem>] {
        &self.items[..self.len()]
    }

    pub(super) fn push(
        &mut self,
        item: FirstRunChecklistItem,
    ) -> Result<(), RecipeValidationError> {
        let index = self.len();
        if index >= MAX_FIRST_RUN_CHECKLIST_ITEMS {
            return Err(RecipeValidationError::FirstRunChecklistCapacityExceeded);
        }

        self.items[index] = Some(item);
        self.len += 1;
        Ok(())
    }
}

impl Default for FirstRunChecklist {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FirstRunChecklistItem {
    VerifyPinMap(PinMapId),
    VerifyTriggerWiring,
    VerifyLoadSensor,
    VerifyWatchdog,
    VerifyCalibrationPersistence,
    VerifyTunerStudioProfile(TunerStudioProfileSelection),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecipeFeature {
    TriggerCapture,
    Ignition,
    Injection,
    FuelStrategy,
    RevLimiter,
    TunerStudio,
    Telemetry,
    Persistence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FeatureBinding {
    BuiltIn,
    CargoFeature(&'static str),
    Unsupported,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardFeatureBindings {
    pub trigger_capture: FeatureBinding,
    pub ignition: FeatureBinding,
    pub injection: FeatureBinding,
    pub fuel_strategy: FeatureBinding,
    pub rev_limiter: FeatureBinding,
    pub tuner_studio: FeatureBinding,
    pub telemetry: FeatureBinding,
    pub persistence: FeatureBinding,
}

impl BoardFeatureBindings {
    pub const fn all(binding: FeatureBinding) -> Self {
        Self {
            trigger_capture: binding,
            ignition: binding,
            injection: binding,
            fuel_strategy: binding,
            rev_limiter: binding,
            tuner_studio: binding,
            telemetry: binding,
            persistence: binding,
        }
    }

    pub const fn all_builtin() -> Self {
        Self::all(FeatureBinding::BuiltIn)
    }

    pub const fn binding_for(self, feature: RecipeFeature) -> FeatureBinding {
        match feature {
            RecipeFeature::TriggerCapture => self.trigger_capture,
            RecipeFeature::Ignition => self.ignition,
            RecipeFeature::Injection => self.injection,
            RecipeFeature::FuelStrategy => self.fuel_strategy,
            RecipeFeature::RevLimiter => self.rev_limiter,
            RecipeFeature::TunerStudio => self.tuner_studio,
            RecipeFeature::Telemetry => self.telemetry,
            RecipeFeature::Persistence => self.persistence,
        }
    }

    pub const fn with(mut self, feature: RecipeFeature, binding: FeatureBinding) -> Self {
        match feature {
            RecipeFeature::TriggerCapture => self.trigger_capture = binding,
            RecipeFeature::Ignition => self.ignition = binding,
            RecipeFeature::Injection => self.injection = binding,
            RecipeFeature::FuelStrategy => self.fuel_strategy = binding,
            RecipeFeature::RevLimiter => self.rev_limiter = binding,
            RecipeFeature::TunerStudio => self.tuner_studio = binding,
            RecipeFeature::Telemetry => self.telemetry = binding,
            RecipeFeature::Persistence => self.persistence = binding,
        }
        self
    }
}

impl Default for BoardFeatureBindings {
    fn default() -> Self {
        Self::all_builtin()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TunerStudioProfileSelection {
    None,
    RequiredButUnselected,
    Recipe(&'static str),
    BoardDefault(&'static str),
}

impl TunerStudioProfileSelection {
    pub const fn is_some(self) -> bool {
        !matches!(self, Self::None)
    }

    pub const fn profile_name(self) -> Option<&'static str> {
        match self {
            Self::None | Self::RequiredButUnselected => None,
            Self::Recipe(profile) | Self::BoardDefault(profile) => Some(profile),
        }
    }

    pub(super) const fn resolve(
        tuner_studio_enabled: bool,
        recipe_profile: Option<&'static str>,
        board_default: Option<&'static str>,
    ) -> Self {
        if !tuner_studio_enabled {
            return Self::None;
        }

        match recipe_profile {
            Some(profile) => Self::Recipe(profile),
            None => match board_default {
                Some(profile) => Self::BoardDefault(profile),
                None => {
                    if tuner_studio_enabled {
                        Self::RequiredButUnselected
                    } else {
                        Self::None
                    }
                }
            },
        }
    }
}
