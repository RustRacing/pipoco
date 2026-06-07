use ecu_board_api::{AuxCommand, EcuOutput, PinMapId, RuntimeBuildId, RuntimeOutputProfile};
use ecu_board_profiles::{
    BoardId, BuildArtifact, FirstRunChecklist, FirstRunChecklistItem, RecipeValidationError,
    TunerStudioProfileSelection,
};
use ecu_runtime::ActionLoweringError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Atmega2560BuildPlanError {
    BoardId {
        expected: BoardId,
        actual: BoardId,
    },
    Artifact {
        expected: BuildArtifact,
        actual: BuildArtifact,
    },
    PinMap {
        expected: PinMapId,
        actual: PinMapId,
    },
    RuntimeBuild {
        expected: RuntimeBuildId,
        actual: RuntimeBuildId,
    },
    CargoFeaturesUnsupported,
    TunerStudioProfileUnsupported {
        actual: TunerStudioProfileSelection,
    },
    FirstRunChecklist {
        expected: &'static [Option<FirstRunChecklistItem>],
        actual: FirstRunChecklist,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Atmega2560RecipePrepareError {
    Recipe(RecipeValidationError),
    UnsupportedRuntimeProfile {
        output_profile: RuntimeOutputProfile,
    },
    BuildPlan(Atmega2560BuildPlanError),
}

impl From<RecipeValidationError> for Atmega2560RecipePrepareError {
    fn from(error: RecipeValidationError) -> Self {
        Self::Recipe(error)
    }
}

impl From<Atmega2560BuildPlanError> for Atmega2560RecipePrepareError {
    fn from(error: Atmega2560BuildPlanError) -> Self {
        Self::BuildPlan(error)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Atmega2560BridgeError {
    OutputBatchFull,
    AuxBatchFull,
    UnsupportedAuxCommand(AuxCommand),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Atmega2560PreparedStepError {
    Bridge(Atmega2560BridgeError),
    UnmappedOutput(EcuOutput),
}

impl From<Atmega2560BridgeError> for Atmega2560PreparedStepError {
    fn from(error: Atmega2560BridgeError) -> Self {
        Self::Bridge(error)
    }
}

impl From<ActionLoweringError> for Atmega2560BridgeError {
    fn from(error: ActionLoweringError) -> Self {
        match error {
            ActionLoweringError::OutputBatchFull | ActionLoweringError::TimingIslandBatchFull => {
                Self::OutputBatchFull
            }
            ActionLoweringError::AuxBatchFull => Self::AuxBatchFull,
        }
    }
}
