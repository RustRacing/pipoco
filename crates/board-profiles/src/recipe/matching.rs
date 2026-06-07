use super::build::{
    BoardBuildMetadata, BoardId, CargoFeatureSet, FeatureBinding, FirmwareBuildPlan,
    FirstRunChecklist, FirstRunChecklistItem, RecipeFeature, TunerStudioProfileSelection,
};
use super::model::BoardSelection;
use super::validation::RecipeValidationError;
use super::FirmwareRecipe;

impl FirmwareRecipe {
    pub fn resolve_build_plan(
        self,
        board: BoardBuildMetadata,
    ) -> Result<FirmwareBuildPlan, RecipeValidationError> {
        if let BoardSelection::Named(requested) = self.board {
            let requested = BoardId::new(requested);
            if requested != board.board_id {
                return Err(RecipeValidationError::BoardSelectionMismatch {
                    requested,
                    actual: board.board_id,
                });
            }
        }

        self.validate_for(board.capabilities)?;

        let mut cargo_features = CargoFeatureSet::new();
        add_enabled_feature(
            self.subsystems.trigger_capture,
            RecipeFeature::TriggerCapture,
            board
                .feature_bindings
                .binding_for(RecipeFeature::TriggerCapture),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.ignition,
            RecipeFeature::Ignition,
            board.feature_bindings.binding_for(RecipeFeature::Ignition),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.injection,
            RecipeFeature::Injection,
            board.feature_bindings.binding_for(RecipeFeature::Injection),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.fuel_strategy,
            RecipeFeature::FuelStrategy,
            board
                .feature_bindings
                .binding_for(RecipeFeature::FuelStrategy),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.rev_limiter,
            RecipeFeature::RevLimiter,
            board
                .feature_bindings
                .binding_for(RecipeFeature::RevLimiter),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.tuner_studio,
            RecipeFeature::TunerStudio,
            board
                .feature_bindings
                .binding_for(RecipeFeature::TunerStudio),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.telemetry,
            RecipeFeature::Telemetry,
            board.feature_bindings.binding_for(RecipeFeature::Telemetry),
            &mut cargo_features,
        )?;
        add_enabled_feature(
            self.subsystems.persistence,
            RecipeFeature::Persistence,
            board
                .feature_bindings
                .binding_for(RecipeFeature::Persistence),
            &mut cargo_features,
        )?;
        if let super::build::BuildArtifact::CargoBin {
            required_features, ..
        } = board.artifact
        {
            cargo_features.validate_extra_features(required_features)?;
        }

        let ts_profile = TunerStudioProfileSelection::resolve(
            self.subsystems.tuner_studio,
            self.ts_profile,
            board.default_ts_profile,
        );
        let mut first_run = FirstRunChecklist::new();
        first_run.push(FirstRunChecklistItem::VerifyPinMap(board.pin_map_id))?;
        if self.inputs.trigger {
            first_run.push(FirstRunChecklistItem::VerifyTriggerWiring)?;
        }
        if self.inputs.load_sensor {
            first_run.push(FirstRunChecklistItem::VerifyLoadSensor)?;
        }
        if self.safety.watchdog_required {
            first_run.push(FirstRunChecklistItem::VerifyWatchdog)?;
        }
        if self.subsystems.persistence {
            first_run.push(FirstRunChecklistItem::VerifyCalibrationPersistence)?;
        }
        if self.subsystems.tuner_studio || ts_profile.is_some() {
            first_run.push(FirstRunChecklistItem::VerifyTunerStudioProfile(ts_profile))?;
        }

        Ok(FirmwareBuildPlan {
            board_id: board.board_id,
            artifact: board.artifact,
            cargo_features,
            pin_map_id: board.pin_map_id,
            runtime_build_id: board.runtime_build_id,
            first_run,
            ts_profile,
        })
    }
}

fn add_enabled_feature(
    enabled: bool,
    feature: RecipeFeature,
    binding: FeatureBinding,
    cargo_features: &mut CargoFeatureSet,
) -> Result<(), RecipeValidationError> {
    if !enabled {
        return Ok(());
    }

    match binding {
        FeatureBinding::BuiltIn => Ok(()),
        FeatureBinding::CargoFeature(name) => cargo_features.push(name),
        FeatureBinding::Unsupported => {
            Err(RecipeValidationError::UnsupportedFeatureBinding { feature })
        }
    }
}
