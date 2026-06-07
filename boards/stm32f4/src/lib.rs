#![cfg_attr(not(test), no_std)]

use ecu_board_api::{BoardCapabilities, PinMapId, RuntimeBuildId};
use ecu_board_profiles::{
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BuildArtifact, FeatureBinding,
};

#[cfg(test)]
mod hal_impl;
#[cfg(test)]
#[allow(dead_code)]
mod ts_support;

pub const TARGET_TRIPLE: &str = "thumbv7em-none-eabihf";
pub const BOARD_ID_STM32F4: BoardId = BoardId::new("stm32f4");
pub const PIN_MAP_STM32F4_IGNITION_BRINGUP: PinMapId = PinMapId::new(0xF405);
pub const RUNTIME_BUILD_ID_STM32F4_ECU: RuntimeBuildId = RuntimeBuildId::new(1);

/// Conservative recipe-level capabilities for the documented `stm32f4-ecu` binary.
///
/// The STM32F4 target has broader code paths behind features, but this metadata
/// only claims the ignition-only bringup slice backed by TIM2 trigger capture.
pub const STM32F4_IGNITION_BRINGUP_CAPABILITIES: BoardCapabilities =
    BoardCapabilities::new(true, true, false, 2, 0, 0, false, false, false);

pub const STM32F4_IGNITION_BRINGUP_FEATURE_BINDINGS: BoardFeatureBindings = BoardFeatureBindings {
    trigger_capture: FeatureBinding::CargoFeature("capture-tim"),
    ignition: FeatureBinding::BuiltIn,
    injection: FeatureBinding::Unsupported,
    fuel_strategy: FeatureBinding::Unsupported,
    rev_limiter: FeatureBinding::BuiltIn,
    tuner_studio: FeatureBinding::Unsupported,
    telemetry: FeatureBinding::Unsupported,
    persistence: FeatureBinding::Unsupported,
};

pub const STM32F4_IGNITION_BRINGUP_BUILD_METADATA: BoardBuildMetadata = BoardBuildMetadata {
    board_id: BOARD_ID_STM32F4,
    artifact: BuildArtifact::CargoBin {
        package: "stm32f4-ecu",
        binary: "stm32f4-ecu",
        target_triple: TARGET_TRIPLE,
        required_features: &["capture-tim"],
    },
    capabilities: STM32F4_IGNITION_BRINGUP_CAPABILITIES,
    pin_map_id: PIN_MAP_STM32F4_IGNITION_BRINGUP,
    runtime_build_id: RUNTIME_BUILD_ID_STM32F4_ECU,
    feature_bindings: STM32F4_IGNITION_BRINGUP_FEATURE_BINDINGS,
    default_ts_profile: None,
};

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_board_profiles::{
        BoardSelection, FirmwareBuildMode, FirmwareRecipe, FirstRunChecklistItem,
        RecipeValidationError,
    };

    #[test]
    fn build_metadata_uses_real_stm32f4_cargo_bin_artifact() {
        let metadata = STM32F4_IGNITION_BRINGUP_BUILD_METADATA;

        assert_eq!(metadata.board_id, BOARD_ID_STM32F4);
        assert_eq!(
            metadata.artifact,
            BuildArtifact::CargoBin {
                package: "stm32f4-ecu",
                binary: "stm32f4-ecu",
                target_triple: "thumbv7em-none-eabihf",
                required_features: &["capture-tim"],
            }
        );
        assert_eq!(metadata.capabilities, STM32F4_IGNITION_BRINGUP_CAPABILITIES);
        assert_eq!(metadata.pin_map_id, PIN_MAP_STM32F4_IGNITION_BRINGUP);
        assert_eq!(metadata.runtime_build_id, RUNTIME_BUILD_ID_STM32F4_ECU);
        assert_eq!(
            metadata.feature_bindings,
            STM32F4_IGNITION_BRINGUP_FEATURE_BINDINGS
        );
        assert_eq!(metadata.default_ts_profile, None);
    }

    #[test]
    fn ignition_bringup_recipe_resolves_against_stm32f4_metadata() {
        let mut recipe = FirmwareRecipe::ignition_only_wasted_spark(4)
            .for_board(BoardSelection::Named(BOARD_ID_STM32F4.get()));
        recipe.safety.watchdog_required = false;

        let plan = recipe
            .resolve_build_plan(STM32F4_IGNITION_BRINGUP_BUILD_METADATA)
            .unwrap();

        assert_eq!(plan.board_id, BOARD_ID_STM32F4);
        assert_eq!(
            plan.artifact,
            STM32F4_IGNITION_BRINGUP_BUILD_METADATA.artifact
        );
        assert_eq!(plan.cargo_features.as_slice(), &["capture-tim"]);
        assert_eq!(plan.pin_map_id, PIN_MAP_STM32F4_IGNITION_BRINGUP);
        assert_eq!(plan.runtime_build_id, RUNTIME_BUILD_ID_STM32F4_ECU);
        assert_eq!(
            plan.first_run.as_slice(),
            &[
                Some(FirstRunChecklistItem::VerifyPinMap(
                    PIN_MAP_STM32F4_IGNITION_BRINGUP,
                )),
                Some(FirstRunChecklistItem::VerifyTriggerWiring),
            ]
        );
    }

    #[test]
    fn cargo_build_invocation_preserves_stm32f4_package_bin_target_mode_and_features() {
        let mut recipe = FirmwareRecipe::ignition_only_wasted_spark(4);
        recipe.safety.watchdog_required = false;

        let invocation = recipe
            .resolve_build_plan(STM32F4_IGNITION_BRINGUP_BUILD_METADATA)
            .unwrap()
            .cargo_build_invocation()
            .unwrap();

        assert_eq!(invocation.package, "stm32f4-ecu");
        assert_eq!(invocation.binary, "stm32f4-ecu");
        assert_eq!(invocation.target_triple, "thumbv7em-none-eabihf");
        assert_eq!(invocation.mode, FirmwareBuildMode::Release);
        assert_eq!(invocation.features.as_slice(), &["capture-tim"]);
    }

    #[test]
    fn named_board_mismatch_rejects_stm32f4_metadata() {
        let mut recipe =
            FirmwareRecipe::ignition_only_wasted_spark(4).for_board(BoardSelection::Named("other"));
        recipe.safety.watchdog_required = false;

        assert_eq!(
            recipe.resolve_build_plan(STM32F4_IGNITION_BRINGUP_BUILD_METADATA),
            Err(RecipeValidationError::BoardSelectionMismatch {
                requested: BoardId::new("other"),
                actual: BOARD_ID_STM32F4,
            })
        );
    }
}
