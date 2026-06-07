#![cfg_attr(not(test), no_std)]

use ecu_board_api::{BoardCapabilities, PinMapId, RuntimeBuildId};
use ecu_board_profiles::{
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BuildArtifact, FeatureBinding,
};

pub mod pinmap;

pub const TARGET_TRIPLE: &str = "thumbv8m.main-none-eabihf";
pub const BOARD_ID_RP2350B: BoardId = BoardId::new("rp2350b");
pub const PIN_MAP_RP2350B_REV_LIMITER_BRINGUP: PinMapId = PinMapId::new(0x2350);
pub const RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP: RuntimeBuildId = RuntimeBuildId::new(1);

/// Conservative recipe-level capabilities for the documented RP2350B rev-limiter bring-up build.
///
/// This metadata intentionally stays below full ECU, TunerStudio, persistence,
/// telemetry, and watchdog claims. It only advertises the smallest bringup
/// slice needed for a recipe to resolve to the real `ecu-rp2350b-min` binary.
pub const RP2350B_REV_LIMITER_BRINGUP_CAPABILITIES: BoardCapabilities =
    BoardCapabilities::new(false, true, false, 1, 0, 0, false, false, false);

pub const RP2350B_REV_LIMITER_BRINGUP_FEATURE_BINDINGS: BoardFeatureBindings =
    BoardFeatureBindings {
        trigger_capture: FeatureBinding::Unsupported,
        ignition: FeatureBinding::BuiltIn,
        injection: FeatureBinding::Unsupported,
        fuel_strategy: FeatureBinding::Unsupported,
        rev_limiter: FeatureBinding::BuiltIn,
        tuner_studio: FeatureBinding::Unsupported,
        telemetry: FeatureBinding::Unsupported,
        persistence: FeatureBinding::Unsupported,
    };

pub const RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA: BoardBuildMetadata = BoardBuildMetadata {
    board_id: BOARD_ID_RP2350B,
    artifact: BuildArtifact::CargoBin {
        package: "ecu-rp2350b",
        binary: "ecu-rp2350b-min",
        target_triple: TARGET_TRIPLE,
        required_features: &["example-bins"],
    },
    capabilities: RP2350B_REV_LIMITER_BRINGUP_CAPABILITIES,
    pin_map_id: PIN_MAP_RP2350B_REV_LIMITER_BRINGUP,
    runtime_build_id: RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP,
    feature_bindings: RP2350B_REV_LIMITER_BRINGUP_FEATURE_BINDINGS,
    default_ts_profile: None,
};

pub const RUNTIME_BUILD_ID_RP2350B_DEMO: RuntimeBuildId =
    RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP;
pub const RP2350B_DEMO_CAPABILITIES: BoardCapabilities = RP2350B_REV_LIMITER_BRINGUP_CAPABILITIES;
pub const RP2350B_DEMO_FEATURE_BINDINGS: BoardFeatureBindings =
    RP2350B_REV_LIMITER_BRINGUP_FEATURE_BINDINGS;
pub const RP2350B_DEMO_BUILD_METADATA: BoardBuildMetadata =
    RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA;

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_board_profiles::{
        BoardSelection, FirmwareBuildMode, FirmwareRecipe, FirstRunChecklistItem,
        RecipeValidationError,
    };

    #[test]
    fn build_metadata_uses_real_rev_limiter_cargo_bin_artifact() {
        let metadata = RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA;

        assert_eq!(metadata.board_id, BOARD_ID_RP2350B);
        assert_eq!(
            metadata.artifact,
            BuildArtifact::CargoBin {
                package: "ecu-rp2350b",
                binary: "ecu-rp2350b-min",
                target_triple: "thumbv8m.main-none-eabihf",
                required_features: &["example-bins"],
            }
        );
        assert_eq!(
            metadata.capabilities,
            RP2350B_REV_LIMITER_BRINGUP_CAPABILITIES
        );
        assert_eq!(metadata.pin_map_id, PIN_MAP_RP2350B_REV_LIMITER_BRINGUP);
        assert_eq!(
            metadata.runtime_build_id,
            RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP
        );
        assert_eq!(
            metadata.feature_bindings,
            RP2350B_REV_LIMITER_BRINGUP_FEATURE_BINDINGS
        );
        assert_eq!(metadata.default_ts_profile, None);
    }

    #[test]
    fn rev_limiter_bringup_recipe_resolves_against_rp2350b_metadata() {
        let mut recipe =
            FirmwareRecipe::rev_limiter().for_board(BoardSelection::Named(BOARD_ID_RP2350B.get()));
        recipe.safety.watchdog_required = false;

        let plan = recipe
            .resolve_build_plan(RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA)
            .unwrap();

        assert_eq!(plan.board_id, BOARD_ID_RP2350B);
        assert_eq!(
            plan.artifact,
            RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA.artifact
        );
        assert!(plan.cargo_features.is_empty());
        assert_eq!(plan.pin_map_id, PIN_MAP_RP2350B_REV_LIMITER_BRINGUP);
        assert_eq!(
            plan.runtime_build_id,
            RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP
        );
        assert_eq!(
            plan.first_run.as_slice(),
            &[Some(FirstRunChecklistItem::VerifyPinMap(
                PIN_MAP_RP2350B_REV_LIMITER_BRINGUP,
            ))]
        );
    }

    #[test]
    fn cargo_build_invocation_preserves_rp2350b_package_bin_target_mode_and_features() {
        let mut recipe = FirmwareRecipe::rev_limiter();
        recipe.safety.watchdog_required = false;

        let invocation = recipe
            .resolve_build_plan(RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA)
            .unwrap()
            .cargo_build_invocation()
            .unwrap();

        assert_eq!(invocation.package, "ecu-rp2350b");
        assert_eq!(invocation.binary, "ecu-rp2350b-min");
        assert_eq!(invocation.target_triple, "thumbv8m.main-none-eabihf");
        assert_eq!(invocation.mode, FirmwareBuildMode::Release);
        assert_eq!(invocation.features.as_slice(), &["example-bins"]);
    }

    #[test]
    fn cargo_build_invocation_deduplicates_rp2350b_required_artifact_features() {
        let mut metadata = RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA;
        metadata.artifact = BuildArtifact::CargoBin {
            package: "ecu-rp2350b",
            binary: "ecu-rp2350b-min",
            target_triple: TARGET_TRIPLE,
            required_features: &["example-bins", "example-bins"],
        };
        let mut recipe = FirmwareRecipe::rev_limiter();
        recipe.safety.watchdog_required = false;

        let invocation = recipe
            .resolve_build_plan(metadata)
            .unwrap()
            .cargo_build_invocation()
            .unwrap();

        assert_eq!(invocation.features.as_slice(), &["example-bins"]);
    }

    #[test]
    fn named_board_mismatch_rejects_rp2350b_metadata() {
        let mut recipe =
            FirmwareRecipe::rev_limiter().for_board(BoardSelection::Named("other-board"));
        recipe.safety.watchdog_required = false;

        assert_eq!(
            recipe.resolve_build_plan(RP2350B_REV_LIMITER_BRINGUP_BUILD_METADATA),
            Err(RecipeValidationError::BoardSelectionMismatch {
                requested: BoardId::new("other-board"),
                actual: BOARD_ID_RP2350B,
            })
        );
    }
}
