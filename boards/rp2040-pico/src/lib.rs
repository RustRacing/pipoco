#![cfg_attr(not(test), no_std)]

use ecu_board_api::{BoardCapabilities, PinMapId, RuntimeBuildId};
use ecu_board_profiles::{
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BuildArtifact, FeatureBinding,
};

pub const TARGET_TRIPLE: &str = "thumbv6m-none-eabi";
pub const BOARD_ID_RP2040_PICO: BoardId = BoardId::new("rp2040-pico");
pub const PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP: PinMapId = PinMapId::new(0x2040);
pub const RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU: RuntimeBuildId = RuntimeBuildId::new(1);
pub const RP2040_PICO_TS_ECU_DEFAULT_TS_PROFILE: &str = "crates/compat/tests/assets/IPW-ECU.ini";

/// Conservative recipe-level capabilities for the `ts-ecu` Pico bringup binary.
///
/// The board crate documents the binary artifact and one ignition-capable
/// bringup channel, but does not yet claim injector, full ECU, telemetry, or
/// watchdog service support at this metadata boundary.
pub const RP2040_PICO_TS_ECU_CAPABILITIES: BoardCapabilities =
    BoardCapabilities::new(true, true, false, 1, 0, 0, false, true, false);

pub const RP2040_PICO_TS_ECU_FEATURE_BINDINGS: BoardFeatureBindings = BoardFeatureBindings {
    trigger_capture: FeatureBinding::CargoFeature("capture-pio"),
    ignition: FeatureBinding::BuiltIn,
    injection: FeatureBinding::Unsupported,
    fuel_strategy: FeatureBinding::Unsupported,
    rev_limiter: FeatureBinding::BuiltIn,
    tuner_studio: FeatureBinding::BuiltIn,
    telemetry: FeatureBinding::Unsupported,
    persistence: FeatureBinding::CargoFeature("flash-kv"),
};

pub const RP2040_PICO_TS_ECU_BUILD_METADATA: BoardBuildMetadata = BoardBuildMetadata {
    board_id: BOARD_ID_RP2040_PICO,
    artifact: BuildArtifact::CargoBin {
        package: "ecu-rp2040-pico",
        binary: "ts-ecu",
        target_triple: TARGET_TRIPLE,
        required_features: &["capture-pio"],
    },
    capabilities: RP2040_PICO_TS_ECU_CAPABILITIES,
    pin_map_id: PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP,
    runtime_build_id: RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU,
    feature_bindings: RP2040_PICO_TS_ECU_FEATURE_BINDINGS,
    default_ts_profile: Some(RP2040_PICO_TS_ECU_DEFAULT_TS_PROFILE),
};

#[cfg(test)]
#[path = "ts_usb_cdc.rs"]
mod ts_usb_cdc;

#[cfg(all(test, feature = "flash-kv", not(target_arch = "arm")))]
#[path = "seq_kv.rs"]
mod seq_kv;

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_board_profiles::{
        BoardSelection, FirmwareBuildMode, FirmwareRecipe, FirstRunChecklistItem,
        RecipeValidationError, TunerStudioProfileSelection,
    };

    #[test]
    fn build_metadata_uses_real_ts_ecu_cargo_bin_artifact() {
        let metadata = RP2040_PICO_TS_ECU_BUILD_METADATA;

        assert_eq!(metadata.board_id, BOARD_ID_RP2040_PICO);
        assert_eq!(
            metadata.artifact,
            BuildArtifact::CargoBin {
                package: "ecu-rp2040-pico",
                binary: "ts-ecu",
                target_triple: "thumbv6m-none-eabi",
                required_features: &["capture-pio"],
            }
        );
        assert_eq!(metadata.capabilities, RP2040_PICO_TS_ECU_CAPABILITIES);
        assert_eq!(metadata.pin_map_id, PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP);
        assert_eq!(
            metadata.runtime_build_id,
            RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU
        );
        assert_eq!(
            metadata.feature_bindings,
            RP2040_PICO_TS_ECU_FEATURE_BINDINGS
        );
        assert_eq!(
            metadata.default_ts_profile,
            Some(RP2040_PICO_TS_ECU_DEFAULT_TS_PROFILE)
        );
    }

    #[test]
    fn ignition_bringup_recipe_resolves_against_rp2040_pico_metadata() {
        let mut recipe = FirmwareRecipe::ignition_only_wasted_spark(2)
            .for_board(BoardSelection::Named(BOARD_ID_RP2040_PICO.get()));
        recipe.safety.watchdog_required = false;

        let plan = recipe
            .resolve_build_plan(RP2040_PICO_TS_ECU_BUILD_METADATA)
            .unwrap();

        assert_eq!(plan.board_id, BOARD_ID_RP2040_PICO);
        assert_eq!(plan.artifact, RP2040_PICO_TS_ECU_BUILD_METADATA.artifact);
        assert_eq!(plan.cargo_features.as_slice(), &["capture-pio"]);
        assert_eq!(plan.pin_map_id, PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP);
        assert_eq!(plan.runtime_build_id, RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU);
        assert_eq!(
            plan.first_run.as_slice(),
            &[
                Some(FirstRunChecklistItem::VerifyPinMap(
                    PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP,
                )),
                Some(FirstRunChecklistItem::VerifyTriggerWiring),
            ]
        );
        assert_eq!(plan.ts_profile, TunerStudioProfileSelection::None);
    }

    #[test]
    fn cargo_build_invocation_preserves_rp2040_package_bin_target_mode_and_features() {
        let mut recipe = FirmwareRecipe::ignition_only_wasted_spark(2);
        recipe.safety.watchdog_required = false;

        let invocation = recipe
            .resolve_build_plan(RP2040_PICO_TS_ECU_BUILD_METADATA)
            .unwrap()
            .cargo_build_invocation()
            .unwrap();

        assert_eq!(invocation.package, "ecu-rp2040-pico");
        assert_eq!(invocation.binary, "ts-ecu");
        assert_eq!(invocation.target_triple, "thumbv6m-none-eabi");
        assert_eq!(invocation.mode, FirmwareBuildMode::Release);
        assert_eq!(invocation.features.as_slice(), &["capture-pio"]);
    }

    #[test]
    fn named_board_mismatch_rejects_rp2040_pico_metadata() {
        let mut recipe =
            FirmwareRecipe::ignition_only_wasted_spark(2).for_board(BoardSelection::Named("other"));
        recipe.safety.watchdog_required = false;

        assert_eq!(
            recipe.resolve_build_plan(RP2040_PICO_TS_ECU_BUILD_METADATA),
            Err(RecipeValidationError::BoardSelectionMismatch {
                requested: BoardId::new("other"),
                actual: BOARD_ID_RP2040_PICO,
            })
        );
    }
}
