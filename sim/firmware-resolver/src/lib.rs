mod commands;
mod errors;
mod selection;
mod ts_ini;

pub use commands::{
    resolve_bin_path, resolve_command, resolve_elf_path, resolve_flash_command,
    resolve_objcopy_command,
};
pub use errors::ResolveError;
pub use selection::FirmwareSelection;
use selection::{resolve_alias, FirmwareAlias};
pub use ts_ini::{
    resolve_ts_asset_path, resolve_ts_generate_command, resolve_ts_generated_path, resolve_ts_ini,
};

use ecu_board_api::{BoardCapabilities, PinMapId, RuntimeBuildId};
use ecu_board_profiles::{
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BuildArtifact, FeatureBinding,
    FirmwareRecipe,
};

#[derive(Debug, Clone, Copy)]
pub(crate) struct FirmwareEntry {
    pub board: &'static str,
    pub recipe: &'static str,
    pub metadata: BoardBuildMetadata,
    pub recipe_fn: fn() -> FirmwareRecipe,
    pub flash_chip: Option<&'static str>,
    pub listed: bool,
}

const FIRMWARE_BOARD_ID_RP2040_PICO: BoardId = BoardId::new("rp2040-pico");
const FIRMWARE_BOARD_ID_RP2350B: BoardId = BoardId::new("rp2350b");
const FIRMWARE_BOARD_ID_STM32F4: BoardId = BoardId::new("stm32f4");

const FIRMWARE_PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP: PinMapId = PinMapId::new(0x2040);
const FIRMWARE_PIN_MAP_RP2350B_REV_LIMITER_BRINGUP: PinMapId = PinMapId::new(0x2350);
const FIRMWARE_PIN_MAP_STM32F4_IGNITION_BRINGUP: PinMapId = PinMapId::new(0xF405);

const FIRMWARE_RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU: RuntimeBuildId = RuntimeBuildId::new(1);
const FIRMWARE_RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP: RuntimeBuildId =
    RuntimeBuildId::new(1);
const FIRMWARE_RUNTIME_BUILD_ID_STM32F4_ECU: RuntimeBuildId = RuntimeBuildId::new(1);

const FIRMWARE_FEATURE_BINDINGS_RP2040_PICO_TS_ECU: BoardFeatureBindings = BoardFeatureBindings {
    trigger_capture: FeatureBinding::CargoFeature("capture-pio"),
    ignition: FeatureBinding::BuiltIn,
    injection: FeatureBinding::Unsupported,
    fuel_strategy: FeatureBinding::Unsupported,
    rev_limiter: FeatureBinding::BuiltIn,
    tuner_studio: FeatureBinding::BuiltIn,
    telemetry: FeatureBinding::Unsupported,
    persistence: FeatureBinding::CargoFeature("flash-kv"),
};

const FIRMWARE_FEATURE_BINDINGS_RP2350B_REV_LIMITER_BRINGUP: BoardFeatureBindings =
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

const FIRMWARE_FEATURE_BINDINGS_STM32F4_IGNITION_BRINGUP: BoardFeatureBindings =
    BoardFeatureBindings {
        trigger_capture: FeatureBinding::CargoFeature("capture-tim"),
        ignition: FeatureBinding::BuiltIn,
        injection: FeatureBinding::Unsupported,
        fuel_strategy: FeatureBinding::Unsupported,
        rev_limiter: FeatureBinding::BuiltIn,
        tuner_studio: FeatureBinding::Unsupported,
        telemetry: FeatureBinding::Unsupported,
        persistence: FeatureBinding::Unsupported,
    };

const FIRMWARE_METADATA_RP2040_PICO_TS_ECU: BoardBuildMetadata = BoardBuildMetadata {
    board_id: FIRMWARE_BOARD_ID_RP2040_PICO,
    artifact: BuildArtifact::CargoBin {
        package: "ecu-rp2040-pico",
        binary: "ts-ecu",
        target_triple: "thumbv6m-none-eabi",
        required_features: &["capture-pio"],
    },
    capabilities: BoardCapabilities::new(true, true, false, 1, 0, 0, false, true, false),
    pin_map_id: FIRMWARE_PIN_MAP_RP2040_PICO_TS_ECU_BRINGUP,
    runtime_build_id: FIRMWARE_RUNTIME_BUILD_ID_RP2040_PICO_TS_ECU,
    feature_bindings: FIRMWARE_FEATURE_BINDINGS_RP2040_PICO_TS_ECU,
    default_ts_profile: Some("crates/compat/tests/assets/IPW-ECU.ini"),
};

const FIRMWARE_METADATA_RP2350B_REV_LIMITER_BRINGUP: BoardBuildMetadata = BoardBuildMetadata {
    board_id: FIRMWARE_BOARD_ID_RP2350B,
    artifact: BuildArtifact::CargoBin {
        package: "ecu-rp2350b",
        binary: "ecu-rp2350b-min",
        target_triple: "thumbv8m.main-none-eabihf",
        required_features: &["example-bins"],
    },
    capabilities: BoardCapabilities::new(false, true, false, 1, 0, 0, false, false, false),
    pin_map_id: FIRMWARE_PIN_MAP_RP2350B_REV_LIMITER_BRINGUP,
    runtime_build_id: FIRMWARE_RUNTIME_BUILD_ID_RP2350B_REV_LIMITER_BRINGUP,
    feature_bindings: FIRMWARE_FEATURE_BINDINGS_RP2350B_REV_LIMITER_BRINGUP,
    default_ts_profile: None,
};

const FIRMWARE_METADATA_STM32F4_IGNITION_BRINGUP: BoardBuildMetadata = BoardBuildMetadata {
    board_id: FIRMWARE_BOARD_ID_STM32F4,
    artifact: BuildArtifact::CargoBin {
        package: "stm32f4-ecu",
        binary: "stm32f4-ecu",
        target_triple: "thumbv7em-none-eabihf",
        required_features: &["capture-tim"],
    },
    capabilities: BoardCapabilities::new(true, true, false, 2, 0, 0, false, false, false),
    pin_map_id: FIRMWARE_PIN_MAP_STM32F4_IGNITION_BRINGUP,
    runtime_build_id: FIRMWARE_RUNTIME_BUILD_ID_STM32F4_ECU,
    feature_bindings: FIRMWARE_FEATURE_BINDINGS_STM32F4_IGNITION_BRINGUP,
    default_ts_profile: None,
};

pub const SUPPORTED_INVOCATIONS: &[(&str, &str)] = &[
    (
        "rp2040-pico",
        "ignition-only-wasted-spark-no-watchdog-bringup",
    ),
    ("rp2350b", "rev-limiter-no-watchdog-bringup"),
    ("stm32f4", "ignition-only-wasted-spark-no-watchdog-bringup"),
];

pub const SUPPORTED_ALIASES: &[FirmwareAlias] = &[
    FirmwareAlias {
        alias: "pico_ignition_only_wasted_spark",
        board: "rp2040-pico",
        recipe: "ignition-only-wasted-spark-no-watchdog-bringup",
    },
    FirmwareAlias {
        alias: "rp2040_pico_ignition_only_wasted_spark",
        board: "rp2040-pico",
        recipe: "ignition-only-wasted-spark-no-watchdog-bringup",
    },
    FirmwareAlias {
        alias: "rp2350_rev_limiter",
        board: "rp2350b",
        recipe: "rev-limiter-no-watchdog-bringup",
    },
    FirmwareAlias {
        alias: "stm32f4_ignition_only_wasted_spark",
        board: "stm32f4",
        recipe: "ignition-only-wasted-spark-no-watchdog-bringup",
    },
];

pub(crate) const FIRMWARE_REGISTRY: &[FirmwareEntry] = &[
    FirmwareEntry {
        board: "rp2040-pico",
        recipe: "ignition-only-wasted-spark-no-watchdog-bringup",
        metadata: FIRMWARE_METADATA_RP2040_PICO_TS_ECU,
        recipe_fn: rp2040_ignition_only_bringup_recipe,
        flash_chip: Some("RP2040"),
        listed: true,
    },
    FirmwareEntry {
        board: "rp2350b",
        recipe: "rev-limiter-no-watchdog-bringup",
        metadata: FIRMWARE_METADATA_RP2350B_REV_LIMITER_BRINGUP,
        recipe_fn: rp2350_rev_limiter_bringup_recipe,
        flash_chip: Some("RP2350"),
        listed: true,
    },
    FirmwareEntry {
        board: "stm32f4",
        recipe: "ignition-only-wasted-spark-no-watchdog-bringup",
        metadata: FIRMWARE_METADATA_STM32F4_IGNITION_BRINGUP,
        recipe_fn: stm32f4_ignition_only_bringup_recipe,
        flash_chip: None,
        listed: true,
    },
];

/// Resolve the `FirmwareRecipe` for a board/recipe pair, without a board selection wrapper.
pub fn resolve_recipe(board: &str, recipe: &str) -> Result<FirmwareRecipe, ResolveError> {
    let (_, _, recipe) = resolve_metadata_and_recipe(board, recipe)?;
    Ok(recipe)
}

pub fn resolve_firmware_selection(
    board_or_alias: &str,
    recipe: Option<&str>,
) -> Result<FirmwareSelection, ResolveError> {
    match recipe {
        Some(recipe) => selection::canonical_selection(board_or_alias, recipe),
        None => resolve_alias(board_or_alias),
    }
}

pub(crate) fn resolve_metadata_and_recipe(
    board: &str,
    recipe: &str,
) -> Result<(&'static str, BoardBuildMetadata, FirmwareRecipe), ResolveError> {
    let entry = FIRMWARE_REGISTRY
        .iter()
        .find(|entry| entry.board == board && entry.recipe == recipe)
        .ok_or(ResolveError::UnsupportedSelection)?;
    Ok((entry.board, entry.metadata, (entry.recipe_fn)()))
}

pub(crate) const fn no_watchdog_bringup(mut recipe: FirmwareRecipe) -> FirmwareRecipe {
    recipe.safety = ecu_board_profiles::SafetyPolicy::no_watchdog(
        "bring-up only: watchdog absent to allow debugger pauses; not for production use",
    );
    recipe
}

fn rp2040_ignition_only_bringup_recipe() -> FirmwareRecipe {
    no_watchdog_bringup(FirmwareRecipe::ignition_only_wasted_spark(2))
}

fn rp2350_rev_limiter_bringup_recipe() -> FirmwareRecipe {
    no_watchdog_bringup(FirmwareRecipe::rev_limiter())
}

fn stm32f4_ignition_only_bringup_recipe() -> FirmwareRecipe {
    no_watchdog_bringup(FirmwareRecipe::ignition_only_wasted_spark(2))
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_board_profiles::TunerStudioProfileSelection;

    #[test]
    fn public_invocation_table_matches_listed_registry_entries() {
        let listed = FIRMWARE_REGISTRY
            .iter()
            .filter(|entry| entry.listed)
            .count();
        assert_eq!(SUPPORTED_INVOCATIONS.len(), listed);
        for entry in FIRMWARE_REGISTRY.iter().filter(|entry| entry.listed) {
            assert!(SUPPORTED_INVOCATIONS
                .iter()
                .any(|(board, recipe)| *board == entry.board && *recipe == entry.recipe));
        }
    }

    #[test]
    fn aliases_point_to_registry_entries() {
        for alias in SUPPORTED_ALIASES {
            assert!(FIRMWARE_REGISTRY
                .iter()
                .any(|entry| entry.board == alias.board && entry.recipe == alias.recipe));
            assert_eq!(
                resolve_firmware_selection(alias.alias, None).unwrap(),
                FirmwareSelection {
                    board: alias.board,
                    recipe: alias.recipe,
                }
            );
        }
    }

    #[test]
    fn listed_invocations_resolve_to_executable_commands() {
        for (board, recipe) in SUPPORTED_INVOCATIONS {
            let command = commands::resolve_command(board, recipe).unwrap();
            assert!(command.starts_with("cargo build -p "));
        }
    }

    #[test]
    fn canonical_watchdog_only_recipes_are_not_supported() {
        assert!(matches!(
            resolve_firmware_selection("rp2040-pico", Some("ignition-only-wasted-spark")),
            Err(ResolveError::UnsupportedSelection)
        ));
        assert!(matches!(
            resolve_firmware_selection("rp2350b", Some("rev-limiter")),
            Err(ResolveError::UnsupportedSelection)
        ));
        assert!(matches!(
            resolve_firmware_selection("stm32f4", Some("ignition-only-wasted-spark")),
            Err(ResolveError::UnsupportedSelection)
        ));
    }

    #[test]
    fn alias_selection_renders_same_command_and_paths_as_canonical_pair() {
        let alias = resolve_firmware_selection("pico_ignition_only_wasted_spark", None).unwrap();
        assert_eq!(alias.board, "rp2040-pico");
        assert_eq!(
            commands::resolve_command(alias.board, alias.recipe).unwrap(),
            commands::resolve_command(
                "rp2040-pico",
                "ignition-only-wasted-spark-no-watchdog-bringup"
            )
            .unwrap()
        );
        assert_eq!(
            commands::resolve_bin_path(alias.board, alias.recipe).unwrap(),
            commands::resolve_bin_path(
                "rp2040-pico",
                "ignition-only-wasted-spark-no-watchdog-bringup"
            )
            .unwrap()
        );
    }

    #[test]
    fn rejects_plans_without_selected_ts_asset() {
        let selection = TunerStudioProfileSelection::RequiredButUnselected;
        assert!(matches!(
            ts_ini::resolve_ts_asset_for_selection(selection),
            Err(ResolveError::NoTunerStudioAsset { selection: rejected })
                if rejected == selection
        ));
    }

    #[test]
    fn unsupported_board_only_selection_is_rejected() {
        assert!(matches!(
            resolve_firmware_selection("atmega2560-speeduino-m5x-rev23", Some("full-ecu")),
            Err(ResolveError::UnsupportedSelection)
        ));
    }
}
