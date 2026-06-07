use super::*;
use ecu_board_api::{AuxSafetyProfile, OutputAuthorityRequirement};
use ecu_board_api::{BoardCapabilities, LoadSourceCapabilities, PinMapId, RuntimeBuildId};
use ecu_domain::CylinderId;

const IGNITION_ONLY_CAPS: BoardCapabilities =
    BoardCapabilities::new(true, true, false, 2, 0, 0, false, false, true);
const INJECTION_ONLY_CAPS: BoardCapabilities =
    BoardCapabilities::new(false, true, false, 0, 2, 0, false, false, true);
const REV_LIMITER_CAPS: BoardCapabilities =
    BoardCapabilities::new(false, true, false, 1, 0, 0, false, false, true);
const NO_WATCHDOG_CAPS: BoardCapabilities =
    BoardCapabilities::new(true, true, false, 2, 2, 0, false, false, false);
const FULL_ECU_CAPS: BoardCapabilities =
    BoardCapabilities::new(true, true, true, 2, 4, 0, false, true, true)
        .with_load_sources(LoadSourceCapabilities::map());
const TEST_BOARD_ID: BoardId = BoardId::new("test-board");
const OTHER_BOARD_ID: BoardId = BoardId::new("other-board");
const TEST_PIN_MAP_ID: PinMapId = PinMapId::new(11);
const TEST_RUNTIME_BUILD_ID: RuntimeBuildId = RuntimeBuildId::new(22);

fn test_metadata(
    capabilities: BoardCapabilities,
    feature_bindings: BoardFeatureBindings,
    default_ts_profile: Option<&'static str>,
) -> BoardBuildMetadata {
    BoardBuildMetadata {
        board_id: TEST_BOARD_ID,
        artifact: BuildArtifact::CargoBin {
            package: "ecu-test-board",
            binary: "ecu-test",
            target_triple: "thumbv7em-none-eabihf",
            required_features: &[],
        },
        capabilities,
        pin_map_id: TEST_PIN_MAP_ID,
        runtime_build_id: TEST_RUNTIME_BUILD_ID,
        feature_bindings,
        default_ts_profile,
    }
}

const fn inline_full_ecu_output_profile() -> FullEcuOutputProfile {
    FullEcuOutputProfile::sequential_wasted_spark(
        [
            CylinderId::new(1),
            CylinderId::new(3),
            CylinderId::new(4),
            CylinderId::new(2),
        ],
        4,
        2,
        AuxSafetyProfile::none(),
        OutputAuthorityRequirement::FullSequential720,
    )
}

#[test]
fn ignition_only_and_injection_only_recipes_are_distinct() {
    let ignition = FirmwareRecipe::ignition_only_wasted_spark(4);
    let injection = FirmwareRecipe::injection_only_batch(2);

    assert!(ignition.subsystems.ignition);
    assert!(!ignition.subsystems.injection);
    assert_eq!(ignition.outputs.ignition_channels, 2);
    assert_eq!(ignition.outputs.injector_channels, 0);
    assert!(matches!(
        ignition.runtime.output_profile,
        RuntimeOutputProfile::IgnitionOnly(_)
    ));

    assert!(injection.subsystems.injection);
    assert!(!injection.subsystems.ignition);
    assert_eq!(injection.outputs.ignition_channels, 0);
    assert_eq!(injection.outputs.injector_channels, 2);
    assert!(matches!(
        injection.runtime.output_profile,
        RuntimeOutputProfile::InjectionOnly(_)
    ));
}

#[test]
fn ignition_only_recipe_does_not_require_injectors() {
    let recipe = FirmwareRecipe::ignition_only_wasted_spark(4);

    assert_eq!(recipe.validate_for(IGNITION_ONLY_CAPS), Ok(()));
    assert_eq!(
        recipe.validate_for(INJECTION_ONLY_CAPS),
        Err(RecipeValidationError::UnsupportedRevLimiter)
    );
}

#[test]
fn injection_only_recipe_does_not_require_ignition() {
    let recipe = FirmwareRecipe::injection_only_batch(2);

    assert_eq!(recipe.validate_for(INJECTION_ONLY_CAPS), Ok(()));
    assert_eq!(
        recipe.validate_for(IGNITION_ONLY_CAPS),
        Err(RecipeValidationError::UnsupportedInjectionOnly)
    );
}

#[test]
fn recipe_validation_checks_channel_counts_and_watchdog() {
    let injection = FirmwareRecipe::injection_only_batch(3);
    assert_eq!(
        injection.validate_for(INJECTION_ONLY_CAPS),
        Err(RecipeValidationError::InjectorChannelCount {
            required: 3,
            available: 2,
        })
    );

    let ignition = FirmwareRecipe::ignition_only_wasted_spark(6);
    assert_eq!(
        ignition.validate_for(IGNITION_ONLY_CAPS),
        Err(RecipeValidationError::IgnitionChannelCount {
            required: 3,
            available: 2,
        })
    );

    assert_eq!(
        FirmwareRecipe::injection_only_single_point().validate_for(NO_WATCHDOG_CAPS),
        Err(RecipeValidationError::MissingWatchdog)
    );
}

#[test]
fn ignition_only_wasted_spark_rejects_odd_or_out_of_range_cylinder_counts() {
    assert_eq!(
        FirmwareRecipe::ignition_only_wasted_spark(1).validate_for(IGNITION_ONLY_CAPS),
        Err(RecipeValidationError::UnsupportedWastedSparkCylinderCount { cylinder_count: 1 })
    );
    assert_eq!(
        FirmwareRecipe::ignition_only_wasted_spark(5).validate_for(IGNITION_ONLY_CAPS),
        Err(RecipeValidationError::UnsupportedWastedSparkCylinderCount { cylinder_count: 5 })
    );
    assert_eq!(
        FirmwareRecipe::ignition_only_wasted_spark(18).validate_for(IGNITION_ONLY_CAPS),
        Err(RecipeValidationError::UnsupportedWastedSparkCylinderCount { cylinder_count: 18 })
    );
}

#[test]
fn rev_limiter_is_rpm_input_plus_ignition_cut_without_fuel() {
    let recipe = FirmwareRecipe::rev_limiter();

    assert!(recipe.inputs.rpm);
    assert!(!recipe.inputs.trigger);
    assert!(recipe.subsystems.rev_limiter);
    assert!(!recipe.subsystems.ignition);
    assert!(!recipe.subsystems.injection);
    assert_eq!(recipe.outputs.ignition_channels, 1);
    assert_eq!(recipe.outputs.injector_channels, 0);
    assert!(recipe.outputs.ignition_cut);
    assert_eq!(recipe.validate_for(REV_LIMITER_CAPS), Ok(()));
}

#[test]
fn full_ecu_recipe_uses_generic_runtime_output_profile() {
    let output_profile = inline_full_ecu_output_profile();
    let recipe = FirmwareRecipe::full_ecu(output_profile);

    assert_eq!(recipe.name, "full_ecu");
    assert!(recipe.subsystems.trigger_capture);
    assert!(recipe.subsystems.ignition);
    assert!(recipe.subsystems.injection);
    assert!(recipe.subsystems.fuel_strategy);
    assert!(recipe.subsystems.persistence);
    assert!(recipe.inputs.rpm);
    assert!(recipe.inputs.trigger);
    assert!(recipe.inputs.cam);
    assert!(recipe.inputs.load_sensor);
    assert_eq!(recipe.outputs.ignition_channels, 2);
    assert_eq!(recipe.outputs.injector_channels, 4);
    assert!(matches!(
        recipe.runtime.output_profile,
        RuntimeOutputProfile::FullEcu(profile) if profile == output_profile
    ));
}

#[test]
fn full_ecu_recipe_validates_required_capabilities() {
    let recipe = FirmwareRecipe::full_ecu(inline_full_ecu_output_profile());

    assert_eq!(recipe.validate_for(FULL_ECU_CAPS), Ok(()));

    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(false, true, true, 2, 4, 0, false, true, true,)
                .with_load_sources(LoadSourceCapabilities::map())
        ),
        Err(RecipeValidationError::UnsupportedFullEcu)
    );
    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(true, true, false, 2, 4, 0, false, true, true,)
                .with_load_sources(LoadSourceCapabilities::map())
        ),
        Err(RecipeValidationError::MissingCamInput)
    );
    assert_eq!(
        recipe.validate_for(BoardCapabilities::new(
            true, true, true, 2, 4, 0, false, true, true,
        )),
        Err(RecipeValidationError::MissingLoadSensor)
    );
    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(true, true, true, 1, 4, 0, false, true, true,)
                .with_load_sources(LoadSourceCapabilities::map())
        ),
        Err(RecipeValidationError::IgnitionChannelCount {
            required: 2,
            available: 1,
        })
    );
    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(true, true, true, 2, 3, 0, false, true, true,)
                .with_load_sources(LoadSourceCapabilities::map())
        ),
        Err(RecipeValidationError::InjectorChannelCount {
            required: 4,
            available: 3,
        })
    );
    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(true, true, true, 2, 4, 0, false, false, true,)
                .with_load_sources(LoadSourceCapabilities::map())
        ),
        Err(RecipeValidationError::MissingCalibrationPersistence)
    );
    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(true, true, true, 2, 4, 0, false, true, false,)
                .with_load_sources(LoadSourceCapabilities::map())
        ),
        Err(RecipeValidationError::MissingWatchdog)
    );
}

#[test]
fn load_sensor_recipe_fails_without_load_source_capability() {
    let mut recipe = FirmwareRecipe::injection_only_batch(2);
    recipe.inputs = InputSourceSet::crank_cam_load();

    assert_eq!(
        recipe.validate_for(BoardCapabilities::new(
            true, true, true, 0, 2, 0, false, false, true,
        )),
        Err(RecipeValidationError::MissingLoadSensor)
    );
    assert_eq!(
        recipe.validate_for(
            BoardCapabilities::new(true, true, true, 0, 2, 0, false, false, true,)
                .with_load_sources(LoadSourceCapabilities::maf())
        ),
        Ok(())
    );
}

#[test]
fn build_plan_binds_metadata_and_emits_features_checklist_and_ts_selection() {
    let bindings = BoardFeatureBindings::all_builtin()
        .with(
            RecipeFeature::TriggerCapture,
            FeatureBinding::CargoFeature("capture-pio"),
        )
        .with(
            RecipeFeature::Injection,
            FeatureBinding::CargoFeature("fuel-batch"),
        )
        .with(
            RecipeFeature::TunerStudio,
            FeatureBinding::CargoFeature("ts-usb"),
        )
        .with(
            RecipeFeature::Persistence,
            FeatureBinding::CargoFeature("flash-kv"),
        );
    let metadata = test_metadata(FULL_ECU_CAPS, bindings, Some("board-default.ini"));
    let mut recipe = FirmwareRecipe::full_ecu(inline_full_ecu_output_profile())
        .for_board(BoardSelection::Named(TEST_BOARD_ID.get()));
    recipe.subsystems.tuner_studio = true;
    recipe.ts_profile = Some("recipe.ini");

    let plan = recipe.resolve_build_plan(metadata).unwrap();

    assert_eq!(plan.board_id, TEST_BOARD_ID);
    assert_eq!(plan.artifact, metadata.artifact);
    assert_eq!(plan.pin_map_id, TEST_PIN_MAP_ID);
    assert_eq!(plan.runtime_build_id, TEST_RUNTIME_BUILD_ID);
    assert_eq!(
        plan.cargo_features.as_slice(),
        &["capture-pio", "fuel-batch", "ts-usb", "flash-kv"]
    );
    assert_eq!(
        plan.first_run.as_slice(),
        &[
            Some(FirstRunChecklistItem::VerifyPinMap(TEST_PIN_MAP_ID)),
            Some(FirstRunChecklistItem::VerifyTriggerWiring),
            Some(FirstRunChecklistItem::VerifyLoadSensor),
            Some(FirstRunChecklistItem::VerifyWatchdog),
            Some(FirstRunChecklistItem::VerifyCalibrationPersistence),
            Some(FirstRunChecklistItem::VerifyTunerStudioProfile(
                TunerStudioProfileSelection::Recipe("recipe.ini"),
            )),
        ]
    );
    assert_eq!(
        plan.ts_profile,
        TunerStudioProfileSelection::Recipe("recipe.ini")
    );
}

#[test]
fn recipe_build_invocation_from_cargo_bin_plan_preserves_artifact_and_features() {
    let bindings = BoardFeatureBindings::all_builtin()
        .with(
            RecipeFeature::TriggerCapture,
            FeatureBinding::CargoFeature("capture-pio"),
        )
        .with(
            RecipeFeature::Injection,
            FeatureBinding::CargoFeature("fuel-batch"),
        );
    let metadata = test_metadata(INJECTION_ONLY_CAPS, bindings, None);
    let recipe = FirmwareRecipe::injection_only_batch(2);

    let invocation = recipe
        .resolve_build_plan(metadata)
        .unwrap()
        .cargo_build_invocation()
        .unwrap();

    assert_eq!(invocation.package, "ecu-test-board");
    assert_eq!(invocation.binary, "ecu-test");
    assert_eq!(invocation.target_triple, "thumbv7em-none-eabihf");
    assert_eq!(invocation.mode, FirmwareBuildMode::Release);
    assert_eq!(invocation.features.as_slice(), &["fuel-batch"]);
}

#[test]
fn recipe_build_invocation_merges_artifact_required_features() {
    let mut metadata = test_metadata(
        INJECTION_ONLY_CAPS,
        BoardFeatureBindings::all_builtin().with(
            RecipeFeature::Injection,
            FeatureBinding::CargoFeature("artifact-runtime"),
        ),
        None,
    );
    metadata.artifact = BuildArtifact::CargoBin {
        package: "ecu-test-board",
        binary: "ecu-test",
        target_triple: "thumbv7em-none-eabihf",
        required_features: &["artifact-runtime", "artifact-io"],
    };
    let recipe = FirmwareRecipe::injection_only_batch(2);

    let plan = recipe.resolve_build_plan(metadata).unwrap();
    let invocation = plan.cargo_build_invocation().unwrap();

    assert_eq!(plan.cargo_features.as_slice(), &["artifact-runtime"]);
    assert_eq!(
        invocation.features.as_slice(),
        &["artifact-runtime", "artifact-io"]
    );
}

#[test]
fn recipe_build_invocation_rejects_board_only_artifact() {
    let mut metadata = test_metadata(REV_LIMITER_CAPS, BoardFeatureBindings::all_builtin(), None);
    metadata.artifact = BuildArtifact::BoardOnly {
        board_id: TEST_BOARD_ID,
    };
    let recipe = FirmwareRecipe::rev_limiter();

    let plan = recipe.resolve_build_plan(metadata).unwrap();

    assert_eq!(
        plan.cargo_build_invocation(),
        Err(BuildInvocationError::NoCargoBinaryArtifact)
    );
}

#[test]
fn recipe_build_invocation_uses_deduplicated_plan_features() {
    let bindings = BoardFeatureBindings::all_builtin()
        .with(
            RecipeFeature::TriggerCapture,
            FeatureBinding::CargoFeature("shared-runtime"),
        )
        .with(
            RecipeFeature::Ignition,
            FeatureBinding::CargoFeature("shared-runtime"),
        )
        .with(
            RecipeFeature::RevLimiter,
            FeatureBinding::CargoFeature("shared-runtime"),
        );
    let metadata = test_metadata(IGNITION_ONLY_CAPS, bindings, None);
    let recipe = FirmwareRecipe::ignition_only_wasted_spark(4);

    let invocation = recipe
        .resolve_build_plan(metadata)
        .unwrap()
        .cargo_build_invocation()
        .unwrap();

    assert_eq!(invocation.features.as_slice(), &["shared-runtime"]);
}

#[test]
fn recipe_build_invocation_deduplicates_artifact_required_features() {
    let bindings = BoardFeatureBindings::all_builtin()
        .with(
            RecipeFeature::TriggerCapture,
            FeatureBinding::CargoFeature("shared-runtime"),
        )
        .with(
            RecipeFeature::Ignition,
            FeatureBinding::CargoFeature("shared-runtime"),
        )
        .with(
            RecipeFeature::RevLimiter,
            FeatureBinding::CargoFeature("shared-runtime"),
        );
    let mut metadata = test_metadata(IGNITION_ONLY_CAPS, bindings, None);
    metadata.artifact = BuildArtifact::CargoBin {
        package: "ecu-test-board",
        binary: "ecu-test",
        target_triple: "thumbv7em-none-eabihf",
        required_features: &["shared-runtime", "artifact-io", "artifact-io"],
    };
    let recipe = FirmwareRecipe::ignition_only_wasted_spark(4);

    let invocation = recipe
        .resolve_build_plan(metadata)
        .unwrap()
        .cargo_build_invocation()
        .unwrap();

    assert_eq!(
        invocation.features.as_slice(),
        &["shared-runtime", "artifact-io"]
    );
}

#[test]
fn tuner_studio_without_recipe_or_default_profile_uses_placeholder() {
    let metadata = test_metadata(FULL_ECU_CAPS, BoardFeatureBindings::all_builtin(), None);
    let mut recipe = FirmwareRecipe::full_ecu(inline_full_ecu_output_profile());
    recipe.subsystems.tuner_studio = true;
    recipe.ts_profile = None;

    let plan = recipe.resolve_build_plan(metadata).unwrap();

    assert_eq!(
        plan.ts_profile,
        TunerStudioProfileSelection::RequiredButUnselected
    );
    assert_eq!(plan.ts_profile.profile_name(), None);
    assert!(plan.ts_profile.is_some());
    assert_eq!(
        plan.first_run.as_slice().last(),
        Some(&Some(FirstRunChecklistItem::VerifyTunerStudioProfile(
            TunerStudioProfileSelection::RequiredButUnselected,
        )))
    );
}

#[test]
fn tuner_studio_profile_requires_enabled_subsystem() {
    let metadata = test_metadata(FULL_ECU_CAPS, BoardFeatureBindings::all_builtin(), None);
    let mut recipe = FirmwareRecipe::full_ecu(inline_full_ecu_output_profile());
    recipe.subsystems.tuner_studio = false;
    recipe.ts_profile = Some("recipe.ini");

    assert_eq!(
        recipe.resolve_build_plan(metadata),
        Err(RecipeValidationError::TunerStudioProfileWithoutSubsystem)
    );
}

#[test]
fn board_default_tuner_studio_profile_is_ignored_when_subsystem_is_disabled() {
    let metadata = test_metadata(
        INJECTION_ONLY_CAPS,
        BoardFeatureBindings::all_builtin(),
        Some("board-default.ini"),
    );
    let recipe = FirmwareRecipe::injection_only_batch(2);

    let plan = recipe.resolve_build_plan(metadata).unwrap();

    assert_eq!(plan.ts_profile, TunerStudioProfileSelection::None);
    assert!(!plan.first_run.as_slice().iter().any(|item| matches!(
        item,
        Some(FirstRunChecklistItem::VerifyTunerStudioProfile(_))
    )));
}

#[test]
fn resolve_build_plan_rejects_artifact_required_feature_capacity_overflow() {
    let bindings = BoardFeatureBindings::all_builtin()
        .with(
            RecipeFeature::TriggerCapture,
            FeatureBinding::CargoFeature("capture-pio"),
        )
        .with(
            RecipeFeature::Ignition,
            FeatureBinding::CargoFeature("ignition"),
        )
        .with(
            RecipeFeature::RevLimiter,
            FeatureBinding::CargoFeature("revlimiter"),
        );
    let mut metadata = test_metadata(IGNITION_ONLY_CAPS, bindings, None);
    metadata.artifact = BuildArtifact::CargoBin {
        package: "ecu-test-board",
        binary: "ecu-test",
        target_triple: "thumbv7em-none-eabihf",
        required_features: &["f1", "f2", "f3", "f4", "f5", "f6"],
    };
    let recipe = FirmwareRecipe::ignition_only_wasted_spark(4);

    assert_eq!(
        recipe.resolve_build_plan(metadata),
        Err(RecipeValidationError::CargoFeatureCapacityExceeded)
    );
}

#[test]
fn named_board_selection_rejects_mismatched_metadata() {
    let recipe =
        FirmwareRecipe::rev_limiter().for_board(BoardSelection::Named(OTHER_BOARD_ID.get()));
    let metadata = test_metadata(REV_LIMITER_CAPS, BoardFeatureBindings::all_builtin(), None);

    assert_eq!(
        recipe.resolve_build_plan(metadata),
        Err(RecipeValidationError::BoardSelectionMismatch {
            requested: OTHER_BOARD_ID,
            actual: TEST_BOARD_ID,
        })
    );
}

#[test]
fn any_compatible_accepts_metadata_but_still_validates_capabilities() {
    let recipe = FirmwareRecipe::rev_limiter();
    let metadata = test_metadata(NO_WATCHDOG_CAPS, BoardFeatureBindings::all_builtin(), None);

    assert_eq!(
        recipe.resolve_build_plan(metadata),
        Err(RecipeValidationError::MissingWatchdog)
    );
}

#[test]
fn resolve_build_plan_propagates_capability_errors() {
    let recipe = FirmwareRecipe::full_ecu(inline_full_ecu_output_profile());
    let metadata = test_metadata(
        BoardCapabilities::new(true, true, false, 2, 4, 0, false, true, true)
            .with_load_sources(LoadSourceCapabilities::map()),
        BoardFeatureBindings::all_builtin(),
        None,
    );

    assert_eq!(
        recipe.resolve_build_plan(metadata),
        Err(RecipeValidationError::MissingCamInput)
    );
}

#[test]
fn unsupported_feature_binding_fails_when_subsystem_is_enabled() {
    let recipe = FirmwareRecipe::injection_only_batch(2);
    let metadata = test_metadata(
        INJECTION_ONLY_CAPS,
        BoardFeatureBindings::all_builtin()
            .with(RecipeFeature::Injection, FeatureBinding::Unsupported),
        None,
    );

    assert_eq!(
        recipe.resolve_build_plan(metadata),
        Err(RecipeValidationError::UnsupportedFeatureBinding {
            feature: RecipeFeature::Injection,
        })
    );
}
