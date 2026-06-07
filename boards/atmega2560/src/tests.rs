use crate::adapter::speeduino_m5x_rev23_aux_pin;
use crate::adapter::validate_speeduino_m5x_rev23_aux_batch;
use crate::adapter::Atmega2560BoardAdapter;
use crate::errors::{
    Atmega2560BridgeError, Atmega2560BuildPlanError, Atmega2560RecipePrepareError,
};
use crate::profile::{
    m50b25tu_full_cop_runtime_profile, m50b25tu_speeduino_m5x_rev23_board_profile,
    Atmega2560BoardProfile, SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA,
    SPEEDUINO_M5X_REV23_CAPABILITIES, SPEEDUINO_M5X_REV23_RESOURCE_LIMITS,
};
use crate::profile::{
    ATMEGA2560_ADC_CHANNELS, ATMEGA2560_TIMER_COMPARE_CHANNELS, ATMEGA2560_TIMER_COUNT,
    BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560, CLOCK_HZ, MAX_AUX_COMMANDS, MAX_OUTPUT_TRANSITIONS,
    PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC, PROFILE_ID_M50B25TU_FULL_COP,
    RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE, SPEEDUINO_M5X_REV23_LOW_CURRENT_AUX_CHANNELS,
    SPEEDUINO_M5X_REV23_PIN_MAP, SPEEDUINO_M5X_REV23_TACH_AUX_CHANNELS,
};
use crate::step_io::{
    Atmega2560MappedOutputBatch, Atmega2560MappedOutputTransition, Atmega2560StepInput,
};
use ecu_board_api::{
    AuxCommand, AuxOutput, AuxValue, EcuOutput, IgnitionProfileId, IgnitionProfileMode,
    OutputLevel, PinMapId, ProfileId, RuntimeBuildId,
};
use ecu_board_api::{FullEcuOutputProfile, RuntimeOutputProfile};
use ecu_board_profiles::{
    profiles::m50b25tu::M50B25TU_FULL_COP, BoardId, BoardSelection, BuildArtifact, FeatureBinding,
    FirmwareRecipe, FirstRunChecklistItem, RecipeFeature, RecipeValidationError,
    TunerStudioProfileSelection,
};
use ecu_domain::ChannelId;
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, Kpa10, Micros, PhaseSyncState, Rpm,
};

#[test]
fn speeduino_m5x_bridge_uses_m50_profile_facts() {
    let engine_profile = core::hint::black_box(M50B25TU_FULL_COP);
    let profile = m50b25tu_speeduino_m5x_rev23_board_profile();
    assert_eq!(engine_profile.engine.cylinders, 6);
    assert!(engine_profile.cam.phase_required_for_sequential);
    assert_eq!(profile.profile_id, PROFILE_ID_M50B25TU_FULL_COP);
    assert_eq!(
        profile.ignition_profile_authority_blocked_mode,
        IgnitionProfileMode::SequentialCopAuthorityBlocked
    );
}

#[test]
fn speeduino_m5x_schematic_pin_map_exposes_six_injectors_and_six_ignitions() {
    assert_eq!(
        SPEEDUINO_M5X_REV23_PIN_MAP
            .injector_pins
            .map(|pin| pin.get()),
        [8, 9, 10, 11, 12, 50]
    );
    assert_eq!(
        SPEEDUINO_M5X_REV23_PIN_MAP
            .ignition_pins
            .map(|pin| pin.get()),
        [40, 38, 52, 48, 36, 34]
    );
}

#[test]
fn speeduino_m5x_capabilities_match_schematic_pin_map_counts() {
    let capabilities = core::hint::black_box(SPEEDUINO_M5X_REV23_CAPABILITIES);
    assert!(capabilities.trigger_input);
    assert!(capabilities.rpm_input);
    assert!(capabilities.cam_input);
    assert_eq!(
        capabilities.ignition_channels,
        SPEEDUINO_M5X_REV23_PIN_MAP.ignition_pins.len() as u8
    );
    assert_eq!(
        capabilities.injector_channels,
        SPEEDUINO_M5X_REV23_PIN_MAP.injector_pins.len() as u8
    );
    assert_eq!(
        capabilities.aux_channels,
        SPEEDUINO_M5X_REV23_LOW_CURRENT_AUX_CHANNELS + SPEEDUINO_M5X_REV23_TACH_AUX_CHANNELS
    );
    assert!(capabilities.telemetry);
    assert!(!capabilities.calibration_persistence);
    assert!(!capabilities.watchdog);
}

#[test]
fn speeduino_m5x_resource_limits_are_board_facts_not_runtime_policy() {
    let limits = core::hint::black_box(SPEEDUINO_M5X_REV23_RESOURCE_LIMITS);
    let adapter = Atmega2560BoardAdapter::m50b25tu_speeduino_m5x_rev23();

    assert_eq!(
        limits,
        Atmega2560BoardAdapter::speeduino_m5x_rev23_resource_limits()
    );
    assert_eq!(adapter.resource_limits(), limits);
    assert_eq!(limits.clock_hz, CLOCK_HZ);
    assert_eq!(limits.adc_channels, ATMEGA2560_ADC_CHANNELS);
    assert_eq!(limits.timer_count, ATMEGA2560_TIMER_COUNT);
    assert_eq!(
        limits.timer_compare_channels,
        ATMEGA2560_TIMER_COMPARE_CHANNELS
    );
    assert_eq!(
        limits.output_channels,
        SPEEDUINO_M5X_REV23_PIN_MAP.injector_pins.len() as u8
            + SPEEDUINO_M5X_REV23_PIN_MAP.ignition_pins.len() as u8
            + SPEEDUINO_M5X_REV23_LOW_CURRENT_AUX_CHANNELS
            + SPEEDUINO_M5X_REV23_TACH_AUX_CHANNELS
    );
    assert_eq!(limits.output_transition_capacity, MAX_OUTPUT_TRANSITIONS);
    assert_eq!(limits.aux_command_capacity, MAX_AUX_COMMANDS);
    assert!(!limits.has_calibration_storage());
}

#[test]
fn speeduino_m5x_capabilities_support_m50_full_ecu_shape() {
    let capabilities = core::hint::black_box(SPEEDUINO_M5X_REV23_CAPABILITIES);
    let engine_profile = core::hint::black_box(M50B25TU_FULL_COP);
    assert!(capabilities.supports_rev_limiter());
    assert!(capabilities.supports_ignition_only());
    assert!(capabilities.supports_full_ecu());
    assert!(capabilities.supports_load_sensor());
    assert!(capabilities.injector_channels >= engine_profile.injection.channels);
    assert!(capabilities.ignition_channels >= engine_profile.engine.cylinders);
}

#[test]
fn speeduino_m5x_build_metadata_uses_board_owned_ids_and_bindings() {
    let metadata = SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA;

    assert_eq!(metadata.board_id, BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560);
    assert_eq!(
        metadata.artifact,
        BuildArtifact::BoardOnly {
            board_id: BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560,
        }
    );
    assert_eq!(metadata.capabilities, SPEEDUINO_M5X_REV23_CAPABILITIES);
    assert_eq!(metadata.pin_map_id, PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC);
    assert_eq!(
        metadata.runtime_build_id,
        RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE
    );
    assert_eq!(metadata.default_ts_profile, None);
    assert_eq!(
        metadata
            .feature_bindings
            .binding_for(RecipeFeature::TriggerCapture),
        FeatureBinding::BuiltIn
    );
    assert_eq!(
        metadata
            .feature_bindings
            .binding_for(RecipeFeature::TunerStudio),
        FeatureBinding::Unsupported
    );
    assert_eq!(
        metadata
            .feature_bindings
            .binding_for(RecipeFeature::Persistence),
        FeatureBinding::Unsupported
    );
}

#[test]
fn speeduino_m5x_full_ecu_recipe_resolves_against_build_metadata() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile());
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;

    let plan = recipe
        .resolve_build_plan(SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA)
        .unwrap();

    assert_eq!(plan.board_id, BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560);
    assert_eq!(
        plan.artifact,
        BuildArtifact::BoardOnly {
            board_id: BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560,
        }
    );
    assert!(plan.cargo_features.is_empty());
    assert_eq!(plan.pin_map_id, PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC);
    assert_eq!(plan.runtime_build_id, RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE);
    assert_eq!(
        plan.first_run.as_slice(),
        &[
            Some(FirstRunChecklistItem::VerifyPinMap(
                PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC,
            )),
            Some(FirstRunChecklistItem::VerifyTriggerWiring),
            Some(FirstRunChecklistItem::VerifyLoadSensor),
        ]
    );
    assert_eq!(plan.ts_profile, TunerStudioProfileSelection::None);
}

#[test]
fn speeduino_m5x_board_adapter_prepares_recipe_into_build_plan() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile());
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;

    let prepared = Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_recipe(recipe).unwrap();

    assert_eq!(
        prepared.plan().board_id,
        BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560
    );
    assert_eq!(
        prepared.plan().pin_map_id,
        PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC
    );
    assert_eq!(
        prepared.adapter().profile().runtime_build_id,
        RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE
    );
}

#[test]
fn speeduino_m5x_recipe_preparation_uses_recipe_runtime_output_profile() {
    let output_profile = FullEcuOutputProfile::sequential_wasted_spark(
        [
            ecu_domain::CylinderId::new(1),
            ecu_domain::CylinderId::new(3),
            ecu_domain::CylinderId::new(4),
            ecu_domain::CylinderId::new(2),
        ],
        4,
        2,
        ecu_board_api::AuxSafetyProfile::none(),
        ecu_board_api::OutputAuthorityRequirement::FullSequential720,
    );
    let mut recipe = FirmwareRecipe::full_ecu(output_profile);
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;

    let prepared = Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_recipe(recipe).unwrap();

    assert_eq!(
        prepared.adapter().profile().runtime_output_profile,
        output_profile
    );
    assert_ne!(
        prepared.adapter().profile().runtime_output_profile,
        m50b25tu_full_cop_runtime_profile()
    );
}

#[test]
fn speeduino_m5x_recipe_preparation_rejects_non_full_ecu_runtime_profiles() {
    let recipe = FirmwareRecipe::ignition_only_wasted_spark(4);

    assert_eq!(
        Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_recipe(recipe),
        Err(Atmega2560RecipePrepareError::UnsupportedRuntimeProfile {
            output_profile: RuntimeOutputProfile::crank_only_wasted_spark(4),
        })
    );
}

#[test]
fn speeduino_m5x_board_adapter_rejects_build_plan_for_other_board() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile());
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;
    let mut plan = recipe
        .resolve_build_plan(SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA)
        .unwrap();
    plan.board_id = BoardId::new("other-atmega2560-board");

    assert_eq!(
        Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_build_plan(plan),
        Err(Atmega2560BuildPlanError::BoardId {
            expected: BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560,
            actual: BoardId::new("other-atmega2560-board"),
        })
    );
}

#[test]
fn speeduino_m5x_board_adapter_rejects_build_plan_with_ts_profile() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile());
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;
    let mut plan = recipe
        .resolve_build_plan(SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA)
        .unwrap();
    plan.ts_profile = TunerStudioProfileSelection::Recipe("speeduino.ini");

    assert_eq!(
        Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_build_plan(plan),
        Err(Atmega2560BuildPlanError::TunerStudioProfileUnsupported {
            actual: TunerStudioProfileSelection::Recipe("speeduino.ini"),
        })
    );
}

#[test]
fn speeduino_m5x_board_adapter_rejects_build_plan_with_wrong_first_run_contract() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile());
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;
    let mut plan = recipe
        .resolve_build_plan(SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA)
        .unwrap();
    plan.first_run = Default::default();

    assert_eq!(
        Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_build_plan(plan),
        Err(Atmega2560BuildPlanError::FirstRunChecklist {
            expected: &[
                Some(FirstRunChecklistItem::VerifyPinMap(
                    PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC,
                )),
                Some(FirstRunChecklistItem::VerifyTriggerWiring),
                Some(FirstRunChecklistItem::VerifyLoadSensor),
            ],
            actual: Default::default(),
        })
    );
}

#[test]
fn speeduino_m5x_prepared_board_step_maps_runtime_outputs_to_avr_pins() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile());
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;
    let mut prepared = Atmega2560BoardAdapter::prepare_speeduino_m5x_rev23_recipe(recipe).unwrap();

    let output = prepared
        .step_mapped(Atmega2560StepInput::bench_synced(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(800),
        ))
        .unwrap();

    assert_eq!(output.step.outputs.len(), output.mapped_outputs.len());
    assert_eq!(output.mapped_outputs.len(), 24);
    for mapped in output.mapped_outputs.iter() {
        assert_eq!(
            Some(mapped.pin),
            SPEEDUINO_M5X_REV23_PIN_MAP.output_pin(mapped.transition.output)
        );
    }
}

#[test]
fn speeduino_m5x_aux_validation_maps_supported_roles_to_schematic_aux_pins() {
    assert_eq!(
        speeduino_m5x_rev23_aux_pin(AuxCommand::new(
            AuxOutput::SafetyRelay(0),
            AuxValue::Level(OutputLevel::High),
        )),
        Some(SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[0].get())
    );
    assert_eq!(
        speeduino_m5x_rev23_aux_pin(AuxCommand::new(AuxOutput::SafetyRelay(1), AuxValue::Off)),
        Some(SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins[1].get())
    );
    assert_eq!(
        speeduino_m5x_rev23_aux_pin(AuxCommand::new(AuxOutput::FrequencyOut(0), AuxValue::Off)),
        Some(SPEEDUINO_M5X_REV23_PIN_MAP.tach1_pin.get())
    );
    assert_eq!(
        speeduino_m5x_rev23_aux_pin(AuxCommand::new(AuxOutput::SafetyRelay(2), AuxValue::Off)),
        Some(SPEEDUINO_M5X_REV23_PIN_MAP.tach2_pin.get())
    );
}

#[test]
fn speeduino_m5x_aux_validation_rejects_unsupported_roles_before_forwarding() {
    let unsupported = AuxCommand::new(AuxOutput::Digital(ChannelId::new(0)), AuxValue::Off);

    assert_eq!(
        validate_speeduino_m5x_rev23_aux_batch::<2>(&{
            let mut batch = ecu_board_api::AuxCommandBatch::<2>::new();
            let _ = batch.push(unsupported);
            batch
        }),
        Err(Atmega2560BridgeError::UnsupportedAuxCommand(unsupported))
    );

    let mut batch = ecu_board_api::AuxCommandBatch::<2>::new();
    batch
        .push(AuxCommand::new(AuxOutput::SafetyRelay(1), AuxValue::Off))
        .unwrap();
    batch.push(unsupported).unwrap();

    assert_eq!(
        validate_speeduino_m5x_rev23_aux_batch(&batch),
        Err(Atmega2560BridgeError::UnsupportedAuxCommand(unsupported))
    );
}

#[test]
fn speeduino_m5x_board_local_batches_report_output_saturation() {
    let mut batch = Atmega2560MappedOutputBatch::<1>::new();
    let item = Atmega2560MappedOutputTransition {
        transition: ecu_board_api::OutputTransition::new(
            EcuOutput::Injector(ecu_domain::ChannelId::new(0)),
            OutputLevel::High,
            ecu_domain::Ticks::new(10),
        ),
        pin: SPEEDUINO_M5X_REV23_PIN_MAP.injector_pins[0],
    };

    assert_eq!(batch.push(item), Ok(()));
    assert_eq!(batch.push(item), Err(()));
}

#[test]
fn speeduino_m5x_named_board_selection_rejects_other_board_metadata() {
    let mut recipe = FirmwareRecipe::full_ecu(m50b25tu_full_cop_runtime_profile())
        .for_board(BoardSelection::Named("other-atmega2560-board"));
    recipe.subsystems.persistence = false;
    recipe.safety.watchdog_required = false;

    assert_eq!(
        recipe.resolve_build_plan(SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA),
        Err(RecipeValidationError::BoardSelectionMismatch {
            requested: BoardId::new("other-atmega2560-board"),
            actual: BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560,
        })
    );
}

#[test]
fn speeduino_m5x_expert_manual_authority_emits_six_injection_and_six_ignition_windows() {
    let profile = m50b25tu_speeduino_m5x_rev23_board_profile();
    let mut bridge = Atmega2560BoardAdapter::new(profile);
    let output = bridge
        .step(Atmega2560StepInput::bench_synced(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(800),
        ))
        .unwrap();

    assert_eq!(output.outputs.len(), 24);
    assert!(!output.cancel_scheduled_outputs);
    assert_eq!(output.telemetry.profile_id, profile.profile_id);
    assert_eq!(
        output.telemetry.ignition_profile_mode,
        profile.ignition_profile_mode
    );
    assert_eq!(
        output.telemetry.snapshot.engine_time.source(),
        AbsoluteTimeAuthority::ExpertManual
    );
    assert!(
        output
            .telemetry
            .snapshot
            .engine_time
            .full_sequential_authorized
    );
    assert_eq!(output.telemetry.pin_map_id, profile.pin_map_id);
}

#[test]
fn speeduino_m5x_telemetry_ids_come_from_supplied_board_profile() {
    let profile = Atmega2560BoardProfile::new(
        m50b25tu_full_cop_runtime_profile(),
        ProfileId::new(0x1234),
        IgnitionProfileId::new(0x56),
        IgnitionProfileMode::SequentialCop,
        IgnitionProfileMode::Disabled,
        PinMapId::new(0x78),
        RuntimeBuildId::new(0x9a),
    );
    let mut bridge = Atmega2560BoardAdapter::new(profile);
    let output = bridge
        .step(Atmega2560StepInput::bench_synced(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(800),
        ))
        .unwrap();

    assert_eq!(output.telemetry.profile_id, profile.profile_id);
    assert_eq!(
        output.telemetry.ignition_profile_id,
        profile.ignition_profile_id
    );
    assert_eq!(
        output.telemetry.ignition_profile_mode,
        profile.ignition_profile_mode
    );
    assert_eq!(output.telemetry.pin_map_id, profile.pin_map_id);
    assert_eq!(output.telemetry.runtime_build_id, profile.runtime_build_id);
}

#[test]
fn speeduino_m5x_synced_step_outputs_all_map_to_schematic_pins() {
    let mut bridge = Atmega2560BoardAdapter::m50b25tu_speeduino_m5x_rev23();
    let output = bridge
        .step(Atmega2560StepInput::bench_synced(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(800),
        ))
        .unwrap();

    let mut injector_seen = [false; 6];
    let mut ignition_seen = [false; 6];
    for transition in output.outputs.iter() {
        let pin = SPEEDUINO_M5X_REV23_PIN_MAP.output_pin(transition.output);
        assert!(pin.is_some());

        match transition.output {
            EcuOutput::Injector(channel) => {
                injector_seen[channel.get() as usize] = true;
            }
            EcuOutput::Ignition(channel) => {
                ignition_seen[channel.get() as usize] = true;
            }
        }
    }

    assert_eq!(injector_seen, [true; 6]);
    assert_eq!(ignition_seen, [true; 6]);
}

#[test]
fn speeduino_m5x_unsynced_step_emits_no_scheduled_windows() {
    let mut bridge = Atmega2560BoardAdapter::m50b25tu_speeduino_m5x_rev23();
    let input =
        Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
            .with_engine_time_authority(EngineTimeAuthority::none());

    let output = bridge.step(input).unwrap();
    assert_eq!(output.outputs.len(), 0);
}

#[test]
fn speeduino_m5x_geometry_only_authority_emits_no_full_sequential_cop_output() {
    let profile = m50b25tu_speeduino_m5x_rev23_board_profile();
    let mut bridge = Atmega2560BoardAdapter::new(profile);
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    let input =
        Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
            .with_engine_time_authority(authority);

    let output = bridge.step(input).unwrap();

    assert_eq!(output.outputs.len(), 0);
    assert_eq!(
        output.telemetry.ignition_profile_mode,
        profile.ignition_profile_authority_blocked_mode
    );
    assert_eq!(
        output.telemetry.snapshot.engine_time.source(),
        AbsoluteTimeAuthority::GeometryOnly
    );
    assert_eq!(
        output.telemetry.snapshot.engine_time.summary,
        ecu_domain::SyncState::Locked { cam_ref: false }
    );
    assert!(
        !output
            .telemetry
            .snapshot
            .engine_time
            .full_sequential_authorized
    );
}

#[test]
fn speeduino_m5x_explicit_engine_time_authority_overrides_conflicting_legacy_flags() {
    let mut bridge = Atmega2560BoardAdapter::m50b25tu_speeduino_m5x_rev23();
    let input =
        Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
            .with_engine_time_authority(EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CamValidated720,
                AbsoluteTimeAuthority::ExpertManual,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            ));

    assert_eq!(
        input.engine_time_authority.absolute,
        AbsoluteTimeAuthority::ExpertManual
    );

    let output = bridge.step(input).unwrap();

    assert_eq!(
        output.telemetry.snapshot.engine_time.source(),
        AbsoluteTimeAuthority::ExpertManual
    );
    assert!(
        output
            .telemetry
            .snapshot
            .engine_time
            .full_sequential_authorized
    );
}

#[test]
fn speeduino_m5x_explicit_engine_time_authority_is_preserved() {
    let authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamObserved720,
        AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    );
    let input =
        Atmega2560StepInput::bench_synced(Micros::new(1_000), Rpm::new(3_000), Kpa10::new(800))
            .with_engine_time_authority(authority);

    assert_eq!(input.engine_time_authority, authority);
}
