use super::*;
use ecu_board_api::BoardCapabilities;
use ecu_domain::AbsoluteTimeAuthority;
use ecu_domain::CylinderId;

use crate::profiles::m50b25tu::{
    m50_runtime_output_profile, M50B25TU_FULL_COP, M50B25TU_MEGA_COMPAT,
    M50_RUNTIME_AUX_SAFETY_PROFILE,
};

#[test]
fn profile_counts_and_firing_order_match_m50_facts() {
    let profile = M50B25TU_MEGA_COMPAT;

    assert_eq!(profile.engine.cylinders, 6);
    assert_eq!(
        profile.engine.firing_order,
        [
            CylinderId::new(1),
            CylinderId::new(5),
            CylinderId::new(3),
            CylinderId::new(6),
            CylinderId::new(2),
            CylinderId::new(4),
        ]
    );
    assert!(matches!(
        profile.trigger.pattern,
        TriggerPattern::MissingTooth {
            nominal_teeth: 60,
            missing_teeth: 2,
        }
    ));
    assert_eq!(profile.cam.cams, 1);
    assert!(profile.cam.phase_required_for_sequential);
    assert_eq!(profile.injection.channels, 6);
}

#[test]
fn ignition_profile_splits_mega_compat_from_full_cop() {
    assert_eq!(M50B25TU_MEGA_COMPAT.name, "M50B25TU_MEGA_COMPAT");
    assert_eq!(M50B25TU_FULL_COP.name, "M50B25TU_FULL_COP");
    assert_eq!(M50B25TU_FULL_COP.trigger, M50B25TU_MEGA_COMPAT.trigger);
    assert!(matches!(
        M50B25TU_MEGA_COMPAT.ignition.topology,
        IgnitionTopology::WastedSpark { coils: 3 }
    ));
    assert!(matches!(
        M50B25TU_FULL_COP.ignition.topology,
        IgnitionTopology::CoilOnPlug { coils: 6 }
    ));
}

#[test]
fn aux_outputs_cover_required_m50_roles() {
    let outputs = M50B25TU_MEGA_COMPAT.aux.outputs;
    assert!(outputs.contains(&AuxOutputRole::VvtIntake));
    assert!(outputs.contains(&AuxOutputRole::IdleOpen));
    assert!(outputs.contains(&AuxOutputRole::IdleClose));
    assert!(outputs.contains(&AuxOutputRole::FuelPump));
    assert!(outputs.contains(&AuxOutputRole::Fan));
    assert!(outputs.contains(&AuxOutputRole::TachOut));
    assert!(outputs.contains(&AuxOutputRole::Cel));
    assert!(outputs.contains(&AuxOutputRole::Boost));
    assert!(outputs.contains(&AuxOutputRole::Disa));
    assert!(outputs.contains(&AuxOutputRole::Spare(1)));
    assert!(outputs.contains(&AuxOutputRole::Spare(2)));
}

#[test]
fn hardware_bindings_cover_m50_sensor_roles_without_pin_claims() {
    let bindings = M50B25TU_MEGA_COMPAT.hardware_map.bindings;

    assert!(bindings.iter().any(|b| b.logical_role == "crank-sensor"));
    assert!(bindings.iter().any(|b| b.logical_role == "cam-sensor"));
    assert!(bindings.iter().any(|b| b.logical_role == "map-sensor"));
    assert!(bindings.iter().any(|b| b.logical_role == "tps"));
    assert!(bindings.iter().any(|b| b.logical_role == "clt"));
    assert!(bindings.iter().any(|b| b.logical_role == "iat"));
    assert!(bindings.iter().any(|b| b.logical_role == "maf-hfm"));
    assert!(bindings.iter().any(|b| b.logical_role == "vbatt"));
    assert!(bindings.iter().any(|b| b.logical_role == "lambda"));
    assert!(bindings.iter().any(|b| b.logical_role == "vss-pulse"));
    assert!(bindings.iter().any(|b| b.logical_role == "knock-input-1"));
    assert!(bindings.iter().any(|b| b.logical_role == "knock-input-2"));
    assert!(bindings.iter().all(|b| !b.symbolic_target.is_empty()));
}

#[test]
fn map_profile_keeps_board_sensor_options_without_guessing_installation() {
    let map = M50B25TU_MEGA_COMPAT.sensor_scaling.map;

    assert_eq!(map.role, MapSensorRole::FirstRunSpeedDensity);
    assert!(map.candidates.contains(&Some(MapSensorModel::Mpxh6400ac6u)));
    assert!(map.candidates.contains(&Some(MapSensorModel::Mpx5700ap)));
}

#[test]
fn sensor_inventory_separates_factory_sensors_from_board_added_map() {
    let entries = M50B25TU_MEGA_COMPAT.sensor_inventory.entries;
    let find = |role| {
        entries
            .iter()
            .find(|entry| entry.role == role)
            .copied()
            .expect("sensor inventory role")
    };

    assert_eq!(
        find(SensorInventoryRole::Crank).support,
        SensorSupport::RequiredForSync
    );
    assert_eq!(
        find(SensorInventoryRole::Cam).support,
        SensorSupport::RequiredForSync
    );
    assert_eq!(
        find(SensorInventoryRole::Tps).presence,
        SensorPresence::FactoryEngine
    );
    assert_eq!(
        find(SensorInventoryRole::Maf).support,
        SensorSupport::RuntimeOptional
    );
    assert_eq!(
        find(SensorInventoryRole::Map).presence,
        SensorPresence::BoardAdded
    );
    assert_eq!(
        find(SensorInventoryRole::Map).support,
        SensorSupport::RuntimeInput
    );
    assert_eq!(
        find(SensorInventoryRole::Baro).presence,
        SensorPresence::Derived
    );
}

#[test]
fn provenance_defaults_stay_symbolic_and_reference_the_speeduino_class() {
    let provenance = M50B25TU_MEGA_COMPAT.hardware_map.provenance;
    assert!(matches!(
        provenance.origin,
        HardwareMapOrigin::SymbolicCompatibilitySketch
    ));
    assert!(matches!(
        provenance.mapping_style,
        HardwareMappingStyle::Symbolic
    ));
    assert_eq!(provenance.reference_board, "Speeduino-M5x Rev 2.3 class");
    assert_eq!(
        provenance.notes,
        "Symbolic compatibility record for an M50B25TU board profile."
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.trigger.primary_speed,
        TriggerSpeed::Crank
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.trigger.secondary.mode,
        SecondaryTriggerMode::SingleToothCam
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.cam.sensor_default,
        CamSensorDefault::VrConditionedOrHallJumper
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.clt,
        SensorScaling::BmwM50Clt
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.iat,
        SensorScaling::BmwM50Iat
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.map.role,
        MapSensorRole::FirstRunSpeedDensity
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.map.candidates,
        [
            Some(MapSensorModel::Mpxh6400ac6u),
            Some(MapSensorModel::Mpx5700ap)
        ]
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.baro.source,
        BaroSourceRole::StartupMapSample
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.tps,
        SensorScaling::ThrottlePositionVoltage
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.maf,
        SensorScaling::BmwM50Hfm
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.vbatt,
        SensorScaling::VBattDivider
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.lambda,
        SensorScaling::LambdaInputSelection
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.vss,
        SensorScaling::VehicleSpeedPulse
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.knock_front,
        SensorScaling::BmwM50KnockWindowed
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.knock_rear,
        SensorScaling::BmwM50KnockWindowed
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.crank,
        SensorScaling::VrConditionedByDefault
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.sensor_scaling.cam,
        SensorScaling::VrConditionedOrHallJumper
    );
}

#[test]
fn m50_trigger_profiles_validate_without_absolute_authority() {
    assert_eq!(M50B25TU_MEGA_COMPAT.trigger.validate(), Ok(()));
    assert_eq!(M50B25TU_FULL_COP.trigger.validate(), Ok(()));
    assert_eq!(
        TriggerAngleAuthority::Unknown.absolute_authority(),
        AbsoluteTimeAuthority::None
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT
            .trigger
            .pattern
            .observed_primary_teeth(),
        Some(58)
    );
    assert_eq!(
        M50B25TU_MEGA_COMPAT.trigger.trigger_angle_atdc_deg10,
        TriggerAngleAuthority::Unknown
    );
    assert!(!matches!(
        M50B25TU_MEGA_COMPAT.trigger.trigger_angle_atdc_deg10,
        TriggerAngleAuthority::CertifiedProfile(_)
    ));
    assert_eq!(
        M50B25TU_MEGA_COMPAT.trigger.declared_absolute_authority(),
        AbsoluteTimeAuthority::None
    );
    assert_ne!(
        M50B25TU_MEGA_COMPAT.trigger.declared_absolute_authority(),
        AbsoluteTimeAuthority::CertifiedProfile
    );
    assert_ne!(
        M50B25TU_FULL_COP.trigger.declared_absolute_authority(),
        AbsoluteTimeAuthority::CertifiedProfile
    );
}

#[test]
fn cam_phase_remains_required_for_sequential_m50_profiles() {
    let mega = M50B25TU_MEGA_COMPAT;
    let full_cop = M50B25TU_FULL_COP;

    assert_eq!(mega.injection.channels, 6);
    assert!(mega.cam.phase_required_for_sequential);
    assert_eq!(mega.cam.phase_edge_action, CamPhaseEdgeAction::SetPhaseA);
    assert_eq!(full_cop.injection.channels, 6);
    assert!(full_cop.cam.phase_required_for_sequential);
    assert_eq!(
        full_cop.cam.phase_edge_action,
        CamPhaseEdgeAction::SetPhaseA
    );
}

#[test]
fn m50_runtime_output_profiles_preserve_board_profile_topology() {
    let mega = m50_runtime_output_profile(M50B25TU_MEGA_COMPAT);
    let full_cop = m50_runtime_output_profile(M50B25TU_FULL_COP);

    assert_eq!(
        mega.authority,
        ecu_board_api::OutputAuthorityRequirement::FullSequential720
    );
    assert_eq!(
        full_cop.authority,
        ecu_board_api::OutputAuthorityRequirement::FullSequential720
    );
    assert_eq!(mega.aux_safety, M50_RUNTIME_AUX_SAFETY_PROFILE);
    assert_eq!(full_cop.aux_safety, M50_RUNTIME_AUX_SAFETY_PROFILE);
    assert!(matches!(
        mega.ignition,
        ecu_board_api::IgnitionOutputProfile::WastedSpark {
            cylinders: 6,
            coils: 3,
        }
    ));
    assert!(matches!(
        full_cop.ignition,
        ecu_board_api::IgnitionOutputProfile::CoilOnPlug {
            cylinders: 6,
            coils: 6,
            phase_required: true,
        }
    ));
    assert!(matches!(
        full_cop.injection,
        ecu_board_api::InjectionOutputProfile::Sequential {
            cylinders: 6,
            channels: 6,
            phase_required: true,
            ..
        }
    ));
}

fn complete_board_for(profile: EngineBoardProfile) -> BoardFirstStartCapabilities {
    BoardFirstStartCapabilities {
        sensors: BoardFirstStartSensorCapabilities {
            crank: true,
            cam: true,
            map: true,
            tps: true,
            clt: true,
            iat: true,
            maf: true,
            vbatt: true,
            lambda: true,
            vss: true,
            knock_front: true,
            knock_rear: true,
            baro: true,
        },
        outputs: BoardFirstStartOutputCapabilities {
            injector_outputs: profile.injection.channels,
            ignition_outputs: match profile.ignition.topology {
                IgnitionTopology::WastedSpark { coils }
                | IgnitionTopology::CoilOnPlug { coils } => coils,
            },
            aux_outputs: profile.aux.outputs.map(Some),
        },
        safety: BoardFirstStartSafetyCapabilities {
            sync_loss_cuts_fuel: true,
            sync_loss_cuts_ignition: true,
            trigger_angle_has_authority: true,
        },
    }
}

#[test]
fn board_capabilities_to_first_start_maps_only_recipe_owned_truth() {
    let capabilities = BoardCapabilities::new(true, true, true, 4, 6, 9, true, true, true)
        .with_load_sources(ecu_board_api::LoadSourceCapabilities::new(
            true, true, true, true,
        ));

    let board = BoardFirstStartCapabilities::from_board_capabilities(capabilities);
    let from_trait: BoardFirstStartCapabilities = capabilities.into();

    assert_eq!(board, from_trait);
    assert!(board.sensors.crank);
    assert!(board.sensors.cam);
    assert!(board.sensors.map);
    assert!(board.sensors.maf);
    assert!(board.sensors.tps);
    assert!(board.sensors.lambda);
    assert_eq!(board.outputs.injector_outputs, 6);
    assert_eq!(board.outputs.ignition_outputs, 4);

    assert!(!board.sensors.clt);
    assert!(!board.sensors.iat);
    assert!(!board.sensors.vbatt);
    assert!(!board.sensors.baro);
    assert!(!board.sensors.knock_front);
    assert!(!board.sensors.knock_rear);
    assert!(!board.sensors.vss);
    assert_eq!(board.outputs.aux_outputs, [None; 11]);
    assert_eq!(board.safety, BoardFirstStartSafetyCapabilities::default());
}

#[test]
fn first_start_overlay_adds_evidence_without_replacing_board_capabilities() {
    let capabilities = BoardCapabilities::new(true, false, false, 2, 4, 0, false, true, true)
        .with_load_sources(ecu_board_api::LoadSourceCapabilities::new(
            true, false, true, false,
        ));

    let board = BoardFirstStartCapabilities::from_board_capabilities(capabilities)
        .with_sensor_evidence(SensorInventoryRole::Clt)
        .with_sensor_evidence(SensorInventoryRole::Iat)
        .with_sensor_evidence(SensorInventoryRole::Vbatt)
        .with_sensor_evidence(SensorInventoryRole::Vss)
        .with_sensor_evidence(SensorInventoryRole::KnockFront)
        .with_sensor_evidence(SensorInventoryRole::KnockRear)
        .with_sensor_evidence(SensorInventoryRole::Baro)
        .with_aux_output_evidence(0, AuxOutputRole::FuelPump)
        .with_aux_output_evidence(10, AuxOutputRole::Fan)
        .with_aux_output_evidence(11, AuxOutputRole::Cel)
        .with_safety_evidence(BoardFirstStartSafetyCapabilities {
            sync_loss_cuts_fuel: true,
            sync_loss_cuts_ignition: true,
            trigger_angle_has_authority: true,
        });

    assert!(board.sensors.crank);
    assert!(!board.sensors.cam);
    assert!(board.sensors.map);
    assert!(!board.sensors.maf);
    assert!(board.sensors.tps);
    assert!(!board.sensors.lambda);
    assert_eq!(board.outputs.injector_outputs, 4);
    assert_eq!(board.outputs.ignition_outputs, 2);

    assert!(board.sensors.clt);
    assert!(board.sensors.iat);
    assert!(board.sensors.vbatt);
    assert!(board.sensors.vss);
    assert!(board.sensors.knock_front);
    assert!(board.sensors.knock_rear);
    assert!(board.sensors.baro);
    assert_eq!(board.outputs.aux_outputs[0], Some(AuxOutputRole::FuelPump));
    assert_eq!(board.outputs.aux_outputs[10], Some(AuxOutputRole::Fan));
    assert!(!board.outputs.supports_aux(AuxOutputRole::Cel));
    assert_eq!(
        board.safety,
        BoardFirstStartSafetyCapabilities {
            sync_loss_cuts_fuel: true,
            sync_loss_cuts_ignition: true,
            trigger_angle_has_authority: true,
        }
    );
}

#[test]
fn conservative_first_start_preset_is_profile_derived_not_target_specific() {
    let profile = M50B25TU_MEGA_COMPAT;
    let preset = conservative_first_start_preset(&profile, FirstStartLoadSource::MapSpeedDensity);

    assert_eq!(preset.required_injector_outputs, profile.injection.channels);
    assert_eq!(preset.required_ignition_outputs, 3);
    assert!(preset
        .required_sensor_roles
        .contains(&Some(SensorInventoryRole::Crank)));
    assert!(preset
        .required_sensor_roles
        .contains(&Some(SensorInventoryRole::Cam)));
    assert!(preset
        .required_sensor_roles
        .contains(&Some(SensorInventoryRole::Map)));
    assert!(preset
        .required_aux_outputs
        .contains(&Some(AuxOutputRole::FuelPump)));
    assert!(preset.requires_trigger_angle_authority);
}

#[test]
fn profile_board_compatibility_accepts_complete_capabilities() {
    let profile = M50B25TU_MEGA_COMPAT;
    let preset = conservative_first_start_preset(&profile, FirstStartLoadSource::MapSpeedDensity);
    let board = complete_board_for(profile);

    let report = check_profile_board_compatibility(&profile, &preset, &board);

    assert!(report.ready(), "{:?}", report);
    assert_eq!(report.issue_count(), 0);
    assert!(report.issues().is_empty());
}

#[test]
fn profile_board_compatibility_reports_missing_sensors_and_outputs() {
    let profile = M50B25TU_MEGA_COMPAT;
    let preset = conservative_first_start_preset(&profile, FirstStartLoadSource::MapSpeedDensity);
    let mut board = complete_board_for(profile);
    board.sensors.map = false;
    board.outputs.injector_outputs = 2;
    board.outputs.ignition_outputs = 2;
    board.safety.trigger_angle_has_authority = false;

    let report = check_profile_board_compatibility(&profile, &preset, &board);

    assert!(!report.ready());
    assert!(report
        .issues()
        .contains(&Some(ProfileCompatibilityIssue::MissingSensor(
            SensorInventoryRole::Map
        ))));
    assert!(report
        .issues()
        .contains(&Some(ProfileCompatibilityIssue::InjectorOutputCount {
            required: 6,
            available: 2,
        })));
    assert!(report
        .issues()
        .contains(&Some(ProfileCompatibilityIssue::IgnitionOutputCount {
            required: 3,
            available: 2,
        })));
    assert!(report.issues().contains(&Some(
        ProfileCompatibilityIssue::MissingTriggerAngleAuthority
    )));
}

#[test]
fn profile_compatibility_report_keeps_bounded_issue_count_on_overflow() {
    let profile = M50B25TU_MEGA_COMPAT;
    let mut preset =
        conservative_first_start_preset(&profile, FirstStartLoadSource::MapSpeedDensity);
    preset.required_sensor_roles = [Some(SensorInventoryRole::Map); 8];
    preset.required_aux_outputs = [Some(AuxOutputRole::FuelPump); 4];
    preset.required_injector_outputs = 16;
    preset.required_ignition_outputs = 16;
    preset.requires_trigger_angle_authority = true;
    preset.requires_sync_loss_fuel_cut = true;
    preset.requires_sync_loss_ignition_cut = true;

    let board = BoardFirstStartCapabilities {
        sensors: BoardFirstStartSensorCapabilities::default(),
        outputs: BoardFirstStartOutputCapabilities {
            injector_outputs: 0,
            ignition_outputs: 0,
            aux_outputs: [None; 11],
        },
        safety: BoardFirstStartSafetyCapabilities {
            sync_loss_cuts_fuel: false,
            sync_loss_cuts_ignition: false,
            trigger_angle_has_authority: false,
        },
    };

    let report = check_profile_board_compatibility(&profile, &preset, &board);

    assert_eq!(report.issue_count(), ProfileCompatibilityReport::CAPACITY);
    assert_eq!(report.issues().len(), ProfileCompatibilityReport::CAPACITY);
    assert_eq!(
        report.issues().last(),
        Some(&Some(ProfileCompatibilityIssue::IssueOverflow))
    );
}
