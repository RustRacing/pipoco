#![cfg_attr(not(test), no_std)]

use ecu_domain::{CylinderId, Micros};
pub use ecu_trigger::{
    EngineTimeLatency, PollLevelPolarity, ResyncPolicy, SecondaryTriggerMode,
    SecondaryTriggerProfile, StartupSyncPolicy, TriggerAngleAuthority, TriggerEdge, TriggerFilter,
    TriggerPattern, TriggerProfile, TriggerSpeed,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EngineBoardProfile {
    pub name: &'static str,
    pub engine: EngineProfile,
    pub trigger: TriggerProfile,
    pub cam: CamProfile,
    pub injection: InjectionProfile,
    pub ignition: IgnitionProfile,
    pub aux: AuxProfile,
    pub safety: SafetyProfile,
    pub sensor_scaling: SensorScalingProfile,
    pub hardware_map: HardwareMapProfile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EngineProfile {
    pub cylinders: u8,
    pub firing_order: [CylinderId; 6],
    pub cycle: EngineCycle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EngineCycle {
    FourStroke720,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CamProfile {
    pub cams: u8,
    pub sensor_default: CamSensorDefault,
    pub phase_required_for_sequential: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamSensorDefault {
    VrConditioned,
    VrConditionedOrHallJumper,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InjectionProfile {
    pub strategy: InjectionStrategy,
    pub channels: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InjectionStrategy {
    Sequential,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IgnitionProfile {
    pub topology: IgnitionTopology,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IgnitionTopology {
    WastedSpark { coils: u8 },
    CoilOnPlug { coils: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxProfile {
    pub outputs: [AuxOutputRole; 11],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AuxOutputRole {
    VanosIntake,
    IdleOpen,
    IdleClose,
    FuelPump,
    Fan,
    TachOut,
    Cel,
    Boost,
    Disa,
    Spare(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyProfile {
    pub sync_loss_cut_fuel: bool,
    pub sync_loss_cut_ignition: bool,
    pub safe_aux_outputs: [AuxOutputRole; 4],
    pub notes: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorScalingProfile {
    pub clt: SensorScaling,
    pub iat: SensorScaling,
    pub map: SensorScaling,
    pub vbatt: SensorScaling,
    pub lambda: SensorScaling,
    pub crank: SensorScaling,
    pub cam: SensorScaling,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorScaling {
    BmwM50Clt,
    BmwM50Iat,
    BoardLocalMap,
    VBattDivider,
    LambdaInputSelection,
    VrConditionedByDefault,
    VrConditionedOrHallJumper,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HardwareMapProfile {
    pub provenance: HardwareMapProvenance,
    pub bindings: [HardwareMapBinding; 18],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HardwareMapProvenance {
    pub origin: HardwareMapOrigin,
    pub mapping_style: HardwareMappingStyle,
    pub reference_board: &'static str,
    pub notes: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareMapOrigin {
    SymbolicCompatibilitySketch,
    SpeeduinoM5xRev23Reference,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HardwareMappingStyle {
    Symbolic,
    ProvenanceOriented,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HardwareMapBinding {
    pub logical_role: &'static str,
    pub symbolic_target: &'static str,
    pub provenance: &'static str,
}

pub const M50B25TU_MEGA_COMPAT: EngineBoardProfile = EngineBoardProfile {
    name: "M50B25TU_MEGA_COMPAT",
    engine: EngineProfile {
        cylinders: 6,
        firing_order: [
            CylinderId::new(1),
            CylinderId::new(5),
            CylinderId::new(3),
            CylinderId::new(6),
            CylinderId::new(2),
            CylinderId::new(4),
        ],
        cycle: EngineCycle::FourStroke720,
    },
    trigger: TriggerProfile {
        pattern: TriggerPattern::MissingTooth {
            nominal_teeth: 60,
            missing_teeth: 2,
        },
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary: SecondaryTriggerProfile {
            mode: SecondaryTriggerMode::SingleToothCam,
            edge: TriggerEdge::Falling,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        trigger_angle_atdc_deg10: TriggerAngleAuthority::Unknown,
        tooth_angle_multiplier: 1,
        filter: TriggerFilter::Weak,
        resync: ResyncPolicy::OnSyncLoss,
        startup: StartupSyncPolicy {
            skip_revolutions: 2,
            require_full_cycle: true,
        },
        latency: EngineTimeLatency {
            primary_edge_delay_us: Micros::new(0),
            secondary_edge_delay_us: Micros::new(0),
            output_schedule_delay_us: Micros::new(0),
        },
    },
    cam: CamProfile {
        cams: 1,
        sensor_default: CamSensorDefault::VrConditionedOrHallJumper,
        phase_required_for_sequential: true,
    },
    injection: InjectionProfile {
        strategy: InjectionStrategy::Sequential,
        channels: 6,
    },
    ignition: IgnitionProfile {
        topology: IgnitionTopology::WastedSpark { coils: 3 },
    },
    aux: AuxProfile {
        outputs: [
            AuxOutputRole::VanosIntake,
            AuxOutputRole::IdleOpen,
            AuxOutputRole::IdleClose,
            AuxOutputRole::FuelPump,
            AuxOutputRole::Fan,
            AuxOutputRole::TachOut,
            AuxOutputRole::Cel,
            AuxOutputRole::Boost,
            AuxOutputRole::Disa,
            AuxOutputRole::Spare(1),
            AuxOutputRole::Spare(2),
        ],
    },
    safety: SafetyProfile {
        sync_loss_cut_fuel: true,
        sync_loss_cut_ignition: true,
        safe_aux_outputs: [
            AuxOutputRole::FuelPump,
            AuxOutputRole::Fan,
            AuxOutputRole::TachOut,
            AuxOutputRole::Cel,
        ],
        notes: "Semantic safe-state guidance only; no pin-level safety claim.",
    },
    sensor_scaling: SensorScalingProfile {
        clt: SensorScaling::BmwM50Clt,
        iat: SensorScaling::BmwM50Iat,
        map: SensorScaling::BoardLocalMap,
        vbatt: SensorScaling::VBattDivider,
        lambda: SensorScaling::LambdaInputSelection,
        crank: SensorScaling::VrConditionedByDefault,
        cam: SensorScaling::VrConditionedOrHallJumper,
    },
    hardware_map: HardwareMapProfile {
        provenance: HardwareMapProvenance {
            origin: HardwareMapOrigin::SymbolicCompatibilitySketch,
            mapping_style: HardwareMappingStyle::Symbolic,
            reference_board: "Speeduino-M5x Rev 2.3 class",
            notes: "Symbolic compatibility record for an M50B25TU board profile.",
        },
        bindings: M50B25TU_HARDWARE_BINDINGS,
    },
};

pub const M50B25TU_FULL_COP: EngineBoardProfile = EngineBoardProfile {
    name: "M50B25TU_FULL_COP",
    ignition: IgnitionProfile {
        topology: IgnitionTopology::CoilOnPlug { coils: 6 },
    },
    ..M50B25TU_MEGA_COMPAT
};

pub const M50B25TU_HARDWARE_BINDINGS: [HardwareMapBinding; 18] = [
    HardwareMapBinding {
        logical_role: "crank-sensor",
        symbolic_target: "board input: crank sense",
        provenance: "speeduino-m5x compatibility sketch",
    },
    HardwareMapBinding {
        logical_role: "cam-sensor",
        symbolic_target: "board input: cam sense",
        provenance: "speeduino-m5x compatibility sketch",
    },
    HardwareMapBinding {
        logical_role: "injector-1",
        symbolic_target: "logical injector channel 1",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "injector-2",
        symbolic_target: "logical injector channel 2",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "injector-3",
        symbolic_target: "logical injector channel 3",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "injector-4",
        symbolic_target: "logical injector channel 4",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "injector-5",
        symbolic_target: "logical injector channel 5",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "injector-6",
        symbolic_target: "logical injector channel 6",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "ignition-1",
        symbolic_target: "logical ignition output 1",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "ignition-2",
        symbolic_target: "logical ignition output 2",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "ignition-3",
        symbolic_target: "logical ignition output 3",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "ignition-4",
        symbolic_target: "logical ignition output 4",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "ignition-5",
        symbolic_target: "logical ignition output 5",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "ignition-6",
        symbolic_target: "logical ignition output 6",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "aux-vanos-intake",
        symbolic_target: "logical aux output vanos intake",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "aux-fuel-pump",
        symbolic_target: "logical aux output fuel pump",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "aux-fan",
        symbolic_target: "logical aux output fan",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "aux-cel",
        symbolic_target: "logical aux output cel",
        provenance: "semantic profile only",
    },
];

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::AbsoluteTimeAuthority;

    #[test]
    fn profile_counts_and_firing_order_match_m50_facts() {
        assert_eq!(M50B25TU_MEGA_COMPAT.engine.cylinders, 6);
        assert_eq!(
            M50B25TU_MEGA_COMPAT.engine.firing_order,
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
            M50B25TU_MEGA_COMPAT.engine.cycle,
            EngineCycle::FourStroke720
        ));
        assert!(matches!(
            M50B25TU_MEGA_COMPAT.trigger.pattern,
            TriggerPattern::MissingTooth {
                nominal_teeth: 60,
                missing_teeth: 2,
            }
        ));
        assert_eq!(M50B25TU_MEGA_COMPAT.cam.cams, 1);
        assert!(M50B25TU_MEGA_COMPAT.cam.phase_required_for_sequential);
        assert_eq!(M50B25TU_MEGA_COMPAT.injection.channels, 6);
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
        assert!(outputs.contains(&AuxOutputRole::VanosIntake));
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
            M50B25TU_MEGA_COMPAT.sensor_scaling.map,
            SensorScaling::BoardLocalMap
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
        assert!(matches!(
            M50B25TU_MEGA_COMPAT.injection.strategy,
            InjectionStrategy::Sequential
        ));
        assert!(M50B25TU_MEGA_COMPAT.cam.phase_required_for_sequential);
        assert!(matches!(
            M50B25TU_FULL_COP.injection.strategy,
            InjectionStrategy::Sequential
        ));
        assert!(M50B25TU_FULL_COP.cam.phase_required_for_sequential);
    }
}
