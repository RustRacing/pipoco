use ecu_board_api::{
    AuxOutput, AuxSafetyProfile, FullEcuOutputProfile, OutputAuthorityRequirement,
};
use ecu_domain::{ChannelId, CylinderId, Micros};
use ecu_trigger::{
    EngineTimeLatency, PollLevelPolarity, ResyncPolicy, SecondaryTriggerMode,
    SecondaryTriggerProfile, StartupSyncPolicy, TriggerAngleAuthority, TriggerEdge, TriggerFilter,
    TriggerPattern, TriggerProfile, TriggerSpeed,
};

use crate::model::*;

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
        phase_edge_action: CamPhaseEdgeAction::SetPhaseA,
    },
    injection: InjectionProfile { channels: 6 },
    ignition: IgnitionProfile {
        topology: IgnitionTopology::WastedSpark { coils: 3 },
    },
    aux: AuxProfile {
        outputs: [
            AuxOutputRole::VvtIntake,
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
        map: MapSensorProfile {
            role: MapSensorRole::FirstRunSpeedDensity,
            candidates: [
                Some(MapSensorModel::Mpxh6400ac6u),
                Some(MapSensorModel::Mpx5700ap),
            ],
        },
        baro: BaroSensorProfile {
            source: BaroSourceRole::StartupMapSample,
        },
        tps: SensorScaling::ThrottlePositionVoltage,
        maf: SensorScaling::BmwM50Hfm,
        vbatt: SensorScaling::VBattDivider,
        lambda: SensorScaling::LambdaInputSelection,
        vss: SensorScaling::VehicleSpeedPulse,
        knock_front: SensorScaling::BmwM50KnockWindowed,
        knock_rear: SensorScaling::BmwM50KnockWindowed,
        crank: SensorScaling::VrConditionedByDefault,
        cam: SensorScaling::VrConditionedOrHallJumper,
    },
    sensor_inventory: SensorInventoryProfile {
        entries: M50B25TU_SENSOR_INVENTORY,
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

pub const M50B25TU_SENSOR_INVENTORY: [SensorInventoryEntry; 13] = [
    SensorInventoryEntry {
        role: SensorInventoryRole::Crank,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RequiredForSync,
        notes: "60-2 crank sensor; required before any fuel or ignition scheduling.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Cam,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RequiredForSync,
        notes: "Single intake cam phase input; required for full sequential phase authority.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Tps,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RuntimeInput,
        notes: "Throttle position input for transient logic and alpha-N fallback.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Clt,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RuntimeInput,
        notes: "Coolant temperature input for warmup, protection, and plausibility.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Iat,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RuntimeInput,
        notes: "Intake air temperature input for air-density and plausibility.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Maf,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RuntimeOptional,
        notes: "Stock HFM can be supported as a load source after bench-verified curve evidence.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Vbatt,
        presence: SensorPresence::FactoryHarnessOrChassis,
        support: SensorSupport::RuntimeInput,
        notes: "Battery voltage divider input for injector deadtime and safety diagnostics.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Lambda,
        presence: SensorPresence::FactoryHarnessOrChassis,
        support: SensorSupport::RuntimeOptional,
        notes: "Narrowband or external controller input; optional for first-run unless policy requires it.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::KnockFront,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RuntimeOptional,
        notes: "Factory knock sensor channel covering part of the inline-six; retard authority needs evidence.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::KnockRear,
        presence: SensorPresence::FactoryEngine,
        support: SensorSupport::RuntimeOptional,
        notes: "Factory knock sensor channel covering the remaining cylinders; retard authority needs evidence.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Vss,
        presence: SensorPresence::FactoryHarnessOrChassis,
        support: SensorSupport::RuntimeOptional,
        notes: "Vehicle-speed pulse input when wired; not required for first start.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Map,
        presence: SensorPresence::BoardAdded,
        support: SensorSupport::RuntimeInput,
        notes: "Board MAP input for speed-density first-run; supported models include MPXH6400AC6U and MPX5700AP.",
    },
    SensorInventoryEntry {
        role: SensorInventoryRole::Baro,
        presence: SensorPresence::Derived,
        support: SensorSupport::RuntimeOptional,
        notes: "Baro may be fixed, sampled from startup MAP, or routed to a second dedicated pressure sensor.",
    },
];

pub const M50B25TU_FULL_COP: EngineBoardProfile = EngineBoardProfile {
    name: "M50B25TU_FULL_COP",
    ignition: IgnitionProfile {
        topology: IgnitionTopology::CoilOnPlug { coils: 6 },
    },
    ..M50B25TU_MEGA_COMPAT
};

pub const M50_RUNTIME_AUX_SAFETY_PROFILE: AuxSafetyProfile = AuxSafetyProfile::off_on_limp([
    AuxOutput::Pwm(ChannelId::new(0)),
    AuxOutput::Digital(ChannelId::new(0)),
    AuxOutput::Digital(ChannelId::new(1)),
]);

pub const fn m50_runtime_output_profile(profile: EngineBoardProfile) -> FullEcuOutputProfile {
    match profile.ignition.topology {
        IgnitionTopology::WastedSpark { coils } => FullEcuOutputProfile::sequential_wasted_spark(
            profile.engine.firing_order,
            profile.injection.channels,
            coils,
            M50_RUNTIME_AUX_SAFETY_PROFILE,
            OutputAuthorityRequirement::FullSequential720,
        ),
        IgnitionTopology::CoilOnPlug { coils } => FullEcuOutputProfile::sequential_coil_on_plug(
            profile.engine.firing_order,
            profile.injection.channels,
            coils,
            M50_RUNTIME_AUX_SAFETY_PROFILE,
            OutputAuthorityRequirement::FullSequential720,
        ),
    }
}

pub const M50B25TU_HARDWARE_BINDINGS: [HardwareMapBinding; 28] = [
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
        logical_role: "map-sensor",
        symbolic_target: "board analog input: local MAP",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "tps",
        symbolic_target: "board analog input: throttle position",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "clt",
        symbolic_target: "board analog input: coolant temperature",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "iat",
        symbolic_target: "board analog input: intake air temperature",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "maf-hfm",
        symbolic_target: "board analog input: stock HFM",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "vbatt",
        symbolic_target: "board analog input: battery voltage divider",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "lambda",
        symbolic_target: "board analog/digital input: lambda controller",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "vss-pulse",
        symbolic_target: "board timer/gpio input: vehicle speed pulse",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "knock-input-1",
        symbolic_target: "board knock front-end channel 1",
        provenance: "semantic profile only",
    },
    HardwareMapBinding {
        logical_role: "knock-input-2",
        symbolic_target: "board knock front-end channel 2",
        provenance: "semantic profile only",
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
        logical_role: "aux-vvt-intake",
        symbolic_target: "logical aux output vvt intake",
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
