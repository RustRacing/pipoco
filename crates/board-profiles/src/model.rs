use ecu_domain::CylinderId;
use ecu_trigger::TriggerProfile;

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
    pub sensor_inventory: SensorInventoryProfile,
    pub hardware_map: HardwareMapProfile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EngineProfile {
    pub cylinders: u8,
    pub firing_order: [CylinderId; 6],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CamProfile {
    pub cams: u8,
    pub sensor_default: CamSensorDefault,
    pub phase_required_for_sequential: bool,
    pub phase_edge_action: CamPhaseEdgeAction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamSensorDefault {
    VrConditioned,
    VrConditionedOrHallJumper,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamPhaseEdgeAction {
    SetPhaseA,
    SetPhaseB,
    Toggle,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InjectionProfile {
    pub channels: u8,
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
    VvtIntake,
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
    pub map: MapSensorProfile,
    pub baro: BaroSensorProfile,
    pub tps: SensorScaling,
    pub maf: SensorScaling,
    pub vbatt: SensorScaling,
    pub lambda: SensorScaling,
    pub vss: SensorScaling,
    pub knock_front: SensorScaling,
    pub knock_rear: SensorScaling,
    pub crank: SensorScaling,
    pub cam: SensorScaling,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MapSensorProfile {
    pub role: MapSensorRole,
    pub candidates: [Option<MapSensorModel>; 2],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BaroSensorProfile {
    pub source: BaroSourceRole,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BaroSourceRole {
    FixedKpa,
    StartupMapSample,
    DedicatedSensor,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapSensorRole {
    FirstRunSpeedDensity,
    OptionalBaroOrDiagnostics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapSensorModel {
    Mpxh6400a,
    Mpxh6400ac6u,
    Mpx5700ap,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorScaling {
    BmwM50Clt,
    BmwM50Iat,
    ThrottlePositionVoltage,
    BmwM50Hfm,
    BmwM50KnockWindowed,
    VehicleSpeedPulse,
    VBattDivider,
    LambdaInputSelection,
    VrConditionedByDefault,
    VrConditionedOrHallJumper,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorInventoryProfile {
    pub entries: [SensorInventoryEntry; 13],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorInventoryEntry {
    pub role: SensorInventoryRole,
    pub presence: SensorPresence,
    pub support: SensorSupport,
    pub notes: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorInventoryRole {
    Crank,
    Cam,
    Tps,
    Clt,
    Iat,
    Maf,
    Vbatt,
    Lambda,
    KnockFront,
    KnockRear,
    Vss,
    Map,
    Baro,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorPresence {
    FactoryEngine,
    FactoryHarnessOrChassis,
    BoardAdded,
    Derived,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorSupport {
    RequiredForSync,
    RuntimeInput,
    RuntimeOptional,
    EvidenceOnly,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HardwareMapProfile {
    pub provenance: HardwareMapProvenance,
    pub bindings: [HardwareMapBinding; 28],
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
