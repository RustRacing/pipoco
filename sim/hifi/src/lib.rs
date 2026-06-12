#![doc = "Host-only high-fidelity engine plant model."]

pub mod cylinder;
pub mod events;
pub mod export;
pub mod flow;
pub mod geometry;
pub mod observers;
pub mod params;
pub mod state;
pub mod thermo;

pub use crate::cylinder::{
    advance_plant_step, converge_open_system_cycles, run_fired_cycle, run_motored_cycle,
    run_motored_cycle_with_heat_loss, run_open_system_cycle,
};
pub use crate::events::{CycleEvent, CycleEventKind, EventTable};
pub use crate::export::{
    default_burn_model_export_config, default_loss_config_export,
    default_open_system_export_config, default_plant_config, export_ve_table, fit_loss_config,
    format_burn_curve, format_loss_config, format_ve_table, generate_default_artifacts,
    parse_burn_curve, parse_loss_config, parse_ve_table, write_default_artifacts, ExportError,
    ExportedBurnCurve, ExportedLossConfig, ExportedVeTable, GeneratedArtifacts, LossFitSample,
    PumpingFitSample, ARTIFACT_FORMAT_VERSION, BURN_CURVE_POINTS, DEFAULT_VE_LOAD_AXIS_KPA10,
    DEFAULT_VE_RPM_AXIS, VE_TABLE_AXIS_POINTS,
};
pub use crate::geometry::GeometryModel;
pub use crate::observers::estimate_exhaust_from_open_cycle;
pub use crate::observers::{estimate_exhaust, estimate_knock, observe_lambda};
pub use crate::params::{
    BurnModel, CombustionConfig, CylinderGeometry, FiredCycleConfig, FiredCycleConfigError,
    GasProperties, InitialChargeState, InjectorConfig, IntegratorConfig, KnockModelConfig,
    LossCorrelationConfig, ManifoldConfig, MotoredConfigError, MotoredCylinderConfig,
    OpenSystemConfig, OpenSystemConfigError, PlantConfig, PlantConfigError, PlantCylinderConfig,
    ResidualConfig, SparkConfig, ThermalBoundary, ThrottleConfig, ValveTiming, WoschniConfig,
};
pub use crate::state::{
    ClosedSystemState, CompositionState, ConvergedOpenSystemResult, CycleConvergence,
    CylinderCommand, CylinderPlantTrace, ExhaustObservation, FiredCycleResult, FiredCycleSample,
    KnockObservation, ManifoldState, MotoredCycleResult, MotoredSample, OpenSystemCycleResult,
    OpenSystemCylinderState, OpenSystemSample, PlantStepInput, PlantStepOutput,
};
