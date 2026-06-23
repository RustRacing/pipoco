#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod cranking;
mod decel;
mod enrichment;
mod fuel;
mod ignition;
mod lambda;
#[cfg(test)]
mod tests;
mod torque;
mod types;

pub use cranking::{CrankingGate, CRANKING_EXIT_RPM, CRANKING_RPM_THRESHOLD};
pub use decel::DecelFuelCutState;
pub use enrichment::{
    AccelerationConfig, AccelerationState, AfterStartConfig, AfterStartState, EnrichmentController,
    EnrichmentInputs, EnrichmentResult, StartupConfig, StartupState, WarmupConfig,
};
pub use fuel::BaseFuelModel;
pub use ignition::{
    DwellConfig, IgnitionInputs, IgnitionLimitReason, IgnitionPlan, IgnitionPlanner,
};
pub use lambda::{
    LambdaDisableReason, LambdaMode, LambdaTrimConfig, LambdaTrimInputs, LambdaTrimPlanner,
    LambdaTrimResult,
};
pub use torque::{AllowedTorque, TorqueArbiter, TorqueInputs, TorqueLimitReason};
pub use types::{
    FuelAfrOverride, FuelAfterstartWindowMode, FuelEngineMode, FuelInputSnapshot, FuelIntent,
    FuelLoadSource, FuelObservations, FuelStartupWindowMode, FuelWarmupTemperatureMode,
};
