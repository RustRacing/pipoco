#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod authority;
pub mod diag;
mod ids;
mod modes;
mod units;
mod vehicle_speed;

pub use authority::{
    AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, EngineTimeAuthorityError,
    PhaseSyncState,
};
pub use ids::{ChannelId, CylinderId, IdentifierError};
pub use modes::{
    CancelReason, CommitPolicy, ControlMode, EnginePhase, FaultCode, FaultSeverity, SyncState,
};
pub use units::{
    CamPhaseDeg10, Degrees10, DwellUs, KnockLevelX100, Kpa10, Lambda100, MassAirFlowX100, Micros,
    Percent, PulseWidthUs, Rpm, Ticks, UnitRangeError, VehicleSpeedKph10,
};
pub use vehicle_speed::{
    vehicle_speed_pulse_step, VehicleSpeedPulseConfig, VehicleSpeedPulseConfigError,
    VehicleSpeedPulseInput, VehicleSpeedPulseResult, VehicleSpeedPulseState,
};
