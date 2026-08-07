#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

mod angles;
mod authority;
mod constants;
pub mod diag;
mod ids;
mod modes;
mod units;
mod vehicle_speed;
pub mod voltage;

pub use angles::{
    cyc7200_distance, duration_us_to_deg10, forward_angle_delta_deg10, micros_for_angle_delta,
    norm7200, norm_deg10, CRANK_REV_DEGREES10, ENGINE_CYCLE_DEGREES10,
};
pub use authority::{
    AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, EngineTimeAuthorityError,
    PhaseSyncState,
};
pub use constants::{MAX_PULSE_WIDTH_US, RPM_CALC_NUMERATOR_EXACT, RPM_CALC_NUMERATOR_FAST};
pub use ids::{ChannelId, CylinderId, IdentifierError};
pub use modes::{
    CancelReason, CommitPolicy, ControlMode, EnginePhase, FaultCode, FaultSeverity, SyncState,
};
pub use units::{
    AfrX100, CamPhaseDeg10, Degrees10, DwellUs, KnockLevelX100, Kpa10, Lambda100, MassAirFlowX100,
    Micros, Millivolts, Percent, PulseWidthUs, RatioX1000, Rpm, TempC10, Ticks, UnitRangeError,
    VePctX100, VehicleSpeedKph10,
};
pub use vehicle_speed::{
    vehicle_speed_pulse_step, VehicleSpeedPulseConfig, VehicleSpeedPulseConfigError,
    VehicleSpeedPulseInput, VehicleSpeedPulseResult, VehicleSpeedPulseState,
};
