use super::dto::*;
use super::registry::*;
use super::stores::*;

mod diagnostics_snapshot;
mod enrichment;
mod expert_trigger;
mod fuel_tune;
mod sensors_limits_angles;
mod tables;

pub use diagnostics_snapshot::*;
pub use enrichment::*;
pub use expert_trigger::*;
pub use fuel_tune::*;
pub use sensors_limits_angles::*;
pub use tables::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PageCodecError {
    WrongSize,
    Invalid,
}

pub(super) const AFR_TARGET_MIN_X10: u16 = 100;
pub(super) const AFR_TARGET_MAX_X10: u16 = 220;
pub(super) const IDLE_DUTY_MAX_X10: u16 = 1000;
pub(super) const VE_TUNE_LOAD_SOURCE_MAX: u8 = 1;
pub(super) const LIMITS_TRIGGER_MAP_BIT: u8 = 0b01;
pub(super) const LIMITS_TRIGGER_TPS_BIT: u8 = 0b10;
pub(super) const LIMITS_TRIGGER_BITS_MASK: u8 = LIMITS_TRIGGER_MAP_BIT | LIMITS_TRIGGER_TPS_BIT;
pub(super) const EXPERT_SCHEMA_VERSION_CURRENT: u16 = 1;
pub(super) const EXPERT_UNLOCK_LOCKED: u8 = 0;
pub(super) const EXPERT_UNLOCK_UNLOCKED: u8 = 1;
pub(super) const TRIGGER_AUTHORITY_EXPERT_MANUAL: u8 = 1;
pub(super) const TRIGGER_AUTHORITY_CERTIFIED_PROFILE: u8 = 3;
pub(super) const TRIGGER_PATTERN_MISSING_TOOTH: u8 = 0;
pub(super) const SECONDARY_TRIGGER_NONE: u8 = 0;
pub(super) const EXPERT_IGNITION_SEQUENTIAL_COP: u8 = 3;
pub(super) const EXPERT_INJECTION_SEQUENTIAL: u8 = 4;
pub(super) const FIXED_TIMING_FIXED: u8 = 1;
pub const VE_TUNE_INJECTOR_DEADTIME_MAX_US: u16 = 10_000;
pub const SENSOR_ADC_INPUT_MAX_MV: u16 = 5000;
