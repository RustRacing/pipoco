//! Page stores for fuel/ignition tables
//!
//! The `SystemSnapshot` and `EcuPageStore` definitions now live in
//! `ecu_ts::pages`; they are re-exported here so `ecu_compat::ts::pages::*` and
//! the core builder (`EcuState::page_store`) keep resolving unchanged.
pub use ecu_ts::pages::{
    ts_page_descriptor, ExpertTriggerPageState, TsPageDescriptor, PAGE_AE, PAGE_AFR_TABLE,
    PAGE_ANGLES, PAGE_ASE, PAGE_CL, PAGE_DFCO, PAGE_DIAG, PAGE_DIAG_LOG, PAGE_EXPERT_TRIGGER,
    PAGE_FAN, PAGE_FUEL, PAGE_IDLE, PAGE_IGN, PAGE_LIMITS, PAGE_SENSORS, PAGE_SNAPSHOT,
    PAGE_VE_TABLE, PAGE_VE_TUNE, PAGE_WUE, TS_PAGE_COUNT, TS_PAGE_DESCRIPTORS,
};
pub use ecu_ts::pages::{EcuPageStore, SystemSnapshot};
use ecu_ts::pages::{DIAG_PAGE_BYTES, EXPERT_TRIGGER_PAGE_BYTES, TABLE_PAGE_BYTES};

pub const TS_DIAG_BYTES: usize = DIAG_PAGE_BYTES;
pub const TS_EXPERT_TRIGGER_BYTES: usize = EXPERT_TRIGGER_PAGE_BYTES;
pub const TS_PAGE_BYTES: usize = TABLE_PAGE_BYTES;

#[cfg(test)]
mod tests;
