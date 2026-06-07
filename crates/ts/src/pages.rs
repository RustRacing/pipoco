//! Core-free TunerStudio page registry, codecs, DTOs, and page stores.

mod codecs;
mod dto;
#[cfg(feature = "runtime")]
mod ecu_store;
mod registry;
mod stores;

pub use codecs::*;
pub use dto::*;
pub use registry::*;
pub use stores::*;

#[cfg(feature = "runtime")]
pub use ecu_store::{EcuPageStore, SystemSnapshot};

pub const TS_PAGE_BYTES: usize = TABLE_PAGE_BYTES;
pub const TS_SENSORS_BYTES: usize = SENSORS_PAGE_BYTES;
pub const TS_DIAG_BYTES: usize = DIAG_PAGE_BYTES;
pub const TS_EXPERT_TRIGGER_BYTES: usize = EXPERT_TRIGGER_PAGE_BYTES;

#[cfg(test)]
mod tests;
