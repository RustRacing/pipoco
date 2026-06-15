//! TunerStudio protocol support (custom, Speeduino/rusEFI style)
//!
//! The generic protocol/server shell lives in the canonical `ecu-ts` crate.
//! Full-state page projection (`pages`) remains here because it depends on
//! `EcuState`.

pub mod pages;

pub use pages::{PAGE_AFR_TABLE, PAGE_FUEL, PAGE_IGN, PAGE_VE_TABLE, PAGE_VE_TUNE};
