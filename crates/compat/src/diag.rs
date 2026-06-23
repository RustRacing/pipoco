//! Minimal diagnostics (DTC-like) tracking for sensor range faults and cam status.
//!
//! The diagnostics types now live in `ecu-domain`; this module re-exports them
//! so `ecu_compat::diag::{...}` resolves unchanged for existing consumers.

pub use ecu_domain::diag::{
    DiagClearSummary, DiagCode, DiagEvent, DiagLog, DiagSource, DiagState, DiagStatus,
};
