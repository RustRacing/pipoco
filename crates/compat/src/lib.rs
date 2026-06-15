//! ECU Compatibility Library
//!
//! Legacy/compatibility ECU facade for shared no_std-safe building blocks.
//! This crate still hosts trigger decoding, safety, TunerStudio/page storage,
//! diagnostics, calibration compatibility, tables, and small embedded support
//! modules used by board and simulation crates.
//!
//! # Architecture
//!
//! Runtime fuel and output behavior has moved out of the historical root app
//! shape. Direct pulse-width fuel strategies live in the split runtime/fuel
//! crates, and scheduled output execution is maintained in `ecu-scheduler`
//! plus the target-common/board adapter path. `ecu-compat` remains a
//! no_std-compatible compatibility boundary for code that still depends on
//! legacy `EcuState` pages, safety helpers, trigger primitives, or
//! TunerStudio-facing data.
//!
//! ## Modules
//!
//! - `trigger`: 60-2 trigger wheel decoder for position and RPM
//! - `tables`: legacy IPW table lookup helpers
//! - `hal`: Hardware abstraction traits
//! - `constants`: compatibility configuration constants
//! - `safety`, `diag`, `sensors`: shared safety and diagnostic primitives
//! - `ts`: TunerStudio and page-store compatibility support
//!
//! ## Design Principles
//!
//! - **Integer-only arithmetic**: No floating-point operations
//! - **Static memory**: No heap allocation, all state in static variables
//! - **Wrapping arithmetic**: Correctly handles timer overflow
//! - **Compatibility boundary**: Keep legacy-facing data stable while new
//!   runtime, fuel, and scheduler behavior stays in split crates
//!
//! ## Ownership
//!
//! `ecu-compat` is a shrinking compatibility shell. New canonical product
//! behavior belongs in the split crates (`ecu-runtime`, `ecu-control`,
//! `ecu-scheduler`, `ecu-calibration`) rather than here. Compatibility-owned
//! surfaces are grouped under [`compat`] and remain public only to support
//! migration and legacy callers. See
//! `aidocs/architecture/adr-0001-core-ownership.md` and
//! `aidocs/architecture/adr-0010-runtime-compat-boundaries.md`.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "transport-can-fd", not(feature = "transport-can")))]
compile_error!("feature `transport-can-fd` requires `transport-can`");

pub mod actuators;
pub mod capture;
mod compat_state;
pub mod constants;
pub mod dfco;
pub mod diag;
mod ecu_state;
pub mod enrichment;
mod fuel_state;
pub mod hal;
pub mod ignition;
pub mod interp;
pub mod knock;
pub mod lambda;
pub mod rev_limiter;
pub mod runtime_adapter;
pub mod safety;
pub mod sensors;
pub mod tables;
pub mod telemetry;
pub mod torque;
pub mod trigger;
pub mod ts;
pub mod units;

/// Explicit compatibility namespace for legacy mirrors and migration shims.
pub mod compat {
    pub use crate::compat_state::{
        CoreAdapterContract, DiagnosticFlags, EcuConfig, EcuDerived, EcuFaults, EcuInputs,
        EcuOutputs, RuntimeSignals, SafetyStatus,
    };
    pub use crate::ecu_state::EcuState;
    pub use crate::runtime_adapter::{
        runtime_fuel_strategy_from_fuel_tune, runtime_fuel_strategy_from_state,
        runtime_semantic_calibration_from_fuel_tune, runtime_semantic_calibration_from_state,
    };
}

pub use capture::CaptureBuffer;
pub use fuel_state::{apply_cl_delta, scale_u16, Corrections};
pub use ignition::{calculate_dwell, calculate_timing, IgnitionCorrections, IgnitionTable};
pub use rev_limiter::{
    apply_limiter_retard, should_inject, update_limiter, LimiterStrategy, RevLimiterConfig,
    RevLimiterState,
};
pub use safety::{
    should_allow_injection, update_flood_clear, FloodClearState, LoadFailureConfig,
    LoadFailureReason, LoadFailureTracker, PowerState, SyncLossTracker, VoltageMonitor,
};
pub use tables::IpwTable;
pub use telemetry::IsrStats;
pub use trigger::{TriggerDecoder, TriggerTiming};
pub use units::{DegX10, Kpa10, Micros, Rpm, Ticks};
