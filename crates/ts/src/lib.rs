#![cfg_attr(not(test), no_std)]

//! Canonical TunerStudio protocol/server shell.
//!
//! The protocol modules (`proto`, `serial`, `server`, `outpc`) are the
//! low-level TS shell and carry no runtime/board dependencies. Runtime- and
//! calibration-facing surfaces live behind explicit `runtime`/`persistence`
//! features so low-level consumers get the shell-only API by default.
//!
//! When the `runtime` feature is enabled, the runtime-facing surface follows
//! the staged-calibration model:
//!
//! - runtime status is mapped only from `ecu_runtime::RuntimeSnapshot`
//! - staged edits operate on `ecu_calibration::CalibrationSnapshot`
//! - commit/reset/persist orchestration is intentionally hosted here for the
//!   current phase
//!
//! See `aidocs/architecture/adr-0010-runtime-compat-boundaries.md`.

pub const TS_SIGNATURE: &[u8] = b"IPW-ECU V0.1";

pub mod outpc;
pub mod pages;
#[cfg(feature = "persistence")]
pub mod persistence;
pub mod proto;
pub mod serial;
pub mod server;

#[cfg(feature = "runtime")]
mod runtime_view;
#[cfg(feature = "runtime")]
pub use runtime_view::{
    apply_expert_trigger_page, decode_expert_trigger_page, encode_expert_trigger_page,
    CalibrationCommandResult, CalibrationEditSurface, CalibrationWrite, ExpertTriggerPageError,
    RuntimeSnapshotAdapter, TunerStudioRuntimeView, EXPERT_TRIGGER_PAGE_LEN, PAGE_EXPERT_TRIGGER,
};
