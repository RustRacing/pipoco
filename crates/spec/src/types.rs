//! Canonical spec model types shared by firmware, simulators, and proof code.
//!
//! Some exported model fields and variants are intentionally not consumed by the
//! host test crate yet. Keep that compatibility allowance local to this module
//! instead of allowing dead code across the whole crate.
#![allow(dead_code)]

mod calibration;
mod collections;
mod events;
mod input;
mod output;
mod state;
mod units;
mod validation;

pub use calibration::{
    AfrOverride, Calibration, DiagnosticCode, FuelModel, InjectionAngleMode, O2SensorMode,
    PwMaxPolicy, TrimPolicy,
};
pub use collections::{
    Axis16, Curve16, CylinderArrayI16, CylinderArrayU16, SignedCurve16, Table2D16,
};
#[allow(unused_imports)]
pub use events::{
    EventBatch, EventBatchFull, EventKind, SemanticEvent, MAX_EVENTS_PER_STEP, MAX_PENDING_EVENTS,
};
pub use input::InputSnapshot;
pub use output::{ObservableOutput, StepResult};
pub use state::{
    AeState, DiagState, EngineMode, KnockState, LogicalState, MathState, PiIntegratorState,
    SchedulerState, SyncState,
};
pub use units::{
    AfrX100, CylinderId, Degrees10, Kpa10, Micros, Millivolts, PulseWidthUs, RatioX1000, Rpm,
    SignedDegrees10, TempC10, VePctX100,
};
pub use validation::{ValidatedCalibration, ValidationError};
