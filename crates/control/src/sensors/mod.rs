//! Canonical sensor processing: plausibility and slew/rate limiting.
//!
//! Single source of truth (ADR 0012, review 011). Both the frozen spec oracle
//! (`ecu-spec`) and the compatibility shell (`ecu-compat`) delegate here.
//! All sensor scalars are plain primitives so neither wrapper pays a
//! unit-type conversion tax.

pub mod plausibility;
pub mod slew;

pub use plausibility::{
    plausibility_step, PlausibilityInput, PlausibilityResult, PlausibilityState,
    PLAUSIBILITY_DEBOUNCE_US, PLAUSIBILITY_MIN_RPM,
};
pub use slew::{slew_step, SlewInput, SlewResult, SlewState};
