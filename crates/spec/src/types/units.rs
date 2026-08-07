//! Spec unit types: re-exported from `ecu-domain` (review 009 / ADR 0012).
//!
//! `ecu-spec` no longer declares its own unit newtypes. `SignedDegrees10` is
//! the same canonical `Degrees10` (i16) type; the spec keeps angle values in
//! `[0, 7200)` at its boundary.

pub use ecu_domain::{
    AfrX100, CylinderId, Degrees10, Kpa10, Micros, Millivolts, PulseWidthUs, RatioX1000, Rpm,
    TempC10, VePctX100,
};

/// Spec-compatible name for the canonical signed angle type (same type as
/// `Degrees10` after the signedness reconciliation).
pub type SignedDegrees10 = Degrees10;
