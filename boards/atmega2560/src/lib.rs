#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
// This board crate is currently a reusable adapter/build-plan surface plus
// tests, not a complete firmware binary. Some exported preparation/mapping
// paths are intentionally unused by the normal library target, and boxing
// build-plan errors would be a worse fit for the embedded/no-alloc boundary.
#![allow(dead_code)]
#![allow(clippy::result_large_err)]

mod adapter;
mod errors;
mod profile;
mod step_io;

#[cfg(test)]
mod tests;

pub use adapter::{Atmega2560BoardAdapter, Atmega2560PreparedFirmware};
pub use errors::{Atmega2560BridgeError, Atmega2560BuildPlanError, Atmega2560RecipePrepareError};
pub use profile::{Atmega2560AnalogPin, Atmega2560BoardProfile, Atmega2560DigitalPin};
pub use step_io::{Atmega2560StepInput, Atmega2560StepOutput};
