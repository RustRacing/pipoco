//! CAN transport, OBD2 services, and module roster (split from a single
//! monolith; review 006). Public API is re-exported at `can::` so callers
//! keep using the crate root paths.

pub mod obd2;
pub mod roster;
pub mod transport;

pub use obd2::*;
pub use roster::*;
pub use transport::*;
