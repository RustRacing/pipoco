//! Physical quantity newtypes for migration to explicit units.

pub use ecu_domain::{Kpa10, Micros, Rpm, Ticks};

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct DegX10(pub i16);

impl DegX10 {
    #[must_use]
    pub const fn new(raw: i16) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> i16 {
        self.0
    }
}
