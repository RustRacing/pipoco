//! Physical quantity newtypes for migration to explicit units.

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Rpm(pub u16);

impl Rpm {
    #[must_use]
    pub const fn new(raw: u16) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Kpa10(pub u16);

impl Kpa10 {
    #[must_use]
    pub const fn new(raw: u16) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Micros(pub u32);

impl Micros {
    #[must_use]
    pub const fn new(raw: u32) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> u32 {
        self.0
    }
}

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

#[repr(transparent)]
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Ticks(pub u32);

impl Ticks {
    #[must_use]
    pub const fn new(raw: u32) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> u32 {
        self.0
    }
}
