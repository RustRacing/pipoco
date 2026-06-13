#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rpm(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Micros(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Kpa10(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TempC10(pub i16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Millivolts(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Degrees10(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SignedDegrees10(pub i16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PulseWidthUs(pub u32);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AfrX100(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VePctX100(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RatioX1000(pub u16);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CylinderId(pub u8);

macro_rules! impl_newtype_api {
    ($name:ident, $raw:ty) => {
        impl $name {
            pub const fn new(value: $raw) -> Self {
                Self(value)
            }

            pub const fn get(self) -> $raw {
                self.0
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self(0)
            }
        }
    };
}

impl_newtype_api!(Rpm, u16);
impl_newtype_api!(Micros, u32);
impl_newtype_api!(Kpa10, u16);
impl_newtype_api!(TempC10, i16);
impl_newtype_api!(Millivolts, u16);
impl_newtype_api!(Degrees10, u16);
impl_newtype_api!(SignedDegrees10, i16);
impl_newtype_api!(PulseWidthUs, u32);
impl_newtype_api!(AfrX100, u16);
impl_newtype_api!(VePctX100, u16);
impl_newtype_api!(RatioX1000, u16);
impl_newtype_api!(CylinderId, u8);
