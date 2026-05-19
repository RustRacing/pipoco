//! Pin mapping for RP2350B target
//! Adjust these values to remap outputs and trigger pin numbers.
#[allow(dead_code)]
pub struct PinMap {
    pub inj1: u8,
    pub inj2: u8,
    pub ign1: u8,
    pub ign2: u8,
    pub trigger: u8,
}

impl PinMap {
    pub const fn defaults() -> Self {
        Self {
            inj1: 0,
            inj2: 1,
            ign1: 2,
            ign2: 3,
            trigger: 4,
        }
    }
}
