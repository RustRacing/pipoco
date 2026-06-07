//! Pin mapping for RP2350B target
//! Adjust these values to remap outputs and trigger pin numbers.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PinMap {
    pub inj1: u8,
    pub inj2: u8,
    pub ign1: u8,
    pub ign2: u8,
    pub trigger: u8,
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum PinMapError {
    GpioOutOfRange { name: &'static str, pin: u8 },
}

impl PinMap {
    pub const MAX_BANK0_GPIO: u8 = 29;

    pub const fn defaults() -> Self {
        Self {
            inj1: 0,
            inj2: 1,
            ign1: 2,
            ign2: 3,
            trigger: 4,
        }
    }

    pub const fn validate(self) -> Result<Self, PinMapError> {
        if !Self::gpio_is_valid(self.inj1) {
            return Err(PinMapError::GpioOutOfRange {
                name: "inj1",
                pin: self.inj1,
            });
        }
        if !Self::gpio_is_valid(self.inj2) {
            return Err(PinMapError::GpioOutOfRange {
                name: "inj2",
                pin: self.inj2,
            });
        }
        if !Self::gpio_is_valid(self.ign1) {
            return Err(PinMapError::GpioOutOfRange {
                name: "ign1",
                pin: self.ign1,
            });
        }
        if !Self::gpio_is_valid(self.ign2) {
            return Err(PinMapError::GpioOutOfRange {
                name: "ign2",
                pin: self.ign2,
            });
        }
        if !Self::gpio_is_valid(self.trigger) {
            return Err(PinMapError::GpioOutOfRange {
                name: "trigger",
                pin: self.trigger,
            });
        }
        Ok(self)
    }

    pub const fn gpio_is_valid(pin: u8) -> bool {
        pin <= Self::MAX_BANK0_GPIO
    }

    pub const fn gpio_mask(pin: u8) -> Result<u32, PinMapError> {
        if Self::gpio_is_valid(pin) {
            Ok(1u32 << pin)
        } else {
            Err(PinMapError::GpioOutOfRange { name: "gpio", pin })
        }
    }

    pub const fn trigger_mask(self) -> Result<u32, PinMapError> {
        Self::gpio_mask_named("trigger", self.trigger)
    }

    pub const fn gpio_irq_edge_high(pin: u8) -> Result<(usize, u32), PinMapError> {
        if !Self::gpio_is_valid(pin) {
            Err(PinMapError::GpioOutOfRange { name: "gpio", pin })
        } else {
            let group = (pin / 8) as usize;
            let bit = ((pin % 8) * 4) + 3;
            Ok((group, 1u32 << bit))
        }
    }

    pub const fn trigger_irq_edge_high(self) -> Result<(usize, u32), PinMapError> {
        if !Self::gpio_is_valid(self.trigger) {
            Err(PinMapError::GpioOutOfRange {
                name: "trigger",
                pin: self.trigger,
            })
        } else {
            Self::gpio_irq_edge_high(self.trigger)
        }
    }

    pub const fn output_pins(self) -> [u8; 4] {
        [self.inj1, self.inj2, self.ign1, self.ign2]
    }

    const fn gpio_mask_named(name: &'static str, pin: u8) -> Result<u32, PinMapError> {
        if Self::gpio_is_valid(pin) {
            Ok(1u32 << pin)
        } else {
            Err(PinMapError::GpioOutOfRange { name, pin })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_pin_map_matches_minimal_output_order() {
        let map = PinMap::defaults().validate().unwrap();

        assert_eq!(map.output_pins(), [0, 1, 2, 3]);
        assert_eq!(map.trigger, 4);
    }

    #[test]
    fn trigger_mask_rejects_out_of_range_gpio_before_shift() {
        let map = PinMap {
            trigger: 32,
            ..PinMap::defaults()
        };

        assert_eq!(
            map.trigger_mask(),
            Err(PinMapError::GpioOutOfRange {
                name: "trigger",
                pin: 32,
            })
        );
    }

    #[test]
    fn trigger_irq_edge_high_uses_pac_grouped_gpio_irq_bits() {
        assert_eq!(PinMap::defaults().trigger_irq_edge_high(), Ok((0, 1 << 19)));

        let high_bank_pin = PinMap {
            trigger: 29,
            ..PinMap::defaults()
        };
        assert_eq!(high_bank_pin.trigger_irq_edge_high(), Ok((3, 1 << 23)));
    }
}
