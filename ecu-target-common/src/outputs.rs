use ecu_core::hal::OutputPin as EcuOutputPin;

/// Simple wrapper that adapts any embedded-hal v1 OutputPin
/// to the ecu_core::hal::OutputPin trait.
pub struct HalOut<P> {
    pin: P,
}

impl<P> HalOut<P> {
    pub fn new(pin: P) -> Self {
        Self { pin }
    }
    pub fn into_inner(self) -> P {
        self.pin
    }
}

impl<P> EcuOutputPin for HalOut<P>
where
    P: embedded_hal::digital::OutputPin,
{
    fn set_high(&mut self) {
        let _ = self.pin.set_high();
    }
    fn set_low(&mut self) {
        let _ = self.pin.set_low();
    }
}

/// Holder for four output channels (inj1, inj2, ign1, ign2)
/// that can be easily passed to the ECU core as trait objects.
pub struct Outputs4<I1, I2, G1, G2> {
    inj1: HalOut<I1>,
    inj2: HalOut<I2>,
    ign1: HalOut<G1>,
    ign2: HalOut<G2>,
}

impl<I1, I2, G1, G2> Outputs4<I1, I2, G1, G2>
where
    I1: embedded_hal::digital::OutputPin,
    I2: embedded_hal::digital::OutputPin,
    G1: embedded_hal::digital::OutputPin,
    G2: embedded_hal::digital::OutputPin,
{
    pub fn new(inj1: I1, inj2: I2, ign1: G1, ign2: G2) -> Self {
        Self {
            inj1: HalOut::new(inj1),
            inj2: HalOut::new(inj2),
            ign1: HalOut::new(ign1),
            ign2: HalOut::new(ign2),
        }
    }

    pub fn as_pins(&mut self) -> [&mut dyn EcuOutputPin; 4] {
        [
            &mut self.inj1,
            &mut self.inj2,
            &mut self.ign1,
            &mut self.ign2,
        ]
    }
}
