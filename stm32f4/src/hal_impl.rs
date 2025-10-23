//! STM32F4 HAL trait implementations
//!
//! Provides concrete implementations of the HAL traits for STM32F4 hardware.

use stm32f4xx_hal::pac;
use ecu_core::hal::{TimeSource, OutputPin};

/// STM32F4 time source using TIM2
///
/// Zero-sized type that accesses TIM2 peripheral via unsafe pointer.
/// TIM2 must be configured before creating instances of this type.
///
/// # Safety
///
/// This type assumes TIM2 is properly initialized and never moved or
/// reconfigured after initialization. The pointer dereference is safe
/// because:
/// - TIM2 peripheral address is fixed in hardware
/// - TIM2 is initialized once in main() before any TriggerDecoder is created
/// - TIM2 configuration is never changed after initialization
#[derive(Copy, Clone)]
pub struct Stm32Time;

impl TimeSource for Stm32Time {
    fn micros(&self) -> u32 {
        // Safety: TIM2 peripheral is initialized in main() and never moved
        unsafe {
            let tim2 = &(*pac::TIM2::ptr());
            tim2.cnt.read().bits()
        }
    }
}

/// STM32F4 GPIO output pin wrapper
///
/// Wraps an embedded-hal OutputPin implementation for use with the ECU core.
pub struct Stm32Pin<P> {
    pub pin: P,
}

impl<P> OutputPin for Stm32Pin<P>
where
    P: embedded_hal::digital::OutputPin,
{
    fn set_high(&mut self) {
        // Ignore result - we assume pin operations always succeed
        let _ = self.pin.set_high();
    }

    fn set_low(&mut self) {
        // Ignore result - we assume pin operations always succeed
        let _ = self.pin.set_low();
    }
}
