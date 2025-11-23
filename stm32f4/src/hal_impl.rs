//! STM32F4 HAL trait implementations
//!
//! Provides concrete implementations of the HAL traits for STM32F4 hardware.

use ecu_core::hal::{OutputPin, ResetController, ResetReason, TimeSource, Watchdog};
use stm32f4xx_hal::pac;

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

/// STM32F4 Independent Watchdog
pub struct Stm32Watchdog {
    iwdg: pac::IWDG,
}

impl Stm32Watchdog {
    pub fn new(iwdg: pac::IWDG) -> Self {
        Self { iwdg }
    }
}

impl Watchdog for Stm32Watchdog {
    fn start(&mut self, timeout_ms: u32) {
        // Enable write access
        unsafe {
            self.iwdg.kr.write(|w| w.key().bits(0x5555));
        }

        // Prescaler selection to approximate timeout; use divider 64
        // timeout = (RLR + 1) / (LSI/ prescaler)
        // Assume ~32kHz LSI; for ~250ms: RLR ~ 12500 / 64 ≈ 195
        self.iwdg.pr.modify(|_, w| unsafe { w.pr().bits(0b011) }); // /32 or /64 depending on part

        let reload: u16 = if timeout_ms <= 100 {
            800
        } else if timeout_ms <= 250 {
            2000
        } else {
            4000
        };
        unsafe {
            self.iwdg.rlr.write(|w| w.rl().bits(reload));
        }

        // Reload and start
        unsafe {
            self.iwdg.kr.write(|w| w.key().bits(0xAAAA));
        } // reload
        unsafe {
            self.iwdg.kr.write(|w| w.key().bits(0xCCCC));
        } // start
    }

    fn pet(&mut self) {
        unsafe {
            self.iwdg.kr.write(|w| w.key().bits(0xAAAA));
        }
    }
}

/// STM32F4 Reset reason reader (from RCC CSR flags)
pub struct Stm32Reset {
    rcc: pac::RCC,
}

impl Stm32Reset {
    pub fn new(rcc: pac::RCC) -> Self {
        Self { rcc }
    }
}

impl ResetController for Stm32Reset {
    fn reason(&self) -> ResetReason {
        let csr = self.rcc.csr.read();
        if csr.borrstf().bit_is_set() {
            return ResetReason::BrownOut;
        }
        if csr.porrstf().bit_is_set() {
            return ResetReason::PowerOn;
        }
        if csr.sftrstf().bit_is_set() {
            return ResetReason::Software;
        }
        if csr.iwdgrstf().bit_is_set() {
            return ResetReason::IndependentWatchdog;
        }
        if csr.wwdgrstf().bit_is_set() {
            return ResetReason::WindowWatchdog;
        }
        if csr.lpwrstf().bit_is_set() {
            return ResetReason::LowPower;
        }
        ResetReason::Unknown
    }

    fn clear(&mut self) {
        // Clear reset flags by setting RMVF
        self.rcc.csr.modify(|_, w| w.rmvf().set_bit());
    }
}
