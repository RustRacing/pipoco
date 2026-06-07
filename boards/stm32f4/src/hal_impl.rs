//! STM32F4 HAL trait implementations
//!
//! Provides concrete implementations of the HAL traits for STM32F4 hardware.

#![cfg_attr(test, allow(dead_code))]

use ecu_board_api::{EcuClock, Watchdog};
use ecu_domain::Micros;
use stm32f4xx_hal::pac;

/// Nominal STM32F4 LSI clock used for IWDG reload calculation.
///
/// The hardware LSI oscillator has broad tolerance. This constant documents the
/// calculation contract only; production safety margins must still account for
/// the actual oscillator tolerance from the MCU datasheet.
pub const IWDG_NOMINAL_LSI_HZ: u32 = 32_000;
pub const IWDG_PRESCALER_DIVIDER: u32 = 64;
const IWDG_PR_DIV64_BITS: u8 = 0b100;
const IWDG_MAX_RELOAD: u16 = 0x0FFF;

pub const fn iwdg_reload_for_timeout_ms(timeout_ms: u32) -> u16 {
    let ticks =
        (timeout_ms as u64 * IWDG_NOMINAL_LSI_HZ as u64) / (1_000 * IWDG_PRESCALER_DIVIDER as u64);
    let reload = ticks.saturating_sub(1);
    if reload > IWDG_MAX_RELOAD as u64 {
        IWDG_MAX_RELOAD
    } else {
        reload as u16
    }
}

#[allow(dead_code)]
pub const fn iwdg_nominal_timeout_ms(reload: u16) -> u32 {
    (((reload as u64 + 1) * IWDG_PRESCALER_DIVIDER as u64 * 1_000) / IWDG_NOMINAL_LSI_HZ as u64)
        as u32
}

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

impl Stm32Time {
    pub fn micros(&self) -> u32 {
        // Safety: TIM2 peripheral is initialized in main() and never moved
        unsafe {
            let tim2 = &(*pac::TIM2::ptr());
            tim2.cnt.read().bits()
        }
    }
}

impl EcuClock for Stm32Time {
    fn now_us(&self) -> Micros {
        Micros::new(self.micros())
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

    pub fn start(&mut self, timeout_ms: u32) {
        // Enable write access
        unsafe {
            self.iwdg.kr.write(|w| w.key().bits(0x5555));
        }

        // Contract: PR=0b100 means /64 on STM32F4, so RLR is computed from
        // nominal 32 kHz LSI. LSI tolerance changes real wall-clock timeout.
        self.iwdg.pr.modify(|_, w| w.pr().bits(IWDG_PR_DIV64_BITS));

        let reload = iwdg_reload_for_timeout_ms(timeout_ms);
        self.iwdg.rlr.write(|w| w.rl().bits(reload));

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn iwdg_reload_matches_documented_250ms_contract() {
        let reload = iwdg_reload_for_timeout_ms(250);

        assert_eq!(reload, 124);
        assert_eq!(iwdg_nominal_timeout_ms(reload), 250);
    }

    #[test]
    fn iwdg_reload_saturates_to_hardware_limit() {
        assert_eq!(iwdg_reload_for_timeout_ms(60_000), IWDG_MAX_RELOAD);
    }
}

impl Watchdog for Stm32Watchdog {
    type Error = core::convert::Infallible;

    fn feed(&mut self) -> Result<(), Self::Error> {
        self.pet();
        Ok(())
    }
}
