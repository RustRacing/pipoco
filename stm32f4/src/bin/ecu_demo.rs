#![no_std]
#![no_main]

use cortex_m_rt::entry;
use ecu_core::EcuApp;
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*};
#[path = "../hal_impl.rs"]
mod hal_impl;
use hal_impl::Stm32Time;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Setup clocks (168MHz)
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Configure TIM2 as 1us counter
    dp.TIM2.psc.write(|w| w.psc().bits(168 - 1));
    dp.TIM2.arr.write(|w| w.arr().bits(u32::MAX));
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit());

    // Initialize EcuApp (no capture wired in demo)
    let mut _app = EcuApp::new(Stm32Time);

    loop {
        cortex_m::asm::wfi();
    }
}
