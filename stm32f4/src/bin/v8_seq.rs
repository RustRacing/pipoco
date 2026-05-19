#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*};

use ecu_runtime::EngineRuntime;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Clocks @168MHz
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // TIM2 as 1us counter
    dp.TIM2.psc.write(|w| w.psc().bits(168 - 1));
    dp.TIM2.arr.write(|w| w.arr().bits(u32::MAX));
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit());

    // Split-runtime placeholder for the V8 sequential example. Channel mapping
    // belongs in target-common/scheduler configuration, not the legacy root app.
    let mut _runtime = EngineRuntime::new();

    // No capture wired in this demo; just idle
    loop {
        cortex_m::asm::wfi();
    }
}
