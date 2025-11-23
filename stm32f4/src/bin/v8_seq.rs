#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*};

use ecu_core::config::{EcuConfig, IgnitionMode, InjectionMode, OutputChannels};
use ecu_core::{Channel, EcuApp};

#[path = "../hal_impl.rs"]
mod hal_impl;
use hal_impl::Stm32Time;

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

    let time = Stm32Time;

    // Prepare 16 logical channels: INJ0..INJ7 = 0..7, IGN0..IGN7 = 8..15
    let mut inj = [Channel::from_index(0); 16];
    let mut ign = [Channel::from_index(0); 16];
    for i in 0..8 {
        inj[i] = Channel::from_index(i as u8);
    }
    for i in 0..8 {
        ign[i] = Channel::from_index((8 + i) as u8);
    }

    let outputs = OutputChannels {
        inj_channels: inj,
        inj_count: 8,
        ign_channels: ign,
        ign_count: 8,
    };
    let cfg = EcuConfig {
        cylinders: 8,
        firing_order: &[1, 8, 4, 3, 6, 5, 7, 2],
        injection_mode: InjectionMode::Sequential,
        ignition_mode: IgnitionMode::Sequential,
        has_cam: true,
        outputs,
        inj_angle_btdc_x10: [0; 16],
        tdc_per_cyl_x10: [0; 16],
        tooth0_angle_x10: 0,
    };

    // Initialize ECU app with V8 sequential config
    let mut _app = EcuApp::new_with_config(time, cfg);

    // No capture wired in this demo; just idle
    loop {
        cortex_m::asm::wfi();
    }
}
