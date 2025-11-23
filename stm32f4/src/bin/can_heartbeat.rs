#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*};

use ecu_core::transport::{CanDevice, CanTransport, Message, Transport};

struct DummyCan;
impl CanDevice for DummyCan {
    type Error = ();
    fn tx_ready(&self) -> bool {
        true
    }
    fn send(&mut self, _id: u32, _data: &[u8]) -> Result<(), Self::Error> {
        Ok(())
    }
    fn try_receive(&mut self, _buf: &mut [u8]) -> Option<(u32, usize)> {
        None
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Minimal CAN transport wiring (DummyCan for compile-time demo)
    let mut can = CanTransport::new(DummyCan);

    // Periodically send heartbeat
    loop {
        let hb = Message::Heartbeat {
            node_id: 1,
            uptime_seconds: 0,
            status: 0,
            error_count: 0,
            cpu_usage: 10,
        };
        let _ = can.send(&hb);
        cortex_m::asm::wfi();
    }
}
