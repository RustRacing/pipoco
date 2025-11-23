#![no_std]
#![no_main]

use cortex_m_rt::entry;
use ecu_core::transport::{CanDevice, CanTransport, Message, Transport};
use panic_halt as _;
use rp235x_hal as hal;

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
    // Minimal skeleton for a sensor→CAN datalogger.
    // Hook up clocks/UART/CAN (transceiver) per board in future.

    // Minimal CAN transport wiring (replace DummyCan with HAL CAN)
    let mut can = CanTransport::new(DummyCan);

    // Placeholder loop: periodically send a heartbeat
    loop {
        let hb = Message::Heartbeat {
            node_id: 3,
            uptime_seconds: 0,
            status: 0,
            error_count: 0,
            cpu_usage: 20,
        };
        let _ = can.send(&hb);
        cortex_m::asm::wfi();
    }
}
