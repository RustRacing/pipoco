#![no_std]
#![no_main]

use cortex_m_rt::entry;
use ecu_core::ts::outpc::Outpc;
use ecu_core::{
    ts::serial::{FrameAssembler, SerialPort},
    ts::{EcuStatePageStore, OutpcProvider, TunerstudioServer},
    EcuState,
};
use panic_halt as _;
use rp235x_hal as _;

struct NullSerial;
impl SerialPort for NullSerial {
    fn read(&mut self, _buf: &mut [u8]) -> usize {
        0
    }
    fn write(&mut self, _buf: &[u8]) -> usize {
        0
    }
}

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm();
        out.tps_percent = s.tps_percent();
        out.vbatt_mv = s.battery_voltage_mv();
        out.map_kpa_x10 = s.map_kpa_x10();
        out.clt_c = 20;
        out.iat_c = 25;
        out.lambda_x100 = 100;
        out.pw_us = s.calculate_fuel(s.rpm(), s.map_kpa_x10() / 10);
        out.dwell_us = s.calculate_dwell() as u16;
        out.advance_x10 = 150;
        out.synced = if s.synced() { 1 } else { 0 };
    }
}

static mut STATE: EcuState = EcuState::new();

#[entry]
fn main() -> ! {
    let state_ptr = &raw mut STATE;
    let provider = Provider { state: state_ptr };
    let state = unsafe { &mut *state_ptr };
    let store = EcuStatePageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);

    let mut serial = NullSerial;
    let mut asm = FrameAssembler::new();
    let mut inbuf = [0u8; 512];
    let mut out = [0u8; 512];
    loop {
        asm.poll_port(&mut serial);
        if let Some(n) = asm.try_pop(&mut inbuf) {
            if let Some(m) = server.handle(&inbuf[..n], &mut out) {
                let _ = serial.write(&out[..m]);
            }
        }
        cortex_m::asm::wfi();
    }
}
