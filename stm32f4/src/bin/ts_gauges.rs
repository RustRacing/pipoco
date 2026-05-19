#![no_std]
#![no_main]

use cortex_m_rt::entry;
use ecu_core::lambda::O2SensorType;
use ecu_core::ts::outpc::Outpc;
use ecu_core::{
    ts::serial::{FrameAssembler, SerialPort},
    ts::{EcuStatePageStore, OutpcProvider, TunerstudioServer},
    EcuState,
};
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*};

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
        out.map_kpa_x10 = s.map_kpa_x10();
        out.tps_percent = s.tps_percent();
        out.clt_c = 20;
        out.iat_c = 25;
        out.vbatt_mv = s.battery_voltage_mv();
        out.lambda_x100 = 100;
        out.pw_us = s.calculate_fuel(s.rpm(), s.map_kpa_x10() / 10);
        out.dwell_us = s.calculate_dwell() as u16;
        out.advance_x10 = 150;
        out.synced = if s.synced() { 1 } else { 0 };
        // Extended fields are mostly board-startup defaults until the target
        // wires the remaining sensors and control outputs.
        out.target_afr_x10 = s.config.cl_config.target_afr_x10;
        out.ego_correction_percent = (100 + s.stft_x10() as i32 / 10).clamp(0, 200) as u8;
        out.ego_sensor = match s.lambda_state.sensor_type {
            O2SensorType::Narrowband => 1,
            O2SensorType::Wideband => 2,
        };
        out.mapdot_kpa_s = 0;
        out.tpsdot_pct_s = 0;
        out.inj_duty_x10 = 0;
        out.idle_duty_x10 = 0;
        out.fan_state = 0;
        out.engine_state = 0;
        out.baro_kpa = 100;
        out.gear = 0;
    }
}

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let _clocks = rcc.cfgr.sysclk(168.MHz()).freeze();

    // Create ECU state & TS server
    static mut STATE: EcuState = EcuState::new();
    let state_ptr = &raw mut STATE;
    let state = unsafe { &mut *state_ptr };
    let provider = Provider {
        state: state_ptr as *const EcuState,
    };
    // Expose fuel & ignition tables to TS
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
