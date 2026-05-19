#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use hal::clocks::init_clocks_and_plls;
use hal::usb::UsbBus;
use hal::{pac, sio::Sio, watchdog::Watchdog};
use rp2040_hal as hal;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort as UsbdSerial;
use usbd_serial::USB_CLASS_CDC;

use ecu_core::ts::outpc::Outpc;
use ecu_core::{
    ts::serial::{FrameAssembler, SerialPort},
    ts::{EcuStatePageStore, OutpcProvider, TunerstudioServer},
    EcuState,
};

struct Cdc<'a, B: usb_device::bus::UsbBus> {
    serial: UsbdSerial<'a, B>,
    dev: UsbDevice<'a, B>,
}

impl<'a, B: usb_device::bus::UsbBus> SerialPort for Cdc<'a, B> {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        let _ = self.dev.poll(&mut [&mut self.serial]);
        self.serial.read(buf).unwrap_or_default()
    }
    fn write(&mut self, buf: &[u8]) -> usize {
        let _ = self.dev.poll(&mut [&mut self.serial]);
        self.serial.write(buf).unwrap_or_default()
    }
}

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        out.rpm = s.rpm();
        out.map_kpa_x10 = 1000;
        out.tps_percent = s.tps_percent();
        out.clt_c = 20;
        out.iat_c = 25;
        out.vbatt_mv = s.battery_voltage_mv();
        out.lambda_x100 = 100;
        out.pw_us = 1000;
        out.dwell_us = 3000;
        out.advance_x10 = 150;
        out.synced = if s.synced() { 1 } else { 0 };
    }
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // External crystal at 12 MHz on Pico
    let clocks = init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let _sio = Sio::new(pac.SIO);

    // USB bus
    let usb_bus: UsbBusAllocator<UsbBus> = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));

    let serial = UsbdSerial::new(&usb_bus);
    let dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000A))
        .manufacturer("IPW")
        .product("TS Gauges")
        .serial_number("IPW-TS-0001")
        .device_class(USB_CLASS_CDC)
        .build();
    let mut cdc = Cdc { serial, dev };

    // ECU and TS server
    let state = cortex_m::singleton!(: EcuState = EcuState::new())
        .expect("EcuState singleton already taken");
    let provider = Provider {
        state: state as *const EcuState,
    };
    let store = EcuStatePageStore {
        fuel: &mut state.config.ipw_table,
        ign: &mut state.config.ignition_table,
    };
    let mut server = TunerstudioServer::new(b"IPW-ECU V0.1", provider, store);

    let mut asm = FrameAssembler::new();
    let mut inbuf = [0u8; 512];
    let mut out = [0u8; 512];

    loop {
        // pump USB and attempt to read
        let mut tmp = [0u8; 64];
        let n = cdc.read(&mut tmp);
        if n > 0 {
            asm.feed(&tmp[..n]);
        }

        if let Some(len) = asm.try_pop(&mut inbuf) {
            if let Some(m) = server.handle(&inbuf[..len], &mut out) {
                let _ = cdc.write(&out[..m]);
            }
        }
        cortex_m::asm::wfi();
    }
}
