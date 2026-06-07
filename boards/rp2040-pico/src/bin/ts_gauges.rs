#![cfg_attr(not(test), no_std)]
#![cfg_attr(not(test), no_main)]

#[cfg(not(test))]
use cortex_m_rt::entry;
#[cfg(not(test))]
use panic_halt as _;

use hal::clocks::init_clocks_and_plls;
use hal::usb::UsbBus;
use hal::{pac, sio::Sio, watchdog::Watchdog};
use rp2040_hal as hal;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::USB_CLASS_CDC;

use ecu_ts::{
    outpc::Outpc,
    serial::{FrameAssembler, SerialPort},
    server::{NoPages, OutpcProvider, TunerstudioServer},
};

#[path = "../ts_usb_cdc.rs"]
mod ts_usb_cdc;
use ts_usb_cdc::{CdcSerial, CdcSerialStats};

struct GaugeState {
    rpm: u16,
    map_kpa_x10: u16,
    tps_percent: u8,
    vbatt_mv: u16,
    lambda_x100: u16,
    pw_us: u16,
    dwell_us: u16,
    synced: u8,
}

const GAUGES: GaugeState = GaugeState {
    rpm: 0,
    map_kpa_x10: 1000,
    tps_percent: 0,
    vbatt_mv: 12_000,
    lambda_x100: 100,
    pw_us: 1000,
    dwell_us: 3000,
    synced: 0,
};

struct Provider;

impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        out.rpm = GAUGES.rpm;
        out.map_kpa_x10 = GAUGES.map_kpa_x10;
        out.tps_percent = GAUGES.tps_percent;
        out.clt_c = 20;
        out.iat_c = 25;
        out.vbatt_mv = GAUGES.vbatt_mv;
        out.lambda_x100 = GAUGES.lambda_x100;
        out.pw_us = GAUGES.pw_us;
        out.dwell_us = GAUGES.dwell_us;
        out.advance_x10 = 150;
        out.synced = GAUGES.synced;
    }
}

#[cfg_attr(not(test), entry)]
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

    let serial = usbd_serial::SerialPort::new(&usb_bus);
    let dev = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0x2E8A, 0x000A))
        .manufacturer("IPW")
        .product("TS Gauges")
        .serial_number("IPW-TS-0001")
        .device_class(USB_CLASS_CDC)
        .build();
    let mut cdc = CdcSerial {
        serial,
        dev,
        stats: CdcSerialStats::new(),
    };

    let mut server = TunerstudioServer::new(ecu_ts::TS_SIGNATURE, Provider, NoPages);

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
