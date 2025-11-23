#![no_std]
#![no_main]

use cortex_m_rt::entry;
use panic_halt as _;

use ecu_target_common::ts::usb_cdc::CdcSerial;
use embedded_hal::adc::OneShot;
use hal::adc::{Adc, AdcPin};
use hal::clocks::{init_clocks_and_plls, Clock};
#[cfg(feature = "capture-pio")]
use hal::gpio::FunctionPio0;
#[cfg(feature = "capture-pio")]
use hal::pio::{PIOExt, ShiftDirection};
use hal::usb::UsbBus;
use hal::watchdog::Watchdog;
use hal::{pac, sio::Sio};
#[cfg(feature = "capture-pio")]
use pio_proc::pio;
use rp2040_hal as hal;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort as UsbdSerial;

use ecu_core::config::{EcuConfig, IgnitionMode, InjectionMode, OutputChannels};
use ecu_core::hal::TimeSource;
use ecu_core::persist::KvStore;
use ecu_core::sensors::model::CalibratedSensor;
use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::pages::EcuPageStore;
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::OutpcProvider;
use ecu_core::{
    dfco::{DfcoConfig, DfcoState},
    enrichment::{AeConfig, AeState},
};
use ecu_core::{CaptureBuffer, EcuApp, EcuState};
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::sensors::adc_pipeline::{
    convert_all as ts_convert_all, AdcConfig as TsAdcConfig, RawCounts as TsRawCounts,
};
use ecu_target_common::ts::service::TsService;
use ecu_target_common::ts::store::PersistedEcuPageStore;
#[cfg(feature = "flash-kv")]
mod seq_kv;
#[cfg(feature = "flash-kv")]
use seq_kv::SeqKv;

// Arduino-style outputs — edit these to remap pins quickly
macro_rules! INJ1_GPIO {
    ($pins:ident) => {
        $pins.gpio0.into_push_pull_output()
    };
}
macro_rules! INJ2_GPIO {
    ($pins:ident) => {
        $pins.gpio1.into_push_pull_output()
    };
}
macro_rules! IGN1_GPIO {
    ($pins:ident) => {
        $pins.gpio2.into_push_pull_output()
    };
}
macro_rules! IGN2_GPIO {
    ($pins:ident) => {
        $pins.gpio3.into_push_pull_output()
    };
}
const TRIGGER_PIN: u8 = 4; // use with GPIO-IRQ or PIO example bins
#[cfg(feature = "capture-cam")]
const CAM_PIN: u8 = 5;

// Sensor pin mapping (Pico ADC channels: 26..29)
// Change these to match your wiring
const PIN_MAP_ADC: u8 = 26; // GPIO26 - MAP sensor
const PIN_TPS_ADC: u8 = 27; // GPIO27 - TPS sensor
const PIN_CLT_ADC: u8 = 28; // GPIO28 - CLT thermistor
const PIN_IAT_ADC: u8 = 29; // GPIO29 - IAT thermistor or VSYS/3

ecu_target_common::capture_ring!(CAPTURE, 128);
static mut APP: Option<EcuApp<RpTime>> = None;
static mut SENS: Sensors = Sensors::new();
static mut AE_STATE: AeState = AeState::new();
static mut DFCO_STATE: DfcoState = DfcoState::new();
#[cfg(feature = "capture-cam")]
static mut CAM_PHASE: u8 = 0; // toggles on cam edges

#[derive(Copy, Clone)]
struct RpTime;
impl TimeSource for RpTime {
    fn micros(&self) -> u32 {
        unsafe { &*pac::TIMER::ptr() }.timerawl.read().bits()
    }
}

#[cfg(feature = "capture-pio")]
pio!(
    program edge_irq_prog {
        wrap_target;
            wait 0 pin 0;
            wait 1 pin 0;
            irq set 0;
            jmp wrap_target;
        wrap;
    }
);

// CdcSerial is provided by ecu-target-common

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        let sens = unsafe { &SENS };
        out.rpm = s.rpm;
        out.map_kpa_x10 = sens.map_kpa_x10;
        out.tps_percent = sens.tps_percent;
        out.clt_c = sens.clt_c;
        out.iat_c = sens.iat_c;
        out.vbatt_mv = sens.vbatt_mv;
        out.lambda_x100 = 100; // TODO: from wideband
        out.pw_us = s.calculate_fuel(2000, 100); // example fuel calc
        out.dwell_us = 3000;
        out.advance_x10 = 150;
        out.synced = if s.synced { 1 } else { 0 };
        // Extended fields
        out.target_afr_x10 = 147;
        out.ego_correction_percent = 100;
        out.ego_sensor = 2; // assume WB
        out.mapdot_kpa_s = sens.mapdot_kpa_s;
        out.tpsdot_pct_s = sens.tpsdot_pct_s;
        // Approximate injector duty_x10 = 10 * pw_us * rpm / 120_000_000
        let duty = ((out.pw_us as u32)
            .saturating_mul(out.rpm as u32)
            .saturating_mul(10))
            / 120_000_000;
        out.inj_duty_x10 = duty.min(1000) as u16;
        out.idle_duty_x10 = 0;
        out.fan_state = 0;
        // Engine state bits: bit0=WUE, bit1=ASE, bit2=CL, bit3=DFCO, bit4=AE, bit5=EMERGENCY
        let mut flags: u16 = 0;
        if sens.clt_c < 60 {
            flags |= 1 << 0;
        } // WUE
          // AE
        let ae = unsafe {
            AE_STATE.update(
                RpTime.micros(),
                sens.tpsdot_pct_s,
                sens.mapdot_kpa_s,
                &AeConfig::DEFAULT,
            )
        };
        if ae > 0 {
            flags |= 1 << 4;
        }
        // DFCO
        let dfco = unsafe {
            DFCO_STATE.update(
                RpTime.micros(),
                out.rpm,
                sens.tps_percent,
                (sens.map_kpa_x10 / 10) as u16,
                &DfcoConfig::DEFAULT,
            )
        };
        if dfco {
            flags |= 1 << 3;
        }
        // Emergency mode (bit5)
        if s.emergency_mode { flags |= 1 << 5; }
        out.engine_state = flags;
        out.baro_kpa = 100;
        out.gear = 0;
    }
}

// Basic ADC sensor reader with simple scaling and plausibility checks
struct Sensors {
    map_kpa_x10: u16,
    tps_percent: u8,
    clt_c: i16,
    iat_c: i16,
    vbatt_mv: u16,
    mapdot_kpa_s: i16,
    tpsdot_pct_s: i16,
    last_map_kpa_x10: u16,
    last_tps_percent: u8,
    last_ts_us: u32,
}
impl Sensors {
    pub const fn new() -> Self {
        Self {
            map_kpa_x10: 1000,
            tps_percent: 0,
            clt_c: 20,
            iat_c: 25,
            vbatt_mv: 12000,
            mapdot_kpa_s: 0,
            tpsdot_pct_s: 0,
            last_map_kpa_x10: 1000,
            last_tps_percent: 0,
            last_ts_us: 0,
        }
    }

    fn update(&mut self, adc: &mut Adc, pins: &mut AdcPins, cal: &ecu_core::sensors::SensorsCal, state: &mut ecu_core::EcuState) {
        // Read raw counts
        let raw = TsRawCounts {
            map: adc.read(&mut pins.map).unwrap_or(0),
            tps: adc.read(&mut pins.tps).unwrap_or(0),
            clt: adc.read(&mut pins.clt).unwrap_or(0),
            iat: adc.read(&mut pins.iat).unwrap_or(0),
            // Reuse IAT channel for VBATT when using VSYS/3
            vbatt: adc.read(&mut pins.iat).unwrap_or(0),
        };

        // When vbatt-vsys is enabled, compute VBATT from VSYS/3 (scale x3).
        // Otherwise, disable VBATT conversion (num=0) and keep previous vbatt_mv.
        #[cfg(feature = "vbatt-vsys")]
        let cfg = TsAdcConfig {
            vref_mv: 3300,
            adc_bits: 12,
            vbatt_scale_num: 3,
            vbatt_scale_den: 1,
        };
        #[cfg(not(feature = "vbatt-vsys"))]
        let cfg = TsAdcConfig {
            vref_mv: 3300,
            adc_bits: 12,
            vbatt_scale_num: 0,
            vbatt_scale_den: 1,
        };
        let out = ts_convert_all(cfg, cal, raw);
        // Clamp and update state diagnostics/emergency
        let now = RpTime.micros();
        let (map_clamped, tps_clamped) = state.process_sensor_update(now, out.map_kpa_x10, out.tps_percent);

        // Slew rate limits (conservative defaults): MAP 1000 kPa×10/s, TPS 300 %/s
        let dt_us = if self.last_ts_us == 0 { 0 } else { now.wrapping_sub(self.last_ts_us) };
        let map_slewed = ecu_target_common::sensors::adc_pipeline::clamp_slew_u16(self.last_map_kpa_x10, map_clamped, 1000, dt_us);
        let tps_slewed = ecu_target_common::sensors::adc_pipeline::clamp_slew_u8(self.last_tps_percent, tps_clamped, 300, dt_us);
        self.map_kpa_x10 = map_slewed;
        self.tps_percent = tps_slewed;
        self.clt_c = out.clt_c;
        #[cfg(not(feature = "vbatt-vsys"))]
        {
            self.iat_c = out.iat_c;
        }
        #[cfg(feature = "vbatt-vsys")]
        {
            self.vbatt_mv = out.vbatt_mv;
        }

        // Derivatives based on time delta
        let rp = RpTime;
        let now = rp.micros();
        let dt_us = now.wrapping_sub(self.last_ts_us);
        if dt_us > 0 {
            let d_map_x10 = map_kpa_x10 as i32 - self.last_map_kpa_x10 as i32;
            let num = d_map_x10.saturating_mul(1_000_000); // scale to per-second
            let den = (10 * (dt_us as i32)).max(1);
            let mapdot = num / den; // kPa/s
            self.mapdot_kpa_s = mapdot.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            let d_tps = self.tps_percent as i32 - self.last_tps_percent as i32;
            let num_tps = d_tps.saturating_mul(1_000_000);
            let tpsdot = num_tps / (dt_us as i32).max(1);
            self.tpsdot_pct_s = tpsdot.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            self.last_map_kpa_x10 = map_kpa_x10;
            self.last_tps_percent = self.tps_percent;
            self.last_ts_us = now;
        } else if self.last_ts_us == 0 {
            self.last_map_kpa_x10 = map_kpa_x10;
            self.last_tps_percent = self.tps_percent;
            self.last_ts_us = now;
        }
    }
}

// Holder for ADC pins
struct AdcPins {
    map: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio26, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
    tps: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio27, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
    clt: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio28, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
    iat: AdcPin<
        hal::gpio::Pin<hal::gpio::bank0::Gpio29, hal::gpio::FunctionNull, hal::gpio::PullDown>,
    >,
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

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

    let sio = Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Optional: configure PIO capture on TRIGGER_PIN
    #[cfg(feature = "capture-pio")]
    {
        // Route trigger pin to PIO0
        match TRIGGER_PIN {
            0 => {
                let _ = pins.gpio0.into_mode::<FunctionPio0>();
            }
            1 => {
                let _ = pins.gpio1.into_mode::<FunctionPio0>();
            }
            2 => {
                let _ = pins.gpio2.into_mode::<FunctionPio0>();
            }
            3 => {
                let _ = pins.gpio3.into_mode::<FunctionPio0>();
            }
            4 => {
                let _ = pins.gpio4.into_mode::<FunctionPio0>();
            }
            5 => {
                let _ = pins.gpio5.into_mode::<FunctionPio0>();
            }
            _ => {
                let _ = pins.gpio4.into_mode::<FunctionPio0>();
            }
        }
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let installed = pio.install(&edge_irq_prog::PROGRAM).unwrap();
        let (mut sm, _rx, _tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            .in_pin_base(TRIGGER_PIN)
            .clock_divisor(1.0)
            .build(sm0);
        sm.set_pindirs([], []);
        sm.start();
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
        }
        pio.clr_irq0();
        pio.sm_set_enabled(0, true);
        pio.set_irq0_source_enabled(rp2040_hal::pio::InterruptSource::Sm0, true);
    }

    // USB bus allocator
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
        .product("TS ECU")
        .serial_number("IPW-TS-ECU-0001")
        .device_class(USB_CLASS_CDC)
        .build();
    let mut cdc = CdcSerial { serial, dev };

    // ECU app and state
    static mut STATE: EcuState = EcuState::new();
    let state = unsafe { &mut STATE };
    let outputs_cfg = OutputChannels::for_4ch();
    #[cfg(feature = "capture-cam")]
    let cfg = EcuConfig {
        cylinders: 4,
        firing_order: &[1, 3, 4, 2],
        injection_mode: InjectionMode::Sequential,
        ignition_mode: IgnitionMode::Sequential,
        has_cam: true,
        outputs: outputs_cfg,
        inj_angle_btdc_x10: [0; 16],
        tdc_per_cyl_x10: [0; 16],
        tooth0_angle_x10: 0,
    };
    #[cfg(not(feature = "capture-cam"))]
    let cfg = EcuConfig {
        cylinders: 4,
        firing_order: &[1, 3, 4, 2],
        injection_mode: InjectionMode::Batch,
        ignition_mode: IgnitionMode::Wasted,
        has_cam: false,
        outputs: outputs_cfg,
        inj_angle_btdc_x10: [0; 16],
        tdc_per_cyl_x10: [0; 16],
        tooth0_angle_x10: 0,
    };
    cortex_m::interrupt::free(|_| unsafe {
        APP = Some(EcuApp::new_with_config(RpTime, cfg));
    });

    // Prepare outputs using shared wrapper
    let inj1 = INJ1_GPIO!(pins);
    let inj2 = INJ2_GPIO!(pins);
    let ign1 = IGN1_GPIO!(pins);
    let ign2 = IGN2_GPIO!(pins);
    let mut outs4 = ecu_target_common::outputs::Outputs4::new(inj1, inj2, ign1, ign2);

    // TS service (uses shared TsService; KV is feature-selectable)
    let provider = Provider {
        state: state as *const EcuState,
    };
    #[cfg(feature = "flash-kv")]
    let mut store = PersistedEcuPageStore::new(state, SeqKv::new());
    #[cfg(not(feature = "flash-kv"))]
    let mut store = PersistedEcuPageStore::new(state, RamKv512::new());
    store.try_load();
    let mut ts = TsService::new(b"IPW-ECU V0.1", provider, store);

    // Framing buffers (for custom commands only)
    let mut asm = ecu_core::ts::serial::FrameAssembler::new();
    let mut inbuf = [0u8; 512];
    let mut out = [0u8; 512];

    // ADC setup and channel pins
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let mut adc_pins = AdcPins {
        map: AdcPin::new(pins.gpio26.into_floating_input()),
        tps: AdcPin::new(pins.gpio27.into_floating_input()),
        clt: AdcPin::new(pins.gpio28.into_floating_input()),
        iat: AdcPin::new(pins.gpio29.into_floating_input()),
    };

    // Optional: configure CAM input via IO_IRQ_BANK0
    #[cfg(feature = "capture-cam")]
    {
        let _cam = pins.gpio5.into_pull_up_input();
        unsafe {
            let io = &*pac::IO_BANK0::ptr();
            // Clear any pending
            io.intr[0].write(|w| unsafe { w.bits(1 << CAM_PIN) });
            // Enable rising edge
            let cur = io.edge_high.read().bits();
            io.edge_high
                .write(|w| unsafe { w.bits(cur | (1 << CAM_PIN)) });
            // Unmask
            let m = io.inte[0].read().bits();
            io.inte[0].write(|w| unsafe { w.bits(m | (1 << CAM_PIN)) });
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }
    }

    loop {
        // Pump USB
        let mut tmp = [0u8; 64];
        let n = cdc.read(&mut tmp);
        if n > 0 {
            asm.feed(&tmp[..n]);
        }
        if let Some(len) = asm.try_pop(&mut inbuf) {
            if let Some((cmd, payload)) = proto::decode_request(&inbuf[..len]) {
                match cmd {
                    Cmd::OutputTest => {
                        if payload.len() >= 6 {
                            let chan = payload[0];
                            let on_ms = u16::from_le_bytes([payload[1], payload[2]]) as u32;
                            let off_ms = u16::from_le_bytes([payload[3], payload[4]]) as u32;
                            let reps = payload[5];
                            for _ in 0..reps {
                                match chan {
                                    0 => {
                                        let _ = inj1.set_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        let _ = inj1.set_low();
                                    }
                                    1 => {
                                        let _ = inj2.set_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        let _ = inj2.set_low();
                                    }
                                    2 => {
                                        let _ = ign1.set_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        let _ = ign1.set_low();
                                    }
                                    3 => {
                                        let _ = ign2.set_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        let _ = ign2.set_low();
                                    }
                                    _ => {}
                                }
                                cortex_m::asm::delay(off_ms * 1000 * 125);
                            }
                            if let Some(mr) = proto::encode_reply(Cmd::OutputTest, b"OK", &mut out)
                            {
                                let _ = cdc.write(&out[..mr]);
                            }
                        }
                    }
                    Cmd::ToothStats => {
                        let rpm = unsafe { (*state).rpm };
                        let synced = unsafe { (*state).synced } as u8;
                        let mut buf = [0u8; 3];
                        buf[0] = (rpm & 0xff) as u8;
                        buf[1] = (rpm >> 8) as u8;
                        buf[2] = synced;
                        if let Some(mr) = proto::encode_reply(Cmd::ToothStats, &buf, &mut out) {
                            let _ = cdc.write(&out[..mr]);
                        }
                    }
                    _ => {
                        if let Some(mr) = ts.handle_frame(&inbuf[..len], &mut out) {
                            let _ = cdc.write(&out[..mr]);
                        }
                    }
                }
            } else if let Some(mr) = ts.handle_frame(&inbuf[..len], &mut out) {
                let _ = cdc.write(&out[..mr]);
            }
        }

        // Update sensors
        unsafe {
            SENS.update(&mut adc, &mut adc_pins, &state.sensors_cal, state);
        }

        // Common tick: drain capture + drive outputs
        ecu_target_common::tick_once!(
            APP,
            capture_pop,
            RpTime.micros(),
            outs4.as_pins(),
            { /* TS pump handled above with custom commands */ },
            ()
        );

        // Optional: simulate trigger edges if capture-pio not enabled
        #[cfg(not(feature = "capture-pio"))]
        {
            cortex_m::asm::delay(24_000);
            capture_push(RpTime.micros());
        }

        // If cam phase toggled, notify app
        #[cfg(feature = "capture-cam")]
        {
            static mut LAST_CAM: u8 = 0;
            unsafe {
                if CAM_PHASE != LAST_CAM {
                    LAST_CAM = CAM_PHASE;
                    if let Some(ref mut a) = APP {
                        a.on_cam_edge();
                    }
                }
            }
        }
    }
}

#[cfg(feature = "capture-pio")]
#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn PIO0_IRQ_0() {
    capture_push(RpTime.micros());
    unsafe {
        let pio = &*pac::PIO0::ptr();
        pio.irq0.write(|w| unsafe { w.bits(1) });
    }
}

#[cfg(feature = "capture-cam")]
#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn IO_IRQ_BANK0() {
    // Toggle phase on cam rising edge
    unsafe {
        CAM_PHASE ^= 1;
        let io = &*pac::IO_BANK0::ptr();
        io.intr[0].write(|w| unsafe { w.bits(1 << CAM_PIN) }); // clear
    }
}
