#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use panic_halt as _;

use embedded_hal::adc::OneShot;
use embedded_hal::digital::v2::OutputPin;
use hal::adc::{Adc, AdcPin};
use hal::clocks::init_clocks_and_plls;
#[cfg(feature = "capture-pio")]
use hal::gpio::FunctionPio0;
#[cfg(feature = "capture-cam")]
use hal::gpio::Interrupt::EdgeHigh;
#[cfg(any(feature = "capture-pio", feature = "capture-cam"))]
use hal::pac::interrupt;
#[cfg(feature = "capture-pio")]
use hal::pio::PIOExt;
use hal::usb::UsbBus;
use hal::watchdog::Watchdog;
use hal::{pac, sio::Sio};
use rp2040_hal as hal;
#[path = "../ts_usb_cdc.rs"]
mod ts_usb_cdc;
use ts_usb_cdc::CdcSerial;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::SerialPort as UsbdSerial;
use usbd_serial::USB_CLASS_CDC;

#[cfg(feature = "capture-cam")]
use core::sync::atomic::{AtomicBool, Ordering};
use ecu_core::hal::TimeSource;
use ecu_core::ts::outpc::Outpc;
use ecu_core::ts::proto::{self, Cmd};
use ecu_core::ts::serial::SerialPort;
use ecu_core::ts::OutpcProvider;
use ecu_core::EcuState;
use ecu_core::{
    dfco::{DfcoConfig, DfcoState},
    enrichment::{AeConfig, AeState, AseConfig, AseState, WueConfig},
};
use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, Rpm};
use ecu_scheduler::TransitionDrainBuffer;
#[cfg(not(feature = "flash-kv"))]
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::sensors::adc_pipeline::{
    convert_all as ts_convert_all, AdcConfig as TsAdcConfig, RawCounts as TsRawCounts,
};
use ecu_target_common::ts::service::TsService;
use ecu_target_common::ts::store::PersistedEcuPageStore;
use ecu_target_common::{
    adapter::{BoardAdapter, BoardEvent},
    bringup::bringup_fuel_model,
    control_inputs::{split_control_inputs_from, SplitControlSignals, SplitControlSignalsSource},
    noop::{NoopCapture, NoopStore, NoopTransport, NoopWatchdog},
    outputs::{ScheduledActionExecutor, ScheduledOutputs4},
    sensor_sample::{LiveLoadSensor, LoadKpa10Source},
    split_tick::run_split_scheduled_tick,
    trigger_adapter::{apply_trigger_timestamp, SplitTriggerAdapter},
};
#[cfg(all(feature = "flash-kv", target_arch = "arm"))]
#[path = "../flash_kv.rs"]
mod flash_kv;
#[cfg(all(feature = "flash-kv", not(target_arch = "arm")))]
mod seq_kv;
#[cfg(all(feature = "flash-kv", target_arch = "arm"))]
use flash_kv::FlashKv;
#[cfg(all(feature = "flash-kv", not(target_arch = "arm")))]
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
// Additional outputs for idle PWM and fan relay (adjust pins as needed)
macro_rules! IDLE_GPIO {
    ($pins:ident) => {
        $pins.gpio6.into_push_pull_output()
    };
}
macro_rules! FAN_GPIO {
    ($pins:ident) => {
        $pins.gpio7.into_push_pull_output()
    };
}
#[cfg(feature = "capture-pio")]
const TRIGGER_PIN: u8 = 4; // use with GPIO-IRQ or PIO example bins
#[cfg(feature = "capture-cam")]
const CAM_PIN: u8 = 5;

ecu_target_common::capture_ring!(CAPTURE, 128);

// Wrapped globals using cortex_m::interrupt::Mutex<RefCell<_>> for no_std-safe access
// INVARIANT: All access to these globals happens via critical sections (interrupts disabled
// or within cortex_m::interrupt::free). The RefCell borrow rules are enforced by the
// single-threaded nature of RP2040 (no Send/Sync concerns).
static SENS: Mutex<RefCell<Sensors>> = Mutex::new(RefCell::new(Sensors::new()));
static AE_STATE: Mutex<RefCell<AeState>> = Mutex::new(RefCell::new(AeState::new()));
static DFCO_STATE: Mutex<RefCell<DfcoState>> = Mutex::new(RefCell::new(DfcoState::new()));
static ASE_STATE: Mutex<RefCell<AseState>> = Mutex::new(RefCell::new(AseState::new()));
static CRANK_GATE: Mutex<RefCell<ecu_core::safety::CrankingGate>> =
    Mutex::new(RefCell::new(ecu_core::safety::CrankingGate::new()));
static CL_STATE: Mutex<RefCell<ClRuntime>> = Mutex::new(RefCell::new(ClRuntime { integ: 0 }));
#[cfg(feature = "capture-cam")]
static CAM_PHASE: AtomicBool = AtomicBool::new(false);

// Simple runtime for open-loop idle PWM
struct ClRuntime {
    integ: i32,
}

struct IdlePwmRuntime {
    last_start_us: u32,
    pin_is_high: bool,
}
impl IdlePwmRuntime {
    const fn new() -> Self {
        Self {
            last_start_us: 0,
            pin_is_high: false,
        }
    }
}

#[derive(Copy, Clone)]
struct RpTime;
impl TimeSource for RpTime {
    fn micros(&self) -> u32 {
        unsafe { &*pac::TIMER::ptr() }.timerawl.read().bits()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Rp2040MapLoad;

impl LoadKpa10Source for Rp2040MapLoad {
    type Error = core::convert::Infallible;

    fn load_kpa10(&mut self) -> Result<Kpa10, Self::Error> {
        let load_kpa10 = cortex_m::interrupt::free(|cs| {
            let sens = SENS.borrow(cs).borrow();
            sens.map_kpa_x10
        });
        Ok(Kpa10::new(load_kpa10))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Rp2040ControlSignals;

impl SplitControlSignalsSource for Rp2040ControlSignals {
    type Error = core::convert::Infallible;

    fn signals(&mut self, ignition_rpm: Rpm) -> Result<SplitControlSignals, Self::Error> {
        let (clt_c, lambda_x100, mapdot_kpa_s, tpsdot_pct_s) = cortex_m::interrupt::free(|cs| {
            let sens = SENS.borrow(cs).borrow();
            (
                sens.clt_c,
                sens.lambda_x100,
                sens.mapdot_kpa_s,
                sens.tpsdot_pct_s,
            )
        });
        Ok(SplitControlSignals {
            clt_c,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(lambda_x100),
            requested_open_loop: false,
            tpsdot_pct_s,
            mapdot_kpa_s,
            spark_advance_x10: Degrees10::new(100),
            ignition_rpm,
        })
    }
}

// CdcSerial is provided by ecu-target-common

struct Provider {
    state: *const EcuState,
}
impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        let s = unsafe { &*self.state };
        // Use critical section for shared mutable state access - extract individual Copy values
        let (
            map_kpa_x10,
            tps_percent,
            clt_c,
            iat_c,
            vbatt_mv,
            lambda_x100,
            mapdot_kpa_s,
            tpsdot_pct_s,
        ) = cortex_m::interrupt::free(|cs| {
            let sens = SENS.borrow(cs).borrow();
            (
                sens.map_kpa_x10,
                sens.tps_percent,
                sens.clt_c,
                sens.iat_c,
                sens.vbatt_mv,
                sens.lambda_x100,
                sens.mapdot_kpa_s,
                sens.tpsdot_pct_s,
            )
        });
        out.rpm = s.rpm();
        out.map_kpa_x10 = map_kpa_x10;
        out.tps_percent = tps_percent;
        out.clt_c = clt_c;
        out.iat_c = iat_c;
        out.vbatt_mv = vbatt_mv;
        out.lambda_x100 = lambda_x100;
        // Example fuel calc with live enrichment state and a simple CL loop.
        let wue_pct = WueConfig::DEFAULT.compute_percent(clt_c);
        let ae_pct = cortex_m::interrupt::free(|cs| {
            AE_STATE.borrow(cs).borrow_mut().update(
                RpTime.micros(),
                tpsdot_pct_s,
                mapdot_kpa_s,
                &AeConfig::DEFAULT,
            )
        });
        let ase_pct = cortex_m::interrupt::free(|cs| {
            ASE_STATE
                .borrow(cs)
                .borrow_mut()
                .update(RpTime.micros(), false, &AseConfig::DEFAULT)
        });
        let cl_cfg = &s.config.cl_config;
        // Compute target lambda from AFR target (approx gasoline stoich 14.7)
        let target_lambda_x100 = ((cl_cfg.target_afr_x10 as u32) * 1000 / 147) as i32;
        let lambda_meas_x100 = lambda_x100 as i32;
        let error = target_lambda_x100 - lambda_meas_x100; // positive -> richer target than measured
                                                           // Proportional gain: kp_i is percent per 100 lambda error
        let kp = cl_cfg.kp_i as i32;
        let mut cl_delta = (error * kp) / 100; // i16 percent
                                               // Integral term (simple accumulator, clamped)
        let ki = cl_cfg.ki_i as i32;
        if ki > 0 {
            let st = cortex_m::interrupt::free(|cs| {
                let mut cl = CL_STATE.borrow(cs).borrow_mut();
                cl.integ = (cl.integ + (error * ki) / 100).clamp(-50, 50);
                cl.integ
            });
            cl_delta += st;
        } else {
            cortex_m::interrupt::free(|cs| {
                CL_STATE.borrow(cs).borrow_mut().integ = 0;
            });
        }
        // Clamp to +/-25%
        cl_delta = cl_delta.clamp(-25, 25);
        out.pw_us = s.calculate_fuel_with_enrichments(
            s.rpm(),
            map_kpa_x10 / 10,
            wue_pct,
            ase_pct,
            ae_pct,
            cl_delta as i16,
        );
        out.dwell_us = s.calculate_dwell() as u16;
        out.advance_x10 = 150;
        out.synced = if s.synced() { 1 } else { 0 };
        // Extended fields
        out.target_afr_x10 = s.config.cl_config.target_afr_x10;
        out.ego_correction_percent = (100 + cl_delta).clamp(0, 200) as u8;
        out.ego_sensor = match s.lambda_state.sensor_type {
            ecu_core::lambda::O2SensorType::Narrowband => 1,
            ecu_core::lambda::O2SensorType::Wideband => 2,
        };
        out.mapdot_kpa_s = mapdot_kpa_s;
        out.tpsdot_pct_s = tpsdot_pct_s;
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
        // WUE (based on config and CLT)
        if wue_pct > 0 {
            flags |= 1 << 0;
        }
        // AE: already updated above in ae_pct calculation
        if ae_pct > 0 {
            flags |= 1 << 4;
        }
        // ASE: trigger when leaving cranking
        let just_started = cortex_m::interrupt::free(|cs| {
            let mut crank_gate = CRANK_GATE.borrow(cs).borrow_mut();
            let prev_crank = crank_gate.is_cranking();
            let _ = crank_gate.update(out.rpm);
            prev_crank && !crank_gate.is_cranking()
        });
        let ase = cortex_m::interrupt::free(|cs| {
            ASE_STATE.borrow(cs).borrow_mut().update(
                RpTime.micros(),
                just_started,
                &AseConfig::DEFAULT,
            )
        });
        if ase > 0 {
            flags |= 1 << 1;
        }
        // DFCO
        let dfco = cortex_m::interrupt::free(|cs| {
            DFCO_STATE.borrow(cs).borrow_mut().update(
                RpTime.micros(),
                out.rpm,
                tps_percent,
                map_kpa_x10 / 10,
                &DfcoConfig::DEFAULT,
            )
        });
        if dfco {
            flags |= 1 << 3;
        }
        // Emergency mode (bit5)
        if s.emergency_mode() {
            flags |= 1 << 5;
        }
        out.engine_state = flags;
        out.baro_kpa = 100;
        out.gear = 0;
    }

    fn engine_running(&self) -> bool {
        unsafe {
            let s = &*self.state;
            s.synced() && s.rpm() > 0
        }
    }
}

// Basic ADC sensor reader with simple scaling and plausibility checks
struct Sensors {
    map_kpa_x10: u16,
    tps_percent: u8,
    clt_c: i16,
    iat_c: i16,
    vbatt_mv: u16,
    lambda_x100: u16,
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
            lambda_x100: 100,
            mapdot_kpa_s: 0,
            tpsdot_pct_s: 0,
            last_map_kpa_x10: 1000,
            last_tps_percent: 0,
            last_ts_us: 0,
        }
    }

    fn update(
        &mut self,
        adc: &mut Adc,
        pins: &mut AdcPins,
        cal: &ecu_core::sensors::SensorsCal,
        state: &mut ecu_core::EcuState,
    ) {
        // Read raw counts
        let iat_counts = adc.read(&mut pins.iat).unwrap_or(0);
        let raw = TsRawCounts {
            map: adc.read(&mut pins.map).unwrap_or(0),
            tps: adc.read(&mut pins.tps).unwrap_or(0),
            clt: adc.read(&mut pins.clt).unwrap_or(0),
            iat: iat_counts,
            // Reuse IAT channel for VBATT when using VSYS/3
            vbatt: iat_counts,
            lambda: iat_counts, // bring-up lambda input reuses the same analog channel
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
        let (map_clamped, tps_clamped) = state.process_sensor_update(
            ecu_core::Micros(now),
            ecu_core::Kpa10(out.map_kpa_x10),
            out.tps_percent,
        );

        // Slew rate limits (conservative defaults): MAP 1000 kPa×10/s, TPS 300 %/s
        let dt_us = if self.last_ts_us == 0 {
            0
        } else {
            now.wrapping_sub(self.last_ts_us)
        };
        let map_slewed = ecu_target_common::sensors::adc_pipeline::clamp_slew_u16(
            self.last_map_kpa_x10,
            map_clamped.0,
            1000,
            dt_us,
        );
        let tps_slewed = ecu_target_common::sensors::adc_pipeline::clamp_slew_u8(
            self.last_tps_percent,
            tps_clamped,
            300,
            dt_us,
        );
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
        self.lambda_x100 = out.lambda_x100;

        // Derivatives based on time delta
        let rp = RpTime;
        let now = rp.micros();
        let dt_us = now.wrapping_sub(self.last_ts_us);
        if dt_us > 0 {
            let d_map_x10 = self.map_kpa_x10 as i32 - self.last_map_kpa_x10 as i32;
            let num = d_map_x10.saturating_mul(1_000_000); // scale to per-second
            let den = (10 * (dt_us as i32)).max(1);
            let mapdot = num / den; // kPa/s
            self.mapdot_kpa_s = mapdot.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            let d_tps = self.tps_percent as i32 - self.last_tps_percent as i32;
            let num_tps = d_tps.saturating_mul(1_000_000);
            let tpsdot = num_tps / (dt_us as i32).max(1);
            self.tpsdot_pct_s = tpsdot.clamp(i16::MIN as i32, i16::MAX as i32) as i16;

            self.last_map_kpa_x10 = self.map_kpa_x10;
            self.last_tps_percent = self.tps_percent;
            self.last_ts_us = now;
        } else if self.last_ts_us == 0 {
            self.last_map_kpa_x10 = self.map_kpa_x10;
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
    let mut pac = pac::Peripherals::take().expect("Peripherals already taken");
    let _core = pac::CorePeripherals::take().expect("CorePeripherals already taken");
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
    .expect("clock initialization failed");

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
        // Route the configured trigger pin to PIO0. Keep GPIO0..GPIO3 owned by
        // injector/ignition outputs below.
        let _trigger = pins.gpio4.into_function::<FunctionPio0>();
        let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
        let program = pio_proc::pio_asm!(
            ".wrap_target",
            "wait 0 pin 0",
            "wait 1 pin 0",
            "irq 0",
            ".wrap",
        );
        let installed = pio
            .install(&program.program)
            .expect("PIO program install failed");
        let (mut sm, _rx, _tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
            .in_pin_base(TRIGGER_PIN)
            .clock_divisor_fixed_point(1, 0)
            .build(sm0);
        sm.set_pindirs([]);
        let irq0 = pio.irq0();
        pio.clear_irq(1);
        irq0.enable_sm_interrupt(0);
        let _sm = sm.start();
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
        }
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

    // TS state remains root-backed until the TS snapshot/page-store migration.
    let state = cortex_m::singleton!(: EcuState = EcuState::new())
        .expect("EcuState singleton already taken");

    // Prepare scheduled outputs using the split scheduler path.
    let inj1 = INJ1_GPIO!(pins);
    let inj2 = INJ2_GPIO!(pins);
    let ign1 = IGN1_GPIO!(pins);
    let ign2 = IGN2_GPIO!(pins);
    let mut outputs = ScheduledOutputs4::new(inj1, inj2, ign1, ign2);
    let mut drain = TransitionDrainBuffer::<8>::new();
    let mut adapter = BoardAdapter::new(
        LiveLoadSensor::new(RpTime, Rp2040MapLoad),
        NoopCapture,
        ScheduledActionExecutor::<8>::new(),
        NoopWatchdog,
        NoopTransport,
        NoopStore,
    );
    adapter.configure_fuel_model(bringup_fuel_model());
    let mut control_signals = Rp2040ControlSignals;
    let mut trigger_adapter = SplitTriggerAdapter::new(RpTime);
    #[cfg(not(feature = "capture-pio"))]
    let mut simulated_trigger_tooth: u8 = 0;
    // Extra outputs
    let mut idle_pin = IDLE_GPIO!(pins);
    let mut fan_pin = FAN_GPIO!(pins);
    let mut idle_pwm = IdlePwmRuntime::new();

    // TS service (uses shared TsService; KV is feature-selectable)
    let provider = Provider {
        state: state as *const EcuState,
    };
    #[cfg(all(feature = "flash-kv", target_arch = "arm"))]
    let mut store = PersistedEcuPageStore::new(state, FlashKv::new_with_state(state));
    #[cfg(all(feature = "flash-kv", not(target_arch = "arm")))]
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
        map: AdcPin::new(pins.gpio26.into_pull_down_disabled()),
        tps: AdcPin::new(pins.gpio27.into_pull_down_disabled()),
        clt: AdcPin::new(pins.gpio28.into_pull_down_disabled()),
        iat: AdcPin::new(pins.gpio29.into_pull_down_disabled()),
    };

    // Optional: configure CAM input via IO_IRQ_BANK0
    #[cfg(feature = "capture-cam")]
    {
        let cam = pins.gpio5.into_pull_up_input();
        cam.set_interrupt_enabled(EdgeHigh, true);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
        }
    }

    #[cfg(feature = "capture-cam")]
    let mut last_cam_phase = CAM_PHASE.load(Ordering::Relaxed);

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
                            let (injectors, ignition) = outputs.as_scheduled_pins();
                            for _ in 0..reps {
                                match chan {
                                    0 => {
                                        injectors[0].set_scheduled_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        injectors[0].set_scheduled_low();
                                    }
                                    1 => {
                                        injectors[1].set_scheduled_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        injectors[1].set_scheduled_low();
                                    }
                                    2 => {
                                        ignition[0].set_scheduled_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        ignition[0].set_scheduled_low();
                                    }
                                    3 => {
                                        ignition[1].set_scheduled_high();
                                        cortex_m::asm::delay(on_ms * 1000 * 125);
                                        ignition[1].set_scheduled_low();
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
                        let rpm = state.rpm;
                        let synced = state.synced as u8;
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
        let sensors_cal = state.config.sensors_cal;
        cortex_m::interrupt::free(|cs| {
            SENS.borrow(cs)
                .borrow_mut()
                .update(&mut adc, &mut adc_pins, &sensors_cal, state)
        });

        // Fan control (on/off with hysteresis)
        {
            let fan_cfg = state.config.fan_config;
            if fan_cfg.enable {
                let clt = cortex_m::interrupt::free(|cs| SENS.borrow(cs).borrow().clt_c);
                if clt >= fan_cfg.on_c {
                    let _ = fan_pin.set_high();
                } else if clt <= fan_cfg.off_c {
                    let _ = fan_pin.set_low();
                }
            } else {
                let _ = fan_pin.set_low();
            }
        }

        // Idle PWM (open-loop)
        {
            let cfg = state.config.idle_config;
            if !cfg.enable || cfg.duty_x10 == 0 || cfg.freq_hz == 0 {
                let _ = idle_pin.set_low();
                idle_pwm.pin_is_high = false;
                idle_pwm.last_start_us = RpTime.micros();
            } else {
                let now = RpTime.micros();
                let period_us =
                    (1_000_000u32).saturating_div(core::cmp::max(1, cfg.freq_hz as u32));
                let on_us = (period_us as u64 * (cfg.duty_x10 as u64) / 1000u64) as u32;
                let elapsed = now.wrapping_sub(idle_pwm.last_start_us);
                if elapsed >= period_us {
                    idle_pwm.last_start_us = now;
                    if on_us > 0 {
                        let _ = idle_pin.set_high();
                        idle_pwm.pin_is_high = true;
                    } else {
                        let _ = idle_pin.set_low();
                        idle_pwm.pin_is_high = false;
                    }
                } else if idle_pwm.pin_is_high && elapsed >= on_us {
                    let _ = idle_pin.set_low();
                    idle_pwm.pin_is_high = false;
                }
            }
        }

        while let Some(ts) = capture_pop() {
            let _ = apply_trigger_timestamp(&mut adapter, &mut trigger_adapter, ts);
            #[cfg(not(feature = "capture-cam"))]
            let _ = adapter.apply_event(BoardEvent::CamEdge {
                at_us: Micros::new(ts),
                cam_seen: true,
            });
        }
        let _ = adapter.poll_sensor();

        let now = Micros::new(RpTime.micros());
        let _ = run_split_scheduled_tick(
            &mut adapter,
            now,
            split_control_inputs_from(&mut control_signals, now, trigger_adapter.rpm())
                .unwrap_or_else(|never| match never {}),
            &mut outputs,
            &mut drain,
        );

        // Optional: simulate trigger edges if capture-pio not enabled
        #[cfg(not(feature = "capture-pio"))]
        {
            let delay_cycles = if simulated_trigger_tooth == 57 {
                48_000
            } else {
                24_000
            };
            cortex_m::asm::delay(delay_cycles);
            capture_push(RpTime.micros());
            simulated_trigger_tooth = if simulated_trigger_tooth == 57 {
                0
            } else {
                simulated_trigger_tooth + 1
            };
        }

        // If cam phase toggled, notify the split runtime.
        #[cfg(feature = "capture-cam")]
        {
            let cam_phase = CAM_PHASE.load(Ordering::Relaxed);
            if cam_phase != last_cam_phase {
                last_cam_phase = cam_phase;
                let _ = adapter.apply_event(BoardEvent::CamEdge {
                    at_us: Micros::new(RpTime.micros()),
                    cam_seen: true,
                });
            }
        }
    }
}

#[cfg(feature = "capture-pio")]
#[allow(non_snake_case)]
#[interrupt]
fn PIO0_IRQ_0() {
    capture_push(RpTime.micros());
    let pio = unsafe { &*pac::PIO0::ptr() };
    pio.irq.write(|w| unsafe { w.irq().bits(1) });
}

#[cfg(feature = "capture-cam")]
#[allow(non_snake_case)]
#[interrupt]
fn IO_IRQ_BANK0() {
    // Toggle phase on cam rising edge
    CAM_PHASE.store(!CAM_PHASE.load(Ordering::Relaxed), Ordering::Relaxed);
    let io = unsafe { &*pac::IO_BANK0::ptr() };
    let group = (CAM_PIN as usize) / 8;
    let edge_high_mask = 0b1000u32 << (((CAM_PIN as u32) & 0x7) * 4);
    io.intr[group].write(|w| unsafe { w.bits(edge_high_mask) });
}
