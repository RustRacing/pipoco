//! STM32F4 ECU Application
//!
//! Bare-metal ECU implementation for STM32F405 microcontroller.
//! Uses the ecu-core library with STM32F4-specific HAL implementations.
//!
//! # Hardware Configuration
//!
//! - **Clock**: 168 MHz system clock
//! - **Timer**: TIM2 configured for microsecond counting
//! - **Trigger Input**: PA0 (rising edge interrupt)
//! - **Outputs**:
//!   - PB0: Injector 1
//!   - PB1: Injector 2
//!   - PB2: Ignition coil 1
//!   - PB3: Ignition coil 2
//!
//! # Memory Safety
//!
//! Uses critical sections (`cortex_m::interrupt::free`) to safely access
//! shared state between ISR and main loop.

#![no_std]
#![no_main]

use cortex_m::interrupt::free as critical_section;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f4xx_hal::{pac, prelude::*};

mod hal_impl;
use ecu_core::constants::fuel::DEFAULT_LOAD_KPA;
use ecu_core::constants::timing::*;
use ecu_core::safety::{apply_safe_state, should_allow_injection, update_flood_clear, OutputLatch};
use ecu_core::{CaptureBuffer, EcuApp};
use hal_impl::Stm32Time;
use hal_impl::{Stm32Reset, Stm32Watchdog};
#[cfg(feature = "ts-usb")]
mod ts_support {
    #![allow(dead_code)]
    use ecu_core::persist::{KvError, KvStore};
    use ecu_core::ts::outpc::Outpc;
    use ecu_core::ts::pages::EcuPageStore;
    use ecu_core::ts::OutpcProvider;
    use ecu_core::EcuState;
    use ecu_target_common::ts::service::TsService;
    use ecu_target_common::ts::store::PersistedEcuPageStore;
    use stm32f4xx_hal::pac;

    // Minimal RAM KV (512-byte pages) for bring-up
    use ecu_target_common::kv::ram::RamKv512;

    // Optional: Flash KV placeholder (per-session RAM mirror; TODO: real flash)
    #[cfg(feature = "flash-kv")]
    pub struct FlashKv;
    #[cfg(feature = "flash-kv")]
    impl FlashKv {
        pub const fn new() -> Self {
            Self
        }
    }
    #[cfg(feature = "flash-kv")]
    impl KvStore for FlashKv {
        fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
            const BASE_A: u32 = 0x080C0000; // Sector 10
            const BASE_B: u32 = 0x080E0000; // Sector 11
            const HDR_SZ: usize = 64;
            const FUEL_OFF: usize = HDR_SZ;
            const IGN_OFF: usize = HDR_SZ + 512;
            const MAGIC: u32 = 0x32504B56; // 'V''K''P''2' LE

            unsafe fn read_header(base: u32) -> (bool, u16, u16, u16, u16) {
                let p = base as *const u8;
                let magic = core::ptr::read_volatile(p as *const u32);
                if magic != MAGIC {
                    return (false, 0, 0, 0, 0);
                }
                let valid = core::ptr::read_volatile(p.add(6) as *const u16);
                if valid != 0 {
                    return (false, 0, 0, 0, 0);
                }
                let seq = core::ptr::read_volatile(p.add(8) as *const u16);
                let fuel_len = core::ptr::read_volatile(p.add(10) as *const u16);
                let ign_len = core::ptr::read_volatile(p.add(12) as *const u16);
                (true, seq, fuel_len, ign_len, 0)
            }

            unsafe fn read_page(base: u32, off: usize, out: &mut [u8]) {
                let p = base as *const u8;
                for i in 0..out.len() {
                    out[i] = core::ptr::read_volatile(p.add(off + i));
                }
            }

            // Choose newest valid sector by seq
            let (a_ok, a_seq, a_flen, a_ilen, _a_rsv) = unsafe { read_header(BASE_A) };
            let (b_ok, b_seq, b_flen, b_ilen, _b_rsv) = unsafe { read_header(BASE_B) };
            let pick_b = b_ok && (!a_ok || b_seq.wrapping_sub(a_seq) < 0x8000);
            let base = if pick_b { BASE_B } else { BASE_A };
            let flen = if pick_b { b_flen } else { a_flen } as usize;
            let ilen = if pick_b { b_ilen } else { a_ilen } as usize;
            if key == b"fuel" {
                if flen != 512 || out.len() < 512 {
                    return Err(KvError::NotFound);
                }
                unsafe {
                    read_page(base, FUEL_OFF, &mut out[..512]);
                }
                Ok(512)
            } else if key == b"ign" {
                if ilen != 512 || out.len() < 512 {
                    return Err(KvError::NotFound);
                }
                unsafe {
                    read_page(base, IGN_OFF, &mut out[..512]);
                }
                Ok(512)
            } else {
                Err(KvError::Io)
            }
        }

        fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
            if data.len() != 512 {
                return Err(KvError::Io);
            }
            const BASE_A: u32 = 0x080C0000; // Sector 10
            const BASE_B: u32 = 0x080E0000; // Sector 11
            const SECTOR_A: u8 = 10;
            const SECTOR_B: u8 = 11;
            const HDR_SZ: usize = 64;
            const FUEL_OFF: usize = HDR_SZ;
            const IGN_OFF: usize = HDR_SZ + 512;
            const MAGIC: u32 = 0x32504B56; // 'V''K''P''2'

            unsafe fn read_seq(base: u32) -> Option<u16> {
                let p = base as *const u8;
                if core::ptr::read_volatile(p as *const u32) != MAGIC {
                    return None;
                }
                if core::ptr::read_volatile(p.add(6) as *const u16) != 0 {
                    return None;
                }
                Some(core::ptr::read_volatile(p.add(8) as *const u16))
            }

            // Read existing pages and sequence
            let mut fuel = [0u8; 512];
            let mut ign = [0u8; 512];
            let mut seq_a = None;
            let mut seq_b = None;
            let _ = self.read(b"fuel", &mut fuel); // ignore NotFound
            let _ = self.read(b"ign", &mut ign);
            unsafe {
                seq_a = read_seq(BASE_A);
                seq_b = read_seq(BASE_B);
            }
            let cur_seq = match (seq_a, seq_b) {
                (Some(a), Some(b)) => {
                    if b.wrapping_sub(a) < 0x8000 {
                        b
                    } else {
                        a
                    }
                }
                (Some(a), None) => a,
                (None, Some(b)) => b,
                _ => 0,
            };
            if key == b"fuel" {
                fuel.copy_from_slice(data);
            } else if key == b"ign" {
                ign.copy_from_slice(data);
            } else {
                return Err(KvError::Io);
            }

            // Prepare header (valid field stays 0xFFFF until the very end)
            let mut hdr = [0xFFu8; HDR_SZ];
            // magic 'VKP2' LE
            hdr[0] = 0x56;
            hdr[1] = 0x4B;
            hdr[2] = 0x50;
            hdr[3] = 0x32;
            // version
            hdr[4] = 2;
            hdr[5] = 0;
            // valid (u16) at [6..8] left as 0xFFFF until commit
            // seq
            let new_seq = cur_seq.wrapping_add(1);
            hdr[8] = (new_seq & 0xFF) as u8;
            hdr[9] = (new_seq >> 8) as u8;
            // lengths
            hdr[10] = 0x00;
            hdr[11] = 0x02; // fuel 512
            hdr[12] = 0x00;
            hdr[13] = 0x02; // ign  512
                            // CRC16s
            let fuel_crc = ecu_core::ts::proto::crc16_ccitt(&fuel);
            let ign_crc = ecu_core::ts::proto::crc16_ccitt(&ign);
            hdr[14] = (fuel_crc & 0xFF) as u8;
            hdr[15] = (fuel_crc >> 8) as u8;
            hdr[16] = (ign_crc & 0xFF) as u8;
            hdr[17] = (ign_crc >> 8) as u8;

            cortex_m::interrupt::free(|_| unsafe {
                let flash = &*pac::FLASH::ptr();
                // Select target sector as the opposite of current valid
                let target = match (seq_a.is_some(), seq_b.is_some()) {
                    (true, false) => (SECTOR_B, BASE_B),
                    (false, true) => (SECTOR_A, BASE_A),
                    (false, false) => (SECTOR_A, BASE_A),
                    (true, true) => {
                        // choose other than the newest
                        if seq_b.unwrap().wrapping_sub(seq_a.unwrap()) < 0x8000 {
                            (SECTOR_A, BASE_A)
                        } else {
                            (SECTOR_B, BASE_B)
                        }
                    }
                };
                let (target_sector, target_base) = target;

                // Wait for not busy and unlock
                while flash.sr.read().bsy().bit_is_set() {}
                if flash.cr.read().lock().bit_is_set() {
                    flash.keyr.write(|w| w.key().bits(0x4567_0123));
                    flash.keyr.write(|w| w.key().bits(0xCDEF_89AB));
                }

                // Erase target sector only (other sector remains valid if power fails)
                while flash.sr.read().bsy().bit_is_set() {}
                flash
                    .cr
                    .modify(|_, w| w.ser().set_bit().snb().bits(target_sector));
                flash.cr.modify(|_, w| w.strt().set_bit());
                while flash.sr.read().bsy().bit_is_set() {}
                flash.cr.modify(|_, w| w.ser().clear_bit());

                // Program half-words (16-bit) with PSIZE=01 and PG=1
                flash
                    .cr
                    .modify(|_, w| unsafe { w.psize().bits(0b01) }.pg().set_bit());
                let mut prog_half = |addr: u32, val: u16| {
                    core::ptr::write_volatile(addr as *mut u16, val);
                    while flash.sr.read().bsy().bit_is_set() {}
                };

                // Write header (without committing valid field)
                let mut a = target_base;
                for i in (0..HDR_SZ).step_by(2) {
                    let v = (hdr[i] as u16) | ((hdr[i + 1] as u16) << 8);
                    prog_half(a, v);
                    a += 2;
                }
                // Write fuel page
                let mut a = target_base + (FUEL_OFF as u32);
                for i in (0..512).step_by(2) {
                    let v = (fuel[i] as u16) | ((fuel[i + 1] as u16) << 8);
                    prog_half(a, v);
                    a += 2;
                }
                // Write ign page
                let mut a = target_base + (IGN_OFF as u32);
                for i in (0..512).step_by(2) {
                    let v = (ign[i] as u16) | ((ign[i + 1] as u16) << 8);
                    prog_half(a, v);
                    a += 2;
                }

                // Commit: set valid field to 0x0000 last (atomic 1->0 transition)
                let valid_addr = target_base + 6; // offset of valid u16
                prog_half(valid_addr, 0x0000);

                // Clear PG and lock
                flash.cr.modify(|_, w| w.pg().clear_bit());
                flash.cr.modify(|_, w| w.lock().set_bit());
            });

            Ok(())
        }
    }

    // Choose KV impl
    #[cfg(feature = "flash-kv")]
    type StoreKv = FlashKv;
    #[cfg(not(feature = "flash-kv"))]
    type StoreKv = RamKv512;

    // Provider reading from EcuState
    pub struct Provider {
        pub state: *const EcuState,
    }
    impl OutpcProvider for Provider {
        fn fill_outpc(&self, out: &mut Outpc) {
            let s = unsafe { &*self.state };
            // Live sensors from our simple sampler
            let sens = unsafe { &SENS };
            out.rpm = s.rpm;
            out.map_kpa_x10 = sens.map_kpa_x10;
            out.tps_percent = sens.tps_percent;
            out.clt_c = sens.clt_c;
            out.iat_c = sens.iat_c;
            out.vbatt_mv = sens.vbatt_mv;
            out.lambda_x100 = 100;
            out.pw_us = s.calculate_fuel(2000, 100);
            out.dwell_us = 3000;
            out.advance_x10 = 150;
            out.synced = if s.synced { 1 } else { 0 };
        }
    }

    // Minimal sensor sampler (placeholder for ADC-backed implementation)
    #[derive(Copy, Clone)]
    pub struct Sensors {
        pub map_kpa_x10: u16,
        pub tps_percent: u8,
        pub clt_c: i16,
        pub iat_c: i16,
        pub vbatt_mv: u16,
        last_tick: u32,
    }
    impl Sensors {
        pub const fn new() -> Self {
            Self {
                map_kpa_x10: 1000,
                tps_percent: 0,
                clt_c: 20,
                iat_c: 25,
                vbatt_mv: 12500,
                last_tick: 0,
            }
        }
        pub fn update(&mut self, now_us: u32, state: &EcuState) {
            // Placeholder dynamics: slowly vary TPS between 0..30% and map 90..110 kPa
            let dt = now_us.wrapping_sub(self.last_tick);
            if dt > 50_000 {
                // ~20 Hz
                self.last_tick = now_us;
                self.tps_percent = self.tps_percent.wrapping_add(1) % 30;
                let base = 1000i32 + ((now_us / 250_000) as i32 % 21) - 10; // 90..110 kPa
                self.map_kpa_x10 = base.clamp(0, 65535) as u16;
                // Mirror battery from state
                self.vbatt_mv = state.battery_voltage_mv;
                // Hold temperatures constant for now
                self.clt_c = 20;
                self.iat_c = 25;
            }
        }
    }

    // Global sensors (TS feature scope)
    pub static mut SENS: Sensors = Sensors::new();

    pub type Service<'a> = TsService<Provider, PersistedEcuPageStore<'a, StoreKv>>;
    pub fn new_service<'a>(state: &'a mut EcuState) -> Service<'a> {
        let provider = Provider {
            state: state as *const _,
        };
        let mut store = PersistedEcuPageStore::new(state, StoreKv::new());
        store.try_load();
        TsService::new(b"IPW-ECU V0.1", provider, store)
    }
}

// Arduino-style global pin declarations for easy remapping
// Adjust these macros to change physical pin assignments
macro_rules! INJ1_PIN {
    ($gpiob:ident) => {
        $gpiob.pb0.into_push_pull_output()
    };
}
macro_rules! INJ2_PIN {
    ($gpiob:ident) => {
        $gpiob.pb1.into_push_pull_output()
    };
}
macro_rules! IGN1_PIN {
    ($gpiob:ident) => {
        $gpiob.pb2.into_push_pull_output()
    };
}
macro_rules! IGN2_PIN {
    ($gpiob:ident) => {
        $gpiob.pb3.into_push_pull_output()
    };
}

// Global state (interrupt accessible)
static mut APP: Option<EcuApp<Stm32Time>> = None;
ecu_target_common::capture_ring!(CAPTURE, 128);

// Output pin states (stored separately to avoid complex lifetime issues in main loop)
static mut INJ1_STATE: bool = false;
static mut INJ2_STATE: bool = false;
static mut IGN1_STATE: bool = false;
static mut IGN2_STATE: bool = false;

// Error tracking
static mut SCHEDULER_FULL_COUNT: u32 = 0;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Setup clocks (168MHz)
    let rcc = dp.RCC.constrain();
    // Configure system clock; for USB we need a valid 48MHz USB clock derived from PLL
    let clocks = rcc.cfgr.sysclk(168.MHz()).require_pll48clk().freeze();

    // Setup timer for microsecond counter
    // TIM2 is 32-bit, perfect for microsecond timing
    //
    // IMPORTANT: Timer prescaler calculation
    // - APB1 clock = 168MHz / 2 = 84MHz (divided by APB1 prescaler)
    // - Timer clock = 84MHz * 2 = 168MHz (TIMx clock multiplier when APB prescaler != 1)
    // - For 1μs ticks: prescaler = 168
    // - PSC register value = 168 - 1 = 167
    //
    // With PSC=167: 168MHz / 168 = 1MHz = 1μs per tick ✓
    dp.TIM2.psc.write(|w| w.psc().bits(168 - 1));
    dp.TIM2.arr.write(|w| w.arr().bits(u32::MAX)); // Max count for 32-bit timer
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit()); // Enable counter

    // Create time source (zero-sized type, just uses TIM2 pointer)
    let time_source = Stm32Time;

    let gpioa = dp.GPIOA.split();

    #[cfg(feature = "capture-tim")]
    {
        // PA0 as TIM2_CH1 (AF1)
        let _trigger_pin = gpioa.pa0.into_alternate();
        // Configure TIM2 CH1 input capture on rising edge with CC1 interrupt
        dp.TIM2.ccmr1_input().modify(|_, w| w.cc1s().ti1());
        dp.TIM2.ccer.modify(|_, w| {
            w.cc1p().clear_bit();
            w.cc1e().set_bit()
        });
        dp.TIM2.dier.modify(|_, w| w.cc1ie().set_bit());
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
        }
    }

    #[cfg(feature = "capture-gpio")]
    {
        // PA0 as input with EXTI0 on rising edge
        let _trigger_pin = gpioa.pa0.into_pull_up_input();
        dp.SYSCFG.exticr[0].modify(|_, w| unsafe { w.exti0().bits(0) });
        dp.EXTI.imr.modify(|_, w| w.mr0().set_bit());
        dp.EXTI.rtsr.modify(|_, w| w.tr0().set_bit());
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI0);
        }
    }

    // Initialize app runtime
    critical_section(|_cs| unsafe {
        APP = Some(EcuApp::new(time_source));
    });

    // Start independent watchdog (~250ms) and clear reset flags (best-effort)
    let mut iwdg = Stm32Watchdog::new(dp.IWDG);
    iwdg.start(250);
    let mut rst = Stm32Reset::new(dp.RCC);
    rst.clear();

    // Enable timer IRQ only in capture-tim path (done above)

    // Setup output pins for injectors/coils
    let gpiob = dp.GPIOB.split();
    // Use macros above to obtain output pins; edit macros to remap
    let inj1 = INJ1_PIN!(gpiob);
    let inj2 = INJ2_PIN!(gpiob);
    let ign1 = IGN1_PIN!(gpiob);
    let ign2 = IGN2_PIN!(gpiob);

    // Force safe state on boot and wrap via shared Outputs4
    let _ = inj1.set_low();
    let _ = inj2.set_low();
    let _ = ign1.set_low();
    let _ = ign2.set_low();
    let mut outs4 = ecu_target_common::outputs::Outputs4::new(inj1, inj2, ign1, ign2);

    // Optional: TunerStudio USB hardware bring-up (feature-gated)
    #[cfg(feature = "ts-usb-hw")]
    let (mut maybe_ts, mut maybe_cdc) = {
        use ecu_target_common::ts::usb_cdc::CdcSerial;
        use stm32f4xx_hal::otg_fs::{UsbBusType, USB};
        use usb_device::prelude::*;
        // Configure USB pins PA11/PA12 to AF10
        let usb_dm = gpioa.pa11.into_alternate();
        let usb_dp = gpioa.pa12.into_alternate();
        // Allocate USB bus
        static mut USB_ALLOC: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;
        let mut cdc_opt: Option<CdcSerial<UsbBusType>> = None;
        unsafe {
            let usb = USB {
                usb_global: dp.OTG_FS_GLOBAL,
                usb_device: dp.OTG_FS_DEVICE,
                usb_pwrclk: dp.OTG_FS_PWRCLK,
                pin_dm: usb_dm,
                pin_dp: usb_dp,
            };
            USB_ALLOC = Some(UsbBusType::new(usb, &clocks));
            let bus = USB_ALLOC.as_ref().unwrap();
            let serial = usbd_serial::SerialPort::new(bus);
            let dev = UsbDeviceBuilder::new(bus, UsbVidPid(0x1d50, 0x6130))
                .manufacturer("IPW")
                .product("TS-ECU")
                .serial_number("STM32F4-TS")
                .device_class(USB_CLASS_CDC)
                .build();
            cdc_opt = Some(CdcSerial { serial, dev });
        }
        let svc = critical_section(|_| unsafe {
            if let Some(ref mut a) = APP {
                ts_support::new_service(&mut a.state)
            } else {
                ts_support::new_service(&mut EcuApp::new(Stm32Time).state)
            }
        });
        (Some(svc), cdc_opt)
    };

    // Main loop - check scheduled events and update pin states
    loop {
        // Update sensors for TS (placeholder or ADC sampling if enabled)
        #[cfg(feature = "ts-usb")]
        {
            use ts_support::SENS;
            let now = Stm32Time.micros();
            unsafe {
                if let Some(ref a) = APP {
                    SENS.update(now, &a.state);
                }
            }
        }

        ecu_target_common::tick_once!(
            APP,
            capture_pop,
            critical_section(|_| unsafe { APP.as_ref().map(|a| a.now()).unwrap_or(0) }),
            outs4.as_pins(),
            {
                #[cfg(feature = "ts-usb-hw")]
                {
                    if let (Some(ref mut svc), Some(ref mut cdc)) = (&mut maybe_ts, &mut maybe_cdc)
                    {
                        svc.pump_with_budget(cdc, 4);
                    }
                }
            },
            iwdg.pet()
        );
    }
}

/// TIM2 interrupt handler
/// - Captures timestamps on CH1 rising edges and pushes into ring buffer
/// - Schedules events when decoder is updated (minimal extra work here)
#[cfg(feature = "capture-tim")]
#[cortex_m_rt::interrupt]
fn TIM2() {
    unsafe {
        // If capture occurred on CH1
        let tim2 = &(*pac::TIM2::ptr());
        if tim2.sr.read().cc1if().bit_is_set() {
            let captured = tim2.ccr1.read().ccr().bits();
            // Clear CC1IF by reading SR then CCR1 (done) and writing 0 to it
            tim2.sr.modify(|_, w| w.cc1if().clear());
            // Push timestamp for main loop processing
            capture_push(captured);
        }
    }
}

#[cfg(feature = "capture-gpio")]
#[cortex_m_rt::interrupt]
fn EXTI0() {
    unsafe {
        let exti = &(*pac::EXTI::ptr());
        // Clear interrupt flag
        exti.pr.write(|w| w.pr0().set_bit());
        // Push timestamp
        capture_push(Stm32Time.micros());
    }
}
