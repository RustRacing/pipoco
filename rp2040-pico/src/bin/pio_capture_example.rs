#![no_std]
#![no_main]

// PIO-based trigger capture example for RP2040 (Raspberry Pi Pico)

use cortex_m_rt::entry;
use panic_halt as _;

use hal::clocks::init_clocks_and_plls;
use hal::pio::{PIOExt, Rx, ShiftDirection, SM0};
use hal::watchdog::Watchdog;
use hal::{gpio::FunctionPio0, pac, sio::Sio, Clock};
use pio_proc::pio;
use rp2040_hal as hal;

use ecu_core::hal::TimeSource;
use ecu_core::{CaptureBuffer, EcuApp};

// Arduino-style pin declaration (change to remap)
const TRIGGER_PIN: u8 = 4; // GPIO4

static mut CAPTURE: CaptureBuffer<128> = CaptureBuffer::new();
static mut APP: Option<EcuApp<RpTime>> = None;

#[inline]
fn capture_push(ts: u32) {
    cortex_m::interrupt::free(|_| unsafe { CAPTURE.push(ts) });
}
fn capture_pop() -> Option<u32> {
    cortex_m::interrupt::free(|_| unsafe { CAPTURE.try_pop() })
}

// Simple TimeSource reading 64-bit microsecond counter
#[derive(Copy, Clone)]
struct RpTime;
impl TimeSource for RpTime {
    fn micros(&self) -> u32 {
        let t = unsafe { &*pac::TIMER::ptr() };
        // Lower 32 bits of TIMERAWL are microseconds
        t.timerawl.read().bits()
    }
}

// PIO program: wait for rising edge on pin, raise IRQ to CPU
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

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    // Clocks: Pico 12 MHz crystal → system clocks
    let _clocks = init_clocks_and_plls(
        12_000_000,
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

    // Route TRIGGER_PIN to PIO0
    let _trig = match TRIGGER_PIN {
        0 => pins.gpio0.into_mode::<FunctionPio0>(),
        1 => pins.gpio1.into_mode::<FunctionPio0>(),
        2 => pins.gpio2.into_mode::<FunctionPio0>(),
        3 => pins.gpio3.into_mode::<FunctionPio0>(),
        4 => pins.gpio4.into_mode::<FunctionPio0>(),
        5 => pins.gpio5.into_mode::<FunctionPio0>(),
        _ => pins.gpio4.into_mode::<FunctionPio0>(),
    };

    // Split PIO0 and install program
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&edge_irq_prog::PROGRAM).unwrap();
    let (mut sm, _rx, _tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_pin_base(TRIGGER_PIN)
        .clock_divisor(1.0)
        .build(sm0);
    sm.set_pindirs([], []);
    sm.start();

    // Enable PIO0 IRQ 0 for IRQ flag 0 from the state machine
    unsafe {
        // Unmask PIO0 IRQ in NVIC
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
    }
    // Clear any pending IRQ flags
    pio.clr_irq0();
    // Map SM0 IRQ to IRQ0 line; enable IRQ0 source 0
    pio.sm_set_enabled(0, true);
    pio.set_irq0_source_enabled(rp2040_hal::pio::InterruptSource::Sm0, true);

    // Initialize app
    cortex_m::interrupt::free(|_| unsafe {
        APP = Some(EcuApp::new(RpTime));
    });

    loop {
        while let Some(ts) = capture_pop() {
            cortex_m::interrupt::free(|_| unsafe {
                if let Some(ref mut a) = APP {
                    a.on_timestamp(ts);
                }
            });
        }
        cortex_m::asm::wfi();
    }
}

#[allow(non_snake_case)]
#[cortex_m_rt::interrupt]
fn PIO0_IRQ_0() {
    // Timestamp and push into ring buffer on every edge
    capture_push(RpTime.micros());
    // Clear PIO IRQ flag 0
    unsafe {
        let pio = &*pac::PIO0::ptr();
        pio.irq0.write(|w| unsafe { w.bits(1) });
    }
}
