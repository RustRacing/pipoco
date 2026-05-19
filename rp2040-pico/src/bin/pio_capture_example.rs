#![no_std]
#![no_main]

// PIO-based trigger capture example for RP2040 (Raspberry Pi Pico)

use core::cell::RefCell;
use cortex_m_rt::entry;
use panic_halt as _;

use hal::clocks::init_clocks_and_plls;
use hal::pac::interrupt;
use hal::pio::PIOExt;
use hal::watchdog::Watchdog;
use hal::{gpio::FunctionPio0, pac, sio::Sio};
use rp2040_hal as hal;

use ecu_core::hal::TimeSource;
use ecu_core::CaptureBuffer;
use ecu_target_common::trigger_adapter::SplitTriggerAdapter;

// Arduino-style pin declaration (change to remap)
const TRIGGER_PIN: u8 = 4; // GPIO4

static CAPTURE: cortex_m::interrupt::Mutex<RefCell<CaptureBuffer<128>>> =
    cortex_m::interrupt::Mutex::new(RefCell::new(CaptureBuffer::new()));

#[inline]
fn capture_push(ts: u32) {
    cortex_m::interrupt::free(|cs| CAPTURE.borrow(cs).borrow_mut().push(ts));
}
fn capture_pop() -> Option<u32> {
    cortex_m::interrupt::free(|cs| CAPTURE.borrow(cs).borrow_mut().try_pop())
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
    match TRIGGER_PIN {
        0 => {
            let _ = pins.gpio0.into_function::<FunctionPio0>();
        }
        1 => {
            let _ = pins.gpio1.into_function::<FunctionPio0>();
        }
        2 => {
            let _ = pins.gpio2.into_function::<FunctionPio0>();
        }
        3 => {
            let _ = pins.gpio3.into_function::<FunctionPio0>();
        }
        4 => {
            let _ = pins.gpio4.into_function::<FunctionPio0>();
        }
        5 => {
            let _ = pins.gpio5.into_function::<FunctionPio0>();
        }
        _ => {
            let _ = pins.gpio4.into_function::<FunctionPio0>();
        }
    }

    // Split PIO0 and install program
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let program = pio_proc::pio_asm!("wait 0 pin 0", "wait 1 pin 0", "irq set 0", "jmp 0");
    let installed = pio.install(&program.program).unwrap();
    let (sm, _rx, _tx) = rp2040_hal::pio::PIOBuilder::from_program(installed)
        .in_pin_base(TRIGGER_PIN)
        .clock_divisor_fixed_point(1, 0)
        .build(sm0);
    sm.start();

    // Enable PIO0 IRQ 0 for IRQ flag 0 from the state machine
    unsafe {
        // Unmask PIO0 IRQ in NVIC
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::PIO0_IRQ_0);
    }
    // Clear any pending IRQ flags
    pio.clear_irq(1);
    pio.irq0().enable_sm_interrupt(0);

    let mut trigger = SplitTriggerAdapter::new(RpTime);

    loop {
        while let Some(ts) = capture_pop() {
            let _event = trigger.on_trigger_edge(ts);
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
    let pio = unsafe { &*pac::PIO0::ptr() };
    pio.irq.write(|w| unsafe { w.irq().bits(1) });
}
