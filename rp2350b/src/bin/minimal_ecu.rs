//! Minimal ECU target for RP2350B using ecu-core
//!
//! Sets up clocks, basic time source, safe outputs, and a scheduler loop.
//! Trigger input capture and real ISR wiring are left as follow-ups.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use hal::pac;
use panic_halt as _;
use rp235x_hal as hal;

use ecu_core::{CaptureBuffer, EcuApp};
#[path = "../pinmap.rs"]
mod pinmap;
use pinmap::PinMap;

// Local HAL glue for ecu-core traits
mod hal_impl {
    use ecu_core::hal::{
        OutputPin as EcuOutputPin, ResetController, ResetReason, TimeSource, Watchdog,
    };
    use hal::pac;
    use rp235x_hal as hal;

    #[derive(Copy, Clone)]
    pub struct Rp2350Time;
    impl TimeSource for Rp2350Time {
        fn micros(&self) -> u32 {
            // Read 32-bit microsecond counter (low word). Wraps naturally.
            unsafe {
                let timer = &*pac::TIMER0::ptr();
                timer.timerawl().read().bits()
            }
        }
    }

    pub struct Rp2350Pin<P> {
        pub pin: P,
    }
    impl<P> EcuOutputPin for Rp2350Pin<P>
    where
        P: embedded_hal::digital::OutputPin,
    {
        fn set_high(&mut self) {
            let _ = self.pin.set_high();
        }
        fn set_low(&mut self) {
            let _ = self.pin.set_low();
        }
    }

    pub struct Rp2350Watchdog {
        wd: hal::Watchdog,
    }
    impl Rp2350Watchdog {
        pub fn new(wd: hal::Watchdog) -> Self {
            Self { wd }
        }
    }
    impl Watchdog for Rp2350Watchdog {
        fn start(&mut self, _ms: u32) {
            self.wd.feed();
        }
        fn pet(&mut self) {
            self.wd.feed();
        }
    }

    pub struct Rp2350Reset;
    impl Rp2350Reset {
        pub fn new() -> Self {
            Self
        }
    }
    impl ResetController for Rp2350Reset {
        fn reason(&self) -> ResetReason {
            ResetReason::Unknown
        }
        fn clear(&mut self) {}
    }
}
use ecu_core::hal::{OutputPin, TimeSource, Watchdog};
use hal_impl::{Rp2350Pin, Rp2350Reset, Rp2350Time, Rp2350Watchdog};

// Pin mapping (Arduino-style). Change these to remap pins.
// Valid tokens: gpio0..gpio29 (bank0). Adjust for your board wiring.
macro_rules! INJ1_GPIO {
    ($pins:ident) => {
        $pins.gpio0
    };
} // Injector 1 default
macro_rules! INJ2_GPIO {
    ($pins:ident) => {
        $pins.gpio1
    };
} // Injector 2 default
macro_rules! IGN1_GPIO {
    ($pins:ident) => {
        $pins.gpio2
    };
} // Ignition 1 default
macro_rules! IGN2_GPIO {
    ($pins:ident) => {
        $pins.gpio3
    };
} // Ignition 2 default

// App runtime and capture buffer
static mut APP: Option<EcuApp<Rp2350Time>> = None;
static mut CAPTURE: CaptureBuffer<64> = CaptureBuffer::new();

#[inline]
fn capture_push(ts: u32) {
    cortex_m::interrupt::free(|_| unsafe {
        CAPTURE.push(ts);
    });
}
fn capture_pop() -> Option<u32> {
    cortex_m::interrupt::free(|_| unsafe { CAPTURE.try_pop() })
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = cortex_m::Peripherals::take().unwrap();

    // Use HAL watchdog handle for clock init
    let mut hw_wd = hal::Watchdog::new(pac.WATCHDOG);

    // Clocks: 12MHz XOSC → system PLL → 150MHz
    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut hw_wd,
    )
    .ok()
    .unwrap();

    // Wrap watchdog per ecu-core trait
    let mut wd = Rp2350Watchdog::new(hw_wd);
    wd.start(250);
    let mut _rst = Rp2350Reset::new();

    // GPIO init: use GPIO0..GPIO3 for INJ1, INJ2, IGN1, IGN2 (adjust per board)
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Pin mapping (adjust in pinmap.rs if using GPIO IRQ)
    const PIN_MAP: PinMap = PinMap::defaults();

    let inj1_hw = INJ1_GPIO!(pins).into_push_pull_output();
    let inj2_hw = INJ2_GPIO!(pins).into_push_pull_output();
    let ign1_hw = IGN1_GPIO!(pins).into_push_pull_output();
    let ign2_hw = IGN2_GPIO!(pins).into_push_pull_output();

    let mut inj1 = Rp2350Pin { pin: inj1_hw };
    let mut inj2 = Rp2350Pin { pin: inj2_hw };
    let mut ign1 = Rp2350Pin { pin: ign1_hw };
    let mut ign2 = Rp2350Pin { pin: ign2_hw };
    inj1.set_low();
    inj2.set_low();
    ign1.set_low();
    ign2.set_low();

    // Initialize app runtime
    cortex_m::interrupt::free(|_| unsafe {
        APP = Some(EcuApp::new(Rp2350Time));
    });

    // Optional: wire GPIO IRQ for trigger edges (feature-gated)
    #[cfg(feature = "capture-gpio")]
    {
        use rp235x_hal::pac::NVIC;
        setup_trigger_irq(&mut pac, PIN_MAP.trigger);
        unsafe { NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0) };
    }

    // Minimal main loop: poll trigger capture, execute scheduler, pet watchdog
    loop {
        // Feed captured edges and drive outputs
        while let Some(ts) = capture_pop() {
            unsafe {
                if let Some(ref mut a) = APP {
                    a.on_timestamp(ts);
                }
            }
        }

        let now = unsafe { APP.as_ref().map(|a| a.now()).unwrap_or(0) };

        let mut outs: [&mut dyn ecu_core::hal::OutputPin; 4] =
            [&mut inj1, &mut inj2, &mut ign1, &mut ign2];
        cortex_m::interrupt::free(|_| unsafe {
            if let Some(ref mut a) = APP {
                a.drive_outputs(now, &mut outs)
            }
        });

        // Optional: pet watchdog
        wd.pet();

        // Very coarse idle delay (spins). Replace with WFI
        cortex_m::asm::wfi();
    }
}

// Interrupt stub: call this from your GPIO/PIO IRQ handler when a rising edge occurs
#[allow(dead_code)]
fn on_trigger_edge() {
    // Timestamp immediately from hardware counter
    let ts = Rp2350Time.micros();
    capture_push(ts);
}

// Optional: Wire a real GPIO/PIO interrupt for trigger edges (feature-gated)
#[cfg(feature = "capture-gpio")]
mod irq {
    use super::*;
    use rp235x_hal::pac;

    pub fn setup_trigger_irq(p: &mut pac::Peripherals, trigger_pin_num: u8) {
        // Route TRIGGER_PIN_NUM to IO IRQ on rising edge
        // Typical steps (adjust per PAC naming):
        // - Enable rising edge detection for the pin
        // - Unmask PROC0 interrupt for the pin
        // - Clear any pending flags
        let iobank = &p.IO_BANK0;
        let mask = 1u32 << trigger_pin_num;
        // Enable rising edge
        unsafe {
            iobank.inte0.write(|w| w.bits(mask));
            iobank.edge_high.write(|w| w.bits(mask));
            // Clear pending
            iobank.intr.write(|w| w.bits(mask));
        }
    }

    #[cortex_m_rt::interrupt]
    fn IO_IRQ_BANK0() {
        // Clear pin interrupt flag and push timestamp
        let mask = 1u32 << PinMap::defaults().trigger;
        let iobank = unsafe { &*pac::IO_BANK0::ptr() };
        // Clear pending flag by writing '1'
        unsafe {
            iobank.intr.write(|w| w.bits(mask));
        }

        let ts = Rp2350Time.micros();
        capture_push(ts);
    }
}
