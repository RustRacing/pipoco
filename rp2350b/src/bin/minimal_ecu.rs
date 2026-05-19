//! Minimal ECU target for RP2350B using the split runtime/scheduler path.
//!
//! Sets up clocks, basic time source, safe outputs, and a scheduler loop.
//! Trigger input capture and real ISR wiring are left as follow-ups.

#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use cortex_m_rt::entry;
use hal::pac;
use panic_halt as _;
use rp235x_hal as hal;

use ecu_core::CaptureBuffer;
use ecu_domain::{Kpa10, Micros};
use ecu_io::Watchdog;
use ecu_scheduler::TransitionDrainBuffer;
use ecu_target_common::{
    adapter::{BoardAdapter, BoardEvent},
    bringup::bringup_fuel_model,
    control_inputs::{split_control_inputs_from, WarmBringupControlSignals},
    noop::{NoopCapture, NoopStore, NoopTransport},
    outputs::{Hal1ScheduledOut, ScheduledActionExecutor, ScheduledOutputs4},
    sensor_sample::FixedLoadSensor,
    split_tick::run_split_scheduled_tick,
    trigger_adapter::{apply_trigger_timestamp, SplitTriggerAdapter},
};
use embedded_hal::digital::OutputPin as _;
#[path = "../pinmap.rs"]
mod pinmap;
use pinmap::PinMap;

// Local HAL glue for the split board adapter.
mod hal_impl {
    use ecu_core::hal::{ResetController, ResetReason, TimeSource};
    use ecu_io::Watchdog;
    use hal::pac;
    use rp235x_hal as hal;

    #[derive(Copy, Clone)]
    pub struct Rp2350Time;
    impl Rp2350Time {
        pub fn micros() -> u32 {
            // Read 32-bit microsecond counter (low word). Wraps naturally.
            unsafe {
                let timer = &*pac::TIMER0::ptr();
                timer.timerawl().read().bits()
            }
        }
    }
    impl TimeSource for Rp2350Time {
        fn micros(&self) -> u32 {
            Self::micros()
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
        type Error = core::convert::Infallible;

        fn feed(&mut self) -> Result<(), Self::Error> {
            self.wd.feed();
            Ok(())
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
use hal_impl::{Rp2350Reset, Rp2350Time, Rp2350Watchdog};

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

// Capture buffer wrapped in no_std-safe critical section access.
static CAPTURE: Mutex<RefCell<CaptureBuffer<64>>> = Mutex::new(RefCell::new(CaptureBuffer::new()));

#[inline]
fn capture_push(ts: u32) {
    cortex_m::interrupt::free(|cs| {
        CAPTURE.borrow(cs).borrow_mut().push(ts);
    });
}
fn capture_pop() -> Option<u32> {
    cortex_m::interrupt::free(|cs| CAPTURE.borrow(cs).borrow_mut().try_pop())
}

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = cortex_m::Peripherals::take().unwrap();

    // Use HAL watchdog handle for clock init
    let mut hw_wd = hal::Watchdog::new(pac.WATCHDOG);

    // Clocks: 12MHz XOSC → system PLL → 150MHz
    let _clocks = hal::clocks::init_clocks_and_plls(
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

    // Wrap watchdog per ecu-io trait.
    let mut wd = Rp2350Watchdog::new(hw_wd);
    let _ = wd.feed();
    let _rst = Rp2350Reset::new();

    // GPIO init: use GPIO0..GPIO3 for INJ1, INJ2, IGN1, IGN2 (adjust per board)
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Pin mapping (adjust in pinmap.rs if using GPIO IRQ)
    let _pin_map = PinMap::defaults();

    let mut inj1 = INJ1_GPIO!(pins).into_push_pull_output();
    let mut inj2 = INJ2_GPIO!(pins).into_push_pull_output();
    let mut ign1 = IGN1_GPIO!(pins).into_push_pull_output();
    let mut ign2 = IGN2_GPIO!(pins).into_push_pull_output();
    let _ = inj1.set_low();
    let _ = inj2.set_low();
    let _ = ign1.set_low();
    let _ = ign2.set_low();

    let mut outputs = ScheduledOutputs4::new(
        Hal1ScheduledOut::new(inj1),
        Hal1ScheduledOut::new(inj2),
        Hal1ScheduledOut::new(ign1),
        Hal1ScheduledOut::new(ign2),
    );
    let mut drain = TransitionDrainBuffer::<8>::new();
    let mut adapter = BoardAdapter::new(
        FixedLoadSensor::new(Rp2350Time, Kpa10::new(700)),
        NoopCapture,
        ScheduledActionExecutor::<8>::new(),
        wd,
        NoopTransport,
        NoopStore,
    );
    adapter.configure_fuel_model(bringup_fuel_model());
    let mut control_signals = WarmBringupControlSignals;
    let mut trigger_adapter = SplitTriggerAdapter::new(Rp2350Time);

    // Optional: wire GPIO IRQ for trigger edges (feature-gated)
    #[cfg(feature = "capture-gpio")]
    {
        use rp235x_hal::pac::NVIC;
        setup_trigger_irq(&mut pac, _pin_map.trigger);
        unsafe { NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0) };
    }

    // Minimal main loop: poll deterministic bring-up sensors, feed captured
    // trigger edges into the split runtime, and apply due scheduled outputs.
    loop {
        while let Some(ts) = capture_pop() {
            let at_us = Micros::new(ts);
            let _ = apply_trigger_timestamp(&mut adapter, &mut trigger_adapter, ts);
            let _ = adapter.apply_event(BoardEvent::CamEdge {
                at_us,
                cam_seen: true,
            });
        }
        let _ = adapter.poll_sensor();

        let now = Micros::new(Rp2350Time::micros());
        let _ = run_split_scheduled_tick(
            &mut adapter,
            now,
            split_control_inputs_from(&mut control_signals, now, trigger_adapter.rpm())
                .unwrap_or_else(|never| match never {}),
            &mut outputs,
            &mut drain,
        );

        // Very coarse idle delay (spins). Replace with WFI
        cortex_m::asm::wfi();
    }
}

// Interrupt stub: call this from your GPIO/PIO IRQ handler when a rising edge occurs
#[allow(dead_code)]
fn on_trigger_edge() {
    // Timestamp immediately from hardware counter
    let ts = Rp2350Time::micros();
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

        let ts = Rp2350Time::micros();
        capture_push(ts);
    }
}
