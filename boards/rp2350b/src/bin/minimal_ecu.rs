//! Minimal ECU target for RP2350B using the split runtime/scheduler path.
//!
//! Sets up clocks, basic time source, safe outputs, and a scheduler loop.
//! Trigger input capture and real ISR wiring are left as follow-ups.

#![no_std]
#![no_main]

use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
#[cfg(feature = "capture-gpio")]
use cortex_m::peripheral::NVIC;
use cortex_m_rt::entry;
use hal::pac;
use panic_halt as _;
use rp235x_hal as hal;

use ecu_board_api::Watchdog;
use ecu_domain::{Kpa10, Micros};
use ecu_io::CaptureBuffer;
#[cfg(feature = "capture-gpio")]
use ecu_rp2350b::pinmap;
use ecu_rp2350b::pinmap::PinMap;
use ecu_scheduler::TransitionDrainBuffer;
use ecu_target_common::{
    adapter::BoardAdapter,
    bringup::bringup_fuel_model,
    control_inputs::{split_control_inputs_from, WarmBringupControlSignals},
    noop::{NoopCapture, NoopStore, NoopTransport},
    outputs::{Hal1ScheduledOut, ScheduledActionExecutor, ScheduledOutputs4},
    sensor_sample::{BoardSensorSnapshotSampleSource, FixedLoadSensor},
    split_tick::run_runtime_scheduled_output_tick,
    trigger_adapter::{apply_trigger_timestamp_to_runtime_adapter, SplitTriggerAdapter},
};
use embedded_hal::digital::OutputPin as _;
#[cfg(feature = "capture-gpio")]
use irq::setup_trigger_irq;

// Local HAL glue for the split board adapter.
mod hal_impl {
    use ecu_board_api::EcuClock;
    use ecu_board_api::Watchdog;
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
    impl EcuClock for Rp2350Time {
        fn now_us(&self) -> ecu_domain::Micros {
            ecu_domain::Micros::new(Self::micros())
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
}
use hal_impl::{Rp2350Time, Rp2350Watchdog};

// HAL pin fields are static types, so these bridge macros must stay aligned
// with PinMap::defaults(). The pin map is still the single numeric source of
// truth used for validation, docs, and IRQ setup.
macro_rules! INJ1_GPIO {
    ($pins:ident) => {
        $pins.gpio0
    };
}
macro_rules! INJ2_GPIO {
    ($pins:ident) => {
        $pins.gpio1
    };
}
macro_rules! IGN1_GPIO {
    ($pins:ident) => {
        $pins.gpio2
    };
}
macro_rules! IGN2_GPIO {
    ($pins:ident) => {
        $pins.gpio3
    };
}

const PIN_MAP: PinMap = PinMap::defaults();
const OUTPUT_PIN_ORDER: [u8; 4] = PIN_MAP.output_pins();
const _: () = assert!(OUTPUT_PIN_ORDER[0] == 0);
const _: () = assert!(OUTPUT_PIN_ORDER[1] == 1);
const _: () = assert!(OUTPUT_PIN_ORDER[2] == 2);
const _: () = assert!(OUTPUT_PIN_ORDER[3] == 3);
#[cfg(feature = "capture-gpio")]
const TRIGGER_IRQ_EDGE_HIGH: (usize, u32) = match PIN_MAP.trigger_irq_edge_high() {
    Ok(edge) => edge,
    Err(_) => panic!("invalid RP2350B trigger pin"),
};

fn safe_halt() -> ! {
    loop {
        cortex_m::asm::wfi();
    }
}

fn take_or_halt<T>(value: Option<T>) -> T {
    match value {
        Some(value) => value,
        None => safe_halt(),
    }
}

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
    let mut pac = take_or_halt(pac::Peripherals::take());
    let _core = take_or_halt(cortex_m::Peripherals::take());

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
    .unwrap_or_else(|| safe_halt());

    // Wrap watchdog per canonical board-api trait.
    let mut wd = Rp2350Watchdog::new(hw_wd);
    let _ = wd.feed();

    let _pin_map = PIN_MAP.validate().unwrap_or_else(|_| safe_halt());

    // Optional: wire GPIO IRQ for trigger edges before IO_BANK0 is consumed by HAL pins.
    #[cfg(feature = "capture-gpio")]
    {
        setup_trigger_irq(_pin_map).unwrap_or_else(|_| safe_halt());
        unsafe { NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0) };
    }

    // GPIO init: output selection is checked against PinMap::defaults() above.
    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

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
        BoardSensorSnapshotSampleSource::new(FixedLoadSensor::new(Rp2350Time, Kpa10::new(700))),
        NoopCapture,
        ScheduledActionExecutor::<8>::new(),
        wd,
        NoopTransport,
        NoopStore,
    );
    adapter.configure_fuel_model(bringup_fuel_model());
    let mut control_signals = WarmBringupControlSignals;
    let mut trigger_adapter = SplitTriggerAdapter::new(Rp2350Time);

    // Minimal main loop: poll deterministic bring-up sensors, feed captured
    // trigger edges into the split runtime, and apply due scheduled outputs.
    loop {
        while let Some(ts) = capture_pop() {
            let _ =
                apply_trigger_timestamp_to_runtime_adapter(&mut adapter, &mut trigger_adapter, ts);
        }
        let _ = adapter.poll_sensor();

        let now = Micros::new(Rp2350Time::micros());
        let _ = run_runtime_scheduled_output_tick(
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
    use crate::pac::interrupt;
    use rp235x_hal::pac;

    pub fn setup_trigger_irq(pin_map: PinMap) -> Result<(), pinmap::PinMapError> {
        // Route TRIGGER_PIN_NUM to IO IRQ on rising edge
        // Typical steps (adjust per PAC naming):
        // - Enable rising edge detection for the pin
        // - Unmask PROC0 interrupt for the pin
        // - Clear any pending flags
        let (group, edge_high_mask) = pin_map.trigger_irq_edge_high()?;
        let iobank = unsafe { &*pac::IO_BANK0::ptr() };
        // Enable rising edge
        unsafe {
            iobank.proc0_inte(group).write(|w| w.bits(edge_high_mask));
            // Clear pending
            iobank.intr(group).write(|w| w.bits(edge_high_mask));
        }
        Ok(())
    }

    #[cortex_m_rt::interrupt]
    fn IO_IRQ_BANK0() {
        // Clear pin interrupt flag and push timestamp
        let iobank = unsafe { &*pac::IO_BANK0::ptr() };
        // Clear pending flag by writing '1'
        unsafe {
            iobank
                .intr(TRIGGER_IRQ_EDGE_HIGH.0)
                .write(|w| w.bits(TRIGGER_IRQ_EDGE_HIGH.1));
        }

        let ts = Rp2350Time::micros();
        capture_push(ts);
    }
}
