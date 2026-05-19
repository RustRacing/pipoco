use std::cell::Cell;

use ecu_core::app::EcuApp;
use ecu_core::hal::OutputPin;
use ecu_core::hal::TimeSource;
use ecu_core::scheduler::{Channel, Scheduler};
use ecu_core::ts::pages::PAGE_AE;
use ecu_core::ts::PageStore;
use ecu_core::{EcuState, TriggerDecoder};

mod simulation;

use simulation::time::SimulatedTime;

struct DummyPin {
    high: bool,
}

impl DummyPin {
    fn new() -> Self {
        Self { high: false }
    }
}

impl OutputPin for DummyPin {
    fn set_high(&mut self) {
        self.high = true;
    }

    fn set_low(&mut self) {
        self.high = false;
    }
}

struct ReentrantTime {
    now_us: Cell<u32>,
    decoder: Cell<*mut TriggerDecoder<ReentrantTime>>,
    reentered: Cell<bool>,
}

impl ReentrantTime {
    fn new() -> Self {
        Self {
            now_us: Cell::new(0),
            decoder: Cell::new(core::ptr::null_mut()),
            reentered: Cell::new(false),
        }
    }
}

impl TimeSource for ReentrantTime {
    fn micros(&self) -> u32 {
        if !self.reentered.get() {
            self.reentered.set(true);
            unsafe {
                let decoder = self.decoder.get();
                if !decoder.is_null() {
                    (*decoder).tooth_edge();
                }
            }
        }
        self.now_us.get()
    }
}

#[test]
#[should_panic(expected = "re-entrancy")]
fn concurrent_access_smoke() {
    let time = SimulatedTime::new();
    let mut app = EcuApp::new(&time);
    let mut scheduler = Scheduler::new();
    let mut pin = DummyPin::new();
    let mut outputs: [&mut dyn OutputPin; 1] = [&mut pin];

    scheduler.schedule_ticks(0, Channel::INJ1, true);
    scheduler.schedule_ticks(100, Channel::INJ1, false);

    let state: &mut EcuState = app.state_mut();
    let mut pages = state.page_store();

    let mut ae_payload = [0u8; 16];
    ae_payload[0..2].copy_from_slice(&(-50i16).to_le_bytes());
    ae_payload[2..4].copy_from_slice(&(25i16).to_le_bytes());
    ae_payload[4] = 10;
    ae_payload[6..10].copy_from_slice(&250u32.to_le_bytes());
    ae_payload[10..14].copy_from_slice(&500u32.to_le_bytes());

    let time = ReentrantTime::new();
    let mut decoder = TriggerDecoder::new(time);
    let decoder_ptr: *mut TriggerDecoder<ReentrantTime> = &mut decoder;
    unsafe {
        (*decoder_ptr).time_source().decoder.set(decoder_ptr);
        (*decoder_ptr).time_source().now_us.set(50);
    }

    for step in 0..32u32 {
        let now = step * 50;
        unsafe {
            (*decoder_ptr).time_source().now_us.set(now);
        }
        scheduler.check_and_execute_ticks(step, &mut outputs);
        pages.write_page(PAGE_AE, &ae_payload).expect("write page");
        decoder.tooth_edge();
    }
}
