//! Sync-loss cancellation integration test.

mod simulation;

use ecu_core::app::EcuApp;
use ecu_core::hal::{OutputPin, TimeSource};
use simulation::time::SimulatedTime;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

struct PulseCapturePin<'a> {
    time: &'a SimulatedTime,
    widths: Rc<RefCell<Vec<u16>>>,
    high_since: Cell<Option<u32>>,
}

impl<'a> PulseCapturePin<'a> {
    fn new(time: &'a SimulatedTime, widths: Rc<RefCell<Vec<u16>>>) -> Self {
        Self {
            time,
            widths,
            high_since: Cell::new(None),
        }
    }
}

impl<'a> OutputPin for PulseCapturePin<'a> {
    fn set_high(&mut self) {
        if self.high_since.get().is_none() {
            self.high_since.set(Some(self.time.micros()));
        }
    }

    fn set_low(&mut self) {
        if let Some(start_us) = self.high_since.replace(None) {
            let duration = self.time.micros().wrapping_sub(start_us);
            self.widths.borrow_mut().push(duration as u16);
        }
    }
}

#[test]
fn sync_loss_cancels_events() {
    let time = SimulatedTime::new();
    let mut app = EcuApp::new(&time);
    let mut engine = simulation::EngineSimulator::new(simulation::EngineConfig::default());
    let mut trigger = simulation::TriggerGenerator::new(simulation::TriggerPattern::SixtyMinusTwo);
    let mut sensors = simulation::SensorSimulator::new();
    let widths = Rc::new(RefCell::new(Vec::new()));
    let mut p0 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p1 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p2 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p3 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p4 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p5 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p6 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut p7 = PulseCapturePin::new(&time, Rc::clone(&widths));
    let mut outputs: [&mut dyn OutputPin; 8] = [
        &mut p0, &mut p1, &mut p2, &mut p3, &mut p4, &mut p5, &mut p6, &mut p7,
    ];

    engine.set_coolant_temp(80);
    engine.set_intake_temp(60);
    engine.start_running();
    engine.set_throttle(0);

    let mut current_time = 0u32;
    let mut synced = false;
    let mut pre_loss_count = 0usize;

    while current_time < 500_000 {
        engine.update(10);
        sensors.update(engine.state());

        {
            let state = app.state_mut();
            state.set_clt_x10(engine.state().coolant_temp_c.saturating_mul(10));
            state.set_iat_x10(engine.state().intake_temp_c.saturating_mul(10));
            state.set_tps_percent(sensors.tps_percent().min(u8::MAX as u16) as u8);
            state.set_map_kpa_x10(sensors.map_kpa());
        }

        if let Some(edge_time) = trigger.next_edge(&engine, current_time) {
            time.set_micros(edge_time);
            app.on_timestamp(edge_time);
            current_time = edge_time;
            synced |= app.state().synced();
        } else {
            time.advance(10);
            current_time = current_time.wrapping_add(10);
        }

        app.drive_outputs(time.micros(), &mut outputs);

        if synced && !widths.borrow().is_empty() {
            pre_loss_count = widths.borrow().len();
            break;
        }
    }

    assert!(synced, "ECU should sync before testing sync loss");
    assert!(
        pre_loss_count > 0,
        "Expected at least one scheduled pulse before sync loss"
    );

    app.on_sync_lost();

    let loss_tick = current_time;
    let target_end = loss_tick + 200_000;
    while current_time < target_end {
        engine.update(10);
        sensors.update(engine.state());
        time.advance(10);
        current_time = current_time.wrapping_add(10);
        app.drive_outputs(time.micros(), &mut outputs);
    }

    assert_eq!(
        widths.borrow().len(),
        pre_loss_count,
        "No injector/coil event should fire after sync loss"
    );
}
