use std::cell::Cell;
use std::rc::Rc;

use ecu_domain::{Degrees10, EngineTimeAuthority, Kpa10, Lambda100, Micros, Rpm};
use ecu_io::{
    ActionExecutor, CalibrationStore, CaptureSample, CaptureSink, SensorSource, TransportPublisher,
    Watchdog,
};
use ecu_rp2040_pico::{BoardAdapter as PicoAdapter, BoardEvent as PicoEvent};
use ecu_rp2350b::{BoardAdapter as Rp2350Adapter, BoardEvent as Rp2350Event};
use ecu_runtime::{
    ControlInputs, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs,
};

#[derive(Clone)]
struct Counters {
    sensor: Rc<Cell<usize>>,
    capture: Rc<Cell<usize>>,
    action: Rc<Cell<usize>>,
    watchdog: Rc<Cell<usize>>,
    transport: Rc<Cell<usize>>,
    store: Rc<Cell<usize>>,
}

impl Counters {
    fn new() -> Self {
        Self {
            sensor: Rc::new(Cell::new(0)),
            capture: Rc::new(Cell::new(0)),
            action: Rc::new(Cell::new(0)),
            watchdog: Rc::new(Cell::new(0)),
            transport: Rc::new(Cell::new(0)),
            store: Rc::new(Cell::new(0)),
        }
    }
}

#[derive(Clone)]
struct MockSensor {
    counters: Counters,
    sample: CaptureSample,
}

impl SensorSource for MockSensor {
    type Error = ();

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        self.counters.sensor.set(self.counters.sensor.get() + 1);
        Ok(self.sample)
    }
}

#[derive(Clone)]
struct MockCapture {
    counters: Counters,
}

impl CaptureSink for MockCapture {
    type Error = ();

    fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
        self.counters.capture.set(self.counters.capture.get() + 1);
        Ok(())
    }
}

#[derive(Clone)]
struct MockActions {
    counters: Counters,
}

impl ActionExecutor for MockActions {
    type Error = ();

    fn execute(&mut self, _action: ecu_runtime::Action) -> Result<(), Self::Error> {
        self.counters.action.set(self.counters.action.get() + 1);
        Ok(())
    }
}

#[derive(Clone)]
struct MockWatchdog {
    counters: Counters,
}

impl Watchdog for MockWatchdog {
    type Error = ();

    fn feed(&mut self) -> Result<(), Self::Error> {
        self.counters.watchdog.set(self.counters.watchdog.get() + 1);
        Ok(())
    }
}

#[derive(Clone)]
struct MockTransport {
    counters: Counters,
}

impl TransportPublisher for MockTransport {
    type Error = ();

    fn publish_snapshot(
        &mut self,
        _snapshot: &ecu_runtime::RuntimeSnapshot,
    ) -> Result<(), Self::Error> {
        self.counters
            .transport
            .set(self.counters.transport.get() + 1);
        Ok(())
    }

    fn publish_calibration(
        &mut self,
        _blob: &ecu_calibration::PersistedCalibrationBlob,
    ) -> Result<(), Self::Error> {
        self.counters
            .transport
            .set(self.counters.transport.get() + 1);
        Ok(())
    }
}

#[derive(Clone)]
struct MockStore {
    counters: Counters,
}

impl CalibrationStore for MockStore {
    type Error = ();

    fn load(&mut self) -> Result<Option<ecu_calibration::PersistedCalibrationBlob>, Self::Error> {
        Ok(None)
    }

    fn save(
        &mut self,
        _blob: &ecu_calibration::PersistedCalibrationBlob,
    ) -> Result<(), Self::Error> {
        self.counters.store.set(self.counters.store.get() + 1);
        Ok(())
    }
}

fn control_inputs() -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(1_000),
            clt_c: 60,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            clt_c: 80,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(3000)),
    }
}

fn sample() -> CaptureSample {
    CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(12),
    }
}

#[test]
fn second_board_port_only_needs_adapter_changes() {
    let counters_a = Counters::new();
    let counters_b = Counters::new();

    let mut rp2350 = Rp2350Adapter::new(
        MockSensor {
            counters: counters_a.clone(),
            sample: sample(),
        },
        MockCapture {
            counters: counters_a.clone(),
        },
        MockActions {
            counters: counters_a.clone(),
        },
        MockWatchdog {
            counters: counters_a.clone(),
        },
        MockTransport {
            counters: counters_a.clone(),
        },
        MockStore {
            counters: counters_a.clone(),
        },
    );
    let mut pico = PicoAdapter::new(
        MockSensor {
            counters: counters_b.clone(),
            sample: sample(),
        },
        MockCapture {
            counters: counters_b.clone(),
        },
        MockActions {
            counters: counters_b.clone(),
        },
        MockWatchdog {
            counters: counters_b.clone(),
        },
        MockTransport {
            counters: counters_b.clone(),
        },
        MockStore {
            counters: counters_b.clone(),
        },
    );

    rp2350.configure_fuel_model(test_fuel_model());
    pico.configure_fuel_model(test_fuel_model());

    rp2350
        .apply_event(Rp2350Event::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            synced: true,
            authority: EngineTimeAuthority::none(),
        })
        .unwrap();
    rp2350
        .apply_event(Rp2350Event::CamEdge {
            at_us: Micros::new(13),
            cam_seen: true,
        })
        .unwrap();
    pico.apply_event(PicoEvent::TriggerEdge {
        at_us: Micros::new(12),
        rpm: Rpm::new(3000),
        angle_x10: Degrees10::new(12),
        synced: true,
        authority: EngineTimeAuthority::none(),
    })
    .unwrap();
    pico.apply_event(PicoEvent::CamEdge {
        at_us: Micros::new(13),
        cam_seen: true,
    })
    .unwrap();

    let rp_result = rp2350
        .apply_event(Rp2350Event::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    let pico_result = pico
        .apply_event(PicoEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(rp_result.control, pico_result.control);
    assert_eq!(
        rp2350.runtime().scheduler_state().mode(),
        pico.runtime().scheduler_state().mode()
    );
    assert_eq!(rp2350.runtime().snapshot(), pico.runtime().snapshot());
    assert!(counters_a.action.get() > 0);
    assert!(counters_b.action.get() > 0);
    assert!(counters_a.transport.get() > 0);
    assert!(counters_b.transport.get() > 0);
    assert!(counters_a.watchdog.get() > 0);
    assert!(counters_b.watchdog.get() > 0);
}

fn test_fuel_model() -> ecu_runtime::BaseFuelModel {
    let rpm_bins = [
        Rpm::new(500),
        Rpm::new(1000),
        Rpm::new(1500),
        Rpm::new(2000),
        Rpm::new(2500),
        Rpm::new(3000),
        Rpm::new(3500),
        Rpm::new(4000),
        Rpm::new(4500),
        Rpm::new(5000),
        Rpm::new(5500),
        Rpm::new(6000),
        Rpm::new(6500),
        Rpm::new(7000),
        Rpm::new(7500),
        Rpm::new(8000),
    ];
    let load_bins = [
        Kpa10::new(200),
        Kpa10::new(300),
        Kpa10::new(400),
        Kpa10::new(500),
        Kpa10::new(600),
        Kpa10::new(700),
        Kpa10::new(800),
        Kpa10::new(900),
        Kpa10::new(1000),
        Kpa10::new(1100),
        Kpa10::new(1200),
        Kpa10::new(1300),
        Kpa10::new(1400),
        Kpa10::new(1500),
        Kpa10::new(1600),
        Kpa10::new(1700),
    ];
    let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(0); 16]; 16];
    pulse_widths[5][5] = ecu_domain::PulseWidthUs::new(2500);
    ecu_runtime::BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
}
