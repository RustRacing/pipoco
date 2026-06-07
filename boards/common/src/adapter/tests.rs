use super::*;
use crate::outputs::{apply_drained_transitions, RawScheduledOutputPin, ScheduledActionExecutor};
use ecu_board_api::BoardSensorSnapshot;
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, EngineTimeAuthority, Kpa10, Lambda100,
    PhaseSyncState, PulseWidthUs, SyncState,
};
use ecu_runtime::{EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs};
use ecu_scheduler::{SchedulerMode, TransitionDrainBuffer};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockSensor(CaptureSample);

impl CaptureSampleSource for MockSensor {
    type Error = ();

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        Ok(self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockCapture {
    count: usize,
    last: Option<CaptureSample>,
}

impl CaptureSink for MockCapture {
    type Error = ();

    fn capture(&mut self, sample: CaptureSample) -> Result<(), Self::Error> {
        self.count += 1;
        self.last = Some(sample);
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockActions {
    count: usize,
}

impl ActionExecutor for MockActions {
    type Error = ();

    fn execute(&mut self, _action: Action) -> Result<(), Self::Error> {
        self.count += 1;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockWatchdog {
    count: usize,
}

impl Watchdog for MockWatchdog {
    type Error = ();

    fn feed(&mut self) -> Result<(), Self::Error> {
        self.count += 1;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockTransport {
    count: usize,
}

impl TransportPublisher for MockTransport {
    type Error = ();

    fn publish_snapshot(
        &mut self,
        _snapshot: &ecu_runtime::RuntimeSnapshot,
    ) -> Result<(), Self::Error> {
        self.count += 1;
        Ok(())
    }

    fn publish_calibration(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        self.count += 1;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockStore {
    saved: usize,
}

impl PersistedCalibrationStore for MockStore {
    type Error = ();

    fn load(&mut self) -> Result<Option<ecu_calibration::PersistedCalibrationBlob>, Self::Error> {
        Ok(None)
    }

    fn save(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        self.saved += 1;
        Ok(())
    }
}

#[derive(Default)]
struct RecordingPin {
    high_count: u8,
    low_count: u8,
}

impl RawScheduledOutputPin for RecordingPin {
    fn set_scheduled_high(&mut self) {
        self.high_count = self.high_count.saturating_add(1);
    }

    fn set_scheduled_low(&mut self) {
        self.low_count = self.low_count.saturating_add(1);
    }
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
    let mut pulse_widths = [[PulseWidthUs::new(0); 16]; 16];
    pulse_widths[0][0] = PulseWidthUs::new(1000);
    pulse_widths[5][5] = PulseWidthUs::new(2500);
    pulse_widths[15][15] = PulseWidthUs::new(4000);

    ecu_runtime::BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
}

fn control_inputs() -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(1_000),
            clt_c: 50,
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
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(2500)),
    }
}

fn primary_locked_authority() -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::None,
        500,
        0,
    )
}

#[test]
fn board_adapter_turns_events_into_runtime_actions() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_fuel_model(test_fuel_model());

    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(13),
            cam_seen: true,
        })
        .unwrap();
    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap();

    assert!(result.is_some());
    assert_eq!(
        adapter.runtime().snapshot().engine.sync,
        SyncState::Locked { cam_ref: false }
    );
    assert_eq!(
        adapter.runtime().scheduler_state().mode(),
        SchedulerMode::Armed
    );
}

#[test]
fn board_adapter_accepts_logical_sensor_snapshot_capture() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let snapshot = BoardSensorSnapshot {
        rpm: Rpm::new(2_300),
        map_kpa10: Kpa10::new(880),
        tps_x100: 4_200,
        ..BoardSensorSnapshot::default()
    };

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(777),
                angle_x10: Degrees10::new(120),
                snapshot,
            },
        })
        .unwrap();

    let expected = CaptureSample {
        at_us: Micros::new(777),
        rpm: Rpm::new(2_300),
        load_kpa10: Kpa10::new(880),
        angle_x10: Degrees10::new(120),
    };
    assert_eq!(adapter.capture().count, 1);
    assert_eq!(adapter.capture().last, Some(expected));
    assert_eq!(adapter.runtime().snapshot().engine.rpm, Rpm::new(2_300));
    assert_eq!(
        adapter.runtime().snapshot().engine.load_kpa10,
        Kpa10::new(880)
    );
    assert_eq!(
        adapter.runtime().snapshot().engine.angle_x10,
        Degrees10::new(120)
    );
}

#[test]
fn board_adapter_defers_control_decisions_to_runtime_step() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(40),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(8),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(41),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(8),
            authority: EngineTimeAuthority::none(),
            synced: false,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(42),
            cam_seen: false,
        })
        .unwrap();

    assert_eq!(
        adapter.runtime().snapshot().control.fuel_pulse_width.get(),
        0
    );
    assert_eq!(
        adapter.runtime().snapshot().control.ignition_advance.get(),
        0
    );
    assert_eq!(adapter.runtime().snapshot().control.dwell.get(), 0);
}

#[test]
fn board_adapter_routes_runtime_actions_to_io_layers_only() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(60),
        rpm: Rpm::new(1500),
        load_kpa10: Kpa10::new(500),
        angle_x10: Degrees10::new(15),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_fuel_model(test_fuel_model());
    adapter.set_staged_dirty(true);
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(66),
            rpm: Rpm::new(1500),
            angle_x10: Degrees10::new(15),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(67),
            cam_seen: true,
        })
        .unwrap();

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(70),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(result.actions.iter().count() > 0);
    assert!(adapter.actions.count > 0);
    assert!(adapter.transport.count > 0);
    assert!(adapter.store.saved > 0);
    assert!(adapter.watchdog.count > 0);
}

#[test]
fn board_adapter_can_queue_and_apply_split_scheduler_transitions() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(80),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(15),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<8>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_fuel_model(test_fuel_model());
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(86),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(15),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(87),
            cam_seen: true,
        })
        .unwrap();

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(90),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(result
        .actions
        .iter()
        .any(|action| matches!(action, Action::ArmScheduler { .. })));
    assert_eq!(adapter.actions().queue().active_count(), 4);

    let mut drained = TransitionDrainBuffer::<8>::new();
    assert_eq!(
        adapter
            .actions()
            .drain_due(Micros::new(10_000), &mut drained),
        4
    );

    let mut inj0 = RecordingPin::default();
    let mut inj1 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut ign1 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 2] = [&mut inj0, &mut inj1];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 2] = [&mut ign0, &mut ign1];
    assert_eq!(
        apply_drained_transitions(&drained, &mut injectors, &mut ignition),
        Ok(4)
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(inj1.high_count, 1);
    assert_eq!(inj1.low_count, 1);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
    assert_eq!(ign1.high_count, 1);
    assert_eq!(ign1.low_count, 1);
}

#[test]
fn board_adapter_sync_loss_clears_split_scheduler_queue() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(120),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(15),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<8>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_fuel_model(test_fuel_model());
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(126),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(15),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(127),
            cam_seen: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(130),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    assert_eq!(adapter.actions().queue().active_count(), 4);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(140),
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(20),
            authority: EngineTimeAuthority::none(),
            synced: false,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(141),
            cam_seen: false,
        })
        .unwrap();
    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(145),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(result
        .actions
        .iter()
        .any(|action| matches!(action, Action::CancelScheduler(_))));
    assert_eq!(adapter.actions().queue().active_count(), 0);
}
