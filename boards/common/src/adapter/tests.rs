use super::*;
use crate::live_inputs::{SplitLiveInputs, SplitLiveTriggerEvent, SplitSyncState};
use crate::outputs::{apply_drained_transitions, RawScheduledOutputPin, ScheduledActionExecutor};
use ecu_board_api::frontier::{TimingIslandPermitMask, TimingIslandStopReason};
use ecu_board_api::{
    BoardSensorSnapshot, BoardSensorSnapshotCapture, CommonActionTelemetry, CommonCamEdgeTelemetry,
    CommonControlReasonTelemetry, CommonControlTelemetry, CommonDecisionTelemetry,
    CommonDiagnosticsTelemetry, CommonEngineTelemetry, CommonEnrichmentTelemetry,
    CommonFaultTransitionTelemetry, CommonFrontierTelemetry, CommonFuelObservationTelemetry,
    CommonFuelStrategyMode, CommonIgnitionLimitReason, CommonLambdaMode,
    CommonPendingInputTelemetry, CommonSchedulerMode, CommonSchedulerOwnershipTelemetry,
    CommonSchedulerReservationTelemetry, CommonSchedulerStateSummaryTelemetry,
    CommonSchedulerWindowTelemetry, CommonShiftArmingTelemetry, CommonSyncTelemetryState,
    CommonTorqueLimitReason, CommonTorqueTelemetry, CommonTriggerEdgeTelemetry,
    CommonValidatedInputTelemetry, EngineTimeAuthorityTelemetry,
};
use ecu_calibration::{
    CalibrationPackageIdentity, CalibrationRevision, CalibrationSchemaVersion, FuelRuntimeTune,
};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ControlMode, CrankSyncState, Degrees10, DwellUs,
    EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10, Lambda100, PhaseSyncState, PulseWidthUs,
    SyncState,
};
use ecu_runtime::{
    runtime_semantic_calibration_from_fuel_tune, Action, ActionBatch, ActionExecutor,
    AllowedTorque, ControlPlan, EnrichmentResult, FuelIntent, FuelObservations,
    IgnitionLimitReason, IgnitionPlan, LambdaMode, LambdaTrimResult, RuntimeFuelStrategy,
    RuntimeSemanticCalibration, RuntimeSemanticState, StepResult, TorqueLimitReason,
    TorqueObservations, ValidatedInputs, RUNTIME_ACTION_CAP,
};
use ecu_runtime::{EnrichmentInputs, FaultState, IgnitionInputs, LambdaTrimInputs, TorqueInputs};
use ecu_scheduler::{
    ChannelId, ExclusiveChannel, IgnitionPlan as SchedulerIgnitionPlan,
    InjectionPlan as SchedulerInjectionPlan, OutputGroup, ScheduledTimingMetrics, SchedulerMode,
    TimedIgnitionPlan, TimedInjectionPlan, TransitionDrainBuffer,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockSensor(CaptureSample);

impl CaptureSampleSource for MockSensor {
    type Error = ();

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        Ok(self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct FailingSensor;

impl CaptureSampleSource for FailingSensor {
    type Error = ();

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        Err(())
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
struct FailingCapture;

impl CaptureSink for FailingCapture {
    type Error = ();

    fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
        Err(())
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

fn semantic_calibration() -> RuntimeSemanticCalibration {
    let tune = FuelRuntimeTune::new([[100; 16]; 16], [[147; 16]; 16], 1000, 0, 0);
    runtime_semantic_calibration_from_fuel_tune(&tune)
}

fn semantic_fuel_cut_calibration(
    launch_rpm_limit: u16,
    flat_shift_rpm_min: u16,
) -> RuntimeSemanticCalibration {
    let tune = ecu_calibration::FuelRuntimeTune::new([[100; 16]; 16], [[147; 16]; 16], 1000, 0, 0);
    let mut calibration = ecu_runtime::runtime_semantic_calibration_from_fuel_tune(&tune);
    calibration.launch_rpm_limit = launch_rpm_limit;
    calibration.launch_cut_cycles = 0;
    calibration.flat_shift_rpm_min = flat_shift_rpm_min;
    calibration.flat_shift_cut_cycles = 0;
    calibration
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
        knock_intensity_x100: 0,
    }
}

fn synthetic_step_result(actions: ActionBatch<RUNTIME_ACTION_CAP>) -> StepResult {
    StepResult {
        validated: ValidatedInputs {
            rpm: Rpm::new(0),
            load_kpa10: Kpa10::new(0),
            angle_x10: Degrees10::new(0),
            clamped: false,
        },
        operating_mode: ControlMode::default(),
        control: ControlPlan {
            base_fuel: PulseWidthUs::new(0),
            enriched_fuel: PulseWidthUs::new(0),
            enrichment: EnrichmentResult::new(0, 0, 0, 0),
            lambda: LambdaTrimResult::new(
                LambdaMode::OpenLoop,
                false,
                Lambda100::new(100),
                Lambda100::new(100),
                0,
            ),
            torque: AllowedTorque::new(0, 0, 0, 0, TorqueLimitReason::None),
            ignition: IgnitionPlan::new(
                Degrees10::new(0),
                DwellUs::new(0),
                IgnitionLimitReason::None,
            ),
            fuel_cut: false,
            spark_cut: false,
            fuel_intent: FuelIntent {
                pulse_width_us: PulseWidthUs::new(0),
                target_angle_deg10: Degrees10::new(0),
                fuel_cut: false,
                spark_cut: false,
                observations: FuelObservations {
                    ve_pct_x100: None,
                    target_afr_x100: None,
                    pw_base_us: 0,
                    pw_corr_us: 0,
                    strategy_is_direct_pw: true,
                },
            },
        },
        actions,
        torque_observations: TorqueObservations {
            request_x1000: 0,
            allowed_x1000: 0,
            actuated_x1000: 0,
        },
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

fn primary_searching_authority() -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        CrankSyncState::PrimarySearching,
        PhaseSyncState::Unknown,
        AbsoluteTimeAuthority::None,
        0,
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
    let actions = ScheduledActionExecutor::<4>::new();
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
fn board_adapter_fault_surface_defaults_to_runtime_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.fault_state(), FaultState::default());
    assert_eq!(adapter.fault_state(), adapter.runtime().snapshot().faults);
}

#[test]
fn board_adapter_scheduler_reservations_default_to_zero_and_track_executor_state() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.scheduler_reservation_telemetry(),
        CommonSchedulerReservationTelemetry::default()
    );

    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(50, 100),
            ignition: timed_ignition(60, 120),
        })
        .unwrap();

    assert_eq!(
        adapter.scheduler_reservation_telemetry(),
        CommonSchedulerReservationTelemetry {
            injector_channels: 1,
            ignition_channels: 1,
            idle_channels: 0,
            fan_channels: 0,
        }
    );
}

#[test]
fn board_adapter_scheduler_reservations_clear_on_sync_loss() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(50, 100),
            ignition: timed_ignition(60, 120),
        })
        .unwrap();
    assert_ne!(
        adapter.scheduler_reservation_telemetry(),
        CommonSchedulerReservationTelemetry::default()
    );

    adapter.actions().on_sync_loss();

    assert_eq!(
        adapter.scheduler_reservation_telemetry(),
        CommonSchedulerReservationTelemetry::default()
    );
}

#[test]
fn board_adapter_observability_snapshot_includes_scheduler_reservations() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(50, 100),
            ignition: timed_ignition(60, 120),
        })
        .unwrap();

    let snapshot = adapter.observability_snapshot();

    assert_eq!(
        snapshot.scheduler_reservations,
        adapter.scheduler_reservation_telemetry()
    );
}

#[test]
fn board_adapter_logical_sensor_snapshot_defaults_to_none() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.logical_sensor_capture(), None);
    assert_eq!(adapter.logical_sensor_snapshot(), None);
}

#[test]
fn board_adapter_torque_telemetry_defaults_to_zero_before_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.torque_telemetry(), CommonTorqueTelemetry::default());
}

#[test]
fn board_adapter_trigger_edge_telemetry_defaults_to_empty_before_any_event() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.poll_sensor().unwrap();

    assert_eq!(
        adapter.trigger_edge_telemetry(),
        CommonTriggerEdgeTelemetry::default()
    );
}

#[test]
fn board_adapter_cam_edge_telemetry_defaults_to_empty_before_any_event() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.cam_edge_telemetry(),
        CommonCamEdgeTelemetry::default()
    );
}

#[test]
fn board_adapter_validated_input_telemetry_defaults_to_zero_before_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.validated_input_telemetry(),
        CommonValidatedInputTelemetry::default()
    );
}

#[test]
fn board_adapter_fuel_observation_telemetry_defaults_to_zero_before_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.fuel_observation_telemetry(),
        CommonFuelObservationTelemetry::default()
    );
}

#[test]
fn board_adapter_enrichment_telemetry_defaults_to_zero_before_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.enrichment_telemetry(),
        CommonEnrichmentTelemetry::default()
    );
}

#[test]
fn board_adapter_non_tick_events_do_not_fabricate_fuel_observation_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(11),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(12),
            cam_seen: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(13),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();

    assert_eq!(
        adapter.fuel_observation_telemetry(),
        CommonFuelObservationTelemetry::default()
    );
    assert_eq!(
        adapter.validated_input_telemetry(),
        CommonValidatedInputTelemetry::default()
    );
}

#[test]
fn board_adapter_trigger_edge_telemetry_tracks_trigger_edge_and_stays_put_on_other_events() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let first = CommonTriggerEdgeTelemetry {
        seen: true,
        at_us: Micros::new(11),
        rpm: Rpm::new(1200),
        angle_x10: Degrees10::new(12),
        authority: primary_locked_authority(),
        synced: true,
    };
    let second = CommonTriggerEdgeTelemetry {
        seen: true,
        at_us: Micros::new(21),
        rpm: Rpm::new(1300),
        angle_x10: Degrees10::new(14),
        authority: primary_searching_authority(),
        synced: false,
    };

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: first.at_us,
            rpm: first.rpm,
            angle_x10: first.angle_x10,
            authority: first.authority,
            synced: first.synced,
        })
        .unwrap();
    assert_eq!(adapter.trigger_edge_telemetry(), first);

    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(12),
            cam_seen: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(13),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(14),
            control: control_inputs(),
        })
        .unwrap();
    assert_eq!(adapter.trigger_edge_telemetry(), first);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: second.at_us,
            rpm: second.rpm,
            angle_x10: second.angle_x10,
            authority: second.authority,
            synced: second.synced,
        })
        .unwrap();
    assert_eq!(adapter.trigger_edge_telemetry(), second);
}

#[test]
fn board_adapter_non_tick_events_do_not_fabricate_enrichment_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(11),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(12),
            cam_seen: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(13),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();

    assert_eq!(
        adapter.enrichment_telemetry(),
        CommonEnrichmentTelemetry::default()
    );
}

#[test]
fn board_adapter_trigger_sensor_and_tick_do_not_fabricate_cam_edge_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(11),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(13),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(14),
            control: control_inputs(),
        })
        .unwrap();

    assert_eq!(
        adapter.cam_edge_telemetry(),
        CommonCamEdgeTelemetry::default()
    );
}

#[test]
fn board_adapter_cam_edge_telemetry_tracks_cam_edge_and_stays_put_on_other_events() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let expected = CommonCamEdgeTelemetry {
        seen: true,
        at_us: Micros::new(12),
        cam_seen: true,
    };

    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: expected.at_us,
            cam_seen: expected.cam_seen,
        })
        .unwrap();
    assert_eq!(adapter.cam_edge_telemetry(), expected);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(13),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(14),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(15),
            control: control_inputs(),
        })
        .unwrap();

    assert_eq!(adapter.cam_edge_telemetry(), expected);
}

#[test]
fn board_adapter_fuel_observation_telemetry_tracks_successful_tick_result() {
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
    adapter.set_staged_dirty(true);

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.fuel_observation_telemetry(),
        CommonFuelObservationTelemetry {
            base_fuel_pulse_width: result.control.base_fuel,
            enriched_fuel_pulse_width: result.control.enriched_fuel,
        }
    );
    assert_eq!(
        adapter.validated_input_telemetry(),
        CommonValidatedInputTelemetry {
            rpm: result.validated.rpm,
            load_kpa10: result.validated.load_kpa10,
            angle_x10: result.validated.angle_x10,
            clamped: result.validated.clamped,
        }
    );
}

#[test]
fn board_adapter_observability_snapshot_includes_cam_edge() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let expected = CommonCamEdgeTelemetry {
        seen: true,
        at_us: Micros::new(21),
        cam_seen: false,
    };

    adapter.set_shift_arming(true, false);
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: expected.at_us,
            cam_seen: expected.cam_seen,
        })
        .unwrap();

    let snapshot = adapter.observability_snapshot();
    assert_eq!(snapshot.cam_edge, expected);
    assert_eq!(
        snapshot.shift_arming,
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );
}

#[test]
fn board_adapter_shift_arming_telemetry_defaults_to_false() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: false,
            flat_shift_armed: false,
        }
    );
}

#[test]
fn board_adapter_shift_arming_telemetry_tracks_set_shift_arming_updates() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.set_shift_arming(true, false);
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );

    adapter.set_shift_arming(false, true);
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: false,
            flat_shift_armed: true,
        }
    );
}

#[test]
fn board_adapter_set_shift_arming_and_record_stores_shift_arming_kind_and_current_sample() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.set_shift_arming_and_record(true, false, &mut trace),
        Ok(())
    );
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false
        }
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::ShiftArming,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_set_shift_arming_and_record_returns_overflow_after_updating_shift_arming() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    trace.push(existing).unwrap();

    let err = adapter
        .set_shift_arming_and_record(true, true, &mut trace)
        .unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: true,
        }
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(existing));
}

#[test]
fn board_adapter_set_shift_arming_and_push_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.set_shift_arming_and_push_pair(true, false, &mut sample_trace, &mut record_trace),
        Ok(())
    );
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(adapter.observability_sample()));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::ShiftArming,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_set_shift_arming_and_push_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    sample_trace.push(sample_before).unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .set_shift_arming_and_push_pair(true, true, &mut sample_trace, &mut record_trace)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: true,
        }
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_set_shift_arming_and_push_pair_returns_sample_overflow_after_record_is_stored() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample::default())
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .set_shift_arming_and_push_pair(true, false, &mut sample_trace, &mut record_trace)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::ShiftArming,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_set_shift_arming_and_push_to_trace_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    assert_eq!(
        adapter.set_shift_arming_and_push_to_trace_pair(true, false, &mut traces),
        Ok(())
    );

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(expected_sample));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::ShiftArming,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_set_shift_arming_and_push_to_trace_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces
        .record_mut()
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .set_shift_arming_and_push_to_trace_pair(true, true, &mut traces)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: true,
        }
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_set_shift_arming_and_push_to_trace_pair_returns_sample_overflow_after_record_is_stored(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 2> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();

    let err = adapter
        .set_shift_arming_and_push_to_trace_pair(true, false, &mut traces)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::ShiftArming,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_record_stores_staged_dirty_kind_and_current_sample() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.set_staged_dirty_and_record(true, &mut trace),
        Ok(())
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_record_returns_overflow_after_updating_staged_dirty() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    trace.push(existing).unwrap();

    let err = adapter
        .set_staged_dirty_and_record(true, &mut trace)
        .unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(existing));
}

#[test]
fn board_adapter_set_staged_dirty_and_push_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.set_staged_dirty_and_push_pair(true, &mut sample_trace, &mut record_trace),
        Ok(())
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        record_trace.get(0).map(|record| record.sample)
    );
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_push_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    sample_trace.push(sample_before).unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .set_staged_dirty_and_push_pair(true, &mut sample_trace, &mut record_trace)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_push_pair_returns_sample_overflow_after_record_is_stored() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample::default())
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .set_staged_dirty_and_push_pair(true, &mut sample_trace, &mut record_trace)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_push_to_trace_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    assert_eq!(
        adapter.set_staged_dirty_and_push_to_trace_pair(true, &mut traces),
        Ok(())
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.sample().get(0),
        traces.record().get(0).map(|record| record.sample)
    );
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_push_to_trace_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces
        .record_mut()
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .set_staged_dirty_and_push_to_trace_pair(true, &mut traces)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_set_staged_dirty_and_push_to_trace_pair_returns_sample_overflow_after_record_is_stored(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 2> =
        FixedCommonObservabilityTracePair::new();
    traces
        .sample_mut()
        .push(CommonObservabilitySample::default())
        .unwrap();

    let err = adapter
        .set_staged_dirty_and_push_to_trace_pair(true, &mut traces)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert!(adapter.calibration_package_identity().staged_dirty);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(
        traces.sample().get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_configure_fuel_model_and_record_stores_fuel_model_config_kind_and_current_sample()
{
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.configure_fuel_model_and_record(test_fuel_model(), &mut trace),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::FuelModelConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_fuel_model_and_record_returns_overflow_after_updating_fuel_model() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    trace.push(existing).unwrap();

    let err = adapter
        .configure_fuel_model_and_record(test_fuel_model(), &mut trace)
        .unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(existing));
}

#[test]
fn board_adapter_configure_fuel_model_and_push_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.configure_fuel_model_and_push_pair(
            test_fuel_model(),
            &mut sample_trace,
            &mut record_trace,
        ),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(
        adapter.observability_snapshot().fuel_strategy_mode,
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(adapter.observability_sample()));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::FuelModelConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_fuel_model_and_push_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    record_trace.push(existing).unwrap();

    let err = adapter
        .configure_fuel_model_and_push_pair(test_fuel_model(), &mut sample_trace, &mut record_trace)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(sample_trace.len(), 0);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(record_trace.get(0), Some(existing));
}

#[test]
fn board_adapter_configure_fuel_model_and_push_pair_returns_sample_overflow_after_record_is_stored()
{
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        })
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .configure_fuel_model_and_push_pair(test_fuel_model(), &mut sample_trace, &mut record_trace)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        })
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::FuelModelConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_fuel_model_and_push_to_trace_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    assert_eq!(
        adapter.configure_fuel_model_and_push_to_trace_pair(test_fuel_model(), &mut traces),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(
        adapter.observability_snapshot().fuel_strategy_mode,
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(adapter.observability_sample()));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::FuelModelConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_fuel_model_and_push_to_trace_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    traces.record_mut().push(existing).unwrap();

    let err = adapter
        .configure_fuel_model_and_push_to_trace_pair(test_fuel_model(), &mut traces)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(traces.sample().len(), 0);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.record().get(0), Some(existing));
}

#[test]
fn board_adapter_configure_fuel_model_and_push_to_trace_pair_returns_sample_overflow_after_record_is_stored(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    traces
        .sample_mut()
        .push(CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        })
        .unwrap();

    let err = adapter
        .configure_fuel_model_and_push_to_trace_pair(test_fuel_model(), &mut traces)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(
        traces.sample().get(0),
        Some(CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        })
    );
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::FuelModelConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_speed_density_semantic_and_record_stores_semantic_config_kind_and_current_sample(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.configure_speed_density_semantic_and_record(
            semantic_fuel_cut_calibration(2_500, 9_000),
            RuntimeSemanticState::default(),
            &mut trace,
        ),
        Ok(())
    );
    assert_eq!(
        adapter.calibration_package_identity(),
        adapter.observability_snapshot().calibration
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SpeedDensitySemanticConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_speed_density_semantic_and_record_returns_overflow_after_updating_semantic_config(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    trace.push(existing).unwrap();

    let err = adapter
        .configure_speed_density_semantic_and_record(
            semantic_fuel_cut_calibration(2_500, 9_000),
            RuntimeSemanticState::default(),
            &mut trace,
        )
        .unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(
        adapter.calibration_package_identity(),
        adapter.observability_snapshot().calibration
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(existing));
}

#[test]
fn board_adapter_configure_speed_density_semantic_and_push_pair_stores_matching_sample_and_record()
{
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.configure_speed_density_semantic_and_push_pair(
            semantic_fuel_cut_calibration(2_500, 9_000),
            RuntimeSemanticState::default(),
            &mut sample_trace,
            &mut record_trace,
        ),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(
        adapter.observability_snapshot().fuel_strategy_mode,
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(adapter.observability_sample()));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SpeedDensitySemanticConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_speed_density_semantic_and_push_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    record_trace.push(existing).unwrap();

    let err = adapter
        .configure_speed_density_semantic_and_push_pair(
            semantic_fuel_cut_calibration(2_500, 9_000),
            RuntimeSemanticState::default(),
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(sample_trace.len(), 0);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(record_trace.get(0), Some(existing));
}

#[test]
fn board_adapter_configure_speed_density_semantic_and_push_pair_returns_sample_overflow_after_record_is_stored(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        })
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .configure_speed_density_semantic_and_push_pair(
            semantic_fuel_cut_calibration(2_500, 9_000),
            RuntimeSemanticState::default(),
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        })
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SpeedDensitySemanticConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_record_stores_strategy_kind_and_current_sample(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    assert_eq!(
        adapter.configure_runtime_fuel_strategy_and_record(
            RuntimeFuelStrategy::SpeedDensityVe { calibration, state },
            &mut trace,
        ),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_record_returns_overflow_after_updating_strategy(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let existing = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    trace.push(existing).unwrap();

    let err = adapter
        .configure_runtime_fuel_strategy_and_record(
            RuntimeFuelStrategy::AlphaN { calibration, state },
            &mut trace,
        )
        .unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::AlphaN);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(existing));
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_push_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    assert_eq!(
        adapter.configure_runtime_fuel_strategy_and_push_pair(
            RuntimeFuelStrategy::SpeedDensityVe { calibration, state },
            &mut sample_trace,
            &mut record_trace,
        ),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        record_trace.get(0).map(|record| record.sample)
    );
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_push_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    sample_trace.push(sample_before).unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    record_trace.push(record_before).unwrap();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    let err = adapter
        .configure_runtime_fuel_strategy_and_push_pair(
            RuntimeFuelStrategy::AlphaN { calibration, state },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::AlphaN);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(record_trace.get(0), Some(record_before));
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_push_pair_returns_sample_overflow_after_record_is_stored(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample::default())
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    let err = adapter
        .configure_runtime_fuel_strategy_and_push_pair(
            RuntimeFuelStrategy::Maf { calibration, state },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::Maf);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_push_to_trace_pair_stores_matching_sample_and_record(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    assert_eq!(
        adapter.configure_runtime_fuel_strategy_and_push_to_trace_pair(
            RuntimeFuelStrategy::SpeedDensityVe { calibration, state },
            &mut traces,
        ),
        Ok(())
    );
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.sample().get(0),
        traces.record().get(0).map(|record| record.sample)
    );
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_push_to_trace_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces.record_mut().push(record_before).unwrap();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    let err = adapter
        .configure_runtime_fuel_strategy_and_push_to_trace_pair(
            RuntimeFuelStrategy::AlphaN { calibration, state },
            &mut traces,
        )
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::AlphaN);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.record().get(0), Some(record_before));
}

#[test]
fn board_adapter_configure_runtime_fuel_strategy_and_push_to_trace_pair_returns_sample_overflow_after_record_is_stored(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 2> =
        FixedCommonObservabilityTracePair::new();
    traces
        .sample_mut()
        .push(CommonObservabilitySample::default())
        .unwrap();
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    let err = adapter
        .configure_runtime_fuel_strategy_and_push_to_trace_pair(
            RuntimeFuelStrategy::Maf { calibration, state },
            &mut traces,
        )
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::Maf);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(
        traces.sample().get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::RuntimeFuelStrategyConfig,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_shift_arming_telemetry_stays_stable_across_non_arming_events() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.set_shift_arming(true, false);

    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(11),
            cam_seen: true,
        })
        .unwrap();
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );

    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(13),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(14),
            control: control_inputs(),
        })
        .unwrap();
    assert_eq!(
        adapter.shift_arming_telemetry(),
        CommonShiftArmingTelemetry {
            launch_armed: true,
            flat_shift_armed: false,
        }
    );
}

#[test]
fn board_adapter_enrichment_telemetry_tracks_successful_tick_result() {
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
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.enrichment_telemetry(),
        CommonEnrichmentTelemetry {
            startup_x100: result.control.enrichment.startup_x100,
            warmup_x100: result.control.enrichment.warmup_x100,
            after_start_x100: result.control.enrichment.after_start_x100,
            acceleration_x100: result.control.enrichment.acceleration_x100,
            total_x100: result.control.enrichment.total_x100(),
        }
    );
}

#[test]
fn board_adapter_shift_arming_launch_cuts_on_next_tick() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.configure_speed_density_semantic(
        semantic_fuel_cut_calibration(2_500, 9_000),
        RuntimeSemanticState::default(),
    );
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter.set_shift_arming(true, false);

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(result.control.fuel_intent.fuel_cut);
    assert!(result.control.fuel_intent.spark_cut);
    assert!(adapter.runtime().snapshot().fuel_cut);
}

#[test]
fn board_adapter_shift_arming_flat_shift_cuts_on_next_tick() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.configure_speed_density_semantic(
        semantic_fuel_cut_calibration(9_000, 2_500),
        RuntimeSemanticState::default(),
    );
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter.set_shift_arming(false, true);

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(result.control.fuel_intent.fuel_cut);
    assert!(result.control.fuel_intent.spark_cut);
    assert!(adapter.runtime().snapshot().fuel_cut);
}

#[test]
fn board_adapter_default_shift_arming_leaves_next_tick_inactive() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.configure_speed_density_semantic(
        semantic_fuel_cut_calibration(2_500, 2_500),
        RuntimeSemanticState::default(),
    );
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(!result.control.fuel_intent.fuel_cut);
    assert!(!result.control.fuel_intent.spark_cut);
    assert!(!adapter.runtime().snapshot().fuel_cut);
}

#[test]
fn board_adapter_torque_telemetry_tracks_successful_tick_result() {
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
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.torque_telemetry(),
        CommonTorqueTelemetry {
            request_x1000: result.torque_observations.request_x1000,
            allowed_x1000: result.torque_observations.allowed_x1000,
            actuated_x1000: result.torque_observations.actuated_x1000,
        }
    );
}

#[test]
fn board_adapter_calibration_package_identity_defaults_to_runtime_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    let identity = adapter.calibration_package_identity();

    assert_eq!(
        identity,
        CalibrationPackageIdentity::from_snapshot(adapter.runtime().calibration_snapshot())
    );
    assert_eq!(identity.schema_version, CalibrationSchemaVersion::CURRENT);
    assert_eq!(identity.active_revision, CalibrationRevision::default());
    assert_eq!(
        identity.staged_base_revision,
        CalibrationRevision::default()
    );
    assert_eq!(identity.staged_revision, CalibrationRevision::default());
    assert!(!identity.staged_dirty);
}

#[test]
fn board_adapter_calibration_package_identity_tracks_runtime_updates() {
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

    adapter.set_staged_dirty(true);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap();

    let identity = adapter.calibration_package_identity();
    let mut expected =
        CalibrationPackageIdentity::from_snapshot(adapter.runtime().calibration_snapshot());
    expected.staged_dirty = true;

    assert_eq!(identity, expected);
}

#[test]
fn board_adapter_fault_surface_tracks_sync_loss_and_recovery() {
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
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(30),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: EngineTimeAuthority::none(),
            synced: false,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(31),
            cam_seen: false,
        })
        .unwrap();
    let sync_loss = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(35),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(sync_loss
        .actions
        .iter()
        .any(|action| matches!(action, Action::CancelScheduler(CancelReason::SyncLoss))));
    assert_eq!(adapter.fault_state(), FaultState::default());

    adapter.runtime.set_fault_state(
        FaultCode::SyncLoss,
        FaultSeverity::Warning,
        CancelReason::SyncLoss,
    );
    assert_eq!(
        adapter.fault_state(),
        FaultState {
            fault: FaultCode::SyncLoss,
            severity: FaultSeverity::Warning,
            cancel_reason: CancelReason::SyncLoss,
        }
    );

    adapter
        .runtime
        .set_fault_state(FaultCode::None, FaultSeverity::Info, CancelReason::Manual);
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(40),
            rpm: Rpm::new(1300),
            angle_x10: Degrees10::new(14),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(41),
            cam_seen: true,
        })
        .unwrap();
    let recovery = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(45),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(recovery
        .actions
        .iter()
        .all(|action| !matches!(action, Action::CancelScheduler(_))));
    assert_eq!(adapter.fault_state(), FaultState::default());
}

#[test]
fn board_adapter_fault_surface_tracks_safety_shutdown() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(50),
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
    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    assert_eq!(
        adapter.fault_state(),
        FaultState {
            fault: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
        }
    );

    let safety_shutdown = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(55),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(safety_shutdown.actions.iter().any(|action| matches!(
        action,
        Action::CancelScheduler(CancelReason::SafetyShutdown)
    )));
    assert_eq!(
        adapter.fault_state(),
        FaultState {
            fault: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
        }
    );
}

#[test]
fn board_adapter_fault_transition_telemetry_defaults_before_any_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.fault_transition_telemetry(),
        CommonFaultTransitionTelemetry::default()
    );
}

#[test]
fn board_adapter_fault_transition_telemetry_stays_put_without_fault_change() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let initial = adapter.fault_transition_telemetry();

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(55),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(adapter.fault_transition_telemetry(), initial);
}

#[test]
fn board_adapter_fault_transition_telemetry_records_and_overwrites_changes() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.runtime.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(60),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.fault_transition_telemetry(),
        CommonFaultTransitionTelemetry {
            changed: true,
            at_us: Micros::new(60),
            previous_fault: FaultCode::None,
            previous_severity: FaultSeverity::Info,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::SensorOutOfRange,
            current_severity: FaultSeverity::Warning,
            current_cancel_reason: CancelReason::Manual,
        }
    );

    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(65),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.fault_transition_telemetry(),
        CommonFaultTransitionTelemetry {
            changed: true,
            at_us: Micros::new(65),
            previous_fault: FaultCode::SensorOutOfRange,
            previous_severity: FaultSeverity::Warning,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::SafetyCut,
            current_severity: FaultSeverity::Critical,
            current_cancel_reason: CancelReason::SafetyShutdown,
        }
    );
}

#[test]
fn board_adapter_accepts_logical_sensor_snapshot_capture() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
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
    let logical_capture = BoardSensorSnapshotCapture {
        at_us: Micros::new(777),
        angle_x10: Degrees10::new(120),
        snapshot,
    };

    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: logical_capture,
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
    assert_eq!(adapter.logical_sensor_capture(), Some(logical_capture));
    assert_eq!(adapter.logical_sensor_snapshot(), Some(snapshot));
    assert_eq!(
        adapter.observability_snapshot().logical_sensor_capture,
        Some(logical_capture)
    );
    assert_eq!(
        adapter.observability_snapshot().logical_sensor_snapshot,
        Some(snapshot)
    );
}

#[test]
fn board_adapter_non_sensor_snapshot_events_do_not_fabricate_logical_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(11),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(adapter.logical_sensor_capture(), None);
    assert_eq!(adapter.logical_sensor_snapshot(), None);

    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(12),
            cam_seen: true,
        })
        .unwrap();
    assert_eq!(adapter.logical_sensor_capture(), None);
    assert_eq!(adapter.logical_sensor_snapshot(), None);

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(13),
            control: control_inputs(),
        })
        .unwrap();
    assert_eq!(adapter.logical_sensor_capture(), None);
    assert_eq!(adapter.logical_sensor_snapshot(), None);
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

#[test]
fn board_adapter_exposes_explicit_split_sync_state_transitions() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(200),
        rpm: Rpm::new(0),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(0),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<8>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.sync_state(), SplitSyncState::NoSignal);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(201),
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            authority: primary_searching_authority(),
            synced: false,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(202),
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            authority: primary_searching_authority(),
            synced: false,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(203),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::CrankSynced);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(204),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CamValidated720,
                AbsoluteTimeAuthority::GeometryOnly,
                900,
                0,
            ),
            synced: true,
        })
        .unwrap();
    assert_eq!(
        adapter.sync_state(),
        SplitSyncState::FullSequentialAuthorized
    );

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(205),
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::SyncLost,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::None,
                0,
                1,
            ),
            synced: false,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::SyncLost);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(206),
            rpm: Rpm::new(1_100),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CamValidated720,
                AbsoluteTimeAuthority::GeometryOnly,
                920,
                0,
            ),
            synced: true,
        })
        .unwrap();
    assert_eq!(
        adapter.sync_state(),
        SplitSyncState::FullSequentialAuthorized
    );
}

#[test]
fn split_live_inputs_and_board_adapter_track_loss_and_recovery_coherently() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(300),
        rpm: Rpm::new(0),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(0),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<8>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut live_inputs = SplitLiveInputs::new();

    live_inputs.apply_event(SplitLiveTriggerEvent::new(
        Rpm::new(0),
        Degrees10::new(0),
        false,
    ));
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(301),
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            authority: primary_searching_authority(),
            synced: false,
        })
        .unwrap();
    assert_eq!(live_inputs.sync_state(), SplitSyncState::Unsynced);
    assert_eq!(adapter.sync_state(), SplitSyncState::Unsynced);

    live_inputs.apply_event(SplitLiveTriggerEvent::new(
        Rpm::new(1_000),
        Degrees10::new(0),
        true,
    ));
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(302),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(live_inputs.sync_state(), SplitSyncState::CrankSynced);
    assert_eq!(adapter.sync_state(), SplitSyncState::CrankSynced);

    live_inputs.apply_event(SplitLiveTriggerEvent::new(
        Rpm::new(0),
        Degrees10::new(0),
        false,
    ));
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(303),
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::SyncLost,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::None,
                0,
                1,
            ),
            synced: false,
        })
        .unwrap();
    assert_eq!(live_inputs.sync_state(), SplitSyncState::SyncLost);
    assert_eq!(adapter.sync_state(), SplitSyncState::SyncLost);

    live_inputs.apply_event(SplitLiveTriggerEvent::new(
        Rpm::new(1_100),
        Degrees10::new(0),
        true,
    ));
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(304),
            rpm: Rpm::new(1_100),
            angle_x10: Degrees10::new(0),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(live_inputs.sync_state(), SplitSyncState::CrankSynced);
    assert_eq!(adapter.sync_state(), SplitSyncState::CrankSynced);
}

#[test]
fn board_adapter_diagnostics_snapshot_tracks_common_surfaces() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    let snapshot = adapter.diagnostics_snapshot();
    assert_eq!(snapshot.sync_state, SplitSyncState::NoSignal);
    assert_eq!(snapshot.fault_state, FaultState::default());
    assert_eq!(snapshot.timing_metrics, ScheduledTimingMetrics::default());
    assert_eq!(snapshot.sync_state, adapter.sync_state());
    assert_eq!(snapshot.fault_state, adapter.fault_state());

    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();

    let scheduled = adapter.diagnostics_snapshot();
    assert_eq!(scheduled.sync_state, SplitSyncState::NoSignal);
    assert_eq!(scheduled.fault_state, FaultState::default());
    assert_eq!(scheduled.timing_metrics, adapter.actions().timing_metrics());
    assert_ne!(scheduled.timing_metrics, ScheduledTimingMetrics::default());

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(400),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(401),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::SyncLost,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::None,
                0,
                1,
            ),
            synced: false,
        })
        .unwrap();

    let sync_lost = adapter.diagnostics_snapshot();
    assert_eq!(sync_lost.sync_state, SplitSyncState::SyncLost);
    assert_eq!(sync_lost.fault_state, FaultState::default());
    assert_eq!(sync_lost.timing_metrics, scheduled.timing_metrics);
    assert_eq!(sync_lost.sync_state, adapter.sync_state());
    assert_eq!(sync_lost.fault_state, adapter.fault_state());

    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let faulted = adapter.diagnostics_snapshot();
    assert_eq!(
        faulted.fault_state,
        FaultState {
            fault: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
        }
    );
    assert_eq!(faulted.sync_state, SplitSyncState::SyncLost);
    assert_eq!(faulted.timing_metrics, scheduled.timing_metrics);
    assert_eq!(faulted.sync_state, adapter.sync_state());
    assert_eq!(faulted.fault_state, adapter.fault_state());
}

#[test]
fn board_adapter_diagnostics_telemetry_defaults_cleanly() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.diagnostics_telemetry(),
        CommonDiagnosticsTelemetry {
            sync_state: CommonSyncTelemetryState::NoSignal,
            fault_code: FaultCode::None,
            fault_severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 0,
            last_drain_count: 0,
            active_queue_count: 0,
            free_queue_slots: 4,
            queue_capacity: 4,
        }
    );
}

#[test]
fn board_adapter_diagnostics_telemetry_projects_scheduler_metrics() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();

    assert_eq!(
        adapter.diagnostics_telemetry(),
        CommonDiagnosticsTelemetry {
            sync_state: CommonSyncTelemetryState::NoSignal,
            fault_code: FaultCode::None,
            fault_severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 4,
            last_drain_count: 0,
            active_queue_count: 4,
            free_queue_slots: 0,
            queue_capacity: 4,
        }
    );
}

#[test]
fn board_adapter_diagnostics_telemetry_projects_last_drain_count_after_draining_scheduler_queue() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();

    let mut drained = TransitionDrainBuffer::<8>::new();
    assert_eq!(
        adapter
            .actions()
            .drain_due(Micros::new(10_000), &mut drained),
        4
    );

    let diagnostics = adapter.diagnostics_telemetry();
    assert_eq!(diagnostics.sync_state, CommonSyncTelemetryState::NoSignal);
    assert_eq!(diagnostics.fault_code, FaultCode::None);
    assert_eq!(diagnostics.fault_severity, FaultSeverity::Info);
    assert_eq!(diagnostics.cancel_reason, CancelReason::Manual);
    assert_eq!(diagnostics.queue_high_water_mark, 4);
    assert_eq!(diagnostics.last_drain_count, 4);
    assert_eq!(diagnostics.active_queue_count, 0);
    assert_eq!(diagnostics.free_queue_slots, 4);
    assert_eq!(diagnostics.queue_capacity, 4);
}

#[test]
fn board_adapter_diagnostics_telemetry_projects_sync_loss_and_fault_state() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .runtime
        .set_engine_time_authority(EngineTimeAuthority::new(
            CrankSyncState::SyncLost,
            PhaseSyncState::Unknown,
            AbsoluteTimeAuthority::None,
            0,
            1,
        ));

    assert_eq!(
        adapter.diagnostics_telemetry().sync_state,
        CommonSyncTelemetryState::SyncLost
    );

    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    assert_eq!(
        adapter.diagnostics_telemetry(),
        CommonDiagnosticsTelemetry {
            sync_state: CommonSyncTelemetryState::SyncLost,
            fault_code: FaultCode::SafetyCut,
            fault_severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 0,
            last_drain_count: 0,
            active_queue_count: 0,
            free_queue_slots: 4,
            queue_capacity: 4,
        }
    );
}

#[test]
fn board_adapter_decision_telemetry_defaults_to_runtime_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.decision_telemetry(),
        CommonDecisionTelemetry {
            control_mode: ControlMode::default(),
            rev_soft_active: false,
            rev_hard_active: false,
            launch_active: false,
            flat_shift_active: false,
            fuel_cut: false,
            spark_cut: false,
        }
    );
}

#[test]
fn board_adapter_decision_telemetry_projects_runtime_snapshot_after_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let snapshot = adapter.runtime().snapshot();
    let decision = adapter.decision_telemetry();
    assert_eq!(decision.control_mode, snapshot.engine.mode);
    assert_eq!(decision.rev_soft_active, snapshot.rev_soft_active);
    assert_eq!(decision.rev_hard_active, snapshot.rev_hard_active);
    assert_eq!(decision.launch_active, snapshot.launch_active);
    assert_eq!(decision.flat_shift_active, snapshot.flat_shift_active);
    assert_eq!(decision.fuel_cut, snapshot.fuel_cut);
    assert_eq!(decision.spark_cut, snapshot.spark_cut);
    assert_eq!(
        decision,
        CommonDecisionTelemetry {
            control_mode: snapshot.engine.mode,
            rev_soft_active: snapshot.rev_soft_active,
            rev_hard_active: snapshot.rev_hard_active,
            launch_active: snapshot.launch_active,
            flat_shift_active: snapshot.flat_shift_active,
            fuel_cut: snapshot.fuel_cut,
            spark_cut: snapshot.spark_cut
        }
    );
}

#[test]
fn board_adapter_control_telemetry_defaults_to_runtime_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let snapshot = adapter.runtime().snapshot();

    assert_eq!(
        adapter.control_telemetry(),
        CommonControlTelemetry {
            fuel_pulse_width: snapshot.control.fuel_pulse_width,
            ignition_advance: snapshot.control.ignition_advance,
            dwell: snapshot.control.dwell,
            lambda_target: snapshot.control.lambda_target,
            torque_limit_x100: snapshot.control.torque_limit_x100,
        }
    );
}

#[test]
fn board_adapter_control_telemetry_projects_runtime_snapshot_after_tick() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let snapshot = adapter.runtime().snapshot();
    assert_eq!(
        adapter.control_telemetry(),
        CommonControlTelemetry {
            fuel_pulse_width: snapshot.control.fuel_pulse_width,
            ignition_advance: snapshot.control.ignition_advance,
            dwell: snapshot.control.dwell,
            lambda_target: snapshot.control.lambda_target,
            torque_limit_x100: snapshot.control.torque_limit_x100,
        }
    );
}

#[test]
fn board_adapter_control_reason_telemetry_defaults_before_any_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.control_reason_telemetry(),
        CommonControlReasonTelemetry::default()
    );
}

#[test]
fn board_adapter_non_tick_events_do_not_fabricate_control_reason_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(10),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(11),
            cam_seen: true,
        })
        .unwrap();

    assert_eq!(
        adapter.control_reason_telemetry(),
        CommonControlReasonTelemetry::default()
    );
}

#[test]
fn board_adapter_control_reason_telemetry_tracks_successful_tick_result() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let expected = CommonControlReasonTelemetry {
        lambda_mode: match result.control.lambda.mode {
            ecu_runtime::LambdaMode::OpenLoop => CommonLambdaMode::OpenLoop,
            ecu_runtime::LambdaMode::ClosedLoop => CommonLambdaMode::ClosedLoop,
        },
        lambda_active: result.control.lambda.active,
        lambda_trim_x100: result.control.lambda.trim_x100,
        ignition_limit_reason: match result.control.ignition.limit_reason {
            ecu_runtime::IgnitionLimitReason::None => CommonIgnitionLimitReason::None,
            ecu_runtime::IgnitionLimitReason::Knock => CommonIgnitionLimitReason::Knock,
            ecu_runtime::IgnitionLimitReason::Torque => CommonIgnitionLimitReason::Torque,
            ecu_runtime::IgnitionLimitReason::RevLimiter => CommonIgnitionLimitReason::RevLimiter,
        },
        torque_limit_reason: match result.control.torque.reason {
            ecu_runtime::TorqueLimitReason::None => CommonTorqueLimitReason::None,
            ecu_runtime::TorqueLimitReason::Idle => CommonTorqueLimitReason::Idle,
            ecu_runtime::TorqueLimitReason::Driver => CommonTorqueLimitReason::Driver,
            ecu_runtime::TorqueLimitReason::RevLimiter => CommonTorqueLimitReason::RevLimiter,
            ecu_runtime::TorqueLimitReason::Knock => CommonTorqueLimitReason::Knock,
            ecu_runtime::TorqueLimitReason::LimpMode => CommonTorqueLimitReason::LimpMode,
        },
    };

    assert_eq!(adapter.control_reason_telemetry(), expected);
}

#[test]
fn board_adapter_engine_telemetry_defaults_to_runtime_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.engine_telemetry(), CommonEngineTelemetry::default());
}

#[test]
fn board_adapter_engine_telemetry_projects_runtime_snapshot_after_trigger_and_tick() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
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
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let snapshot = adapter.runtime().snapshot();
    assert_eq!(
        adapter.engine_telemetry(),
        CommonEngineTelemetry {
            rpm: snapshot.engine.rpm,
            load_kpa10: snapshot.engine.load_kpa10,
            angle_x10: snapshot.engine.angle_x10,
            phase: snapshot.engine.phase,
        }
    );
}

#[test]
fn board_adapter_engine_time_telemetry_defaults_to_none_authority() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.engine_time_telemetry(),
        EngineTimeAuthorityTelemetry::new(EngineTimeAuthority::none())
    );
}

#[test]
fn board_adapter_engine_time_telemetry_projects_trigger_and_sync_updates() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

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

    let authority_after_trigger = adapter.runtime().engine_time_authority();
    assert_eq!(
        adapter.engine_time_telemetry(),
        EngineTimeAuthorityTelemetry::new(authority_after_trigger)
    );

    let sequential_authority = EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        PhaseSyncState::CamValidated720,
        AbsoluteTimeAuthority::GeometryOnly,
        900,
        0,
    );
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(14),
            rpm: Rpm::new(1300),
            angle_x10: Degrees10::new(14),
            authority: sequential_authority,
            synced: true,
        })
        .unwrap();

    let authority_after_sequential_trigger = adapter.runtime().engine_time_authority();
    assert_eq!(
        adapter.engine_time_telemetry(),
        EngineTimeAuthorityTelemetry::new(authority_after_sequential_trigger)
    );
}

#[test]
fn board_adapter_observability_snapshot_defaults_cleanly() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let expected = CommonObservabilitySnapshot {
        diagnostics: CommonDiagnosticsTelemetry {
            sync_state: CommonSyncTelemetryState::NoSignal,
            fault_code: FaultCode::None,
            fault_severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 0,
            last_drain_count: 0,
            active_queue_count: 0,
            free_queue_slots: 4,
            queue_capacity: 4,
        },
        fault_state: FaultState::default(),
        sync_state: adapter.sync_state(),
        decision: CommonDecisionTelemetry::default(),
        fuel_strategy_mode: CommonFuelStrategyMode::DirectPulseWidthTable,
        shift_arming: adapter.shift_arming_telemetry(),
        pending_input: adapter.pending_input_telemetry(),
        actions: CommonActionTelemetry::default(),
        control: adapter.control_telemetry(),
        control_reasons: adapter.control_reason_telemetry(),
        fuel: adapter.fuel_observation_telemetry(),
        enrichment: adapter.enrichment_telemetry(),
        torque: adapter.torque_telemetry(),
        cam_edge: adapter.cam_edge_telemetry(),
        engine: CommonEngineTelemetry::default(),
        validated: adapter.validated_input_telemetry(),
        frontier: adapter.frontier_telemetry(),
        scheduler_ownership: adapter.scheduler_ownership_telemetry(),
        scheduler_reservations: adapter.scheduler_reservation_telemetry(),
        scheduler_state_summary: adapter.scheduler_state_summary_telemetry(),
        scheduler_window: adapter.scheduler_window_telemetry(),
        engine_time: EngineTimeAuthorityTelemetry::new(EngineTimeAuthority::none()),
        fault_transition: adapter.fault_transition_telemetry(),
        calibration: adapter.calibration_package_identity(),
        capture_sample: None,
        logical_sensor_capture: None,
        logical_sensor_snapshot: None,
        trigger_edge: adapter.trigger_edge_telemetry(),
    };

    let snapshot = adapter.observability_snapshot();
    assert_eq!(snapshot.sync_state, SplitSyncState::NoSignal);
    assert_eq!(snapshot.sync_state, adapter.sync_state());
    assert_eq!(snapshot.fault_state, adapter.fault_state());
    assert_eq!(snapshot, expected);
}

#[test]
fn board_adapter_scheduler_ownership_telemetry_defaults_cleanly() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.scheduler_ownership_telemetry(),
        CommonSchedulerOwnershipTelemetry {
            mode: CommonSchedulerMode::Idle,
            active_groups: 0,
            injection_count: 0,
            ignition_count: 0,
        }
    );
}

#[test]
fn board_adapter_scheduler_ownership_telemetry_tracks_arm_and_sync_loss() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();

    let armed = adapter.scheduler_ownership_telemetry();
    assert_eq!(armed.mode, CommonSchedulerMode::Armed);
    assert!(armed.active_groups > 0);
    assert!(armed.injection_count > 0);
    assert!(armed.ignition_count > 0);

    adapter.actions().on_sync_loss();

    assert_eq!(
        adapter.scheduler_ownership_telemetry(),
        CommonSchedulerOwnershipTelemetry {
            mode: CommonSchedulerMode::Suspended,
            active_groups: 0,
            injection_count: 0,
            ignition_count: 0,
        }
    );
}

#[test]
fn board_adapter_scheduler_state_summary_telemetry_tracks_arm_and_sync_loss() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    assert_eq!(
        adapter.scheduler_state_summary_telemetry(),
        CommonSchedulerStateSummaryTelemetry { armed: false }
    );

    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();

    assert_eq!(
        adapter.scheduler_state_summary_telemetry(),
        CommonSchedulerStateSummaryTelemetry { armed: true }
    );

    adapter.actions().on_sync_loss();

    assert_eq!(
        adapter.scheduler_state_summary_telemetry(),
        CommonSchedulerStateSummaryTelemetry { armed: false }
    );
}

#[test]
fn board_adapter_scheduler_window_telemetry_defaults_cleanly() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.scheduler_window_telemetry(),
        CommonSchedulerWindowTelemetry {
            last_injection_start: None,
            last_injection_end: None,
            last_ignition_start: None,
            last_ignition_end: None,
        }
    );
}

#[test]
fn board_adapter_scheduler_window_telemetry_tracks_admitted_windows_and_survives_sync_loss() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .actions()
        .frontier_mut()
        .schedule_injection(
            Micros::new(0),
            Micros::new(100),
            Micros::new(120),
            SchedulerInjectionPlan {
                output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(0)),
                pulse_width: PulseWidthUs::new(20),
            },
        )
        .unwrap();
    adapter
        .actions()
        .frontier_mut()
        .schedule_ignition(
            Micros::new(0),
            Micros::new(200),
            Micros::new(230),
            SchedulerIgnitionPlan {
                output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
                dwell: DwellUs::new(30),
                advance: Degrees10::new(0),
            },
        )
        .unwrap();

    let expected = CommonSchedulerWindowTelemetry {
        last_injection_start: Some(Micros::new(100)),
        last_injection_end: Some(Micros::new(120)),
        last_ignition_start: Some(Micros::new(200)),
        last_ignition_end: Some(Micros::new(230)),
    };

    assert_eq!(adapter.scheduler_window_telemetry(), expected);

    adapter.actions().on_sync_loss();

    assert_eq!(adapter.scheduler_window_telemetry(), expected);
}

#[test]
fn board_adapter_frontier_telemetry_defaults_cleanly() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.frontier_telemetry(),
        CommonFrontierTelemetry {
            active_horizon_id: None,
            horizon_start_us: None,
            horizon_end_us: None,
            last_accepted_horizon_id: None,
            last_accepted_horizon_start_us: None,
            last_accepted_horizon_end_us: None,
            heartbeat_deadline_us: None,
            active_permit_mask: TimingIslandPermitMask::NONE,
            active_stop_reason: TimingIslandStopReason::None,
        }
    );
    assert_eq!(
        adapter.scheduler_window_telemetry(),
        CommonSchedulerWindowTelemetry {
            last_injection_start: None,
            last_injection_end: None,
            last_ignition_start: None,
            last_ignition_end: None,
        }
    );
}

#[test]
fn board_adapter_frontier_telemetry_tracks_commit_and_sync_loss() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let permit_mask = TimingIslandPermitMask::new(
        TimingIslandPermitMask::IGNITION | TimingIslandPermitMask::INJECTOR,
    );

    assert!(adapter.actions().commit_frontier_horizon(
        21,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        permit_mask,
    ));

    assert_eq!(
        adapter.frontier_telemetry(),
        CommonFrontierTelemetry {
            active_horizon_id: Some(21),
            horizon_start_us: Some(Micros::new(100)),
            horizon_end_us: Some(Micros::new(260)),
            last_accepted_horizon_id: Some(21),
            last_accepted_horizon_start_us: Some(Micros::new(100)),
            last_accepted_horizon_end_us: Some(Micros::new(260)),
            heartbeat_deadline_us: Some(Micros::new(150)),
            active_permit_mask: permit_mask,
            active_stop_reason: TimingIslandStopReason::None,
        }
    );

    adapter.actions().on_sync_loss();

    assert_eq!(
        adapter.frontier_telemetry(),
        CommonFrontierTelemetry {
            active_horizon_id: None,
            horizon_start_us: None,
            horizon_end_us: None,
            last_accepted_horizon_id: Some(21),
            last_accepted_horizon_start_us: Some(Micros::new(100)),
            last_accepted_horizon_end_us: Some(Micros::new(260)),
            heartbeat_deadline_us: None,
            active_permit_mask: TimingIslandPermitMask::NONE,
            active_stop_reason: TimingIslandStopReason::SyncLost,
        }
    );
}

#[test]
fn board_adapter_fuel_strategy_mode_defaults_to_direct_pulse_width_table() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::DirectPulseWidthTable
    );
}

#[test]
fn board_adapter_fuel_strategy_mode_tracks_semantic_and_runtime_strategy_selection() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let calibration = semantic_calibration();
    let state = RuntimeSemanticState::default();

    adapter.configure_speed_density_semantic(calibration, state);
    assert_eq!(
        adapter.fuel_strategy_mode(),
        CommonFuelStrategyMode::SpeedDensityVe
    );

    adapter.configure_runtime_fuel_strategy(RuntimeFuelStrategy::AlphaN { calibration, state });
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::AlphaN);

    adapter.configure_runtime_fuel_strategy(RuntimeFuelStrategy::Maf { calibration, state });
    assert_eq!(adapter.fuel_strategy_mode(), CommonFuelStrategyMode::Maf);
}

#[test]
fn board_adapter_observability_snapshot_reports_fuel_strategy_mode() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let snapshot = adapter.observability_snapshot();

    assert_eq!(snapshot.fuel_strategy_mode, adapter.fuel_strategy_mode());
}

#[test]
fn board_adapter_observability_snapshot_tracks_sync_state_updates() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    let snapshot = adapter.observability_snapshot();
    assert_eq!(snapshot.sync_state, adapter.sync_state());

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(400),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let snapshot = adapter.observability_snapshot();
    assert_eq!(snapshot.sync_state, adapter.sync_state());

    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(401),
            cam_seen: true,
        })
        .unwrap();

    let snapshot = adapter.observability_snapshot();
    assert_eq!(snapshot.sync_state, adapter.sync_state());
}

#[test]
fn board_adapter_observability_snapshot_projects_runtime_fault_state() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let snapshot = adapter.observability_snapshot();
    let expected = FaultState {
        fault: FaultCode::SafetyCut,
        severity: FaultSeverity::Critical,
        cancel_reason: CancelReason::SafetyShutdown,
    };

    assert_eq!(snapshot.fault_state, expected);
    assert_eq!(snapshot.fault_state, adapter.fault_state());
}

#[test]
fn board_adapter_capture_sample_defaults_to_none() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.capture_sample(), None);
}

#[test]
fn board_adapter_poll_sensor_stores_the_sampled_capture() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.poll_sensor().unwrap(), sample);
    assert_eq!(adapter.capture_sample(), Some(sample));
    assert_eq!(adapter.capture().count, 1);
    assert_eq!(adapter.capture().last, Some(sample));
}

#[test]
fn board_adapter_trigger_edge_stores_the_synthetic_capture_sample() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(11),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1200),
                    map_kpa10: Kpa10::new(450),
                    tps_x100: 4_200,
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();

    let expected = CaptureSample {
        at_us: Micros::new(12),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: expected.at_us,
            rpm: expected.rpm,
            angle_x10: expected.angle_x10,
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    assert_eq!(adapter.capture_sample(), Some(expected));
    assert_eq!(adapter.capture().last, Some(expected));
}

#[test]
fn board_adapter_sensor_snapshot_capture_stores_the_mapped_capture_sample() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let logical_capture = BoardSensorSnapshotCapture {
        at_us: Micros::new(777),
        angle_x10: Degrees10::new(120),
        snapshot: BoardSensorSnapshot {
            rpm: Rpm::new(2_300),
            map_kpa10: Kpa10::new(880),
            tps_x100: 4_200,
            ..BoardSensorSnapshot::default()
        },
    };

    let expected = map_speed_density_capture_sample(logical_capture);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: logical_capture,
        })
        .unwrap();

    assert_eq!(adapter.capture_sample(), Some(expected));
    assert_eq!(adapter.capture().last, Some(expected));
    assert_eq!(adapter.logical_sensor_capture(), Some(logical_capture));
}

#[test]
fn board_adapter_cam_edge_and_tick_do_not_fabricate_a_capture_sample() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
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
    let expected = adapter.capture_sample();

    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(13),
            cam_seen: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(adapter.capture_sample(), expected);
}

#[test]
fn board_adapter_observability_snapshot_tracks_scheduler_diagnostics_changes() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();

    assert_eq!(
        adapter.observability_snapshot().diagnostics,
        adapter.diagnostics_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().diagnostics,
        CommonDiagnosticsTelemetry {
            sync_state: CommonSyncTelemetryState::NoSignal,
            fault_code: FaultCode::None,
            fault_severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 4,
            last_drain_count: 0,
            active_queue_count: 4,
            free_queue_slots: 0,
            queue_capacity: 4,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().decision,
        adapter.decision_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().actions,
        adapter.action_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().control,
        adapter.control_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().control_reasons,
        adapter.control_reason_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().torque,
        adapter.torque_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().enrichment,
        adapter.enrichment_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().validated,
        adapter.validated_input_telemetry()
    );
}

#[test]
fn board_adapter_observability_snapshot_tracks_calibration_identity_changes_after_staged_dirty_update(
) {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.set_staged_dirty(true);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap();

    let mut expected =
        CalibrationPackageIdentity::from_snapshot(adapter.runtime().calibration_snapshot());
    expected.staged_dirty = true;

    assert_eq!(adapter.observability_snapshot().calibration, expected);
    assert!(adapter.observability_snapshot().calibration.staged_dirty);
}

#[test]
fn board_adapter_observability_snapshot_matches_individual_accessors() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .unwrap();
    adapter.set_staged_dirty(true);

    let snapshot = adapter.observability_snapshot();

    assert_eq!(snapshot.diagnostics, adapter.diagnostics_telemetry());
    assert_eq!(snapshot.decision, adapter.decision_telemetry());
    assert_eq!(snapshot.fuel_strategy_mode, adapter.fuel_strategy_mode());
    assert_eq!(snapshot.control, adapter.control_telemetry());
    assert_eq!(snapshot.control_reasons, adapter.control_reason_telemetry());
    assert_eq!(snapshot.torque, adapter.torque_telemetry());
    assert_eq!(snapshot.fuel, adapter.fuel_observation_telemetry());
    assert_eq!(snapshot.engine, adapter.engine_telemetry());
    assert_eq!(snapshot.validated, adapter.validated_input_telemetry());
    assert_eq!(snapshot.frontier, adapter.frontier_telemetry());
    assert_eq!(
        snapshot.scheduler_ownership,
        adapter.scheduler_ownership_telemetry()
    );
    assert_eq!(snapshot.engine_time, adapter.engine_time_telemetry());
    assert_eq!(
        snapshot.fault_transition,
        adapter.fault_transition_telemetry()
    );
    assert_eq!(snapshot.calibration, adapter.calibration_package_identity());
    assert_eq!(snapshot.capture_sample, adapter.capture_sample());
    assert_eq!(
        snapshot.logical_sensor_capture,
        adapter.logical_sensor_capture()
    );
    assert_eq!(
        snapshot.logical_sensor_snapshot,
        adapter.logical_sensor_snapshot()
    );
    assert_eq!(snapshot.trigger_edge, adapter.trigger_edge_telemetry());
    assert_eq!(
        snapshot,
        CommonObservabilitySnapshot {
            diagnostics: adapter.diagnostics_telemetry(),
            fault_state: adapter.fault_state(),
            sync_state: adapter.sync_state(),
            decision: adapter.decision_telemetry(),
            fuel_strategy_mode: adapter.fuel_strategy_mode(),
            shift_arming: adapter.shift_arming_telemetry(),
            pending_input: adapter.pending_input_telemetry(),
            actions: adapter.action_telemetry(),
            control: adapter.control_telemetry(),
            control_reasons: adapter.control_reason_telemetry(),
            fuel: adapter.fuel_observation_telemetry(),
            enrichment: adapter.enrichment_telemetry(),
            torque: adapter.torque_telemetry(),
            cam_edge: adapter.cam_edge_telemetry(),
            engine: adapter.engine_telemetry(),
            validated: adapter.validated_input_telemetry(),
            frontier: adapter.frontier_telemetry(),
            scheduler_ownership: adapter.scheduler_ownership_telemetry(),
            scheduler_reservations: adapter.scheduler_reservation_telemetry(),
            scheduler_state_summary: adapter.scheduler_state_summary_telemetry(),
            scheduler_window: adapter.scheduler_window_telemetry(),
            engine_time: adapter.engine_time_telemetry(),
            fault_transition: adapter.fault_transition_telemetry(),
            calibration: adapter.calibration_package_identity(),
            capture_sample: adapter.capture_sample(),
            logical_sensor_capture: adapter.logical_sensor_capture(),
            logical_sensor_snapshot: adapter.logical_sensor_snapshot(),
            trigger_edge: adapter.trigger_edge_telemetry(),
        }
    );
}

#[test]
fn board_adapter_observability_sample_defaults_to_timestamp_zero_and_current_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let sample = adapter.observability_sample();

    assert_eq!(
        sample,
        CommonObservabilitySample {
            at_us: Micros::new(0),
            snapshot: adapter.observability_snapshot(),
        }
    );
    assert_eq!(sample.at_us, Micros::new(0));
    assert_eq!(sample.snapshot, adapter.observability_snapshot());
}

#[test]
fn board_adapter_pending_input_telemetry_defaults_cleanly() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: Micros::new(0),
            rpm: Rpm::default(),
            load_kpa10: Kpa10::default(),
            angle_x10: Degrees10::default(),
            authority: EngineTimeAuthority::none(),
        }
    );
}

#[test]
fn board_adapter_poll_sensor_updates_pending_input_telemetry() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.poll_sensor().unwrap();

    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: sample.at_us,
            rpm: sample.rpm,
            load_kpa10: sample.load_kpa10,
            angle_x10: sample.angle_x10,
            authority: EngineTimeAuthority::none(),
        }
    );
}

#[test]
fn board_adapter_trigger_edge_updates_pending_input_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let authority = primary_locked_authority();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(1300),
            angle_x10: Degrees10::new(14),
            authority,
            synced: true,
        })
        .unwrap();

    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: Micros::new(12),
            rpm: Rpm::new(1300),
            load_kpa10: Kpa10::default(),
            angle_x10: Degrees10::new(14),
            authority,
        }
    );
}

#[test]
fn board_adapter_cam_edge_can_change_pending_input_authority() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(1300),
            angle_x10: Degrees10::new(14),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    let before = adapter.pending_input_telemetry();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(13),
            cam_seen: true,
        })
        .unwrap();
    let after = adapter.pending_input_telemetry();

    assert_eq!(after.now_us, Micros::new(13));
    assert_eq!(after.authority, adapter.runtime().engine_time_authority());
    assert_ne!(after.authority, before.authority);
}

#[test]
fn board_adapter_sensor_snapshot_capture_updates_pending_input_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let logical_capture = BoardSensorSnapshotCapture {
        at_us: Micros::new(777),
        angle_x10: Degrees10::new(120),
        snapshot: BoardSensorSnapshot {
            rpm: Rpm::new(2_300),
            map_kpa10: Kpa10::new(880),
            tps_x100: 4_200,
            ..BoardSensorSnapshot::default()
        },
    };

    let expected = map_speed_density_capture_sample(logical_capture);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: logical_capture,
        })
        .unwrap();

    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: expected.at_us,
            rpm: expected.rpm,
            load_kpa10: expected.load_kpa10,
            angle_x10: expected.angle_x10,
            authority: EngineTimeAuthority::none(),
        }
    );
}

#[test]
fn board_adapter_observability_snapshot_includes_pending_input() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(1300),
            angle_x10: Degrees10::new(14),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let snapshot = adapter.observability_snapshot();
    assert_eq!(snapshot.pending_input, adapter.pending_input_telemetry());
}

#[test]
fn board_adapter_action_telemetry_defaults_to_zero_before_tick() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    assert_eq!(adapter.action_telemetry(), CommonActionTelemetry::default());
    assert_eq!(adapter.action_telemetry().total_action_count, 0);
}

#[test]
fn board_adapter_non_tick_events_do_not_fabricate_action_telemetry() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(11),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(12),
            cam_seen: true,
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(13),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();

    assert_eq!(adapter.action_telemetry(), CommonActionTelemetry::default());
    assert_eq!(adapter.action_telemetry().total_action_count, 0);
}

#[test]
fn board_adapter_action_telemetry_tracks_successful_tick_result() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.set_staged_dirty(true);

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let mut expected = CommonActionTelemetry::default();
    assert!(result
        .actions
        .iter()
        .any(|action| matches!(action, Action::PersistCalibration)));
    for action in result.actions.iter() {
        expected.total_action_count = expected.total_action_count.saturating_add(1);
        match action {
            Action::ArmScheduler { .. } => {
                expected.arm_scheduler_count = expected.arm_scheduler_count.saturating_add(1);
            }
            Action::ArmInjection(_) => {
                expected.arm_injection_count = expected.arm_injection_count.saturating_add(1);
            }
            Action::ArmIgnition(_) => {
                expected.arm_ignition_count = expected.arm_ignition_count.saturating_add(1);
            }
            Action::ApplyAux(commands) => {
                expected.apply_aux_count = expected.apply_aux_count.saturating_add(1);
                expected.apply_aux_command_count = expected
                    .apply_aux_command_count
                    .saturating_add(commands.len().min(u8::MAX as usize) as u8);
            }
            Action::PublishSnapshot => {
                expected.publish_snapshot = true;
                expected.publish_snapshot_count = expected.publish_snapshot_count.saturating_add(1);
            }
            Action::PersistCalibration => {
                expected.persist_calibration = true;
                expected.persist_calibration_count =
                    expected.persist_calibration_count.saturating_add(1);
            }
            Action::CancelScheduler(cancel_reason) => {
                if expected.cancel_scheduler && expected.cancel_reason != cancel_reason {
                    expected.multiple_cancel_reasons = true;
                }
                expected.cancel_scheduler = true;
                expected.cancel_reason = cancel_reason;
            }
            Action::Idle => {
                expected.idle_count = expected.idle_count.saturating_add(1);
            }
        }
    }

    assert!(expected.total_action_count > 0);
    assert!(expected.persist_calibration);
    assert!(expected.persist_calibration_count > 0);
    assert_eq!(adapter.action_telemetry(), expected);
}

#[test]
fn board_adapter_action_telemetry_tracks_cancel_scheduler_count_and_reason() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(50),
        rpm: Rpm::new(1500),
        load_kpa10: Kpa10::new(500),
        angle_x10: Degrees10::new(15),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_fuel_model(test_fuel_model());
    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(55),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let mut expected = CommonActionTelemetry::default();
    for action in result.actions.iter() {
        expected.total_action_count = expected.total_action_count.saturating_add(1);
        match action {
            Action::ArmScheduler { .. } => {
                expected.arm_scheduler_count = expected.arm_scheduler_count.saturating_add(1);
            }
            Action::ArmInjection(_) => {
                expected.arm_injection_count = expected.arm_injection_count.saturating_add(1);
            }
            Action::ArmIgnition(_) => {
                expected.arm_ignition_count = expected.arm_ignition_count.saturating_add(1);
            }
            Action::ApplyAux(commands) => {
                expected.apply_aux_count = expected.apply_aux_count.saturating_add(1);
                expected.apply_aux_command_count = expected
                    .apply_aux_command_count
                    .saturating_add(commands.len().min(u8::MAX as usize) as u8);
            }
            Action::PublishSnapshot => {
                expected.publish_snapshot = true;
                expected.publish_snapshot_count = expected.publish_snapshot_count.saturating_add(1);
            }
            Action::PersistCalibration => {
                expected.persist_calibration = true;
                expected.persist_calibration_count =
                    expected.persist_calibration_count.saturating_add(1);
            }
            Action::CancelScheduler(cancel_reason) => {
                if expected.cancel_scheduler && expected.cancel_reason != cancel_reason {
                    expected.multiple_cancel_reasons = true;
                }
                expected.cancel_scheduler = true;
                expected.cancel_reason = cancel_reason;
                expected.cancel_scheduler_count = expected.cancel_scheduler_count.saturating_add(1);
            }
            Action::Idle => {
                expected.idle_count = expected.idle_count.saturating_add(1);
            }
        }
    }

    assert!(expected.total_action_count > 0);
    assert!(expected.cancel_scheduler);
    assert_eq!(expected.cancel_reason, CancelReason::SafetyShutdown);
    assert!(expected.cancel_scheduler_count > 0);
    assert_eq!(adapter.action_telemetry(), expected);
}

#[test]
fn board_adapter_action_telemetry_flags_multiple_cancel_reasons() {
    let mut actions = ActionBatch::<RUNTIME_ACTION_CAP>::new();
    assert!(actions.push(Action::CancelScheduler(CancelReason::SafetyShutdown)));
    assert!(actions.push(Action::CancelScheduler(CancelReason::SyncLoss)));

    let result = synthetic_step_result(actions);
    let telemetry = super::common_action_telemetry(&result);

    assert!(telemetry.cancel_scheduler);
    assert!(telemetry.cancel_scheduler_count >= 2);
    assert!(telemetry.multiple_cancel_reasons);
    assert_eq!(telemetry.cancel_reason, CancelReason::SyncLoss);
}

#[test]
fn board_adapter_action_telemetry_tracks_apply_aux_command_count_in_limp_home() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(50),
        rpm: Rpm::new(1500),
        load_kpa10: Kpa10::new(500),
        angle_x10: Degrees10::new(15),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_fuel_model(test_fuel_model());
    adapter.runtime.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );

    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(55),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let mut expected = CommonActionTelemetry::default();
    let mut saw_apply_aux = false;
    for action in result.actions.iter() {
        expected.total_action_count = expected.total_action_count.saturating_add(1);
        match action {
            Action::ArmScheduler { .. } => {
                expected.arm_scheduler_count = expected.arm_scheduler_count.saturating_add(1);
            }
            Action::ArmInjection(_) => {
                expected.arm_injection_count = expected.arm_injection_count.saturating_add(1);
            }
            Action::ArmIgnition(_) => {
                expected.arm_ignition_count = expected.arm_ignition_count.saturating_add(1);
            }
            Action::ApplyAux(commands) => {
                saw_apply_aux = true;
                expected.apply_aux_count = expected.apply_aux_count.saturating_add(1);
                expected.apply_aux_command_count = expected
                    .apply_aux_command_count
                    .saturating_add(commands.len().min(u8::MAX as usize) as u8);
            }
            Action::PublishSnapshot => {
                expected.publish_snapshot = true;
                expected.publish_snapshot_count = expected.publish_snapshot_count.saturating_add(1);
            }
            Action::PersistCalibration => {
                expected.persist_calibration = true;
                expected.persist_calibration_count =
                    expected.persist_calibration_count.saturating_add(1);
            }
            Action::CancelScheduler(cancel_reason) => {
                if expected.cancel_scheduler && expected.cancel_reason != cancel_reason {
                    expected.multiple_cancel_reasons = true;
                }
                expected.cancel_scheduler = true;
                expected.cancel_reason = cancel_reason;
                expected.cancel_scheduler_count = expected.cancel_scheduler_count.saturating_add(1);
            }
            Action::Idle => {
                expected.idle_count = expected.idle_count.saturating_add(1);
            }
        }
    }

    assert!(expected.total_action_count > 0);
    assert!(saw_apply_aux);
    assert!(expected.apply_aux_command_count > 0);
    assert_eq!(adapter.action_telemetry(), expected);
    assert_eq!(adapter.observability_snapshot().actions, expected);
}

#[test]
fn board_adapter_observability_sample_tracks_trigger_and_tick_timestamps() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(adapter.observability_sample().at_us, Micros::new(12));

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    assert_eq!(adapter.observability_sample().at_us, Micros::new(20));
}

#[test]
fn board_adapter_observability_sample_matches_observability_snapshot() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.set_staged_dirty(true);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let snapshot = adapter.observability_snapshot();
    let sample = adapter.observability_sample();

    assert_eq!(sample.snapshot, snapshot);
    assert_eq!(
        sample,
        CommonObservabilitySample {
            at_us: Micros::new(20),
            snapshot,
        }
    );
}

#[test]
fn board_adapter_observability_sample_is_a_pure_timestamp_plus_snapshot_composition() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
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

    let sample = adapter.observability_sample();

    assert_eq!(sample.at_us, Micros::new(13));
    assert_eq!(sample.snapshot, adapter.observability_snapshot());
    assert_eq!(
        sample,
        CommonObservabilitySample {
            at_us: Micros::new(13),
            snapshot: adapter.observability_snapshot(),
        }
    );
}

#[test]
fn fixed_common_observability_trace_defaults_cleanly() {
    let trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();

    assert_eq!(trace.len(), 0);
    assert_eq!(trace.capacity(), 2);
    assert_eq!(trace.free_slots(), 2);
    assert!(trace.is_empty());
    assert_eq!(trace.overflow_count(), 0);
    assert_eq!(trace.get(0), None);
}

#[test]
fn fixed_common_observability_trace_status_tracks_lifecycle() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(2),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let third = CommonObservabilitySample {
        at_us: Micros::new(3),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 0,
            capacity: 2,
            free_slots: 2,
            overflow_count: 0,
        }
    );

    trace.push(first).unwrap();
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 1,
            capacity: 2,
            free_slots: 1,
            overflow_count: 0,
        }
    );

    trace.push(second).unwrap();
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityTraceOverflow { capacity: 2 });
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 2,
            capacity: 2,
            free_slots: 0,
            overflow_count: 1,
        }
    );

    assert_eq!(trace.take_overflow_count(), 1);
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 2,
            capacity: 2,
            free_slots: 0,
            overflow_count: 0,
        }
    );

    let mut out = [None; 1];
    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 1);
    assert_eq!(out, [Some(first)]);
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 1,
            capacity: 2,
            free_slots: 1,
            overflow_count: 0,
        }
    );
    assert_eq!(trace.get(0), Some(second));
}

#[test]
fn common_observability_trace_status_pair_reports_independent_statuses() {
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(11),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_second = CommonObservabilitySample {
        at_us: Micros::new(12),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(21),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(22),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    let sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    assert_eq!(
        common_observability_trace_status(&sample_trace, &record_trace),
        CommonObservabilityTracePairStatus {
            sample: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 1,
                free_slots: 1,
                overflow_count: 0,
            },
            record: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 1,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );

    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    sample_trace.push(sample_first).unwrap();
    sample_trace.push(sample_second).unwrap();
    let record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    assert_eq!(
        common_observability_trace_status(&sample_trace, &record_trace),
        CommonObservabilityTracePairStatus {
            sample: CommonObservabilityTraceStatus {
                len: 2,
                capacity: 2,
                free_slots: 0,
                overflow_count: 0,
            },
            record: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 1,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );

    let sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    record_trace.push(record_first).unwrap();
    record_trace.push(record_second).unwrap();
    assert_eq!(
        common_observability_trace_status(&sample_trace, &record_trace),
        CommonObservabilityTracePairStatus {
            sample: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 1,
                free_slots: 1,
                overflow_count: 0,
            },
            record: CommonObservabilityTraceStatus {
                len: 2,
                capacity: 2,
                free_slots: 0,
                overflow_count: 0,
            },
        }
    );
}

#[test]
fn common_observability_trace_status_pair_preserves_independent_overflow_states() {
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(31),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(41),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    sample_trace.push(sample_first).unwrap();
    let sample_overflow = sample_trace.push(CommonObservabilitySample {
        at_us: Micros::new(32),
        snapshot: CommonObservabilitySnapshot::default(),
    });
    record_trace.push(record_first).unwrap();
    let record_overflow = record_trace.push(CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorPoll,
        sample: CommonObservabilitySample {
            at_us: Micros::new(42),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    });

    assert_eq!(
        sample_overflow,
        Err(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        record_overflow,
        Err(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(
        common_observability_trace_status(&sample_trace, &record_trace),
        CommonObservabilityTracePairStatus {
            sample: CommonObservabilityTraceStatus {
                len: 1,
                capacity: 1,
                free_slots: 0,
                overflow_count: 1,
            },
            record: CommonObservabilityTraceStatus {
                len: 1,
                capacity: 1,
                free_slots: 0,
                overflow_count: 1,
            },
        }
    );
}

#[test]
fn common_observability_trace_status_pair_matches_direct_status_calls() {
    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    sample_trace
        .push(CommonObservabilitySample {
            at_us: Micros::new(51),
            snapshot: CommonObservabilitySnapshot::default(),
        })
        .unwrap();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorSnapshotCapture,
            sample: CommonObservabilitySample {
                at_us: Micros::new(61),
                snapshot: CommonObservabilitySnapshot::default(),
            },
        })
        .unwrap();
    let _ = record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: CommonObservabilitySample {
                at_us: Micros::new(62),
                snapshot: CommonObservabilitySnapshot::default(),
            },
        })
        .unwrap_err();

    let pair_status = common_observability_trace_status(&sample_trace, &record_trace);

    assert_eq!(pair_status.sample, sample_trace.status());
    assert_eq!(pair_status.record, record_trace.status());
}

#[test]
fn fixed_common_observability_trace_pair_defaults_to_empty_status() {
    let pair: FixedCommonObservabilityTracePair<2, 1> = FixedCommonObservabilityTracePair::new();
    let sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        pair.status(),
        common_observability_trace_status(&sample_trace, &record_trace)
    );
    assert_eq!(pair.sample().status(), sample_trace.status());
    assert_eq!(pair.record().status(), record_trace.status());
}

#[test]
fn fixed_common_observability_trace_pair_mut_accessors_feed_status() {
    let mut pair: FixedCommonObservabilityTracePair<2, 2> =
        FixedCommonObservabilityTracePair::new();
    let sample = CommonObservabilitySample {
        at_us: Micros::new(71),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(72),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    pair.sample_mut().push(sample).unwrap();
    pair.record_mut().push(record).unwrap();

    assert_eq!(
        pair.status(),
        CommonObservabilityTracePairStatus {
            sample: CommonObservabilityTraceStatus {
                len: 1,
                capacity: 2,
                free_slots: 1,
                overflow_count: 0,
            },
            record: CommonObservabilityTraceStatus {
                len: 1,
                capacity: 2,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(pair.sample().get(0), Some(sample));
    assert_eq!(pair.record().get(0), Some(record));
}

#[test]
fn fixed_common_observability_trace_pair_drain_cycle_matches_free_helper() {
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(81),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_second = CommonObservabilitySample {
        at_us: Micros::new(82),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(91),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(92),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    let mut pair: FixedCommonObservabilityTracePair<2, 2> =
        FixedCommonObservabilityTracePair::new();
    pair.sample_mut().push(sample_first).unwrap();
    pair.sample_mut().push(sample_second).unwrap();
    pair.record_mut().push(record_first).unwrap();
    pair.record_mut().push(record_second).unwrap();

    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    sample_trace.push(sample_first).unwrap();
    sample_trace.push(sample_second).unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    record_trace.push(record_first).unwrap();
    record_trace.push(record_second).unwrap();

    let mut pair_sample_out = [CommonObservabilitySample::default(); 2];
    let mut pair_record_out = [CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample::default(),
    }; 2];
    let mut helper_sample_out = [None; 2];
    let mut helper_record_out = [None; 2];

    let pair_report = pair.drain_cycle(&mut pair_sample_out, &mut pair_record_out);
    let helper_report = drain_common_observability_cycle(
        &mut sample_trace,
        &mut helper_sample_out,
        &mut record_trace,
        &mut helper_record_out,
    );

    assert_eq!(pair_report, helper_report);
    assert_eq!(pair_sample_out, [sample_first, sample_second]);
    assert_eq!(pair_record_out, [record_first, record_second]);
    assert_eq!(helper_sample_out, [Some(sample_first), Some(sample_second)]);
    assert_eq!(helper_record_out, [Some(record_first), Some(record_second)]);
    assert_eq!(pair.sample().status(), sample_trace.status());
    assert_eq!(pair.record().status(), record_trace.status());
    assert_eq!(
        pair.status(),
        common_observability_trace_status(pair.sample(), pair.record())
    );
}

#[test]
fn fixed_common_observability_trace_pair_accessors_expose_underlying_traces() {
    let mut pair: FixedCommonObservabilityTracePair<2, 2> =
        FixedCommonObservabilityTracePair::new();
    let sample = CommonObservabilitySample {
        at_us: Micros::new(101),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorPoll,
        sample: CommonObservabilitySample {
            at_us: Micros::new(102),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    pair.sample_mut().push(sample).unwrap();
    pair.record_mut().push(record).unwrap();

    let sample_trace = pair.sample();
    let record_trace = pair.record();

    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(record_trace.get(0), Some(record));
    assert_eq!(
        pair.status(),
        common_observability_trace_status(sample_trace, record_trace)
    );
}

#[test]
fn fixed_common_observability_trace_reports_headroom_consistently() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(2),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let third = CommonObservabilitySample {
        at_us: Micros::new(3),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    assert_eq!(trace.capacity(), 2);
    assert_eq!(trace.free_slots(), 2);

    trace.push(first).unwrap();
    assert_eq!(trace.free_slots(), 1);

    trace.push(second).unwrap();
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityTraceOverflow { capacity: 2 });
    assert_eq!(trace.capacity(), 2);
    assert_eq!(trace.free_slots(), 0);

    let mut out = [None; 1];
    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 1);
    assert_eq!(out, [Some(first)]);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.free_slots(), 1);
    assert_eq!(trace.get(0), Some(second));
}

#[test]
fn fixed_common_observability_trace_pushes_samples_in_order() {
    let mut trace: FixedCommonObservabilityTrace<3> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(2),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    assert_eq!(trace.push(first), Ok(()));
    assert_eq!(trace.push(second), Ok(()));
    assert_eq!(trace.len(), 2);
    assert_eq!(trace.get(0), Some(first));
    assert_eq!(trace.get(1), Some(second));
}

#[test]
fn fixed_common_observability_trace_reports_overflow_explicitly() {
    let mut trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(10),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(11),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    assert_eq!(trace.push(first), Ok(()));
    let err = trace.push(second).unwrap_err();

    assert_eq!(err, CommonObservabilityTraceOverflow { capacity: 1 });
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.overflow_count(), 1);
    assert_eq!(trace.get(0), Some(first));
}

#[test]
fn fixed_common_observability_trace_takes_overflow_count_without_touching_samples() {
    let mut trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(12),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(13),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    assert_eq!(trace.take_overflow_count(), 0);
    assert_eq!(trace.push(first), Ok(()));
    let err = trace.push(second).unwrap_err();

    assert_eq!(err, CommonObservabilityTraceOverflow { capacity: 1 });
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));
    assert_eq!(trace.take_overflow_count(), 1);
    assert_eq!(trace.take_overflow_count(), 0);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));

    let third = CommonObservabilitySample {
        at_us: Micros::new(14),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityTraceOverflow { capacity: 1 });
    assert_eq!(trace.take_overflow_count(), 1);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));
}

#[test]
fn fixed_common_observability_trace_clear_resets_len_and_preserves_capacity_behavior() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(20),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(21),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let third = CommonObservabilitySample {
        at_us: Micros::new(22),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.clear();

    assert_eq!(trace.len(), 0);
    assert!(trace.is_empty());
    assert_eq!(trace.get(0), None);

    assert_eq!(trace.push(first), Ok(()));
    assert_eq!(trace.push(second), Ok(()));
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityTraceOverflow { capacity: 2 });
    assert_eq!(trace.len(), 2);
    assert_eq!(trace.overflow_count(), 1);
    assert_eq!(trace.get(0), Some(first));
    assert_eq!(trace.get(1), Some(second));
}

#[test]
fn fixed_common_observability_trace_drain_into_empty_trace_returns_zero() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let mut out = [None; 2];

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 0);
    assert!(trace.is_empty());
    assert_eq!(trace.len(), 0);
    assert_eq!(out, [None, None]);
}

#[test]
fn fixed_common_observability_trace_drain_into_larger_output_clears_trace() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(31),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(32),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out = [None; 3];

    trace.push(first).unwrap();
    trace.push(second).unwrap();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 2);
    assert!(trace.is_empty());
    assert_eq!(trace.len(), 0);
    assert_eq!(trace.get(0), None);
    assert_eq!(out, [Some(first), Some(second), None]);
}

#[test]
fn fixed_common_observability_trace_drain_into_smaller_output_compacts_remainder() {
    let mut trace: FixedCommonObservabilityTrace<3> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(41),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(42),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let third = CommonObservabilitySample {
        at_us: Micros::new(43),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out = [None; 2];

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.push(third).unwrap();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 2);
    assert_eq!(out, [Some(first), Some(second)]);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(third));
    assert_eq!(trace.get(1), None);
}

#[test]
fn fixed_common_observability_trace_drain_into_empty_output_preserves_trace() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(51),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out: [Option<CommonObservabilitySample>; 0] = [];

    trace.push(first).unwrap();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 0);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));
}

#[test]
fn fixed_common_observability_trace_drain_preserves_overflow_count() {
    let mut trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(61),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(62),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out = [None; 1];

    trace.push(first).unwrap();
    let _ = trace.push(second).unwrap_err();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 1);
    assert_eq!(out, [Some(first)]);
    assert_eq!(trace.overflow_count(), 1);
    assert!(trace.is_empty());
}

#[test]
fn fixed_common_observability_trace_drain_with_status_returns_zero_for_empty_trace() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let mut out = [Some(CommonObservabilitySample::default()); 2];

    let report = trace.drain_with_status(&mut out);

    assert_eq!(report.drained, 0);
    assert_eq!(
        report.status,
        CommonObservabilityTraceStatus {
            len: 0,
            capacity: 2,
            free_slots: 2,
            overflow_count: 0,
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(out, [None, None]);
}

#[test]
fn fixed_common_observability_trace_drain_with_status_clears_larger_output() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(71),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(72),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out = [None; 3];

    trace.push(first).unwrap();
    trace.push(second).unwrap();

    let report = trace.drain_with_status(&mut out);

    assert_eq!(report.drained, 2);
    assert_eq!(
        report.status,
        CommonObservabilityTraceStatus {
            len: 0,
            capacity: 2,
            free_slots: 2,
            overflow_count: 0,
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(trace.get(0), None);
    assert_eq!(out, [Some(first), Some(second), None]);
}

#[test]
fn fixed_common_observability_trace_drain_with_status_compacts_remainder_and_preserves_overflow() {
    let mut trace: FixedCommonObservabilityTrace<3> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(81),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(82),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let third = CommonObservabilitySample {
        at_us: Micros::new(83),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let fourth = CommonObservabilitySample {
        at_us: Micros::new(84),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out = [None; 2];

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.push(third).unwrap();
    let _ = trace.push(fourth).unwrap_err();

    let report = trace.drain_with_status(&mut out);

    assert_eq!(report.drained, 2);
    assert_eq!(
        report.status,
        CommonObservabilityTraceStatus {
            len: 1,
            capacity: 3,
            free_slots: 2,
            overflow_count: 1,
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(out, [Some(first), Some(second)]);
    assert_eq!(trace.get(0), Some(third));
    assert_eq!(trace.overflow_count(), 1);
}

#[test]
fn fixed_common_observability_trace_drain_cycle_returns_zero_for_empty_trace() {
    let mut trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let mut out = [Some(CommonObservabilitySample::default()); 2];

    let report = trace.drain_cycle(&mut out);

    assert_eq!(
        report,
        CommonObservabilityTraceCycleReport {
            drained: 0,
            overflow_count: 0,
            status: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 2,
                free_slots: 2,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(trace.overflow_count(), 0);
    assert_eq!(out, [None, None]);
}

#[test]
fn fixed_common_observability_trace_drain_cycle_compacts_remainder_and_captures_overflow() {
    let mut trace: FixedCommonObservabilityTrace<3> = FixedCommonObservabilityTrace::new();
    let first = CommonObservabilitySample {
        at_us: Micros::new(111),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let second = CommonObservabilitySample {
        at_us: Micros::new(112),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let third = CommonObservabilitySample {
        at_us: Micros::new(113),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let fourth = CommonObservabilitySample {
        at_us: Micros::new(114),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut out = [None; 1];

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.push(third).unwrap();
    let _ = trace.push(fourth).unwrap_err();

    let report = trace.drain_cycle(&mut out);

    assert_eq!(
        report,
        CommonObservabilityTraceCycleReport {
            drained: 1,
            overflow_count: 1,
            status: CommonObservabilityTraceStatus {
                len: 2,
                capacity: 3,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(trace.overflow_count(), 0);
    assert_eq!(out, [Some(first)]);
    assert_eq!(trace.get(0), Some(second));
    assert_eq!(trace.get(1), Some(third));
}

#[test]
fn board_adapter_push_observability_sample_stores_current_sample() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(7),
            cam_seen: true,
        })
        .unwrap();

    let expected = adapter.observability_sample();
    let mut trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();

    assert_eq!(adapter.push_observability_sample(&mut trace), Ok(()));
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(expected));
}

#[test]
fn fixed_common_observability_record_trace_defaults_cleanly() {
    let trace: FixedCommonObservabilityRecordTrace<2> = FixedCommonObservabilityRecordTrace::new();

    assert_eq!(trace.len(), 0);
    assert_eq!(trace.capacity(), 2);
    assert_eq!(trace.free_slots(), 2);
    assert!(trace.is_empty());
    assert_eq!(trace.overflow_count(), 0);
    assert_eq!(trace.get(0), None);
}

#[test]
fn fixed_common_observability_record_trace_status_tracks_lifecycle() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(2),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(3),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 0,
            capacity: 2,
            free_slots: 2,
            overflow_count: 0,
        }
    );

    trace.push(first).unwrap();
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 1,
            capacity: 2,
            free_slots: 1,
            overflow_count: 0,
        }
    );

    trace.push(second).unwrap();
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 2 });
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 2,
            capacity: 2,
            free_slots: 0,
            overflow_count: 1,
        }
    );

    assert_eq!(trace.take_overflow_count(), 1);
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 2,
            capacity: 2,
            free_slots: 0,
            overflow_count: 0,
        }
    );

    let mut out = [None; 1];
    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 1);
    assert_eq!(out, [Some(first)]);
    assert_eq!(
        trace.status(),
        CommonObservabilityTraceStatus {
            len: 1,
            capacity: 2,
            free_slots: 1,
            overflow_count: 0,
        }
    );
    assert_eq!(trace.get(0), Some(second));
}

#[test]
fn fixed_common_observability_record_trace_reports_headroom_consistently() {
    let mut trace: FixedCommonObservabilityRecordTrace<3> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(2),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(3),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let fourth = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorPoll,
        sample: CommonObservabilitySample {
            at_us: Micros::new(4),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    assert_eq!(trace.capacity(), 3);
    assert_eq!(trace.free_slots(), 3);

    trace.push(first).unwrap();
    assert_eq!(trace.free_slots(), 2);

    trace.push(second).unwrap();
    trace.push(third).unwrap();
    let err = trace.push(fourth).unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 3 });
    assert_eq!(trace.capacity(), 3);
    assert_eq!(trace.free_slots(), 0);

    let mut out = [None; 2];
    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 2);
    assert_eq!(out, [Some(first), Some(second)]);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.free_slots(), 2);
    assert_eq!(trace.get(0), Some(third));
}

#[test]
fn fixed_common_observability_record_trace_stores_records_in_order() {
    let mut trace: FixedCommonObservabilityRecordTrace<3> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(1),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(2),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    assert_eq!(trace.push(first), Ok(()));
    assert_eq!(trace.push(second), Ok(()));
    assert_eq!(trace.len(), 2);
    assert_eq!(trace.get(0), Some(first));
    assert_eq!(trace.get(1), Some(second));
}

#[test]
fn fixed_common_observability_record_trace_reports_overflow_explicitly() {
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorSnapshotCapture,
        sample: CommonObservabilitySample {
            at_us: Micros::new(10),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(11),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    assert_eq!(trace.push(first), Ok(()));
    let err = trace.push(second).unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.overflow_count(), 1);
    assert_eq!(trace.get(0), Some(first));
}

#[test]
fn fixed_common_observability_record_trace_takes_overflow_count_without_touching_records() {
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(12),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(13),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    assert_eq!(trace.take_overflow_count(), 0);
    assert_eq!(trace.push(first), Ok(()));
    let err = trace.push(second).unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));
    assert_eq!(trace.take_overflow_count(), 1);
    assert_eq!(trace.take_overflow_count(), 0);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));

    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(14),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 1 });
    assert_eq!(trace.take_overflow_count(), 1);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));
}

#[test]
fn fixed_common_observability_record_trace_clear_resets_len_and_preserves_capacity_behavior() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(20),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(21),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(22),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.clear();

    assert_eq!(trace.len(), 0);
    assert!(trace.is_empty());
    assert_eq!(trace.get(0), None);

    assert_eq!(trace.push(first), Ok(()));
    assert_eq!(trace.push(second), Ok(()));
    let err = trace.push(third).unwrap_err();

    assert_eq!(err, CommonObservabilityRecordTraceOverflow { capacity: 2 });
    assert_eq!(trace.len(), 2);
    assert_eq!(trace.overflow_count(), 1);
    assert_eq!(trace.get(0), Some(first));
    assert_eq!(trace.get(1), Some(second));
}

#[test]
fn fixed_common_observability_record_trace_drain_into_empty_trace_returns_zero() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let mut out = [None; 2];

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 0);
    assert!(trace.is_empty());
    assert_eq!(trace.len(), 0);
    assert_eq!(out, [None, None]);
}

#[test]
fn fixed_common_observability_record_trace_drain_into_larger_output_clears_trace() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(31),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(32),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out = [None; 3];

    trace.push(first).unwrap();
    trace.push(second).unwrap();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 2);
    assert!(trace.is_empty());
    assert_eq!(trace.len(), 0);
    assert_eq!(trace.get(0), None);
    assert_eq!(out, [Some(first), Some(second), None]);
}

#[test]
fn fixed_common_observability_record_trace_drain_into_smaller_output_compacts_remainder() {
    let mut trace: FixedCommonObservabilityRecordTrace<3> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(41),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(42),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(43),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out = [None; 2];

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.push(third).unwrap();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 2);
    assert_eq!(out, [Some(first), Some(second)]);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(third));
    assert_eq!(trace.get(1), None);
}

#[test]
fn fixed_common_observability_record_trace_drain_into_empty_output_preserves_trace() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(51),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out: [Option<CommonObservabilityRecord>; 0] = [];

    trace.push(first).unwrap();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 0);
    assert_eq!(trace.len(), 1);
    assert_eq!(trace.get(0), Some(first));
}

#[test]
fn fixed_common_observability_record_trace_drain_preserves_overflow_count() {
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(61),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(62),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out = [None; 1];

    trace.push(first).unwrap();
    let _ = trace.push(second).unwrap_err();

    let drained = trace.drain_into(&mut out);

    assert_eq!(drained, 1);
    assert_eq!(out, [Some(first)]);
    assert_eq!(trace.overflow_count(), 1);
    assert!(trace.is_empty());
}

#[test]
fn fixed_common_observability_record_trace_drain_with_status_returns_zero_for_empty_trace() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let mut out = [Some(CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample::default(),
    }); 2];

    let report = trace.drain_with_status(&mut out);

    assert_eq!(report.drained, 0);
    assert_eq!(
        report.status,
        CommonObservabilityTraceStatus {
            len: 0,
            capacity: 2,
            free_slots: 2,
            overflow_count: 0,
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(out, [None, None]);
}

#[test]
fn fixed_common_observability_record_trace_drain_with_status_clears_larger_output() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(91),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(92),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out = [None; 3];

    trace.push(first).unwrap();
    trace.push(second).unwrap();

    let report = trace.drain_with_status(&mut out);

    assert_eq!(report.drained, 2);
    assert_eq!(
        report.status,
        CommonObservabilityTraceStatus {
            len: 0,
            capacity: 2,
            free_slots: 2,
            overflow_count: 0,
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(trace.get(0), None);
    assert_eq!(out, [Some(first), Some(second), None]);
}

#[test]
fn fixed_common_observability_record_trace_drain_with_status_compacts_remainder_and_preserves_overflow(
) {
    let mut trace: FixedCommonObservabilityRecordTrace<3> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(101),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(102),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(103),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let fourth = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorPoll,
        sample: CommonObservabilitySample {
            at_us: Micros::new(104),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out = [None; 2];

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.push(third).unwrap();
    let _ = trace.push(fourth).unwrap_err();

    let report = trace.drain_with_status(&mut out);

    assert_eq!(report.drained, 2);
    assert_eq!(
        report.status,
        CommonObservabilityTraceStatus {
            len: 1,
            capacity: 3,
            free_slots: 2,
            overflow_count: 1,
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(out, [Some(first), Some(second)]);
    assert_eq!(trace.get(0), Some(third));
    assert_eq!(trace.overflow_count(), 1);
}

#[test]
fn fixed_common_observability_record_trace_drain_cycle_returns_zero_for_empty_trace() {
    let mut trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let mut out = [Some(CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample::default(),
    }); 2];

    let report = trace.drain_cycle(&mut out);

    assert_eq!(
        report,
        CommonObservabilityTraceCycleReport {
            drained: 0,
            overflow_count: 0,
            status: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 2,
                free_slots: 2,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(trace.overflow_count(), 0);
    assert_eq!(out, [None, None]);
}

#[test]
fn fixed_common_observability_record_trace_drain_cycle_compacts_remainder_and_captures_overflow() {
    let mut trace: FixedCommonObservabilityRecordTrace<3> =
        FixedCommonObservabilityRecordTrace::new();
    let first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(121),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(122),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(123),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let fourth = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorPoll,
        sample: CommonObservabilitySample {
            at_us: Micros::new(124),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut out = [None; 1];

    trace.push(first).unwrap();
    trace.push(second).unwrap();
    trace.push(third).unwrap();
    let _ = trace.push(fourth).unwrap_err();

    let report = trace.drain_cycle(&mut out);

    assert_eq!(
        report,
        CommonObservabilityTraceCycleReport {
            drained: 1,
            overflow_count: 1,
            status: CommonObservabilityTraceStatus {
                len: 2,
                capacity: 3,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(trace.status(), report.status);
    assert_eq!(trace.overflow_count(), 0);
    assert_eq!(out, [Some(first)]);
    assert_eq!(trace.get(0), Some(second));
    assert_eq!(trace.get(1), Some(third));
}

#[test]
fn drain_common_observability_cycle_returns_zero_for_empty_traces() {
    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let mut sample_out = [Some(CommonObservabilitySample::default()); 2];
    let mut record_out = [Some(CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample::default(),
    }); 2];

    let report = drain_common_observability_cycle(
        &mut sample_trace,
        &mut sample_out,
        &mut record_trace,
        &mut record_out,
    );

    assert_eq!(
        report,
        CommonObservabilityDrainCycleReport {
            sample: CommonObservabilityTraceCycleReport {
                drained: 0,
                overflow_count: 0,
                status: CommonObservabilityTraceStatus {
                    len: 0,
                    capacity: 2,
                    free_slots: 2,
                    overflow_count: 0,
                },
            },
            record: CommonObservabilityTraceCycleReport {
                drained: 0,
                overflow_count: 0,
                status: CommonObservabilityTraceStatus {
                    len: 0,
                    capacity: 2,
                    free_slots: 2,
                    overflow_count: 0,
                },
            },
        }
    );
    assert_eq!(sample_trace.status(), report.sample.status);
    assert_eq!(record_trace.status(), report.record.status);
    assert_eq!(sample_out, [None, None]);
    assert_eq!(record_out, [None, None]);
}

#[test]
fn drain_common_observability_cycle_drains_full_traces_in_order() {
    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(131),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_second = CommonObservabilitySample {
        at_us: Micros::new(132),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(141),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(142),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut sample_out = [None; 2];
    let mut record_out = [None; 2];

    sample_trace.push(sample_first).unwrap();
    sample_trace.push(sample_second).unwrap();
    record_trace.push(record_first).unwrap();
    record_trace.push(record_second).unwrap();

    let report = drain_common_observability_cycle(
        &mut sample_trace,
        &mut sample_out,
        &mut record_trace,
        &mut record_out,
    );

    assert_eq!(
        report.sample,
        CommonObservabilityTraceCycleReport {
            drained: 2,
            overflow_count: 0,
            status: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 2,
                free_slots: 2,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(
        report.record,
        CommonObservabilityTraceCycleReport {
            drained: 2,
            overflow_count: 0,
            status: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 2,
                free_slots: 2,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(sample_trace.status(), report.sample.status);
    assert_eq!(record_trace.status(), report.record.status);
    assert_eq!(sample_out, [Some(sample_first), Some(sample_second)]);
    assert_eq!(record_out, [Some(record_first), Some(record_second)]);
    assert!(sample_trace.is_empty());
    assert!(record_trace.is_empty());
}

#[test]
fn drain_common_observability_cycle_preserves_remainders_with_smaller_outputs() {
    let mut sample_trace: FixedCommonObservabilityTrace<3> = FixedCommonObservabilityTrace::new();
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(151),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_second = CommonObservabilitySample {
        at_us: Micros::new(152),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_third = CommonObservabilitySample {
        at_us: Micros::new(153),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut record_trace: FixedCommonObservabilityRecordTrace<3> =
        FixedCommonObservabilityRecordTrace::new();
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::TriggerEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(161),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CamEdge,
        sample: CommonObservabilitySample {
            at_us: Micros::new(162),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_third = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(163),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut sample_out = [None; 1];
    let mut record_out = [None; 1];

    sample_trace.push(sample_first).unwrap();
    sample_trace.push(sample_second).unwrap();
    sample_trace.push(sample_third).unwrap();
    record_trace.push(record_first).unwrap();
    record_trace.push(record_second).unwrap();
    record_trace.push(record_third).unwrap();

    let report = drain_common_observability_cycle(
        &mut sample_trace,
        &mut sample_out,
        &mut record_trace,
        &mut record_out,
    );

    assert_eq!(
        report.sample,
        CommonObservabilityTraceCycleReport {
            drained: 1,
            overflow_count: 0,
            status: CommonObservabilityTraceStatus {
                len: 2,
                capacity: 3,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(
        report.record,
        CommonObservabilityTraceCycleReport {
            drained: 1,
            overflow_count: 0,
            status: CommonObservabilityTraceStatus {
                len: 2,
                capacity: 3,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(sample_trace.status(), report.sample.status);
    assert_eq!(record_trace.status(), report.record.status);
    assert_eq!(sample_out, [Some(sample_first)]);
    assert_eq!(record_out, [Some(record_first)]);
    assert_eq!(sample_trace.get(0), Some(sample_second));
    assert_eq!(sample_trace.get(1), Some(sample_third));
    assert_eq!(record_trace.get(0), Some(record_second));
    assert_eq!(record_trace.get(1), Some(record_third));
}

#[test]
fn drain_common_observability_cycle_captures_overflow_independently_for_both_traces() {
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(171),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_second = CommonObservabilitySample {
        at_us: Micros::new(172),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::SensorPoll,
        sample: CommonObservabilitySample {
            at_us: Micros::new(181),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: CommonObservabilitySample {
            at_us: Micros::new(182),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut sample_out = [None; 1];
    let mut record_out = [None; 1];

    sample_trace.push(sample_first).unwrap();
    let _ = sample_trace.push(sample_second).unwrap_err();
    record_trace.push(record_first).unwrap();
    let _ = record_trace.push(record_second).unwrap_err();

    let report = drain_common_observability_cycle(
        &mut sample_trace,
        &mut sample_out,
        &mut record_trace,
        &mut record_out,
    );

    assert_eq!(
        report.sample,
        CommonObservabilityTraceCycleReport {
            drained: 1,
            overflow_count: 1,
            status: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 1,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(
        report.record,
        CommonObservabilityTraceCycleReport {
            drained: 1,
            overflow_count: 1,
            status: CommonObservabilityTraceStatus {
                len: 0,
                capacity: 1,
                free_slots: 1,
                overflow_count: 0,
            },
        }
    );
    assert_eq!(sample_trace.status(), report.sample.status);
    assert_eq!(record_trace.status(), report.record.status);
    assert_eq!(sample_trace.overflow_count(), 0);
    assert_eq!(record_trace.overflow_count(), 0);
    assert_eq!(sample_out, [Some(sample_first)]);
    assert_eq!(record_out, [Some(record_first)]);
    assert!(sample_trace.is_empty());
    assert!(record_trace.is_empty());
}

#[test]
fn drain_common_observability_cycle_reports_post_call_state_for_both_traces() {
    let mut sample_trace: FixedCommonObservabilityTrace<3> = FixedCommonObservabilityTrace::new();
    let sample_first = CommonObservabilitySample {
        at_us: Micros::new(191),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_second = CommonObservabilitySample {
        at_us: Micros::new(192),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let sample_third = CommonObservabilitySample {
        at_us: Micros::new(193),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    let record_first = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::ShiftArming,
        sample: CommonObservabilitySample {
            at_us: Micros::new(201),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let record_second = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::CalibrationStagedDirty,
        sample: CommonObservabilitySample {
            at_us: Micros::new(202),
            snapshot: CommonObservabilitySnapshot::default(),
        },
    };
    let mut sample_out = [None; 2];
    let mut record_out = [None; 1];

    sample_trace.push(sample_first).unwrap();
    sample_trace.push(sample_second).unwrap();
    sample_trace.push(sample_third).unwrap();
    record_trace.push(record_first).unwrap();
    record_trace.push(record_second).unwrap();

    let report = drain_common_observability_cycle(
        &mut sample_trace,
        &mut sample_out,
        &mut record_trace,
        &mut record_out,
    );

    assert_eq!(sample_trace.status(), report.sample.status);
    assert_eq!(record_trace.status(), report.record.status);
    assert_eq!(report.sample.drained, 2);
    assert_eq!(report.record.drained, 1);
    assert_eq!(sample_out, [Some(sample_first), Some(sample_second)]);
    assert_eq!(record_out, [Some(record_first)]);
    assert_eq!(sample_trace.get(0), Some(sample_third));
    assert_eq!(record_trace.get(0), Some(record_second));
}

#[test]
fn board_adapter_push_observability_record_stores_expected_kind_plus_current_sample() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(7),
            cam_seen: true,
        })
        .unwrap();

    let expected_sample = adapter.observability_sample();
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.push_observability_record(&mut trace, CommonObservabilityRecordKind::CamEdge),
        Ok(())
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_push_observability_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(7),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let expected_sample = adapter.observability_sample();
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(
        adapter.push_observability_pair(
            &mut sample_trace,
            &mut record_trace,
            CommonObservabilityRecordKind::TriggerEdge,
        ),
        Ok(())
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(expected_sample));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_push_observability_pair_returns_record_overflow_before_touching_sample_trace() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(7),
            cam_seen: true,
        })
        .unwrap();

    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    sample_trace.push(sample_before).unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .push_observability_pair(
            &mut sample_trace,
            &mut record_trace,
            CommonObservabilityRecordKind::CamEdge,
        )
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_push_observability_pair_returns_sample_overflow_after_record_is_stored() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(9),
                angle_x10: Degrees10::new(12),
                snapshot: BoardSensorSnapshot::default(),
            },
        })
        .unwrap();

    let expected_sample = adapter.observability_sample();
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample::default())
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .push_observability_pair(
            &mut sample_trace,
            &mut record_trace,
            CommonObservabilityRecordKind::SensorSnapshotCapture,
        )
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorSnapshotCapture,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_observability_helpers_still_store_single_trace_entries() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap();

    let expected_sample = adapter.observability_sample();
    let mut sample_trace: FixedCommonObservabilityTrace<2> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();

    assert_eq!(adapter.push_observability_sample(&mut sample_trace), Ok(()));
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(expected_sample));

    assert_eq!(
        adapter.push_observability_record(&mut record_trace, CommonObservabilityRecordKind::Tick,),
        Ok(())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_poll_sensor_and_record_stores_sensor_poll_kind_and_current_sample() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let sampled = adapter.poll_sensor_and_record(&mut trace).unwrap();

    assert_eq!(sampled, sample);
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorPoll,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_poll_sensor_and_record_returns_sensor_error_without_recording() {
    let sensor = FailingSensor;
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter.poll_sensor_and_record(&mut trace).unwrap_err();

    assert!(matches!(
        err,
        PollSensorAndRecordError::Poll(BoardAdapterError::Sensor(()))
    ));
    assert_eq!(trace.len(), 0);
    assert!(trace.is_empty());
}

#[test]
fn board_adapter_poll_sensor_and_record_returns_capture_error_without_recording() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = FailingCapture;
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter.poll_sensor_and_record(&mut trace).unwrap_err();

    assert!(matches!(
        err,
        PollSensorAndRecordError::Poll(BoardAdapterError::Capture(()))
    ));
    assert_eq!(trace.len(), 0);
    assert!(trace.is_empty());
}

#[test]
fn board_adapter_poll_sensor_and_record_returns_record_overflow_after_updating_sample() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: CommonObservabilitySample {
                at_us: Micros::new(1),
                snapshot: CommonObservabilitySnapshot::default(),
            },
        })
        .unwrap();

    let err = adapter.poll_sensor_and_record(&mut trace).unwrap_err();

    assert_eq!(
        err,
        PollSensorAndRecordError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(trace.len(), 1);
    assert_eq!(adapter.capture_sample(), Some(sample));
    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: sample.at_us,
            rpm: sample.rpm,
            load_kpa10: sample.load_kpa10,
            angle_x10: sample.angle_x10,
            authority: EngineTimeAuthority::none(),
        }
    );
}

#[test]
fn board_adapter_poll_sensor_and_push_pair_stores_matching_sample_and_record() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let sampled = adapter
        .poll_sensor_and_push_pair(&mut sample_trace, &mut record_trace)
        .unwrap();

    let expected_sample = adapter.observability_sample();

    assert_eq!(sampled, sample);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(expected_sample));
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorPoll,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_poll_sensor_and_push_pair_returns_sensor_error_without_recording() {
    let sensor = FailingSensor;
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .poll_sensor_and_push_pair(&mut sample_trace, &mut record_trace)
        .unwrap_err();

    assert!(matches!(
        err,
        PollSensorAndPushPairError::Poll(BoardAdapterError::Sensor(()))
    ));
    assert_eq!(sample_trace.len(), 0);
    assert_eq!(record_trace.len(), 0);
}

#[test]
fn board_adapter_poll_sensor_and_push_pair_returns_record_overflow_before_touching_sample_trace() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    sample_trace.push(sample_before).unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .poll_sensor_and_push_pair(&mut sample_trace, &mut record_trace)
        .unwrap_err();

    assert_eq!(
        err,
        PollSensorAndPushPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
    assert_eq!(adapter.capture_sample(), Some(sample));
    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: sample.at_us,
            rpm: sample.rpm,
            load_kpa10: sample.load_kpa10,
            angle_x10: sample.angle_x10,
            authority: EngineTimeAuthority::none(),
        }
    );
}

#[test]
fn board_adapter_poll_sensor_and_push_pair_returns_sample_overflow_after_record_is_stored() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    sample_trace
        .push(CommonObservabilitySample::default())
        .unwrap();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .poll_sensor_and_push_pair(&mut sample_trace, &mut record_trace)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PollSensorAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorPoll,
            sample: expected_sample,
        })
    );
    assert_eq!(adapter.capture_sample(), Some(sample));
    assert_eq!(
        adapter.pending_input_telemetry(),
        CommonPendingInputTelemetry {
            now_us: sample.at_us,
            rpm: sample.rpm,
            load_kpa10: sample.load_kpa10,
            angle_x10: sample.angle_x10,
            authority: EngineTimeAuthority::none(),
        }
    );
}

#[test]
fn board_adapter_poll_sensor_and_push_to_trace_pair_stores_matching_sample_and_record() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    let sampled = adapter
        .poll_sensor_and_push_to_trace_pair(&mut traces)
        .unwrap();
    let expected_sample = adapter.observability_sample();

    assert_eq!(sampled, sample);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(expected_sample));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorPoll,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_poll_sensor_and_push_to_trace_pair_returns_sensor_error_without_recording() {
    let sensor = FailingSensor;
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces.record_mut().push(record_before).unwrap();

    let err = adapter
        .poll_sensor_and_push_to_trace_pair(&mut traces)
        .unwrap_err();

    assert!(matches!(
        err,
        PollSensorAndPushPairError::Poll(BoardAdapterError::Sensor(()))
    ));
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.record().get(0), Some(record_before));
}

#[test]
fn board_adapter_poll_sensor_and_push_to_trace_pair_returns_capture_error_without_recording() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = FailingCapture;
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces.record_mut().push(record_before).unwrap();

    let err = adapter
        .poll_sensor_and_push_to_trace_pair(&mut traces)
        .unwrap_err();

    assert!(matches!(
        err,
        PollSensorAndPushPairError::Poll(BoardAdapterError::Capture(()))
    ));
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.record().get(0), Some(record_before));
}

#[test]
fn board_adapter_poll_sensor_and_push_to_trace_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces.record_mut().push(record_before).unwrap();

    let err = adapter
        .poll_sensor_and_push_to_trace_pair(&mut traces)
        .unwrap_err();

    assert_eq!(
        err,
        PollSensorAndPushPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.record().get(0), Some(record_before));
    assert_eq!(adapter.capture_sample(), Some(sample));
}

#[test]
fn board_adapter_poll_sensor_and_push_to_trace_pair_returns_sample_overflow_after_record_is_stored()
{
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let sensor = MockSensor(sample);
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 2> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();

    let err = adapter
        .poll_sensor_and_push_to_trace_pair(&mut traces)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PollSensorAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorPoll,
            sample: expected_sample,
        })
    );
    assert_eq!(adapter.capture_sample(), Some(sample));
}

#[test]
fn board_adapter_push_observability_to_trace_pair_stores_matching_sample_and_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<2, 2> =
        FixedCommonObservabilityTracePair::new();
    let kind = CommonObservabilityRecordKind::SensorPoll;
    let expected_sample = adapter.observability_sample();

    adapter
        .push_observability_to_trace_pair(&mut traces, kind)
        .unwrap();

    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(expected_sample));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_push_observability_to_trace_pair_returns_record_overflow_before_touching_sample_trace(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces
        .record_mut()
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .push_observability_to_trace_pair(&mut traces, CommonObservabilityRecordKind::Tick)
        .unwrap_err();

    assert_eq!(
        err,
        PushObservabilityPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_push_observability_to_trace_pair_returns_sample_overflow_after_record_is_stored() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 2> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();
    let kind = CommonObservabilityRecordKind::SensorPoll;

    let err = adapter
        .push_observability_to_trace_pair(&mut traces, kind)
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        PushObservabilityPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_event_observability_kind_maps_each_variant() {
    assert_eq!(
        BoardEvent::TriggerEdge {
            at_us: Micros::new(1),
            rpm: Rpm::new(2),
            angle_x10: Degrees10::new(3),
            authority: EngineTimeAuthority::none(),
            synced: false,
        }
        .observability_kind(),
        CommonObservabilityRecordKind::TriggerEdge
    );
    assert_eq!(
        BoardEvent::CamEdge {
            at_us: Micros::new(4),
            cam_seen: true,
        }
        .observability_kind(),
        CommonObservabilityRecordKind::CamEdge
    );
    assert_eq!(
        BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(5),
                angle_x10: Degrees10::new(6),
                snapshot: BoardSensorSnapshot::default(),
            },
        }
        .observability_kind(),
        CommonObservabilityRecordKind::SensorSnapshotCapture
    );
    assert_eq!(
        BoardEvent::Tick {
            now_us: Micros::new(7),
            control: control_inputs(),
        }
        .observability_kind(),
        CommonObservabilityRecordKind::Tick
    );
}

#[test]
fn board_adapter_apply_event_and_record_stores_successful_non_tick_event() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let result = adapter
        .apply_event_and_record(
            BoardEvent::CamEdge {
                at_us: Micros::new(7),
                cam_seen: true,
            },
            &mut trace,
        )
        .unwrap();

    assert_eq!(result, None);
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_apply_event_and_record_stores_successful_tick_event_and_returns_step_result() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let result = adapter
        .apply_event_and_record(
            BoardEvent::Tick {
                now_us: Micros::new(20),
                control: control_inputs(),
            },
            &mut trace,
        )
        .unwrap();

    assert!(result.is_some());
    assert_eq!(trace.len(), 1);
    assert_eq!(
        trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_apply_event_and_record_skips_record_when_apply_event_fails() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = FailingCapture;
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let err = adapter
        .apply_event_and_record(
            BoardEvent::SensorSnapshotCapture {
                capture: BoardSensorSnapshotCapture {
                    at_us: Micros::new(12),
                    angle_x10: Degrees10::new(12),
                    snapshot: BoardSensorSnapshot {
                        rpm: Rpm::new(1200),
                        map_kpa10: Kpa10::new(450),
                        ..BoardSensorSnapshot::default()
                    },
                },
            },
            &mut trace,
        )
        .unwrap_err();

    assert!(matches!(
        err,
        ApplyAndRecordError::Apply(BoardAdapterError::Capture(()))
    ));
    assert_eq!(trace.len(), 0);
}

#[test]
fn board_adapter_apply_event_and_record_returns_record_overflow_after_applying_event() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();
    let filler_sample = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };

    for kind in [
        CommonObservabilityRecordKind::TriggerEdge,
        CommonObservabilityRecordKind::CamEdge,
        CommonObservabilityRecordKind::SensorSnapshotCapture,
        CommonObservabilityRecordKind::Tick,
    ] {
        trace
            .push(CommonObservabilityRecord {
                kind,
                sample: filler_sample,
            })
            .unwrap();
    }

    let err = adapter
        .apply_event_and_record(
            BoardEvent::SensorSnapshotCapture {
                capture: BoardSensorSnapshotCapture {
                    at_us: Micros::new(12),
                    angle_x10: Degrees10::new(12),
                    snapshot: BoardSensorSnapshot {
                        rpm: Rpm::new(1200),
                        map_kpa10: Kpa10::new(450),
                        ..BoardSensorSnapshot::default()
                    },
                },
            },
            &mut trace,
        )
        .unwrap_err();

    assert_eq!(
        err,
        ApplyAndRecordError::Record(CommonObservabilityRecordTraceOverflow { capacity: 4 })
    );
    assert_eq!(trace.len(), 4);
    assert_eq!(adapter.capture().count, 1);
    assert_eq!(adapter.runtime().snapshot().engine.rpm, Rpm::new(1200));
    assert_eq!(
        adapter.runtime().snapshot().engine.load_kpa10,
        Kpa10::new(450)
    );
}

#[test]
fn board_adapter_apply_event_and_push_pair_stores_successful_non_tick_event() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let result = adapter
        .apply_event_and_push_pair(
            BoardEvent::CamEdge {
                at_us: Micros::new(7),
                cam_seen: true,
            },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap();

    assert_eq!(result, None);
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        record_trace.get(0).map(|record| record.sample)
    );
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_pair_stores_successful_tick_event_and_returns_step_result() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<4> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    let result = adapter
        .apply_event_and_push_pair(
            BoardEvent::Tick {
                now_us: Micros::new(20),
                control: control_inputs(),
            },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap();

    assert!(result.is_some());
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        record_trace.get(0).map(|record| record.sample)
    );
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_pair_skips_both_traces_when_apply_event_fails() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = FailingCapture;
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    sample_trace.push(sample_before).unwrap();
    record_trace.push(record_before).unwrap();

    let err = adapter
        .apply_event_and_push_pair(
            BoardEvent::SensorSnapshotCapture {
                capture: BoardSensorSnapshotCapture {
                    at_us: Micros::new(12),
                    angle_x10: Degrees10::new(12),
                    snapshot: BoardSensorSnapshot {
                        rpm: Rpm::new(1200),
                        map_kpa10: Kpa10::new(450),
                        ..BoardSensorSnapshot::default()
                    },
                },
            },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    assert!(matches!(
        err,
        ApplyAndPushPairError::Apply(BoardAdapterError::Capture(()))
    ));
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(record_trace.get(0), Some(record_before));
}

#[test]
fn board_adapter_apply_event_and_push_pair_returns_record_overflow_after_successful_apply() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    sample_trace.push(sample_before).unwrap();
    record_trace
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .apply_event_and_push_pair(
            BoardEvent::CamEdge {
                at_us: Micros::new(7),
                cam_seen: true,
            },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    assert_eq!(
        err,
        ApplyAndPushPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(sample_trace.get(0), Some(sample_before));
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_pair_returns_sample_overflow_after_record_is_stored() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut sample_trace: FixedCommonObservabilityTrace<1> = FixedCommonObservabilityTrace::new();
    let mut record_trace: FixedCommonObservabilityRecordTrace<2> =
        FixedCommonObservabilityRecordTrace::new();
    sample_trace
        .push(CommonObservabilitySample::default())
        .unwrap();

    let err = adapter
        .apply_event_and_push_pair(
            BoardEvent::SensorSnapshotCapture {
                capture: BoardSensorSnapshotCapture {
                    at_us: Micros::new(9),
                    angle_x10: Degrees10::new(12),
                    snapshot: BoardSensorSnapshot::default(),
                },
            },
            &mut sample_trace,
            &mut record_trace,
        )
        .unwrap_err();

    assert_eq!(
        err,
        ApplyAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(sample_trace.len(), 1);
    assert_eq!(
        sample_trace.get(0),
        Some(CommonObservabilitySample::default())
    );
    assert_eq!(record_trace.len(), 1);
    assert_eq!(
        record_trace.get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorSnapshotCapture,
            sample: adapter.observability_sample(),
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_to_trace_pair_stores_successful_non_tick_event() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    let result = adapter
        .apply_event_and_push_to_trace_pair(
            BoardEvent::CamEdge {
                at_us: Micros::new(7),
                cam_seen: true,
            },
            &mut traces,
        )
        .unwrap();

    let expected_sample = adapter.observability_sample();

    assert_eq!(result, None);
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(expected_sample));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_to_trace_pair_stores_successful_tick_event_and_returns_step_result(
) {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<4, 4> =
        FixedCommonObservabilityTracePair::new();

    let result = adapter
        .apply_event_and_push_to_trace_pair(
            BoardEvent::Tick {
                now_us: Micros::new(20),
                control: control_inputs(),
            },
            &mut traces,
        )
        .unwrap();

    let expected_sample = adapter.observability_sample();

    assert!(result.is_some());
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.sample().get(0), Some(expected_sample));
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: expected_sample,
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_to_trace_pair_skips_both_traces_when_apply_event_fails() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = FailingCapture;
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    let record_before = CommonObservabilityRecord {
        kind: CommonObservabilityRecordKind::Tick,
        sample: sample_before,
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces.record_mut().push(record_before).unwrap();

    let err = adapter
        .apply_event_and_push_to_trace_pair(
            BoardEvent::SensorSnapshotCapture {
                capture: BoardSensorSnapshotCapture {
                    at_us: Micros::new(12),
                    angle_x10: Degrees10::new(12),
                    snapshot: BoardSensorSnapshot {
                        rpm: Rpm::new(1200),
                        map_kpa10: Kpa10::new(450),
                        ..BoardSensorSnapshot::default()
                    },
                },
            },
            &mut traces,
        )
        .unwrap_err();

    assert!(matches!(
        err,
        ApplyAndPushPairError::Apply(BoardAdapterError::Capture(()))
    ));
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(traces.record().get(0), Some(record_before));
}

#[test]
fn board_adapter_apply_event_and_push_to_trace_pair_returns_record_overflow_after_successful_apply()
{
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();
    traces
        .record_mut()
        .push(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
        .unwrap();

    let err = adapter
        .apply_event_and_push_to_trace_pair(
            BoardEvent::CamEdge {
                at_us: Micros::new(7),
                cam_seen: true,
            },
            &mut traces,
        )
        .unwrap_err();

    assert_eq!(
        err,
        ApplyAndPushPairError::Record(CommonObservabilityRecordTraceOverflow { capacity: 1 })
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::Tick,
            sample: sample_before,
        })
    );
}

#[test]
fn board_adapter_apply_event_and_push_to_trace_pair_returns_sample_overflow_after_record_is_stored()
{
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut traces: FixedCommonObservabilityTracePair<1, 2> =
        FixedCommonObservabilityTracePair::new();
    let sample_before = CommonObservabilitySample {
        at_us: Micros::new(1),
        snapshot: CommonObservabilitySnapshot::default(),
    };
    traces.sample_mut().push(sample_before).unwrap();

    let err = adapter
        .apply_event_and_push_to_trace_pair(
            BoardEvent::SensorSnapshotCapture {
                capture: BoardSensorSnapshotCapture {
                    at_us: Micros::new(9),
                    angle_x10: Degrees10::new(12),
                    snapshot: BoardSensorSnapshot::default(),
                },
            },
            &mut traces,
        )
        .unwrap_err();

    let expected_sample = adapter.observability_sample();

    assert_eq!(
        err,
        ApplyAndPushPairError::Sample(CommonObservabilityTraceOverflow { capacity: 1 })
    );
    assert_eq!(traces.sample().len(), 1);
    assert_eq!(traces.sample().get(0), Some(sample_before));
    assert_eq!(traces.record().len(), 1);
    assert_eq!(
        traces.record().get(0),
        Some(CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::SensorSnapshotCapture,
            sample: expected_sample,
        })
    );
}

fn timed_injection(start_at: u32, end_at: u32) -> TimedInjectionPlan {
    TimedInjectionPlan {
        plan: SchedulerInjectionPlan {
            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(0)),
            pulse_width: PulseWidthUs::new((end_at - start_at) as u16),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}

fn timed_ignition(start_at: u32, end_at: u32) -> TimedIgnitionPlan {
    TimedIgnitionPlan {
        plan: SchedulerIgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(0)),
            dwell: DwellUs::new((end_at - start_at) as u16),
            advance: Degrees10::new(0),
        },
        start_at: Micros::new(start_at),
        end_at: Micros::new(end_at),
    }
}
