use super::*;
use crate::live_inputs::{SplitLiveInputs, SplitLiveTriggerEvent, SplitSyncState};
use crate::outputs::{apply_drained_transitions, RawScheduledOutputPin, ScheduledActionExecutor};
use ecu_board_api::frontier::{TimingIslandPermitMask, TimingIslandStopReason};
use ecu_board_api::{
    BoardSensorSnapshot, BoardSensorSnapshotCapture, BoardSensorValidityFlags,
    CommonActionTelemetry, CommonAfterstartTelemetry, CommonAfterstartWindowMode,
    CommonCamEdgeTelemetry, CommonControlReasonTelemetry, CommonControlTelemetry,
    CommonDecisionTelemetry, CommonDiagnosticsTelemetry, CommonEngineTelemetry,
    CommonEnrichmentTelemetry, CommonFaultTransitionAction, CommonFaultTransitionEventId,
    CommonFaultTransitionEventTelemetry, CommonFaultTransitionTelemetry, CommonFrontierFaultAction,
    CommonFrontierFaultEventId, CommonFrontierFaultTelemetry, CommonFrontierTelemetry,
    CommonFuelObservationTelemetry, CommonFuelStrategyMode, CommonHighRateLogTelemetry,
    CommonIgnitionLimitReason, CommonLambdaActivity, CommonLambdaCorrectionTelemetry,
    CommonLambdaDisableReason, CommonLambdaMode, CommonLambdaTelemetry, CommonLimpActionLevel,
    CommonLimpActionSource, CommonLimpActionTelemetry, CommonPendingInputTelemetry,
    CommonProtectionAction, CommonProtectionLevel, CommonProtectionPersistence,
    CommonProtectionSource, CommonProtectionTelemetry, CommonRuntimeFaultTelemetry,
    CommonSchedulerMode, CommonSchedulerOwnershipTelemetry, CommonSchedulerReservationTelemetry,
    CommonSchedulerStateSummaryTelemetry, CommonSchedulerWindowTelemetry,
    CommonShiftArmingTelemetry, CommonStartupTelemetry, CommonStartupWindowMode,
    CommonSyncTelemetryState, CommonTorqueLimitReason, CommonTorqueTelemetry,
    CommonTransientEnrichmentTelemetry, CommonTriggerEdgeTelemetry, CommonValidatedInputTelemetry,
    CommonWarmupTelemetry, CommonWarmupTemperatureMode, EngineTimeAuthorityTelemetry,
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
use ecu_transport::Message;

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

impl SchedulerObservabilitySource for MockActions {
    fn timing_metrics(&self) -> ScheduledTimingMetrics {
        ScheduledTimingMetrics::default()
    }

    fn active_queue_count(&self) -> u8 {
        0
    }

    fn free_queue_slots(&self) -> u8 {
        0
    }

    fn queue_capacity(&self) -> u8 {
        0
    }

    fn frontier_telemetry(&self) -> CommonFrontierTelemetry {
        CommonFrontierTelemetry::default()
    }

    fn scheduler_ownership_telemetry(&self) -> CommonSchedulerOwnershipTelemetry {
        CommonSchedulerOwnershipTelemetry::default()
    }

    fn scheduler_reservation_telemetry(&self) -> CommonSchedulerReservationTelemetry {
        CommonSchedulerReservationTelemetry::default()
    }

    fn scheduler_state_summary_telemetry(&self) -> CommonSchedulerStateSummaryTelemetry {
        CommonSchedulerStateSummaryTelemetry::default()
    }

    fn scheduler_window_telemetry(&self) -> CommonSchedulerWindowTelemetry {
        CommonSchedulerWindowTelemetry::default()
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

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
struct PreloadedCalibrationStore {
    blob: ecu_calibration::PersistedCalibrationBlob,
    loaded: usize,
    saved: usize,
}

impl PersistedCalibrationStore for PreloadedCalibrationStore {
    type Error = ();

    fn load(&mut self) -> Result<Option<ecu_calibration::PersistedCalibrationBlob>, Self::Error> {
        self.loaded += 1;
        Ok(Some(self.blob))
    }

    fn save(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        self.blob = *blob;
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

#[derive(Default)]
struct FailingPin {
    high_count: u8,
    low_count: u8,
}

impl RawScheduledOutputPin for FailingPin {
    fn set_scheduled_high(&mut self) {
        self.high_count = self.high_count.saturating_add(1);
    }

    fn set_scheduled_low(&mut self) {
        self.low_count = self.low_count.saturating_add(1);
    }

    fn try_set_scheduled_high(&mut self) -> Result<(), crate::outputs::ScheduledOutputPinError> {
        self.high_count = self.high_count.saturating_add(1);
        Err(crate::outputs::ScheduledOutputPinError::SetHigh)
    }

    fn try_set_scheduled_low(&mut self) -> Result<(), crate::outputs::ScheduledOutputPinError> {
        self.low_count = self.low_count.saturating_add(1);
        Err(crate::outputs::ScheduledOutputPinError::SetLow)
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

fn constant_test_fuel_model(pulse_width: PulseWidthUs) -> ecu_runtime::BaseFuelModel {
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
    ecu_runtime::BaseFuelModel::new(rpm_bins, load_bins, [[pulse_width; 16]; 16])
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

fn semantic_ae_freeze_calibration() -> RuntimeSemanticCalibration {
    let mut calibration = semantic_calibration();
    calibration.ae_tps_threshold_curve.values = [1; 16];
    calibration.ae_map_threshold_curve.values = [1; 16];
    calibration.ae_shot_curve_us.values = [250; 16];
    calibration.ae_decay_steps_curve.values = [2; 16];
    calibration.ae_decay_ratio_curve_x1000.values = [1000; 16];
    calibration
}

fn semantic_afterstart_calibration(
    window_cycles: u16,
    correction_x1000: u16,
) -> RuntimeSemanticCalibration {
    let mut calibration = semantic_calibration();
    calibration.afterstart_table.values = [[correction_x1000; 16]; 16];
    calibration.afterstart_window_cycles = window_cycles;
    calibration
}

fn semantic_warmup_calibration(correction_x1000: u16) -> RuntimeSemanticCalibration {
    let mut calibration = semantic_calibration();
    calibration.warmup_curve.values = [correction_x1000; 16];
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
            now_us: Micros::new(1_000),
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(2500)),
        fuel_sensors: ecu_runtime::FuelSensorInputs::default(),
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
                ecu_runtime::LambdaDisableReason::None,
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
                    pw_air_us: None,
                    pw_corr_us: 0,
                    warmup_active: false,
                    warmup_correction_x100: 100,
                    warmup_temperature_mode: ecu_runtime::FuelWarmupTemperatureMode::Inactive,
                    startup_active: false,
                    startup_window_remaining: 0,
                    startup_window_mode: ecu_runtime::FuelStartupWindowMode::Inactive,
                    afterstart_active: false,
                    afterstart_window_remaining: 0,
                    afterstart_window_mode: ecu_runtime::FuelAfterstartWindowMode::Inactive,
                    lambda_ae_freeze_active: false,
                    ae_pulse_us: 0,
                    ae_decay_steps_remaining: 0,
                    lambda_correction_x1000: 1000,
                    lambda_integrator_acc: 0,
                    lambda_integrator_min_acc: 0,
                    lambda_integrator_max_acc: 0,
                    lambda_integrator_frozen: false,
                    idle_active: false,
                    idle_duty_x1000: 0,
                    idle_integrator_acc: 0,
                    idle_integrator_min_acc: 0,
                    idle_integrator_max_acc: 0,
                    idle_integrator_frozen: false,
                    advance_deg10_trim: 0,
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
    let actions = ScheduledActionExecutor::<4>::new();
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
    let actions = ScheduledActionExecutor::<4>::new();
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
fn board_adapter_afterstart_telemetry_tracks_semantic_afterstart_window_state() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_speed_density_semantic(
        semantic_afterstart_calibration(3, 1200),
        RuntimeSemanticState::default(),
    );
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(2500),
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

    assert_eq!(
        adapter.afterstart_telemetry(),
        CommonAfterstartTelemetry {
            active: true,
            remaining_window: 2,
            window_mode: CommonAfterstartWindowMode::Cycles,
        }
    );
    assert_eq!(
        adapter.diagnostics_telemetry().afterstart,
        adapter.afterstart_telemetry()
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("afterstart shell should survive through observability records");
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .afterstart,
        adapter.afterstart_telemetry()
    );
}

#[test]
fn board_adapter_warmup_telemetry_tracks_direct_temperature_state() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(2500),
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
    adapter
        .runtime
        .configure_warmup_enrichment(ecu_runtime::WarmupConfig {
            start_c: 0,
            end_c: 100,
            max_percent_x100: 150,
            min_percent_x100: 100,
        });
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(2500),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let mut cold = control_inputs();
    cold.enrichment.clt_c = -10;
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: cold,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.warmup_telemetry(),
        CommonWarmupTelemetry {
            active: true,
            correction_x100: 150,
            temperature_mode: CommonWarmupTemperatureMode::ColdClamp,
        }
    );
    assert_eq!(
        adapter.diagnostics_telemetry().warmup,
        adapter.warmup_telemetry()
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("warmup shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.warmup,
        adapter.warmup_telemetry()
    );
}

#[test]
fn board_adapter_enrichment_telemetry_uses_semantic_warmup_correction() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(2500),
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
        semantic_warmup_calibration(1200),
        RuntimeSemanticState::default(),
    );
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(2500),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let mut cold = control_inputs();
    cold.enrichment.clt_c = -10;
    let result = adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: cold,
        })
        .unwrap()
        .unwrap();

    assert_eq!(result.control.enrichment.warmup_x100, 135);
    assert_eq!(adapter.warmup_telemetry().correction_x100, 120);
    assert_eq!(adapter.enrichment_telemetry().warmup_x100, 120);
}

#[test]
fn board_adapter_startup_telemetry_tracks_direct_startup_window_state() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(2500),
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
            rpm: Rpm::new(2500),
            angle_x10: Degrees10::new(12),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();

    let mut cranking = control_inputs();
    cranking.enrichment.cranking = true;
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: cranking,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.startup_telemetry(),
        CommonStartupTelemetry {
            active: true,
            remaining_window: 3000,
            window_mode: CommonStartupWindowMode::Milliseconds,
        }
    );
    assert_eq!(
        adapter.diagnostics_telemetry().startup,
        adapter.startup_telemetry()
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("startup shell should survive through observability records");
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .startup,
        adapter.startup_telemetry()
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
    let actions = ScheduledActionExecutor::<4>::new();
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
    assert_eq!(
        adapter.decision_telemetry().fuel_cut_reason,
        CommonCutReason::Launch
    );
    assert_eq!(
        adapter.decision_telemetry().spark_cut_reason,
        CommonCutReason::Launch
    );
}

#[test]
fn board_adapter_cut_reason_telemetry_keeps_channel_specific_provenance() {
    let snapshot = ecu_runtime::RuntimeSnapshot {
        fuel_cut: true,
        spark_cut: true,
        direct_fuel_cut_request: true,
        launch_active: true,
        ..ecu_runtime::RuntimeSnapshot::default()
    };

    assert_eq!(
        common_fuel_cut_reason(&snapshot),
        CommonCutReason::DirectRequest
    );
    assert_eq!(common_spark_cut_reason(&snapshot), CommonCutReason::Launch);
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
    let actions = ScheduledActionExecutor::<4>::new();
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
    assert_eq!(
        adapter.decision_telemetry().fuel_cut_reason,
        CommonCutReason::FlatShift
    );
    assert_eq!(
        adapter.decision_telemetry().spark_cut_reason,
        CommonCutReason::FlatShift
    );
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
    assert_ne!(identity.checksum.get(), 0);
}

#[test]
fn board_adapter_obd2_vehicle_identity_uses_calibration_package_identity() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    let vehicle_identity =
        crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter);

    assert_eq!(
        vehicle_identity,
        crate::transport_service::Obd2VehicleIdentity::from_calibration_identity(
            adapter.calibration_package_identity()
        )
    );
    assert_eq!(&vehicle_identity.vin[0..9], b"C00000000");
    assert_ne!(&vehicle_identity.vin[9..17], b"00000000");
    assert_ne!(
        vehicle_identity.vin,
        crate::transport_service::Obd2VehicleIdentity::from_signature(ecu_ts::TS_SIGNATURE).vin
    );
}

#[test]
fn board_adapter_obd2_vehicle_identity_prefers_provisioned_identity() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.set_provisioned_obd2_identity(
        crate::transport_service::ProvisionedObd2Identity::from_ascii(
            Some(b"trackvin000000001"),
            Some(b"trackcal000000001"),
            Some(b"m50-a1"),
        ),
    );

    let vehicle_identity =
        crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter);

    assert_eq!(&vehicle_identity.vin, b"TRACKVIN000000001");
    assert_eq!(&vehicle_identity.calibration_id, b"TRACKCAL000000001");
    assert_eq!(vehicle_identity.ecu_name, [b'M', b'5', b'0', b'A', b'1', 0]);
    assert_ne!(
        vehicle_identity.vin,
        crate::transport_service::Obd2VehicleIdentity::from_calibration_identity(
            adapter.calibration_package_identity()
        )
        .vin
    );
}

#[test]
fn board_adapter_installs_provisioned_obd2_identity_record() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.install_provisioned_obd2_identity_record(
        crate::transport_service::Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"trackvin000000002"),
            Some(b"trackcal000000002"),
            Some(b"m50-b2"),
        ),
    );

    let vehicle_identity =
        crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter);

    assert_eq!(&vehicle_identity.vin, b"TRACKVIN000000002");
    assert_eq!(&vehicle_identity.calibration_id, b"TRACKCAL000000002");
    assert_eq!(vehicle_identity.ecu_name, [b'M', b'5', b'0', b'B', b'2', 0]);
}

#[test]
fn board_adapter_installed_obd2_identity_record_reaches_mode09_dispatch() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.install_provisioned_obd2_identity_record(
        crate::transport_service::Obd2ProvisionedIdentityRecord::from_ascii(
            Some(b"trackvin000000004"),
            Some(b"trackcal000000004"),
            Some(b"m50-d4"),
        ),
    );

    let identity = crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter);
    let history = crate::transport_service::LiveObd2RequestOwner::obd2_retained_history(&adapter);
    let vin_request = ecu_transport::Message::Obd2Request {
        service: 0x09,
        parameter_id: Some(0x02),
        payload_len: 0,
        payload: [0; 6],
    };
    let calibration_id_request = ecu_transport::Message::Obd2Request {
        service: 0x09,
        parameter_id: Some(0x04),
        payload_len: 0,
        payload: [0; 6],
    };

    let vin = ecu_transport::CanObd2MultiServiceDispatchSurface::dispatch_outcome(
        &vin_request,
        history.dispatch_inputs_with_vehicle_identity(identity),
    )
    .expect("provisioned VIN dispatch should succeed");
    let calibration_id = ecu_transport::CanObd2MultiServiceDispatchSurface::dispatch_outcome(
        &calibration_id_request,
        history.dispatch_inputs_with_vehicle_identity(identity),
    )
    .expect("provisioned calibration ID dispatch should succeed");

    match vin {
        ecu_transport::CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo(dispatch) => {
            assert_eq!(dispatch.info_type_id, 0x02);
            assert_eq!(dispatch.total_payload_len, 17);
            assert_eq!(
                dispatch.segments[0].as_ref(),
                Some(&ecu_transport::CanObd2SegmentedResponseFrame {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 0,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"TRACKV",
                })
            );
        }
        other => panic!("unexpected VIN dispatch: {other:?}"),
    }
    match calibration_id {
        ecu_transport::CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo(dispatch) => {
            assert_eq!(dispatch.info_type_id, 0x04);
            assert_eq!(dispatch.total_payload_len, 17);
            assert_eq!(
                dispatch.segments[0].as_ref(),
                Some(&ecu_transport::CanObd2SegmentedResponseFrame {
                    service: 0x49,
                    parameter_id: Some(0x04),
                    sequence_index: 0,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"TRACKC",
                })
            );
        }
        other => panic!("unexpected calibration ID dispatch: {other:?}"),
    }
}

#[test]
fn board_adapter_flash_write_fault_status_reaches_mode09_dispatch() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.set_obd2_flash_write_fault_status(Some(ecu_transport::CanObd2FlashWriteFaultStatus {
        present: true,
        phase: ecu_transport::CanObd2FlashWriteFaultPhase::Program,
        sr_bits: 0x0000_00F2,
    }));

    let history = crate::transport_service::LiveObd2RequestOwner::obd2_retained_history(&adapter);
    let request = ecu_transport::Message::Obd2Request {
        service: 0x09,
        parameter_id: Some(ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID),
        payload_len: 0,
        payload: [0; 6],
    };

    let outcome = ecu_transport::CanObd2MultiServiceDispatchSurface::dispatch_outcome(
        &request,
        history.dispatch_inputs_with_vehicle_identity_and_statuses(
            crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter),
            crate::transport_service::LiveObd2RequestOwner::obd2_identity_key_lifecycle_status(
                &adapter,
            ),
            crate::transport_service::LiveObd2RequestOwner::obd2_flash_write_fault_status(&adapter),
        ),
    )
    .expect("flash fault dispatch should succeed");

    match outcome {
        ecu_transport::CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo(dispatch) => {
            assert_eq!(
                dispatch.info_type_id,
                ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID
            );
            assert_eq!(
                dispatch.segments[0].as_ref(),
                Some(&ecu_transport::CanObd2SegmentedResponseFrame {
                    service: 0x49,
                    parameter_id: Some(ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID),
                    sequence_index: 0,
                    segment_count: 1,
                    total_payload_len: ecu_transport::CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN,
                    segment_len: 6,
                    segment: [0x80, 0x03, 0x00, 0x00, 0x00, 0xF2],
                })
            );
        }
        other => panic!("unexpected flash fault dispatch: {other:?}"),
    }
}

#[test]
fn board_adapter_empty_provisioned_obd2_identity_record_preserves_fallback() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let fallback = crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter);

    adapter.install_provisioned_obd2_identity_record(
        crate::transport_service::Obd2ProvisionedIdentityRecord::default(),
    );

    assert_eq!(
        crate::transport_service::LiveObd2RequestOwner::obd2_vehicle_identity(&adapter),
        fallback
    );
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
    let expected = CalibrationPackageIdentity::from_snapshot_with_staged_dirty(
        adapter.runtime().calibration_snapshot(),
        true,
    );

    assert_eq!(identity, expected);
    assert_ne!(
        identity.checksum,
        CalibrationPackageIdentity::from_snapshot(adapter.runtime().calibration_snapshot())
            .checksum
    );
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
    assert_eq!(
        adapter.observability_snapshot().limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::OutputSuppressed,
            source: CommonLimpActionSource::SyncAuthority,
            cancel_scheduler: true,
            cancel_reason: CancelReason::SyncLoss,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );

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
    assert_eq!(
        adapter.observability_snapshot().limp_action,
        CommonLimpActionTelemetry::default()
    );
}

#[test]
fn board_adapter_clear_diagnostics_clears_runtime_fault_surface() {
    let sample = CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    };
    let mut adapter = BoardAdapter::new(
        MockSensor(sample),
        MockCapture::default(),
        MockActions::default(),
        MockWatchdog::default(),
        MockTransport::default(),
        MockStore::default(),
    );
    adapter.runtime.set_fault_state(
        FaultCode::SafetyCut,
        FaultSeverity::Critical,
        CancelReason::SafetyShutdown,
    );

    let summary = adapter.clear_diagnostics();

    assert_eq!(
        summary,
        ecu_domain::diag::DiagClearSummary {
            cleared_active_count: 1,
            cleared_log_entries: 0,
            emergency_cleared: false,
        }
    );
    assert_eq!(adapter.fault_state(), FaultState::default());
}

#[test]
fn board_adapter_clear_diagnostics_clears_shared_diag_log() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(65),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    adapter.runtime.set_fault_state(
        FaultCode::SensorOutOfRange,
        FaultSeverity::Warning,
        CancelReason::Manual,
    );
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(55),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert!(adapter
        .diag_log()
        .events
        .iter()
        .any(|entry| entry.is_some()));

    let summary = adapter.clear_diagnostics();

    assert_eq!(summary.cleared_log_entries, 1);
    assert!(adapter
        .diag_log()
        .events
        .iter()
        .all(|entry| entry.is_none()));
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
    assert_eq!(
        adapter.observability_snapshot().limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: true,
            cancel_reason: CancelReason::SafetyShutdown,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
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
            event: CommonFaultTransitionEventTelemetry {
                event_id: CommonFaultTransitionEventId::FaultEntered,
                severity: FaultSeverity::Warning,
                action: CommonFaultTransitionAction::LimpHome,
            },
            previous_fault: FaultCode::None,
            previous_severity: FaultSeverity::Info,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::SensorOutOfRange,
            current_severity: FaultSeverity::Warning,
            current_cancel_reason: CancelReason::Manual,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().fault_transition,
        adapter.fault_transition_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().fault,
        CommonRuntimeFaultTelemetry {
            active: true,
            fault_code: FaultCode::SensorOutOfRange,
            severity: FaultSeverity::Warning,
            cancel_reason: CancelReason::Manual,
            action: CommonFaultTransitionAction::LimpHome,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::Degraded,
            source: CommonProtectionSource::RuntimeFault,
            action: CommonProtectionAction::LimpHome,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::AuxOnly,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: true,
            aux_command_count: 1,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("fault-enter record stores current fault shell");
    assert_eq!(
        trace.get(0).expect("entered record").sample.snapshot.fault,
        CommonRuntimeFaultTelemetry {
            active: true,
            fault_code: FaultCode::SensorOutOfRange,
            severity: FaultSeverity::Warning,
            cancel_reason: CancelReason::Manual,
            action: CommonFaultTransitionAction::LimpHome,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("entered record")
            .sample
            .snapshot
            .protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::Degraded,
            source: CommonProtectionSource::RuntimeFault,
            action: CommonProtectionAction::LimpHome,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("entered record")
            .sample
            .snapshot
            .limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::AuxOnly,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: true,
            aux_command_count: 1,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("entered record")
            .sample
            .snapshot
            .high_rate_log
            .fault,
        CommonRuntimeFaultTelemetry {
            active: true,
            fault_code: FaultCode::SensorOutOfRange,
            severity: FaultSeverity::Warning,
            cancel_reason: CancelReason::Manual,
            action: CommonFaultTransitionAction::LimpHome,
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
            event: CommonFaultTransitionEventTelemetry {
                event_id: CommonFaultTransitionEventId::FaultUpdated,
                severity: FaultSeverity::Critical,
                action: CommonFaultTransitionAction::Shutdown,
            },
            previous_fault: FaultCode::SensorOutOfRange,
            previous_severity: FaultSeverity::Warning,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::SafetyCut,
            current_severity: FaultSeverity::Critical,
            current_cancel_reason: CancelReason::SafetyShutdown,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().fault_transition,
        adapter.fault_transition_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().fault,
        CommonRuntimeFaultTelemetry {
            active: true,
            fault_code: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
            action: CommonFaultTransitionAction::Shutdown,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::ShutdownDriving,
            source: CommonProtectionSource::RuntimeFault,
            action: CommonProtectionAction::Shutdown,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: true,
            cancel_reason: CancelReason::SafetyShutdown,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("fault-update record stores current fault shell");
    assert_eq!(
        trace.get(0).expect("updated record").sample.snapshot.fault,
        CommonRuntimeFaultTelemetry {
            active: true,
            fault_code: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
            action: CommonFaultTransitionAction::Shutdown,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("updated record")
            .sample
            .snapshot
            .limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: true,
            cancel_reason: CancelReason::SafetyShutdown,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("updated record")
            .sample
            .snapshot
            .protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::ShutdownDriving,
            source: CommonProtectionSource::RuntimeFault,
            action: CommonProtectionAction::Shutdown,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("updated record")
            .sample
            .snapshot
            .high_rate_log
            .fault,
        CommonRuntimeFaultTelemetry {
            active: true,
            fault_code: FaultCode::SafetyCut,
            severity: FaultSeverity::Critical,
            cancel_reason: CancelReason::SafetyShutdown,
            action: CommonFaultTransitionAction::Shutdown,
        }
    );
}

#[test]
fn board_adapter_fault_transition_telemetry_records_clear_transition() {
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
            now_us: Micros::new(70),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    adapter
        .runtime
        .set_fault_state(FaultCode::None, FaultSeverity::Info, CancelReason::Manual);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(75),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.fault_transition_telemetry(),
        CommonFaultTransitionTelemetry {
            changed: true,
            at_us: Micros::new(75),
            event: CommonFaultTransitionEventTelemetry {
                event_id: CommonFaultTransitionEventId::FaultCleared,
                severity: FaultSeverity::Info,
                action: CommonFaultTransitionAction::Cleared,
            },
            previous_fault: FaultCode::SensorOutOfRange,
            previous_severity: FaultSeverity::Warning,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::None,
            current_severity: FaultSeverity::Info,
            current_cancel_reason: CancelReason::Manual,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().fault_transition,
        adapter.fault_transition_telemetry()
    );
    assert_eq!(
        adapter.observability_snapshot().fault,
        CommonRuntimeFaultTelemetry::default()
    );
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("fault-clear record stores cleared fault shell");
    assert_eq!(
        trace.get(0).expect("cleared record").sample.snapshot.fault,
        CommonRuntimeFaultTelemetry::default()
    );
    assert_eq!(
        trace
            .get(0)
            .expect("cleared record")
            .sample
            .snapshot
            .high_rate_log
            .fault,
        CommonRuntimeFaultTelemetry::default()
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
fn board_adapter_live_obd2_owner_projects_current_sensor_data() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(777),
                angle_x10: Degrees10::new(120),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(2_300),
                    map_kpa10: Kpa10::new(880),
                    tps_x100: 4_200,
                    clt_c10: 850,
                    iat_c10: 300,
                    vbatt_mv: 13_500,
                    lambda_x100: Lambda100::new(101),
                    validity: BoardSensorValidityFlags::from_channels(false, false, false, true),
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();

    assert_eq!(
        crate::transport_service::LiveObd2RequestOwner::current_obd2_sensor_data(&adapter),
        Message::SensorData {
            map_kpa_x10: 880,
            tps_percent: 42,
            iat_offset: 70,
            clt_offset: 125,
            voltage_x10: 135,
            lambda_x100: 101,
            flags: 0,
            timestamp_us: 777,
        }
    );
}

#[test]
fn board_adapter_live_obd2_owner_projects_recent_fault_transition_event() {
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
            now_us: Micros::new(70),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let events = crate::transport_service::LiveObd2RequestOwner::recent_obd2_diag_events(&adapter);
    let event = events[0].expect("fault-enter transition should project one recent diag event");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::MapRange);
    assert_eq!(event.timestamp, Micros::new(70));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(0));
    assert_eq!(event.start_us, 70);
    assert_eq!(event.end_us, 70);
    assert!(events[1].is_none());
}

#[test]
fn board_adapter_live_obd2_owner_exposes_shared_diag_log_entries() {
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
            now_us: Micros::new(80),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("diag log should retain recent fault transition");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::MapRange);
    assert_eq!(event.timestamp, Micros::new(80));
}

#[test]
fn board_adapter_push_live_diag_event_exposes_persist_crc_fault() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.push_live_diag_event(ecu_domain::diag::DiagEvent {
        code: ecu_domain::diag::DiagCode::PersistCrcFault,
        timestamp: Micros::new(0),
        source: ecu_domain::diag::DiagSource::User,
        context: None,
        start_us: 0,
        end_us: 0,
    });

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("explicit live diag event should populate shared log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::PersistCrcFault);
    assert_eq!(event.timestamp, Micros::new(0));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::User);
    assert_eq!(
        adapter.retained_obd2_history.stored_dtcs(),
        &[ecu_domain::diag::DiagCode::PersistCrcFault],
    );
    assert_eq!(
        adapter.retained_obd2_history.freeze_frame_dtc(),
        Some(ecu_domain::diag::DiagCode::PersistCrcFault),
    );
}

#[test]
fn board_adapter_record_store_integrity_status_latches_persist_crc_fault_once() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.record_store_integrity_status(crate::kv::ab::StoreIntegrityStatus::Valid);
    adapter.record_store_integrity_status(
        crate::kv::ab::StoreIntegrityStatus::ValidWithCorruptSibling,
    );
    adapter.record_store_integrity_status(crate::kv::ab::StoreIntegrityStatus::Corrupt);

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("integrity corruption should populate shared log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::PersistCrcFault);
    assert!(diag_log[1].is_none());
    assert_eq!(
        adapter.retained_obd2_history.stored_dtcs(),
        &[ecu_domain::diag::DiagCode::PersistCrcFault],
    );
    assert_eq!(
        adapter.retained_obd2_history.freeze_frame_dtc(),
        Some(ecu_domain::diag::DiagCode::PersistCrcFault),
    );
}

#[test]
fn board_adapter_can_restore_retained_obd2_history_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut source = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    source.push_live_diag_event(ecu_domain::diag::DiagEvent {
        code: ecu_domain::diag::DiagCode::PersistCrcFault,
        timestamp: Micros::new(11),
        source: ecu_domain::diag::DiagSource::User,
        context: Some(7),
        start_us: 11,
        end_us: 11,
    });
    let snapshot = source.obd2_retained_history_snapshot();

    let mut restored = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    restored.restore_obd2_retained_history(&snapshot);

    assert_eq!(
        crate::transport_service::LiveObd2RequestOwner::obd2_retained_history(&restored)
            .stored_dtcs(),
        &[ecu_domain::diag::DiagCode::PersistCrcFault],
    );
    assert_eq!(
        crate::transport_service::LiveObd2RequestOwner::obd2_retained_history(&restored)
            .freeze_frame_dtc(),
        Some(ecu_domain::diag::DiagCode::PersistCrcFault),
    );
    assert_eq!(restored.obd2_retained_history_snapshot(), snapshot);
}

#[test]
fn board_adapter_retained_obd2_history_restore_does_not_rearm_frontier() {
    let mut source = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        MockStore::default(),
    );
    source.push_live_diag_event(ecu_domain::diag::DiagEvent {
        code: ecu_domain::diag::DiagCode::PersistCrcFault,
        timestamp: Micros::new(11),
        source: ecu_domain::diag::DiagSource::User,
        context: Some(7),
        start_us: 11,
        end_us: 11,
    });
    let retained_snapshot = source.obd2_retained_history_snapshot();

    let mut restored = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        MockStore::default(),
    );
    restored.restore_obd2_retained_history(&retained_snapshot);

    assert_eq!(restored.obd2_retained_history_snapshot(), retained_snapshot);
    assert_eq!(restored.actions().queue().active_count(), 0);
    assert_eq!(restored.frontier_telemetry().active_horizon_id, None);
    assert_eq!(
        restored.frontier_telemetry().active_permit_mask,
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        restored.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::None
    );

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        restored.actions().drain_and_apply_due(
            Micros::new(1_000),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Ok(0)
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn board_adapter_preloaded_calibration_store_does_not_rearm_frontier_on_restart() {
    let mut adapter = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        PreloadedCalibrationStore::default(),
    );

    assert_eq!(adapter.store().loaded, 0);
    assert_eq!(adapter.actions().queue().active_count(), 0);
    assert_eq!(adapter.frontier_telemetry().active_horizon_id, None);
    assert_eq!(
        adapter.frontier_telemetry().active_permit_mask,
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        adapter.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::None
    );

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        adapter.actions().drain_and_apply_due(
            Micros::new(1_000),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Ok(0)
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn board_adapter_persist_calibration_action_does_not_rearm_frontier() {
    let mut adapter = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        PreloadedCalibrationStore::default(),
    );
    let mut actions = ActionBatch::<RUNTIME_ACTION_CAP>::new();
    assert!(actions.push(Action::PersistCalibration));
    let result = synthetic_step_result(actions);

    adapter.execute_step(&result).unwrap();

    assert_eq!(adapter.store().saved, 1);
    assert_eq!(adapter.actions().queue().active_count(), 0);
    assert_eq!(adapter.frontier_telemetry().active_horizon_id, None);
    assert_eq!(
        adapter.frontier_telemetry().active_permit_mask,
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        adapter.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::None
    );

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        adapter.actions().drain_and_apply_due(
            Micros::new(1_000),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Ok(0)
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn board_adapter_live_obd2_owner_logs_map_recovery_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(90),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1_400),
                    map_kpa10: Kpa10::new(50),
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(100),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(190),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1_400),
                    map_kpa10: Kpa10::new(150),
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(200),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(3_000_200),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared MAP recovery should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::MapRange);
    assert_eq!(event.timestamp, Micros::new(3_000_200));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(150));
    assert_eq!(event.start_us, 100);
    assert_eq!(event.end_us, 3_000_200);
}

#[test]
fn board_adapter_live_obd2_owner_logs_low_voltage_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(90),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1_400),
                    map_kpa10: Kpa10::new(650),
                    vbatt_mv: 7_900,
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(95),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared low-voltage source should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::LowVoltage);
    assert_eq!(event.timestamp, Micros::new(95));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(7_900));
    assert_eq!(event.start_us, 95);
    assert_eq!(event.end_us, 0);
}

#[test]
fn board_adapter_live_obd2_owner_logs_overvoltage_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(90),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1_400),
                    map_kpa10: Kpa10::new(650),
                    vbatt_mv: 16_501,
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(95),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared overvoltage source should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::Overvoltage);
    assert_eq!(event.timestamp, Micros::new(95));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(16_501));
    assert_eq!(event.start_us, 95);
    assert_eq!(event.end_us, 0);

    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(100),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();
    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    assert!(
        diag_log[1].is_none(),
        "latched overvoltage should not spam repeated tick events"
    );

    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(105),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1_400),
                    map_kpa10: Kpa10::new(650),
                    vbatt_mv: 16_500,
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(110),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(115),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(1_400),
                    map_kpa10: Kpa10::new(650),
                    vbatt_mv: 16_501,
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(120),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[1].expect("re-entered overvoltage should log a new event");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::Overvoltage);
    assert_eq!(event.timestamp, Micros::new(120));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(16_501));
    assert_eq!(event.start_us, 120);
    assert_eq!(event.end_us, 0);
}

#[test]
fn board_adapter_live_obd2_owner_logs_knock_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    let mut calibration = semantic_calibration();
    calibration.knock_threshold_x100 = 500;
    adapter.configure_speed_density_semantic(calibration, RuntimeSemanticState::default());
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(100),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(2_600),
                    map_kpa10: Kpa10::new(920),
                    knock_x100: ecu_domain::KnockLevelX100::new(600),
                    validity: BoardSensorValidityFlags::from_channels(false, true, false, false),
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(105),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared knock source should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::KnockDetected);
    assert_eq!(event.timestamp, Micros::new(105));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(600));
    assert_eq!(event.start_us, 105);
    assert_eq!(event.end_us, 0);
}

#[test]
fn board_adapter_live_obd2_owner_logs_low_oil_pressure_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::PressureSnapshot {
            now_us: Micros::new(105),
            oil_pressure_kpa10: Kpa10::new(900),
            fuel_pressure_kpa10: Kpa10::new(3_000),
            oil_valid: true,
            fuel_valid: true,
        })
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared oil-pressure source should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::OilPressureLow);
    assert_eq!(event.timestamp, Micros::new(105));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(900));
    assert_eq!(event.start_us, 105);
    assert_eq!(event.end_us, 0);
}

#[test]
fn board_adapter_live_obd2_owner_logs_low_fuel_pressure_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::PressureSnapshot {
            now_us: Micros::new(105),
            oil_pressure_kpa10: Kpa10::new(1_500),
            fuel_pressure_kpa10: Kpa10::new(2_400),
            oil_valid: true,
            fuel_valid: true,
        })
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared fuel-pressure source should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::FuelPressureLow);
    assert_eq!(event.timestamp, Micros::new(105));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(2_400));
    assert_eq!(event.start_us, 105);
    assert_eq!(event.end_us, 0);
}

#[test]
fn board_adapter_live_obd2_owner_logs_invalid_lambda_from_shared_sensor_snapshot() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = MockActions::default();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::SensorSnapshotCapture {
            capture: BoardSensorSnapshotCapture {
                at_us: Micros::new(100),
                angle_x10: Degrees10::new(30),
                snapshot: BoardSensorSnapshot {
                    rpm: Rpm::new(2_600),
                    map_kpa10: Kpa10::new(920),
                    lambda_x100: Lambda100::new(85),
                    validity: BoardSensorValidityFlags::new(0),
                    ..BoardSensorSnapshot::default()
                },
            },
        })
        .unwrap();
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(105),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    let diag_log = crate::transport_service::LiveObd2RequestOwner::obd2_diag_log_events(&adapter);
    let event = diag_log[0].expect("shared lambda-validity source should populate diag log");
    assert_eq!(event.code, ecu_domain::diag::DiagCode::LambdaInvalid);
    assert_eq!(event.timestamp, Micros::new(105));
    assert_eq!(event.source, ecu_domain::diag::DiagSource::Sensor);
    assert_eq!(event.context, Some(85));
    assert_eq!(event.start_us, 105);
    assert_eq!(event.end_us, 0);
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
            rpm: Rpm::new(900),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::GeometryOnly,
                880,
                0,
            ),
            synced: true,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::SyncSuspect);
    assert_eq!(
        adapter.diagnostics_telemetry().sync_state,
        CommonSyncTelemetryState::SyncSuspect
    );

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(204),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: primary_locked_authority(),
            synced: true,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::CrankSynced);

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(205),
            rpm: Rpm::new(1_000),
            angle_x10: Degrees10::new(0),
            authority: EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CamObserved720,
                AbsoluteTimeAuthority::GeometryOnly,
                900,
                0,
            ),
            synced: true,
        })
        .unwrap();
    assert_eq!(adapter.sync_state(), SplitSyncState::CamSynced);
    assert_eq!(
        adapter.diagnostics_telemetry().sync_state,
        CommonSyncTelemetryState::CamSynced
    );

    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(206),
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
            at_us: Micros::new(207),
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
            at_us: Micros::new(208),
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
            fault: CommonRuntimeFaultTelemetry::default(),
            lambda: CommonLambdaTelemetry::default(),
            lambda_correction: CommonLambdaCorrectionTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            protection: CommonProtectionTelemetry::default(),
            limp_action: CommonLimpActionTelemetry::default(),
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
            fault: CommonRuntimeFaultTelemetry::default(),
            lambda: CommonLambdaTelemetry::default(),
            lambda_correction: CommonLambdaCorrectionTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            protection: CommonProtectionTelemetry::default(),
            limp_action: CommonLimpActionTelemetry::default(),
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
    assert_eq!(diagnostics.fault, CommonRuntimeFaultTelemetry::default());
    assert_eq!(diagnostics.protection, CommonProtectionTelemetry::default());
    assert_eq!(
        diagnostics.limp_action,
        CommonLimpActionTelemetry::default()
    );
    assert_eq!(diagnostics.queue_high_water_mark, 4);
    assert_eq!(diagnostics.last_drain_count, 4);
    assert_eq!(diagnostics.active_queue_count, 0);
    assert_eq!(diagnostics.free_queue_slots, 4);
    assert_eq!(diagnostics.queue_capacity, 4);
}

#[test]
fn board_adapter_observability_record_captures_sustained_delayed_drain_metrics() {
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
    let first = adapter.diagnostics_telemetry();
    assert_eq!(first.late_event_count, 4);
    assert_eq!(first.max_lateness_us, 9_900);
    assert_eq!(first.queue_high_water_mark, 4);
    assert_eq!(first.last_drain_count, 4);
    assert_eq!(first.active_queue_count, 0);

    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(20_000, 20_020),
            ignition: timed_ignition(20_100, 20_130),
        })
        .unwrap();
    assert_eq!(
        adapter
            .actions()
            .drain_due(Micros::new(31_000), &mut drained),
        4
    );

    let diagnostics = adapter.diagnostics_telemetry();
    assert_eq!(diagnostics.late_event_count, 8);
    assert_eq!(diagnostics.max_lateness_us, 11_000);
    assert_eq!(diagnostics.queue_high_water_mark, 4);
    assert_eq!(diagnostics.last_drain_count, 4);
    assert_eq!(diagnostics.active_queue_count, 0);
    assert_eq!(diagnostics.free_queue_slots, 4);
    assert_eq!(diagnostics.queue_capacity, 4);
    assert_eq!(
        adapter.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::None
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("record trace stores sustained metrics snapshot");
    let record = trace.get(0).expect("record present");
    assert_eq!(record.sample.snapshot.diagnostics, diagnostics);
    assert_eq!(
        record.sample.snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::None
    );
}

#[test]
fn board_adapter_observability_record_captures_late_drain_metrics_and_board_output_fault() {
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
        .expect("scheduler action queues");
    assert!(adapter.actions().commit_frontier_horizon(
        38,
        Micros::new(100),
        Micros::new(10_100),
        Micros::new(10_050),
        TimingIslandPermitMask::ALL,
    ));

    let mut inj0 = FailingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        adapter.actions().drain_and_apply_due(
            Micros::new(10_000),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Err(crate::outputs::TransitionApplyError::PinWrite {
            kind: ecu_scheduler::ScheduledTransitionKind::Injector,
            channel: ChannelId::new(0),
            level: ecu_scheduler::ScheduledLevel::High,
            error: crate::outputs::ScheduledOutputPinError::SetHigh,
        })
    );

    let diagnostics = adapter.diagnostics_telemetry();
    assert_eq!(diagnostics.late_event_count, 4);
    assert_eq!(diagnostics.max_lateness_us, 9_900);
    let snapshot = adapter.observability_snapshot();
    assert_eq!(
        snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::BoardOutputFault
    );
    assert_eq!(
        snapshot.frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
    assert_eq!(
        snapshot.protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::ShutdownDriving,
            source: CommonProtectionSource::FrontierFault,
            action: CommonProtectionAction::SafeStateTransition,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );
    assert_eq!(
        snapshot.limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: CommonLimpActionSource::FrontierFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("record trace stores post-fault observation");
    let record = trace.get(0).expect("record present");
    assert_eq!(record.sample.snapshot.diagnostics, diagnostics);
    assert_eq!(
        record.sample.snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::BoardOutputFault
    );
    assert_eq!(
        record.sample.snapshot.frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
    assert_eq!(
        record.sample.snapshot.protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::ShutdownDriving,
            source: CommonProtectionSource::FrontierFault,
            action: CommonProtectionAction::SafeStateTransition,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );
    assert_eq!(
        record.sample.snapshot.limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: CommonLimpActionSource::FrontierFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );
    assert_eq!(
        record.sample.snapshot.high_rate_log,
        CommonHighRateLogTelemetry {
            decision: adapter.decision_telemetry(),
            fault: adapter.runtime_fault_telemetry(),
            frontier_fault: CommonFrontierFaultTelemetry {
                event_id: CommonFrontierFaultEventId::BoardOutputFault,
                severity: FaultSeverity::Critical,
                action: CommonFrontierFaultAction::SafeStateTransition,
            },
            late_event_count: diagnostics.late_event_count,
            max_lateness_us: diagnostics.max_lateness_us,
            calibration_checksum: adapter.calibration_package_identity().checksum.get(),
        }
    );
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
            fault: CommonRuntimeFaultTelemetry {
                active: true,
                fault_code: FaultCode::SafetyCut,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::SafetyShutdown,
                action: CommonFaultTransitionAction::Shutdown,
            },
            lambda: CommonLambdaTelemetry::default(),
            lambda_correction: CommonLambdaCorrectionTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            protection: CommonProtectionTelemetry {
                level: CommonProtectionLevel::ShutdownDriving,
                source: CommonProtectionSource::RuntimeFault,
                action: CommonProtectionAction::Shutdown,
                persistence: CommonProtectionPersistence::LatchedUntilClear,
            },
            limp_action: CommonLimpActionTelemetry::default(),
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
            fuel_cut_reason: CommonCutReason::None,
            spark_cut_reason: CommonCutReason::None,
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
            spark_cut: snapshot.spark_cut,
            fuel_cut_reason: CommonCutReason::None,
            spark_cut_reason: CommonCutReason::None,
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
        lambda_disable_reason: match result.control.lambda.disable_reason {
            ecu_runtime::LambdaDisableReason::None => CommonLambdaDisableReason::None,
            ecu_runtime::LambdaDisableReason::RequestedOpenLoop => {
                CommonLambdaDisableReason::RequestedOpenLoop
            }
            ecu_runtime::LambdaDisableReason::SensorInvalid => {
                CommonLambdaDisableReason::SensorInvalid
            }
            ecu_runtime::LambdaDisableReason::WarmupGate => CommonLambdaDisableReason::WarmupGate,
            ecu_runtime::LambdaDisableReason::LowLoadGate => CommonLambdaDisableReason::LowLoadGate,
            ecu_runtime::LambdaDisableReason::StartupDelay => {
                CommonLambdaDisableReason::StartupDelay
            }
            ecu_runtime::LambdaDisableReason::PowerReductionCut => {
                CommonLambdaDisableReason::PowerReductionCut
            }
            ecu_runtime::LambdaDisableReason::AccelerationEnrichment => {
                CommonLambdaDisableReason::AccelerationEnrichment
            }
        },
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
fn board_adapter_lambda_telemetry_tracks_active_closed_loop_state() {
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
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Active,
            reason: CommonLambdaDisableReason::None,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );
    assert_eq!(
        adapter.diagnostics_telemetry().lambda,
        adapter.lambda_telemetry()
    );
    assert_eq!(
        adapter.diagnostics_telemetry().lambda_correction,
        adapter.lambda_correction_telemetry()
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("active lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Active,
            reason: CommonLambdaDisableReason::None,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_lambda_telemetry_tracks_requested_open_loop_freeze_state() {
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

    let mut inputs = control_inputs();
    inputs.lambda.requested_open_loop = true;

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: inputs,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::RequestedOpenLoop,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("requested-open-loop lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::RequestedOpenLoop,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_lambda_telemetry_tracks_sensor_invalid_state() {
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

    let mut inputs = control_inputs();
    inputs.lambda.lambda_valid = false;

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: inputs,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::SensorInvalid,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("invalid-lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::SensorInvalid,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_lambda_telemetry_tracks_warmup_gate_state() {
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

    let mut inputs = control_inputs();
    inputs.lambda.clt_c = 20;

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: inputs,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::WarmupGate,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("warmup-gated lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::WarmupGate,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_lambda_telemetry_tracks_low_load_gate_state() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(1200),
        load_kpa10: Kpa10::new(200),
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

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::LowLoadGate,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("low-load lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::LowLoadGate,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_lambda_telemetry_tracks_startup_delay_state() {
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

    let mut inputs = control_inputs();
    inputs.lambda.now_us = Micros::new(20);
    inputs.lambda.just_started = true;

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter
        .runtime
        .configure_lambda_trim(ecu_runtime::LambdaTrimConfig {
            startup_delay_us: Micros::new(2_000_000),
            ..ecu_runtime::LambdaTrimConfig::DEFAULT
        });
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control: inputs,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::StartupDelay,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("startup-delay lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Inactive,
            reason: CommonLambdaDisableReason::StartupDelay,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_lambda_telemetry_tracks_power_reduction_cut_freeze_state() {
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

    for (fuel_cut, spark_cut) in [(true, false), (false, true)] {
        let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
        adapter.runtime.set_direct_cut_requests(fuel_cut, spark_cut);
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
            .apply_event(BoardEvent::Tick {
                now_us: Micros::new(20),
                control: control_inputs(),
            })
            .unwrap()
            .unwrap();

        assert_eq!(
            adapter.lambda_telemetry(),
            CommonLambdaTelemetry {
                activity: CommonLambdaActivity::Frozen,
                reason: CommonLambdaDisableReason::PowerReductionCut,
            }
        );
        assert_eq!(
            adapter.lambda_correction_telemetry(),
            CommonLambdaCorrectionTelemetry {
                measured_lambda: Lambda100::new(100),
                target_lambda: adapter.control_telemetry().lambda_target,
                trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
                status: adapter.lambda_telemetry(),
            }
        );

        let mut trace: FixedCommonObservabilityRecordTrace<1> =
            FixedCommonObservabilityRecordTrace::new();
        adapter
            .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
            .expect("cut-freeze lambda shell should survive through observability records");
        assert_eq!(
            trace.get(0).expect("record present").sample.snapshot.lambda,
            CommonLambdaTelemetry {
                activity: CommonLambdaActivity::Frozen,
                reason: CommonLambdaDisableReason::PowerReductionCut,
            }
        );
    }
}

#[test]
fn board_adapter_lambda_telemetry_tracks_acceleration_enrichment_freeze_state() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(2500),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    adapter.configure_speed_density_semantic(
        semantic_ae_freeze_calibration(),
        RuntimeSemanticState::default(),
    );
    adapter.poll_sensor().unwrap();
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(2500),
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

    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::AccelerationEnrichment,
        }
    );
    assert_eq!(
        adapter.lambda_correction_telemetry(),
        CommonLambdaCorrectionTelemetry {
            measured_lambda: Lambda100::new(100),
            target_lambda: adapter.control_telemetry().lambda_target,
            trim_x100: adapter.control_reason_telemetry().lambda_trim_x100,
            status: adapter.lambda_telemetry(),
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("ae-freeze lambda shell should survive through observability records");
    assert_eq!(
        trace.get(0).expect("record present").sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::AccelerationEnrichment,
        }
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .lambda_correction,
        adapter.lambda_correction_telemetry()
    );
}

#[test]
fn board_adapter_transient_enrichment_telemetry_tracks_direct_pulse_width_ae() {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(2500),
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
        .configure_fuel_model(constant_test_fuel_model(PulseWidthUs::new(2500)));
    adapter
        .runtime
        .configure_acceleration_enrichment(ecu_runtime::AccelerationConfig {
            tpsdot_thresh_pct_s: 10,
            mapdot_thresh_kpa_s: 10,
            percent_x100: 120,
            decay_time_ms: 400,
            lockout_ms: 0,
        });
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
    let mut control = control_inputs();
    control.enrichment.tpsdot_pct_s = 25;
    adapter
        .apply_event(BoardEvent::Tick {
            now_us: Micros::new(20),
            control,
        })
        .unwrap()
        .unwrap();

    assert_eq!(
        adapter.transient_enrichment_telemetry(),
        CommonTransientEnrichmentTelemetry {
            acceleration_active: true,
            acceleration_pulse_us: 525,
            acceleration_decay_steps_remaining: 0,
        }
    );
    assert_eq!(
        adapter.lambda_telemetry(),
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::AccelerationEnrichment,
        }
    );
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
            fault: CommonRuntimeFaultTelemetry::default(),
            lambda: CommonLambdaTelemetry::default(),
            lambda_correction: CommonLambdaCorrectionTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            protection: CommonProtectionTelemetry::default(),
            limp_action: CommonLimpActionTelemetry::default(),
            late_event_count: 0,
            max_lateness_us: 0,
            queue_high_water_mark: 0,
            last_drain_count: 0,
            active_queue_count: 0,
            free_queue_slots: 4,
            queue_capacity: 4,
        },
        fault_state: FaultState::default(),
        fault: CommonRuntimeFaultTelemetry::default(),
        lambda: CommonLambdaTelemetry::default(),
        lambda_correction: CommonLambdaCorrectionTelemetry::default(),
        warmup: CommonWarmupTelemetry::default(),
        startup: CommonStartupTelemetry::default(),
        afterstart: CommonAfterstartTelemetry::default(),
        transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
        protection: CommonProtectionTelemetry::default(),
        limp_action: CommonLimpActionTelemetry::default(),
        high_rate_log: adapter.high_rate_log_telemetry(),
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
            fault: CommonFrontierFaultTelemetry::default(),
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
            fault: CommonFrontierFaultTelemetry::default(),
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
            fault: CommonFrontierFaultTelemetry {
                event_id: CommonFrontierFaultEventId::SyncLost,
                severity: FaultSeverity::Warning,
                action: CommonFrontierFaultAction::SafeStateTransition,
            },
        }
    );
}

#[test]
fn board_adapter_frontier_telemetry_tracks_admitted_event_rejection() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    assert!(adapter.actions().commit_frontier_horizon(
        34,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("initial scheduler window queues");

    assert_eq!(
        adapter.actions().execute(Action::ArmScheduler {
            injection: timed_injection(300, 320),
            ignition: timed_ignition(400, 430),
        }),
        Err(ecu_scheduler::ScheduleError::QueueFull)
    );
    assert_eq!(
        adapter.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::AdmittedEventRejected
    );
    assert_eq!(
        adapter.frontier_telemetry().fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::AdmittedEventRejected,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().frontier.active_stop_reason,
        TimingIslandStopReason::AdmittedEventRejected
    );
    assert_eq!(
        adapter.observability_snapshot().frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::AdmittedEventRejected,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
    let snapshot = adapter.observability_snapshot();
    assert_eq!(
        snapshot.protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::ShutdownDriving,
            source: CommonProtectionSource::FrontierFault,
            action: CommonProtectionAction::SafeStateTransition,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );
    assert_eq!(
        snapshot.limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::ShutdownDriving,
            source: CommonLimpActionSource::FrontierFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: false,
            aux_command_count: 0,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("record trace stores admitted-event rejection");
    let record = trace.get(0).expect("record present");
    assert_eq!(record.sample.snapshot.protection, snapshot.protection);
    assert_eq!(record.sample.snapshot.limp_action, snapshot.limp_action);
    assert_eq!(
        record.sample.snapshot.high_rate_log,
        CommonHighRateLogTelemetry {
            decision: adapter.decision_telemetry(),
            fault: adapter.runtime_fault_telemetry(),
            frontier_fault: CommonFrontierFaultTelemetry {
                event_id: CommonFrontierFaultEventId::AdmittedEventRejected,
                severity: FaultSeverity::Critical,
                action: CommonFrontierFaultAction::SafeStateTransition,
            },
            late_event_count: snapshot.diagnostics.late_event_count,
            max_lateness_us: snapshot.diagnostics.max_lateness_us,
            calibration_checksum: adapter.calibration_package_identity().checksum.get(),
        }
    );
}

#[test]
fn board_adapter_supervisor_restart_does_not_restore_live_frontier_or_queued_outputs() {
    let mut adapter = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        MockStore::default(),
    );
    assert!(adapter.actions().commit_frontier_horizon(
        37,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("pre-restart scheduler action queues");
    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("pre-restart UI/log record stores");
    assert_eq!(adapter.actions().queue().active_count(), 4);
    assert_eq!(trace.len(), 1);
    let pre_restart_record = trace.get(0).expect("pre-restart record present");
    assert_eq!(
        pre_restart_record
            .sample
            .snapshot
            .frontier
            .active_horizon_id,
        Some(37)
    );
    assert_eq!(
        pre_restart_record
            .sample
            .snapshot
            .frontier
            .heartbeat_deadline_us,
        Some(Micros::new(300))
    );
    assert_eq!(
        pre_restart_record
            .sample
            .snapshot
            .frontier
            .active_permit_mask,
        TimingIslandPermitMask::ALL
    );
    assert_eq!(
        pre_restart_record.sample.snapshot.scheduler_ownership,
        CommonSchedulerOwnershipTelemetry {
            mode: CommonSchedulerMode::Armed,
            active_groups: ecu_scheduler::OutputGroup::Injector.mask()
                | ecu_scheduler::OutputGroup::Ignition.mask(),
            injection_count: 1,
            ignition_count: 1,
        }
    );

    let mut restarted = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        MockStore::default(),
    );
    assert_eq!(restarted.actions().queue().active_count(), 0);
    assert_eq!(
        restarted.frontier_telemetry().active_horizon_id,
        None,
        "fresh supervisor instance must not restore an active timing horizon"
    );
    assert_eq!(
        restarted.frontier_telemetry().last_accepted_horizon_id,
        None
    );
    assert_eq!(restarted.frontier_telemetry().heartbeat_deadline_us, None);
    assert_eq!(
        restarted.frontier_telemetry().active_permit_mask,
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        restarted.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::None
    );

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        restarted.actions().drain_and_apply_due(
            Micros::new(1_000),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Ok(0)
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn board_adapter_control_core_heartbeat_expiry_suppresses_stale_outputs() {
    let mut adapter = BoardAdapter::new(
        MockSensor(CaptureSample::default()),
        MockCapture::default(),
        ScheduledActionExecutor::<4>::new(),
        MockWatchdog::default(),
        MockTransport::default(),
        MockStore::default(),
    );
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("scheduler action queues");
    assert!(adapter.actions().commit_frontier_horizon(
        38,
        Micros::new(100),
        Micros::new(260),
        Micros::new(150),
        TimingIslandPermitMask::ALL,
    ));
    assert_eq!(adapter.actions().queue().active_count(), 4);
    assert_eq!(adapter.frontier_telemetry().active_horizon_id, Some(38));
    assert_eq!(
        adapter.frontier_telemetry().heartbeat_deadline_us,
        Some(Micros::new(150))
    );
    assert_eq!(
        adapter.frontier_telemetry().active_permit_mask,
        TimingIslandPermitMask::ALL
    );
    assert_eq!(
        adapter.scheduler_ownership_telemetry(),
        CommonSchedulerOwnershipTelemetry {
            mode: CommonSchedulerMode::Armed,
            active_groups: ecu_scheduler::OutputGroup::Injector.mask()
                | ecu_scheduler::OutputGroup::Ignition.mask(),
            injection_count: 1,
            ignition_count: 1,
        }
    );
    assert_eq!(
        adapter.scheduler_reservation_telemetry(),
        CommonSchedulerReservationTelemetry {
            injector_channels: 1,
            ignition_channels: 1,
            idle_channels: 0,
            fan_channels: 0,
        }
    );

    let mut inj0 = RecordingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();
    assert_eq!(
        adapter.actions().drain_and_apply_due(
            Micros::new(151),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Ok(0)
    );

    let snapshot = adapter.observability_snapshot();
    assert_eq!(adapter.actions().queue().active_count(), 0);
    assert_eq!(
        snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::HeartbeatExpired
    );
    assert_eq!(
        snapshot.frontier.active_permit_mask,
        TimingIslandPermitMask::NONE
    );
    assert_eq!(
        snapshot.protection,
        CommonProtectionTelemetry {
            level: CommonProtectionLevel::Degraded,
            source: CommonProtectionSource::FrontierFault,
            action: CommonProtectionAction::OutputSuppressed,
            persistence: CommonProtectionPersistence::LatchedUntilRecovery,
        }
    );
    assert_eq!(inj0.high_count, 0);
    assert_eq!(inj0.low_count, 0);
    assert_eq!(ign0.high_count, 0);
    assert_eq!(ign0.low_count, 0);
}

#[test]
fn board_adapter_observability_snapshot_tracks_board_output_fault() {
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
        .expect("scheduler action queues");
    assert!(adapter.actions().commit_frontier_horizon(
        35,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));

    let mut inj0 = FailingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        adapter.actions().drain_and_apply_due(
            Micros::new(240),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Err(crate::outputs::TransitionApplyError::PinWrite {
            kind: ecu_scheduler::ScheduledTransitionKind::Injector,
            channel: ChannelId::new(0),
            level: ecu_scheduler::ScheduledLevel::High,
            error: crate::outputs::ScheduledOutputPinError::SetHigh,
        })
    );
    assert_eq!(
        adapter.frontier_telemetry().active_stop_reason,
        TimingIslandStopReason::BoardOutputFault
    );
    assert_eq!(
        adapter.frontier_telemetry().fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
    assert_eq!(
        adapter.observability_snapshot().frontier.active_stop_reason,
        TimingIslandStopReason::BoardOutputFault
    );
    assert_eq!(
        adapter.observability_snapshot().frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
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
            fault: CommonRuntimeFaultTelemetry::default(),
            lambda: CommonLambdaTelemetry::default(),
            lambda_correction: CommonLambdaCorrectionTelemetry::default(),
            warmup: CommonWarmupTelemetry::default(),
            startup: CommonStartupTelemetry::default(),
            afterstart: CommonAfterstartTelemetry::default(),
            transient_enrichment: CommonTransientEnrichmentTelemetry::default(),
            protection: CommonProtectionTelemetry::default(),
            limp_action: CommonLimpActionTelemetry::default(),
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

    let expected = CalibrationPackageIdentity::from_snapshot_with_staged_dirty(
        adapter.runtime().calibration_snapshot(),
        true,
    );

    assert_eq!(adapter.observability_snapshot().calibration, expected);
    assert_eq!(
        adapter
            .observability_snapshot()
            .high_rate_log
            .calibration_checksum,
        expected.checksum.get()
    );
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
    assert_eq!(snapshot.lambda, adapter.lambda_telemetry());
    assert_eq!(
        snapshot.lambda_correction,
        adapter.lambda_correction_telemetry()
    );
    assert_eq!(snapshot.torque, adapter.torque_telemetry());
    assert_eq!(snapshot.fuel, adapter.fuel_observation_telemetry());
    assert_eq!(snapshot.engine, adapter.engine_telemetry());
    assert_eq!(snapshot.validated, adapter.validated_input_telemetry());
    assert_eq!(snapshot.frontier, adapter.frontier_telemetry());
    assert_eq!(snapshot.fault, adapter.runtime_fault_telemetry());
    assert_eq!(snapshot.warmup, adapter.warmup_telemetry());
    assert_eq!(snapshot.startup, adapter.startup_telemetry());
    assert_eq!(snapshot.afterstart, adapter.afterstart_telemetry());
    assert_eq!(snapshot.protection, adapter.protection_telemetry());
    assert_eq!(snapshot.limp_action, adapter.limp_action_telemetry());
    assert_eq!(snapshot.high_rate_log, adapter.high_rate_log_telemetry());
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
            fault: adapter.runtime_fault_telemetry(),
            lambda: adapter.lambda_telemetry(),
            lambda_correction: adapter.lambda_correction_telemetry(),
            warmup: adapter.warmup_telemetry(),
            startup: adapter.startup_telemetry(),
            afterstart: adapter.afterstart_telemetry(),
            transient_enrichment: adapter.transient_enrichment_telemetry(),
            protection: adapter.protection_telemetry(),
            limp_action: adapter.limp_action_telemetry(),
            high_rate_log: adapter.high_rate_log_telemetry(),
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
    assert_eq!(
        adapter.limp_action_telemetry(),
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::AuxOnly,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: true,
            aux_command_count: expected.apply_aux_command_count,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("limp-home action record stores current live action shell");
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .limp_action,
        CommonLimpActionTelemetry {
            level: CommonLimpActionLevel::AuxOnly,
            source: CommonLimpActionSource::RuntimeFault,
            cancel_scheduler: false,
            cancel_reason: CancelReason::Manual,
            apply_aux: true,
            aux_command_count: expected.apply_aux_command_count,
            persistence: CommonProtectionPersistence::LatchedUntilClear,
        }
    );
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
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .limp_action,
        adapter.limp_action_telemetry()
    );
    assert_eq!(
        trace
            .get(0)
            .expect("record present")
            .sample
            .snapshot
            .high_rate_log,
        adapter.high_rate_log_telemetry()
    );
}

#[test]
fn board_adapter_push_observability_record_captures_critical_runtime_control_and_calibration_state()
{
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(450),
        angle_x10: Degrees10::new(12),
    });
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();
    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);

    adapter.configure_speed_density_semantic(
        semantic_fuel_cut_calibration(2_500, 9_000),
        RuntimeSemanticState::default(),
    );
    adapter.set_staged_dirty(true);
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

    let expected_calibration = CalibrationPackageIdentity::from_snapshot_with_staged_dirty(
        adapter.runtime().calibration_snapshot(),
        true,
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("record trace stores critical runtime state");

    let record = trace.get(0).expect("record present");
    assert_eq!(record.kind, CommonObservabilityRecordKind::Tick);
    assert_eq!(record.sample.snapshot, adapter.observability_snapshot());

    assert_eq!(
        record.sample.snapshot.decision,
        adapter.decision_telemetry()
    );
    assert_ne!(
        record.sample.snapshot.decision,
        CommonDecisionTelemetry::default()
    );
    assert!(record.sample.snapshot.decision.fuel_cut);
    assert!(record.sample.snapshot.decision.spark_cut);

    assert_eq!(record.sample.snapshot.control, adapter.control_telemetry());
    assert_ne!(
        record.sample.snapshot.control,
        CommonControlTelemetry::default()
    );

    assert_eq!(
        record.sample.snapshot.control_reasons,
        adapter.control_reason_telemetry()
    );
    assert_eq!(record.sample.snapshot.lambda, adapter.lambda_telemetry());
    assert_eq!(
        record.sample.snapshot.control_reasons.lambda_mode,
        CommonLambdaMode::ClosedLoop
    );
    assert!(!record.sample.snapshot.control_reasons.lambda_active);
    assert_eq!(
        record.sample.snapshot.control_reasons.lambda_disable_reason,
        CommonLambdaDisableReason::PowerReductionCut
    );
    assert_eq!(
        record.sample.snapshot.lambda,
        CommonLambdaTelemetry {
            activity: CommonLambdaActivity::Frozen,
            reason: CommonLambdaDisableReason::PowerReductionCut,
        }
    );

    assert_eq!(
        record.sample.snapshot.fuel,
        adapter.fuel_observation_telemetry()
    );
    assert_eq!(
        record.sample.snapshot.fuel,
        CommonFuelObservationTelemetry {
            base_fuel_pulse_width: result.control.base_fuel,
            enriched_fuel_pulse_width: result.control.enriched_fuel,
        }
    );

    assert_eq!(
        record.sample.snapshot.enrichment,
        adapter.enrichment_telemetry()
    );
    assert_eq!(
        record.sample.snapshot.enrichment,
        CommonEnrichmentTelemetry {
            startup_x100: result.control.enrichment.startup_x100,
            warmup_x100: result
                .control
                .fuel_intent
                .observations
                .warmup_correction_x100,
            after_start_x100: result.control.enrichment.after_start_x100,
            acceleration_x100: result.control.enrichment.acceleration_x100,
            total_x100: result.control.enrichment.total_x100(),
        }
    );

    assert_eq!(record.sample.snapshot.torque, adapter.torque_telemetry());
    assert_eq!(
        record.sample.snapshot.torque,
        CommonTorqueTelemetry {
            request_x1000: result.torque_observations.request_x1000,
            allowed_x1000: result.torque_observations.allowed_x1000,
            actuated_x1000: result.torque_observations.actuated_x1000,
        }
    );
    assert!(record.sample.snapshot.torque.request_x1000 > 0);

    assert_eq!(record.sample.snapshot.calibration, expected_calibration);
    assert!(record.sample.snapshot.calibration.staged_dirty);
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
fn board_adapter_push_observability_record_captures_admitted_event_rejection_stop_reason() {
    let sensor = MockSensor(CaptureSample::default());
    let capture = MockCapture::default();
    let actions = ScheduledActionExecutor::<4>::new();
    let watchdog = MockWatchdog::default();
    let transport = MockTransport::default();
    let store = MockStore::default();

    let mut adapter = BoardAdapter::new(sensor, capture, actions, watchdog, transport, store);
    assert!(adapter.actions().commit_frontier_horizon(
        36,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));
    adapter
        .actions()
        .execute(Action::ArmScheduler {
            injection: timed_injection(100, 120),
            ignition: timed_ignition(200, 230),
        })
        .expect("initial scheduler window queues");
    assert_eq!(
        adapter.actions().execute(Action::ArmScheduler {
            injection: timed_injection(300, 320),
            ignition: timed_ignition(400, 430),
        }),
        Err(ecu_scheduler::ScheduleError::QueueFull)
    );

    let mut trace: FixedCommonObservabilityRecordTrace<1> =
        FixedCommonObservabilityRecordTrace::new();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("record trace stores stop reason snapshot");

    let record = trace.get(0).expect("record present");
    assert_eq!(record.kind, CommonObservabilityRecordKind::Tick);
    assert_eq!(
        record.sample.snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::AdmittedEventRejected
    );
    assert_eq!(
        record.sample.snapshot.frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::AdmittedEventRejected,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
}

#[test]
fn board_adapter_push_observability_record_tracks_repeated_frontier_sync_loss_cycles() {
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
    let mut trace: FixedCommonObservabilityRecordTrace<4> =
        FixedCommonObservabilityRecordTrace::new();

    assert!(adapter.actions().commit_frontier_horizon(
        50,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        permit_mask,
    ));
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("initial frontier snapshot records");
    adapter.actions().on_sync_loss();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("first sync loss snapshot records");

    assert!(adapter.actions().commit_frontier_horizon(
        51,
        Micros::new(400),
        Micros::new(560),
        Micros::new(600),
        permit_mask,
    ));
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("recovery snapshot records");
    adapter.actions().on_sync_loss();
    adapter
        .push_observability_record(&mut trace, CommonObservabilityRecordKind::Tick)
        .expect("second sync loss snapshot records");

    assert_eq!(
        trace
            .get(0)
            .expect("initial")
            .sample
            .snapshot
            .frontier
            .active_stop_reason,
        TimingIslandStopReason::None
    );
    assert_eq!(
        trace
            .get(1)
            .expect("first sync loss")
            .sample
            .snapshot
            .frontier
            .active_stop_reason,
        TimingIslandStopReason::SyncLost
    );
    assert_eq!(
        trace
            .get(2)
            .expect("recovery")
            .sample
            .snapshot
            .frontier
            .active_horizon_id,
        Some(51)
    );
    assert_eq!(
        trace
            .get(2)
            .expect("recovery")
            .sample
            .snapshot
            .frontier
            .active_stop_reason,
        TimingIslandStopReason::None
    );
    assert_eq!(
        trace
            .get(3)
            .expect("second sync loss")
            .sample
            .snapshot
            .frontier
            .active_stop_reason,
        TimingIslandStopReason::SyncLost
    );
}

#[test]
fn board_adapter_push_observability_trace_pair_captures_board_output_fault_stop_reason() {
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
        .expect("scheduler action queues");
    assert!(adapter.actions().commit_frontier_horizon(
        37,
        Micros::new(100),
        Micros::new(260),
        Micros::new(300),
        TimingIslandPermitMask::ALL,
    ));

    let mut inj0 = FailingPin::default();
    let mut ign0 = RecordingPin::default();
    let mut injectors: [&mut dyn RawScheduledOutputPin; 1] = [&mut inj0];
    let mut ignition: [&mut dyn RawScheduledOutputPin; 1] = [&mut ign0];
    let mut drained = TransitionDrainBuffer::<4>::new();

    assert_eq!(
        adapter.actions().drain_and_apply_due(
            Micros::new(240),
            &mut drained,
            &mut injectors,
            &mut ignition,
        ),
        Err(crate::outputs::TransitionApplyError::PinWrite {
            kind: ecu_scheduler::ScheduledTransitionKind::Injector,
            channel: ChannelId::new(0),
            level: ecu_scheduler::ScheduledLevel::High,
            error: crate::outputs::ScheduledOutputPinError::SetHigh,
        })
    );

    let mut traces: FixedCommonObservabilityTracePair<1, 1> =
        FixedCommonObservabilityTracePair::new();
    adapter
        .push_observability_to_trace_pair(&mut traces, CommonObservabilityRecordKind::Tick)
        .expect("trace pair stores stop reason snapshot");
    let (samples, records) = traces.split_mut();
    let sample = samples.get(0).expect("sample present");
    let record = records.get(0).expect("record present");
    assert_eq!(
        sample.snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::BoardOutputFault
    );
    assert_eq!(
        sample.snapshot.frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
    );
    assert_eq!(
        record.sample.snapshot.frontier.active_stop_reason,
        TimingIslandStopReason::BoardOutputFault
    );
    assert_eq!(
        record.sample.snapshot.frontier.fault,
        CommonFrontierFaultTelemetry {
            event_id: CommonFrontierFaultEventId::BoardOutputFault,
            severity: FaultSeverity::Critical,
            action: CommonFrontierFaultAction::SafeStateTransition,
        }
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
