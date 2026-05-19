//! x86 runtime board slice built on the new board capability traits.
//!
//! This module keeps the deterministic host-side path separate from the older
//! `EcuApp`-based x86 board glue. It exercises `ecu_runtime::EngineRuntime`
//! through the new `ecu_board_api` trait surface and records output, aux, and
//! telemetry activity in fixed-capacity buffers.

use ecu_board_api::{
    AuxCommand, AuxCommandBatch, AuxOutput, AuxOutputSink, AuxValue, CalibrationPage,
    CalibrationStore, EcuClock, EcuOutput, EdgeBatch, EngineTimeAuthorityTelemetry,
    IgnitionProfileId, IgnitionProfileMode, OutputLevel, OutputScheduler, OutputTransition,
    OutputTransitionBatch, PinMapId, ProfileId, RuntimeBuildId, SensorSnapshot, SensorSource,
    TelemetryFrame, TelemetrySink, TriggerEdge, TriggerEdgeSource,
};
use ecu_board_profiles::{M50B25TU_FULL_COP, M50B25TU_MEGA_COMPAT};
use ecu_domain::{
    CancelReason, Degrees10, EnginePhase, Kpa10, Lambda100, Micros, Percent, PulseWidthUs, Rpm,
    SyncState, Ticks,
};
use ecu_runtime::{
    Action, BaseFuelModel, ControlInputs, EngineRuntime, EnrichmentInputs, IgnitionInputs,
    LambdaTrimInputs, RuntimeOutputProfile, StepInputs, TorqueInputs,
};
use ecu_sim_core as core_plant;

use crate::x86_board::X86PlantBridgeDiagnostics;

const X86_TRIGGER_EDGE_CAP: usize = 8;
const X86_OUTPUT_TRANSITION_CAP: usize = 128;
const X86_AUX_COMMAND_CAP: usize = 16;
const X86_CAL_PAGE_SIZE: usize = 64;
const X86_CAL_PAGE_COUNT: usize = 4;

const X86_PROFILE_ID: ProfileId = ProfileId::new(0x50b2);
const X86_PIN_MAP_ID: PinMapId = PinMapId::new(23);
const X86_RUNTIME_BUILD_ID: RuntimeBuildId = RuntimeBuildId::new(1);

fn m50_ignition_profile_id(profile: RuntimeOutputProfile) -> IgnitionProfileId {
    match profile {
        RuntimeOutputProfile::LegacySingleChannel => IgnitionProfileId::new(1),
        RuntimeOutputProfile::M50(m50) => match m50.ignition_mode {
            ecu_runtime::M50IgnitionMode::WastedSpark3 => IgnitionProfileId::new(3),
            ecu_runtime::M50IgnitionMode::SequentialCop6 => IgnitionProfileId::new(6),
        },
    }
}

fn ignition_profile_mode(profile: RuntimeOutputProfile) -> IgnitionProfileMode {
    match profile {
        RuntimeOutputProfile::LegacySingleChannel => IgnitionProfileMode::WastedSpark,
        RuntimeOutputProfile::M50(m50) => match m50.ignition_mode {
            ecu_runtime::M50IgnitionMode::WastedSpark3 => IgnitionProfileMode::WastedSpark,
            ecu_runtime::M50IgnitionMode::SequentialCop6 => IgnitionProfileMode::SequentialCop,
        },
    }
}

fn known_m50_base_fuel_model() -> BaseFuelModel {
    BaseFuelModel::new(
        [Rpm::new(0); 16],
        [Kpa10::new(0); 16],
        [[PulseWidthUs::new(1_000); 16]; 16],
    )
}

fn baseline_control_inputs(now_us: Micros, rpm: Rpm) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us,
            clt_c: 80,
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
        torque: TorqueInputs::new(500, 0, 1_000, 1_000, 1_000),
        ignition: IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, rpm),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum X86RuntimeBoardError {
    TriggerEdgeOverflow,
    OutputOverflow,
    AuxOverflow,
    CalibrationOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct X86RuntimeBoardDiagnostics {
    pub drained_trigger_edges: usize,
    pub scheduled_transition_count: usize,
    pub aux_command_count: usize,
    pub telemetry_publish_count: usize,
    pub cancel_all_count: usize,
    pub force_safe_state_count: usize,
    pub last_cancel_reason: Option<CancelReason>,
    pub engine_time: EngineTimeAuthorityTelemetry,
    pub synced: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86RuntimeTickResult {
    pub now_us: Micros,
    pub trigger_edges: EdgeBatch<X86_TRIGGER_EDGE_CAP>,
    pub sensor_snapshot: SensorSnapshot,
    pub step_result: ecu_runtime::StepResult,
    pub scheduled_outputs: OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    pub aux_commands: AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    pub telemetry: Option<TelemetryFrame>,
    pub diagnostics: X86RuntimeBoardDiagnostics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86RuntimePlantBridgeFrame<const CYL: usize, const MAX_EVENTS: usize> {
    pub ecu_outputs: core_plant::EcuOutputFrame<CYL, MAX_EVENTS>,
    pub diagnostics: X86PlantBridgeDiagnostics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeterministicClock {
    now_us: Micros,
}

impl DeterministicClock {
    pub const fn new(now_us: Micros) -> Self {
        Self { now_us }
    }

    pub fn set_now_us(&mut self, now_us: Micros) {
        self.now_us = now_us;
    }
}

impl Default for DeterministicClock {
    fn default() -> Self {
        Self::new(Micros::new(1_000))
    }
}

impl EcuClock for DeterministicClock {
    fn now_us(&self) -> Micros {
        self.now_us
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedTriggerEdgeSource {
    edges: EdgeBatch<X86_TRIGGER_EDGE_CAP>,
    drained_total: usize,
}

impl FixedTriggerEdgeSource {
    pub const fn new() -> Self {
        Self {
            edges: EdgeBatch::new(),
            drained_total: 0,
        }
    }

    pub fn set_edges(&mut self, edges: &[TriggerEdge]) -> Result<(), X86RuntimeBoardError> {
        self.edges.clear();
        for edge in edges {
            self.edges
                .push(*edge)
                .map_err(|_| X86RuntimeBoardError::TriggerEdgeOverflow)?;
        }
        Ok(())
    }

    pub fn is_empty(&self) -> bool {
        self.edges.is_empty()
    }
}

impl Default for FixedTriggerEdgeSource {
    fn default() -> Self {
        Self::new()
    }
}

impl TriggerEdgeSource<X86_TRIGGER_EDGE_CAP> for FixedTriggerEdgeSource {
    type Error = X86RuntimeBoardError;

    fn drain_edges(
        &mut self,
        out: &mut EdgeBatch<X86_TRIGGER_EDGE_CAP>,
    ) -> Result<(), Self::Error> {
        out.clear();
        for edge in self.edges.iter() {
            out.push(*edge)
                .map_err(|_| X86RuntimeBoardError::TriggerEdgeOverflow)?;
            self.drained_total += 1;
        }
        self.edges.clear();
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedSensorSource {
    snapshot: SensorSnapshot,
    sample_count: usize,
}

impl FixedSensorSource {
    pub const fn new(snapshot: SensorSnapshot) -> Self {
        Self {
            snapshot,
            sample_count: 0,
        }
    }

    pub fn set_snapshot(&mut self, snapshot: SensorSnapshot) {
        self.snapshot = snapshot;
    }
}

impl Default for FixedSensorSource {
    fn default() -> Self {
        Self::new(SensorSnapshot::new(
            Micros::new(1_000),
            Rpm::new(3_000),
            Kpa10::new(450),
            Percent::new(12),
            840,
            550,
            12_500,
            Lambda100::new(100),
            SyncState::Synced,
            EnginePhase::Running,
        ))
    }
}

impl ecu_board_api::SensorSource for FixedSensorSource {
    type Error = X86RuntimeBoardError;

    fn sample(&mut self, now_us: Micros) -> Result<SensorSnapshot, Self::Error> {
        self.sample_count += 1;
        let mut snapshot = self.snapshot;
        snapshot.now_us = now_us;
        Ok(snapshot)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordingOutputScheduler {
    history: OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    cancel_all_count: usize,
    force_safe_state_count: usize,
}

impl RecordingOutputScheduler {
    pub const fn new() -> Self {
        Self {
            history: OutputTransitionBatch::new(),
            cancel_all_count: 0,
            force_safe_state_count: 0,
        }
    }

    pub fn history(&self) -> OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP> {
        self.history
    }

    pub fn diagnostics(&self) -> (usize, usize) {
        (self.cancel_all_count, self.force_safe_state_count)
    }
}

impl Default for RecordingOutputScheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputScheduler<X86_OUTPUT_TRANSITION_CAP> for RecordingOutputScheduler {
    type Error = X86RuntimeBoardError;

    fn schedule(
        &mut self,
        batch: &OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    ) -> Result<(), Self::Error> {
        for transition in batch.iter() {
            self.history
                .push(*transition)
                .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
        }
        Ok(())
    }

    fn cancel_all(&mut self) -> Result<(), Self::Error> {
        self.cancel_all_count += 1;
        Ok(())
    }

    fn force_safe_state(&mut self) {
        self.force_safe_state_count += 1;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordingAuxOutputSink {
    history: AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    apply_count: usize,
}

impl RecordingAuxOutputSink {
    pub const fn new() -> Self {
        Self {
            history: AuxCommandBatch::new(),
            apply_count: 0,
        }
    }

    pub fn history(&self) -> AuxCommandBatch<X86_AUX_COMMAND_CAP> {
        self.history
    }
}

impl Default for RecordingAuxOutputSink {
    fn default() -> Self {
        Self::new()
    }
}

impl AuxOutputSink<X86_AUX_COMMAND_CAP> for RecordingAuxOutputSink {
    type Error = X86RuntimeBoardError;

    fn apply_aux(
        &mut self,
        batch: &AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    ) -> Result<(), Self::Error> {
        for command in batch.iter() {
            self.history
                .push(*command)
                .map_err(|_| X86RuntimeBoardError::AuxOverflow)?;
        }
        self.apply_count += 1;
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordingTelemetrySink {
    last: Option<TelemetryFrame>,
    publish_count: usize,
}

impl RecordingTelemetrySink {
    pub const fn new() -> Self {
        Self {
            last: None,
            publish_count: 0,
        }
    }

    pub fn last(&self) -> Option<TelemetryFrame> {
        self.last
    }
}

impl Default for RecordingTelemetrySink {
    fn default() -> Self {
        Self::new()
    }
}

impl TelemetrySink for RecordingTelemetrySink {
    type Error = X86RuntimeBoardError;

    fn publish(&mut self, frame: &TelemetryFrame) -> Result<(), Self::Error> {
        self.publish_count += 1;
        self.last = Some(*frame);
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedCalibrationStore {
    pages: [[u8; X86_CAL_PAGE_SIZE]; X86_CAL_PAGE_COUNT],
    page_lens: [usize; X86_CAL_PAGE_COUNT],
}

impl FixedCalibrationStore {
    pub const fn new() -> Self {
        Self {
            pages: [[0; X86_CAL_PAGE_SIZE]; X86_CAL_PAGE_COUNT],
            page_lens: [0; X86_CAL_PAGE_COUNT],
        }
    }

    fn page_index(page: CalibrationPage) -> usize {
        (page.get() as usize) % X86_CAL_PAGE_COUNT
    }
}

impl Default for FixedCalibrationStore {
    fn default() -> Self {
        Self::new()
    }
}

impl CalibrationStore for FixedCalibrationStore {
    type Error = X86RuntimeBoardError;

    fn read_page(&mut self, page: CalibrationPage, out: &mut [u8]) -> Result<usize, Self::Error> {
        let idx = Self::page_index(page);
        let len = self.page_lens[idx].min(out.len());
        out[..len].copy_from_slice(&self.pages[idx][..len]);
        Ok(len)
    }

    fn write_page(&mut self, page: CalibrationPage, bytes: &[u8]) -> Result<(), Self::Error> {
        if bytes.len() > X86_CAL_PAGE_SIZE {
            return Err(X86RuntimeBoardError::CalibrationOverflow);
        }
        let idx = Self::page_index(page);
        self.pages[idx][..bytes.len()].copy_from_slice(bytes);
        self.page_lens[idx] = bytes.len();
        Ok(())
    }
}

#[derive(Debug)]
pub struct X86RuntimeBoard {
    runtime: EngineRuntime,
    clock: DeterministicClock,
    trigger_edges: FixedTriggerEdgeSource,
    sensors: FixedSensorSource,
    outputs: RecordingOutputScheduler,
    aux: RecordingAuxOutputSink,
    telemetry: RecordingTelemetrySink,
    calibration: FixedCalibrationStore,
    diagnostics: X86RuntimeBoardDiagnostics,
}

impl Default for X86RuntimeBoard {
    fn default() -> Self {
        Self::new()
    }
}

impl X86RuntimeBoard {
    pub fn new() -> Self {
        let mut runtime = EngineRuntime::new();
        runtime.configure_m50_mega_compatible();
        runtime.configure_fuel_model(known_m50_base_fuel_model());

        Self {
            runtime,
            clock: DeterministicClock::default(),
            trigger_edges: FixedTriggerEdgeSource::default(),
            sensors: FixedSensorSource::default(),
            outputs: RecordingOutputScheduler::default(),
            aux: RecordingAuxOutputSink::default(),
            telemetry: RecordingTelemetrySink::default(),
            calibration: FixedCalibrationStore::default(),
            diagnostics: X86RuntimeBoardDiagnostics::default(),
        }
    }

    pub fn runtime_mut(&mut self) -> &mut EngineRuntime {
        &mut self.runtime
    }

    pub fn set_clock(&mut self, now_us: Micros) {
        self.clock.set_now_us(now_us);
    }

    pub fn set_sensor_snapshot(&mut self, snapshot: SensorSnapshot) {
        self.sensors.set_snapshot(snapshot);
    }

    pub fn set_trigger_edges(&mut self, edges: &[TriggerEdge]) -> Result<(), X86RuntimeBoardError> {
        self.trigger_edges.set_edges(edges)
    }

    pub fn configure_full_cop(&mut self) {
        self.runtime.configure_m50_full_cop();
    }

    pub fn configure_mega_compatible(&mut self) {
        self.runtime.configure_m50_mega_compatible();
    }

    pub fn scheduled_outputs(&self) -> OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP> {
        self.outputs.history()
    }

    pub fn aux_commands(&self) -> AuxCommandBatch<X86_AUX_COMMAND_CAP> {
        self.aux.history()
    }

    pub fn telemetry_frame(&self) -> Option<TelemetryFrame> {
        self.telemetry.last()
    }

    pub fn diagnostics(&self) -> X86RuntimeBoardDiagnostics {
        self.diagnostics
    }

    pub fn build_core_output_frame<const CYL: usize, const MAX_EVENTS: usize>(
        &self,
    ) -> X86RuntimePlantBridgeFrame<CYL, MAX_EVENTS> {
        bridge_output_transitions_to_core_frame(&self.outputs.history())
    }

    pub fn step_once(&mut self) -> Result<X86RuntimeTickResult, X86RuntimeBoardError> {
        let now_us = self.clock.now_us();
        let mut drained_edges = EdgeBatch::<X86_TRIGGER_EDGE_CAP>::new();
        self.trigger_edges.drain_edges(&mut drained_edges)?;
        let sensor_snapshot = self.sensors.sample(now_us)?;

        self.diagnostics.drained_trigger_edges = drained_edges.len();
        self.diagnostics.engine_time = sensor_snapshot.engine_time;
        self.diagnostics.synced = sensor_snapshot.engine_time.summary == SyncState::Synced;
        self.runtime
            .set_engine_time_authority(sensor_snapshot.engine_time.authority);

        let trigger_synced =
            sensor_snapshot.engine_time.summary == SyncState::Synced && !drained_edges.is_empty();
        let control_inputs = baseline_control_inputs(now_us, sensor_snapshot.rpm);
        let step_result = self.runtime.step(
            StepInputs {
                now_us,
                rpm: sensor_snapshot.rpm.get() as u32,
                load_kpa10: sensor_snapshot.map.get() as u32,
                angle_x10: 0,
                trigger_synced,
                cam_seen: trigger_synced,
                launch_armed: false,
                flat_shift_armed: false,
            },
            control_inputs,
        );

        self.route_actions(&step_result, sensor_snapshot)?;

        let telemetry = self.telemetry.last();

        Ok(X86RuntimeTickResult {
            now_us,
            trigger_edges: drained_edges,
            sensor_snapshot,
            step_result,
            scheduled_outputs: self.outputs.history(),
            aux_commands: self.aux.history(),
            telemetry,
            diagnostics: self.diagnostics,
        })
    }

    #[allow(deprecated)]
    fn route_actions(
        &mut self,
        step_result: &ecu_runtime::StepResult,
        sensor_snapshot: SensorSnapshot,
    ) -> Result<(), X86RuntimeBoardError> {
        let runtime_snapshot = self.runtime.snapshot();
        let mut safe_state_applied = false;
        for action in step_result.actions.iter() {
            match action {
                Action::ArmScheduler { .. } => self.schedule_arm(action)?,
                Action::CancelScheduler(reason) => {
                    self.outputs.cancel_all()?;
                    self.outputs.force_safe_state();
                    self.diagnostics.cancel_all_count += 1;
                    self.diagnostics.force_safe_state_count += 1;
                    self.diagnostics.last_cancel_reason = Some(reason);
                    safe_state_applied = true;
                }
                Action::PublishSnapshot => {
                    let frame = self.build_telemetry_frame(sensor_snapshot, runtime_snapshot);
                    self.telemetry.publish(&frame)?;
                    self.diagnostics.telemetry_publish_count += 1;
                }
                Action::PersistCalibration => {
                    self.calibration.write_page(
                        CalibrationPage::new(0),
                        &sensor_snapshot.now_us.get().to_le_bytes(),
                    )?;
                }
                Action::ApplyAux(batch) => {
                    self.apply_aux_batch(&batch)?;
                }
                Action::SetFan(enabled) => {
                    self.apply_aux_batch(&fan_aux_batch(enabled))?;
                }
                Action::Idle => {
                    self.outputs.force_safe_state();
                    self.diagnostics.force_safe_state_count += 1;
                    safe_state_applied = true;
                }
            }
        }

        if runtime_snapshot.engine.sync != SyncState::Synced && !safe_state_applied {
            self.outputs.force_safe_state();
            self.diagnostics.force_safe_state_count += 1;
        }

        self.diagnostics.scheduled_transition_count = self.outputs.history().len();
        self.diagnostics.aux_command_count = self.aux.history().len();
        Ok(())
    }

    fn apply_aux_batch(
        &mut self,
        batch: &AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    ) -> Result<(), X86RuntimeBoardError> {
        self.aux.apply_aux(batch)?;
        self.diagnostics.aux_command_count = self.aux.history().len();
        Ok(())
    }

    fn schedule_arm(&mut self, action: Action) -> Result<(), X86RuntimeBoardError> {
        let Action::ArmScheduler {
            injection,
            ignition,
        } = action
        else {
            return Ok(());
        };
        let mut batch = OutputTransitionBatch::<X86_OUTPUT_TRANSITION_CAP>::new();

        batch
            .push(OutputTransition::new(
                EcuOutput::Injector(injection.plan.output.channel()),
                OutputLevel::High,
                Ticks::new(injection.start_at.get()),
            ))
            .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
        batch
            .push(OutputTransition::new(
                EcuOutput::Injector(injection.plan.output.channel()),
                OutputLevel::Low,
                Ticks::new(injection.end_at.get()),
            ))
            .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
        batch
            .push(OutputTransition::new(
                EcuOutput::Ignition(ignition.plan.output.channel()),
                OutputLevel::High,
                Ticks::new(ignition.start_at.get()),
            ))
            .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
        batch
            .push(OutputTransition::new(
                EcuOutput::Ignition(ignition.plan.output.channel()),
                OutputLevel::Low,
                Ticks::new(ignition.end_at.get()),
            ))
            .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;

        self.outputs.schedule(&batch)?;
        Ok(())
    }

    fn build_telemetry_frame(
        &self,
        sensor_snapshot: SensorSnapshot,
        runtime_snapshot: ecu_runtime::RuntimeSnapshot,
    ) -> TelemetryFrame {
        let profile = self.runtime.output_profile();
        let ignition_mode = ignition_profile_mode(profile);

        TelemetryFrame::new(
            sensor_snapshot,
            X86_PROFILE_ID,
            m50_ignition_profile_id(profile),
            ignition_mode,
            X86_PIN_MAP_ID,
            X86_RUNTIME_BUILD_ID,
            runtime_snapshot.engine.mode,
            runtime_snapshot.faults.fault,
            runtime_snapshot.faults.severity,
            runtime_snapshot.control.ignition_advance,
            runtime_snapshot.control.dwell,
            runtime_snapshot.control.fuel_pulse_width,
        )
    }
}

/// Compose the x86 runtime board traits around one deterministic runtime tick.
pub fn run_x86_runtime_tick(
    board: &mut X86RuntimeBoard,
) -> Result<X86RuntimeTickResult, X86RuntimeBoardError> {
    board.step_once()
}

pub fn bridge_output_transitions_to_core_frame<const CYL: usize, const MAX_EVENTS: usize>(
    transitions: &OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
) -> X86RuntimePlantBridgeFrame<CYL, MAX_EVENTS> {
    let mut frame = X86RuntimePlantBridgeFrame {
        ecu_outputs: core_plant::EcuOutputFrame::<CYL, MAX_EVENTS>::empty(),
        diagnostics: X86PlantBridgeDiagnostics::empty(),
    };
    let mut injector_high_at_us = [None; CYL];
    let mut ignition_high_at_us = [None; CYL];

    for transition in transitions.iter() {
        let (channel, high_at_us, is_injector) = match transition.output {
            EcuOutput::Injector(channel) => {
                (channel.get() as usize, &mut injector_high_at_us, true)
            }
            EcuOutput::Ignition(channel) => {
                (channel.get() as usize, &mut ignition_high_at_us, false)
            }
        };

        if channel >= CYL {
            frame.diagnostics.out_of_range_channel_count += 1;
            continue;
        }

        match transition.level {
            OutputLevel::High => {
                if high_at_us[channel].is_some() {
                    frame.diagnostics.duplicate_high_count += 1;
                } else {
                    high_at_us[channel] = Some(transition.at.get());
                }
            }
            OutputLevel::Low => {
                let Some(start_us) = high_at_us[channel].take() else {
                    frame.diagnostics.orphan_low_count += 1;
                    continue;
                };
                let end_us = transition.at.get();
                if end_us < start_us {
                    frame.diagnostics.out_of_order_transition_count += 1;
                    continue;
                }

                let width_us = end_us - start_us;
                if is_injector {
                    // The x86 board path only exposes pulse width, so the
                    // bridge uses a fixed nominal injector flow to convert
                    // that width into a plant fuel mass. Keep the value
                    // stable so the test remains deterministic.
                    let command = core_plant::InjectionCommand {
                        cylinder: core_plant::CylinderIndex(channel as u8),
                        mode: core_plant::InjectionTimingMode::StartOfInjection,
                        angle_deg10: core_plant::CrankDeg10(0),
                        pulse_width_us: core_plant::Micros(width_us),
                        injector_flow_ug_per_us: core_plant::MicrogramsPerMicros(5),
                        deadtime_us: core_plant::Micros(0),
                    };
                    if frame.ecu_outputs.injection_events.push(command).is_err() {
                        frame.diagnostics.capacity_overflow_count += 1;
                    }
                } else {
                    let command = core_plant::SparkCommand {
                        cylinder: core_plant::CylinderIndex(channel as u8),
                        spark_angle_deg10: core_plant::CrankDeg10(0),
                        dwell_us: core_plant::Micros(width_us),
                        coil_energy_x1000: 1_000,
                    };
                    if frame.ecu_outputs.spark_events.push(command).is_err() {
                        frame.diagnostics.capacity_overflow_count += 1;
                    }
                }
            }
        }
    }

    frame.diagnostics.open_high_count = injector_high_at_us
        .iter()
        .chain(ignition_high_at_us.iter())
        .filter(|entry| entry.is_some())
        .count() as u32;

    frame
}

fn fan_aux_batch(enabled: bool) -> AuxCommandBatch<X86_AUX_COMMAND_CAP> {
    let mut batch = AuxCommandBatch::new();
    let _ = batch.push(AuxCommand::new(
        AuxOutput::Fan,
        if enabled {
            AuxValue::Level(OutputLevel::High)
        } else {
            AuxValue::Off
        },
    ));
    batch
}

pub fn m50_mega_profile() -> &'static ecu_board_profiles::EngineBoardProfile {
    &M50B25TU_MEGA_COMPAT
}

pub fn m50_full_cop_profile() -> &'static ecu_board_profiles::EngineBoardProfile {
    &M50B25TU_FULL_COP
}
