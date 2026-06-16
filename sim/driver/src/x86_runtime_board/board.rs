use ecu_board_api::{
    legacy::{CalibrationPage, CalibrationStore},
    AuxCommandBatch, AuxOutputSink, EcuClock, EcuOutput, EdgeBatch, FullEcuOutputProfile,
    OutputLevel, OutputScheduler, OutputTransition, OutputTransitionBatch, RuntimeOutputProfile,
    SensorSnapshot, SensorSource, TelemetryFrame, TelemetrySink, TriggerEdge, TriggerEdgeSource,
};
use ecu_domain::{CancelReason, FaultCode, FaultSeverity, Micros, SyncState, Ticks};
use ecu_runtime::ingress::AuthorityStepInputs;
use ecu_runtime::{Action, BaseFuelModel, EngineRuntime, RuntimeFuelStrategy};

use super::bridge::bridge_output_transitions_to_core_frame;
use super::io::{
    DeterministicClock, FixedCalibrationStore, FixedSensorSource, FixedTriggerEdgeSource,
    RecordingAuxOutputSink, RecordingOutputScheduler, RecordingTelemetrySink,
};
use super::types::{
    baseline_control_inputs, ignition_profile_id, ignition_profile_mode,
    X86RuntimeBoardDiagnostics, X86RuntimeBoardError, X86RuntimePlantBridgeFrame,
    X86RuntimeTickResult, X86_PIN_MAP_ID, X86_PROFILE_ID, X86_RUNTIME_BUILD_ID,
};
use super::{X86_AUX_COMMAND_CAP, X86_OUTPUT_TRANSITION_CAP, X86_TRIGGER_EDGE_CAP};

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
    pending_launch_armed: bool,
    pending_flat_shift_armed: bool,
}

impl Default for X86RuntimeBoard {
    fn default() -> Self {
        Self::new()
    }
}

impl X86RuntimeBoard {
    fn default_runtime() -> EngineRuntime {
        let mut runtime = EngineRuntime::new();
        runtime.configure_output_profile(RuntimeOutputProfile::default());
        runtime.configure_fuel_model(BaseFuelModel::default());
        runtime
    }

    pub fn new() -> Self {
        Self {
            runtime: Self::default_runtime(),
            clock: DeterministicClock::default(),
            trigger_edges: FixedTriggerEdgeSource::default(),
            sensors: FixedSensorSource::default(),
            outputs: RecordingOutputScheduler::default(),
            aux: RecordingAuxOutputSink::default(),
            telemetry: RecordingTelemetrySink::default(),
            calibration: FixedCalibrationStore::default(),
            diagnostics: X86RuntimeBoardDiagnostics::default(),
            pending_launch_armed: false,
            pending_flat_shift_armed: false,
        }
    }

    pub fn runtime_mut(&mut self) -> &mut EngineRuntime {
        &mut self.runtime
    }

    pub fn configure_fuel_model(&mut self, fuel_model: BaseFuelModel) {
        self.runtime.configure_fuel_model(fuel_model);
    }

    pub fn configure_runtime_fuel_strategy(&mut self, strategy: RuntimeFuelStrategy) {
        self.runtime.configure_runtime_fuel_model(strategy);
    }

    pub fn configure_full_ecu(&mut self, profile: FullEcuOutputProfile) {
        self.runtime.configure_full_ecu(profile);
    }

    pub fn configure_batch_injection(&mut self, cylinders: u8) {
        self.runtime.configure_batch_injection(cylinders);
    }

    pub fn configure_crank_only_wasted_spark(&mut self, coils: u8) {
        self.runtime.configure_crank_only_wasted_spark(coils);
    }

    pub fn set_fault_state(
        &mut self,
        fault: FaultCode,
        severity: FaultSeverity,
        cancel_reason: CancelReason,
    ) {
        self.runtime.set_fault_state(fault, severity, cancel_reason);
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

    pub fn set_shift_arming(&mut self, launch_armed: bool, flat_shift_armed: bool) {
        self.pending_launch_armed = launch_armed;
        self.pending_flat_shift_armed = flat_shift_armed;
    }

    pub fn scheduled_outputs(&self) -> OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP> {
        self.outputs.history()
    }

    pub fn safe_state_outputs(&self) -> OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP> {
        self.outputs.safe_state_history()
    }

    pub fn mark_output_high_for_test(&mut self, output: EcuOutput, at: Ticks) {
        self.outputs.mark_output_high(output, at);
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
        self.diagnostics.synced = matches!(
            sensor_snapshot.engine_time.summary,
            SyncState::Locked { .. }
        );
        let control_inputs = baseline_control_inputs(now_us, sensor_snapshot.rpm);
        let step_result = self.runtime.step_with_authority(
            AuthorityStepInputs::new(
                now_us,
                sensor_snapshot.rpm.get() as u32,
                sensor_snapshot.map.get() as u32,
                0,
                sensor_snapshot.engine_time.authority,
                self.pending_launch_armed,
                self.pending_flat_shift_armed,
                false,
            ),
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

    fn route_actions(
        &mut self,
        step_result: &ecu_runtime::StepResult,
        sensor_snapshot: SensorSnapshot,
    ) -> Result<(), X86RuntimeBoardError> {
        let runtime_snapshot = self.runtime.snapshot();
        let mut safe_state_applied = false;
        for action in step_result.actions.iter() {
            match action {
                Action::ArmScheduler { .. } | Action::ArmInjection(_) | Action::ArmIgnition(_) => {
                    self.schedule_arm(action)?
                }
                Action::CancelScheduler(reason) => {
                    self.outputs.cancel_all()?;
                    self.outputs.set_safe_state_at(sensor_snapshot.now_us);
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
                Action::Idle => {
                    self.outputs.set_safe_state_at(sensor_snapshot.now_us);
                    self.outputs.force_safe_state();
                    self.diagnostics.force_safe_state_count += 1;
                    safe_state_applied = true;
                }
            }
        }

        if !matches!(runtime_snapshot.engine.sync, SyncState::Locked { .. }) && !safe_state_applied
        {
            self.outputs.set_safe_state_at(sensor_snapshot.now_us);
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
        let mut batch = OutputTransitionBatch::<X86_OUTPUT_TRANSITION_CAP>::new();
        let mut push_pair = |output: EcuOutput,
                             start_at: Micros,
                             end_at: Micros|
         -> Result<(), X86RuntimeBoardError> {
            batch
                .push(OutputTransition::new(
                    output,
                    OutputLevel::High,
                    Ticks::new(start_at.get()),
                ))
                .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
            batch
                .push(OutputTransition::new(
                    output,
                    OutputLevel::Low,
                    Ticks::new(end_at.get()),
                ))
                .map_err(|_| X86RuntimeBoardError::OutputOverflow)?;
            Ok(())
        };

        match action {
            Action::ArmScheduler {
                injection,
                ignition,
            } => {
                push_pair(
                    EcuOutput::Injector(injection.plan.output.channel()),
                    injection.start_at,
                    injection.end_at,
                )?;
                push_pair(
                    EcuOutput::Ignition(ignition.plan.output.channel()),
                    ignition.start_at,
                    ignition.end_at,
                )?;
            }
            Action::ArmInjection(injection) => {
                push_pair(
                    EcuOutput::Injector(injection.plan.output.channel()),
                    injection.start_at,
                    injection.end_at,
                )?;
            }
            Action::ArmIgnition(ignition) => {
                push_pair(
                    EcuOutput::Ignition(ignition.plan.output.channel()),
                    ignition.start_at,
                    ignition.end_at,
                )?;
            }
            Action::CancelScheduler(_)
            | Action::PublishSnapshot
            | Action::PersistCalibration
            | Action::ApplyAux(_)
            | Action::Idle => return Ok(()),
        }

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
            ignition_profile_id(profile),
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
