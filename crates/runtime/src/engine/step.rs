use super::*;
use crate::AuthorityStepInputs;

impl EngineRuntime {
    fn validate_step_scalars(
        &mut self,
        timestamp: Micros,
        rpm: u32,
        load_kpa10: u32,
        angle_x10: i32,
    ) -> ValidatedInputs {
        const MAX_RPM: u32 = 9000;
        const MAX_LOAD: u32 = 2000;
        const MAX_ANGLE_X10: i32 = 7200;

        let clamped_rpm = rpm.min(MAX_RPM) as u16;
        let clamped_load = load_kpa10.min(MAX_LOAD) as u16;
        let clamped_angle = angle_x10.clamp(-MAX_ANGLE_X10, MAX_ANGLE_X10) as i16;

        let validated = ValidatedInputs {
            rpm: Rpm::new(clamped_rpm),
            load_kpa10: Kpa10::new(clamped_load),
            angle_x10: Degrees10::new(clamped_angle),
            clamped: clamped_rpm as u32 != rpm
                || clamped_load as u32 != load_kpa10
                || clamped_angle as i32 != angle_x10,
        };
        self.record_signal_stage(SignalStage::ObservationValidator, timestamp);
        validated
    }

    fn derive_operating_mode(&self) -> ControlMode {
        if self.faults.severity == FaultSeverity::Critical
            || self.faults.fault == FaultCode::SafetyCut
        {
            ControlMode::Shutdown
        } else if self.faults.severity == FaultSeverity::Warning
            || self.faults.fault == FaultCode::SensorOutOfRange
        {
            ControlMode::LimpHome
        } else if self.engine.sync == SyncState::Unsynced {
            ControlMode::OpenLoop
        } else if self.engine.phase == EnginePhase::Running {
            ControlMode::ClosedLoop
        } else {
            ControlMode::OpenLoop
        }
    }

    /// Compatibility/support stepping path using raw scalar inputs.
    ///
    /// Prefer [`EngineRuntime::step_with_authority`] for product integration so
    /// the runtime receives structured engine-time authority instead of deriving
    /// it from boolean sync/cam flags.
    pub fn step(&mut self, inputs: StepInputs, control_inputs: ControlInputs) -> StepResult {
        let validated = self.validate_step_scalars(
            inputs.now_us,
            inputs.rpm,
            inputs.load_kpa10,
            inputs.angle_x10,
        );
        let authority = derive_engine_time_authority(
            self.engine.engine_time_authority,
            inputs.trigger_synced,
            inputs.cam_seen,
            validated.rpm,
        );
        self.step_with_validated(
            inputs.now_us,
            validated,
            control_inputs,
            authority,
            inputs.launch_armed,
            inputs.flat_shift_armed,
            inputs.safety_latch_request,
        )
    }

    /// Step the runtime with canonical authority-aware product inputs.
    ///
    /// This is the canonical product ingress.
    pub fn step_with_authority(
        &mut self,
        inputs: AuthorityStepInputs,
        control_inputs: ControlInputs,
    ) -> StepResult {
        let validated = self.validate_step_scalars(
            inputs.now_us,
            inputs.rpm,
            inputs.load_kpa10,
            inputs.angle_x10,
        );
        self.step_with_validated(
            inputs.now_us,
            validated,
            control_inputs,
            inputs.authority,
            inputs.launch_armed,
            inputs.flat_shift_armed,
            inputs.safety_latch_request,
        )
    }

    /// Step the runtime from the formal differential input surface.
    ///
    /// This helper is intended for FM0016 representability and conformance work.
    /// It is the only ingress that honors `DifferentialInputSnapshot::{mode,
    /// fuel_cut, spark_cut}` directly. Product integrations should keep using
    /// [`EngineRuntime::step`] or [`EngineRuntime::step_with_authority`].
    pub fn step_with_differential_input(
        &mut self,
        input: DifferentialInputSnapshot,
        control_inputs: ControlInputs,
    ) -> StepResult {
        self.set_direct_cut_requests(input.fuel_cut, input.spark_cut);

        match input.mode {
            RuntimeEngineMode::Shutdown => self.set_fault_state(
                FaultCode::SafetyCut,
                FaultSeverity::Critical,
                CancelReason::SafetyShutdown,
            ),
            _ => self.set_fault_state(FaultCode::None, FaultSeverity::Info, CancelReason::Manual),
        }

        let mut step_inputs = input.to_step_inputs();
        match input.mode {
            RuntimeEngineMode::Off => {
                step_inputs.rpm = 0;
                step_inputs.load_kpa10 = 0;
                step_inputs.trigger_synced = false;
                step_inputs.cam_seen = false;
            }
            RuntimeEngineMode::Cranking => {
                step_inputs.rpm = step_inputs.rpm.max(1);
                step_inputs.trigger_synced = false;
                step_inputs.cam_seen = false;
            }
            RuntimeEngineMode::Running => {
                step_inputs.rpm = step_inputs.rpm.max(1);
                step_inputs.trigger_synced = true;
                step_inputs.cam_seen = true;
            }
            RuntimeEngineMode::Shutdown => {}
        }

        self.step(step_inputs, control_inputs)
    }

    #[allow(clippy::too_many_arguments)]
    fn step_with_validated(
        &mut self,
        now_us: Micros,
        validated: ValidatedInputs,
        control_inputs: ControlInputs,
        authority: EngineTimeAuthority,
        launch_armed: bool,
        flat_shift_armed: bool,
        safety_latch_request: bool,
    ) -> StepResult {
        self.engine.rpm = validated.rpm;
        self.engine.angle_x10 = validated.angle_x10;
        self.engine.load_kpa10 = validated.load_kpa10;
        self.set_engine_time_authority_inner(authority);
        self.engine.mode = self.derive_operating_mode();
        let control = self.compose_control(
            &validated,
            control_inputs,
            launch_armed,
            flat_shift_armed,
            safety_latch_request,
        );
        self.record_signal_stage(SignalStage::PolicyConsumer, now_us);
        let actions = self.emit_actions(now_us, &control);
        self.apply_control_cut_state(&control);
        let torque_observations = TorqueObservations::from_step(
            control.torque,
            self.engine.mode,
            self.engine.phase,
            self.rev_hard_active,
            self.fuel_cut,
            self.spark_cut,
        );
        self.refresh_snapshot();
        self.record_signal_stage(SignalStage::RuntimeSnapshotBuilder, now_us);

        StepResult {
            validated,
            operating_mode: self.engine.mode,
            control,
            actions,
            torque_observations,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_control::IgnitionInputs;

    #[test]
    fn step_records_only_runtime_owned_signal_stages() {
        let mut runtime = EngineRuntime::new();
        let ignition = IgnitionInputs::new(Degrees10::new(0), 0, 0, 0, false, Rpm::new(0));
        let inputs = StepInputs {
            now_us: Micros::new(42),
            rpm: 1_234,
            load_kpa10: 321,
            angle_x10: 87,
            trigger_synced: true,
            cam_seen: true,
            launch_armed: false,
            flat_shift_armed: false,
            safety_latch_request: false,
        };

        let _ = runtime.step(inputs, ControlInputs::spark_only(Micros::new(42), ignition));

        let counters = runtime.signal_assembly_counters;
        assert_eq!(counters.observation_validator.seen, 1);
        assert_eq!(counters.observation_validator.accepted, 1);
        assert_eq!(
            counters.observation_validator.last_timestamp,
            Micros::new(42)
        );

        assert_eq!(counters.policy_consumer.seen, 1);
        assert_eq!(counters.policy_consumer.accepted, 1);
        assert_eq!(counters.policy_consumer.last_timestamp, Micros::new(42));

        assert_eq!(counters.runtime_snapshot_builder.seen, 1);
        assert_eq!(counters.runtime_snapshot_builder.accepted, 1);
        assert_eq!(
            counters.runtime_snapshot_builder.last_timestamp,
            Micros::new(42)
        );

        assert_eq!(counters.signal_capture.seen, 0);
        assert_eq!(counters.signal_normalizer.seen, 0);
        assert_eq!(counters.observation_publisher.seen, 0);
        assert_eq!(counters.observation_reader.seen, 0);
    }
}
