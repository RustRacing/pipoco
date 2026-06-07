use super::*;

impl EngineRuntime {
    fn validate_inputs(&self, inputs: StepInputs) -> ValidatedInputs {
        const MAX_RPM: u32 = 9000;
        const MAX_LOAD: u32 = 2000;
        const MAX_ANGLE_X10: i32 = 7200;

        let clamped_rpm = inputs.rpm.min(MAX_RPM) as u16;
        let clamped_load = inputs.load_kpa10.min(MAX_LOAD) as u16;
        let clamped_angle = inputs.angle_x10.clamp(-MAX_ANGLE_X10, MAX_ANGLE_X10) as i16;

        ValidatedInputs {
            rpm: Rpm::new(clamped_rpm),
            load_kpa10: Kpa10::new(clamped_load),
            angle_x10: Degrees10::new(clamped_angle),
            clamped: clamped_rpm as u32 != inputs.rpm
                || clamped_load as u32 != inputs.load_kpa10
                || clamped_angle as i32 != inputs.angle_x10,
        }
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

    pub fn step(&mut self, inputs: StepInputs, control_inputs: ControlInputs) -> StepResult {
        let authority = self.engine.engine_time_authority;
        self.step_with_authority(inputs, control_inputs, authority)
    }

    /// Step the runtime with structured engine-time authority already supplied by the caller.
    ///
    /// Board adapters should use this when they have a decoder/profile authority snapshot so
    /// output gating does not fall back to boolean sync/cam inputs.
    pub fn step_with_authority(
        &mut self,
        inputs: StepInputs,
        control_inputs: ControlInputs,
        authority: EngineTimeAuthority,
    ) -> StepResult {
        let validated = self.validate_inputs(inputs);
        let authority = derive_engine_time_authority(
            authority,
            inputs.trigger_synced,
            inputs.cam_seen,
            validated.rpm,
        );
        self.step_with_validated(inputs.now_us, validated, control_inputs, authority)
    }

    fn step_with_validated(
        &mut self,
        now_us: Micros,
        validated: ValidatedInputs,
        control_inputs: ControlInputs,
        authority: EngineTimeAuthority,
    ) -> StepResult {
        self.engine.rpm = validated.rpm;
        self.engine.angle_x10 = validated.angle_x10;
        self.engine.load_kpa10 = validated.load_kpa10;
        self.set_engine_time_authority_inner(authority);
        self.engine.mode = self.derive_operating_mode();
        let control = self.compose_control(&validated, control_inputs);
        let actions = self.emit_actions(now_us, &control);
        self.apply_control_cut_state(&control);
        let torque_observations = TorqueObservations::from_step(
            control.torque,
            self.engine.mode,
            self.engine.phase,
            actions,
        );
        self.refresh_snapshot();

        StepResult {
            validated,
            operating_mode: self.engine.mode,
            control,
            actions,
            torque_observations,
        }
    }
}
