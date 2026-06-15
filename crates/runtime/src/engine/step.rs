use super::*;
use crate::AuthorityStepInputs;
use ecu_board_api::frontier::{
    TimingIslandHorizonSequenceId, TimingIslandPermitMask, TimingIslandStopReason,
    HEARTBEAT_EXPIRY_US, HORIZON_SEQUENCE_BITS, MAX_HORIZON_US,
};

#[allow(dead_code)]
pub(crate) type FrontierHorizonSequenceId = TimingIslandHorizonSequenceId;
#[allow(dead_code)]
pub(crate) const FRONTIER_HORIZON_SEQUENCE_BITS: u8 = HORIZON_SEQUENCE_BITS;
#[allow(dead_code)]
pub(crate) const FRONTIER_HEARTBEAT_EXPIRY_US: Micros = HEARTBEAT_EXPIRY_US;
#[allow(dead_code)]
pub(crate) const FRONTIER_MAX_HORIZON_US: Micros = MAX_HORIZON_US;
#[allow(dead_code)]
pub(crate) const FRONTIER_DEFAULT_PERMIT_MASK: TimingIslandPermitMask =
    TimingIslandPermitMask::NONE;
#[allow(dead_code)]
pub(crate) const FRONTIER_DEFAULT_STOP_REASON: TimingIslandStopReason =
    TimingIslandStopReason::None;

impl EngineRuntime {
    fn validate_step_scalars(&self, rpm: u32, load_kpa10: u32, angle_x10: i32) -> ValidatedInputs {
        const MAX_RPM: u32 = 9000;
        const MAX_LOAD: u32 = 2000;
        const MAX_ANGLE_X10: i32 = 7200;

        let clamped_rpm = rpm.min(MAX_RPM) as u16;
        let clamped_load = load_kpa10.min(MAX_LOAD) as u16;
        let clamped_angle = angle_x10.clamp(-MAX_ANGLE_X10, MAX_ANGLE_X10) as i16;

        ValidatedInputs {
            rpm: Rpm::new(clamped_rpm),
            load_kpa10: Kpa10::new(clamped_load),
            angle_x10: Degrees10::new(clamped_angle),
            clamped: clamped_rpm as u32 != rpm
                || clamped_load as u32 != load_kpa10
                || clamped_angle as i32 != angle_x10,
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

    /// Compatibility/support stepping path using raw scalar inputs.
    ///
    /// Prefer [`EngineRuntime::step_with_authority`] for product integration so
    /// the runtime receives structured engine-time authority instead of deriving
    /// it from boolean sync/cam flags.
    pub fn step(&mut self, inputs: StepInputs, control_inputs: ControlInputs) -> StepResult {
        let validated = self.validate_step_scalars(inputs.rpm, inputs.load_kpa10, inputs.angle_x10);
        let authority = derive_engine_time_authority(
            self.engine.engine_time_authority,
            inputs.trigger_synced,
            inputs.cam_seen,
            validated.rpm,
        );
        self.step_with_validated(inputs.now_us, validated, control_inputs, authority)
    }

    /// Step the runtime with canonical authority-aware product inputs.
    ///
    /// This is the canonical product ingress.
    pub fn step_with_authority(
        &mut self,
        inputs: AuthorityStepInputs,
        control_inputs: ControlInputs,
    ) -> StepResult {
        let validated = self.validate_step_scalars(inputs.rpm, inputs.load_kpa10, inputs.angle_x10);
        self.step_with_validated(inputs.now_us, validated, control_inputs, inputs.authority)
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
