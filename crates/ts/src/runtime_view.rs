use ecu_calibration::{
    ActiveCalibration, CalibrationClass, CalibrationDiff, CalibrationSnapshot, CommitRules,
    CommitVerdict, ExpertTriggerCalibration, ExpertTriggerRecordError,
    ExpertTriggerValidationError, PersistedCalibrationBlob, StagedCalibration,
    EXPERT_TRIGGER_RECORD_LEN,
};
use ecu_domain::CommitPolicy;
use ecu_domain::{CancelReason, ControlMode, EnginePhase, FaultCode, FaultSeverity, SyncState};
use ecu_runtime::RuntimeSnapshot;

pub const PAGE_EXPERT_TRIGGER: u8 = 16;
pub const EXPERT_TRIGGER_PAGE_LEN: usize = EXPERT_TRIGGER_RECORD_LEN;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpertTriggerPageError {
    Record(ExpertTriggerRecordError),
    Transition(ExpertTriggerValidationError),
}

pub fn encode_expert_trigger_page(
    calibration: &ExpertTriggerCalibration,
    out: &mut [u8],
) -> Result<usize, ExpertTriggerPageError> {
    calibration
        .encode_record(out)
        .map_err(ExpertTriggerPageError::Record)
}

pub fn decode_expert_trigger_page(
    data: &[u8],
) -> Result<ExpertTriggerCalibration, ExpertTriggerPageError> {
    ExpertTriggerCalibration::decode_record(data).map_err(ExpertTriggerPageError::Record)
}

pub fn apply_expert_trigger_page(
    current: &ExpertTriggerCalibration,
    data: &[u8],
) -> Result<ExpertTriggerCalibration, ExpertTriggerPageError> {
    let proposed = decode_expert_trigger_page(data)?;
    proposed
        .validate_transition_from(current)
        .map_err(ExpertTriggerPageError::Transition)?;
    Ok(proposed)
}

/// TunerStudio-facing runtime view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TunerStudioRuntimeView {
    pub sync_state: SyncState,
    pub engine_phase: EnginePhase,
    pub control_mode: ControlMode,
    pub rpm: u16,
    pub load_kpa10: u16,
    pub angle_x10: i16,
    pub fuel_pulse_width_us: u16,
    pub ignition_advance_deg10: i16,
    pub dwell_us: u16,
    pub lambda_target_x100: u16,
    pub torque_limit_x100: u16,
    pub fault_code: FaultCode,
    pub fault_severity: FaultSeverity,
    pub cancel_reason: CancelReason,
}

/// Adapter that maps runtime snapshots into a TS view.
///
/// This surface reads only `RuntimeSnapshot`; it does not inspect live runtime
/// internals directly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeSnapshotAdapter;

impl RuntimeSnapshotAdapter {
    pub const fn new() -> Self {
        Self
    }

    pub fn fill_outpc(&self, snapshot: &RuntimeSnapshot, out: &mut crate::outpc::Outpc) {
        self.map(snapshot).fill_outpc_runtime_fields(out);
    }

    pub fn map(&self, snapshot: &RuntimeSnapshot) -> TunerStudioRuntimeView {
        TunerStudioRuntimeView {
            sync_state: snapshot.engine.sync,
            engine_phase: snapshot.engine.phase,
            control_mode: snapshot.engine.mode,
            rpm: snapshot.engine.rpm.get(),
            load_kpa10: snapshot.engine.load_kpa10.get(),
            angle_x10: snapshot.engine.angle_x10.get(),
            fuel_pulse_width_us: snapshot.control.fuel_pulse_width.get(),
            ignition_advance_deg10: snapshot.control.ignition_advance.get(),
            dwell_us: snapshot.control.dwell.get(),
            lambda_target_x100: snapshot.control.lambda_target.get(),
            torque_limit_x100: snapshot.control.torque_limit_x100,
            fault_code: snapshot.faults.fault,
            fault_severity: snapshot.faults.severity,
            cancel_reason: snapshot.faults.cancel_reason,
        }
    }
}

impl TunerStudioRuntimeView {
    pub fn fill_outpc_runtime_fields(&self, out: &mut crate::outpc::Outpc) {
        out.rpm = self.rpm;
        out.map_kpa_x10 = self.load_kpa10;
        out.pw_us = self.fuel_pulse_width_us;
        out.dwell_us = self.dwell_us;
        out.advance_x10 = self.ignition_advance_deg10;
        out.synced = u8::from(matches!(self.sync_state, SyncState::Locked { .. }));
    }
}

/// TS-visible calibration write intent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationWrite {
    NoChange,
    MetadataOnly,
    RuntimeSafe,
    RuntimeSensitive,
    SafetyCritical,
}

impl CalibrationWrite {
    fn class(self) -> CalibrationClass {
        match self {
            CalibrationWrite::NoChange => CalibrationClass::NoChange,
            CalibrationWrite::MetadataOnly => CalibrationClass::MetadataOnly,
            CalibrationWrite::RuntimeSafe => CalibrationClass::RuntimeSafe,
            CalibrationWrite::RuntimeSensitive => CalibrationClass::RuntimeSensitive,
            CalibrationWrite::SafetyCritical => CalibrationClass::SafetyCritical,
        }
    }
}

/// TS write surface that only mutates staged calibration.
///
/// For the current phase, this type is also the accepted home for
/// commit/reset/persist orchestration over `CalibrationSnapshot` plus
/// `CommitRules`. Active calibration still changes only through commit flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationEditSurface {
    snapshot: CalibrationSnapshot,
    diff: CalibrationDiff,
}

impl CalibrationEditSurface {
    pub fn new(snapshot: CalibrationSnapshot) -> Self {
        let diff = CalibrationDiff::from_views(
            snapshot.active,
            snapshot.staged,
            CalibrationClass::NoChange,
        );
        Self { snapshot, diff }
    }

    pub fn apply_write(&mut self, write: CalibrationWrite) -> CalibrationDiff {
        if write != CalibrationWrite::NoChange {
            self.snapshot.staged.mark_dirty();
        }
        self.diff =
            CalibrationDiff::from_views(self.snapshot.active, self.snapshot.staged, write.class());
        self.diff
    }

    pub fn apply_expert_trigger_page(
        &mut self,
        data: &[u8],
    ) -> Result<CalibrationDiff, ExpertTriggerPageError> {
        let current = self.snapshot.active.calibration().geometry.expert_trigger;
        let proposed = apply_expert_trigger_page(&current, data)?;
        let mut staged_calibration = self.snapshot.staged.calibration();
        if staged_calibration.geometry.expert_trigger == proposed {
            self.diff = CalibrationDiff::from_views(
                self.snapshot.active,
                self.snapshot.staged,
                CalibrationClass::NoChange,
            );
            return Ok(self.diff);
        }
        staged_calibration.set_expert_trigger(proposed);
        self.snapshot.staged.replace_calibration(staged_calibration);
        self.diff = CalibrationDiff::from_views(
            self.snapshot.active,
            self.snapshot.staged,
            CalibrationClass::SafetyCritical,
        );
        Ok(self.diff)
    }

    pub fn snapshot(&self) -> CalibrationSnapshot {
        self.snapshot
    }

    pub fn diff(&self) -> CalibrationDiff {
        self.diff
    }

    pub fn commit(&mut self, policy: CommitPolicy, runtime_safe: bool) -> CalibrationCommandResult {
        let verdict = CommitRules::evaluate(policy, self.diff, runtime_safe);
        match verdict {
            CommitVerdict::AllowNow => {
                let committed = self.diff;
                self.snapshot.active = ActiveCalibration::new(
                    self.snapshot.staged.revision(),
                    self.snapshot.staged.calibration(),
                );
                self.snapshot.staged = StagedCalibration::new(
                    self.snapshot.active.revision(),
                    self.snapshot.active.calibration(),
                );
                self.diff = CalibrationDiff::from_views(
                    self.snapshot.active,
                    self.snapshot.staged,
                    CalibrationClass::NoChange,
                );
                CalibrationCommandResult::Committed(committed)
            }
            CommitVerdict::Defer => CalibrationCommandResult::Deferred(self.diff),
        }
    }

    pub fn persist(&self) -> PersistedCalibrationBlob {
        PersistedCalibrationBlob::new(self.snapshot)
    }

    pub fn reset_staged(&mut self) -> CalibrationCommandResult {
        self.snapshot.staged = StagedCalibration::new(
            self.snapshot.active.revision(),
            self.snapshot.active.calibration(),
        );
        self.diff = CalibrationDiff::from_views(
            self.snapshot.active,
            self.snapshot.staged,
            CalibrationClass::NoChange,
        );
        CalibrationCommandResult::Reset(self.diff)
    }
}

/// Outcome of a TS calibration command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationCommandResult {
    Committed(CalibrationDiff),
    Deferred(CalibrationDiff),
    Reset(CalibrationDiff),
}

#[cfg(test)]
#[path = "runtime_view_tests.rs"]
mod tests;
