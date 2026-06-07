/// Runtime sync state.
///
/// This is the smallest first-pass vocabulary for decoder-owned engine sync.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SyncState {
    #[default]
    Unsynced,
    Provisional,
    Locked {
        cam_ref: bool,
    },
}

/// Engine operating phase.
///
/// This is intentionally coarse and describes the engine's broad lifecycle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum EnginePhase {
    #[default]
    Off,
    Cranking,
    Running,
    Stopping,
}

/// High-level control mode.
///
/// These modes describe how downstream control logic should behave.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum ControlMode {
    #[default]
    OpenLoop,
    ClosedLoop,
    LimpHome,
    Shutdown,
}

/// Fault severity.
///
/// Higher severity values represent conditions that should dominate display and policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum FaultSeverity {
    #[default]
    Info,
    Warning,
    Critical,
}

/// Fault classification.
///
/// This first-pass set is intentionally small and can grow as the runtime splits responsibilities.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum FaultCode {
    #[default]
    None,
    SyncLoss,
    SensorOutOfRange,
    CalibrationInvalid,
    SafetyCut,
    ActuatorFault,
}

/// Reason for cancelling queued work.
///
/// Cancellation is explicit so scheduler and runtime code do not rely on sentinels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum CancelReason {
    #[default]
    Manual,
    SyncLoss,
    SafetyShutdown,
    Commit,
    Timeout,
}

/// Policy to use when applying a staged calibration commit.
///
/// Commit policy stays separate from the calibration payload itself.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum CommitPolicy {
    #[default]
    Immediate,
    SafeOnly,
    Deferred,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enums_expose_the_expected_default_states() {
        assert_eq!(SyncState::default(), SyncState::Unsynced);
        assert_eq!(EnginePhase::default(), EnginePhase::Off);
        assert_eq!(ControlMode::default(), ControlMode::OpenLoop);
        assert_eq!(FaultSeverity::default(), FaultSeverity::Info);
        assert_eq!(FaultCode::default(), FaultCode::None);
        assert_eq!(CancelReason::default(), CancelReason::Manual);
        assert_eq!(CommitPolicy::default(), CommitPolicy::Immediate);
    }

    #[test]
    fn enums_cover_a_representable_state_space() {
        let states = [
            SyncState::Unsynced,
            SyncState::Provisional,
            SyncState::Locked { cam_ref: false },
        ];
        assert_eq!(states.len(), 3);

        let phases = [
            EnginePhase::Off,
            EnginePhase::Cranking,
            EnginePhase::Running,
            EnginePhase::Stopping,
        ];
        assert_eq!(phases.len(), 4);

        let modes = [
            ControlMode::OpenLoop,
            ControlMode::ClosedLoop,
            ControlMode::LimpHome,
            ControlMode::Shutdown,
        ];
        assert_eq!(modes.len(), 4);

        let severities = [
            FaultSeverity::Info,
            FaultSeverity::Warning,
            FaultSeverity::Critical,
        ];
        assert_eq!(severities.len(), 3);

        let faults = [
            FaultCode::None,
            FaultCode::SyncLoss,
            FaultCode::SensorOutOfRange,
            FaultCode::CalibrationInvalid,
            FaultCode::SafetyCut,
            FaultCode::ActuatorFault,
        ];
        assert_eq!(faults.len(), 6);

        let reasons = [
            CancelReason::Manual,
            CancelReason::SyncLoss,
            CancelReason::SafetyShutdown,
            CancelReason::Commit,
            CancelReason::Timeout,
        ];
        assert_eq!(reasons.len(), 5);

        let policies = [
            CommitPolicy::Immediate,
            CommitPolicy::SafeOnly,
            CommitPolicy::Deferred,
        ];
        assert_eq!(policies.len(), 3);
    }
}
