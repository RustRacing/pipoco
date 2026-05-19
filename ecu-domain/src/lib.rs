#![cfg_attr(not(test), no_std)]

/// Engine speed in revolutions per minute.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Rpm(u16);

impl Rpm {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Micros(u32);

impl Micros {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Scheduler ticks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Ticks(u32);

impl Ticks {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Manifold pressure in kPa x 10.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Kpa10(u16);

impl Kpa10 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Percent value in the range 0-100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Percent(u8);

impl Percent {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Injector pulse width in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PulseWidthUs(u16);

impl PulseWidthUs {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Dwell time in microseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct DwellUs(u16);

impl DwellUs {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Crank angle in tenths of a degree.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Degrees10(i16);

impl Degrees10 {
    pub const fn new(value: i16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i16 {
        self.0
    }
}

/// Lambda value scaled by 100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Lambda100(u16);

impl Lambda100 {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Cylinder identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CylinderId(u8);

impl CylinderId {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Output channel identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ChannelId(u8);

impl ChannelId {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

/// Runtime sync state.
///
/// This is the smallest first-pass vocabulary for decoder-owned engine sync.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SyncState {
    #[default]
    Unsynced,
    Syncing,
    Synced,
}

/// Primary crank trigger synchronization state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum CrankSyncState {
    #[default]
    NoSignal,
    PrimarySearching,
    PrimaryLocked,
    SyncLost,
}

/// 720-degree phase evidence available to engine-time consumers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum PhaseSyncState {
    #[default]
    Unknown,
    CrankOnly360,
    CamObserved720,
    CamValidated720,
}

/// Source of absolute crank-angle timing authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum AbsoluteTimeAuthority {
    #[default]
    None,
    GeometryOnly,
    ExpertManual,
    CommunityProfile,
    CertifiedProfile,
    BenchLearned,
}

/// Structured engine-time authority used before legacy `SyncState` summaries.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct EngineTimeAuthority {
    pub crank: CrankSyncState,
    pub phase: PhaseSyncState,
    pub absolute: AbsoluteTimeAuthority,
    pub confidence_x1000: u16,
    pub sync_loss_count: u16,
}

impl Default for EngineTimeAuthority {
    fn default() -> Self {
        Self::none()
    }
}

impl EngineTimeAuthority {
    pub const MAX_CONFIDENCE_X1000: u16 = 1000;

    pub const fn none() -> Self {
        Self {
            crank: CrankSyncState::NoSignal,
            phase: PhaseSyncState::Unknown,
            absolute: AbsoluteTimeAuthority::None,
            confidence_x1000: 0,
            sync_loss_count: 0,
        }
    }

    pub const fn new(
        crank: CrankSyncState,
        phase: PhaseSyncState,
        absolute: AbsoluteTimeAuthority,
        confidence_x1000: u16,
        sync_loss_count: u16,
    ) -> Self {
        Self {
            crank,
            phase,
            absolute,
            confidence_x1000,
            sync_loss_count,
        }
    }

    pub const fn has_primary_lock(self) -> bool {
        matches!(self.crank, CrankSyncState::PrimaryLocked)
    }

    pub const fn has_absolute_timing(self) -> bool {
        !matches!(self.absolute, AbsoluteTimeAuthority::None)
    }

    pub const fn is_certified_profile(self) -> bool {
        matches!(self.absolute, AbsoluteTimeAuthority::CertifiedProfile)
    }

    /// Lossy compatibility view for code that still consumes `SyncState`.
    ///
    /// This must not be used by itself to authorize sequential outputs.
    pub const fn compatibility_summary(self) -> SyncState {
        if !self.has_primary_lock() {
            return SyncState::Unsynced;
        }

        if matches!(self.phase, PhaseSyncState::Unknown) || !self.has_absolute_timing() {
            return SyncState::Syncing;
        }

        SyncState::Synced
    }

    pub const fn validate(self) -> Result<(), EngineTimeAuthorityError> {
        if self.confidence_x1000 > Self::MAX_CONFIDENCE_X1000 {
            return Err(EngineTimeAuthorityError::ConfidenceOutOfRange);
        }

        if !self.has_primary_lock() && self.has_absolute_timing() {
            return Err(EngineTimeAuthorityError::AbsoluteTimingWithoutPrimaryLock);
        }

        Ok(())
    }
}

/// Validation failures for engine-time authority snapshots.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EngineTimeAuthorityError {
    ConfidenceOutOfRange,
    AbsoluteTimingWithoutPrimaryLock,
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
    fn quantity_newtypes_round_trip() {
        assert_eq!(Rpm::new(1234).get(), 1234);
        assert_eq!(Micros::new(42).get(), 42);
        assert_eq!(Ticks::new(7).get(), 7);
        assert_eq!(Kpa10::new(987).get(), 987);
        assert_eq!(Percent::new(88).get(), 88);
        assert_eq!(PulseWidthUs::new(1500).get(), 1500);
        assert_eq!(DwellUs::new(2500).get(), 2500);
        assert_eq!(Degrees10::new(-125).get(), -125);
        assert_eq!(Lambda100::new(101).get(), 101);
    }

    #[test]
    fn id_types_support_comparison() {
        assert!(CylinderId::new(1) < CylinderId::new(2));
        assert!(ChannelId::new(3) < ChannelId::new(4));
    }

    #[test]
    fn enums_expose_the_expected_default_states() {
        assert_eq!(SyncState::default(), SyncState::Unsynced);
        assert_eq!(EnginePhase::default(), EnginePhase::Off);
        assert_eq!(ControlMode::default(), ControlMode::OpenLoop);
        assert_eq!(FaultSeverity::default(), FaultSeverity::Info);
        assert_eq!(FaultCode::default(), FaultCode::None);
        assert_eq!(CancelReason::default(), CancelReason::Manual);
        assert_eq!(CommitPolicy::default(), CommitPolicy::Immediate);
        assert_eq!(CrankSyncState::default(), CrankSyncState::NoSignal);
        assert_eq!(PhaseSyncState::default(), PhaseSyncState::Unknown);
        assert_eq!(
            AbsoluteTimeAuthority::default(),
            AbsoluteTimeAuthority::None
        );
        assert_eq!(EngineTimeAuthority::default(), EngineTimeAuthority::none());
    }

    #[test]
    fn enums_cover_a_representable_state_space() {
        let states = [SyncState::Unsynced, SyncState::Syncing, SyncState::Synced];
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

        let crank_states = [
            CrankSyncState::NoSignal,
            CrankSyncState::PrimarySearching,
            CrankSyncState::PrimaryLocked,
            CrankSyncState::SyncLost,
        ];
        assert_eq!(crank_states.len(), 4);

        let phase_states = [
            PhaseSyncState::Unknown,
            PhaseSyncState::CrankOnly360,
            PhaseSyncState::CamObserved720,
            PhaseSyncState::CamValidated720,
        ];
        assert_eq!(phase_states.len(), 4);

        let absolute_authorities = [
            AbsoluteTimeAuthority::None,
            AbsoluteTimeAuthority::GeometryOnly,
            AbsoluteTimeAuthority::ExpertManual,
            AbsoluteTimeAuthority::CommunityProfile,
            AbsoluteTimeAuthority::CertifiedProfile,
            AbsoluteTimeAuthority::BenchLearned,
        ];
        assert_eq!(absolute_authorities.len(), 6);
    }

    #[test]
    fn engine_time_authority_maps_to_legacy_sync_summary() {
        assert_eq!(
            EngineTimeAuthority::none().compatibility_summary(),
            SyncState::Unsynced
        );
        assert_eq!(
            EngineTimeAuthority::new(
                CrankSyncState::PrimarySearching,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::None,
                100,
                0,
            )
            .compatibility_summary(),
            SyncState::Unsynced
        );
        assert_eq!(
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::None,
                500,
                0,
            )
            .compatibility_summary(),
            SyncState::Syncing
        );
        assert_eq!(
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                900,
                0,
            )
            .compatibility_summary(),
            SyncState::Synced
        );
    }

    #[test]
    fn engine_time_authority_validation_rejects_impossible_snapshots() {
        assert_eq!(
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            )
            .validate(),
            Ok(())
        );

        assert_eq!(
            EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000 + 1,
                0,
            )
            .validate(),
            Err(EngineTimeAuthorityError::ConfidenceOutOfRange)
        );

        assert_eq!(
            EngineTimeAuthority::new(
                CrankSyncState::SyncLost,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::CertifiedProfile,
                0,
                1,
            )
            .validate(),
            Err(EngineTimeAuthorityError::AbsoluteTimingWithoutPrimaryLock)
        );
    }
}
