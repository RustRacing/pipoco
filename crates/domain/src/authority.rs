use crate::SyncState;

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

    pub const fn try_new(
        crank: CrankSyncState,
        phase: PhaseSyncState,
        absolute: AbsoluteTimeAuthority,
        confidence_x1000: u16,
        sync_loss_count: u16,
    ) -> Result<Self, EngineTimeAuthorityError> {
        let authority = Self::new(crank, phase, absolute, confidence_x1000, sync_loss_count);
        match authority.validate() {
            Ok(()) => Ok(authority),
            Err(error) => Err(error),
        }
    }

    pub const fn has_primary_lock(self) -> bool {
        matches!(self.crank, CrankSyncState::PrimaryLocked)
    }

    pub const fn has_absolute_timing(self) -> bool {
        !matches!(self.absolute, AbsoluteTimeAuthority::None)
    }

    /// Lossy compatibility view for code that still consumes `SyncState`.
    ///
    /// This must not be used by itself to authorize sequential outputs.
    pub const fn compatibility_summary(self) -> SyncState {
        if !self.has_primary_lock() {
            return SyncState::Unsynced;
        }

        if matches!(self.phase, PhaseSyncState::Unknown) || !self.has_absolute_timing() {
            return SyncState::Provisional;
        }

        SyncState::Locked { cam_ref: false }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn enums_expose_the_expected_default_states() {
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
            SyncState::Provisional
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
            SyncState::Locked { cam_ref: false }
        );
    }

    #[test]
    fn engine_time_authority_validation_rejects_impossible_snapshots() {
        assert_eq!(
            EngineTimeAuthority::try_new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            ),
            Ok(EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000,
                0,
            ))
        );

        assert_eq!(
            EngineTimeAuthority::try_new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::GeometryOnly,
                EngineTimeAuthority::MAX_CONFIDENCE_X1000 + 1,
                0,
            ),
            Err(EngineTimeAuthorityError::ConfidenceOutOfRange)
        );

        assert_eq!(
            EngineTimeAuthority::try_new(
                CrankSyncState::SyncLost,
                PhaseSyncState::Unknown,
                AbsoluteTimeAuthority::CertifiedProfile,
                0,
                1,
            ),
            Err(EngineTimeAuthorityError::AbsoluteTimingWithoutPrimaryLock)
        );
    }
}
