use ecu_domain::{CrankSyncState, Degrees10, EngineTimeAuthority, PhaseSyncState, Rpm};

/// Compact shared sync-state surface for the common board path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SplitSyncState {
    #[default]
    NoSignal,
    Unsynced,
    CrankSynced,
    FullSequentialAuthorized,
    SyncLost,
}

impl SplitSyncState {
    pub const fn from_authority(authority: EngineTimeAuthority) -> Self {
        match authority.crank {
            CrankSyncState::NoSignal => Self::NoSignal,
            CrankSyncState::PrimarySearching => Self::Unsynced,
            CrankSyncState::SyncLost => Self::SyncLost,
            CrankSyncState::PrimaryLocked => {
                if matches!(authority.phase, PhaseSyncState::CamValidated720) {
                    Self::FullSequentialAuthorized
                } else {
                    Self::CrankSynced
                }
            }
        }
    }

    pub const fn from_trigger_sync(previous: Self, synced: bool) -> Self {
        if synced {
            match previous {
                Self::NoSignal | Self::Unsynced | Self::SyncLost => Self::CrankSynced,
                other => other,
            }
        } else {
            match previous {
                Self::NoSignal | Self::Unsynced => Self::Unsynced,
                Self::CrankSynced | Self::FullSequentialAuthorized => Self::SyncLost,
                Self::SyncLost => Self::SyncLost,
            }
        }
    }
}

/// Minimal live trigger update consumed by board sensor live sinks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitLiveTriggerEvent {
    rpm: Rpm,
    angle_x10: Degrees10,
    synced: bool,
}

impl SplitLiveTriggerEvent {
    pub const fn new(rpm: Rpm, angle_x10: Degrees10, synced: bool) -> Self {
        Self {
            rpm,
            angle_x10,
            synced,
        }
    }

    pub const fn rpm(self) -> Rpm {
        self.rpm
    }

    pub const fn angle_x10(self) -> Degrees10 {
        self.angle_x10
    }

    pub const fn synced(self) -> bool {
        self.synced
    }
}

/// Last live engine inputs shared by board sensor and control-input plumbing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitLiveInputs {
    rpm: Rpm,
    angle_x10: Degrees10,
    trigger_synced: bool,
    sync_state: SplitSyncState,
}

impl SplitLiveInputs {
    pub const fn new() -> Self {
        Self {
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(0),
            trigger_synced: false,
            sync_state: SplitSyncState::NoSignal,
        }
    }

    pub fn apply_event(&mut self, event: SplitLiveTriggerEvent) {
        self.rpm = event.rpm();
        self.angle_x10 = event.angle_x10();
        self.trigger_synced = event.synced();
        self.sync_state = SplitSyncState::from_trigger_sync(self.sync_state, event.synced());
    }

    pub const fn rpm(self) -> Rpm {
        self.rpm
    }

    pub const fn angle_x10(self) -> Degrees10 {
        self.angle_x10
    }

    pub const fn trigger_synced(self) -> bool {
        self.trigger_synced
    }

    pub const fn sync_state(self) -> SplitSyncState {
        self.sync_state
    }
}

impl Default for SplitLiveInputs {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn live_inputs_start_unsynced_and_zeroed() {
        let inputs = SplitLiveInputs::new();

        assert_eq!(inputs.rpm(), Rpm::new(0));
        assert_eq!(inputs.angle_x10(), Degrees10::new(0));
        assert!(!inputs.trigger_synced());
        assert_eq!(inputs.sync_state(), SplitSyncState::NoSignal);
    }

    #[test]
    fn live_inputs_update_sync_state_from_trigger_events() {
        let mut inputs = SplitLiveInputs::new();

        inputs.apply_event(SplitLiveTriggerEvent::new(
            Rpm::new(1_500),
            Degrees10::new(120),
            false,
        ));
        assert_eq!(inputs.sync_state(), SplitSyncState::Unsynced);

        inputs.apply_event(SplitLiveTriggerEvent::new(
            Rpm::new(1_500),
            Degrees10::new(120),
            true,
        ));
        assert_eq!(inputs.sync_state(), SplitSyncState::CrankSynced);

        inputs.apply_event(SplitLiveTriggerEvent::new(
            Rpm::new(1_500),
            Degrees10::new(120),
            false,
        ));
        assert_eq!(inputs.sync_state(), SplitSyncState::SyncLost);
    }

    #[test]
    fn sync_state_maps_authority_snapshots() {
        assert_eq!(
            SplitSyncState::from_authority(EngineTimeAuthority::none()),
            SplitSyncState::NoSignal
        );
        assert_eq!(
            SplitSyncState::from_authority(EngineTimeAuthority::new(
                CrankSyncState::PrimarySearching,
                PhaseSyncState::Unknown,
                ecu_domain::AbsoluteTimeAuthority::None,
                0,
                0,
            )),
            SplitSyncState::Unsynced
        );
        assert_eq!(
            SplitSyncState::from_authority(EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CrankOnly360,
                ecu_domain::AbsoluteTimeAuthority::GeometryOnly,
                900,
                0,
            )),
            SplitSyncState::CrankSynced
        );
        assert_eq!(
            SplitSyncState::from_authority(EngineTimeAuthority::new(
                CrankSyncState::PrimaryLocked,
                PhaseSyncState::CamValidated720,
                ecu_domain::AbsoluteTimeAuthority::GeometryOnly,
                900,
                0,
            )),
            SplitSyncState::FullSequentialAuthorized
        );
        assert_eq!(
            SplitSyncState::from_authority(EngineTimeAuthority::new(
                CrankSyncState::SyncLost,
                PhaseSyncState::Unknown,
                ecu_domain::AbsoluteTimeAuthority::None,
                0,
                1,
            )),
            SplitSyncState::SyncLost
        );
    }
}
