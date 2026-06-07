use ecu_domain::{AbsoluteTimeAuthority, EngineTimeAuthority, PhaseSyncState};

fn absolute_authorizes_full_sequential(absolute: AbsoluteTimeAuthority) -> bool {
    matches!(
        absolute,
        AbsoluteTimeAuthority::ExpertManual
            | AbsoluteTimeAuthority::CommunityProfile
            | AbsoluteTimeAuthority::CertifiedProfile
            | AbsoluteTimeAuthority::BenchLearned
    )
}

/// Runtime gate for outputs that require known 720-degree phase.
///
/// Legacy `SyncState::Synced` summaries are intentionally not enough here.
pub fn runtime_full_sequential_authorized(authority: EngineTimeAuthority) -> bool {
    authority.validate().is_ok()
        && authority.has_primary_lock()
        && matches!(authority.phase, PhaseSyncState::CamValidated720)
        && absolute_authorizes_full_sequential(authority.absolute)
}
