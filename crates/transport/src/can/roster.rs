//! Module heartbeat, roster registry, and event log (split from can monolith; review 006).

use crate::Message;

pub const CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS: u32 = 5;
pub const CAN_HEARTBEAT_TIMEOUT_MS: u32 = 100;
pub const CAN_MODULE_ROSTER_CAPACITY: usize = 16;
pub const CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY: usize = CAN_MODULE_ROSTER_CAPACITY * 2;
pub const CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY: usize = CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY * 2;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleHeartbeatStatus {
    Ok,
    Warning,
    Error,
    Unknown(u8),
}

impl CanModuleHeartbeatStatus {
    pub const fn from_raw(status: u8) -> Self {
        match status {
            0 => Self::Ok,
            1 => Self::Warning,
            2 => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleHeartbeatState {
    Starting,
    Alive,
    Degraded,
    Unavailable,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleHeartbeatContract {
    pub node_id: u8,
    pub uptime_seconds: u32,
    pub status: CanModuleHeartbeatStatus,
    pub error_count: u16,
    pub cpu_usage: u8,
    pub state: CanModuleHeartbeatState,
}

impl CanModuleHeartbeatContract {
    pub const fn from_fields(
        node_id: u8,
        uptime_seconds: u32,
        status: u8,
        error_count: u16,
        cpu_usage: u8,
        stale: bool,
    ) -> Self {
        let decoded_status = CanModuleHeartbeatStatus::from_raw(status);
        let state = if stale {
            CanModuleHeartbeatState::Unavailable
        } else if uptime_seconds <= CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS {
            CanModuleHeartbeatState::Starting
        } else if matches!(decoded_status, CanModuleHeartbeatStatus::Ok) && error_count == 0 {
            CanModuleHeartbeatState::Alive
        } else {
            CanModuleHeartbeatState::Degraded
        };

        Self {
            node_id,
            uptime_seconds,
            status: decoded_status,
            error_count,
            cpu_usage,
            state,
        }
    }

    pub fn from_message(message: &Message, stale: bool) -> Option<Self> {
        match *message {
            Message::Heartbeat {
                node_id,
                uptime_seconds,
                status,
                error_count,
                cpu_usage,
            } => Some(Self::from_fields(
                node_id,
                uptime_seconds,
                status,
                error_count,
                cpu_usage,
                stale,
            )),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanHeartbeatObservationPolicy {
    pub timeout_ms: u32,
}

impl Default for CanHeartbeatObservationPolicy {
    fn default() -> Self {
        Self {
            timeout_ms: CAN_HEARTBEAT_TIMEOUT_MS,
        }
    }
}

impl CanHeartbeatObservationPolicy {
    pub const fn is_stale(self, age_ms: Option<u32>) -> bool {
        match age_ms {
            Some(age_ms) => age_ms > self.timeout_ms,
            None => true,
        }
    }

    pub fn observe_message(
        self,
        message: &Message,
        age_ms: Option<u32>,
    ) -> Option<CanModuleHeartbeatContract> {
        CanModuleHeartbeatContract::from_message(message, self.is_stale(age_ms))
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleAvailabilityTransition {
    NoChange(CanModuleHeartbeatState),
    Changed {
        from: CanModuleHeartbeatState,
        to: CanModuleHeartbeatState,
    },
}

impl CanModuleAvailabilityTransition {
    pub fn between(previous: CanModuleHeartbeatState, current: CanModuleHeartbeatState) -> Self {
        if previous == current {
            Self::NoChange(current)
        } else {
            Self::Changed {
                from: previous,
                to: current,
            }
        }
    }

    pub const fn current(self) -> CanModuleHeartbeatState {
        match self {
            Self::NoChange(state) => state,
            Self::Changed { to, .. } => to,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanRemoteModuleFallbackPolicy {
    RemoteDataTrusted,
    HoldLastKnownGood,
    RequireLocalFallback,
}

impl CanRemoteModuleFallbackPolicy {
    pub const fn from_state(state: CanModuleHeartbeatState) -> Self {
        match state {
            CanModuleHeartbeatState::Alive => Self::RemoteDataTrusted,
            CanModuleHeartbeatState::Starting | CanModuleHeartbeatState::Degraded => {
                Self::HoldLastKnownGood
            }
            CanModuleHeartbeatState::Unavailable => Self::RequireLocalFallback,
        }
    }

    pub const fn from_contract(contract: CanModuleHeartbeatContract) -> Self {
        Self::from_state(contract.state)
    }

    pub const fn from_transition(transition: CanModuleAvailabilityTransition) -> Self {
        Self::from_state(transition.current())
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEntry {
    pub heartbeat: CanModuleHeartbeatContract,
    pub availability_transition: CanModuleAvailabilityTransition,
    pub fallback_policy: CanRemoteModuleFallbackPolicy,
}

impl CanModuleRosterEntry {
    pub const fn from_contract_and_transition(
        heartbeat: CanModuleHeartbeatContract,
        availability_transition: CanModuleAvailabilityTransition,
    ) -> Self {
        Self {
            heartbeat,
            availability_transition,
            fallback_policy: CanRemoteModuleFallbackPolicy::from_transition(
                availability_transition,
            ),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterSnapshotEntry {
    pub module: CanModuleRosterEntry,
    pub heartbeat_age_ms: u32,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterSnapshot {
    pub len: usize,
    pub entries: [Option<CanModuleRosterSnapshotEntry>; CAN_MODULE_ROSTER_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterSummary {
    pub total_count: usize,
    pub alive_count: usize,
    pub starting_count: usize,
    pub degraded_count: usize,
    pub unavailable_count: usize,
    pub trusted_count: usize,
    pub hold_last_known_good_count: usize,
    pub require_local_fallback_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanModuleRosterStartupReadinessState {
    #[default]
    NoRemoteModules,
    ReadyForTrust,
    WaitingForStartup,
    HoldingLastKnownGood,
    BlockedByUnavailable,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterStartupReadiness {
    pub state: CanModuleRosterStartupReadinessState,
    pub total_module_count: usize,
    pub ready_for_trust_module_count: usize,
    pub waiting_on_startup_module_count: usize,
    pub degraded_hold_last_known_good_module_count: usize,
    pub blocked_by_unavailable_module_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanModuleRosterShutdownCoordinationState {
    #[default]
    NoRemoteModules,
    NoShutdownNeeded,
    CoordinatedShutdownRecommended,
    LocalTakeoverRequired,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterShutdownCoordination {
    pub state: CanModuleRosterShutdownCoordinationState,
    pub total_module_count: usize,
    pub degraded_module_count: usize,
    pub unavailable_module_count: usize,
    pub require_local_fallback_module_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterChangeDigest {
    pub changed_module_count: usize,
    pub became_alive_count: usize,
    pub became_degraded_count: usize,
    pub became_unavailable_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterChangeSetEntry {
    pub node_id: u8,
    pub previous_state: Option<CanModuleHeartbeatState>,
    pub current_state: Option<CanModuleHeartbeatState>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleRosterEventMeaning {
    FirstObservation(CanModuleHeartbeatState),
    RecoveryToAlive {
        from: CanModuleHeartbeatState,
    },
    TransitionToDegraded {
        from: CanModuleHeartbeatState,
    },
    TimedOutToUnavailable,
    Other {
        previous_state: Option<CanModuleHeartbeatState>,
        current_state: Option<CanModuleHeartbeatState>,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEventInterpretation {
    pub meaning: CanModuleRosterEventMeaning,
    pub fallback_impact: CanRemoteModuleFallbackPolicy,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterChangeSet {
    pub len: usize,
    pub entries: [Option<CanModuleRosterChangeSetEntry>; CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEventLog {
    pub len: usize,
    pub dropped_count: u32,
    pub entries: [Option<CanModuleRosterChangeSetEntry>; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterEventLogWatermark {
    pub retained_event_count: usize,
    pub dropped_event_count: u32,
    pub total_written_event_count: u32,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEventLogDelta {
    pub previous_watermark: CanModuleRosterEventLogWatermark,
    pub current_watermark: CanModuleRosterEventLogWatermark,
    pub retained_new_event_count: usize,
    pub dropped_unread_event_count: u32,
    pub entries: [Option<CanModuleRosterChangeSetEntry>; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterEventDeltaSummary {
    pub retained_unread_event_count: usize,
    pub dropped_unread_event_count: u32,
    pub first_observation_count: usize,
    pub recovery_to_alive_count: usize,
    pub transition_to_degraded_count: usize,
    pub timed_out_to_unavailable_count: usize,
    pub trust_impact_count: usize,
    pub hold_last_known_good_impact_count: usize,
    pub require_local_fallback_impact_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterLatestUnreadEventEntry {
    pub node_id: u8,
    pub interpretation: CanModuleRosterEventInterpretation,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterLatestUnreadEventSet {
    pub len: usize,
    pub entries: [Option<CanModuleRosterLatestUnreadEventEntry>; CAN_MODULE_ROSTER_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterLatestUnreadSummary {
    pub latest_unread_node_count: usize,
    pub first_observation_count: usize,
    pub recovery_to_alive_count: usize,
    pub transition_to_degraded_count: usize,
    pub timed_out_to_unavailable_count: usize,
    pub trust_impact_count: usize,
    pub hold_last_known_good_impact_count: usize,
    pub require_local_fallback_impact_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterLatestUnreadShutdownImpactSummary {
    pub latest_unread_node_count: usize,
    pub no_shutdown_needed_count: usize,
    pub coordinated_shutdown_recommended_count: usize,
    pub local_takeover_required_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterLatestUnreadStartupReadinessSummary {
    pub latest_unread_node_count: usize,
    pub ready_for_trust_count: usize,
    pub waiting_on_startup_count: usize,
    pub holding_last_known_good_count: usize,
    pub blocked_by_unavailable_count: usize,
}

impl Default for CanModuleRosterSnapshot {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for CanModuleRosterChangeSet {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for CanModuleRosterEventLog {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for CanModuleRosterEventLogDelta {
    fn default() -> Self {
        Self {
            previous_watermark: CanModuleRosterEventLogWatermark::default(),
            current_watermark: CanModuleRosterEventLogWatermark::default(),
            retained_new_event_count: 0,
            dropped_unread_event_count: 0,
            entries: [None; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
        }
    }
}

impl Default for CanModuleRosterLatestUnreadEventSet {
    fn default() -> Self {
        Self::new()
    }
}

impl CanModuleRosterEventDeltaSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(_) => self.first_observation_count += 1,
            CanModuleRosterEventMeaning::RecoveryToAlive { .. } => {
                self.recovery_to_alive_count += 1
            }
            CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                self.transition_to_degraded_count += 1;
            }
            CanModuleRosterEventMeaning::TimedOutToUnavailable => {
                self.timed_out_to_unavailable_count += 1;
            }
            CanModuleRosterEventMeaning::Other { .. } => {}
        }
        match interpretation.fallback_impact {
            CanRemoteModuleFallbackPolicy::RemoteDataTrusted => self.trust_impact_count += 1,
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood => {
                self.hold_last_known_good_impact_count += 1;
            }
            CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
                self.require_local_fallback_impact_count += 1;
            }
        }
    }
}

impl CanModuleRosterLatestUnreadEventSet {
    pub const fn new() -> Self {
        Self {
            len: 0,
            entries: [None; CAN_MODULE_ROSTER_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterLatestUnreadEventEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterLatestUnreadEventEntry> {
        self.entries
            .iter()
            .copied()
            .flatten()
            .find(|entry| entry.node_id == node_id)
    }

    pub fn summary(&self) -> CanModuleRosterLatestUnreadSummary {
        let mut summary = CanModuleRosterLatestUnreadSummary::default();
        let mut index = 0usize;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation);
            }
            index += 1;
        }
        summary
    }

    pub fn shutdown_impact_summary(&self) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        let mut summary = CanModuleRosterLatestUnreadShutdownImpactSummary::default();
        let mut index = 0usize;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation);
            }
            index += 1;
        }
        summary
    }

    pub fn startup_readiness_summary(&self) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        let mut summary = CanModuleRosterLatestUnreadStartupReadinessSummary::default();
        let mut index = 0usize;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation);
            }
            index += 1;
        }
        summary
    }
}

impl CanModuleRosterLatestUnreadSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        self.latest_unread_node_count += 1;
        match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(_) => self.first_observation_count += 1,
            CanModuleRosterEventMeaning::RecoveryToAlive { .. } => {
                self.recovery_to_alive_count += 1
            }
            CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                self.transition_to_degraded_count += 1;
            }
            CanModuleRosterEventMeaning::TimedOutToUnavailable => {
                self.timed_out_to_unavailable_count += 1;
            }
            CanModuleRosterEventMeaning::Other { .. } => {}
        }
        match interpretation.fallback_impact {
            CanRemoteModuleFallbackPolicy::RemoteDataTrusted => self.trust_impact_count += 1,
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood => {
                self.hold_last_known_good_impact_count += 1;
            }
            CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
                self.require_local_fallback_impact_count += 1;
            }
        }
    }
}

impl CanModuleRosterLatestUnreadShutdownImpactSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        self.latest_unread_node_count += 1;
        match shutdown_coordination_from_interpretation(interpretation) {
            CanModuleRosterShutdownCoordinationState::NoRemoteModules => {}
            CanModuleRosterShutdownCoordinationState::NoShutdownNeeded => {
                self.no_shutdown_needed_count += 1;
            }
            CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended => {
                self.coordinated_shutdown_recommended_count += 1;
            }
            CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired => {
                self.local_takeover_required_count += 1;
            }
        }
    }
}

impl CanModuleRosterLatestUnreadStartupReadinessSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        self.latest_unread_node_count += 1;
        match startup_readiness_from_interpretation(interpretation) {
            CanModuleRosterStartupReadinessState::NoRemoteModules => {}
            CanModuleRosterStartupReadinessState::ReadyForTrust => {
                self.ready_for_trust_count += 1;
            }
            CanModuleRosterStartupReadinessState::WaitingForStartup => {
                self.waiting_on_startup_count += 1;
            }
            CanModuleRosterStartupReadinessState::HoldingLastKnownGood => {
                self.holding_last_known_good_count += 1;
            }
            CanModuleRosterStartupReadinessState::BlockedByUnavailable => {
                self.blocked_by_unavailable_count += 1;
            }
        }
    }
}

const fn shutdown_coordination_from_interpretation(
    interpretation: CanModuleRosterEventInterpretation,
) -> CanModuleRosterShutdownCoordinationState {
    match interpretation.fallback_impact {
        CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
            CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired
        }
        CanRemoteModuleFallbackPolicy::RemoteDataTrusted
        | CanRemoteModuleFallbackPolicy::HoldLastKnownGood => match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Degraded)
            | CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended
            }
            CanModuleRosterEventMeaning::FirstObservation(_)
            | CanModuleRosterEventMeaning::RecoveryToAlive { .. }
            | CanModuleRosterEventMeaning::TimedOutToUnavailable
            | CanModuleRosterEventMeaning::Other { .. } => {
                CanModuleRosterShutdownCoordinationState::NoShutdownNeeded
            }
        },
    }
}

const fn startup_readiness_from_interpretation(
    interpretation: CanModuleRosterEventInterpretation,
) -> CanModuleRosterStartupReadinessState {
    match interpretation.fallback_impact {
        CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
            CanModuleRosterStartupReadinessState::BlockedByUnavailable
        }
        CanRemoteModuleFallbackPolicy::RemoteDataTrusted => {
            CanModuleRosterStartupReadinessState::ReadyForTrust
        }
        CanRemoteModuleFallbackPolicy::HoldLastKnownGood => match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Starting) => {
                CanModuleRosterStartupReadinessState::WaitingForStartup
            }
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Degraded)
            | CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                CanModuleRosterStartupReadinessState::HoldingLastKnownGood
            }
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Alive)
            | CanModuleRosterEventMeaning::RecoveryToAlive { .. } => {
                CanModuleRosterStartupReadinessState::ReadyForTrust
            }
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Unavailable)
            | CanModuleRosterEventMeaning::TimedOutToUnavailable => {
                CanModuleRosterStartupReadinessState::BlockedByUnavailable
            }
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Starting),
                ..
            } => CanModuleRosterStartupReadinessState::WaitingForStartup,
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Degraded),
                ..
            } => CanModuleRosterStartupReadinessState::HoldingLastKnownGood,
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Alive),
                ..
            } => CanModuleRosterStartupReadinessState::ReadyForTrust,
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Unavailable) | None,
                ..
            } => CanModuleRosterStartupReadinessState::BlockedByUnavailable,
        },
    }
}

impl CanModuleRosterEventLogDelta {
    pub fn summary(&self) -> CanModuleRosterEventDeltaSummary {
        let mut summary = CanModuleRosterEventDeltaSummary {
            retained_unread_event_count: self.retained_new_event_count,
            dropped_unread_event_count: self.dropped_unread_event_count,
            ..CanModuleRosterEventDeltaSummary::default()
        };
        let mut index = 0usize;
        while index < self.retained_new_event_count {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation());
            }
            index += 1;
        }
        summary
    }

    pub fn latest_per_node(&self) -> CanModuleRosterLatestUnreadEventSet {
        let mut latest = CanModuleRosterLatestUnreadEventSet::new();
        let mut index = 0usize;
        while index < self.retained_new_event_count {
            if let Some(entry) = self.entries[index] {
                let latest_entry = CanModuleRosterLatestUnreadEventEntry {
                    node_id: entry.node_id,
                    interpretation: entry.interpretation(),
                };
                if let Some(existing_index) = latest.entries[..latest.len].iter().position(
                    |slot| matches!(slot, Some(existing) if existing.node_id == entry.node_id),
                ) {
                    latest.entries[existing_index] = Some(latest_entry);
                } else if latest.len < CAN_MODULE_ROSTER_CAPACITY {
                    latest.entries[latest.len] = Some(latest_entry);
                    latest.len += 1;
                }
            }
            index += 1;
        }
        latest
    }

    pub fn latest_unread_summary(&self) -> CanModuleRosterLatestUnreadSummary {
        self.latest_per_node().summary()
    }

    pub fn latest_unread_shutdown_impact_summary(
        &self,
    ) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        self.latest_per_node().shutdown_impact_summary()
    }

    pub fn latest_unread_startup_readiness_summary(
        &self,
    ) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        self.latest_per_node().startup_readiness_summary()
    }
}

impl CanModuleRosterSnapshot {
    pub const fn new() -> Self {
        Self {
            len: 0,
            entries: [None; CAN_MODULE_ROSTER_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterSnapshotEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterSnapshotEntry> {
        self.entries
            .iter()
            .copied()
            .flatten()
            .find(|entry| entry.module.heartbeat.node_id == node_id)
    }

    pub fn summary(&self) -> CanModuleRosterSummary {
        let mut summary = CanModuleRosterSummary::default();
        let mut index = 0;

        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.total_count += 1;
                match entry.module.heartbeat.state {
                    CanModuleHeartbeatState::Alive => summary.alive_count += 1,
                    CanModuleHeartbeatState::Starting => summary.starting_count += 1,
                    CanModuleHeartbeatState::Degraded => summary.degraded_count += 1,
                    CanModuleHeartbeatState::Unavailable => summary.unavailable_count += 1,
                }
                match entry.module.fallback_policy {
                    CanRemoteModuleFallbackPolicy::RemoteDataTrusted => {
                        summary.trusted_count += 1;
                    }
                    CanRemoteModuleFallbackPolicy::HoldLastKnownGood => {
                        summary.hold_last_known_good_count += 1;
                    }
                    CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
                        summary.require_local_fallback_count += 1;
                    }
                }
            }
            index += 1;
        }

        summary
    }

    pub fn startup_readiness(&self) -> CanModuleRosterStartupReadiness {
        self.summary().startup_readiness()
    }

    pub fn shutdown_coordination(&self) -> CanModuleRosterShutdownCoordination {
        self.summary().shutdown_coordination()
    }

    pub fn change_digest_since(
        &self,
        previous: CanModuleRosterSnapshot,
    ) -> CanModuleRosterChangeDigest {
        self.change_set_since(previous).digest()
    }

    pub fn change_set_since(&self, previous: CanModuleRosterSnapshot) -> CanModuleRosterChangeSet {
        let mut change_set = CanModuleRosterChangeSet::new();
        let mut processed: [u8; CAN_MODULE_ROSTER_CAPACITY] = [0; CAN_MODULE_ROSTER_CAPACITY];
        let mut processed_len = 0usize;
        let mut index = 0usize;

        while index < self.len {
            if let Some(current) = self.entries[index] {
                let previous_state = previous
                    .get(current.module.heartbeat.node_id)
                    .map(|entry| entry.module.heartbeat.state);
                change_set.push_change(
                    current.module.heartbeat.node_id,
                    previous_state,
                    Some(current.module.heartbeat.state),
                );
                processed[processed_len] = current.module.heartbeat.node_id;
                processed_len += 1;
            }
            index += 1;
        }

        index = 0;
        while index < previous.len {
            if let Some(old) = previous.entries[index] {
                let already_processed =
                    processed[..processed_len].contains(&old.module.heartbeat.node_id);
                if !already_processed {
                    change_set.push_change(
                        old.module.heartbeat.node_id,
                        Some(old.module.heartbeat.state),
                        None,
                    );
                }
            }
            index += 1;
        }

        change_set
    }
}

impl CanModuleRosterSummary {
    pub const fn startup_readiness(self) -> CanModuleRosterStartupReadiness {
        let ready_for_trust_module_count = self.trusted_count;
        let waiting_on_startup_module_count = self.starting_count;
        let degraded_hold_last_known_good_module_count = self.degraded_count;
        let blocked_by_unavailable_module_count = self.unavailable_count;
        let state = if self.total_count == 0 {
            CanModuleRosterStartupReadinessState::NoRemoteModules
        } else if blocked_by_unavailable_module_count > 0 {
            CanModuleRosterStartupReadinessState::BlockedByUnavailable
        } else if degraded_hold_last_known_good_module_count > 0 {
            CanModuleRosterStartupReadinessState::HoldingLastKnownGood
        } else if waiting_on_startup_module_count > 0 {
            CanModuleRosterStartupReadinessState::WaitingForStartup
        } else {
            CanModuleRosterStartupReadinessState::ReadyForTrust
        };

        CanModuleRosterStartupReadiness {
            state,
            total_module_count: self.total_count,
            ready_for_trust_module_count,
            waiting_on_startup_module_count,
            degraded_hold_last_known_good_module_count,
            blocked_by_unavailable_module_count,
        }
    }

    pub const fn shutdown_coordination(self) -> CanModuleRosterShutdownCoordination {
        let degraded_module_count = self.degraded_count;
        let unavailable_module_count = self.unavailable_count;
        let require_local_fallback_module_count = self.require_local_fallback_count;
        let state = if self.total_count == 0 {
            CanModuleRosterShutdownCoordinationState::NoRemoteModules
        } else if unavailable_module_count > 0 || require_local_fallback_module_count > 0 {
            CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired
        } else if degraded_module_count > 0 {
            CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended
        } else {
            CanModuleRosterShutdownCoordinationState::NoShutdownNeeded
        };

        CanModuleRosterShutdownCoordination {
            state,
            total_module_count: self.total_count,
            degraded_module_count,
            unavailable_module_count,
            require_local_fallback_module_count,
        }
    }
}

impl CanModuleRosterChangeDigest {
    fn observe_transition(
        &mut self,
        previous: Option<CanModuleHeartbeatState>,
        current: Option<CanModuleHeartbeatState>,
    ) {
        if previous == current {
            return;
        }

        self.changed_module_count += 1;
        match current {
            Some(CanModuleHeartbeatState::Alive) => self.became_alive_count += 1,
            Some(CanModuleHeartbeatState::Degraded) => self.became_degraded_count += 1,
            Some(CanModuleHeartbeatState::Unavailable) => self.became_unavailable_count += 1,
            Some(CanModuleHeartbeatState::Starting) | None => {}
        }
    }
}

impl CanModuleRosterChangeSetEntry {
    pub const fn meaning(self) -> CanModuleRosterEventMeaning {
        match (self.previous_state, self.current_state) {
            (None, Some(state)) => CanModuleRosterEventMeaning::FirstObservation(state),
            (Some(from), Some(CanModuleHeartbeatState::Alive))
                if matches!(
                    from,
                    CanModuleHeartbeatState::Starting
                        | CanModuleHeartbeatState::Degraded
                        | CanModuleHeartbeatState::Unavailable
                ) =>
            {
                CanModuleRosterEventMeaning::RecoveryToAlive { from }
            }
            (Some(from), Some(CanModuleHeartbeatState::Degraded)) => {
                CanModuleRosterEventMeaning::TransitionToDegraded { from }
            }
            (Some(_), Some(CanModuleHeartbeatState::Unavailable)) => {
                CanModuleRosterEventMeaning::TimedOutToUnavailable
            }
            (previous_state, current_state) => CanModuleRosterEventMeaning::Other {
                previous_state,
                current_state,
            },
        }
    }

    pub const fn fallback_impact(self) -> CanRemoteModuleFallbackPolicy {
        match self.current_state {
            Some(CanModuleHeartbeatState::Alive) => {
                CanRemoteModuleFallbackPolicy::RemoteDataTrusted
            }
            Some(CanModuleHeartbeatState::Starting) | Some(CanModuleHeartbeatState::Degraded) => {
                CanRemoteModuleFallbackPolicy::HoldLastKnownGood
            }
            Some(CanModuleHeartbeatState::Unavailable) | None => {
                CanRemoteModuleFallbackPolicy::RequireLocalFallback
            }
        }
    }

    pub const fn interpretation(self) -> CanModuleRosterEventInterpretation {
        CanModuleRosterEventInterpretation {
            meaning: self.meaning(),
            fallback_impact: self.fallback_impact(),
        }
    }
}

impl CanModuleRosterChangeSet {
    pub const fn new() -> Self {
        Self {
            len: 0,
            entries: [None; CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterChangeSetEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterChangeSetEntry> {
        self.entries
            .iter()
            .copied()
            .flatten()
            .find(|entry| entry.node_id == node_id)
    }

    pub fn digest(&self) -> CanModuleRosterChangeDigest {
        let mut digest = CanModuleRosterChangeDigest::default();
        let mut index = 0;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                digest.observe_transition(entry.previous_state, entry.current_state);
            }
            index += 1;
        }
        digest
    }

    fn push_change(
        &mut self,
        node_id: u8,
        previous_state: Option<CanModuleHeartbeatState>,
        current_state: Option<CanModuleHeartbeatState>,
    ) {
        if previous_state == current_state {
            return;
        }
        if self.len >= CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY {
            return;
        }
        self.entries[self.len] = Some(CanModuleRosterChangeSetEntry {
            node_id,
            previous_state,
            current_state,
        });
        self.len += 1;
    }
}

impl CanModuleRosterEventLog {
    pub const fn new() -> Self {
        Self {
            len: 0,
            dropped_count: 0,
            entries: [None; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterChangeSetEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn entry_meaning(&self, index: usize) -> Option<CanModuleRosterEventMeaning> {
        self.entry(index)
            .map(CanModuleRosterChangeSetEntry::meaning)
    }

    pub fn entry_interpretation(&self, index: usize) -> Option<CanModuleRosterEventInterpretation> {
        self.entry(index)
            .map(CanModuleRosterChangeSetEntry::interpretation)
    }

    pub fn watermark(&self) -> CanModuleRosterEventLogWatermark {
        CanModuleRosterEventLogWatermark {
            retained_event_count: self.len,
            dropped_event_count: self.dropped_count,
            total_written_event_count: self.dropped_count.saturating_add(self.len as u32),
        }
    }

    pub fn delta_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventLogDelta {
        let current_watermark = self.watermark();
        let new_event_count = current_watermark
            .total_written_event_count
            .saturating_sub(previous_watermark.total_written_event_count);
        let earliest_retained_total = current_watermark
            .total_written_event_count
            .saturating_sub(self.len as u32);
        let retained_start_total = previous_watermark
            .total_written_event_count
            .max(earliest_retained_total);
        let retained_new_event_count = current_watermark
            .total_written_event_count
            .saturating_sub(retained_start_total)
            .min(self.len as u32) as usize;
        let dropped_unread_event_count =
            new_event_count.saturating_sub(retained_new_event_count as u32);
        let mut delta = CanModuleRosterEventLogDelta {
            previous_watermark,
            current_watermark,
            retained_new_event_count,
            dropped_unread_event_count,
            ..CanModuleRosterEventLogDelta::default()
        };
        let mut source_index = self.len.saturating_sub(retained_new_event_count);
        let mut target_index = 0usize;
        while source_index < self.len {
            delta.entries[target_index] = self.entries[source_index];
            source_index += 1;
            target_index += 1;
        }
        delta
    }

    pub fn delta_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventDeltaSummary {
        self.delta_since(previous_watermark).summary()
    }

    pub fn delta_latest_per_node_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadEventSet {
        self.delta_since(previous_watermark).latest_per_node()
    }

    pub fn delta_latest_unread_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadSummary {
        self.delta_since(previous_watermark).latest_unread_summary()
    }

    pub fn delta_latest_unread_shutdown_impact_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        self.delta_since(previous_watermark)
            .latest_unread_shutdown_impact_summary()
    }

    pub fn delta_latest_unread_startup_readiness_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        self.delta_since(previous_watermark)
            .latest_unread_startup_readiness_summary()
    }

    fn push_change(
        &mut self,
        node_id: u8,
        previous_state: Option<CanModuleHeartbeatState>,
        current_state: Option<CanModuleHeartbeatState>,
    ) {
        if previous_state == current_state {
            return;
        }

        let entry = CanModuleRosterChangeSetEntry {
            node_id,
            previous_state,
            current_state,
        };

        if self.len < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY {
            self.entries[self.len] = Some(entry);
            self.len += 1;
            return;
        }

        let mut index = 1usize;
        while index < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY {
            self.entries[index - 1] = self.entries[index];
            index += 1;
        }
        self.entries[CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY - 1] = Some(entry);
        self.dropped_count = self.dropped_count.saturating_add(1);
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleRosterUpdate {
    Inserted(CanModuleRosterEntry),
    Updated(CanModuleRosterEntry),
    RejectedCapacity { node_id: u8 },
}

impl CanModuleRosterUpdate {
    pub const fn entry(self) -> Option<CanModuleRosterEntry> {
        match self {
            Self::Inserted(entry) | Self::Updated(entry) => Some(entry),
            Self::RejectedCapacity { .. } => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterRegistry {
    slots: [Option<CanModuleRosterEntry>; CAN_MODULE_ROSTER_CAPACITY],
    heartbeat_age_ms: [u32; CAN_MODULE_ROSTER_CAPACITY],
    event_log: CanModuleRosterEventLog,
    len: usize,
}

impl Default for CanModuleRosterRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl CanModuleRosterRegistry {
    pub const fn new() -> Self {
        Self {
            slots: [None; CAN_MODULE_ROSTER_CAPACITY],
            heartbeat_age_ms: [0; CAN_MODULE_ROSTER_CAPACITY],
            event_log: CanModuleRosterEventLog::new(),
            len: 0,
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn event_log(&self) -> CanModuleRosterEventLog {
        self.event_log
    }

    pub fn event_log_watermark(&self) -> CanModuleRosterEventLogWatermark {
        self.event_log.watermark()
    }

    pub fn event_log_delta_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventLogDelta {
        self.event_log.delta_since(previous_watermark)
    }

    pub fn event_log_delta_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventDeltaSummary {
        self.event_log.delta_summary_since(previous_watermark)
    }

    pub fn event_log_delta_latest_per_node_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadEventSet {
        self.event_log
            .delta_latest_per_node_since(previous_watermark)
    }

    pub fn event_log_delta_latest_unread_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadSummary {
        self.event_log
            .delta_latest_unread_summary_since(previous_watermark)
    }

    pub fn event_log_delta_latest_unread_shutdown_impact_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        self.event_log
            .delta_latest_unread_shutdown_impact_summary_since(previous_watermark)
    }

    pub fn event_log_delta_latest_unread_startup_readiness_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        self.event_log
            .delta_latest_unread_startup_readiness_summary_since(previous_watermark)
    }

    pub fn snapshot(&self) -> CanModuleRosterSnapshot {
        let mut snapshot = CanModuleRosterSnapshot::new();
        let mut source_index = 0;
        let mut target_index = 0;

        while source_index < CAN_MODULE_ROSTER_CAPACITY {
            if let Some(module) = self.slots[source_index] {
                snapshot.entries[target_index] = Some(CanModuleRosterSnapshotEntry {
                    module,
                    heartbeat_age_ms: self.heartbeat_age_ms[source_index],
                });
                target_index += 1;
            }
            source_index += 1;
        }
        snapshot.len = target_index;
        snapshot
    }

    pub fn summary(&self) -> CanModuleRosterSummary {
        self.snapshot().summary()
    }

    pub fn startup_readiness(&self) -> CanModuleRosterStartupReadiness {
        self.summary().startup_readiness()
    }

    pub fn shutdown_coordination(&self) -> CanModuleRosterShutdownCoordination {
        self.summary().shutdown_coordination()
    }

    pub fn change_digest_since(
        &self,
        previous: CanModuleRosterSnapshot,
    ) -> CanModuleRosterChangeDigest {
        self.change_set_since(previous).digest()
    }

    pub fn change_set_since(&self, previous: CanModuleRosterSnapshot) -> CanModuleRosterChangeSet {
        self.snapshot().change_set_since(previous)
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterEntry> {
        self.find_slot_index(node_id)
            .and_then(|slot_index| self.slots[slot_index])
    }

    pub fn observe_contract(
        &mut self,
        heartbeat: CanModuleHeartbeatContract,
    ) -> CanModuleRosterUpdate {
        self.observe_contract_with_age_ms(heartbeat, 0)
    }

    pub fn observe_contract_with_age_ms(
        &mut self,
        heartbeat: CanModuleHeartbeatContract,
        age_ms: u32,
    ) -> CanModuleRosterUpdate {
        if let Some(slot_index) = self.find_slot_index(heartbeat.node_id) {
            let previous = self.slots[slot_index].expect("known slot");
            self.event_log.push_change(
                heartbeat.node_id,
                Some(previous.heartbeat.state),
                Some(heartbeat.state),
            );
            let entry = CanModuleRosterEntry::from_contract_and_transition(
                heartbeat,
                CanModuleAvailabilityTransition::between(previous.heartbeat.state, heartbeat.state),
            );
            self.slots[slot_index] = Some(entry);
            self.heartbeat_age_ms[slot_index] = age_ms;
            CanModuleRosterUpdate::Updated(entry)
        } else if let Some(slot_index) = self.first_empty_slot_index() {
            self.event_log
                .push_change(heartbeat.node_id, None, Some(heartbeat.state));
            let entry = CanModuleRosterEntry::from_contract_and_transition(
                heartbeat,
                CanModuleAvailabilityTransition::NoChange(heartbeat.state),
            );
            self.slots[slot_index] = Some(entry);
            self.heartbeat_age_ms[slot_index] = age_ms;
            self.len += 1;
            CanModuleRosterUpdate::Inserted(entry)
        } else {
            CanModuleRosterUpdate::RejectedCapacity {
                node_id: heartbeat.node_id,
            }
        }
    }

    pub fn observe_message(
        &mut self,
        policy: CanHeartbeatObservationPolicy,
        message: &Message,
        age_ms: Option<u32>,
    ) -> Option<CanModuleRosterUpdate> {
        policy.observe_message(message, age_ms).map(|heartbeat| {
            self.observe_contract_with_age_ms(heartbeat, observed_age_ms(policy, age_ms))
        })
    }

    pub fn sweep_stale(
        &mut self,
        policy: CanHeartbeatObservationPolicy,
        elapsed_ms: u32,
    ) -> CanModuleRosterSweepSummary {
        let mut summary = CanModuleRosterSweepSummary::default();

        let mut slot_index = 0;
        while slot_index < CAN_MODULE_ROSTER_CAPACITY {
            if let Some(entry) = self.slots[slot_index] {
                let next_age_ms = self.heartbeat_age_ms[slot_index].saturating_add(elapsed_ms);
                self.heartbeat_age_ms[slot_index] = next_age_ms;

                if policy.is_stale(Some(next_age_ms)) {
                    summary.stale_count += 1;

                    if entry.heartbeat.state != CanModuleHeartbeatState::Unavailable {
                        self.event_log.push_change(
                            entry.heartbeat.node_id,
                            Some(entry.heartbeat.state),
                            Some(CanModuleHeartbeatState::Unavailable),
                        );
                        let heartbeat = CanModuleHeartbeatContract {
                            state: CanModuleHeartbeatState::Unavailable,
                            ..entry.heartbeat
                        };
                        let updated = CanModuleRosterEntry::from_contract_and_transition(
                            heartbeat,
                            CanModuleAvailabilityTransition::between(
                                entry.heartbeat.state,
                                CanModuleHeartbeatState::Unavailable,
                            ),
                        );
                        self.slots[slot_index] = Some(updated);
                        summary.transitioned_count += 1;
                    } else if !matches!(
                        entry.availability_transition,
                        CanModuleAvailabilityTransition::NoChange(
                            CanModuleHeartbeatState::Unavailable
                        )
                    ) {
                        self.slots[slot_index] =
                            Some(CanModuleRosterEntry::from_contract_and_transition(
                                entry.heartbeat,
                                CanModuleAvailabilityTransition::NoChange(
                                    CanModuleHeartbeatState::Unavailable,
                                ),
                            ));
                    }
                }
            }
            slot_index += 1;
        }

        summary
    }

    fn find_slot_index(&self, node_id: u8) -> Option<usize> {
        self.slots
            .iter()
            .position(|slot| matches!(slot, Some(entry) if entry.heartbeat.node_id == node_id))
    }

    fn first_empty_slot_index(&self) -> Option<usize> {
        self.slots.iter().position(Option::is_none)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterSweepSummary {
    pub stale_count: usize,
    pub transitioned_count: usize,
}

const fn observed_age_ms(policy: CanHeartbeatObservationPolicy, age_ms: Option<u32>) -> u32 {
    match age_ms {
        Some(age_ms) => age_ms,
        None => policy.timeout_ms.saturating_add(1),
    }
}
