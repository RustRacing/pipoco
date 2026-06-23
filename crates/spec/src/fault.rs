#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SpecFaultCode {
    #[default]
    None,
    SensorOutOfRange,
    SafetyCut,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SpecFaultSeverity {
    #[default]
    Info,
    Warning,
    Critical,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SpecCancelReason {
    #[default]
    None,
    Manual,
    SafetyShutdown,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SpecFaultAction {
    #[default]
    None,
    ObserveOnly,
    LimpHome,
    Shutdown,
    Cleared,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum SpecFaultPersistence {
    #[default]
    Inactive,
    LatchedUntilClear,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SpecFaultState {
    pub code: SpecFaultCode,
    pub severity: SpecFaultSeverity,
    pub cancel_reason: SpecCancelReason,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SpecFaultEvent {
    pub active: bool,
    pub action: SpecFaultAction,
    pub persistence: SpecFaultPersistence,
}

pub const fn fault_event_from_state(state: SpecFaultState) -> SpecFaultEvent {
    let active = !matches!(state.code, SpecFaultCode::None);
    let (action, persistence) = if !active {
        (SpecFaultAction::None, SpecFaultPersistence::Inactive)
    } else if matches!(state.cancel_reason, SpecCancelReason::SafetyShutdown)
        || matches!(state.severity, SpecFaultSeverity::Critical)
        || matches!(state.code, SpecFaultCode::SafetyCut)
    {
        (
            SpecFaultAction::Shutdown,
            SpecFaultPersistence::LatchedUntilClear,
        )
    } else if matches!(state.severity, SpecFaultSeverity::Warning)
        || matches!(state.code, SpecFaultCode::SensorOutOfRange)
    {
        (
            SpecFaultAction::LimpHome,
            SpecFaultPersistence::LatchedUntilClear,
        )
    } else {
        (
            SpecFaultAction::ObserveOnly,
            SpecFaultPersistence::LatchedUntilClear,
        )
    };

    SpecFaultEvent {
        active,
        action,
        persistence,
    }
}

pub const fn fault_event_for_clear(previous: SpecFaultState) -> SpecFaultEvent {
    if matches!(previous.code, SpecFaultCode::None) {
        SpecFaultEvent {
            active: false,
            action: SpecFaultAction::None,
            persistence: SpecFaultPersistence::Inactive,
        }
    } else {
        SpecFaultEvent {
            active: false,
            action: SpecFaultAction::Cleared,
            persistence: SpecFaultPersistence::Inactive,
        }
    }
}
