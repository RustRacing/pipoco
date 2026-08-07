use crate::output_profiles::OutputAuthorityRequirement;
use crate::safety::SafetyPermitMask;
use ecu_domain::Ticks;

/// Logical output criticality class used to choose the legal transport and
/// stale-handling policy.
#[repr(u8)]
#[allow(clippy::enum_variant_names)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum OutputClass {
    ClassACombustionCritical = 0,
    ClassBBoundedLocal = 1,
    #[default]
    ClassCSlowSupervisory = 2,
}

/// Stable identifier for an output command.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct OutputCommandId(u32);

impl OutputCommandId {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Deadline associated with an output command.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct OutputDeadline(Ticks);

impl OutputDeadline {
    pub const fn new(value: Ticks) -> Self {
        Self(value)
    }

    pub const fn get(self) -> Ticks {
        self.0
    }
}

/// Stale-command policy used when a command can no longer be executed exactly
/// as requested.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum OutputStaleBehavior {
    #[default]
    Reject = 0,
    Cancel = 1,
    Degrade = 2,
    Retry = 3,
    ReportOnly = 4,
}

/// Backend-agnostic reason that an output command did not reach execution.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum OutputRejectReason {
    #[default]
    CommandQueueFull = 0,
    PermitDenied = 1,
    BackendFault = 2,
    StaleCommand = 3,
}

/// Logical header carried with every output command.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputCommandHeader {
    pub command_id: OutputCommandId,
    pub class: OutputClass,
    pub requested_at: Ticks,
    pub deadline: OutputDeadline,
    pub permit_mask: SafetyPermitMask,
    pub authority: OutputAuthorityRequirement,
    pub stale_behavior: OutputStaleBehavior,
}

impl OutputCommandHeader {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        command_id: OutputCommandId,
        class: OutputClass,
        requested_at: Ticks,
        deadline: OutputDeadline,
        permit_mask: SafetyPermitMask,
        authority: OutputAuthorityRequirement,
        stale_behavior: OutputStaleBehavior,
    ) -> Self {
        Self {
            command_id,
            class,
            requested_at,
            deadline,
            permit_mask,
            authority,
            stale_behavior,
        }
    }
}

impl Default for OutputCommandHeader {
    fn default() -> Self {
        Self {
            command_id: OutputCommandId::default(),
            class: OutputClass::default(),
            requested_at: Ticks::new(0),
            deadline: OutputDeadline::default(),
            permit_mask: SafetyPermitMask::NONE,
            authority: OutputAuthorityRequirement::CrankSynchronized,
            stale_behavior: OutputStaleBehavior::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn discriminants_are_stable() {
        assert_eq!(OutputClass::ClassACombustionCritical as u8, 0);
        assert_eq!(OutputClass::ClassBBoundedLocal as u8, 1);
        assert_eq!(OutputClass::ClassCSlowSupervisory as u8, 2);

        assert_eq!(OutputStaleBehavior::Reject as u8, 0);
        assert_eq!(OutputStaleBehavior::Cancel as u8, 1);
        assert_eq!(OutputStaleBehavior::Degrade as u8, 2);
        assert_eq!(OutputStaleBehavior::Retry as u8, 3);
        assert_eq!(OutputStaleBehavior::ReportOnly as u8, 4);

        assert_eq!(OutputRejectReason::CommandQueueFull as u8, 0);
        assert_eq!(OutputRejectReason::PermitDenied as u8, 1);
        assert_eq!(OutputRejectReason::BackendFault as u8, 2);
        assert_eq!(OutputRejectReason::StaleCommand as u8, 3);
    }

    #[test]
    fn newtypes_round_trip() {
        let command_id = OutputCommandId::new(42);
        let deadline = OutputDeadline::new(Ticks::new(1234));

        assert_eq!(command_id.get(), 42);
        assert_eq!(deadline.get(), Ticks::new(1234));
    }

    #[test]
    fn defaults_are_expected() {
        assert_eq!(OutputClass::default(), OutputClass::ClassCSlowSupervisory);
        assert_eq!(OutputStaleBehavior::default(), OutputStaleBehavior::Reject);

        let header = OutputCommandHeader::default();
        assert_eq!(header.command_id, OutputCommandId::default());
        assert_eq!(header.class, OutputClass::ClassCSlowSupervisory);
        assert_eq!(header.requested_at, Ticks::new(0));
        assert_eq!(header.deadline, OutputDeadline::default());
        assert_eq!(header.permit_mask, SafetyPermitMask::NONE);
        assert_eq!(
            header.authority,
            OutputAuthorityRequirement::CrankSynchronized
        );
        assert_eq!(header.stale_behavior, OutputStaleBehavior::Reject);
    }
}
