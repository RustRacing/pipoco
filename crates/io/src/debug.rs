//! Shared debug vocabulary for signal and output frontiers.

/// Debug trace detail levels.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum TraceLevel {
    #[default]
    FinalState = 0,
    RequestAndFinal = 1,
    NamedAssembly = 2,
    BackendMechanism = 3,
    FullTrace = 4,
}

/// Named stages in the signal frontier assembly.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SignalStage {
    #[default]
    SignalCapture = 0,
    SignalNormalizer = 1,
    ObservationValidator = 2,
    ObservationPublisher = 3,
    ObservationReader = 4,
    RuntimeSnapshotBuilder = 5,
    PolicyConsumer = 6,
}

impl SignalStage {
    /// Zero-based stage index for table-driven counters.
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Number of signal assembly stages.
    pub const fn count() -> usize {
        Self::PolicyConsumer as usize + 1
    }
}

/// Named stages in the output frontier assembly.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum OutputStage {
    #[default]
    OutputIntent = 0,
    OutputPlanner = 1,
    OutputAdmission = 2,
    OutputArmer = 3,
    OutputExecutor = 4,
    OutputObserver = 5,
}

impl OutputStage {
    /// Zero-based stage index for table-driven counters.
    pub const fn index(self) -> usize {
        self as usize
    }

    /// Number of output assembly stages.
    pub const fn count() -> usize {
        Self::OutputObserver as usize + 1
    }
}

/// Outcome values shared across debug trace points.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum StageOutcome {
    #[default]
    Seen = 0,
    Accepted = 1,
    Rejected = 2,
    Dropped = 3,
    Stale = 4,
    Overrun = 5,
    Planned = 6,
    Admitted = 7,
    Armed = 8,
    Executed = 9,
    Completed = 10,
    Cancelled = 11,
    Late = 12,
    BackendFault = 13,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trace_level_discriminants_and_default_are_stable() {
        assert_eq!(TraceLevel::FinalState as u8, 0);
        assert_eq!(TraceLevel::RequestAndFinal as u8, 1);
        assert_eq!(TraceLevel::NamedAssembly as u8, 2);
        assert_eq!(TraceLevel::BackendMechanism as u8, 3);
        assert_eq!(TraceLevel::FullTrace as u8, 4);
        assert_eq!(TraceLevel::default(), TraceLevel::FinalState);
    }

    #[test]
    fn signal_stage_discriminants_and_default_are_stable() {
        assert_eq!(SignalStage::SignalCapture as u8, 0);
        assert_eq!(SignalStage::SignalNormalizer as u8, 1);
        assert_eq!(SignalStage::ObservationValidator as u8, 2);
        assert_eq!(SignalStage::ObservationPublisher as u8, 3);
        assert_eq!(SignalStage::ObservationReader as u8, 4);
        assert_eq!(SignalStage::RuntimeSnapshotBuilder as u8, 5);
        assert_eq!(SignalStage::PolicyConsumer as u8, 6);
        assert_eq!(SignalStage::SignalCapture.index(), 0);
        assert_eq!(SignalStage::count(), 7);
        assert_eq!(SignalStage::default(), SignalStage::SignalCapture);
    }

    #[test]
    fn output_stage_discriminants_and_default_are_stable() {
        assert_eq!(OutputStage::OutputIntent as u8, 0);
        assert_eq!(OutputStage::OutputPlanner as u8, 1);
        assert_eq!(OutputStage::OutputAdmission as u8, 2);
        assert_eq!(OutputStage::OutputArmer as u8, 3);
        assert_eq!(OutputStage::OutputExecutor as u8, 4);
        assert_eq!(OutputStage::OutputObserver as u8, 5);
        assert_eq!(OutputStage::OutputAdmission.index(), 2);
        assert_eq!(OutputStage::count(), 6);
        assert_eq!(OutputStage::default(), OutputStage::OutputIntent);
    }

    #[test]
    fn stage_outcome_discriminants_and_default_are_stable() {
        assert_eq!(StageOutcome::Seen as u8, 0);
        assert_eq!(StageOutcome::Accepted as u8, 1);
        assert_eq!(StageOutcome::Rejected as u8, 2);
        assert_eq!(StageOutcome::Dropped as u8, 3);
        assert_eq!(StageOutcome::Stale as u8, 4);
        assert_eq!(StageOutcome::Overrun as u8, 5);
        assert_eq!(StageOutcome::Planned as u8, 6);
        assert_eq!(StageOutcome::Admitted as u8, 7);
        assert_eq!(StageOutcome::Armed as u8, 8);
        assert_eq!(StageOutcome::Executed as u8, 9);
        assert_eq!(StageOutcome::Completed as u8, 10);
        assert_eq!(StageOutcome::Cancelled as u8, 11);
        assert_eq!(StageOutcome::Late as u8, 12);
        assert_eq!(StageOutcome::BackendFault as u8, 13);
        assert_eq!(StageOutcome::default(), StageOutcome::Seen);
    }
}
