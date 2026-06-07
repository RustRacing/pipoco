//! Generic software-readiness report primitives.
//!
//! These types intentionally aggregate evidence produced elsewhere. They do not
//! encode engine-specific rules; profile requirements come from
//! `ecu-board-profiles`, simulator evidence comes from host runs, and external
//! metadata/TS gates remain explicit inputs.

use ecu_board_profiles::ProfileCompatibilityReport;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimulatorReadinessEvidence {
    pub sync_acquired: bool,
    pub output_schedule_seen: bool,
    pub fault_cut_seen: bool,
    pub deterministic_replay: bool,
}

impl SimulatorReadinessEvidence {
    pub const fn ready(self) -> bool {
        self.sync_acquired
            && self.output_schedule_seen
            && self.fault_cut_seen
            && self.deterministic_replay
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SoftwareReadinessReport {
    pub compatibility: ProfileCompatibilityReport,
    pub simulator: SimulatorReadinessEvidence,
    pub ts_pages_valid: bool,
    pub evidence_metadata_valid: bool,
}

impl SoftwareReadinessReport {
    pub const fn ready(self) -> bool {
        self.compatibility.ready()
            && self.simulator.ready()
            && self.ts_pages_valid
            && self.evidence_metadata_valid
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn passing_report() -> SoftwareReadinessReport {
        SoftwareReadinessReport {
            compatibility: ProfileCompatibilityReport::new(),
            simulator: SimulatorReadinessEvidence {
                sync_acquired: true,
                output_schedule_seen: true,
                fault_cut_seen: true,
                deterministic_replay: true,
            },
            ts_pages_valid: true,
            evidence_metadata_valid: true,
        }
    }

    #[test]
    fn software_report_requires_every_gate() {
        assert!(passing_report().ready());

        let mut report = passing_report();
        report.simulator.fault_cut_seen = false;
        assert!(!report.ready());

        let mut report = passing_report();
        report.ts_pages_valid = false;
        assert!(!report.ready());

        let mut report = passing_report();
        report.evidence_metadata_valid = false;
        assert!(!report.ready());
    }
}
