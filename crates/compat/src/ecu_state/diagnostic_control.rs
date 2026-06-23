use super::EcuState;
use crate::diag;

impl EcuState {
    /// Clear request-owned diagnostic state without mutating trigger configuration.
    pub fn clear_diagnostics(&mut self) -> diag::DiagClearSummary {
        let mut summary = diag::DiagClearSummary::default();

        summary.cleared_active_count += u8::from(self.diag_map.is_active());
        summary.cleared_active_count += u8::from(self.diag_tps.is_active());
        summary.cleared_active_count += u8::from(self.diag_cam.is_active());
        summary.cleared_log_entries = self
            .diag_log()
            .events
            .iter()
            .filter(|entry| entry.is_some())
            .count() as u8;
        summary.emergency_cleared = self.emergency_mode();

        self.diag_map = diag::DiagState::new();
        self.diag_tps = diag::DiagState::new();
        self.diag_cam = diag::DiagState::new();
        self.faults.diag_log = diag::DiagLog::new();
        self.set_emergency_mode(false);

        summary
    }
}
