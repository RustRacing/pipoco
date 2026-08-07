use super::*;

impl SnapshotPageStore {
    pub fn read_page_from(page: u8, snapshot: &SnapshotPage, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_SNAPSHOT => snapshot.encode(out).ok(),
            _ => None,
        }
    }
}

impl DiagnosticPageStore {
    pub fn read_page_from(
        page: u8,
        snapshot: &DiagnosticSnapshot,
        log_entries: &[Option<DiagnosticLogEntry>; DIAG_LOG_ENTRY_COUNT],
        out: &mut [u8],
    ) -> Option<usize> {
        match page {
            PAGE_DIAG => DiagPage {
                current_tooth_count: snapshot.current_tooth_count,
                cam_seen: snapshot.cam_seen,
                sync_state: snapshot.sync_state,
                phase_state: snapshot.phase_state,
                absolute_authority: snapshot.absolute_authority,
                trigger_angle_source: snapshot.trigger_angle_source,
                output_gating_reason: snapshot.output_gating_reason,
                last_sync_loss_reason: snapshot.last_sync_loss_reason,
                primary_rpm: snapshot.primary_rpm,
                detected_gap_ratio: snapshot.detected_gap_ratio,
                sync_loss_counter: snapshot.sync_loss_counter,
                board_pin_map_identity: snapshot.board_pin_map_identity,
                profile_identity: snapshot.profile_identity,
                profile_hash: snapshot.profile_hash,
                current_fault_code: snapshot.current_fault_code,
                current_fault_severity: snapshot.current_fault_severity,
                current_fault_action: snapshot.current_fault_action,
                current_cancel_reason: snapshot.current_cancel_reason,
                fault_flags: snapshot.fault_flags,
                latest_diag_code: snapshot.latest_diag_code,
            }
            .encode(out)
            .ok(),
            PAGE_DIAG_LOG => Self::diag_log_page(log_entries).encode(out).ok(),
            _ => None,
        }
    }

    fn diag_log_page(
        log_entries: &[Option<DiagnosticLogEntry>; DIAG_LOG_ENTRY_COUNT],
    ) -> DiagLogPage {
        let mut entries = [DiagLogEntryPage::empty(); DIAG_LOG_ENTRY_COUNT];
        for (entry, source) in entries.iter_mut().zip(log_entries.iter()) {
            *entry = match source {
                Some(source) => DiagLogEntryPage::new(
                    source.code,
                    source.severity,
                    source.action,
                    source.source,
                    source.context_present,
                    source.context,
                    source.start_us,
                    source.end_us,
                ),
                None => DiagLogEntryPage::empty(),
            };
        }
        DiagLogPage::new(entries)
    }
}

impl PageStore for DiagnosticPageStore {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_DIAG => Some(DIAG_PAGE_BYTES),
            PAGE_DIAG_LOG => Some(DIAG_LOG_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            &DiagnosticSnapshot {
                current_tooth_count: self.current_tooth_count,
                cam_seen: self.cam_seen,
                sync_state: self.sync_state,
                phase_state: self.phase_state,
                absolute_authority: self.absolute_authority,
                trigger_angle_source: self.trigger_angle_source,
                output_gating_reason: self.output_gating_reason,
                last_sync_loss_reason: self.last_sync_loss_reason,
                primary_rpm: self.primary_rpm,
                detected_gap_ratio: self.detected_gap_ratio,
                sync_loss_counter: self.sync_loss_counter,
                board_pin_map_identity: self.board_pin_map_identity,
                profile_identity: self.profile_identity,
                profile_hash: self.profile_hash,
                current_fault_code: self.current_fault_code,
                current_fault_severity: self.current_fault_severity,
                current_fault_action: self.current_fault_action,
                current_cancel_reason: self.current_cancel_reason,
                fault_flags: self.fault_flags,
                latest_diag_code: self.latest_diag_code,
            },
            &self.log_entries,
            out,
        )
    }

    fn write_page(&mut self, _page: u8, _data: &[u8]) -> Result<(), PageError> {
        Err(PageError::Invalid)
    }
}

impl PageStore for SnapshotPageStore {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_SNAPSHOT => Some(SNAPSHOT_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            &SnapshotPage {
                rpm: self.rpm,
                sync_code: self.sync_code,
                cancel_reason: self.cancel_reason,
                base_pw_us: self.base_pw_us,
                enrich_mult_x100: self.enrich_mult_x100,
                stft_x10: self.stft_x10,
                fuel_mult_x100: self.fuel_mult_x100,
                final_pw_us: self.final_pw_us,
                fault_code: self.fault_code,
                fault_severity: self.fault_severity,
                isr_count: self.isr_count,
                isr_max_us: self.isr_max_us,
                isr_avg_us: self.isr_avg_us,
            },
            out,
        )
    }

    fn write_page(&mut self, _page: u8, _data: &[u8]) -> Result<(), PageError> {
        Err(PageError::Invalid)
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}
