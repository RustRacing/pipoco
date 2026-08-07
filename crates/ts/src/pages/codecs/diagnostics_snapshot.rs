use super::*;

pub fn encode_snapshot_page(page: &SnapshotPage, out: &mut [u8]) -> Result<usize, PageError> {
    if out.len() < SNAPSHOT_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    out[0..2].copy_from_slice(&page.rpm.to_le_bytes());
    out[2] = page.sync_code;
    out[3] = page.cancel_reason;
    out[4..8].copy_from_slice(&page.base_pw_us.to_le_bytes());
    out[8..10].copy_from_slice(&page.enrich_mult_x100.to_le_bytes());
    out[10..12].copy_from_slice(&page.stft_x10.to_le_bytes());
    out[12..14].copy_from_slice(&page.fuel_mult_x100.to_le_bytes());
    out[14..18].copy_from_slice(&page.final_pw_us.to_le_bytes());
    out[18] = page.fault_code;
    out[19] = page.fault_severity;
    out[20..24].copy_from_slice(&page.isr_count.to_le_bytes());
    out[24..28].copy_from_slice(&page.isr_max_us.to_le_bytes());
    out[28..32].copy_from_slice(&page.isr_avg_us.to_le_bytes());
    Ok(SNAPSHOT_PAGE_BYTES)
}

pub fn encode_diag_page(page: &DiagPage, out: &mut [u8]) -> Result<usize, PageError> {
    if out.len() < DIAG_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    out[..DIAG_PAGE_BYTES].fill(0);
    out[0] = page.current_tooth_count;
    out[1] = u8::from(page.cam_seen);
    out[2] = page.sync_state;
    out[3] = page.phase_state;
    out[4] = page.absolute_authority;
    out[5] = page.trigger_angle_source;
    out[6] = page.output_gating_reason;
    out[7] = page.last_sync_loss_reason;
    out[8..10].copy_from_slice(&page.primary_rpm.to_le_bytes());
    out[10..12].copy_from_slice(&page.detected_gap_ratio.to_le_bytes());
    out[12..14].copy_from_slice(&page.sync_loss_counter.to_le_bytes());
    out[14..16].copy_from_slice(&page.board_pin_map_identity.to_le_bytes());
    out[16..20].copy_from_slice(&page.profile_identity.to_le_bytes());
    out[20..24].copy_from_slice(&page.profile_hash.to_le_bytes());
    out[24] = page.current_fault_code;
    out[25] = page.current_fault_severity;
    out[26] = page.current_fault_action;
    out[27] = page.current_cancel_reason;
    out[28] = page.fault_flags;
    out[29] = page.latest_diag_code;
    Ok(DIAG_PAGE_BYTES)
}

pub fn decode_diag_page(data: &[u8]) -> Result<DiagPage, PageError> {
    if data.len() != DIAG_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    Ok(DiagPage {
        current_tooth_count: data[0],
        cam_seen: data[1] != 0,
        sync_state: data[2],
        phase_state: data[3],
        absolute_authority: data[4],
        trigger_angle_source: data[5],
        output_gating_reason: data[6],
        last_sync_loss_reason: data[7],
        primary_rpm: u16::from_le_bytes([data[8], data[9]]),
        detected_gap_ratio: u16::from_le_bytes([data[10], data[11]]),
        sync_loss_counter: u16::from_le_bytes([data[12], data[13]]),
        board_pin_map_identity: u16::from_le_bytes([data[14], data[15]]),
        profile_identity: u32::from_le_bytes([data[16], data[17], data[18], data[19]]),
        profile_hash: u32::from_le_bytes([data[20], data[21], data[22], data[23]]),
        current_fault_code: data[24],
        current_fault_severity: data[25],
        current_fault_action: data[26],
        current_cancel_reason: data[27],
        fault_flags: data[28],
        latest_diag_code: data[29],
    })
}

pub fn encode_diag_log_page(page: &DiagLogPage, out: &mut [u8]) -> Result<usize, PageError> {
    if out.len() < DIAG_LOG_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    for (idx, entry) in page.entries.iter().enumerate() {
        let offset = idx * DIAG_LOG_ENTRY_BYTES;
        out[offset] = entry.code;
        out[offset + 1] = entry.severity;
        out[offset + 2] = entry.action;
        out[offset + 3] = entry.source
            | if entry.context_present {
                TS_DIAG_SOURCE_CONTEXT_PRESENT
            } else {
                0
            };
        out[offset + 4..offset + 8].copy_from_slice(&entry.start_us.to_le_bytes());
        out[offset + 8..offset + 12].copy_from_slice(&entry.end_us.to_le_bytes());
        out[offset + 12..offset + 16].copy_from_slice(&entry.context.to_le_bytes());
    }
    Ok(DIAG_LOG_PAGE_BYTES)
}

pub fn decode_diag_log_page(data: &[u8]) -> Result<DiagLogPage, PageError> {
    if data.len() != DIAG_LOG_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    let mut entries = [DiagLogEntryPage::empty(); DIAG_LOG_ENTRY_COUNT];
    for (idx, entry) in entries.iter_mut().enumerate() {
        let offset = idx * DIAG_LOG_ENTRY_BYTES;
        let source = data[offset + 3];
        *entry = DiagLogEntryPage {
            code: data[offset],
            severity: data[offset + 1],
            action: data[offset + 2],
            source: source & !TS_DIAG_SOURCE_CONTEXT_PRESENT,
            context_present: (source & TS_DIAG_SOURCE_CONTEXT_PRESENT) != 0,
            start_us: u32::from_le_bytes([
                data[offset + 4],
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
            ]),
            end_us: u32::from_le_bytes([
                data[offset + 8],
                data[offset + 9],
                data[offset + 10],
                data[offset + 11],
            ]),
            context: u32::from_le_bytes([
                data[offset + 12],
                data[offset + 13],
                data[offset + 14],
                data[offset + 15],
            ]),
        };
    }
    Ok(DiagLogPage { entries })
}
