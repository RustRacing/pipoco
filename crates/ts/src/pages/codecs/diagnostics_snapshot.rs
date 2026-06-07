use super::*;

pub fn encode_snapshot_page(page: &SnapshotPage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < SNAPSHOT_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0..2].copy_from_slice(&page.rpm.to_le_bytes());
    out[2] = page.sync_code;
    out[3] = 0;
    out[4..8].copy_from_slice(&page.base_pw_us.to_le_bytes());
    out[8..10].copy_from_slice(&page.enrich_mult_x100.to_le_bytes());
    out[10..12].copy_from_slice(&page.stft_x10.to_le_bytes());
    out[12..14].copy_from_slice(&page.fuel_mult_x100.to_le_bytes());
    out[14..18].copy_from_slice(&page.final_pw_us.to_le_bytes());
    out[18] = page.fault_code;
    out[19] = 0;
    out[20..24].copy_from_slice(&page.isr_count.to_le_bytes());
    out[24..28].copy_from_slice(&page.isr_max_us.to_le_bytes());
    out[28..32].copy_from_slice(&page.isr_avg_us.to_le_bytes());
    Ok(SNAPSHOT_PAGE_BYTES)
}

pub fn encode_diag_page(page: &DiagPage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < DIAG_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
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
    Ok(DIAG_PAGE_BYTES)
}

pub fn decode_diag_page(data: &[u8]) -> Result<DiagPage, PageCodecError> {
    if data.len() != DIAG_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
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
    })
}

pub fn encode_diag_log_page(page: &DiagLogPage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < DIAG_LOG_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    for (idx, entry) in page.entries.iter().enumerate() {
        let offset = idx * DIAG_LOG_ENTRY_BYTES;
        out[offset] = entry.code;
        out[offset + 1..offset + 5].copy_from_slice(&entry.start_us.to_le_bytes());
        out[offset + 5..offset + 9].copy_from_slice(&entry.end_us.to_le_bytes());
    }
    Ok(DIAG_LOG_PAGE_BYTES)
}

pub fn decode_diag_log_page(data: &[u8]) -> Result<DiagLogPage, PageCodecError> {
    if data.len() != DIAG_LOG_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut entries = [DiagLogEntryPage::empty(); DIAG_LOG_ENTRY_COUNT];
    for (idx, entry) in entries.iter_mut().enumerate() {
        let offset = idx * DIAG_LOG_ENTRY_BYTES;
        *entry = DiagLogEntryPage {
            code: data[offset],
            start_us: u32::from_le_bytes([
                data[offset + 1],
                data[offset + 2],
                data[offset + 3],
                data[offset + 4],
            ]),
            end_us: u32::from_le_bytes([
                data[offset + 5],
                data[offset + 6],
                data[offset + 7],
                data[offset + 8],
            ]),
        };
    }
    Ok(DiagLogPage { entries })
}
