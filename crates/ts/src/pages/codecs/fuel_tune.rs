use super::*;

pub fn encode_ve_tune_page(
    page: &VeTunePage,
    limits: VeTunePageLimits,
    out: &mut [u8],
) -> Result<usize, PageError> {
    if out.len() < VE_TUNE_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }
    limits.validate()?;

    out[0..2].copy_from_slice(&page.target_afr_x10.to_le_bytes());
    out[2..4].copy_from_slice(&page.kp_i.to_le_bytes());
    out[4..6].copy_from_slice(&page.ki_i.to_le_bytes());
    out[6..8].copy_from_slice(&page.required_fuel_us.to_le_bytes());
    out[8..10].copy_from_slice(&page.injector_deadtime_us.to_le_bytes());
    out[10] = page.ve_load_source;
    out[11] = 0;
    out[12..14].copy_from_slice(&limits.min_pulse_width_us.to_le_bytes());
    out[14..16].copy_from_slice(&limits.max_pulse_width_us.to_le_bytes());
    Ok(VE_TUNE_PAGE_BYTES)
}

pub fn decode_ve_tune_page(data: &[u8]) -> Result<VeTunePage, PageError> {
    if data.len() < VE_TUNE_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    let page = VeTunePage {
        target_afr_x10: u16::from_le_bytes([data[0], data[1]]),
        kp_i: u16::from_le_bytes([data[2], data[3]]),
        ki_i: u16::from_le_bytes([data[4], data[5]]),
        required_fuel_us: u16::from_le_bytes([data[6], data[7]]),
        injector_deadtime_us: u16::from_le_bytes([data[8], data[9]]),
        ve_load_source: data[10],
    };

    if !(AFR_TARGET_MIN_X10..=AFR_TARGET_MAX_X10).contains(&page.target_afr_x10) {
        return Err(PageError::Invalid);
    }
    if page.ve_load_source > VE_TUNE_LOAD_SOURCE_MAX {
        return Err(PageError::Invalid);
    }

    Ok(page)
}
