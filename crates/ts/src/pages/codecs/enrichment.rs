use super::*;

pub fn encode_ae_page(page: &AePage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < AE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0..2].copy_from_slice(&page.tpsdot_thresh_pct_s.to_le_bytes());
    out[2..4].copy_from_slice(&page.mapdot_thresh_kpa_s.to_le_bytes());
    out[4] = page.percent_gain;
    out[5] = 0;
    out[6..10].copy_from_slice(&page.decay_time_ms.to_le_bytes());
    out[10..14].copy_from_slice(&page.lockout_ms.to_le_bytes());
    out[14] = 0;
    out[15] = 0;
    Ok(AE_PAGE_BYTES)
}

pub fn decode_ae_page(data: &[u8]) -> Result<AePage, PageCodecError> {
    if data.len() < AE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = AePage {
        tpsdot_thresh_pct_s: i16::from_le_bytes([data[0], data[1]]),
        mapdot_thresh_kpa_s: i16::from_le_bytes([data[2], data[3]]),
        percent_gain: data[4],
        decay_time_ms: u32::from_le_bytes([data[6], data[7], data[8], data[9]]),
        lockout_ms: u32::from_le_bytes([data[10], data[11], data[12], data[13]]),
    };

    if page.percent_gain > 100 || page.decay_time_ms == 0 {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}

pub fn encode_dfco_page(page: &DfcoPage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < DFCO_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0] = page.tps_max_pct;
    out[1] = 0;
    out[2..4].copy_from_slice(&page.map_max_kpa.to_le_bytes());
    out[4..6].copy_from_slice(&page.rpm_min.to_le_bytes());
    out[6..8].copy_from_slice(&page.rpm_max.to_le_bytes());
    out[8..12].copy_from_slice(&page.delay_ms.to_le_bytes());
    out[12..16].copy_from_slice(&page.resume_hyst_ms.to_le_bytes());
    Ok(DFCO_PAGE_BYTES)
}

pub fn decode_dfco_page(data: &[u8]) -> Result<DfcoPage, PageCodecError> {
    if data.len() < DFCO_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = DfcoPage {
        tps_max_pct: data[0],
        map_max_kpa: u16::from_le_bytes([data[2], data[3]]),
        rpm_min: u16::from_le_bytes([data[4], data[5]]),
        rpm_max: u16::from_le_bytes([data[6], data[7]]),
        delay_ms: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
        resume_hyst_ms: u32::from_le_bytes([data[12], data[13], data[14], data[15]]),
    };

    if page.tps_max_pct > 100 || page.rpm_min == 0 || page.rpm_max < page.rpm_min {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}

pub fn encode_wue_page(page: &WuePage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < WUE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0] = page.max_percent;
    out[1] = page.min_percent;
    out[2..4].copy_from_slice(&page.start_c.to_le_bytes());
    out[4..6].copy_from_slice(&page.end_c.to_le_bytes());
    out[6] = 0;
    out[7] = 0;
    Ok(WUE_PAGE_BYTES)
}

pub fn decode_wue_page(data: &[u8]) -> Result<WuePage, PageCodecError> {
    if data.len() < WUE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = WuePage {
        max_percent: data[0],
        min_percent: data[1],
        start_c: i16::from_le_bytes([data[2], data[3]]),
        end_c: i16::from_le_bytes([data[4], data[5]]),
    };

    if page.max_percent > 100 || page.min_percent > 100 || page.start_c >= page.end_c {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}

pub fn encode_ase_page(page: &AsePage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < ASE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0] = page.percent;
    out[1] = 0;
    out[2..6].copy_from_slice(&page.taper_time_ms.to_le_bytes());
    out[6..8].copy_from_slice(&page.lockout_ms.to_le_bytes());
    Ok(ASE_PAGE_BYTES)
}

pub fn decode_ase_page(data: &[u8]) -> Result<AsePage, PageCodecError> {
    if data.len() < ASE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = AsePage {
        percent: data[0],
        taper_time_ms: u32::from_le_bytes([data[2], data[3], data[4], data[5]]),
        lockout_ms: u16::from_le_bytes([data[6], data[7]]),
    };

    if page.percent > 100 || page.taper_time_ms == 0 {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}

pub fn encode_idle_page(page: &IdlePage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < IDLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0] = page.enable as u8;
    out[1..3].copy_from_slice(&page.duty_x10.to_le_bytes());
    out[3..5].copy_from_slice(&page.freq_hz.to_le_bytes());
    out[5] = 0;
    Ok(IDLE_PAGE_BYTES)
}

pub fn decode_idle_page(data: &[u8]) -> Result<IdlePage, PageCodecError> {
    if data.len() < IDLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = IdlePage {
        enable: data[0] != 0,
        duty_x10: u16::from_le_bytes([data[1], data[2]]),
        freq_hz: u16::from_le_bytes([data[3], data[4]]),
    };

    if page.duty_x10 > IDLE_DUTY_MAX_X10 || page.freq_hz == 0 {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}

pub fn encode_fan_page(page: &FanPage, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < FAN_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0] = page.enable as u8;
    out[1..3].copy_from_slice(&page.on_c.to_le_bytes());
    out[3..5].copy_from_slice(&page.off_c.to_le_bytes());
    out[5] = 0;
    Ok(FAN_PAGE_BYTES)
}

pub fn decode_fan_page(data: &[u8]) -> Result<FanPage, PageCodecError> {
    if data.len() < FAN_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = FanPage {
        enable: data[0] != 0,
        on_c: i16::from_le_bytes([data[1], data[2]]),
        off_c: i16::from_le_bytes([data[3], data[4]]),
    };

    if page.on_c <= page.off_c {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}

pub fn encode_closed_loop_page(
    page: &ClosedLoopPage,
    out: &mut [u8],
) -> Result<usize, PageCodecError> {
    if out.len() < CLOSED_LOOP_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[0] = page.enable as u8;
    out[1] = 0;
    out[2..4].copy_from_slice(&page.target_afr_x10.to_le_bytes());
    out[4..6].copy_from_slice(&page.kp_i.to_le_bytes());
    out[6..8].copy_from_slice(&page.ki_i.to_le_bytes());
    Ok(CLOSED_LOOP_PAGE_BYTES)
}

pub fn decode_closed_loop_page(data: &[u8]) -> Result<ClosedLoopPage, PageCodecError> {
    if data.len() < CLOSED_LOOP_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = ClosedLoopPage {
        enable: data[0] != 0,
        target_afr_x10: u16::from_le_bytes([data[2], data[3]]),
        kp_i: u16::from_le_bytes([data[4], data[5]]),
        ki_i: u16::from_le_bytes([data[6], data[7]]),
    };

    if !(AFR_TARGET_MIN_X10..=AFR_TARGET_MAX_X10).contains(&page.target_afr_x10) {
        return Err(PageCodecError::Invalid);
    }

    Ok(page)
}
