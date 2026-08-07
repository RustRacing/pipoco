use super::*;

pub fn encode_angles_page(page: &AnglesPage, out: &mut [u8]) -> Result<usize, PageError> {
    if out.len() < ANGLES_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    let mut idx = 0usize;
    for v in page.inj_angles_x10.iter() {
        out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
        idx += 2;
    }
    for v in page.tdc_angles_x10.iter() {
        out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
        idx += 2;
    }
    out[idx..idx + 2].copy_from_slice(&page.tooth0_angle_x10.to_le_bytes());
    idx += 2;
    out[idx..idx + 2].copy_from_slice(&page.cam_timeout_ms.to_le_bytes());
    Ok(ANGLES_PAGE_BYTES)
}

pub fn decode_angles_page(data: &[u8]) -> Result<AnglesPage, PageError> {
    if data.len() < ANGLES_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    let mut idx = 0usize;
    let mut r16 = |data: &[u8]| -> u16 {
        let v = u16::from_le_bytes([data[idx], data[idx + 1]]);
        idx += 2;
        v
    };

    let mut inj_angles_x10 = [0u16; 16];
    for v in inj_angles_x10.iter_mut() {
        let angle = r16(data);
        if angle > 3600 {
            return Err(PageError::Invalid);
        }
        *v = angle;
    }

    let mut tdc_angles_x10 = [0u16; 16];
    for v in tdc_angles_x10.iter_mut() {
        let angle = r16(data);
        if angle > 7200 {
            return Err(PageError::Invalid);
        }
        *v = angle;
    }

    let tooth0_angle_x10 = r16(data);
    if tooth0_angle_x10 > 3600 {
        return Err(PageError::Invalid);
    }
    let cam_timeout_ms = r16(data);

    Ok(AnglesPage {
        inj_angles_x10,
        tdc_angles_x10,
        tooth0_angle_x10,
        cam_timeout_ms,
    })
}

pub fn encode_sensors_page(page: &SensorsPage, out: &mut [u8]) -> Result<usize, PageError> {
    // Layout (LE): see decode_sensors_page. Reserved tail bytes 108..128 are
    // left untouched.
    if out.len() < SENSORS_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }
    let mut idx = 0;
    let mut w16 = |out: &mut [u8], v: u16| {
        out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
        idx += 2;
    };
    w16(out, page.tps_min_counts);
    w16(out, page.tps_max_counts);
    w16(out, page.map_v0_mv);
    w16(out, page.map_kpa0_x10);
    w16(out, page.map_v1_mv);
    w16(out, page.map_kpa1_x10);
    for v in page.clt_deg_c.iter() {
        out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
        idx += 2;
    }
    for v in page.iat_deg_c.iter() {
        out[idx..idx + 2].copy_from_slice(&v.to_le_bytes());
        idx += 2;
    }
    for v in page.clt_ohms.iter() {
        out[idx..idx + 4].copy_from_slice(&v.to_le_bytes());
        idx += 4;
    }
    for v in page.iat_ohms.iter() {
        out[idx..idx + 4].copy_from_slice(&v.to_le_bytes());
        idx += 4;
    }
    Ok(SENSORS_PAGE_BYTES)
}

pub fn decode_sensors_page(data: &[u8]) -> Result<SensorsPage, PageError> {
    // Layout (LE):
    // 0: tps_min_counts (u16), 2: tps_max_counts (u16)
    // 4: map_v0_mv (u16), 6: map_kpa0_x10 (u16)
    // 8: map_v1_mv (u16), 10: map_kpa1_x10 (u16)
    // 12..28: clt_deg_c [8] (i16)
    // 28..44: iat_deg_c [8] (i16)
    // 44..76: clt_ohms [8] (u32)
    // 76..108: iat_ohms [8] (u32)
    // 108..128: reserved (ignored)
    if data.len() < SENSORS_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }
    let mut idx = 0;
    let mut r16 = |data: &[u8]| -> u16 {
        let v = u16::from_le_bytes([data[idx], data[idx + 1]]);
        idx += 2;
        v
    };
    let tps_min_counts = r16(data);
    let tps_max_counts = r16(data);
    if tps_min_counts >= tps_max_counts {
        return Err(PageError::Invalid);
    }
    let map_v0_mv = r16(data);
    let map_kpa0_x10 = r16(data);
    let map_v1_mv = r16(data);
    let map_kpa1_x10 = r16(data);
    if map_v1_mv <= map_v0_mv || map_kpa1_x10 <= map_kpa0_x10 || map_v1_mv > SENSOR_ADC_INPUT_MAX_MV
    {
        return Err(PageError::Invalid);
    }
    let mut clt_deg_c = [0i16; 8];
    let mut iat_deg_c = [0i16; 8];
    let mut clt_ohms = [0u32; 8];
    let mut iat_ohms = [0u32; 8];
    for v in clt_deg_c.iter_mut() {
        *v = i16::from_le_bytes([data[idx], data[idx + 1]]);
        idx += 2;
    }
    for v in iat_deg_c.iter_mut() {
        *v = i16::from_le_bytes([data[idx], data[idx + 1]]);
        idx += 2;
    }
    for v in clt_ohms.iter_mut() {
        *v = u32::from_le_bytes([data[idx], data[idx + 1], data[idx + 2], data[idx + 3]]);
        idx += 4;
    }
    for v in iat_ohms.iter_mut() {
        *v = u32::from_le_bytes([data[idx], data[idx + 1], data[idx + 2], data[idx + 3]]);
        idx += 4;
    }
    Ok(SensorsPage {
        tps_min_counts,
        tps_max_counts,
        map_v0_mv,
        map_kpa0_x10,
        map_v1_mv,
        map_kpa1_x10,
        clt_deg_c,
        iat_deg_c,
        clt_ohms,
        iat_ohms,
    })
}

pub fn encode_limits_page(page: &LimitsPage, out: &mut [u8]) -> Result<usize, PageError> {
    if out.len() < LIMITS_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    out[0..2].copy_from_slice(&page.map_min_kpa_x10.to_le_bytes());
    out[2..4].copy_from_slice(&page.map_max_kpa_x10.to_le_bytes());
    out[4] = page.tps_min_percent;
    out[5] = page.tps_max_percent;
    out[6..8].copy_from_slice(&page.clear_time_s.to_le_bytes());
    let mut trigger = 0u8;
    if page.emerg_trig_map {
        trigger |= LIMITS_TRIGGER_MAP_BIT;
    }
    if page.emerg_trig_tps {
        trigger |= LIMITS_TRIGGER_TPS_BIT;
    }
    out[8] = trigger;
    out[9] = 0;
    Ok(LIMITS_PAGE_BYTES)
}

pub fn decode_limits_page(data: &[u8]) -> Result<LimitsPage, PageError> {
    if data.len() < LIMITS_PAGE_BYTES {
        return Err(PageError::WrongSize);
    }

    let map_min_kpa_x10 = u16::from_le_bytes([data[0], data[1]]);
    let map_max_kpa_x10 = u16::from_le_bytes([data[2], data[3]]);
    let tps_min_percent = data[4];
    let tps_max_percent = data[5];
    let clear_time_s = u16::from_le_bytes([data[6], data[7]]);
    let trigger = data[8];

    if map_min_kpa_x10 == 0 || map_max_kpa_x10 <= map_min_kpa_x10 {
        return Err(PageError::Invalid);
    }
    if tps_max_percent > 100 || tps_max_percent <= tps_min_percent {
        return Err(PageError::Invalid);
    }
    if clear_time_s == 0 {
        return Err(PageError::Invalid);
    }
    if trigger & !LIMITS_TRIGGER_BITS_MASK != 0 {
        return Err(PageError::Invalid);
    }

    Ok(LimitsPage {
        map_min_kpa_x10,
        map_max_kpa_x10,
        tps_min_percent,
        tps_max_percent,
        clear_time_s,
        emerg_trig_map: trigger & LIMITS_TRIGGER_MAP_BIT != 0,
        emerg_trig_tps: trigger & LIMITS_TRIGGER_TPS_BIT != 0,
    })
}
