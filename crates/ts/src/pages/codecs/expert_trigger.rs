use super::*;

pub fn encode_expert_trigger_page(
    page: &ExpertTriggerPage,
    out: &mut [u8],
) -> Result<usize, PageCodecError> {
    validate_expert_trigger_page(page, None, true)?;
    if out.len() < EXPERT_TRIGGER_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    out[..EXPERT_TRIGGER_PAGE_BYTES].fill(0);
    out[0..2].copy_from_slice(&page.schema_version.to_le_bytes());
    out[2] = page.expert_unlock;
    out[3] = page.authority;
    out[4..8].copy_from_slice(&page.profile_identity.to_le_bytes());
    out[8..12].copy_from_slice(&page.profile_hash.to_le_bytes());
    out[12] = page.trigger_pattern;
    out[13] = page.primary_base_teeth;
    out[14] = page.missing_teeth;
    out[15] = page.primary_trigger_speed;
    out[16..18].copy_from_slice(&page.trigger_angle_atdc_deg10.to_le_bytes());
    out[18] = page.trigger_angle_multiplier;
    out[19] = page.primary_trigger_edge;
    out[20] = page.secondary_trigger_edge;
    out[21] = page.secondary_trigger_mode;
    out[22] = page.poll_level_polarity;
    out[23] = page.trigger_filter;
    out[24] = u8::from(page.resync_every_cycle);
    out[25] = page.skip_cycles;
    out[26] = page.ignition_mode;
    out[27] = page.injection_layout;
    out[28] = page.fixed_timing_mode;
    out[30..32].copy_from_slice(&page.fixed_timing_deg10.to_le_bytes());
    Ok(EXPERT_TRIGGER_PAGE_BYTES)
}

pub fn decode_expert_trigger_page(data: &[u8]) -> Result<ExpertTriggerPage, PageCodecError> {
    decode_expert_trigger_page_with_current(data, None)
}

#[cfg(feature = "calibration")]
pub fn expert_trigger_page_from_calibration(
    cal: &ecu_calibration::ExpertTriggerCalibration,
) -> ExpertTriggerPage {
    ExpertTriggerPage {
        schema_version: cal.schema_version.get(),
        expert_unlock: cal.expert_unlock.code(),
        authority: cal.authority.code(),
        profile_identity: cal.profile_identity,
        profile_hash: cal.profile_hash,
        trigger_pattern: cal.trigger_pattern.code(),
        primary_base_teeth: cal.primary_base_teeth,
        missing_teeth: cal.missing_teeth,
        primary_trigger_speed: cal.primary_trigger_speed.code(),
        trigger_angle_atdc_deg10: cal.trigger_angle_atdc_deg10,
        trigger_angle_multiplier: cal.trigger_angle_multiplier,
        primary_trigger_edge: cal.primary_trigger_edge.code(),
        secondary_trigger_edge: cal.secondary_trigger_edge.code(),
        secondary_trigger_mode: cal.secondary_trigger_mode.code(),
        poll_level_polarity: cal.poll_level_polarity.code(),
        trigger_filter: cal.trigger_filter.code(),
        resync_every_cycle: cal.resync_every_cycle,
        skip_cycles: cal.skip_cycles,
        ignition_mode: cal.ignition_mode.code(),
        injection_layout: cal.injection_layout.code(),
        fixed_timing_mode: cal.fixed_timing_mode.code(),
        fixed_timing_deg10: cal.fixed_timing_deg10,
    }
}

#[cfg(feature = "calibration")]
pub fn expert_trigger_calibration_from_page(
    page: ExpertTriggerPage,
) -> Result<ecu_calibration::ExpertTriggerCalibration, PageCodecError> {
    use ecu_calibration::{
        CalibrationSchemaVersion, ExpertIgnitionMode, ExpertInjectionLayout,
        ExpertTriggerCalibration, ExpertUnlock, FixedTimingMode, PollLevelPolarity,
        PrimaryTriggerSpeed, SecondaryTriggerMode, TriggerAuthority, TriggerEdge, TriggerFilter,
        TriggerPattern,
    };

    let cal = ExpertTriggerCalibration {
        schema_version: CalibrationSchemaVersion::new(page.schema_version),
        expert_unlock: ExpertUnlock::from_code(page.expert_unlock)
            .ok_or(PageCodecError::Invalid)?,
        authority: TriggerAuthority::from_code(page.authority).ok_or(PageCodecError::Invalid)?,
        profile_identity: page.profile_identity,
        profile_hash: page.profile_hash,
        trigger_pattern: TriggerPattern::from_code(page.trigger_pattern)
            .ok_or(PageCodecError::Invalid)?,
        primary_base_teeth: page.primary_base_teeth,
        missing_teeth: page.missing_teeth,
        primary_trigger_speed: PrimaryTriggerSpeed::from_code(page.primary_trigger_speed)
            .ok_or(PageCodecError::Invalid)?,
        trigger_angle_atdc_deg10: page.trigger_angle_atdc_deg10,
        trigger_angle_multiplier: page.trigger_angle_multiplier,
        primary_trigger_edge: TriggerEdge::from_code(page.primary_trigger_edge)
            .ok_or(PageCodecError::Invalid)?,
        secondary_trigger_edge: TriggerEdge::from_code(page.secondary_trigger_edge)
            .ok_or(PageCodecError::Invalid)?,
        secondary_trigger_mode: SecondaryTriggerMode::from_code(page.secondary_trigger_mode)
            .ok_or(PageCodecError::Invalid)?,
        poll_level_polarity: PollLevelPolarity::from_code(page.poll_level_polarity)
            .ok_or(PageCodecError::Invalid)?,
        trigger_filter: TriggerFilter::from_code(page.trigger_filter)
            .ok_or(PageCodecError::Invalid)?,
        resync_every_cycle: page.resync_every_cycle,
        skip_cycles: page.skip_cycles,
        ignition_mode: ExpertIgnitionMode::from_code(page.ignition_mode)
            .ok_or(PageCodecError::Invalid)?,
        injection_layout: ExpertInjectionLayout::from_code(page.injection_layout)
            .ok_or(PageCodecError::Invalid)?,
        fixed_timing_mode: FixedTimingMode::from_code(page.fixed_timing_mode)
            .ok_or(PageCodecError::Invalid)?,
        fixed_timing_deg10: page.fixed_timing_deg10,
    };
    cal.validate().map_err(|_| PageCodecError::Invalid)?;
    Ok(cal)
}

pub fn decode_trusted_expert_trigger_page(
    data: &[u8],
) -> Result<ExpertTriggerPage, PageCodecError> {
    decode_expert_trigger_page_inner(data, None, true)
}

pub fn decode_expert_trigger_page_with_current(
    data: &[u8],
    current: Option<&ExpertTriggerPage>,
) -> Result<ExpertTriggerPage, PageCodecError> {
    decode_expert_trigger_page_inner(data, current, false)
}

fn decode_expert_trigger_page_inner(
    data: &[u8],
    current: Option<&ExpertTriggerPage>,
    allow_certified_profile: bool,
) -> Result<ExpertTriggerPage, PageCodecError> {
    if data.len() != EXPERT_TRIGGER_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let page = ExpertTriggerPage {
        schema_version: u16::from_le_bytes([data[0], data[1]]),
        expert_unlock: data[2],
        authority: data[3],
        profile_identity: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
        profile_hash: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
        trigger_pattern: data[12],
        primary_base_teeth: data[13],
        missing_teeth: data[14],
        primary_trigger_speed: data[15],
        trigger_angle_atdc_deg10: u16::from_le_bytes([data[16], data[17]]),
        trigger_angle_multiplier: data[18],
        primary_trigger_edge: data[19],
        secondary_trigger_edge: data[20],
        secondary_trigger_mode: data[21],
        poll_level_polarity: data[22],
        trigger_filter: data[23],
        resync_every_cycle: data[24] != 0,
        skip_cycles: data[25],
        ignition_mode: data[26],
        injection_layout: data[27],
        fixed_timing_mode: data[28],
        fixed_timing_deg10: i16::from_le_bytes([data[30], data[31]]),
    };

    validate_expert_trigger_page(&page, current, allow_certified_profile)?;
    Ok(page)
}

fn validate_expert_trigger_page(
    page: &ExpertTriggerPage,
    current: Option<&ExpertTriggerPage>,
    allow_certified_profile: bool,
) -> Result<(), PageCodecError> {
    if page.schema_version != EXPERT_SCHEMA_VERSION_CURRENT {
        return Err(PageCodecError::Invalid);
    }
    if !allow_certified_profile && page.authority == TRIGGER_AUTHORITY_CERTIFIED_PROFILE {
        return Err(PageCodecError::Invalid);
    }
    if page.expert_unlock > EXPERT_UNLOCK_UNLOCKED
        || page.authority > 4
        || page.trigger_pattern > 3
        || page.primary_trigger_speed > 1
        || page.primary_trigger_edge > 1
        || page.secondary_trigger_edge > 1
        || page.secondary_trigger_mode > 5
        || page.poll_level_polarity > 1
        || page.trigger_filter > 3
        || page.ignition_mode > 3
        || page.injection_layout > 4
        || page.fixed_timing_mode > 1
    {
        return Err(PageCodecError::Invalid);
    }

    if page.authority == TRIGGER_AUTHORITY_EXPERT_MANUAL
        && page.expert_unlock != EXPERT_UNLOCK_UNLOCKED
    {
        return Err(PageCodecError::Invalid);
    }
    if page.authority == TRIGGER_AUTHORITY_CERTIFIED_PROFILE {
        if page.profile_identity == 0 || page.profile_hash == 0 {
            return Err(PageCodecError::Invalid);
        }
        if page.expert_unlock != EXPERT_UNLOCK_LOCKED {
            return Err(PageCodecError::Invalid);
        }
    }

    if page.primary_base_teeth == 0 {
        return Err(PageCodecError::Invalid);
    }
    if page.trigger_pattern == TRIGGER_PATTERN_MISSING_TOOTH {
        if page.primary_base_teeth < 2
            || page.missing_teeth == 0
            || page.missing_teeth >= page.primary_base_teeth
        {
            return Err(PageCodecError::Invalid);
        }
    } else if page.missing_teeth != 0 {
        return Err(PageCodecError::Invalid);
    }

    if page.trigger_angle_atdc_deg10 > 7200 {
        return Err(PageCodecError::Invalid);
    }
    if page.trigger_angle_multiplier == 0 || page.trigger_angle_multiplier > 8 {
        return Err(PageCodecError::Invalid);
    }
    if page.resync_every_cycle && page.secondary_trigger_mode == SECONDARY_TRIGGER_NONE {
        return Err(PageCodecError::Invalid);
    }
    if (page.ignition_mode == EXPERT_IGNITION_SEQUENTIAL_COP
        || page.injection_layout == EXPERT_INJECTION_SEQUENTIAL)
        && page.secondary_trigger_mode == SECONDARY_TRIGGER_NONE
    {
        return Err(PageCodecError::Invalid);
    }
    if page.skip_cycles > 16 {
        return Err(PageCodecError::Invalid);
    }
    if page.fixed_timing_mode == FIXED_TIMING_FIXED
        && !(-100..=600).contains(&page.fixed_timing_deg10)
    {
        return Err(PageCodecError::Invalid);
    }

    if let Some(current) = current {
        if current.authority == TRIGGER_AUTHORITY_CERTIFIED_PROFILE {
            if page.authority == TRIGGER_AUTHORITY_CERTIFIED_PROFILE && page != current {
                return Err(PageCodecError::Invalid);
            }
            if page.authority != TRIGGER_AUTHORITY_CERTIFIED_PROFILE
                && page.expert_unlock != EXPERT_UNLOCK_UNLOCKED
            {
                return Err(PageCodecError::Invalid);
            }
        }
    }

    Ok(())
}
