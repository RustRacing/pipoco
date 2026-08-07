use super::*;

/// Renders the AE/DFCO/WUE/ASE setup pages from the grouped enrichment setup structs.
pub fn read_enrichment_page(
    page: u8,
    ae: &AeSetup,
    dfco: &DfcoSetup,
    wue: &WueSetup,
    ase: &AseSetup,
    out: &mut [u8],
) -> Option<usize> {
    match page {
        PAGE_AE => AePage::new(
            ae.tpsdot_thresh_pct_s,
            ae.mapdot_thresh_kpa_s,
            ae.percent_gain,
            ae.decay_time_ms,
            ae.lockout_ms,
        )
        .encode(out)
        .ok(),
        PAGE_DFCO => DfcoPage::new(
            dfco.tps_max_pct,
            dfco.map_max_kpa,
            dfco.rpm_min,
            dfco.rpm_max,
            dfco.delay_ms,
            dfco.resume_hyst_ms,
        )
        .encode(out)
        .ok(),
        PAGE_WUE => WuePage::new(wue.max_percent, wue.min_percent, wue.start_c, wue.end_c)
            .encode(out)
            .ok(),
        PAGE_ASE => AsePage::new(ase.percent, ase.taper_time_ms, ase.lockout_ms as u16)
            .encode(out)
            .ok(),
        _ => None,
    }
}

/// Decodes an AE/DFCO/WUE/ASE setup page into the matching grouped setup struct.
pub fn write_enrichment_page(
    page: u8,
    data: &[u8],
    ae: &mut AeSetup,
    dfco: &mut DfcoSetup,
    wue: &mut WueSetup,
    ase: &mut AseSetup,
) -> Result<(), PageError> {
    match page {
        PAGE_AE => {
            let p = AePage::decode(data)?;
            ae.tpsdot_thresh_pct_s = p.tpsdot_thresh_pct_s;
            ae.mapdot_thresh_kpa_s = p.mapdot_thresh_kpa_s;
            ae.percent_gain = p.percent_gain;
            ae.decay_time_ms = p.decay_time_ms;
            ae.lockout_ms = p.lockout_ms;
            Ok(())
        }
        PAGE_DFCO => {
            let p = DfcoPage::decode(data)?;
            dfco.tps_max_pct = p.tps_max_pct;
            dfco.map_max_kpa = p.map_max_kpa;
            dfco.rpm_min = p.rpm_min;
            dfco.rpm_max = p.rpm_max;
            dfco.delay_ms = p.delay_ms;
            dfco.resume_hyst_ms = p.resume_hyst_ms;
            Ok(())
        }
        PAGE_WUE => {
            let p = WuePage::decode(data)?;
            wue.max_percent = p.max_percent;
            wue.min_percent = p.min_percent;
            wue.start_c = p.start_c;
            wue.end_c = p.end_c;
            Ok(())
        }
        PAGE_ASE => {
            let p = AsePage::decode(data)?;
            ase.percent = p.percent;
            ase.taper_time_ms = p.taper_time_ms;
            ase.lockout_ms = p.lockout_ms as u32;
            Ok(())
        }
        _ => Err(PageError::Invalid),
    }
}
