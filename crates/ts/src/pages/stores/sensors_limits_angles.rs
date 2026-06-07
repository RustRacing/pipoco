use super::*;

impl LimitsPageStore<'_> {
    pub fn read_page_from(page: u8, limits: &LimitsSetup, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_LIMITS => LimitsPage::new(
                limits.map_min_kpa_x10,
                limits.map_max_kpa_x10,
                limits.tps_min_percent,
                limits.tps_max_percent,
                limits.clear_time_s,
                limits.emerg_trig_map,
                limits.emerg_trig_tps,
            )
            .encode(out)
            .ok(),
            _ => None,
        }
    }

    fn write_limits(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = LimitsPage::decode(data).map_err(page_codec_error_to_page_error)?;
        *self.map_min_kpa_x10 = page.map_min_kpa_x10;
        *self.map_max_kpa_x10 = page.map_max_kpa_x10;
        *self.tps_min_percent = page.tps_min_percent;
        *self.tps_max_percent = page.tps_max_percent;
        *self.clear_time_s = page.clear_time_s;
        *self.emerg_trig_map = page.emerg_trig_map;
        *self.emerg_trig_tps = page.emerg_trig_tps;
        Ok(())
    }
}

impl PageStore for LimitsPageStore<'_> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_LIMITS => Some(LIMITS_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            &LimitsSetup {
                map_min_kpa_x10: *self.map_min_kpa_x10,
                map_max_kpa_x10: *self.map_max_kpa_x10,
                tps_min_percent: *self.tps_min_percent,
                tps_max_percent: *self.tps_max_percent,
                clear_time_s: *self.clear_time_s,
                emerg_trig_map: *self.emerg_trig_map,
                emerg_trig_tps: *self.emerg_trig_tps,
            },
            out,
        )
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_LIMITS => self.write_limits(data),
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}

impl SensorsPageStore<'_> {
    pub fn read_page_from(page: u8, sensors: &SensorsSetup, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_SENSORS => SensorsPage {
                tps_min_counts: sensors.tps_min_counts,
                tps_max_counts: sensors.tps_max_counts,
                map_v0_mv: sensors.map_v0_mv,
                map_kpa0_x10: sensors.map_kpa0_x10,
                map_v1_mv: sensors.map_v1_mv,
                map_kpa1_x10: sensors.map_kpa1_x10,
                clt_deg_c: *sensors.clt_deg_c,
                iat_deg_c: *sensors.iat_deg_c,
                clt_ohms: *sensors.clt_ohms,
                iat_ohms: *sensors.iat_ohms,
            }
            .encode(out)
            .ok(),
            _ => None,
        }
    }

    fn write_sensors(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = SensorsPage::decode(data).map_err(page_codec_error_to_page_error)?;
        *self.tps_min_counts = page.tps_min_counts;
        *self.tps_max_counts = page.tps_max_counts;
        *self.map_v0_mv = page.map_v0_mv;
        *self.map_kpa0_x10 = page.map_kpa0_x10;
        *self.map_v1_mv = page.map_v1_mv;
        *self.map_kpa1_x10 = page.map_kpa1_x10;
        *self.clt_deg_c = page.clt_deg_c;
        *self.iat_deg_c = page.iat_deg_c;
        *self.clt_ohms = page.clt_ohms;
        *self.iat_ohms = page.iat_ohms;
        Ok(())
    }
}

impl PageStore for SensorsPageStore<'_> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_SENSORS => Some(SENSORS_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            &SensorsSetup {
                tps_min_counts: *self.tps_min_counts,
                tps_max_counts: *self.tps_max_counts,
                map_v0_mv: *self.map_v0_mv,
                map_kpa0_x10: *self.map_kpa0_x10,
                map_v1_mv: *self.map_v1_mv,
                map_kpa1_x10: *self.map_kpa1_x10,
                clt_deg_c: self.clt_deg_c,
                iat_deg_c: self.iat_deg_c,
                clt_ohms: self.clt_ohms,
                iat_ohms: self.iat_ohms,
            },
            out,
        )
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_SENSORS => self.write_sensors(data),
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}

impl AnglesPageStore<'_> {
    pub fn read_page_from(page: u8, angles: &AnglesSetup, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_ANGLES => AnglesPage::new(
                *angles.inj_angles_x10,
                *angles.tdc_angles_x10,
                angles.tooth0_angle_x10,
                angles.cam_timeout_ms,
            )
            .encode(out)
            .ok(),
            _ => None,
        }
    }

    fn write_angles(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = AnglesPage::decode(data).map_err(page_codec_error_to_page_error)?;
        *self.inj_angles_x10 = page.inj_angles_x10;
        *self.tdc_angles_x10 = page.tdc_angles_x10;
        *self.tooth0_angle_x10 = page.tooth0_angle_x10;
        *self.cam_timeout_ms = page.cam_timeout_ms;
        Ok(())
    }
}

impl PageStore for AnglesPageStore<'_> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_ANGLES => Some(ANGLES_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            &AnglesSetup {
                inj_angles_x10: self.inj_angles_x10,
                tdc_angles_x10: self.tdc_angles_x10,
                tooth0_angle_x10: *self.tooth0_angle_x10,
                cam_timeout_ms: *self.cam_timeout_ms,
            },
            out,
        )
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_ANGLES => self.write_angles(data),
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}
