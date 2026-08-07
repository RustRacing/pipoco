use super::*;

impl FuelTunePageStore<'_> {
    pub fn read_page_from(
        page: u8,
        ve: &VeTable,
        afr: &AfrTable,
        tune: &VeTuneSetup,
        limits: VeTunePageLimits,
        out: &mut [u8],
    ) -> Option<usize> {
        match page {
            PAGE_VE_TUNE => {
                let page = VeTunePage::new(
                    tune.target_afr_x10,
                    tune.kp_i,
                    tune.ki_i,
                    tune.required_fuel_us,
                    tune.injector_deadtime_us,
                    tune.ve_load_source,
                );
                encode_ve_tune_page(&page, limits, out).ok()
            }
            PAGE_VE_TABLE => encode_ve_table_page(ve, out).ok(),
            PAGE_AFR_TABLE => encode_afr_table_page(afr, out).ok(),
            _ => None,
        }
    }

    fn write_ve_tune(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = decode_ve_tune_page(data).and_then(|page| page.apply_limits(self.limits))?;
        *self.target_afr_x10 = page.target_afr_x10;
        *self.kp_i = page.kp_i;
        *self.ki_i = page.ki_i;
        *self.required_fuel_us = page.required_fuel_us;
        *self.injector_deadtime_us = page.injector_deadtime_us;
        *self.ve_load_source = page.ve_load_source;
        Ok(())
    }

    fn write_ve_table(&mut self, data: &[u8]) -> Result<(), PageError> {
        decode_ve_table_page_into(data, self.ve)
    }

    fn write_afr_table(&mut self, data: &[u8]) -> Result<(), PageError> {
        decode_afr_table_page_into(data, self.afr)?;
        *self.target_afr_x10 = self.afr[0][0];
        Ok(())
    }
}

impl PageStore for FuelTunePageStore<'_> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_VE_TUNE => Some(VE_TUNE_PAGE_BYTES),
            PAGE_VE_TABLE | PAGE_AFR_TABLE => Some(TABLE_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            self.ve,
            self.afr,
            &VeTuneSetup {
                target_afr_x10: *self.target_afr_x10,
                kp_i: *self.kp_i,
                ki_i: *self.ki_i,
                required_fuel_us: *self.required_fuel_us,
                injector_deadtime_us: *self.injector_deadtime_us,
                ve_load_source: *self.ve_load_source,
            },
            self.limits,
            out,
        )
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_VE_TUNE => self.write_ve_tune(data),
            PAGE_VE_TABLE => self.write_ve_table(data),
            PAGE_AFR_TABLE => self.write_afr_table(data),
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}
