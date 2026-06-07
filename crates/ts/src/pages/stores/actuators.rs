use super::*;

impl ActuatorPageStore<'_> {
    pub fn read_page_from(
        page: u8,
        idle: &IdleSetup,
        fan: &FanSetup,
        cl: &ClSetup,
        out: &mut [u8],
    ) -> Option<usize> {
        match page {
            PAGE_IDLE => IdlePage::new(idle.enable, idle.duty_x10, idle.freq_hz)
                .encode(out)
                .ok(),
            PAGE_FAN => FanPage::new(fan.enable, fan.on_c, fan.off_c)
                .encode(out)
                .ok(),
            PAGE_CL => ClosedLoopPage::new(cl.enable, cl.target_afr_x10, cl.kp_i, cl.ki_i)
                .encode(out)
                .ok(),
            _ => None,
        }
    }

    fn write_idle(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = IdlePage::decode(data).map_err(page_codec_error_to_page_error)?;
        *self.idle_enable = page.enable;
        *self.idle_duty_x10 = page.duty_x10;
        *self.idle_freq_hz = page.freq_hz;
        Ok(())
    }

    fn write_fan(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = FanPage::decode(data).map_err(page_codec_error_to_page_error)?;
        *self.fan_enable = page.enable;
        *self.fan_on_c = page.on_c;
        *self.fan_off_c = page.off_c;
        Ok(())
    }

    fn write_cl(&mut self, data: &[u8]) -> Result<(), PageError> {
        let page = ClosedLoopPage::decode(data).map_err(page_codec_error_to_page_error)?;
        *self.cl_enable = page.enable;
        *self.cl_target_afr_x10 = page.target_afr_x10;
        *self.cl_kp_i = page.kp_i;
        *self.cl_ki_i = page.ki_i;
        Ok(())
    }
}

impl PageStore for ActuatorPageStore<'_> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_IDLE => Some(IDLE_PAGE_BYTES),
            PAGE_FAN => Some(FAN_PAGE_BYTES),
            PAGE_CL => Some(CLOSED_LOOP_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        Self::read_page_from(
            page,
            &IdleSetup {
                enable: *self.idle_enable,
                duty_x10: *self.idle_duty_x10,
                freq_hz: *self.idle_freq_hz,
            },
            &FanSetup {
                enable: *self.fan_enable,
                on_c: *self.fan_on_c,
                off_c: *self.fan_off_c,
            },
            &ClSetup {
                enable: *self.cl_enable,
                target_afr_x10: *self.cl_target_afr_x10,
                kp_i: *self.cl_kp_i,
                ki_i: *self.cl_ki_i,
            },
            out,
        )
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_IDLE => self.write_idle(data),
            PAGE_FAN => self.write_fan(data),
            PAGE_CL => self.write_cl(data),
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}
