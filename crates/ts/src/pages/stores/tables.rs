use super::*;

impl<'a> FuelIgnPageStore<'a> {
    fn read_fuel(&self, out: &mut [u8]) -> Option<usize> {
        encode_fuel_table_page(self.fuel, out).ok()
    }

    fn write_fuel(&mut self, data: &[u8]) -> Result<(), PageError> {
        decode_fuel_table_page_into(data, self.fuel)
    }

    fn read_ign(&self, out: &mut [u8]) -> Option<usize> {
        encode_ignition_table_page(self.ign, out).ok()
    }

    fn write_ign(&mut self, data: &[u8]) -> Result<(), PageError> {
        decode_ignition_table_page_into(data, self.ign)
    }
}

impl<'a> PageStore for FuelIgnPageStore<'a> {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_FUEL | PAGE_IGN => Some(TABLE_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_FUEL => self.read_fuel(out),
            PAGE_IGN => self.read_ign(out),
            _ => None,
        }
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_FUEL => self.write_fuel(data),
            PAGE_IGN => self.write_ign(data),
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}
