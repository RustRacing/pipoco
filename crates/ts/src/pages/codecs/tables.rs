use super::*;

pub fn encode_fuel_table_page(table: &FuelTable, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            out[idx..idx + 2].copy_from_slice(&cell.to_le_bytes());
            idx += 2;
        }
    }
    Ok(TABLE_PAGE_BYTES)
}

pub fn decode_fuel_table_page(data: &[u8]) -> Result<FuelPage, PageCodecError> {
    let mut cells = [[0u16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
    decode_fuel_table_page_into(data, &mut cells)?;
    Ok(FuelPage { cells })
}

pub fn decode_fuel_table_page_into(
    data: &[u8],
    table: &mut FuelTable,
) -> Result<(), PageCodecError> {
    if data.len() != TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            *cell = u16::from_le_bytes([data[idx], data[idx + 1]]);
            idx += 2;
        }
    }
    Ok(())
}

pub fn encode_ignition_table_page(
    table: &IgnitionTable,
    out: &mut [u8],
) -> Result<usize, PageCodecError> {
    if out.len() < TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            out[idx..idx + 2].copy_from_slice(&cell.to_le_bytes());
            idx += 2;
        }
    }
    Ok(TABLE_PAGE_BYTES)
}

pub fn decode_ignition_table_page(data: &[u8]) -> Result<IgnitionPage, PageCodecError> {
    let mut cells = [[0i16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
    decode_ignition_table_page_into(data, &mut cells)?;
    Ok(IgnitionPage { cells })
}

pub fn decode_ignition_table_page_into(
    data: &[u8],
    table: &mut IgnitionTable,
) -> Result<(), PageCodecError> {
    if data.len() != TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            *cell = i16::from_le_bytes([data[idx], data[idx + 1]]);
            idx += 2;
        }
    }
    Ok(())
}

pub fn encode_ve_table_page(table: &VeTable, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            out[idx..idx + 2].copy_from_slice(&cell.to_le_bytes());
            idx += 2;
        }
    }
    Ok(TABLE_PAGE_BYTES)
}

pub fn decode_ve_table_page(data: &[u8]) -> Result<VePage, PageCodecError> {
    let mut cells = [[0u16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
    decode_ve_table_page_into(data, &mut cells)?;
    Ok(VePage { cells })
}

pub fn decode_ve_table_page_into(data: &[u8], table: &mut VeTable) -> Result<(), PageCodecError> {
    if data.len() != TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            *cell = u16::from_le_bytes([data[idx], data[idx + 1]]);
            idx += 2;
        }
    }
    Ok(())
}

pub fn encode_afr_table_page(table: &AfrTable, out: &mut [u8]) -> Result<usize, PageCodecError> {
    if out.len() < TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            out[idx..idx + 2].copy_from_slice(&cell.to_le_bytes());
            idx += 2;
        }
    }
    Ok(TABLE_PAGE_BYTES)
}

pub fn decode_afr_table_page(data: &[u8]) -> Result<AfrPage, PageCodecError> {
    let mut cells = [[0u16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
    decode_afr_table_page_into(data, &mut cells)?;
    Ok(AfrPage { cells })
}

pub fn decode_afr_table_page_into(data: &[u8], table: &mut AfrTable) -> Result<(), PageCodecError> {
    if data.len() != TABLE_PAGE_BYTES {
        return Err(PageCodecError::WrongSize);
    }

    let mut idx = 0;
    for row in table {
        for cell in row {
            let target = u16::from_le_bytes([data[idx], data[idx + 1]]);
            if !(AFR_TARGET_MIN_X10..=AFR_TARGET_MAX_X10).contains(&target) {
                return Err(PageCodecError::Invalid);
            }
            *cell = target;
            idx += 2;
        }
    }
    Ok(())
}
