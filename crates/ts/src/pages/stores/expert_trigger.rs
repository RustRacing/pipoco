use super::*;

const fn default_expert_trigger_record() -> [u8; EXPERT_TRIGGER_PAGE_BYTES] {
    let page = ExpertTriggerPage::default_layout();
    let mut record = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
    record[0] = page.schema_version as u8;
    record[1] = (page.schema_version >> 8) as u8;
    record[2] = page.expert_unlock;
    record[3] = page.authority;
    record[4] = page.profile_identity as u8;
    record[5] = (page.profile_identity >> 8) as u8;
    record[6] = (page.profile_identity >> 16) as u8;
    record[7] = (page.profile_identity >> 24) as u8;
    record[8] = page.profile_hash as u8;
    record[9] = (page.profile_hash >> 8) as u8;
    record[10] = (page.profile_hash >> 16) as u8;
    record[11] = (page.profile_hash >> 24) as u8;
    record[12] = page.trigger_pattern;
    record[13] = page.primary_base_teeth;
    record[14] = page.missing_teeth;
    record[15] = page.primary_trigger_speed;
    record[16] = page.trigger_angle_atdc_deg10 as u8;
    record[17] = (page.trigger_angle_atdc_deg10 >> 8) as u8;
    record[18] = page.trigger_angle_multiplier;
    record[19] = page.primary_trigger_edge;
    record[20] = page.secondary_trigger_edge;
    record[21] = page.secondary_trigger_mode;
    record[22] = page.poll_level_polarity;
    record[23] = page.trigger_filter;
    record[24] = page.resync_every_cycle as u8;
    record[25] = page.skip_cycles;
    record[26] = page.ignition_mode;
    record[27] = page.injection_layout;
    record[28] = page.fixed_timing_mode;
    record[30] = page.fixed_timing_deg10 as u8;
    record[31] = ((page.fixed_timing_deg10 as u16) >> 8) as u8;
    record
}

pub const DEFAULT_EXPERT_TRIGGER_RECORD: [u8; EXPERT_TRIGGER_PAGE_BYTES] =
    default_expert_trigger_record();

/// Root-owned view of the canonical expert-trigger TS record layout.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ExpertTriggerPageState {
    record: [u8; EXPERT_TRIGGER_PAGE_BYTES],
}
impl ExpertTriggerPageState {
    pub const fn new() -> Self {
        Self {
            record: DEFAULT_EXPERT_TRIGGER_RECORD,
        }
    }

    pub fn authority_code(&self) -> u8 {
        self.record[3]
    }

    pub fn profile_identity(&self) -> u32 {
        u32::from_le_bytes([
            self.record[4],
            self.record[5],
            self.record[6],
            self.record[7],
        ])
    }

    pub fn profile_hash(&self) -> u32 {
        u32::from_le_bytes([
            self.record[8],
            self.record[9],
            self.record[10],
            self.record[11],
        ])
    }
}

impl Default for ExpertTriggerPageState {
    fn default() -> Self {
        Self::new()
    }
}

impl PageStore for ExpertTriggerPageState {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_EXPERT_TRIGGER => Some(EXPERT_TRIGGER_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        if page != PAGE_EXPERT_TRIGGER || out.len() < EXPERT_TRIGGER_PAGE_BYTES {
            return None;
        }
        out[..EXPERT_TRIGGER_PAGE_BYTES].copy_from_slice(&self.record);
        Some(EXPERT_TRIGGER_PAGE_BYTES)
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        if page != PAGE_EXPERT_TRIGGER {
            return Err(PageError::Invalid);
        }
        let current = decode_expert_trigger_record(&self.record)?;
        let proposed = decode_expert_trigger_page_with_current(data, Some(&current))?;
        encode_expert_trigger_page(&proposed, &mut self.record)?;
        Ok(())
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        Ok(())
    }
}

pub fn decode_expert_trigger_record(data: &[u8]) -> Result<ExpertTriggerPage, PageError> {
    decode_expert_trigger_page(data)
}
