use super::codecs::*;
use super::registry::*;
use super::stores::*;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelPage {
    pub cells: FuelTable,
}

impl FuelPage {
    pub const fn new(cells: FuelTable) -> Self {
        Self { cells }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_fuel_table_page(&self.cells, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_fuel_table_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IgnitionPage {
    pub cells: IgnitionTable,
}

impl IgnitionPage {
    pub const fn new(cells: IgnitionTable) -> Self {
        Self { cells }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_ignition_table_page(&self.cells, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_ignition_table_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VePage {
    pub cells: VeTable,
}

impl VePage {
    pub const fn new(cells: VeTable) -> Self {
        Self { cells }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_ve_table_page(&self.cells, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_ve_table_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AfrPage {
    pub cells: AfrTable,
}

impl AfrPage {
    pub const fn new(cells: AfrTable) -> Self {
        Self { cells }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_afr_table_page(&self.cells, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_afr_table_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AePage {
    pub tpsdot_thresh_pct_s: i16,
    pub mapdot_thresh_kpa_s: i16,
    pub percent_gain: u8,
    pub decay_time_ms: u32,
    pub lockout_ms: u32,
}

impl AePage {
    pub const fn new(
        tpsdot_thresh_pct_s: i16,
        mapdot_thresh_kpa_s: i16,
        percent_gain: u8,
        decay_time_ms: u32,
        lockout_ms: u32,
    ) -> Self {
        Self {
            tpsdot_thresh_pct_s,
            mapdot_thresh_kpa_s,
            percent_gain,
            decay_time_ms,
            lockout_ms,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_ae_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_ae_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DfcoPage {
    pub tps_max_pct: u8,
    pub map_max_kpa: u16,
    pub rpm_min: u16,
    pub rpm_max: u16,
    pub delay_ms: u32,
    pub resume_hyst_ms: u32,
}

impl DfcoPage {
    pub const fn new(
        tps_max_pct: u8,
        map_max_kpa: u16,
        rpm_min: u16,
        rpm_max: u16,
        delay_ms: u32,
        resume_hyst_ms: u32,
    ) -> Self {
        Self {
            tps_max_pct,
            map_max_kpa,
            rpm_min,
            rpm_max,
            delay_ms,
            resume_hyst_ms,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_dfco_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_dfco_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WuePage {
    pub max_percent: u8,
    pub min_percent: u8,
    pub start_c: i16,
    pub end_c: i16,
}

impl WuePage {
    pub const fn new(max_percent: u8, min_percent: u8, start_c: i16, end_c: i16) -> Self {
        Self {
            max_percent,
            min_percent,
            start_c,
            end_c,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_wue_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_wue_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AsePage {
    pub percent: u8,
    pub taper_time_ms: u32,
    pub lockout_ms: u16,
}

impl AsePage {
    pub const fn new(percent: u8, taper_time_ms: u32, lockout_ms: u16) -> Self {
        Self {
            percent,
            taper_time_ms,
            lockout_ms,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_ase_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_ase_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdlePage {
    pub enable: bool,
    pub duty_x10: u16,
    pub freq_hz: u16,
}

impl IdlePage {
    pub const fn new(enable: bool, duty_x10: u16, freq_hz: u16) -> Self {
        Self {
            enable,
            duty_x10,
            freq_hz,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_idle_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_idle_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FanPage {
    pub enable: bool,
    pub on_c: i16,
    pub off_c: i16,
}

impl FanPage {
    pub const fn new(enable: bool, on_c: i16, off_c: i16) -> Self {
        Self {
            enable,
            on_c,
            off_c,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_fan_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_fan_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClosedLoopPage {
    pub enable: bool,
    pub target_afr_x10: u16,
    pub kp_i: u16,
    pub ki_i: u16,
}

impl ClosedLoopPage {
    pub const fn new(enable: bool, target_afr_x10: u16, kp_i: u16, ki_i: u16) -> Self {
        Self {
            enable,
            target_afr_x10,
            kp_i,
            ki_i,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_closed_loop_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_closed_loop_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AnglesPage {
    pub inj_angles_x10: [u16; 16],
    pub tdc_angles_x10: [u16; 16],
    pub tooth0_angle_x10: u16,
    pub cam_timeout_ms: u16,
}

impl AnglesPage {
    pub const fn new(
        inj_angles_x10: [u16; 16],
        tdc_angles_x10: [u16; 16],
        tooth0_angle_x10: u16,
        cam_timeout_ms: u16,
    ) -> Self {
        Self {
            inj_angles_x10,
            tdc_angles_x10,
            tooth0_angle_x10,
            cam_timeout_ms,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_angles_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_angles_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorsPage {
    pub tps_min_counts: u16,
    pub tps_max_counts: u16,
    pub map_v0_mv: u16,
    pub map_kpa0_x10: u16,
    pub map_v1_mv: u16,
    pub map_kpa1_x10: u16,
    pub clt_deg_c: [i16; 8],
    pub iat_deg_c: [i16; 8],
    pub clt_ohms: [u32; 8],
    pub iat_ohms: [u32; 8],
}

impl SensorsPage {
    /// Encode into the wire layout, leaving reserved tail bytes (108..128)
    /// untouched. Returns the full page length on success.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_sensors_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_sensors_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LimitsPage {
    pub map_min_kpa_x10: u16,
    pub map_max_kpa_x10: u16,
    pub tps_min_percent: u8,
    pub tps_max_percent: u8,
    pub clear_time_s: u16,
    pub emerg_trig_map: bool,
    pub emerg_trig_tps: bool,
}

impl LimitsPage {
    pub const fn new(
        map_min_kpa_x10: u16,
        map_max_kpa_x10: u16,
        tps_min_percent: u8,
        tps_max_percent: u8,
        clear_time_s: u16,
        emerg_trig_map: bool,
        emerg_trig_tps: bool,
    ) -> Self {
        Self {
            map_min_kpa_x10,
            map_max_kpa_x10,
            tps_min_percent,
            tps_max_percent,
            clear_time_s,
            emerg_trig_map,
            emerg_trig_tps,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_limits_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_limits_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VeTunePage {
    pub target_afr_x10: u16,
    pub kp_i: u16,
    pub ki_i: u16,
    pub required_fuel_us: u16,
    pub injector_deadtime_us: u16,
    pub ve_load_source: u8,
}

impl VeTunePage {
    pub const fn new(
        target_afr_x10: u16,
        kp_i: u16,
        ki_i: u16,
        required_fuel_us: u16,
        injector_deadtime_us: u16,
        ve_load_source: u8,
    ) -> Self {
        Self {
            target_afr_x10,
            kp_i,
            ki_i,
            required_fuel_us,
            injector_deadtime_us,
            ve_load_source,
        }
    }

    pub fn encode(&self, out: &mut [u8], limits: VeTunePageLimits) -> Result<usize, PageError> {
        encode_ve_tune_page(self, limits, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_ve_tune_page(data)
    }

    pub fn apply_limits(mut self, limits: VeTunePageLimits) -> Result<Self, PageError> {
        limits.validate()?;
        self.required_fuel_us = self
            .required_fuel_us
            .clamp(limits.min_pulse_width_us, limits.max_pulse_width_us);
        self.injector_deadtime_us = self
            .injector_deadtime_us
            .min(limits.max_injector_deadtime_us);
        Ok(self)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SnapshotPage {
    pub rpm: u16,
    pub sync_code: u8,
    pub cancel_reason: u8,
    pub base_pw_us: u32,
    pub enrich_mult_x100: u16,
    pub stft_x10: i16,
    pub fuel_mult_x100: u16,
    pub final_pw_us: u32,
    pub fault_code: u8,
    pub fault_severity: u8,
    pub isr_count: u32,
    pub isr_max_us: u32,
    pub isr_avg_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagPage {
    pub current_tooth_count: u8,
    pub cam_seen: bool,
    pub sync_state: u8,
    pub phase_state: u8,
    pub absolute_authority: u8,
    pub trigger_angle_source: u8,
    pub output_gating_reason: u8,
    pub last_sync_loss_reason: u8,
    pub primary_rpm: u16,
    pub detected_gap_ratio: u16,
    pub sync_loss_counter: u16,
    pub board_pin_map_identity: u16,
    pub profile_identity: u32,
    pub profile_hash: u32,
    pub current_fault_code: u8,
    pub current_fault_severity: u8,
    pub current_fault_action: u8,
    pub current_cancel_reason: u8,
    pub fault_flags: u8,
    pub latest_diag_code: u8,
}

impl DiagPage {
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_diag_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_diag_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagLogEntryPage {
    pub code: u8,
    pub severity: u8,
    pub action: u8,
    pub source: u8,
    pub context_present: bool,
    pub context: u32,
    pub start_us: u32,
    pub end_us: u32,
}

impl DiagLogEntryPage {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        code: u8,
        severity: u8,
        action: u8,
        source: u8,
        context_present: bool,
        context: u32,
        start_us: u32,
        end_us: u32,
    ) -> Self {
        Self {
            code,
            severity,
            action,
            source,
            context_present,
            context,
            start_us,
            end_us,
        }
    }

    pub const fn empty() -> Self {
        Self {
            code: 0,
            severity: 0,
            action: 0,
            source: 0,
            context_present: false,
            context: 0,
            start_us: 0,
            end_us: 0,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagLogPage {
    pub entries: [DiagLogEntryPage; DIAG_LOG_ENTRY_COUNT],
}

impl DiagLogPage {
    pub const fn new(entries: [DiagLogEntryPage; DIAG_LOG_ENTRY_COUNT]) -> Self {
        Self { entries }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_diag_log_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_diag_log_page(data)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpertTriggerPage {
    pub schema_version: u16,
    pub expert_unlock: u8,
    pub authority: u8,
    pub profile_identity: u32,
    pub profile_hash: u32,
    pub trigger_pattern: u8,
    pub primary_base_teeth: u8,
    pub missing_teeth: u8,
    pub primary_trigger_speed: u8,
    pub trigger_angle_atdc_deg10: u16,
    pub trigger_angle_multiplier: u8,
    pub primary_trigger_edge: u8,
    pub secondary_trigger_edge: u8,
    pub secondary_trigger_mode: u8,
    pub poll_level_polarity: u8,
    pub trigger_filter: u8,
    pub resync_every_cycle: bool,
    pub skip_cycles: u8,
    pub ignition_mode: u8,
    pub injection_layout: u8,
    pub fixed_timing_mode: u8,
    pub fixed_timing_deg10: i16,
}

impl ExpertTriggerPage {
    pub const fn default_layout() -> Self {
        Self {
            schema_version: EXPERT_SCHEMA_VERSION_CURRENT,
            expert_unlock: EXPERT_UNLOCK_LOCKED,
            authority: 0,
            profile_identity: 0,
            profile_hash: 0,
            trigger_pattern: TRIGGER_PATTERN_MISSING_TOOTH,
            primary_base_teeth: 60,
            missing_teeth: 2,
            primary_trigger_speed: 0,
            trigger_angle_atdc_deg10: 0,
            trigger_angle_multiplier: 1,
            primary_trigger_edge: 0,
            secondary_trigger_edge: 0,
            secondary_trigger_mode: SECONDARY_TRIGGER_NONE,
            poll_level_polarity: 0,
            trigger_filter: 2,
            resync_every_cycle: false,
            skip_cycles: 2,
            ignition_mode: 1,
            injection_layout: 1,
            fixed_timing_mode: 0,
            fixed_timing_deg10: 100,
        }
    }

    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_expert_trigger_page(self, out)
    }

    pub fn decode(data: &[u8]) -> Result<Self, PageError> {
        decode_expert_trigger_page(data)
    }
}

impl SnapshotPage {
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, PageError> {
        encode_snapshot_page(self, out)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VeTunePageLimits {
    pub min_pulse_width_us: u16,
    pub max_pulse_width_us: u16,
    pub max_injector_deadtime_us: u16,
}

impl VeTunePageLimits {
    pub const fn new(min_pulse_width_us: u16, max_pulse_width_us: u16) -> Self {
        Self {
            min_pulse_width_us,
            max_pulse_width_us,
            max_injector_deadtime_us: VE_TUNE_INJECTOR_DEADTIME_MAX_US,
        }
    }

    pub const fn with_deadtime_cap(
        min_pulse_width_us: u16,
        max_pulse_width_us: u16,
        max_injector_deadtime_us: u16,
    ) -> Self {
        Self {
            min_pulse_width_us,
            max_pulse_width_us,
            max_injector_deadtime_us,
        }
    }

    pub(super) fn validate(self) -> Result<(), PageError> {
        if self.min_pulse_width_us > self.max_pulse_width_us {
            return Err(PageError::Invalid);
        }
        Ok(())
    }
}
