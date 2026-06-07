use crate::server::{PageError, PageStore, PersistError};

use super::codecs::*;
use super::dto::*;
use super::registry::*;

mod actuators;
mod diagnostics_snapshot;
mod enrichment;
mod expert_trigger;
mod fuel_tune;
mod sensors_limits_angles;
mod tables;

pub use enrichment::*;
pub use expert_trigger::*;

pub type FuelTable = [[u16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
pub type IgnitionTable = [[i16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
pub type VeTable = [[u16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];
pub type AfrTable = [[u16; TABLE_AXIS_LEN]; TABLE_AXIS_LEN];

fn page_codec_error_to_page_error(err: PageCodecError) -> PageError {
    match err {
        PageCodecError::WrongSize => PageError::WrongSize,
        PageCodecError::Invalid => PageError::Invalid,
    }
}

/// Core-free page store for the setup tables that are just TS wire tables.
pub struct FuelIgnPageStore<'a> {
    pub fuel: &'a mut FuelTable,
    pub ign: &'a mut IgnitionTable,
}

/// VE-tune scalar setup grouped for the page boundary.
#[derive(Copy, Clone)]
pub struct VeTuneSetup {
    pub target_afr_x10: u16,
    pub kp_i: u16,
    pub ki_i: u16,
    pub required_fuel_us: u16,
    pub injector_deadtime_us: u16,
    pub ve_load_source: u8,
}

/// Core-free page store for VE fueling setup pages.
pub struct FuelTunePageStore<'a> {
    pub ve: &'a mut VeTable,
    pub afr: &'a mut AfrTable,
    pub target_afr_x10: &'a mut u16,
    pub kp_i: &'a mut u16,
    pub ki_i: &'a mut u16,
    pub required_fuel_us: &'a mut u16,
    pub injector_deadtime_us: &'a mut u16,
    pub ve_load_source: &'a mut u8,
    pub limits: VeTunePageLimits,
}

/// Idle-control setup, mirroring the core idle config for the page boundary.
#[derive(Copy, Clone)]
pub struct IdleSetup {
    pub enable: bool,
    pub duty_x10: u16,
    pub freq_hz: u16,
}

/// Fan-control setup, mirroring the core fan config for the page boundary.
#[derive(Copy, Clone)]
pub struct FanSetup {
    pub enable: bool,
    pub on_c: i16,
    pub off_c: i16,
}

/// Closed-loop setup, mirroring the core closed-loop config for the page boundary.
#[derive(Copy, Clone)]
pub struct ClSetup {
    pub enable: bool,
    pub target_afr_x10: u16,
    pub kp_i: u16,
    pub ki_i: u16,
}

/// Core-free page store for scalar actuator setup pages.
pub struct ActuatorPageStore<'a> {
    pub idle_enable: &'a mut bool,
    pub idle_duty_x10: &'a mut u16,
    pub idle_freq_hz: &'a mut u16,
    pub fan_enable: &'a mut bool,
    pub fan_on_c: &'a mut i16,
    pub fan_off_c: &'a mut i16,
    pub cl_enable: &'a mut bool,
    pub cl_target_afr_x10: &'a mut u16,
    pub cl_kp_i: &'a mut u16,
    pub cl_ki_i: &'a mut u16,
}

/// Sensor/emergency limit setup grouped for the page boundary.
#[derive(Copy, Clone)]
pub struct LimitsSetup {
    pub map_min_kpa_x10: u16,
    pub map_max_kpa_x10: u16,
    pub tps_min_percent: u8,
    pub tps_max_percent: u8,
    pub clear_time_s: u16,
    pub emerg_trig_map: bool,
    pub emerg_trig_tps: bool,
}

/// Sensor calibration setup grouped for the page boundary.
#[derive(Copy, Clone)]
pub struct SensorsSetup<'a> {
    pub tps_min_counts: u16,
    pub tps_max_counts: u16,
    pub map_v0_mv: u16,
    pub map_kpa0_x10: u16,
    pub map_v1_mv: u16,
    pub map_kpa1_x10: u16,
    pub clt_deg_c: &'a [i16; 8],
    pub iat_deg_c: &'a [i16; 8],
    pub clt_ohms: &'a [u32; 8],
    pub iat_ohms: &'a [u32; 8],
}

/// Trigger-angle setup grouped for the page boundary.
#[derive(Copy, Clone)]
pub struct AnglesSetup<'a> {
    pub inj_angles_x10: &'a [u16; 16],
    pub tdc_angles_x10: &'a [u16; 16],
    pub tooth0_angle_x10: u16,
    pub cam_timeout_ms: u16,
}

/// Core-free page store for scalar sensor/emergency limit setup.
pub struct LimitsPageStore<'a> {
    pub map_min_kpa_x10: &'a mut u16,
    pub map_max_kpa_x10: &'a mut u16,
    pub tps_min_percent: &'a mut u8,
    pub tps_max_percent: &'a mut u8,
    pub clear_time_s: &'a mut u16,
    pub emerg_trig_map: &'a mut bool,
    pub emerg_trig_tps: &'a mut bool,
}

/// Core-free page store for scalar sensor calibration setup.
pub struct SensorsPageStore<'a> {
    pub tps_min_counts: &'a mut u16,
    pub tps_max_counts: &'a mut u16,
    pub map_v0_mv: &'a mut u16,
    pub map_kpa0_x10: &'a mut u16,
    pub map_v1_mv: &'a mut u16,
    pub map_kpa1_x10: &'a mut u16,
    pub clt_deg_c: &'a mut [i16; 8],
    pub iat_deg_c: &'a mut [i16; 8],
    pub clt_ohms: &'a mut [u32; 8],
    pub iat_ohms: &'a mut [u32; 8],
}

/// Core-free page store for trigger angle setup.
pub struct AnglesPageStore<'a> {
    pub inj_angles_x10: &'a mut [u16; 16],
    pub tdc_angles_x10: &'a mut [u16; 16],
    pub tooth0_angle_x10: &'a mut u16,
    pub cam_timeout_ms: &'a mut u16,
}

/// Core-free page store for the runtime snapshot page.
pub struct SnapshotPageStore {
    pub rpm: u16,
    pub sync_code: u8,
    pub base_pw_us: u32,
    pub enrich_mult_x100: u16,
    pub stft_x10: i16,
    pub fuel_mult_x100: u16,
    pub final_pw_us: u32,
    pub fault_code: u8,
    pub isr_count: u32,
    pub isr_max_us: u32,
    pub isr_avg_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticLogEntry {
    pub code: u8,
    pub start_us: u32,
    pub end_us: u32,
}

/// Grouped diagnostic-page scalars, replacing the positional tuple at the page boundary.
#[derive(Copy, Clone)]
pub struct DiagnosticSnapshot {
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
}

/// Core-free page store for diagnostic runtime pages.
pub struct DiagnosticPageStore {
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
    pub log_entries: [Option<DiagnosticLogEntry>; DIAG_LOG_ENTRY_COUNT],
}

/// Acceleration-enrichment setup, mirroring the core AE config for the page boundary.
#[derive(Copy, Clone)]
pub struct AeSetup {
    pub tpsdot_thresh_pct_s: i16,
    pub mapdot_thresh_kpa_s: i16,
    pub percent_gain: u8,
    pub decay_time_ms: u32,
    pub lockout_ms: u32,
}

/// Decel-fuel-cut setup, mirroring the core DFCO config for the page boundary.
#[derive(Copy, Clone)]
pub struct DfcoSetup {
    pub tps_max_pct: u8,
    pub map_max_kpa: u16,
    pub rpm_min: u16,
    pub rpm_max: u16,
    pub delay_ms: u32,
    pub resume_hyst_ms: u32,
}

/// Warm-up-enrichment setup, mirroring the core WUE config for the page boundary.
#[derive(Copy, Clone)]
pub struct WueSetup {
    pub max_percent: u8,
    pub min_percent: u8,
    pub start_c: i16,
    pub end_c: i16,
}

/// After-start-enrichment setup, mirroring the core ASE config for the page boundary.
#[derive(Copy, Clone)]
pub struct AseSetup {
    pub percent: u8,
    pub taper_time_ms: u32,
    pub lockout_ms: u32,
}
