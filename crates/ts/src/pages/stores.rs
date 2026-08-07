use crate::server::{PageError, PageStore, PersistError};

use super::codecs::*;
use super::dto::*;
use super::registry::*;
#[cfg(feature = "runtime")]
use ecu_domain::diag::DiagCode;
#[cfg(feature = "runtime")]
use ecu_domain::{CancelReason, FaultCode, FaultSeverity};

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

pub const TS_CURRENT_FAULT_NONE: u8 = 0;
pub const TS_CURRENT_FAULT_MAP_RANGE: u8 = 1;
pub const TS_CURRENT_FAULT_TPS_RANGE: u8 = 2;
pub const TS_CURRENT_FAULT_CAM_MISSING: u8 = 3;
pub const TS_CURRENT_FAULT_LOW_VOLTAGE: u8 = 4;
pub const TS_CURRENT_FAULT_OVERVOLTAGE: u8 = 5;
pub const TS_CURRENT_FAULT_MAP_FAILURE_HIGH_LOAD: u8 = 6;
pub const TS_CURRENT_FAULT_TPS_MAP_PLAUSIBILITY: u8 = 7;
pub const TS_CURRENT_FAULT_KNOCK_DETECTED: u8 = 8;
pub const TS_CURRENT_FAULT_PERSIST_CRC_FAULT: u8 = 9;
pub const TS_CURRENT_FAULT_SYNC_LOSS: u8 = 10;
pub const TS_CURRENT_FAULT_SENSOR_OUT_OF_RANGE: u8 = 11;
pub const TS_CURRENT_FAULT_CALIBRATION_INVALID: u8 = 12;
pub const TS_CURRENT_FAULT_SAFETY_CUT: u8 = 13;
pub const TS_CURRENT_FAULT_ACTUATOR_FAULT: u8 = 14;

pub const TS_FAULT_SEVERITY_NONE: u8 = 0;
pub const TS_FAULT_SEVERITY_INFO: u8 = 1;
pub const TS_FAULT_SEVERITY_WARNING: u8 = 2;
pub const TS_FAULT_SEVERITY_CRITICAL: u8 = 3;

pub const TS_FAULT_ACTION_NONE: u8 = 0;
pub const TS_FAULT_ACTION_OBSERVE_ONLY: u8 = 1;
pub const TS_FAULT_ACTION_LIMP_HOME: u8 = 2;
pub const TS_FAULT_ACTION_SHUTDOWN: u8 = 3;

pub const TS_CANCEL_REASON_NONE: u8 = 0;
pub const TS_CANCEL_REASON_SYNC_LOSS: u8 = 1;
pub const TS_CANCEL_REASON_SAFETY_SHUTDOWN: u8 = 2;
pub const TS_CANCEL_REASON_COMMIT: u8 = 3;
pub const TS_CANCEL_REASON_TIMEOUT: u8 = 4;

pub const TS_FAULT_FLAG_ACTIVE: u8 = 1 << 0;
pub const TS_FAULT_FLAG_EMERGENCY_MODE: u8 = 1 << 1;
pub const TS_FAULT_FLAG_SNAPSHOT_PRESENT: u8 = 1 << 2;
pub const TS_FAULT_FLAG_DIAG_LOG_PRESENT: u8 = 1 << 3;

pub const TS_DIAG_SOURCE_NONE: u8 = 0;
pub const TS_DIAG_SOURCE_SENSOR: u8 = 1;
pub const TS_DIAG_SOURCE_TRIGGER: u8 = 2;
pub const TS_DIAG_SOURCE_SCHEDULER: u8 = 3;
pub const TS_DIAG_SOURCE_SAFETY: u8 = 4;
pub const TS_DIAG_SOURCE_USER: u8 = 5;
pub const TS_DIAG_SOURCE_CONTEXT_PRESENT: u8 = 1 << 7;

#[cfg(feature = "runtime")]
pub const fn ts_fault_code_from_diag_code(code: DiagCode) -> u8 {
    code.to_u8()
}

#[cfg(feature = "runtime")]
pub const fn ts_fault_code_from_runtime_fault(fault: FaultCode) -> u8 {
    match fault {
        FaultCode::None => TS_CURRENT_FAULT_NONE,
        FaultCode::SyncLoss => TS_CURRENT_FAULT_SYNC_LOSS,
        FaultCode::SensorOutOfRange => TS_CURRENT_FAULT_SENSOR_OUT_OF_RANGE,
        FaultCode::CalibrationInvalid => TS_CURRENT_FAULT_CALIBRATION_INVALID,
        FaultCode::SafetyCut => TS_CURRENT_FAULT_SAFETY_CUT,
        FaultCode::ActuatorFault => TS_CURRENT_FAULT_ACTUATOR_FAULT,
    }
}

#[cfg(feature = "runtime")]
pub const fn ts_fault_severity_code(severity: FaultSeverity) -> u8 {
    match severity {
        FaultSeverity::Info => TS_FAULT_SEVERITY_INFO,
        FaultSeverity::Warning => TS_FAULT_SEVERITY_WARNING,
        FaultSeverity::Critical => TS_FAULT_SEVERITY_CRITICAL,
    }
}

#[cfg(feature = "runtime")]
pub const fn ts_cancel_reason_code(reason: CancelReason) -> u8 {
    match reason {
        CancelReason::Manual => TS_CANCEL_REASON_NONE,
        CancelReason::SyncLoss => TS_CANCEL_REASON_SYNC_LOSS,
        CancelReason::SafetyShutdown => TS_CANCEL_REASON_SAFETY_SHUTDOWN,
        CancelReason::Commit => TS_CANCEL_REASON_COMMIT,
        CancelReason::Timeout => TS_CANCEL_REASON_TIMEOUT,
    }
}

pub const fn ts_diag_log_severity_code(code: u8) -> u8 {
    match code {
        TS_CURRENT_FAULT_NONE => TS_FAULT_SEVERITY_NONE,
        TS_CURRENT_FAULT_PERSIST_CRC_FAULT => TS_FAULT_SEVERITY_CRITICAL,
        _ => TS_FAULT_SEVERITY_WARNING,
    }
}

pub const fn ts_diag_log_action_code(code: u8, source: u8) -> u8 {
    match code {
        TS_CURRENT_FAULT_NONE => TS_FAULT_ACTION_NONE,
        TS_CURRENT_FAULT_PERSIST_CRC_FAULT => TS_FAULT_ACTION_SHUTDOWN,
        TS_CURRENT_FAULT_MAP_FAILURE_HIGH_LOAD | TS_CURRENT_FAULT_TPS_MAP_PLAUSIBILITY => {
            TS_FAULT_ACTION_LIMP_HOME
        }
        _ => match source {
            TS_DIAG_SOURCE_TRIGGER | TS_DIAG_SOURCE_SCHEDULER | TS_DIAG_SOURCE_SAFETY => {
                TS_FAULT_ACTION_LIMP_HOME
            }
            _ => TS_FAULT_ACTION_OBSERVE_ONLY,
        },
    }
}

#[cfg(feature = "runtime")]
pub const fn ts_diag_source_code(source: ecu_domain::diag::DiagSource) -> u8 {
    match source {
        ecu_domain::diag::DiagSource::Sensor => TS_DIAG_SOURCE_SENSOR,
        ecu_domain::diag::DiagSource::Trigger => TS_DIAG_SOURCE_TRIGGER,
        ecu_domain::diag::DiagSource::Scheduler => TS_DIAG_SOURCE_SCHEDULER,
        ecu_domain::diag::DiagSource::Safety => TS_DIAG_SOURCE_SAFETY,
        ecu_domain::diag::DiagSource::User => TS_DIAG_SOURCE_USER,
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
pub struct DiagnosticLogEntry {
    pub code: u8,
    pub severity: u8,
    pub action: u8,
    pub source: u8,
    pub context_present: bool,
    pub context: u32,
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
    pub current_fault_code: u8,
    pub current_fault_severity: u8,
    pub current_fault_action: u8,
    pub current_cancel_reason: u8,
    pub fault_flags: u8,
    pub latest_diag_code: u8,
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
    pub current_fault_code: u8,
    pub current_fault_severity: u8,
    pub current_fault_action: u8,
    pub current_cancel_reason: u8,
    pub fault_flags: u8,
    pub latest_diag_code: u8,
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
