use ecu_calibration::sensors::SensorsCal;

use crate::compat_state::{EcuConfig, EcuDerived, EcuFaults, EcuInputs, EcuOutputs};
use crate::constants::fuel::DEFAULT_PULSE_WIDTH_US;
use crate::fuel_state::Corrections;
use crate::telemetry::IsrStats;
use crate::units::{Micros, Rpm};
use crate::{
    actuators, constants, dfco, diag, enrichment, ignition, knock, lambda, rev_limiter, safety,
    sensors, torque, trigger, ts,
};

pub struct EcuState {
    /// Single source of truth for runtime sensor/trigger inputs (review 002).
    /// Public scalar mirrors were removed; use the accessor/setter methods.
    pub(crate) inputs: EcuInputs,
    pub(crate) derived: EcuDerived,
    pub(crate) outputs: EcuOutputs,
    pub config: EcuConfig,
    pub rev_limiter_state: rev_limiter::RevLimiterState,
    pub flood_clear_state: safety::FloodClearState,
    pub sync_loss_tracker: safety::SyncLossTracker,
    pub diag_map: diag::DiagState,
    pub diag_tps: diag::DiagState,
    pub diag_cam: diag::DiagState,
    pub(crate) ae_state: enrichment::AeState,
    pub(crate) ase_state: enrichment::AseState,
    pub(crate) corrections: Corrections,
    pub(crate) ignition_corrections: ignition::IgnitionCorrections,
    pub voltage_monitor: safety::VoltageMonitor,
    pub load_failure_tracker: safety::LoadFailureTracker,
    pub lambda_state: lambda::LambdaState,
    pub ltft_manager: lambda::LtftManager,
    pub knock_controller: knock::KnockController,
    pub torque_controller: torque::TorqueController,
    pub fuel_mult_x100: u16,
    pub isr_stats: IsrStats,
    pub snapshot: ts::pages::SystemSnapshot,
    pub faults: EcuFaults,
    pub(crate) expert_trigger: ts::pages::ExpertTriggerPageState,
}

impl EcuState {
    /// Create new ECU state with defaults
    pub const fn new() -> Self {
        Self {
            inputs: EcuInputs {
                rpm: 0,
                synced: false,
                tooth_count: 0,
                // Default sensor readings: 12.5 V battery, 20.0 C CLT/IAT,
                // 0% TPS, 100.0 kPa MAP, no enrichment history.
                battery_voltage_mv: 12_500,
                clt_x10: 200,
                iat_x10: 200,
                tps_percent: 0,
                map_kpa_x10: 1000,
                last_enrichment_update_us: 0,
                last_enrichment_tps_percent: 0,
                last_enrichment_map_kpa_x10: 0,
            },
            derived: EcuDerived {
                wue_percent: 0,
                ase_percent: 0,
                ae_percent: 0,
                stft_x10: 0,
                ltft_manager: lambda::LtftManager::new(),
                fuel_mult_x100: 100,
            },
            outputs: EcuOutputs {
                final_pw: Micros::new(0),
                commanded_advance_x10: 0,
            },
            config: EcuConfig {
                ipw_table: [[DEFAULT_PULSE_WIDTH_US; 16]; 16],
                ve_table: [[100; 16]; 16],
                afr_table: [[actuators::ClConfig::DEFAULT.target_afr_x10; 16]; 16],
                required_fuel_us: DEFAULT_PULSE_WIDTH_US,
                injector_deadtime_us: 800,
                ve_load_source: 0,
                ignition_table: [[constants::ignition::DEFAULT_TIMING_BTDC; 16]; 16],
                sensors_cal: SensorsCal::default(),
                sensors_limits: sensors::SensorsLimits::default(),
                ae_config: enrichment::AeConfig::DEFAULT,
                wue_config: enrichment::WueConfig::DEFAULT,
                ase_config: enrichment::AseConfig::DEFAULT,
                dfco_config: dfco::DfcoConfig::DEFAULT,
                idle_config: actuators::IdleConfig::DEFAULT,
                fan_config: actuators::FanConfig::DEFAULT,
                cl_config: actuators::ClConfig::DEFAULT,
                load_failure_config: safety::LoadFailureConfig::DEFAULT,
                plausibility_config: sensors::plausibility::PlausibilityConfig::DEFAULT,
                rate_config: sensors::plausibility::RateConfig::DEFAULT,
                lambda_config: lambda::LambdaConfig::DEFAULT,
                rev_limiter_config: rev_limiter::RevLimiterConfig::DEFAULT,
                inj_angle_btdc_x10: [0; 16],
                tdc_per_cyl_x10: [0; 16],
                tooth0_angle_x10: 0,
                cam_missing_timeout_ms: 500,
            },
            rev_limiter_state: rev_limiter::RevLimiterState::new(),
            flood_clear_state: safety::FloodClearState::new(),
            sync_loss_tracker: safety::SyncLossTracker::new(),
            diag_map: diag::DiagState::new(),
            diag_tps: diag::DiagState::new(),
            diag_cam: diag::DiagState::new(),
            ae_state: enrichment::AeState::new(),
            ase_state: enrichment::AseState::new(),
            corrections: Corrections::DEFAULT,
            ignition_corrections: ignition::IgnitionCorrections::DEFAULT,
            voltage_monitor: safety::VoltageMonitor::new(),
            load_failure_tracker: safety::LoadFailureTracker::new(),
            lambda_state: lambda::LambdaState::new(),
            ltft_manager: lambda::LtftManager::new(),
            knock_controller: knock::KnockController::new(),
            torque_controller: torque::TorqueController::new(),
            fuel_mult_x100: 100,
            isr_stats: IsrStats::new(),
            snapshot: ts::pages::SystemSnapshot {
                rpm: Rpm::new(0),
                sync: trigger::SyncState::Unsynced,
                base_pw: Micros::new(0),
                enrich_mult_x100: 100,
                stft_x10: 0,
                fuel_mult_x100: 100,
                final_pw: Micros::new(0),
                last_fault_code: 0,
                fault_severity: 0,
                cancel_reason: 0,
                isr_count: 0,
                isr_max_us: 0,
                isr_avg_us: 0,
            },
            faults: EcuFaults {
                emergency_trigger_map_oob: false,
                emergency_trigger_tps_oob: false,
                emergency_mode: false,
                diag_log: diag::DiagLog::new(),
            },
            expert_trigger: ts::pages::ExpertTriggerPageState::new(),
        }
    }
}

impl Default for EcuState {
    fn default() -> Self {
        Self::new()
    }
}
