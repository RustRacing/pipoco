//! ECU Core Library
//!
//! Minimal viable ECU (Engine Control Unit) implementation in Rust.
//! Designed for no_std embedded environments with zero dependencies.
//!
//! # Architecture
//!
//! This library uses an IPW (Injector Pulse Width) table approach instead of
//! traditional VE (Volumetric Efficiency) calculations. This eliminates complex
//! math from the embedded module - all calculations are pre-computed and stored
//! in lookup tables.
//!
//! ## Modules
//!
//! - `trigger`: 60-2 trigger wheel decoder for position and RPM
//! - `tables`: IPW table lookup (no interpolation)
//! - `scheduler`: Event scheduling for injection and ignition
//! - `hal`: Hardware abstraction traits
//! - `constants`: System-wide configuration constants
//! - `transport`: Transport-agnostic inter-component communication
//!
//! ## Design Principles
//!
//! - **Integer-only arithmetic**: No floating-point operations
//! - **Static memory**: No heap allocation, all state in static variables
//! - **Wrapping arithmetic**: Correctly handles timer overflow
//! - **Minimal dependencies**: Zero external dependencies in core library

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "transport-can-fd", not(feature = "transport-can")))]
compile_error!("feature `transport-can-fd` requires `transport-can`");

pub mod actuators;
#[cfg(feature = "legacy-root-scheduler")]
pub mod app;
pub mod capture;
#[cfg(feature = "legacy-root-scheduler")]
pub mod config;
pub mod constants;
pub mod dfco;
pub mod diag;
pub mod enrichment;
pub mod hal;
pub mod ignition;
pub mod knock;
pub mod lambda;
#[cfg(feature = "management-experimental")]
pub mod management;
pub mod persist;
pub mod rev_limiter;
pub mod safety;
#[cfg(feature = "legacy-root-scheduler")]
pub mod scheduler;
pub mod sensors;
pub mod tables;
pub mod telemetry;
pub mod torque;
pub mod transport;
pub mod trigger;
pub mod ts;
pub mod units;
pub mod ve_engine;

pub use capture::CaptureBuffer;
pub use ignition::{calculate_dwell, calculate_timing, IgnitionCorrections, IgnitionTable};
pub use rev_limiter::{
    apply_limiter_retard, should_inject, update_limiter, LimiterStrategy, RevLimiterConfig,
    RevLimiterState,
};
pub use safety::{
    should_allow_injection, update_flood_clear, FloodClearState, LoadFailureConfig,
    LoadFailureReason, LoadFailureTracker, PowerState, SyncLossTracker, VoltageMonitor,
};
pub use tables::IpwTable;
pub use telemetry::IsrStats;
pub use transport::{Message, Transport, TransportError, TransportStats};
pub use trigger::{TriggerDecoder, TriggerTiming};
pub use units::{DegX10, Kpa10, Micros, Rpm, Ticks};

#[cfg(feature = "transport-bbqueue")]
pub use transport::BbqTransport;

#[cfg(feature = "interp-bilinear")]
pub type ActiveTable = tables::IpwTableBilinear;
#[cfg(not(feature = "interp-bilinear"))]
pub type ActiveTable = tables::IpwTableNearest;

// ---------------------------------------------------------------------------
// Root/core boundary adapter contracts
// ---------------------------------------------------------------------------

/// Adapter contracts for root/core boundary fields.
///
/// These document the fundamental model differences between the root ecu-core
/// implementation (IPW table lookup) and the spec oracle (VE model).
///
/// DO NOT add new variants without a corresponding test in fm0016_core_reducer.rs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoreAdapterContract {
    /// Root uses IPW (Injector Pulse Width) tables — direct (rpm, load) → PW mapping.
    /// Spec oracle uses VE (Volumetric Efficiency) model with full correction pipeline.
    IpwVsVeFuelModel,
    /// Root ignition timing from IPW table vs spec timing table lookup.
    TimingTableVsFrozenSpec,
    /// Base PW computed via IPW vs spec VE displacement model.
    BasePwIncomparable,
    /// Corrected PW computed via IPW corrections vs spec VE corrections.
    CorrectedPwIncomparable,
}

// ---------------------------------------------------------------------------
// Test-only observability accessors (test builds only)
// ---------------------------------------------------------------------------

/// Runtime mirror of the live scalar inputs that are still duplicated on `EcuState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSignals {
    pub rpm: u16,
    pub synced: bool,
    pub tooth_count: u8,
    pub battery_voltage_mv: u16,
    pub clt_x10: i16,
    pub iat_x10: i16,
    pub tps_percent: u8,
    pub map_kpa_x10: u16,
}

/// Diagnostic fault flags that are mirrored in `EcuState::faults`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticFlags {
    pub emergency_trigger_map_oob: bool,
    pub emergency_trigger_tps_oob: bool,
    pub emergency_mode: bool,
}

/// Safety outputs derived from limiter and cut state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyStatus {
    pub fuel_cut_active: bool,
    pub spark_cut_active: bool,
}

impl EcuState {
    /// Current injector pulse width for a given rpm/load lookup.
    ///
    /// Returns the table-lookup PW before corrections are applied.
    pub fn injection_pulse_width(&self, rpm: u16, load: u16) -> u16 {
        self.calculate_fuel(rpm, load)
    }

    /// Current ignition dwell time in microseconds.
    pub fn ignition_dwell_us(&self) -> u32 {
        self.calculate_dwell()
    }

    /// Current ignition advance for a given rpm/load in degrees BTDC.
    ///
    /// Returns advance before per-cylinder limiting.
    pub fn ignition_advance_deg(&self, rpm: u16, load: u16) -> i16 {
        self.calculate_ignition_timing(rpm, load)
    }

    /// Runtime mirror of the current scalar inputs exposed on `EcuState`.
    pub fn runtime_signals(&self) -> RuntimeSignals {
        RuntimeSignals {
            rpm: self.rpm,
            synced: self.synced,
            tooth_count: self.tooth_count,
            battery_voltage_mv: self.inputs.battery_voltage_mv,
            clt_x10: self.clt_x10,
            iat_x10: self.iat_x10,
            tps_percent: self.tps_percent,
            map_kpa_x10: self.map_kpa_x10,
        }
    }

    /// Current RPM.
    pub fn current_rpm(&self) -> u16 {
        self.runtime_signals().rpm
    }

    /// Current sync state.
    pub fn current_synced(&self) -> bool {
        self.runtime_signals().synced
    }

    /// Diagnostic fault flags that back the legacy tuple accessor.
    pub fn diagnostic_flags(&self) -> DiagnosticFlags {
        DiagnosticFlags {
            emergency_trigger_map_oob: self.faults.emergency_trigger_map_oob,
            emergency_trigger_tps_oob: self.faults.emergency_trigger_tps_oob,
            emergency_mode: self.faults.emergency_mode,
        }
    }

    /// Active fault flags: (emap_oob, etps_oob, emode).
    pub fn current_fault_flags(&self) -> (bool, bool, bool) {
        let flags = self.diagnostic_flags();
        (
            flags.emergency_trigger_map_oob,
            flags.emergency_trigger_tps_oob,
            flags.emergency_mode,
        )
    }

    /// Safety cut state derived from limiter and cut logic.
    pub fn safety_status(&self) -> SafetyStatus {
        SafetyStatus {
            fuel_cut_active: !self.should_inject_fuel(0),
            spark_cut_active: self.rev_limiter_state.ignition_retard != 0,
        }
    }

    /// Fuel cut active — true if rev limiter or other safety is cutting fuel.
    pub fn fuel_cut_active(&self) -> bool {
        self.safety_status().fuel_cut_active
    }

    /// Spark cut active — true if rev limiter or DFCO is cutting ignition.
    pub fn spark_cut_active(&self) -> bool {
        self.safety_status().spark_cut_active
    }
}

use constants::corrections::*;
use constants::fuel::*;
/// Fixed-point math helper (no floats!)
///
/// Multiplies value by (multiplier / 100) using integer arithmetic only.
/// Uses saturating multiplication to prevent overflow.
///
/// # Arguments
/// * `value` - Base value (e.g., pulse width in microseconds)
/// * `multiplier` - Correction factor scaled by 100 (e.g., 150 = 1.5x, 80 = 0.8x)
///
/// # Returns
/// Corrected value, saturated at u16::MAX if overflow would occur
///
/// # Example
/// ```
/// use ecu_core::scale_u16;
///
/// assert_eq!(scale_u16(1000, 150), 1500);  // 1.5x
/// assert_eq!(scale_u16(1000, 80), 800);    // 0.8x
/// assert_eq!(scale_u16(1000, 100), 1000);  // 1.0x (no change)
/// ```
pub fn scale_u16(value: u16, multiplier: u8) -> u16 {
    // Use saturating multiply to prevent overflow
    let intermediate = (value as u32).saturating_mul(multiplier as u32);
    let result = intermediate / 100;

    // Clamp to u16::MAX
    if result > u16::MAX as u32 {
        u16::MAX
    } else {
        result as u16
    }
}

/// Apply a signed closed-loop delta in percent to a pulse width.
/// Positive increases fuel, negative decreases.
pub fn apply_cl_delta(pw: u16, cl_delta_percent: i16) -> u16 {
    if cl_delta_percent == 0 {
        return pw;
    }
    if cl_delta_percent > 0 {
        let m = (100i16 + cl_delta_percent).clamp(0, 200) as u8;
        scale_u16(pw, m)
    } else {
        // Decrease: scale by (100 - |delta|)
        let m = (100i16 - (-cl_delta_percent)).clamp(0, 200) as u8;
        scale_u16(pw, m)
    }
}

/// Correction multipliers (100 = 1.0x)
///
/// All corrections are represented as integers scaled by 100 to avoid
/// floating-point operations. A value of 100 means no correction (1.0x).
///
/// # Examples
/// - 150 = 1.5x (add 50% fuel)
/// - 80 = 0.8x (reduce fuel by 20%)
/// - 100 = 1.0x (no change)
#[derive(Debug, Clone, Copy)]
pub struct Corrections {
    /// Coolant temperature correction
    pub clt: u8,
    /// Intake air temperature correction
    pub iat: u8,
    /// Battery voltage correction (compensates for injector opening time)
    pub vbatt: u8,
}

impl Corrections {
    /// Default corrections (1.0x all - no corrections applied)
    pub const DEFAULT: Self = Self {
        clt: UNITY_CORRECTION,
        iat: UNITY_CORRECTION,
        vbatt: UNITY_CORRECTION,
    };

    /// Create new corrections with specified values
    pub const fn new(clt: u8, iat: u8, vbatt: u8) -> Self {
        Self { clt, iat, vbatt }
    }
}

/// Global ECU state
///
/// Contains all state needed for ECU operation. Designed to be stored
/// in a static variable for access from ISR context.
pub struct EcuConfig {
    pub ipw_table: [[u16; 16]; 16],
    pub ignition_table: [[i16; 16]; 16],
    pub sensors_cal: sensors::SensorsCal,
    pub sensors_limits: sensors::SensorsLimits,
    pub ae_config: enrichment::AeConfig,
    pub wue_config: enrichment::WueConfig,
    pub ase_config: enrichment::AseConfig,
    pub dfco_config: dfco::DfcoConfig,
    pub idle_config: actuators::IdleConfig,
    pub fan_config: actuators::FanConfig,
    pub cl_config: actuators::ClConfig,
    pub load_failure_config: safety::LoadFailureConfig,
    pub plausibility_config: sensors::plausibility::PlausibilityConfig,
    pub rate_config: sensors::plausibility::RateConfig,
    pub lambda_config: lambda::LambdaConfig,
    pub corrections: Corrections,
    pub ignition_corrections: ignition::IgnitionCorrections,
    pub rev_limiter_config: rev_limiter::RevLimiterConfig,
    pub inj_angle_btdc_x10: [u16; 16],
    pub tdc_per_cyl_x10: [u16; 16],
    pub tooth0_angle_x10: u16,
    pub cam_missing_timeout_ms: u16,
}

/// Trigger-derived live inputs that are in the process of being separated
/// from the main `EcuState` layout.
#[derive(Debug, Clone, Copy)]
pub struct EcuInputs {
    pub rpm: u16,
    pub synced: bool,
    pub tooth_count: u8,
    pub battery_voltage_mv: u16,
    pub clt_x10: i16,
    pub iat_x10: i16,
    pub tps_percent: u8,
    pub map_kpa_x10: u16,
    pub last_enrichment_update_us: u32,
    pub last_enrichment_tps_percent: u8,
    pub last_enrichment_map_kpa_x10: u16,
}

/// Derived enrichment outputs that are in the process of being separated
/// from the main `EcuState` layout.
#[derive(Debug, Clone, Copy)]
pub struct EcuDerived {
    pub wue_percent: u8,
    pub ase_percent: u8,
    pub ae_percent: u8,
    pub stft_x10: i16,
    pub ltft_manager: lambda::LtftManager,
    pub fuel_mult_x100: u16,
}

/// Runtime output cache that is in the process of being separated from the
/// main `EcuState` layout.
#[derive(Debug, Clone, Copy)]
pub struct EcuOutputs {
    pub final_pw: Micros,
    pub commanded_advance_x10: i16,
}

/// Fault and diagnostics cache that is in the process of being separated from
/// the main `EcuState` layout.
#[derive(Debug)]
pub struct EcuFaults {
    pub emergency_trigger_map_oob: bool,
    pub emergency_trigger_tps_oob: bool,
    pub emergency_mode: bool,
    pub diag_log: diag::DiagLog<16>,
}

impl EcuInputs {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        rpm: u16,
        synced: bool,
        tooth_count: u8,
        battery_voltage_mv: u16,
        clt_x10: i16,
        iat_x10: i16,
        tps_percent: u8,
        map_kpa_x10: u16,
        last_enrichment_update_us: u32,
        last_enrichment_tps_percent: u8,
        last_enrichment_map_kpa_x10: u16,
    ) -> Self {
        Self {
            rpm,
            synced,
            tooth_count,
            battery_voltage_mv,
            clt_x10,
            iat_x10,
            tps_percent,
            map_kpa_x10,
            last_enrichment_update_us,
            last_enrichment_tps_percent,
            last_enrichment_map_kpa_x10,
        }
    }
}

pub struct EcuState {
    pub rpm: u16,
    pub synced: bool,
    pub tooth_count: u8,
    inputs: EcuInputs,
    derived: EcuDerived,
    outputs: EcuOutputs,
    pub config: EcuConfig,
    pub rev_limiter_state: rev_limiter::RevLimiterState,
    pub clt_x10: i16,
    pub iat_x10: i16,
    pub tps_percent: u8,  // Throttle position (0-100%) (clamped)
    pub map_kpa_x10: u16, // MAP (kPa*10) (clamped)
    pub flood_clear_state: safety::FloodClearState,
    pub sync_loss_tracker: safety::SyncLossTracker,
    pub diag_map: diag::DiagState,
    pub diag_tps: diag::DiagState,
    pub diag_cam: diag::DiagState,
    ae_state: enrichment::AeState,
    ase_state: enrichment::AseState,
    pub voltage_monitor: safety::VoltageMonitor,
    pub load_failure_tracker: safety::LoadFailureTracker,
    pub plausibility_state: sensors::plausibility::PlausibilityState,
    pub rate_state: sensors::plausibility::RateValidationState,
    pub lambda_state: lambda::LambdaState,
    pub ltft_manager: lambda::LtftManager,
    pub knock_controller: knock::KnockController,
    pub torque_controller: torque::TorqueController,
    pub fuel_mult_x100: u16,
    pub isr_stats: IsrStats,
    pub snapshot: ts::pages::SystemSnapshot,
    pub faults: EcuFaults,
    expert_trigger: ts::pages::ExpertTriggerPageState,
}

impl EcuState {
    /// Create new ECU state with defaults
    pub const fn new() -> Self {
        Self {
            rpm: 0,
            synced: false,
            tooth_count: 0,
            inputs: EcuInputs::new(0, false, 0, 12500, 200, 200, 0, 1000, 0, 0, 0),
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
                ignition_table: [[constants::ignition::DEFAULT_TIMING_BTDC; 16]; 16],
                sensors_cal: sensors::SensorsCal::default(),
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
                corrections: Corrections::DEFAULT,
                ignition_corrections: ignition::IgnitionCorrections::DEFAULT,
                rev_limiter_config: rev_limiter::RevLimiterConfig::DEFAULT,
                inj_angle_btdc_x10: [0; 16],
                tdc_per_cyl_x10: [0; 16],
                tooth0_angle_x10: 0,
                cam_missing_timeout_ms: 500,
            },
            rev_limiter_state: rev_limiter::RevLimiterState::new(),
            clt_x10: 200,
            iat_x10: 200,
            tps_percent: 0, // Throttle closed (clamped)
            map_kpa_x10: 1000,
            flood_clear_state: safety::FloodClearState::new(),
            sync_loss_tracker: safety::SyncLossTracker::new(),
            diag_map: diag::DiagState::new(),
            diag_tps: diag::DiagState::new(),
            diag_cam: diag::DiagState::new(),
            ae_state: enrichment::AeState::new(),
            ase_state: enrichment::AseState::new(),
            voltage_monitor: safety::VoltageMonitor::new(),
            load_failure_tracker: safety::LoadFailureTracker::new(),
            plausibility_state: sensors::plausibility::PlausibilityState::new(),
            rate_state: sensors::plausibility::RateValidationState::new(),
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
                last_fault: None,
                isr_stats: IsrStats::new(),
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

    pub fn corrections(&self) -> &Corrections {
        &self.config.corrections
    }

    pub fn corrections_mut(&mut self) -> &mut Corrections {
        &mut self.config.corrections
    }

    pub fn ipw_table(&self) -> &[[u16; 16]; 16] {
        &self.config.ipw_table
    }

    pub fn ipw_table_mut(&mut self) -> &mut [[u16; 16]; 16] {
        &mut self.config.ipw_table
    }

    pub fn ignition_table(&self) -> &[[i16; 16]; 16] {
        &self.config.ignition_table
    }

    pub fn ignition_table_mut(&mut self) -> &mut [[i16; 16]; 16] {
        &mut self.config.ignition_table
    }

    pub fn sensors_cal(&self) -> &sensors::SensorsCal {
        &self.config.sensors_cal
    }

    pub fn sensors_cal_mut(&mut self) -> &mut sensors::SensorsCal {
        &mut self.config.sensors_cal
    }

    pub fn sensors_limits(&self) -> &sensors::SensorsLimits {
        &self.config.sensors_limits
    }

    pub fn sensors_limits_mut(&mut self) -> &mut sensors::SensorsLimits {
        &mut self.config.sensors_limits
    }

    pub fn ae_config(&self) -> &enrichment::AeConfig {
        &self.config.ae_config
    }

    pub fn ae_config_mut(&mut self) -> &mut enrichment::AeConfig {
        &mut self.config.ae_config
    }

    pub fn wue_config(&self) -> &enrichment::WueConfig {
        &self.config.wue_config
    }

    pub fn wue_config_mut(&mut self) -> &mut enrichment::WueConfig {
        &mut self.config.wue_config
    }

    pub fn ase_config(&self) -> &enrichment::AseConfig {
        &self.config.ase_config
    }

    pub fn ase_config_mut(&mut self) -> &mut enrichment::AseConfig {
        &mut self.config.ase_config
    }

    pub fn dfco_config(&self) -> &dfco::DfcoConfig {
        &self.config.dfco_config
    }

    pub fn dfco_config_mut(&mut self) -> &mut dfco::DfcoConfig {
        &mut self.config.dfco_config
    }

    pub fn idle_config(&self) -> &actuators::IdleConfig {
        &self.config.idle_config
    }

    pub fn idle_config_mut(&mut self) -> &mut actuators::IdleConfig {
        &mut self.config.idle_config
    }

    pub fn fan_config(&self) -> &actuators::FanConfig {
        &self.config.fan_config
    }

    pub fn fan_config_mut(&mut self) -> &mut actuators::FanConfig {
        &mut self.config.fan_config
    }

    pub fn cl_config(&self) -> &actuators::ClConfig {
        &self.config.cl_config
    }

    pub fn cl_config_mut(&mut self) -> &mut actuators::ClConfig {
        &mut self.config.cl_config
    }

    pub fn load_failure_config(&self) -> &safety::LoadFailureConfig {
        &self.config.load_failure_config
    }

    pub fn load_failure_config_mut(&mut self) -> &mut safety::LoadFailureConfig {
        &mut self.config.load_failure_config
    }

    pub fn plausibility_config(&self) -> &sensors::plausibility::PlausibilityConfig {
        &self.config.plausibility_config
    }

    pub fn plausibility_config_mut(&mut self) -> &mut sensors::plausibility::PlausibilityConfig {
        &mut self.config.plausibility_config
    }

    pub fn rate_config(&self) -> &sensors::plausibility::RateConfig {
        &self.config.rate_config
    }

    pub fn rate_config_mut(&mut self) -> &mut sensors::plausibility::RateConfig {
        &mut self.config.rate_config
    }

    pub fn lambda_config(&self) -> &lambda::LambdaConfig {
        &self.config.lambda_config
    }

    pub fn lambda_config_mut(&mut self) -> &mut lambda::LambdaConfig {
        &mut self.config.lambda_config
    }

    pub fn ignition_corrections(&self) -> &ignition::IgnitionCorrections {
        &self.config.ignition_corrections
    }

    pub fn ignition_corrections_mut(&mut self) -> &mut ignition::IgnitionCorrections {
        &mut self.config.ignition_corrections
    }

    pub fn rev_limiter_config(&self) -> &rev_limiter::RevLimiterConfig {
        &self.config.rev_limiter_config
    }

    pub fn rev_limiter_config_mut(&mut self) -> &mut rev_limiter::RevLimiterConfig {
        &mut self.config.rev_limiter_config
    }

    pub fn inj_angle_btdc_x10(&self) -> &[u16; 16] {
        &self.config.inj_angle_btdc_x10
    }

    pub fn inj_angle_btdc_x10_mut(&mut self) -> &mut [u16; 16] {
        &mut self.config.inj_angle_btdc_x10
    }

    pub fn tdc_per_cyl_x10(&self) -> &[u16; 16] {
        &self.config.tdc_per_cyl_x10
    }

    pub fn tdc_per_cyl_x10_mut(&mut self) -> &mut [u16; 16] {
        &mut self.config.tdc_per_cyl_x10
    }

    pub fn tooth0_angle_x10(&self) -> u16 {
        self.config.tooth0_angle_x10
    }

    pub fn tooth0_angle_x10_mut(&mut self) -> &mut u16 {
        &mut self.config.tooth0_angle_x10
    }

    pub fn cam_missing_timeout_ms(&self) -> u16 {
        self.config.cam_missing_timeout_ms
    }

    pub fn cam_missing_timeout_ms_mut(&mut self) -> &mut u16 {
        &mut self.config.cam_missing_timeout_ms
    }

    pub fn rpm(&self) -> u16 {
        self.runtime_signals().rpm
    }

    pub fn synced(&self) -> bool {
        self.runtime_signals().synced
    }

    pub fn tooth_count(&self) -> u8 {
        self.runtime_signals().tooth_count
    }

    fn trigger_inputs(&self) -> EcuInputs {
        self.inputs
    }

    pub fn set_rpm(&mut self, rpm: u16) {
        self.rpm = rpm;
        self.inputs.rpm = rpm;
    }

    pub fn set_synced(&mut self, synced: bool) {
        self.synced = synced;
        self.inputs.synced = synced;
    }

    pub fn set_tooth_count(&mut self, tooth_count: u8) {
        self.tooth_count = tooth_count;
        self.inputs.tooth_count = tooth_count;
    }

    pub fn battery_voltage_mv(&self) -> u16 {
        self.runtime_signals().battery_voltage_mv
    }

    pub fn set_battery_voltage_mv(&mut self, battery_voltage_mv: u16) {
        self.inputs.battery_voltage_mv = battery_voltage_mv;
    }

    pub fn last_enrichment_update_us(&self) -> u32 {
        self.inputs.last_enrichment_update_us
    }

    pub fn last_enrichment_tps_percent(&self) -> u8 {
        self.inputs.last_enrichment_tps_percent
    }

    pub fn last_enrichment_map_kpa_x10(&self) -> u16 {
        self.inputs.last_enrichment_map_kpa_x10
    }

    pub fn wue_percent(&self) -> u8 {
        self.derived.wue_percent
    }

    pub fn ase_percent(&self) -> u8 {
        self.derived.ase_percent
    }

    pub fn ae_percent(&self) -> u8 {
        self.derived.ae_percent
    }

    pub fn stft_x10(&self) -> i16 {
        self.lambda_state.stft_x10
    }

    pub fn set_stft_x10(&mut self, stft_x10: i16) {
        self.derived.stft_x10 = stft_x10;
        self.lambda_state.stft_x10 = stft_x10;
    }

    pub fn ltft_manager(&self) -> &lambda::LtftManager {
        &self.derived.ltft_manager
    }

    pub fn ltft_manager_mut(&mut self) -> &mut lambda::LtftManager {
        &mut self.derived.ltft_manager
    }

    pub fn fuel_mult_x100(&self) -> u16 {
        self.derived.fuel_mult_x100
    }

    pub fn set_fuel_mult_x100(&mut self, fuel_mult_x100: u16) {
        self.derived.fuel_mult_x100 = fuel_mult_x100;
    }

    pub fn final_pw_output(&self) -> Micros {
        self.outputs.final_pw
    }

    pub fn commanded_advance_x10_output(&self) -> i16 {
        self.outputs.commanded_advance_x10
    }

    pub fn set_commanded_advance_x10_output(&mut self, value: i16) {
        self.outputs.commanded_advance_x10 = value;
    }

    pub fn emergency_trigger_map_oob(&self) -> bool {
        self.faults.emergency_trigger_map_oob
    }

    pub fn emergency_trigger_map_oob_mut(&mut self) -> &mut bool {
        &mut self.faults.emergency_trigger_map_oob
    }

    pub fn set_emergency_trigger_map_oob(&mut self, value: bool) {
        self.faults.emergency_trigger_map_oob = value;
    }

    pub fn emergency_trigger_tps_oob(&self) -> bool {
        self.faults.emergency_trigger_tps_oob
    }

    pub fn emergency_trigger_tps_oob_mut(&mut self) -> &mut bool {
        &mut self.faults.emergency_trigger_tps_oob
    }

    pub fn set_emergency_trigger_tps_oob(&mut self, value: bool) {
        self.faults.emergency_trigger_tps_oob = value;
    }

    pub fn emergency_mode(&self) -> bool {
        self.faults.emergency_mode
    }

    pub fn emergency_mode_ref(&self) -> &bool {
        &self.faults.emergency_mode
    }

    pub fn emergency_mode_mut(&mut self) -> &mut bool {
        &mut self.faults.emergency_mode
    }

    pub fn set_emergency_mode(&mut self, value: bool) {
        self.faults.emergency_mode = value;
    }

    pub fn diag_log(&self) -> &diag::DiagLog<16> {
        &self.faults.diag_log
    }

    pub fn diag_log_mut(&mut self) -> &mut diag::DiagLog<16> {
        &mut self.faults.diag_log
    }

    pub fn page_store(&mut self) -> crate::ts::pages::EcuPageStore<'_> {
        crate::ts::pages::EcuPageStore {
            fuel: &mut self.config.ipw_table,
            ign: &mut self.config.ignition_table,
            sens: &mut self.config.sensors_cal,
            ae: &mut self.config.ae_config,
            dfco: &mut self.config.dfco_config,
            wue: &mut self.config.wue_config,
            ase: &mut self.config.ase_config,
            idle: &mut self.config.idle_config,
            fan: &mut self.config.fan_config,
            cl: &mut self.config.cl_config,
            limits: &mut self.config.sensors_limits,
            emerg_trig_map: &mut self.faults.emergency_trigger_map_oob,
            emerg_trig_tps: &mut self.faults.emergency_trigger_tps_oob,
            diag_emergency: &self.faults.emergency_mode,
            diag_map: &self.diag_map,
            diag_tps: &self.diag_tps,
            diag_cam: &self.diag_cam,
            diag_log: &self.faults.diag_log,
            isr_stats: &self.isr_stats,
            snapshot: &self.snapshot,
            tooth_count: &self.tooth_count,
            sync_loss_tracker: &self.sync_loss_tracker,
            angles_inj: &mut self.config.inj_angle_btdc_x10,
            angles_tdc: &mut self.config.tdc_per_cyl_x10,
            tooth0_angle_x10: &mut self.config.tooth0_angle_x10,
            cam_timeout_ms: &mut self.config.cam_missing_timeout_ms,
            expert_trigger: &mut self.expert_trigger,
        }
    }

    pub fn set_trigger_inputs(&mut self, rpm: u16, synced: bool, tooth_count: u8) {
        self.rpm = rpm;
        self.synced = synced;
        self.tooth_count = tooth_count;
        self.inputs.rpm = rpm;
        self.inputs.synced = synced;
        self.inputs.tooth_count = tooth_count;
    }

    pub fn clt_x10(&self) -> i16 {
        self.runtime_signals().clt_x10
    }

    pub fn iat_x10(&self) -> i16 {
        self.runtime_signals().iat_x10
    }

    pub fn set_clt_x10(&mut self, clt_x10: i16) {
        self.clt_x10 = clt_x10;
        self.inputs.clt_x10 = clt_x10;
    }

    pub fn set_iat_x10(&mut self, iat_x10: i16) {
        self.iat_x10 = iat_x10;
        self.inputs.iat_x10 = iat_x10;
    }

    pub fn tps_percent(&self) -> u8 {
        self.runtime_signals().tps_percent
    }

    pub fn map_kpa_x10(&self) -> u16 {
        self.runtime_signals().map_kpa_x10
    }

    pub fn set_tps_percent(&mut self, tps_percent: u8) {
        self.tps_percent = tps_percent;
        self.inputs.tps_percent = tps_percent;
    }

    pub fn set_map_kpa_x10(&mut self, map_kpa_x10: u16) {
        self.map_kpa_x10 = map_kpa_x10;
        self.inputs.map_kpa_x10 = map_kpa_x10;
    }

    /// Update enrichment state machines and cache the current percentages.
    ///
    /// This only refreshes enrichment state. It does not change the fuel
    /// calculation path yet.
    pub fn refresh_enrichments(
        &mut self,
        now_us: u32,
        clt_x10: i16,
        iat_x10: i16,
        tps_percent: u8,
        map_kpa_x10: u16,
    ) {
        let clt_c = clt_x10 / 10;
        let _iat_c = iat_x10 / 10;

        self.derived.wue_percent = self.config.wue_config.compute_percent(clt_c);

        let first_tick = self.inputs.last_enrichment_update_us == 0;
        let tpsdot_pct_s = if first_tick {
            0
        } else {
            let dt_us = now_us
                .wrapping_sub(self.inputs.last_enrichment_update_us)
                .max(1);
            let dt_s = dt_us as i64;
            let delta = tps_percent as i64 - self.inputs.last_enrichment_tps_percent as i64;
            ((delta * 1_000_000) / dt_s) as i16
        };
        let mapdot_kpa_s = if first_tick {
            0
        } else {
            let dt_us = now_us
                .wrapping_sub(self.inputs.last_enrichment_update_us)
                .max(1);
            let dt_s = dt_us as i64;
            let delta = map_kpa_x10 as i64 - self.inputs.last_enrichment_map_kpa_x10 as i64;
            ((delta * 1_000_000) / dt_s) as i16
        };

        self.derived.ae_percent =
            self.ae_state
                .update(now_us, tpsdot_pct_s, mapdot_kpa_s, &self.config.ae_config);

        let trigger_inputs = self.trigger_inputs();
        let just_started = first_tick && trigger_inputs.synced && trigger_inputs.rpm > 0;
        self.derived.ase_percent =
            self.ase_state
                .update(now_us, just_started, &self.config.ase_config);

        self.inputs.last_enrichment_update_us = now_us;
        self.inputs.last_enrichment_tps_percent = tps_percent;
        self.inputs.last_enrichment_map_kpa_x10 = map_kpa_x10;
    }

    /// Apply the current torque arbitration result to cached outputs.
    pub fn apply_torque_result(&mut self, result: &crate::torque::arbiter::TorqueResult) {
        self.set_fuel_mult_x100(result.fuel_mult_x100);
    }

    /// Calculate fuel pulse width with corrections
    ///
    /// Performs the complete fuel calculation:
    /// 1. Table lookup for base pulse width
    /// 2. Apply temperature and voltage corrections
    /// 3. Clamp to valid range
    ///
    /// Uses integer-only arithmetic with saturating operations to prevent overflow.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa (or TPS %)
    ///
    /// # Returns
    /// Final pulse width in microseconds, clamped to MIN/MAX limits
    pub fn calculate_fuel(&self, rpm: u16, load: u16) -> u16 {
        let table = IpwTable {
            rpm_bins: RPM_BINS,
            load_bins: LOAD_BINS,
            values: self.config.ipw_table,
        };

        // 1. Base lookup
        let mut pw = table.lookup(rpm, load);

        // 2. Apply corrections sequentially with saturation
        pw = scale_u16(pw, self.corrections().clt);
        pw = scale_u16(pw, self.corrections().iat);
        pw = scale_u16(pw, self.corrections().vbatt);

        // 3. Clamp to reasonable range
        pw = pw.clamp(MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US);

        pw
    }

    /// Calculate fuel and apply additional enrichment percentages (WUE/ASE/AE).
    /// Percentages are 0..=100 where 0 means no extra fuel, 20 means +20%.
    pub fn calculate_fuel_with_enrichments(
        &self,
        rpm: u16,
        load: u16,
        wue_percent: u8,
        ase_percent: u8,
        ae_percent: u8,
        cl_delta_percent: i16,
    ) -> u16 {
        let mut pw = self.calculate_fuel(rpm, load);
        // Apply enrichments multiplicatively: pw *= (100 + pct) / 100
        let enrich = |val: u16, pct: u8| -> u16 {
            let mult = (100u16 + pct as u16) as u8; // safe up to 200
            scale_u16(val, mult)
        };
        pw = enrich(pw, wue_percent);
        pw = enrich(pw, ase_percent);
        pw = enrich(pw, ae_percent);
        // Apply closed-loop delta (may increase or decrease)
        pw = apply_cl_delta(pw, cl_delta_percent);
        pw.clamp(MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
    }

    /// Calculate the final injector pulse width after enrichment and torque trims.
    pub fn final_pw(&self, rpm: Rpm, load: Kpa10) -> Micros {
        let base = self.calculate_fuel(rpm.raw(), load.raw()) as u32;

        let enrich_mult_x100 = [
            self.derived.wue_percent,
            self.derived.ase_percent,
            self.derived.ae_percent,
        ]
        .into_iter()
        .fold(100u32, |acc, pct| {
            acc.saturating_mul(100 + pct as u32) / 100
        });

        let e = base.saturating_mul(enrich_mult_x100) / 100;
        let stft = self.stft_x10() as i32;
        let l = if stft >= 0 {
            e.saturating_add(e.saturating_mul(stft as u32) / 1000)
        } else {
            e.saturating_sub(e.saturating_mul((-stft) as u32) / 1000)
        };
        let t = l.saturating_mul(self.fuel_mult_x100() as u32) / 100;

        Micros::new(t.clamp(MIN_PULSE_WIDTH_US as u32, MAX_PULSE_WIDTH_US as u32))
    }

    pub fn refresh_snapshot(&mut self) {
        let trigger_inputs = self.trigger_inputs();
        let base_pw = self.calculate_fuel(trigger_inputs.rpm, self.map_kpa_x10 / 10);
        let final_pw = self.final_pw(Rpm::new(trigger_inputs.rpm), Kpa10::new(self.map_kpa_x10));
        self.outputs.final_pw = final_pw;
        let enrich_mult_x100 = [
            self.derived.wue_percent,
            self.derived.ase_percent,
            self.derived.ae_percent,
        ]
        .into_iter()
        .fold(100u32, |acc, pct| {
            acc.saturating_mul(100 + pct as u32) / 100
        }) as u16;
        let last_fault = self
            .diag_log()
            .events
            .iter()
            .rev()
            .find_map(|ev| ev.as_ref().map(|ev| ev.code));
        self.snapshot = ts::pages::SystemSnapshot {
            rpm: Rpm::new(trigger_inputs.rpm),
            sync: if trigger_inputs.synced {
                trigger::SyncState::Locked { cam_ref: false }
            } else {
                trigger::SyncState::Unsynced
            },
            base_pw: Micros::new(base_pw as u32),
            enrich_mult_x100,
            stft_x10: self.stft_x10(),
            fuel_mult_x100: self.fuel_mult_x100(),
            final_pw,
            last_fault,
            isr_stats: self.isr_stats,
        };
    }

    /// Calculate ignition timing with corrections
    ///
    /// Performs the complete ignition timing calculation:
    /// 1. Table lookup for base timing
    /// 2. Apply temperature and knock corrections
    /// 3. Clamp to safe limits
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC (positive = advance, negative = retard)
    pub fn calculate_ignition_timing(&self, rpm: u16, load: u16) -> i16 {
        let table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.config.ignition_table,
        };

        // 1. Base lookup
        let base_timing = table.lookup(rpm, load);

        // 2. Apply corrections and clamp
        ignition::calculate_timing(base_timing, self.ignition_corrections())
    }

    /// Calculate coil dwell time based on battery voltage
    ///
    /// # Returns
    /// Dwell time in microseconds
    pub fn calculate_dwell(&self) -> u32 {
        ignition::calculate_dwell(self.battery_voltage_mv())
    }

    /// Initialize IPW table with linear test values
    ///
    /// Creates a simple linear fuel map for initial testing.
    /// More fuel at higher load, slightly less at higher RPM.
    ///
    /// This is a helper method for hardware testing. Real tuning data
    /// should be loaded from external storage or CAN.
    pub fn init_linear_table(&mut self) {
        for row in 0..16 {
            for col in 0..16 {
                let base = DEFAULT_PULSE_WIDTH_US;
                let load_factor = (row as u16).saturating_mul(50); // 0-750us
                let rpm_factor = (col as u16).saturating_mul(10); // 0-150us

                // More fuel at higher load, slightly less at higher RPM
                self.config.ipw_table[row][col] =
                    base.saturating_add(load_factor).saturating_sub(rpm_factor);
            }
        }
    }

    /// Initialize ignition table with conservative values
    ///
    /// Creates a conservative ignition map safe for initial testing.
    /// Should be replaced with properly tuned values for production.
    pub fn init_ignition_table(&mut self) {
        let mut table = ignition::IgnitionTable {
            rpm_bins: constants::fuel::RPM_BINS,
            load_bins: constants::fuel::LOAD_BINS,
            values: self.config.ignition_table,
        };

        ignition::init_conservative_table(&mut table);
        self.config.ignition_table = table.values;
    }

    /// Update rev limiter state based on current RPM
    ///
    /// Should be called every engine cycle or in main loop.
    /// Updates internal limiter state which affects fuel and ignition.
    pub fn update_rev_limiter(&mut self) {
        let config = *self.rev_limiter_config();
        rev_limiter::update_limiter(
            self.trigger_inputs().rpm,
            &config,
            &mut self.rev_limiter_state,
        );
    }

    /// Check if fuel injection should proceed (considers rev limiter)
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should occur, `false` if limiter is cutting fuel
    pub fn should_inject_fuel(&self, cylinder: u8) -> bool {
        rev_limiter::should_inject(&self.rev_limiter_state, cylinder)
    }

    /// Calculate ignition timing with all corrections (including rev limiter)
    ///
    /// This is the main method to use - applies ignition corrections AND rev limiter retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter(&self, rpm: u16, load: u16) -> i16 {
        self.calculate_ignition_timing_with_limiter_cyl(rpm, load, 0)
    }

    /// Calculate ignition timing with all corrections (including rev limiter and knock)
    ///
    /// This is the main method to use - applies ignition corrections, rev limiter retard,
    /// and per-cylinder knock retard.
    ///
    /// # Arguments
    /// * `rpm` - Engine speed in RPM
    /// * `load` - Engine load in kPa
    /// * `cylinder` - Cylinder number (0-7) for per-cylinder knock retard
    ///
    /// # Returns
    /// Final timing in degrees BTDC with all corrections applied
    pub fn calculate_ignition_timing_with_limiter_cyl(
        &self,
        rpm: u16,
        load: u16,
        cylinder: u8,
    ) -> i16 {
        // Get base timing with normal corrections
        let base_timing = self.calculate_ignition_timing(rpm, load);

        // Apply rev limiter retard
        let with_limiter = rev_limiter::apply_limiter_retard(base_timing, &self.rev_limiter_state);

        // Apply knock retard (returns negative value)
        let knock_retard = self.knock_controller.get_retard_degrees(cylinder);
        (with_limiter + knock_retard).max(constants::ignition::MIN_TIMING_BTDC)
    }

    /// Update flood clear state based on current conditions
    ///
    /// Should be called every engine cycle or main loop iteration.
    ///
    /// # Returns
    /// `true` if flood clear is active (fuel should be cut)
    pub fn update_flood_clear(&mut self) -> bool {
        safety::update_flood_clear(
            self.trigger_inputs().rpm,
            self.tps_percent,
            &mut self.flood_clear_state,
        )
    }

    /// Record a sync loss event
    ///
    /// Call this when trigger sync is lost. The tracker will determine
    /// if this is an ESD glitch (recoverable) or real failure (shutdown).
    ///
    /// # Arguments
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if engine should shut down, `false` if should attempt recovery
    pub fn record_sync_loss(&mut self, current_time_us: u32) -> bool {
        self.synced = false;
        self.inputs.synced = false;
        self.sync_loss_tracker.record_sync_loss(current_time_us)
    }

    /// Record successful sync recovery
    ///
    /// Call this when sync is successfully re-established after a loss.
    pub fn record_sync_recovery(&mut self) {
        self.synced = true;
        self.inputs.synced = true;
        self.sync_loss_tracker.record_recovery();
    }

    /// Reset sync loss window after sustained good operation
    ///
    /// Call this periodically (e.g., every 10 seconds) when sync is stable.
    /// This allows the system to recover from old ESD events.
    pub fn reset_sync_loss_window(&mut self) {
        self.sync_loss_tracker.reset_window();
    }

    /// Check if fuel injection should proceed considering ALL safety features
    ///
    /// This is the master safety check. Returns `true` only if:
    /// - Not in flood clear mode
    /// - Not shut down due to sync loss
    /// - Rev limiter allows injection
    /// - Engine is synced
    /// - Voltage is not critically low
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder number (0-3)
    ///
    /// # Returns
    /// `true` if injection should proceed, `false` otherwise
    pub fn should_inject_with_all_safety(&self, cylinder: u8) -> bool {
        let trigger_inputs = self.trigger_inputs();
        // Must be synced
        if !trigger_inputs.synced {
            return false;
        }
        // Emergency mode blocks fuel
        if self.emergency_mode() {
            return false;
        }

        // Check voltage - critical low voltage blocks fuel
        if self.voltage_monitor.should_block_fuel() {
            return false;
        }

        // Check flood clear and shutdown
        if !safety::should_allow_injection(
            self.flood_clear_state.active,
            self.sync_loss_tracker.is_shutdown(),
        ) {
            return false;
        }

        // Check rev limiter
        if !self.should_inject_fuel(cylinder) {
            return false;
        }

        true
    }

    /// Update voltage monitor with current battery reading
    ///
    /// Should be called periodically (e.g., every 10-100ms) with ADC reading.
    ///
    /// # Arguments
    /// * `voltage_mv` - Battery voltage in millivolts
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current power state
    pub fn update_voltage(&mut self, voltage_mv: u16, now_us: u32) -> safety::PowerState {
        let state = self.voltage_monitor.update(voltage_mv, now_us);

        // Update battery_voltage_mv for other calculations (injector dead time, dwell)
        self.set_battery_voltage_mv(voltage_mv);

        // Log diagnostic events on state transitions
        if state == safety::PowerState::Critical && !self.diag_map.is_active() {
            // Log low voltage event (reusing diag infrastructure)
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::LowVoltage,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Sensor,
                context: Some(voltage_mv as u32),
                start_us: now_us,
                end_us: 0, // Will be updated when recovered
            });
        }

        state
    }

    /// Get effective RPM limit considering all sources
    ///
    /// Returns the most restrictive RPM limit from:
    /// - Rev limiter config
    /// - Voltage limp mode
    /// - Load failure limp mode
    pub fn get_effective_rpm_limit(&self) -> u16 {
        let mut limit = self.rev_limiter_config().max_rpm;
        let load_failure_config = *self.load_failure_config();

        // Apply voltage limp limit if active
        if let Some(voltage_limit) = self.voltage_monitor.get_rpm_limit() {
            limit = limit.min(voltage_limit);
        }

        // Apply load failure limp limit if active
        if let Some(load_limit) = self
            .load_failure_tracker
            .get_rpm_limit(&load_failure_config)
        {
            limit = limit.min(load_limit);
        }

        limit
    }

    /// Check for load failure condition (MAP fault at high RPM)
    ///
    /// Should be called after process_sensor_update to check if MAP fault
    /// combined with high RPM requires limp mode activation.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `true` if in load-failure limp mode
    pub fn check_load_failure(&mut self, now_us: u32) -> bool {
        let map_fault = self.diag_map.is_active();
        let load_failure_config = *self.load_failure_config();
        let trigger_inputs = self.trigger_inputs();
        let in_limp = self.load_failure_tracker.check(
            map_fault,
            trigger_inputs.rpm,
            &load_failure_config,
            now_us,
        );

        // Log event when entering limp mode
        if in_limp && self.load_failure_tracker.entered_us == now_us {
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::MapFailureHighLoad,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Safety,
                context: Some(trigger_inputs.rpm as u32),
                start_us: now_us,
                end_us: 0,
            });
        }

        in_limp
    }

    /// Check TPS vs MAP sensor plausibility
    ///
    /// Detects implausible sensor combinations that indicate sensor failure.
    /// Should be called after process_sensor_update.
    ///
    /// # Arguments
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// The confirmed plausibility fault (if any)
    pub fn check_plausibility(&mut self, now_us: u32) -> sensors::plausibility::PlausibilityFault {
        let old_has_fault = self.plausibility_state.has_fault();
        let plausibility_config = *self.plausibility_config();

        let fault = self.plausibility_state.check(
            self.tps_percent,
            self.map_kpa_x10,
            self.trigger_inputs().rpm,
            &plausibility_config,
            now_us,
        );

        // Log event when fault is first confirmed
        if self.plausibility_state.has_fault() && !old_has_fault {
            let tps_percent = self.tps_percent as u32;
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::TpsMapPlausibility,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Safety,
                context: Some(tps_percent),
                start_us: now_us,
                end_us: 0,
            });
        }

        fault
    }

    /// Check if there's a plausibility fault active
    pub fn has_plausibility_fault(&self) -> bool {
        self.plausibility_state.has_fault()
    }

    /// Validate sensor rate-of-change
    ///
    /// Filters out impossible sensor spikes that indicate noise or failure.
    /// Should be called before process_sensor_update for best filtering.
    ///
    /// # Arguments
    /// * `tps_percent` - Raw TPS reading
    /// * `map_kpa_x10` - Raw MAP reading
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// (validated_tps, validated_map) - Filtered values
    pub fn validate_sensor_rates(
        &mut self,
        tps_percent: u8,
        map_kpa_x10: u16,
        now_us: u32,
    ) -> (u8, u16) {
        let rate_config = *self.rate_config();
        let (validated_tps, validated_map, _, _) =
            self.rate_state
                .validate(tps_percent, map_kpa_x10, &rate_config, now_us);

        (validated_tps, validated_map)
    }

    /// Check if any sensor rate was rejected in the last update
    pub fn any_rate_rejected(&self) -> bool {
        self.rate_state.any_rejected()
    }

    /// Update LTFT learning
    ///
    /// Call this periodically (e.g., 10Hz) during normal operation.
    /// LTFT will only learn when conditions are stable.
    ///
    /// # Arguments
    /// * `clt_c` - Coolant temperature in Celsius
    /// * `now_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// Current LTFT value for the operating point (percent x10)
    pub fn update_ltft(&mut self, clt_c: i16, now_us: u32) -> i16 {
        let trigger_inputs = self.trigger_inputs();
        let map_kpa_x10 = self.map_kpa_x10;
        let stft_x10 = self.stft_x10();
        let lambda_active = self.lambda_state.active;
        self.ltft_manager_mut().update(
            trigger_inputs.rpm,
            map_kpa_x10,
            clt_c,
            stft_x10,
            lambda_active,
            now_us,
        )
    }

    /// Get combined fuel trim (STFT + LTFT)
    ///
    /// # Returns
    /// Combined fuel trim (percent x10), clamped to ±20%
    pub fn get_total_fuel_trim(&self) -> i16 {
        self.ltft_manager().get_total_trim(
            self.stft_x10(),
            self.trigger_inputs().rpm,
            self.map_kpa_x10,
        )
    }

    /// Reset LTFT learning
    ///
    /// Clears all learned values. Use via TunerStudio command or after
    /// major engine changes that invalidate learned data.
    pub fn reset_ltft(&mut self) {
        self.ltft_manager_mut().reset();
    }

    /// Check if LTFT learning is currently active
    pub fn is_ltft_learning(&self) -> bool {
        self.ltft_manager().state.learning_active
    }

    /// Get number of LTFT cells that have been learned
    pub fn ltft_learned_cell_count(&self) -> u8 {
        self.ltft_manager().table.learned_cell_count()
    }

    /// Process a knock sensor sample
    ///
    /// Call this during the knock window with the current sensor reading.
    ///
    /// # Arguments
    /// * `cylinder` - Cylinder index (0-7)
    /// * `level` - Knock sensor reading
    /// * `clt_c` - Coolant temperature in Celsius
    /// * `now_us` - Current timestamp
    ///
    /// # Returns
    /// `true` if knock was detected
    pub fn process_knock_sample(
        &mut self,
        cylinder: u8,
        level: u16,
        clt_c: i16,
        now_us: u32,
    ) -> bool {
        let detected = self.knock_controller.process(
            cylinder,
            level,
            self.trigger_inputs().rpm,
            clt_c,
            now_us,
        );

        // Log knock event to diagnostics
        if detected {
            self.diag_log_mut().push(diag::DiagEvent {
                code: diag::DiagCode::KnockDetected,
                timestamp: Micros::new(now_us),
                source: diag::DiagSource::Sensor,
                context: Some(cylinder as u32),
                start_us: now_us,
                end_us: 0,
            });
        }

        detected
    }

    /// Update knock timing recovery
    ///
    /// Call this periodically (e.g., every 100ms) to allow timing recovery.
    pub fn update_knock_recovery(&mut self, now_us: u32) {
        self.knock_controller.update_recovery(now_us);
    }

    /// Reset knock controller state
    pub fn reset_knock(&mut self) {
        self.knock_controller.reset();
    }

    /// Check if any knock retard is active
    pub fn has_knock_retard(&self) -> bool {
        self.knock_controller
            .state
            .has_retard(&self.knock_controller.config)
    }

    /// Get total knock count across all cylinders
    pub fn total_knock_count(&self) -> u32 {
        self.knock_controller.state.total_knock_count
    }

    /// Update torque controller with current conditions
    ///
    /// Call this periodically (e.g., in main loop) to update torque arbitration.
    ///
    /// # Arguments
    /// * `iat_c` - Intake air temperature in Celsius
    ///
    /// # Returns
    /// Arbitrated torque target (Nm x10)
    pub fn update_torque(&mut self, iat_c: i16) -> i16 {
        self.torque_controller
            .update(self.trigger_inputs().rpm, self.map_kpa_x10, iat_c)
    }

    /// Submit a driver torque request based on pedal position
    ///
    /// # Arguments
    /// * `pedal_percent` - Accelerator pedal position (0-100%)
    /// * `now_us` - Current timestamp
    pub fn request_driver_torque(&mut self, pedal_percent: u8, now_us: u32) {
        self.torque_controller
            .request_driver(pedal_percent, self.trigger_inputs().rpm, now_us);
    }

    /// Submit an idle controller torque request
    ///
    /// # Arguments
    /// * `target_rpm` - Target idle RPM
    /// * `now_us` - Current timestamp
    pub fn request_idle_torque(&mut self, target_rpm: u16, now_us: u32) {
        self.torque_controller
            .request_idle(target_rpm, self.trigger_inputs().rpm, now_us);
    }

    /// Submit torque limits based on current safety states
    ///
    /// Call this after updating safety monitors to apply torque limits.
    pub fn apply_safety_torque_limits(&mut self, now_us: u32) {
        // Rev limiter
        let rev_limited = self.rev_limiter_state.active;
        self.torque_controller
            .request_rev_limit(rev_limited, now_us);

        // Limp mode from voltage or load failure
        let limp_active = self.voltage_monitor.limp_active || self.load_failure_tracker.in_limp;
        self.torque_controller.request_limp(limp_active, now_us);
    }

    /// Get actuator targets from torque controller
    ///
    /// Returns fuel/timing modifications to achieve torque target.
    pub fn get_torque_actuators(&self) -> torque::ActuatorTargets {
        self.torque_controller
            .get_actuator_targets(self.trigger_inputs().rpm)
    }

    /// Check if torque is being limited
    pub fn is_torque_limited(&self) -> bool {
        self.torque_controller.is_limited()
    }

    /// Reset torque controller
    pub fn reset_torque(&mut self) {
        self.torque_controller.reset();
    }
}

impl Default for EcuState {
    fn default() -> Self {
        Self::new()
    }
}

impl EcuState {
    /// Clamp sensor values, update diag states, and set/clear emergency mode.
    /// Returns (clamped_map_kpa_x10, clamped_tps_percent).
    pub fn process_sensor_update(
        &mut self,
        now_us: Micros,
        raw_map_kpa_x10: Kpa10,
        raw_tps_percent: u8,
    ) -> (Kpa10, u8) {
        let lim = self.config.sensors_limits;
        let now_us = now_us.raw();
        let raw_map_kpa_x10 = raw_map_kpa_x10.raw();
        let map = raw_map_kpa_x10.clamp(lim.map_min_kpa_x10, lim.map_max_kpa_x10);
        let tps = raw_tps_percent.clamp(lim.tps_min_percent, lim.tps_max_percent);

        // MAP diag
        let map_oob =
            raw_map_kpa_x10 < lim.map_min_kpa_x10 || raw_map_kpa_x10 > lim.map_max_kpa_x10;
        if map_oob {
            if !self.diag_map.is_active() {
                self.diag_map.latch(Micros::new(now_us));
                self.diag_map.start_us = now_us;
                self.diag_map.in_range_since_us = 0;
                if self.emergency_trigger_map_oob() {
                    self.set_emergency_mode(true);
                }
            }
        } else if self.diag_map.is_active() {
            if self.diag_map.in_range_since_us == 0 {
                self.diag_map.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_map.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_map.start_us);
                self.diag_map.total_us = self.diag_map.total_us.saturating_add(dur);
                let start_us = self.diag_map.start_us;
                self.diag_log_mut().push(diag::DiagEvent {
                    code: diag::DiagCode::MapRange,
                    timestamp: Micros::new(now_us),
                    source: diag::DiagSource::Sensor,
                    context: Some(raw_map_kpa_x10 as u32),
                    start_us,
                    end_us: now_us,
                });
                self.diag_map.clear(Micros::new(now_us));
                self.diag_map = diag::DiagState::new();
            }
        }

        // TPS diag
        let tps_oob =
            raw_tps_percent < lim.tps_min_percent || raw_tps_percent > lim.tps_max_percent;
        if tps_oob {
            if !self.diag_tps.is_active() {
                self.diag_tps.latch(Micros::new(now_us));
                self.diag_tps.start_us = now_us;
                self.diag_tps.in_range_since_us = 0;
                if self.emergency_trigger_tps_oob() {
                    self.set_emergency_mode(true);
                }
            }
        } else if self.diag_tps.is_active() {
            if self.diag_tps.in_range_since_us == 0 {
                self.diag_tps.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_tps.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_tps.start_us);
                self.diag_tps.total_us = self.diag_tps.total_us.saturating_add(dur);
                let start_us = self.diag_tps.start_us;
                self.diag_log_mut().push(diag::DiagEvent {
                    code: diag::DiagCode::TpsRange,
                    timestamp: Micros::new(now_us),
                    source: diag::DiagSource::Sensor,
                    context: Some(raw_tps_percent as u32),
                    start_us,
                    end_us: now_us,
                });
                self.diag_tps.clear(Micros::new(now_us));
                self.diag_tps = diag::DiagState::new();
            }
        }

        // Clear emergency mode if triggers inactive
        if self.emergency_mode() {
            let map_emerg_active = self.emergency_trigger_map_oob() && self.diag_map.is_active();
            let tps_emerg_active = self.emergency_trigger_tps_oob() && self.diag_tps.is_active();
            if !(map_emerg_active || tps_emerg_active) {
                self.set_emergency_mode(false);
            }
        }

        self.set_map_kpa_x10(map);
        self.set_tps_percent(tps);
        (Kpa10::new(map), tps)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scale_u16_normal() {
        assert_eq!(scale_u16(1000, 150), 1500); // 1.5x
        assert_eq!(scale_u16(1000, 80), 800); // 0.8x
        assert_eq!(scale_u16(1000, 100), 1000); // 1.0x
        assert_eq!(scale_u16(500, 200), 1000); // 2.0x
    }

    #[test]
    fn test_scale_u16_saturation() {
        // Test overflow protection
        assert_eq!(scale_u16(u16::MAX, 200), u16::MAX); // Would overflow
        assert_eq!(scale_u16(50000, 200), u16::MAX); // Would overflow
    }

    #[test]
    fn test_fuel_calculation_clamping() {
        let mut state = EcuState::new();

        // Test minimum clamping (with very low correction)
        state.corrections_mut().clt = 10; // 0.1x (very low)
        let pw = state.calculate_fuel(3000, 60);
        assert_eq!(pw, MIN_PULSE_WIDTH_US);

        // Test maximum clamping (with very high base value and correction)
        // First set a high base value in the table
        // 3000 RPM maps to RPM bin index 5, 60 kPa maps to load bin index 4
        // Table is [load_idx][rpm_idx]
        state.config.ipw_table[4][5] = 15000; // 15ms base
        state.corrections_mut().clt = 255; // 2.55x (very high)
        state.corrections_mut().iat = 255;
        state.corrections_mut().vbatt = 255;
        // This should result in: 15000 * 2.55 * 2.55 * 2.55 = 249,146 which exceeds MAX
        let pw = state.calculate_fuel(3000, 60); // Maps to bin [4][5]
        assert_eq!(pw, MAX_PULSE_WIDTH_US);
    }

    #[test]
    fn test_process_sensor_update_keeps_getters_in_sync() {
        let mut state = EcuState::new();

        let (map, tps) = state.process_sensor_update(Micros::new(0), Kpa10::new(777), 42);

        assert_eq!(map.raw(), 777);
        assert_eq!(tps, 42);
        assert_eq!(state.map_kpa_x10(), 777);
        assert_eq!(state.tps_percent(), 42);
    }

    #[test]
    fn test_runtime_signals_view_tracks_scalar_mirrors() {
        let mut state = EcuState::new();
        state.set_rpm(2750);
        state.set_synced(true);
        state.set_tooth_count(7);
        state.set_battery_voltage_mv(12_450);
        state.set_clt_x10(830);
        state.set_iat_x10(410);
        state.set_tps_percent(17);
        state.set_map_kpa_x10(812);

        let runtime = state.runtime_signals();
        assert_eq!(
            runtime,
            RuntimeSignals {
                rpm: 2750,
                synced: true,
                tooth_count: 7,
                battery_voltage_mv: 12_450,
                clt_x10: 830,
                iat_x10: 410,
                tps_percent: 17,
                map_kpa_x10: 812,
            }
        );
        assert_eq!(state.current_rpm(), runtime.rpm);
        assert_eq!(state.current_synced(), runtime.synced);
        assert_eq!(state.rpm(), runtime.rpm);
        assert_eq!(state.synced(), runtime.synced);
        assert_eq!(state.tooth_count(), runtime.tooth_count);
        assert_eq!(state.battery_voltage_mv(), runtime.battery_voltage_mv);
        assert_eq!(state.clt_x10(), runtime.clt_x10);
        assert_eq!(state.iat_x10(), runtime.iat_x10);
        assert_eq!(state.tps_percent(), runtime.tps_percent);
        assert_eq!(state.map_kpa_x10(), runtime.map_kpa_x10);
    }

    #[test]
    fn test_diagnostic_and_safety_views_track_legacy_accessors() {
        let mut state = EcuState::new();
        state.set_emergency_trigger_map_oob(true);
        state.set_emergency_trigger_tps_oob(false);
        state.set_emergency_mode(true);
        state.rev_limiter_state.active = true;
        state.rev_limiter_state.fuel_cut_percent = 100;
        state.rev_limiter_state.ignition_retard = 12;

        let diagnostic = state.diagnostic_flags();
        let safety = state.safety_status();

        assert_eq!(
            diagnostic,
            DiagnosticFlags {
                emergency_trigger_map_oob: true,
                emergency_trigger_tps_oob: false,
                emergency_mode: true,
            }
        );
        assert_eq!(state.current_fault_flags(), (true, false, true));
        assert!(safety.fuel_cut_active);
        assert!(safety.spark_cut_active);
        assert!(state.fuel_cut_active());
        assert!(state.spark_cut_active());
    }

    #[test]
    fn test_fuel_calculation_normal() {
        let state = EcuState::new();

        // With default corrections (1.0x), should return table value
        let pw = state.calculate_fuel(3000, 60);
        assert_eq!(pw, DEFAULT_PULSE_WIDTH_US);
    }

    #[test]
    fn test_final_pw_base_only() {
        let state = EcuState::new();
        let base = state.calculate_fuel(3000, 60);

        assert_eq!(
            state.final_pw(Rpm::new(3000), Kpa10::new(60)),
            Micros::new(base as u32)
        );
    }

    #[test]
    fn test_final_pw_wue_active() {
        let mut state = EcuState::new();
        state.derived.wue_percent = 20;
        state.derived.ase_percent = 0;
        state.derived.ae_percent = 0;
        state.set_stft_x10(0);
        state.set_fuel_mult_x100(100);

        let base = state.calculate_fuel(3000, 60) as u32;
        assert_eq!(
            state.final_pw(Rpm::new(3000), Kpa10::new(60)),
            Micros::new((base * 120) / 100)
        );
    }

    #[test]
    fn test_final_pw_stft_plus_four_percent() {
        let mut state = EcuState::new();
        state.derived.wue_percent = 0;
        state.derived.ase_percent = 0;
        state.derived.ae_percent = 0;
        state.set_stft_x10(40);
        state.set_fuel_mult_x100(100);

        let base = state.calculate_fuel(3000, 60) as u32;
        assert_eq!(
            state.final_pw(Rpm::new(3000), Kpa10::new(60)),
            Micros::new((base * 104) / 100)
        );
    }

    #[test]
    fn test_final_pw_torque_multiplier_70_percent() {
        let mut state = EcuState::new();
        state.derived.wue_percent = 0;
        state.derived.ase_percent = 0;
        state.derived.ae_percent = 0;
        state.set_stft_x10(0);
        state.set_fuel_mult_x100(70);

        let base = state.calculate_fuel(3000, 60) as u32;
        assert_eq!(
            state.final_pw(Rpm::new(3000), Kpa10::new(60)),
            Micros::new((base * 70) / 100)
        );
    }

    #[test]
    fn test_linear_table_initialization() {
        let mut state = EcuState::new();
        state.init_linear_table();

        // Verify table has been populated
        // First cell should be base + 0 - 0
        assert_eq!(state.config.ipw_table[0][0], DEFAULT_PULSE_WIDTH_US);

        // Last cell should be base + 750 - 150
        let expected = DEFAULT_PULSE_WIDTH_US + 750 - 150;
        assert_eq!(state.config.ipw_table[15][15], expected);

        // Verify middle cell has reasonable value
        assert!(state.config.ipw_table[8][8] > DEFAULT_PULSE_WIDTH_US);
    }

    // --- Knock Integration Tests ---

    #[test]
    fn test_ecustate_knock_process_sample() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1; // Immediate detection

        // Below threshold - no knock
        let detected = state.process_knock_sample(0, 50, 80, 1000);
        assert!(!detected);
        assert!(!state.has_knock_retard());

        // Above threshold - knock detected
        let detected = state.process_knock_sample(0, 150, 80, 2000);
        assert!(detected);
        assert!(state.has_knock_retard());

        // Check that diag log contains knock event
        assert!(state
            .diag_log()
            .events
            .iter()
            .filter_map(|e| e.as_ref())
            .any(|e| e.code == diag::DiagCode::KnockDetected));
    }

    #[test]
    fn test_ecustate_knock_affects_timing() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.knock_controller.config.retard_step_x10 = 30; // 3 degrees per knock

        // Get base timing
        let base = state.calculate_ignition_timing_with_limiter_cyl(3000, 80, 0);

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 1000);

        // Check timing is retarded
        let after_knock = state.calculate_ignition_timing_with_limiter_cyl(3000, 80, 0);
        assert!(after_knock < base, "Timing should be retarded after knock");
        assert_eq!(base - after_knock, 3, "Should retard by 3 degrees");
    }

    #[test]
    fn test_ecustate_knock_recovery() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.knock_controller.config.retard_step_x10 = 50; // 5 degrees
        state.knock_controller.config.recovery_rate_x10 = 100; // 10 degrees/sec for faster test

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 0);
        assert!(state.has_knock_retard());

        // Recover over time (need enough time for 50 x10 units at 100 x10/sec = 0.5 sec)
        // With 100ms intervals, need 5 calls
        for i in 0..6 {
            state.update_knock_recovery(100_000 + i * 100_000);
        }

        // Should have recovered
        assert!(!state.has_knock_retard());
    }

    #[test]
    fn test_ecustate_knock_disabled_conditions() {
        let mut state = EcuState::new();
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1; // Immediate detection
        state.knock_controller.config.min_rpm = 2000;
        state.knock_controller.config.min_clt_c = 60;

        // Low RPM - disabled
        state.set_rpm(1500);
        let detected = state.process_knock_sample(0, 200, 80, 1000);
        assert!(!detected);

        // Cold engine - disabled
        state.set_rpm(3000);
        let detected = state.process_knock_sample(0, 200, 50, 2000);
        assert!(!detected);

        // Warm engine, good RPM - enabled
        let detected = state.process_knock_sample(0, 200, 80, 3000);
        assert!(detected);
    }

    // --- LTFT Integration Tests ---

    #[test]
    fn test_ecustate_ltft_learning() {
        let mut state = EcuState::new();
        state.set_rpm(2500);
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.set_stft_x10(30); // 3% rich
        state.ltft_manager_mut().config.enable = true;

        // Initial trim should be 0
        let trim = state.get_total_fuel_trim();
        assert_eq!(trim, 30); // Just STFT

        // Update LTFT several times with steady conditions
        for i in 0..20 {
            state.update_ltft(80, i * 1_000_000);
        }

        // Should be learning
        assert!(state.is_ltft_learning() || state.ltft_learned_cell_count() > 0);
    }

    #[test]
    fn test_ecustate_ltft_disabled_cold() {
        let mut state = EcuState::new();
        state.set_rpm(2500);
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.set_stft_x10(30);
        state.ltft_manager_mut().config.enable = true;
        state.ltft_manager_mut().config.min_clt_c = 70;

        // Cold engine - LTFT should not learn
        for i in 0..20 {
            state.update_ltft(50, i * 1_000_000);
        }

        assert_eq!(state.ltft_learned_cell_count(), 0);
    }

    #[test]
    fn test_ecustate_ltft_reset() {
        let mut state = EcuState::new();
        state.set_rpm(2500);
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.set_stft_x10(30);
        state.ltft_manager_mut().config.enable = true;

        // Learn for a while
        for i in 0..20 {
            state.update_ltft(80, i * 1_000_000);
        }

        // Reset
        state.reset_ltft();

        // All cells should be cleared
        assert_eq!(state.ltft_learned_cell_count(), 0);
    }

    // --- Torque Integration Tests ---

    #[test]
    fn test_ecustate_torque_driver_request() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.map_kpa_x10 = 800;

        // First update to get max available
        state.update_torque(25);

        // Driver pedal at 50%
        state.request_driver_torque(50, 1000);
        let torque = state.update_torque(25);

        // Should have ~50% of max available
        assert!(torque > 0);
        assert!(torque <= state.torque_controller.max_available_x10);
    }

    #[test]
    fn test_ecustate_torque_safety_limits() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.map_kpa_x10 = 800;

        // Update and request full power
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        let full_power = state.update_torque(25);

        // Activate rev limiter
        state.rev_limiter_state.active = true;
        state.apply_safety_torque_limits(2000);
        let limited = state.update_torque(25);

        // Should be severely limited
        assert!(limited < full_power, "Rev limiter should limit torque");
        assert!(state.is_torque_limited());
    }

    #[test]
    fn test_ecustate_torque_actuators() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.map_kpa_x10 = 800;

        // Update and request 50%
        state.update_torque(25);
        state.request_driver_torque(50, 1000);
        state.update_torque(25);

        let targets = state.get_torque_actuators();

        // Should have some fuel reduction or timing retard if limited
        // At 50% pedal with full MAP, driver usually gets what they want
        assert!(targets.fuel_mult_x100 <= 100);
    }

    #[test]
    fn test_ecustate_torque_zero_rpm() {
        let mut state = EcuState::new();
        state.set_rpm(0);
        state.map_kpa_x10 = 800;

        // Should handle 0 RPM gracefully
        let torque = state.update_torque(25);
        assert_eq!(torque, 0); // No torque at 0 RPM
    }

    // --- Cross-Module Integration Tests ---

    #[test]
    fn test_integration_knock_reduces_torque() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.map_kpa_x10 = 800;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.knock_controller.config.retard_step_x10 = 50; // 5 degrees

        // Update torque to get max available
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        let base_torque = state.update_torque(25);

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 2000);
        assert!(state.has_knock_retard());

        // Submit knock-based torque request
        let knock_retard = state.knock_controller.state.get_retard(0);
        state
            .torque_controller
            .arbiter
            .request(torque::request::knock_torque_request(
                knock_retard,
                state.torque_controller.max_available_x10,
                3000,
            ));
        let reduced_torque = state
            .torque_controller
            .arbiter
            .arbitrate(state.torque_controller.max_available_x10);

        // Knock should reduce available torque
        assert!(
            reduced_torque < base_torque,
            "Knock should reduce arbitrated torque"
        );
    }

    #[test]
    fn test_integration_torque_affects_actuators() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.map_kpa_x10 = 800;

        // Full power request
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        state.update_torque(25);

        let full_power_targets = state.get_torque_actuators();

        // Activate rev limiter
        state.rev_limiter_state.active = true;
        state.apply_safety_torque_limits(2000);
        state.update_torque(25);

        let limited_targets = state.get_torque_actuators();

        // Rev limiter should cause actuator changes
        assert!(
            limited_targets.fuel_cut
                || limited_targets.timing_reduced
                || limited_targets.fuel_mult_x100 < full_power_targets.fuel_mult_x100,
            "Rev limiter should cause actuator intervention"
        );
    }

    #[test]
    fn test_integration_limp_mode_propagation() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.map_kpa_x10 = 800;

        // Full power request
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        let full_power = state.update_torque(25);

        // Trigger load failure -> limp mode
        state.load_failure_tracker.in_limp = true;
        state.apply_safety_torque_limits(2000);
        let limp_torque = state.update_torque(25);

        // Limp mode should limit torque to ~30%
        assert!(
            limp_torque < full_power / 2,
            "Limp mode should severely limit torque"
        );
        assert!(state.is_torque_limited());
    }

    #[test]
    fn test_integration_multiple_safety_systems() {
        let mut state = EcuState::new();
        state.set_rpm(6500);
        state.map_kpa_x10 = 900;
        state.knock_controller.config.enable = true;
        state.knock_controller.config.threshold = 100;
        state.knock_controller.config.debounce_count = 1;
        state.rev_limiter_config_mut().max_rpm = 6500;

        // Driver wants full power
        state.update_torque(25);
        state.request_driver_torque(100, 1000);

        // Trigger knock
        state.process_knock_sample(0, 200, 80, 2000);

        // Update rev limiter (at hard limit)
        let rev_config = *state.rev_limiter_config();
        rev_limiter::update_limiter(state.rpm(), &rev_config, &mut state.rev_limiter_state);

        // Apply safety limits
        state.apply_safety_torque_limits(3000);

        // Get final timing with all corrections
        let final_timing = state.calculate_ignition_timing_with_limiter_cyl(state.rpm(), 80, 0);
        let base_timing = state.calculate_ignition_timing(state.rpm(), 80);

        // Timing should be reduced by both knock and rev limiter
        assert!(
            final_timing < base_timing,
            "Safety systems should reduce timing"
        );
    }

    #[test]
    fn test_integration_lambda_ltft_combined_trim() {
        let mut state = EcuState::new();
        state.set_rpm(2500);
        state.map_kpa_x10 = 600;
        state.lambda_state.active = true;
        state.set_stft_x10(30); // 3% STFT
        state.ltft_manager_mut().config.enable = true;

        // Pre-learn some LTFT (stft=20, rate=50, max_trim=200)
        let rpm = state.rpm();
        let map_kpa_x10 = state.map_kpa_x10;
        state
            .ltft_manager_mut()
            .table
            .learn(rpm, map_kpa_x10, 20, 50, 200);

        // Get combined trim
        let total_trim = state.get_total_fuel_trim();

        // Should combine STFT + LTFT
        assert!(total_trim > 30, "Combined trim should include LTFT");
        assert!(total_trim <= 200, "Combined trim should be clamped");
    }

    #[test]
    fn test_integration_sync_loss_disables_injection() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.set_synced(true);

        // Should allow injection when synced
        assert!(state.should_inject_with_all_safety(0));

        // Record sync loss
        let should_shutdown = state.record_sync_loss(1000);

        if !should_shutdown {
            // First loss doesn't shutdown, but should still not inject
            assert!(!state.synced());
            assert!(
                !state.should_inject_with_all_safety(0),
                "Should not inject without sync"
            );
        }
    }

    #[test]
    fn test_integration_voltage_affects_safety() {
        let mut state = EcuState::new();
        state.set_rpm(3000);
        state.set_synced(true);

        // Normal voltage - should inject
        assert!(state.should_inject_with_all_safety(0));

        // Critical low voltage
        state.voltage_monitor.limp_active = true;

        // Apply safety torque limits
        state.update_torque(25);
        state.request_driver_torque(100, 1000);
        state.apply_safety_torque_limits(2000);
        let _limited_torque = state.update_torque(25);

        // Limp mode should be active
        assert!(state.is_torque_limited());
    }
}
