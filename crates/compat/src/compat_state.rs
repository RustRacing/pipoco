use crate::Micros;
use crate::{diag, lambda};

/// Adapter contracts for root/core boundary fields.
///
/// These document the model differences between legacy ecu-compat compatibility
/// table helpers and runtime/spec-oracle fuel models.
///
/// DO NOT add new variants without a corresponding test in fm0016_core_reducer.rs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoreAdapterContract {
    /// Legacy core uses IPW (Injector Pulse Width) tables — direct (rpm, load) → PW mapping.
    /// Spec oracle uses VE (Volumetric Efficiency) model with full correction pipeline.
    IpwVsVeFuelModel,
    /// Root ignition timing from IPW table vs spec timing table lookup.
    TimingTableVsFrozenSpec,
    /// Base PW computed via IPW vs spec VE displacement model.
    BasePwIncomparable,
    /// Corrected PW computed via IPW corrections vs spec VE corrections.
    CorrectedPwIncomparable,
}

/// Product-owned observable surface for the root `EcuState` compatibility
/// boundary.
///
/// This is the narrow FM0016-facing view that legacy compatibility tests should
/// consume instead of reconstructing the same state ad hoc in test code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoreObservedSurface {
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
    pub base_pw_us: u16,
    pub final_pw_us: u16,
    pub final_pw_output_us: u32,
    pub fuel_mult_x100: u16,
    pub wue_percent: u8,
    pub ase_percent: u8,
    pub ae_percent: u8,
    pub enrich_mult_x100: u16,
    pub stft_x10: i16,
    pub clt_enrich_pct: u16,
    pub iat_enrich_pct: u16,
    pub vbatt_enrich_pct: u16,
    pub ign_clt_correction_deg: i16,
    pub ign_iat_correction_deg: i16,
    pub ign_knock_retard_deg: i16,
    pub spark_base_timing_deg: i16,
    pub spark_advance_deg: i16,
    pub spark_advance_with_limiter_deg: i16,
    pub commanded_advance_x10_output: i16,
    pub dwell_us: u32,
    pub rev_limiter_active: bool,
    pub rev_limiter_fuel_cut_pct: u8,
    pub rev_limiter_ign_retard_deg: i16,
    pub ltft_learning: bool,
    pub ltft_learned_cell_count: u8,
    pub knock_retard_active: bool,
    pub total_knock_count: u32,
    pub torque_limited: bool,
    pub emergency_trigger_map_oob: bool,
    pub emergency_trigger_tps_oob: bool,
    pub emergency_mode: bool,
    pub last_fault_code: u8,
    pub isr_count: u32,
    pub isr_max_us: u32,
    pub isr_avg_us: u32,
    pub has_fault: bool,
    pub inject_fuel_allowed: bool,
    pub fuel_cut: bool,
    pub spark_cut: bool,
}

/// Runtime mirror of the live scalar inputs that are still duplicated on `EcuState`.
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
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
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DiagnosticFlags {
    pub emergency_trigger_map_oob: bool,
    pub emergency_trigger_tps_oob: bool,
    pub emergency_mode: bool,
}

/// Safety outputs derived from limiter and cut state.
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyStatus {
    pub fuel_cut_active: bool,
    pub spark_cut_active: bool,
}

/// Global ECU state configuration.
///
/// Relocated to `ecu-calibration`; re-exported here so existing
/// `ecu_compat::EcuConfig` / `compat_state::EcuConfig` paths resolve unchanged.
pub use ecu_calibration::EcuConfig;

/// Trigger-derived live inputs that are in the process of being separated
/// from the main `EcuState` layout.
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
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
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
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
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
#[derive(Debug, Clone, Copy)]
pub struct EcuOutputs {
    pub final_pw: Micros,
    pub commanded_advance_x10: i16,
}

/// Fault and diagnostics cache that is in the process of being separated from
/// the main `EcuState` layout.
///
/// Compatibility-only surface: keep stable for migration, but do not add new
/// runtime-facing fields without an explicit migration rationale.
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
