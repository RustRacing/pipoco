use crate::{
    outputs::runtime_full_sequential_authorized, Action, ActionBatch, RuntimeFuelStrategy,
    RuntimeOutputProfile, RuntimeScheduledOutputKind, RUNTIME_ACTION_CAP,
};
use ecu_calibration::{CalibrationPackageIdentity, CalibrationSnapshot};
use ecu_control::{
    AllowedTorque, EnrichmentInputs, EnrichmentResult, FuelAfterstartWindowMode, FuelIntent,
    FuelStartupWindowMode, FuelWarmupTemperatureMode, IgnitionInputs, IgnitionPlan,
    LambdaDisableReason, LambdaMode, LambdaTrimInputs, LambdaTrimResult, TorqueInputs,
};
use ecu_domain::{
    CancelReason, ControlMode, Degrees10, DwellUs, EnginePhase, EngineTimeAuthority, FaultCode,
    FaultSeverity, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm, SyncState,
};
use ecu_scheduler::{SchedulerState, MODEL_MAX_PENDING};

/// Engine-owned runtime state shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EngineState {
    pub sync: SyncState,
    pub engine_time_authority: EngineTimeAuthority,
    pub phase: EnginePhase,
    pub mode: ControlMode,
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
}

/// Control-owned runtime state shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControlState {
    pub fuel_pulse_width: PulseWidthUs,
    pub ignition_advance: Degrees10,
    pub dwell: DwellUs,
    pub lambda_target: Lambda100,
    pub torque_limit_x100: u16,
}

/// Fault-owned runtime state shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FaultState {
    pub fault: FaultCode,
    pub severity: FaultSeverity,
    pub cancel_reason: CancelReason,
}

/// Calibration-owned runtime state shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CalibrationState {
    pub active: CalibrationSnapshot,
    pub staged_dirty: bool,
}

/// Compact calibration package identity extracted from runtime-owned state.
pub type RuntimeCalibrationIdentityObservations = CalibrationPackageIdentity;

/// Snapshot of runtime-owned state for publication and diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeSnapshot {
    pub engine: EngineState,
    pub control: ControlState,
    pub faults: FaultState,
    pub calibration: CalibrationState,
    pub scheduler: SchedulerState,
    pub output_profile: RuntimeOutputProfile,
    pub fuel_strategy_mode: RuntimeFuelStrategyMode,
    /// Soft rev limiter is currently active.
    pub rev_soft_active: bool,
    /// Hard rev limiter is currently active.
    pub rev_hard_active: bool,
    /// Launch limiter is currently active.
    pub launch_active: bool,
    /// Flat-shift limiter is currently active.
    pub flat_shift_active: bool,
    /// Safety latch is currently active.
    pub safety_latched: bool,
    /// Direct fuel-cut request is currently active.
    pub direct_fuel_cut_request: bool,
    /// Direct spark-cut request is currently active.
    pub direct_spark_cut_request: bool,
    /// Fuel cut is currently active.
    pub fuel_cut: bool,
    /// Spark cut is currently active.
    pub spark_cut: bool,
    /// FM0016-compatible legacy cut reason code derived from runtime-owned state.
    pub legacy_cut_reason_code: u8,
    /// Last knock intensity ingressed on the runtime product step path.
    pub knock_intensity_x100: u16,
    /// Semantic knock retard currently retained by the selected fuel strategy.
    pub knock_retard_deg10: i16,
}

/// Compatibility projection of runtime cut state into the FM0016 legacy view.
///
/// This is not the native runtime cut model. Native runtime state keeps
/// channel-specific `fuel_cut` and `spark_cut` ownership separate. The legacy
/// projection exists only for compatibility with FM0016 oracle rows that still
/// use older cut-active semantics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeLegacyCutFlags {
    pub fuel_cut: bool,
    pub spark_cut: bool,
}

/// Runtime rejection for malformed engine-time authority supplied by an adapter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeAuthorityError {
    pub authority: EngineTimeAuthority,
    pub reason: ecu_domain::EngineTimeAuthorityError,
}

/// Trigger decoder observation owned by the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TriggerObservation {
    pub at_us: Micros,
    pub rpm: Rpm,
    pub angle_x10: Degrees10,
    pub synced: bool,
}

/// Cam decoder observation owned by the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CamObservation {
    pub at_us: Micros,
    pub cam_seen: bool,
}

/// Observation input that the runtime converts into sync state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecoderObservation {
    Trigger(TriggerObservation),
    Cam(CamObservation),
}

/// Compatibility/support raw inputs accepted by the runtime step pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepInputs {
    pub now_us: Micros,
    pub rpm: u32,
    pub load_kpa10: u32,
    pub angle_x10: i32,
    pub trigger_synced: bool,
    pub cam_seen: bool,
    /// Launch arming input flag from the fixture.
    pub launch_armed: bool,
    /// Flat-shift arming input flag from the fixture.
    pub flat_shift_armed: bool,
    /// Explicit safety-latch request on the live step ingress.
    pub safety_latch_request: bool,
}

/// Canonical product ingress for one runtime step.
///
/// Unlike [`StepInputs`], this frame carries engine-time authority directly and
/// does not rely on boolean sync/cam side channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuthorityStepInputs {
    pub now_us: Micros,
    pub rpm: u32,
    pub load_kpa10: u32,
    pub angle_x10: i32,
    pub authority: EngineTimeAuthority,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub safety_latch_request: bool,
}

impl AuthorityStepInputs {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        now_us: Micros,
        rpm: u32,
        load_kpa10: u32,
        angle_x10: i32,
        authority: EngineTimeAuthority,
        launch_armed: bool,
        flat_shift_armed: bool,
        safety_latch_request: bool,
    ) -> Self {
        Self {
            now_us,
            rpm,
            load_kpa10,
            angle_x10,
            authority,
            launch_armed,
            flat_shift_armed,
            safety_latch_request,
        }
    }
}

/// Runtime engine-mode input used for formal differential fixture mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeEngineMode {
    Off,
    Cranking,
    Running,
    Shutdown,
}

/// Runtime AFR override input used for formal differential fixture mapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeAfrOverride {
    None,
    Some(u16),
}

/// Expanded runtime input surface for FM0016 differential fixture representability.
///
/// `mode`, `fuel_cut`, and `spark_cut` are formal-only fields. They are honored
/// by [`crate::EngineRuntime::step_with_differential_input`], not by
/// [`DifferentialInputSnapshot::to_step_inputs`] or
/// [`DifferentialInputSnapshot::to_authority_step_inputs`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DifferentialInputSnapshot {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub baro_kpa10: Kpa10,
    pub vbatt_mv: u16,
    pub sync: SyncState,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub mode: RuntimeEngineMode,
    pub target_afr_override_x100: RuntimeAfrOverride,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub safety_latch_request: bool,
}

impl DifferentialInputSnapshot {
    pub fn to_step_inputs(self) -> StepInputs {
        StepInputs {
            now_us: self.now_us,
            rpm: self.rpm.get() as u32,
            load_kpa10: self.load_kpa10.get() as u32,
            angle_x10: self.angle_x10.get() as i32,
            trigger_synced: matches!(self.sync, SyncState::Locked { .. }),
            cam_seen: matches!(self.sync, SyncState::Locked { .. }),
            launch_armed: self.launch_armed,
            flat_shift_armed: self.flat_shift_armed,
            safety_latch_request: self.safety_latch_request,
        }
    }

    pub fn to_authority_step_inputs(self) -> AuthorityStepInputs {
        AuthorityStepInputs {
            now_us: self.now_us,
            rpm: self.rpm.get() as u32,
            load_kpa10: self.load_kpa10.get() as u32,
            angle_x10: self.angle_x10.get() as i32,
            authority: authority_from_sync_summary(self.sync),
            launch_armed: self.launch_armed,
            flat_shift_armed: self.flat_shift_armed,
            safety_latch_request: self.safety_latch_request,
        }
    }
}

const fn authority_from_sync_summary(sync: SyncState) -> EngineTimeAuthority {
    match sync {
        SyncState::Locked { .. } => EngineTimeAuthority::new(
            ecu_domain::CrankSyncState::PrimaryLocked,
            ecu_domain::PhaseSyncState::CrankOnly360,
            ecu_domain::AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        ),
        SyncState::Provisional => EngineTimeAuthority::new(
            ecu_domain::CrankSyncState::PrimarySearching,
            ecu_domain::PhaseSyncState::Unknown,
            ecu_domain::AbsoluteTimeAuthority::None,
            0,
            0,
        ),
        SyncState::Unsynced => EngineTimeAuthority::none(),
    }
}

/// Inputs for the composed control planners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelSensorInputs {
    pub maf_valid: bool,
    pub maf_x100: u16,
    pub iat_c10: i16,
    pub vbatt_mv: u16,
    pub baro_valid: bool,
    pub baro_kpa10: Kpa10,
}

impl FuelSensorInputs {
    pub const fn explicit_substitutions() -> Self {
        Self {
            maf_valid: false,
            maf_x100: 0,
            iat_c10: 250,
            vbatt_mv: 12_000,
            baro_valid: false,
            baro_kpa10: Kpa10::new(1010),
        }
    }
}

impl Default for FuelSensorInputs {
    fn default() -> Self {
        Self::explicit_substitutions()
    }
}

/// Inputs for the composed control planners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlInputs {
    pub enrichment: EnrichmentInputs,
    pub lambda: LambdaTrimInputs,
    pub torque: TorqueInputs,
    pub ignition: IgnitionInputs,
    pub fuel_sensors: FuelSensorInputs,
    pub knock_intensity_x100: u16,
}

impl ControlInputs {
    pub const fn spark_only(now_us: Micros, ignition: IgnitionInputs) -> Self {
        Self {
            enrichment: EnrichmentInputs {
                now_us,
                clt_c: 0,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                now_us,
                clt_c: 0,
                just_started: false,
                lambda_valid: false,
                measured_lambda100: Lambda100::new(100),
                requested_open_loop: true,
            },
            torque: TorqueInputs::new(100, 100, 100, 100, 100),
            ignition,
            fuel_sensors: FuelSensorInputs::explicit_substitutions(),
            knock_intensity_x100: 0,
        }
    }
}

/// Composed control intent produced by the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlPlan {
    pub base_fuel: PulseWidthUs,
    pub enriched_fuel: PulseWidthUs,
    pub enrichment: EnrichmentResult,
    pub lambda: LambdaTrimResult,
    pub torque: AllowedTorque,
    pub ignition: IgnitionPlan,
    /// Fuel cut is currently active.
    pub fuel_cut: bool,
    /// Spark cut is currently active.
    pub spark_cut: bool,
    /// Scheduler-facing bounded fuel intent used to arm injection outputs.
    pub fuel_intent: FuelIntent,
}

/// Product-owned torque observations emitted by `EngineRuntime::step`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TorqueObservations {
    /// Torque request in x1000, derived from the product x100 request.
    /// This is emitted by runtime product code, not a conformance helper.
    pub request_x1000: u16,
    /// Torque allowed in x1000, derived from the product x100 limiter output.
    /// The live step path zeroes this for `EnginePhase::Off`,
    /// `ControlMode::Shutdown`, or an active product hard-rev cut.
    pub allowed_x1000: u16,
    /// Torque actuated in x1000, gated by the product cut state visible on the
    /// step path.
    ///
    /// This is derived from the current-step product fuel/spark cut outputs,
    /// not from action-shape heuristics or semantic scaffolding.
    pub actuated_x1000: u16,
}

impl TorqueObservations {
    pub fn from_step(
        torque: AllowedTorque,
        operating_mode: ControlMode,
        engine_phase: EnginePhase,
        rev_hard_active: bool,
        fuel_cut: bool,
        spark_cut: bool,
    ) -> Self {
        let request_x1000 = torque.requested_x1000;
        let allowed_x1000 = if matches!(operating_mode, ControlMode::Shutdown)
            || matches!(engine_phase, EnginePhase::Off)
            || rev_hard_active
        {
            0
        } else {
            torque.allowed_x1000
        };
        let actuated_x1000 = if fuel_cut || spark_cut {
            0
        } else {
            allowed_x1000
        };

        Self {
            request_x1000,
            allowed_x1000,
            actuated_x1000,
        }
    }
}

/// Sanitized inputs owned by the runtime after validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidatedInputs {
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
    pub clamped: bool,
}

/// Outcome of one deterministic runtime step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StepResult {
    pub validated: ValidatedInputs,
    pub operating_mode: ControlMode,
    pub control: ControlPlan,
    pub actions: ActionBatch<RUNTIME_ACTION_CAP>,
    pub torque_observations: TorqueObservations,
}

/// Compact current cut-state provenance extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeCutObservations {
    /// Fuel cut is currently active.
    pub fuel_cut: bool,
    /// Spark cut is currently active.
    pub spark_cut: bool,
    /// Current cut-state provenance.
    pub reason: RuntimeCutReason,
}

/// Current cut-state provenance extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeCutReason {
    #[default]
    None,
    SafetyLatched,
    DirectRequest,
    Shutdown,
    HardRev,
    Launch,
    FlatShift,
    FuelOnly,
    SoftRev,
    SparkOnly,
    KnockRetard,
}

/// Compact current protection meaning extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeProtectionObservations {
    /// Current runtime protection level.
    pub level: RuntimeProtectionLevel,
    /// Current source driving protection behavior.
    pub source: RuntimeProtectionSource,
    /// Immediate action meaning of the current protection state.
    pub action: RuntimeProtectionAction,
    /// How the current protection state is expected to clear.
    pub persistence: RuntimeProtectionPersistence,
}

/// Current protection level extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeProtectionLevel {
    #[default]
    Inactive,
    Degraded,
    ShutdownDriving,
}

/// Current protection source extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeProtectionSource {
    #[default]
    None,
    RuntimeFault,
    SafetyLatch,
    ControlMode,
}

/// Immediate protection action meaning extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeProtectionAction {
    #[default]
    None,
    ObserveOnly,
    LimpHome,
    OutputSuppressed,
    Shutdown,
}

/// Current protection persistence extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeProtectionPersistence {
    #[default]
    Inactive,
    Reversible,
    LatchedUntilClear,
}

/// Current runtime fault shell extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeFaultObservations {
    /// Whether a runtime fault is currently active.
    pub active: bool,
    /// Current runtime fault identity.
    pub fault: FaultCode,
    /// Current runtime fault severity.
    pub severity: FaultSeverity,
    /// Current runtime fault cancel reason.
    pub cancel_reason: CancelReason,
    /// Current action meaning implied by the runtime fault state.
    pub action: RuntimeFaultAction,
    /// How the current fault-driven protection action is expected to clear.
    pub persistence: RuntimeProtectionPersistence,
}

/// Current runtime fault action meaning extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeFaultAction {
    #[default]
    None,
    ObserveOnly,
    LimpHome,
    Shutdown,
}

#[inline]
const fn runtime_fault_action_policy(
    fault: FaultCode,
    severity: FaultSeverity,
    cancel_reason: CancelReason,
) -> (RuntimeFaultAction, RuntimeProtectionPersistence) {
    if matches!(fault, FaultCode::None) {
        (
            RuntimeFaultAction::None,
            RuntimeProtectionPersistence::Inactive,
        )
    } else if matches!(cancel_reason, CancelReason::SafetyShutdown)
        || matches!(severity, FaultSeverity::Critical)
        || matches!(fault, FaultCode::SafetyCut)
    {
        (
            RuntimeFaultAction::Shutdown,
            RuntimeProtectionPersistence::LatchedUntilClear,
        )
    } else if matches!(severity, FaultSeverity::Warning)
        || matches!(fault, FaultCode::SensorOutOfRange)
    {
        (
            RuntimeFaultAction::LimpHome,
            RuntimeProtectionPersistence::LatchedUntilClear,
        )
    } else {
        (
            RuntimeFaultAction::ObserveOnly,
            RuntimeProtectionPersistence::LatchedUntilClear,
        )
    }
}

#[inline]
const fn runtime_protection_action_from_fault_action(
    action: RuntimeFaultAction,
) -> RuntimeProtectionAction {
    match action {
        RuntimeFaultAction::None => RuntimeProtectionAction::None,
        RuntimeFaultAction::ObserveOnly => RuntimeProtectionAction::ObserveOnly,
        RuntimeFaultAction::LimpHome => RuntimeProtectionAction::LimpHome,
        RuntimeFaultAction::Shutdown => RuntimeProtectionAction::Shutdown,
    }
}

/// Current engine-time authority shell extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeAuthorityObservations {
    /// Full runtime authority snapshot.
    pub authority: EngineTimeAuthority,
    /// Current compatibility summary.
    pub summary: SyncState,
    /// Current engine phase derived from the authority snapshot.
    pub phase: EnginePhase,
    /// Whether the current authority admits full sequential outputs.
    pub full_sequential_authorized: bool,
}

/// Current knock shell extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeKnockObservations {
    /// Last knock intensity ingressed on the runtime product step path.
    pub intensity_x100: u16,
    /// Retained knock retard on the active runtime strategy.
    pub retard_deg10: i16,
}

/// Current runtime action shell extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeActionObservations {
    /// Total action count in the current runtime step.
    pub total_action_count: u8,
    /// Number of combined scheduler-arm actions.
    pub arm_scheduler_count: u8,
    /// Number of direct injection-arm actions.
    pub arm_injection_count: u8,
    /// Number of direct ignition-arm actions.
    pub arm_ignition_count: u8,
    /// Number of auxiliary-apply actions.
    pub apply_aux_count: u8,
    /// Total auxiliary command count across all apply-aux actions.
    pub apply_aux_command_count: u8,
    /// Number of idle actions.
    pub idle_count: u8,
    /// Number of publish-snapshot actions.
    pub publish_snapshot_count: u8,
    /// Whether at least one publish-snapshot action is present.
    pub publish_snapshot: bool,
    /// Whether at least one persist-calibration action is present.
    pub persist_calibration: bool,
    /// Number of persist-calibration actions.
    pub persist_calibration_count: u8,
    /// Whether at least one cancel-scheduler action is present.
    pub cancel_scheduler: bool,
    /// Last cancel reason observed on the current action batch.
    pub cancel_reason: CancelReason,
    /// Number of cancel-scheduler actions.
    pub cancel_scheduler_count: u8,
    /// Whether multiple distinct cancel reasons were observed.
    pub multiple_cancel_reasons: bool,
}

/// Current scheduled-transition shell extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeTransitionObservations {
    /// Total scheduled transition count across the current runtime step.
    pub total_transition_count: u8,
    /// Total injector transition count across the current runtime step.
    pub injector_transition_count: u8,
    /// Total ignition transition count across the current runtime step.
    pub ignition_transition_count: u8,
    /// Earliest scheduled transition time across all outputs.
    pub earliest_transition_at_us: Option<Micros>,
    /// Latest scheduled transition time across all outputs.
    pub latest_transition_at_us: Option<Micros>,
    /// Earliest injector transition time.
    pub earliest_injector_transition_at_us: Option<Micros>,
    /// Latest injector transition time.
    pub latest_injector_transition_at_us: Option<Micros>,
    /// Earliest ignition transition time.
    pub earliest_ignition_transition_at_us: Option<Micros>,
    /// Latest ignition transition time.
    pub latest_ignition_transition_at_us: Option<Micros>,
    /// Whether any action transition export failed unexpectedly.
    pub export_error: bool,
}

/// Compact current fuel-strategy shell extracted from runtime product state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeFuelStrategyMode {
    #[default]
    DirectPulseWidthTable,
    SpeedDensityVe,
    AlphaN,
    Maf,
}

/// Observable surface for runtime conformance.
/// Fields read from StepResult, RuntimeSnapshot, ControlPlan, ActionBatch, and public runtime state only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeObservedSurface {
    pub rpm: u16,
    pub sync: bool,
    /// Product-owned current engine shell extracted from `RuntimeSnapshot`.
    pub runtime_engine: EngineState,
    /// Product-owned current control shell extracted from `RuntimeSnapshot`.
    pub runtime_control: ControlState,
    /// Product-owned current calibration shell extracted from `RuntimeSnapshot`.
    pub runtime_calibration: CalibrationState,
    /// Product-owned calibration package identity extracted from `RuntimeSnapshot`.
    pub runtime_calibration_identity: RuntimeCalibrationIdentityObservations,
    /// Product-owned current scheduler shell extracted from `RuntimeSnapshot`.
    pub runtime_scheduler: SchedulerState,
    /// Product-owned current output-profile shell extracted from `RuntimeSnapshot`.
    pub runtime_output_profile: RuntimeOutputProfile,
    /// Product-owned current fuel-strategy shell extracted from `RuntimeSnapshot`.
    pub runtime_fuel_strategy: RuntimeFuelStrategyMode,
    /// Product-owned current authority shell extracted from `RuntimeSnapshot`.
    pub runtime_authority: RuntimeAuthorityObservations,
    /// Product-owned validated input shell extracted directly from `StepResult`.
    pub runtime_validated: ValidatedInputs,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub legacy_cut_reason_code: u8,
    /// Product-owned current runtime cut-state shell extracted from `RuntimeSnapshot`.
    pub runtime_cut: RuntimeCutObservations,
    /// Product-owned current runtime fault shell extracted from `RuntimeSnapshot`.
    pub runtime_fault: RuntimeFaultObservations,
    /// Product-owned current runtime protection shell extracted from `RuntimeSnapshot`.
    pub runtime_protection: RuntimeProtectionObservations,
    /// Product-owned current knock shell extracted from `RuntimeSnapshot`.
    pub runtime_knock: RuntimeKnockObservations,
    pub knock_intensity_x100: u16,
    pub knock_retard_deg10: i16,
    pub torque_request_x100: u16,
    pub torque_allowed_x100: u16,
    pub torque_actuated_x100: u16,
    /// Product-owned torque request observation in x1000.
    pub torque_request_x1000: u16,
    /// Product-owned torque allowed observation in x1000.
    pub torque_allowed_x1000: u16,
    /// Product-owned torque actuated observation in x1000.
    pub torque_actuated_x1000: u16,
    /// Product-owned torque observation shell extracted directly from `StepResult`.
    pub runtime_torque: TorqueObservations,
    /// Base fuel pulse-width in microseconds (from ControlPlan.base_fuel).
    /// NOTE: runtime uses IPW table lookup, NOT VE computation. The IPW value
    /// is NOT semantically equivalent to spec's VE-derived pw_base_us.
    pub runtime_base_fuel_pw_us: u16,
    /// Enriched fuel pulse-width in microseconds (from ControlPlan.enriched_fuel).
    /// NOTE: This is the post-enrichment IPW, not comparable to spec pw_corr_us
    /// unless runtime enrichment semantics match spec correction semantics.
    pub runtime_enriched_fuel_pw_us: u16,
    /// Lambda target as Lambda100 ratio (e.g., 142 = 1.42).
    /// NOTE: This is a lambda ratio, NOT AFR. Do NOT compare to target_afr_x100.
    pub runtime_lambda_target_x100: u16,
    /// Product-owned runtime lambda-state shell extracted from `StepResult`.
    pub runtime_lambda: RuntimeLambdaObservations,
    /// Product-owned runtime lambda-correction shell extracted from `StepResult`.
    pub runtime_lambda_correction: RuntimeLambdaCorrectionObservations,
    /// Product-owned runtime fuel-state shell extracted from `StepResult`.
    pub runtime_fuel: RuntimeFuelObservations,
    /// Product-owned runtime fuel-core shell extracted from `StepResult`.
    pub runtime_fuel_core: RuntimeFuelCoreObservations,
    /// Product-owned runtime idle-state shell extracted from `StepResult`.
    pub runtime_idle: RuntimeIdleObservations,
    /// Product-owned runtime ignition-trim shell extracted from `StepResult`.
    pub runtime_ignition_trim: RuntimeIgnitionTrimObservations,
    /// Product-owned ignition plan shell extracted directly from `StepResult`.
    pub runtime_ignition: IgnitionPlan,
    /// Product-owned action shell extracted directly from `StepResult`.
    pub runtime_actions: RuntimeActionObservations,
    /// Product-owned scheduled-transition shell extracted directly from `StepResult`.
    pub runtime_transitions: RuntimeTransitionObservations,
    pub ignition_advance_deg10: i16,
    pub dwell_us: u16,
    pub control_mode: ControlMode,
    pub validated_rpm: u16,
    pub validated_load_kpa10: u16,
    pub validated_clamped: bool,
}

/// Adapter contracts for runtime fields that cannot be directly compared to spec.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeAdapterContract {
    /// VE fuel percentage (x100) - runtime uses IPW table, not VE computation.
    VeFuelPercentage,
    /// Target AFR (x100) - runtime does not expose target AFR result.
    TargetAfr,
    /// Base fuel PW - runtime IPW vs spec VE base PW are architecturally different.
    BaseFuelPw,
    /// Air fuel PW - runtime does not expose speed-density air PW.
    AirFuelPw,
    /// Corrected fuel PW - runtime uses different correction pipeline than spec.
    CorrectedFuelPw,
    /// Idle duty - runtime idle controller state not exposed in public API.
    IdleDuty,
    /// Lambda correction - runtime lambda trim result differs from spec lambda_correction.
    LambdaCorrection,
    /// Ignition advance trim - runtime advance is raw table vs spec trim value.
    IgnitionAdvanceTrim,
    /// Cut reason code - runtime does not independently derive cut reason.
    CutReasonCode,
    /// Fuel-cut input flag - runtime does not model the frozen oracle cut input directly.
    FuelCutInput,
    /// Spark-cut input flag - runtime does not model the frozen oracle cut input directly.
    SparkCutInput,
    /// Safety latch - runtime does not expose the direct latch source on the step path.
    SafetyLatched,
    /// Launch cut - runtime does not expose the direct launch cut source on the step path.
    LaunchCut,
    /// Flat-shift cut - runtime does not expose the direct flat-shift cut source on the step path.
    FlatShiftCut,
    /// Knock intensity - runtime does not expose knock sensor output.
    KnockIntensity,
    /// Torque allowed - `StepResult::torque_observations` exposes a partial
    /// x1000 limiter surface from the product step path, including
    /// Off/Shutdown and hard-rev zeroing. The row stays adapter-contract until
    /// the FM0016 runtime harness is updated to compare against the same
    /// product-owned torque path.
    TorqueAllowed,
    /// Torque request - `StepResult::torque_observations` exposes a
    /// product-owned x1000 request surface; the row stays adapter-contract
    /// until the FM0016 runtime harness is updated to compare against the same
    /// product-owned torque path.
    TorqueRequest,
    /// Torque actuated - `StepResult::torque_observations` now uses the
    /// current-step product fuel/spark cut outputs instead of action-shape
    /// heuristics, but the row stays adapter-contract until the FM0016 runtime
    /// harness is updated to compare against the same product-owned torque path.
    TorqueActuated,
    /// Idle integrator state - runtime idle integrator not exposed in public API.
    IdleIntegratorState,
    /// Lambda integrator state - runtime lambda integrator not exposed in public API.
    LambdaIntegratorState,
}

/// Extract fuel observations from a runtime StepResult.
/// This reads only from product code (StepResult, ControlPlan, ActionBatch)
/// and does NOT use oracle_result or ObservableOutput.
#[inline]
pub fn extract_fuel_observations(result: &StepResult) -> RuntimeFuelObservations {
    let observations = result.control.fuel_intent.observations;
    RuntimeFuelObservations {
        base_fuel_pw_us: result.control.base_fuel.get(),
        enriched_fuel_pw_us: result.control.enriched_fuel.get(),
        lambda_target_x100: result.control.lambda.target_lambda100.get(),
        startup_active: observations.startup_active,
        startup_window_remaining: observations.startup_window_remaining,
        startup_window_mode: observations.startup_window_mode,
        warmup_active: observations.warmup_active,
        warmup_correction_x100: observations.warmup_correction_x100,
        warmup_temperature_mode: observations.warmup_temperature_mode,
        afterstart_active: observations.afterstart_active,
        afterstart_window_remaining: observations.afterstart_window_remaining,
        afterstart_window_mode: observations.afterstart_window_mode,
        transient_enrichment_active: observations.lambda_ae_freeze_active,
        transient_enrichment_pulse_us: observations.ae_pulse_us,
        transient_enrichment_decay_steps_remaining: observations.ae_decay_steps_remaining,
    }
}

/// Fuel-related observations from a runtime step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeFuelObservations {
    /// Base fuel pulse-width in microseconds.
    pub base_fuel_pw_us: u16,
    /// Enriched fuel pulse-width in microseconds.
    pub enriched_fuel_pw_us: u16,
    /// Lambda target (Lambda100 ratio, e.g., 142 = 1.42 lambda).
    pub lambda_target_x100: u16,
    /// Startup taper is currently active.
    pub startup_active: bool,
    /// Remaining startup window in the unit described by `startup_window_mode`.
    pub startup_window_remaining: u16,
    /// Meaning of the remaining startup window.
    pub startup_window_mode: FuelStartupWindowMode,
    /// Warmup enrichment is currently active.
    pub warmup_active: bool,
    /// Current warmup correction in x100.
    pub warmup_correction_x100: u16,
    /// Current warmup temperature-band classification.
    pub warmup_temperature_mode: FuelWarmupTemperatureMode,
    /// Afterstart taper is currently active.
    pub afterstart_active: bool,
    /// Remaining afterstart window in the unit described by `afterstart_window_mode`.
    pub afterstart_window_remaining: u16,
    /// Meaning of the remaining afterstart window.
    pub afterstart_window_mode: FuelAfterstartWindowMode,
    /// Transient enrichment is currently active.
    pub transient_enrichment_active: bool,
    /// Current transient enrichment pulse width in microseconds.
    pub transient_enrichment_pulse_us: u16,
    /// Remaining transient enrichment decay steps.
    pub transient_enrichment_decay_steps_remaining: u16,
}

/// Strategy-owned fuel-core observations from a runtime step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeFuelCoreObservations {
    /// Whether semantic fuel-core fields are available on the current strategy path.
    pub semantic_available: bool,
    /// Current VE percentage in x100 when available.
    pub ve_pct_x100: Option<u16>,
    /// Current target AFR in x100 when available.
    pub target_afr_x100: Option<u16>,
    /// Strategy-owned base pulse width in microseconds.
    pub pw_base_us: u16,
    /// Strategy-owned air-scaled pulse width in microseconds when available.
    pub pw_air_us: Option<u16>,
    /// Strategy-owned corrected pulse width in microseconds.
    pub pw_corr_us: u16,
}

/// Extract strategy-owned fuel-core observations from a runtime StepResult.
#[inline]
pub fn extract_fuel_core_observations(result: &StepResult) -> RuntimeFuelCoreObservations {
    let observations = result.control.fuel_intent.observations;
    RuntimeFuelCoreObservations {
        semantic_available: !observations.strategy_is_direct_pw,
        ve_pct_x100: observations.ve_pct_x100,
        target_afr_x100: observations.target_afr_x100,
        pw_base_us: observations.pw_base_us,
        pw_air_us: observations.pw_air_us,
        pw_corr_us: observations.pw_corr_us,
    }
}

/// Idle-related observations from a runtime step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeIdleObservations {
    /// Whether the current fuel strategy exposes a meaningful idle-control shell.
    pub active: bool,
    /// Current idle duty command in x1000.
    pub duty_x1000: u16,
    /// Current idle PI integrator accumulator.
    pub integrator_acc: i32,
    /// Minimum permitted idle PI integrator accumulator.
    pub integrator_min_acc: i32,
    /// Maximum permitted idle PI integrator accumulator.
    pub integrator_max_acc: i32,
    /// Whether the idle PI integrator is currently frozen.
    pub integrator_frozen: bool,
}

/// Extract idle observations from a runtime StepResult.
#[inline]
pub fn extract_idle_observations(result: &StepResult) -> RuntimeIdleObservations {
    let observations = result.control.fuel_intent.observations;
    RuntimeIdleObservations {
        active: observations.idle_active,
        duty_x1000: observations.idle_duty_x1000,
        integrator_acc: observations.idle_integrator_acc,
        integrator_min_acc: observations.idle_integrator_min_acc,
        integrator_max_acc: observations.idle_integrator_max_acc,
        integrator_frozen: observations.idle_integrator_frozen,
    }
}

/// Lambda-correction observations from a runtime step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeLambdaCorrectionObservations {
    /// Whether closed-loop lambda is actively correcting on the current step.
    pub active: bool,
    /// Current lambda correction factor in x1000.
    pub correction_x1000: u16,
    /// Whether an integrator state is available on the current strategy path.
    pub integrator_available: bool,
    /// Current lambda PI integrator accumulator.
    pub integrator_acc: i32,
    /// Minimum permitted lambda PI integrator accumulator.
    pub integrator_min_acc: i32,
    /// Maximum permitted lambda PI integrator accumulator.
    pub integrator_max_acc: i32,
    /// Whether the lambda PI integrator is currently frozen.
    pub integrator_frozen: bool,
}

/// Extract lambda-correction observations from a runtime StepResult.
#[inline]
pub fn extract_lambda_correction_observations(
    result: &StepResult,
) -> RuntimeLambdaCorrectionObservations {
    let lambda = result.control.lambda;
    let observations = result.control.fuel_intent.observations;
    RuntimeLambdaCorrectionObservations {
        active: lambda.active,
        correction_x1000: observations.lambda_correction_x1000,
        integrator_available: !observations.strategy_is_direct_pw,
        integrator_acc: observations.lambda_integrator_acc,
        integrator_min_acc: observations.lambda_integrator_min_acc,
        integrator_max_acc: observations.lambda_integrator_max_acc,
        integrator_frozen: observations.lambda_integrator_frozen,
    }
}

/// Ignition-trim observations from a runtime step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeIgnitionTrimObservations {
    /// Whether a non-zero semantic ignition trim is currently active.
    pub active: bool,
    /// Current semantic ignition trim in deg10.
    pub trim_deg10: i16,
}

/// Extract ignition-trim observations from a runtime StepResult.
#[inline]
pub fn extract_ignition_trim_observations(result: &StepResult) -> RuntimeIgnitionTrimObservations {
    let trim_deg10 = result.control.fuel_intent.observations.advance_deg10_trim;
    RuntimeIgnitionTrimObservations {
        active: trim_deg10 != 0,
        trim_deg10,
    }
}

/// Lambda-related observations from a runtime step result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeLambdaObservations {
    /// Current runtime lambda mode.
    pub mode: LambdaMode,
    /// Whether closed-loop lambda is actively correcting.
    pub active: bool,
    /// Target lambda in x100 ratio units.
    pub target_lambda_x100: u16,
    /// Measured lambda in x100 ratio units.
    pub measured_lambda_x100: u16,
    /// Current lambda trim in x100.
    pub trim_x100: i16,
    /// Current disable or freeze reason.
    pub disable_reason: LambdaDisableReason,
}

/// Extract lambda observations from a runtime StepResult.
#[inline]
pub fn extract_lambda_observations(result: &StepResult) -> RuntimeLambdaObservations {
    RuntimeLambdaObservations {
        mode: result.control.lambda.mode,
        active: result.control.lambda.active,
        target_lambda_x100: result.control.lambda.target_lambda100.get(),
        measured_lambda_x100: result.control.lambda.measured_lambda100.get(),
        trim_x100: result.control.lambda.trim_x100,
        disable_reason: result.control.lambda.disable_reason,
    }
}

/// Extract current cut-state observations from the runtime snapshot.
#[inline]
pub fn extract_cut_observations(snapshot: &RuntimeSnapshot) -> RuntimeCutObservations {
    let reason = if snapshot.safety_latched {
        RuntimeCutReason::SafetyLatched
    } else if snapshot.direct_fuel_cut_request || snapshot.direct_spark_cut_request {
        RuntimeCutReason::DirectRequest
    } else if matches!(snapshot.engine.mode, ControlMode::Shutdown) {
        RuntimeCutReason::Shutdown
    } else if snapshot.rev_hard_active {
        RuntimeCutReason::HardRev
    } else if snapshot.launch_active {
        RuntimeCutReason::Launch
    } else if snapshot.flat_shift_active {
        RuntimeCutReason::FlatShift
    } else if snapshot.fuel_cut && !snapshot.spark_cut {
        RuntimeCutReason::FuelOnly
    } else if snapshot.rev_soft_active && snapshot.spark_cut {
        RuntimeCutReason::SoftRev
    } else if snapshot.spark_cut {
        RuntimeCutReason::SparkOnly
    } else if snapshot.legacy_cut_reason_code == 7 {
        RuntimeCutReason::KnockRetard
    } else {
        RuntimeCutReason::None
    };

    RuntimeCutObservations {
        fuel_cut: snapshot.fuel_cut,
        spark_cut: snapshot.spark_cut,
        reason,
    }
}

/// Extract current protection observations from the runtime snapshot.
#[inline]
pub fn extract_protection_observations(
    snapshot: &RuntimeSnapshot,
) -> RuntimeProtectionObservations {
    if snapshot.safety_latched {
        return RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::ShutdownDriving,
            source: RuntimeProtectionSource::SafetyLatch,
            action: RuntimeProtectionAction::OutputSuppressed,
            persistence: RuntimeProtectionPersistence::LatchedUntilClear,
        };
    }

    if snapshot.faults.fault != FaultCode::None {
        let (fault_action, persistence) = runtime_fault_action_policy(
            snapshot.faults.fault,
            snapshot.faults.severity,
            snapshot.faults.cancel_reason,
        );
        let action = runtime_protection_action_from_fault_action(fault_action);

        return RuntimeProtectionObservations {
            level: if action == RuntimeProtectionAction::Shutdown {
                RuntimeProtectionLevel::ShutdownDriving
            } else {
                RuntimeProtectionLevel::Degraded
            },
            source: RuntimeProtectionSource::RuntimeFault,
            action,
            persistence,
        };
    }

    match snapshot.engine.mode {
        ControlMode::Shutdown => RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::ShutdownDriving,
            source: RuntimeProtectionSource::ControlMode,
            action: RuntimeProtectionAction::Shutdown,
            persistence: RuntimeProtectionPersistence::Reversible,
        },
        ControlMode::LimpHome => RuntimeProtectionObservations {
            level: RuntimeProtectionLevel::Degraded,
            source: RuntimeProtectionSource::ControlMode,
            action: RuntimeProtectionAction::LimpHome,
            persistence: RuntimeProtectionPersistence::Reversible,
        },
        _ => RuntimeProtectionObservations::default(),
    }
}

/// Extract current runtime fault observations from the runtime snapshot.
#[inline]
pub fn extract_fault_observations(snapshot: &RuntimeSnapshot) -> RuntimeFaultObservations {
    let active = snapshot.faults.fault != FaultCode::None;
    let (action, persistence) = runtime_fault_action_policy(
        snapshot.faults.fault,
        snapshot.faults.severity,
        snapshot.faults.cancel_reason,
    );

    RuntimeFaultObservations {
        active,
        fault: snapshot.faults.fault,
        severity: snapshot.faults.severity,
        cancel_reason: snapshot.faults.cancel_reason,
        action,
        persistence,
    }
}

/// Extract current authority observations from the runtime snapshot.
#[inline]
pub fn extract_authority_observations(snapshot: &RuntimeSnapshot) -> RuntimeAuthorityObservations {
    RuntimeAuthorityObservations {
        authority: snapshot.engine.engine_time_authority,
        summary: snapshot.engine.sync,
        phase: snapshot.engine.phase,
        full_sequential_authorized: runtime_full_sequential_authorized(
            snapshot.engine.engine_time_authority,
        ),
    }
}

/// Extract current knock observations from the runtime snapshot.
#[inline]
pub fn extract_knock_observations(snapshot: &RuntimeSnapshot) -> RuntimeKnockObservations {
    RuntimeKnockObservations {
        intensity_x100: snapshot.knock_intensity_x100,
        retard_deg10: snapshot.knock_retard_deg10,
    }
}

/// Extract current engine observations from the runtime snapshot.
#[inline]
pub fn extract_engine_observations(snapshot: &RuntimeSnapshot) -> EngineState {
    snapshot.engine
}

/// Extract current calibration observations from the runtime snapshot.
#[inline]
pub fn extract_calibration_observations(snapshot: &RuntimeSnapshot) -> CalibrationState {
    snapshot.calibration
}

/// Extract current calibration package identity from the runtime snapshot.
#[inline]
pub fn extract_calibration_identity_observations(
    snapshot: &RuntimeSnapshot,
) -> RuntimeCalibrationIdentityObservations {
    CalibrationPackageIdentity::from_snapshot_with_staged_dirty(
        snapshot.calibration.active,
        snapshot.calibration.staged_dirty,
    )
}

/// Extract current control observations from the runtime snapshot.
#[inline]
pub fn extract_control_observations(snapshot: &RuntimeSnapshot) -> ControlState {
    snapshot.control
}

/// Extract current scheduler observations from the runtime snapshot.
#[inline]
pub fn extract_scheduler_observations(snapshot: &RuntimeSnapshot) -> SchedulerState {
    snapshot.scheduler
}

/// Extract current output-profile observations from the runtime snapshot.
#[inline]
pub fn extract_output_profile_observations(snapshot: &RuntimeSnapshot) -> RuntimeOutputProfile {
    snapshot.output_profile
}

/// Map the full runtime fuel strategy into a compact product-owned mode shell.
#[inline]
pub(crate) fn runtime_fuel_strategy_mode(
    strategy: &RuntimeFuelStrategy,
) -> RuntimeFuelStrategyMode {
    match strategy {
        RuntimeFuelStrategy::DirectPulseWidthTable(_) => {
            RuntimeFuelStrategyMode::DirectPulseWidthTable
        }
        RuntimeFuelStrategy::SpeedDensityVe { .. } => RuntimeFuelStrategyMode::SpeedDensityVe,
        RuntimeFuelStrategy::AlphaN { .. } => RuntimeFuelStrategyMode::AlphaN,
        RuntimeFuelStrategy::Maf { .. } => RuntimeFuelStrategyMode::Maf,
    }
}

/// Extract current fuel-strategy observations from the runtime snapshot.
#[inline]
pub fn extract_fuel_strategy_observations(snapshot: &RuntimeSnapshot) -> RuntimeFuelStrategyMode {
    snapshot.fuel_strategy_mode
}

/// Extract validated input observations from the runtime step result.
#[inline]
pub fn extract_validated_observations(result: &StepResult) -> ValidatedInputs {
    result.validated
}

/// Extract ignition observations from the runtime step result.
#[inline]
pub fn extract_ignition_observations(result: &StepResult) -> IgnitionPlan {
    result.control.ignition
}

/// Extract runtime action observations from the runtime step result.
#[inline]
pub fn extract_action_observations(result: &StepResult) -> RuntimeActionObservations {
    let mut action_observations = RuntimeActionObservations::default();

    for action in result.actions.iter() {
        action_observations.total_action_count =
            action_observations.total_action_count.saturating_add(1);
        match action {
            Action::ArmScheduler { .. } => {
                action_observations.arm_scheduler_count =
                    action_observations.arm_scheduler_count.saturating_add(1);
            }
            Action::ArmInjection(_) => {
                action_observations.arm_injection_count =
                    action_observations.arm_injection_count.saturating_add(1);
            }
            Action::ArmIgnition(_) => {
                action_observations.arm_ignition_count =
                    action_observations.arm_ignition_count.saturating_add(1);
            }
            Action::ApplyAux(commands) => {
                action_observations.apply_aux_count =
                    action_observations.apply_aux_count.saturating_add(1);
                action_observations.apply_aux_command_count = action_observations
                    .apply_aux_command_count
                    .saturating_add(commands.len().min(u8::MAX as usize) as u8);
            }
            Action::PublishSnapshot => {
                action_observations.publish_snapshot = true;
                action_observations.publish_snapshot_count =
                    action_observations.publish_snapshot_count.saturating_add(1);
            }
            Action::PersistCalibration => {
                action_observations.persist_calibration = true;
                action_observations.persist_calibration_count = action_observations
                    .persist_calibration_count
                    .saturating_add(1);
            }
            Action::CancelScheduler(cancel_reason) => {
                if action_observations.cancel_scheduler
                    && action_observations.cancel_reason != cancel_reason
                {
                    action_observations.multiple_cancel_reasons = true;
                }
                action_observations.cancel_scheduler = true;
                action_observations.cancel_reason = cancel_reason;
                action_observations.cancel_scheduler_count =
                    action_observations.cancel_scheduler_count.saturating_add(1);
            }
            Action::Idle => {
                action_observations.idle_count = action_observations.idle_count.saturating_add(1);
            }
        }
    }

    action_observations
}

fn update_earliest_transition(slot: &mut Option<Micros>, at_us: Micros) {
    match *slot {
        Some(current) if current.get() <= at_us.get() => {}
        _ => *slot = Some(at_us),
    }
}

fn update_latest_transition(slot: &mut Option<Micros>, at_us: Micros) {
    match *slot {
        Some(current) if current.get() >= at_us.get() => {}
        _ => *slot = Some(at_us),
    }
}

/// Extract scheduled-transition observations from the runtime step result.
#[inline]
pub fn extract_transition_observations(result: &StepResult) -> RuntimeTransitionObservations {
    let mut observations = RuntimeTransitionObservations::default();

    for action in result.actions.iter() {
        match action.export_scheduled_transitions::<MODEL_MAX_PENDING>() {
            Ok(batch) => {
                for transition in batch.iter() {
                    observations.total_transition_count =
                        observations.total_transition_count.saturating_add(1);
                    update_earliest_transition(
                        &mut observations.earliest_transition_at_us,
                        transition.at_us,
                    );
                    update_latest_transition(
                        &mut observations.latest_transition_at_us,
                        transition.at_us,
                    );
                    match transition.kind {
                        RuntimeScheduledOutputKind::Injector => {
                            observations.injector_transition_count =
                                observations.injector_transition_count.saturating_add(1);
                            update_earliest_transition(
                                &mut observations.earliest_injector_transition_at_us,
                                transition.at_us,
                            );
                            update_latest_transition(
                                &mut observations.latest_injector_transition_at_us,
                                transition.at_us,
                            );
                        }
                        RuntimeScheduledOutputKind::Ignition => {
                            observations.ignition_transition_count =
                                observations.ignition_transition_count.saturating_add(1);
                            update_earliest_transition(
                                &mut observations.earliest_ignition_transition_at_us,
                                transition.at_us,
                            );
                            update_latest_transition(
                                &mut observations.latest_ignition_transition_at_us,
                                transition.at_us,
                            );
                        }
                    }
                }
            }
            Err(_) => observations.export_error = true,
        }
    }

    observations
}

/// Extract the runtime-owned observed surface used by runtime conformance.
#[inline]
pub fn extract_runtime_observed_surface(
    result: &StepResult,
    snapshot: &RuntimeSnapshot,
) -> RuntimeObservedSurface {
    let engine = extract_engine_observations(snapshot);
    let control = extract_control_observations(snapshot);
    let calibration = extract_calibration_observations(snapshot);
    let calibration_identity = extract_calibration_identity_observations(snapshot);
    let scheduler = extract_scheduler_observations(snapshot);
    let output_profile = extract_output_profile_observations(snapshot);
    let fuel_strategy = extract_fuel_strategy_observations(snapshot);
    let authority = extract_authority_observations(snapshot);
    let validated = extract_validated_observations(result);
    let ignition = extract_ignition_observations(result);
    let actions = extract_action_observations(result);
    let transitions = extract_transition_observations(result);
    let fuel = extract_fuel_observations(result);
    let fuel_core = extract_fuel_core_observations(result);
    let idle = extract_idle_observations(result);
    let lambda = extract_lambda_observations(result);
    let lambda_correction = extract_lambda_correction_observations(result);
    let ignition_trim = extract_ignition_trim_observations(result);
    let cut = extract_cut_observations(snapshot);
    let fault = extract_fault_observations(snapshot);
    let protection = extract_protection_observations(snapshot);
    let knock = extract_knock_observations(snapshot);
    let torque = extract_torque_observations(result);
    RuntimeObservedSurface {
        rpm: result.validated.rpm.get(),
        sync: result.validated.rpm.get() > 0,
        runtime_engine: engine,
        runtime_control: control,
        runtime_calibration: calibration,
        runtime_calibration_identity: calibration_identity,
        runtime_scheduler: scheduler,
        runtime_output_profile: output_profile,
        runtime_fuel_strategy: fuel_strategy,
        runtime_authority: authority,
        runtime_validated: validated,
        fuel_cut: snapshot.fuel_cut,
        spark_cut: snapshot.spark_cut,
        legacy_cut_reason_code: snapshot.legacy_cut_reason_code,
        runtime_cut: cut,
        runtime_fault: fault,
        runtime_protection: protection,
        runtime_knock: knock,
        knock_intensity_x100: snapshot.knock_intensity_x100,
        knock_retard_deg10: snapshot.knock_retard_deg10,
        torque_request_x100: result.control.torque.requested_x100,
        torque_allowed_x100: result.control.torque.allowed_x100,
        torque_actuated_x100: 0,
        torque_request_x1000: torque.request_x1000,
        torque_allowed_x1000: torque.allowed_x1000,
        torque_actuated_x1000: torque.actuated_x1000,
        runtime_torque: torque,
        runtime_base_fuel_pw_us: fuel.base_fuel_pw_us,
        runtime_enriched_fuel_pw_us: fuel.enriched_fuel_pw_us,
        runtime_lambda_target_x100: lambda.target_lambda_x100,
        runtime_lambda: lambda,
        runtime_lambda_correction: lambda_correction,
        runtime_fuel: fuel,
        runtime_fuel_core: fuel_core,
        runtime_idle: idle,
        runtime_ignition_trim: ignition_trim,
        runtime_ignition: ignition,
        runtime_actions: actions,
        runtime_transitions: transitions,
        ignition_advance_deg10: result.control.ignition.advance_deg10.get(),
        dwell_us: result.control.ignition.dwell_us.get(),
        control_mode: result.operating_mode,
        validated_rpm: result.validated.rpm.get(),
        validated_load_kpa10: result.validated.load_kpa10.get(),
        validated_clamped: result.validated.clamped,
    }
}

/// Extract torque observations from a runtime StepResult.
/// This reads only from product code (`StepResult`) and does NOT use
/// oracle_result or test-side scaling.
#[inline]
pub fn extract_torque_observations(result: &StepResult) -> TorqueObservations {
    result.torque_observations
}
