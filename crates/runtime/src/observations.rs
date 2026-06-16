use crate::{ActionBatch, RUNTIME_ACTION_CAP};
use ecu_calibration::CalibrationSnapshot;
use ecu_control::{
    AllowedTorque, EnrichmentInputs, EnrichmentResult, FuelIntent, IgnitionInputs, IgnitionPlan,
    LambdaTrimInputs, LambdaTrimResult, TorqueInputs,
};
use ecu_domain::{
    CancelReason, ControlMode, Degrees10, DwellUs, EnginePhase, EngineTimeAuthority, FaultCode,
    FaultSeverity, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm, SyncState,
};

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

/// Snapshot of runtime-owned state for publication and diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeSnapshot {
    pub engine: EngineState,
    pub control: ControlState,
    pub faults: FaultState,
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
pub struct ControlInputs {
    pub enrichment: EnrichmentInputs,
    pub lambda: LambdaTrimInputs,
    pub torque: TorqueInputs,
    pub ignition: IgnitionInputs,
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
                clt_c: 0,
                lambda_valid: false,
                measured_lambda100: Lambda100::new(100),
                requested_open_loop: true,
            },
            torque: TorqueInputs::new(100, 100, 100, 100, 100),
            ignition,
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

/// Observable surface for runtime conformance.
/// Fields read from StepResult, RuntimeSnapshot, ControlPlan, ActionBatch, and public runtime state only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeObservedSurface {
    pub rpm: u16,
    pub sync: bool,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub legacy_cut_reason_code: u8,
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
    RuntimeFuelObservations {
        base_fuel_pw_us: result.control.base_fuel.get(),
        enriched_fuel_pw_us: result.control.enriched_fuel.get(),
        lambda_target_x100: result.control.lambda.target_lambda100.get(),
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
}

/// Extract torque observations from a runtime StepResult.
/// This reads only from product code (`StepResult`) and does NOT use
/// oracle_result or test-side scaling.
#[inline]
pub fn extract_torque_observations(result: &StepResult) -> TorqueObservations {
    result.torque_observations
}
