#![cfg_attr(not(test), no_std)]

use ecu_board_api::{AuxCommand, AuxCommandBatch, AuxOutput, AuxValue, OutputLevel};
pub use ecu_calibration::CalibrationSnapshot;
pub use ecu_control::{
    AccelerationConfig, AccelerationState, AfterStartConfig, AfterStartState, AllowedTorque,
    BaseFuelModel, DwellConfig, EnrichmentController, EnrichmentInputs, EnrichmentResult,
    FuelBaseCalculator, IgnitionInputs, IgnitionPlan, IgnitionPlanner, LambdaTrimConfig,
    LambdaTrimInputs, LambdaTrimPlanner, LambdaTrimResult, StartupConfig, TorqueArbiter,
    TorqueInputs, WarmupConfig,
};
pub use ecu_scheduler::SchedulerState;

use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, ControlMode, CrankSyncState, CylinderId,
    Degrees10, DwellUs, EnginePhase, EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10,
    Lambda100, Micros, PhaseSyncState, PulseWidthUs, Rpm, SyncState,
};
use ecu_scheduler::{
    ExclusiveChannel, InjectionPlan, OutputGroup, TimedIgnitionPlan, TimedInjectionPlan,
};

const RUNTIME_AUX_COMMAND_CAP: usize = 16;

/// Board-facing action emitted by runtime decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Action {
    ArmScheduler {
        injection: TimedInjectionPlan,
        ignition: TimedIgnitionPlan,
    },
    CancelScheduler(CancelReason),
    PublishSnapshot,
    PersistCalibration,
    ApplyAux(AuxCommandBatch<RUNTIME_AUX_COMMAND_CAP>),
    #[deprecated(note = "compatibility only; prefer ApplyAux")]
    SetFan(bool),
    Idle,
}

/// Fixed-size action batch emitted by a runtime step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActionBatch<const N: usize> {
    actions: [Option<Action>; N],
    len: usize,
}

impl<const N: usize> ActionBatch<N> {
    pub const fn new() -> Self {
        Self {
            actions: [None; N],
            len: 0,
        }
    }

    pub fn push(&mut self, action: Action) -> bool {
        if self.len == N {
            return false;
        }
        self.actions[self.len] = Some(action);
        self.len += 1;
        true
    }

    pub fn len(self) -> usize {
        self.len
    }

    pub fn is_empty(self) -> bool {
        self.len == 0
    }

    pub fn iter(self) -> impl Iterator<Item = Action> {
        self.actions.into_iter().flatten()
    }
}

impl<const N: usize> Default for ActionBatch<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Runtime-owned output profile selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeOutputProfile {
    #[default]
    LegacySingleChannel,
    M50(M50OutputProfile),
}

impl RuntimeOutputProfile {
    pub const fn legacy_single_channel() -> Self {
        Self::LegacySingleChannel
    }

    pub const fn m50_mega_compatible() -> Self {
        Self::M50(M50OutputProfile::mega_compatible())
    }

    pub const fn m50_full_cop() -> Self {
        Self::M50(M50OutputProfile::full_cop())
    }
}

/// M50 ignition topology supported by the runtime-owned profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum M50IgnitionMode {
    WastedSpark3,
    SequentialCop6,
}

/// Minimal M50 runtime profile surface used for action emission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct M50OutputProfile {
    pub firing_order: [CylinderId; 6],
    pub ignition_mode: M50IgnitionMode,
}

impl M50OutputProfile {
    pub const fn mega_compatible() -> Self {
        Self {
            firing_order: [
                CylinderId::new(1),
                CylinderId::new(5),
                CylinderId::new(3),
                CylinderId::new(6),
                CylinderId::new(2),
                CylinderId::new(4),
            ],
            ignition_mode: M50IgnitionMode::WastedSpark3,
        }
    }

    pub const fn full_cop() -> Self {
        Self {
            firing_order: [
                CylinderId::new(1),
                CylinderId::new(5),
                CylinderId::new(3),
                CylinderId::new(6),
                CylinderId::new(2),
                CylinderId::new(4),
            ],
            ignition_mode: M50IgnitionMode::SequentialCop6,
        }
    }

    const fn injector_channel(slot: usize) -> ChannelId {
        ChannelId::new(slot as u8)
    }

    const fn ignition_channel(self, slot: usize) -> ChannelId {
        match self.ignition_mode {
            M50IgnitionMode::WastedSpark3 => ChannelId::new((slot % 3) as u8),
            M50IgnitionMode::SequentialCop6 => ChannelId::new(slot as u8),
        }
    }

    fn cycle_slot_us(self, rpm: Rpm) -> u32 {
        let rpm = u32::from(rpm.get());
        if rpm == 0 {
            return 0;
        }

        let slot_us = 20_000_000u64 / u64::from(rpm);
        slot_us.clamp(1, u32::MAX as u64) as u32
    }
}

fn absolute_authorizes_full_sequential(absolute: AbsoluteTimeAuthority) -> bool {
    matches!(
        absolute,
        AbsoluteTimeAuthority::ExpertManual
            | AbsoluteTimeAuthority::CommunityProfile
            | AbsoluteTimeAuthority::CertifiedProfile
            | AbsoluteTimeAuthority::BenchLearned
    )
}

/// Runtime gate for outputs that require known 720-degree phase.
///
/// Legacy `SyncState::Synced` summaries are intentionally not enough here.
pub fn runtime_full_sequential_authorized(authority: EngineTimeAuthority) -> bool {
    authority.validate().is_ok()
        && authority.has_primary_lock()
        && matches!(authority.phase, PhaseSyncState::CamValidated720)
        && absolute_authorizes_full_sequential(authority.absolute)
}

/// Fast-path runtime observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FastEvent {
    TriggerEdge { at_us: Micros },
    SensorSample { rpm: Rpm, load_kpa10: Kpa10 },
    SyncUpdate { synced: bool },
    ControlTick { at_us: Micros },
}

/// Slow-path runtime observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlowEvent {
    CalibrationCommitted,
    SnapshotRequested,
    PersistRequested,
}

/// Runtime event surface split into fast and slow lanes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event {
    Fast(FastEvent),
    Slow(SlowEvent),
}

impl Event {
    fn key(self) -> EventKey {
        match self {
            Event::Fast(FastEvent::TriggerEdge { .. }) => EventKey::FastTriggerEdge,
            Event::Fast(FastEvent::SensorSample { .. }) => EventKey::FastSensorSample,
            Event::Fast(FastEvent::SyncUpdate { .. }) => EventKey::FastSyncUpdate,
            Event::Fast(FastEvent::ControlTick { .. }) => EventKey::FastControlTick,
            Event::Slow(SlowEvent::CalibrationCommitted) => EventKey::SlowCalibrationCommitted,
            Event::Slow(SlowEvent::SnapshotRequested) => EventKey::SlowSnapshotRequested,
            Event::Slow(SlowEvent::PersistRequested) => EventKey::SlowPersistRequested,
        }
    }

    pub fn is_fast(self) -> bool {
        matches!(self, Event::Fast(_))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EventKey {
    FastTriggerEdge,
    FastSensorSample,
    FastSyncUpdate,
    FastControlTick,
    SlowCalibrationCommitted,
    SlowSnapshotRequested,
    SlowPersistRequested,
}

/// Result of queueing an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueResult {
    Enqueued,
    Coalesced,
    Overflowed,
}

/// Queue overflow classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueOverflow {
    FastFull,
    SlowFull,
}

/// Fixed-size runtime queue split.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RuntimeQueues<const FAST: usize, const SLOW: usize> {
    fast: Queue<Event, FAST>,
    slow: Queue<Event, SLOW>,
}

impl<const FAST: usize, const SLOW: usize> RuntimeQueues<FAST, SLOW> {
    pub const fn new() -> Self {
        Self {
            fast: Queue::new(),
            slow: Queue::new(),
        }
    }

    pub fn push(&mut self, event: Event) -> Result<QueueResult, QueueOverflow> {
        match event {
            Event::Fast(_) => self
                .fast
                .push_coalescing(event)
                .map_err(|_| QueueOverflow::FastFull),
            Event::Slow(_) => self
                .slow
                .push_fifo(event)
                .map_err(|_| QueueOverflow::SlowFull),
        }
    }

    pub fn pop_fast(&mut self) -> Option<Event> {
        self.fast.pop()
    }

    pub fn pop_slow(&mut self) -> Option<Event> {
        self.slow.pop()
    }

    pub fn fast_len(&self) -> usize {
        self.fast.len()
    }

    pub fn slow_len(&self) -> usize {
        self.slow.len()
    }
}

impl<const FAST: usize, const SLOW: usize> Default for RuntimeQueues<FAST, SLOW> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Queue<T: Copy, const N: usize> {
    buf: [Option<T>; N],
    head: usize,
    len: usize,
}

impl<T: Copy, const N: usize> Queue<T, N> {
    const fn new() -> Self {
        Self {
            buf: [None; N],
            head: 0,
            len: 0,
        }
    }

    fn len(&self) -> usize {
        self.len
    }

    fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        let item = self.buf[self.head].take();
        self.head = (self.head + 1) % N.max(1);
        self.len -= 1;
        item
    }

    fn push_fifo(&mut self, item: T) -> Result<QueueResult, ()> {
        if self.len == N {
            return Err(());
        }
        let tail = (self.head + self.len) % N.max(1);
        self.buf[tail] = Some(item);
        self.len += 1;
        Ok(QueueResult::Enqueued)
    }
}

impl<const N: usize> Queue<Event, N> {
    fn push_coalescing(&mut self, item: Event) -> Result<QueueResult, ()> {
        let key = item.key();
        let mut idx = 0usize;
        while idx < self.len {
            let slot = (self.head + idx) % N.max(1);
            if self.buf[slot].map(|existing| existing.key()) == Some(key) {
                self.buf[slot] = Some(item);
                return Ok(QueueResult::Coalesced);
            }
            idx += 1;
        }

        self.push_fifo(item)
    }
}

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
    /// Fuel cut is currently active.
    pub fuel_cut: bool,
    /// Spark cut is currently active.
    pub spark_cut: bool,
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

/// Raw inputs accepted by the runtime step pipeline.
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
}

impl DifferentialInputSnapshot {
    pub fn to_step_inputs(self) -> StepInputs {
        StepInputs {
            now_us: self.now_us,
            rpm: self.rpm.get() as u32,
            load_kpa10: self.load_kpa10.get() as u32,
            angle_x10: self.angle_x10.get() as i32,
            trigger_synced: self.sync == SyncState::Synced,
            cam_seen: self.sync == SyncState::Synced,
            launch_armed: self.launch_armed,
            flat_shift_armed: self.flat_shift_armed,
        }
    }
}

/// Inputs for the composed control planners.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ControlInputs {
    pub enrichment: EnrichmentInputs,
    pub lambda: LambdaTrimInputs,
    pub torque: TorqueInputs,
    pub ignition: IgnitionInputs,
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
}

/// Product-owned torque observations emitted by `EngineRuntime::step`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TorqueObservations {
    /// Torque request in x1000, derived from the product x100 request.
    /// This is emitted by runtime product code, not a conformance helper.
    pub request_x1000: u16,
    /// Torque allowed in x1000, derived from the product x100 limiter output.
    /// The live step path zeroes this for either `EnginePhase::Off` or
    /// `ControlMode::Shutdown`.
    pub allowed_x1000: u16,
    /// Torque actuated in x1000, gated by the product cut state visible on the
    /// step path.
    ///
    /// The runtime still does not own the full safety_latched/fuel_cut/
    /// spark_cut/launch_cut/flat_shift_cut lattice as step inputs, so this is
    /// a partial observation surface rather than a full conformance row.
    pub actuated_x1000: u16,
}

impl TorqueObservations {
    pub fn from_step<const N: usize>(
        torque: AllowedTorque,
        operating_mode: ControlMode,
        engine_phase: EnginePhase,
        actions: ActionBatch<N>,
    ) -> Self {
        let request_x1000 = torque.requested_x100.saturating_mul(10);
        let allowed_x1000 = if matches!(operating_mode, ControlMode::Shutdown)
            || matches!(engine_phase, EnginePhase::Off)
        {
            0
        } else {
            torque.allowed_x100.saturating_mul(10)
        };
        let actuated_x1000 = if torque_cut_gated(actions) {
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
    pub actions: ActionBatch<12>,
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
    /// x1000 limiter surface; Off/Shutdown zeroing is product-owned, but the
    /// row stays grouped with torque until actuated torque owns every cut.
    TorqueAllowed,
    /// Torque request - `StepResult::torque_observations` exposes a partial
    /// product-owned x1000 request surface; the row stays adapter-contract
    /// only because the torque group is not closed until actuated torque owns
    /// the full cut lattice.
    TorqueRequest,
    /// Torque actuated - `StepResult::torque_observations` exposes partial
    /// cut-gated x1000 actuation, but not the full safety/fuel/spark/launch/
    /// flat-shift cut lattice required by the frozen oracle.
    TorqueActuated,
    /// Idle integrator state - runtime idle integrator not exposed in public API.
    IdleIntegratorState,
    /// Lambda integrator state - runtime lambda integrator not exposed in public API.
    LambdaIntegratorState,
}

// ---------------------------------------------------------------------------
// v11 Runtime Semantic Torque Evaluator
// ---------------------------------------------------------------------------

/// Input for the runtime semantic torque evaluator.
///
/// This is semantic-oracle scaffolding for FM0016 comparisons, not a product
/// runtime observation surface. Product torque observations still come from
/// `EngineRuntime::step`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTorqueInput {
    /// Throttle position sensor reading (x100).
    pub tps_x100: u16,
    /// Current engine mode.
    pub mode: RuntimeSemanticEngineMode,
    /// Fuel cut is active.
    pub fuel_cut: bool,
    /// Spark cut is active.
    pub spark_cut: bool,
    /// Safety latch is set.
    pub safety_latched: bool,
    /// Soft rev limiter is active.
    pub rev_soft_active: bool,
    /// Hard rev limiter is active.
    pub rev_hard_active: bool,
    /// Launch cut is active.
    pub launch_cut: bool,
    /// Flat-shift cut is active.
    pub flat_shift_cut: bool,
}

/// Output from the runtime semantic torque evaluator.
///
/// This is a test-only semantic mirror of the frozen torque pipeline. Do not
/// treat these values as product-owned runtime observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTorqueResult {
    /// Torque request (x1000) — from TPS input.
    pub torque_request_x1000: u16,
    /// Torque allowed (x1000) — after limiter ceiling.
    pub torque_allowed_x1000: u16,
    /// Torque actuated (x1000) — after cut gating.
    pub torque_actuated_x1000: u16,
    /// Fuel trim (x1000) — mirrors torque_actuated.
    pub fuel_trim_x1000: u16,
    /// Spark trim (x1000) — mirrors torque_actuated.
    pub spark_trim_x1000: u16,
}

/// Torque scale constant (x1000 max = 1.0).
pub const TORQUE_SCALE_X1000_MAX: u16 = 1000;

fn clamp_x1000(value: u16) -> u16 {
    core::cmp::min(value, TORQUE_SCALE_X1000_MAX)
}

/// Mode-based torque ceiling: Off/Shutdown → 0, Cranking/Running → 1000.
fn torque_mode_ceiling(mode: RuntimeSemanticEngineMode) -> u16 {
    match mode {
        RuntimeSemanticEngineMode::Off | RuntimeSemanticEngineMode::Shutdown => 0,
        RuntimeSemanticEngineMode::Cranking | RuntimeSemanticEngineMode::Running => {
            TORQUE_SCALE_X1000_MAX
        }
    }
}

/// Evaluate torque request/allowed/actuated using frozen oracle semantics.
///
/// This mirrors `spec_oracle::torque::torque_pipeline_step` exactly for
/// conformance scaffolding. The product runtime path remains separate.
#[must_use]
pub fn runtime_semantic_evaluate_torque(
    input: RuntimeSemanticTorqueInput,
) -> RuntimeSemanticTorqueResult {
    // Stage 1: request from TPS.
    // Keep the stage names aligned with the frozen oracle so fixture diffs are
    // easy to compare, but do not read this as a product path.
    let torque_request_x1000 = clamp_x1000(input.tps_x100 / 10);

    // Stage 2: limiter ceiling from mode + hard rev cut.
    let limiter_ceiling_x1000 = torque_mode_ceiling(input.mode);
    let torque_allowed_x1000 = if input.rev_hard_active {
        0 // hard rev cut zeroes allowed
    } else {
        core::cmp::min(torque_request_x1000, limiter_ceiling_x1000)
    };

    // Stage 3+4: cut gating.
    let any_cut = input.safety_latched
        || input.fuel_cut
        || input.spark_cut
        || input.launch_cut
        || input.flat_shift_cut;
    let torque_actuated_x1000 = if any_cut { 0 } else { torque_allowed_x1000 };

    // Stage 5: trim projection.
    let fuel_trim_x1000 = torque_actuated_x1000;
    let spark_trim_x1000 = torque_actuated_x1000;

    RuntimeSemanticTorqueResult {
        torque_request_x1000,
        torque_allowed_x1000,
        torque_actuated_x1000,
        fuel_trim_x1000,
        spark_trim_x1000,
    }
}

// ---------------------------------------------------------------------------
// v9 Runtime Semantic Fuel/Cut Evaluator Types
// ---------------------------------------------------------------------------

/// Fixed table length for v9 semantic types.
pub const RUNTIME_SEMANTIC_TABLE_LEN: usize = 16;

/// An axis of a runtime semantic table or curve with up to 16 entries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticAxis16 {
    /// Number of valid entries (must be in 2..=16 for valid interpolation).
    pub len: u8,
    /// Axis values. Only the first `len` entries are valid.
    pub values: [u16; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// A 2D table of u16 values used for VE and AFR lookups.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTable2dU16 {
    /// RPM axis values.
    pub rpm_axis: RuntimeSemanticAxis16,
    /// Load axis values.
    pub load_axis: RuntimeSemanticAxis16,
    /// Table values indexed by [load_index][rpm_index].
    pub values: [[u16; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// A 1D curve of u16 values used for single-axis corrections.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticCurve16U16 {
    /// Axis values.
    pub axis: RuntimeSemanticAxis16,
    /// Curve values indexed by axis position.
    pub values: [u16; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// AFR override for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticAfrOverride {
    /// No override — use table lookup.
    None,
    /// Override with this fixed AFR × 100 value.
    Some(u16),
}

/// Engine operating mode for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticEngineMode {
    Off,
    Cranking,
    Running,
    Shutdown,
}

/// Full calibration surface required by the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticCalibration {
    pub ve_table: RuntimeSemanticTable2dU16,
    pub afr_target_table: RuntimeSemanticTable2dU16,
    pub deadtime_table_us: RuntimeSemanticTable2dU16,
    pub clt_corr_curve: RuntimeSemanticCurve16U16,
    pub iat_corr_curve: RuntimeSemanticCurve16U16,
    pub baro_corr_curve: RuntimeSemanticCurve16U16,
    pub vbat_corr_curve: RuntimeSemanticCurve16U16,
    pub cranking_curve: RuntimeSemanticCurve16U16,
    pub afterstart_table: RuntimeSemanticTable2dU16,
    pub warmup_curve: RuntimeSemanticCurve16U16,
    pub ae_tps_threshold_curve: RuntimeSemanticCurve16U16,
    pub ae_map_threshold_curve: RuntimeSemanticCurve16U16,
    pub ae_shot_curve_us: RuntimeSemanticCurve16U16,
    pub ae_decay_steps_curve: RuntimeSemanticCurve16U16,
    pub ae_decay_ratio_curve_x1000: RuntimeSemanticCurve16U16,
    pub required_fuel_us: u32,
    pub pref_kpa10: u16,
    pub stoich_afr_x100: u16,
    pub pw_max_us: u32,
    pub afterstart_window_cycles: u16,
    pub dfco_entry_rpm: u16,
    pub dfco_exit_rpm: u16,
    pub dfco_entry_tps_x100: u16,
    pub dfco_exit_tps_x100: u16,
    pub dfco_entry_map_kpa10: u16,
    pub dfco_delay_cycles: u16,
    pub soft_rev_rpm: u16,
    pub hard_rev_rpm: u16,
    pub rev_hysteresis_rpm: u16,
    pub soft_retard_max_deg10: u16,
    pub launch_rpm_limit: u16,
    pub launch_cut_cycles: u16,
    pub flat_shift_rpm_min: u16,
    pub flat_shift_cut_cycles: u16,
    pub knock_threshold_x100: u16,
    pub knock_retard_step_deg10: u16,
    pub knock_retard_max_deg10: u16,
    pub knock_recovery_step_deg10: u16,
    pub knock_recovery_delay_cycles: u16,
    /// Proportional gain for lambda closed-loop correction (x1000).
    pub lambda_kp_x1000: u16,
    /// Integral gain for lambda closed-loop correction (x1000).
    pub lambda_ki_x1000: u16,
}

/// Input snapshot for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticInputSnapshot {
    pub t_us: Micros,
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub load_kpa10: Kpa10,
    pub tps_x100: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub baro_kpa10: Kpa10,
    pub vbatt_mv: u16,
    pub knock_intensity_x100: u16,
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
    pub sync: SyncState,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub mode: RuntimeSemanticEngineMode,
    pub target_afr_override_x100: RuntimeSemanticAfrOverride,
}

// ---------------------------------------------------------------------------
// v11 Runtime Semantic Lambda PI Types
// ---------------------------------------------------------------------------

/// PI integrator state for the semantic lambda closed-loop evaluator.
/// This is the runtime-owned equivalent of `ecu_spec::PiIntegratorState`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticPiIntegratorState {
    /// Accumulator value.
    pub acc: i32,
    /// Minimum accumulator value.
    pub min_acc: i32,
    /// Maximum accumulator value.
    pub max_acc: i32,
    /// True if the integrator is frozen (not accumulating).
    pub frozen: bool,
}

impl Default for RuntimeSemanticPiIntegratorState {
    fn default() -> Self {
        Self {
            acc: 0,
            min_acc: RUNTIME_SEMANTIC_LAMBDA_MIN_ACC,
            max_acc: RUNTIME_SEMANTIC_LAMBDA_MAX_ACC,
            frozen: false,
        }
    }
}

/// Mutable state fragment for the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RuntimeSemanticState {
    pub afterstart_cycle_count: u32,
    pub last_valid_load_kpa10: u16,
    pub last_valid_map_kpa10: u16,
    pub ae_active: bool,
    pub ae_pulse_us: u32,
    pub ae_decay_steps_remaining: u16,
    pub lambda_integrator_acc: i32,
    pub dfco_active: bool,
    pub dfco_qualify_counter: u16,
    pub rev_soft_active: bool,
    pub rev_hard_active: bool,
    pub safety_latched: bool,
    pub sensor_plausibility_latched: bool,
    pub launch_active: bool,
    pub launch_cut_cycle_count: u16,
    pub flat_shift_active: bool,
    pub flat_shift_cut_cycle_count: u16,
    pub knock_retard_deg10: i16,
    pub knock_recovery_counter: u16,
}

/// Observable fuel and cut outputs from the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticFuelObservations {
    pub ve_pct_x100: u16,
    pub target_afr_x100: u16,
    pub pw_base_us: u32,
    pub pw_air_us: u32,
    pub pw_corr_us: u32,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    /// Lambda closed-loop correction factor (x1000, e.g. 1000 = 1.000).
    pub lambda_correction_x1000: u16,
    /// Lambda PI integrator state after this step.
    pub lambda_integrator_state: RuntimeSemanticPiIntegratorState,
    /// Final ignition advance trim in deg10 from runtime semantic cut/knock logic.
    pub advance_deg10_trim: i16,
}

/// Errors from the v9 semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticFuelError {
    AxisLenInvalid,
    AxisNotStrictlyIncreasing,
    ZeroPrefKpa,
    ZeroTargetAfr,
    CorrectionRangeInvalid,
}

// ---------------------------------------------------------------------------
// v10 Runtime Semantic Schedule Types — runtime-owned, no_std, Verus-friendly
// ---------------------------------------------------------------------------

/// A 2D table of i16 values used for spark advance lookups.
/// Indexed by [load_index][rpm_index].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTable2dI16 {
    /// RPM axis values.
    pub rpm_axis: RuntimeSemanticAxis16,
    /// Load axis values.
    pub load_axis: RuntimeSemanticAxis16,
    /// Table values indexed by [load_index][rpm_index].
    pub values: [[i16; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// A 2D table of u32 values used for dwell time lookups.
/// Indexed by [load_index][rpm_index].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticTable2dU32 {
    /// RPM axis values.
    pub rpm_axis: RuntimeSemanticAxis16,
    /// Load axis values.
    pub load_axis: RuntimeSemanticAxis16,
    /// Table values indexed by [load_index][rpm_index].
    pub values: [[u32; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// Cylinder phase array for schedule events.
/// Valid count is 1..=8. Every live phase value must be < 7200.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticCylinderArrayU16 {
    /// Number of active cylinders (1..=8).
    pub count: u8,
    /// Phase values in crank-angle units (0.1 degrees). Only first `count` are valid.
    pub values: [u16; RUNTIME_SEMANTIC_TABLE_LEN],
}

/// Injection angle specification mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticInjectionAngleMode {
    /// Start-of-injection mode.
    StartOfInjection,
    /// End-of-injection mode.
    EndOfInjection,
}

/// Full calibration surface required by the v10 semantic schedule evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleCalibration {
    /// Spark advance table in deg10 units.
    pub spark_advance_table_deg10: RuntimeSemanticTable2dI16,
    /// Dwell time table in microseconds.
    pub dwell_table_us: RuntimeSemanticTable2dU32,
    /// Injection target table in deg10 units.
    pub injection_target_table_deg10: RuntimeSemanticTable2dU16,
    /// Injection angle mode (SOI or EOI).
    pub injection_angle_mode: RuntimeSemanticInjectionAngleMode,
    /// Cylinder phase array in deg10 units.
    pub cylinder_phase_deg10: RuntimeSemanticCylinderArrayU16,
}

/// Diagnostic codes from the schedule semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticScheduleDiagnostic {
    /// No diagnostic active.
    None,
    /// Fuel cut is active.
    FuelCutActive,
    /// Spark cut is active.
    SparkCutActive,
    /// Scheduler is unsynchronized.
    Unsynced,
    /// Calibration data is invalid.
    CalibrationInvalid,
}

/// Kind of schedule event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticScheduleEventKind {
    /// Injector opens.
    InjectionOpen,
    /// Injector closes.
    InjectionClose,
    /// Coil begins charging.
    CoilChargeStart,
    /// Coil fires (spark event).
    CoilFire,
}

/// A single scheduled event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleEvent {
    /// Kind of event.
    pub kind: RuntimeSemanticScheduleEventKind,
    /// Cylinder index (0-based).
    pub cylinder: u8,
    /// Event angle in deg10 units (0..7199).
    pub angle_deg10: u16,
}

/// A batch of scheduled events (up to 64).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleEventBatch {
    /// Number of valid events in the batch.
    pub len: u8,
    /// Event array (up to 64 events).
    pub events: [RuntimeSemanticScheduleEvent; 64],
}

/// Output observations from the schedule semantic evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeSemanticScheduleObservations {
    /// Injection target angle in deg10 units.
    pub injection_target_deg10: u16,
    /// Spark advance in deg10 units (signed).
    pub spark_advance_deg10: i16,
    /// Dwell time in microseconds.
    pub dwell_us: u32,
    /// Injection duration in deg10 units.
    pub injection_duration_deg10: u16,
    /// Dwell duration in deg10 units.
    pub dwell_duration_deg10: u16,
    /// Start-of-injection angles per cylinder in deg10.
    pub soi_deg10: RuntimeSemanticCylinderArrayU16,
    /// End-of-injection angles per cylinder in deg10.
    pub eoi_deg10: RuntimeSemanticCylinderArrayU16,
    /// Spark event angles per cylinder in deg10.
    pub spark_deg10: RuntimeSemanticCylinderArrayU16,
    /// Dwell start angles per cylinder in deg10.
    pub dwell_start_deg10: RuntimeSemanticCylinderArrayU16,
    /// Scheduled event batch.
    pub events: RuntimeSemanticScheduleEventBatch,
    /// Schedule diagnostic.
    pub diagnostic: RuntimeSemanticScheduleDiagnostic,
}

/// Errors from the v10 semantic schedule evaluator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeSemanticScheduleError {
    /// Table axis length is invalid (< 2 for interpolation).
    AxisLenInvalid,
    /// Table axis values are not strictly increasing.
    AxisNotStrictlyIncreasing,
    /// Cylinder count is out of range.
    CylinderCountInvalid,
    /// A cylinder phase value is >= 7200.
    CylinderPhaseInvalid,
    /// Computed duration overflowed the target type.
    DurationOverflow,
    /// Event batch capacity (64) was exceeded.
    EventBatchFull,
}

// ---------------------------------------------------------------------------
// v11 Runtime Semantic Lambda PI Constants
// ---------------------------------------------------------------------------

/// Lambda closed-loop deadband in x1000 units (error whose absolute value is
/// within this is treated as zero).
const RUNTIME_SEMANTIC_LAMBDA_DEADBAND_X1000: i32 = 10;
/// Minimum PI integrator accumulator value.
const RUNTIME_SEMANTIC_LAMBDA_MIN_ACC: i32 = -2000;
/// Maximum PI integrator accumulator value.
const RUNTIME_SEMANTIC_LAMBDA_MAX_ACC: i32 = 2000;
/// Minimum lambda correction factor in x1000 units (0.750).
const RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000: u16 = 750;
/// Maximum lambda correction factor in x1000 units (1.250).
const RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000: u16 = 1250;
/// Fixed lambda error for the v11 frozen oracle path (always zero).
const RUNTIME_SEMANTIC_LAMBDA_ERROR_X1000: i32 = 0;

// ---------------------------------------------------------------------------
// v9 Runtime Semantic Fuel Evaluator — pure, deterministic, no_std
// ---------------------------------------------------------------------------

#[inline]
const fn clamp_u16_s(value: u16, lo: u16, hi: u16) -> u16 {
    if value < lo {
        lo
    } else if value > hi {
        hi
    } else {
        value
    }
}

#[inline]
const fn mul_div_floor_u64(num: u64, mul: u64, div: u64) -> u64 {
    if div == 0 {
        0
    } else {
        (num * mul) / div
    }
}

#[inline]
fn mul_ratio_x1000_floor(value: u32, ratio_x1000: u32) -> u32 {
    ((value as u64) * (ratio_x1000 as u64) / 1000) as u32
}

/// Find the segment index for a clipped value using left-closed/right-open
/// intervals, with the final upper boundary treated as closed.
fn semantic_find_segment(axis: &RuntimeSemanticAxis16, x: u16) -> usize {
    let len = axis.len as usize;
    if len < 2 {
        return 0;
    }
    let clipped = clamp_u16_s(x, axis.values[0], axis.values[len - 1]);
    let mut idx = 0usize;
    while idx + 1 < len {
        let lo = axis.values[idx];
        let hi = axis.values[idx + 1];
        let is_last = idx + 1 == len - 1;
        if clipped >= lo && (clipped < hi || (is_last && clipped == hi)) {
            return idx;
        }
        idx += 1;
    }
    len - 2
}

/// Linear interpolation for u16 (floor semantics).
fn semantic_lerp_u16(x0: u16, x1: u16, y0: u16, y1: u16, x: u16) -> u32 {
    if x1 <= x0 {
        return y0 as u32;
    }
    let x_clip = clamp_u16_s(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as u32
}

/// Bilinear interpolation on a 2D table (floor semantics).
fn semantic_bilerp_u16(table: &RuntimeSemanticTable2dU16, rpm: u16, load: u16) -> u32 {
    let len_rpm = table.rpm_axis.len as usize;
    let len_load = table.load_axis.len as usize;
    if len_rpm < 2 || len_load < 2 {
        return table.values[0][0] as u32;
    }
    let rpm_idx = semantic_find_segment(&table.rpm_axis, rpm);
    let load_idx = semantic_find_segment(&table.load_axis, load);

    let rpm_lo = table.rpm_axis.values[rpm_idx];
    let rpm_hi = table.rpm_axis.values[(rpm_idx + 1).min(len_rpm - 1)];
    let load_lo = table.load_axis.values[load_idx];
    let load_hi = table.load_axis.values[(load_idx + 1).min(len_load - 1)];

    let v00 = table.values[load_idx][rpm_idx];
    let v01 = table.values[(load_idx + 1).min(len_load - 1)][rpm_idx];
    let v10 = table.values[load_idx][(rpm_idx + 1).min(len_rpm - 1)];
    let v11 = table.values[(load_idx + 1).min(len_load - 1)][(rpm_idx + 1).min(len_rpm - 1)];

    let interp_lo = semantic_lerp_u16(rpm_lo, rpm_hi, v00, v10, rpm);
    let interp_hi = semantic_lerp_u16(rpm_lo, rpm_hi, v01, v11, rpm);
    semantic_lerp_u16(load_lo, load_hi, interp_lo as u16, interp_hi as u16, load)
}

/// Curve lookup with clipping and left-closed/right-open semantics.
fn semantic_curve_lookup(curve: &RuntimeSemanticCurve16U16, x: u16) -> u32 {
    let len = curve.axis.len as usize;
    if len < 2 {
        return curve.values[0] as u32;
    }
    let idx = semantic_find_segment(&curve.axis, x);
    let next_idx = (idx + 1).min(len - 1);
    semantic_lerp_u16(
        curve.axis.values[idx],
        curve.axis.values[next_idx],
        curve.values[idx],
        curve.values[next_idx],
        x,
    )
}

/// Validate that an axis is strictly increasing and length is valid.
fn validate_axis(axis: &RuntimeSemanticAxis16) -> Result<(), RuntimeSemanticFuelError> {
    let len = axis.len as usize;
    if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len) {
        return Err(RuntimeSemanticFuelError::AxisLenInvalid);
    }
    let mut idx = 0usize;
    while idx + 1 < len {
        if axis.values[idx] >= axis.values[idx + 1] {
            return Err(RuntimeSemanticFuelError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    Ok(())
}

/// Validate the calibration axes.
fn validate_calibration(cal: &RuntimeSemanticCalibration) -> Result<(), RuntimeSemanticFuelError> {
    validate_axis(&cal.ve_table.rpm_axis)?;
    validate_axis(&cal.ve_table.load_axis)?;
    validate_axis(&cal.afr_target_table.rpm_axis)?;
    validate_axis(&cal.afr_target_table.load_axis)?;
    validate_axis(&cal.deadtime_table_us.rpm_axis)?;
    validate_axis(&cal.deadtime_table_us.load_axis)?;
    validate_axis(&cal.clt_corr_curve.axis)?;
    validate_axis(&cal.iat_corr_curve.axis)?;
    validate_axis(&cal.baro_corr_curve.axis)?;
    validate_axis(&cal.vbat_corr_curve.axis)?;
    validate_axis(&cal.cranking_curve.axis)?;
    validate_axis(&cal.afterstart_table.rpm_axis)?;
    validate_axis(&cal.afterstart_table.load_axis)?;
    validate_axis(&cal.warmup_curve.axis)?;
    validate_axis(&cal.ae_tps_threshold_curve.axis)?;
    validate_axis(&cal.ae_map_threshold_curve.axis)?;
    validate_axis(&cal.ae_shot_curve_us.axis)?;
    validate_axis(&cal.ae_decay_steps_curve.axis)?;
    validate_axis(&cal.ae_decay_ratio_curve_x1000.axis)?;
    Ok(())
}

/// Evaluate AE pulse width and update AE state.
fn evaluate_ae(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    state: &mut RuntimeSemanticState,
) {
    let load_delta = input.load_kpa10.get() as i32 - state.last_valid_load_kpa10 as i32;
    let map_delta = input.map_kpa10.get() as i32 - state.last_valid_map_kpa10 as i32;
    let abs_load_delta = if load_delta < 0 {
        load_delta.saturating_neg() as u32
    } else {
        load_delta as u32
    };
    let abs_map_delta = if map_delta < 0 {
        map_delta.saturating_neg() as u32
    } else {
        map_delta as u32
    };

    let rpm = input.rpm.get();
    let ae_tps_threshold = semantic_curve_lookup(&cal.ae_tps_threshold_curve, rpm);
    let ae_map_threshold =
        semantic_curve_lookup(&cal.ae_map_threshold_curve, input.load_kpa10.get());

    let triggered = abs_load_delta >= ae_tps_threshold || abs_map_delta >= ae_map_threshold;

    if triggered {
        state.ae_pulse_us = semantic_curve_lookup(&cal.ae_shot_curve_us, input.load_kpa10.get());
        state.ae_decay_steps_remaining =
            semantic_curve_lookup(&cal.ae_decay_steps_curve, input.load_kpa10.get()) as u16;
        state.ae_active = true;
    } else if state.ae_decay_steps_remaining > 0 {
        let decay_index = state.ae_decay_steps_remaining - 1;
        let decay_ratio_x1000 = semantic_curve_lookup(&cal.ae_decay_ratio_curve_x1000, decay_index);
        state.ae_pulse_us = ((state.ae_pulse_us as u64) * (decay_ratio_x1000 as u64) / 1000) as u32;
        state.ae_decay_steps_remaining = decay_index;
    } else {
        state.ae_pulse_us = 0;
        state.ae_active = false;
    }
}

/// Hysteresis latch helper.
fn latch_with_hysteresis(current: bool, value: u16, threshold: u16, hysteresis: u16) -> bool {
    if current {
        value > threshold.saturating_sub(hysteresis)
    } else {
        value >= threshold
    }
}

/// Determine final cut flags using the v9 priority arbiter.
fn evaluate_cut_arbitration(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    state: &mut RuntimeSemanticState,
) -> (bool, bool) {
    // Priority 1: Safety latch
    let safety_latch = input.fuel_cut
        || input.spark_cut
        || matches!(input.mode, RuntimeSemanticEngineMode::Shutdown)
        || state.sensor_plausibility_latched;

    // Safety latch self-holding: clears when mode is Off AND cuts are false
    if state.safety_latched {
        if matches!(input.mode, RuntimeSemanticEngineMode::Off)
            && !input.fuel_cut
            && !input.spark_cut
            && !state.sensor_plausibility_latched
        {
            state.safety_latched = false;
        }
    } else if safety_latch {
        state.safety_latched = true;
    }

    if state.safety_latched {
        return (true, true);
    }

    // Priority 2: Hard rev limit
    state.rev_hard_active = latch_with_hysteresis(
        state.rev_hard_active,
        input.rpm.get(),
        cal.hard_rev_rpm,
        cal.rev_hysteresis_rpm,
    );
    if state.rev_hard_active {
        return (true, true);
    }

    // Priority 3: Launch cut
    if input.launch_armed && input.rpm.get() >= cal.launch_rpm_limit {
        if cal.launch_cut_cycles == 0 {
            return (true, true);
        }
        let phase = state.launch_cut_cycle_count % (cal.launch_cut_cycles + 1);
        if phase < cal.launch_cut_cycles {
            state.launch_active = true;
            return (true, true);
        }
    }

    // Priority 4: Flat-shift cut
    if input.flat_shift_armed && input.rpm.get() >= cal.flat_shift_rpm_min {
        if cal.flat_shift_cut_cycles == 0 {
            return (true, true);
        }
        let phase = state.flat_shift_cut_cycle_count % (cal.flat_shift_cut_cycles + 1);
        if phase < cal.flat_shift_cut_cycles {
            state.flat_shift_active = true;
            return (true, true);
        }
    }

    // Priority 5: DFCO (Running mode only)
    if matches!(input.mode, RuntimeSemanticEngineMode::Running) {
        if state.dfco_active {
            state.dfco_active =
                input.rpm.get() > cal.dfco_exit_rpm && input.tps_x100 <= cal.dfco_exit_tps_x100;
        } else if input.rpm.get() >= cal.dfco_entry_rpm
            && input.tps_x100 <= cal.dfco_entry_tps_x100
            && input.map_kpa10.get() <= cal.dfco_entry_map_kpa10
        {
            state.dfco_qualify_counter = state.dfco_qualify_counter.saturating_add(1);
            if state.dfco_qualify_counter >= cal.dfco_delay_cycles {
                state.dfco_active = true;
            }
        } else {
            state.dfco_qualify_counter = 0;
        }
        if state.dfco_active {
            return (true, false);
        }
    }

    // Priority 6: Soft rev spark cut (no fuel cut)
    state.rev_soft_active = latch_with_hysteresis(
        state.rev_soft_active,
        input.rpm.get(),
        cal.soft_rev_rpm,
        cal.rev_hysteresis_rpm,
    );
    if state.rev_soft_active {
        return (false, true);
    }

    // Priority 7: Knock (no cuts in v9)
    let knock_active = input.knock_intensity_x100 >= cal.knock_threshold_x100;
    if knock_active {
        state.knock_recovery_counter = 0;
        if state.knock_retard_deg10 < (cal.knock_retard_max_deg10 as i16) {
            let step = cal.knock_retard_step_deg10 as i16;
            state.knock_retard_deg10 = state.knock_retard_deg10.saturating_add(step);
            if state.knock_retard_deg10 > cal.knock_retard_max_deg10 as i16 {
                state.knock_retard_deg10 = cal.knock_retard_max_deg10 as i16;
            }
        }
    } else if state.knock_retard_deg10 > 0 {
        if state.knock_recovery_counter >= cal.knock_recovery_delay_cycles {
            let step = cal.knock_recovery_step_deg10 as i16;
            state.knock_retard_deg10 = state.knock_retard_deg10.saturating_sub(step);
            if state.knock_retard_deg10 < 0 {
                state.knock_retard_deg10 = 0;
            }
            state.knock_recovery_counter = 0;
        } else {
            state.knock_recovery_counter = state.knock_recovery_counter.saturating_add(1);
        }
    } else {
        state.knock_recovery_counter = 0;
    }

    // Priority 8: No cut
    (false, false)
}

fn semantic_knock_retard_for_trim(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    current_retard_deg10: i16,
    recovery_counter: u16,
) -> i16 {
    let detected = input.knock_intensity_x100 >= cal.knock_threshold_x100;
    if detected {
        let step = cal.knock_retard_step_deg10 as i16;
        let max = cal.knock_retard_max_deg10 as i16;
        let next = current_retard_deg10.saturating_add(step);
        if next > max {
            max
        } else {
            next
        }
    } else if current_retard_deg10 > 0 {
        if recovery_counter >= cal.knock_recovery_delay_cycles {
            let step = cal.knock_recovery_step_deg10 as i16;
            let next = current_retard_deg10.saturating_sub(step);
            if next < 0 {
                0
            } else {
                next
            }
        } else {
            current_retard_deg10
        }
    } else {
        0
    }
}

// --------------------------------------------------------------------------
// v11 Runtime Semantic Lambda PI Helpers
// --------------------------------------------------------------------------

/// Absolute value for i32 without using std.
#[inline]
fn semantic_abs_i32(value: i32) -> i32 {
    if value < 0 {
        value.saturating_neg()
    } else {
        value
    }
}

/// Apply deadband to raw lambda error.
#[inline]
fn semantic_effective_lambda_error(raw: i32) -> i32 {
    if semantic_abs_i32(raw) <= RUNTIME_SEMANTIC_LAMBDA_DEADBAND_X1000 {
        0
    } else {
        raw
    }
}

/// Saturating conversion from i32 to u16.
#[inline]
fn semantic_i32_to_u16_saturating(value: i32) -> u16 {
    if value < 0 {
        0
    } else if value > i32::from(u16::MAX) {
        u16::MAX
    } else {
        value as u16
    }
}

/// Floor-divide with correct negative-product semantics matching spec.
/// quotient = product / denom; if product < 0 and product % denom != 0, subtract 1.
fn semantic_mul_div_floor_i32(numer: i32, factor: i32, denom: i32) -> i32 {
    // Use i64 intermediates to avoid overflow in product
    let product = numer as i64 * factor as i64;
    let denom_i64 = denom as i64;
    if denom_i64 == 0 {
        return 0;
    }
    let quotient = product / denom_i64;
    let remainder = product % denom_i64;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (quotient + correction) as i32
}

/// Evaluate one step of the lambda closed-loop PI controller.
///
/// Returns `(lambda_correction_x1000, RuntimeSemanticPiIntegratorState)`.
///
/// The AE state used here must be the post-evaluate_ae state so that the
/// freeze gate uses the same `ae_active` as the spec oracle.
/// The `fuel_cut` and `spark_cut` parameters must be the post-arbiter values
/// (not raw input cuts) to match the spec oracle's freeze gate inputs.
fn runtime_semantic_lambda_step(
    cal: &RuntimeSemanticCalibration,
    input: &RuntimeSemanticInputSnapshot,
    fuel_cut: bool,
    spark_cut: bool,
    lambda_integrator_acc: i32,
    ae_active_after_eval: bool,
) -> (u16, RuntimeSemanticPiIntegratorState) {
    // Effective error is always 0 in v11 (frozen oracle path uses lambda_error_x1000=0)
    let error = semantic_effective_lambda_error(RUNTIME_SEMANTIC_LAMBDA_ERROR_X1000);

    // P term: floor(error * kp / 1000)
    let p_term = semantic_mul_div_floor_i32(error, cal.lambda_kp_x1000 as i32, 1000);

    // I step: floor(error * ki / 1000)
    let i_step = semantic_mul_div_floor_i32(error, cal.lambda_ki_x1000 as i32, 1000);

    let acc = lambda_integrator_acc;
    let corr_pre = 1000 + p_term + acc;

    // Freeze gate — uses post-arbiter cuts to match spec oracle
    let freeze_gate = input.clt_c10 < 700 || ae_active_after_eval || fuel_cut || spark_cut;

    // Anti-windup freeze
    let anti_windup_freeze = (corr_pre <= i32::from(RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000)
        && i_step < 0)
        || (corr_pre >= i32::from(RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000) && i_step > 0);

    let freeze = freeze_gate || anti_windup_freeze;

    let acc_next = if freeze {
        acc
    } else {
        let sum = acc.saturating_add(i_step);
        // Clamp to [RUNTIME_SEMANTIC_LAMBDA_MIN_ACC, RUNTIME_SEMANTIC_LAMBDA_MAX_ACC]
        sum.clamp(
            RUNTIME_SEMANTIC_LAMBDA_MIN_ACC,
            RUNTIME_SEMANTIC_LAMBDA_MAX_ACC,
        )
    };

    let correction_pre = 1000 + p_term + acc_next;
    let lambda_correction_x1000 = {
        let raw = semantic_i32_to_u16_saturating(correction_pre);
        raw.clamp(
            RUNTIME_SEMANTIC_LAMBDA_CORR_MIN_X1000,
            RUNTIME_SEMANTIC_LAMBDA_CORR_MAX_X1000,
        )
    };

    (
        lambda_correction_x1000,
        RuntimeSemanticPiIntegratorState {
            acc: acc_next,
            min_acc: RUNTIME_SEMANTIC_LAMBDA_MIN_ACC,
            max_acc: RUNTIME_SEMANTIC_LAMBDA_MAX_ACC,
            frozen: freeze,
        },
    )
}

/// The main v9 semantic fuel evaluator.
///
/// Pure, deterministic, no_std/no_alloc, Verus-friendly.
pub fn runtime_semantic_evaluate_fuel(
    cal: &RuntimeSemanticCalibration,
    input: RuntimeSemanticInputSnapshot,
    mut state: RuntimeSemanticState,
) -> Result<RuntimeSemanticFuelObservations, RuntimeSemanticFuelError> {
    // Validate calibration axes
    validate_calibration(cal)?;

    // VE lookup
    let ve_pct_x100 = semantic_bilerp_u16(&cal.ve_table, input.rpm.get(), input.load_kpa10.get());

    // Target AFR
    // Spec: override is returned directly (clamped to [500, 3000]), not via bilerp
    let target_afr_x100 = match input.target_afr_override_x100 {
        RuntimeSemanticAfrOverride::Some(v) => clamp_u16_s(v, 500, 3000) as u32,
        RuntimeSemanticAfrOverride::None => semantic_bilerp_u16(
            &cal.afr_target_table,
            input.rpm.get(),
            input.load_kpa10.get(),
        ),
    };

    if target_afr_x100 == 0 {
        return Err(RuntimeSemanticFuelError::ZeroTargetAfr);
    }

    // Base PW
    let pw_base_us =
        mul_div_floor_u64(cal.required_fuel_us as u64, ve_pct_x100 as u64, 10_000) as u32;

    // Air PW
    if cal.pref_kpa10 == 0 {
        return Err(RuntimeSemanticFuelError::ZeroPrefKpa);
    }
    let pw_air_us = mul_div_floor_u64(
        pw_base_us as u64,
        input.map_kpa10.get() as u64,
        cal.pref_kpa10 as u64,
    ) as u32;

    // Corrections
    let deadtime_us: u32 = semantic_bilerp_u16(
        &cal.deadtime_table_us,
        input.vbatt_mv,
        input.baro_kpa10.get(),
    );

    let clt_corr_x1000 = semantic_curve_lookup(&cal.clt_corr_curve, input.clt_c10.max(0) as u16);
    let iat_corr_x1000 = semantic_curve_lookup(&cal.iat_corr_curve, input.iat_c10.max(0) as u16);
    let baro_corr_x1000 = semantic_curve_lookup(&cal.baro_corr_curve, input.baro_kpa10.get());
    let vbat_corr_x1000 = semantic_curve_lookup(&cal.vbat_corr_curve, input.vbatt_mv);

    let cranking_corr_x1000 = if matches!(input.mode, RuntimeSemanticEngineMode::Cranking) {
        semantic_curve_lookup(&cal.cranking_curve, input.clt_c10.max(0) as u16)
    } else {
        1000
    };

    let afterstart_corr_x1000 = if matches!(input.mode, RuntimeSemanticEngineMode::Running)
        && state.afterstart_cycle_count <= cal.afterstart_window_cycles as u32
    {
        let cycles = state.afterstart_cycle_count.min(u16::MAX as u32) as u16;
        semantic_bilerp_u16(&cal.afterstart_table, cycles, input.clt_c10.max(0) as u16)
    } else {
        1000
    };

    let warmup_corr_x1000 = semantic_curve_lookup(&cal.warmup_curve, input.clt_c10.max(0) as u16);
    let afr_corr_x1000 =
        mul_div_floor_u64(cal.stoich_afr_x100 as u64, 1000, target_afr_x100 as u64) as u32;
    let trim_corr_x1000: u32 = 1000;

    // Evaluate AE first so ae_active state is available for lambda freeze gate.
    // This matches the spec oracle order: ae_step before lambda_step.
    evaluate_ae(cal, &input, &mut state);
    let ae_active_after_eval = state.ae_active;

    let knock_retard_before_arbiter = state.knock_retard_deg10;
    let knock_recovery_counter_before_arbiter = state.knock_recovery_counter;

    // Call cut arbitration ONCE and reuse the result.
    // Calling twice would mutate state between calls and change the second result.
    let (fuel_cut_post_arbiter, spark_cut_post_arbiter) =
        evaluate_cut_arbitration(cal, &input, &mut state);

    // Compute lambda PI correction using the v11 semantic lambda step.
    // The freeze gate uses post-arbiter cuts to match the spec oracle.
    let (lambda_correction_x1000, lambda_integrator_state) = runtime_semantic_lambda_step(
        cal,
        &input,
        fuel_cut_post_arbiter,
        spark_cut_post_arbiter,
        state.lambda_integrator_acc,
        ae_active_after_eval,
    );

    let lambda_corr_x1000 = lambda_correction_x1000 as u32;

    // Apply corrections in order
    let mut pw = pw_air_us;
    pw = mul_ratio_x1000_floor(pw, cranking_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, afterstart_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, warmup_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, clt_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, iat_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, baro_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, vbat_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, afr_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, lambda_corr_x1000);
    pw = mul_ratio_x1000_floor(pw, trim_corr_x1000);

    // Add AE pulse and deadtime (saturating)
    pw = pw.saturating_add(state.ae_pulse_us);
    pw = pw.saturating_add(deadtime_us);

    // Reuse the already-computed cut arbitration result
    let (fuel_cut, spark_cut) = (fuel_cut_post_arbiter, spark_cut_post_arbiter);
    let soft_rev_active_for_trim = latch_with_hysteresis(
        state.rev_soft_active,
        input.rpm.get(),
        cal.soft_rev_rpm,
        cal.rev_hysteresis_rpm,
    );
    let soft_rev_trim_deg10 = if soft_rev_active_for_trim {
        -(cal.soft_retard_max_deg10 as i16)
    } else {
        0
    };
    let knock_retard_for_trim = semantic_knock_retard_for_trim(
        cal,
        &input,
        knock_retard_before_arbiter,
        knock_recovery_counter_before_arbiter,
    );
    let advance_deg10_trim = soft_rev_trim_deg10.saturating_sub(knock_retard_for_trim);

    // If direct input fuel_cut, zero corrected PW
    let pw_corr_us = if fuel_cut || input.fuel_cut {
        0
    } else {
        pw.min(cal.pw_max_us)
    };

    Ok(RuntimeSemanticFuelObservations {
        ve_pct_x100: ve_pct_x100 as u16,
        target_afr_x100: target_afr_x100 as u16,
        pw_base_us,
        pw_air_us,
        pw_corr_us,
        fuel_cut,
        spark_cut,
        lambda_correction_x1000,
        lambda_integrator_state,
        advance_deg10_trim,
    })
}

// --------------------------------------------------------------------------
// v10 Runtime Semantic Schedule Evaluator — pure, deterministic, no_std
// --------------------------------------------------------------------------

fn semantic_lerp_i32(x0: u16, x1: u16, y0: i32, y1: i32, x: u16) -> i32 {
    if x1 <= x0 {
        return y0;
    }
    let x_clip = clamp_u16_s(x, x0, x1);
    let num = (x_clip - x0) as i64;
    let den = (x1 - x0) as i64;
    let delta = y1 as i64 - y0 as i64;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i64 + quotient + correction) as i32
}

fn semantic_lerp_u32(x0: u16, x1: u16, y0: u32, y1: u32, x: u16) -> u32 {
    if x1 <= x0 {
        return y0;
    }
    let x_clip = clamp_u16_s(x, x0, x1);
    let num = (x_clip - x0) as i128;
    let den = (x1 - x0) as i128;
    let delta = y1 as i128 - y0 as i128;
    let product = delta * num;
    let quotient = product / den;
    let remainder = product % den;
    let correction = if product < 0 && remainder != 0 { -1 } else { 0 };
    (y0 as i128 + quotient + correction) as u32
}

/// Bilinear interpolation on a 2D table of i16 (floor semantics).
/// Handles negative values with widened intermediates.
fn semantic_bilerp_i16(table: &RuntimeSemanticTable2dI16, rpm: u16, load: u16) -> i32 {
    let len_rpm = table.rpm_axis.len as usize;
    let len_load = table.load_axis.len as usize;
    if len_rpm < 2 || len_load < 2 {
        return table.values[0][0] as i32;
    }
    let rpm_idx = semantic_find_segment(&table.rpm_axis, rpm);
    let load_idx = semantic_find_segment(&table.load_axis, load);

    let rpm_lo = table.rpm_axis.values[rpm_idx];
    let rpm_hi = table.rpm_axis.values[(rpm_idx + 1).min(len_rpm - 1)];
    let load_lo = table.load_axis.values[load_idx];
    let load_hi = table.load_axis.values[(load_idx + 1).min(len_load - 1)];

    let v00 = table.values[load_idx][rpm_idx] as i32;
    let v01 = table.values[(load_idx + 1).min(len_load - 1)][rpm_idx] as i32;
    let v10 = table.values[load_idx][(rpm_idx + 1).min(len_rpm - 1)] as i32;
    let v11 = table.values[(load_idx + 1).min(len_load - 1)][(rpm_idx + 1).min(len_rpm - 1)] as i32;

    let interp_lo = semantic_lerp_i32(rpm_lo, rpm_hi, v00, v10, rpm);
    let interp_hi = semantic_lerp_i32(rpm_lo, rpm_hi, v01, v11, rpm);
    semantic_lerp_i32(load_lo, load_hi, interp_lo, interp_hi, load)
}

/// Bilinear interpolation on a 2D table of u32 (floor semantics).
/// Matches the structure of semantic_bilerp_u16 but returns u32.
fn semantic_bilerp_u32(table: &RuntimeSemanticTable2dU32, rpm: u16, load: u16) -> u32 {
    let len_rpm = table.rpm_axis.len as usize;
    let len_load = table.load_axis.len as usize;
    if len_rpm < 2 || len_load < 2 {
        return table.values[0][0];
    }
    let rpm_idx = semantic_find_segment(&table.rpm_axis, rpm);
    let load_idx = semantic_find_segment(&table.load_axis, load);

    let rpm_lo = table.rpm_axis.values[rpm_idx];
    let rpm_hi = table.rpm_axis.values[(rpm_idx + 1).min(len_rpm - 1)];
    let load_lo = table.load_axis.values[load_idx];
    let load_hi = table.load_axis.values[(load_idx + 1).min(len_load - 1)];

    let v00 = table.values[load_idx][rpm_idx];
    let v01 = table.values[(load_idx + 1).min(len_load - 1)][rpm_idx];
    let v10 = table.values[load_idx][(rpm_idx + 1).min(len_rpm - 1)];
    let v11 = table.values[(load_idx + 1).min(len_load - 1)][(rpm_idx + 1).min(len_rpm - 1)];

    let interp_lo = semantic_lerp_u32(rpm_lo, rpm_hi, v00, v10, rpm);
    let interp_hi = semantic_lerp_u32(rpm_lo, rpm_hi, v01, v11, rpm);
    semantic_lerp_u32(load_lo, load_hi, interp_lo, interp_hi, load)
}

/// Normalize an i32 angle into [0, 7200) for signed values.
#[inline]
fn norm7200_i32(x: i32) -> u16 {
    // Normalize using mod 7200, then handle negative by adding 7200
    let mut v = x % 7200;
    if v < 0 {
        v += 7200;
    }
    v as u16
}

/// Validate the schedule calibration tables and cylinder phases.
fn validate_schedule_calibration(
    cal: &RuntimeSemanticScheduleCalibration,
) -> Result<(), RuntimeSemanticScheduleError> {
    // Check spark_advance_table_deg10 axes
    {
        let len_rpm = cal.spark_advance_table_deg10.rpm_axis.len as usize;
        let len_load = cal.spark_advance_table_deg10.load_axis.len as usize;
        if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_rpm)
            || !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_load)
        {
            return Err(RuntimeSemanticScheduleError::AxisLenInvalid);
        }
    }
    // Check dwell_table_us axes
    {
        let len_rpm = cal.dwell_table_us.rpm_axis.len as usize;
        let len_load = cal.dwell_table_us.load_axis.len as usize;
        if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_rpm)
            || !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_load)
        {
            return Err(RuntimeSemanticScheduleError::AxisLenInvalid);
        }
    }
    // Check injection_target_table_deg10 axes
    {
        let len_rpm = cal.injection_target_table_deg10.rpm_axis.len as usize;
        let len_load = cal.injection_target_table_deg10.load_axis.len as usize;
        if !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_rpm)
            || !(2..=RUNTIME_SEMANTIC_TABLE_LEN).contains(&len_load)
        {
            return Err(RuntimeSemanticScheduleError::AxisLenInvalid);
        }
    }

    // Validate each axis is strictly increasing
    let mut idx = 0usize;
    let len_rpm = cal.spark_advance_table_deg10.rpm_axis.len as usize;
    while idx + 1 < len_rpm {
        if cal.spark_advance_table_deg10.rpm_axis.values[idx]
            >= cal.spark_advance_table_deg10.rpm_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_load = cal.spark_advance_table_deg10.load_axis.len as usize;
    while idx + 1 < len_load {
        if cal.spark_advance_table_deg10.load_axis.values[idx]
            >= cal.spark_advance_table_deg10.load_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_rpm = cal.dwell_table_us.rpm_axis.len as usize;
    while idx + 1 < len_rpm {
        if cal.dwell_table_us.rpm_axis.values[idx] >= cal.dwell_table_us.rpm_axis.values[idx + 1] {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_load = cal.dwell_table_us.load_axis.len as usize;
    while idx + 1 < len_load {
        if cal.dwell_table_us.load_axis.values[idx] >= cal.dwell_table_us.load_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_rpm = cal.injection_target_table_deg10.rpm_axis.len as usize;
    while idx + 1 < len_rpm {
        if cal.injection_target_table_deg10.rpm_axis.values[idx]
            >= cal.injection_target_table_deg10.rpm_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }
    let mut idx = 0usize;
    let len_load = cal.injection_target_table_deg10.load_axis.len as usize;
    while idx + 1 < len_load {
        if cal.injection_target_table_deg10.load_axis.values[idx]
            >= cal.injection_target_table_deg10.load_axis.values[idx + 1]
        {
            return Err(RuntimeSemanticScheduleError::AxisNotStrictlyIncreasing);
        }
        idx += 1;
    }

    // Cylinder count
    if !(1..=8).contains(&cal.cylinder_phase_deg10.count) {
        return Err(RuntimeSemanticScheduleError::CylinderCountInvalid);
    }

    // Cylinder phase values must be < 7200
    let mut idx = 0usize;
    let count = cal.cylinder_phase_deg10.count as usize;
    while idx < count {
        if cal.cylinder_phase_deg10.values[idx] >= 7200 {
            return Err(RuntimeSemanticScheduleError::CylinderPhaseInvalid);
        }
        idx += 1;
    }

    Ok(())
}

/// The main v10 semantic schedule evaluator.
///
/// Pure, deterministic, no_std/no_alloc, Verus-friendly.
pub fn runtime_semantic_evaluate_schedule(
    cal: &RuntimeSemanticScheduleCalibration,
    input: RuntimeSemanticInputSnapshot,
    fuel: RuntimeSemanticFuelObservations,
) -> Result<RuntimeSemanticScheduleObservations, RuntimeSemanticScheduleError> {
    runtime_semantic_evaluate_schedule_with_authority(cal, input, fuel, EngineTimeAuthority::none())
}

/// Semantic schedule evaluator variant for full sequential authority checks.
pub fn runtime_semantic_evaluate_schedule_with_authority(
    cal: &RuntimeSemanticScheduleCalibration,
    input: RuntimeSemanticInputSnapshot,
    fuel: RuntimeSemanticFuelObservations,
    authority: EngineTimeAuthority,
) -> Result<RuntimeSemanticScheduleObservations, RuntimeSemanticScheduleError> {
    // Validate calibration
    validate_schedule_calibration(cal)?;

    let rpm = input.rpm.get();
    let load_kpa10 = input.load_kpa10.get();

    // Duration conversions
    let injection_duration_deg10 = {
        let raw = (fuel.pw_corr_us as u64 * rpm as u64 * 6u64) / 100_000u64;
        if raw > u16::MAX as u64 {
            return Err(RuntimeSemanticScheduleError::DurationOverflow);
        }
        raw as u16
    };

    let dwell_us = semantic_bilerp_u32(&cal.dwell_table_us, rpm, load_kpa10);
    let dwell_duration_deg10 = {
        let raw = (dwell_us as u64 * rpm as u64 * 6u64) / 100_000u64;
        if raw > u16::MAX as u64 {
            return Err(RuntimeSemanticScheduleError::DurationOverflow);
        }
        raw as u16
    };

    // Table lookups
    let base_spark_advance_deg10 =
        semantic_bilerp_i16(&cal.spark_advance_table_deg10, rpm, load_kpa10);
    let spark_advance_deg10 = (base_spark_advance_deg10 + fuel.advance_deg10_trim as i32)
        .clamp(i16::MIN as i32, i16::MAX as i32);
    let injection_target_deg10 =
        semantic_bilerp_u16(&cal.injection_target_table_deg10, rpm, load_kpa10);

    // Per-cylinder angle law
    let mut soi_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    let mut eoi_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    let mut spark_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
    let mut dwell_start_deg10_values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];

    let count = cal.cylinder_phase_deg10.count as usize;
    let mut cyl = 0usize;
    while cyl < count {
        let phase = cal.cylinder_phase_deg10.values[cyl];

        let (soi, eoi) = match cal.injection_angle_mode {
            RuntimeSemanticInjectionAngleMode::EndOfInjection => {
                // Use i32 arithmetic then norm7200 to match spec behavior.
                // phase and injection_target are u16 but subtraction may underflow
                // (e.g., phase=0, inj_target=360: 0-360 = -360 on i32, norm7200 => 6840).
                let phase_i = phase as i32;
                let inj_tgt_i = injection_target_deg10 as i32;
                let inj_dur_i = injection_duration_deg10 as i32;
                let eoi_i = phase_i - inj_tgt_i;
                let eoi = norm7200_i32(eoi_i);
                let soi_i = eoi_i - inj_dur_i;
                let soi = norm7200_i32(soi_i);
                (soi, eoi)
            }
            RuntimeSemanticInjectionAngleMode::StartOfInjection => {
                let phase_i = phase as i32;
                let inj_tgt_i = injection_target_deg10 as i32;
                let inj_dur_i = injection_duration_deg10 as i32;
                let soi_i = phase_i - inj_tgt_i;
                let soi = norm7200_i32(soi_i);
                let eoi_i = soi_i + inj_dur_i;
                let eoi = norm7200_i32(eoi_i);
                (soi, eoi)
            }
        };

        let phase_i: i32 = phase as i32;
        let spark = norm7200_i32(phase_i - spark_advance_deg10);
        let dwell_start = norm7200_i32(phase_i - spark_advance_deg10 - dwell_duration_deg10 as i32);

        soi_deg10_values[cyl] = soi;
        eoi_deg10_values[cyl] = eoi;
        spark_deg10_values[cyl] = spark;
        dwell_start_deg10_values[cyl] = dwell_start;

        cyl += 1;
    }

    // Event emission
    let engine_enabled = matches!(
        input.mode,
        RuntimeSemanticEngineMode::Cranking | RuntimeSemanticEngineMode::Running
    );
    let sync_enabled =
        input.sync == SyncState::Synced && runtime_full_sequential_authorized(authority);

    let mut events = [RuntimeSemanticScheduleEvent {
        kind: RuntimeSemanticScheduleEventKind::InjectionOpen,
        cylinder: 0,
        angle_deg10: 0,
    }; 64];
    let mut event_count: usize = 0;

    let enabled = engine_enabled && sync_enabled;
    let fuel_events_enabled = enabled && !fuel.fuel_cut && fuel.pw_corr_us > 0;
    let spark_events_enabled = enabled && !fuel.spark_cut;

    let mut cyl = 0usize;
    while cyl < count {
        if event_count + 4 > 64 {
            return Err(RuntimeSemanticScheduleError::EventBatchFull);
        }
        if fuel_events_enabled {
            // InjectionOpen
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::InjectionOpen,
                cylinder: cyl as u8,
                angle_deg10: soi_deg10_values[cyl],
            };
            event_count += 1;
            // InjectionClose
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::InjectionClose,
                cylinder: cyl as u8,
                angle_deg10: eoi_deg10_values[cyl],
            };
            event_count += 1;
        }
        if spark_events_enabled {
            // CoilChargeStart (dwell begin)
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::CoilChargeStart,
                cylinder: cyl as u8,
                angle_deg10: dwell_start_deg10_values[cyl],
            };
            event_count += 1;
            // CoilFire (spark)
            events[event_count] = RuntimeSemanticScheduleEvent {
                kind: RuntimeSemanticScheduleEventKind::CoilFire,
                cylinder: cyl as u8,
                angle_deg10: spark_deg10_values[cyl],
            };
            event_count += 1;
        }
        cyl += 1;
    }

    let diagnostic = if fuel.fuel_cut {
        RuntimeSemanticScheduleDiagnostic::FuelCutActive
    } else if fuel.spark_cut {
        RuntimeSemanticScheduleDiagnostic::SparkCutActive
    } else if engine_enabled && !sync_enabled {
        RuntimeSemanticScheduleDiagnostic::Unsynced
    } else {
        RuntimeSemanticScheduleDiagnostic::None
    };

    Ok(RuntimeSemanticScheduleObservations {
        injection_target_deg10: injection_target_deg10 as u16,
        spark_advance_deg10: spark_advance_deg10 as i16,
        dwell_us,
        injection_duration_deg10,
        dwell_duration_deg10,
        soi_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: soi_deg10_values,
        },
        eoi_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: eoi_deg10_values,
        },
        spark_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: spark_deg10_values,
        },
        dwell_start_deg10: RuntimeSemanticCylinderArrayU16 {
            count: cal.cylinder_phase_deg10.count,
            values: dwell_start_deg10_values,
        },
        events: RuntimeSemanticScheduleEventBatch {
            len: event_count as u8,
            events,
        },
        diagnostic,
    })
}

/// Field-by-field conformance status for runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeConformanceStatus {
    Covered,
    AdapterContract(RuntimeAdapterContract),
}

// ---------------------------------------------------------------------------
// Runtime fuel extraction helpers
// ---------------------------------------------------------------------------

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

#[allow(deprecated)]
fn torque_cut_gated<const N: usize>(actions: ActionBatch<N>) -> bool {
    for action in actions.iter() {
        match action {
            Action::CancelScheduler(_) => return true,
            Action::ArmScheduler {
                injection,
                ignition,
            } => {
                if injection.plan.pulse_width.get() == 0 || ignition.plan.dwell.get() == 0 {
                    return true;
                }
            }
            Action::PublishSnapshot
            | Action::PersistCalibration
            | Action::ApplyAux(_)
            | Action::SetFan(_)
            | Action::Idle => {}
        }
    }
    false
}

// ---------------------------------------------------------------------------
// Unit conversion helpers
// ---------------------------------------------------------------------------

/// Converts runtime torque percent-x100 to spec torque percent-x1000.
///
/// The spec uses x1000 units (e.g., 10000 = 100.00%) and runtime uses
/// x100 units (e.g., 10000 = 100.00%). This helper applies x10 scaling
/// only when the two values represent the same percent-style quantity.
///
/// # Saturation
///
/// Uses `saturating_mul(10)` so that `u16::MAX * 10` does not wrap.
/// The result saturates at `u16::MAX` (65535).
///
/// # When NOT to use
///
/// Do NOT use this for torque_allowed unless runtime arbiter semantics
/// have been confirmed as the same limiter stack as the spec oracle.
#[inline]
pub const fn runtime_x100_to_spec_x1000(runtime_x100: u16) -> u16 {
    runtime_x100.saturating_mul(10)
}

fn engine_phase_from_authority(authority: EngineTimeAuthority, rpm: Rpm) -> EnginePhase {
    if rpm.get() == 0 {
        EnginePhase::Off
    } else if authority.has_primary_lock() && !matches!(authority.phase, PhaseSyncState::Unknown) {
        EnginePhase::Running
    } else {
        EnginePhase::Cranking
    }
}

fn derive_engine_time_authority(
    current: EngineTimeAuthority,
    trigger_synced: bool,
    cam_seen: bool,
    rpm: Rpm,
) -> EngineTimeAuthority {
    if !trigger_synced {
        let crank = if current.has_primary_lock() {
            CrankSyncState::SyncLost
        } else if rpm.get() > 0 {
            CrankSyncState::PrimarySearching
        } else {
            CrankSyncState::NoSignal
        };
        let sync_loss_count = if current.has_primary_lock() {
            current.sync_loss_count.saturating_add(1)
        } else {
            current.sync_loss_count
        };

        return EngineTimeAuthority::new(
            crank,
            PhaseSyncState::Unknown,
            AbsoluteTimeAuthority::None,
            0,
            sync_loss_count,
        );
    }

    let phase = if cam_seen {
        if matches!(current.phase, PhaseSyncState::CamValidated720) {
            PhaseSyncState::CamValidated720
        } else {
            PhaseSyncState::CamObserved720
        }
    } else {
        PhaseSyncState::CrankOnly360
    };
    let absolute = if current.has_absolute_timing() {
        current.absolute
    } else {
        AbsoluteTimeAuthority::GeometryOnly
    };
    let confidence_x1000 = if current.confidence_x1000 == 0 {
        EngineTimeAuthority::MAX_CONFIDENCE_X1000
    } else {
        current
            .confidence_x1000
            .min(EngineTimeAuthority::MAX_CONFIDENCE_X1000)
    };

    EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        phase,
        absolute,
        confidence_x1000,
        current.sync_loss_count,
    )
}

fn output_profile_requires_full_sequential_authority(profile: RuntimeOutputProfile) -> bool {
    // M50 runtime actions always pair ignition with per-cylinder sequential fuel.
    matches!(profile, RuntimeOutputProfile::M50(_))
}

/// First-pass runtime owner that groups all state shells in one place.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct EngineRuntime {
    engine: EngineState,
    control: ControlState,
    faults: FaultState,
    calibration: CalibrationState,
    scheduler: SchedulerState,
    planners: ControlPlannerState,
    runtime_snapshot: RuntimeSnapshot,
    calibration_snapshot: CalibrationSnapshot,
    output_profile: RuntimeOutputProfile,
    /// Fuel cut active flag from last step.
    fuel_cut: bool,
    /// Spark cut active flag from last step.
    spark_cut: bool,
}

/// Owned control planner state kept by the runtime.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlPlannerState {
    pub fuel_model: BaseFuelModel,
    pub enrichment: EnrichmentController,
    pub lambda: LambdaTrimPlanner,
    pub torque: TorqueArbiter,
    pub ignition: IgnitionPlanner,
}

impl Default for ControlPlannerState {
    fn default() -> Self {
        Self {
            fuel_model: BaseFuelModel::default(),
            enrichment: EnrichmentController::new(),
            lambda: LambdaTrimPlanner::new(),
            torque: TorqueArbiter::new(),
            ignition: IgnitionPlanner::new(),
        }
    }
}

impl EngineRuntime {
    pub fn new() -> Self {
        Self {
            engine: EngineState {
                sync: SyncState::Unsynced,
                engine_time_authority: EngineTimeAuthority::none(),
                phase: EnginePhase::Off,
                mode: ControlMode::OpenLoop,
                rpm: Rpm::new(0),
                load_kpa10: Kpa10::new(0),
                angle_x10: Degrees10::new(0),
            },
            control: ControlState {
                fuel_pulse_width: PulseWidthUs::new(0),
                ignition_advance: Degrees10::new(0),
                dwell: DwellUs::new(0),
                lambda_target: Lambda100::new(100),
                torque_limit_x100: 100,
            },
            faults: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            calibration: CalibrationState {
                active: CalibrationSnapshot::default(),
                staged_dirty: false,
            },
            scheduler: SchedulerState::new(),
            planners: ControlPlannerState::default(),
            runtime_snapshot: RuntimeSnapshot {
                engine: EngineState {
                    sync: SyncState::Unsynced,
                    engine_time_authority: EngineTimeAuthority::none(),
                    phase: EnginePhase::Off,
                    mode: ControlMode::OpenLoop,
                    rpm: Rpm::new(0),
                    load_kpa10: Kpa10::new(0),
                    angle_x10: Degrees10::new(0),
                },
                control: ControlState {
                    fuel_pulse_width: PulseWidthUs::new(0),
                    ignition_advance: Degrees10::new(0),
                    dwell: DwellUs::new(0),
                    lambda_target: Lambda100::new(100),
                    torque_limit_x100: 100,
                },
                faults: FaultState {
                    fault: FaultCode::None,
                    severity: FaultSeverity::Info,
                    cancel_reason: CancelReason::Manual,
                },
                rev_soft_active: false,
                rev_hard_active: false,
                launch_active: false,
                flat_shift_active: false,
                fuel_cut: false,
                spark_cut: false,
            },
            calibration_snapshot: CalibrationSnapshot::default(),
            output_profile: RuntimeOutputProfile::default(),
            fuel_cut: false,
            spark_cut: false,
        }
    }

    pub fn snapshot(&self) -> RuntimeSnapshot {
        self.runtime_snapshot
    }

    pub fn calibration_snapshot(&self) -> CalibrationSnapshot {
        self.calibration_snapshot
    }

    pub fn scheduler_state(&self) -> SchedulerState {
        self.scheduler
    }

    pub fn configure_fuel_model(&mut self, fuel_model: BaseFuelModel) {
        self.planners.fuel_model = fuel_model;
    }

    pub fn configure_output_profile(&mut self, profile: RuntimeOutputProfile) {
        self.output_profile = profile;
    }

    pub fn configure_m50_mega_compatible(&mut self) {
        self.configure_output_profile(RuntimeOutputProfile::m50_mega_compatible());
    }

    pub fn configure_m50_full_cop(&mut self) {
        self.configure_output_profile(RuntimeOutputProfile::m50_full_cop());
    }

    pub fn output_profile(&self) -> RuntimeOutputProfile {
        self.output_profile
    }

    pub fn engine_time_authority(&self) -> EngineTimeAuthority {
        self.engine.engine_time_authority
    }

    pub fn set_engine_time_authority(&mut self, authority: EngineTimeAuthority) {
        self.set_engine_time_authority_inner(authority);
        self.refresh_snapshot();
    }

    fn set_engine_time_authority_inner(&mut self, authority: EngineTimeAuthority) {
        let authority = if authority.validate().is_ok() {
            authority
        } else {
            EngineTimeAuthority::none()
        };
        self.engine.engine_time_authority = authority;
        self.engine.sync = authority.compatibility_summary();
        self.engine.phase = engine_phase_from_authority(authority, self.engine.rpm);
    }

    pub fn set_fault_state(
        &mut self,
        fault: FaultCode,
        severity: FaultSeverity,
        cancel_reason: CancelReason,
    ) {
        self.faults = FaultState {
            fault,
            severity,
            cancel_reason,
        };
        self.refresh_snapshot();
    }

    pub fn set_staged_dirty(&mut self, dirty: bool) {
        self.calibration.staged_dirty = dirty;
        self.refresh_snapshot();
    }

    pub fn apply_sensor_sample(&mut self, rpm: Rpm, load_kpa10: Kpa10, angle_x10: Degrees10) {
        self.engine.rpm = rpm;
        self.engine.load_kpa10 = load_kpa10;
        self.engine.angle_x10 = angle_x10;
        self.refresh_snapshot();
    }

    fn compose_control(
        &mut self,
        validated: &ValidatedInputs,
        inputs: ControlInputs,
    ) -> ControlPlan {
        let base_fuel = self
            .planners
            .fuel_model
            .calculate_base_fuel(validated.rpm, validated.load_kpa10);
        let enrichment = self.planners.enrichment.update(
            inputs.enrichment,
            &StartupConfig::default(),
            &WarmupConfig::default(),
            &AfterStartConfig::default(),
            &AccelerationConfig::default(),
        );
        let lambda = self
            .planners
            .lambda
            .update(inputs.lambda, &LambdaTrimConfig::default());
        let torque = self.planners.torque.evaluate(inputs.torque);
        let ignition = self
            .planners
            .ignition
            .plan(inputs.ignition, &DwellConfig::default());
        let enriched_fuel = enrichment.apply_to(base_fuel);

        self.control = ControlState {
            fuel_pulse_width: enriched_fuel,
            ignition_advance: ignition.advance_deg10,
            dwell: ignition.dwell_us,
            lambda_target: lambda.target_lambda100,
            torque_limit_x100: torque.allowed_x100,
        };

        ControlPlan {
            base_fuel,
            enriched_fuel,
            enrichment,
            lambda,
            torque,
            ignition,
            fuel_cut: false,
            spark_cut: false,
        }
    }

    fn validate_inputs(&self, inputs: StepInputs) -> ValidatedInputs {
        const MAX_RPM: u32 = 9000;
        const MAX_LOAD: u32 = 2000;
        const MAX_ANGLE_X10: i32 = 7200;

        let clamped_rpm = inputs.rpm.min(MAX_RPM) as u16;
        let clamped_load = inputs.load_kpa10.min(MAX_LOAD) as u16;
        let clamped_angle = inputs.angle_x10.clamp(-MAX_ANGLE_X10, MAX_ANGLE_X10) as i16;

        ValidatedInputs {
            rpm: Rpm::new(clamped_rpm),
            load_kpa10: Kpa10::new(clamped_load),
            angle_x10: Degrees10::new(clamped_angle),
            clamped: clamped_rpm as u32 != inputs.rpm
                || clamped_load as u32 != inputs.load_kpa10
                || clamped_angle as i32 != inputs.angle_x10,
        }
    }

    fn derive_operating_mode(&self) -> ControlMode {
        if self.faults.severity == FaultSeverity::Critical
            || self.faults.fault == FaultCode::SafetyCut
        {
            ControlMode::Shutdown
        } else if self.faults.severity == FaultSeverity::Warning
            || self.faults.fault == FaultCode::SensorOutOfRange
        {
            ControlMode::LimpHome
        } else if self.engine.sync == SyncState::Unsynced {
            ControlMode::OpenLoop
        } else if self.engine.phase == EnginePhase::Running {
            ControlMode::ClosedLoop
        } else {
            ControlMode::OpenLoop
        }
    }

    pub fn step(&mut self, inputs: StepInputs, control_inputs: ControlInputs) -> StepResult {
        let authority = self.engine.engine_time_authority;
        self.step_with_authority(inputs, control_inputs, authority)
    }

    /// Step the runtime with structured engine-time authority already supplied by the caller.
    ///
    /// Board adapters should use this when they have a decoder/profile authority snapshot so
    /// output gating does not fall back to boolean sync/cam inputs.
    pub fn step_with_authority(
        &mut self,
        inputs: StepInputs,
        control_inputs: ControlInputs,
        authority: EngineTimeAuthority,
    ) -> StepResult {
        let validated = self.validate_inputs(inputs);
        let authority = derive_engine_time_authority(
            authority,
            inputs.trigger_synced,
            inputs.cam_seen,
            validated.rpm,
        );
        self.step_with_validated(inputs.now_us, validated, control_inputs, authority)
    }

    fn step_with_validated(
        &mut self,
        now_us: Micros,
        validated: ValidatedInputs,
        control_inputs: ControlInputs,
        authority: EngineTimeAuthority,
    ) -> StepResult {
        self.engine.rpm = validated.rpm;
        self.engine.angle_x10 = validated.angle_x10;
        self.engine.load_kpa10 = validated.load_kpa10;
        self.set_engine_time_authority_inner(authority);
        self.engine.mode = self.derive_operating_mode();
        let control = self.compose_control(&validated, control_inputs);
        let actions = self.emit_actions(now_us, &control);
        let torque_observations = TorqueObservations::from_step(
            control.torque,
            self.engine.mode,
            self.engine.phase,
            actions,
        );
        self.refresh_snapshot();

        StepResult {
            validated,
            operating_mode: self.engine.mode,
            control,
            actions,
            torque_observations,
        }
    }

    fn emit_actions(&mut self, now_us: Micros, control: &ControlPlan) -> ActionBatch<12> {
        let mut actions = ActionBatch::new();

        if self.engine.mode == ControlMode::Shutdown
            || self.faults.fault == FaultCode::SafetyCut
            || self.faults.severity == FaultSeverity::Critical
        {
            self.scheduler.on_hard_safety_shutdown();
            let _ = actions.push(Action::CancelScheduler(CancelReason::SafetyShutdown));
        } else if self.engine.sync == SyncState::Unsynced && self.scheduler.is_armed() {
            self.scheduler.on_sync_loss();
            let _ = actions.push(Action::CancelScheduler(CancelReason::SyncLoss));
        } else if output_profile_requires_full_sequential_authority(self.output_profile)
            && !runtime_full_sequential_authorized(self.engine.engine_time_authority)
        {
            if self.scheduler.is_armed() {
                self.scheduler.on_sync_loss();
                let _ = actions.push(Action::CancelScheduler(CancelReason::SyncLoss));
            } else {
                let _ = actions.push(Action::Idle);
            }
        } else if self.engine.sync == SyncState::Synced {
            match self.output_profile {
                RuntimeOutputProfile::LegacySingleChannel => {
                    let inj = TimedInjectionPlan {
                        plan: InjectionPlan {
                            output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
                            pulse_width: control.enriched_fuel,
                        },
                        start_at: now_us,
                        end_at: Micros::new(
                            now_us
                                .get()
                                .saturating_add(control.enriched_fuel.get() as u32),
                        ),
                    };
                    let ign = TimedIgnitionPlan {
                        plan: self.make_ignition_plan(control, now_us),
                        start_at: now_us,
                        end_at: Micros::new(
                            now_us
                                .get()
                                .saturating_add(control.ignition.dwell_us.get() as u32),
                        ),
                    };
                    self.scheduler.arm_group(OutputGroup::Injector);
                    self.scheduler.arm_group(OutputGroup::Ignition);
                    let _ = actions.push(Action::ArmScheduler {
                        injection: inj,
                        ignition: ign,
                    });
                }
                RuntimeOutputProfile::M50(profile) => {
                    self.scheduler.arm_group(OutputGroup::Injector);
                    self.scheduler.arm_group(OutputGroup::Ignition);
                    let cycle_slot_us = profile.cycle_slot_us(self.engine.rpm);
                    for (slot, _) in profile.firing_order.iter().enumerate() {
                        let slot_offset = cycle_slot_us.saturating_mul(slot as u32);
                        let injection_start = Micros::new(now_us.get().saturating_add(slot_offset));
                        let injection_end = Micros::new(
                            injection_start
                                .get()
                                .saturating_add(control.enriched_fuel.get() as u32)
                                .max(injection_start.get().saturating_add(1)),
                        );
                        let ignition_start = Micros::new(now_us.get().saturating_add(slot_offset));
                        let ignition_end = Micros::new(
                            ignition_start
                                .get()
                                .saturating_add(control.ignition.dwell_us.get() as u32)
                                .max(ignition_start.get().saturating_add(1)),
                        );
                        let _ = actions.push(Action::ArmScheduler {
                            injection: TimedInjectionPlan {
                                plan: InjectionPlan {
                                    output: ExclusiveChannel::new(
                                        OutputGroup::Injector,
                                        M50OutputProfile::injector_channel(slot),
                                    ),
                                    pulse_width: control.enriched_fuel,
                                },
                                start_at: injection_start,
                                end_at: injection_end,
                            },
                            ignition: TimedIgnitionPlan {
                                plan: ecu_scheduler::IgnitionPlan {
                                    output: ExclusiveChannel::new(
                                        OutputGroup::Ignition,
                                        profile.ignition_channel(slot),
                                    ),
                                    dwell: control.ignition.dwell_us,
                                    advance: control.ignition.advance_deg10,
                                },
                                start_at: ignition_start,
                                end_at: ignition_end,
                            },
                        });
                    }
                }
            }
        } else {
            let _ = actions.push(Action::Idle);
        }

        if self.engine.mode == ControlMode::LimpHome {
            let _ = actions.push(Action::ApplyAux(self.limp_home_aux_commands()));
        }

        if self.calibration.staged_dirty {
            let _ = actions.push(Action::PersistCalibration);
        }

        let _ = actions.push(Action::PublishSnapshot);
        actions
    }

    fn limp_home_aux_commands(&self) -> AuxCommandBatch<RUNTIME_AUX_COMMAND_CAP> {
        let mut commands = AuxCommandBatch::new();
        let _ = commands.push(AuxCommand::new(
            AuxOutput::Fan,
            AuxValue::Level(OutputLevel::High),
        ));

        if matches!(self.output_profile, RuntimeOutputProfile::M50(_)) {
            let _ = commands.push(AuxCommand::new(AuxOutput::VanosIntake, AuxValue::Off));
            let _ = commands.push(AuxCommand::new(AuxOutput::IdleValveOpen, AuxValue::Off));
            let _ = commands.push(AuxCommand::new(AuxOutput::IdleValveClose, AuxValue::Off));
        }

        commands
    }

    fn make_ignition_plan(
        &self,
        control: &ControlPlan,
        _now_us: Micros,
    ) -> ecu_scheduler::IgnitionPlan {
        ecu_scheduler::IgnitionPlan {
            output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
            dwell: control.ignition.dwell_us,
            advance: control.ignition.advance_deg10,
        }
    }

    pub fn apply_decoder_observation(&mut self, observation: DecoderObservation) {
        self.apply_decoder_observation_inner(observation, true);
    }

    fn apply_decoder_observation_inner(&mut self, observation: DecoderObservation, refresh: bool) {
        match observation {
            DecoderObservation::Trigger(trigger) => {
                self.engine.rpm = trigger.rpm;
                self.engine.angle_x10 = trigger.angle_x10;
                let cam_seen = matches!(
                    self.engine.engine_time_authority.phase,
                    PhaseSyncState::CamObserved720 | PhaseSyncState::CamValidated720
                );
                let authority = derive_engine_time_authority(
                    self.engine.engine_time_authority,
                    trigger.synced,
                    cam_seen,
                    trigger.rpm,
                );
                self.set_engine_time_authority_inner(authority);
            }
            DecoderObservation::Cam(cam) => {
                let authority = derive_engine_time_authority(
                    self.engine.engine_time_authority,
                    self.engine.engine_time_authority.has_primary_lock(),
                    cam.cam_seen,
                    self.engine.rpm,
                );
                self.set_engine_time_authority_inner(authority);
            }
        }

        if refresh {
            self.refresh_snapshot();
        }
    }

    fn refresh_snapshot(&mut self) {
        self.runtime_snapshot = RuntimeSnapshot {
            engine: self.engine,
            control: self.control,
            faults: self.faults,
            rev_soft_active: false,
            rev_hard_active: false,
            launch_active: false,
            flat_shift_active: false,
            fuel_cut: self.fuel_cut,
            spark_cut: self.spark_cut,
        };
        self.calibration_snapshot = self.calibration.active;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_board_api::{AuxCommand, AuxOutput, AuxValue, OutputLevel};
    use ecu_board_profiles::M50B25TU_FULL_COP;
    #[cfg(test)]
    use ecu_calibration::{
        ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertUnlock,
        SecondaryTriggerMode, TriggerAuthority,
    };
    use ecu_spec::{
        default_reference_calibration, step as spec_step, InputSnapshot as SpecInputSnapshot,
        LogicalState,
    };

    fn test_fuel_model() -> BaseFuelModel {
        let rpm_bins = [
            Rpm::new(500),
            Rpm::new(1000),
            Rpm::new(1500),
            Rpm::new(2000),
            Rpm::new(2500),
            Rpm::new(3000),
            Rpm::new(3500),
            Rpm::new(4000),
            Rpm::new(4500),
            Rpm::new(5000),
            Rpm::new(5500),
            Rpm::new(6000),
            Rpm::new(6500),
            Rpm::new(7000),
            Rpm::new(7500),
            Rpm::new(8000),
        ];
        let load_bins = [
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1000),
            Kpa10::new(1100),
            Kpa10::new(1200),
            Kpa10::new(1300),
            Kpa10::new(1400),
            Kpa10::new(1500),
            Kpa10::new(1600),
            Kpa10::new(1700),
        ];
        let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(0); 16]; 16];
        pulse_widths[5][5] = ecu_domain::PulseWidthUs::new(2500);
        BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
    }

    fn authority(
        crank: CrankSyncState,
        phase: PhaseSyncState,
        absolute: AbsoluteTimeAuthority,
    ) -> EngineTimeAuthority {
        EngineTimeAuthority::new(
            crank,
            phase,
            absolute,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        )
    }

    fn validated_expert_authority() -> EngineTimeAuthority {
        let calibration = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            injection_layout: ExpertInjectionLayout::Sequential,
            ..ExpertTriggerCalibration::default()
        };
        let startup_authority = authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
        );

        calibration
            .to_runtime_engine_time_authority(startup_authority)
            .expect("validated expert authority")
    }

    fn running_step_inputs(
        now_us: u32,
        rpm: u32,
        trigger_synced: bool,
        cam_seen: bool,
    ) -> StepInputs {
        StepInputs {
            now_us: Micros::new(now_us),
            rpm,
            load_kpa10: 700,
            angle_x10: 2_000,
            trigger_synced,
            cam_seen,
            launch_armed: false,
            flat_shift_armed: false,
        }
    }

    fn running_control_inputs(now_us: u32, rpm: u16) -> ControlInputs {
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(now_us),
                clt_c: 20,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: ecu_domain::Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(
                ecu_domain::Degrees10::new(100),
                0,
                0,
                0,
                false,
                Rpm::new(rpm),
            ),
        }
    }

    fn arm_scheduler_count<const N: usize>(actions: ActionBatch<N>) -> usize {
        actions
            .iter()
            .filter(|action| matches!(*action, Action::ArmScheduler { .. }))
            .count()
    }

    fn canonical_runtime_input() -> SpecInputSnapshot {
        SpecInputSnapshot {
            t_us: ecu_spec::Micros(10_000),
            rpm: ecu_spec::Rpm(1000),
            map_kpa10: ecu_spec::Kpa10(1000),
            load_kpa10: ecu_spec::Kpa10(1000),
            tps_x100: 0,
            clt_c10: ecu_spec::TempC10(4),
            iat_c10: ecu_spec::TempC10(20),
            baro_kpa10: ecu_spec::Kpa10(1000),
            vbatt_mv: ecu_spec::Millivolts(12_000),
            knock_intensity_x100: 0,
            launch_armed: false,
            flat_shift_armed: false,
            sync: ecu_spec::SyncState::Synced,
            fuel_cut: false,
            spark_cut: false,
            mode: ecu_spec::EngineMode::Running,
            target_afr_override_x100: ecu_spec::AfrOverride::None,
        }
    }

    #[test]
    fn differential_input_snapshot_represents_fm0016_fields() {
        let snapshot = DifferentialInputSnapshot {
            now_us: Micros::new(10_000),
            rpm: Rpm::new(1200),
            map_kpa10: Kpa10::new(920),
            load_kpa10: Kpa10::new(870),
            angle_x10: Degrees10::new(2000),
            clt_c10: -350,
            iat_c10: -120,
            baro_kpa10: Kpa10::new(980),
            vbatt_mv: 11_800,
            sync: SyncState::Unsynced,
            fuel_cut: true,
            spark_cut: true,
            mode: RuntimeEngineMode::Shutdown,
            target_afr_override_x100: RuntimeAfrOverride::Some(4000),
            launch_armed: false,
            flat_shift_armed: false,
        };

        let mapped = snapshot.to_step_inputs();
        assert_eq!(mapped.now_us, Micros::new(10_000));
        assert_eq!(mapped.rpm, 1200);
        assert_eq!(mapped.load_kpa10, 870);
        assert_eq!(mapped.angle_x10, 2000);
        assert!(!mapped.trigger_synced);
        assert!(!mapped.cam_seen);
    }

    fn assert_within(
        field: &str,
        runtime: u32,
        oracle: u32,
        tolerance: u32,
        input: &SpecInputSnapshot,
        fixture: &str,
    ) {
        let difference = runtime.abs_diff(oracle);
        assert!(
            difference <= tolerance,
            "input_snapshot={input:?}\ncalibration_fixture={fixture}\nruntime_output={runtime}\noracle_output={oracle}\nfield={field}\ndifference={difference}\ntolerance={tolerance}"
        );
    }

    fn semantic_axis2() -> RuntimeSemanticAxis16 {
        let mut values = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
        values[0] = 100;
        values[1] = 200;
        RuntimeSemanticAxis16 { len: 2, values }
    }

    fn semantic_table_u16(value: u16) -> RuntimeSemanticTable2dU16 {
        RuntimeSemanticTable2dU16 {
            rpm_axis: semantic_axis2(),
            load_axis: semantic_axis2(),
            values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
        }
    }

    fn semantic_table_i16(value: i16) -> RuntimeSemanticTable2dI16 {
        RuntimeSemanticTable2dI16 {
            rpm_axis: semantic_axis2(),
            load_axis: semantic_axis2(),
            values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
        }
    }

    fn semantic_table_u32(value: u32) -> RuntimeSemanticTable2dU32 {
        RuntimeSemanticTable2dU32 {
            rpm_axis: semantic_axis2(),
            load_axis: semantic_axis2(),
            values: [[value; RUNTIME_SEMANTIC_TABLE_LEN]; RUNTIME_SEMANTIC_TABLE_LEN],
        }
    }

    fn semantic_schedule_calibration(dwell_us: u32) -> RuntimeSemanticScheduleCalibration {
        let mut phases = [0u16; RUNTIME_SEMANTIC_TABLE_LEN];
        phases[0] = 0;
        RuntimeSemanticScheduleCalibration {
            spark_advance_table_deg10: semantic_table_i16(150),
            dwell_table_us: semantic_table_u32(dwell_us),
            injection_target_table_deg10: semantic_table_u16(360),
            injection_angle_mode: RuntimeSemanticInjectionAngleMode::EndOfInjection,
            cylinder_phase_deg10: RuntimeSemanticCylinderArrayU16 {
                count: 1,
                values: phases,
            },
        }
    }

    fn semantic_schedule_input(rpm: u16, sync: SyncState) -> RuntimeSemanticInputSnapshot {
        RuntimeSemanticInputSnapshot {
            t_us: Micros::new(0),
            rpm: Rpm::new(rpm),
            map_kpa10: Kpa10::new(100),
            load_kpa10: Kpa10::new(100),
            tps_x100: 0,
            clt_c10: 800,
            iat_c10: 250,
            baro_kpa10: Kpa10::new(1000),
            vbatt_mv: 12_000,
            knock_intensity_x100: 0,
            launch_armed: false,
            flat_shift_armed: false,
            sync,
            fuel_cut: false,
            spark_cut: false,
            mode: RuntimeSemanticEngineMode::Running,
            target_afr_override_x100: RuntimeSemanticAfrOverride::None,
        }
    }

    fn semantic_fuel_observations(
        pw_corr_us: u32,
        fuel_cut: bool,
        spark_cut: bool,
    ) -> RuntimeSemanticFuelObservations {
        RuntimeSemanticFuelObservations {
            ve_pct_x100: 8000,
            target_afr_x100: 1470,
            pw_base_us: 1000,
            pw_air_us: 1000,
            pw_corr_us,
            fuel_cut,
            spark_cut,
            lambda_correction_x1000: 1000,
            lambda_integrator_state: RuntimeSemanticPiIntegratorState::default(),
            advance_deg10_trim: 0,
        }
    }

    #[test]
    fn runtime_semantic_schedule_preserves_u32_dwell_values() {
        let out = runtime_semantic_evaluate_schedule(
            &semantic_schedule_calibration(70_000),
            semantic_schedule_input(100, SyncState::Synced),
            semantic_fuel_observations(1000, false, false),
        )
        .expect("valid schedule");

        assert_eq!(out.dwell_us, 70_000);
        assert_eq!(out.dwell_duration_deg10, 420);
    }

    #[test]
    fn runtime_semantic_schedule_rejects_dwell_duration_overflow() {
        let err = runtime_semantic_evaluate_schedule(
            &semantic_schedule_calibration(2_000_000),
            semantic_schedule_input(8000, SyncState::Synced),
            semantic_fuel_observations(0, false, false),
        )
        .expect_err("dwell duration should overflow u16 deg10");

        assert_eq!(err, RuntimeSemanticScheduleError::DurationOverflow);
    }

    #[test]
    fn runtime_semantic_schedule_cut_diagnostic_precedes_unsynced() {
        let out = runtime_semantic_evaluate_schedule(
            &semantic_schedule_calibration(2500),
            semantic_schedule_input(1000, SyncState::Unsynced),
            semantic_fuel_observations(0, true, true),
        )
        .expect("valid schedule");

        assert_eq!(
            out.diagnostic,
            RuntimeSemanticScheduleDiagnostic::FuelCutActive
        );
        assert_eq!(out.events.len, 0);
    }

    #[test]
    fn fast_events_coalesce_by_kind() {
        let mut queues: RuntimeQueues<2, 2> = RuntimeQueues::new();

        assert_eq!(
            queues.push(Event::Fast(FastEvent::SensorSample {
                rpm: Rpm::new(1000),
                load_kpa10: Kpa10::new(300),
            })),
            Ok(QueueResult::Enqueued)
        );
        assert_eq!(
            queues.push(Event::Fast(FastEvent::SensorSample {
                rpm: Rpm::new(1500),
                load_kpa10: Kpa10::new(450),
            })),
            Ok(QueueResult::Coalesced)
        );

        match queues.pop_fast() {
            Some(Event::Fast(FastEvent::SensorSample { rpm, load_kpa10 })) => {
                assert_eq!(rpm.get(), 1500);
                assert_eq!(load_kpa10.get(), 450);
            }
            other => panic!("unexpected event: {:?}", other),
        }
    }

    #[test]
    fn slow_events_fifo_and_overflow() {
        let mut queues: RuntimeQueues<1, 1> = RuntimeQueues::new();

        assert_eq!(
            queues.push(Event::Slow(SlowEvent::SnapshotRequested)),
            Ok(QueueResult::Enqueued)
        );
        assert_eq!(
            queues.push(Event::Slow(SlowEvent::PersistRequested)),
            Err(QueueOverflow::SlowFull)
        );
        assert_eq!(
            queues.pop_slow(),
            Some(Event::Slow(SlowEvent::SnapshotRequested))
        );
    }

    #[test]
    fn runtime_queue_split_prioritizes_fast_and_reports_overflow() {
        let mut queues: RuntimeQueues<2, 1> = RuntimeQueues::new();

        assert_eq!(
            queues.push(Event::Slow(SlowEvent::SnapshotRequested)),
            Ok(QueueResult::Enqueued)
        );
        assert_eq!(
            queues.push(Event::Fast(FastEvent::TriggerEdge {
                at_us: Micros::new(1),
            })),
            Ok(QueueResult::Enqueued)
        );
        assert_eq!(
            queues.push(Event::Fast(FastEvent::TriggerEdge {
                at_us: Micros::new(2),
            })),
            Ok(QueueResult::Coalesced)
        );
        assert_eq!(
            queues.push(Event::Slow(SlowEvent::PersistRequested)),
            Err(QueueOverflow::SlowFull)
        );
        assert!(matches!(
            queues.pop_fast(),
            Some(Event::Fast(FastEvent::TriggerEdge { .. }))
        ));
        assert!(matches!(
            queues.pop_slow(),
            Some(Event::Slow(SlowEvent::SnapshotRequested))
        ));
    }

    #[test]
    fn fast_and_slow_lanes_are_independent() {
        let mut queues: RuntimeQueues<1, 1> = RuntimeQueues::new();

        assert_eq!(
            queues.push(Event::Fast(FastEvent::TriggerEdge {
                at_us: Micros::new(10),
            })),
            Ok(QueueResult::Enqueued)
        );
        assert_eq!(
            queues.push(Event::Slow(SlowEvent::CalibrationCommitted)),
            Ok(QueueResult::Enqueued)
        );

        assert!(matches!(
            queues.pop_fast(),
            Some(Event::Fast(FastEvent::TriggerEdge { .. }))
        ));
        assert!(matches!(
            queues.pop_slow(),
            Some(Event::Slow(SlowEvent::CalibrationCommitted))
        ));
    }

    #[test]
    fn runtime_differential_mapping_matches_oracle_fuel_and_state() {
        let input = canonical_runtime_input();
        let oracle = spec_step(
            &default_reference_calibration(),
            input,
            &LogicalState::default(),
        );
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = {
            let rpm_bins = [
                Rpm::new(500),
                Rpm::new(1000),
                Rpm::new(1500),
                Rpm::new(2000),
                Rpm::new(2500),
                Rpm::new(3000),
                Rpm::new(3500),
                Rpm::new(4000),
                Rpm::new(4500),
                Rpm::new(5000),
                Rpm::new(5500),
                Rpm::new(6000),
                Rpm::new(6500),
                Rpm::new(7000),
                Rpm::new(7500),
                Rpm::new(8000),
            ];
            let load_bins = [
                Kpa10::new(200),
                Kpa10::new(300),
                Kpa10::new(400),
                Kpa10::new(500),
                Kpa10::new(600),
                Kpa10::new(700),
                Kpa10::new(800),
                Kpa10::new(900),
                Kpa10::new(1000),
                Kpa10::new(1100),
                Kpa10::new(1200),
                Kpa10::new(1300),
                Kpa10::new(1400),
                Kpa10::new(1500),
                Kpa10::new(1600),
                Kpa10::new(1700),
            ];
            let mut pulse_widths = [[ecu_domain::PulseWidthUs::new(2500); 16]; 16];
            pulse_widths[0][0] = ecu_domain::PulseWidthUs::new(2500);
            BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
        };

        let result = runtime.step(
            StepInputs {
                now_us: ecu_domain::Micros::new(10_000),
                rpm: input.rpm.0 as u32,
                load_kpa10: input.load_kpa10.0 as u32,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: ecu_domain::Micros::new(10_000),
                    clt_c: 4,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(100, 100, 100, 100, 100),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(150),
                    0,
                    0,
                    0,
                    false,
                    ecu_domain::Rpm::new(1000),
                ),
            },
        );

        assert_eq!(runtime.engine.rpm.get(), input.rpm.0);
        assert_eq!(runtime.engine.load_kpa10.get(), input.load_kpa10.0);
        assert_eq!(runtime.engine.sync, ecu_domain::SyncState::Synced);
        assert_eq!(runtime.engine.mode, ecu_domain::ControlMode::ClosedLoop);
        assert_eq!(result.control.ignition.advance_deg10.get(), 150);
        assert_within(
            "pw_corr_us",
            result.control.enriched_fuel.get() as u32,
            oracle.output.pw_corr_us.0,
            1,
            &input,
            "canonical_reference_calibration",
        );
        assert_eq!(oracle.output.diagnostic, ecu_spec::DiagnosticCode::None);
        assert_eq!(
            oracle.next_state.diag.current,
            ecu_spec::DiagnosticCode::None
        );
    }

    #[test]
    fn engine_runtime_layout_defaults_cleanly() {
        let runtime = EngineRuntime::new();

        assert_eq!(runtime.engine.sync, SyncState::Unsynced);
        assert_eq!(runtime.engine_time_authority(), EngineTimeAuthority::none());
        assert_eq!(runtime.engine.phase, EnginePhase::Off);
        assert_eq!(runtime.engine.angle_x10.get(), 0);
        assert_eq!(runtime.control.lambda_target.get(), 100);
        assert_eq!(runtime.faults.severity, FaultSeverity::Info);
        assert!(!runtime.calibration.staged_dirty);
        assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
        assert_eq!(runtime.runtime_snapshot.engine.phase, EnginePhase::Off);
        assert_eq!(runtime.calibration_snapshot, CalibrationSnapshot::default());
        assert_eq!(
            runtime.output_profile(),
            RuntimeOutputProfile::legacy_single_channel()
        );
    }

    #[test]
    fn decoder_observations_do_not_let_cam_seen_certify_sync_by_itself() {
        let mut runtime = EngineRuntime::new();

        runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
            at_us: Micros::new(10),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(45),
            synced: false,
        }));

        assert_eq!(runtime.engine.sync, SyncState::Unsynced);
        assert_eq!(runtime.engine.phase, EnginePhase::Cranking);
        assert_eq!(runtime.engine.rpm.get(), 1200);
        assert_eq!(runtime.engine.angle_x10.get(), 45);
        assert_eq!(
            runtime.engine_time_authority().crank,
            CrankSyncState::PrimarySearching
        );

        runtime.apply_decoder_observation(DecoderObservation::Cam(CamObservation {
            at_us: Micros::new(20),
            cam_seen: true,
        }));

        assert_eq!(runtime.engine.sync, SyncState::Unsynced);
        assert_eq!(runtime.engine.phase, EnginePhase::Cranking);
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));

        runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
            at_us: Micros::new(30),
            rpm: Rpm::new(1200),
            angle_x10: Degrees10::new(90),
            synced: true,
        }));

        assert_eq!(runtime.engine.sync, SyncState::Synced);
        assert_eq!(runtime.engine.phase, EnginePhase::Running);
        assert_eq!(runtime.runtime_snapshot.engine.sync, SyncState::Synced);
        assert_eq!(
            runtime.engine_time_authority().phase,
            PhaseSyncState::CrankOnly360
        );
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));
    }

    #[test]
    fn runtime_full_sequential_gate_requires_validated_phase_and_absolute_authority() {
        let crank_only = authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::GeometryOnly,
        );
        let cam_observed_expert = authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamObserved720,
            AbsoluteTimeAuthority::ExpertManual,
        );
        let cam_validated_geometry = authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
        );
        let cam_validated_unknown = authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::None,
        );
        let cam_validated_expert = validated_expert_authority();
        let cam_validated_certified = authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::CertifiedProfile,
        );

        assert_eq!(crank_only.compatibility_summary(), SyncState::Synced);
        assert!(!runtime_full_sequential_authorized(crank_only));
        assert!(!runtime_full_sequential_authorized(cam_observed_expert));
        assert!(!runtime_full_sequential_authorized(cam_validated_geometry));
        assert!(!runtime_full_sequential_authorized(cam_validated_unknown));
        assert!(runtime_full_sequential_authorized(cam_validated_expert));
        assert!(runtime_full_sequential_authorized(cam_validated_certified));
        assert_ne!(
            cam_validated_certified.absolute,
            cam_validated_expert.absolute
        );
        assert!(cam_validated_certified.is_certified_profile());
    }

    #[test]
    fn runtime_m50_full_cop_blocks_unknown_absolute_even_with_validated_720_phase() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_full_cop();
        let m50_absolute_authority = M50B25TU_FULL_COP
            .trigger
            .trigger_angle_atdc_deg10
            .absolute_authority();
        assert_eq!(m50_absolute_authority, AbsoluteTimeAuthority::None);
        runtime.set_engine_time_authority(authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            m50_absolute_authority,
        ));

        let result = runtime.step(
            running_step_inputs(6_000, 3_000, true, true),
            running_control_inputs(6_000, 3_000),
        );

        assert_eq!(arm_scheduler_count(result.actions), 0);
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));
    }

    #[test]
    fn set_engine_time_authority_sanitizes_invalid_snapshot() {
        let mut runtime = EngineRuntime::new();
        runtime.set_engine_time_authority(authority(
            CrankSyncState::NoSignal,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::ExpertManual,
        ));

        assert_eq!(runtime.engine_time_authority(), EngineTimeAuthority::none());
        assert_eq!(runtime.engine.sync, SyncState::Unsynced);
        assert_eq!(runtime.engine.phase, EnginePhase::Off);
    }

    #[test]
    fn step_with_authority_keeps_structured_baseline_when_inputs_match() {
        let mut runtime = EngineRuntime::new();

        let authority = validated_expert_authority();
        let result = runtime.step_with_authority(
            running_step_inputs(10, 3000, true, true),
            running_control_inputs(10, 3000),
            authority,
        );

        assert_eq!(runtime.engine_time_authority(), authority);
        assert_eq!(runtime.engine.sync, SyncState::Synced);
        assert_eq!(
            runtime.runtime_snapshot.engine.engine_time_authority,
            authority
        );
        assert_eq!(result.validated.rpm, Rpm::new(3000));
    }

    #[test]
    fn runtime_step_validates_inputs_and_orders_derivation() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.faults.severity = FaultSeverity::Warning;
        runtime.calibration.staged_dirty = true;

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 8_000,
                trigger_synced: false,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(0),
                    clt_c: 20,
                    cranking: true,
                    just_started: true,
                    tpsdot_pct_s: 200,
                    mapdot_kpa_s: 90,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(96),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(92, 80, 120, 118, 110),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(110),
                    8,
                    2,
                    4,
                    false,
                    Rpm::new(2800),
                ),
            },
        );

        assert_eq!(result.validated.rpm.get(), 3000);
        assert_eq!(result.validated.load_kpa10.get(), 700);
        assert_eq!(result.validated.angle_x10.get(), 7200);
        assert!(result.validated.clamped);
        assert_eq!(runtime.engine.sync, SyncState::Unsynced);
        assert_eq!(runtime.engine.phase, EnginePhase::Cranking);
        assert_eq!(result.operating_mode, ControlMode::LimpHome);
        assert_eq!(runtime.engine.mode, ControlMode::LimpHome);
        assert_eq!(runtime.runtime_snapshot.engine.mode, ControlMode::LimpHome);
        assert_eq!(result.control.base_fuel.get(), 2500);
        assert!(result.control.enriched_fuel.get() >= result.control.base_fuel.get());
        assert_eq!(
            runtime.control.fuel_pulse_width,
            result.control.enriched_fuel
        );
        assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
        let mut actions = result.actions.iter();
        assert_eq!(actions.next(), Some(Action::Idle));
        match actions.next() {
            Some(Action::ApplyAux(batch)) => {
                assert_eq!(
                    batch.as_slice(),
                    &[AuxCommand::new(
                        AuxOutput::Fan,
                        AuxValue::Level(OutputLevel::High)
                    )]
                );
            }
            other => panic!("expected aux batch, got {other:?}"),
        }
        assert_eq!(actions.next(), Some(Action::PersistCalibration));
        assert_eq!(actions.next(), Some(Action::PublishSnapshot));
        assert!(actions.next().is_none());
    }

    #[test]
    fn runtime_m50_limp_home_emits_fan_and_noop_aux_commands() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_mega_compatible();
        runtime.set_engine_time_authority(validated_expert_authority());
        runtime.set_fault_state(
            FaultCode::SensorOutOfRange,
            FaultSeverity::Warning,
            CancelReason::Manual,
        );

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(5_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(5_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(3000),
                ),
            },
        );

        let mut arm_count = 0usize;
        let mut aux_seen = false;
        let mut publish_seen = false;

        for action in result.actions.iter() {
            match action {
                Action::ArmScheduler { .. } => arm_count += 1,
                Action::ApplyAux(batch) => {
                    aux_seen = true;
                    assert_eq!(
                        batch.as_slice(),
                        &[
                            AuxCommand::new(AuxOutput::Fan, AuxValue::Level(OutputLevel::High)),
                            AuxCommand::new(AuxOutput::VanosIntake, AuxValue::Off),
                            AuxCommand::new(AuxOutput::IdleValveOpen, AuxValue::Off),
                            AuxCommand::new(AuxOutput::IdleValveClose, AuxValue::Off),
                        ]
                    );
                }
                Action::PublishSnapshot => publish_seen = true,
                other => panic!("unexpected action in m50 limp-home path: {other:?}"),
            }
        }

        assert_eq!(arm_count, 6);
        assert!(aux_seen);
        assert!(publish_seen);
    }

    #[test]
    fn runtime_unsynced_path_emits_idle_and_snapshot() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(3_000),
                rpm: 0,
                load_kpa10: 0,
                angle_x10: 0,
                trigger_synced: false,
                cam_seen: false,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(3_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(0),
                ),
            },
        );

        let mut actions = result.actions.iter();
        assert_eq!(actions.next(), Some(Action::Idle));
        assert_eq!(actions.next(), Some(Action::PublishSnapshot));
        assert!(actions.next().is_none());
    }

    #[test]
    fn runtime_shutdown_path_emits_cancel_and_snapshot() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.set_fault_state(
            FaultCode::SafetyCut,
            FaultSeverity::Critical,
            CancelReason::SafetyShutdown,
        );

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(4_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(4_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(3000),
                ),
            },
        );

        let mut actions = result.actions.iter();
        assert_eq!(
            actions.next(),
            Some(Action::CancelScheduler(CancelReason::SafetyShutdown))
        );
        assert_eq!(actions.next(), Some(Action::PublishSnapshot));
        assert!(actions.next().is_none());
    }

    #[test]
    fn runtime_m50_mega_profile_blocks_crank_only_primary_lock() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_mega_compatible();
        runtime.set_engine_time_authority(authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::GeometryOnly,
        ));

        let result = runtime.step(
            running_step_inputs(5_000, 3_000, true, false),
            running_control_inputs(5_000, 3_000),
        );

        assert_eq!(runtime.engine.sync, SyncState::Synced);
        assert_eq!(arm_scheduler_count(result.actions), 0);
        assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));
    }

    #[test]
    fn runtime_m50_mega_validated_authority_emits_six_injectors_and_wasted_spark() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_mega_compatible();
        runtime.set_engine_time_authority(validated_expert_authority());
        runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
            at_us: Micros::new(1_000),
            rpm: Rpm::new(3_000),
            angle_x10: Degrees10::new(120),
            synced: true,
        }));

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(5_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(5_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(3000),
                ),
            },
        );

        let mut injector_channels = [u8::MAX; 6];
        let mut ignition_channels = [u8::MAX; 6];
        let mut start_times = [0u32; 6];
        let mut seen = 0usize;
        let mut publish_seen = false;
        for action in result.actions.iter() {
            match action {
                Action::ArmScheduler {
                    injection,
                    ignition,
                } => {
                    start_times[seen] = injection.start_at.get();
                    injector_channels[seen] = injection.plan.output.channel().get();
                    ignition_channels[seen] = ignition.plan.output.channel().get();
                    seen += 1;
                }
                Action::PublishSnapshot => publish_seen = true,
                other => panic!("unexpected action in m50 mega profile: {other:?}"),
            }
        }

        assert_eq!(seen, 6);
        assert!(start_times
            .windows(2)
            .all(|window| window[1] - window[0] == 6_666));
        assert!(start_times[5] - start_times[0] < 40_000);
        assert_eq!(injector_channels, [0, 1, 2, 3, 4, 5]);
        assert_eq!(ignition_channels, [0, 1, 2, 0, 1, 2]);
        assert!(ignition_channels.iter().all(|channel| *channel <= 2));
        assert!(publish_seen);
    }

    #[test]
    fn runtime_m50_cycle_slot_spacing_is_sixth_of_720_degree_cycle() {
        let profile = M50OutputProfile::mega_compatible();

        assert_eq!(profile.cycle_slot_us(Rpm::new(0)), 0);
        assert_eq!(profile.cycle_slot_us(Rpm::new(3_000)), 6_666);
    }

    #[test]
    fn runtime_m50_full_cop_profile_with_validated_authority_emits_ignition_channel_five() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_full_cop();
        runtime.set_engine_time_authority(validated_expert_authority());
        runtime.apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
            at_us: Micros::new(1_000),
            rpm: Rpm::new(3_000),
            angle_x10: Degrees10::new(120),
            synced: true,
        }));

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(6_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(6_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(3000),
                ),
            },
        );

        let mut seen = 0usize;
        let mut last_ignition_channel = None;
        let mut publish_seen = false;
        for action in result.actions.iter() {
            match action {
                Action::ArmScheduler {
                    injection: _,
                    ignition,
                } => {
                    last_ignition_channel = Some(ignition.plan.output.channel().get());
                    seen += 1;
                }
                Action::PublishSnapshot => publish_seen = true,
                other => panic!("unexpected action in m50 full-cop profile: {other:?}"),
            }
        }

        assert_eq!(seen, 6);
        assert_eq!(last_ignition_channel, Some(5));
        assert!(publish_seen);
    }

    #[test]
    fn runtime_m50_full_cop_blocks_crank_only_primary_lock() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_full_cop();
        runtime.set_engine_time_authority(authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::GeometryOnly,
        ));

        let result = runtime.step(
            running_step_inputs(6_000, 3_000, true, false),
            running_control_inputs(6_000, 3_000),
        );

        assert_eq!(runtime.engine.sync, SyncState::Synced);
        assert_eq!(arm_scheduler_count(result.actions), 0);
        assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));
    }

    #[test]
    fn runtime_m50_full_cop_blocks_cam_observed_but_not_validated() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_full_cop();
        runtime.set_engine_time_authority(authority(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamObserved720,
            AbsoluteTimeAuthority::ExpertManual,
        ));

        let result = runtime.step(
            running_step_inputs(6_000, 3_000, true, true),
            running_control_inputs(6_000, 3_000),
        );

        assert_eq!(runtime.engine.sync, SyncState::Synced);
        assert_eq!(
            runtime.engine_time_authority().phase,
            PhaseSyncState::CamObserved720
        );
        assert_eq!(arm_scheduler_count(result.actions), 0);
        assert_eq!(runtime.scheduler.mode(), ecu_scheduler::SchedulerMode::Idle);
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));
    }

    #[test]
    fn runtime_snapshot_matches_state_after_step() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(2_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(2_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(3000),
                ),
            },
        );

        assert_eq!(runtime.runtime_snapshot.engine, runtime.engine);
        assert_eq!(runtime.runtime_snapshot.control, runtime.control);
        assert_eq!(runtime.runtime_snapshot.faults, runtime.faults);
        assert_eq!(result.control.base_fuel.get(), 2500);
    }

    #[test]
    fn sync_loss_cancels_pending_outputs() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();

        let _ = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 3_000,
                load_kpa10: 700,
                angle_x10: 2_000,
                trigger_synced: true,
                cam_seen: true,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(1_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(3000),
                ),
            },
        );

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(2_000),
                rpm: 0,
                load_kpa10: 0,
                angle_x10: 0,
                trigger_synced: false,
                cam_seen: false,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(2_000),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(90, 90, 90, 90, 90),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(100),
                    0,
                    0,
                    0,
                    false,
                    Rpm::new(0),
                ),
            },
        );

        let mut actions = result.actions.iter();
        assert_eq!(
            actions.next(),
            Some(Action::CancelScheduler(CancelReason::SyncLoss))
        );
        assert_eq!(actions.next(), Some(Action::PublishSnapshot));
        assert!(actions.next().is_none());
        assert_eq!(
            runtime.scheduler.mode(),
            ecu_scheduler::SchedulerMode::Suspended
        );
    }

    #[test]
    fn runtime_m50_full_cop_sync_loss_cancels_pending_outputs() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();
        runtime.configure_m50_full_cop();
        runtime.set_engine_time_authority(validated_expert_authority());

        let first = runtime.step(
            running_step_inputs(1_000, 3_000, true, true),
            running_control_inputs(1_000, 3_000),
        );

        assert_eq!(arm_scheduler_count(first.actions), 6);
        assert_eq!(
            runtime.scheduler.mode(),
            ecu_scheduler::SchedulerMode::Armed
        );

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(2_000),
                rpm: 0,
                load_kpa10: 0,
                angle_x10: 0,
                trigger_synced: false,
                cam_seen: false,
                launch_armed: false,
                flat_shift_armed: false,
            },
            running_control_inputs(2_000, 0),
        );

        let mut actions = result.actions.iter();
        assert_eq!(
            actions.next(),
            Some(Action::CancelScheduler(CancelReason::SyncLoss))
        );
        assert_eq!(actions.next(), Some(Action::PublishSnapshot));
        assert!(actions.next().is_none());
        assert_eq!(
            runtime.scheduler.mode(),
            ecu_scheduler::SchedulerMode::Suspended
        );
        assert!(!runtime_full_sequential_authorized(
            runtime.engine_time_authority()
        ));
    }

    #[test]
    fn torque_observations_zero_allowed_when_engine_is_off() {
        let mut runtime = EngineRuntime::new();
        runtime.planners.fuel_model = test_fuel_model();

        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(500),
                rpm: 0,
                load_kpa10: 0,
                angle_x10: 0,
                trigger_synced: false,
                cam_seen: false,
                launch_armed: false,
                flat_shift_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(500),
                    clt_c: 20,
                    cranking: false,
                    just_started: false,
                    tpsdot_pct_s: 0,
                    mapdot_kpa_s: 0,
                },
                lambda: LambdaTrimInputs {
                    clt_c: 80,
                    lambda_valid: true,
                    measured_lambda100: ecu_domain::Lambda100::new(100),
                    requested_open_loop: false,
                },
                torque: TorqueInputs::new(75, 50, 100, 100, 100),
                ignition: IgnitionInputs::new(
                    ecu_domain::Degrees10::new(120),
                    0,
                    0,
                    0,
                    false,
                    ecu_domain::Rpm::new(0),
                ),
            },
        );

        assert_eq!(result.operating_mode, ControlMode::OpenLoop);
        assert_eq!(result.torque_observations.request_x1000, 750);
        assert_eq!(result.torque_observations.allowed_x1000, 0);
        assert_eq!(result.torque_observations.actuated_x1000, 0);
    }

    #[test]
    fn differential_input_snapshot_preserves_all_fixture_fields() {
        use crate::{RuntimeAfrOverride, RuntimeEngineMode};

        let cases = [
            // Fields: now_us, rpm, map_kpa10, load_kpa10, angle_x10, clt_c10, iat_c10,
            //         baro_kpa10, vbatt_mv, sync, fuel_cut, spark_cut, mode, target_afr_override_x100
            (
                DifferentialInputSnapshot {
                    now_us: Micros::new(1_000_000),
                    rpm: Rpm::new(1500),
                    map_kpa10: Kpa10::new(950),
                    load_kpa10: Kpa10::new(1000),
                    angle_x10: Degrees10::new(2000),
                    clt_c10: 800,
                    iat_c10: 250,
                    baro_kpa10: Kpa10::new(1013),
                    vbatt_mv: 12_100,
                    sync: SyncState::Synced,
                    fuel_cut: false,
                    spark_cut: false,
                    mode: RuntimeEngineMode::Running,
                    target_afr_override_x100: RuntimeAfrOverride::None,
                    launch_armed: false,
                    flat_shift_armed: false,
                },
                "running synced",
            ),
            (
                DifferentialInputSnapshot {
                    now_us: Micros::new(2_000_000),
                    rpm: Rpm::new(0),
                    map_kpa10: Kpa10::new(0),
                    load_kpa10: Kpa10::new(0),
                    angle_x10: Degrees10::new(0),
                    clt_c10: -120,
                    iat_c10: -80,
                    baro_kpa10: Kpa10::new(950),
                    vbatt_mv: 11_500,
                    sync: SyncState::Unsynced,
                    fuel_cut: true,
                    spark_cut: true,
                    mode: RuntimeEngineMode::Off,
                    target_afr_override_x100: RuntimeAfrOverride::Some(1470),
                    launch_armed: false,
                    flat_shift_armed: false,
                },
                "cut with negative temps and AFR override",
            ),
            (
                DifferentialInputSnapshot {
                    now_us: Micros::new(3_000_000),
                    rpm: Rpm::new(500),
                    map_kpa10: Kpa10::new(300),
                    load_kpa10: Kpa10::new(400),
                    angle_x10: Degrees10::new(1000),
                    clt_c10: -300,
                    iat_c10: -400,
                    baro_kpa10: Kpa10::new(850),
                    vbatt_mv: 13_500,
                    sync: SyncState::Syncing,
                    fuel_cut: false,
                    spark_cut: false,
                    mode: RuntimeEngineMode::Cranking,
                    target_afr_override_x100: RuntimeAfrOverride::None,
                    launch_armed: false,
                    flat_shift_armed: false,
                },
                "cranking with cold temps and syncing",
            ),
        ];

        for (snap, label) in cases {
            // to_step_inputs preserves all fields needed for runtime step
            let step = snap.to_step_inputs();
            assert_eq!(step.now_us, snap.now_us, "now_us for {label}");
            assert_eq!(step.rpm, snap.rpm.get() as u32, "rpm for {label}");
            assert_eq!(
                step.load_kpa10,
                snap.load_kpa10.get() as u32,
                "load_kpa10 for {label}"
            );
            assert_eq!(
                step.angle_x10,
                snap.angle_x10.get() as i32,
                "angle_x10 for {label}"
            );
            assert_eq!(
                step.trigger_synced,
                snap.sync == SyncState::Synced,
                "trigger_synced for {label}"
            );
            assert_eq!(
                step.cam_seen,
                snap.sync == SyncState::Synced,
                "cam_seen for {label}"
            );
        }
    }
}
