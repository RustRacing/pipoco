//! Telemetry DTOs.

use crate::capabilities::{IgnitionProfileId, PinMapId, ProfileId, RuntimeBuildId};
use crate::frontier::{
    TimingIslandHorizonSequenceId, TimingIslandPermitMask, TimingIslandStopReason,
};
use crate::sensors::SensorSnapshot;
use ecu_domain::{
    CancelReason, ControlMode, Degrees10, DwellUs, EnginePhase, EngineTimeAuthority, FaultCode,
    FaultSeverity, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm, SyncState,
};

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum CommonSyncTelemetryState {
    #[default]
    NoSignal,
    Unsynced,
    CrankSynced,
    FullSequentialAuthorized,
    SyncLost,
    CamSynced,
    SyncSuspect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonDiagnosticsTelemetry {
    pub sync_state: CommonSyncTelemetryState,
    pub fault_code: FaultCode,
    pub fault_severity: FaultSeverity,
    pub cancel_reason: CancelReason,
    pub fault: CommonRuntimeFaultTelemetry,
    pub lambda: CommonLambdaTelemetry,
    pub lambda_correction: CommonLambdaCorrectionTelemetry,
    pub warmup: CommonWarmupTelemetry,
    pub startup: CommonStartupTelemetry,
    pub afterstart: CommonAfterstartTelemetry,
    pub transient_enrichment: CommonTransientEnrichmentTelemetry,
    pub protection: CommonProtectionTelemetry,
    pub limp_action: CommonLimpActionTelemetry,
    pub late_event_count: u32,
    pub max_lateness_us: u32,
    pub queue_high_water_mark: u8,
    pub last_drain_count: u8,
    pub active_queue_count: u8,
    pub free_queue_slots: u8,
    pub queue_capacity: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonHighRateLogTelemetry {
    pub decision: CommonDecisionTelemetry,
    pub fault: CommonRuntimeFaultTelemetry,
    pub frontier_fault: CommonFrontierFaultTelemetry,
    pub late_event_count: u32,
    pub max_lateness_us: u32,
    pub calibration_checksum: u32,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonProtectionLevel {
    #[default]
    Inactive = 0,
    Degraded = 1,
    ShutdownDriving = 2,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonProtectionSource {
    #[default]
    None = 0,
    RuntimeFault = 1,
    FrontierFault = 2,
    ControlMode = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonProtectionAction {
    #[default]
    None = 0,
    ObserveOnly = 1,
    LimpHome = 2,
    OutputSuppressed = 3,
    SafeStateTransition = 4,
    Shutdown = 5,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonProtectionPersistence {
    #[default]
    Inactive = 0,
    Reversible = 1,
    LatchedUntilClear = 2,
    LatchedUntilRecovery = 3,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonProtectionTelemetry {
    pub level: CommonProtectionLevel,
    pub source: CommonProtectionSource,
    pub action: CommonProtectionAction,
    pub persistence: CommonProtectionPersistence,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonLimpActionLevel {
    #[default]
    Inactive = 0,
    AuxOnly = 1,
    OutputSuppressed = 2,
    ShutdownDriving = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonLimpActionSource {
    #[default]
    None = 0,
    RuntimeFault = 1,
    FrontierFault = 2,
    ControlMode = 3,
    SyncAuthority = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonLimpActionTelemetry {
    pub level: CommonLimpActionLevel,
    pub source: CommonLimpActionSource,
    pub cancel_scheduler: bool,
    pub cancel_reason: CancelReason,
    pub apply_aux: bool,
    pub aux_command_count: u8,
    pub persistence: CommonProtectionPersistence,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonDecisionTelemetry {
    pub control_mode: ControlMode,
    pub rev_soft_active: bool,
    pub rev_hard_active: bool,
    pub launch_active: bool,
    pub flat_shift_active: bool,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub fuel_cut_reason: CommonCutReason,
    pub spark_cut_reason: CommonCutReason,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonCutReason {
    #[default]
    None = 0,
    SafetyLatched = 1,
    DirectRequest = 2,
    Shutdown = 3,
    HardRev = 4,
    Launch = 5,
    FlatShift = 6,
    FuelOnly = 7,
    SoftRev = 8,
    SparkOnly = 9,
    KnockRetard = 10,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonShiftArmingTelemetry {
    pub launch_armed: bool,
    pub flat_shift_armed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonSchedulerMode {
    #[default]
    Idle,
    Armed,
    Suspended,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonSchedulerOwnershipTelemetry {
    pub mode: CommonSchedulerMode,
    pub active_groups: u8,
    pub injection_count: u8,
    pub ignition_count: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonSchedulerReservationTelemetry {
    pub injector_channels: u128,
    pub ignition_channels: u128,
    pub idle_channels: u128,
    pub fan_channels: u128,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonSchedulerStateSummaryTelemetry {
    pub armed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonSchedulerWindowTelemetry {
    pub last_injection_start: Option<Micros>,
    pub last_injection_end: Option<Micros>,
    pub last_ignition_start: Option<Micros>,
    pub last_ignition_end: Option<Micros>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonPendingInputTelemetry {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
    pub authority: EngineTimeAuthority,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonControlTelemetry {
    pub fuel_pulse_width: PulseWidthUs,
    pub ignition_advance: Degrees10,
    pub dwell: DwellUs,
    pub lambda_target: Lambda100,
    pub torque_limit_x100: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonLambdaMode {
    #[default]
    OpenLoop,
    ClosedLoop,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonLambdaActivity {
    #[default]
    Inactive = 0,
    Frozen = 1,
    Active = 2,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonLambdaDisableReason {
    #[default]
    None = 0,
    OpenLoop = 1,
    RequestedOpenLoop = 2,
    SensorInvalid = 3,
    WarmupGate = 4,
    LowLoadGate = 5,
    StartupDelay = 6,
    PowerReductionCut = 7,
    AccelerationEnrichment = 8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonLambdaTelemetry {
    pub activity: CommonLambdaActivity,
    pub reason: CommonLambdaDisableReason,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonLambdaCorrectionTelemetry {
    pub measured_lambda: Lambda100,
    pub target_lambda: Lambda100,
    pub trim_x100: i16,
    pub status: CommonLambdaTelemetry,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonIgnitionLimitReason {
    #[default]
    None,
    Knock,
    Torque,
    RevLimiter,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonTorqueLimitReason {
    #[default]
    None,
    Idle,
    Driver,
    RevLimiter,
    Knock,
    LimpMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonControlReasonTelemetry {
    pub lambda_mode: CommonLambdaMode,
    pub lambda_active: bool,
    pub lambda_trim_x100: i16,
    pub lambda_disable_reason: CommonLambdaDisableReason,
    pub ignition_limit_reason: CommonIgnitionLimitReason,
    pub torque_limit_reason: CommonTorqueLimitReason,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonFuelStrategyMode {
    #[default]
    DirectPulseWidthTable,
    SpeedDensityVe,
    AlphaN,
    Maf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonFuelObservationTelemetry {
    pub base_fuel_pulse_width: PulseWidthUs,
    pub enriched_fuel_pulse_width: PulseWidthUs,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonEnrichmentTelemetry {
    pub startup_x100: u16,
    pub warmup_x100: u16,
    pub after_start_x100: u16,
    pub acceleration_x100: u16,
    pub total_x100: u16,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonWarmupTemperatureMode {
    #[default]
    Inactive = 0,
    ColdClamp = 1,
    Interpolating = 2,
    HotClamp = 3,
    NeutralFallback = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonWarmupTelemetry {
    pub active: bool,
    pub correction_x100: u16,
    pub temperature_mode: CommonWarmupTemperatureMode,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonStartupWindowMode {
    #[default]
    Inactive = 0,
    Milliseconds = 1,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonStartupTelemetry {
    pub active: bool,
    pub remaining_window: u16,
    pub window_mode: CommonStartupWindowMode,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonAfterstartWindowMode {
    #[default]
    Inactive = 0,
    Milliseconds = 1,
    Cycles = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonAfterstartTelemetry {
    pub active: bool,
    pub remaining_window: u16,
    pub window_mode: CommonAfterstartWindowMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonTransientEnrichmentTelemetry {
    pub acceleration_active: bool,
    pub acceleration_pulse_us: u16,
    pub acceleration_decay_steps_remaining: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonActionTelemetry {
    pub total_action_count: u8,
    pub arm_scheduler_count: u8,
    pub arm_injection_count: u8,
    pub arm_ignition_count: u8,
    pub apply_aux_count: u8,
    pub apply_aux_command_count: u8,
    pub idle_count: u8,
    pub publish_snapshot_count: u8,
    pub publish_snapshot: bool,
    pub persist_calibration: bool,
    pub persist_calibration_count: u8,
    pub cancel_scheduler: bool,
    pub cancel_reason: CancelReason,
    pub cancel_scheduler_count: u8,
    pub multiple_cancel_reasons: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonFrontierTelemetry {
    pub active_horizon_id: Option<TimingIslandHorizonSequenceId>,
    pub horizon_start_us: Option<Micros>,
    pub horizon_end_us: Option<Micros>,
    pub last_accepted_horizon_id: Option<TimingIslandHorizonSequenceId>,
    pub last_accepted_horizon_start_us: Option<Micros>,
    pub last_accepted_horizon_end_us: Option<Micros>,
    pub heartbeat_deadline_us: Option<Micros>,
    pub active_permit_mask: TimingIslandPermitMask,
    pub active_stop_reason: TimingIslandStopReason,
    pub fault: CommonFrontierFaultTelemetry,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonFrontierFaultEventId {
    #[default]
    None = 0,
    SyncLost = 1,
    HeartbeatExpired = 2,
    HorizonExpired = 3,
    PermitDenied = 4,
    TimingFault = 5,
    AdmittedEventRejected = 6,
    BoardOutputFault = 7,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonFrontierFaultAction {
    #[default]
    None = 0,
    OutputSuppressed = 1,
    SafeStateTransition = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonFrontierFaultTelemetry {
    pub event_id: CommonFrontierFaultEventId,
    pub severity: FaultSeverity,
    pub action: CommonFrontierFaultAction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonTorqueTelemetry {
    pub request_x1000: u16,
    pub allowed_x1000: u16,
    pub actuated_x1000: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonEngineTelemetry {
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
    pub phase: EnginePhase,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonTriggerEdgeTelemetry {
    pub seen: bool,
    pub at_us: Micros,
    pub rpm: Rpm,
    pub angle_x10: Degrees10,
    pub authority: EngineTimeAuthority,
    pub synced: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonCamEdgeTelemetry {
    pub seen: bool,
    pub at_us: Micros,
    pub cam_seen: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonValidatedInputTelemetry {
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
    pub clamped: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonRuntimeFaultTelemetry {
    pub active: bool,
    pub fault_code: FaultCode,
    pub severity: FaultSeverity,
    pub cancel_reason: CancelReason,
    pub action: CommonFaultTransitionAction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CommonFaultTransitionTelemetry {
    pub changed: bool,
    pub at_us: Micros,
    pub event: CommonFaultTransitionEventTelemetry,
    pub previous_fault: FaultCode,
    pub previous_severity: FaultSeverity,
    pub previous_cancel_reason: CancelReason,
    pub current_fault: FaultCode,
    pub current_severity: FaultSeverity,
    pub current_cancel_reason: CancelReason,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonFaultTransitionEventId {
    #[default]
    None = 0,
    FaultEntered = 1,
    FaultUpdated = 2,
    FaultCleared = 3,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum CommonFaultTransitionAction {
    #[default]
    None = 0,
    ObserveOnly = 1,
    LimpHome = 2,
    Shutdown = 3,
    Cleared = 4,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonFaultTransitionEventTelemetry {
    pub event_id: CommonFaultTransitionEventId,
    pub severity: FaultSeverity,
    pub action: CommonFaultTransitionAction,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EngineTimeAuthorityTelemetry {
    pub authority: EngineTimeAuthority,
    pub summary: SyncState,
    pub full_sequential_authorized: bool,
}

impl EngineTimeAuthorityTelemetry {
    pub const fn new(authority: EngineTimeAuthority) -> Self {
        Self {
            authority,
            summary: authority.compatibility_summary(),
            full_sequential_authorized:
                crate::timing_island::engine_time_authorizes_full_sequential(authority),
        }
    }

    pub const fn legacy(sync_state: SyncState) -> Self {
        Self::new(crate::timing_island::legacy::sync_state_authority(
            sync_state,
        ))
    }

    pub const fn source(self) -> ecu_domain::AbsoluteTimeAuthority {
        self.authority.absolute
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct TelemetryFrame {
    pub snapshot: SensorSnapshot,
    pub profile_id: ProfileId,
    pub ignition_profile_id: IgnitionProfileId,
    pub ignition_profile_mode: IgnitionProfileMode,
    pub pin_map_id: PinMapId,
    pub runtime_build_id: RuntimeBuildId,
    pub control_mode: ControlMode,
    pub fault_code: FaultCode,
    pub fault_severity: FaultSeverity,
    pub ignition_advance: Degrees10,
    pub dwell_us: DwellUs,
    pub injector_pulse_width_us: PulseWidthUs,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum IgnitionProfileMode {
    #[default]
    WastedSpark,
    Disabled,
    SequentialCop,
    SequentialCopAuthorityBlocked,
}

impl TelemetryFrame {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        snapshot: SensorSnapshot,
        profile_id: ProfileId,
        ignition_profile_id: IgnitionProfileId,
        ignition_profile_mode: IgnitionProfileMode,
        pin_map_id: PinMapId,
        runtime_build_id: RuntimeBuildId,
        control_mode: ControlMode,
        fault_code: FaultCode,
        fault_severity: FaultSeverity,
        ignition_advance: Degrees10,
        dwell_us: DwellUs,
        injector_pulse_width_us: PulseWidthUs,
    ) -> Self {
        Self {
            snapshot,
            profile_id,
            ignition_profile_id,
            ignition_profile_mode,
            pin_map_id,
            runtime_build_id,
            control_mode,
            fault_code,
            fault_severity,
            ignition_advance,
            dwell_us,
            injector_pulse_width_us,
        }
    }
}
