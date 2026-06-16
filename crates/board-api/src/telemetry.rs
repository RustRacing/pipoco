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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CommonDiagnosticsTelemetry {
    pub sync_state: CommonSyncTelemetryState,
    pub fault_code: FaultCode,
    pub fault_severity: FaultSeverity,
    pub cancel_reason: CancelReason,
    pub late_event_count: u32,
    pub max_lateness_us: u32,
    pub queue_high_water_mark: u8,
    pub last_drain_count: u8,
    pub active_queue_count: u8,
    pub free_queue_slots: u8,
    pub queue_capacity: u8,
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

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CommonFaultTransitionTelemetry {
    pub changed: bool,
    pub at_us: Micros,
    pub previous_fault: FaultCode,
    pub previous_severity: FaultSeverity,
    pub previous_cancel_reason: CancelReason,
    pub current_fault: FaultCode,
    pub current_severity: FaultSeverity,
    pub current_cancel_reason: CancelReason,
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
