use crate::compat::StepInputs;
use crate::ingress::RuntimeAuthorityError;
use crate::observations::DifferentialInputSnapshot;
use crate::{
    runtime_full_sequential_authorized, Action, ActionBatch, CalibrationState, ControlInputs,
    ControlPlan, ControlState, DecoderObservation, EngineState, FaultState, RuntimeEngineMode,
    RuntimeLegacyCutFlags, RuntimeOutputProfile, RuntimeSemanticAfrOverride,
    RuntimeSemanticCalibration, RuntimeSemanticEngineMode, RuntimeSemanticInputSnapshot,
    RuntimeSemanticState, RuntimeSnapshot, StepResult, TorqueObservations, ValidatedInputs,
    RUNTIME_ACTION_CAP, RUNTIME_AUX_COMMAND_CAP,
};
use ecu_board_api::{AuxCommand, AuxCommandBatch, AuxOutput, AuxValue, OutputLevel};
use ecu_calibration::CalibrationSnapshot;
use ecu_control::{
    AccelerationConfig, AfterStartConfig, BaseFuelModel, DwellConfig, EnrichmentController,
    FuelAfrOverride, FuelEngineMode, FuelInputSnapshot, FuelIntent, FuelLoadSource,
    FuelObservations, IgnitionPlanner, LambdaTrimConfig, LambdaTrimPlanner, StartupConfig,
    TorqueArbiter, TorqueLimitReason, WarmupConfig,
};
use ecu_domain::{
    AbsoluteTimeAuthority, CancelReason, ChannelId, ControlMode, CrankSyncState, Degrees10,
    DwellUs, EnginePhase, EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10, Lambda100, Micros,
    PhaseSyncState, PulseWidthUs, Rpm, SyncState,
};
use ecu_scheduler::{
    CrankSnapshot, ExclusiveChannel, FuelOutputProfile, FuelPlan, IgnitionScheduler, InjectionPlan,
    InjectionScheduler, OutputGroup, SchedulerState, SparkOutputProfile, SparkPlan,
    TimedIgnitionPlan, TimedInjectionPlan,
};

use crate::{semantic::runtime_semantic_evaluate_fuel_with_state, FullEcuOutputProfile};

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
    match profile {
        RuntimeOutputProfile::FullEcu(profile) => {
            profile.authority.requires_full_sequential()
                || profile.ignition.phase_required()
                || profile.injection.phase_required()
        }
        _ => false,
    }
}

/// First-pass runtime owner that groups all state shells in one place.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct EngineRuntime {
    pub(crate) engine: EngineState,
    pub(crate) control: ControlState,
    pub(crate) faults: FaultState,
    pub(crate) calibration: CalibrationState,
    pub(crate) scheduler: SchedulerState,
    pub(crate) planners: ControlPlannerState,
    pub(crate) runtime_snapshot: RuntimeSnapshot,
    pub(crate) calibration_snapshot: CalibrationSnapshot,
    pub(crate) output_profile: RuntimeOutputProfile,
    /// Soft rev limiter active flag from last step.
    pub(crate) rev_soft_active: bool,
    /// Hard rev limiter active flag from last step.
    pub(crate) rev_hard_active: bool,
    /// Launch limiter active flag from last step.
    pub(crate) launch_active: bool,
    /// Flat-shift limiter active flag from last step.
    pub(crate) flat_shift_active: bool,
    /// Direct fuel cut request for the next step.
    pub(crate) direct_fuel_cut_request: bool,
    /// Direct spark cut request for the next step.
    pub(crate) direct_spark_cut_request: bool,
    /// Safety latch active flag from last step.
    pub(crate) safety_latched: bool,
    /// Fuel cut active flag from last step.
    pub(crate) fuel_cut: bool,
    /// Spark cut active flag from last step.
    pub(crate) spark_cut: bool,
    /// Last knock intensity ingressed on the runtime step path.
    pub(crate) knock_intensity_x100: u16,
    /// Last semantic knock retard retained by the selected fuel strategy.
    pub(crate) knock_retard_deg10: i16,
}

/// Owned control planner state kept by the runtime.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControlPlannerState {
    pub fuel_strategy: RuntimeFuelStrategy,
    pub startup_config: StartupConfig,
    pub warmup_config: WarmupConfig,
    pub after_start_config: AfterStartConfig,
    pub acceleration_config: AccelerationConfig,
    pub lambda_config: LambdaTrimConfig,
    pub dwell_config: DwellConfig,
    pub enrichment: EnrichmentController,
    pub lambda: LambdaTrimPlanner,
    pub torque: TorqueArbiter,
    pub ignition: IgnitionPlanner,
}

impl Default for ControlPlannerState {
    fn default() -> Self {
        Self {
            fuel_strategy: RuntimeFuelStrategy::default(),
            startup_config: StartupConfig::default(),
            warmup_config: WarmupConfig::default(),
            after_start_config: AfterStartConfig::default(),
            acceleration_config: AccelerationConfig::default(),
            lambda_config: LambdaTrimConfig::default(),
            dwell_config: DwellConfig::default(),
            enrichment: EnrichmentController::new(),
            lambda: LambdaTrimPlanner::new(),
            torque: TorqueArbiter::new(),
            ignition: IgnitionPlanner::new(),
        }
    }
}

/// Runtime fuel strategy boundary.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RuntimeFuelStrategy {
    DirectPulseWidthTable(BaseFuelModel),
    SpeedDensityVe {
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    },
    AlphaN {
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    },
    Maf {
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    },
}

impl Default for RuntimeFuelStrategy {
    fn default() -> Self {
        Self::DirectPulseWidthTable(BaseFuelModel::default())
    }
}

mod actions;
mod config;
mod control;
mod snapshot;
mod step;
