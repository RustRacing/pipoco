use crate::plant_bridge::X86PlantBridgeDiagnostics;
use ecu_board_api::{
    AuxCommandBatch, EdgeBatch, EngineTimeAuthorityTelemetry, IgnitionOutputProfile,
    IgnitionProfileId, IgnitionProfileMode, OutputTransitionBatch, PinMapId, ProfileId,
    RuntimeBuildId, RuntimeOutputProfile, SensorSnapshot, SparkOutputMode, TelemetryFrame,
};
use ecu_domain::{CancelReason, Degrees10, Lambda100, Micros, Rpm};
use ecu_runtime::{
    ControlInputs, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs,
};
use ecu_sim_core::prelude::EcuOutputFrame;

use super::{X86_AUX_COMMAND_CAP, X86_OUTPUT_TRANSITION_CAP, X86_TRIGGER_EDGE_CAP};

pub(super) const X86_PROFILE_ID: ProfileId = ProfileId::new(0x50b2);
pub(super) const X86_PIN_MAP_ID: PinMapId = PinMapId::new(23);
pub(super) const X86_RUNTIME_BUILD_ID: RuntimeBuildId = RuntimeBuildId::new(1);

pub(super) fn ignition_profile_id(profile: RuntimeOutputProfile) -> IgnitionProfileId {
    match profile {
        RuntimeOutputProfile::LegacySingleChannel => IgnitionProfileId::new(1),
        RuntimeOutputProfile::InjectionOnly(_) => IgnitionProfileId::new(0),
        RuntimeOutputProfile::IgnitionOnly(spark) => match spark.mode {
            SparkOutputMode::SingleCoil => IgnitionProfileId::new(1),
            SparkOutputMode::WastedSpark => {
                IgnitionProfileId::new(spark.events_per_crank_rev() as u16)
            }
        },
        RuntimeOutputProfile::FullEcu(full) => match full.ignition {
            IgnitionOutputProfile::WastedSpark { coils, .. }
            | IgnitionOutputProfile::CoilOnPlug { coils, .. } => {
                IgnitionProfileId::new(u16::from(coils))
            }
        },
    }
}

pub(super) fn ignition_profile_mode(profile: RuntimeOutputProfile) -> IgnitionProfileMode {
    match profile {
        RuntimeOutputProfile::LegacySingleChannel => IgnitionProfileMode::WastedSpark,
        RuntimeOutputProfile::InjectionOnly(_) => IgnitionProfileMode::Disabled,
        RuntimeOutputProfile::IgnitionOnly(spark) => match spark.mode {
            SparkOutputMode::SingleCoil | SparkOutputMode::WastedSpark => {
                IgnitionProfileMode::WastedSpark
            }
        },
        RuntimeOutputProfile::FullEcu(full) => match full.ignition {
            IgnitionOutputProfile::WastedSpark { .. } => IgnitionProfileMode::WastedSpark,
            IgnitionOutputProfile::CoilOnPlug { .. } => IgnitionProfileMode::SequentialCop,
        },
    }
}

pub(super) fn baseline_control_inputs(now_us: Micros, rpm: Rpm) -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us,
            clt_c: 80,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            now_us,
            clt_c: 80,
            just_started: false,
            lambda_valid: true,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(500, 0, 1_000, 1_000, 1_000),
        ignition: IgnitionInputs::new(Degrees10::new(120), 0, 0, 0, false, rpm),
        fuel_sensors: ecu_runtime::FuelSensorInputs::default(),
        knock_intensity_x100: 0,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum X86RuntimeBoardError {
    TriggerEdgeOverflow,
    OutputOverflow,
    AuxOverflow,
    CalibrationOverflow,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct X86RuntimeBoardDiagnostics {
    pub drained_trigger_edges: usize,
    pub scheduled_transition_count: usize,
    pub aux_command_count: usize,
    pub telemetry_publish_count: usize,
    pub cancel_all_count: usize,
    pub force_safe_state_count: usize,
    pub last_cancel_reason: Option<CancelReason>,
    pub engine_time: EngineTimeAuthorityTelemetry,
    pub synced: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86RuntimeTickResult {
    pub now_us: Micros,
    pub trigger_edges: EdgeBatch<X86_TRIGGER_EDGE_CAP>,
    pub sensor_snapshot: SensorSnapshot,
    pub step_result: ecu_runtime::StepResult,
    pub scheduled_outputs: OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
    pub aux_commands: AuxCommandBatch<X86_AUX_COMMAND_CAP>,
    pub telemetry: Option<TelemetryFrame>,
    pub diagnostics: X86RuntimeBoardDiagnostics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct X86RuntimePlantBridgeFrame<const CYL: usize, const MAX_EVENTS: usize> {
    pub ecu_outputs: EcuOutputFrame<CYL, MAX_EVENTS>,
    pub diagnostics: X86PlantBridgeDiagnostics,
}
