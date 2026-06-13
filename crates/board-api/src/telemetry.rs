//! Telemetry DTOs.

use crate::capabilities::{IgnitionProfileId, PinMapId, ProfileId, RuntimeBuildId};
use crate::sensors::SensorSnapshot;
use ecu_domain::{
    ControlMode, Degrees10, DwellUs, EngineTimeAuthority, FaultCode, FaultSeverity, PulseWidthUs,
    SyncState,
};

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
