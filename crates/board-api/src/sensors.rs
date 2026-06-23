//! Logical board sensor snapshot types.

use crate::telemetry::EngineTimeAuthorityTelemetry;
use ecu_domain::{
    Degrees10, EnginePhase, EngineTimeAuthority, Kpa10, Lambda100, MassAirFlowX100, Micros,
    Percent, Rpm, SyncState, VehicleSpeedKph10,
};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SensorSnapshot {
    pub now_us: Micros,
    pub rpm: Rpm,
    pub map: Kpa10,
    pub throttle: Percent,
    pub coolant_temp_c10: i16,
    pub intake_temp_c10: i16,
    pub battery_mv: u16,
    pub lambda: Lambda100,
    pub sync_state: SyncState,
    pub engine_time: EngineTimeAuthorityTelemetry,
    pub engine_phase: EnginePhase,
}

impl SensorSnapshot {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        now_us: Micros,
        rpm: Rpm,
        map: Kpa10,
        throttle: Percent,
        coolant_temp_c10: i16,
        intake_temp_c10: i16,
        battery_mv: u16,
        lambda: Lambda100,
        sync_state: SyncState,
        engine_phase: EnginePhase,
    ) -> Self {
        Self::new_with_engine_time_authority(
            now_us,
            rpm,
            map,
            throttle,
            coolant_temp_c10,
            intake_temp_c10,
            battery_mv,
            lambda,
            crate::timing_island::legacy::sync_state_authority(sync_state),
            engine_phase,
        )
    }

    #[allow(clippy::too_many_arguments)]
    pub const fn new_with_engine_time_authority(
        now_us: Micros,
        rpm: Rpm,
        map: Kpa10,
        throttle: Percent,
        coolant_temp_c10: i16,
        intake_temp_c10: i16,
        battery_mv: u16,
        lambda: Lambda100,
        engine_time_authority: EngineTimeAuthority,
        engine_phase: EnginePhase,
    ) -> Self {
        let engine_time = EngineTimeAuthorityTelemetry::new(engine_time_authority);
        Self {
            now_us,
            rpm,
            map,
            throttle,
            coolant_temp_c10,
            intake_temp_c10,
            battery_mv,
            lambda,
            sync_state: engine_time.summary,
            engine_time,
            engine_phase,
        }
    }
}

/// Packed validity flags for optional logical board sensor channels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BoardSensorValidityFlags(u8);

impl BoardSensorValidityFlags {
    pub const MAF: u8 = 1 << 0;
    pub const KNOCK: u8 = 1 << 1;
    pub const VEHICLE_SPEED: u8 = 1 << 2;
    pub const LAMBDA: u8 = 1 << 3;
    pub const OIL_PRESSURE: u8 = 1 << 4;
    pub const FUEL_PRESSURE: u8 = 1 << 5;

    pub const fn new(bits: u8) -> Self {
        Self(bits)
    }

    pub const fn from_channels(
        maf_valid: bool,
        knock_valid: bool,
        vehicle_speed_valid: bool,
        lambda_valid: bool,
    ) -> Self {
        let mut bits = 0;
        if maf_valid {
            bits |= Self::MAF;
        }
        if knock_valid {
            bits |= Self::KNOCK;
        }
        if vehicle_speed_valid {
            bits |= Self::VEHICLE_SPEED;
        }
        if lambda_valid {
            bits |= Self::LAMBDA;
        }
        Self(bits)
    }

    pub const fn bits(self) -> u8 {
        self.0
    }

    pub const fn contains(self, bit: u8) -> bool {
        (self.0 & bit) != 0
    }
}

/// Logical board-level sensor snapshot for telemetry and board plumbing.
///
/// This is the board-api contract above raw sensor transports. Lower layers may
/// adapt raw electrical frames into this type, but generic consumers should not
/// depend on those raw frame types.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BoardSensorSnapshot {
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub tps_x100: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub vbatt_mv: u16,
    pub baro_kpa10: Kpa10,
    pub maf_x100: MassAirFlowX100,
    pub knock_x100: ecu_domain::KnockLevelX100,
    pub vehicle_speed_kph10: VehicleSpeedKph10,
    pub cam_phase_deg10: Option<ecu_domain::CamPhaseDeg10>,
    pub lambda_x100: Lambda100,
    pub validity: BoardSensorValidityFlags,
}

/// Logical board sensor snapshot with capture timing needed by trigger adapters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoardSensorSnapshotCapture {
    pub at_us: Micros,
    pub angle_x10: Degrees10,
    pub snapshot: BoardSensorSnapshot,
}

pub trait BoardSensorSnapshotCaptureSource {
    type Error;

    fn next_snapshot_capture(&mut self) -> Result<Option<BoardSensorSnapshotCapture>, Self::Error>;
}

/// Logical capture sample consumed by board adapters and runtime schedulers.
///
/// Raw electrical capture buffers live in `ecu-io`; this type is already above
/// that layer because it carries decoded RPM, load, and crank angle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct CaptureSample {
    pub at_us: Micros,
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
}

pub trait CaptureSampleSource {
    type Error;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error>;
}

pub trait CaptureSink {
    type Error;

    fn capture(&mut self, sample: CaptureSample) -> Result<(), Self::Error>;
}
