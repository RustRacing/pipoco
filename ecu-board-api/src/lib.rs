#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]

use ecu_domain::{
    AbsoluteTimeAuthority, ChannelId, ControlMode, CrankSyncState, Degrees10, DwellUs, EnginePhase,
    EngineTimeAuthority, FaultCode, FaultSeverity, Kpa10, Lambda100, Micros, Percent,
    PhaseSyncState, PulseWidthUs, Rpm, SyncState, Ticks,
};

pub trait EcuClock {
    fn now_us(&self) -> Micros;
}

pub trait TriggerEdgeSource<const N: usize> {
    type Error;

    fn drain_edges(&mut self, out: &mut EdgeBatch<N>) -> Result<(), Self::Error>;
}

pub trait SensorSource {
    type Error;

    fn sample(&mut self, now: Micros) -> Result<SensorSnapshot, Self::Error>;
}

pub trait OutputScheduler<const N: usize> {
    type Error;

    fn schedule(&mut self, batch: &OutputTransitionBatch<N>) -> Result<(), Self::Error>;

    fn cancel_all(&mut self) -> Result<(), Self::Error>;

    fn force_safe_state(&mut self);
}

pub trait AuxOutputSink<const N: usize> {
    type Error;

    fn apply_aux(&mut self, batch: &AuxCommandBatch<N>) -> Result<(), Self::Error>;
}

pub trait TelemetrySink {
    type Error;

    fn publish(&mut self, frame: &TelemetryFrame) -> Result<(), Self::Error>;
}

pub trait CalibrationStore {
    type Error;

    fn read_page(&mut self, page: CalibrationPage, out: &mut [u8]) -> Result<usize, Self::Error>;

    fn write_page(&mut self, page: CalibrationPage, bytes: &[u8]) -> Result<(), Self::Error>;
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationPage(u8);

impl CalibrationPage {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum EdgeKind {
    #[default]
    Rising,
    Falling,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum OutputLevel {
    #[default]
    Low,
    High,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum AuxValue {
    #[default]
    Off,
    Duty(Percent),
    Level(OutputLevel),
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum EcuOutput {
    Injector(ChannelId),
    Ignition(ChannelId),
}

impl Default for EcuOutput {
    fn default() -> Self {
        Self::Injector(ChannelId::new(0))
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum AuxOutput {
    VanosIntake,
    IdleValveOpen,
    IdleValveClose,
    FuelPump,
    Fan,
    TachOut,
    Cel,
    Boost,
    Disa,
    SpareRelay(u8),
    Digital(ChannelId),
    Pwm(ChannelId),
}

impl Default for AuxOutput {
    fn default() -> Self {
        Self::Digital(ChannelId::new(0))
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct TriggerEdge {
    pub kind: EdgeKind,
    pub at: Ticks,
}

impl TriggerEdge {
    pub const EMPTY: Self = Self {
        kind: EdgeKind::Rising,
        at: Ticks::new(0),
    };

    pub const fn new(kind: EdgeKind, at: Ticks) -> Self {
        Self { kind, at }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct OutputTransition {
    pub output: EcuOutput,
    pub level: OutputLevel,
    pub at: Ticks,
}

impl OutputTransition {
    pub const EMPTY: Self = Self {
        output: EcuOutput::Injector(ChannelId::new(0)),
        level: OutputLevel::Low,
        at: Ticks::new(0),
    };

    pub const fn new(output: EcuOutput, level: OutputLevel, at: Ticks) -> Self {
        Self { output, level, at }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct AuxCommand {
    pub output: AuxOutput,
    pub value: AuxValue,
}

impl AuxCommand {
    pub const EMPTY: Self = Self {
        output: AuxOutput::Digital(ChannelId::new(0)),
        value: AuxValue::Off,
    };

    pub const fn new(output: AuxOutput, value: AuxValue) -> Self {
        Self { output, value }
    }
}

pub const fn absolute_time_authorizes_full_sequential(source: AbsoluteTimeAuthority) -> bool {
    matches!(
        source,
        AbsoluteTimeAuthority::ExpertManual
            | AbsoluteTimeAuthority::CommunityProfile
            | AbsoluteTimeAuthority::CertifiedProfile
            | AbsoluteTimeAuthority::BenchLearned
    )
}

pub const fn engine_time_authorizes_full_sequential(authority: EngineTimeAuthority) -> bool {
    authority.confidence_x1000 <= EngineTimeAuthority::MAX_CONFIDENCE_X1000
        && matches!(authority.crank, CrankSyncState::PrimaryLocked)
        && matches!(authority.phase, PhaseSyncState::CamValidated720)
        && absolute_time_authorizes_full_sequential(authority.absolute)
}

pub const fn legacy_sync_state_authority(sync_state: SyncState) -> EngineTimeAuthority {
    match sync_state {
        SyncState::Unsynced => EngineTimeAuthority::none(),
        SyncState::Syncing => EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::Unknown,
            AbsoluteTimeAuthority::None,
            0,
            0,
        ),
        SyncState::Synced => EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        ),
    }
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
            full_sequential_authorized: engine_time_authorizes_full_sequential(authority),
        }
    }

    pub const fn legacy(sync_state: SyncState) -> Self {
        Self::new(legacy_sync_state_authority(sync_state))
    }

    pub const fn source(self) -> AbsoluteTimeAuthority {
        self.authority.absolute
    }
}

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
            legacy_sync_state_authority(sync_state),
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

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ProfileId(u16);

impl ProfileId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct IgnitionProfileId(u16);

impl IgnitionProfileId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum IgnitionProfileMode {
    #[default]
    WastedSpark,
    SequentialCop,
    SequentialCopAuthorityBlocked,
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PinMapId(u16);

impl PinMapId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct RuntimeBuildId(u32);

impl RuntimeBuildId {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
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

macro_rules! fixed_batch {
    ($name:ident, $item:ty, $empty:expr) => {
        #[repr(C)]
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub struct $name<const N: usize> {
            len: usize,
            items: [$item; N],
        }

        impl<const N: usize> $name<N> {
            pub const fn new() -> Self {
                Self {
                    len: 0,
                    items: [$empty; N],
                }
            }

            pub const fn capacity(&self) -> usize {
                N
            }

            pub const fn len(&self) -> usize {
                self.len
            }

            pub const fn is_empty(&self) -> bool {
                self.len == 0
            }

            pub fn clear(&mut self) {
                self.len = 0;
            }

            pub fn push(&mut self, item: $item) -> Result<(), $item> {
                if self.len == N {
                    return Err(item);
                }

                self.items[self.len] = item;
                self.len += 1;
                Ok(())
            }

            pub fn iter(&self) -> core::slice::Iter<'_, $item> {
                self.items[..self.len].iter()
            }

            pub fn as_slice(&self) -> &[$item] {
                &self.items[..self.len]
            }
        }

        impl<const N: usize> Default for $name<N> {
            fn default() -> Self {
                Self::new()
            }
        }
    };
}

fixed_batch!(EdgeBatch, TriggerEdge, TriggerEdge::EMPTY);
fixed_batch!(
    OutputTransitionBatch,
    OutputTransition,
    OutputTransition::EMPTY
);
fixed_batch!(AuxCommandBatch, AuxCommand, AuxCommand::EMPTY);

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::needs_drop;

    #[test]
    fn edge_batch_push_until_full_then_rejects() {
        let mut batch = EdgeBatch::<4>::new();

        for idx in 0..4 {
            assert!(batch
                .push(TriggerEdge::new(EdgeKind::Rising, Ticks::new(idx)))
                .is_ok());
        }

        assert_eq!(batch.len(), 4);
        assert_eq!(
            batch.push(TriggerEdge::new(EdgeKind::Falling, Ticks::new(99))),
            Err(TriggerEdge::new(EdgeKind::Falling, Ticks::new(99)))
        );

        let mut seen = 0;
        for (idx, edge) in batch.iter().enumerate() {
            assert_eq!(edge.kind, EdgeKind::Rising);
            assert_eq!(edge.at, Ticks::new(idx as u32));
            seen += 1;
        }

        assert_eq!(seen, 4);
    }

    #[test]
    fn output_transition_batch_clear_resets_len_but_preserves_capacity() {
        let mut batch = OutputTransitionBatch::<2>::new();

        assert!(batch
            .push(OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(1)),
                OutputLevel::High,
                Ticks::new(10),
            ))
            .is_ok());
        assert!(batch
            .push(OutputTransition::new(
                EcuOutput::Ignition(ChannelId::new(2)),
                OutputLevel::Low,
                Ticks::new(11),
            ))
            .is_ok());

        assert!(batch.push(OutputTransition::EMPTY).is_err());
        batch.clear();

        assert!(batch.is_empty());
        assert_eq!(batch.capacity(), 2);

        assert!(batch
            .push(OutputTransition::new(
                EcuOutput::Injector(ChannelId::new(3)),
                OutputLevel::High,
                Ticks::new(12),
            ))
            .is_ok());
        assert_eq!(batch.len(), 1);
        assert_eq!(batch.as_slice()[0].at, Ticks::new(12));
    }

    #[test]
    fn core_types_are_copy_and_do_not_need_drop() {
        fn assert_copy<T: Copy>() {}

        assert_copy::<TriggerEdge>();
        assert_copy::<OutputTransition>();
        assert_copy::<AuxCommand>();
        assert_copy::<EngineTimeAuthorityTelemetry>();
        assert_copy::<SensorSnapshot>();
        assert_copy::<TelemetryFrame>();
        assert_copy::<EdgeBatch<4>>();
        assert_copy::<OutputTransitionBatch<4>>();
        assert_copy::<AuxCommandBatch<4>>();

        assert!(!needs_drop::<TriggerEdge>());
        assert!(!needs_drop::<OutputTransition>());
        assert!(!needs_drop::<AuxCommand>());
        assert!(!needs_drop::<EngineTimeAuthorityTelemetry>());
        assert!(!needs_drop::<SensorSnapshot>());
        assert!(!needs_drop::<ProfileId>());
        assert!(!needs_drop::<IgnitionProfileId>());
        assert!(!needs_drop::<PinMapId>());
        assert!(!needs_drop::<RuntimeBuildId>());
        assert!(!needs_drop::<IgnitionProfileMode>());
        assert!(!needs_drop::<TelemetryFrame>());
        assert!(!needs_drop::<EdgeBatch<4>>());
        assert!(!needs_drop::<OutputTransitionBatch<4>>());
        assert!(!needs_drop::<AuxCommandBatch<4>>());
    }

    #[test]
    fn aux_output_named_variants_construct_and_round_trip() {
        let variants = [
            AuxOutput::VanosIntake,
            AuxOutput::IdleValveOpen,
            AuxOutput::IdleValveClose,
            AuxOutput::FuelPump,
            AuxOutput::Fan,
            AuxOutput::TachOut,
            AuxOutput::Cel,
            AuxOutput::Boost,
            AuxOutput::Disa,
            AuxOutput::SpareRelay(7),
            AuxOutput::Digital(ChannelId::new(9)),
            AuxOutput::Pwm(ChannelId::new(10)),
        ];

        let mut batch = AuxCommandBatch::<12>::new();
        for (idx, output) in variants.into_iter().enumerate() {
            assert!(batch
                .push(AuxCommand::new(
                    output,
                    if idx % 2 == 0 {
                        AuxValue::Off
                    } else {
                        AuxValue::Level(OutputLevel::High)
                    },
                ))
                .is_ok());
        }

        assert_eq!(batch.len(), 12);
        assert_eq!(batch.as_slice()[9].output, AuxOutput::SpareRelay(7));
        assert_eq!(
            batch.as_slice()[10].output,
            AuxOutput::Digital(ChannelId::new(9))
        );
        assert_eq!(
            batch.as_slice()[11].output,
            AuxOutput::Pwm(ChannelId::new(10))
        );
    }

    #[test]
    fn telemetry_frame_identity_fields_round_trip() {
        let snapshot = SensorSnapshot::new(
            Micros::new(123),
            Rpm::new(456),
            Kpa10::new(789),
            Percent::new(12),
            34,
            56,
            12_345,
            Lambda100::new(98),
            SyncState::Synced,
            EnginePhase::Running,
        );

        let frame = TelemetryFrame::new(
            snapshot,
            ProfileId::new(11),
            IgnitionProfileId::new(22),
            IgnitionProfileMode::SequentialCop,
            PinMapId::new(33),
            RuntimeBuildId::new(44),
            ControlMode::ClosedLoop,
            FaultCode::default(),
            FaultSeverity::default(),
            Degrees10::new(15),
            DwellUs::new(2500),
            PulseWidthUs::new(3700),
        );

        assert_eq!(frame.snapshot, snapshot);
        assert_eq!(frame.profile_id, ProfileId::new(11));
        assert_eq!(frame.ignition_profile_id, IgnitionProfileId::new(22));
        assert_eq!(
            frame.ignition_profile_mode,
            IgnitionProfileMode::SequentialCop
        );
        assert_eq!(frame.pin_map_id, PinMapId::new(33));
        assert_eq!(frame.runtime_build_id, RuntimeBuildId::new(44));
        assert_eq!(frame.control_mode, ControlMode::ClosedLoop);
        assert_eq!(
            frame.snapshot.engine_time.source(),
            AbsoluteTimeAuthority::GeometryOnly
        );
        assert!(!frame.snapshot.engine_time.full_sequential_authorized);
    }

    #[test]
    fn telemetry_reports_explicit_engine_time_authority_source_and_gate() {
        let authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::ExpertManual,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );
        let snapshot = SensorSnapshot::new_with_engine_time_authority(
            Micros::new(123),
            Rpm::new(456),
            Kpa10::new(789),
            Percent::new(12),
            34,
            56,
            12_345,
            Lambda100::new(98),
            authority,
            EnginePhase::Running,
        );

        assert_eq!(snapshot.sync_state, SyncState::Synced);
        assert_eq!(snapshot.engine_time.summary, SyncState::Synced);
        assert_eq!(
            snapshot.engine_time.source(),
            AbsoluteTimeAuthority::ExpertManual
        );
        assert!(snapshot.engine_time.full_sequential_authorized);
    }

    #[test]
    fn geometry_only_authority_does_not_authorize_full_sequential() {
        let authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );
        let telemetry = EngineTimeAuthorityTelemetry::new(authority);

        assert_eq!(telemetry.summary, SyncState::Synced);
        assert_eq!(telemetry.source(), AbsoluteTimeAuthority::GeometryOnly);
        assert!(!telemetry.full_sequential_authorized);
    }
}
