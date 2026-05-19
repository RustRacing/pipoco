#![no_std]
#![forbid(unsafe_code)]

use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, EngineTimeAuthority, Micros, PhaseSyncState,
    Rpm, Ticks,
};

pub const ENGINE_CYCLE_DEGREES10: i16 = 7200;

/// Opaque identifier for non-uniform or OEM trigger families.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PatternId(u16);

impl PatternId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }

    pub const fn is_valid(self) -> bool {
        self.0 != 0
    }
}

/// Primary trigger wheel family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TriggerPattern {
    MissingTooth {
        nominal_teeth: u8,
        missing_teeth: u8,
    },
    BasicDistributor,
    DualWheel {
        primary_teeth: u8,
    },
    NonUniform {
        pattern_id: PatternId,
    },
}

impl TriggerPattern {
    pub const fn validate(self) -> Result<(), TriggerValidationError> {
        match self {
            Self::MissingTooth {
                nominal_teeth,
                missing_teeth,
            } => validate_missing_tooth(nominal_teeth, missing_teeth),
            Self::BasicDistributor => Ok(()),
            Self::DualWheel { primary_teeth } => {
                if primary_teeth == 0 {
                    return Err(TriggerValidationError::PrimaryTeethZero);
                }

                Ok(())
            }
            Self::NonUniform { pattern_id } => {
                if !pattern_id.is_valid() {
                    return Err(TriggerValidationError::PatternIdZero);
                }

                Ok(())
            }
        }
    }

    pub const fn observed_primary_teeth(self) -> Option<u8> {
        match self {
            Self::MissingTooth {
                nominal_teeth,
                missing_teeth,
            } => {
                if validate_missing_tooth(nominal_teeth, missing_teeth).is_err() {
                    None
                } else {
                    Some(nominal_teeth - missing_teeth)
                }
            }
            Self::DualWheel { primary_teeth } => {
                if primary_teeth == 0 {
                    None
                } else {
                    Some(primary_teeth)
                }
            }
            Self::BasicDistributor | Self::NonUniform { .. } => None,
        }
    }
}

/// Primary wheel speed relative to crankshaft speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TriggerSpeed {
    #[default]
    Crank,
    Cam,
}

/// Electrical edge selected for trigger capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TriggerEdge {
    #[default]
    Rising,
    Falling,
}

/// Instantaneous electrical level sampled from a trigger input.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TriggerLevel {
    #[default]
    Low,
    High,
}

impl TriggerLevel {
    pub const fn matches_polarity(self, polarity: PollLevelPolarity) -> bool {
        matches!(
            (self, polarity),
            (Self::High, PollLevelPolarity::ActiveHigh) | (Self::Low, PollLevelPolarity::ActiveLow)
        )
    }
}

/// Secondary trigger source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SecondaryTriggerMode {
    #[default]
    None,
    SingleToothCam,
    FourMinusOneCam,
    PollLevel,
    MultiToothCam {
        teeth: u8,
    },
    OemPattern {
        pattern_id: PatternId,
    },
}

impl SecondaryTriggerMode {
    pub const fn validate(self) -> Result<(), TriggerValidationError> {
        match self {
            Self::None | Self::SingleToothCam | Self::FourMinusOneCam | Self::PollLevel => Ok(()),
            Self::MultiToothCam { teeth } => {
                if teeth == 0 {
                    return Err(TriggerValidationError::SecondaryTeethZero);
                }

                Ok(())
            }
            Self::OemPattern { pattern_id } => {
                if !pattern_id.is_valid() {
                    return Err(TriggerValidationError::PatternIdZero);
                }

                Ok(())
            }
        }
    }
}

/// Polarity used when a secondary input is read as a level instead of edges.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PollLevelPolarity {
    #[default]
    ActiveHigh,
    ActiveLow,
}

/// Secondary trigger configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct SecondaryTriggerProfile {
    pub mode: SecondaryTriggerMode,
    pub edge: TriggerEdge,
    pub poll_level: PollLevelPolarity,
}

impl Default for SecondaryTriggerProfile {
    fn default() -> Self {
        Self {
            mode: SecondaryTriggerMode::None,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        }
    }
}

impl SecondaryTriggerProfile {
    pub const fn validate(self) -> Result<(), TriggerValidationError> {
        self.mode.validate()
    }
}

/// Source and value of the primary tooth-1 angle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TriggerAngleAuthority {
    #[default]
    Unknown,
    ExpertManual(Degrees10),
    CommunityProfile(Degrees10),
    CertifiedProfile(Degrees10),
    BenchLearned(Degrees10),
}

impl TriggerAngleAuthority {
    pub const fn absolute_authority(self) -> AbsoluteTimeAuthority {
        match self {
            Self::Unknown => AbsoluteTimeAuthority::None,
            Self::ExpertManual(_) => AbsoluteTimeAuthority::ExpertManual,
            Self::CommunityProfile(_) => AbsoluteTimeAuthority::CommunityProfile,
            Self::CertifiedProfile(_) => AbsoluteTimeAuthority::CertifiedProfile,
            Self::BenchLearned(_) => AbsoluteTimeAuthority::BenchLearned,
        }
    }

    pub const fn angle_deg10(self) -> Option<Degrees10> {
        match self {
            Self::Unknown => None,
            Self::ExpertManual(angle)
            | Self::CommunityProfile(angle)
            | Self::CertifiedProfile(angle)
            | Self::BenchLearned(angle) => Some(angle),
        }
    }

    pub const fn normalized_angle_deg10(self) -> Option<Degrees10> {
        match self.angle_deg10() {
            Some(angle) => Some(normalize_engine_cycle_deg10(angle)),
            None => None,
        }
    }
}

/// Trigger edge filter strength.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum TriggerFilter {
    #[default]
    Off,
    Weak,
    Medium,
    Aggressive,
}

/// Runtime resynchronization behavior.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ResyncPolicy {
    #[default]
    Disabled,
    OnSyncLoss,
    EveryCycle,
}

/// Startup policy before outputs can consume engine-time authority.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct StartupSyncPolicy {
    pub skip_revolutions: u8,
    pub require_full_cycle: bool,
}

/// Fixed latency compensation inputs for engine-time calculations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EngineTimeLatency {
    pub primary_edge_delay_us: Micros,
    pub secondary_edge_delay_us: Micros,
    pub output_schedule_delay_us: Micros,
}

impl StartupSyncPolicy {
    pub const fn authority_ready(self, completed_revolutions: u8) -> bool {
        completed_revolutions >= self.skip_revolutions
            && (!self.require_full_cycle || completed_revolutions > 0)
    }
}

impl EngineTimeLatency {
    pub const fn total_us(self) -> u32 {
        self.primary_edge_delay_us
            .get()
            .saturating_add(self.secondary_edge_delay_us.get())
            .saturating_add(self.output_schedule_delay_us.get())
    }
}

/// Full trigger profile carried by later board-profile and calibration slices.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TriggerProfile {
    pub pattern: TriggerPattern,
    pub primary_speed: TriggerSpeed,
    pub primary_edge: TriggerEdge,
    pub secondary: SecondaryTriggerProfile,
    pub trigger_angle_atdc_deg10: TriggerAngleAuthority,
    pub tooth_angle_multiplier: u8,
    pub filter: TriggerFilter,
    pub resync: ResyncPolicy,
    pub startup: StartupSyncPolicy,
    pub latency: EngineTimeLatency,
}

impl TriggerProfile {
    pub const fn validate(self) -> Result<(), TriggerValidationError> {
        if let Err(error) = self.pattern.validate() {
            return Err(error);
        }

        if let Err(error) = self.secondary.validate() {
            return Err(error);
        }

        if self.tooth_angle_multiplier == 0 {
            return Err(TriggerValidationError::ToothAngleMultiplierZero);
        }

        Ok(())
    }

    pub const fn declared_absolute_authority(self) -> AbsoluteTimeAuthority {
        self.trigger_angle_atdc_deg10.absolute_authority()
    }

    pub const fn startup_authority_ready(self, completed_revolutions: u8) -> bool {
        self.startup.authority_ready(completed_revolutions)
    }

    pub const fn latency_us(self) -> u32 {
        self.latency.total_us()
    }

    pub const fn missing_tooth_decoder_config(
        self,
        minimum_edge_interval: Ticks,
        gap_ratio_threshold_x1000: u16,
    ) -> Result<MissingToothDecoderConfig, TriggerValidationError> {
        match MissingToothDecoderConfig::from_profile(
            self,
            minimum_edge_interval,
            gap_ratio_threshold_x1000,
        ) {
            Some(config) => Ok(config),
            None => Err(TriggerValidationError::UnsupportedTriggerPattern),
        }
    }
}

pub const DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000: u16 = 1500;
const RATIO_SCALE_X1000: u64 = 1000;
const CRANK_REV_DEGREES10: u32 = 3600;
const MICROS_PER_MINUTE: u64 = 60_000_000;

/// Timestamped primary edge captured by board code before decoder ingestion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PrimaryEdgeSample {
    pub timestamp: Ticks,
    pub edge: TriggerEdge,
}

impl PrimaryEdgeSample {
    pub const EMPTY: Self = Self {
        timestamp: Ticks::new(0),
        edge: TriggerEdge::Rising,
    };

    pub const fn new(timestamp: Ticks) -> Self {
        Self::with_edge(timestamp, TriggerEdge::Rising)
    }

    pub const fn with_edge(timestamp: Ticks, edge: TriggerEdge) -> Self {
        Self { timestamp, edge }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PrimaryEdgeBatchError {
    Full,
}

/// Fixed-capacity primary edge batch for allocation-free ISR-to-decoder handoff.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PrimaryEdgeBatch<const CAPACITY: usize> {
    edges: [PrimaryEdgeSample; CAPACITY],
    len: usize,
}

impl<const CAPACITY: usize> Default for PrimaryEdgeBatch<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> PrimaryEdgeBatch<CAPACITY> {
    pub const fn new() -> Self {
        Self {
            edges: [PrimaryEdgeSample::EMPTY; CAPACITY],
            len: 0,
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn capacity(&self) -> usize {
        CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub const fn is_full(&self) -> bool {
        self.len == CAPACITY
    }

    pub fn push(&mut self, edge: PrimaryEdgeSample) -> Result<(), PrimaryEdgeBatchError> {
        if self.is_full() {
            return Err(PrimaryEdgeBatchError::Full);
        }

        self.edges[self.len] = edge;
        self.len += 1;
        Ok(())
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn as_slice(&self) -> &[PrimaryEdgeSample] {
        &self.edges[..self.len]
    }
}

/// Missing-tooth decoder setup derived from an expert trigger profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MissingToothDecoderConfig {
    pub nominal_teeth: u8,
    pub missing_teeth: u8,
    pub primary_speed: TriggerSpeed,
    pub primary_edge: TriggerEdge,
    pub secondary: SecondaryTriggerProfile,
    pub trigger_angle_atdc_deg10: TriggerAngleAuthority,
    pub tooth_angle_multiplier: u8,
    pub minimum_edge_interval: Ticks,
    pub gap_ratio_threshold_x1000: u16,
}

impl MissingToothDecoderConfig {
    pub const fn new(
        nominal_teeth: u8,
        missing_teeth: u8,
        trigger_angle_atdc_deg10: TriggerAngleAuthority,
    ) -> Self {
        Self {
            nominal_teeth,
            missing_teeth,
            primary_speed: TriggerSpeed::Crank,
            primary_edge: TriggerEdge::Rising,
            secondary: SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::None,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            trigger_angle_atdc_deg10,
            tooth_angle_multiplier: 1,
            minimum_edge_interval: Ticks::new(0),
            gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        }
    }

    pub const fn from_profile(
        profile: TriggerProfile,
        minimum_edge_interval: Ticks,
        gap_ratio_threshold_x1000: u16,
    ) -> Option<Self> {
        match profile.pattern {
            TriggerPattern::MissingTooth {
                nominal_teeth,
                missing_teeth,
            } => Some(Self {
                nominal_teeth,
                missing_teeth,
                primary_speed: profile.primary_speed,
                primary_edge: profile.primary_edge,
                secondary: profile.secondary,
                trigger_angle_atdc_deg10: profile.trigger_angle_atdc_deg10,
                tooth_angle_multiplier: profile.tooth_angle_multiplier,
                minimum_edge_interval,
                gap_ratio_threshold_x1000,
            }),
            TriggerPattern::BasicDistributor
            | TriggerPattern::DualWheel { .. }
            | TriggerPattern::NonUniform { .. } => None,
        }
    }

    const fn normalized_gap_threshold_x1000(self) -> u16 {
        if self.gap_ratio_threshold_x1000 == 0 {
            DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000
        } else {
            self.gap_ratio_threshold_x1000
        }
    }
}

/// Profile-aware missing-tooth decoder that overlays startup and latency policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ProfiledMissingToothDecoder {
    decoder: MissingToothDecoder,
    profile: TriggerProfile,
    revolutions_since_lock: u8,
    saw_primary_gap: bool,
}

impl ProfiledMissingToothDecoder {
    pub fn try_new(
        profile: TriggerProfile,
        minimum_edge_interval: Ticks,
        gap_ratio_threshold_x1000: u16,
    ) -> Result<Self, TriggerValidationError> {
        let config = profile
            .missing_tooth_decoder_config(minimum_edge_interval, gap_ratio_threshold_x1000)?;

        Ok(Self {
            decoder: MissingToothDecoder::try_new(config)?,
            profile,
            revolutions_since_lock: 0,
            saw_primary_gap: false,
        })
    }

    pub fn ingest_primary_edge(
        &mut self,
        timestamp: Ticks,
    ) -> Result<MissingToothDecoderEvent, SyncLossReason> {
        let event = match self.decoder.ingest_primary_edge(timestamp) {
            Ok(event) => event,
            Err(reason) => {
                self.revolutions_since_lock = 0;
                self.saw_primary_gap = false;
                return Err(reason);
            }
        };

        match event {
            MissingToothDecoderEvent::Gap { .. } => {
                if self.saw_primary_gap {
                    self.revolutions_since_lock = self.revolutions_since_lock.saturating_add(1);
                } else {
                    self.saw_primary_gap = true;
                }
            }
            MissingToothDecoderEvent::FirstEdge | MissingToothDecoderEvent::Searching => {
                if !self.decoder.is_primary_locked() {
                    self.revolutions_since_lock = 0;
                    self.saw_primary_gap = false;
                }
            }
            MissingToothDecoderEvent::IgnoredByFilter | MissingToothDecoderEvent::Tooth { .. } => {}
        }

        Ok(event)
    }

    pub fn ingest_secondary_edge(&mut self, edge: TriggerEdge) -> Result<(), SyncLossReason> {
        self.decoder.ingest_secondary_edge(edge)
    }

    pub fn ingest_secondary_level(&mut self, level: TriggerLevel) -> Result<(), SyncLossReason> {
        self.decoder.ingest_secondary_level(level)
    }

    pub fn crank_angle_at(&self, timestamp: Ticks) -> Option<Degrees10> {
        if !self.decoder.is_primary_locked() {
            return None;
        }

        let observed_teeth = self.profile.pattern.observed_primary_teeth()?;
        let rev_ticks = self
            .decoder
            .last_normal_interval()
            .get()
            .saturating_mul(u32::from(observed_teeth));
        if rev_ticks == 0 {
            return self
                .profile
                .trigger_angle_atdc_deg10
                .normalized_angle_deg10();
        }

        let trigger_angle = self
            .profile
            .trigger_angle_atdc_deg10
            .normalized_angle_deg10()?;
        let latency_us = self.profile.latency_us();
        let last_gap_timestamp = self.decoder.last_edge_timestamp()?;
        let compensated_now = timestamp.get().saturating_sub(latency_us);
        if compensated_now <= last_gap_timestamp.get() {
            return Some(trigger_angle);
        }

        let elapsed = u64::from(compensated_now - last_gap_timestamp.get());
        let revolutions =
            elapsed.saturating_mul(u64::from(CRANK_REV_DEGREES10)) / u64::from(rev_ticks);
        Some(normalize_engine_cycle_deg10_i32(
            i32::from(trigger_angle.get()) + revolutions as i32,
        ))
    }

    pub fn rpm(&self) -> Rpm {
        self.decoder.observation().primary_rpm
    }

    pub fn synced(&self) -> bool {
        self.decoder.is_primary_locked()
    }

    pub fn reset_sync(&mut self) {
        self.decoder.reset();
        self.revolutions_since_lock = 0;
        self.saw_primary_gap = false;
    }

    pub fn authority(&self) -> EngineTimeAuthority {
        let authority = self.decoder.diagnostics().authority;
        if !authority.has_primary_lock() {
            return authority;
        }

        if self
            .profile
            .startup_authority_ready(self.revolutions_since_lock)
        {
            return authority;
        }

        EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::None,
            500,
            authority.sync_loss_count,
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MissingToothDecoderEvent {
    FirstEdge,
    IgnoredByFilter,
    Searching,
    Tooth { current_tooth: u8 },
    Gap { current_tooth: u8 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PrimaryEdgeIngestion {
    pub processed_edges: u8,
    pub ignored_edges: u8,
    pub last_event: Option<MissingToothDecoderEvent>,
    pub last_sync_loss: Option<SyncLossReason>,
}

/// Allocation-free missing-tooth decoder using integer interval ratios.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MissingToothDecoder {
    config: MissingToothDecoderConfig,
    observed_teeth: u8,
    last_edge_timestamp: Option<Ticks>,
    previous_interval: Ticks,
    last_normal_interval: Ticks,
    current_tooth: u8,
    teeth_since_gap: u8,
    primary_locked: bool,
    phase_validated: bool,
    secondary_seen_since_gap: bool,
    cam_seen: bool,
    diagnostics: TriggerDiagnostics,
}

impl MissingToothDecoder {
    pub const fn try_new(
        config: MissingToothDecoderConfig,
    ) -> Result<Self, TriggerValidationError> {
        if let Err(error) = validate_missing_tooth(config.nominal_teeth, config.missing_teeth) {
            return Err(error);
        }

        if config.tooth_angle_multiplier == 0 {
            return Err(TriggerValidationError::ToothAngleMultiplierZero);
        }

        Ok(Self {
            observed_teeth: config.nominal_teeth - config.missing_teeth,
            config,
            last_edge_timestamp: None,
            previous_interval: Ticks::new(0),
            last_normal_interval: Ticks::new(0),
            current_tooth: 0,
            teeth_since_gap: 0,
            primary_locked: false,
            phase_validated: false,
            secondary_seen_since_gap: false,
            cam_seen: false,
            diagnostics: TriggerDiagnostics::empty(),
        })
    }

    pub fn reset(&mut self) {
        self.last_edge_timestamp = None;
        self.previous_interval = Ticks::new(0);
        self.last_normal_interval = Ticks::new(0);
        self.current_tooth = 0;
        self.teeth_since_gap = 0;
        self.primary_locked = false;
        self.phase_validated = false;
        self.secondary_seen_since_gap = false;
        self.cam_seen = false;
        self.diagnostics = TriggerDiagnostics::empty();
    }

    pub const fn observation(&self) -> DecoderObservation {
        self.diagnostics.observation
    }

    pub const fn diagnostics(&self) -> TriggerDiagnostics {
        self.diagnostics
    }

    pub const fn is_primary_locked(&self) -> bool {
        self.primary_locked
    }

    pub const fn last_normal_interval(&self) -> Ticks {
        self.last_normal_interval
    }

    pub const fn last_edge_timestamp(&self) -> Option<Ticks> {
        self.last_edge_timestamp
    }

    pub fn ingest_primary_edge(
        &mut self,
        timestamp: Ticks,
    ) -> Result<MissingToothDecoderEvent, SyncLossReason> {
        self.ingest_primary_edge_with_kind(timestamp, self.config.primary_edge)
    }

    pub fn ingest_primary_edge_with_kind(
        &mut self,
        timestamp: Ticks,
        edge: TriggerEdge,
    ) -> Result<MissingToothDecoderEvent, SyncLossReason> {
        if edge != self.config.primary_edge {
            self.diagnostics.last_sync_loss = Some(SyncLossReason::UnexpectedPrimaryEdge);
            if !self.primary_locked {
                self.set_searching_authority();
            }
            return Err(SyncLossReason::UnexpectedPrimaryEdge);
        }

        let Some(last_timestamp) = self.last_edge_timestamp else {
            self.last_edge_timestamp = Some(timestamp);
            self.set_searching_authority();
            return Ok(MissingToothDecoderEvent::FirstEdge);
        };

        let interval = elapsed_ticks(timestamp, last_timestamp);
        if interval.get() < self.config.minimum_edge_interval.get() {
            self.diagnostics.observation.crank_angle_deg10 = self.crank_angle_at(timestamp);
            return Ok(MissingToothDecoderEvent::IgnoredByFilter);
        }

        if self.previous_interval.get() == 0 || interval.get() == 0 {
            self.last_edge_timestamp = Some(timestamp);
            self.previous_interval = interval;
            self.last_normal_interval = interval;
            self.teeth_since_gap = self.teeth_since_gap.saturating_add(1);
            self.update_observation(timestamp, interval, 0);
            self.set_searching_authority();
            return Ok(MissingToothDecoderEvent::Searching);
        }

        let (is_gap, ratio) = missing_tooth_gap_detected(
            interval,
            self.previous_interval,
            self.config.normalized_gap_threshold_x1000(),
        )?;

        if is_gap {
            self.accept_gap(timestamp, interval, ratio)
        } else {
            self.accept_tooth(timestamp, interval, ratio)
        }
    }

    pub fn ingest_secondary_edge(&mut self, edge: TriggerEdge) -> Result<(), SyncLossReason> {
        match self.config.secondary.mode {
            SecondaryTriggerMode::SingleToothCam => {
                if edge != self.config.secondary.edge {
                    return self.record_secondary_fault(SyncLossReason::PhaseMismatch);
                }

                self.cam_seen = true;
                self.diagnostics.observation.cam_seen = true;
                if self.primary_locked {
                    self.secondary_seen_since_gap = true;
                    self.phase_validated = true;
                    self.set_locked_authority();
                }

                Ok(())
            }
            SecondaryTriggerMode::None
            | SecondaryTriggerMode::PollLevel
            | SecondaryTriggerMode::FourMinusOneCam
            | SecondaryTriggerMode::MultiToothCam { .. }
            | SecondaryTriggerMode::OemPattern { .. } => {
                self.record_secondary_fault(SyncLossReason::ConfigurationInvalid)
            }
        }
    }

    pub fn ingest_secondary_level(&mut self, level: TriggerLevel) -> Result<(), SyncLossReason> {
        match self.config.secondary.mode {
            SecondaryTriggerMode::PollLevel => {
                if !self.primary_locked || self.current_tooth != 1 {
                    return self.record_secondary_fault(SyncLossReason::PhaseMismatch);
                }

                if !level.matches_polarity(self.config.secondary.poll_level) {
                    return self.record_secondary_fault(SyncLossReason::PhaseMismatch);
                }

                self.cam_seen = true;
                self.secondary_seen_since_gap = true;
                self.phase_validated = true;
                self.diagnostics.observation.cam_seen = true;
                self.set_locked_authority();
                Ok(())
            }
            SecondaryTriggerMode::None
            | SecondaryTriggerMode::SingleToothCam
            | SecondaryTriggerMode::FourMinusOneCam
            | SecondaryTriggerMode::MultiToothCam { .. }
            | SecondaryTriggerMode::OemPattern { .. } => {
                self.record_secondary_fault(SyncLossReason::ConfigurationInvalid)
            }
        }
    }

    pub fn ingest_primary_edges<const CAPACITY: usize>(
        &mut self,
        batch: &PrimaryEdgeBatch<CAPACITY>,
    ) -> PrimaryEdgeIngestion {
        let mut result = PrimaryEdgeIngestion::default();
        for edge in batch.as_slice() {
            match self.ingest_primary_edge_with_kind(edge.timestamp, edge.edge) {
                Ok(MissingToothDecoderEvent::IgnoredByFilter) => {
                    result.ignored_edges = result.ignored_edges.saturating_add(1);
                    result.last_event = Some(MissingToothDecoderEvent::IgnoredByFilter);
                }
                Ok(event) => {
                    result.processed_edges = result.processed_edges.saturating_add(1);
                    result.last_event = Some(event);
                }
                Err(reason) => {
                    result.processed_edges = result.processed_edges.saturating_add(1);
                    result.last_sync_loss = Some(reason);
                }
            }
        }

        result
    }

    pub fn crank_angle_at(&self, timestamp: Ticks) -> Option<Degrees10> {
        if !self.primary_locked || self.current_tooth == 0 {
            return None;
        }

        let trigger_angle = self
            .config
            .trigger_angle_atdc_deg10
            .normalized_angle_deg10()?;
        let tooth_angle = self.tooth_angle_deg10();
        let tooth_offset = (u32::from(self.current_tooth.saturating_sub(1)) * tooth_angle) as i32;
        let elapsed_angle = match (self.last_edge_timestamp, self.last_normal_interval.get()) {
            (Some(last_timestamp), interval) if interval > 0 => {
                let elapsed = elapsed_ticks(timestamp, last_timestamp).get() as u64;
                elapsed.saturating_mul(u64::from(tooth_angle)) / u64::from(interval)
            }
            _ => 0,
        };
        let angle = i32::from(trigger_angle.get()) + tooth_offset + elapsed_angle as i32;

        Some(normalize_engine_cycle_deg10_i32(angle))
    }

    fn accept_gap(
        &mut self,
        timestamp: Ticks,
        interval: Ticks,
        ratio_x1000: u16,
    ) -> Result<MissingToothDecoderEvent, SyncLossReason> {
        if self.primary_locked && self.teeth_since_gap != self.observed_teeth {
            self.drop_sync(
                SyncLossReason::WrongToothCount,
                timestamp,
                interval,
                ratio_x1000,
            );
            return Err(SyncLossReason::WrongToothCount);
        }

        self.validate_secondary_before_next_gap();
        self.primary_locked = true;
        self.current_tooth = 1;
        self.teeth_since_gap = 1;
        self.secondary_seen_since_gap = false;
        self.last_edge_timestamp = Some(timestamp);
        self.previous_interval = self.last_normal_interval;
        self.update_observation(timestamp, interval, ratio_x1000);
        self.set_locked_authority();

        Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
    }

    fn accept_tooth(
        &mut self,
        timestamp: Ticks,
        interval: Ticks,
        ratio_x1000: u16,
    ) -> Result<MissingToothDecoderEvent, SyncLossReason> {
        if self.primary_locked {
            if self.current_tooth >= self.observed_teeth {
                self.drop_sync(
                    SyncLossReason::WrongToothCount,
                    timestamp,
                    interval,
                    ratio_x1000,
                );
                return Err(SyncLossReason::WrongToothCount);
            }

            self.current_tooth = self.current_tooth.saturating_add(1);
            self.teeth_since_gap = self.teeth_since_gap.saturating_add(1);
        } else {
            self.teeth_since_gap = self.teeth_since_gap.saturating_add(1);
        }

        self.last_edge_timestamp = Some(timestamp);
        self.previous_interval = interval;
        self.last_normal_interval = interval;
        self.update_observation(timestamp, interval, ratio_x1000);
        if self.primary_locked {
            self.set_locked_authority();
            Ok(MissingToothDecoderEvent::Tooth {
                current_tooth: self.current_tooth,
            })
        } else {
            self.set_searching_authority();
            Ok(MissingToothDecoderEvent::Searching)
        }
    }

    fn drop_sync(
        &mut self,
        reason: SyncLossReason,
        timestamp: Ticks,
        interval: Ticks,
        ratio_x1000: u16,
    ) {
        let sync_loss_count = self.diagnostics.authority.sync_loss_count.saturating_add(1);
        self.primary_locked = false;
        self.phase_validated = false;
        self.secondary_seen_since_gap = false;
        self.cam_seen = false;
        self.current_tooth = 0;
        self.teeth_since_gap = 0;
        self.last_edge_timestamp = Some(timestamp);
        self.previous_interval = Ticks::new(0);
        self.last_normal_interval = Ticks::new(0);
        self.diagnostics.observation = DecoderObservation {
            current_tooth: 0,
            detected_gap_ratio_x1000: ratio_x1000,
            primary_rpm: Rpm::new(0),
            cam_seen: false,
            crank_angle_deg10: None,
            last_primary_interval: interval,
        };
        self.diagnostics.authority = EngineTimeAuthority::new(
            CrankSyncState::SyncLost,
            PhaseSyncState::Unknown,
            AbsoluteTimeAuthority::None,
            0,
            sync_loss_count,
        );
        self.diagnostics.last_sync_loss = Some(reason);
    }

    fn update_observation(&mut self, timestamp: Ticks, interval: Ticks, ratio_x1000: u16) {
        self.diagnostics.observation = DecoderObservation {
            current_tooth: self.current_tooth,
            detected_gap_ratio_x1000: ratio_x1000,
            primary_rpm: self.current_rpm(),
            cam_seen: self.cam_seen,
            crank_angle_deg10: self.crank_angle_at(timestamp),
            last_primary_interval: interval,
        };
    }

    fn set_searching_authority(&mut self) {
        let sync_loss_count = self.diagnostics.authority.sync_loss_count;
        self.diagnostics.authority = EngineTimeAuthority::new(
            CrankSyncState::PrimarySearching,
            PhaseSyncState::Unknown,
            AbsoluteTimeAuthority::None,
            0,
            sync_loss_count,
        );
    }

    fn set_locked_authority(&mut self) {
        let sync_loss_count = self.diagnostics.authority.sync_loss_count;
        let (phase, absolute, confidence_x1000) = if self.phase_validated {
            (
                PhaseSyncState::CamValidated720,
                self.config.trigger_angle_atdc_deg10.absolute_authority(),
                1000,
            )
        } else {
            (
                PhaseSyncState::CrankOnly360,
                AbsoluteTimeAuthority::None,
                500,
            )
        };
        self.diagnostics.authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            phase,
            absolute,
            confidence_x1000,
            sync_loss_count,
        );
    }

    fn validate_secondary_before_next_gap(&mut self) {
        if self.primary_locked
            && matches!(
                self.config.secondary.mode,
                SecondaryTriggerMode::SingleToothCam | SecondaryTriggerMode::PollLevel
            )
            && !self.secondary_seen_since_gap
        {
            let _ = self.record_secondary_fault(SyncLossReason::SecondaryTimeout);
        }
    }

    fn record_secondary_fault(&mut self, reason: SyncLossReason) -> Result<(), SyncLossReason> {
        self.phase_validated = false;
        self.secondary_seen_since_gap = false;
        self.diagnostics.last_sync_loss = Some(reason);
        if self.primary_locked {
            self.set_locked_authority();
        } else {
            self.set_searching_authority();
        }
        Err(reason)
    }

    fn current_rpm(&self) -> Rpm {
        rpm_from_tooth_interval(
            self.last_normal_interval,
            self.config.nominal_teeth,
            self.config.primary_speed,
        )
    }

    fn tooth_angle_deg10(&self) -> u32 {
        CRANK_REV_DEGREES10.saturating_mul(u32::from(self.config.tooth_angle_multiplier))
            / u32::from(self.config.nominal_teeth)
    }
}

/// Decoder-observed instantaneous state for telemetry and tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DecoderObservation {
    pub current_tooth: u8,
    pub detected_gap_ratio_x1000: u16,
    pub primary_rpm: Rpm,
    pub cam_seen: bool,
    pub crank_angle_deg10: Option<Degrees10>,
    pub last_primary_interval: Ticks,
}

impl Default for DecoderObservation {
    fn default() -> Self {
        Self {
            current_tooth: 0,
            detected_gap_ratio_x1000: 0,
            primary_rpm: Rpm::new(0),
            cam_seen: false,
            crank_angle_deg10: None,
            last_primary_interval: Ticks::new(0),
        }
    }
}

/// Loss reason emitted by the trigger decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SyncLossReason {
    MissingPrimaryEdge,
    UnexpectedPrimaryEdge,
    InvalidGapRatio,
    WrongToothCount,
    SecondaryTimeout,
    PhaseMismatch,
    ConfigurationInvalid,
}

/// Allocation-free diagnostics snapshot produced by decoder slices.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct TriggerDiagnostics {
    pub observation: DecoderObservation,
    pub authority: EngineTimeAuthority,
    pub last_sync_loss: Option<SyncLossReason>,
}

impl TriggerDiagnostics {
    pub const fn empty() -> Self {
        Self {
            observation: DecoderObservation {
                current_tooth: 0,
                detected_gap_ratio_x1000: 0,
                primary_rpm: Rpm::new(0),
                cam_seen: false,
                crank_angle_deg10: None,
                last_primary_interval: Ticks::new(0),
            },
            authority: EngineTimeAuthority::none(),
            last_sync_loss: None,
        }
    }
}

impl Default for TriggerDiagnostics {
    fn default() -> Self {
        Self::empty()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TriggerValidationError {
    NominalTeethZero,
    MissingTeethZero,
    MissingTeethNotLessThanNominal,
    NotEnoughObservedTeeth,
    UnsupportedTriggerPattern,
    PrimaryTeethZero,
    SecondaryTeethZero,
    PatternIdZero,
    ToothAngleMultiplierZero,
}

pub const fn normalize_engine_cycle_deg10(angle: Degrees10) -> Degrees10 {
    let mut normalized = angle.get() % ENGINE_CYCLE_DEGREES10;
    if normalized < 0 {
        normalized += ENGINE_CYCLE_DEGREES10;
    }

    Degrees10::new(normalized)
}

fn normalize_engine_cycle_deg10_i32(angle: i32) -> Degrees10 {
    let mut normalized = angle % i32::from(ENGINE_CYCLE_DEGREES10);
    if normalized < 0 {
        normalized += i32::from(ENGINE_CYCLE_DEGREES10);
    }

    Degrees10::new(normalized as i16)
}

fn elapsed_ticks(timestamp: Ticks, previous: Ticks) -> Ticks {
    Ticks::new(timestamp.get().wrapping_sub(previous.get()))
}

fn missing_tooth_gap_detected(
    interval: Ticks,
    previous_interval: Ticks,
    threshold_x1000: u16,
) -> Result<(bool, u16), SyncLossReason> {
    let interval_ticks = interval.get();
    let previous_ticks = previous_interval.get();
    if interval_ticks == 0 || previous_ticks == 0 {
        return Err(SyncLossReason::InvalidGapRatio);
    }

    let scaled_interval = u64::from(interval_ticks).saturating_mul(RATIO_SCALE_X1000);
    let scaled_threshold = u64::from(previous_ticks).saturating_mul(u64::from(threshold_x1000));
    let ratio = scaled_interval / u64::from(previous_ticks);

    Ok((scaled_interval >= scaled_threshold, saturating_u16(ratio)))
}

fn rpm_from_tooth_interval(interval: Ticks, nominal_teeth: u8, primary_speed: TriggerSpeed) -> Rpm {
    if interval.get() == 0 || nominal_teeth == 0 {
        return Rpm::new(0);
    }

    let revolutions_per_primary_rev = match primary_speed {
        TriggerSpeed::Crank => 1,
        TriggerSpeed::Cam => 2,
    };
    let revolution_ticks = u64::from(interval.get()).saturating_mul(u64::from(nominal_teeth));
    if revolution_ticks == 0 {
        return Rpm::new(0);
    }

    let rpm = MICROS_PER_MINUTE.saturating_mul(revolutions_per_primary_rev) / revolution_ticks;
    Rpm::new(saturating_u16(rpm))
}

fn saturating_u16(value: u64) -> u16 {
    if value > u64::from(u16::MAX) {
        u16::MAX
    } else {
        value as u16
    }
}

const fn validate_missing_tooth(
    nominal_teeth: u8,
    missing_teeth: u8,
) -> Result<(), TriggerValidationError> {
    if nominal_teeth == 0 {
        return Err(TriggerValidationError::NominalTeethZero);
    }

    if missing_teeth == 0 {
        return Err(TriggerValidationError::MissingTeethZero);
    }

    if missing_teeth >= nominal_teeth {
        return Err(TriggerValidationError::MissingTeethNotLessThanNominal);
    }

    if nominal_teeth - missing_teeth < 2 {
        return Err(TriggerValidationError::NotEnoughObservedTeeth);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::{CrankSyncState, PhaseSyncState, SyncState};
    use proptest::prelude::*;

    const VALID_60_MINUS_2: TriggerProfile = TriggerProfile {
        pattern: TriggerPattern::MissingTooth {
            nominal_teeth: 60,
            missing_teeth: 2,
        },
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary: SecondaryTriggerProfile {
            mode: SecondaryTriggerMode::SingleToothCam,
            edge: TriggerEdge::Falling,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        trigger_angle_atdc_deg10: TriggerAngleAuthority::Unknown,
        tooth_angle_multiplier: 1,
        filter: TriggerFilter::Weak,
        resync: ResyncPolicy::OnSyncLoss,
        startup: StartupSyncPolicy {
            skip_revolutions: 2,
            require_full_cycle: true,
        },
        latency: EngineTimeLatency {
            primary_edge_delay_us: Micros::new(0),
            secondary_edge_delay_us: Micros::new(0),
            output_schedule_delay_us: Micros::new(0),
        },
    };

    #[test]
    fn missing_tooth_profile_validates_and_reports_observed_teeth() {
        assert_eq!(VALID_60_MINUS_2.validate(), Ok(()));
        assert_eq!(VALID_60_MINUS_2.pattern.observed_primary_teeth(), Some(58));
    }

    #[test]
    fn unknown_trigger_angle_cannot_claim_certified_authority() {
        assert_eq!(
            TriggerAngleAuthority::Unknown.absolute_authority(),
            AbsoluteTimeAuthority::None
        );
        assert_eq!(
            VALID_60_MINUS_2.declared_absolute_authority(),
            AbsoluteTimeAuthority::None
        );

        let certified = TriggerAngleAuthority::CertifiedProfile(Degrees10::new(840));
        assert_eq!(
            certified.absolute_authority(),
            AbsoluteTimeAuthority::CertifiedProfile
        );
    }

    #[test]
    fn angle_normalization_stays_in_engine_cycle_range() {
        assert_eq!(normalize_engine_cycle_deg10(Degrees10::new(0)).get(), 0);
        assert_eq!(normalize_engine_cycle_deg10(Degrees10::new(7200)).get(), 0);
        assert_eq!(
            normalize_engine_cycle_deg10(Degrees10::new(-1)).get(),
            ENGINE_CYCLE_DEGREES10 - 1
        );
        assert_eq!(
            TriggerAngleAuthority::ExpertManual(Degrees10::new(7215))
                .normalized_angle_deg10()
                .map(Degrees10::get),
            Some(15)
        );
    }

    #[test]
    fn validation_rejects_invalid_missing_tooth_combinations() {
        assert_eq!(
            TriggerPattern::MissingTooth {
                nominal_teeth: 0,
                missing_teeth: 1,
            }
            .validate(),
            Err(TriggerValidationError::NominalTeethZero)
        );
        assert_eq!(
            TriggerPattern::MissingTooth {
                nominal_teeth: 60,
                missing_teeth: 0,
            }
            .validate(),
            Err(TriggerValidationError::MissingTeethZero)
        );
        assert_eq!(
            TriggerPattern::MissingTooth {
                nominal_teeth: 2,
                missing_teeth: 2,
            }
            .validate(),
            Err(TriggerValidationError::MissingTeethNotLessThanNominal)
        );
        assert_eq!(
            TriggerPattern::MissingTooth {
                nominal_teeth: 2,
                missing_teeth: 1,
            }
            .validate(),
            Err(TriggerValidationError::NotEnoughObservedTeeth)
        );
    }

    #[test]
    fn validation_rejects_other_zero_sized_profile_parts() {
        assert_eq!(
            TriggerPattern::DualWheel { primary_teeth: 0 }.validate(),
            Err(TriggerValidationError::PrimaryTeethZero)
        );
        assert_eq!(
            TriggerPattern::NonUniform {
                pattern_id: PatternId::new(0),
            }
            .validate(),
            Err(TriggerValidationError::PatternIdZero)
        );
        assert_eq!(
            SecondaryTriggerMode::MultiToothCam { teeth: 0 }.validate(),
            Err(TriggerValidationError::SecondaryTeethZero)
        );
        assert_eq!(
            TriggerProfile {
                tooth_angle_multiplier: 0,
                ..VALID_60_MINUS_2
            }
            .validate(),
            Err(TriggerValidationError::ToothAngleMultiplierZero)
        );
    }

    #[test]
    fn diagnostics_defaults_are_safe_and_allocation_free() {
        let diagnostics = TriggerDiagnostics::default();
        assert_eq!(diagnostics.observation, DecoderObservation::default());
        assert_eq!(diagnostics.authority, EngineTimeAuthority::none());
        assert_eq!(diagnostics.last_sync_loss, None);
        assert_eq!(diagnostics.authority.crank, CrankSyncState::NoSignal);
        assert_eq!(diagnostics.authority.phase, PhaseSyncState::Unknown);
    }

    fn decoder_config(trigger_angle: Degrees10) -> MissingToothDecoderConfig {
        MissingToothDecoderConfig {
            nominal_teeth: 60,
            missing_teeth: 2,
            primary_speed: TriggerSpeed::Crank,
            primary_edge: TriggerEdge::Rising,
            secondary: SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::None,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(trigger_angle),
            tooth_angle_multiplier: 1,
            minimum_edge_interval: Ticks::new(50),
            gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        }
    }

    fn new_decoder(config: MissingToothDecoderConfig) -> MissingToothDecoder {
        match MissingToothDecoder::try_new(config) {
            Ok(decoder) => decoder,
            Err(error) => panic!("decoder config rejected: {error:?}"),
        }
    }

    fn lock_60_minus_2(decoder: &mut MissingToothDecoder) {
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(0)),
            Ok(MissingToothDecoderEvent::FirstEdge)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(1000)),
            Ok(MissingToothDecoderEvent::Searching)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(4000)),
            Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
        );
    }

    fn single_tooth_cam_config(trigger_angle: Degrees10) -> MissingToothDecoderConfig {
        MissingToothDecoderConfig {
            secondary: SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::SingleToothCam,
                edge: TriggerEdge::Falling,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            ..decoder_config(trigger_angle)
        }
    }

    fn poll_level_config(trigger_angle: Degrees10) -> MissingToothDecoderConfig {
        MissingToothDecoderConfig {
            secondary: SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::PollLevel,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            ..decoder_config(trigger_angle)
        }
    }

    fn profiled_config(
        trigger_angle: Degrees10,
        secondary: SecondaryTriggerProfile,
        startup: StartupSyncPolicy,
        latency: EngineTimeLatency,
    ) -> TriggerProfile {
        TriggerProfile {
            pattern: TriggerPattern::MissingTooth {
                nominal_teeth: 60,
                missing_teeth: 2,
            },
            primary_speed: TriggerSpeed::Crank,
            primary_edge: TriggerEdge::Rising,
            secondary,
            trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(trigger_angle),
            tooth_angle_multiplier: 1,
            filter: TriggerFilter::Off,
            resync: ResyncPolicy::OnSyncLoss,
            startup,
            latency,
        }
    }

    fn profiled_decoder(profile: TriggerProfile) -> ProfiledMissingToothDecoder {
        match ProfiledMissingToothDecoder::try_new(
            profile,
            Ticks::new(50),
            DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        ) {
            Ok(decoder) => decoder,
            Err(error) => panic!("profile rejected: {error:?}"),
        }
    }

    fn drive_missing_tooth_cycle(
        decoder: &mut ProfiledMissingToothDecoder,
        mut timestamp: u32,
        normal_interval: u32,
    ) -> u32 {
        for expected_tooth in 2..=58 {
            timestamp = timestamp.saturating_add(normal_interval);
            assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(timestamp)),
                Ok(MissingToothDecoderEvent::Tooth {
                    current_tooth: expected_tooth,
                })
            );
        }

        timestamp = timestamp.saturating_add(normal_interval * 2);
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
        );
        timestamp
    }

    #[test]
    fn false_edge_below_filter_threshold_is_ignored() {
        let mut decoder = new_decoder(decoder_config(Degrees10::new(100)));
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(1000)),
            Ok(MissingToothDecoderEvent::FirstEdge)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(2000)),
            Ok(MissingToothDecoderEvent::Searching)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(2010)),
            Ok(MissingToothDecoderEvent::IgnoredByFilter)
        );

        let observation = decoder.observation();
        assert_eq!(observation.last_primary_interval, Ticks::new(1000));
        assert_eq!(observation.primary_rpm, Rpm::new(1000));

        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(3000)),
            Ok(MissingToothDecoderEvent::Searching)
        );
        assert_eq!(
            decoder.observation().last_primary_interval,
            Ticks::new(1000)
        );
    }

    #[test]
    fn sixty_minus_two_detects_tooth_one_after_gap_and_reports_rpm() {
        let mut decoder = new_decoder(decoder_config(Degrees10::new(840)));
        lock_60_minus_2(&mut decoder);

        let observation = decoder.observation();
        assert!(decoder.is_primary_locked());
        assert_eq!(observation.current_tooth, 1);
        assert_eq!(observation.detected_gap_ratio_x1000, 3000);
        assert_eq!(observation.primary_rpm, Rpm::new(1000));
        assert_eq!(observation.crank_angle_deg10.map(Degrees10::get), Some(840));
        assert_eq!(decoder.diagnostics().last_sync_loss, None);
    }

    #[test]
    fn wrong_primary_edge_is_rejected_without_advancing_decoder() {
        let mut decoder = new_decoder(decoder_config(Degrees10::new(840)));

        assert_eq!(
            decoder.ingest_primary_edge_with_kind(Ticks::new(0), TriggerEdge::Falling),
            Err(SyncLossReason::UnexpectedPrimaryEdge)
        );
        assert!(!decoder.is_primary_locked());
        assert_eq!(decoder.observation().last_primary_interval, Ticks::new(0));
        assert_eq!(
            decoder.diagnostics().last_sync_loss,
            Some(SyncLossReason::UnexpectedPrimaryEdge)
        );

        lock_60_minus_2(&mut decoder);
        assert!(decoder.is_primary_locked());
    }

    #[test]
    fn single_tooth_cam_requires_matching_edge_after_primary_lock() {
        let mut decoder = new_decoder(single_tooth_cam_config(Degrees10::new(840)));

        assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
        assert_eq!(
            decoder.diagnostics().authority.phase,
            PhaseSyncState::Unknown
        );

        lock_60_minus_2(&mut decoder);
        assert_eq!(
            decoder.ingest_secondary_edge(TriggerEdge::Rising),
            Err(SyncLossReason::PhaseMismatch)
        );
        assert_eq!(
            decoder.diagnostics().authority.phase,
            PhaseSyncState::CrankOnly360
        );

        assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
        let diagnostics = decoder.diagnostics();
        assert!(diagnostics.observation.cam_seen);
        assert_eq!(diagnostics.authority.phase, PhaseSyncState::CamValidated720);
        assert_eq!(
            diagnostics.authority.absolute,
            AbsoluteTimeAuthority::ExpertManual
        );
    }

    #[test]
    fn poll_level_validates_level_at_tooth_one_before_phase_promotion() {
        let mut decoder = new_decoder(poll_level_config(Degrees10::new(840)));
        lock_60_minus_2(&mut decoder);

        assert_eq!(
            decoder.ingest_secondary_level(TriggerLevel::Low),
            Err(SyncLossReason::PhaseMismatch)
        );
        assert_eq!(
            decoder.diagnostics().authority.phase,
            PhaseSyncState::CrankOnly360
        );

        assert_eq!(decoder.ingest_secondary_level(TriggerLevel::High), Ok(()));
        let diagnostics = decoder.diagnostics();
        assert!(diagnostics.observation.cam_seen);
        assert_eq!(diagnostics.authority.phase, PhaseSyncState::CamValidated720);
        assert_eq!(
            diagnostics.authority.absolute,
            AbsoluteTimeAuthority::ExpertManual
        );

        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(5000)),
            Ok(MissingToothDecoderEvent::Tooth { current_tooth: 2 })
        );
        assert_eq!(
            decoder.ingest_secondary_level(TriggerLevel::High),
            Err(SyncLossReason::PhaseMismatch)
        );
    }

    #[test]
    fn profiled_decoder_holds_absolute_authority_until_startup_policy_is_satisfied() {
        let profile = profiled_config(
            Degrees10::new(840),
            SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::SingleToothCam,
                edge: TriggerEdge::Falling,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            StartupSyncPolicy {
                skip_revolutions: 2,
                require_full_cycle: true,
            },
            EngineTimeLatency::default(),
        );
        let mut decoder = profiled_decoder(profile);

        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(0)),
            Ok(MissingToothDecoderEvent::FirstEdge)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(1000)),
            Ok(MissingToothDecoderEvent::Searching)
        );
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(4000)),
            Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
        );
        assert_eq!(
            decoder.authority().compatibility_summary(),
            SyncState::Syncing
        );
        assert_eq!(decoder.authority().absolute, AbsoluteTimeAuthority::None);

        assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
        assert_eq!(
            decoder.authority().compatibility_summary(),
            SyncState::Syncing
        );
        assert_eq!(decoder.authority().absolute, AbsoluteTimeAuthority::None);

        let mut timestamp = 4000;
        timestamp = drive_missing_tooth_cycle(&mut decoder, timestamp, 1000);
        assert_eq!(
            decoder.authority().compatibility_summary(),
            SyncState::Syncing
        );
        assert_eq!(decoder.authority().absolute, AbsoluteTimeAuthority::None);

        assert_eq!(decoder.ingest_secondary_edge(TriggerEdge::Falling), Ok(()));
        let _ = drive_missing_tooth_cycle(&mut decoder, timestamp, 1000);
        let authority = decoder.authority();
        assert_eq!(authority.phase, PhaseSyncState::CamValidated720);
        assert_eq!(authority.absolute, AbsoluteTimeAuthority::ExpertManual);
        assert_eq!(authority.compatibility_summary(), SyncState::Synced);
    }

    #[test]
    fn profiled_decoder_compensates_angle_queries_for_latency() {
        let zero_latency = profiled_config(
            Degrees10::new(0),
            SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::None,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            StartupSyncPolicy::default(),
            EngineTimeLatency::default(),
        );
        let delayed_latency = profiled_config(
            Degrees10::new(0),
            SecondaryTriggerProfile {
                mode: SecondaryTriggerMode::None,
                edge: TriggerEdge::Rising,
                poll_level: PollLevelPolarity::ActiveHigh,
            },
            StartupSyncPolicy::default(),
            EngineTimeLatency {
                primary_edge_delay_us: Micros::new(100),
                secondary_edge_delay_us: Micros::new(50),
                output_schedule_delay_us: Micros::new(100),
            },
        );

        let mut zero = profiled_decoder(zero_latency);
        let mut delayed = profiled_decoder(delayed_latency);

        for decoder in [&mut zero, &mut delayed] {
            assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(0)),
                Ok(MissingToothDecoderEvent::FirstEdge)
            );
            assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(1000)),
                Ok(MissingToothDecoderEvent::Searching)
            );
            assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(4000)),
                Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
            );
        }

        assert_eq!(
            zero.crank_angle_at(Ticks::new(4500)).map(Degrees10::get),
            Some(31)
        );
        assert_eq!(
            delayed.crank_angle_at(Ticks::new(4500)).map(Degrees10::get),
            Some(15)
        );
    }

    #[test]
    fn crank_angle_wraps_to_engine_cycle_range() {
        let mut decoder = new_decoder(decoder_config(Degrees10::new(7190)));
        lock_60_minus_2(&mut decoder);
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(5000)),
            Ok(MissingToothDecoderEvent::Tooth { current_tooth: 2 })
        );

        let angle_at_edge = decoder.observation().crank_angle_deg10.map(Degrees10::get);
        let angle_between_edges = decoder.crank_angle_at(Ticks::new(5500)).map(Degrees10::get);
        assert_eq!(angle_at_edge, Some(50));
        assert_eq!(angle_between_edges, Some(80));
        for timestamp in [Ticks::new(5000), Ticks::new(25_000), Ticks::new(125_000)] {
            let angle = decoder.crank_angle_at(timestamp).map(Degrees10::get);
            assert!(matches!(angle, Some(0..=7199)));
        }
    }

    #[test]
    fn wrong_tooth_count_drops_sync_and_records_reason() {
        let mut decoder = new_decoder(decoder_config(Degrees10::new(0)));
        lock_60_minus_2(&mut decoder);

        let mut timestamp = 4000;
        for expected_tooth in 2..=58 {
            timestamp += 1000;
            assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(timestamp)),
                Ok(MissingToothDecoderEvent::Tooth {
                    current_tooth: expected_tooth,
                })
            );
        }

        timestamp += 1000;
        assert_eq!(
            decoder.ingest_primary_edge(Ticks::new(timestamp)),
            Err(SyncLossReason::WrongToothCount)
        );
        let diagnostics = decoder.diagnostics();
        assert!(!decoder.is_primary_locked());
        assert_eq!(
            diagnostics.last_sync_loss,
            Some(SyncLossReason::WrongToothCount)
        );
        assert_eq!(diagnostics.authority.crank, CrankSyncState::SyncLost);
        assert_eq!(diagnostics.authority.sync_loss_count, 1);
    }

    #[test]
    fn cranking_jitter_does_not_falsely_certify_full_authority() {
        let mut decoder = new_decoder(decoder_config(Degrees10::new(120)));
        let timestamps = [
            Ticks::new(0),
            Ticks::new(1000),
            Ticks::new(2400),
            Ticks::new(3300),
            Ticks::new(4600),
            Ticks::new(5600),
        ];

        for timestamp in timestamps {
            let _ = decoder.ingest_primary_edge(timestamp);
        }

        let diagnostics = decoder.diagnostics();
        assert!(!decoder.is_primary_locked());
        assert_ne!(
            diagnostics.authority.compatibility_summary(),
            SyncState::Synced
        );
        assert_eq!(diagnostics.authority.absolute, AbsoluteTimeAuthority::None);
        assert!(
            diagnostics.observation.detected_gap_ratio_x1000
                < DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000
        );
    }

    #[test]
    fn fixed_capacity_batch_ingests_edges_without_allocation() {
        let mut batch: PrimaryEdgeBatch<4> = PrimaryEdgeBatch::new();
        assert_eq!(batch.capacity(), 4);
        assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(0))), Ok(()));
        assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(1000))), Ok(()));
        assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(4000))), Ok(()));
        assert_eq!(batch.push(PrimaryEdgeSample::new(Ticks::new(4010))), Ok(()));
        assert_eq!(
            batch.push(PrimaryEdgeSample::new(Ticks::new(5000))),
            Err(PrimaryEdgeBatchError::Full)
        );

        let mut decoder = new_decoder(decoder_config(Degrees10::new(0)));
        let result = decoder.ingest_primary_edges(&batch);
        assert_eq!(result.processed_edges, 3);
        assert_eq!(result.ignored_edges, 1);
        assert_eq!(
            result.last_event,
            Some(MissingToothDecoderEvent::IgnoredByFilter)
        );
        assert!(decoder.is_primary_locked());
        batch.clear();
        assert!(batch.is_empty());
    }

    #[test]
    fn deterministic_small_generated_streams_do_not_panic_or_overrun() {
        for nominal_teeth in 3..=12 {
            for missing_teeth in 1..nominal_teeth {
                if nominal_teeth - missing_teeth < 2 {
                    continue;
                }

                let config = MissingToothDecoderConfig {
                    nominal_teeth,
                    missing_teeth,
                    primary_speed: TriggerSpeed::Crank,
                    primary_edge: TriggerEdge::Rising,
                    secondary: SecondaryTriggerProfile {
                        mode: SecondaryTriggerMode::None,
                        edge: TriggerEdge::Rising,
                        poll_level: PollLevelPolarity::ActiveHigh,
                    },
                    trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(Degrees10::new(
                        0,
                    )),
                    tooth_angle_multiplier: 1,
                    minimum_edge_interval: Ticks::new(1),
                    gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
                };

                let Ok(mut decoder) = MissingToothDecoder::try_new(config) else {
                    continue;
                };
                let observed_teeth = nominal_teeth - missing_teeth;
                let normal_interval = 100;
                let gap_interval = normal_interval * u32::from(missing_teeth + 1);
                let mut timestamp = 0;
                let mut batch: PrimaryEdgeBatch<32> = PrimaryEdgeBatch::new();

                assert_eq!(
                    batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                    Ok(())
                );
                timestamp += normal_interval;
                assert_eq!(
                    batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                    Ok(())
                );
                timestamp += gap_interval;
                assert_eq!(
                    batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                    Ok(())
                );
                for _ in 1..observed_teeth {
                    timestamp += normal_interval;
                    assert_eq!(
                        batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                        Ok(())
                    );
                }
                timestamp += gap_interval;
                assert_eq!(
                    batch.push(PrimaryEdgeSample::new(Ticks::new(timestamp))),
                    Ok(())
                );

                let result = decoder.ingest_primary_edges(&batch);
                assert_eq!(usize::from(result.processed_edges), batch.len());
                assert_eq!(result.ignored_edges, 0);
                assert_eq!(result.last_sync_loss, None);
            }
        }
    }

    proptest! {
        #[test]
        fn prop_missing_tooth_stream_keeps_gap_and_angle_order(
            nominal_teeth in 3u8..16,
            missing_teeth in 1u8..8,
            normal_interval in 50u32..1000u32,
        ) {
            prop_assume!(missing_teeth < nominal_teeth);
            prop_assume!(nominal_teeth - missing_teeth >= 2);

            let config = MissingToothDecoderConfig {
                nominal_teeth,
                missing_teeth,
                primary_speed: TriggerSpeed::Crank,
                primary_edge: TriggerEdge::Rising,
                secondary: SecondaryTriggerProfile {
                    mode: SecondaryTriggerMode::None,
                    edge: TriggerEdge::Rising,
                    poll_level: PollLevelPolarity::ActiveHigh,
                },
                trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(Degrees10::new(0)),
                tooth_angle_multiplier: 1,
                minimum_edge_interval: Ticks::new(1),
                gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
            };

            let mut decoder = MissingToothDecoder::try_new(config)
                .expect("valid missing-tooth config was rejected");

            let observed_teeth = nominal_teeth - missing_teeth;
            let mut timestamp = 0u32;

            prop_assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(timestamp)),
                Ok(MissingToothDecoderEvent::FirstEdge)
            );
            timestamp = timestamp.saturating_add(normal_interval);
            prop_assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(timestamp)),
                Ok(MissingToothDecoderEvent::Searching)
            );
            timestamp = timestamp.saturating_add(normal_interval * u32::from(missing_teeth + 1));
            prop_assert_eq!(
                decoder.ingest_primary_edge(Ticks::new(timestamp)),
                Ok(MissingToothDecoderEvent::Gap { current_tooth: 1 })
            );

            let mut last_angle = decoder
                .observation()
                .crank_angle_deg10
                .map(Degrees10::get)
                .unwrap_or_default();

            prop_assert_eq!(last_angle, 0);

            for expected_tooth in 2..=observed_teeth {
                timestamp = timestamp.saturating_add(normal_interval);
                prop_assert_eq!(
                    decoder.ingest_primary_edge(Ticks::new(timestamp)),
                    Ok(MissingToothDecoderEvent::Tooth {
                        current_tooth: expected_tooth,
                    })
                );
                let angle = decoder
                    .observation()
                    .crank_angle_deg10
                    .map(Degrees10::get)
                    .unwrap_or_default();
                prop_assert!(angle >= last_angle);
                last_angle = angle;
            }
        }
    }
}
