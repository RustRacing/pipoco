use ecu_domain::{AbsoluteTimeAuthority, Degrees10};

use crate::diag::TriggerValidationError;
use crate::math::normalize_engine_cycle_deg10;

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

/// Secondary trigger modes supported by the runtime missing-tooth decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum RuntimeSecondaryTriggerMode {
    #[default]
    None,
    SingleToothCam,
    PollLevel,
}

/// Runtime-supported secondary trigger configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RuntimeSecondaryTriggerProfile {
    pub mode: RuntimeSecondaryTriggerMode,
    pub edge: TriggerEdge,
    pub poll_level: PollLevelPolarity,
}

impl Default for RuntimeSecondaryTriggerProfile {
    fn default() -> Self {
        Self {
            mode: RuntimeSecondaryTriggerMode::None,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        }
    }
}

impl RuntimeSecondaryTriggerProfile {
    pub const fn from_import_profile(
        profile: SecondaryTriggerProfile,
    ) -> Result<Self, TriggerValidationError> {
        if let Err(error) = profile.validate() {
            return Err(error);
        }

        let mode = match profile.mode {
            SecondaryTriggerMode::None => RuntimeSecondaryTriggerMode::None,
            SecondaryTriggerMode::SingleToothCam => RuntimeSecondaryTriggerMode::SingleToothCam,
            SecondaryTriggerMode::PollLevel => RuntimeSecondaryTriggerMode::PollLevel,
            SecondaryTriggerMode::FourMinusOneCam
            | SecondaryTriggerMode::MultiToothCam { .. }
            | SecondaryTriggerMode::OemPattern { .. } => {
                return Err(TriggerValidationError::UnsupportedSecondaryTriggerMode);
            }
        };

        Ok(Self {
            mode,
            edge: profile.edge,
            poll_level: profile.poll_level,
        })
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

impl StartupSyncPolicy {
    pub const fn authority_ready(self, completed_revolutions: u8) -> bool {
        completed_revolutions >= self.skip_revolutions
            && (!self.require_full_cycle || completed_revolutions > 0)
    }
}

/// Fixed latency compensation inputs for engine-time calculations.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct EngineTimeLatency {
    pub primary_edge_delay_us: ecu_domain::Micros,
    pub secondary_edge_delay_us: ecu_domain::Micros,
    pub output_schedule_delay_us: ecu_domain::Micros,
}

impl EngineTimeLatency {
    pub const fn total_us(self) -> u32 {
        self.primary_edge_delay_us
            .get()
            .saturating_add(self.secondary_edge_delay_us.get())
            .saturating_add(self.output_schedule_delay_us.get())
    }
}

/// Import/calibration trigger profile.
///
/// This is intentionally broader than the first runtime decoder. It can carry
/// Speeduino-like metadata and future/OEM patterns, but callers must convert it
/// to a runtime profile before scheduling engine time from it.
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

    pub const fn validate_profiled_decoder_runtime_policy(
        self,
    ) -> Result<(), TriggerValidationError> {
        if !matches!(self.filter, TriggerFilter::Off) {
            return Err(TriggerValidationError::UnsupportedProfileFilter);
        }

        if !matches!(self.resync, ResyncPolicy::OnSyncLoss) {
            return Err(TriggerValidationError::UnsupportedResyncPolicy);
        }

        Ok(())
    }

    pub const fn runtime_missing_tooth_profile(
        self,
    ) -> Result<RuntimeMissingToothProfile, TriggerValidationError> {
        RuntimeMissingToothProfile::from_import_profile(self)
    }
}

/// Missing-tooth profile accepted by the runtime profiled decoder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RuntimeMissingToothProfile {
    pub nominal_teeth: u8,
    pub missing_teeth: u8,
    pub primary_speed: TriggerSpeed,
    pub primary_edge: TriggerEdge,
    pub secondary: RuntimeSecondaryTriggerProfile,
    pub trigger_angle_atdc_deg10: TriggerAngleAuthority,
    pub tooth_angle_multiplier: u8,
    pub startup: StartupSyncPolicy,
    pub latency: EngineTimeLatency,
}

impl RuntimeMissingToothProfile {
    pub const fn from_import_profile(
        profile: TriggerProfile,
    ) -> Result<Self, TriggerValidationError> {
        if let Err(error) = profile.validate() {
            return Err(error);
        }

        if let Err(error) = profile.validate_profiled_decoder_runtime_policy() {
            return Err(error);
        }

        let (nominal_teeth, missing_teeth) = match profile.pattern {
            TriggerPattern::MissingTooth {
                nominal_teeth,
                missing_teeth,
            } => (nominal_teeth, missing_teeth),
            TriggerPattern::BasicDistributor
            | TriggerPattern::DualWheel { .. }
            | TriggerPattern::NonUniform { .. } => {
                return Err(TriggerValidationError::UnsupportedTriggerPattern);
            }
        };

        let secondary = match RuntimeSecondaryTriggerProfile::from_import_profile(profile.secondary)
        {
            Ok(secondary) => secondary,
            Err(error) => return Err(error),
        };

        Ok(Self {
            nominal_teeth,
            missing_teeth,
            primary_speed: profile.primary_speed,
            primary_edge: profile.primary_edge,
            secondary,
            trigger_angle_atdc_deg10: profile.trigger_angle_atdc_deg10,
            tooth_angle_multiplier: profile.tooth_angle_multiplier,
            startup: profile.startup,
            latency: profile.latency,
        })
    }

    pub const fn validate(self) -> Result<(), TriggerValidationError> {
        if let Err(error) = validate_missing_tooth(self.nominal_teeth, self.missing_teeth) {
            return Err(error);
        }

        if self.tooth_angle_multiplier == 0 {
            return Err(TriggerValidationError::ToothAngleMultiplierZero);
        }

        Ok(())
    }

    pub const fn observed_primary_teeth(self) -> u8 {
        self.nominal_teeth - self.missing_teeth
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
}

pub(crate) const fn validate_missing_tooth(
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
