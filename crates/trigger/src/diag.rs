use ecu_domain::{Rpm, Ticks};

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
    pub authority: ecu_domain::EngineTimeAuthority,
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
            authority: ecu_domain::EngineTimeAuthority::none(),
            last_sync_loss: None,
        }
    }
}

impl Default for TriggerDiagnostics {
    fn default() -> Self {
        Self::empty()
    }
}

/// Decoder-observed instantaneous state for telemetry and tests.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct DecoderObservation {
    pub current_tooth: u8,
    pub detected_gap_ratio_x1000: u16,
    pub primary_rpm: Rpm,
    pub cam_seen: bool,
    pub crank_angle_deg10: Option<ecu_domain::Degrees10>,
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

/// Validation failures for trigger profiles and decoder setup.
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
    UnsupportedProfileFilter,
    UnsupportedResyncPolicy,
    UnsupportedSecondaryTriggerMode,
}
