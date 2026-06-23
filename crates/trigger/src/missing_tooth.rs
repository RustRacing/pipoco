use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, Degrees10, EngineTimeAuthority, PhaseSyncState, Rpm,
    Ticks,
};

use crate::diag::{DecoderObservation, SyncLossReason, TriggerDiagnostics, TriggerValidationError};
use crate::edge_batch::PrimaryEdgeBatch;
use crate::math::{
    elapsed_ticks, missing_tooth_gap_detected, normalize_engine_cycle_deg10_i32,
    rpm_from_tooth_interval,
};
use crate::profile::{
    validate_missing_tooth, RuntimeMissingToothProfile, RuntimeSecondaryTriggerMode,
    RuntimeSecondaryTriggerProfile, TriggerAngleAuthority, TriggerEdge, TriggerLevel, TriggerSpeed,
};

pub const DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000: u16 = 1500;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MissingToothDecoderConfig {
    pub nominal_teeth: u8,
    pub missing_teeth: u8,
    pub primary_speed: TriggerSpeed,
    pub primary_edge: TriggerEdge,
    pub secondary: RuntimeSecondaryTriggerProfile,
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
            secondary: RuntimeSecondaryTriggerProfile {
                mode: RuntimeSecondaryTriggerMode::None,
                edge: TriggerEdge::Rising,
                poll_level: crate::profile::PollLevelPolarity::ActiveHigh,
            },
            trigger_angle_atdc_deg10,
            tooth_angle_multiplier: 1,
            minimum_edge_interval: Ticks::new(0),
            gap_ratio_threshold_x1000: DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        }
    }

    pub const fn from_runtime_profile(
        profile: RuntimeMissingToothProfile,
        minimum_edge_interval: Ticks,
        gap_ratio_threshold_x1000: u16,
    ) -> Self {
        Self {
            nominal_teeth: profile.nominal_teeth,
            missing_teeth: profile.missing_teeth,
            primary_speed: profile.primary_speed,
            primary_edge: profile.primary_edge,
            secondary: profile.secondary,
            trigger_angle_atdc_deg10: profile.trigger_angle_atdc_deg10,
            tooth_angle_multiplier: profile.tooth_angle_multiplier,
            minimum_edge_interval,
            gap_ratio_threshold_x1000,
        }
    }

    pub(crate) const fn normalized_gap_threshold_x1000(self) -> u16 {
        if self.gap_ratio_threshold_x1000 == 0 {
            DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000
        } else {
            self.gap_ratio_threshold_x1000
        }
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

    /// Raw decoder diagnostics before profile-level startup gating is applied.
    ///
    /// Consumers using a [`ProfiledMissingToothDecoder`](crate::ProfiledMissingToothDecoder)
    /// should prefer its profile-gated diagnostics/authority methods for final
    /// scheduling authority.
    pub(crate) const fn raw_diagnostics(&self) -> TriggerDiagnostics {
        self.diagnostics
    }

    /// Raw decoder diagnostics before profile-level startup gating is applied.
    ///
    /// This is retained for direct decoder users; profile-aware consumers should
    /// use [`ProfiledMissingToothDecoder::diagnostics`](crate::ProfiledMissingToothDecoder::diagnostics).
    pub const fn diagnostics(&self) -> TriggerDiagnostics {
        self.raw_diagnostics()
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
            RuntimeSecondaryTriggerMode::SingleToothCam => {
                if edge != self.config.secondary.edge {
                    return self.record_secondary_fault(SyncLossReason::PhaseMismatch);
                }

                self.cam_seen = true;
                self.diagnostics.observation.cam_seen = true;
                if self.primary_locked {
                    if self.current_tooth != 1 {
                        return self.record_secondary_fault(SyncLossReason::PhaseMismatch);
                    }
                    self.secondary_seen_since_gap = true;
                    self.phase_validated = true;
                    self.set_locked_authority();
                }

                Ok(())
            }
            RuntimeSecondaryTriggerMode::None | RuntimeSecondaryTriggerMode::PollLevel => {
                self.record_secondary_fault(SyncLossReason::ConfigurationInvalid)
            }
        }
    }

    pub fn ingest_secondary_level(&mut self, level: TriggerLevel) -> Result<(), SyncLossReason> {
        match self.config.secondary.mode {
            RuntimeSecondaryTriggerMode::PollLevel => {
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
            RuntimeSecondaryTriggerMode::None | RuntimeSecondaryTriggerMode::SingleToothCam => {
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
                RuntimeSecondaryTriggerMode::SingleToothCam
                    | RuntimeSecondaryTriggerMode::PollLevel
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
        (crate::math::CRANK_REV_DEGREES10)
            .saturating_mul(u32::from(self.config.tooth_angle_multiplier))
            / u32::from(self.config.nominal_teeth)
    }
}
