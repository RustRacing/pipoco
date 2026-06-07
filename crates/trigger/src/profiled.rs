use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, PhaseSyncState, Rpm, Ticks,
};

use crate::diag::SyncLossReason;
use crate::diag::TriggerDiagnostics;
use crate::math::{normalize_engine_cycle_deg10_i32, CRANK_REV_DEGREES10};
use crate::missing_tooth::{
    MissingToothDecoder, MissingToothDecoderConfig, MissingToothDecoderEvent,
};
use crate::profile::{RuntimeMissingToothProfile, TriggerEdge, TriggerLevel, TriggerProfile};

/// Profile-aware missing-tooth decoder that overlays startup and latency policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct ProfiledMissingToothDecoder {
    decoder: MissingToothDecoder,
    profile: RuntimeMissingToothProfile,
    revolutions_since_lock: u8,
    saw_primary_gap: bool,
}

impl ProfiledMissingToothDecoder {
    pub fn try_new(
        profile: RuntimeMissingToothProfile,
        minimum_edge_interval: Ticks,
        gap_ratio_threshold_x1000: u16,
    ) -> Result<Self, crate::diag::TriggerValidationError> {
        profile.validate()?;
        let config = MissingToothDecoderConfig::from_runtime_profile(
            profile,
            minimum_edge_interval,
            gap_ratio_threshold_x1000,
        );

        Ok(Self {
            decoder: MissingToothDecoder::try_new(config)?,
            profile,
            revolutions_since_lock: 0,
            saw_primary_gap: false,
        })
    }

    pub fn try_new_from_import_profile(
        profile: TriggerProfile,
        minimum_edge_interval: Ticks,
        gap_ratio_threshold_x1000: u16,
    ) -> Result<Self, crate::diag::TriggerValidationError> {
        Self::try_new(
            RuntimeMissingToothProfile::from_import_profile(profile)?,
            minimum_edge_interval,
            gap_ratio_threshold_x1000,
        )
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

    pub fn crank_angle_at(&self, timestamp: Ticks) -> Option<ecu_domain::Degrees10> {
        if !self.decoder.is_primary_locked() {
            return None;
        }

        let observed_teeth = self.profile.observed_primary_teeth();
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
        let authority = self.decoder.raw_diagnostics().authority;
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

    /// Raw decoder diagnostics before profile-level startup gating is applied.
    pub(crate) fn raw_diagnostics(&self) -> TriggerDiagnostics {
        self.decoder.raw_diagnostics()
    }

    /// Profile-gated diagnostics safe for consumers that need final engine-time authority.
    pub fn diagnostics(&self) -> TriggerDiagnostics {
        let mut diagnostics = self.raw_diagnostics();
        diagnostics.authority = self.authority();
        diagnostics
    }
}
