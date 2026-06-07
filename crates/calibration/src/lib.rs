#![cfg_attr(not(test), no_std)]

pub mod configs;
pub mod expert_trigger;
pub mod kv;
pub mod model;
pub mod runtime_tune;
pub mod sensors;

pub use configs::{
    AeConfig, AseConfig, ClConfig, DfcoConfig, EcuConfig, FanConfig, IdleConfig, LambdaConfig,
    LimiterStrategy, LoadFailureConfig, LtftConfig, O2SensorType, PlausibilityConfig, RateConfig,
    RevLimiterConfig, SensorsLimits, WueConfig,
};

pub use expert_trigger::{
    ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration, ExpertTriggerRecordError,
    ExpertTriggerRuntimeAuthorityError, ExpertTriggerValidationError, ExpertUnlock,
    FixedTimingMode, PollLevelPolarity, PrimaryTriggerSpeed, SecondaryTriggerMode,
    TriggerAuthority, TriggerEdge, TriggerFilter, TriggerPattern, EXPERT_TRIGGER_RECORD_LEN,
};
pub use kv::{KvError, KvStore, PersistError};
pub use model::{
    ActiveCalibration, Calibration, CalibrationClass, CalibrationDiff, CalibrationRevision,
    CalibrationSchemaVersion, CalibrationSnapshot, CommitRules, CommitVerdict,
    PersistedCalibrationBlob, PersistedCalibrationStore, StagedCalibration,
};
pub use runtime_tune::{
    FuelRuntimeTable16, FuelRuntimeTune, FUEL_RUNTIME_LOAD_BINS, FUEL_RUNTIME_RPM_BINS,
    FUEL_RUNTIME_TABLE_DIM,
};

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::GeometryCalibration;
    use ecu_domain::{AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, PhaseSyncState};

    #[test]
    fn calibration_groups_default_to_empty_shells() {
        let calibration = Calibration::default();

        assert_eq!(calibration.geometry, GeometryCalibration::default());
        assert_eq!(
            calibration.geometry.expert_trigger.schema_version,
            CalibrationSchemaVersion::CURRENT
        );
        assert_eq!(calibration.fuel, model::FuelCalibration);
        assert_eq!(calibration.ignition, model::IgnitionCalibration);
        assert_eq!(calibration.lambda, model::LambdaCalibration);
        assert_eq!(calibration.torque, model::TorqueCalibration);
        assert_eq!(calibration.idle, model::IdleCalibration);
        assert_eq!(calibration.aux, model::AuxCalibration);
        assert_eq!(calibration.safety, model::SafetyCalibration);
        assert_eq!(calibration.sensors, model::SensorCalibration);
    }

    #[test]
    fn calibration_group_types_are_copyable() {
        let geometry = model::GeometryCalibration::default();
        let duplicate = geometry;

        assert_eq!(geometry, duplicate);
    }

    #[test]
    fn fuel_runtime_tune_carries_runtime_boundary_fields() {
        let mut ve_table = [[100; FUEL_RUNTIME_TABLE_DIM]; FUEL_RUNTIME_TABLE_DIM];
        let mut afr_table = [[147; FUEL_RUNTIME_TABLE_DIM]; FUEL_RUNTIME_TABLE_DIM];
        ve_table[2][3] = 81;
        afr_table[4][5] = 132;

        let tune = FuelRuntimeTune::new(ve_table, afr_table, 2400, 775, 1);

        assert_eq!(tune.ve_table[2][3], 81);
        assert_eq!(tune.afr_table[4][5], 132);
        assert_eq!(tune.required_fuel_us, 2400);
        assert_eq!(tune.injector_deadtime_us, 775);
        assert_eq!(tune.ve_load_source, 1);
    }

    #[test]
    fn fuel_runtime_axes_match_legacy_core_defaults() {
        assert_eq!(
            FUEL_RUNTIME_RPM_BINS,
            [
                500, 1000, 1500, 2000, 2500, 3000, 3500, 4000, 4500, 5000, 5500, 6000, 6500, 7000,
                7500, 8000,
            ]
        );
        assert_eq!(
            FUEL_RUNTIME_LOAD_BINS,
            [20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170,]
        );
    }

    #[test]
    fn active_calibration_tracks_revision_and_payload() {
        let calibration = Calibration::default();
        let active = ActiveCalibration::new(CalibrationRevision::new(7), calibration);

        assert_eq!(active.revision().get(), 7);
        assert_eq!(active.calibration(), calibration);
    }

    #[test]
    fn staged_calibration_tracks_dirty_state_and_revision() {
        let calibration = Calibration::default();
        let mut staged = StagedCalibration::new(CalibrationRevision::new(3), calibration);

        assert_eq!(staged.base_revision().get(), 3);
        assert_eq!(staged.revision().get(), 3);
        assert!(!staged.is_dirty());

        staged.mark_dirty();
        assert!(staged.is_dirty());
        assert_eq!(staged.revision().get(), 4);

        staged.clear_dirty();
        assert!(!staged.is_dirty());
        assert_eq!(staged.calibration(), calibration);
    }

    #[test]
    fn calibration_snapshot_keeps_views_separate() {
        let snapshot = CalibrationSnapshot::default();

        assert_eq!(snapshot.active.revision().get(), 0);
        assert_eq!(snapshot.staged.base_revision().get(), 0);
        assert_eq!(snapshot.staged.revision().get(), 0);
        assert!(!snapshot.staged.is_dirty());
    }

    struct MockPersistedStore(Option<PersistedCalibrationBlob>);

    impl PersistedCalibrationStore for MockPersistedStore {
        type Error = ();

        fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error> {
            Ok(self.0)
        }

        fn save(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
            self.0 = Some(*blob);
            Ok(())
        }
    }

    #[test]
    fn persisted_calibration_store_round_trips_blob() {
        let blob = PersistedCalibrationBlob::default();

        let mut store = MockPersistedStore(None);
        store.save(&blob).unwrap();

        assert_eq!(store.load().unwrap(), Some(blob));
    }

    #[test]
    fn calibration_diff_carries_revision_metadata() {
        let active = ActiveCalibration::new(CalibrationRevision::new(2), Calibration::default());
        let staged = StagedCalibration::new(CalibrationRevision::new(5), Calibration::default());
        let diff = CalibrationDiff::from_views(active, staged, CalibrationClass::RuntimeSafe);

        assert_eq!(diff.base_revision.get(), 2);
        assert_eq!(diff.staged_revision.get(), 5);
        assert_eq!(diff.class, CalibrationClass::RuntimeSafe);
    }

    #[test]
    fn commit_rules_allow_metadata_and_runtime_safe_changes_when_safe() {
        let diff = CalibrationDiff::new(
            CalibrationRevision::new(1),
            CalibrationRevision::new(2),
            CalibrationClass::MetadataOnly,
        );

        assert_eq!(
            CommitRules::evaluate(ecu_domain::CommitPolicy::Immediate, diff, false),
            CommitVerdict::AllowNow
        );

        let diff = CalibrationDiff::new(
            CalibrationRevision::new(1),
            CalibrationRevision::new(2),
            CalibrationClass::RuntimeSensitive,
        );

        assert_eq!(
            CommitRules::evaluate(ecu_domain::CommitPolicy::SafeOnly, diff, false),
            CommitVerdict::Defer
        );
        assert_eq!(
            CommitRules::evaluate(ecu_domain::CommitPolicy::SafeOnly, diff, true),
            CommitVerdict::AllowNow
        );
    }

    #[test]
    fn commit_rules_defer_all_when_policy_is_deferred() {
        let diff = CalibrationDiff::new(
            CalibrationRevision::new(1),
            CalibrationRevision::new(3),
            CalibrationClass::Mixed,
        );

        assert_eq!(
            CommitRules::evaluate(ecu_domain::CommitPolicy::Deferred, diff, true),
            CommitVerdict::Defer
        );
    }

    #[test]
    fn persisted_blob_carries_schema_version_and_revision_metadata() {
        let mut staged =
            StagedCalibration::new(CalibrationRevision::new(4), Calibration::default());
        staged.mark_dirty();
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged,
        };
        let blob = PersistedCalibrationBlob::new(snapshot);

        assert_eq!(blob.schema_version(), CalibrationSchemaVersion::CURRENT);
        assert_eq!(blob.revision().get(), 9);
        assert_eq!(blob.snapshot(), snapshot);
    }

    #[test]
    fn expert_trigger_roundtrip_preserves_all_fields() {
        let expert = ExpertTriggerCalibration {
            schema_version: CalibrationSchemaVersion::CURRENT,
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            trigger_pattern: TriggerPattern::MissingTooth,
            primary_base_teeth: 60,
            missing_teeth: 2,
            primary_trigger_speed: PrimaryTriggerSpeed::Crank,
            trigger_angle_atdc_deg10: 840,
            trigger_angle_multiplier: 2,
            primary_trigger_edge: TriggerEdge::Falling,
            secondary_trigger_edge: TriggerEdge::Rising,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            poll_level_polarity: PollLevelPolarity::High,
            trigger_filter: TriggerFilter::Aggressive,
            resync_every_cycle: true,
            skip_cycles: 3,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            injection_layout: ExpertInjectionLayout::Sequential,
            fixed_timing_mode: FixedTimingMode::Fixed,
            fixed_timing_deg10: 100,
        };
        let mut bytes = [0u8; EXPERT_TRIGGER_RECORD_LEN];

        assert_eq!(
            expert.encode_record(&mut bytes),
            Ok(EXPERT_TRIGGER_RECORD_LEN)
        );

        let decoded = ExpertTriggerCalibration::decode_record(&bytes).expect("decode record");
        assert_eq!(decoded, expert);
    }

    #[test]
    fn invalid_expert_trigger_config_is_rejected() {
        let locked_manual = ExpertTriggerCalibration {
            authority: TriggerAuthority::ExpertManual,
            ..ExpertTriggerCalibration::default()
        };
        assert_eq!(
            locked_manual.validate(),
            Err(ExpertTriggerValidationError::ExpertUnlockRequired)
        );

        let impossible_missing_tooth = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            primary_base_teeth: 2,
            missing_teeth: 2,
            ..ExpertTriggerCalibration::default()
        };
        assert_eq!(
            impossible_missing_tooth.validate(),
            Err(ExpertTriggerValidationError::InvalidMissingTeeth)
        );
    }

    #[test]
    fn sequential_modes_require_secondary_trigger_evidence() {
        let no_cam = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            injection_layout: ExpertInjectionLayout::Sequential,
            ..ExpertTriggerCalibration::default()
        };

        assert_eq!(
            no_cam.validate(),
            Err(ExpertTriggerValidationError::SecondaryRequired)
        );
    }

    #[test]
    fn certified_profile_requires_trusted_transition_surface() {
        let current = ExpertTriggerCalibration::default();
        let certified = ExpertTriggerCalibration {
            authority: TriggerAuthority::CertifiedProfile,
            profile_identity: 0x4D353054,
            profile_hash: 0x55AA_1234,
            ..ExpertTriggerCalibration::default()
        };
        assert_eq!(
            certified.validate_transition_from(&current),
            Err(ExpertTriggerValidationError::CertifiedProfileRequiresTrustedPath)
        );

        let current = certified;
        let stale_manual_blob = ExpertTriggerCalibration {
            trigger_angle_atdc_deg10: 120,
            ..current
        };
        let explicit_expert_conversion = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            trigger_angle_atdc_deg10: 120,
            ..current
        };

        assert_eq!(
            stale_manual_blob.validate_transition_from(&current),
            Err(ExpertTriggerValidationError::CertifiedProfileOverwrite)
        );
        assert_eq!(
            explicit_expert_conversion.validate_transition_from(&current),
            Ok(())
        );
    }

    #[test]
    fn certified_profile_cannot_be_unlocked_into_manual_path() {
        let unlocked_certified = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::CertifiedProfile,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            ..ExpertTriggerCalibration::default()
        };

        assert_eq!(
            unlocked_certified.validate(),
            Err(ExpertTriggerValidationError::CertifiedProfileMustStayLocked)
        );
    }

    #[test]
    fn validated_manual_trigger_can_promote_primary_locked_startup_authority() {
        let calibration = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            injection_layout: ExpertInjectionLayout::Sequential,
            ..ExpertTriggerCalibration::default()
        };
        let startup_authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            3,
        );

        let runtime_authority = calibration
            .to_runtime_engine_time_authority(startup_authority)
            .expect("manual authority");

        assert_eq!(runtime_authority.crank, CrankSyncState::PrimaryLocked);
        assert_eq!(runtime_authority.phase, PhaseSyncState::CamValidated720);
        assert_eq!(
            runtime_authority.absolute,
            AbsoluteTimeAuthority::ExpertManual
        );
        assert_eq!(runtime_authority.sync_loss_count, 3);
        assert!(runtime_authority.has_primary_lock());
    }

    #[test]
    fn manual_runtime_authority_requires_primary_lock_and_720_for_sequential_cop() {
        let calibration = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            injection_layout: ExpertInjectionLayout::Sequential,
            ..ExpertTriggerCalibration::default()
        };
        let crank_only = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CrankOnly360,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );
        let no_lock = EngineTimeAuthority::new(
            CrankSyncState::NoSignal,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );

        assert_eq!(
            calibration.to_runtime_engine_time_authority(crank_only),
            Err(ExpertTriggerRuntimeAuthorityError::CamValidated720Required)
        );
        assert_eq!(
            calibration.to_runtime_engine_time_authority(no_lock),
            Err(ExpertTriggerRuntimeAuthorityError::DecoderPrimaryLockRequired)
        );
    }

    #[test]
    fn certified_profile_does_not_convert_to_manual_runtime_authority() {
        let calibration = ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Locked,
            authority: TriggerAuthority::CertifiedProfile,
            profile_identity: 0x4D353054,
            profile_hash: 0xA5A5_1234,
            ..ExpertTriggerCalibration::default()
        };
        let startup_authority = EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
            EngineTimeAuthority::MAX_CONFIDENCE_X1000,
            0,
        );

        assert_eq!(
            calibration.to_runtime_engine_time_authority(startup_authority),
            Err(ExpertTriggerRuntimeAuthorityError::ExpertManualAuthorityRequired)
        );
    }

    #[test]
    fn expert_unlock_is_persisted_with_schema_version() {
        let mut calibration = Calibration::default();
        calibration.set_expert_trigger(ExpertTriggerCalibration {
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
            ignition_mode: ExpertIgnitionMode::SequentialCop,
            ..ExpertTriggerCalibration::default()
        });
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(11), calibration),
            staged: StagedCalibration::new(CalibrationRevision::new(11), calibration),
        };
        let blob = PersistedCalibrationBlob::new(snapshot);

        assert_eq!(blob.schema_version(), CalibrationSchemaVersion::CURRENT);
        assert_eq!(
            blob.snapshot()
                .active
                .calibration()
                .geometry
                .expert_trigger
                .schema_version,
            CalibrationSchemaVersion::CURRENT
        );
        assert_eq!(
            blob.snapshot()
                .active
                .calibration()
                .geometry
                .expert_trigger
                .expert_unlock,
            ExpertUnlock::Unlocked
        );
    }
}
