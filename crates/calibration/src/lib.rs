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
    ActiveCalibration, Calibration, CalibrationClass, CalibrationDiff, CalibrationHardwareTargetId,
    CalibrationPackageChecksum, CalibrationPackageCompareResult, CalibrationPackageCompatibility,
    CalibrationPackageIdentity, CalibrationPackageMigrationResult,
    CalibrationPackageMigrationStatus, CalibrationPackageReview, CalibrationPackageTargetIdentity,
    CalibrationPackageWireError, CalibrationRevision, CalibrationRuntimeBuildId,
    CalibrationSchemaVersion, CalibrationSnapshot, CommitRules, CommitVerdict,
    PersistedCalibrationBlob, PersistedCalibrationPackage, PersistedCalibrationStore,
    StagedCalibration,
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
    fn calibration_package_identity_from_default_snapshot_is_empty() {
        let identity = CalibrationPackageIdentity::from_snapshot(CalibrationSnapshot::default());

        assert_eq!(identity.schema_version, CalibrationSchemaVersion::CURRENT);
        assert_eq!(identity.active_revision, CalibrationRevision::default());
        assert_eq!(
            identity.staged_base_revision,
            CalibrationRevision::default()
        );
        assert_eq!(identity.staged_revision, CalibrationRevision::default());
        assert!(!identity.staged_dirty);
        assert_eq!(
            identity.checksum,
            CalibrationPackageChecksum::from_snapshot(
                CalibrationSchemaVersion::CURRENT,
                CalibrationSnapshot::default(),
                false,
            )
        );
    }

    #[test]
    fn calibration_package_identity_from_dirty_snapshot_carries_revisions_and_dirty_flag() {
        let mut staged =
            StagedCalibration::new(CalibrationRevision::new(4), Calibration::default());
        staged.mark_dirty();
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged,
        };

        let identity = CalibrationPackageIdentity::from_snapshot(snapshot);

        assert_eq!(identity.schema_version, CalibrationSchemaVersion::CURRENT);
        assert_eq!(identity.active_revision, CalibrationRevision::new(9));
        assert_eq!(identity.staged_base_revision, CalibrationRevision::new(4));
        assert_eq!(identity.staged_revision, CalibrationRevision::new(5));
        assert!(identity.staged_dirty);
        assert_eq!(
            identity.checksum,
            CalibrationPackageChecksum::from_snapshot(
                CalibrationSchemaVersion::CURRENT,
                snapshot,
                true,
            )
        );
    }

    #[test]
    fn calibration_package_identity_from_blob_uses_blob_schema_version_and_snapshot_revisions() {
        let mut staged =
            StagedCalibration::new(CalibrationRevision::new(4), Calibration::default());
        staged.mark_dirty();
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged,
        };
        let blob = PersistedCalibrationBlob::new(snapshot);

        let identity = CalibrationPackageIdentity::from_blob(blob);

        assert_eq!(identity.schema_version, CalibrationSchemaVersion::CURRENT);
        assert_eq!(identity.active_revision, CalibrationRevision::new(9));
        assert_eq!(identity.staged_base_revision, CalibrationRevision::new(4));
        assert_eq!(identity.staged_revision, CalibrationRevision::new(5));
        assert!(identity.staged_dirty);
        assert_eq!(
            identity.checksum,
            CalibrationPackageChecksum::from_blob(blob)
        );
    }

    #[test]
    fn calibration_package_checksum_changes_with_package_payload() {
        let base = CalibrationSnapshot::default();
        let mut active_calibration = Calibration::default();
        active_calibration.set_expert_trigger(sample_wire_expert_trigger(0xA11C_E550, 100));
        let changed = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(1), active_calibration),
            staged: StagedCalibration::default(),
        };

        let base_checksum = CalibrationPackageIdentity::from_snapshot(base).checksum;
        let changed_checksum = CalibrationPackageIdentity::from_snapshot(changed).checksum;

        assert_ne!(base_checksum, changed_checksum);
        assert_ne!(base_checksum.get(), 0);
        assert_ne!(changed_checksum.get(), 0);
    }

    #[test]
    fn calibration_package_checksum_changes_between_invalid_trigger_payloads() {
        let first_trigger = ExpertTriggerCalibration {
            primary_base_teeth: 0,
            profile_hash: 0x1111_2222,
            ..ExpertTriggerCalibration::default()
        };
        let mut first_calibration = Calibration::default();
        first_calibration.set_expert_trigger(first_trigger);

        let mut second_trigger = first_trigger;
        second_trigger.profile_hash = 0x3333_4444;
        let mut second_calibration = Calibration::default();
        second_calibration.set_expert_trigger(second_trigger);

        let first = CalibrationPackageIdentity::from_snapshot(CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::default(), first_calibration),
            staged: StagedCalibration::default(),
        });
        let second = CalibrationPackageIdentity::from_snapshot(CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::default(), second_calibration),
            staged: StagedCalibration::default(),
        });

        assert_ne!(first.checksum, second.checksum);
    }

    #[test]
    fn calibration_package_target_identity_from_snapshot_carries_target_metadata() {
        let mut staged =
            StagedCalibration::new(CalibrationRevision::new(4), Calibration::default());
        staged.mark_dirty();
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged,
        };

        let identity = CalibrationPackageTargetIdentity::from_snapshot(
            snapshot,
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(
            identity.package.schema_version,
            CalibrationSchemaVersion::CURRENT
        );
        assert_eq!(
            identity.package.active_revision,
            CalibrationRevision::new(9)
        );
        assert_eq!(
            identity.package.staged_base_revision,
            CalibrationRevision::new(4)
        );
        assert_eq!(
            identity.package.staged_revision,
            CalibrationRevision::new(5)
        );
        assert!(identity.package.staged_dirty);
        assert_eq!(identity.runtime_build_id.get(), 0x1234_5678);
        assert_eq!(identity.hardware_target_id.get(), 0x2040);
    }

    #[test]
    fn persisted_calibration_package_from_blob_carries_identity_and_blob() {
        let mut staged =
            StagedCalibration::new(CalibrationRevision::new(4), Calibration::default());
        staged.mark_dirty();
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged,
        };
        let blob = PersistedCalibrationBlob::new(snapshot);

        let package = PersistedCalibrationPackage::new(
            blob,
            CalibrationRuntimeBuildId::new(0x0000_0007),
            CalibrationHardwareTargetId::new(0x00A7),
        );

        assert_eq!(package.blob, blob);
        assert_eq!(
            package.identity.package,
            CalibrationPackageIdentity::from_blob(blob)
        );
        assert_eq!(package.identity.runtime_build_id.get(), 0x0000_0007);
        assert_eq!(package.identity.hardware_target_id.get(), 0x00A7);
    }

    #[test]
    fn calibration_package_target_identity_reports_compatibility_and_mismatches() {
        let identity = CalibrationPackageTargetIdentity::from_snapshot(
            CalibrationSnapshot::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(
            identity.compatibility_with(
                CalibrationRuntimeBuildId::new(0x1234_5678),
                CalibrationHardwareTargetId::new(0x2040),
            ),
            CalibrationPackageCompatibility::Compatible
        );
        assert_eq!(
            identity.compatibility_with(
                CalibrationRuntimeBuildId::new(0x8765_4321),
                CalibrationHardwareTargetId::new(0x2040),
            ),
            CalibrationPackageCompatibility::RuntimeBuildMismatch {
                expected: CalibrationRuntimeBuildId::new(0x8765_4321),
                actual: CalibrationRuntimeBuildId::new(0x1234_5678),
            }
        );
        assert_eq!(
            identity.compatibility_with(
                CalibrationRuntimeBuildId::new(0x1234_5678),
                CalibrationHardwareTargetId::new(0xF405),
            ),
            CalibrationPackageCompatibility::HardwareTargetMismatch {
                expected: CalibrationHardwareTargetId::new(0xF405),
                actual: CalibrationHardwareTargetId::new(0x2040),
            }
        );

        let legacy_identity = CalibrationPackageTargetIdentity {
            package: CalibrationPackageIdentity {
                schema_version: CalibrationSchemaVersion::new(0),
                ..CalibrationPackageIdentity::default()
            },
            runtime_build_id: CalibrationRuntimeBuildId::new(0x1234_5678),
            hardware_target_id: CalibrationHardwareTargetId::new(0x2040),
        };
        assert_eq!(
            legacy_identity.compatibility_with(
                CalibrationRuntimeBuildId::new(0x1234_5678),
                CalibrationHardwareTargetId::new(0x2040),
            ),
            CalibrationPackageCompatibility::SchemaVersionMismatch {
                expected: CalibrationSchemaVersion::CURRENT,
                actual: CalibrationSchemaVersion::new(0),
            }
        );
    }

    #[test]
    fn persisted_calibration_package_review_marks_current_package_compatible_without_migration() {
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let review = package.review_against(
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(review.package, package.identity);
        assert_eq!(
            review.compatibility,
            CalibrationPackageCompatibility::Compatible
        );
        assert_eq!(
            review.migration,
            CalibrationPackageMigrationStatus::NoMigrationRequired
        );
    }

    #[test]
    fn persisted_calibration_package_review_reports_target_mismatch_without_migration() {
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let review = package.review_against(
            CalibrationRuntimeBuildId::new(0x8765_4321),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(review.package, package.identity);
        assert_eq!(
            review.compatibility,
            CalibrationPackageCompatibility::RuntimeBuildMismatch {
                expected: CalibrationRuntimeBuildId::new(0x8765_4321),
                actual: CalibrationRuntimeBuildId::new(0x1234_5678),
            }
        );
        assert_eq!(
            review.migration,
            CalibrationPackageMigrationStatus::NoMigrationRequired
        );
    }

    #[test]
    fn persisted_calibration_package_review_marks_older_schema_as_migration_required() {
        let blob = PersistedCalibrationBlob::default();
        let package = PersistedCalibrationPackage {
            blob,
            identity: CalibrationPackageTargetIdentity {
                package: CalibrationPackageIdentity {
                    schema_version: CalibrationSchemaVersion::new(0),
                    ..CalibrationPackageIdentity::from_blob(blob)
                },
                runtime_build_id: CalibrationRuntimeBuildId::new(0x1234_5678),
                hardware_target_id: CalibrationHardwareTargetId::new(0x2040),
            },
        };

        let review = package.review_against(
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(review.package, package.identity);
        assert_eq!(
            review.compatibility,
            CalibrationPackageCompatibility::SchemaVersionMismatch {
                expected: CalibrationSchemaVersion::CURRENT,
                actual: CalibrationSchemaVersion::new(0),
            }
        );
        assert_eq!(
            review.migration,
            CalibrationPackageMigrationStatus::MigrationRequired {
                from: CalibrationSchemaVersion::new(0),
                to: CalibrationSchemaVersion::CURRENT,
            }
        );
    }

    #[test]
    fn persisted_calibration_package_compare_reports_no_change_for_identical_identity() {
        let current = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let compare = current.compare_against(current);

        assert_eq!(compare.current, current.identity);
        assert_eq!(compare.candidate, current.identity);
        assert!(!compare.any_change());
        assert!(!compare.schema_changed);
        assert!(!compare.runtime_build_changed);
        assert!(!compare.hardware_target_changed);
        assert!(!compare.checksum_changed);
        assert!(!compare.active_revision_changed);
        assert!(!compare.staged_base_revision_changed);
        assert!(!compare.staged_revision_changed);
        assert!(!compare.staged_dirty_changed);
    }

    #[test]
    fn persisted_calibration_package_compare_reports_revision_change() {
        let current = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );
        let candidate_blob = PersistedCalibrationBlob::new(CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged: StagedCalibration::new(CalibrationRevision::new(9), Calibration::default()),
        });
        let candidate = PersistedCalibrationPackage::new(
            candidate_blob,
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let compare = current.compare_against(candidate);

        assert!(compare.any_change());
        assert!(!compare.schema_changed);
        assert!(!compare.runtime_build_changed);
        assert!(!compare.hardware_target_changed);
        assert!(compare.checksum_changed);
        assert!(compare.active_revision_changed);
        assert!(compare.staged_base_revision_changed);
        assert!(compare.staged_revision_changed);
        assert!(!compare.staged_dirty_changed);
    }

    #[test]
    fn persisted_calibration_package_compare_reports_target_change() {
        let current = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );
        let candidate = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x8765_4321),
            CalibrationHardwareTargetId::new(0xF405),
        );

        let compare = current.compare_against(candidate);

        assert!(compare.any_change());
        assert!(!compare.schema_changed);
        assert!(!compare.checksum_changed);
        assert!(compare.runtime_build_changed);
        assert!(compare.hardware_target_changed);
        assert!(!compare.active_revision_changed);
        assert!(!compare.staged_base_revision_changed);
        assert!(!compare.staged_revision_changed);
        assert!(!compare.staged_dirty_changed);
    }

    #[test]
    fn persisted_calibration_package_compare_reports_checksum_only_change() {
        let current = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );
        let mut active_calibration = Calibration::default();
        active_calibration.set_expert_trigger(sample_wire_expert_trigger(0xA11C_E550, 100));
        let candidate_blob = PersistedCalibrationBlob::new(CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::default(), active_calibration),
            staged: StagedCalibration::default(),
        });
        let candidate = PersistedCalibrationPackage::new(
            candidate_blob,
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let compare = current.compare_against(candidate);

        assert!(compare.any_change());
        assert!(!compare.schema_changed);
        assert!(compare.checksum_changed);
        assert!(!compare.runtime_build_changed);
        assert!(!compare.hardware_target_changed);
        assert!(!compare.active_revision_changed);
        assert!(!compare.staged_base_revision_changed);
        assert!(!compare.staged_revision_changed);
        assert!(!compare.staged_dirty_changed);
    }

    #[test]
    fn persisted_calibration_package_migration_leaves_current_package_unchanged() {
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let migrated = package.migrate_against(
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(
            migrated,
            CalibrationPackageMigrationResult::Unchanged {
                review: package.review_against(
                    CalibrationRuntimeBuildId::new(0x1234_5678),
                    CalibrationHardwareTargetId::new(0x2040),
                ),
                package,
            }
        );
    }

    #[test]
    fn persisted_calibration_package_migration_normalizes_older_schema_package() {
        let snapshot = CalibrationSnapshot {
            active: ActiveCalibration::new(CalibrationRevision::new(9), Calibration::default()),
            staged: StagedCalibration::new(CalibrationRevision::new(9), Calibration::default()),
        };
        let blob = PersistedCalibrationBlob::new_with_schema_version_for_test(
            snapshot,
            CalibrationSchemaVersion::new(0),
        );
        let legacy = PersistedCalibrationPackage {
            blob,
            identity: CalibrationPackageTargetIdentity {
                package: CalibrationPackageIdentity::from_blob(blob),
                runtime_build_id: CalibrationRuntimeBuildId::new(0x1234_5678),
                hardware_target_id: CalibrationHardwareTargetId::new(0x2040),
            },
        };

        let migrated = legacy.migrate_against(
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let CalibrationPackageMigrationResult::Migrated { review, package } = migrated else {
            panic!("expected migrated package result");
        };

        assert_eq!(
            review,
            legacy.review_against(
                CalibrationRuntimeBuildId::new(0x1234_5678),
                CalibrationHardwareTargetId::new(0x2040),
            )
        );
        assert_eq!(
            review.compatibility,
            CalibrationPackageCompatibility::SchemaVersionMismatch {
                expected: CalibrationSchemaVersion::CURRENT,
                actual: CalibrationSchemaVersion::new(0),
            }
        );
        assert_eq!(
            review.migration,
            CalibrationPackageMigrationStatus::MigrationRequired {
                from: CalibrationSchemaVersion::new(0),
                to: CalibrationSchemaVersion::CURRENT,
            }
        );
        assert_eq!(
            package.identity.package.schema_version,
            CalibrationSchemaVersion::CURRENT
        );
        assert_eq!(
            package.blob.schema_version(),
            CalibrationSchemaVersion::CURRENT
        );
        assert_eq!(package.identity.runtime_build_id.get(), 0x1234_5678);
        assert_eq!(package.identity.hardware_target_id.get(), 0x2040);
        assert_eq!(package.blob.snapshot(), legacy.blob.snapshot());
    }

    #[test]
    fn persisted_calibration_package_migration_rejects_runtime_build_mismatch() {
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );

        let migrated = package.migrate_against(
            CalibrationRuntimeBuildId::new(0x8765_4321),
            CalibrationHardwareTargetId::new(0x2040),
        );

        assert_eq!(
            migrated,
            CalibrationPackageMigrationResult::Rejected {
                review: package.review_against(
                    CalibrationRuntimeBuildId::new(0x8765_4321),
                    CalibrationHardwareTargetId::new(0x2040),
                ),
            }
        );
    }

    #[test]
    fn persisted_calibration_package_wire_roundtrips_current_package() {
        let mut active_calibration = Calibration::default();
        active_calibration.set_expert_trigger(sample_wire_expert_trigger(0xA11C_E550, 100));
        let mut staged_calibration = Calibration::default();
        staged_calibration.set_expert_trigger(sample_wire_expert_trigger(0xB11C_E551, 250));
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::new(CalibrationSnapshot {
                active: ActiveCalibration::new(CalibrationRevision::new(9), active_calibration),
                staged: {
                    let mut staged =
                        StagedCalibration::new(CalibrationRevision::new(4), staged_calibration);
                    staged.mark_dirty();
                    staged
                },
            }),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );
        let mut bytes = [0u8; PersistedCalibrationPackage::WIRE_LEN];

        let len = package.encode_wire(&mut bytes).expect("encode package");
        let decoded = PersistedCalibrationPackage::decode_wire(&bytes).expect("decode package");

        assert_eq!(len, PersistedCalibrationPackage::WIRE_LEN);
        assert_eq!(decoded, package);
    }

    #[test]
    fn persisted_calibration_package_wire_preserves_metadata_and_snapshot() {
        let mut active_calibration = Calibration::default();
        active_calibration.set_expert_trigger(sample_wire_expert_trigger(0xA11C_E550, 100));
        let mut staged_calibration = Calibration::default();
        staged_calibration.set_expert_trigger(sample_wire_expert_trigger(0xB11C_E551, 250));
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::new(CalibrationSnapshot {
                active: ActiveCalibration::new(CalibrationRevision::new(17), active_calibration),
                staged: {
                    let mut staged =
                        StagedCalibration::new(CalibrationRevision::new(12), staged_calibration);
                    staged.mark_dirty();
                    staged
                },
            }),
            CalibrationRuntimeBuildId::new(0x8765_4321),
            CalibrationHardwareTargetId::new(0xF405),
        );
        let mut bytes = [0u8; PersistedCalibrationPackage::WIRE_LEN];

        package.encode_wire(&mut bytes).expect("encode package");
        let decoded = PersistedCalibrationPackage::decode_wire(&bytes).expect("decode package");

        assert_eq!(
            decoded.identity.runtime_build_id,
            package.identity.runtime_build_id
        );
        assert_eq!(
            decoded.identity.hardware_target_id,
            package.identity.hardware_target_id
        );
        assert_eq!(
            decoded.identity.package.schema_version,
            package.identity.package.schema_version
        );
        assert_eq!(
            decoded.identity.package.active_revision,
            package.identity.package.active_revision
        );
        assert_eq!(
            decoded.identity.package.staged_base_revision,
            package.identity.package.staged_base_revision
        );
        assert_eq!(
            decoded.identity.package.staged_revision,
            package.identity.package.staged_revision
        );
        assert_eq!(
            decoded.identity.package.staged_dirty,
            package.identity.package.staged_dirty
        );
        assert_eq!(decoded.blob.snapshot(), package.blob.snapshot());
    }

    #[test]
    fn persisted_calibration_package_wire_rejects_truncated_bytes() {
        let err = PersistedCalibrationPackage::decode_wire(&[0u8; 12]).expect_err("short wire");
        assert_eq!(err, CalibrationPackageWireError::WrongSize);
    }

    #[test]
    fn persisted_calibration_package_wire_rejects_malformed_expert_trigger_payload() {
        let package = PersistedCalibrationPackage::new(
            PersistedCalibrationBlob::default(),
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        );
        let mut bytes = [0u8; PersistedCalibrationPackage::WIRE_LEN];
        package.encode_wire(&mut bytes).expect("encode package");
        let active_start = 28;
        bytes[active_start + 29] = 0xFF;

        let err = PersistedCalibrationPackage::decode_wire(&bytes).expect_err("malformed wire");
        assert!(matches!(
            err,
            CalibrationPackageWireError::ActiveExpertTrigger(
                ExpertTriggerRecordError::NonCanonicalRecord
            )
        ));
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

    fn sample_wire_expert_trigger(
        profile_hash: u32,
        fixed_timing_deg10: i16,
    ) -> ExpertTriggerCalibration {
        ExpertTriggerCalibration {
            schema_version: CalibrationSchemaVersion::CURRENT,
            expert_unlock: ExpertUnlock::Unlocked,
            authority: TriggerAuthority::ExpertManual,
            profile_identity: 0x4D353054,
            profile_hash,
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
            fixed_timing_deg10,
        }
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
