use crate::{ExpertTriggerCalibration, ExpertTriggerRecordError, EXPERT_TRIGGER_RECORD_LEN};

/// Top-level calibration container grouped by domain.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Calibration {
    pub geometry: GeometryCalibration,
    pub fuel: FuelCalibration,
    pub ignition: IgnitionCalibration,
    pub lambda: LambdaCalibration,
    pub torque: TorqueCalibration,
    pub idle: IdleCalibration,
    pub aux: AuxCalibration,
    pub safety: SafetyCalibration,
    pub sensors: SensorCalibration,
}

impl Calibration {
    pub fn set_expert_trigger(&mut self, expert_trigger: ExpertTriggerCalibration) {
        self.geometry.expert_trigger = expert_trigger;
    }
}

/// Monotonic calibration revision identifier.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationRevision(u32);

impl CalibrationRevision {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Calibration currently in use by the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ActiveCalibration {
    revision: CalibrationRevision,
    calibration: Calibration,
}

impl ActiveCalibration {
    pub const fn new(revision: CalibrationRevision, calibration: Calibration) -> Self {
        Self {
            revision,
            calibration,
        }
    }

    pub const fn revision(self) -> CalibrationRevision {
        self.revision
    }

    pub const fn calibration(self) -> Calibration {
        self.calibration
    }
}

/// Calibration that is being edited before activation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StagedCalibration {
    base_revision: CalibrationRevision,
    revision: CalibrationRevision,
    dirty: bool,
    calibration: Calibration,
}

impl StagedCalibration {
    pub const fn new(base_revision: CalibrationRevision, calibration: Calibration) -> Self {
        Self {
            base_revision,
            revision: base_revision,
            dirty: false,
            calibration,
        }
    }

    pub const fn base_revision(self) -> CalibrationRevision {
        self.base_revision
    }

    pub const fn revision(self) -> CalibrationRevision {
        self.revision
    }

    pub const fn calibration(self) -> Calibration {
        self.calibration
    }

    pub const fn is_dirty(self) -> bool {
        self.dirty
    }

    pub fn mark_dirty(&mut self) {
        self.dirty = true;
        self.revision = CalibrationRevision::new(self.revision.get().saturating_add(1));
    }

    pub fn clear_dirty(&mut self) {
        self.dirty = false;
    }

    pub fn replace_calibration(&mut self, calibration: Calibration) {
        if self.calibration != calibration {
            self.calibration = calibration;
            self.mark_dirty();
        }
    }
}

/// Snapshot of both calibration views.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CalibrationSnapshot {
    pub active: ActiveCalibration,
    pub staged: StagedCalibration,
}

/// Compact calibration package identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CalibrationPackageIdentity {
    pub schema_version: CalibrationSchemaVersion,
    pub active_revision: CalibrationRevision,
    pub staged_base_revision: CalibrationRevision,
    pub staged_revision: CalibrationRevision,
    pub staged_dirty: bool,
    pub checksum: CalibrationPackageChecksum,
}

impl CalibrationPackageIdentity {
    pub fn from_snapshot(snapshot: CalibrationSnapshot) -> Self {
        Self::from_snapshot_with_staged_dirty(snapshot, snapshot.staged.is_dirty())
    }

    pub fn from_snapshot_with_staged_dirty(
        snapshot: CalibrationSnapshot,
        staged_dirty: bool,
    ) -> Self {
        Self {
            schema_version: CalibrationSchemaVersion::CURRENT,
            active_revision: snapshot.active.revision(),
            staged_base_revision: snapshot.staged.base_revision(),
            staged_revision: snapshot.staged.revision(),
            staged_dirty,
            checksum: CalibrationPackageChecksum::from_snapshot(
                CalibrationSchemaVersion::CURRENT,
                snapshot,
                staged_dirty,
            ),
        }
    }

    pub fn from_blob(blob: PersistedCalibrationBlob) -> Self {
        let snapshot = blob.snapshot();

        Self {
            schema_version: blob.schema_version(),
            active_revision: snapshot.active.revision(),
            staged_base_revision: snapshot.staged.base_revision(),
            staged_revision: snapshot.staged.revision(),
            staged_dirty: snapshot.staged.is_dirty(),
            checksum: CalibrationPackageChecksum::from_snapshot(
                blob.schema_version(),
                snapshot,
                snapshot.staged.is_dirty(),
            ),
        }
    }
}

/// Deterministic checksum for the canonical calibration package payload.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationPackageChecksum(u32);

impl CalibrationPackageChecksum {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }

    pub fn from_blob(blob: PersistedCalibrationBlob) -> Self {
        let snapshot = blob.snapshot();
        Self::from_snapshot(blob.schema_version(), snapshot, snapshot.staged.is_dirty())
    }

    pub fn from_snapshot(
        schema_version: CalibrationSchemaVersion,
        snapshot: CalibrationSnapshot,
        staged_dirty: bool,
    ) -> Self {
        let mut state = crc32_init();
        state = crc32_update(state, &schema_version.get().to_le_bytes());
        state = crc32_update(state, &snapshot.active.revision().get().to_le_bytes());
        state = crc32_update(state, &snapshot.staged.base_revision().get().to_le_bytes());
        state = crc32_update(state, &snapshot.staged.revision().get().to_le_bytes());
        state = crc32_update(state, &[u8::from(staged_dirty), 0, 0, 0]);

        let active_record =
            expert_trigger_checksum_record(snapshot.active.calibration().geometry.expert_trigger);
        state = crc32_update(state, &active_record);

        let staged_record =
            expert_trigger_checksum_record(snapshot.staged.calibration().geometry.expert_trigger);
        state = crc32_update(state, &staged_record);

        Self(crc32_finish(state))
    }
}

const fn crc32_init() -> u32 {
    0xffff_ffff
}

const fn crc32_finish(state: u32) -> u32 {
    !state
}

fn crc32_update(mut state: u32, bytes: &[u8]) -> u32 {
    for byte in bytes {
        state ^= u32::from(*byte);
        for _ in 0..8 {
            if state & 1 == 1 {
                state = (state >> 1) ^ 0xedb8_8320;
            } else {
                state >>= 1;
            }
        }
    }
    state
}

/// Canonical checksum record for an expert trigger (review 004).
///
/// Always the canonical wire layout: validation is intentionally skipped so a
/// non-canonical fallback record can never be hashed. Invalid trigger states
/// are rejected at the decode/boundary instead.
fn expert_trigger_checksum_record(
    expert_trigger: ExpertTriggerCalibration,
) -> [u8; EXPERT_TRIGGER_RECORD_LEN] {
    let mut record = [0u8; EXPERT_TRIGGER_RECORD_LEN];
    let written = expert_trigger
        .encode_record_unchecked(&mut record)
        .expect("record buffer is exactly EXPERT_TRIGGER_RECORD_LEN");
    debug_assert_eq!(written, EXPERT_TRIGGER_RECORD_LEN);
    record
}

/// Opaque firmware-build identifier carried with a persisted calibration package.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationRuntimeBuildId(u32);

impl CalibrationRuntimeBuildId {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

/// Opaque hardware-target identifier carried with a persisted calibration package.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationHardwareTargetId(u16);

impl CalibrationHardwareTargetId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Persisted calibration package identity bound to a firmware build and hardware target.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CalibrationPackageTargetIdentity {
    pub package: CalibrationPackageIdentity,
    pub runtime_build_id: CalibrationRuntimeBuildId,
    pub hardware_target_id: CalibrationHardwareTargetId,
}

impl CalibrationPackageTargetIdentity {
    pub fn from_snapshot(
        snapshot: CalibrationSnapshot,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Self {
        Self {
            package: CalibrationPackageIdentity::from_snapshot(snapshot),
            runtime_build_id,
            hardware_target_id,
        }
    }

    pub fn from_blob(
        blob: PersistedCalibrationBlob,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Self {
        Self {
            package: CalibrationPackageIdentity::from_blob(blob),
            runtime_build_id,
            hardware_target_id,
        }
    }

    pub fn compatibility_with(
        self,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> CalibrationPackageCompatibility {
        if self.package.schema_version != CalibrationSchemaVersion::CURRENT {
            return CalibrationPackageCompatibility::SchemaVersionMismatch {
                expected: CalibrationSchemaVersion::CURRENT,
                actual: self.package.schema_version,
            };
        }
        if self.runtime_build_id != runtime_build_id {
            return CalibrationPackageCompatibility::RuntimeBuildMismatch {
                expected: runtime_build_id,
                actual: self.runtime_build_id,
            };
        }
        if self.hardware_target_id != hardware_target_id {
            return CalibrationPackageCompatibility::HardwareTargetMismatch {
                expected: hardware_target_id,
                actual: self.hardware_target_id,
            };
        }
        CalibrationPackageCompatibility::Compatible
    }
}

/// Compatibility verdict for a persisted calibration package against a target runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationPackageCompatibility {
    Compatible,
    SchemaVersionMismatch {
        expected: CalibrationSchemaVersion,
        actual: CalibrationSchemaVersion,
    },
    RuntimeBuildMismatch {
        expected: CalibrationRuntimeBuildId,
        actual: CalibrationRuntimeBuildId,
    },
    HardwareTargetMismatch {
        expected: CalibrationHardwareTargetId,
        actual: CalibrationHardwareTargetId,
    },
}

/// Migration status for a reviewed calibration package.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationPackageMigrationStatus {
    NoMigrationRequired,
    MigrationRequired {
        from: CalibrationSchemaVersion,
        to: CalibrationSchemaVersion,
    },
}

/// Compact review result for a persisted calibration package against a target runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationPackageReview {
    pub package: CalibrationPackageTargetIdentity,
    pub compatibility: CalibrationPackageCompatibility,
    pub migration: CalibrationPackageMigrationStatus,
}

/// Migration transform result for a persisted calibration package.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationPackageMigrationResult {
    Unchanged {
        review: CalibrationPackageReview,
        package: PersistedCalibrationPackage,
    },
    Migrated {
        review: CalibrationPackageReview,
        package: PersistedCalibrationPackage,
    },
    Rejected {
        review: CalibrationPackageReview,
    },
}

/// Wire-format decode/encode error for persisted calibration packages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CalibrationPackageWireError {
    WrongSize,
    BadMagic,
    NonCanonical,
    ActiveExpertTrigger(ExpertTriggerRecordError),
    StagedExpertTrigger(ExpertTriggerRecordError),
}

/// Compact metadata comparison result between two target-bound calibration packages.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationPackageCompareResult {
    pub current: CalibrationPackageTargetIdentity,
    pub candidate: CalibrationPackageTargetIdentity,
    pub schema_changed: bool,
    pub checksum_changed: bool,
    pub runtime_build_changed: bool,
    pub hardware_target_changed: bool,
    pub active_revision_changed: bool,
    pub staged_base_revision_changed: bool,
    pub staged_revision_changed: bool,
    pub staged_dirty_changed: bool,
}

impl CalibrationPackageCompareResult {
    pub const fn any_change(self) -> bool {
        self.schema_changed
            || self.checksum_changed
            || self.runtime_build_changed
            || self.hardware_target_changed
            || self.active_revision_changed
            || self.staged_base_revision_changed
            || self.staged_revision_changed
            || self.staged_dirty_changed
    }
}

/// Persisted calibration schema version.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationSchemaVersion(u16);

impl CalibrationSchemaVersion {
    pub const CURRENT: Self = Self(1);

    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

/// Canonical persisted calibration blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistedCalibrationBlob {
    schema_version: CalibrationSchemaVersion,
    revision: CalibrationRevision,
    snapshot: CalibrationSnapshot,
}

impl PersistedCalibrationBlob {
    pub const fn new(snapshot: CalibrationSnapshot) -> Self {
        Self {
            schema_version: CalibrationSchemaVersion::CURRENT,
            revision: snapshot.active.revision(),
            snapshot,
        }
    }

    #[cfg(test)]
    pub(crate) const fn new_with_schema_version_for_test(
        snapshot: CalibrationSnapshot,
        schema_version: CalibrationSchemaVersion,
    ) -> Self {
        Self {
            schema_version,
            revision: snapshot.active.revision(),
            snapshot,
        }
    }

    pub const fn schema_version(self) -> CalibrationSchemaVersion {
        self.schema_version
    }

    pub const fn revision(self) -> CalibrationRevision {
        self.revision
    }

    pub const fn snapshot(self) -> CalibrationSnapshot {
        self.snapshot
    }
}

impl Default for PersistedCalibrationBlob {
    fn default() -> Self {
        Self::new(CalibrationSnapshot::default())
    }
}

/// Persisted calibration package plus compatibility metadata for firmware and target binding.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistedCalibrationPackage {
    pub blob: PersistedCalibrationBlob,
    pub identity: CalibrationPackageTargetIdentity,
}

impl PersistedCalibrationPackage {
    const WIRE_MAGIC: [u8; 4] = *b"PCPK";
    pub const WIRE_LEN: usize = 4 + 4 + 2 + 2 + 4 + 4 + 4 + 1 + 3 + EXPERT_TRIGGER_RECORD_LEN * 2;

    pub fn new(
        blob: PersistedCalibrationBlob,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Self {
        Self {
            identity: CalibrationPackageTargetIdentity::from_blob(
                blob,
                runtime_build_id,
                hardware_target_id,
            ),
            blob,
        }
    }

    pub fn review_against(
        self,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> CalibrationPackageReview {
        let migration = if self.identity.package.schema_version == CalibrationSchemaVersion::CURRENT
        {
            CalibrationPackageMigrationStatus::NoMigrationRequired
        } else {
            CalibrationPackageMigrationStatus::MigrationRequired {
                from: self.identity.package.schema_version,
                to: CalibrationSchemaVersion::CURRENT,
            }
        };

        CalibrationPackageReview {
            package: self.identity,
            compatibility: self
                .identity
                .compatibility_with(runtime_build_id, hardware_target_id),
            migration,
        }
    }

    pub fn compare_against(
        self,
        candidate: PersistedCalibrationPackage,
    ) -> CalibrationPackageCompareResult {
        CalibrationPackageCompareResult {
            current: self.identity,
            candidate: candidate.identity,
            schema_changed: self.identity.package.schema_version
                != candidate.identity.package.schema_version,
            checksum_changed: self.identity.package.checksum != candidate.identity.package.checksum,
            runtime_build_changed: self.identity.runtime_build_id
                != candidate.identity.runtime_build_id,
            hardware_target_changed: self.identity.hardware_target_id
                != candidate.identity.hardware_target_id,
            active_revision_changed: self.identity.package.active_revision
                != candidate.identity.package.active_revision,
            staged_base_revision_changed: self.identity.package.staged_base_revision
                != candidate.identity.package.staged_base_revision,
            staged_revision_changed: self.identity.package.staged_revision
                != candidate.identity.package.staged_revision,
            staged_dirty_changed: self.identity.package.staged_dirty
                != candidate.identity.package.staged_dirty,
        }
    }

    pub fn migrate_against(
        self,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> CalibrationPackageMigrationResult {
        let review = self.review_against(runtime_build_id, hardware_target_id);

        if self.identity.runtime_build_id != runtime_build_id
            || self.identity.hardware_target_id != hardware_target_id
        {
            return CalibrationPackageMigrationResult::Rejected { review };
        }

        if self.identity.package.schema_version == CalibrationSchemaVersion::CURRENT {
            return CalibrationPackageMigrationResult::Unchanged {
                review,
                package: self,
            };
        }

        CalibrationPackageMigrationResult::Migrated {
            review,
            package: PersistedCalibrationPackage::new(
                PersistedCalibrationBlob::new(self.blob.snapshot),
                runtime_build_id,
                hardware_target_id,
            ),
        }
    }

    pub fn encode_wire(self, out: &mut [u8]) -> Result<usize, CalibrationPackageWireError> {
        if out.len() < Self::WIRE_LEN {
            return Err(CalibrationPackageWireError::WrongSize);
        }

        out[..Self::WIRE_LEN].fill(0);
        out[0..4].copy_from_slice(&Self::WIRE_MAGIC);
        out[4..8].copy_from_slice(&self.identity.runtime_build_id.get().to_le_bytes());
        out[8..10].copy_from_slice(&self.identity.hardware_target_id.get().to_le_bytes());
        out[10..12].copy_from_slice(&self.blob.schema_version().get().to_le_bytes());
        out[12..16].copy_from_slice(&self.blob.snapshot().active.revision().get().to_le_bytes());
        out[16..20].copy_from_slice(
            &self
                .blob
                .snapshot()
                .staged
                .base_revision()
                .get()
                .to_le_bytes(),
        );
        out[20..24].copy_from_slice(&self.blob.snapshot().staged.revision().get().to_le_bytes());
        out[24] = u8::from(self.blob.snapshot().staged.is_dirty());

        let active_start = 28;
        let staged_start = active_start + EXPERT_TRIGGER_RECORD_LEN;
        self.blob
            .snapshot()
            .active
            .calibration()
            .geometry
            .expert_trigger
            .encode_record(&mut out[active_start..active_start + EXPERT_TRIGGER_RECORD_LEN])
            .map_err(CalibrationPackageWireError::ActiveExpertTrigger)?;
        self.blob
            .snapshot()
            .staged
            .calibration()
            .geometry
            .expert_trigger
            .encode_record(&mut out[staged_start..staged_start + EXPERT_TRIGGER_RECORD_LEN])
            .map_err(CalibrationPackageWireError::StagedExpertTrigger)?;
        Ok(Self::WIRE_LEN)
    }

    pub fn decode_wire(data: &[u8]) -> Result<Self, CalibrationPackageWireError> {
        if data.len() < Self::WIRE_LEN {
            return Err(CalibrationPackageWireError::WrongSize);
        }
        if data[0..4] != Self::WIRE_MAGIC {
            return Err(CalibrationPackageWireError::BadMagic);
        }
        if data[24] > 1 || data[25] != 0 || data[26] != 0 || data[27] != 0 {
            return Err(CalibrationPackageWireError::NonCanonical);
        }

        let runtime_build_id = CalibrationRuntimeBuildId::new(u32::from_le_bytes([
            data[4], data[5], data[6], data[7],
        ]));
        let hardware_target_id =
            CalibrationHardwareTargetId::new(u16::from_le_bytes([data[8], data[9]]));
        let blob_schema_version =
            CalibrationSchemaVersion::new(u16::from_le_bytes([data[10], data[11]]));
        let active_revision =
            CalibrationRevision::new(u32::from_le_bytes([data[12], data[13], data[14], data[15]]));
        let staged_base_revision =
            CalibrationRevision::new(u32::from_le_bytes([data[16], data[17], data[18], data[19]]));
        let staged_revision =
            CalibrationRevision::new(u32::from_le_bytes([data[20], data[21], data[22], data[23]]));
        let staged_dirty = data[24] == 1;

        let active_start = 28;
        let staged_start = active_start + EXPERT_TRIGGER_RECORD_LEN;
        let active_expert = ExpertTriggerCalibration::decode_record(
            &data[active_start..active_start + EXPERT_TRIGGER_RECORD_LEN],
        )
        .map_err(CalibrationPackageWireError::ActiveExpertTrigger)?;
        let staged_expert = ExpertTriggerCalibration::decode_record(
            &data[staged_start..staged_start + EXPERT_TRIGGER_RECORD_LEN],
        )
        .map_err(CalibrationPackageWireError::StagedExpertTrigger)?;

        let mut active_calibration = Calibration::default();
        active_calibration.set_expert_trigger(active_expert);
        let mut staged_calibration = Calibration::default();
        staged_calibration.set_expert_trigger(staged_expert);

        let active = ActiveCalibration::new(active_revision, active_calibration);
        let staged = StagedCalibration {
            base_revision: staged_base_revision,
            revision: staged_revision,
            dirty: staged_dirty,
            calibration: staged_calibration,
        };
        let snapshot = CalibrationSnapshot { active, staged };
        let blob = PersistedCalibrationBlob {
            schema_version: blob_schema_version,
            revision: active_revision,
            snapshot,
        };

        Ok(Self::new(blob, runtime_build_id, hardware_target_id))
    }
}

/// Persistence surface for canonical calibration blobs.
pub trait PersistedCalibrationStore {
    type Error;

    fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error>;

    fn save(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error>;
}

/// Coarse classification for a staged calibration change.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum CalibrationClass {
    NoChange,
    MetadataOnly,
    RuntimeSafe,
    RuntimeSensitive,
    SafetyCritical,
    Mixed,
}

/// Summary of the difference between active and staged calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationDiff {
    pub base_revision: CalibrationRevision,
    pub staged_revision: CalibrationRevision,
    pub class: CalibrationClass,
}

impl CalibrationDiff {
    pub const fn new(
        base_revision: CalibrationRevision,
        staged_revision: CalibrationRevision,
        class: CalibrationClass,
    ) -> Self {
        Self {
            base_revision,
            staged_revision,
            class,
        }
    }

    pub const fn from_views(
        active: ActiveCalibration,
        staged: StagedCalibration,
        class: CalibrationClass,
    ) -> Self {
        Self::new(active.revision(), staged.revision(), class)
    }
}

/// Result of evaluating whether a staged change can be activated.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommitVerdict {
    AllowNow,
    Defer,
}

/// First-pass commit rule surface for staged calibration activation.
pub struct CommitRules;

impl CommitRules {
    pub const fn evaluate(
        policy: ecu_domain::CommitPolicy,
        diff: CalibrationDiff,
        runtime_safe: bool,
    ) -> CommitVerdict {
        match policy {
            ecu_domain::CommitPolicy::Deferred => CommitVerdict::Defer,
            ecu_domain::CommitPolicy::SafeOnly => {
                if runtime_safe {
                    CommitVerdict::AllowNow
                } else {
                    CommitVerdict::Defer
                }
            }
            ecu_domain::CommitPolicy::Immediate => match diff.class {
                CalibrationClass::NoChange | CalibrationClass::MetadataOnly => {
                    CommitVerdict::AllowNow
                }
                CalibrationClass::RuntimeSafe if runtime_safe => CommitVerdict::AllowNow,
                CalibrationClass::RuntimeSensitive if runtime_safe => CommitVerdict::AllowNow,
                CalibrationClass::SafetyCritical | CalibrationClass::Mixed => {
                    if runtime_safe {
                        CommitVerdict::AllowNow
                    } else {
                        CommitVerdict::Defer
                    }
                }
                _ => CommitVerdict::Defer,
            },
        }
    }
}

/// Geometry-related calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GeometryCalibration {
    pub expert_trigger: ExpertTriggerCalibration,
}

/// Fuel-related calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct FuelCalibration;

/// Ignition-related calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IgnitionCalibration;

/// Lambda-related calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct LambdaCalibration;

/// Torque-related calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TorqueCalibration;

/// Idle-related calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct IdleCalibration;

/// Auxiliary-output calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AuxCalibration;

/// Safety-related calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SafetyCalibration;

/// Sensor calibration shell.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SensorCalibration;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::expert_trigger::test_support::sample_wire_expert_trigger;

    #[test]
    fn calibration_groups_default_to_empty_shells() {
        let calibration = Calibration::default();

        assert_eq!(calibration.geometry, GeometryCalibration::default());
        assert_eq!(
            calibration.geometry.expert_trigger.schema_version,
            CalibrationSchemaVersion::CURRENT
        );
        assert_eq!(calibration.fuel, FuelCalibration);
        assert_eq!(calibration.ignition, IgnitionCalibration);
        assert_eq!(calibration.lambda, LambdaCalibration);
        assert_eq!(calibration.torque, TorqueCalibration);
        assert_eq!(calibration.idle, IdleCalibration);
        assert_eq!(calibration.aux, AuxCalibration);
        assert_eq!(calibration.safety, SafetyCalibration);
        assert_eq!(calibration.sensors, SensorCalibration);
    }

    #[test]
    fn calibration_group_types_are_copyable() {
        let geometry = GeometryCalibration::default();
        let duplicate = geometry;

        assert_eq!(geometry, duplicate);
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
}
