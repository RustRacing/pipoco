use crate::ExpertTriggerCalibration;

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
