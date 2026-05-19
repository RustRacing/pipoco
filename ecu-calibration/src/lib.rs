#![cfg_attr(not(test), no_std)]

use ecu_domain::{AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, PhaseSyncState};

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

pub const EXPERT_TRIGGER_RECORD_LEN: usize = 48;

/// Geometry-related calibration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct GeometryCalibration {
    pub expert_trigger: ExpertTriggerCalibration,
}

macro_rules! code_enum {
    (
        $(#[$meta:meta])*
        pub enum $name:ident {
            $($variant:ident = $code:expr,)+
        }
        default $default:ident
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        pub enum $name {
            $($variant,)+
        }

        impl $name {
            pub const fn code(self) -> u8 {
                match self {
                    $(Self::$variant => $code,)+
                }
            }

            pub const fn from_code(code: u8) -> Option<Self> {
                match code {
                    $($code => Some(Self::$variant),)+
                    _ => None,
                }
            }
        }

        impl Default for $name {
            fn default() -> Self {
                Self::$default
            }
        }
    };
}

code_enum! {
    /// Explicit expert-mode unlock.
    pub enum ExpertUnlock {
        Locked = 0,
        Unlocked = 1,
    }
    default Locked
}

code_enum! {
    /// Source of the absolute engine-time authority.
    pub enum TriggerAuthority {
        None = 0,
        ExpertManual = 1,
        CommunityProfile = 2,
        CertifiedProfile = 3,
        BenchLearned = 4,
    }
    default None
}

code_enum! {
    pub enum TriggerPattern {
        MissingTooth = 0,
        BasicDistributor = 1,
        DualWheel = 2,
        NonUniform = 3,
    }
    default MissingTooth
}

code_enum! {
    pub enum PrimaryTriggerSpeed {
        Crank = 0,
        Cam = 1,
    }
    default Crank
}

code_enum! {
    pub enum TriggerEdge {
        Rising = 0,
        Falling = 1,
    }
    default Rising
}

code_enum! {
    pub enum SecondaryTriggerMode {
        None = 0,
        SingleToothCam = 1,
        FourMinusOneCam = 2,
        PollLevel = 3,
        MultiToothCam = 4,
        OemPattern = 5,
    }
    default None
}

code_enum! {
    pub enum PollLevelPolarity {
        Low = 0,
        High = 1,
    }
    default Low
}

code_enum! {
    pub enum TriggerFilter {
        Off = 0,
        Weak = 1,
        Medium = 2,
        Aggressive = 3,
    }
    default Medium
}

code_enum! {
    pub enum ExpertIgnitionMode {
        SingleCoil = 0,
        WastedSpark = 1,
        WastedCop = 2,
        SequentialCop = 3,
    }
    default WastedSpark
}

code_enum! {
    pub enum ExpertInjectionLayout {
        Batch = 0,
        Paired = 1,
        SemiSequential = 2,
        Banked = 3,
        Sequential = 4,
    }
    default Paired
}

code_enum! {
    pub enum FixedTimingMode {
        Table = 0,
        Fixed = 1,
    }
    default Table
}

/// Canonical expert trigger setup. TunerStudio pages are just one byte-level
/// view over this record; safety decisions should consume this typed surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExpertTriggerCalibration {
    pub schema_version: CalibrationSchemaVersion,
    pub expert_unlock: ExpertUnlock,
    pub authority: TriggerAuthority,
    pub profile_identity: u32,
    pub profile_hash: u32,
    pub trigger_pattern: TriggerPattern,
    pub primary_base_teeth: u8,
    pub missing_teeth: u8,
    pub primary_trigger_speed: PrimaryTriggerSpeed,
    pub trigger_angle_atdc_deg10: u16,
    pub trigger_angle_multiplier: u8,
    pub primary_trigger_edge: TriggerEdge,
    pub secondary_trigger_edge: TriggerEdge,
    pub secondary_trigger_mode: SecondaryTriggerMode,
    pub poll_level_polarity: PollLevelPolarity,
    pub trigger_filter: TriggerFilter,
    pub resync_every_cycle: bool,
    pub skip_cycles: u8,
    pub ignition_mode: ExpertIgnitionMode,
    pub injection_layout: ExpertInjectionLayout,
    pub fixed_timing_mode: FixedTimingMode,
    pub fixed_timing_deg10: i16,
}

impl Default for ExpertTriggerCalibration {
    fn default() -> Self {
        Self {
            schema_version: CalibrationSchemaVersion::CURRENT,
            expert_unlock: ExpertUnlock::Locked,
            authority: TriggerAuthority::None,
            profile_identity: 0,
            profile_hash: 0,
            trigger_pattern: TriggerPattern::MissingTooth,
            primary_base_teeth: 60,
            missing_teeth: 2,
            primary_trigger_speed: PrimaryTriggerSpeed::Crank,
            trigger_angle_atdc_deg10: 0,
            trigger_angle_multiplier: 1,
            primary_trigger_edge: TriggerEdge::Rising,
            secondary_trigger_edge: TriggerEdge::Rising,
            secondary_trigger_mode: SecondaryTriggerMode::None,
            poll_level_polarity: PollLevelPolarity::Low,
            trigger_filter: TriggerFilter::Medium,
            resync_every_cycle: false,
            skip_cycles: 2,
            ignition_mode: ExpertIgnitionMode::WastedSpark,
            injection_layout: ExpertInjectionLayout::Paired,
            fixed_timing_mode: FixedTimingMode::Table,
            fixed_timing_deg10: 100,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpertTriggerValidationError {
    SchemaVersionMismatch,
    ExpertUnlockRequired,
    CertifiedProfileRequiresTrustedPath,
    CertifiedProfileIdentityRequired,
    CertifiedProfileMustStayLocked,
    CertifiedProfileOverwrite,
    InvalidPrimaryBaseTeeth,
    InvalidMissingTeeth,
    MissingTeethOnlyForMissingTooth,
    InvalidTriggerAngle,
    InvalidTriggerAngleMultiplier,
    SecondaryRequired,
    InvalidSkipCycles,
    InvalidFixedTiming,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpertTriggerRuntimeAuthorityError {
    InvalidCalibration(ExpertTriggerValidationError),
    ExpertManualAuthorityRequired,
    DecoderPrimaryLockRequired,
    CamValidated720Required,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExpertTriggerRecordError {
    WrongSize,
    InvalidEnum,
    InvalidCalibration(ExpertTriggerValidationError),
}

impl ExpertTriggerCalibration {
    fn requires_cam_validated_720(self) -> bool {
        matches!(self.ignition_mode, ExpertIgnitionMode::SequentialCop)
            || matches!(self.injection_layout, ExpertInjectionLayout::Sequential)
    }

    pub fn validate(&self) -> Result<(), ExpertTriggerValidationError> {
        if self.schema_version != CalibrationSchemaVersion::CURRENT {
            return Err(ExpertTriggerValidationError::SchemaVersionMismatch);
        }
        if self.authority == TriggerAuthority::ExpertManual
            && self.expert_unlock != ExpertUnlock::Unlocked
        {
            return Err(ExpertTriggerValidationError::ExpertUnlockRequired);
        }
        if self.authority == TriggerAuthority::CertifiedProfile {
            if self.profile_identity == 0 || self.profile_hash == 0 {
                return Err(ExpertTriggerValidationError::CertifiedProfileIdentityRequired);
            }
            if self.expert_unlock != ExpertUnlock::Locked {
                return Err(ExpertTriggerValidationError::CertifiedProfileMustStayLocked);
            }
        }
        if self.primary_base_teeth == 0 {
            return Err(ExpertTriggerValidationError::InvalidPrimaryBaseTeeth);
        }
        match self.trigger_pattern {
            TriggerPattern::MissingTooth => {
                if self.primary_base_teeth < 2
                    || self.missing_teeth == 0
                    || self.missing_teeth >= self.primary_base_teeth
                {
                    return Err(ExpertTriggerValidationError::InvalidMissingTeeth);
                }
            }
            _ if self.missing_teeth != 0 => {
                return Err(ExpertTriggerValidationError::MissingTeethOnlyForMissingTooth);
            }
            _ => {}
        }
        if self.trigger_angle_atdc_deg10 > 7200 {
            return Err(ExpertTriggerValidationError::InvalidTriggerAngle);
        }
        if self.trigger_angle_multiplier == 0 || self.trigger_angle_multiplier > 8 {
            return Err(ExpertTriggerValidationError::InvalidTriggerAngleMultiplier);
        }
        if self.resync_every_cycle && self.secondary_trigger_mode == SecondaryTriggerMode::None {
            return Err(ExpertTriggerValidationError::SecondaryRequired);
        }
        if (self.ignition_mode == ExpertIgnitionMode::SequentialCop
            || self.injection_layout == ExpertInjectionLayout::Sequential)
            && self.secondary_trigger_mode == SecondaryTriggerMode::None
        {
            return Err(ExpertTriggerValidationError::SecondaryRequired);
        }
        if self.skip_cycles > 16 {
            return Err(ExpertTriggerValidationError::InvalidSkipCycles);
        }
        if self.fixed_timing_mode == FixedTimingMode::Fixed
            && !(-100..=600).contains(&self.fixed_timing_deg10)
        {
            return Err(ExpertTriggerValidationError::InvalidFixedTiming);
        }
        Ok(())
    }

    pub fn to_runtime_engine_time_authority(
        self,
        startup_authority: EngineTimeAuthority,
    ) -> Result<EngineTimeAuthority, ExpertTriggerRuntimeAuthorityError> {
        self.validate()
            .map_err(ExpertTriggerRuntimeAuthorityError::InvalidCalibration)?;

        if self.authority != TriggerAuthority::ExpertManual {
            return Err(ExpertTriggerRuntimeAuthorityError::ExpertManualAuthorityRequired);
        }
        if !startup_authority.has_primary_lock() {
            return Err(ExpertTriggerRuntimeAuthorityError::DecoderPrimaryLockRequired);
        }
        if self.requires_cam_validated_720()
            && !matches!(startup_authority.phase, PhaseSyncState::CamValidated720)
        {
            return Err(ExpertTriggerRuntimeAuthorityError::CamValidated720Required);
        }

        Ok(EngineTimeAuthority::new(
            CrankSyncState::PrimaryLocked,
            startup_authority.phase,
            AbsoluteTimeAuthority::ExpertManual,
            startup_authority.confidence_x1000,
            startup_authority.sync_loss_count,
        ))
    }

    pub fn validate_transition_from(
        &self,
        current: &Self,
    ) -> Result<(), ExpertTriggerValidationError> {
        self.validate()?;
        if self.authority == TriggerAuthority::CertifiedProfile
            && current.authority != TriggerAuthority::CertifiedProfile
        {
            return Err(ExpertTriggerValidationError::CertifiedProfileRequiresTrustedPath);
        }
        if current.authority == TriggerAuthority::CertifiedProfile
            && self.authority == TriggerAuthority::CertifiedProfile
            && self != current
        {
            return Err(ExpertTriggerValidationError::CertifiedProfileOverwrite);
        }
        if current.authority == TriggerAuthority::CertifiedProfile
            && self.authority != TriggerAuthority::CertifiedProfile
            && self.expert_unlock != ExpertUnlock::Unlocked
        {
            return Err(ExpertTriggerValidationError::ExpertUnlockRequired);
        }
        Ok(())
    }

    pub fn encode_record(&self, out: &mut [u8]) -> Result<usize, ExpertTriggerRecordError> {
        self.validate()
            .map_err(ExpertTriggerRecordError::InvalidCalibration)?;
        if out.len() < EXPERT_TRIGGER_RECORD_LEN {
            return Err(ExpertTriggerRecordError::WrongSize);
        }
        out[..EXPERT_TRIGGER_RECORD_LEN].fill(0);
        out[0..2].copy_from_slice(&self.schema_version.get().to_le_bytes());
        out[2] = self.expert_unlock.code();
        out[3] = self.authority.code();
        out[4..8].copy_from_slice(&self.profile_identity.to_le_bytes());
        out[8..12].copy_from_slice(&self.profile_hash.to_le_bytes());
        out[12] = self.trigger_pattern.code();
        out[13] = self.primary_base_teeth;
        out[14] = self.missing_teeth;
        out[15] = self.primary_trigger_speed.code();
        out[16..18].copy_from_slice(&self.trigger_angle_atdc_deg10.to_le_bytes());
        out[18] = self.trigger_angle_multiplier;
        out[19] = self.primary_trigger_edge.code();
        out[20] = self.secondary_trigger_edge.code();
        out[21] = self.secondary_trigger_mode.code();
        out[22] = self.poll_level_polarity.code();
        out[23] = self.trigger_filter.code();
        out[24] = u8::from(self.resync_every_cycle);
        out[25] = self.skip_cycles;
        out[26] = self.ignition_mode.code();
        out[27] = self.injection_layout.code();
        out[28] = self.fixed_timing_mode.code();
        out[30..32].copy_from_slice(&self.fixed_timing_deg10.to_le_bytes());
        Ok(EXPERT_TRIGGER_RECORD_LEN)
    }

    pub fn decode_record(data: &[u8]) -> Result<Self, ExpertTriggerRecordError> {
        if data.len() < EXPERT_TRIGGER_RECORD_LEN {
            return Err(ExpertTriggerRecordError::WrongSize);
        }
        let record = Self {
            schema_version: CalibrationSchemaVersion::new(u16::from_le_bytes([data[0], data[1]])),
            expert_unlock: ExpertUnlock::from_code(data[2])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            authority: TriggerAuthority::from_code(data[3])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            profile_identity: u32::from_le_bytes([data[4], data[5], data[6], data[7]]),
            profile_hash: u32::from_le_bytes([data[8], data[9], data[10], data[11]]),
            trigger_pattern: TriggerPattern::from_code(data[12])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            primary_base_teeth: data[13],
            missing_teeth: data[14],
            primary_trigger_speed: PrimaryTriggerSpeed::from_code(data[15])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            trigger_angle_atdc_deg10: u16::from_le_bytes([data[16], data[17]]),
            trigger_angle_multiplier: data[18],
            primary_trigger_edge: TriggerEdge::from_code(data[19])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            secondary_trigger_edge: TriggerEdge::from_code(data[20])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            secondary_trigger_mode: SecondaryTriggerMode::from_code(data[21])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            poll_level_polarity: PollLevelPolarity::from_code(data[22])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            trigger_filter: TriggerFilter::from_code(data[23])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            resync_every_cycle: data[24] != 0,
            skip_cycles: data[25],
            ignition_mode: ExpertIgnitionMode::from_code(data[26])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            injection_layout: ExpertInjectionLayout::from_code(data[27])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            fixed_timing_mode: FixedTimingMode::from_code(data[28])
                .ok_or(ExpertTriggerRecordError::InvalidEnum)?,
            fixed_timing_deg10: i16::from_le_bytes([data[30], data[31]]),
        };
        record
            .validate()
            .map_err(ExpertTriggerRecordError::InvalidCalibration)?;
        Ok(record)
    }
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
