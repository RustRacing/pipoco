//! Core-free persisted TunerStudio page load/burn helpers.

use crate::pages::PageError;
use crate::pages::{
    decode_trusted_expert_trigger_page, encode_expert_trigger_page,
    expert_trigger_calibration_from_page, expert_trigger_page_from_calibration, ts_page_descriptor,
    ANGLES_PAGE_BYTES, EXPERT_TRIGGER_PAGE_BYTES, PAGE_ANGLES, PAGE_EXPERT_TRIGGER, PAGE_FUEL,
    PAGE_IGN, TABLE_PAGE_BYTES,
};
use crate::server::PageStore;
#[cfg(feature = "runtime")]
use crate::server::{CompatibilityInfoReport, CompatibilityMigrationCode, CompatibilityStatusCode};
#[cfg(feature = "runtime")]
use crate::{CalibrationEditSurface, CalibrationPackageApplyResult};
use ecu_calibration::kv::{
    PERSIST_KEY_ANGLES, PERSIST_KEY_CAL_PACKAGE, PERSIST_KEY_EXPERT_TRIGGER, PERSIST_KEY_FUEL,
    PERSIST_KEY_IGN,
};
#[cfg(feature = "runtime")]
use ecu_calibration::{
    CalibrationHardwareTargetId, CalibrationPackageCompatibility,
    CalibrationPackageMigrationStatus, CalibrationPackageReview, CalibrationPackageWireError,
    CalibrationRuntimeBuildId, CalibrationSchemaVersion,
};
use ecu_calibration::{
    ExpertTriggerCalibration, FuelRuntimeTune, KvStore, PersistError, PersistedCalibrationPackage,
};

#[cfg(feature = "runtime")]
pub trait CalibrationPackageSession {
    fn export_current_package(
        &self,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> PersistedCalibrationPackage;

    fn import_candidate_package(
        &mut self,
        candidate: PersistedCalibrationPackage,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> CalibrationPackageApplyResult;

    fn apply_expert_trigger_page(&mut self, data: &[u8]) -> Result<(), PageError>;

    fn expert_trigger_calibration(&self) -> ExpertTriggerCalibration;
}

#[cfg(feature = "runtime")]
impl CalibrationPackageSession for CalibrationEditSurface {
    fn export_current_package(
        &self,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> PersistedCalibrationPackage {
        CalibrationEditSurface::export_current_package(self, runtime_build_id, hardware_target_id)
    }

    fn import_candidate_package(
        &mut self,
        candidate: PersistedCalibrationPackage,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> CalibrationPackageApplyResult {
        CalibrationEditSurface::import_candidate_package(
            self,
            candidate,
            runtime_build_id,
            hardware_target_id,
        )
    }

    fn apply_expert_trigger_page(&mut self, data: &[u8]) -> Result<(), PageError> {
        CalibrationEditSurface::apply_expert_trigger_page(self, data)
            .map(|_| ())
            .map_err(|_| PageError::Invalid)
    }

    fn expert_trigger_calibration(&self) -> ExpertTriggerCalibration {
        CalibrationEditSurface::expert_trigger_calibration(self)
    }
}

/// One persisted TunerStudio page binding.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PersistedPageSpec {
    pub page: u8,
    pub key: &'static [u8],
    pub len: usize,
}

/// Setup pages currently persisted as raw TunerStudio page bytes.
pub const TS_SETUP_PERSISTED_PAGES: [PersistedPageSpec; 3] = [
    PersistedPageSpec {
        page: PAGE_FUEL,
        key: PERSIST_KEY_FUEL,
        len: match ts_page_descriptor(PAGE_FUEL) {
            Some(descriptor) => descriptor.len,
            None => 0,
        },
    },
    PersistedPageSpec {
        page: PAGE_IGN,
        key: PERSIST_KEY_IGN,
        len: match ts_page_descriptor(PAGE_IGN) {
            Some(descriptor) => descriptor.len,
            None => 0,
        },
    },
    PersistedPageSpec {
        page: PAGE_ANGLES,
        key: PERSIST_KEY_ANGLES,
        len: match ts_page_descriptor(PAGE_ANGLES) {
            Some(descriptor) => descriptor.len,
            None => 0,
        },
    },
];

/// Pages successfully written through the persisted TS store.
///
/// This is transport/storage evidence only. Runtime owners decide which page
/// numbers require live recalculation.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct WrittenPageSet {
    bits: u128,
}

impl WrittenPageSet {
    pub const fn new() -> Self {
        Self { bits: 0 }
    }

    pub fn mark(&mut self, page: u8) {
        if page < 128 {
            self.bits |= 1u128 << page;
        }
    }

    pub const fn contains(self, page: u8) -> bool {
        page < 128 && (self.bits & (1u128 << page)) != 0
    }

    pub const fn is_empty(self) -> bool {
        self.bits == 0
    }

    pub const fn bits(self) -> u128 {
        self.bits
    }
}

/// Shared policy for deciding when TS page writes require runtime fuel retuning.
///
/// This keeps runtime retune ownership in one place even when the live
/// calibration seam remains page-store based.
pub const fn written_pages_require_runtime_fuel_retune(pages: WrittenPageSet) -> bool {
    pages.contains(crate::pages::PAGE_VE_TUNE)
        || pages.contains(crate::pages::PAGE_VE_TABLE)
        || pages.contains(crate::pages::PAGE_AFR_TABLE)
}

/// Tracks successful TS page writes without interpreting their runtime meaning.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub struct PageWriteReporter {
    pending: WrittenPageSet,
}

impl PageWriteReporter {
    pub const fn new() -> Self {
        Self {
            pending: WrittenPageSet::new(),
        }
    }

    pub fn clear(&mut self) {
        self.pending = WrittenPageSet::new();
    }

    pub fn mark_page_write(&mut self, page: u8) {
        self.pending.mark(page);
    }

    pub const fn has_pending_update(&self) -> bool {
        !self.pending.is_empty()
    }

    pub fn take_written_pages(&mut self) -> Option<WrittenPageSet> {
        if self.pending.is_empty() {
            return None;
        }
        let pages = self.pending;
        self.clear();
        Some(pages)
    }
}

/// Load persisted raw page bytes into `store`.
///
/// Missing keys, short reads, oversized specs, and invalid page writes are
/// intentionally ignored to preserve the legacy best-effort `try_load` path.
pub fn try_load_pages<S, KV, const MAX: usize>(
    store: &mut S,
    kv: &mut KV,
    specs: &[PersistedPageSpec],
) where
    S: PageStore + ?Sized,
    KV: KvStore,
{
    let mut buf = [0u8; MAX];
    for spec in specs {
        if spec.len > MAX {
            continue;
        }
        let data = &mut buf[..spec.len];
        if let Ok(n) = kv.read(spec.key, data) {
            if n == spec.len {
                let _ = store.write_page(spec.page, data);
            }
        }
    }
}

/// Burn raw page bytes from `store` into `kv`.
pub fn burn_pages<S, KV, const MAX: usize>(
    store: &S,
    kv: &mut KV,
    specs: &[PersistedPageSpec],
) -> Result<(), PersistError>
where
    S: PageStore + ?Sized,
    KV: KvStore,
{
    let mut buf = [0u8; MAX];
    for spec in specs {
        if spec.len > MAX {
            return Err(PersistError::Fail);
        }
        let data = &mut buf[..spec.len];
        if store.read_page(spec.page, data).unwrap_or(0) == spec.len {
            kv.write(spec.key, data).map_err(PersistError::from)?;
        }
    }
    Ok(())
}

/// Best-effort load of the expert-trigger calibration from persisted TS bytes.
///
/// Missing, short, or invalid records are ignored to match the legacy setup-page
/// load path.
pub fn try_load_expert_trigger_calibration<KV: KvStore>(
    kv: &mut KV,
) -> Option<ecu_calibration::ExpertTriggerCalibration> {
    let mut buf = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
    let n = kv.read(PERSIST_KEY_EXPERT_TRIGGER, &mut buf).ok()?;
    if n != EXPERT_TRIGGER_PAGE_BYTES {
        return None;
    }
    decode_trusted_expert_trigger_page(&buf)
        .ok()
        .and_then(|page| expert_trigger_calibration_from_page(page).ok())
}

/// Burn the expert-trigger calibration as the canonical TS expert-trigger page.
pub fn burn_expert_trigger_calibration<KV: KvStore>(
    kv: &mut KV,
    cal: &ecu_calibration::ExpertTriggerCalibration,
) -> Result<(), PersistError> {
    let mut buf = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
    let page = expert_trigger_page_from_calibration(cal);
    encode_expert_trigger_page(&page, &mut buf).map_err(|_| PersistError::Fail)?;
    kv.write(PERSIST_KEY_EXPERT_TRIGGER, &buf)
        .map_err(PersistError::from)
}

/// Best-effort load of a canonical persisted calibration package from KV.
pub fn try_load_calibration_package<KV: KvStore>(
    kv: &mut KV,
) -> Option<PersistedCalibrationPackage> {
    let mut buf = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    let n = kv.read(PERSIST_KEY_CAL_PACKAGE, &mut buf).ok()?;
    if n != PersistedCalibrationPackage::WIRE_LEN {
        return None;
    }
    PersistedCalibrationPackage::decode_wire(&buf).ok()
}

/// Burn a canonical persisted calibration package wire record into KV.
pub fn burn_calibration_package<KV: KvStore>(
    kv: &mut KV,
    package: PersistedCalibrationPackage,
) -> Result<(), PersistError> {
    let mut buf = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    package
        .encode_wire(&mut buf)
        .map_err(|_| PersistError::Fail)?;
    kv.write(PERSIST_KEY_CAL_PACKAGE, &buf)
        .map_err(PersistError::from)
}

/// Core-free provider of the live page-body store plus its runtime fuel tune.
///
/// Implementors own whatever backing state (e.g. a legacy `EcuState`) the page
/// bodies project from, and rebuild a `PageStore` view per call so the generic
/// owner never holds a long-lived borrow of that state.
pub trait PageStoreProvider {
    type Pages<'a>: PageStore
    where
        Self: 'a;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R;
    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R;
    fn runtime_fuel_tune(&self) -> FuelRuntimeTune;
}

/// Generic, core-free owner of a persisted full TunerStudio page store.
///
/// Owns the KV backend, the expert-trigger calibration, persisted-page
/// load/burn mechanics, expert-trigger read/write/transition handling, and
/// runtime fuel-tune update reporting. The page bodies and the live
/// `FuelRuntimeTune` snapshot come from the generic [`PageStoreProvider`], so
/// this type has no dependency on `ecu-compat` or `EcuState`.
pub struct PersistedTsPageStore<P, KV> {
    provider: P,
    kv: KV,
    expert_trigger: ExpertTriggerCalibration,
    page_writes: PageWriteReporter,
}

impl<P: PageStoreProvider, KV: KvStore> PersistedTsPageStore<P, KV> {
    pub fn new(provider: P, kv: KV) -> Self {
        Self {
            provider,
            kv,
            expert_trigger: ExpertTriggerCalibration::default(),
            page_writes: PageWriteReporter::new(),
        }
    }

    /// Attempt to load persisted setup pages and expert-trigger calibration.
    pub fn try_load(&mut self) {
        let expert_trigger = try_load_expert_trigger_calibration(&mut self.kv);
        let Self { provider, kv, .. } = self;
        provider.with_pages_mut(|inner| {
            try_load_pages::<_, _, TABLE_PAGE_BYTES>(inner, kv, &TS_SETUP_PERSISTED_PAGES);
        });
        if let Some(expert_trigger) = expert_trigger {
            self.expert_trigger = expert_trigger;
        }
        self.page_writes.clear();
    }

    /// Attempt to load one persisted calibration package wire record.
    pub fn try_load_package(&mut self) -> Option<PersistedCalibrationPackage> {
        try_load_calibration_package(&mut self.kv)
    }

    /// Reset persisted pages in memory to safe defaults (does not write KV until burn).
    pub fn factory_reset(&mut self) {
        self.provider.with_pages_mut(|inner| {
            let _ = inner.write_page(PAGE_FUEL, &[0u8; TABLE_PAGE_BYTES]);
            let _ = inner.write_page(PAGE_IGN, &[0u8; TABLE_PAGE_BYTES]);
            let mut angles = [0u8; ANGLES_PAGE_BYTES];
            // tooth0_angle_x10=0 at [64..66], cam_timeout_ms default=500 at [66..68]
            angles[66..68].copy_from_slice(&500u16.to_le_bytes());
            let _ = inner.write_page(PAGE_ANGLES, &angles);
        });
        self.expert_trigger = ExpertTriggerCalibration::default();
    }

    pub fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        self.provider.runtime_fuel_tune()
    }

    pub fn expert_trigger_calibration(&self) -> ExpertTriggerCalibration {
        self.expert_trigger
    }

    pub fn kv(&self) -> &KV {
        &self.kv
    }

    pub fn kv_mut(&mut self) -> &mut KV {
        &mut self.kv
    }

    pub fn set_expert_trigger_calibration(&mut self, calibration: ExpertTriggerCalibration) {
        self.expert_trigger = calibration;
    }

    pub fn take_written_pages(&mut self) -> Option<WrittenPageSet> {
        self.page_writes.take_written_pages()
    }

    /// Burn one persisted calibration package wire record through the owned KV seam.
    pub fn burn_package(
        &mut self,
        package: PersistedCalibrationPackage,
    ) -> Result<(), PersistError> {
        burn_calibration_package(&mut self.kv, package)
    }

    /// Export the current TS calibration state as a target-bound package and
    /// persist it through the owned KV seam.
    #[cfg(feature = "runtime")]
    pub fn export_and_burn_current_package(
        &mut self,
        session: &impl CalibrationPackageSession,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Result<PersistedCalibrationPackage, PersistError> {
        let package = session.export_current_package(runtime_build_id, hardware_target_id);
        self.burn_package(package)?;
        Ok(package)
    }

    /// Export the current TS calibration state as canonical package wire bytes.
    #[cfg(feature = "runtime")]
    pub fn export_current_package_wire(
        &self,
        session: &impl CalibrationPackageSession,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
        out: &mut [u8],
    ) -> Result<usize, CalibrationPackageWireError> {
        session
            .export_current_package(runtime_build_id, hardware_target_id)
            .encode_wire(out)
    }

    /// Attempt to load one persisted package wire record and route it through
    /// the TS-side import/apply workflow.
    #[cfg(feature = "runtime")]
    pub fn try_load_and_import_package(
        &mut self,
        session: &mut impl CalibrationPackageSession,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Option<CalibrationPackageApplyResult> {
        let package = self.try_load_package()?;
        let result =
            session.import_candidate_package(package, runtime_build_id, hardware_target_id);
        if !matches!(result, CalibrationPackageApplyResult::Rejected { .. }) {
            self.set_expert_trigger_calibration(session.expert_trigger_calibration());
        }
        Some(result)
    }

    /// Decode one canonical package wire record and route it through the TS-side
    /// import/apply workflow.
    #[cfg(feature = "runtime")]
    pub fn import_candidate_package_wire(
        &mut self,
        session: &mut impl CalibrationPackageSession,
        data: &[u8],
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Result<CalibrationPackageApplyResult, CalibrationPackageWireError> {
        let package = PersistedCalibrationPackage::decode_wire(data)?;
        let result =
            session.import_candidate_package(package, runtime_build_id, hardware_target_id);
        if !matches!(result, CalibrationPackageApplyResult::Rejected { .. }) {
            self.set_expert_trigger_calibration(session.expert_trigger_calibration());
        }
        Ok(result)
    }

    fn read_expert_trigger(&self, out: &mut [u8]) -> Option<usize> {
        let page = expert_trigger_page_from_calibration(&self.expert_trigger);
        encode_expert_trigger_page(&page, out).ok()
    }

    fn write_expert_trigger(&mut self, data: &[u8]) -> Result<(), PageError> {
        let proposed_page = decode_trusted_expert_trigger_page(data)?;
        let proposed = expert_trigger_calibration_from_page(proposed_page)?;
        proposed
            .validate_transition_from(&self.expert_trigger)
            .map_err(|_| PageError::Invalid)?;
        self.expert_trigger = proposed;
        Ok(())
    }
}

/// TS server-facing wrapper that binds the persisted TS page store, the staged
/// calibration edit surface, and the target-bound package identity into one
/// command-capable store owner.
#[cfg(feature = "runtime")]
pub struct TsPackageCommandStore<S, C> {
    store: S,
    session: C,
    runtime_build_id: CalibrationRuntimeBuildId,
    hardware_target_id: CalibrationHardwareTargetId,
}

#[cfg(feature = "runtime")]
impl<S, C> TsPackageCommandStore<S, C> {
    pub fn new(
        store: S,
        session: C,
        runtime_build_id: CalibrationRuntimeBuildId,
        hardware_target_id: CalibrationHardwareTargetId,
    ) -> Self {
        Self {
            store,
            session,
            runtime_build_id,
            hardware_target_id,
        }
    }

    pub fn store(&self) -> &S {
        &self.store
    }

    pub fn store_mut(&mut self) -> &mut S {
        &mut self.store
    }

    pub fn session(&self) -> &C {
        &self.session
    }

    pub fn session_mut(&mut self) -> &mut C {
        &mut self.session
    }
}

#[cfg(feature = "runtime")]
impl<P: PageStoreProvider, KV: KvStore, C> TsPackageCommandStore<PersistedTsPageStore<P, KV>, C> {
    pub fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        self.store.runtime_fuel_tune()
    }

    pub fn take_written_pages(&mut self) -> Option<WrittenPageSet> {
        self.store.take_written_pages()
    }

    pub fn runtime_build_id(&self) -> CalibrationRuntimeBuildId {
        self.runtime_build_id
    }

    pub fn hardware_target_id(&self) -> CalibrationHardwareTargetId {
        self.hardware_target_id
    }
}

#[cfg(feature = "runtime")]
fn compatibility_status_code(
    compatibility: CalibrationPackageCompatibility,
) -> CompatibilityStatusCode {
    match compatibility {
        CalibrationPackageCompatibility::Compatible => CompatibilityStatusCode::Compatible,
        CalibrationPackageCompatibility::SchemaVersionMismatch { .. } => {
            CompatibilityStatusCode::SchemaVersionMismatch
        }
        CalibrationPackageCompatibility::RuntimeBuildMismatch { .. } => {
            CompatibilityStatusCode::RuntimeBuildMismatch
        }
        CalibrationPackageCompatibility::HardwareTargetMismatch { .. } => {
            CompatibilityStatusCode::HardwareTargetMismatch
        }
    }
}

#[cfg(feature = "runtime")]
fn compatibility_report_from_review(review: CalibrationPackageReview) -> CompatibilityInfoReport {
    let (expected_schema_version, actual_schema_version) = match review.compatibility {
        CalibrationPackageCompatibility::SchemaVersionMismatch { expected, actual } => {
            (expected.get(), actual.get())
        }
        _ => (
            CalibrationSchemaVersion::CURRENT.get(),
            review.package.package.schema_version.get(),
        ),
    };
    let (expected_runtime_build_id, actual_runtime_build_id) = match review.compatibility {
        CalibrationPackageCompatibility::RuntimeBuildMismatch { expected, actual } => {
            (expected.get(), actual.get())
        }
        _ => (
            review.package.runtime_build_id.get(),
            review.package.runtime_build_id.get(),
        ),
    };
    let (expected_hardware_target_id, actual_hardware_target_id) = match review.compatibility {
        CalibrationPackageCompatibility::HardwareTargetMismatch { expected, actual } => {
            (expected.get(), actual.get())
        }
        _ => (
            review.package.hardware_target_id.get(),
            review.package.hardware_target_id.get(),
        ),
    };
    CompatibilityInfoReport {
        status: compatibility_status_code(review.compatibility),
        migration: match review.migration {
            CalibrationPackageMigrationStatus::NoMigrationRequired => {
                CompatibilityMigrationCode::None
            }
            CalibrationPackageMigrationStatus::MigrationRequired { .. } => {
                CompatibilityMigrationCode::MigrationRequired
            }
        },
        expected_schema_version,
        actual_schema_version,
        expected_runtime_build_id,
        actual_runtime_build_id,
        expected_hardware_target_id,
        actual_hardware_target_id,
    }
}

#[cfg(feature = "runtime")]
impl<P: PageStoreProvider, KV: KvStore, C: CalibrationPackageSession>
    TsPackageCommandStore<PersistedTsPageStore<P, KV>, C>
{
    pub fn compatibility_report(&self) -> CompatibilityInfoReport {
        let current_package = self
            .session
            .export_current_package(self.runtime_build_id, self.hardware_target_id);
        compatibility_report_from_review(
            current_package.review_against(self.runtime_build_id, self.hardware_target_id),
        )
    }
}

impl<P: PageStoreProvider, KV: KvStore> PageStore for PersistedTsPageStore<P, KV> {
    fn page_len(&self, page: u8) -> Option<usize> {
        if page == PAGE_EXPERT_TRIGGER {
            return Some(EXPERT_TRIGGER_PAGE_BYTES);
        }
        self.provider.with_pages(|inner| inner.page_len(page))
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        if page == PAGE_EXPERT_TRIGGER {
            return self.read_expert_trigger(out);
        }
        self.provider.with_pages(|inner| inner.read_page(page, out))
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        if page == PAGE_EXPERT_TRIGGER {
            return self.write_expert_trigger(data);
        }
        let Self {
            provider,
            page_writes,
            ..
        } = self;
        let result = provider.with_pages_mut(|inner| inner.write_page(page, data));
        if result.is_ok() {
            page_writes.mark_page_write(page);
        }
        result
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        let Self {
            provider,
            kv,
            expert_trigger,
            ..
        } = self;
        provider.with_pages(|inner| {
            burn_pages::<_, _, TABLE_PAGE_BYTES>(inner, kv, &TS_SETUP_PERSISTED_PAGES)
        })?;
        burn_expert_trigger_calibration(kv, expert_trigger)?;
        Ok(())
    }
}

#[cfg(feature = "runtime")]
impl<P: PageStoreProvider, KV: KvStore, C: CalibrationPackageSession> PageStore
    for TsPackageCommandStore<PersistedTsPageStore<P, KV>, C>
{
    fn page_len(&self, page: u8) -> Option<usize> {
        self.store.page_len(page)
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        self.store.read_page(page, out)
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        self.store.write_page(page, data)?;
        if page == PAGE_EXPERT_TRIGGER {
            self.session.apply_expert_trigger_page(data)?;
        }
        Ok(())
    }

    fn burn(&mut self) -> Result<(), PersistError> {
        self.store.burn()?;
        self.store.export_and_burn_current_package(
            &self.session,
            self.runtime_build_id,
            self.hardware_target_id,
        )?;
        Ok(())
    }

    fn export_package_wire(&self, out: &mut [u8]) -> Result<usize, PageError> {
        self.store
            .export_current_package_wire(
                &self.session,
                self.runtime_build_id,
                self.hardware_target_id,
                out,
            )
            .map_err(|_| PageError::Invalid)
    }

    fn import_package_wire(&mut self, data: &[u8]) -> Result<(), PageError> {
        match self.store.import_candidate_package_wire(
            &mut self.session,
            data,
            self.runtime_build_id,
            self.hardware_target_id,
        ) {
            Ok(CalibrationPackageApplyResult::Rejected { .. }) => Err(PageError::Invalid),
            Ok(_) => Ok(()),
            Err(_) => Err(PageError::Invalid),
        }
    }
}

#[cfg(test)]
#[path = "persistence_tests.rs"]
mod tests;
