//! Core-free persisted TunerStudio page load/burn helpers.

use crate::pages::PageCodecError;
use crate::pages::{
    decode_trusted_expert_trigger_page, encode_expert_trigger_page,
    expert_trigger_calibration_from_page, expert_trigger_page_from_calibration, ts_page_descriptor,
    ANGLES_PAGE_BYTES, EXPERT_TRIGGER_PAGE_BYTES, PAGE_ANGLES, PAGE_EXPERT_TRIGGER, PAGE_FUEL,
    PAGE_IGN, TABLE_PAGE_BYTES,
};
use crate::server::{PageError, PageStore, PersistError as ServerPersistError};
use ecu_calibration::kv::{
    PERSIST_KEY_ANGLES, PERSIST_KEY_EXPERT_TRIGGER, PERSIST_KEY_FUEL, PERSIST_KEY_IGN,
};
use ecu_calibration::{ExpertTriggerCalibration, FuelRuntimeTune, KvError, KvStore, PersistError};

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
            kv.write(spec.key, data)
                .map_err(kv_error_to_persist_error)?;
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
        .map_err(kv_error_to_persist_error)
}

fn kv_error_to_persist_error(err: KvError) -> PersistError {
    match err {
        KvError::EngineRunning => PersistError::EngineRunning,
        _ => PersistError::Fail,
    }
}

fn page_codec_error_to_page_error(err: PageCodecError) -> PageError {
    match err {
        PageCodecError::WrongSize => PageError::WrongSize,
        PageCodecError::Invalid => PageError::Invalid,
    }
}

fn calibration_persist_error_to_server_persist_error(err: PersistError) -> ServerPersistError {
    match err {
        PersistError::EngineRunning => ServerPersistError::EngineRunning,
        PersistError::Fail => ServerPersistError::Fail,
    }
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

    pub fn take_written_pages(&mut self) -> Option<WrittenPageSet> {
        self.page_writes.take_written_pages()
    }

    fn read_expert_trigger(&self, out: &mut [u8]) -> Option<usize> {
        let page = expert_trigger_page_from_calibration(&self.expert_trigger);
        encode_expert_trigger_page(&page, out).ok()
    }

    fn write_expert_trigger(&mut self, data: &[u8]) -> Result<(), PageError> {
        let proposed_page =
            decode_trusted_expert_trigger_page(data).map_err(page_codec_error_to_page_error)?;
        let proposed = expert_trigger_calibration_from_page(proposed_page)
            .map_err(page_codec_error_to_page_error)?;
        proposed
            .validate_transition_from(&self.expert_trigger)
            .map_err(|_| PageError::Invalid)?;
        self.expert_trigger = proposed;
        Ok(())
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

    fn burn(&mut self) -> Result<(), ServerPersistError> {
        let Self {
            provider,
            kv,
            expert_trigger,
            ..
        } = self;
        provider
            .with_pages(|inner| {
                burn_pages::<_, _, TABLE_PAGE_BYTES>(inner, kv, &TS_SETUP_PERSISTED_PAGES)
            })
            .map_err(calibration_persist_error_to_server_persist_error)?;
        burn_expert_trigger_calibration(kv, expert_trigger)
            .map_err(calibration_persist_error_to_server_persist_error)?;
        Ok(())
    }
}

#[cfg(test)]
#[path = "persistence_tests.rs"]
mod tests;
