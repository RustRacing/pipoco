use super::*;
use crate::pages::PAGE_VE_TABLE;
use crate::server::{PageError, PersistError as ServerPersistError};

const MAX: usize = TABLE_PAGE_BYTES;

#[test]
fn page_write_reporter_tracks_successful_page_writes_without_runtime_policy() {
    let mut reporter = PageWriteReporter::new();
    reporter.mark_page_write(PAGE_FUEL);
    reporter.mark_page_write(PAGE_VE_TABLE);
    assert!(reporter.has_pending_update());

    let pages = reporter
        .take_written_pages()
        .expect("written pages should be reported");
    assert!(pages.contains(PAGE_FUEL));
    assert!(pages.contains(PAGE_VE_TABLE));
    assert!(!pages.contains(PAGE_IGN));
    assert!(!reporter.has_pending_update());
    assert_eq!(reporter.take_written_pages(), None);
}

#[derive(Default)]
struct MockPages {
    fuel: Option<[u8; TABLE_PAGE_BYTES]>,
    ign: Option<[u8; TABLE_PAGE_BYTES]>,
    ve: Option<[u8; TABLE_PAGE_BYTES]>,
    angles: Option<[u8; ANGLES_PAGE_BYTES]>,
    reject_angles: bool,
}

impl PageStore for MockPages {
    fn page_len(&self, page: u8) -> Option<usize> {
        match page {
            PAGE_FUEL | PAGE_IGN | PAGE_VE_TABLE => Some(TABLE_PAGE_BYTES),
            PAGE_ANGLES => Some(ANGLES_PAGE_BYTES),
            _ => None,
        }
    }

    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_FUEL => {
                let data = self.fuel.as_ref()?;
                out[..TABLE_PAGE_BYTES].copy_from_slice(data);
                Some(TABLE_PAGE_BYTES)
            }
            PAGE_IGN => {
                let data = self.ign.as_ref()?;
                out[..TABLE_PAGE_BYTES].copy_from_slice(data);
                Some(TABLE_PAGE_BYTES)
            }
            PAGE_VE_TABLE => {
                let data = self.ve.as_ref()?;
                out[..TABLE_PAGE_BYTES].copy_from_slice(data);
                Some(TABLE_PAGE_BYTES)
            }
            PAGE_ANGLES => {
                let data = self.angles.as_ref()?;
                out[..ANGLES_PAGE_BYTES].copy_from_slice(data);
                Some(ANGLES_PAGE_BYTES)
            }
            _ => None,
        }
    }

    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_FUEL if data.len() == TABLE_PAGE_BYTES => {
                let mut page = [0u8; TABLE_PAGE_BYTES];
                page.copy_from_slice(data);
                self.fuel = Some(page);
                Ok(())
            }
            PAGE_IGN if data.len() == TABLE_PAGE_BYTES => {
                let mut page = [0u8; TABLE_PAGE_BYTES];
                page.copy_from_slice(data);
                self.ign = Some(page);
                Ok(())
            }
            PAGE_VE_TABLE if data.len() == TABLE_PAGE_BYTES => {
                let mut page = [0u8; TABLE_PAGE_BYTES];
                page.copy_from_slice(data);
                self.ve = Some(page);
                Ok(())
            }
            PAGE_ANGLES if data.len() == ANGLES_PAGE_BYTES && !self.reject_angles => {
                let mut page = [0u8; ANGLES_PAGE_BYTES];
                page.copy_from_slice(data);
                self.angles = Some(page);
                Ok(())
            }
            _ => Err(PageError::Invalid),
        }
    }

    fn burn(&mut self) -> Result<(), ServerPersistError> {
        Ok(())
    }
}

#[derive(Default)]
struct MockKv {
    fuel: Option<([u8; TABLE_PAGE_BYTES], usize)>,
    ign: Option<([u8; TABLE_PAGE_BYTES], usize)>,
    angles: Option<([u8; ANGLES_PAGE_BYTES], usize)>,
    expert_trigger: Option<([u8; EXPERT_TRIGGER_PAGE_BYTES], usize)>,
    write_error: Option<KvError>,
    fail_on_write: Option<usize>,
    write_count: usize,
}

impl MockKv {
    fn with_fuel(mut self, fill: u8, len: usize) -> Self {
        self.fuel = Some(([fill; TABLE_PAGE_BYTES], len));
        self
    }

    fn with_ign(mut self, fill: u8, len: usize) -> Self {
        self.ign = Some(([fill; TABLE_PAGE_BYTES], len));
        self
    }

    fn with_angles(mut self, fill: u8, len: usize) -> Self {
        self.angles = Some(([fill; ANGLES_PAGE_BYTES], len));
        self
    }

    fn with_expert_trigger(mut self, data: [u8; EXPERT_TRIGGER_PAGE_BYTES], len: usize) -> Self {
        self.expert_trigger = Some((data, len));
        self
    }
}

impl KvStore for MockKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        if key == PERSIST_KEY_FUEL {
            let (data, len) = self.fuel.as_ref().ok_or(KvError::NotFound)?;
            out[..*len].copy_from_slice(&data[..*len]);
            return Ok(*len);
        }
        if key == PERSIST_KEY_IGN {
            let (data, len) = self.ign.as_ref().ok_or(KvError::NotFound)?;
            out[..*len].copy_from_slice(&data[..*len]);
            return Ok(*len);
        }
        if key == PERSIST_KEY_ANGLES {
            let (data, len) = self.angles.as_ref().ok_or(KvError::NotFound)?;
            out[..*len].copy_from_slice(&data[..*len]);
            return Ok(*len);
        }
        if key == PERSIST_KEY_EXPERT_TRIGGER {
            let (data, len) = self.expert_trigger.as_ref().ok_or(KvError::NotFound)?;
            out[..*len].copy_from_slice(&data[..*len]);
            return Ok(*len);
        }
        Err(KvError::NotFound)
    }

    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), KvError> {
        self.write_count += 1;
        if self.fail_on_write == Some(self.write_count) {
            return Err(self.write_error.unwrap_or(KvError::Io));
        }
        if let Some(err) = self.write_error {
            return Err(err);
        }
        if key == PERSIST_KEY_FUEL && data.len() == TABLE_PAGE_BYTES {
            let mut page = [0u8; TABLE_PAGE_BYTES];
            page.copy_from_slice(data);
            self.fuel = Some((page, data.len()));
            return Ok(());
        }
        if key == PERSIST_KEY_IGN && data.len() == TABLE_PAGE_BYTES {
            let mut page = [0u8; TABLE_PAGE_BYTES];
            page.copy_from_slice(data);
            self.ign = Some((page, data.len()));
            return Ok(());
        }
        if key == PERSIST_KEY_ANGLES && data.len() == ANGLES_PAGE_BYTES {
            let mut page = [0u8; ANGLES_PAGE_BYTES];
            page.copy_from_slice(data);
            self.angles = Some((page, data.len()));
            return Ok(());
        }
        if key == PERSIST_KEY_EXPERT_TRIGGER && data.len() == EXPERT_TRIGGER_PAGE_BYTES {
            let mut page = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
            page.copy_from_slice(data);
            self.expert_trigger = Some((page, data.len()));
            return Ok(());
        }
        Err(KvError::Io)
    }
}

struct MockProvider {
    pages: MockPages,
    tune: FuelRuntimeTune,
}

impl Default for MockProvider {
    fn default() -> Self {
        Self {
            pages: MockPages::default(),
            tune: FuelRuntimeTune::new([[11; 16]; 16], [[147; 16]; 16], 2400, 775, 1),
        }
    }
}

impl PageStoreProvider for MockProvider {
    type Pages<'a> = MockPages;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R {
        f(&mut self.pages)
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        f(&self.pages)
    }

    fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        self.tune
    }
}

#[test]
fn setup_specs_match_current_page_layout() {
    assert_eq!(
        TS_SETUP_PERSISTED_PAGES,
        [
            PersistedPageSpec {
                page: PAGE_FUEL,
                key: b"fuel",
                len: TABLE_PAGE_BYTES,
            },
            PersistedPageSpec {
                page: PAGE_IGN,
                key: b"ign",
                len: TABLE_PAGE_BYTES,
            },
            PersistedPageSpec {
                page: PAGE_ANGLES,
                key: b"angles",
                len: ANGLES_PAGE_BYTES,
            },
        ]
    );
}

#[test]
fn try_load_pages_ignores_missing_short_and_invalid_pages() {
    let mut store = MockPages {
        reject_angles: true,
        ..MockPages::default()
    };
    let mut kv = MockKv::default()
        .with_fuel(0x11, TABLE_PAGE_BYTES)
        .with_ign(0x22, TABLE_PAGE_BYTES - 1)
        .with_angles(0x33, ANGLES_PAGE_BYTES);

    try_load_pages::<_, _, MAX>(&mut store, &mut kv, &TS_SETUP_PERSISTED_PAGES);

    assert_eq!(store.fuel, Some([0x11; TABLE_PAGE_BYTES]));
    assert_eq!(store.ign, None);
    assert_eq!(store.angles, None);
}

#[test]
fn burn_pages_writes_exact_readable_pages() {
    let store = MockPages {
        fuel: Some([0x11; TABLE_PAGE_BYTES]),
        ign: Some([0x22; TABLE_PAGE_BYTES]),
        angles: Some([0x33; ANGLES_PAGE_BYTES]),
        reject_angles: false,
        ..MockPages::default()
    };
    let mut kv = MockKv::default();

    burn_pages::<_, _, MAX>(&store, &mut kv, &TS_SETUP_PERSISTED_PAGES).expect("burn setup pages");

    assert_eq!(kv.fuel, Some(([0x11; TABLE_PAGE_BYTES], TABLE_PAGE_BYTES)));
    assert_eq!(kv.ign, Some(([0x22; TABLE_PAGE_BYTES], TABLE_PAGE_BYTES)));
    assert_eq!(
        kv.angles,
        Some(([0x33; ANGLES_PAGE_BYTES], ANGLES_PAGE_BYTES))
    );
}

#[test]
fn burn_pages_preserves_engine_running_error() {
    let store = MockPages {
        fuel: Some([0x11; TABLE_PAGE_BYTES]),
        ..MockPages::default()
    };
    let mut kv = MockKv {
        write_error: Some(KvError::EngineRunning),
        ..MockKv::default()
    };

    assert_eq!(
        burn_pages::<_, _, MAX>(&store, &mut kv, &TS_SETUP_PERSISTED_PAGES),
        Err(PersistError::EngineRunning)
    );
}

#[test]
fn burn_pages_mid_burn_failure_leaves_prior_pages_written() {
    let store = MockPages {
        fuel: Some([0x11; TABLE_PAGE_BYTES]),
        ign: Some([0x22; TABLE_PAGE_BYTES]),
        angles: Some([0x33; ANGLES_PAGE_BYTES]),
        reject_angles: false,
        ..MockPages::default()
    };
    let mut kv = MockKv {
        fail_on_write: Some(2),
        ..MockKv::default()
    };

    assert_eq!(
        burn_pages::<_, _, MAX>(&store, &mut kv, &TS_SETUP_PERSISTED_PAGES),
        Err(PersistError::Fail)
    );

    assert_eq!(kv.write_count, 2);
    assert_eq!(kv.fuel, Some(([0x11; TABLE_PAGE_BYTES], TABLE_PAGE_BYTES)));
    assert_eq!(kv.ign, None);
    assert_eq!(kv.angles, None);
}

#[test]
fn try_load_expert_trigger_calibration_ignores_missing_short_and_invalid() {
    let mut missing = MockKv::default();
    assert_eq!(try_load_expert_trigger_calibration(&mut missing), None);

    let mut short = MockKv::default().with_expert_trigger(
        [0u8; EXPERT_TRIGGER_PAGE_BYTES],
        EXPERT_TRIGGER_PAGE_BYTES - 1,
    );
    assert_eq!(try_load_expert_trigger_calibration(&mut short), None);

    let mut invalid = MockKv::default().with_expert_trigger(
        [0xFFu8; EXPERT_TRIGGER_PAGE_BYTES],
        EXPERT_TRIGGER_PAGE_BYTES,
    );
    assert_eq!(try_load_expert_trigger_calibration(&mut invalid), None);
}

#[test]
fn burn_expert_trigger_calibration_writes_exact_page() {
    let calibration = ecu_calibration::ExpertTriggerCalibration::default();
    let page = expert_trigger_page_from_calibration(&calibration);
    let mut expected = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
    encode_expert_trigger_page(&page, &mut expected).expect("encode expert trigger page");
    let mut kv = MockKv::default();

    burn_expert_trigger_calibration(&mut kv, &calibration).expect("burn expert trigger");

    assert_eq!(
        kv.expert_trigger,
        Some((expected, EXPERT_TRIGGER_PAGE_BYTES))
    );
}

#[test]
fn burn_expert_trigger_calibration_preserves_engine_running_error() {
    let mut kv = MockKv {
        write_error: Some(KvError::EngineRunning),
        ..MockKv::default()
    };

    assert_eq!(
        burn_expert_trigger_calibration(
            &mut kv,
            &ecu_calibration::ExpertTriggerCalibration::default()
        ),
        Err(PersistError::EngineRunning)
    );
}

#[test]
fn persisted_ts_page_store_reports_written_pages_without_interpreting_runtime_policy() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());

    assert_eq!(store.take_written_pages(), None);
    store
        .write_page(PAGE_FUEL, &[0x11; TABLE_PAGE_BYTES])
        .expect("fuel write");
    let pages = store
        .take_written_pages()
        .expect("successful fuel page write should be reported");
    assert!(pages.contains(PAGE_FUEL));
    assert!(!pages.contains(PAGE_VE_TABLE));

    store
        .write_page(PAGE_VE_TABLE, &[0x22; TABLE_PAGE_BYTES])
        .expect("ve write");
    let pages = store
        .take_written_pages()
        .expect("successful VE page write should be reported");
    assert!(pages.contains(PAGE_VE_TABLE));
    assert_eq!(store.runtime_fuel_tune().required_fuel_us, 2400);
    assert_eq!(store.runtime_fuel_tune().injector_deadtime_us, 775);
    assert_eq!(store.take_written_pages(), None);
}

#[test]
fn persisted_ts_page_store_owns_expert_trigger_page() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let calibration = ecu_calibration::ExpertTriggerCalibration::default();
    let page = expert_trigger_page_from_calibration(&calibration);
    let mut encoded = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
    encode_expert_trigger_page(&page, &mut encoded).expect("encode expert trigger");

    store
        .write_page(PAGE_EXPERT_TRIGGER, &encoded)
        .expect("expert trigger write");
    let mut roundtrip = [0u8; EXPERT_TRIGGER_PAGE_BYTES];
    assert_eq!(
        store.read_page(PAGE_EXPERT_TRIGGER, &mut roundtrip),
        Some(EXPERT_TRIGGER_PAGE_BYTES)
    );
    assert_eq!(roundtrip, encoded);

    assert!(matches!(
        store.write_page(PAGE_EXPERT_TRIGGER, &[0xFF; EXPERT_TRIGGER_PAGE_BYTES]),
        Err(PageError::Invalid)
    ));
}
