use super::*;
use crate::pages::PAGE_VE_TABLE;
#[cfg(feature = "runtime")]
use crate::server::{OutpcProvider, TsServer};
use crate::server::{PageError, PersistError as ServerPersistError};
#[cfg(feature = "runtime")]
use crate::{CalibrationEditSurface, CalibrationPackageApplyResult};
#[cfg(feature = "runtime")]
use ecu_calibration::{
    ActiveCalibration, Calibration, CalibrationClass, CalibrationRevision, CalibrationSnapshot,
    ExpertTriggerCalibration, StagedCalibration,
};
use ecu_calibration::{
    CalibrationHardwareTargetId, CalibrationPackageWireError, CalibrationRuntimeBuildId, KvError,
    PersistedCalibrationBlob,
};

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
    package: Option<([u8; PersistedCalibrationPackage::WIRE_LEN], usize)>,
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

    fn with_package(
        mut self,
        data: [u8; PersistedCalibrationPackage::WIRE_LEN],
        len: usize,
    ) -> Self {
        self.package = Some((data, len));
        self
    }
}

impl KvStore for MockKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, KvError> {
        if key == PERSIST_KEY_CAL_PACKAGE {
            let (data, len) = self.package.as_ref().ok_or(KvError::NotFound)?;
            out[..*len].copy_from_slice(&data[..*len]);
            return Ok(*len);
        }
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
        if key == PERSIST_KEY_CAL_PACKAGE && data.len() == PersistedCalibrationPackage::WIRE_LEN {
            let mut page = [0u8; PersistedCalibrationPackage::WIRE_LEN];
            page.copy_from_slice(data);
            self.package = Some((page, data.len()));
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

#[cfg(feature = "runtime")]
fn sample_package_surface(expert_trigger: ExpertTriggerCalibration) -> CalibrationEditSurface {
    let active = ActiveCalibration::new(CalibrationRevision::new(7), Calibration::default());
    let mut staged_calibration = Calibration::default();
    staged_calibration.set_expert_trigger(expert_trigger);
    let staged = StagedCalibration::new(active.revision(), staged_calibration);
    CalibrationEditSurface::new(CalibrationSnapshot { active, staged })
}

#[cfg(feature = "runtime")]
struct DummyOutpcProvider;

#[cfg(feature = "runtime")]
impl OutpcProvider for DummyOutpcProvider {
    fn fill_outpc(&self, out: &mut crate::outpc::Outpc) {
        out.rpm = 1234;
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
fn persisted_setup_specs_match_descriptor_metadata() {
    for spec in TS_SETUP_PERSISTED_PAGES {
        let descriptor =
            ts_page_descriptor(spec.page).expect("persisted setup page must be registered");
        assert!(descriptor.persisted_setup_page);
        assert_eq!(descriptor.len, spec.len);
    }
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
fn try_load_calibration_package_ignores_missing_short_and_invalid_wire() {
    let mut missing = MockKv::default();
    assert_eq!(try_load_calibration_package(&mut missing), None);

    let mut short = MockKv::default().with_package(
        [0u8; PersistedCalibrationPackage::WIRE_LEN],
        PersistedCalibrationPackage::WIRE_LEN - 1,
    );
    assert_eq!(try_load_calibration_package(&mut short), None);

    let mut invalid = MockKv::default().with_package(
        [0xFFu8; PersistedCalibrationPackage::WIRE_LEN],
        PersistedCalibrationPackage::WIRE_LEN,
    );
    assert_eq!(try_load_calibration_package(&mut invalid), None);
}

#[test]
fn burn_calibration_package_writes_exact_wire_record() {
    let package = PersistedCalibrationPackage::new(
        PersistedCalibrationBlob::default(),
        CalibrationRuntimeBuildId::new(0x1234_5678),
        CalibrationHardwareTargetId::new(0x2040),
    );
    let mut expected = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    package
        .encode_wire(&mut expected)
        .expect("encode package wire");
    let mut kv = MockKv::default();

    burn_calibration_package(&mut kv, package).expect("burn calibration package");

    assert_eq!(
        kv.package,
        Some((expected, PersistedCalibrationPackage::WIRE_LEN))
    );
    assert_eq!(try_load_calibration_package(&mut kv), Some(package));
}

#[test]
fn burn_calibration_package_preserves_engine_running_error() {
    let package = PersistedCalibrationPackage::new(
        PersistedCalibrationBlob::default(),
        CalibrationRuntimeBuildId::new(0x1234_5678),
        CalibrationHardwareTargetId::new(0x2040),
    );
    let mut kv = MockKv {
        write_error: Some(KvError::EngineRunning),
        ..MockKv::default()
    };

    assert_eq!(
        burn_calibration_package(&mut kv, package),
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

#[test]
fn persisted_ts_page_store_package_load_falls_back_when_record_missing() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    assert_eq!(store.try_load_package(), None);
}

#[test]
fn persisted_ts_page_store_burns_and_reloads_package_wire() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let package = PersistedCalibrationPackage::new(
        PersistedCalibrationBlob::default(),
        CalibrationRuntimeBuildId::new(0x1234_5678),
        CalibrationHardwareTargetId::new(0x2040),
    );

    store
        .burn_package(package)
        .expect("burn package through store");

    assert_eq!(store.try_load_package(), Some(package));
}

#[test]
fn persisted_ts_page_store_package_burn_preserves_engine_running_error() {
    let mut store = PersistedTsPageStore::new(
        MockProvider::default(),
        MockKv {
            write_error: Some(KvError::EngineRunning),
            ..MockKv::default()
        },
    );
    let package = PersistedCalibrationPackage::new(
        PersistedCalibrationBlob::default(),
        CalibrationRuntimeBuildId::new(0x1234_5678),
        CalibrationHardwareTargetId::new(0x2040),
    );

    assert!(matches!(
        store.burn_package(package),
        Err(ServerPersistError::EngineRunning)
    ));
}

#[cfg(feature = "runtime")]
#[test]
fn export_and_burn_current_package_matches_runtime_view_export_surface() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let expert_trigger = ExpertTriggerCalibration {
        profile_hash: 0xCA11_BA7E,
        ..ExpertTriggerCalibration::default()
    };
    let surface = sample_package_surface(expert_trigger);
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);

    let exported = store
        .export_and_burn_current_package(&surface, runtime_build_id, hardware_target_id)
        .expect("export and burn current package");

    assert_eq!(
        exported,
        surface.export_current_package(runtime_build_id, hardware_target_id)
    );
    assert_eq!(store.try_load_package(), Some(exported));
}

#[cfg(feature = "runtime")]
#[test]
fn try_load_and_import_package_applies_persisted_package_through_store_boundary() {
    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0xD00D_BEEF,
        fixed_timing_deg10: 250,
        ..ExpertTriggerCalibration::default()
    };
    let candidate_surface = sample_package_surface(candidate_trigger);
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);

    let burned = store
        .export_and_burn_current_package(&candidate_surface, runtime_build_id, hardware_target_id)
        .expect("burn compatible package");
    let mut current_surface = sample_package_surface(ExpertTriggerCalibration::default());

    let applied = store
        .try_load_and_import_package(&mut current_surface, runtime_build_id, hardware_target_id)
        .expect("persisted package should load");

    match applied {
        CalibrationPackageApplyResult::AppliedUnchanged { package, diff, .. } => {
            assert_eq!(package, burned);
            assert_eq!(diff.class, CalibrationClass::SafetyCritical);
        }
        other => panic!("unexpected apply result: {other:?}"),
    }

    assert_eq!(
        current_surface
            .snapshot()
            .staged
            .calibration()
            .geometry
            .expert_trigger,
        candidate_trigger
    );
    assert_eq!(store.expert_trigger_calibration(), candidate_trigger);
}

#[cfg(feature = "runtime")]
#[test]
fn try_load_and_import_package_rejects_incompatible_package_without_mutating_surface() {
    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0xBAD0_C0DE,
        ..ExpertTriggerCalibration::default()
    };
    let candidate_surface = sample_package_surface(candidate_trigger);
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let expected_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let mismatched_build_id = CalibrationRuntimeBuildId::new(0x8765_4321);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);

    store
        .export_and_burn_current_package(
            &candidate_surface,
            mismatched_build_id,
            hardware_target_id,
        )
        .expect("burn mismatched package");
    let mut current_surface = sample_package_surface(ExpertTriggerCalibration::default());
    let before = current_surface.snapshot();

    let applied = store
        .try_load_and_import_package(&mut current_surface, expected_build_id, hardware_target_id)
        .expect("persisted package should load");

    match applied {
        CalibrationPackageApplyResult::Rejected { workflow } => {
            assert!(matches!(
                workflow.candidate_review.compatibility,
                ecu_calibration::CalibrationPackageCompatibility::RuntimeBuildMismatch { .. }
            ));
        }
        other => panic!("unexpected apply result: {other:?}"),
    }

    assert_eq!(current_surface.snapshot(), before);
}

#[cfg(feature = "runtime")]
#[test]
fn export_current_package_wire_matches_canonical_package_encoding() {
    let store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let expert_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_0001,
        ..ExpertTriggerCalibration::default()
    };
    let surface = sample_package_surface(expert_trigger);
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let package = surface.export_current_package(runtime_build_id, hardware_target_id);
    let mut expected = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    let mut actual = [0u8; PersistedCalibrationPackage::WIRE_LEN];

    package
        .encode_wire(&mut expected)
        .expect("encode expected package wire");
    let written = store
        .export_current_package_wire(&surface, runtime_build_id, hardware_target_id, &mut actual)
        .expect("export package wire");

    assert_eq!(written, PersistedCalibrationPackage::WIRE_LEN);
    assert_eq!(actual, expected);
}

#[cfg(feature = "runtime")]
#[test]
fn import_candidate_package_wire_applies_valid_wire_without_persistence_roundtrip() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_0002,
        fixed_timing_deg10: 250,
        ..ExpertTriggerCalibration::default()
    };
    let candidate_surface = sample_package_surface(candidate_trigger);
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let mut wire = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    store
        .export_current_package_wire(
            &candidate_surface,
            runtime_build_id,
            hardware_target_id,
            &mut wire,
        )
        .expect("encode valid candidate wire");
    let mut current_surface = sample_package_surface(ExpertTriggerCalibration::default());

    let applied = store
        .import_candidate_package_wire(
            &mut current_surface,
            &wire,
            runtime_build_id,
            hardware_target_id,
        )
        .expect("import valid candidate wire");

    match applied {
        CalibrationPackageApplyResult::AppliedUnchanged { diff, .. } => {
            assert_eq!(diff.class, CalibrationClass::SafetyCritical);
        }
        other => panic!("unexpected apply result: {other:?}"),
    }

    assert_eq!(
        current_surface
            .snapshot()
            .staged
            .calibration()
            .geometry
            .expert_trigger,
        candidate_trigger
    );
}

#[cfg(feature = "runtime")]
#[test]
fn import_candidate_package_wire_rejects_malformed_wire_without_mutating_surface() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let mut current_surface = sample_package_surface(ExpertTriggerCalibration::default());
    let before = current_surface.snapshot();

    let err = store
        .import_candidate_package_wire(
            &mut current_surface,
            &[0xFF; PersistedCalibrationPackage::WIRE_LEN],
            CalibrationRuntimeBuildId::new(0x1234_5678),
            CalibrationHardwareTargetId::new(0x2040),
        )
        .expect_err("malformed wire must fail");

    assert_eq!(err, CalibrationPackageWireError::BadMagic);
    assert_eq!(current_surface.snapshot(), before);
}

#[cfg(feature = "runtime")]
#[test]
fn import_candidate_package_wire_rejects_incompatible_wire_without_mutating_surface() {
    let mut store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_0003,
        ..ExpertTriggerCalibration::default()
    };
    let candidate_surface = sample_package_surface(candidate_trigger);
    let expected_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let mismatched_build_id = CalibrationRuntimeBuildId::new(0x8765_4321);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let mut wire = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    store
        .export_current_package_wire(
            &candidate_surface,
            mismatched_build_id,
            hardware_target_id,
            &mut wire,
        )
        .expect("encode mismatched candidate wire");
    let mut current_surface = sample_package_surface(ExpertTriggerCalibration::default());
    let before = current_surface.snapshot();

    let applied = store
        .import_candidate_package_wire(
            &mut current_surface,
            &wire,
            expected_build_id,
            hardware_target_id,
        )
        .expect("incompatible wire still decodes");

    match applied {
        CalibrationPackageApplyResult::Rejected { workflow } => {
            assert!(matches!(
                workflow.candidate_review.compatibility,
                ecu_calibration::CalibrationPackageCompatibility::RuntimeBuildMismatch { .. }
            ));
        }
        other => panic!("unexpected apply result: {other:?}"),
    }

    assert_eq!(current_surface.snapshot(), before);
}

#[cfg(feature = "runtime")]
#[test]
fn package_export_command_returns_canonical_wire_bytes() {
    let store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let expert_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_1001,
        ..ExpertTriggerCalibration::default()
    };
    let surface = sample_package_surface(expert_trigger);
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let mut expected = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    surface
        .export_current_package(runtime_build_id, hardware_target_id)
        .encode_wire(&mut expected)
        .expect("encode expected wire");
    let command_store =
        TsPackageCommandStore::new(store, surface, runtime_build_id, hardware_target_id);
    let mut server = TsServer::new(crate::TS_SIGNATURE, DummyOutpcProvider, command_store);
    let mut req = [0u8; 64];
    let mut out = [0u8; 512];
    let len = crate::proto::encode_reply(crate::proto::Cmd::PackageExport, &[], &mut req)
        .expect("encode package export request");
    let rlen = server
        .handle(&req[..len], &mut out)
        .expect("handle package export");
    let (cmd, payload) =
        crate::proto::decode_request(&out[..rlen]).expect("decode package export reply");

    assert_eq!(cmd, crate::proto::Cmd::PackageExport);
    assert_eq!(payload, &expected[..]);
}

#[cfg(feature = "runtime")]
#[test]
fn package_import_command_applies_valid_wire_and_rejects_incompatible_wire() {
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let encode_store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let candidate_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_1002,
        fixed_timing_deg10: 250,
        ..ExpertTriggerCalibration::default()
    };
    let candidate_surface = sample_package_surface(candidate_trigger);
    let mut valid_wire = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    encode_store
        .export_current_package_wire(
            &candidate_surface,
            runtime_build_id,
            hardware_target_id,
            &mut valid_wire,
        )
        .expect("encode valid wire");
    let mut incompatible_wire = [0u8; PersistedCalibrationPackage::WIRE_LEN];
    encode_store
        .export_current_package_wire(
            &candidate_surface,
            CalibrationRuntimeBuildId::new(0x8765_4321),
            hardware_target_id,
            &mut incompatible_wire,
        )
        .expect("encode incompatible wire");

    let apply_store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let current_surface = sample_package_surface(ExpertTriggerCalibration::default());
    let command_store = TsPackageCommandStore::new(
        apply_store,
        current_surface,
        runtime_build_id,
        hardware_target_id,
    );
    let mut server = TsServer::new(crate::TS_SIGNATURE, DummyOutpcProvider, command_store);
    let mut req = [0u8; 512];
    let mut out = [0u8; 512];

    let len = crate::proto::encode_reply(crate::proto::Cmd::PackageImport, &valid_wire, &mut req)
        .expect("encode valid package import request");
    let rlen = server
        .handle(&req[..len], &mut out)
        .expect("handle valid package import");
    let (cmd, payload) =
        crate::proto::decode_request(&out[..rlen]).expect("decode valid import reply");
    assert_eq!(cmd, crate::proto::Cmd::PackageImport);
    assert_eq!(payload, b"OK");

    let len = crate::proto::encode_reply(
        crate::proto::Cmd::PackageImport,
        &incompatible_wire,
        &mut req,
    )
    .expect("encode incompatible package import request");
    let rlen = server
        .handle(&req[..len], &mut out)
        .expect("handle incompatible package import");
    let (cmd, payload) =
        crate::proto::decode_request(&out[..rlen]).expect("decode incompatible import reply");
    assert_eq!(cmd, crate::proto::Cmd::PackageImport);
    assert_eq!(payload, b"ERR");

    assert_eq!(
        server
            .store()
            .session()
            .snapshot()
            .staged
            .calibration()
            .geometry
            .expert_trigger,
        candidate_trigger
    );
    assert_eq!(
        server.store().store().expert_trigger_calibration(),
        candidate_trigger
    );
}

#[cfg(feature = "runtime")]
#[test]
fn package_import_command_rejects_malformed_wire_without_mutating_surface() {
    let store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let surface = sample_package_surface(ExpertTriggerCalibration::default());
    let before = surface.snapshot();
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let command_store =
        TsPackageCommandStore::new(store, surface, runtime_build_id, hardware_target_id);
    let mut server = TsServer::new(crate::TS_SIGNATURE, DummyOutpcProvider, command_store);
    let mut req = [0u8; 512];
    let mut out = [0u8; 512];
    let len = crate::proto::encode_reply(
        crate::proto::Cmd::PackageImport,
        &[0xFF; PersistedCalibrationPackage::WIRE_LEN],
        &mut req,
    )
    .expect("encode malformed package import request");
    let rlen = server
        .handle(&req[..len], &mut out)
        .expect("handle malformed package import");
    let (cmd, payload) =
        crate::proto::decode_request(&out[..rlen]).expect("decode malformed import reply");

    assert_eq!(cmd, crate::proto::Cmd::PackageImport);
    assert_eq!(payload, b"ERR");
    assert_eq!(server.store().session().snapshot(), before);
}

#[cfg(feature = "runtime")]
#[test]
fn package_command_store_burn_persists_current_package_wire() {
    let store = PersistedTsPageStore::new(MockProvider::default(), MockKv::default());
    let expert_trigger = ExpertTriggerCalibration {
        profile_hash: 0x51A7_3001,
        ..ExpertTriggerCalibration::default()
    };
    let surface = sample_package_surface(expert_trigger);
    let runtime_build_id = CalibrationRuntimeBuildId::new(0x1234_5678);
    let hardware_target_id = CalibrationHardwareTargetId::new(0x2040);
    let expected = surface.export_current_package(runtime_build_id, hardware_target_id);
    let command_store =
        TsPackageCommandStore::new(store, surface, runtime_build_id, hardware_target_id);
    let mut server = TsServer::new(crate::TS_SIGNATURE, DummyOutpcProvider, command_store);
    let mut req = [0u8; 64];
    let mut out = [0u8; 512];
    let len =
        crate::proto::encode_reply(crate::proto::Cmd::Burn, &[], &mut req).expect("encode burn");
    let rlen = server.handle(&req[..len], &mut out).expect("handle burn");
    let (cmd, payload) = crate::proto::decode_request(&out[..rlen]).expect("decode burn reply");

    assert_eq!(cmd, crate::proto::Cmd::Burn);
    assert_eq!(payload, b"OK");
    assert_eq!(
        server.store_mut().store_mut().try_load_package(),
        Some(expected)
    );
}
