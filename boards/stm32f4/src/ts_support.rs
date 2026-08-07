//! TunerStudio support module for STM32F4 target.
//!
//! This module intentionally isolates all TS-related state and persistence
//! plumbing from the main entrypoint. Behavior remains unchanged and all public
//! entrypoints are wired through narrow helpers used by `main.rs`.

#[allow(unused_imports)]
#[cfg(any(feature = "flash-kv", test))]
use crate::store_support::{
    crc16, header_page_len, newest_sector, read_header, read_page, target_sector_for_next_write,
    validate_page_image, ActiveSector, FlashLayout, FlashLayoutError, FlashPageKey, KvHeader,
    STM32F405_TS_KV_LAYOUT,
};
#[cfg(test)]
use crate::{PIN_MAP_STM32F4_IGNITION_BRINGUP, RUNTIME_BUILD_ID_STM32F4_ECU};
use core::cell::RefCell;
use cortex_m::interrupt::free;
use cortex_m::interrupt::Mutex;
use ecu_calibration::{
    CalibrationHardwareTargetId, CalibrationRuntimeBuildId, EcuConfig, FuelRuntimeTune,
};
use ecu_compat::ts::CompatCalibrationSession;
use ecu_domain::diag::DiagLog;
use ecu_domain::Micros;
#[cfg(any(feature = "flash-kv", test))]
use ecu_target_common::kv::layout::{
    ANGLES_PAGE_LEN, FUEL_PAGE_LEN, IGN_PAGE_LEN, PAGE_HEADER_LEN,
};
#[cfg(not(feature = "flash-kv"))]
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::ts::page_store::{
    build_ecu_page_store, build_system_snapshot, diag_log_entries_from, PageRuntime, SnapshotInputs,
};
use ecu_target_common::ts::service::TsService;
use ecu_target_common::ts::state_ptr::{StatePtr, StateRef};
use ecu_ts::outpc::Outpc;
use ecu_ts::pages::{EcuPageStore, ExpertTriggerPageState, SystemSnapshot, DIAG_LOG_ENTRY_COUNT};
use ecu_ts::persistence::{PageStoreProvider, PersistedTsPageStore, TsPackageCommandStore};
use ecu_ts::server::OutpcProvider;
use ecu_ts::RuntimeSnapshotAdapter;
#[cfg(not(test))]
use stm32f4_ecu::{PIN_MAP_STM32F4_IGNITION_BRINGUP, RUNTIME_BUILD_ID_STM32F4_ECU};

/// Board-owned TunerStudio state, replacing the legacy compatibility-core
/// engine-state singleton. STM32F4 is a bring-up target with no flash-kv
/// calibration source, so the config starts at calibration defaults and the
/// runtime page surface is served with zeroed/idle snapshot scalars (the board
/// never ran the core snapshot refresh, so this preserves the prior byte
/// output).
pub(crate) struct BoardTsState {
    pub config: EcuConfig,
    diag_log: DiagLog<DIAG_LOG_ENTRY_COUNT>,
    snapshot: SystemSnapshot,
    expert_trigger: ExpertTriggerPageState,
    emerg_trig_map: bool,
    emerg_trig_tps: bool,
    tooth_count: u8,
    sync_loss_counter: u16,
}

/// Calibration-default [`EcuConfig`] for STM32F4 bring-up.
///
/// Mirrors the field values the legacy compatibility-core engine state produced
/// at construction, so the served calibration pages are byte-identical to the
/// previous behavior.
pub(crate) const fn default_ecu_config() -> EcuConfig {
    use ecu_calibration::configs::{
        AeConfig, AseConfig, ClConfig, DfcoConfig, FanConfig, IdleConfig, LambdaConfig,
        LoadFailureConfig, PlausibilityConfig, RateConfig, RevLimiterConfig, SensorsLimits,
        WueConfig,
    };
    use ecu_calibration::sensors::SensorsCal;

    EcuConfig {
        ipw_table: [[1000; 16]; 16],
        ve_table: [[100; 16]; 16],
        afr_table: [[147; 16]; 16],
        required_fuel_us: 1000,
        injector_deadtime_us: 800,
        ve_load_source: 0,
        ignition_table: [[15; 16]; 16],
        sensors_cal: SensorsCal::default(),
        sensors_limits: SensorsLimits::default(),
        ae_config: AeConfig::DEFAULT,
        wue_config: WueConfig::DEFAULT,
        ase_config: AseConfig::DEFAULT,
        dfco_config: DfcoConfig::DEFAULT,
        idle_config: IdleConfig::DEFAULT,
        fan_config: FanConfig::DEFAULT,
        cl_config: ClConfig::DEFAULT,
        load_failure_config: LoadFailureConfig::DEFAULT,
        plausibility_config: PlausibilityConfig::DEFAULT,
        rate_config: RateConfig::DEFAULT,
        lambda_config: LambdaConfig::DEFAULT,
        rev_limiter_config: RevLimiterConfig::DEFAULT,
        inj_angle_btdc_x10: [0; 16],
        tdc_per_cyl_x10: [0; 16],
        tooth0_angle_x10: 0,
        cam_missing_timeout_ms: 500,
    }
}

impl BoardTsState {
    pub(crate) fn new() -> Self {
        Self {
            config: default_ecu_config(),
            diag_log: DiagLog::new(),
            snapshot: build_system_snapshot(SnapshotInputs {
                rpm: 0,
                sync: ecu_domain::SyncState::Unsynced,
                base_pw_us: 0,
                enrich_mult_x100: 100,
                stft_x10: 0,
                fuel_mult_x100: 100,
                final_pw: Micros::new(0),
                last_fault_code: 0,
                fault_severity: ecu_ts::pages::TS_FAULT_SEVERITY_NONE,
                cancel_reason: ecu_ts::pages::TS_CANCEL_REASON_NONE,
                isr_count: 0,
                isr_max_us: 0,
                isr_avg_us: 0,
            }),
            expert_trigger: ExpertTriggerPageState::new(),
            emerg_trig_map: false,
            emerg_trig_tps: false,
            tooth_count: 0,
            sync_loss_counter: 0,
        }
    }

    fn page_store(&mut self) -> EcuPageStore<'_> {
        let diag_log_entries = diag_log_entries_from(&self.diag_log);
        build_ecu_page_store(
            &mut self.config,
            PageRuntime {
                snapshot: &self.snapshot,
                tooth_count: self.tooth_count,
                sync_loss_counter: self.sync_loss_counter,
                current_fault_code: self.snapshot.last_fault_code,
                current_fault_severity: self.snapshot.fault_severity,
                current_fault_action: ecu_ts::pages::TS_FAULT_ACTION_NONE,
                current_cancel_reason: self.snapshot.cancel_reason,
                fault_flags: u8::from(self.snapshot.last_fault_code != 0),
                latest_diag_code: ecu_target_common::ts::page_store::last_fault_code_from(
                    &self.diag_log,
                ),
                emerg_trig_map: &mut self.emerg_trig_map,
                emerg_trig_tps: &mut self.emerg_trig_tps,
                diag_log_entries,
                expert_trigger: &mut self.expert_trigger,
            },
        )
    }
}

// TS-only board overlay for values not already provided by the runtime snapshot.
#[derive(Copy, Clone)]
struct TsSensorOverlay {
    tps_percent: u8,
    clt_c: i16,
    iat_c: i16,
    vbatt_mv: u16,
}

impl TsSensorOverlay {
    const fn new() -> Self {
        Self {
            tps_percent: 0,
            clt_c: 20,
            iat_c: 25,
            vbatt_mv: 12500,
        }
    }

    fn update(&mut self) {
        self.vbatt_mv = 12500;
    }
}

static TS_SENSORS: Mutex<RefCell<TsSensorOverlay>> =
    Mutex::new(RefCell::new(TsSensorOverlay::new()));

pub(crate) fn update_sensors() {
    free(|cs| {
        TS_SENSORS.borrow(cs).borrow_mut().update();
    });
}

#[cfg(feature = "flash-kv")]
type StoreKv = crate::store_support::FlashKv;
#[cfg(not(feature = "flash-kv"))]
type StoreKv = RamKv512;

pub(crate) struct BoardTsPageStoreProvider {
    state: StatePtr<BoardTsState>,
}

impl BoardTsPageStoreProvider {
    pub(crate) fn new(state: &mut BoardTsState) -> Self {
        Self {
            state: StatePtr::new(state),
        }
    }
}

impl PageStoreProvider for BoardTsPageStoreProvider {
    type Pages<'a> = EcuPageStore<'a>;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let mut pages = state.page_store();
                f(&mut pages)
            })
        }
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let pages = state.page_store();
                f(&pages)
            })
        }
    }

    fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with(|state| {
                FuelRuntimeTune::new(
                    state.config.ve_table,
                    state.config.afr_table,
                    state.config.required_fuel_us,
                    state.config.injector_deadtime_us,
                    state.config.ve_load_source,
                )
            })
        }
    }
}

pub(crate) struct Provider {
    runtime: StateRef<ecu_runtime::EngineRuntime>,
    runtime_adapter: RuntimeSnapshotAdapter,
}

impl OutpcProvider for Provider {
    fn fill_outpc(&self, out: &mut Outpc) {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        let snapshot = unsafe { self.runtime.with(|runtime| runtime.snapshot()) };
        self.runtime_adapter.fill_outpc(&snapshot, out);
        free(|cs| {
            let sens = TS_SENSORS.borrow(cs).borrow();
            out.tps_percent = sens.tps_percent;
            out.clt_c = sens.clt_c;
            out.iat_c = sens.iat_c;
            out.vbatt_mv = sens.vbatt_mv;
            out.lambda_x100 = 100;
        });
    }

    fn engine_running(&self) -> bool {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        let snapshot = unsafe { self.runtime.with(|runtime| runtime.snapshot()) };
        matches!(snapshot.engine.sync, ecu_domain::SyncState::Locked { .. })
            && snapshot.engine.rpm.get() > 0
    }
}

type Service = TsService<
    Provider,
    TsPackageCommandStore<
        PersistedTsPageStore<BoardTsPageStoreProvider, StoreKv>,
        CompatCalibrationSession,
    >,
>;

pub(crate) fn new_service(
    state: &mut BoardTsState,
    runtime: &ecu_runtime::EngineRuntime,
) -> Service {
    let provider = Provider {
        runtime: StateRef::new(runtime),
        runtime_adapter: RuntimeSnapshotAdapter::new(),
    };
    let mut store = PersistedTsPageStore::new(BoardTsPageStoreProvider::new(state), StoreKv::new());
    store.try_load();
    let runtime_build_id = CalibrationRuntimeBuildId::new(RUNTIME_BUILD_ID_STM32F4_ECU.get());
    let hardware_target_id =
        CalibrationHardwareTargetId::new(PIN_MAP_STM32F4_IGNITION_BRINGUP.get());
    let mut session = CompatCalibrationSession::from_persisted_store(&store);
    let _ = store.try_load_and_import_package(&mut session, runtime_build_id, hardware_target_id);
    let store = TsPackageCommandStore::new(store, session, runtime_build_id, hardware_target_id);
    TsService::new(ecu_ts::TS_SIGNATURE, provider, store)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_ts::persistence::PageStoreProvider;
    use ecu_ts::server::PageStore;

    const PAGE_FUEL: u8 = 1;

    fn committed_header(seq: u16, fuel: &[u8], ign: &[u8], angles: &[u8]) -> KvHeader {
        KvHeader {
            ok: true,
            seq,
            fuel_len: fuel.len() as u16,
            ign_len: ign.len() as u16,
            angles_len: angles.len() as u16,
            fuel_crc: crc16(fuel),
            ign_crc: crc16(ign),
            angles_crc: crc16(angles),
        }
    }

    fn invalid_header() -> KvHeader {
        KvHeader {
            ok: false,
            seq: 0,
            fuel_len: 0,
            ign_len: 0,
            angles_len: 0,
            fuel_crc: 0,
            ign_crc: 0,
            angles_crc: 0,
        }
    }

    #[test]
    fn flash_layout_places_pages_inside_reserved_sector() {
        let layout = FlashLayout::STM32F405_TS_KV.unwrap();

        assert_eq!(layout.base_a, 0x080C_0000);
        assert_eq!(layout.base_b, 0x080E_0000);
        assert_eq!(layout.fuel.offset, PAGE_HEADER_LEN);
        assert_eq!(layout.ign.offset, PAGE_HEADER_LEN + FUEL_PAGE_LEN);
        assert_eq!(
            layout.angles.offset,
            PAGE_HEADER_LEN + FUEL_PAGE_LEN + IGN_PAGE_LEN
        );
        assert!(layout.angles.offset + layout.angles.len <= FlashLayout::SECTOR_BYTES);
    }

    #[test]
    fn flash_layout_rejects_invalid_bounds_and_aliases() {
        assert_eq!(
            FlashLayout::new(0x080C_0000, 0x080E_0000, 10, 11, 8, 1, 1, 1),
            Err(FlashLayoutError::HeaderTooSmall)
        );
        assert_eq!(
            FlashLayout::new(0x080C_0000, 0x080C_0000, 10, 11, PAGE_HEADER_LEN, 1, 1, 1),
            Err(FlashLayoutError::SectorAlias)
        );
        assert_eq!(
            FlashLayout::new(
                0x080C_0000,
                0x080E_0000,
                10,
                11,
                PAGE_HEADER_LEN,
                FlashLayout::SECTOR_BYTES,
                1,
                1
            ),
            Err(FlashLayoutError::PageOverflow)
        );
    }

    #[test]
    fn flash_header_page_validation_rejects_crc_and_length_mismatch() {
        let layout = FlashLayout::STM32F405_TS_KV.unwrap();
        let fuel = [0xA5; FUEL_PAGE_LEN];
        let ign = [0x5A; IGN_PAGE_LEN];
        let angles = [0x11; ANGLES_PAGE_LEN];
        let hdr = committed_header(7, &fuel, &ign, &angles);

        assert!(validate_page_image(layout, hdr, FlashPageKey::Fuel, &fuel));

        let mut corrupt = fuel;
        corrupt[0] ^= 0x01;
        assert!(!validate_page_image(
            layout,
            hdr,
            FlashPageKey::Fuel,
            &corrupt
        ));

        let short = &fuel[..FUEL_PAGE_LEN - 1];
        assert!(!validate_page_image(layout, hdr, FlashPageKey::Fuel, short));
    }

    #[test]
    fn flash_sequence_selection_handles_wrap_and_invalid_sector() {
        let empty = [0u8; FUEL_PAGE_LEN];
        let hdr_a = committed_header(10, &empty, &[0u8; IGN_PAGE_LEN], &[0u8; ANGLES_PAGE_LEN]);
        let hdr_b = committed_header(12, &empty, &[0u8; IGN_PAGE_LEN], &[0u8; ANGLES_PAGE_LEN]);
        assert_eq!(newest_sector(hdr_a, hdr_b), Some(ActiveSector::B));
        assert_eq!(target_sector_for_next_write(hdr_a, hdr_b), ActiveSector::A);

        let wrapped_old = committed_header(
            0xFFFE,
            &empty,
            &[0u8; IGN_PAGE_LEN],
            &[0u8; ANGLES_PAGE_LEN],
        );
        let wrapped_new =
            committed_header(1, &empty, &[0u8; IGN_PAGE_LEN], &[0u8; ANGLES_PAGE_LEN]);
        assert_eq!(
            newest_sector(wrapped_old, wrapped_new),
            Some(ActiveSector::B)
        );

        let invalid = KvHeader { ..invalid_header() };
        assert_eq!(newest_sector(invalid, invalid), None);
        assert_eq!(
            target_sector_for_next_write(invalid, invalid),
            ActiveSector::A
        );
    }

    #[test]
    fn page_store_provider_reuses_single_state_pointer_for_read_and_write_views() {
        let mut state = BoardTsState::new();
        let mut provider = BoardTsPageStoreProvider::new(&mut state);
        let mut page = [0u8; FUEL_PAGE_LEN];
        let mut out = [0u8; FUEL_PAGE_LEN];

        provider.with_pages(|pages| {
            assert_eq!(pages.read_page(PAGE_FUEL, &mut page), Some(FUEL_PAGE_LEN));
        });
        page[0] ^= 0x5A;

        provider.with_pages_mut(|pages| {
            pages
                .write_page(PAGE_FUEL, &page)
                .expect("fuel page write succeeds");
        });

        provider.with_pages(|pages| {
            assert_eq!(pages.read_page(PAGE_FUEL, &mut out), Some(FUEL_PAGE_LEN));
            assert_eq!(out, page);
        });
    }
}
