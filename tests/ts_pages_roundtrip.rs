//! Verify TS page stores for sensors/AE/DFCO and fuel/ign persistence

use ecu_core::persist::KvStore;
use ecu_core::ts::pages::{
    EcuStatePageStore, PersistedPageStore, PAGE_AE, PAGE_DFCO, PAGE_DIAG, PAGE_EXPERT_TRIGGER,
    PAGE_FUEL, PAGE_IGN, PAGE_SENSORS, TS_DIAG_BYTES, TS_EXPERT_TRIGGER_BYTES, TS_PAGE_DESCRIPTORS,
};
use ecu_core::ts::PageStore;
use ecu_target_common::kv::ram::RamKv512;
use ecu_target_common::ts::store::PersistedEcuPageStore;
struct SharedKv {
    inner: std::rc::Rc<std::cell::RefCell<RamKv512>>,
}
impl SharedKv {
    fn new() -> Self {
        Self {
            inner: std::rc::Rc::new(std::cell::RefCell::new(RamKv512::new())),
        }
    }
}
impl Clone for SharedKv {
    fn clone(&self) -> Self {
        Self {
            inner: self.inner.clone(),
        }
    }
}
impl KvStore for SharedKv {
    fn read(&mut self, key: &[u8], out: &mut [u8]) -> Result<usize, ecu_core::persist::KvError> {
        self.inner.borrow_mut().read(key, out)
    }
    fn write(&mut self, key: &[u8], data: &[u8]) -> Result<(), ecu_core::persist::KvError> {
        self.inner.borrow_mut().write(key, data)
    }
}
use ecu_calibration::{
    CalibrationSchemaVersion, ExpertIgnitionMode, ExpertInjectionLayout, ExpertTriggerCalibration,
    ExpertUnlock, PrimaryTriggerSpeed, SecondaryTriggerMode, TriggerAuthority, TriggerEdge,
    TriggerFilter, TriggerPattern,
};
use ecu_core::EcuState;
use ecu_domain::{
    AbsoluteTimeAuthority, CrankSyncState, EngineTimeAuthority, Kpa10, Micros, PhaseSyncState, Rpm,
};
use ecu_runtime::{
    runtime_full_sequential_authorized, Action, ControlInputs, EngineRuntime, EnrichmentInputs,
    IgnitionInputs, LambdaTrimInputs, StepInputs, TorqueInputs,
};

#[test]
fn sensors_ae_dfco_read_write() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();

    // Build sensors page payload (128 bytes) with modified values
    let mut sdata = [0u8; 128];
    // tps min/max
    sdata[0..2].copy_from_slice(&100u16.to_le_bytes());
    sdata[2..4].copy_from_slice(&3900u16.to_le_bytes());
    // map v0/kpa0, v1/kpa1
    sdata[4..6].copy_from_slice(&600u16.to_le_bytes());
    sdata[6..8].copy_from_slice(&90u16.to_le_bytes());
    sdata[8..10].copy_from_slice(&4600u16.to_le_bytes());
    sdata[10..12].copy_from_slice(&2400u16.to_le_bytes());
    // leave the rest default; write and read back
    pages
        .write_page(PAGE_SENSORS, &sdata)
        .expect("write sensors");
    let mut out = [0u8; 128];
    let n = pages
        .read_page(PAGE_SENSORS, &mut out)
        .expect("read sensors");
    assert_eq!(n, 128);
    assert_eq!(&out[..12], &sdata[..12]);

    // Limits page (10 bytes)
    let mut ldata = [0u8; 10];
    ldata[0..2].copy_from_slice(&150u16.to_le_bytes()); // map_min
    ldata[2..4].copy_from_slice(&2800u16.to_le_bytes()); // map_max
    ldata[4] = 2; // tps_min
    ldata[5] = 98; // tps_max
    ldata[6..8].copy_from_slice(&2u16.to_le_bytes()); // clear_time_s
    ldata[8] = 0b11; // both triggers on
    ldata[9] = 0; // reserved
    pages
        .write_page(ecu_core::ts::pages::PAGE_LIMITS, &ldata)
        .expect("write limits");
    let mut lread = [0u8; 10];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_LIMITS, &mut lread)
        .expect("read limits");
    assert_eq!(n, 10);
    assert_eq!(&lread[..], &ldata[..]);

    // AE page (16 bytes) write/read
    let mut aedata = [0u8; 16];
    aedata[0..2].copy_from_slice(&(-50i16).to_le_bytes()); // tpsdot_thresh
    aedata[2..4].copy_from_slice(&(25i16).to_le_bytes()); // mapdot_thresh
    aedata[4] = 15; // percent_gain
                    // pad one byte
    aedata[6..10].copy_from_slice(&(250u32).to_le_bytes()); // decay
    aedata[10..14].copy_from_slice(&(500u32).to_le_bytes()); // lockout
    pages.write_page(PAGE_AE, &aedata).expect("write ae");
    let mut aeread = [0u8; 16];
    let n = pages.read_page(PAGE_AE, &mut aeread).expect("read ae");
    assert_eq!(n, 16);
    assert_eq!(&aedata[..], &aeread[..]);

    // DFCO page (16 bytes) write/read
    let mut d = [0u8; 16];
    d[0] = 5; // tps_max_pct
    d[2..4].copy_from_slice(&80u16.to_le_bytes()); // map_max_kpa
    d[4..6].copy_from_slice(&1800u16.to_le_bytes()); // rpm_min
    d[6..8].copy_from_slice(&3500u16.to_le_bytes()); // rpm_max
    d[8..12].copy_from_slice(&(500u32).to_le_bytes()); // delay
    d[12..16].copy_from_slice(&(1000u32).to_le_bytes()); // resume hyst
    pages.write_page(PAGE_DFCO, &d).expect("write dfco");
    let mut dread = [0u8; 16];
    let n = pages.read_page(PAGE_DFCO, &mut dread).expect("read dfco");
    assert_eq!(n, 16);
    assert_eq!(&d[..], &dread[..]);

    // Idle page (6 bytes) write/read
    let mut idle = [0u8; 6];
    idle[0] = 1; // enable
    idle[1..3].copy_from_slice(&(450u16).to_le_bytes()); // 45.0%
    idle[3..5].copy_from_slice(&(100u16).to_le_bytes()); // 100Hz
    pages
        .write_page(ecu_core::ts::pages::PAGE_IDLE, &idle)
        .expect("write idle");
    let mut idread = [0u8; 6];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_IDLE, &mut idread)
        .expect("read idle");
    assert_eq!(n, 6);
    assert_eq!(&idle[..], &idread[..]);

    // Fan page (6 bytes) write/read
    let mut fan = [0u8; 6];
    fan[0] = 1; // enable
    fan[1..3].copy_from_slice(&(95i16).to_le_bytes()); // on at 95C
    fan[3..5].copy_from_slice(&(90i16).to_le_bytes()); // off at 90C
    pages
        .write_page(ecu_core::ts::pages::PAGE_FAN, &fan)
        .expect("write fan");
    let mut fnread = [0u8; 6];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_FAN, &mut fnread)
        .expect("read fan");
    assert_eq!(n, 6);
    assert_eq!(&fan[..], &fnread[..]);

    // CL page (8 bytes) write/read
    let mut cl = [0u8; 8];
    cl[0] = 1; // enable
    cl[2..4].copy_from_slice(&(147u16).to_le_bytes());
    cl[4..6].copy_from_slice(&(10u16).to_le_bytes());
    cl[6..8].copy_from_slice(&(2u16).to_le_bytes());
    pages
        .write_page(ecu_core::ts::pages::PAGE_CL, &cl)
        .expect("write cl");
    let mut clread = [0u8; 8];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_CL, &mut clread)
        .expect("read cl");
    assert_eq!(n, 8);
    assert_eq!(&cl[..], &clread[..]);

    // WUE page (8 bytes) write/read
    let mut wue = [0u8; 8];
    wue[0] = 30; // max percent
    wue[1] = 0; // min percent
    wue[2..4].copy_from_slice(&(-10i16).to_le_bytes()); // start_c
    wue[4..6].copy_from_slice(&(60i16).to_le_bytes()); // end_c
    pages
        .write_page(ecu_core::ts::pages::PAGE_WUE, &wue)
        .expect("write wue");
    let mut wread = [0u8; 8];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_WUE, &mut wread)
        .expect("read wue");
    assert_eq!(n, 8);
    assert_eq!(&wue[..], &wread[..]);

    // ASE page (8 bytes) write/read
    let mut ase = [0u8; 8];
    ase[0] = 15; // percent
    ase[2..6].copy_from_slice(&(3000u32).to_le_bytes()); // taper 3s
    ase[6..8].copy_from_slice(&(1000u16).to_le_bytes()); // lockout 1s
    pages
        .write_page(ecu_core::ts::pages::PAGE_ASE, &ase)
        .expect("write ase");
    let mut aread = [0u8; 8];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_ASE, &mut aread)
        .expect("read ase");
    assert_eq!(n, 8);
    assert_eq!(&ase[..], &aread[..]);
}

#[test]
fn sensors_ae_dfco_negative_sizes_and_ranges() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();

    // Limits page invalid cases
    let mut bad = [0u8; 10];
    // map_min=0 invalid
    bad[0..2].copy_from_slice(&0u16.to_le_bytes());
    bad[2..4].copy_from_slice(&2000u16.to_le_bytes());
    bad[4] = 1; // tps_min
    bad[5] = 50; // tps_max
    bad[6..8].copy_from_slice(&1u16.to_le_bytes());
    bad[8] = 0;
    bad[9] = 0;
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad)
        .is_err());

    // map_max <= map_min invalid
    bad[0..2].copy_from_slice(&100u16.to_le_bytes());
    bad[2..4].copy_from_slice(&100u16.to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad)
        .is_err());

    // tps_max > 100 invalid
    bad[2..4].copy_from_slice(&2000u16.to_le_bytes());
    bad[5] = 150;
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad)
        .is_err());

    // tps_max <= tps_min invalid
    bad[4] = 90;
    bad[5] = 80;
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad)
        .is_err());

    // clear_time_s == 0 invalid
    bad[4] = 1;
    bad[5] = 2;
    bad[6..8].copy_from_slice(&0u16.to_le_bytes());
    assert!(pages
        .write_page(ecu_core::ts::pages::PAGE_LIMITS, &bad)
        .is_err());

    // Wrong size payloads
    assert!(pages.write_page(PAGE_SENSORS, &[0u8; 10]).is_err());
    assert!(pages.write_page(PAGE_AE, &[0u8; 8]).is_err());
    assert!(pages.write_page(PAGE_DFCO, &[0u8; 8]).is_err());

    // Invalid ranges: TPS min >= max, MAP calibration not increasing
    let mut sdata = [0u8; 128];
    sdata[0..2].copy_from_slice(&4000u16.to_le_bytes()); // min
    sdata[2..4].copy_from_slice(&1000u16.to_le_bytes()); // max (invalid)
    assert!(pages.write_page(PAGE_SENSORS, &sdata).is_err());

    let mut sdata2 = [0u8; 128];
    sdata2[0..2].copy_from_slice(&100u16.to_le_bytes());
    sdata2[2..4].copy_from_slice(&200u16.to_le_bytes());
    sdata2[4..6].copy_from_slice(&2000u16.to_le_bytes()); // v0
    sdata2[6..8].copy_from_slice(&500u16.to_le_bytes()); // k0
    sdata2[8..10].copy_from_slice(&1500u16.to_le_bytes()); // v1 <= v0 (invalid)
    sdata2[10..12].copy_from_slice(&400u16.to_le_bytes()); // k1 <= k0
    assert!(pages.write_page(PAGE_SENSORS, &sdata2).is_err());

    // AE: percent_gain > 100
    let mut aedata = [0u8; 16];
    aedata[4] = 150; // percent
    assert!(pages.write_page(PAGE_AE, &aedata).is_err());

    // DFCO: tps_max > 100, rpm_min 0, rpm_max < rpm_min
    let mut dfco = [0u8; 16];
    dfco[0] = 150; // tps_max invalid
    assert!(pages.write_page(PAGE_DFCO, &dfco).is_err());
}

#[test]
fn angles_page_roundtrip() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();

    let mut ang = [0u8; 68];
    for i in 0..16u16 {
        let off = (i as usize) * 2;
        ang[off..off + 2].copy_from_slice(&(i * 10).to_le_bytes());
    }
    for i in 0..16u16 {
        let off = 32 + (i as usize) * 2;
        ang[off..off + 2].copy_from_slice(&(100 + i).to_le_bytes());
    }
    ang[64..66].copy_from_slice(&123u16.to_le_bytes());
    ang[66..68].copy_from_slice(&750u16.to_le_bytes()); // cam timeout ms
    pages
        .write_page(ecu_core::ts::pages::PAGE_ANGLES, &ang)
        .expect("write angles");
    let mut out = [0u8; 68];
    let n = pages
        .read_page(ecu_core::ts::pages::PAGE_ANGLES, &mut out)
        .expect("read angles");
    assert_eq!(n, 68);
    assert_eq!(&out[..], &ang[..]);
}

#[test]
fn fuel_ign_page_roundtrip() {
    let mut state = EcuState::new();
    let mut pages = state.page_store();

    let mut fuel = [0u8; 512];
    fuel[0..2].copy_from_slice(&1234u16.to_le_bytes());
    fuel[30..32].copy_from_slice(&4321u16.to_le_bytes());
    let mut ign = [0u8; 512];
    ign[0..2].copy_from_slice(&(-7i16).to_le_bytes());
    ign[30..32].copy_from_slice(&(12i16).to_le_bytes());

    for (page, payload) in [(PAGE_FUEL, fuel), (PAGE_IGN, ign)] {
        assert_eq!(pages.page_len(page), Some(512));
        pages
            .write_page(page, &payload)
            .expect("write persisted page");
        let mut out = [0u8; 512];
        let n = pages
            .read_page(page, &mut out)
            .expect("read persisted page");
        assert_eq!(n, 512);
        assert_eq!(out, payload);
    }
}

#[test]
fn fuel_ign_persist_roundtrip() {
    let mut state = EcuState::new();
    // change a couple cells
    state.config.ipw_table[0][0] = 1234;
    state.config.ignition_table[0][0] = -7;

    let kv = SharedKv::new();
    {
        let mut store = PersistedPageStore::new(
            EcuStatePageStore {
                fuel: &mut state.config.ipw_table,
                ign: &mut state.config.ignition_table,
            },
            kv.clone(),
        );
        // Burn to KV
        store.burn().expect("burn");
    }
    // Reset state and load from KV using a fresh store (release prior borrows)
    state.config.ipw_table[0][0] = 0;
    state.config.ignition_table[0][0] = 0;
    let mut store2 = PersistedPageStore::new(
        EcuStatePageStore {
            fuel: &mut state.config.ipw_table,
            ign: &mut state.config.ignition_table,
        },
        kv.clone(),
    );
    store2.try_load();
    assert_eq!(state.config.ipw_table[0][0], 1234);
    assert_eq!(state.config.ignition_table[0][0], -7);
}

fn sample_expert_trigger() -> ExpertTriggerCalibration {
    ExpertTriggerCalibration {
        expert_unlock: ExpertUnlock::Unlocked,
        authority: TriggerAuthority::ExpertManual,
        profile_identity: 0x4D353054,
        profile_hash: 0xE771_6601,
        trigger_pattern: TriggerPattern::MissingTooth,
        primary_base_teeth: 60,
        missing_teeth: 2,
        primary_trigger_speed: PrimaryTriggerSpeed::Crank,
        trigger_angle_atdc_deg10: 840,
        primary_trigger_edge: TriggerEdge::Falling,
        secondary_trigger_edge: TriggerEdge::Rising,
        secondary_trigger_mode: SecondaryTriggerMode::SingleToothCam,
        trigger_filter: TriggerFilter::Aggressive,
        resync_every_cycle: true,
        skip_cycles: 4,
        ignition_mode: ExpertIgnitionMode::SequentialCop,
        injection_layout: ExpertInjectionLayout::Sequential,
        ..ExpertTriggerCalibration::default()
    }
}

#[test]
fn expert_trigger_page_is_advertised() {
    let descriptor = TS_PAGE_DESCRIPTORS
        .iter()
        .find(|descriptor| descriptor.page == PAGE_EXPERT_TRIGGER)
        .expect("expert trigger descriptor");

    assert_eq!(descriptor.len, TS_EXPERT_TRIGGER_BYTES);
    assert!(descriptor.writable);
    assert_eq!(descriptor.label, "expert_trigger");
}

#[test]
fn expert_trigger_page_roundtrip() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let expert = sample_expert_trigger();
    let mut page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    expert
        .encode_record(&mut page)
        .expect("encode expert trigger");

    assert_eq!(
        store.page_len(PAGE_EXPERT_TRIGGER),
        Some(ecu_calibration::EXPERT_TRIGGER_RECORD_LEN)
    );
    store
        .write_page(PAGE_EXPERT_TRIGGER, &page)
        .expect("write expert trigger");

    let mut out = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    assert_eq!(
        store.read_page(PAGE_EXPERT_TRIGGER, &mut out),
        Some(ecu_calibration::EXPERT_TRIGGER_RECORD_LEN)
    );
    assert_eq!(ExpertTriggerCalibration::decode_record(&out), Ok(expert));
}

#[test]
fn invalid_expert_trigger_page_is_rejected_before_burn() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let expert = sample_expert_trigger();
    let mut page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    expert
        .encode_record(&mut page)
        .expect("encode expert trigger");
    page[2] = ExpertUnlock::Locked.code();

    assert!(store.write_page(PAGE_EXPERT_TRIGGER, &page).is_err());
}

#[test]
fn certified_expert_trigger_profile_is_rejected_by_ts_page_store() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let certified = ExpertTriggerCalibration {
        expert_unlock: ExpertUnlock::Locked,
        authority: TriggerAuthority::CertifiedProfile,
        profile_identity: 0x4D353054,
        profile_hash: 0x55AA_1234,
        ..ExpertTriggerCalibration::default()
    };
    let mut page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    certified
        .encode_record(&mut page)
        .expect("encode certified expert trigger");

    assert!(store.write_page(PAGE_EXPERT_TRIGGER, &page).is_err());
}

#[test]
fn expert_trigger_resync_every_cycle_is_canonicalized_on_write() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let expert = sample_expert_trigger();
    let mut page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    expert
        .encode_record(&mut page)
        .expect("encode expert trigger");
    page[24] = 2;

    store
        .write_page(PAGE_EXPERT_TRIGGER, &page)
        .expect("write expert trigger");

    let mut out = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    assert_eq!(
        store.read_page(PAGE_EXPERT_TRIGGER, &mut out),
        Some(ecu_calibration::EXPERT_TRIGGER_RECORD_LEN)
    );
    assert_eq!(out[24], 1);
    let decoded = ExpertTriggerCalibration::decode_record(&out).expect("decode expert trigger");
    assert!(decoded.resync_every_cycle);
}

#[test]
fn expert_unlock_persists_with_schema_version() {
    let mut state = EcuState::new();
    let kv = SharedKv::new();
    let expert = sample_expert_trigger();
    let mut page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    expert
        .encode_record(&mut page)
        .expect("encode expert trigger");
    {
        let mut store = PersistedEcuPageStore::new(&mut state, kv.clone());
        store
            .write_page(PAGE_EXPERT_TRIGGER, &page)
            .expect("write expert trigger");
        store.burn().expect("burn expert trigger");
    }

    let mut store = PersistedEcuPageStore::new(&mut state, kv.clone());
    store.try_load();
    let mut out = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    assert_eq!(
        store.read_page(PAGE_EXPERT_TRIGGER, &mut out),
        Some(ecu_calibration::EXPERT_TRIGGER_RECORD_LEN)
    );
    let decoded = ExpertTriggerCalibration::decode_record(&out).expect("decode expert trigger");
    assert_eq!(decoded.schema_version, CalibrationSchemaVersion::CURRENT);
    assert_eq!(decoded.expert_unlock, ExpertUnlock::Unlocked);
    assert_eq!(decoded, expert);
}

fn runtime_authority(
    phase: PhaseSyncState,
    absolute: AbsoluteTimeAuthority,
) -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        CrankSyncState::PrimaryLocked,
        phase,
        absolute,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    )
}

fn runtime_step_inputs() -> StepInputs {
    StepInputs {
        now_us: Micros::new(6_000),
        rpm: 3_000,
        load_kpa10: Kpa10::new(700).get() as u32,
        angle_x10: 2_000,
        trigger_synced: true,
        cam_seen: true,
        launch_armed: false,
        flat_shift_armed: false,
    }
}

fn runtime_control_inputs() -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(6_000),
            clt_c: 20,
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            clt_c: 80,
            lambda_valid: true,
            measured_lambda100: ecu_domain::Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(
            ecu_domain::Degrees10::new(100),
            0,
            0,
            0,
            false,
            Rpm::new(3_000),
        ),
    }
}

#[test]
fn expert_trigger_persistence_unlock_does_not_arm_m50_full_cop_without_runtime_authority() {
    let kv = SharedKv::new();
    let expert = sample_expert_trigger();
    let mut page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    expert
        .encode_record(&mut page)
        .expect("encode expert trigger");

    {
        let mut state = EcuState::new();
        let mut store = PersistedEcuPageStore::new(&mut state, kv.clone());
        store
            .write_page(PAGE_EXPERT_TRIGGER, &page)
            .expect("write expert trigger");
        store.burn().expect("burn expert trigger");
    }

    let mut state = EcuState::new();
    let mut store = PersistedEcuPageStore::new(&mut state, kv.clone());
    store.try_load();
    let mut out = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    assert_eq!(
        store.read_page(PAGE_EXPERT_TRIGGER, &mut out),
        Some(ecu_calibration::EXPERT_TRIGGER_RECORD_LEN)
    );
    let persisted = ExpertTriggerCalibration::decode_record(&out).expect("decode expert trigger");
    assert_eq!(persisted.expert_unlock, ExpertUnlock::Unlocked);
    assert_eq!(persisted.authority, TriggerAuthority::ExpertManual);

    let mut runtime = EngineRuntime::new();
    runtime.configure_m50_full_cop();
    let result = runtime.step(runtime_step_inputs(), runtime_control_inputs());
    assert_eq!(
        result
            .actions
            .iter()
            .filter(|action| matches!(action, Action::ArmScheduler { .. }))
            .count(),
        0
    );
    assert!(!runtime_full_sequential_authorized(
        runtime.engine_time_authority()
    ));

    let crank_only = runtime_authority(
        PhaseSyncState::CrankOnly360,
        AbsoluteTimeAuthority::GeometryOnly,
    );
    assert!(persisted
        .to_runtime_engine_time_authority(crank_only)
        .is_err());
    runtime.set_engine_time_authority(crank_only);
    let result = runtime.step(runtime_step_inputs(), runtime_control_inputs());
    assert_eq!(
        result
            .actions
            .iter()
            .filter(|action| matches!(action, Action::ArmScheduler { .. }))
            .count(),
        0
    );

    let validated = persisted
        .to_runtime_engine_time_authority(runtime_authority(
            PhaseSyncState::CamValidated720,
            AbsoluteTimeAuthority::GeometryOnly,
        ))
        .expect("validated expert runtime authority");
    runtime.set_engine_time_authority(validated);
    let result = runtime.step(runtime_step_inputs(), runtime_control_inputs());
    assert_eq!(
        result
            .actions
            .iter()
            .filter(|action| matches!(action, Action::ArmScheduler { .. }))
            .count(),
        6
    );
}

#[test]
fn diag_page_exposes_placeholder_contract_and_live_trigger_evidence() {
    let mut state = EcuState::new();
    let expert = sample_expert_trigger();
    let mut expert_page = [0u8; ecu_calibration::EXPERT_TRIGGER_RECORD_LEN];
    expert
        .encode_record(&mut expert_page)
        .expect("encode expert trigger");

    state.set_trigger_inputs(2_750, true, 19);
    state.record_sync_loss(1_234);
    state.set_synced(true);
    state.refresh_snapshot();

    let mut store = state.page_store();
    store
        .write_page(PAGE_EXPERT_TRIGGER, &expert_page)
        .expect("write expert trigger");

    let mut out = [0u8; TS_DIAG_BYTES];
    assert_eq!(store.page_len(PAGE_DIAG), Some(TS_DIAG_BYTES));
    assert_eq!(store.read_page(PAGE_DIAG, &mut out), Some(TS_DIAG_BYTES));
    assert_eq!(out[0], 19);
    assert_eq!(out[1], 0);
    assert_eq!(out[2], 2);
    assert_eq!(out[3], 2);
    assert_eq!(out[4], TriggerAuthority::ExpertManual.code());
    assert_eq!(out[5], 0);
    assert_eq!(out[6], 0);
    assert_eq!(out[7], 0);
    assert_eq!(u16::from_le_bytes([out[8], out[9]]), 2_750);
    assert_eq!(u16::from_le_bytes([out[10], out[11]]), 0);
    assert_eq!(u16::from_le_bytes([out[12], out[13]]), 1);
    assert_eq!(u16::from_le_bytes([out[14], out[15]]), 0);
    assert_eq!(
        u32::from_le_bytes([out[16], out[17], out[18], out[19]]),
        expert.profile_identity
    );
    assert_eq!(
        u32::from_le_bytes([out[20], out[21], out[22], out[23]]),
        expert.profile_hash
    );
    assert_eq!(&out[24..], &[0; 8]);
}
