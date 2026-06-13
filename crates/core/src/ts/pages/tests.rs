use super::*;
use crate::constants::fuel as fuel_consts;
use crate::trigger::SyncState;
use crate::units::{Micros, Rpm};
use crate::EcuState;
use ecu_ts::pages::{
    AePage, AsePage, DfcoPage, PageCodecFamily, TsPageDescriptor, WuePage, AE_PAGE_BYTES,
    ANGLES_PAGE_BYTES, ASE_PAGE_BYTES, DFCO_PAGE_BYTES, DIAG_LOG_PAGE_BYTES, LIMITS_PAGE_BYTES,
    SENSORS_PAGE_BYTES, SNAPSHOT_PAGE_BYTES, WUE_PAGE_BYTES,
};
use ecu_ts::server::{PageError, PageStore};

#[test]
fn descriptor_registry_covers_all_known_pages() {
    const EXPECTED: [TsPageDescriptor; TS_PAGE_COUNT] = [
        TsPageDescriptor {
            page: PAGE_FUEL,
            len: TS_PAGE_BYTES,
            writable: true,
            label: "fuel",
            persisted_setup_page: true,
            codec_family: PageCodecFamily::FuelTable,
        },
        TsPageDescriptor {
            page: PAGE_IGN,
            len: TS_PAGE_BYTES,
            writable: true,
            label: "ign",
            persisted_setup_page: true,
            codec_family: PageCodecFamily::IgnitionTable,
        },
        TsPageDescriptor {
            page: PAGE_SENSORS,
            len: SENSORS_PAGE_BYTES,
            writable: true,
            label: "sensors",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Sensors,
        },
        TsPageDescriptor {
            page: PAGE_AE,
            len: AE_PAGE_BYTES,
            writable: true,
            label: "ae",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Ae,
        },
        TsPageDescriptor {
            page: PAGE_DFCO,
            len: DFCO_PAGE_BYTES,
            writable: true,
            label: "dfco",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Dfco,
        },
        TsPageDescriptor {
            page: PAGE_LIMITS,
            len: LIMITS_PAGE_BYTES,
            writable: true,
            label: "limits",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Limits,
        },
        TsPageDescriptor {
            page: PAGE_DIAG,
            len: TS_DIAG_BYTES,
            writable: false,
            label: "diag",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Diag,
        },
        TsPageDescriptor {
            page: PAGE_DIAG_LOG,
            len: DIAG_LOG_PAGE_BYTES,
            writable: false,
            label: "diag_log",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::DiagLog,
        },
        TsPageDescriptor {
            page: PAGE_ANGLES,
            len: ANGLES_PAGE_BYTES,
            writable: true,
            label: "angles",
            persisted_setup_page: true,
            codec_family: PageCodecFamily::Angles,
        },
        TsPageDescriptor {
            page: PAGE_WUE,
            len: WUE_PAGE_BYTES,
            writable: true,
            label: "wue",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Wue,
        },
        TsPageDescriptor {
            page: PAGE_ASE,
            len: ASE_PAGE_BYTES,
            writable: true,
            label: "ase",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Ase,
        },
        TsPageDescriptor {
            page: PAGE_IDLE,
            len: 6,
            writable: true,
            label: "idle",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Idle,
        },
        TsPageDescriptor {
            page: PAGE_FAN,
            len: 6,
            writable: true,
            label: "fan",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Fan,
        },
        TsPageDescriptor {
            page: PAGE_CL,
            len: 8,
            writable: true,
            label: "cl",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::ClosedLoop,
        },
        TsPageDescriptor {
            page: PAGE_SNAPSHOT,
            len: SNAPSHOT_PAGE_BYTES,
            writable: false,
            label: "snapshot",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Snapshot,
        },
        TsPageDescriptor {
            page: PAGE_EXPERT_TRIGGER,
            len: TS_EXPERT_TRIGGER_BYTES,
            writable: true,
            label: "expert_trigger",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::ExpertTrigger,
        },
        TsPageDescriptor {
            page: PAGE_VE_TUNE,
            len: 16,
            writable: true,
            label: "ve_tune",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::VeTune,
        },
        TsPageDescriptor {
            page: PAGE_VE_TABLE,
            len: TS_PAGE_BYTES,
            writable: true,
            label: "ve_table",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::VeTable,
        },
        TsPageDescriptor {
            page: PAGE_AFR_TABLE,
            len: TS_PAGE_BYTES,
            writable: true,
            label: "afr_table",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::AfrTable,
        },
    ];

    assert_eq!(TS_PAGE_DESCRIPTORS, EXPECTED);
    for descriptor in TS_PAGE_DESCRIPTORS {
        assert_eq!(
            ts_page_descriptor(descriptor.page),
            Some(descriptor),
            "descriptor lookup must be stable for page {}",
            descriptor.page
        );
    }
    assert!(ts_page_descriptor(99).is_none());
}

#[test]
fn descriptor_lengths_match_read_paths() {
    let mut state = EcuState::new();
    let store = state.page_store();

    for descriptor in TS_PAGE_DESCRIPTORS {
        assert_eq!(store.page_len(descriptor.page), Some(descriptor.len));
        let mut out = vec![0u8; descriptor.len];
        assert_eq!(
            store.read_page(descriptor.page, &mut out),
            Some(descriptor.len),
            "page {} ({}): read size drift",
            descriptor.page,
            descriptor.label
        );
    }
}

#[test]
fn snapshot_page_delegates_legacy_32_byte_layout() {
    let mut state = EcuState::new();
    state.snapshot = SystemSnapshot {
        rpm: Rpm::new(0x1234),
        sync: SyncState::Locked { cam_ref: true },
        base_pw: Micros::new(0x0102_0304),
        enrich_mult_x100: 0x0506,
        stft_x10: -123,
        fuel_mult_x100: 0x0708,
        final_pw: Micros::new(0x1112_1314),
        last_fault_code: 8,
        isr_count: 0x2122_2324,
        isr_max_us: 0x3132_3334,
        isr_avg_us: 0x4142_4344,
    };
    let store = state.page_store();
    let mut out = [0xAAu8; SNAPSHOT_PAGE_BYTES + 2];

    assert_eq!(
        store.read_page(PAGE_SNAPSHOT, &mut out),
        Some(SNAPSHOT_PAGE_BYTES)
    );

    let mut expected = [0u8; SNAPSHOT_PAGE_BYTES];
    expected[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
    expected[2] = 2;
    expected[3] = 0;
    expected[4..8].copy_from_slice(&0x0102_0304u32.to_le_bytes());
    expected[8..10].copy_from_slice(&0x0506u16.to_le_bytes());
    expected[10..12].copy_from_slice(&(-123i16).to_le_bytes());
    expected[12..14].copy_from_slice(&0x0708u16.to_le_bytes());
    expected[14..18].copy_from_slice(&0x1112_1314u32.to_le_bytes());
    expected[18] = 8;
    expected[19] = 0;
    expected[20..24].copy_from_slice(&0x2122_2324u32.to_le_bytes());
    expected[24..28].copy_from_slice(&0x3132_3334u32.to_le_bytes());
    expected[28..32].copy_from_slice(&0x4142_4344u32.to_le_bytes());
    assert_eq!(&out[..SNAPSHOT_PAGE_BYTES], &expected);
    assert_eq!(&out[SNAPSHOT_PAGE_BYTES..], &[0xAA, 0xAA]);

    let mut short_out = [0u8; SNAPSHOT_PAGE_BYTES - 1];
    assert_eq!(store.read_page(PAGE_SNAPSHOT, &mut short_out), None);
}

#[test]
fn fuel_and_ign_pages_roundtrip_core_tables() {
    let mut state = EcuState::new();
    let mut fuel_page = [0u8; TS_PAGE_BYTES];
    let mut ign_page = [0u8; TS_PAGE_BYTES];
    fuel_page[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
    fuel_page[2..4].copy_from_slice(&0xABCDu16.to_le_bytes());
    fuel_page[510..512].copy_from_slice(&0x0102u16.to_le_bytes());
    ign_page[0..2].copy_from_slice(&(-100i16).to_le_bytes());
    ign_page[2..4].copy_from_slice(&250i16.to_le_bytes());
    ign_page[510..512].copy_from_slice(&(-1i16).to_le_bytes());

    let mut store = state.page_store();
    assert!(store.write_page(PAGE_FUEL, &fuel_page).is_ok());
    assert!(store.write_page(PAGE_IGN, &ign_page).is_ok());

    let mut out = [0u8; TS_PAGE_BYTES];
    assert_eq!(store.read_page(PAGE_FUEL, &mut out), Some(TS_PAGE_BYTES));
    assert_eq!(out, fuel_page);
    assert_eq!(store.read_page(PAGE_IGN, &mut out), Some(TS_PAGE_BYTES));
    assert_eq!(out, ign_page);
}

#[test]
fn ve_and_afr_pages_roundtrip_core_tables() {
    let mut state = EcuState::new();
    let mut ve_page = [0u8; TS_PAGE_BYTES];
    let mut afr_page = [0u8; TS_PAGE_BYTES];
    ve_page[0..2].copy_from_slice(&0x1234u16.to_le_bytes());
    ve_page[2..4].copy_from_slice(&0xABCDu16.to_le_bytes());
    ve_page[510..512].copy_from_slice(&0x0102u16.to_le_bytes());
    for idx in (0..TS_PAGE_BYTES).step_by(2) {
        afr_page[idx..idx + 2].copy_from_slice(&147u16.to_le_bytes());
    }
    afr_page[0..2].copy_from_slice(&120u16.to_le_bytes());
    afr_page[2..4].copy_from_slice(&220u16.to_le_bytes());
    afr_page[510..512].copy_from_slice(&100u16.to_le_bytes());

    let mut store = state.page_store();
    assert!(store.write_page(PAGE_VE_TABLE, &ve_page).is_ok());
    assert!(store.write_page(PAGE_AFR_TABLE, &afr_page).is_ok());

    let mut out = [0u8; TS_PAGE_BYTES];
    assert_eq!(
        store.read_page(PAGE_VE_TABLE, &mut out),
        Some(TS_PAGE_BYTES)
    );
    assert_eq!(out, ve_page);
    assert_eq!(
        store.read_page(PAGE_AFR_TABLE, &mut out),
        Some(TS_PAGE_BYTES)
    );
    assert_eq!(out, afr_page);
    assert_eq!(store.cl.target_afr_x10, 120);
}

#[test]
fn afr_table_page_rejects_wrong_size_and_invalid_targets() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    assert!(matches!(
        store.write_page(PAGE_AFR_TABLE, &[0u8; TS_PAGE_BYTES - 1]),
        Err(PageError::WrongSize)
    ));

    let mut afr_page = [0u8; TS_PAGE_BYTES];
    for idx in (0..TS_PAGE_BYTES).step_by(2) {
        afr_page[idx..idx + 2].copy_from_slice(&147u16.to_le_bytes());
    }
    afr_page[20..22].copy_from_slice(&221u16.to_le_bytes());

    assert!(matches!(
        store.write_page(PAGE_AFR_TABLE, &afr_page),
        Err(PageError::Invalid)
    ));
}

#[test]
fn enrichment_pages_roundtrip_through_core_state() {
    let mut state = EcuState::new();
    let mut store = state.page_store();

    let ae = AePage::new(-125, 75, 25, 450, 175);
    let mut ae_payload = [0u8; AE_PAGE_BYTES];
    ae.encode(&mut ae_payload).expect("encode ae");
    assert!(store.write_page(PAGE_AE, &ae_payload).is_ok());
    let mut out_ae = [0u8; AE_PAGE_BYTES];
    assert_eq!(store.read_page(PAGE_AE, &mut out_ae), Some(AE_PAGE_BYTES));
    assert_eq!(out_ae, ae_payload);

    let dfco = DfcoPage::new(3, 35, 1600, 6500, 250, 350);
    let mut dfco_payload = [0u8; DFCO_PAGE_BYTES];
    dfco.encode(&mut dfco_payload).expect("encode dfco");
    assert!(store.write_page(PAGE_DFCO, &dfco_payload).is_ok());
    let mut out_dfco = [0u8; DFCO_PAGE_BYTES];
    assert_eq!(
        store.read_page(PAGE_DFCO, &mut out_dfco),
        Some(DFCO_PAGE_BYTES)
    );
    assert_eq!(out_dfco, dfco_payload);

    let wue = WuePage::new(45, 5, -10, 70);
    let mut wue_payload = [0u8; WUE_PAGE_BYTES];
    wue.encode(&mut wue_payload).expect("encode wue");
    assert!(store.write_page(PAGE_WUE, &wue_payload).is_ok());
    let mut out_wue = [0u8; WUE_PAGE_BYTES];
    assert_eq!(
        store.read_page(PAGE_WUE, &mut out_wue),
        Some(WUE_PAGE_BYTES)
    );
    assert_eq!(out_wue, wue_payload);

    let ase = AsePage::new(30, 4_000, 1_500);
    let mut ase_payload = [0u8; ASE_PAGE_BYTES];
    ase.encode(&mut ase_payload).expect("encode ase");
    assert!(store.write_page(PAGE_ASE, &ase_payload).is_ok());
    let mut out_ase = [0u8; ASE_PAGE_BYTES];
    assert_eq!(
        store.read_page(PAGE_ASE, &mut out_ase),
        Some(ASE_PAGE_BYTES)
    );
    assert_eq!(out_ase, ase_payload);
}

#[test]
fn enrichment_pages_reject_wrong_size_and_invalid_values() {
    let mut state = EcuState::new();
    let mut store = state.page_store();

    assert!(matches!(
        store.write_page(PAGE_AE, &[0u8; AE_PAGE_BYTES - 1]),
        Err(PageError::WrongSize)
    ));
    let mut ae = [0u8; AE_PAGE_BYTES];
    ae[4] = 101;
    ae[6..10].copy_from_slice(&400u32.to_le_bytes());
    assert!(matches!(
        store.write_page(PAGE_AE, &ae),
        Err(PageError::Invalid)
    ));

    assert!(matches!(
        store.write_page(PAGE_DFCO, &[0u8; DFCO_PAGE_BYTES - 1]),
        Err(PageError::WrongSize)
    ));
    let mut dfco = [0u8; DFCO_PAGE_BYTES];
    dfco[0] = 2;
    dfco[4..6].copy_from_slice(&7000u16.to_le_bytes());
    dfco[6..8].copy_from_slice(&1500u16.to_le_bytes());
    assert!(matches!(
        store.write_page(PAGE_DFCO, &dfco),
        Err(PageError::Invalid)
    ));

    assert!(matches!(
        store.write_page(PAGE_WUE, &[0u8; WUE_PAGE_BYTES - 1]),
        Err(PageError::WrongSize)
    ));
    let mut wue = [0u8; WUE_PAGE_BYTES];
    wue[0] = 40;
    wue[2..4].copy_from_slice(&60i16.to_le_bytes());
    wue[4..6].copy_from_slice(&60i16.to_le_bytes());
    assert!(matches!(
        store.write_page(PAGE_WUE, &wue),
        Err(PageError::Invalid)
    ));

    assert!(matches!(
        store.write_page(PAGE_ASE, &[0u8; ASE_PAGE_BYTES - 1]),
        Err(PageError::WrongSize)
    ));
    let mut ase = [0u8; ASE_PAGE_BYTES];
    ase[0] = 20;
    assert!(matches!(
        store.write_page(PAGE_ASE, &ase),
        Err(PageError::Invalid)
    ));
}

#[test]
fn ve_tune_page_roundtrip_updates_fuel_scalars() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let mut payload = [0u8; 16];
    payload[0..2].copy_from_slice(&150u16.to_le_bytes()); // target afr x10
    payload[2..4].copy_from_slice(&10u16.to_le_bytes()); // kp
    payload[4..6].copy_from_slice(&5u16.to_le_bytes()); // ki
    payload[6..8].copy_from_slice(&2500u16.to_le_bytes()); // required fuel us
    payload[8..10].copy_from_slice(&900u16.to_le_bytes()); // deadtime us
    payload[10] = 1; // load source TPS
    payload[12..14].copy_from_slice(&fuel_consts::MIN_PULSE_WIDTH_US.to_le_bytes());
    payload[14..16].copy_from_slice(&fuel_consts::MAX_PULSE_WIDTH_US.to_le_bytes());
    assert!(store.write_page(PAGE_VE_TUNE, &payload).is_ok());

    let mut out = [0u8; 16];
    assert_eq!(store.read_page(PAGE_VE_TUNE, &mut out), Some(16));
    assert_eq!(u16::from_le_bytes([out[0], out[1]]), 150);
    assert_eq!(u16::from_le_bytes([out[2], out[3]]), 10);
    assert_eq!(u16::from_le_bytes([out[4], out[5]]), 5);
    assert_eq!(u16::from_le_bytes([out[6], out[7]]), 2500);
    assert_eq!(u16::from_le_bytes([out[8], out[9]]), 900);
    assert_eq!(out[10], 1);
}

#[test]
fn ve_tune_rejects_invalid_load_source() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let mut payload = [0u8; 16];
    payload[0..2].copy_from_slice(&150u16.to_le_bytes());
    payload[2..4].copy_from_slice(&10u16.to_le_bytes());
    payload[4..6].copy_from_slice(&5u16.to_le_bytes());
    payload[6..8].copy_from_slice(&2200u16.to_le_bytes());
    payload[8..10].copy_from_slice(&800u16.to_le_bytes());
    payload[10] = 2; // invalid (only 0=MAP,1=TPS)
    payload[12..14].copy_from_slice(&fuel_consts::MIN_PULSE_WIDTH_US.to_le_bytes());
    payload[14..16].copy_from_slice(&fuel_consts::MAX_PULSE_WIDTH_US.to_le_bytes());
    assert!(store.write_page(PAGE_VE_TUNE, &payload).is_err());
}

#[test]
fn ve_tune_rejects_invalid_target_afr() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let mut payload = [0u8; 16];
    payload[0..2].copy_from_slice(&99u16.to_le_bytes());
    payload[2..4].copy_from_slice(&10u16.to_le_bytes());
    payload[4..6].copy_from_slice(&5u16.to_le_bytes());
    payload[6..8].copy_from_slice(&2200u16.to_le_bytes());
    payload[8..10].copy_from_slice(&800u16.to_le_bytes());
    payload[10] = 0;

    assert!(matches!(
        store.write_page(PAGE_VE_TUNE, &payload),
        Err(PageError::Invalid)
    ));

    payload[0..2].copy_from_slice(&221u16.to_le_bytes());
    assert!(matches!(
        store.write_page(PAGE_VE_TUNE, &payload),
        Err(PageError::Invalid)
    ));
}

#[test]
fn ve_tune_clamps_required_fuel_and_caps_deadtime() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let mut payload = [0u8; 16];
    payload[0..2].copy_from_slice(&150u16.to_le_bytes());
    payload[2..4].copy_from_slice(&10u16.to_le_bytes());
    payload[4..6].copy_from_slice(&5u16.to_le_bytes());
    payload[6..8].copy_from_slice(&0u16.to_le_bytes());
    payload[8..10].copy_from_slice(&12_000u16.to_le_bytes());
    payload[10] = 0;

    assert!(store.write_page(PAGE_VE_TUNE, &payload).is_ok());
    assert_eq!(*store.required_fuel_us, fuel_consts::MIN_PULSE_WIDTH_US);
    assert_eq!(*store.injector_deadtime_us, 10_000);

    payload[6..8].copy_from_slice(&(u16::MAX).to_le_bytes());
    payload[8..10].copy_from_slice(&900u16.to_le_bytes());
    assert!(store.write_page(PAGE_VE_TUNE, &payload).is_ok());
    assert_eq!(*store.required_fuel_us, fuel_consts::MAX_PULSE_WIDTH_US);
    assert_eq!(*store.injector_deadtime_us, 900);
}

#[test]
fn ve_tune_write_accepts_extra_bytes() {
    let mut state = EcuState::new();
    let mut store = state.page_store();
    let mut payload = [0u8; 20];
    payload[0..2].copy_from_slice(&150u16.to_le_bytes());
    payload[2..4].copy_from_slice(&10u16.to_le_bytes());
    payload[4..6].copy_from_slice(&5u16.to_le_bytes());
    payload[6..8].copy_from_slice(&2500u16.to_le_bytes());
    payload[8..10].copy_from_slice(&900u16.to_le_bytes());
    payload[10] = 1;
    payload[16..20].copy_from_slice(&[0xAA, 0xBB, 0xCC, 0xDD]);

    assert!(store.write_page(PAGE_VE_TUNE, &payload).is_ok());
    assert_eq!(store.cl.target_afr_x10, 150);
    assert_eq!(*store.required_fuel_us, 2500);
    assert_eq!(*store.ve_load_source, 1);
}
