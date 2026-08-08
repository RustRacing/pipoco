use super::*;
use std::collections::BTreeSet;

#[test]
fn descriptor_registry_covers_all_known_pages() {
    const EXPECTED: [TsPageDescriptor; TS_PAGE_COUNT] = [
        TsPageDescriptor {
            page: PAGE_FUEL,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "fuel",
            persisted_setup_page: true,
            codec_family: PageCodecFamily::FuelTable,
        },
        TsPageDescriptor {
            page: PAGE_IGN,
            len: TABLE_PAGE_BYTES,
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
            len: DIAG_PAGE_BYTES,
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
            len: IDLE_PAGE_BYTES,
            writable: true,
            label: "idle",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Idle,
        },
        TsPageDescriptor {
            page: PAGE_FAN,
            len: FAN_PAGE_BYTES,
            writable: true,
            label: "fan",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::Fan,
        },
        TsPageDescriptor {
            page: PAGE_CL,
            len: CLOSED_LOOP_PAGE_BYTES,
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
            len: EXPERT_TRIGGER_PAGE_BYTES,
            writable: true,
            label: "expert_trigger",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::ExpertTrigger,
        },
        TsPageDescriptor {
            page: PAGE_VE_TUNE,
            len: VE_TUNE_PAGE_BYTES,
            writable: true,
            label: "ve_tune",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::VeTune,
        },
        TsPageDescriptor {
            page: PAGE_VE_TABLE,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "ve_table",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::VeTable,
        },
        TsPageDescriptor {
            page: PAGE_AFR_TABLE,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "afr_table",
            persisted_setup_page: false,
            codec_family: PageCodecFamily::AfrTable,
        },
    ];

    assert_eq!(TS_PAGE_DESCRIPTORS, EXPECTED);
    for descriptor in TS_PAGE_DESCRIPTORS {
        assert_eq!(ts_page_descriptor(descriptor.page), Some(descriptor));
    }
    assert!(ts_page_descriptor(99).is_none());
}

#[test]
fn descriptor_registry_page_ids_are_unique() {
    let unique: BTreeSet<u8> = TS_PAGE_DESCRIPTORS
        .iter()
        .map(|descriptor| descriptor.page)
        .collect();
    assert_eq!(unique.len(), TS_PAGE_DESCRIPTORS.len());
}

#[test]
fn descriptor_registry_schema_versions_cover_all_known_pages() {
    for descriptor in TS_PAGE_DESCRIPTORS {
        assert_eq!(ts_page_schema_version(descriptor.page), Some(1));
    }
    assert!(ts_page_schema_version(99).is_none());
}

#[test]
fn diag_log_page_roundtrips_meaning_fields() {
    let page = DiagLogPage::new([
        DiagLogEntryPage::new(
            8,
            TS_FAULT_SEVERITY_WARNING,
            TS_FAULT_ACTION_LIMP_HOME,
            TS_DIAG_SOURCE_SAFETY,
            true,
            0x0102_0304,
            10,
            20,
        ),
        DiagLogEntryPage::new(
            9,
            TS_FAULT_SEVERITY_CRITICAL,
            TS_FAULT_ACTION_SHUTDOWN,
            TS_DIAG_SOURCE_USER,
            false,
            0,
            30,
            40,
        ),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
    ]);
    let mut out = [0u8; DIAG_LOG_PAGE_BYTES];
    assert_eq!(page.encode(&mut out), Ok(DIAG_LOG_PAGE_BYTES));
    assert_eq!(out[0], 8);
    assert_eq!(out[1], TS_FAULT_SEVERITY_WARNING);
    assert_eq!(out[2], TS_FAULT_ACTION_LIMP_HOME);
    assert_eq!(
        out[3],
        TS_DIAG_SOURCE_SAFETY | TS_DIAG_SOURCE_CONTEXT_PRESENT
    );
    assert_eq!(&out[12..16], &0x0102_0304u32.to_le_bytes());
    assert_eq!(DiagLogPage::decode(&out).expect("decode diag log"), page);
}

#[test]
fn diag_log_entry_timestamps_use_documented_byte_offsets() {
    // IPW-ECU.ini [DiagLog]: entryStartUs_offset = 4, entryEndUs_offset = 8, entryStride = 16.
    let page = DiagLogPage::new([
        DiagLogEntryPage::new(
            8,
            TS_FAULT_SEVERITY_WARNING,
            TS_FAULT_ACTION_LIMP_HOME,
            TS_DIAG_SOURCE_SAFETY,
            true,
            0x0102_0304,
            0x1112_1314,
            0x2122_2324,
        ),
        DiagLogEntryPage::new(
            9,
            TS_FAULT_SEVERITY_CRITICAL,
            TS_FAULT_ACTION_SHUTDOWN,
            TS_DIAG_SOURCE_USER,
            false,
            0x0506_0708,
            0x3132_3334,
            0x4142_4344,
        ),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
        DiagLogEntryPage::empty(),
    ]);
    let mut out = [0u8; DIAG_LOG_PAGE_BYTES];
    assert_eq!(page.encode(&mut out), Ok(DIAG_LOG_PAGE_BYTES));

    assert_eq!(DIAG_LOG_ENTRY_BYTES, 16);
    assert_eq!(&out[4..8], &0x1112_1314u32.to_le_bytes(), "entry0 start_us");
    assert_eq!(&out[8..12], &0x2122_2324u32.to_le_bytes(), "entry0 end_us");
    assert_eq!(
        &out[12..16],
        &0x0102_0304u32.to_le_bytes(),
        "entry0 context"
    );
    assert_eq!(
        &out[20..24],
        &0x3132_3334u32.to_le_bytes(),
        "entry1 start_us"
    );
    assert_eq!(&out[24..28], &0x4142_4344u32.to_le_bytes(), "entry1 end_us");
    assert_eq!(
        &out[28..32],
        &0x0506_0708u32.to_le_bytes(),
        "entry1 context"
    );

    // Decode must read the same absolute offsets, not merely mirror encode.
    let decoded = DiagLogPage::decode(&out).expect("decode diag log");
    assert_eq!(decoded.entries[0].start_us, 0x1112_1314);
    assert_eq!(decoded.entries[0].end_us, 0x2122_2324);
    assert_eq!(decoded.entries[1].start_us, 0x3132_3334);
    assert_eq!(decoded.entries[1].end_us, 0x4142_4344);

    let mut wire = [0u8; DIAG_LOG_PAGE_BYTES];
    wire[0] = 8;
    wire[1] = TS_FAULT_SEVERITY_WARNING;
    wire[2] = TS_FAULT_ACTION_LIMP_HOME;
    wire[3] = TS_DIAG_SOURCE_SAFETY;
    wire[4..8].copy_from_slice(&0xDEAD_BEEFu32.to_le_bytes());
    wire[8..12].copy_from_slice(&0x0BAD_F00Du32.to_le_bytes());
    let from_wire = DiagLogPage::decode(&wire).expect("decode raw wire");
    assert_eq!(from_wire.entries[0].start_us, 0xDEAD_BEEF);
    assert_eq!(from_wire.entries[0].end_us, 0x0BAD_F00D);
}

#[test]
fn limits_page_uses_explicit_little_endian_byte_offsets() {
    // Limits page wire layout (IPW-ECU.ini [Limits], size = 10):
    //   0..2 map_min_kpa_x10, 2..4 map_max_kpa_x10, 4 tps_min_percent,
    //   5 tps_max_percent, 6..8 clear_time_s, 8 trigger bits, 9 reserved.
    let page = LimitsPage::new(0x0102, 0x0304, 5, 60, 0x0708, true, false);
    let mut out = [0u8; LIMITS_PAGE_BYTES];
    assert_eq!(page.encode(&mut out), Ok(LIMITS_PAGE_BYTES));

    assert_eq!(&out[0..2], &0x0102u16.to_le_bytes(), "map_min_kpa_x10");
    assert_eq!(&out[2..4], &0x0304u16.to_le_bytes(), "map_max_kpa_x10");
    assert_eq!(out[4], 5, "tps_min_percent");
    assert_eq!(out[5], 60, "tps_max_percent");
    assert_eq!(&out[6..8], &0x0708u16.to_le_bytes(), "clear_time_s");
    assert_eq!(out[8], 0b01, "trigger bits: MAP set, TPS clear");
    assert_eq!(out[9], 0, "reserved");

    let page_tps = LimitsPage::new(0x0102, 0x0304, 5, 60, 0x0708, false, true);
    let mut out_tps = [0u8; LIMITS_PAGE_BYTES];
    assert_eq!(page_tps.encode(&mut out_tps), Ok(LIMITS_PAGE_BYTES));
    assert_eq!(out_tps[8], 0b10, "trigger bits: TPS set, MAP clear");

    // Decode must read the same absolute offsets independently of encode.
    let wire = [0x02u8, 0x01, 0x04, 0x03, 5, 60, 0x08, 0x07, 0b11, 0];
    let decoded = LimitsPage::decode(&wire).expect("decode limits");
    assert_eq!(decoded.map_min_kpa_x10, 0x0102);
    assert_eq!(decoded.map_max_kpa_x10, 0x0304);
    assert_eq!(decoded.tps_min_percent, 5);
    assert_eq!(decoded.tps_max_percent, 60);
    assert_eq!(decoded.clear_time_s, 0x0708);
    assert!(decoded.emerg_trig_map);
    assert!(decoded.emerg_trig_tps);
}
