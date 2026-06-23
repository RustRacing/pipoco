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
