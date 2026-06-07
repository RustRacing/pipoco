use super::*;

#[test]
fn descriptor_registry_covers_all_known_pages() {
    const EXPECTED: [TsPageDescriptor; TS_PAGE_COUNT] = [
        TsPageDescriptor {
            page: PAGE_FUEL,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "fuel",
        },
        TsPageDescriptor {
            page: PAGE_IGN,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "ign",
        },
        TsPageDescriptor {
            page: PAGE_SENSORS,
            len: SENSORS_PAGE_BYTES,
            writable: true,
            label: "sensors",
        },
        TsPageDescriptor {
            page: PAGE_AE,
            len: AE_PAGE_BYTES,
            writable: true,
            label: "ae",
        },
        TsPageDescriptor {
            page: PAGE_DFCO,
            len: DFCO_PAGE_BYTES,
            writable: true,
            label: "dfco",
        },
        TsPageDescriptor {
            page: PAGE_LIMITS,
            len: LIMITS_PAGE_BYTES,
            writable: true,
            label: "limits",
        },
        TsPageDescriptor {
            page: PAGE_DIAG,
            len: DIAG_PAGE_BYTES,
            writable: false,
            label: "diag",
        },
        TsPageDescriptor {
            page: PAGE_DIAG_LOG,
            len: DIAG_LOG_PAGE_BYTES,
            writable: false,
            label: "diag_log",
        },
        TsPageDescriptor {
            page: PAGE_ANGLES,
            len: ANGLES_PAGE_BYTES,
            writable: true,
            label: "angles",
        },
        TsPageDescriptor {
            page: PAGE_WUE,
            len: WUE_PAGE_BYTES,
            writable: true,
            label: "wue",
        },
        TsPageDescriptor {
            page: PAGE_ASE,
            len: ASE_PAGE_BYTES,
            writable: true,
            label: "ase",
        },
        TsPageDescriptor {
            page: PAGE_IDLE,
            len: IDLE_PAGE_BYTES,
            writable: true,
            label: "idle",
        },
        TsPageDescriptor {
            page: PAGE_FAN,
            len: FAN_PAGE_BYTES,
            writable: true,
            label: "fan",
        },
        TsPageDescriptor {
            page: PAGE_CL,
            len: CLOSED_LOOP_PAGE_BYTES,
            writable: true,
            label: "cl",
        },
        TsPageDescriptor {
            page: PAGE_SNAPSHOT,
            len: SNAPSHOT_PAGE_BYTES,
            writable: false,
            label: "snapshot",
        },
        TsPageDescriptor {
            page: PAGE_EXPERT_TRIGGER,
            len: EXPERT_TRIGGER_PAGE_BYTES,
            writable: true,
            label: "expert_trigger",
        },
        TsPageDescriptor {
            page: PAGE_VE_TUNE,
            len: VE_TUNE_PAGE_BYTES,
            writable: true,
            label: "ve_tune",
        },
        TsPageDescriptor {
            page: PAGE_VE_TABLE,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "ve_table",
        },
        TsPageDescriptor {
            page: PAGE_AFR_TABLE,
            len: TABLE_PAGE_BYTES,
            writable: true,
            label: "afr_table",
        },
    ];

    assert_eq!(TS_PAGE_DESCRIPTORS, EXPECTED);
    for descriptor in TS_PAGE_DESCRIPTORS {
        assert_eq!(ts_page_descriptor(descriptor.page), Some(descriptor));
    }
    assert!(ts_page_descriptor(99).is_none());
}
