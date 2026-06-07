pub const TABLE_AXIS_LEN: usize = 16;
pub const TABLE_CELL_COUNT: usize = TABLE_AXIS_LEN * TABLE_AXIS_LEN;
pub const TABLE_PAGE_BYTES: usize = TABLE_CELL_COUNT * 2;
pub const AE_PAGE_BYTES: usize = 16;
pub const DFCO_PAGE_BYTES: usize = 16;
pub const WUE_PAGE_BYTES: usize = 8;
pub const ASE_PAGE_BYTES: usize = 8;
pub const IDLE_PAGE_BYTES: usize = 6;
pub const FAN_PAGE_BYTES: usize = 6;
pub const CLOSED_LOOP_PAGE_BYTES: usize = 8;
pub const ANGLES_PAGE_BYTES: usize = 68;
pub const SENSORS_PAGE_BYTES: usize = 128;
pub const LIMITS_PAGE_BYTES: usize = 10;
pub const VE_TUNE_PAGE_BYTES: usize = 16;
pub const SNAPSHOT_PAGE_BYTES: usize = 32;
pub const EXPERT_TRIGGER_PAGE_BYTES: usize = 48;
pub const DIAG_PAGE_BYTES: usize = 32;
pub const DIAG_LOG_ENTRY_BYTES: usize = 9;
pub const DIAG_LOG_ENTRY_COUNT: usize = 16;
pub const DIAG_LOG_PAGE_BYTES: usize = DIAG_LOG_ENTRY_BYTES * DIAG_LOG_ENTRY_COUNT;

/// Page numbers used by the generated TunerStudio metadata.
pub const PAGE_FUEL: u8 = 1;
pub const PAGE_IGN: u8 = 2;
pub const PAGE_SENSORS: u8 = 3;
pub const PAGE_AE: u8 = 4;
pub const PAGE_DFCO: u8 = 5;
pub const PAGE_LIMITS: u8 = 6;
pub const PAGE_DIAG: u8 = 7;
pub const PAGE_DIAG_LOG: u8 = 8;
pub const PAGE_ANGLES: u8 = 9;
pub const PAGE_WUE: u8 = 10;
pub const PAGE_ASE: u8 = 11;
pub const PAGE_IDLE: u8 = 12;
pub const PAGE_FAN: u8 = 13;
pub const PAGE_CL: u8 = 14;
pub const PAGE_SNAPSHOT: u8 = 15;
pub const PAGE_EXPERT_TRIGGER: u8 = 16;
pub const PAGE_VE_TUNE: u8 = 17;
pub const PAGE_VE_TABLE: u8 = 18;
pub const PAGE_AFR_TABLE: u8 = 19;

pub const TS_PAGE_COUNT: usize = 19;

/// Descriptor for a TunerStudio page.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct TsPageDescriptor {
    pub page: u8,
    pub len: usize,
    pub writable: bool,
    pub label: &'static str,
}

pub const TS_PAGE_DESCRIPTORS: [TsPageDescriptor; TS_PAGE_COUNT] = [
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

pub fn ts_page_descriptor(page: u8) -> Option<TsPageDescriptor> {
    TS_PAGE_DESCRIPTORS
        .iter()
        .copied()
        .find(|descriptor| descriptor.page == page)
}
