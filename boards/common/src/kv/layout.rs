/// Shared page sizes for TunerStudio-backed KV stores.
///
/// These values are used by board-local flash/KV backends and the shared
/// target-common RAM store so the page vocabulary stays aligned.
pub const FUEL_PAGE_LEN: usize = 512;
pub const IGN_PAGE_LEN: usize = 512;
pub const ANGLES_PAGE_LEN: usize = 68;
pub const EXPERT_TRIGGER_PAGE_LEN: usize = ecu_calibration::EXPERT_TRIGGER_RECORD_LEN;

/// Shared header size for the simple flash-backed page formats used by board
/// bring-up storage.
pub const PAGE_HEADER_LEN: usize = 64;
