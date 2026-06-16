#![no_std]

pub use ecu_spec::{
    factory_reset as spec_factory_reset, persist_decode as spec_persist_decode,
    persist_encode as spec_persist_encode, persist_migrate as spec_persist_migrate,
    EncodedPersistRecord, PersistDecodeError, PersistEncodeError, PersistMigrationError,
    PersistPage, PersistPageId, PERSIST_ANGLES_PAGE_BYTES, PERSIST_FUEL_PAGE_BYTES,
    PERSIST_IGNITION_PAGE_BYTES, PERSIST_MAX_PAYLOAD_BYTES, PERSIST_RECORD_MAX_BYTES,
    PERSIST_SCHEMA_VERSION_CURRENT,
};

/// Typed adapter-contract ownership for target-common persistence boundaries that
/// are exercised in host tests but still require adapter evidence for full
/// end-to-end equivalence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TargetCommonAdapterContract {
    PersistCodecOwnership,
    PersistMigrationOwnership,
    FactoryResetCommitOwnership,
}

pub fn persist_encode(page: &PersistPage) -> Result<EncodedPersistRecord, PersistEncodeError> {
    spec_persist_encode(page)
}

pub fn persist_decode(record: &[u8]) -> Result<PersistPage, PersistDecodeError> {
    spec_persist_decode(record)
}

pub fn persist_migrate(
    page_id: PersistPageId,
    from_version: u16,
    to_version: u16,
    source_payload: &[u8],
) -> Result<PersistPage, PersistMigrationError> {
    spec_persist_migrate(page_id, from_version, to_version, source_payload)
}

pub fn factory_reset(page_bytes: &[u8]) -> Result<EncodedPersistRecord, PersistDecodeError> {
    spec_factory_reset(page_bytes)
}

pub mod ts {
    pub mod outpc;
    pub mod page_store;
    pub mod service;
    pub mod state_ptr;
    #[cfg(feature = "ts-usb")]
    pub mod usb_cdc;
}

pub mod kv {
    pub mod ab;
    pub mod layout;
    pub mod ram;
}

pub mod adapter;
pub mod bringup;
pub mod capture;
pub mod control_inputs;
pub mod live_inputs;
#[cfg(any(test, feature = "test-support"))]
pub mod noop;
pub mod outputs;
pub mod sensor_sample;
pub mod split_tick;
pub mod sensors {
    pub mod adc_pipeline;
    pub mod map;
    pub mod vss;
}
pub mod trigger_adapter;
