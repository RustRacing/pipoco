#![no_std]

pub mod ts {
    pub mod service;
    pub mod store;
    #[cfg(feature = "ts-usb")]
    pub mod usb_cdc;
}

pub mod kv {
    pub mod layout;
    pub mod ram;
}

pub mod adapter;
pub mod bringup;
pub mod capture;
pub mod control_inputs;
pub mod live_inputs;
pub mod noop;
pub mod outputs;
pub mod sensor_sample;
pub mod split_tick;
pub mod sensors {
    pub mod adc_pipeline;
}
pub mod trigger_adapter;

// ---------------------------------------------------------------------
// Board adapter contracts
// ---------------------------------------------------------------------

/// Adapter contracts for board adapter fields that require hardware for
/// full validation but have host-side test coverage.
///
/// These document the boundary between the abstract ECU domain types and
/// concrete board implementations (RP2040 Pico, RP2350B, STM32F4).
///
/// DO NOT add new variants without a corresponding test in
/// `tests/fm0016_board_adapter_contract.rs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardAdapterContract {
    /// RP2040 Pico uses Rpm, Degrees10, Kpa10, Micros from ecu_domain.
    Rp2040PicoUnits,
    /// RP2350B uses Rpm, Degrees10, Kpa10, Micros from ecu_domain.
    Rp2350BUnits,
    /// STM32F4 uses Rpm, Degrees10, Kpa10, Micros from ecu_domain.
    Stm32F4Units,
    /// RP2040 Pico event ordering: TriggerEdge → CamEdge → SensorFrame → Tick.
    Rp2040PicoEventOrdering,
    /// RP2350B event ordering: TriggerEdge → CamEdge → SensorFrame → Tick.
    Rp2350BEventOrdering,
    /// STM32F4 event ordering: TriggerEdge → CamEdge → SensorFrame → Tick.
    Stm32F4EventOrdering,
    /// Persistence page IDs: PAGE_FUEL=1, PAGE_IGN=2, PAGE_SENSORS=3, etc.
    PageIdRouting,
    /// Page sizes: fuel/ign pages are 512 bytes, angles page is 68 bytes.
    PageSizeRouting,
    /// Burn serializes fuel/ign/angles pages to KV store atomically.
    BurnWritesFuelIgnAngles,
    /// Save writes a page and waits for next burn to persist.
    SaveDeferredToBurn,
    /// Try load reads fuel/ign/angles from KV into memory; empty KV is no-op.
    TryLoadReadsFuelIgnAnglesOrNoop,
    /// TS page routing: burn/save route fuel→b"fuel", ign→b"ign", angles→b"angles".
    TsPageRoutingBurnSave,
}

/// Field-by-field conformance status for board adapters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardAdapterConformanceStatus {
    /// Observed from real product execution path.
    Covered,
    /// Requires typed adapter contract due to hardware dependency.
    AdapterContract(BoardAdapterContract),
}

/// Adapter contracts for target-common persistence semantics that are owned by
/// board/host adapters rather than the frozen oracle.
///
/// DO NOT add new variants without a corresponding test in
/// `tests/fm0016_board_adapter_contract.rs`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetCommonAdapterContract {
    /// Persistence encode/decode ownership is implemented by target-common page stores.
    PersistCodecOwnership,
    /// Persistence migration ownership remains adapter-owned until product migration exists.
    PersistMigrationOwnership,
    /// Factory reset is covered by host stores; hardware commit remains target-owned.
    FactoryResetCommitOwnership,
}

/// Field-by-field conformance status for target-common adapter contracts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetCommonAdapterConformanceStatus {
    /// Observed from real product execution path.
    Covered,
    /// Requires typed target-common adapter contract.
    AdapterContract(TargetCommonAdapterContract),
}
