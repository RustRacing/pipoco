use ecu_core::ts::pages::{EcuStatePageStore, PersistedPageStore};
use ecu_core::EcuState;
use ecu_target_common::kv::ram::RamKv512;

#[test]
fn factory_reset_restores_defaults() {
    let mut state = EcuState::new();

    // Mutate a couple of cells to non-defaults
    state.ipw_table[0][0] = 1234;
    state.ignition_table[15][15] = -7;

    // Empty KV
    let kv = RamKv512::new();

    // 1) try_load should not overwrite when KV empty
    {
        let pages = EcuStatePageStore { fuel: &mut state.ipw_table, ign: &mut state.ignition_table };
        let mut store = PersistedPageStore::new(pages, kv);
        store.try_load();
        // drop before reading state
    }
    assert_eq!(state.ipw_table[0][0], 1234);
    assert_eq!(state.ignition_table[15][15], -7);

    // 2) factory_reset sets defaults
    {
        let pages = EcuStatePageStore { fuel: &mut state.ipw_table, ign: &mut state.ignition_table };
        let mut store = PersistedPageStore::new(pages, RamKv512::new());
        store.factory_reset();
    }
    use ecu_core::constants::{fuel as fuel_consts, ignition as ign_consts};
    assert_eq!(state.ipw_table[0][0], fuel_consts::DEFAULT_PULSE_WIDTH_US);
    assert_eq!(state.ipw_table[7][8], fuel_consts::DEFAULT_PULSE_WIDTH_US);
    assert_eq!(state.ignition_table[0][0], ign_consts::DEFAULT_TIMING_BTDC);
    assert_eq!(state.ignition_table[15][15], ign_consts::DEFAULT_TIMING_BTDC);
}
