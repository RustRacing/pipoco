use ecu_calibration::FuelRuntimeTune;
use ecu_target_common::ts::page_store::{
    build_ecu_page_store, build_system_snapshot, diag_log_entries_from, PageRuntime, SnapshotInputs,
};
use ecu_target_common::ts::state_ptr::StatePtr;
use ecu_ts::pages::EcuPageStore;
use ecu_ts::persistence::PageStoreProvider;

use crate::ts_runtime::BoardEcuState;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Rp2040TsOutpcConfig {
    pub(crate) target_afr_x10: u16,
    pub(crate) cl_kp_i: u16,
    pub(crate) cl_ki_i: u16,
    pub(crate) ego_sensor: u8,
    pub(crate) emergency_mode: bool,
}

impl Rp2040TsOutpcConfig {
    pub(crate) const fn new() -> Self {
        Self {
            target_afr_x10: 147,
            cl_kp_i: 0,
            cl_ki_i: 0,
            ego_sensor: 1,
            emergency_mode: false,
        }
    }

    pub(crate) fn from_state(state: &BoardEcuState) -> Self {
        let cl_config = state.config.cl_config;
        Self {
            target_afr_x10: cl_config.target_afr_x10,
            cl_kp_i: cl_config.kp_i,
            cl_ki_i: cl_config.ki_i,
            ego_sensor: match state.o2_sensor {
                ecu_calibration::O2SensorType::Narrowband => 1,
                ecu_calibration::O2SensorType::Wideband => 2,
            },
            emergency_mode: state.emergency_mode(),
        }
    }
}

pub(crate) fn refresh_ts_outpc_config_from_state(state: &BoardEcuState) -> Rp2040TsOutpcConfig {
    Rp2040TsOutpcConfig::from_state(state)
}

pub(crate) struct EcuStatePageStoreProvider {
    state: StatePtr<BoardEcuState>,
}

impl EcuStatePageStoreProvider {
    pub(crate) fn new(state: &mut BoardEcuState) -> Self {
        Self {
            state: StatePtr::new(state),
        }
    }
}

fn board_page_store(state: &mut BoardEcuState) -> EcuPageStore<'_> {
    let diag_log_entries = diag_log_entries_from(&state.diag_log);
    state.snapshot = build_system_snapshot(SnapshotInputs {
        rpm: 0,
        synced: false,
        base_pw_us: 0,
        enrich_mult_x100: 100,
        stft_x10: 0,
        fuel_mult_x100: 100,
        final_pw: ecu_domain::Micros::new(0),
        last_fault_code: 0,
        isr_count: 0,
        isr_max_us: 0,
        isr_avg_us: 0,
    });
    let BoardEcuState {
        config,
        snapshot,
        tooth_count,
        sync_loss_counter,
        emerg_trig_map,
        emerg_trig_tps,
        expert_trigger,
        ..
    } = state;
    build_ecu_page_store(
        config,
        PageRuntime {
            snapshot,
            tooth_count,
            sync_loss_counter: *sync_loss_counter,
            emerg_trig_map,
            emerg_trig_tps,
            diag_log_entries,
            expert_trigger,
        },
    )
}

impl PageStoreProvider for EcuStatePageStoreProvider {
    type Pages<'a> = EcuPageStore<'a>;

    fn with_pages_mut<R>(&mut self, f: impl FnOnce(&mut Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let mut pages = board_page_store(state);
                f(&mut pages)
            })
        }
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let pages = board_page_store(state);
                f(&pages)
            })
        }
    }

    fn runtime_fuel_tune(&self) -> FuelRuntimeTune {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with(|state| {
                FuelRuntimeTune::new(
                    state.config.ve_table,
                    state.config.afr_table,
                    state.config.required_fuel_us,
                    state.config.injector_deadtime_us,
                    state.config.ve_load_source,
                )
            })
        }
    }
}
