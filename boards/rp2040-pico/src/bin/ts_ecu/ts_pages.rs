use ecu_calibration::FuelRuntimeTune;
use ecu_runtime::{EngineRuntime, RuntimeSnapshot};
use ecu_target_common::ts::page_store::{
    build_ecu_page_store, build_system_snapshot, diag_log_entries_from, PageRuntime, SnapshotInputs,
};
use ecu_target_common::ts::state_ptr::{StatePtr, StateRef};
use ecu_ts::pages::{
    ts_fault_code_from_diag_code, ts_fault_code_from_runtime_fault, ts_fault_severity_code,
    EcuPageStore, TS_CANCEL_REASON_NONE, TS_CURRENT_FAULT_NONE, TS_FAULT_ACTION_LIMP_HOME,
    TS_FAULT_ACTION_NONE, TS_FAULT_ACTION_OBSERVE_ONLY, TS_FAULT_ACTION_SHUTDOWN,
    TS_FAULT_FLAG_ACTIVE, TS_FAULT_FLAG_DIAG_LOG_PRESENT, TS_FAULT_FLAG_EMERGENCY_MODE,
    TS_FAULT_FLAG_SNAPSHOT_PRESENT,
};
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
    runtime: StateRef<EngineRuntime>,
}

impl EcuStatePageStoreProvider {
    pub(crate) fn new(state: &mut BoardEcuState, runtime: &EngineRuntime) -> Self {
        Self {
            state: StatePtr::new(state),
            runtime: StateRef::new(runtime),
        }
    }
}

fn latest_diag_code(state: &BoardEcuState) -> u8 {
    state
        .diag_log
        .events
        .iter()
        .rev()
        .find_map(|ev| ev.as_ref().map(|ev| ev.code.to_u8()))
        .unwrap_or(0)
}

fn runtime_fault_action(snapshot: &RuntimeSnapshot) -> u8 {
    if snapshot.faults.fault == ecu_domain::FaultCode::None {
        TS_FAULT_ACTION_NONE
    } else if snapshot.faults.cancel_reason == ecu_domain::CancelReason::SafetyShutdown
        || snapshot.faults.severity == ecu_domain::FaultSeverity::Critical
    {
        TS_FAULT_ACTION_SHUTDOWN
    } else if snapshot.faults.severity == ecu_domain::FaultSeverity::Warning
        || snapshot.faults.fault == ecu_domain::FaultCode::SensorOutOfRange
    {
        TS_FAULT_ACTION_LIMP_HOME
    } else {
        TS_FAULT_ACTION_OBSERVE_ONLY
    }
}

fn current_fault_surface(
    state: &BoardEcuState,
    snapshot: &RuntimeSnapshot,
) -> (u8, u8, u8, u8, u8) {
    let code = if state.diag_map.is_active() {
        ts_fault_code_from_diag_code(ecu_domain::diag::DiagCode::MapRange)
    } else if state.diag_tps.is_active() {
        ts_fault_code_from_diag_code(ecu_domain::diag::DiagCode::TpsRange)
    } else if snapshot.faults.fault != ecu_domain::FaultCode::None {
        ts_fault_code_from_runtime_fault(snapshot.faults.fault)
    } else {
        TS_CURRENT_FAULT_NONE
    };
    let severity = if code == TS_CURRENT_FAULT_NONE {
        0
    } else if state.diag_map.is_active() || state.diag_tps.is_active() {
        ecu_ts::pages::TS_FAULT_SEVERITY_WARNING
    } else {
        ts_fault_severity_code(snapshot.faults.severity)
    };
    let action = if code == TS_CURRENT_FAULT_NONE {
        TS_FAULT_ACTION_NONE
    } else if state.diag_map.is_active() || state.diag_tps.is_active() {
        if state.emergency_mode() {
            TS_FAULT_ACTION_LIMP_HOME
        } else {
            TS_FAULT_ACTION_OBSERVE_ONLY
        }
    } else {
        runtime_fault_action(snapshot)
    };
    let cancel_reason = if snapshot.faults.fault == ecu_domain::FaultCode::None {
        TS_CANCEL_REASON_NONE
    } else {
        ecu_ts::pages::ts_cancel_reason_code(snapshot.faults.cancel_reason)
    };
    let mut flags = 0u8;
    if code != TS_CURRENT_FAULT_NONE {
        flags |= TS_FAULT_FLAG_ACTIVE | TS_FAULT_FLAG_SNAPSHOT_PRESENT;
    }
    if state.emergency_mode() {
        flags |= TS_FAULT_FLAG_EMERGENCY_MODE;
    }
    if latest_diag_code(state) != 0 {
        flags |= TS_FAULT_FLAG_DIAG_LOG_PRESENT;
    }
    (code, severity, action, cancel_reason, flags)
}

fn board_page_store(
    state: &mut BoardEcuState,
    runtime_snapshot: RuntimeSnapshot,
) -> EcuPageStore<'_> {
    let diag_log_entries = diag_log_entries_from(&state.diag_log);
    let latest_diag_code = latest_diag_code(state);
    let (
        current_fault_code,
        current_fault_severity,
        current_fault_action,
        current_cancel_reason,
        fault_flags,
    ) = current_fault_surface(state, &runtime_snapshot);
    state.snapshot = build_system_snapshot(SnapshotInputs {
        rpm: runtime_snapshot.engine.rpm.get(),
        sync: runtime_snapshot.engine.sync,
        base_pw_us: runtime_snapshot.control.fuel_pulse_width.get(),
        enrich_mult_x100: 100,
        stft_x10: 0,
        fuel_mult_x100: 100,
        final_pw: ecu_domain::Micros::new(runtime_snapshot.control.fuel_pulse_width.get()),
        last_fault_code: current_fault_code,
        fault_severity: current_fault_severity,
        cancel_reason: current_cancel_reason,
        isr_count: state.snapshot.isr_count,
        isr_max_us: state.snapshot.isr_max_us,
        isr_avg_us: state.snapshot.isr_avg_us,
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
            tooth_count: *tooth_count,
            sync_loss_counter: *sync_loss_counter,
            current_fault_code,
            current_fault_severity,
            current_fault_action,
            current_cancel_reason,
            fault_flags,
            latest_diag_code,
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
                let runtime_snapshot = self.runtime.with(|runtime| runtime.snapshot());
                let mut pages = board_page_store(state, runtime_snapshot);
                f(&mut pages)
            })
        }
    }

    fn with_pages<R>(&self, f: impl FnOnce(&Self::Pages<'_>) -> R) -> R {
        // SAFETY: TS provider callbacks run transiently from the main loop while the
        // engine state is not otherwise borrowed.
        unsafe {
            self.state.with_mut(|state| {
                let runtime_snapshot = self.runtime.with(|runtime| runtime.snapshot());
                let pages = board_page_store(state, runtime_snapshot);
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

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::diag::{DiagCode, DiagEvent, DiagSource};
    use ecu_domain::{
        CancelReason, FaultCode, FaultSeverity, Micros, PulseWidthUs, Rpm, SyncState,
    };
    use ecu_runtime::RuntimeSnapshot;
    use ecu_ts::pages::{
        DIAG_LOG_ENTRY_BYTES, PAGE_DIAG, PAGE_DIAG_LOG, PAGE_SNAPSHOT,
        TS_CURRENT_FAULT_SENSOR_OUT_OF_RANGE, TS_DIAG_BYTES, TS_DIAG_SOURCE_SAFETY,
        TS_FAULT_ACTION_LIMP_HOME, TS_FAULT_FLAG_ACTIVE, TS_FAULT_FLAG_SNAPSHOT_PRESENT,
        TS_FAULT_SEVERITY_WARNING,
    };
    use ecu_ts::server::PageStore;

    #[test]
    fn board_page_store_projects_runtime_fault_surface_into_diag_and_snapshot_pages() {
        let mut state = BoardEcuState::new();
        let mut runtime_snapshot = RuntimeSnapshot::default();
        runtime_snapshot.engine.rpm = Rpm::new(2_750);
        runtime_snapshot.engine.sync = SyncState::Locked { cam_ref: true };
        runtime_snapshot.control.fuel_pulse_width = PulseWidthUs::new(3_200);
        runtime_snapshot.faults.fault = FaultCode::SensorOutOfRange;
        runtime_snapshot.faults.severity = FaultSeverity::Warning;
        runtime_snapshot.faults.cancel_reason = CancelReason::Manual;

        let store = board_page_store(&mut state, runtime_snapshot);

        let mut diag = [0u8; TS_DIAG_BYTES];
        assert_eq!(store.read_page(PAGE_DIAG, &mut diag), Some(TS_DIAG_BYTES));
        assert_eq!(diag[8..10], 2_750u16.to_le_bytes());
        assert_eq!(diag[24], TS_CURRENT_FAULT_SENSOR_OUT_OF_RANGE);
        assert_eq!(diag[25], TS_FAULT_SEVERITY_WARNING);
        assert_eq!(diag[26], TS_FAULT_ACTION_LIMP_HOME);
        assert_eq!(diag[27], TS_CANCEL_REASON_NONE);
        assert_eq!(
            diag[28],
            TS_FAULT_FLAG_ACTIVE | TS_FAULT_FLAG_SNAPSHOT_PRESENT
        );
        assert_eq!(diag[29], 0);

        let mut snapshot = [0u8; 32];
        assert_eq!(store.read_page(PAGE_SNAPSHOT, &mut snapshot), Some(32));
        assert_eq!(snapshot[2], 2);
        assert_eq!(snapshot[3], TS_CANCEL_REASON_NONE);
        assert_eq!(u16::from_le_bytes([snapshot[0], snapshot[1]]), 2_750);
        assert_eq!(
            u32::from_le_bytes([snapshot[14], snapshot[15], snapshot[16], snapshot[17]]),
            3_200
        );
        assert_eq!(snapshot[18], TS_CURRENT_FAULT_SENSOR_OUT_OF_RANGE);
        assert_eq!(snapshot[19], TS_FAULT_SEVERITY_WARNING);
    }

    #[test]
    fn board_page_store_projects_diag_log_meaning_surface() {
        let mut state = BoardEcuState::new();
        state.diag_log.push(DiagEvent {
            code: DiagCode::KnockDetected,
            timestamp: Micros::new(100),
            source: DiagSource::Safety,
            context: None,
            start_us: 100,
            end_us: 200,
        });

        let store = board_page_store(&mut state, RuntimeSnapshot::default());

        let mut diag_log = [0u8; DIAG_LOG_ENTRY_BYTES * 16];
        assert_eq!(
            store.read_page(PAGE_DIAG_LOG, &mut diag_log),
            Some(DIAG_LOG_ENTRY_BYTES * 16)
        );
        assert_eq!(diag_log[0], DiagCode::KnockDetected.to_u8());
        assert_eq!(diag_log[1], TS_FAULT_SEVERITY_WARNING);
        assert_eq!(diag_log[2], TS_FAULT_ACTION_LIMP_HOME);
        assert_eq!(diag_log[3], TS_DIAG_SOURCE_SAFETY);
        assert_eq!(
            u32::from_le_bytes([diag_log[4], diag_log[5], diag_log[6], diag_log[7]]),
            100
        );
        assert_eq!(
            u32::from_le_bytes([diag_log[8], diag_log[9], diag_log[10], diag_log[11]]),
            200
        );
        assert_eq!(
            u32::from_le_bytes([diag_log[12], diag_log[13], diag_log[14], diag_log[15]]),
            0
        );
    }

    #[test]
    fn board_state_clear_diagnostics_resets_live_faults_and_log() {
        let mut state = BoardEcuState::new();
        state.emerg_trig_map = true;
        state.emerg_trig_tps = true;
        state.emergency_mode = true;
        state.diag_map.latch(Micros::new(10));
        state.diag_tps.latch(Micros::new(20));
        state.diag_log.push(DiagEvent {
            code: DiagCode::MapRange,
            timestamp: Micros::new(100),
            source: DiagSource::Sensor,
            context: Some(100),
            start_us: 10,
            end_us: 40,
        });
        state.diag_log.push(DiagEvent {
            code: DiagCode::TpsRange,
            timestamp: Micros::new(200),
            source: DiagSource::Sensor,
            context: Some(50),
            start_us: 20,
            end_us: 60,
        });

        let summary = state.clear_diagnostics();

        assert_eq!(
            summary,
            ecu_domain::diag::DiagClearSummary {
                cleared_active_count: 2,
                cleared_log_entries: 2,
                emergency_cleared: true,
            }
        );
        assert!(!state.diag_map.is_active());
        assert!(!state.diag_tps.is_active());
        assert!(!state.emergency_mode());
        assert!(state.diag_log.events.iter().all(|entry| entry.is_none()));
        assert_eq!(state.diag_log.head, 0);
        assert!(state.emerg_trig_map);
        assert!(state.emerg_trig_tps);
    }

    #[test]
    fn board_state_clear_diagnostics_is_noop_for_clean_state() {
        let mut state = BoardEcuState::new();

        let summary = state.clear_diagnostics();

        assert_eq!(summary, ecu_domain::diag::DiagClearSummary::default());
        assert!(!state.diag_map.is_active());
        assert!(!state.diag_tps.is_active());
        assert!(!state.emergency_mode());
        assert!(state.diag_log.events.iter().all(|entry| entry.is_none()));
    }
}
