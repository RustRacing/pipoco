//! Core-free assembly of [`ecu_ts::pages::EcuPageStore`] from explicit inputs.
//!
//! Reimplements the legacy page-store / snapshot-refresh assembly that used to
//! live in the compatibility core, but takes every datum as a plain parameter so
//! boards can build the TS page surface without depending on the core crate. The
//! byte/encode behavior (field mapping, `DiagCode::to_u8` usage) is identical to
//! the legacy builder, so a board using this produces byte-identical TS pages.

use ecu_calibration::configs::EcuConfig;
use ecu_domain::diag::DiagLog;
use ecu_domain::{Micros, Rpm, SyncState};
use ecu_ts::pages::{
    ts_diag_log_action_code, ts_diag_log_severity_code, ts_diag_source_code, DiagnosticLogEntry,
    EcuPageStore, ExpertTriggerPageState, SystemSnapshot, DIAG_LOG_ENTRY_COUNT,
};

/// Runtime state needed to assemble the diagnostic and snapshot pages.
///
/// Every field is a core-free primitive or `ecu-ts`/`ecu-domain` type. In the
/// legacy core path these came from runtime state fields; the board now
/// owns/computes them and passes them in.
pub struct PageRuntime<'a> {
    pub snapshot: &'a SystemSnapshot,
    pub tooth_count: &'a u8,
    pub sync_loss_counter: u16,
    pub current_fault_code: u8,
    pub current_fault_severity: u8,
    pub current_fault_action: u8,
    pub current_cancel_reason: u8,
    pub fault_flags: u8,
    pub latest_diag_code: u8,
    pub emerg_trig_map: &'a mut bool,
    pub emerg_trig_tps: &'a mut bool,
    pub diag_log_entries: [Option<DiagnosticLogEntry>; DIAG_LOG_ENTRY_COUNT],
    pub expert_trigger: &'a mut ExpertTriggerPageState,
}

/// Build the diagnostic-log-entry array from a domain [`DiagLog`].
///
/// Identical encode to the core builder: each populated slot maps to a
/// [`DiagnosticLogEntry`] using `DiagCode::to_u8()`.
pub fn diag_log_entries_from<const N: usize>(
    diag_log: &DiagLog<N>,
) -> [Option<DiagnosticLogEntry>; DIAG_LOG_ENTRY_COUNT] {
    let mut entries = [None; DIAG_LOG_ENTRY_COUNT];
    for (entry, slot) in entries.iter_mut().zip(diag_log.events.iter()) {
        *entry = slot.as_ref().map(|ev| DiagnosticLogEntry {
            code: ev.code.to_u8(),
            severity: ts_diag_log_severity_code(ev.code.to_u8()),
            action: ts_diag_log_action_code(ev.code.to_u8(), ts_diag_source_code(ev.source)),
            source: ts_diag_source_code(ev.source),
            context_present: ev.context.is_some(),
            context: ev.context.unwrap_or(0),
            start_us: ev.start_us,
            end_us: ev.end_us,
        });
    }
    entries
}

/// Derive `last_fault_code` from a domain [`DiagLog`] the same way the core
/// builder does: the most recent populated event's code, or `0`.
pub fn last_fault_code_from<const N: usize>(diag_log: &DiagLog<N>) -> u8 {
    diag_log
        .events
        .iter()
        .rev()
        .find_map(|ev| ev.as_ref().map(|ev| ev.code.to_u8()))
        .unwrap_or(0)
}

/// Scalar runtime inputs for [`build_system_snapshot`].
///
/// `synced` replaces the core `trigger_inputs().synced` flag (a core-owned
/// computation); the board computes it and passes a plain `bool`.
pub struct SnapshotInputs {
    pub rpm: u16,
    pub sync: SyncState,
    pub base_pw_us: u32,
    pub enrich_mult_x100: u16,
    pub stft_x10: i16,
    pub fuel_mult_x100: u16,
    pub final_pw: Micros,
    pub last_fault_code: u8,
    pub fault_severity: u8,
    pub cancel_reason: u8,
    pub isr_count: u32,
    pub isr_max_us: u32,
    pub isr_avg_us: u32,
}

/// Assemble a [`SystemSnapshot`] from core-free scalar inputs, mirroring the
/// legacy snapshot-refresh field mapping (including the `synced` ->
/// `SyncState` translation).
pub fn build_system_snapshot(inputs: SnapshotInputs) -> SystemSnapshot {
    SystemSnapshot {
        rpm: Rpm::new(inputs.rpm),
        sync: inputs.sync,
        base_pw: Micros::new(inputs.base_pw_us),
        enrich_mult_x100: inputs.enrich_mult_x100,
        stft_x10: inputs.stft_x10,
        fuel_mult_x100: inputs.fuel_mult_x100,
        final_pw: inputs.final_pw,
        last_fault_code: inputs.last_fault_code,
        fault_severity: inputs.fault_severity,
        cancel_reason: inputs.cancel_reason,
        isr_count: inputs.isr_count,
        isr_max_us: inputs.isr_max_us,
        isr_avg_us: inputs.isr_avg_us,
    }
}

/// Assemble an [`EcuPageStore`] from a persisted [`EcuConfig`] and core-free
/// runtime state.
///
/// This is the core-free analogue of the legacy page-store builder. The config
/// fields map exactly as the legacy builder mapped its `config.*` fields, and the
/// runtime fields map exactly as it mapped its snapshot, fault flags, tooth
/// count, sync-loss counter, and expert-trigger state.
pub fn build_ecu_page_store<'a>(
    config: &'a mut EcuConfig,
    runtime: PageRuntime<'a>,
) -> EcuPageStore<'a> {
    EcuPageStore {
        fuel: &mut config.ipw_table,
        ve: &mut config.ve_table,
        afr: &mut config.afr_table,
        required_fuel_us: &mut config.required_fuel_us,
        injector_deadtime_us: &mut config.injector_deadtime_us,
        ve_load_source: &mut config.ve_load_source,
        ign: &mut config.ignition_table,
        sens: &mut config.sensors_cal,
        ae: &mut config.ae_config,
        dfco: &mut config.dfco_config,
        wue: &mut config.wue_config,
        ase: &mut config.ase_config,
        idle: &mut config.idle_config,
        fan: &mut config.fan_config,
        cl: &mut config.cl_config,
        limits: &mut config.sensors_limits,
        emerg_trig_map: runtime.emerg_trig_map,
        emerg_trig_tps: runtime.emerg_trig_tps,
        diag_log_entries: runtime.diag_log_entries,
        snapshot: runtime.snapshot,
        tooth_count: runtime.tooth_count,
        sync_loss_counter: runtime.sync_loss_counter,
        current_fault_code: runtime.current_fault_code,
        current_fault_severity: runtime.current_fault_severity,
        current_fault_action: runtime.current_fault_action,
        current_cancel_reason: runtime.current_cancel_reason,
        fault_flags: runtime.fault_flags,
        latest_diag_code: runtime.latest_diag_code,
        angles_inj: &mut config.inj_angle_btdc_x10,
        angles_tdc: &mut config.tdc_per_cyl_x10,
        tooth0_angle_x10: &mut config.tooth0_angle_x10,
        cam_timeout_ms: &mut config.cam_missing_timeout_ms,
        expert_trigger: runtime.expert_trigger,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_calibration::configs::{
        AeConfig, AseConfig, ClConfig, DfcoConfig, FanConfig, IdleConfig, LambdaConfig,
        LoadFailureConfig, PlausibilityConfig, RateConfig, RevLimiterConfig, SensorsLimits,
        WueConfig,
    };
    use ecu_calibration::sensors::SensorsCal;
    use ecu_domain::diag::{DiagCode, DiagEvent, DiagLog, DiagSource};
    use ecu_ts::pages::{
        PAGE_FUEL, PAGE_SNAPSHOT, TS_CANCEL_REASON_NONE, TS_FAULT_ACTION_NONE,
        TS_FAULT_FLAG_DIAG_LOG_PRESENT, TS_FAULT_SEVERITY_NONE,
    };
    use ecu_ts::server::PageStore;

    fn test_config() -> EcuConfig {
        EcuConfig {
            ipw_table: [[1000; 16]; 16],
            ve_table: [[100; 16]; 16],
            afr_table: [[147; 16]; 16],
            required_fuel_us: 1000,
            injector_deadtime_us: 800,
            ve_load_source: 0,
            ignition_table: [[100; 16]; 16],
            sensors_cal: SensorsCal::default(),
            sensors_limits: SensorsLimits::default(),
            ae_config: AeConfig::DEFAULT,
            wue_config: WueConfig::DEFAULT,
            ase_config: AseConfig::DEFAULT,
            dfco_config: DfcoConfig::DEFAULT,
            idle_config: IdleConfig::DEFAULT,
            fan_config: FanConfig::DEFAULT,
            cl_config: ClConfig::DEFAULT,
            load_failure_config: LoadFailureConfig::DEFAULT,
            plausibility_config: PlausibilityConfig::DEFAULT,
            rate_config: RateConfig::DEFAULT,
            lambda_config: LambdaConfig::DEFAULT,
            rev_limiter_config: RevLimiterConfig::DEFAULT,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
            cam_missing_timeout_ms: 500,
        }
    }

    #[test]
    fn diag_log_entries_mirror_core_encode() {
        let mut diag_log: DiagLog<DIAG_LOG_ENTRY_COUNT> = DiagLog::new();
        diag_log.push(DiagEvent {
            code: DiagCode::KnockDetected,
            timestamp: Micros::new(10),
            source: DiagSource::Safety,
            context: Some(7),
            start_us: 10,
            end_us: 20,
        });
        let entries = diag_log_entries_from(&diag_log);
        let first = entries[0].expect("first slot populated");
        assert_eq!(first.code, DiagCode::KnockDetected.to_u8());
        assert_eq!(first.severity, ecu_ts::pages::TS_FAULT_SEVERITY_WARNING);
        assert_eq!(first.action, ecu_ts::pages::TS_FAULT_ACTION_LIMP_HOME);
        assert_eq!(first.source, ecu_ts::pages::TS_DIAG_SOURCE_SAFETY);
        assert!(first.context_present);
        assert_eq!(first.context, 7);
        assert_eq!(first.start_us, 10);
        assert_eq!(first.end_us, 20);
        assert_eq!(
            last_fault_code_from(&diag_log),
            DiagCode::KnockDetected.to_u8()
        );
    }

    #[test]
    fn build_store_reads_pages() {
        let mut config = test_config();
        let snapshot = build_system_snapshot(SnapshotInputs {
            rpm: 1500,
            sync: SyncState::Locked { cam_ref: false },
            base_pw_us: 3000,
            enrich_mult_x100: 100,
            stft_x10: 0,
            fuel_mult_x100: 100,
            final_pw: Micros::new(3200),
            last_fault_code: 0,
            fault_severity: TS_FAULT_SEVERITY_NONE,
            cancel_reason: TS_CANCEL_REASON_NONE,
            isr_count: 7,
            isr_max_us: 12,
            isr_avg_us: 4,
        });
        let tooth_count: u8 = 36;
        let mut emerg_trig_map = false;
        let mut emerg_trig_tps = false;
        let mut expert_trigger = ExpertTriggerPageState::new();
        let runtime = PageRuntime {
            snapshot: &snapshot,
            tooth_count: &tooth_count,
            sync_loss_counter: 2,
            current_fault_code: 0,
            current_fault_severity: TS_FAULT_SEVERITY_NONE,
            current_fault_action: TS_FAULT_ACTION_NONE,
            current_cancel_reason: TS_CANCEL_REASON_NONE,
            fault_flags: TS_FAULT_FLAG_DIAG_LOG_PRESENT,
            latest_diag_code: 0,
            emerg_trig_map: &mut emerg_trig_map,
            emerg_trig_tps: &mut emerg_trig_tps,
            diag_log_entries: [None; DIAG_LOG_ENTRY_COUNT],
            expert_trigger: &mut expert_trigger,
        };
        let store = build_ecu_page_store(&mut config, runtime);

        let fuel_len = store.page_len(PAGE_FUEL).expect("fuel page len");
        let mut buf = [0u8; 1024];
        let read = store
            .read_page(PAGE_FUEL, &mut buf[..fuel_len])
            .expect("read fuel page");
        assert_eq!(read, fuel_len);

        let snap_len = store.page_len(PAGE_SNAPSHOT).expect("snapshot page len");
        let snap_read = store
            .read_page(PAGE_SNAPSHOT, &mut buf[..snap_len])
            .expect("read snapshot page");
        assert_eq!(snap_read, snap_len);
    }
}
