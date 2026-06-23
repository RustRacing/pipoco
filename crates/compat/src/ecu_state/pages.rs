use super::EcuState;
use crate::units::{Kpa10, Micros, Rpm};
use crate::{trigger, ts};
use ecu_ts::pages::{
    ts_diag_log_action_code, ts_diag_log_severity_code, ts_diag_source_code,
    ts_fault_code_from_diag_code, DiagnosticLogEntry, DIAG_LOG_ENTRY_COUNT, TS_CANCEL_REASON_NONE,
    TS_CURRENT_FAULT_CAM_MISSING, TS_CURRENT_FAULT_NONE, TS_FAULT_ACTION_LIMP_HOME,
    TS_FAULT_ACTION_NONE, TS_FAULT_ACTION_OBSERVE_ONLY, TS_FAULT_FLAG_ACTIVE,
    TS_FAULT_FLAG_DIAG_LOG_PRESENT, TS_FAULT_FLAG_EMERGENCY_MODE, TS_FAULT_FLAG_SNAPSHOT_PRESENT,
    TS_FAULT_SEVERITY_NONE, TS_FAULT_SEVERITY_WARNING,
};

fn latest_diag_code(state: &EcuState) -> u8 {
    state
        .diag_log()
        .events
        .iter()
        .rev()
        .find_map(|ev| ev.as_ref().map(|ev| ev.code.to_u8()))
        .unwrap_or(0)
}

fn current_fault_surface(state: &EcuState) -> (u8, u8, u8, u8, u8) {
    let mut code = TS_CURRENT_FAULT_NONE;
    if state.diag_map.is_active() {
        code = ts_fault_code_from_diag_code(crate::diag::DiagCode::MapRange);
    } else if state.diag_tps.is_active() {
        code = ts_fault_code_from_diag_code(crate::diag::DiagCode::TpsRange);
    } else if state.diag_cam.is_active() {
        code = TS_CURRENT_FAULT_CAM_MISSING;
    }

    let active = code != TS_CURRENT_FAULT_NONE;
    let severity = if active {
        TS_FAULT_SEVERITY_WARNING
    } else {
        TS_FAULT_SEVERITY_NONE
    };
    let action = if !active {
        TS_FAULT_ACTION_NONE
    } else if state.emergency_mode() {
        TS_FAULT_ACTION_LIMP_HOME
    } else {
        TS_FAULT_ACTION_OBSERVE_ONLY
    };
    let mut flags = 0u8;
    if active {
        flags |= TS_FAULT_FLAG_ACTIVE | TS_FAULT_FLAG_SNAPSHOT_PRESENT;
    }
    if state.emergency_mode() {
        flags |= TS_FAULT_FLAG_EMERGENCY_MODE;
    }
    if latest_diag_code(state) != 0 {
        flags |= TS_FAULT_FLAG_DIAG_LOG_PRESENT;
    }
    (code, severity, action, TS_CANCEL_REASON_NONE, flags)
}

impl EcuState {
    pub fn page_store(&mut self) -> crate::ts::pages::EcuPageStore<'_> {
        let mut diag_log_entries = [None; DIAG_LOG_ENTRY_COUNT];
        for (entry, slot) in diag_log_entries
            .iter_mut()
            .zip(self.faults.diag_log.events.iter())
        {
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
        let sync_loss_counter = self.sync_loss_tracker.total_losses;
        let latest_diag_code = latest_diag_code(self);
        let (
            current_fault_code,
            current_fault_severity,
            current_fault_action,
            current_cancel_reason,
            fault_flags,
        ) = current_fault_surface(self);
        crate::ts::pages::EcuPageStore {
            fuel: &mut self.config.ipw_table,
            ve: &mut self.config.ve_table,
            afr: &mut self.config.afr_table,
            required_fuel_us: &mut self.config.required_fuel_us,
            injector_deadtime_us: &mut self.config.injector_deadtime_us,
            ve_load_source: &mut self.config.ve_load_source,
            ign: &mut self.config.ignition_table,
            sens: &mut self.config.sensors_cal,
            ae: &mut self.config.ae_config,
            dfco: &mut self.config.dfco_config,
            wue: &mut self.config.wue_config,
            ase: &mut self.config.ase_config,
            idle: &mut self.config.idle_config,
            fan: &mut self.config.fan_config,
            cl: &mut self.config.cl_config,
            limits: &mut self.config.sensors_limits,
            emerg_trig_map: &mut self.faults.emergency_trigger_map_oob,
            emerg_trig_tps: &mut self.faults.emergency_trigger_tps_oob,
            diag_log_entries,
            snapshot: &self.snapshot,
            tooth_count: &self.tooth_count,
            sync_loss_counter,
            current_fault_code,
            current_fault_severity,
            current_fault_action,
            current_cancel_reason,
            fault_flags,
            latest_diag_code,
            angles_inj: &mut self.config.inj_angle_btdc_x10,
            angles_tdc: &mut self.config.tdc_per_cyl_x10,
            tooth0_angle_x10: &mut self.config.tooth0_angle_x10,
            cam_timeout_ms: &mut self.config.cam_missing_timeout_ms,
            expert_trigger: &mut self.expert_trigger,
        }
    }

    pub fn refresh_snapshot(&mut self) {
        let trigger_inputs = self.trigger_inputs();
        let base_pw = self.calculate_fuel(trigger_inputs.rpm, self.map_kpa_x10 / 10);
        let final_pw = self.final_pw(Rpm::new(trigger_inputs.rpm), Kpa10::new(self.map_kpa_x10));
        let commanded_advance_x10 =
            self.calculate_ignition_timing_with_limiter(trigger_inputs.rpm, self.map_kpa_x10) * 10;
        self.outputs.final_pw = final_pw;
        self.outputs.commanded_advance_x10 = commanded_advance_x10;
        let enrich_mult_x100 = [
            self.derived.wue_percent,
            self.derived.ase_percent,
            self.derived.ae_percent,
        ]
        .into_iter()
        .fold(100u32, |acc, pct| {
            acc.saturating_mul(100 + pct as u32) / 100
        }) as u16;
        let (current_fault_code, current_fault_severity, _, current_cancel_reason, _) =
            current_fault_surface(self);
        self.snapshot = ts::pages::SystemSnapshot {
            rpm: Rpm::new(trigger_inputs.rpm),
            sync: if trigger_inputs.synced {
                trigger::SyncState::Locked { cam_ref: false }
            } else {
                trigger::SyncState::Unsynced
            },
            base_pw: Micros::new(base_pw as u32),
            enrich_mult_x100,
            stft_x10: self.stft_x10(),
            fuel_mult_x100: self.fuel_mult_x100(),
            final_pw,
            last_fault_code: current_fault_code,
            fault_severity: current_fault_severity,
            cancel_reason: current_cancel_reason,
            isr_count: self.isr_stats.count,
            isr_max_us: self.isr_stats.max_us,
            isr_avg_us: self.isr_stats.avg_us,
        };
    }
}
