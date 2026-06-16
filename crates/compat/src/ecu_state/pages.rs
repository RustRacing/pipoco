use super::EcuState;
use crate::units::{Kpa10, Micros, Rpm};
use crate::{trigger, ts};
use ecu_ts::pages::{DiagnosticLogEntry, DIAG_LOG_ENTRY_COUNT};

impl EcuState {
    pub fn page_store(&mut self) -> crate::ts::pages::EcuPageStore<'_> {
        let mut diag_log_entries = [None; DIAG_LOG_ENTRY_COUNT];
        for (entry, slot) in diag_log_entries
            .iter_mut()
            .zip(self.faults.diag_log.events.iter())
        {
            *entry = slot.as_ref().map(|ev| DiagnosticLogEntry {
                code: ev.code.to_u8(),
                start_us: ev.start_us,
                end_us: ev.end_us,
            });
        }
        let sync_loss_counter = self.sync_loss_tracker.total_losses;
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
        let last_fault_code = self
            .diag_log()
            .events
            .iter()
            .rev()
            .find_map(|ev| ev.as_ref().map(|ev| ev.code.to_u8()))
            .unwrap_or(0);
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
            last_fault_code,
            isr_count: self.isr_stats.count,
            isr_max_us: self.isr_stats.max_us,
            isr_avg_us: self.isr_stats.avg_us,
        };
    }
}
