//! Composite ECU page store and system snapshot.
//!
//! Maps runtime/calibration state (ecu-domain + ecu-calibration types) onto the
//! core-free TS page codecs/DTOs in this crate. Gated behind the `runtime`
//! feature because it depends on the calibration config and domain unit types.

use super::{
    encode_fuel_table_page, encode_ignition_table_page, read_enrichment_page, ts_page_descriptor,
    write_enrichment_page, ActuatorPageStore, AeSetup, AnglesPageStore, AnglesSetup, AseSetup,
    ClSetup, DfcoSetup, DiagnosticLogEntry, DiagnosticPageStore, DiagnosticSnapshot,
    ExpertTriggerPageState, FanSetup, FuelIgnPageStore, FuelTunePageStore, IdleSetup,
    LimitsPageStore, LimitsSetup, SensorsPageStore, SensorsSetup, SnapshotPageStore,
    VeTunePageLimits, VeTuneSetup, WueSetup, DIAG_LOG_ENTRY_COUNT, PAGE_AE, PAGE_AFR_TABLE,
    PAGE_ANGLES, PAGE_ASE, PAGE_CL, PAGE_DFCO, PAGE_DIAG, PAGE_DIAG_LOG, PAGE_EXPERT_TRIGGER,
    PAGE_FAN, PAGE_FUEL, PAGE_IDLE, PAGE_IGN, PAGE_LIMITS, PAGE_SENSORS, PAGE_SNAPSHOT,
    PAGE_VE_TABLE, PAGE_VE_TUNE, PAGE_WUE,
};
use crate::server::{PageError, PageStore};
use ecu_calibration::configs::{
    AeConfig, AseConfig, ClConfig, DfcoConfig, FanConfig, IdleConfig, SensorsLimits, WueConfig,
};
use ecu_calibration::sensors::SensorsCal;
use ecu_domain::{Micros, Rpm, SyncState};

const MIN_PULSE_WIDTH_US: u16 = 500;
const MAX_PULSE_WIDTH_US: u16 = 20000;

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct SystemSnapshot {
    pub rpm: Rpm,
    pub sync: SyncState,
    pub base_pw: Micros,
    pub enrich_mult_x100: u16,
    pub stft_x10: i16,
    pub fuel_mult_x100: u16,
    pub final_pw: Micros,
    pub last_fault_code: u8,
    pub isr_count: u32,
    pub isr_max_us: u32,
    pub isr_avg_us: u32,
}

fn ve_tune_limits() -> VeTunePageLimits {
    VeTunePageLimits::new(MIN_PULSE_WIDTH_US, MAX_PULSE_WIDTH_US)
}

/// Composite page store including fuel, ignition, and sensors calibration
pub struct EcuPageStore<'a> {
    pub fuel: &'a mut [[u16; 16]; 16],
    pub ve: &'a mut [[u16; 16]; 16],
    pub afr: &'a mut [[u16; 16]; 16],
    pub required_fuel_us: &'a mut u16,
    pub injector_deadtime_us: &'a mut u16,
    pub ve_load_source: &'a mut u8,
    pub ign: &'a mut [[i16; 16]; 16],
    pub sens: &'a mut SensorsCal,
    pub ae: &'a mut AeConfig,
    pub dfco: &'a mut DfcoConfig,
    pub wue: &'a mut WueConfig,
    pub ase: &'a mut AseConfig,
    pub idle: &'a mut IdleConfig,
    pub fan: &'a mut FanConfig,
    pub cl: &'a mut ClConfig,
    pub limits: &'a mut SensorsLimits,
    pub emerg_trig_map: &'a mut bool,
    pub emerg_trig_tps: &'a mut bool,
    pub diag_log_entries: [Option<DiagnosticLogEntry>; DIAG_LOG_ENTRY_COUNT],
    pub snapshot: &'a SystemSnapshot,
    pub tooth_count: &'a u8,
    pub sync_loss_counter: u16,
    pub angles_inj: &'a mut [u16; 16],
    pub angles_tdc: &'a mut [u16; 16],
    pub tooth0_angle_x10: &'a mut u16,
    pub cam_timeout_ms: &'a mut u16,
    pub expert_trigger: &'a mut ExpertTriggerPageState,
}

impl<'a> EcuPageStore<'a> {
    fn diag_snapshot(&self) -> DiagnosticSnapshot {
        let sync_state = match self.snapshot.sync {
            SyncState::Unsynced => 0,
            SyncState::Provisional => 1,
            SyncState::Locked { .. } => 2,
        };
        let phase_state = match self.snapshot.sync {
            SyncState::Unsynced => 0,
            SyncState::Provisional => 1,
            SyncState::Locked { cam_ref: false } => 2,
            SyncState::Locked { cam_ref: true } => 3,
        };

        DiagnosticSnapshot {
            current_tooth_count: *self.tooth_count,
            cam_seen: matches!(self.snapshot.sync, SyncState::Locked { cam_ref: true }),
            sync_state,
            phase_state,
            absolute_authority: self.expert_trigger.authority_code(),
            trigger_angle_source: 0,
            output_gating_reason: 0,
            last_sync_loss_reason: 0,
            primary_rpm: self.snapshot.rpm.raw(),
            detected_gap_ratio: 0,
            sync_loss_counter: self.sync_loss_counter,
            board_pin_map_identity: 0,
            profile_identity: self.expert_trigger.profile_identity(),
            profile_hash: self.expert_trigger.profile_hash(),
        }
    }

    fn read_diag_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        let snapshot = self.diag_snapshot();
        DiagnosticPageStore::read_page_from(page, &snapshot, &self.diag_log_entries, out)
    }

    fn ae_setup(&self) -> AeSetup {
        AeSetup {
            tpsdot_thresh_pct_s: self.ae.tpsdot_thresh_pct_s,
            mapdot_thresh_kpa_s: self.ae.mapdot_thresh_kpa_s,
            percent_gain: self.ae.percent_gain,
            decay_time_ms: self.ae.decay_time_ms,
            lockout_ms: self.ae.lockout_ms,
        }
    }

    fn dfco_setup(&self) -> DfcoSetup {
        DfcoSetup {
            tps_max_pct: self.dfco.tps_max_pct,
            map_max_kpa: self.dfco.map_max_kpa,
            rpm_min: self.dfco.rpm_min,
            rpm_max: self.dfco.rpm_max,
            delay_ms: self.dfco.delay_ms,
            resume_hyst_ms: self.dfco.resume_hyst_ms,
        }
    }

    fn wue_setup(&self) -> WueSetup {
        WueSetup {
            max_percent: self.wue.max_percent,
            min_percent: self.wue.min_percent,
            start_c: self.wue.start_c,
            end_c: self.wue.end_c,
        }
    }

    fn ase_setup(&self) -> AseSetup {
        AseSetup {
            percent: self.ase.percent,
            taper_time_ms: self.ase.taper_time_ms,
            lockout_ms: self.ase.lockout_ms,
        }
    }

    fn idle_setup(&self) -> IdleSetup {
        IdleSetup {
            enable: self.idle.enable,
            duty_x10: self.idle.duty_x10,
            freq_hz: self.idle.freq_hz,
        }
    }

    fn fan_setup(&self) -> FanSetup {
        FanSetup {
            enable: self.fan.enable,
            on_c: self.fan.on_c,
            off_c: self.fan.off_c,
        }
    }

    fn cl_setup(&self) -> ClSetup {
        ClSetup {
            enable: self.cl.enable,
            target_afr_x10: self.cl.target_afr_x10,
            kp_i: self.cl.kp_i,
            ki_i: self.cl.ki_i,
        }
    }

    fn sensors_setup(&self) -> SensorsSetup<'_> {
        SensorsSetup {
            tps_min_counts: self.sens.tps_min_counts,
            tps_max_counts: self.sens.tps_max_counts,
            map_v0_mv: self.sens.map_v0_mv,
            map_kpa0_x10: self.sens.map_kpa0_x10,
            map_v1_mv: self.sens.map_v1_mv,
            map_kpa1_x10: self.sens.map_kpa1_x10,
            clt_deg_c: &self.sens.clt_deg_c,
            iat_deg_c: &self.sens.iat_deg_c,
            clt_ohms: &self.sens.clt_ohms,
            iat_ohms: &self.sens.iat_ohms,
        }
    }

    fn limits_setup(&self) -> LimitsSetup {
        LimitsSetup {
            map_min_kpa_x10: self.limits.map_min_kpa_x10,
            map_max_kpa_x10: self.limits.map_max_kpa_x10,
            tps_min_percent: self.limits.tps_min_percent,
            tps_max_percent: self.limits.tps_max_percent,
            clear_time_s: self.limits.clear_time_s,
            emerg_trig_map: *self.emerg_trig_map,
            emerg_trig_tps: *self.emerg_trig_tps,
        }
    }
}

impl<'a> PageStore for EcuPageStore<'a> {
    fn page_len(&self, page: u8) -> Option<usize> {
        ts_page_descriptor(page).map(|descriptor| descriptor.len)
    }
    fn read_page(&self, page: u8, out: &mut [u8]) -> Option<usize> {
        match page {
            PAGE_FUEL => encode_fuel_table_page(self.fuel, out).ok(),
            PAGE_IGN => encode_ignition_table_page(self.ign, out).ok(),
            PAGE_SENSORS => SensorsPageStore::read_page_from(page, &self.sensors_setup(), out),
            PAGE_LIMITS => LimitsPageStore::read_page_from(page, &self.limits_setup(), out),
            PAGE_DIAG | PAGE_DIAG_LOG => self.read_diag_page(page, out),
            PAGE_ANGLES => AnglesPageStore::read_page_from(
                page,
                &AnglesSetup {
                    inj_angles_x10: self.angles_inj,
                    tdc_angles_x10: self.angles_tdc,
                    tooth0_angle_x10: *self.tooth0_angle_x10,
                    cam_timeout_ms: *self.cam_timeout_ms,
                },
                out,
            ),
            PAGE_AE | PAGE_DFCO | PAGE_WUE | PAGE_ASE => read_enrichment_page(
                page,
                &self.ae_setup(),
                &self.dfco_setup(),
                &self.wue_setup(),
                &self.ase_setup(),
                out,
            ),
            PAGE_IDLE | PAGE_FAN | PAGE_CL => ActuatorPageStore::read_page_from(
                page,
                &self.idle_setup(),
                &self.fan_setup(),
                &self.cl_setup(),
                out,
            ),
            PAGE_VE_TUNE | PAGE_VE_TABLE | PAGE_AFR_TABLE => FuelTunePageStore::read_page_from(
                page,
                self.ve,
                self.afr,
                &VeTuneSetup {
                    target_afr_x10: self.cl.target_afr_x10,
                    kp_i: self.cl.kp_i,
                    ki_i: self.cl.ki_i,
                    required_fuel_us: *self.required_fuel_us,
                    injector_deadtime_us: *self.injector_deadtime_us,
                    ve_load_source: *self.ve_load_source,
                },
                ve_tune_limits(),
                out,
            ),
            PAGE_SNAPSHOT => {
                let sync_code = match self.snapshot.sync {
                    SyncState::Unsynced => 0,
                    SyncState::Provisional => 1,
                    SyncState::Locked { .. } => 2,
                };
                SnapshotPageStore::read_page_from(
                    page,
                    self.snapshot.rpm.raw(),
                    sync_code,
                    self.snapshot.base_pw.raw(),
                    self.snapshot.enrich_mult_x100,
                    self.snapshot.stft_x10,
                    self.snapshot.fuel_mult_x100,
                    self.snapshot.final_pw.raw(),
                    self.snapshot.last_fault_code,
                    self.snapshot.isr_count,
                    self.snapshot.isr_max_us,
                    self.snapshot.isr_avg_us,
                    out,
                )
            }
            PAGE_EXPERT_TRIGGER => self.expert_trigger.read_page(page, out),
            _ => None,
        }
    }
    fn write_page(&mut self, page: u8, data: &[u8]) -> Result<(), PageError> {
        match page {
            PAGE_FUEL | PAGE_IGN => FuelIgnPageStore {
                fuel: self.fuel,
                ign: self.ign,
            }
            .write_page(page, data),
            PAGE_SENSORS => SensorsPageStore {
                tps_min_counts: &mut self.sens.tps_min_counts,
                tps_max_counts: &mut self.sens.tps_max_counts,
                map_v0_mv: &mut self.sens.map_v0_mv,
                map_kpa0_x10: &mut self.sens.map_kpa0_x10,
                map_v1_mv: &mut self.sens.map_v1_mv,
                map_kpa1_x10: &mut self.sens.map_kpa1_x10,
                clt_deg_c: &mut self.sens.clt_deg_c,
                iat_deg_c: &mut self.sens.iat_deg_c,
                clt_ohms: &mut self.sens.clt_ohms,
                iat_ohms: &mut self.sens.iat_ohms,
            }
            .write_page(page, data),
            PAGE_LIMITS => LimitsPageStore {
                map_min_kpa_x10: &mut self.limits.map_min_kpa_x10,
                map_max_kpa_x10: &mut self.limits.map_max_kpa_x10,
                tps_min_percent: &mut self.limits.tps_min_percent,
                tps_max_percent: &mut self.limits.tps_max_percent,
                clear_time_s: &mut self.limits.clear_time_s,
                emerg_trig_map: self.emerg_trig_map,
                emerg_trig_tps: self.emerg_trig_tps,
            }
            .write_page(page, data),
            PAGE_AE | PAGE_DFCO | PAGE_WUE | PAGE_ASE => {
                let mut ae = self.ae_setup();
                let mut dfco = self.dfco_setup();
                let mut wue = self.wue_setup();
                let mut ase = self.ase_setup();
                write_enrichment_page(page, data, &mut ae, &mut dfco, &mut wue, &mut ase)?;
                self.ae.tpsdot_thresh_pct_s = ae.tpsdot_thresh_pct_s;
                self.ae.mapdot_thresh_kpa_s = ae.mapdot_thresh_kpa_s;
                self.ae.percent_gain = ae.percent_gain;
                self.ae.decay_time_ms = ae.decay_time_ms;
                self.ae.lockout_ms = ae.lockout_ms;
                self.dfco.tps_max_pct = dfco.tps_max_pct;
                self.dfco.map_max_kpa = dfco.map_max_kpa;
                self.dfco.rpm_min = dfco.rpm_min;
                self.dfco.rpm_max = dfco.rpm_max;
                self.dfco.delay_ms = dfco.delay_ms;
                self.dfco.resume_hyst_ms = dfco.resume_hyst_ms;
                self.wue.max_percent = wue.max_percent;
                self.wue.min_percent = wue.min_percent;
                self.wue.start_c = wue.start_c;
                self.wue.end_c = wue.end_c;
                self.ase.percent = ase.percent;
                self.ase.taper_time_ms = ase.taper_time_ms;
                self.ase.lockout_ms = ase.lockout_ms;
                Ok(())
            }
            PAGE_IDLE | PAGE_FAN | PAGE_CL => ActuatorPageStore {
                idle_enable: &mut self.idle.enable,
                idle_duty_x10: &mut self.idle.duty_x10,
                idle_freq_hz: &mut self.idle.freq_hz,
                fan_enable: &mut self.fan.enable,
                fan_on_c: &mut self.fan.on_c,
                fan_off_c: &mut self.fan.off_c,
                cl_enable: &mut self.cl.enable,
                cl_target_afr_x10: &mut self.cl.target_afr_x10,
                cl_kp_i: &mut self.cl.kp_i,
                cl_ki_i: &mut self.cl.ki_i,
            }
            .write_page(page, data),
            PAGE_VE_TUNE | PAGE_VE_TABLE | PAGE_AFR_TABLE => FuelTunePageStore {
                ve: self.ve,
                afr: self.afr,
                target_afr_x10: &mut self.cl.target_afr_x10,
                kp_i: &mut self.cl.kp_i,
                ki_i: &mut self.cl.ki_i,
                required_fuel_us: self.required_fuel_us,
                injector_deadtime_us: self.injector_deadtime_us,
                ve_load_source: self.ve_load_source,
                limits: ve_tune_limits(),
            }
            .write_page(page, data),
            PAGE_DIAG => Err(PageError::Invalid),
            PAGE_EXPERT_TRIGGER => self.expert_trigger.write_page(page, data),
            PAGE_ANGLES => AnglesPageStore {
                inj_angles_x10: self.angles_inj,
                tdc_angles_x10: self.angles_tdc,
                tooth0_angle_x10: self.tooth0_angle_x10,
                cam_timeout_ms: self.cam_timeout_ms,
            }
            .write_page(page, data),
            _ => Err(PageError::Invalid),
        }
    }
}
