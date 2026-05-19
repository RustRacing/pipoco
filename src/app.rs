//! Generic ECU application runtime helper
//!
//! Owns decoder, state, scheduler, latch, and stats.
//! Targets feed timestamps and drive outputs via this API.

use crate::config::{EcuConfig, IgnitionMode, InjectionMode};
use crate::constants::fuel::DEFAULT_LOAD_KPA;
use crate::constants::timing::{
    DWELL_TIME_US, IGNITION_TOOTH, INJECTION_DELAY_US, INJECTION_TOOTH,
};
use crate::hal::{OutputPin, TimeSource};
use crate::safety::{apply_safe_state, update_flood_clear, OutputLatch};
use crate::scheduler::{Channel, Scheduler};
use crate::telemetry::IsrStats;
use crate::units::{Kpa10, Micros, Rpm};
use crate::{EcuState, TriggerDecoder};
// Configurable via EcuState.cam_missing_timeout_ms

pub struct EcuApp<T: TimeSource> {
    decoder: TriggerDecoder<T>,
    state: EcuState,
    scheduler: Scheduler,
    latch: OutputLatch,
    stats: IsrStats,
    scheduler_full_count: u32,
    config: EcuConfig,
    inj_idx: u8,
    ign_idx: u8,
    // Precomputed sequential orders (fixed-size, zero-extended)
    inj_seq: [Channel; 16],
    inj_seq_len: u8,
    ign_seq: [Channel; 16],
    ign_seq_len: u8,
    // Cam phase tracking
    cam_phase_known: bool,
    last_cam_edge_us: u32,
}

impl<T: TimeSource> EcuApp<T> {
    pub fn new(time_source: T) -> Self {
        let mut state = EcuState::new();
        state.init_linear_table();
        Self {
            decoder: TriggerDecoder::new(time_source),
            state,
            scheduler: Scheduler::new(),
            latch: OutputLatch::new(),
            stats: IsrStats::new(),
            scheduler_full_count: 0,
            config: EcuConfig::default_4c_batch_wasted(),
            inj_idx: 0,
            ign_idx: 0,
            inj_seq: [Channel::from_index(0); 16],
            inj_seq_len: 0,
            ign_seq: [Channel::from_index(0); 16],
            ign_seq_len: 0,
            cam_phase_known: false,
            last_cam_edge_us: 0,
        }
    }

    /// Create app with explicit configuration (modes, channels, firing order)
    pub fn new_with_config(time_source: T, config: EcuConfig) -> Self {
        let mut s = Self::new(time_source);
        s.config = config;
        // Precompute sequential orders for performance (zero-extended)
        // Injection order
        if matches!(s.config.injection_mode, InjectionMode::Sequential) {
            let len: u8;
            if !s.config.firing_order.is_empty() && s.config.outputs.inj_count > 0 {
                let max = core::cmp::min(
                    s.config.firing_order.len() as u8,
                    s.config.outputs.inj_count,
                ) as usize;
                for i in 0..max {
                    let cyl = s.config.firing_order[i] as usize;
                    // Convention: inj_channels[x] corresponds to cylinder x (1-based)
                    let idx = if cyl >= 1 && (cyl as u8) <= s.config.outputs.inj_count {
                        cyl - 1
                    } else {
                        i
                    };
                    s.inj_seq[i] = s.config.outputs.inj_channels[idx];
                }
                len = max as u8;
            } else {
                // Fallback: use inj_channels order
                let cnt = s.config.outputs.inj_count as usize;
                for i in 0..cnt {
                    s.inj_seq[i] = s.config.outputs.inj_channels[i];
                }
                len = s.config.outputs.inj_count;
            }
            s.inj_seq_len = len;
        }
        // Ignition order
        if matches!(s.config.ignition_mode, IgnitionMode::Sequential) {
            let len: u8;
            if !s.config.firing_order.is_empty() && s.config.outputs.ign_count > 0 {
                let max = core::cmp::min(
                    s.config.firing_order.len() as u8,
                    s.config.outputs.ign_count,
                ) as usize;
                for i in 0..max {
                    let cyl = s.config.firing_order[i] as usize;
                    let idx = if cyl >= 1 && (cyl as u8) <= s.config.outputs.ign_count {
                        cyl - 1
                    } else {
                        i
                    };
                    s.ign_seq[i] = s.config.outputs.ign_channels[idx];
                }
                len = max as u8;
            } else {
                let cnt = s.config.outputs.ign_count as usize;
                for i in 0..cnt {
                    s.ign_seq[i] = s.config.outputs.ign_channels[i];
                }
                len = s.config.outputs.ign_count;
            }
            s.ign_seq_len = len;
        }
        s
    }

    /// Feed a captured timestamp (microseconds) into the decoder,
    /// and schedule events as needed.
    pub fn on_timestamp(&mut self, ts_us: u32) {
        let t0 = self.decoder.time_source().micros();
        self.decoder.tooth_edge_with_timestamp(ts_us);

        if self.decoder.synced() {
            self.state.set_rpm(self.decoder.rpm().raw());
            self.state.set_synced(true);
            self.state.set_tooth_count(self.decoder.tooth());

            let now = ts_us;

            if self.decoder.tooth() == INJECTION_TOOTH {
                // Master safety check: synced, flood clear, sync shutdown, rev limiter
                let _ = update_flood_clear(
                    self.state.rpm(),
                    self.state.tps_percent(),
                    &mut self.state.flood_clear_state,
                );
                if self.state.should_inject_with_all_safety(0) {
                    self.schedule_injection(now);
                }
            }

            if self.decoder.tooth() == IGNITION_TOOTH {
                self.schedule_ignition(now);
            }

            // Periodic cam diagnostic check
            self.check_cam_diag(now);

            let t1 = self.decoder.time_source().micros();
            let dur = t1.wrapping_sub(t0);
            self.stats.update(dur);
            self.state.isr_stats = self.stats;
            self.state.refresh_snapshot();
        } else if self.state.synced() {
            self.on_sync_lost();
        }
    }

    /// Drive outputs for any due events or force safe state if latched
    pub fn drive_outputs(&mut self, _now: u32, outputs: &mut [&mut dyn OutputPin]) {
        if self.latch.is_latched() {
            apply_safe_state(outputs);
        } else {
            let now_ticks = self.decoder.time_source().ticks();
            self.scheduler.check_and_execute_ticks(now_ticks, outputs);
        }
    }

    /// Read current time from time source
    pub fn now(&self) -> u32 {
        self.decoder.time_source().micros()
    }

    pub fn stats(&self) -> IsrStats {
        self.stats
    }

    /// Borrow the live ECU state owned by the application.
    pub fn state(&self) -> &EcuState {
        &self.state
    }

    /// Borrow the live ECU state mutably.
    pub fn state_mut(&mut self) -> &mut EcuState {
        &mut self.state
    }

    /// Handle a detected sync loss by canceling pending events and forcing safe outputs.
    pub fn on_sync_lost(&mut self) {
        self.state.set_synced(false);
        self.scheduler.deactivate_all();
        self.latch.latch_off();
    }

    /// Notify the app of a cam edge (phase reference)
    /// For demo purposes, this resets sequential indices when synced.
    pub fn on_cam_edge(&mut self) {
        if self.decoder.synced() {
            self.cam_phase_known = true;
            self.inj_idx = 0;
            self.ign_idx = 0;
            let now = self.decoder.time_source().micros();
            self.last_cam_edge_us = now;
            if self.state.diag_cam.is_active() {
                let dur = now.wrapping_sub(self.state.diag_cam.start_us);
                self.state.diag_cam.total_us = self.state.diag_cam.total_us.saturating_add(dur);
                let context = self.state.config.cam_missing_timeout_ms as u32;
                let start_us = self.state.diag_cam.start_us;
                self.state.diag_log_mut().push(crate::diag::DiagEvent {
                    code: crate::diag::DiagCode::CamMissing,
                    timestamp: Micros::new(now),
                    source: crate::diag::DiagSource::Trigger,
                    context: Some(context),
                    start_us,
                    end_us: now,
                });
                self.state.diag_cam = crate::diag::DiagState::new();
            }
            // Mark cam present in diagnostics (clear cam missing if any)
            // Future: integrate with diag_cam using EcuState timestamps
        }
    }

    fn check_cam_diag(&mut self, now: u32) {
        if !self.config.has_cam {
            return;
        }
        let timeout_us = (self.state.config.cam_missing_timeout_ms as u32) * 1000;
        let missing = if self.last_cam_edge_us == 0 {
            now >= timeout_us
        } else {
            now.wrapping_sub(self.last_cam_edge_us) >= timeout_us
        };
        if missing && !self.state.diag_cam.is_active() {
            self.state.diag_cam.latch(Micros::new(now));
            self.state.diag_cam.start_us = now;
            self.state.diag_cam.in_range_since_us = 0;
            let timeout = timeout_us;
            self.state.diag_log_mut().push(crate::diag::DiagEvent {
                code: crate::diag::DiagCode::CamMissing,
                timestamp: Micros::new(now),
                source: crate::diag::DiagSource::Trigger,
                context: Some(timeout),
                start_us: now,
                end_us: 0,
            });
        }
    }

    fn schedule_injection(&mut self, now: u32) {
        let load = DEFAULT_LOAD_KPA;
        self.state.refresh_enrichments(
            now,
            self.state.clt_x10(),
            self.state.iat_x10(),
            self.state.tps_percent(),
            self.state.map_kpa_x10(),
        );
        self.state.apply_safety_torque_limits(now);
        self.state.update_torque(self.state.iat_x10() / 10);
        let torque_targets = self.state.get_torque_actuators();
        self.state
            .apply_torque_result(&crate::torque::arbiter::TorqueResult::new(
                torque_targets.fuel_mult_x100 as u16,
            ));
        let pw = self
            .state
            .final_pw(Rpm::new(self.state.rpm()), Kpa10::new(load))
            .raw();

        match self.config.injection_mode {
            InjectionMode::Batch => {
                // Fire all configured injectors
                let delay = now.wrapping_add(INJECTION_DELAY_US);
                let ts = self.decoder.time_source();
                let now_ticks = ts.ticks();
                let hz = ts.freq_hz();
                let dt0 = delay.wrapping_sub(now);
                let start_ticks =
                    now_ticks.wrapping_add(((dt0 as u64) * (hz as u64) / 1_000_000u64) as u32);
                let width_ticks = ((pw as u64) * (hz as u64) / 1_000_000u64) as u32;
                for i in 0..(self.config.outputs.inj_count as usize) {
                    let ch = self.config.outputs.inj_channels[i];
                    if !self.scheduler.schedule_ticks(start_ticks, ch, true)
                        || !self.scheduler.schedule_ticks(
                            start_ticks.wrapping_add(width_ticks),
                            ch,
                            false,
                        )
                    {
                        self.scheduler_full_count = self.scheduler_full_count.saturating_add(1);
                    }
                }
                if self.scheduler_full_count > 8 {
                    self.latch.latch_off();
                }
            }
            InjectionMode::Sequential => {
                // If cam is available but phase unknown, degrade to batch for safety
                if self.config.has_cam && !self.cam_phase_known {
                    let delay = now.wrapping_add(INJECTION_DELAY_US);
                    let ts = self.decoder.time_source();
                    let now_ticks = ts.ticks();
                    let hz = ts.freq_hz();
                    let dt0 = delay.wrapping_sub(now);
                    let start_ticks =
                        now_ticks.wrapping_add(((dt0 as u64) * (hz as u64) / 1_000_000u64) as u32);
                    let width_ticks = ((pw as u64) * (hz as u64) / 1_000_000u64) as u32;
                    for i in 0..(self.config.outputs.inj_count as usize) {
                        let ch = self.config.outputs.inj_channels[i];
                        if !self.scheduler.schedule_ticks(start_ticks, ch, true)
                            || !self.scheduler.schedule_ticks(
                                start_ticks.wrapping_add(width_ticks),
                                ch,
                                false,
                            )
                        {
                            self.scheduler_full_count = self.scheduler_full_count.saturating_add(1);
                        }
                    }
                    if self.scheduler_full_count > 8 {
                        self.latch.latch_off();
                    }
                    return;
                }

                let len = if self.inj_seq_len == 0 {
                    1
                } else {
                    self.inj_seq_len
                } as usize;
                let base = now.wrapping_add(INJECTION_DELAY_US);
                #[cfg(feature = "sched-angle-disable")]
                {
                    let rpm = core::cmp::max(1u32, self.state.rpm() as u32);
                    let rev_us = 60_000_000u32 / rpm;
                    let half_rev_us = rev_us / 2;
                    let idx0 = (self.inj_idx as usize) % len;
                    let ch0 = self.inj_seq[idx0];
                    let ang0_x10 = self.state.config.inj_angle_btdc_x10[idx0] as u32;
                    let start0 = base.wrapping_add((rev_us.saturating_mul(ang0_x10)) / 3600);
                    let idx1 = (self.inj_idx.wrapping_add(1) as usize) % len;
                    let ch1 = self.inj_seq[idx1];
                    let ang1_x10 = self.state.config.inj_angle_btdc_x10[idx1] as u32;
                    let start1 = base
                        .wrapping_add(half_rev_us)
                        .wrapping_add((rev_us.saturating_mul(ang1_x10)) / 3600);
                    let ts = self.decoder.time_source();
                    let now_ticks = ts.ticks();
                    let hz = ts.freq_hz();
                    self.scheduler.deactivate_channel_all(ch0);
                    self.scheduler.deactivate_channel_all(ch1);
                    let dt0 = start0.wrapping_sub(base);
                    let dt1 = start1.wrapping_sub(base);
                    let start0_ticks =
                        now_ticks.wrapping_add(((dt0 as u64) * (hz as u64) / 1_000_000u64) as u32);
                    let start1_ticks =
                        now_ticks.wrapping_add(((dt1 as u64) * (hz as u64) / 1_000_000u64) as u32);
                    let width_ticks = ((pw as u64) * (hz as u64) / 1_000_000u64) as u32;
                    let _ = self.scheduler.schedule_ticks(start0_ticks, ch0, true);
                    let _ = self.scheduler.schedule_ticks(
                        start0_ticks.wrapping_add(width_ticks),
                        ch0,
                        false,
                    );
                    let _ = self.scheduler.schedule_ticks(start1_ticks, ch1, true);
                    let _ = self.scheduler.schedule_ticks(
                        start1_ticks.wrapping_add(width_ticks),
                        ch1,
                        false,
                    );
                }
                #[cfg(not(feature = "sched-angle-disable"))]
                {
                    // Angle-accurate path
                    let modulo = if self.config.has_cam && self.cam_phase_known {
                        7200u16
                    } else {
                        3600u16
                    };
                    // First cyl
                    let idx0 = (self.inj_idx as usize) % len;
                    let ch0 = self.inj_seq[idx0];
                    let inj0_btdc_x10 = self.state.config.inj_angle_btdc_x10[idx0];
                    let tdc0_x10 = self.state.config.tdc_per_cyl_x10[idx0];
                    let target0 = (tdc0_x10
                        .wrapping_sub(inj0_btdc_x10)
                        .wrapping_add(self.state.config.tooth0_angle_x10))
                        % modulo;
                    let start0 = self
                        .decoder
                        .time_for_target_angle(base, target0, modulo)
                        .unwrap_or(base);
                    let ts = self.decoder.time_source();
                    let now_ticks = ts.ticks();
                    let hz = ts.freq_hz();
                    self.scheduler.deactivate_channel_all(ch0);
                    let dt0 = start0.wrapping_sub(base);
                    let start0_ticks =
                        now_ticks.wrapping_add(((dt0 as u64) * (hz as u64) / 1_000_000u64) as u32);
                    let width0_ticks = ((pw as u64) * (hz as u64) / 1_000_000u64) as u32;
                    let _ = self.scheduler.schedule_ticks(start0_ticks, ch0, true);
                    let _ = self.scheduler.schedule_ticks(
                        start0_ticks.wrapping_add(width0_ticks),
                        ch0,
                        false,
                    );
                    // Second cyl
                    let idx1 = (self.inj_idx.wrapping_add(1) as usize) % len;
                    let ch1 = self.inj_seq[idx1];
                    let inj1_btdc_x10 = self.state.config.inj_angle_btdc_x10[idx1];
                    let tdc1_x10 = self.state.config.tdc_per_cyl_x10[idx1];
                    let target1 = (tdc1_x10
                        .wrapping_sub(inj1_btdc_x10)
                        .wrapping_add(self.state.config.tooth0_angle_x10))
                        % modulo;
                    let start1 = self
                        .decoder
                        .time_for_target_angle(base, target1, modulo)
                        .unwrap_or(base);
                    self.scheduler.deactivate_channel_all(ch1);
                    let dt1 = start1.wrapping_sub(base);
                    let start1_ticks =
                        now_ticks.wrapping_add(((dt1 as u64) * (hz as u64) / 1_000_000u64) as u32);
                    let width1_ticks = ((pw as u64) * (hz as u64) / 1_000_000u64) as u32;
                    let _ = self.scheduler.schedule_ticks(start1_ticks, ch1, true);
                    let _ = self.scheduler.schedule_ticks(
                        start1_ticks.wrapping_add(width1_ticks),
                        ch1,
                        false,
                    );
                }
                // Advance by two for next revolution call
                self.inj_idx = self.inj_idx.wrapping_add(2) % (len as u8);
                if self.scheduler_full_count > 8 {
                    self.latch.latch_off();
                }
            }
        }
    }

    fn schedule_ignition(&mut self, now: u32) {
        match self.config.ignition_mode {
            IgnitionMode::Wasted => {
                // Two channels fire together (pair)
                let start = now;
                let end = now.wrapping_add(DWELL_TIME_US);
                for i in 0..(self.config.outputs.ign_count as usize).min(2) {
                    let ch = self.config.outputs.ign_channels[i];
                    if !self.scheduler.schedule(Micros::new(start), ch, true) {
                        self.scheduler_full_count = self.scheduler_full_count.saturating_add(1);
                    }
                    if !self.scheduler.schedule(Micros::new(end), ch, false) {
                        self.scheduler_full_count = self.scheduler_full_count.saturating_add(1);
                    }
                }
            }
            IgnitionMode::Sequential => {
                // Angle-based spark scheduling; dwell starts dwell_us before spark
                let len = if self.ign_seq_len == 0 {
                    1
                } else {
                    self.ign_seq_len
                } as usize;
                let idx = (self.ign_idx as usize) % len;
                let ch = self.ign_seq[idx];
                // Compute target spark angle = TDC(cyl) - advance
                let advance_deg = self
                    .state
                    .calculate_ignition_timing(self.state.rpm(), DEFAULT_LOAD_KPA)
                    as i32; // deg
                let advance_x10 = if advance_deg <= 0 {
                    0u16
                } else {
                    (advance_deg as u16) * 10
                };
                self.state
                    .set_commanded_advance_x10_output(advance_x10 as i16);
                let ts = self.decoder.time_source();
                let now_ticks = ts.ticks();
                let hz = ts.freq_hz();
                self.scheduler.deactivate_channel_all(ch);
                #[cfg(feature = "sched-angle-disable")]
                {
                    let rpm = core::cmp::max(1u32, self.state.rpm() as u32);
                    let rev_us = 60_000_000u32 / rpm;
                    let tdc_x10 = self.state.config.tdc_per_cyl_x10[idx];
                    let spark_ang_x10 = (tdc_x10
                        .wrapping_sub(advance_x10)
                        .wrapping_add(self.state.config.tooth0_angle_x10))
                        % 3600;
                    let spark_time =
                        now.wrapping_add((rev_us.saturating_mul(spark_ang_x10 as u32)) / 3600);
                    let dwell_us = self.state.calculate_dwell();
                    let mut dwell_start = spark_time.wrapping_sub(dwell_us);
                    if dwell_start < now {
                        dwell_start = now;
                    }
                    let start_ticks = now_ticks.wrapping_add(
                        ((dwell_start.wrapping_sub(now) as u64) * (hz as u64) / 1_000_000u64)
                            as u32,
                    );
                    let spark_ticks = now_ticks.wrapping_add(
                        ((spark_time.wrapping_sub(now) as u64) * (hz as u64) / 1_000_000u64) as u32,
                    );
                    let _ = self.scheduler.schedule_ticks(start_ticks, ch, true);
                    let _ = self.scheduler.schedule_ticks(spark_ticks, ch, false);
                }
                #[cfg(not(feature = "sched-angle-disable"))]
                {
                    let modulo = if self.config.has_cam && self.cam_phase_known {
                        7200u16
                    } else {
                        3600u16
                    };
                    let tdc_x10 = self.state.config.tdc_per_cyl_x10[idx];
                    let spark_ang_x10 = (tdc_x10
                        .wrapping_sub(advance_x10)
                        .wrapping_add(self.state.config.tooth0_angle_x10))
                        % modulo;
                    let spark_time = self
                        .decoder
                        .time_for_target_angle(now, spark_ang_x10, modulo)
                        .unwrap_or(now);
                    let dwell_us = self.state.calculate_dwell();
                    let mut dwell_start = spark_time.wrapping_sub(dwell_us);
                    if dwell_start < now {
                        dwell_start = now;
                    }
                    let start_ticks = now_ticks.wrapping_add(
                        ((dwell_start.wrapping_sub(now) as u64) * (hz as u64) / 1_000_000u64)
                            as u32,
                    );
                    let spark_ticks = now_ticks.wrapping_add(
                        ((spark_time.wrapping_sub(now) as u64) * (hz as u64) / 1_000_000u64) as u32,
                    );
                    let _ = self.scheduler.schedule_ticks(start_ticks, ch, true);
                    let _ = self.scheduler.schedule_ticks(spark_ticks, ch, false);
                }
                self.ign_idx = self.ign_idx.wrapping_add(1) % (len as u8);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::OutputChannels;

    #[derive(Copy, Clone)]
    struct MockTime;
    impl TimeSource for MockTime {
        fn micros(&self) -> u32 {
            0
        }
    }

    fn _count_active(app: &EcuApp<MockTime>) -> usize {
        app.scheduler.active_count()
    }

    #[test]
    fn test_wasted_spark_schedules_two_channels() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        let mut ign = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        ign[0] = Channel::IGN1;
        ign[1] = Channel::IGN2;
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 2,
            ign_channels: ign,
            ign_count: 2,
        };
        let cfg = EcuConfig {
            cylinders: 4,
            firing_order: &[1, 3, 4, 2],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);

        app.schedule_ignition(1000);

        // Expect 4 events: two start (both channels), two end (both channels)
        let mut s = app.scheduler;
        let mut starts = 0;
        let mut ends = 0;
        let mut start_channels = [0u8; 2];
        let mut end_channels = [0u8; 2];
        for e in s.events_mut().iter().filter(|e| e.is_active()) {
            if e.state() {
                start_channels[starts] = e.channel().as_u8();
                starts += 1;
            } else {
                end_channels[ends] = e.channel().as_u8();
                ends += 1;
            }
        }
        assert_eq!(starts, 2);
        assert_eq!(ends, 2);
        // Should be IGN1 and IGN2
        assert!(start_channels.contains(&Channel::IGN1.as_u8()));
        assert!(start_channels.contains(&Channel::IGN2.as_u8()));
        assert!(end_channels.contains(&Channel::IGN1.as_u8()));
        assert!(end_channels.contains(&Channel::IGN2.as_u8()));
    }

    #[test]
    fn test_sequential_ignition_rotates_channels() {
        let time = MockTime;
        let inj = [Channel::from_index(0); 16];
        let mut ign = [Channel::from_index(0); 16];
        ign[0] = Channel::IGN1;
        ign[1] = Channel::IGN2;
        ign[2] = Channel::from_index(4);
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 0,
            ign_channels: ign,
            ign_count: 3,
        };
        let cfg = EcuConfig {
            cylinders: 3,
            firing_order: &[1, 2, 3],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Sequential,
            has_cam: true,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);

        app.schedule_ignition(1000);
        app.schedule_ignition(2000);

        let mut s = app.scheduler;
        let actives: Vec<_> = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active())
            .map(|e| (e.time(), e.channel().as_u8(), e.state()))
            .collect();
        // Expect 4 events total: 2 per call
        assert_eq!(actives.len(), 4);
        let first_ch = actives[0].1; // first start
        let third_ch = actives[2].1; // start of second schedule
        assert_ne!(first_ch, third_ch, "Sequential should rotate channels");
    }

    #[test]
    fn test_commanded_advance_cache_tracks_ignition_timing() {
        let time = MockTime;
        let inj = [Channel::from_index(0); 16];
        let mut ign = [Channel::from_index(0); 16];
        ign[0] = Channel::IGN1;
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 0,
            ign_channels: ign,
            ign_count: 1,
        };
        let cfg = EcuConfig {
            cylinders: 4,
            firing_order: &[1, 3, 4, 2],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Sequential,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        app.state_mut().set_rpm(3000);

        app.schedule_ignition(1000);

        let advance_deg =
            app.state()
                .calculate_ignition_timing(app.state().rpm(), DEFAULT_LOAD_KPA) as i32;
        let expected = if advance_deg <= 0 {
            0
        } else {
            (advance_deg as i16) * 10
        };
        assert_eq!(app.state().commanded_advance_x10_output(), expected);
    }

    #[test]
    fn test_batch_injection_all_channels() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        inj[2] = Channel::from_index(5);
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 3,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 3,
            firing_order: &[1, 2, 3],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);

        app.schedule_injection(500);

        let mut s = app.scheduler;
        let starts = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.state())
            .count();
        let ends = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && !e.state())
            .count();
        assert_eq!(starts, 3);
        assert_eq!(ends, 3);
    }

    #[test]
    fn test_sequential_injection_rotates_channels() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 2,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 2,
            firing_order: &[1, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);

        app.schedule_injection(100);
        app.schedule_injection(200);

        let mut s = app.scheduler;
        let actives: Vec<_> = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.state())
            .map(|e| e.channel().as_u8())
            .collect();
        // Should have two start events on different channels
        assert!(actives.contains(&Channel::INJ1.as_u8()));
        assert!(actives.contains(&Channel::INJ2.as_u8()));
    }

    #[test]
    fn test_sequential_injection_reschedules_future() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 2,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 2,
            firing_order: &[1, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);

        app.state.set_rpm(2000);
        app.schedule_injection(1000);
        let mut s = app.scheduler;
        let _initial: std::vec::Vec<u32> = s
            .events_mut()
            .iter()
            .filter(|e| {
                e.is_active()
                    && e.state()
                    && (e.channel().as_u8() == Channel::INJ1.as_u8()
                        || e.channel().as_u8() == Channel::INJ2.as_u8())
            })
            .map(|e| e.time())
            .collect();

        // Change RPM significantly and reschedule
        app.scheduler = s; // put back
        app.state.set_rpm(6000);
        app.schedule_injection(2000);
        let mut s2 = app.scheduler;
        let inj1_starts = s2
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.state() && e.channel().as_u8() == Channel::INJ1.as_u8())
            .count();
        let inj2_starts = s2
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.state() && e.channel().as_u8() == Channel::INJ2.as_u8())
            .count();
        assert_eq!(inj1_starts, 1);
        assert_eq!(inj2_starts, 1);
        app.scheduler = s2; // restore for drop
    }

    #[test]
    fn test_sequential_cam_fallback_degrades_to_batch() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        inj[2] = Channel::from_index(2);
        inj[3] = Channel::from_index(3);
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 4,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 4,
            firing_order: &[1, 3, 4, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: true, // cam present but phase unknown
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        app.schedule_injection(1000);
        let mut s = app.scheduler;
        // Expect batch: all configured injectors have an ON event
        let mut channels_with_on = 0;
        for ch in 0..4u8 {
            let count = s
                .events_mut()
                .iter()
                .filter(|e| e.is_active() && e.channel().as_u8() == ch && e.state())
                .count();
            if count > 0 {
                channels_with_on += 1;
            }
        }
        assert_eq!(
            channels_with_on, 4,
            "should schedule all injectors in batch"
        );
        app.scheduler = s;
    }

    #[test]
    fn test_sequential_720_coverage_window_per_cycle() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        inj[2] = Channel::from_index(2);
        inj[3] = Channel::from_index(3);
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 4,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 4,
            firing_order: &[1, 3, 4, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: true,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: {
                let mut a = [0u16; 16];
                a[0] = 0;
                a[1] = 1800;
                a[2] = 3600;
                a[3] = 5400;
                a
            },
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        // Sync decoder and set cam phase known
        app.on_timestamp(1000);
        app.on_timestamp(3000);
        app.on_cam_edge();

        app.state.set_rpm(2000);
        // Two calls schedule two cylinders each, covering 4 per 720° window
        app.schedule_injection(10_000);
        app.schedule_injection(20_000);
        let mut s = app.scheduler;
        let mut ons = [0u32; 4];
        let mut offs = [0u32; 4];
        for e in s.events_mut().iter().filter(|e| e.is_active()) {
            let ch = e.channel().as_u8() as usize;
            if ch < 4 {
                if e.state() {
                    ons[ch] += 1;
                } else {
                    offs[ch] += 1;
                }
            }
        }
        for i in 0..4 {
            assert_eq!(ons[i], 1, "one ON per channel");
            assert_eq!(offs[i], 1, "one OFF per channel");
        }
        app.scheduler = s;
    }

    #[test]
    fn test_sequential_reschedule_stress_active_bound() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        inj[2] = Channel::from_index(2);
        inj[3] = Channel::from_index(3);
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 4,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 4,
            firing_order: &[1, 3, 4, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        // Ramp RPM and schedule repeatedly; ensure active events stay bounded
        for i in 0..500u32 {
            app.state.set_rpm(if i % 2 == 0 { 1200 } else { 6500 });
            app.schedule_injection(10_000 + i * 100);
            let active = app.scheduler.active_count();
            assert!(active <= 8, "active events bounded, got {active}");
        }
    }

    #[cfg(feature = "sched-angle-disable")]
    #[test]
    fn test_sched_simple_reschedule_clears_future() {
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 2,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 2,
            firing_order: &[1, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        app.state.set_rpm(1500);
        app.schedule_injection(10_000);
        // Trigger initial scheduling, then update RPM and schedule again
        let _ = app.scheduler.active_count();
        app.state.set_rpm(6000);
        app.schedule_injection(20_000);
        let mut s2 = app.scheduler;
        let inj1_on = s2
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::INJ1.as_u8() && e.state())
            .count();
        let inj2_on = s2
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::INJ2.as_u8() && e.state())
            .count();
        assert_eq!(inj1_on, 1);
        assert_eq!(inj2_on, 1);
        app.scheduler = s2;
    }
    #[test]
    fn test_sequential_ignition_reschedules_future() {
        let time = MockTime;
        let inj = [Channel::from_index(0); 16];
        let mut ign = [Channel::from_index(0); 16];
        ign[0] = Channel::IGN1;
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 0,
            ign_channels: ign,
            ign_count: 1,
        };
        let cfg = EcuConfig {
            cylinders: 1,
            firing_order: &[1],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Sequential,
            has_cam: true,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        app.state.set_rpm(2000);
        app.schedule_ignition(1000);
        let mut s = app.scheduler;
        let _initial: std::vec::Vec<u32> = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::IGN1.as_u8())
            .map(|e| e.time())
            .collect();
        app.scheduler = s;
        app.state.set_rpm(6000);
        app.schedule_ignition(2000);
        let mut s2 = app.scheduler;
        let on_count = s2
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::IGN1.as_u8() && e.state())
            .count();
        let off_count = s2
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::IGN1.as_u8() && !e.state())
            .count();
        assert_eq!(on_count, 1);
        assert_eq!(off_count, 1);
        app.scheduler = s2;
    }

    #[test]
    fn test_ignition_dwell_past_now_is_clamped() {
        let time = MockTime;
        let inj = [Channel::from_index(0); 16];
        let mut ign = [Channel::from_index(0); 16];
        ign[0] = Channel::IGN1;
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 0,
            ign_channels: ign,
            ign_count: 1,
        };
        let cfg = EcuConfig {
            cylinders: 1,
            firing_order: &[1],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Sequential,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        // Force large dwell by simulating low battery
        app.state.set_battery_voltage_mv(5000); // max dwell
                                                // No decoder rev estimate so spark_time defaults to now; dwell_start would be < now without clamp
        app.schedule_ignition(10_000);
        let mut s = app.scheduler;
        // Collect IGN1 events
        let mut times: Vec<(bool, u32)> = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::IGN1.as_u8())
            .map(|e| (e.state(), e.time()))
            .collect();
        assert_eq!(times.len(), 2, "one ON and one OFF expected");
        times.sort_by_key(|t| t.1);
        // ON first then OFF, and ON not scheduled far in past (relative to 0 ticks)
        assert!(times[0].0, "first should be ON");
        assert!(!times[1].0, "second should be OFF");
        assert!(times[0].1 <= times[1].1, "ON should be <= OFF time");
        app.scheduler = s;
    }

    #[cfg(feature = "sched-angle-disable")]
    #[test]
    fn test_sched_simple_injection_guard() {
        // Sanity guard: in sched-angle-disable mode, sequential injection schedules
        // two channels per revolution using the simple RPM model.
        let time = MockTime;
        let mut inj = [Channel::from_index(0); 16];
        inj[0] = Channel::INJ1;
        inj[1] = Channel::INJ2;
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 2,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 2,
            firing_order: &[1, 2],
            injection_mode: InjectionMode::Sequential,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: false,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        app.state.set_rpm(1500);
        app.schedule_injection(10_000);

        let mut s = app.scheduler;
        // Count start events for INJ1 and INJ2
        let inj1_on = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::INJ1.as_u8() && e.state())
            .count();
        let inj2_on = s
            .events_mut()
            .iter()
            .filter(|e| e.is_active() && e.channel().as_u8() == Channel::INJ2.as_u8() && e.state())
            .count();
        assert_eq!(inj1_on, 1, "INJ1 should have one start event");
        assert_eq!(inj2_on, 1, "INJ2 should have one start event");
        app.scheduler = s; // restore
    }

    #[test]
    fn test_cam_missing_diag_and_clear() {
        // Fixed time source returning > timeout
        struct FixedTime;
        impl TimeSource for FixedTime {
            fn micros(&self) -> u32 {
                600_000
            }
        }
        let time = FixedTime;
        let inj = [Channel::from_index(0); 16];
        let ign = [Channel::from_index(0); 16];
        let outputs = OutputChannels {
            inj_channels: inj,
            inj_count: 0,
            ign_channels: ign,
            ign_count: 0,
        };
        let cfg = EcuConfig {
            cylinders: 1,
            firing_order: &[1],
            injection_mode: InjectionMode::Batch,
            ignition_mode: IgnitionMode::Wasted,
            has_cam: true,
            outputs,
            inj_angle_btdc_x10: [0; 16],
            tdc_per_cyl_x10: [0; 16],
            tooth0_angle_x10: 0,
        };
        let mut app = EcuApp::new_with_config(time, cfg);
        app.state.config.cam_missing_timeout_ms = 500;
        // Seed decoder sync (two edges with a gap)
        app.on_timestamp(1_000);
        app.on_timestamp(3_000);
        // No flag before timeout
        assert!(!app.state.diag_cam.is_active());
        // Directly invoke cam diagnostic check at time past timeout
        app.check_cam_diag(600_000);
        assert!(
            app.state.diag_cam.is_active(),
            "cam missing should be active"
        );
        // Cam edge clears and logs event
        app.on_cam_edge();
        assert!(
            !app.state.diag_cam.is_active(),
            "cam missing cleared after edge"
        );
        assert!(
            app.state.diag_log().events.iter().any(
                |e| matches!(e, Some(ev) if matches!(ev.code, crate::diag::DiagCode::CamMissing))
            ),
            "expected a CamMissing event in log"
        );
    }
}
