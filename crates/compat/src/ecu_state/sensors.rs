use super::EcuState;
use crate::diag;
use crate::units::{Kpa10, Micros};

impl EcuState {
    /// Clamp sensor values, update diag states, and set/clear emergency mode.
    /// Returns (clamped_map_kpa_x10, clamped_tps_percent).
    pub fn process_sensor_update(
        &mut self,
        now_us: Micros,
        raw_map_kpa_x10: Kpa10,
        raw_tps_percent: u8,
    ) -> (Kpa10, u8) {
        let lim = self.config.sensors_limits;
        let now_us = now_us.raw();
        let raw_map_kpa_x10 = raw_map_kpa_x10.raw();
        let map = raw_map_kpa_x10.clamp(lim.map_min_kpa_x10, lim.map_max_kpa_x10);
        let tps = raw_tps_percent.clamp(lim.tps_min_percent, lim.tps_max_percent);

        // MAP diag
        let map_oob =
            raw_map_kpa_x10 < lim.map_min_kpa_x10 || raw_map_kpa_x10 > lim.map_max_kpa_x10;
        if map_oob {
            if !self.diag_map.is_active() {
                self.diag_map.latch(Micros::new(now_us));
                self.diag_map.start_us = now_us;
                self.diag_map.in_range_since_us = 0;
                if self.emergency_trigger_map_oob() {
                    self.set_emergency_mode(true);
                }
            }
        } else if self.diag_map.is_active() {
            if self.diag_map.in_range_since_us == 0 {
                self.diag_map.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_map.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_map.start_us);
                self.diag_map.total_us = self.diag_map.total_us.saturating_add(dur);
                let start_us = self.diag_map.start_us;
                self.diag_log_mut().push(diag::DiagEvent {
                    code: diag::DiagCode::MapRange,
                    timestamp: Micros::new(now_us),
                    source: diag::DiagSource::Sensor,
                    context: Some(raw_map_kpa_x10 as u32),
                    start_us,
                    end_us: now_us,
                });
                self.diag_map.clear(Micros::new(now_us));
                self.diag_map = diag::DiagState::new();
            }
        }

        // TPS diag
        let tps_oob =
            raw_tps_percent < lim.tps_min_percent || raw_tps_percent > lim.tps_max_percent;
        if tps_oob {
            if !self.diag_tps.is_active() {
                self.diag_tps.latch(Micros::new(now_us));
                self.diag_tps.start_us = now_us;
                self.diag_tps.in_range_since_us = 0;
                if self.emergency_trigger_tps_oob() {
                    self.set_emergency_mode(true);
                }
            }
        } else if self.diag_tps.is_active() {
            if self.diag_tps.in_range_since_us == 0 {
                self.diag_tps.in_range_since_us = now_us;
            }
            let clear_time_us = (lim.clear_time_s as u32) * 1_000_000;
            if now_us.wrapping_sub(self.diag_tps.in_range_since_us) >= clear_time_us {
                let dur = now_us.wrapping_sub(self.diag_tps.start_us);
                self.diag_tps.total_us = self.diag_tps.total_us.saturating_add(dur);
                let start_us = self.diag_tps.start_us;
                self.diag_log_mut().push(diag::DiagEvent {
                    code: diag::DiagCode::TpsRange,
                    timestamp: Micros::new(now_us),
                    source: diag::DiagSource::Sensor,
                    context: Some(raw_tps_percent as u32),
                    start_us,
                    end_us: now_us,
                });
                self.diag_tps.clear(Micros::new(now_us));
                self.diag_tps = diag::DiagState::new();
            }
        }

        // Clear emergency mode if triggers inactive
        if self.emergency_mode() {
            let map_emerg_active = self.emergency_trigger_map_oob() && self.diag_map.is_active();
            let tps_emerg_active = self.emergency_trigger_tps_oob() && self.diag_tps.is_active();
            if !(map_emerg_active || tps_emerg_active) {
                self.set_emergency_mode(false);
            }
        }

        self.set_map_kpa_x10(map);
        self.set_tps_percent(tps);
        (Kpa10::new(map), tps)
    }
}
