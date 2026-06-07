use ecu_domain::{Degrees10, Kpa10, Micros, Rpm};
use ecu_sim::QueueOverflow;

use crate::{EcuSimSensorFrame, EcuSimStatus, CRANK_TEETH_PER_CYCLE, DEG10_PER_TOOTH};

use super::model::{
    rpm_from_tooth_period, EcuSimHandle, MIN_SYNC_LOSS_TIMEOUT_US, SYNC_LOSS_PERIOD_MULTIPLIER,
};

impl EcuSimHandle {
    pub(crate) fn set_time(&mut self, now_us: u32) -> EcuSimStatus {
        match self.require_init() {
            Ok(()) => {
                self.now_us = now_us;
                EcuSimStatus::Ok
            }
            Err(status) => status,
        }
    }

    pub(crate) fn set_sensors(&mut self, frame: EcuSimSensorFrame) -> EcuSimStatus {
        if let Err(status) = self.require_init() {
            return status;
        }
        self.now_us = frame.now_us;
        self.sensors = frame;
        let sync_status = self.maybe_mark_sync_lost(frame.now_us);
        if sync_status != EcuSimStatus::Ok {
            return sync_status;
        }
        let status = self.enqueue_sensor_frame(frame.now_us);
        if status != EcuSimStatus::Ok {
            return status;
        }
        self.drain_sim_steps()
    }

    pub(crate) fn on_crank_edge(&mut self, ts_us: u32) -> EcuSimStatus {
        if let Err(status) = self.require_init() {
            return status;
        }
        if let Some(last) = self.last_crank_edge_us {
            if ts_us <= last {
                return EcuSimStatus::ErrInvalid;
            }
            let period_us = ts_us - last;
            self.last_crank_period_us = Some(period_us);
            self.rpm = rpm_from_tooth_period(period_us);
            self.synced = true;
        }

        self.last_crank_edge_us = Some(ts_us);
        self.now_us = ts_us;
        self.tooth = ((u16::from(self.tooth) + 1) % CRANK_TEETH_PER_CYCLE) as u8;
        self.angle_x10 = i16::from(self.tooth) * DEG10_PER_TOOTH;

        let trigger_status = match self.sim.trigger_edge(
            Micros::new(ts_us),
            Rpm::new(self.rpm),
            Degrees10::new(self.angle_x10),
            self.synced,
        ) {
            Ok(_) => EcuSimStatus::Ok,
            Err(_) => EcuSimStatus::ErrEventOverflow,
        };
        if trigger_status != EcuSimStatus::Ok {
            return trigger_status;
        }

        if !self.cfg.has_cam && self.synced {
            match self.sim.cam_edge(Micros::new(ts_us), true) {
                Ok(_) => {}
                Err(_) => return EcuSimStatus::ErrEventOverflow,
            }
        }
        self.drain_sim_steps()
    }

    pub(crate) fn on_cam_edge(&mut self, ts_us: u32) -> EcuSimStatus {
        if let Err(status) = self.require_init() {
            return status;
        }
        if !self.cfg.has_cam {
            return EcuSimStatus::ErrInvalid;
        }
        self.now_us = ts_us;
        match self.sim.cam_edge(Micros::new(ts_us), true) {
            Ok(_) => self.drain_sim_steps(),
            Err(_) => EcuSimStatus::ErrEventOverflow,
        }
    }

    pub(crate) fn step(&mut self, now_us: u32) -> EcuSimStatus {
        if let Err(status) = self.require_init() {
            return status;
        }

        self.now_us = now_us;
        let sync_status = self.maybe_mark_sync_lost(now_us);
        if sync_status != EcuSimStatus::Ok {
            return sync_status;
        }
        let sensor_status = self.enqueue_sensor_frame(now_us);
        if sensor_status != EcuSimStatus::Ok {
            return sensor_status;
        }
        match self
            .sim
            .tick(Micros::new(now_us), self.control_inputs(now_us))
        {
            Ok(_) => {
                let status = self.drain_sim_steps();
                if status != EcuSimStatus::Ok {
                    status
                } else if self.overflow_latched {
                    EcuSimStatus::ErrEventOverflow
                } else {
                    EcuSimStatus::Ok
                }
            }
            Err(QueueOverflow::FastFull | QueueOverflow::SlowFull) => {
                EcuSimStatus::ErrEventOverflow
            }
        }
    }

    fn enqueue_sensor_frame(&mut self, at_us: u32) -> EcuSimStatus {
        match self.sim.sensor_frame(
            Micros::new(at_us),
            Rpm::new(self.rpm),
            Kpa10::new(self.sensors.map_kpa10),
            Degrees10::new(self.angle_x10),
        ) {
            Ok(_) => EcuSimStatus::Ok,
            Err(_) => EcuSimStatus::ErrEventOverflow,
        }
    }

    fn maybe_mark_sync_lost(&mut self, now_us: u32) -> EcuSimStatus {
        if !self.synced {
            return EcuSimStatus::Ok;
        }

        let Some(last_edge_us) = self.last_crank_edge_us else {
            return EcuSimStatus::Ok;
        };
        let Some(elapsed_us) = now_us.checked_sub(last_edge_us) else {
            return EcuSimStatus::Ok;
        };

        let dynamic_timeout = self
            .last_crank_period_us
            .and_then(|period| period.checked_mul(SYNC_LOSS_PERIOD_MULTIPLIER))
            .unwrap_or(MIN_SYNC_LOSS_TIMEOUT_US)
            .max(MIN_SYNC_LOSS_TIMEOUT_US);
        if elapsed_us <= dynamic_timeout {
            return EcuSimStatus::Ok;
        }

        self.synced = false;
        self.rpm = 0;
        self.last_crank_edge_us = None;
        self.last_crank_period_us = None;

        let trigger_status = match self.sim.trigger_edge(
            Micros::new(now_us),
            Rpm::new(0),
            Degrees10::new(self.angle_x10),
            false,
        ) {
            Ok(_) => EcuSimStatus::Ok,
            Err(_) => EcuSimStatus::ErrEventOverflow,
        };
        if trigger_status != EcuSimStatus::Ok {
            return trigger_status;
        }

        match self.sim.cam_edge(Micros::new(now_us), false) {
            Ok(_) => self.drain_sim_steps(),
            Err(_) => EcuSimStatus::ErrEventOverflow,
        }
    }
}
