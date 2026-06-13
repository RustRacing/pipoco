#![cfg_attr(not(test), no_std)]

pub mod output_capture;
pub mod plant;
pub mod trace_replay;
pub mod trigger_pattern;

use ecu_domain::{Degrees10, Kpa10, Micros, Rpm};
use ecu_runtime::{
    ingress::AuthorityStepInputs, CamObservation, ControlInputs, DecoderObservation, EngineRuntime,
    StepResult, TriggerObservation,
};

/// Board-like simulation events that feed the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimEvent {
    TriggerEdge {
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        synced: bool,
    },
    CamEdge {
        at_us: Micros,
        cam_seen: bool,
    },
    SensorFrame {
        at_us: Micros,
        rpm: Rpm,
        load_kpa10: Kpa10,
        angle_x10: Degrees10,
    },
    Tick {
        now_us: Micros,
        control: ControlInputs,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueResult {
    Enqueued,
    Coalesced,
    Overflowed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QueueOverflow {
    FastFull,
    SlowFull,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimulationStep {
    pub event: SimEvent,
    pub result: Option<StepResult>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SimulationHarness<const FAST: usize, const SLOW: usize> {
    runtime: EngineRuntime,
    queues: SimQueues<FAST, SLOW>,
    pending_now_us: Micros,
    pending_rpm: u32,
    pending_load_kpa10: u32,
    pending_angle_x10: i32,
    pending_launch_armed: bool,
    pending_flat_shift_armed: bool,
    last_result: Option<StepResult>,
}

impl<const FAST: usize, const SLOW: usize> SimulationHarness<FAST, SLOW> {
    pub fn new(runtime: EngineRuntime) -> Self {
        Self {
            runtime,
            queues: SimQueues::new(),
            pending_now_us: Micros::new(0),
            pending_rpm: 0,
            pending_load_kpa10: 0,
            pending_angle_x10: 0,
            pending_launch_armed: false,
            pending_flat_shift_armed: false,
            last_result: None,
        }
    }

    pub fn runtime(&self) -> &EngineRuntime {
        &self.runtime
    }

    pub fn last_result(&self) -> Option<StepResult> {
        self.last_result
    }

    pub fn enqueue(&mut self, event: SimEvent) -> Result<QueueResult, QueueOverflow> {
        match event {
            SimEvent::Tick { .. } => self.queues.slow.push_slow(event),
            _ => self.queues.fast.push_coalescing_fast(event),
        }
    }

    pub fn trigger_edge(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        synced: bool,
    ) -> Result<QueueResult, QueueOverflow> {
        self.enqueue(SimEvent::TriggerEdge {
            at_us,
            rpm,
            angle_x10,
            synced,
        })
    }

    pub fn cam_edge(
        &mut self,
        at_us: Micros,
        cam_seen: bool,
    ) -> Result<QueueResult, QueueOverflow> {
        self.enqueue(SimEvent::CamEdge { at_us, cam_seen })
    }

    pub fn sensor_frame(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        load_kpa10: Kpa10,
        angle_x10: Degrees10,
    ) -> Result<QueueResult, QueueOverflow> {
        self.enqueue(SimEvent::SensorFrame {
            at_us,
            rpm,
            load_kpa10,
            angle_x10,
        })
    }

    pub fn tick(
        &mut self,
        now_us: Micros,
        control: ControlInputs,
    ) -> Result<QueueResult, QueueOverflow> {
        self.enqueue(SimEvent::Tick { now_us, control })
    }

    pub fn drain_one(&mut self) -> Option<SimulationStep> {
        let event = self.queues.pop_fast().or_else(|| self.queues.pop_slow())?;
        let result = self.apply_event(event);
        Some(SimulationStep { event, result })
    }

    pub fn drain_until_idle(&mut self) -> usize {
        let mut processed = 0usize;
        let mut loop_count = 0;
        while self.drain_one().is_some() {
            processed += 1;
            loop_count += 1;
            if loop_count > 70 {
                // Safety valve: prevent runaway loop in malformed input
                break;
            }
        }
        processed
    }

    fn apply_event(&mut self, event: SimEvent) -> Option<StepResult> {
        match event {
            SimEvent::TriggerEdge {
                at_us,
                rpm,
                angle_x10,
                synced,
            } => {
                self.pending_now_us = at_us;
                self.pending_rpm = u32::from(rpm.get());
                self.pending_angle_x10 = i32::from(angle_x10.get());
                self.runtime
                    .apply_decoder_observation(DecoderObservation::Trigger(TriggerObservation {
                        at_us,
                        rpm,
                        angle_x10,
                        synced,
                    }));
                None
            }
            SimEvent::CamEdge { at_us, cam_seen } => {
                self.pending_now_us = at_us;
                self.runtime
                    .apply_decoder_observation(DecoderObservation::Cam(CamObservation {
                        at_us,
                        cam_seen,
                    }));
                None
            }
            SimEvent::SensorFrame {
                at_us,
                rpm,
                load_kpa10,
                angle_x10,
            } => {
                self.pending_now_us = at_us;
                self.pending_rpm = u32::from(rpm.get());
                self.pending_load_kpa10 = u32::from(load_kpa10.get());
                self.pending_angle_x10 = i32::from(angle_x10.get());
                self.runtime.apply_sensor_sample(rpm, load_kpa10, angle_x10);
                None
            }
            SimEvent::Tick { now_us, control } => {
                self.pending_now_us = now_us;
                let inputs = AuthorityStepInputs::new(
                    self.pending_now_us,
                    self.pending_rpm,
                    self.pending_load_kpa10,
                    self.pending_angle_x10,
                    self.runtime.engine_time_authority(),
                    self.pending_launch_armed,
                    self.pending_flat_shift_armed,
                );
                let result = self.runtime.step_with_authority(inputs, control);
                self.last_result = Some(result);
                Some(result)
            }
        }
    }
}

impl<const FAST: usize, const SLOW: usize> Default for SimulationHarness<FAST, SLOW> {
    fn default() -> Self {
        Self::new(EngineRuntime::new())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct SimQueues<const FAST: usize, const SLOW: usize> {
    fast: Queue<FAST>,
    slow: Queue<SLOW>,
}

impl<const FAST: usize, const SLOW: usize> SimQueues<FAST, SLOW> {
    const fn new() -> Self {
        Self {
            fast: Queue::new(),
            slow: Queue::new(),
        }
    }

    fn pop_fast(&mut self) -> Option<SimEvent> {
        self.fast.pop()
    }

    fn pop_slow(&mut self) -> Option<SimEvent> {
        self.slow.pop()
    }
}

impl<const FAST: usize, const SLOW: usize> Default for SimQueues<FAST, SLOW> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct Queue<const N: usize> {
    buf: [Option<SimEvent>; N],
    head: usize,
    len: usize,
}

impl<const N: usize> Queue<N> {
    const fn new() -> Self {
        Self {
            buf: [None; N],
            head: 0,
            len: 0,
        }
    }

    fn pop(&mut self) -> Option<SimEvent> {
        if self.len == 0 {
            return None;
        }
        let item = self.buf[self.head].take();
        self.head = (self.head + 1) % N.max(1);
        self.len -= 1;
        item
    }

    fn push_fast(&mut self, item: SimEvent) -> Result<QueueResult, QueueOverflow> {
        if self.len == N {
            return Err(QueueOverflow::FastFull);
        }
        let tail = (self.head + self.len) % N.max(1);
        self.buf[tail] = Some(item);
        self.len += 1;
        Ok(QueueResult::Enqueued)
    }

    fn push_slow(&mut self, item: SimEvent) -> Result<QueueResult, QueueOverflow> {
        if self.len == N {
            return Err(QueueOverflow::SlowFull);
        }
        let tail = (self.head + self.len) % N.max(1);
        self.buf[tail] = Some(item);
        self.len += 1;
        Ok(QueueResult::Enqueued)
    }

    fn push_coalescing_fast(&mut self, item: SimEvent) -> Result<QueueResult, QueueOverflow> {
        if matches!(item, SimEvent::Tick { .. }) {
            return self.push_fast(item);
        }

        let key = core::mem::discriminant(&item);
        let mut idx = 0usize;
        while idx < self.len {
            let slot = (self.head + idx) % N.max(1);
            if self.buf[slot].map(|existing| core::mem::discriminant(&existing)) == Some(key) {
                self.buf[slot] = Some(item);
                return Ok(QueueResult::Coalesced);
            }
            idx += 1;
        }

        self.push_fast(item)
    }
}

#[cfg(test)]
mod tests;
