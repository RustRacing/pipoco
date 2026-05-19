#![cfg_attr(not(test), no_std)]

pub mod output_capture;
pub mod plant;
pub mod trace_replay;
pub mod trigger_pattern;

use ecu_domain::{Degrees10, Kpa10, Micros, Rpm};
use ecu_runtime::{
    CamObservation, ControlInputs, DecoderObservation, EngineRuntime, StepInputs, StepResult,
    TriggerObservation,
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
    pending_inputs: StepInputs,
    last_result: Option<StepResult>,
}

impl<const FAST: usize, const SLOW: usize> SimulationHarness<FAST, SLOW> {
    pub fn new(runtime: EngineRuntime) -> Self {
        Self {
            runtime,
            queues: SimQueues::new(),
            pending_inputs: StepInputs {
                now_us: Micros::new(0),
                rpm: 0,
                load_kpa10: 0,
                angle_x10: 0,
                trigger_synced: false,
                cam_seen: false,
                flat_shift_armed: false,
                launch_armed: false,
            },
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
                self.pending_inputs.now_us = at_us;
                self.pending_inputs.rpm = u32::from(rpm.get());
                self.pending_inputs.angle_x10 = i32::from(angle_x10.get());
                self.pending_inputs.trigger_synced = synced;
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
                self.pending_inputs.now_us = at_us;
                self.pending_inputs.cam_seen = cam_seen;
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
                self.pending_inputs.now_us = at_us;
                self.pending_inputs.rpm = u32::from(rpm.get());
                self.pending_inputs.load_kpa10 = u32::from(load_kpa10.get());
                self.pending_inputs.angle_x10 = i32::from(angle_x10.get());
                self.runtime.apply_sensor_sample(rpm, load_kpa10, angle_x10);
                None
            }
            SimEvent::Tick { now_us, control } => {
                self.pending_inputs.now_us = now_us;
                let result = self.runtime.step(self.pending_inputs, control);
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
mod tests {
    use super::*;
    use ecu_domain::{FaultCode, FaultSeverity, Lambda100};
    use ecu_runtime::{EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs};

    fn control_inputs() -> ControlInputs {
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(1_000),
                clt_c: 40,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 10,
                mapdot_kpa_s: 10,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(3000)),
        }
    }

    #[test]
    fn harness_applies_board_like_events_to_runtime() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

        assert!(matches!(
            sim.trigger_edge(Micros::new(10), Rpm::new(1200), Degrees10::new(45), false),
            Ok(QueueResult::Enqueued)
        ));
        assert!(matches!(
            sim.cam_edge(Micros::new(12), true),
            Ok(QueueResult::Enqueued)
        ));
        assert!(matches!(
            sim.sensor_frame(
                Micros::new(14),
                Rpm::new(1800),
                Kpa10::new(600),
                Degrees10::new(60)
            ),
            Ok(QueueResult::Enqueued)
        ));
        assert!(matches!(
            sim.tick(Micros::new(20), control_inputs()),
            Ok(QueueResult::Enqueued)
        ));

        assert_eq!(sim.drain_until_idle(), 4);
        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.rpm.get(), 1800);
        assert_eq!(snapshot.engine.load_kpa10.get(), 600);
        assert_eq!(snapshot.engine.angle_x10.get(), 60);
        assert!(sim.last_result().is_some());
    }

    #[test]
    fn fast_events_coalesce_by_kind() {
        let mut sim: SimulationHarness<2, 1> = SimulationHarness::default();

        assert!(matches!(
            sim.trigger_edge(Micros::new(1), Rpm::new(1000), Degrees10::new(20), false),
            Ok(QueueResult::Enqueued)
        ));
        assert!(matches!(
            sim.trigger_edge(Micros::new(2), Rpm::new(1500), Degrees10::new(40), true),
            Ok(QueueResult::Coalesced)
        ));
        assert_eq!(sim.drain_until_idle(), 1);
        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.rpm.get(), 1500);
        assert_eq!(snapshot.engine.angle_x10.get(), 40);
    }

    #[test]
    fn cold_start_scenario_runs_through_public_runtime_api() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

        sim.trigger_edge(Micros::new(5), Rpm::new(650), Degrees10::new(10), false)
            .unwrap();
        sim.cam_edge(Micros::new(6), false).unwrap();
        sim.sensor_frame(
            Micros::new(8),
            Rpm::new(650),
            Kpa10::new(250),
            Degrees10::new(10),
        )
        .unwrap();
        sim.tick(Micros::new(10), control_inputs()).unwrap();

        sim.drain_until_idle();

        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.phase, ecu_domain::EnginePhase::Cranking);
        assert_eq!(snapshot.engine.sync, ecu_domain::SyncState::Unsynced);
        assert!(matches!(
            sim.last_result().unwrap().actions.iter().next(),
            Some(ecu_runtime::Action::Idle)
        ));
    }

    #[test]
    fn hot_start_scenario_arms_scheduler_and_reaches_closed_loop() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

        sim.trigger_edge(Micros::new(20), Rpm::new(1800), Degrees10::new(30), true)
            .unwrap();
        sim.cam_edge(Micros::new(21), true).unwrap();
        sim.sensor_frame(
            Micros::new(22),
            Rpm::new(1800),
            Kpa10::new(600),
            Degrees10::new(30),
        )
        .unwrap();
        sim.tick(Micros::new(24), control_inputs()).unwrap();

        sim.drain_until_idle();

        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.phase, ecu_domain::EnginePhase::Running);
        assert_eq!(snapshot.engine.mode, ecu_domain::ControlMode::ClosedLoop);
        assert!(matches!(
            sim.last_result().unwrap().actions.iter().next(),
            Some(ecu_runtime::Action::ArmScheduler { .. })
        ));
    }

    #[test]
    fn acceleration_scenario_updates_runtime_through_ticks() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

        sim.trigger_edge(Micros::new(30), Rpm::new(1200), Degrees10::new(15), true)
            .unwrap();
        sim.sensor_frame(
            Micros::new(31),
            Rpm::new(1200),
            Kpa10::new(350),
            Degrees10::new(15),
        )
        .unwrap();
        sim.tick(Micros::new(32), control_inputs()).unwrap();
        sim.sensor_frame(
            Micros::new(40),
            Rpm::new(2200),
            Kpa10::new(520),
            Degrees10::new(18),
        )
        .unwrap();
        sim.tick(Micros::new(42), control_inputs()).unwrap();

        sim.drain_until_idle();

        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.rpm.get(), 2200);
        assert_eq!(snapshot.engine.load_kpa10.get(), 520);
        assert_eq!(snapshot.engine.angle_x10.get(), 18);
        assert!(sim.last_result().is_some());
    }

    #[test]
    fn sync_loss_and_recovery_scenario_cancels_then_rearms_outputs() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

        sim.trigger_edge(Micros::new(50), Rpm::new(1500), Degrees10::new(20), true)
            .unwrap();
        sim.cam_edge(Micros::new(51), true).unwrap();
        sim.sensor_frame(
            Micros::new(52),
            Rpm::new(1500),
            Kpa10::new(450),
            Degrees10::new(20),
        )
        .unwrap();
        sim.tick(Micros::new(54), control_inputs()).unwrap();
        sim.drain_until_idle();
        assert!(matches!(
            sim.last_result().unwrap().actions.iter().next(),
            Some(ecu_runtime::Action::ArmScheduler { .. })
        ));

        sim.trigger_edge(Micros::new(60), Rpm::new(0), Degrees10::new(20), false)
            .unwrap();
        sim.cam_edge(Micros::new(61), false).unwrap();
        sim.sensor_frame(
            Micros::new(62),
            Rpm::new(0),
            Kpa10::new(0),
            Degrees10::new(20),
        )
        .unwrap();
        sim.tick(Micros::new(64), control_inputs()).unwrap();
        sim.drain_until_idle();
        assert!(sim
            .last_result()
            .unwrap()
            .actions
            .iter()
            .any(|action| matches!(action, ecu_runtime::Action::CancelScheduler(_))));
        assert_ne!(
            sim.runtime().scheduler_state().mode(),
            ecu_scheduler::SchedulerMode::Armed
        );

        sim.trigger_edge(Micros::new(70), Rpm::new(1500), Degrees10::new(20), true)
            .unwrap();
        sim.cam_edge(Micros::new(71), true).unwrap();
        sim.tick(Micros::new(74), control_inputs()).unwrap();
        sim.drain_until_idle();
        assert!(matches!(
            sim.last_result().unwrap().actions.iter().next(),
            Some(ecu_runtime::Action::ArmScheduler { .. })
        ));
    }

    #[test]
    fn sensor_fault_scenario_surfaces_limp_home_state() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
        sim.runtime.set_fault_state(
            FaultCode::SensorOutOfRange,
            FaultSeverity::Warning,
            ecu_domain::CancelReason::Manual,
        );

        sim.trigger_edge(Micros::new(90), Rpm::new(900), Degrees10::new(5), true)
            .unwrap();
        sim.cam_edge(Micros::new(91), true).unwrap();
        sim.sensor_frame(
            Micros::new(92),
            Rpm::new(900),
            Kpa10::new(300),
            Degrees10::new(5),
        )
        .unwrap();
        sim.tick(Micros::new(94), control_inputs()).unwrap();
        sim.drain_until_idle();

        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.faults.fault, FaultCode::SensorOutOfRange);
        assert_eq!(snapshot.faults.severity, FaultSeverity::Warning);
        assert_eq!(snapshot.engine.mode, ecu_domain::ControlMode::LimpHome);
        assert!(sim
            .last_result()
            .unwrap()
            .actions
            .iter()
            .any(|action| match action {
                ecu_runtime::Action::ApplyAux(commands) => commands.len() == 1,
                _ => false,
            }));
    }

    #[test]
    fn sync_loss_cancels_pending_outputs_through_scheduler() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();

        sim.trigger_edge(Micros::new(100), Rpm::new(1600), Degrees10::new(22), true)
            .unwrap();
        sim.cam_edge(Micros::new(101), true).unwrap();
        sim.sensor_frame(
            Micros::new(102),
            Rpm::new(1600),
            Kpa10::new(500),
            Degrees10::new(22),
        )
        .unwrap();
        sim.tick(Micros::new(104), control_inputs()).unwrap();
        sim.drain_until_idle();
        assert_eq!(
            sim.runtime().scheduler_state().mode(),
            ecu_scheduler::SchedulerMode::Armed
        );

        sim.trigger_edge(Micros::new(110), Rpm::new(0), Degrees10::new(22), false)
            .unwrap();
        sim.cam_edge(Micros::new(111), false).unwrap();
        sim.sensor_frame(
            Micros::new(112),
            Rpm::new(0),
            Kpa10::new(0),
            Degrees10::new(22),
        )
        .unwrap();
        sim.tick(Micros::new(114), control_inputs()).unwrap();
        sim.drain_until_idle();

        assert_ne!(
            sim.runtime().scheduler_state().mode(),
            ecu_scheduler::SchedulerMode::Armed
        );
        assert!(sim
            .last_result()
            .unwrap()
            .actions
            .iter()
            .any(|action| matches!(action, ecu_runtime::Action::CancelScheduler(_))));
    }

    #[test]
    fn degraded_and_substituted_inputs_remain_visible_in_snapshot_and_faults() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
        sim.runtime.set_fault_state(
            FaultCode::SensorOutOfRange,
            FaultSeverity::Warning,
            ecu_domain::CancelReason::Manual,
        );

        sim.trigger_edge(Micros::new(200), Rpm::new(750), Degrees10::new(12), false)
            .unwrap();
        sim.cam_edge(Micros::new(201), false).unwrap();
        sim.sensor_frame(
            Micros::new(202),
            Rpm::new(775),
            Kpa10::new(345),
            Degrees10::new(14),
        )
        .unwrap();
        sim.tick(Micros::new(204), control_inputs()).unwrap();
        sim.drain_until_idle();

        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.faults.fault, FaultCode::SensorOutOfRange);
        assert_eq!(snapshot.faults.severity, FaultSeverity::Warning);
        assert_eq!(snapshot.engine.rpm.get(), 775);
        assert_eq!(snapshot.engine.load_kpa10.get(), 345);
        assert_eq!(snapshot.engine.angle_x10.get(), 14);
        assert_eq!(snapshot.engine.rpm.get(), 775);
        assert_eq!(snapshot.engine.load_kpa10.get(), 345);
    }
}
