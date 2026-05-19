use std::sync::{Arc, Condvar, Mutex, OnceLock};

use ecu_board_api::{AuxCommand, AuxCommandBatch, AuxOutput, AuxValue, OutputLevel};
use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm};
use ecu_runtime::{
    Action, BaseFuelModel, ControlInputs, EngineRuntime, EnrichmentInputs, IgnitionInputs,
    LambdaTrimInputs, StepResult, TorqueInputs,
};
use ecu_scheduler::{ScheduleExport, ScheduledLevel, ScheduledTransition, ScheduledTransitionKind};
use ecu_sim::{QueueOverflow, SimulationHarness};

use crate::encoding::snapshot_from_runtime;
#[cfg(any(test, feature = "test-support"))]
use crate::encoding::{encode_cancel_reason, encode_fault_code, encode_fault_severity};
use crate::{
    EcuSimHandleOpaque, EcuSimInitCfg, EcuSimOutputEvent, EcuSimOutputKind, EcuSimSensorFrame,
    EcuSimSnapshot, EcuSimStatus, CRANK_TEETH_PER_CYCLE, CRANK_TEETH_PER_REV, DEG10_PER_TOOTH,
    ECU_SIM_MAX_CHANNELS, ECU_SIM_MAX_EVENTS, ECU_SIM_MAX_FIRING_ORDER, FAST_QUEUE_CAP,
    SLOW_QUEUE_CAP,
};
#[cfg(any(test, feature = "test-support"))]
use ecu_domain::{CancelReason, FaultCode, FaultSeverity};

const MIN_SYNC_LOSS_TIMEOUT_US: u32 = 4_000;
const SYNC_LOSS_PERIOD_MULTIPLIER: u32 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RuntimeConfig {
    has_cam: bool,
    inj_count: u8,
    inj_channels: [u8; ECU_SIM_MAX_CHANNELS],
    ign_count: u8,
    ign_channels: [u8; ECU_SIM_MAX_CHANNELS],
}

impl Default for RuntimeConfig {
    fn default() -> Self {
        Self {
            has_cam: true,
            inj_count: 1,
            inj_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ign_count: 1,
            ign_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        }
    }
}

impl RuntimeConfig {
    pub(crate) fn from_ffi(cfg: EcuSimInitCfg) -> Result<Self, EcuSimStatus> {
        validate_mode(
            cfg.inj_mode,
            super::EcuSimInjMode::Batch as i32,
            super::EcuSimInjMode::Sequential as i32,
        )?;
        validate_mode(
            cfg.ign_mode,
            super::EcuSimIgnMode::Wasted as i32,
            super::EcuSimIgnMode::Sequential as i32,
        )?;

        if cfg.cylinders == 0 || usize::from(cfg.cylinders) > ECU_SIM_MAX_FIRING_ORDER {
            return Err(EcuSimStatus::ErrInvalid);
        }
        if usize::from(cfg.firing_len) > ECU_SIM_MAX_FIRING_ORDER {
            return Err(EcuSimStatus::ErrInvalid);
        }
        if usize::from(cfg.inj_count) > ECU_SIM_MAX_CHANNELS
            || usize::from(cfg.ign_count) > ECU_SIM_MAX_CHANNELS
            || cfg.inj_count == 0
            || cfg.ign_count == 0
        {
            return Err(EcuSimStatus::ErrInvalid);
        }

        let mut idx = 0usize;
        while idx < usize::from(cfg.firing_len) {
            let cyl = cfg.firing_order[idx];
            if cyl == 0 || cyl > cfg.cylinders {
                return Err(EcuSimStatus::ErrInvalid);
            }
            idx += 1;
        }

        Ok(Self {
            has_cam: cfg.has_cam != 0,
            inj_count: cfg.inj_count,
            inj_channels: cfg.inj_channels,
            ign_count: cfg.ign_count,
            ign_channels: cfg.ign_channels,
        })
    }

    fn map_channel(self, kind: ScheduledTransitionKind, runtime_channel: u8) -> u8 {
        let index = usize::from(runtime_channel.saturating_sub(1));
        match kind {
            ScheduledTransitionKind::Injector if index < usize::from(self.inj_count) => {
                self.inj_channels[index]
            }
            ScheduledTransitionKind::Ignition if index < usize::from(self.ign_count) => {
                self.ign_channels[index]
            }
            _ => runtime_channel,
        }
    }
}

fn validate_mode(value: i32, low: i32, high: i32) -> Result<(), EcuSimStatus> {
    if value < low || value > high {
        Err(EcuSimStatus::ErrInvalid)
    } else {
        Ok(())
    }
}

#[derive(Debug)]
struct OutputEventQueue<const N: usize> {
    events: [EcuSimOutputEvent; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> OutputEventQueue<N> {
    const fn new() -> Self {
        Self {
            events: [EcuSimOutputEvent::ZERO; N],
            len: 0,
            overflow_count: 0,
        }
    }

    fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn push_sorted(&mut self, event: EcuSimOutputEvent) -> Result<(), ()> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(());
        }

        let mut index = self.len;
        while index > 0 && event_less(event, self.events[index - 1]) {
            self.events[index] = self.events[index - 1];
            index -= 1;
        }
        self.events[index] = event;
        self.len += 1;
        Ok(())
    }

    fn pop_front(&mut self) -> Option<EcuSimOutputEvent> {
        if self.len == 0 {
            return None;
        }
        let event = self.events[0];
        let mut index = 1usize;
        while index < self.len {
            self.events[index - 1] = self.events[index];
            index += 1;
        }
        self.len -= 1;
        Some(event)
    }
}

impl<const N: usize> Default for OutputEventQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub(crate) fn event_less(left: EcuSimOutputEvent, right: EcuSimOutputEvent) -> bool {
    (left.time_us, left.kind, left.channel, left.high)
        < (right.time_us, right.kind, right.channel, right.high)
}

#[derive(Debug)]
pub(crate) struct EcuSimHandle {
    initialized: bool,
    sim: SimulationHarness<FAST_QUEUE_CAP, SLOW_QUEUE_CAP>,
    outputs: OutputEventQueue<ECU_SIM_MAX_EVENTS>,
    cfg: RuntimeConfig,
    now_us: u32,
    sensors: EcuSimSensorFrame,
    rpm: u16,
    tooth: u8,
    angle_x10: i16,
    synced: bool,
    last_crank_edge_us: Option<u32>,
    last_crank_period_us: Option<u32>,
    overflow_latched: bool,
    #[cfg(any(test, feature = "test-support"))]
    fault_override: Option<(FaultCode, FaultSeverity, CancelReason)>,
}

const HANDLE_COOKIE: usize = 0xA5;
const HANDLE_COOKIE_BITS: usize = 8;
const HANDLE_SLOT_BITS: usize = 16;
const HANDLE_GENERATION_BITS: usize = 16;
const HANDLE_NONCE_BITS: usize =
    usize::BITS as usize - HANDLE_COOKIE_BITS - HANDLE_SLOT_BITS - HANDLE_GENERATION_BITS;
const HANDLE_SLOT_MASK: usize = (1usize << HANDLE_SLOT_BITS) - 1;
const HANDLE_GENERATION_MASK: usize = (1usize << HANDLE_GENERATION_BITS) - 1;
const HANDLE_NONCE_MASK: usize = (1usize << HANDLE_NONCE_BITS) - 1;
const HANDLE_GENERATION_SHIFT: usize = HANDLE_SLOT_BITS;
const HANDLE_NONCE_SHIFT: usize = HANDLE_SLOT_BITS + HANDLE_GENERATION_BITS;
const HANDLE_COOKIE_SHIFT: usize = HANDLE_NONCE_SHIFT + HANDLE_NONCE_BITS;
const HANDLE_COOKIE_MASK: usize = (1usize << HANDLE_COOKIE_BITS) - 1;

#[derive(Debug)]
struct HandleGate {
    destroyed: bool,
    active_calls: usize,
}

#[derive(Debug)]
struct HandleControl {
    state: Mutex<EcuSimHandle>,
    gate: Mutex<HandleGate>,
    gate_changed: Condvar,
}

impl HandleControl {
    fn new() -> Self {
        Self {
            state: Mutex::new(EcuSimHandle::new()),
            gate: Mutex::new(HandleGate {
                destroyed: false,
                active_calls: 0,
            }),
            gate_changed: Condvar::new(),
        }
    }

    fn begin_call(self: &Arc<Self>) -> Result<HandleCallGuard, EcuSimStatus> {
        let mut gate = match self.gate.lock() {
            Ok(gate) => gate,
            Err(poisoned) => poisoned.into_inner(),
        };
        if gate.destroyed {
            return Err(EcuSimStatus::ErrInvalid);
        }
        gate.active_calls += 1;
        Ok(HandleCallGuard {
            control: Arc::clone(self),
        })
    }

    fn destroy(&self) {
        let mut gate = match self.gate.lock() {
            Ok(gate) => gate,
            Err(poisoned) => poisoned.into_inner(),
        };
        gate.destroyed = true;
        while gate.active_calls > 0 {
            gate = match self.gate_changed.wait(gate) {
                Ok(gate) => gate,
                Err(poisoned) => poisoned.into_inner(),
            };
        }
    }
}

#[derive(Debug)]
struct HandleSlot {
    generation: u32,
    nonce: u32,
    control: Option<Arc<HandleControl>>,
}

impl HandleSlot {
    fn new(nonce: u32) -> Self {
        Self {
            generation: 1,
            nonce,
            control: None,
        }
    }
}

#[derive(Debug, Default)]
struct HandleRegistry {
    slots: Vec<HandleSlot>,
    free_slots: Vec<usize>,
    next_nonce: u32,
}

impl HandleRegistry {
    fn allocate_nonce(&mut self) -> u32 {
        self.next_nonce = (self.next_nonce % HANDLE_NONCE_MASK as u32) + 1;
        self.next_nonce
    }
}

struct HandleCallGuard {
    control: Arc<HandleControl>,
}

impl Drop for HandleCallGuard {
    fn drop(&mut self) {
        let mut gate = match self.control.gate.lock() {
            Ok(gate) => gate,
            Err(poisoned) => poisoned.into_inner(),
        };
        debug_assert!(gate.active_calls > 0);
        gate.active_calls -= 1;
        if gate.active_calls == 0 {
            self.control.gate_changed.notify_all();
        }
    }
}

impl EcuSimHandle {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn init(&mut self, cfg: RuntimeConfig) {
        *self = Self {
            initialized: true,
            sim: new_sim_harness(),
            outputs: OutputEventQueue::new(),
            cfg,
            now_us: 0,
            sensors: EcuSimSensorFrame::default(),
            rpm: 0,
            tooth: 0,
            angle_x10: 0,
            synced: false,
            last_crank_edge_us: None,
            last_crank_period_us: None,
            overflow_latched: false,
            #[cfg(any(test, feature = "test-support"))]
            fault_override: None,
        };
    }

    pub(crate) fn reset(&mut self) {
        *self = Self::default();
    }

    #[cfg(any(test, feature = "test-support"))]
    pub(crate) fn initialized(&self) -> bool {
        self.initialized
    }

    pub(crate) fn require_init(&self) -> Result<(), EcuSimStatus> {
        if self.initialized {
            Ok(())
        } else {
            Err(EcuSimStatus::ErrNotInit)
        }
    }

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

    fn drain_sim_steps(&mut self) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        while let Some(step) = self.sim.drain_one() {
            if let Some(result) = step.result {
                if self.capture_step_result(result) == EcuSimStatus::ErrEventOverflow {
                    status = EcuSimStatus::ErrEventOverflow;
                }
            }
        }
        status
    }

    fn capture_step_result(&mut self, result: StepResult) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        for action in result.actions.iter() {
            if self.capture_action(action) == EcuSimStatus::ErrEventOverflow {
                status = EcuSimStatus::ErrEventOverflow;
            }
        }
        status
    }

    #[allow(deprecated)]
    fn capture_action(&mut self, action: Action) -> EcuSimStatus {
        match action {
            Action::ArmScheduler {
                injection,
                ignition,
            } => {
                let mut status = EcuSimStatus::Ok;
                if let Ok(export) = injection.export_transitions::<2>() {
                    if self.capture_export(export) == EcuSimStatus::ErrEventOverflow {
                        status = EcuSimStatus::ErrEventOverflow;
                    }
                }
                if let Ok(export) = ignition.export_transitions::<2>() {
                    if self.capture_export(export) == EcuSimStatus::ErrEventOverflow {
                        status = EcuSimStatus::ErrEventOverflow;
                    }
                }
                status
            }
            Action::ApplyAux(batch) => self.capture_aux_batch(&batch),
            Action::SetFan(enabled) => self.capture_aux_command(AuxCommand::new(
                AuxOutput::Fan,
                if enabled {
                    AuxValue::Level(OutputLevel::High)
                } else {
                    AuxValue::Off
                },
            )),
            Action::CancelScheduler(_)
            | Action::PublishSnapshot
            | Action::PersistCalibration
            | Action::Idle => EcuSimStatus::Ok,
        }
    }

    fn capture_aux_batch<const N: usize>(&mut self, batch: &AuxCommandBatch<N>) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        for command in batch.iter() {
            if self.capture_aux_command(*command) == EcuSimStatus::ErrEventOverflow {
                status = EcuSimStatus::ErrEventOverflow;
            }
        }
        status
    }

    fn capture_aux_command(&mut self, command: AuxCommand) -> EcuSimStatus {
        match command.output {
            AuxOutput::Fan => self.push_output(EcuSimOutputEvent {
                time_us: self.now_us,
                channel: 0,
                kind: EcuSimOutputKind::Fan as i32,
                high: u8::from(matches!(command.value, AuxValue::Level(OutputLevel::High))),
            }),
            _ => EcuSimStatus::Ok,
        }
    }

    fn capture_export<const N: usize>(&mut self, export: ScheduleExport<N>) -> EcuSimStatus {
        let mut status = EcuSimStatus::Ok;
        let mut idx = 0usize;
        while idx < usize::from(export.len) && idx < N {
            if let Some(transition) = export.transitions[idx] {
                if self.capture_transition(transition) == EcuSimStatus::ErrEventOverflow {
                    status = EcuSimStatus::ErrEventOverflow;
                }
            }
            idx += 1;
        }
        status
    }

    fn capture_transition(&mut self, transition: ScheduledTransition) -> EcuSimStatus {
        self.push_output(EcuSimOutputEvent {
            time_us: transition.at_us.get(),
            channel: self
                .cfg
                .map_channel(transition.kind, transition.channel.get()),
            kind: match transition.kind {
                ScheduledTransitionKind::Injector => EcuSimOutputKind::Injector as i32,
                ScheduledTransitionKind::Ignition => EcuSimOutputKind::Ignition as i32,
            },
            high: match transition.level {
                ScheduledLevel::Low => 0,
                ScheduledLevel::High => 1,
            },
        })
    }

    fn push_output(&mut self, event: EcuSimOutputEvent) -> EcuSimStatus {
        match self.outputs.push_sorted(event) {
            Ok(()) => EcuSimStatus::Ok,
            Err(()) => {
                self.overflow_latched = true;
                EcuSimStatus::ErrEventOverflow
            }
        }
    }

    pub(crate) fn dequeue_event(&mut self) -> Option<EcuSimOutputEvent> {
        self.outputs.pop_front()
    }

    pub(crate) fn outputs_empty(&self) -> bool {
        self.outputs.is_empty()
    }

    pub(crate) fn clear_overflow_latch(&mut self) {
        self.overflow_latched = false;
    }

    fn control_inputs(&self, now_us: u32) -> ControlInputs {
        let clt_c = self.sensors.clt_c10 / 10;
        let measured_lambda = if self.sensors.lambda_x100 == 0 {
            100
        } else {
            self.sensors.lambda_x100
        };
        let throttle_pct = (u32::from(self.sensors.tps_x100) / 100).min(100) as u16;
        let driver_request = throttle_pct.max(20);

        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us: Micros::new(now_us),
                clt_c,
                cranking: self.rpm > 0 && self.rpm < 400,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c,
                lambda_valid: self.sensors.lambda_valid != 0,
                measured_lambda100: Lambda100::new(measured_lambda),
                requested_open_loop: self.sensors.lambda_valid == 0,
            },
            torque: TorqueInputs::new(driver_request, 30, 100, 100, 100),
            ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(self.rpm)),
        }
    }

    pub(crate) fn snapshot(&self) -> EcuSimSnapshot {
        let snapshot = snapshot_from_runtime(
            self.sim.runtime().snapshot(),
            self.now_us,
            self.tooth,
            self.outputs.overflow_count,
        );
        self.apply_test_fault_override(snapshot)
    }

    #[cfg(any(test, feature = "test-support"))]
    pub(crate) fn inject_fault_for_test(
        &mut self,
        fault: FaultCode,
        severity: FaultSeverity,
        cancel_reason: CancelReason,
    ) {
        self.fault_override = Some((fault, severity, cancel_reason));
    }

    #[cfg(any(test, feature = "test-support"))]
    fn apply_test_fault_override(&self, mut snapshot: EcuSimSnapshot) -> EcuSimSnapshot {
        if let Some((fault, severity, cancel_reason)) = self.fault_override {
            snapshot.fault_code = encode_fault_code(fault);
            snapshot.fault_severity = encode_fault_severity(severity);
            snapshot.cancel_reason = encode_cancel_reason(cancel_reason);
        }
        snapshot
    }

    #[cfg(not(any(test, feature = "test-support")))]
    fn apply_test_fault_override(&self, snapshot: EcuSimSnapshot) -> EcuSimSnapshot {
        snapshot
    }
}

impl Default for EcuSimHandle {
    fn default() -> Self {
        Self {
            initialized: false,
            sim: new_sim_harness(),
            outputs: OutputEventQueue::new(),
            cfg: RuntimeConfig::default(),
            now_us: 0,
            sensors: EcuSimSensorFrame::default(),
            rpm: 0,
            tooth: 0,
            angle_x10: 0,
            synced: false,
            last_crank_edge_us: None,
            last_crank_period_us: None,
            overflow_latched: false,
            #[cfg(any(test, feature = "test-support"))]
            fault_override: None,
        }
    }
}

fn new_sim_harness() -> SimulationHarness<FAST_QUEUE_CAP, SLOW_QUEUE_CAP> {
    let mut runtime = EngineRuntime::new();
    runtime.configure_fuel_model(default_ffi_fuel_model());
    SimulationHarness::new(runtime)
}

fn default_ffi_fuel_model() -> BaseFuelModel {
    BaseFuelModel::new(
        [
            Rpm::new(0),
            Rpm::new(500),
            Rpm::new(1_000),
            Rpm::new(1_500),
            Rpm::new(2_000),
            Rpm::new(2_500),
            Rpm::new(3_000),
            Rpm::new(3_500),
            Rpm::new(4_000),
            Rpm::new(4_500),
            Rpm::new(5_000),
            Rpm::new(5_500),
            Rpm::new(6_000),
            Rpm::new(6_500),
            Rpm::new(7_000),
            Rpm::new(7_500),
        ],
        [
            Kpa10::new(0),
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1_000),
            Kpa10::new(1_100),
            Kpa10::new(1_200),
            Kpa10::new(1_300),
            Kpa10::new(1_400),
            Kpa10::new(1_500),
            Kpa10::new(1_600),
        ],
        [[PulseWidthUs::new(2_500); 16]; 16],
    )
}

fn rpm_from_tooth_period(period_us: u32) -> u16 {
    if period_us == 0 {
        return 0;
    }
    let rpm = 60_000_000u32 / period_us / CRANK_TEETH_PER_REV;
    rpm.min(u32::from(u16::MAX)) as u16
}

static STATE: OnceLock<Mutex<EcuSimHandle>> = OnceLock::new();
static HANDLE_REGISTRY: OnceLock<Mutex<HandleRegistry>> = OnceLock::new();

#[cfg(test)]
static TEST_LOCK: OnceLock<Mutex<()>> = OnceLock::new();

#[cfg(test)]
pub(crate) fn acquire_test_lock() -> std::sync::MutexGuard<'static, ()> {
    TEST_LOCK
        .get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
}

fn state_mutex() -> &'static Mutex<EcuSimHandle> {
    STATE.get_or_init(|| Mutex::new(EcuSimHandle::new()))
}

fn handle_registry() -> &'static Mutex<HandleRegistry> {
    HANDLE_REGISTRY.get_or_init(|| Mutex::new(HandleRegistry::default()))
}

pub(crate) fn with_state<R>(f: impl FnOnce(&mut EcuSimHandle) -> R) -> R {
    let mut state = match state_mutex().lock() {
        Ok(state) => state,
        Err(poisoned) => poisoned.into_inner(),
    };
    f(&mut state)
}

fn lock_handle_registry() -> std::sync::MutexGuard<'static, HandleRegistry> {
    match handle_registry().lock() {
        Ok(registry) => registry,
        Err(poisoned) => poisoned.into_inner(),
    }
}

fn encode_handle_token(
    slot: usize,
    generation: u32,
    nonce: u32,
) -> Option<*mut EcuSimHandleOpaque> {
    if slot > HANDLE_SLOT_MASK {
        return None;
    }
    let generation = (generation as usize) & HANDLE_GENERATION_MASK;
    let nonce = (nonce as usize) & HANDLE_NONCE_MASK;
    if generation == 0 || nonce == 0 {
        return None;
    }
    let token = ((HANDLE_COOKIE & HANDLE_COOKIE_MASK) << HANDLE_COOKIE_SHIFT)
        | (nonce << HANDLE_NONCE_SHIFT)
        | (generation << HANDLE_GENERATION_SHIFT)
        | slot;
    if token == 0 {
        None
    } else {
        Some(token as *mut EcuSimHandleOpaque)
    }
}

fn decode_handle_token(handle: *mut EcuSimHandleOpaque) -> Option<(usize, u32, u32)> {
    let token = handle as usize;
    if token == 0 {
        return None;
    }
    if ((token >> HANDLE_COOKIE_SHIFT) & HANDLE_COOKIE_MASK) != HANDLE_COOKIE {
        return None;
    }
    let slot = token & HANDLE_SLOT_MASK;
    let generation = ((token >> HANDLE_GENERATION_SHIFT) & HANDLE_GENERATION_MASK) as u32;
    let nonce = ((token >> HANDLE_NONCE_SHIFT) & HANDLE_NONCE_MASK) as u32;
    if generation == 0 || nonce == 0 {
        return None;
    }
    Some((slot, generation, nonce))
}

pub(crate) fn create_handle() -> Option<*mut EcuSimHandleOpaque> {
    let mut registry = lock_handle_registry();
    let nonce = registry.allocate_nonce();
    let slot = if let Some(slot) = registry.free_slots.pop() {
        slot
    } else {
        let slot = registry.slots.len();
        if slot > HANDLE_SLOT_MASK {
            return None;
        }
        registry.slots.push(HandleSlot::new(nonce));
        slot
    };
    let slot_state = registry.slots.get_mut(slot)?;
    debug_assert!(slot_state.control.is_none());
    slot_state.nonce = nonce;

    let handle = encode_handle_token(slot, slot_state.generation, slot_state.nonce)?;
    slot_state.control = Some(Arc::new(HandleControl::new()));
    Some(handle)
}

pub(crate) fn destroy_handle(handle: *mut EcuSimHandleOpaque) {
    let Some((slot, generation, nonce)) = decode_handle_token(handle) else {
        return;
    };
    let control = {
        let mut registry = lock_handle_registry();
        let Some(slot_state) = registry.slots.get_mut(slot) else {
            return;
        };
        if slot_state.generation != generation || slot_state.nonce != nonce {
            return;
        }
        let Some(control) = slot_state.control.take() else {
            return;
        };
        if slot_state.generation == HANDLE_GENERATION_MASK as u32 {
            // Retire exhausted slots instead of wrapping generation values and
            // allowing a stale token to alias a future handle.
        } else {
            slot_state.generation += 1;
            registry.free_slots.push(slot);
        }
        control
    };
    control.destroy();
}

pub(crate) fn with_handle<R>(
    handle: *mut EcuSimHandleOpaque,
    f: impl FnOnce(&mut EcuSimHandle) -> R,
) -> Result<R, EcuSimStatus> {
    let Some((slot, generation, nonce)) = decode_handle_token(handle) else {
        return Err(EcuSimStatus::ErrInvalid);
    };
    let control = {
        let registry = lock_handle_registry();
        let Some(slot_state) = registry.slots.get(slot) else {
            return Err(EcuSimStatus::ErrInvalid);
        };
        if slot_state.generation != generation || slot_state.nonce != nonce {
            return Err(EcuSimStatus::ErrInvalid);
        }
        let Some(control) = slot_state.control.as_ref() else {
            return Err(EcuSimStatus::ErrInvalid);
        };
        Arc::clone(control)
    };

    let call_guard = control.begin_call()?;
    let result = {
        let mut state = match control.state.lock() {
            Ok(state) => state,
            Err(poisoned) => poisoned.into_inner(),
        };
        f(&mut state)
    };
    drop(call_guard);
    Ok(result)
}

#[cfg(test)]
pub(crate) fn handle_registry_stats() -> (usize, usize, usize, u32) {
    let registry = lock_handle_registry();
    let live = registry
        .slots
        .iter()
        .filter(|slot| slot.control.is_some())
        .count();
    let max_generation = registry
        .slots
        .iter()
        .map(|slot| slot.generation)
        .max()
        .unwrap_or(0);
    (
        live,
        registry.slots.len(),
        registry.free_slots.len(),
        max_generation,
    )
}

#[cfg(test)]
pub(crate) fn max_handle_generation_for_test() -> u32 {
    HANDLE_GENERATION_MASK as u32
}

#[cfg(test)]
pub(crate) fn fabricate_wrong_nonce_handle_for_test(
    handle: *mut EcuSimHandleOpaque,
) -> Option<*mut EcuSimHandleOpaque> {
    let (slot, generation, nonce) = decode_handle_token(handle)?;
    let wrong_nonce = (nonce % HANDLE_NONCE_MASK as u32) + 1;
    if wrong_nonce == nonce {
        return None;
    }
    encode_handle_token(slot, generation, wrong_nonce)
}

#[cfg(test)]
pub(crate) fn force_next_handle_generation_for_test(generation: u32) -> bool {
    if generation == 0 || generation > HANDLE_GENERATION_MASK as u32 {
        return false;
    }
    let mut registry = lock_handle_registry();
    let Some(slot) = registry.free_slots.last().copied() else {
        return false;
    };
    let Some(slot_state) = registry.slots.get_mut(slot) else {
        return false;
    };
    if slot_state.control.is_some() {
        return false;
    }
    slot_state.generation = generation;
    true
}
