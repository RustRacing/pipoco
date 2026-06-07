use crate::sensor_sample::map_speed_density_capture_sample;
use ecu_board_api::{
    BoardSensorSnapshotCapture, CaptureSample, CaptureSampleSource, CaptureSink, Watchdog,
};
use ecu_calibration::{PersistedCalibrationBlob, PersistedCalibrationStore};
use ecu_domain::EngineTimeAuthority;
use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_runtime::{
    Action, ActionExecutor, ControlInputs, DecoderObservation, EngineRuntime, RuntimeFuelStrategy,
    RuntimeSemanticCalibration, RuntimeSemanticState, StepInputs, StepResult, TransportPublisher,
};

/// Board-like event surface for the runtime-driven board adapters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardEvent {
    TriggerEdge {
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        authority: EngineTimeAuthority,
        synced: bool,
    },
    CamEdge {
        at_us: Micros,
        cam_seen: bool,
    },
    /// Logical board snapshot capture projected to the runtime's MAP load path.
    SensorSnapshotCapture {
        capture: BoardSensorSnapshotCapture,
    },
    Tick {
        now_us: Micros,
        control: ControlInputs,
    },
}

/// Adapter error split by IO capability.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardAdapterError<S, C, A, W, T, P> {
    Sensor(S),
    Capture(C),
    Action(A),
    Watchdog(W),
    Transport(T),
    Persistence(P),
}

pub type AdapterResult<S, C, A, W, T, P, Output> =
    Result<Output, BoardAdapterError<S, C, A, W, T, P>>;

fn initial_inputs() -> StepInputs {
    StepInputs {
        now_us: Micros::new(0),
        rpm: 0,
        load_kpa10: 0,
        angle_x10: 0,
        trigger_synced: false,
        cam_seen: false,
        flat_shift_armed: false,
        launch_armed: false,
    }
}

/// Shared board adapter that turns board events into runtime updates and IO actions.
///
/// Invariants:
/// - board events update runtime input state only
/// - control decisions come from `EngineRuntime::step`
/// - action delivery is limited to I/O plumbing and publishing
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoardAdapter<S, C, A, W, T, P> {
    runtime: EngineRuntime,
    sensor: S,
    capture: C,
    actions: A,
    watchdog: W,
    transport: T,
    store: P,
    pending_inputs: StepInputs,
}

impl<S, C, A, W, T, P> BoardAdapter<S, C, A, W, T, P>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    pub fn new(sensor: S, capture: C, actions: A, watchdog: W, transport: T, store: P) -> Self {
        Self {
            runtime: EngineRuntime::new(),
            sensor,
            capture,
            actions,
            watchdog,
            transport,
            store,
            pending_inputs: initial_inputs(),
        }
    }

    pub fn runtime(&self) -> &EngineRuntime {
        &self.runtime
    }

    pub fn configure_fuel_model(&mut self, fuel_model: ecu_runtime::BaseFuelModel) {
        self.runtime.configure_fuel_model(fuel_model);
    }

    pub fn configure_speed_density_semantic(
        &mut self,
        calibration: RuntimeSemanticCalibration,
        state: RuntimeSemanticState,
    ) {
        self.runtime.configure_speed_density_ve(calibration, state);
    }

    pub fn configure_runtime_fuel_strategy(&mut self, strategy: RuntimeFuelStrategy) {
        self.runtime.configure_runtime_fuel_model(strategy);
    }

    pub fn set_staged_dirty(&mut self, dirty: bool) {
        self.runtime.set_staged_dirty(dirty);
    }

    pub fn sensor(&mut self) -> &mut S {
        &mut self.sensor
    }

    pub fn capture(&mut self) -> &mut C {
        &mut self.capture
    }

    pub fn actions(&mut self) -> &mut A {
        &mut self.actions
    }

    pub fn watchdog(&mut self) -> &mut W {
        &mut self.watchdog
    }

    pub fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn store(&mut self) -> &mut P {
        &mut self.store
    }

    #[allow(clippy::type_complexity)]
    pub fn poll_sensor(
        &mut self,
    ) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, CaptureSample>
    {
        let sample = self.sensor.sample().map_err(BoardAdapterError::Sensor)?;
        self.pending_inputs.now_us = sample.at_us;
        self.pending_inputs.rpm = u32::from(sample.rpm.get());
        self.pending_inputs.load_kpa10 = u32::from(sample.load_kpa10.get());
        self.pending_inputs.angle_x10 = i32::from(sample.angle_x10.get());
        self.capture
            .capture(sample)
            .map_err(BoardAdapterError::Capture)?;
        Ok(sample)
    }

    #[allow(clippy::type_complexity)]
    pub fn apply_event(
        &mut self,
        event: BoardEvent,
    ) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, Option<StepResult>>
    {
        match event {
            BoardEvent::TriggerEdge {
                at_us,
                rpm,
                angle_x10,
                authority,
                synced,
            } => {
                self.pending_inputs.now_us = at_us;
                self.pending_inputs.rpm = u32::from(rpm.get());
                self.pending_inputs.angle_x10 = i32::from(angle_x10.get());
                self.pending_inputs.trigger_synced = synced;
                self.capture
                    .capture(CaptureSample {
                        at_us,
                        rpm,
                        load_kpa10: self.runtime.snapshot().engine.load_kpa10,
                        angle_x10,
                    })
                    .map_err(BoardAdapterError::Capture)?;
                self.runtime.apply_sensor_sample(
                    rpm,
                    self.runtime.snapshot().engine.load_kpa10,
                    angle_x10,
                );
                self.runtime.set_engine_time_authority(authority);
                Ok(None)
            }
            BoardEvent::CamEdge { at_us, cam_seen } => {
                self.pending_inputs.now_us = at_us;
                self.pending_inputs.cam_seen = cam_seen;
                self.runtime
                    .apply_decoder_observation(DecoderObservation::Cam(
                        ecu_runtime::CamObservation { at_us, cam_seen },
                    ));
                Ok(None)
            }
            BoardEvent::SensorSnapshotCapture { capture } => {
                let sample = map_speed_density_capture_sample(capture);
                self.pending_inputs.now_us = sample.at_us;
                self.pending_inputs.rpm = u32::from(sample.rpm.get());
                self.pending_inputs.load_kpa10 = u32::from(sample.load_kpa10.get());
                self.pending_inputs.angle_x10 = i32::from(sample.angle_x10.get());
                self.runtime
                    .apply_sensor_sample(sample.rpm, sample.load_kpa10, sample.angle_x10);
                self.capture
                    .capture(sample)
                    .map_err(BoardAdapterError::Capture)?;
                Ok(None)
            }
            BoardEvent::Tick { now_us, control } => {
                self.pending_inputs.now_us = now_us;
                let result = self.runtime.step_with_authority(
                    self.pending_inputs,
                    control,
                    self.runtime.engine_time_authority(),
                );
                self.execute_step(&result)?;
                Ok(Some(result))
            }
        }
    }

    #[allow(clippy::type_complexity)]
    pub fn execute_step(
        &mut self,
        result: &StepResult,
    ) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, ()> {
        for action in result.actions.iter() {
            match action {
                Action::PublishSnapshot => self
                    .transport
                    .publish_snapshot(&self.runtime.snapshot())
                    .map_err(BoardAdapterError::Transport)?,
                Action::PersistCalibration => self
                    .store
                    .save(&PersistedCalibrationBlob::new(
                        self.runtime.calibration_snapshot(),
                    ))
                    .map_err(BoardAdapterError::Persistence)?,
                Action::Idle => {}
                other => self
                    .actions
                    .execute(other)
                    .map_err(BoardAdapterError::Action)?,
            }
        }

        self.watchdog.feed().map_err(BoardAdapterError::Watchdog)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests;
