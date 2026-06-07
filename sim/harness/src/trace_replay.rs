//! Trace replay helpers for feeding recorded traces into simulation harnesses.
//!
//! Provides `TraceReplay` for deterministic replay of edge, sensor, and tick
//! traces into `ecu_sim::SimulationHarness` or other targets implementing the
//! narrow replay trait.

use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_io::trace::{TraceInputKind, TracePayload, TraceRecord};
use ecu_io::{EdgeLine, EdgePolarity};
use ecu_runtime::ControlInputs;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayError {
    /// The trace record has an unsupported input/payload combination.
    UnsupportedRecord,
    /// Trace time is not monotonically increasing.
    NonMonotonicTime,
    /// The destination harness queue is full.
    QueueFull,
    /// Sensor load cannot be represented by the harness MAP type.
    MapOutOfRange { load_kpa10: u32 },
}

/// Target for replay operations.
///
/// This trait allows `TraceReplay` to work with both `SimulationHarness`
/// and compatible wrappers around live harness types.
pub trait ReplayTarget {
    /// Push a trigger (crank) edge into the target.
    fn trigger_edge(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        synced: bool,
    ) -> Result<(), ReplayError>;

    /// Push a cam edge into the target.
    fn cam_edge(&mut self, at_us: Micros, cam_seen: bool) -> Result<(), ReplayError>;

    /// Push a sensor frame into the target.
    fn sensor_frame(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        load_kpa10: u32,
        angle_x10: Degrees10,
    ) -> Result<(), ReplayError>;

    /// Push a control/runtime tick into the target.
    fn tick(&mut self, now_us: Micros, control: ControlInputs) -> Result<(), ReplayError>;
}

/// Replay engine that feeds trace records into a destination harness.
///
/// Validates record ordering and consistency, then routes each record
/// to the appropriate harness method based on `input_kind`.
#[derive(Debug, Clone, Copy)]
pub struct TraceReplay<T> {
    target: T,
    last_time_us: u32,
}

impl<T> TraceReplay<T> {
    /// Create a new trace replay targeting the given harness.
    pub fn new(target: T) -> Self {
        Self {
            target,
            last_time_us: 0,
        }
    }

    /// Consume this replay and return the underlying target.
    pub fn into_target(self) -> T {
        self.target
    }

    /// Get a mutable reference to the underlying target.
    pub fn target_mut(&mut self) -> &mut T {
        &mut self.target
    }

    /// Get a reference to the underlying target.
    pub fn target(&self) -> &T {
        &self.target
    }
}

impl<const FAST: usize, const SLOW: usize> ReplayTarget for crate::SimulationHarness<FAST, SLOW> {
    fn trigger_edge(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        synced: bool,
    ) -> Result<(), ReplayError> {
        crate::SimulationHarness::trigger_edge(self, at_us, rpm, angle_x10, synced)
            .map(|_| ())
            .map_err(|_| ReplayError::QueueFull)
    }

    fn cam_edge(&mut self, at_us: Micros, cam_seen: bool) -> Result<(), ReplayError> {
        crate::SimulationHarness::cam_edge(self, at_us, cam_seen)
            .map(|_| ())
            .map_err(|_| ReplayError::QueueFull)
    }

    fn sensor_frame(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        load_kpa10: u32,
        angle_x10: Degrees10,
    ) -> Result<(), ReplayError> {
        use ecu_domain::Kpa10;
        let load_kpa10 =
            u16::try_from(load_kpa10).map_err(|_| ReplayError::MapOutOfRange { load_kpa10 })?;
        crate::SimulationHarness::sensor_frame(self, at_us, rpm, Kpa10::new(load_kpa10), angle_x10)
            .map(|_| ())
            .map_err(|_| ReplayError::QueueFull)
    }

    fn tick(&mut self, now_us: Micros, control: ControlInputs) -> Result<(), ReplayError> {
        crate::SimulationHarness::tick(self, now_us, control)
            .map(|_| ())
            .map_err(|_| ReplayError::QueueFull)
    }
}

/// Blanket impl: mutable reference to a ReplayTarget is also a ReplayTarget.
impl<T: ReplayTarget + ?Sized> ReplayTarget for &mut T {
    fn trigger_edge(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        angle_x10: Degrees10,
        synced: bool,
    ) -> Result<(), ReplayError> {
        (**self).trigger_edge(at_us, rpm, angle_x10, synced)
    }

    fn cam_edge(&mut self, at_us: Micros, cam_seen: bool) -> Result<(), ReplayError> {
        (**self).cam_edge(at_us, cam_seen)
    }

    fn sensor_frame(
        &mut self,
        at_us: Micros,
        rpm: Rpm,
        load_kpa10: u32,
        angle_x10: Degrees10,
    ) -> Result<(), ReplayError> {
        (**self).sensor_frame(at_us, rpm, load_kpa10, angle_x10)
    }

    fn tick(&mut self, now_us: Micros, control: ControlInputs) -> Result<(), ReplayError> {
        (**self).tick(now_us, control)
    }
}

/// Error returned when trace record validation fails.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecordValidationError {
    /// Input kind and payload are inconsistent.
    UnsupportedRecord,
}

impl<T: ReplayTarget> TraceReplay<T> {
    /// Validate that a trace record's input_kind and payload are consistent.
    pub fn validate_record(record: &TraceRecord) -> Result<(), RecordValidationError> {
        match (record.input_kind(), record.payload()) {
            // Valid combinations
            (TraceInputKind::None, TracePayload::None) => Ok(()),
            (TraceInputKind::Edge, TracePayload::Edge(_)) => Ok(()),
            (TraceInputKind::Sensor, TracePayload::Sensor(_)) => Ok(()),
            (TraceInputKind::Tick, TracePayload::Sensor(_)) => Ok(()),
            // Output records belong to output trace capture, not input replay.
            (TraceInputKind::Output, _) => Err(RecordValidationError::UnsupportedRecord),
            // Invalid: TraceInputKind::None with a non-None payload
            (TraceInputKind::None, _) => Err(RecordValidationError::UnsupportedRecord),
            // Invalid: Edge input with non-Edge payload
            (TraceInputKind::Edge, _) => Err(RecordValidationError::UnsupportedRecord),
            // Invalid: Sensor input with non-Sensor payload
            (TraceInputKind::Sensor, _) => Err(RecordValidationError::UnsupportedRecord),
            // Invalid: Tick input with non-Sensor payload (Tick requires Sensor frame for runtime step)
            (TraceInputKind::Tick, _) => Err(RecordValidationError::UnsupportedRecord),
        }
    }

    /// Feed a single trace record into the target harness.
    ///
    /// Returns `Ok(())` on success.
    /// Returns `Err(ReplayError::NonMonotonicTime)` if `record.time_us` is less
    /// than the time of any previously processed record.
    /// Returns `Err(ReplayError::UnsupportedRecord)` if `input_kind` and `payload`
    /// are inconsistent.
    /// Returns `Err(ReplayError::QueueFull)` if the harness cannot accept more events.
    pub fn replay_one(&mut self, record: &TraceRecord) -> Result<(), ReplayError> {
        // Check monotonic time
        if record.time_us.get() < self.last_time_us {
            return Err(ReplayError::NonMonotonicTime);
        }

        // Validate record consistency
        Self::validate_record(record).map_err(|_| ReplayError::UnsupportedRecord)?;

        // Route to appropriate target method based on input_kind
        match record.input_kind() {
            TraceInputKind::None => {
                // TracePayload::None only - nothing to feed, just update time
                self.last_time_us = record.time_us.get();
                Ok(())
            }
            TraceInputKind::Edge => {
                if let TracePayload::Edge(edge) = record.payload() {
                    let synced = record.synced;
                    match edge.line {
                        EdgeLine::Crank => {
                            self.target.trigger_edge(
                                edge.at_us,
                                edge.rpm,
                                edge.angle_x10,
                                synced,
                            )?;
                        }
                        EdgeLine::Cam => {
                            // Cam edge: cam_seen is true on rising edge
                            let cam_seen = matches!(edge.polarity, EdgePolarity::Rising);
                            self.target.cam_edge(edge.at_us, cam_seen)?;
                        }
                    }
                    self.last_time_us = record.time_us.get();
                    Ok(())
                } else {
                    // Should not happen due to validate_record check
                    Err(ReplayError::UnsupportedRecord)
                }
            }
            TraceInputKind::Sensor => {
                if let TracePayload::Sensor(frame) = record.payload() {
                    self.target.sensor_frame(
                        frame.at_us,
                        frame.rpm,
                        frame.map_kpa10.get() as u32,
                        frame.angle_x10,
                    )?;
                    self.last_time_us = record.time_us.get();
                    Ok(())
                } else {
                    Err(ReplayError::UnsupportedRecord)
                }
            }
            TraceInputKind::Tick => {
                if let TracePayload::Sensor(frame) = record.payload() {
                    // Build ControlInputs from the sensor frame
                    // For replay, we use default/zero control inputs since
                    // the original trace may not have stored detailed control intent.
                    // The sensor frame provides the environmental inputs (rpm, map, etc.)
                    // which drive the runtime step.
                    let control = default_control_inputs(record.time_us);
                    self.target.tick(frame.at_us, control)?;
                    self.last_time_us = record.time_us.get();
                    Ok(())
                } else {
                    Err(ReplayError::UnsupportedRecord)
                }
            }
            TraceInputKind::Output => Err(ReplayError::UnsupportedRecord),
        }
    }

    /// Feed a slice of trace records into the target harness.
    ///
    /// Records are processed in order. Processing stops on the first error.
    pub fn replay_all(&mut self, records: &[TraceRecord]) -> Result<(), ReplayError> {
        for record in records {
            self.replay_one(record)?;
        }
        Ok(())
    }
}

/// Create default control inputs for a tick event during replay.
///
/// Since trace records capture sensor and decoder state but not the full
/// control intent, we use reasonable defaults that allow the runtime to
/// produce deterministic outputs.
fn default_control_inputs(now_us: Micros) -> ControlInputs {
    use ecu_domain::Lambda100;
    use ecu_runtime::{EnrichmentInputs, IgnitionInputs, LambdaTrimInputs, TorqueInputs};

    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us,
            clt_c: 80, // Default coolant temp
            cranking: false,
            just_started: false,
            tpsdot_pct_s: 0,
            mapdot_kpa_s: 0,
        },
        lambda: LambdaTrimInputs {
            clt_c: 80,
            lambda_valid: false,
            measured_lambda100: Lambda100::new(100),
            requested_open_loop: false,
        },
        torque: TorqueInputs::new(90, 90, 90, 90, 90),
        ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(1000)),
    }
}

#[cfg(test)]
#[path = "trace_replay_tests.rs"]
mod tests;
