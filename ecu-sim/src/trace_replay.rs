//! Trace replay helpers for feeding recorded traces into simulation harnesses.
//!
//! Provides `TraceReplay` for deterministic replay of edge, sensor, and tick
//! traces into `ecu_sim::SimulationHarness`. Root integration tests can use
//! a compatible wrapper to target the live `EcuApp` harness.

use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_io::{EdgeLine, EdgePolarity, TraceInputKind, TracePayload, TraceRecord};
use ecu_runtime::ControlInputs;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReplayError {
    /// The trace record has an unsupported input/payload combination.
    UnsupportedRecord,
    /// Trace time is not monotonically increasing.
    NonMonotonicTime,
    /// The destination harness queue is full.
    QueueFull,
    /// The capture buffer is full.
    CaptureFull,
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
    last_tick_was_injected: bool,
}

impl<T> TraceReplay<T> {
    /// Create a new trace replay targeting the given harness.
    pub fn new(target: T) -> Self {
        Self {
            target,
            last_time_us: 0,
            last_tick_was_injected: false,
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
        crate::SimulationHarness::sensor_frame(
            self,
            at_us,
            rpm,
            Kpa10::new(load_kpa10 as u16),
            angle_x10,
        )
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
    /// Tick record missing required control inputs.
    MissingControlInputs,
}

impl<T: ReplayTarget> TraceReplay<T> {
    /// Validate that a trace record's input_kind and payload are consistent.
    pub fn validate_record(record: &TraceRecord) -> Result<(), RecordValidationError> {
        match (record.input_kind, &record.payload) {
            // Valid combinations
            (TraceInputKind::None, TracePayload::None) => Ok(()),
            (TraceInputKind::Edge, TracePayload::Edge(_)) => Ok(()),
            (TraceInputKind::Sensor, TracePayload::Sensor(_)) => Ok(()),
            (TraceInputKind::Tick, TracePayload::Sensor(_)) => Ok(()),
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
        match record.input_kind {
            TraceInputKind::None => {
                // TracePayload::None only - nothing to feed, just update time
                self.last_time_us = record.time_us.get();
                Ok(())
            }
            TraceInputKind::Edge => {
                if let TracePayload::Edge(edge) = record.payload {
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
                if let TracePayload::Sensor(frame) = record.payload {
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
                if let TracePayload::Sensor(frame) = record.payload {
                    // Build ControlInputs from the sensor frame
                    // For replay, we use default/zero control inputs since
                    // the original trace may not have stored detailed control intent.
                    // The sensor frame provides the environmental inputs (rpm, map, etc.)
                    // which drive the runtime step.
                    let control = default_control_inputs(record.time_us);
                    self.target.tick(frame.at_us, control)?;
                    self.last_time_us = record.time_us.get();
                    self.last_tick_was_injected = true;
                    Ok(())
                } else {
                    Err(ReplayError::UnsupportedRecord)
                }
            }
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
mod tests {
    use super::*;
    use crate::SimulationHarness;
    use ecu_domain::{FaultCode, Kpa10};
    use ecu_io::{EdgeSample, SensorFrame};

    /// Helper: make a TraceRecord for an edge event.
    fn make_edge_record(
        step_index: u32,
        time_us: u32,
        line: EdgeLine,
        polarity: EdgePolarity,
        rpm: u16,
        angle_x10: i16,
        synced: bool,
    ) -> TraceRecord {
        TraceRecord {
            step_index,
            time_us: Micros::new(time_us),
            input_kind: TraceInputKind::Edge,
            payload: TracePayload::Edge(EdgeSample {
                at_us: Micros::new(time_us),
                line,
                polarity,
                angle_x10: Degrees10::new(angle_x10),
                rpm: Rpm::new(rpm),
            }),
            rpm: Rpm::new(rpm),
            synced,
            tooth: 0,
            angle_x10: Degrees10::new(angle_x10),
            diagnostic_code: FaultCode::None,
        }
    }

    /// Helper: make a TraceRecord for a sensor frame.
    fn make_sensor_record(
        step_index: u32,
        time_us: u32,
        rpm: u16,
        map_kpa10: u16,
        angle_x10: i16,
    ) -> TraceRecord {
        TraceRecord {
            step_index,
            time_us: Micros::new(time_us),
            input_kind: TraceInputKind::Sensor,
            payload: TracePayload::Sensor(SensorFrame {
                at_us: Micros::new(time_us),
                rpm: Rpm::new(rpm),
                map_kpa10: Kpa10::new(map_kpa10),
                angle_x10: Degrees10::new(angle_x10),
                tps_x100: 1000,
                clt_c10: 800,
                iat_c10: 300,
                vbatt_mv: 12400,
                baro_kpa10: Kpa10::new(1000),
                lambda_valid: false,
                lambda_x100: ecu_domain::Lambda100::new(100),
            }),
            rpm: Rpm::new(rpm),
            synced: true,
            tooth: 0,
            angle_x10: Degrees10::new(angle_x10),
            diagnostic_code: FaultCode::None,
        }
    }

    /// Helper: make a TraceRecord for a tick event.
    fn make_tick_record(step_index: u32, time_us: u32, rpm: u16) -> TraceRecord {
        TraceRecord {
            step_index,
            time_us: Micros::new(time_us),
            input_kind: TraceInputKind::Tick,
            payload: TracePayload::Sensor(SensorFrame {
                at_us: Micros::new(time_us),
                rpm: Rpm::new(rpm),
                map_kpa10: Kpa10::new(600),
                angle_x10: Degrees10::new(0),
                tps_x100: 1000,
                clt_c10: 800,
                iat_c10: 300,
                vbatt_mv: 12400,
                baro_kpa10: Kpa10::new(1000),
                lambda_valid: false,
                lambda_x100: ecu_domain::Lambda100::new(100),
            }),
            rpm: Rpm::new(rpm),
            synced: true,
            tooth: 0,
            angle_x10: Degrees10::new(0),
            diagnostic_code: FaultCode::None,
        }
    }

    /// Helper: make a TraceRecord for a None input.
    fn make_none_record(step_index: u32, time_us: u32) -> TraceRecord {
        TraceRecord {
            step_index,
            time_us: Micros::new(time_us),
            input_kind: TraceInputKind::None,
            payload: TracePayload::None,
            rpm: Rpm::new(0),
            synced: false,
            tooth: 0,
            angle_x10: Degrees10::new(0),
            diagnostic_code: FaultCode::None,
        }
    }

    #[test]
    fn replay_rejects_nonmonotonic_time() {
        let sim: SimulationHarness<4, 2> = SimulationHarness::default();
        let mut replay = TraceReplay::new(sim);

        let record1 = make_edge_record(
            0,
            100,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            0,
            false,
        );
        let record2 = make_edge_record(
            1,
            50, // Time going backwards!
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            60,
            false,
        );

        assert!(replay.replay_one(&record1).is_ok());
        let result = replay.replay_one(&record2);
        assert!(matches!(result, Err(ReplayError::NonMonotonicTime)));
    }

    #[test]
    fn replay_validates_record_consistency() {
        let sim: SimulationHarness<4, 2> = SimulationHarness::default();
        let mut replay = TraceReplay::new(sim);

        // Edge input with Sensor payload - invalid
        let invalid_record = TraceRecord {
            step_index: 0,
            time_us: Micros::new(100),
            input_kind: TraceInputKind::Edge,
            payload: TracePayload::Sensor(SensorFrame {
                at_us: Micros::new(100),
                rpm: Rpm::new(1200),
                map_kpa10: Kpa10::new(600),
                angle_x10: Degrees10::new(0),
                tps_x100: 1000,
                clt_c10: 800,
                iat_c10: 300,
                vbatt_mv: 12400,
                baro_kpa10: Kpa10::new(1000),
                lambda_valid: false,
                lambda_x100: ecu_domain::Lambda100::new(100),
            }),
            rpm: Rpm::new(1200),
            synced: false,
            tooth: 0,
            angle_x10: Degrees10::new(0),
            diagnostic_code: FaultCode::None,
        };

        let result = replay.replay_one(&invalid_record);
        assert!(matches!(result, Err(ReplayError::UnsupportedRecord)));
    }

    #[test]
    fn replay_ignores_none_input_with_none_payload() {
        let sim: SimulationHarness<4, 2> = SimulationHarness::default();
        let mut replay = TraceReplay::new(sim);

        let record = make_none_record(0, 100);
        assert!(replay.replay_one(&record).is_ok());
    }

    #[test]
    fn replay_feeds_edge_records_as_edges() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
        let mut replay = TraceReplay::new(&mut sim);

        let crank_record = make_edge_record(
            0,
            100,
            EdgeLine::Crank,
            EdgePolarity::Rising,
            1200,
            0,
            false,
        );
        let cam_record =
            make_edge_record(1, 102, EdgeLine::Cam, EdgePolarity::Rising, 1200, 0, true);

        assert!(replay.replay_one(&crank_record).is_ok());
        assert!(replay.replay_one(&cam_record).is_ok());

        // Drain and verify
        let count = sim.drain_until_idle();
        assert_eq!(count, 2);
    }

    #[test]
    fn replay_feeds_sensor_records_as_sensor_frames() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
        let mut replay = TraceReplay::new(&mut sim);

        let record = make_sensor_record(0, 100, 1800, 600, 0);
        assert!(replay.replay_one(&record).is_ok());

        sim.drain_until_idle();
        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.rpm.get(), 1800);
    }

    #[test]
    fn replay_feeds_tick_records_as_ticks() {
        let mut sim: SimulationHarness<4, 2> = SimulationHarness::default();
        let mut replay = TraceReplay::new(&mut sim);

        let record = make_tick_record(0, 100, 1200);
        assert!(replay.replay_one(&record).is_ok());

        sim.drain_until_idle();
        let result = sim.last_result();
        assert!(result.is_some());
    }

    #[test]
    fn replay_same_trace_twice_produces_identical_results() {
        // Create a simple trace
        let trace: &[TraceRecord] = &[
            make_edge_record(
                0,
                100,
                EdgeLine::Crank,
                EdgePolarity::Rising,
                1200,
                0,
                false,
            ),
            make_edge_record(
                1,
                160,
                EdgeLine::Crank,
                EdgePolarity::Rising,
                1200,
                60,
                false,
            ),
            make_sensor_record(2, 170, 1200, 600, 60),
            make_tick_record(3, 200, 1200),
        ];

        // First replay
        let mut sim1: SimulationHarness<8, 4> = SimulationHarness::default();
        let mut replay1 = TraceReplay::new(&mut sim1);
        replay1.replay_all(trace).unwrap();
        sim1.drain_until_idle();

        // Get first result
        let result1 = sim1.last_result();

        // Second replay
        let mut sim2: SimulationHarness<8, 4> = SimulationHarness::default();
        let mut replay2 = TraceReplay::new(&mut sim2);
        replay2.replay_all(trace).unwrap();
        sim2.drain_until_idle();

        // Get second result
        let result2 = sim2.last_result();

        // Results should be identical
        assert_eq!(result1, result2);
    }

    #[test]
    fn unsupported_record_combinations_return_unsupported_record() {
        // Test: Edge input with None payload
        {
            let record = TraceRecord {
                step_index: 0,
                time_us: Micros::new(100),
                input_kind: TraceInputKind::Edge,
                payload: TracePayload::None,
                rpm: Rpm::new(0),
                synced: false,
                tooth: 0,
                angle_x10: Degrees10::new(0),
                diagnostic_code: FaultCode::None,
            };
            assert!(matches!(
                TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record),
                Err(RecordValidationError::UnsupportedRecord)
            ));
        }

        // Test: Sensor input with Edge payload
        {
            let record = TraceRecord {
                step_index: 0,
                time_us: Micros::new(100),
                input_kind: TraceInputKind::Sensor,
                payload: TracePayload::Edge(EdgeSample {
                    at_us: Micros::new(100),
                    line: EdgeLine::Crank,
                    polarity: EdgePolarity::Rising,
                    angle_x10: Degrees10::new(0),
                    rpm: Rpm::new(1200),
                }),
                rpm: Rpm::new(1200),
                synced: false,
                tooth: 0,
                angle_x10: Degrees10::new(0),
                diagnostic_code: FaultCode::None,
            };
            assert!(matches!(
                TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record),
                Err(RecordValidationError::UnsupportedRecord)
            ));
        }

        // Test: Tick input with None payload
        {
            let record = TraceRecord {
                step_index: 0,
                time_us: Micros::new(100),
                input_kind: TraceInputKind::Tick,
                payload: TracePayload::None,
                rpm: Rpm::new(0),
                synced: false,
                tooth: 0,
                angle_x10: Degrees10::new(0),
                diagnostic_code: FaultCode::None,
            };
            assert!(matches!(
                TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record),
                Err(RecordValidationError::UnsupportedRecord)
            ));
        }
    }

    #[test]
    fn validate_record_accepts_valid_combinations() {
        // None input + None payload
        {
            let record = make_none_record(0, 100);
            assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
        }

        // Edge input + Edge payload
        {
            let record = make_edge_record(
                0,
                100,
                EdgeLine::Crank,
                EdgePolarity::Rising,
                1200,
                0,
                false,
            );
            assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
        }

        // Sensor input + Sensor payload
        {
            let record = make_sensor_record(0, 100, 1800, 600, 0);
            assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
        }

        // Tick input + Sensor payload
        {
            let record = make_tick_record(0, 100, 1200);
            assert!(TraceReplay::<SimulationHarness<4, 2>>::validate_record(&record).is_ok());
        }
    }

    #[test]
    fn replay_preserves_order_for_equal_timestamps() {
        let mut sim: SimulationHarness<8, 4> = SimulationHarness::default();
        let mut replay = TraceReplay::new(&mut sim);

        // Create multiple records with same timestamp
        let records = &[
            make_edge_record(
                0,
                100,
                EdgeLine::Crank,
                EdgePolarity::Rising,
                1200,
                0,
                false,
            ),
            make_edge_record(1, 100, EdgeLine::Cam, EdgePolarity::Rising, 1200, 0, true),
            make_sensor_record(2, 100, 1200, 600, 0),
        ];

        // All should succeed - order is preserved
        for record in records {
            assert!(replay.replay_one(record).is_ok());
        }

        // All events should be in the queue
        let count = sim.drain_until_idle();
        assert_eq!(count, 3);
    }

    #[test]
    fn sixty_minus_two_generated_trace_replays_into_runtime_harness() {
        use crate::trigger_pattern::{MissingToothEdgeGenerator, MissingToothPattern};

        // Generate a 60-2 trace for exactly one revolution.
        // The 60-2 pattern produces 60 emitted edges per revolution:
        // 1 cam edge (at cam_phase_deg10=0) + 59 crank edges.
        // Using a 60-element array ensures we stop at the revolution boundary
        // before any GAP_EXIT re-emits tooth 0 at time=0 (which would violate
        // monotonic time ordering in the trace).
        let pattern = MissingToothPattern::sixty_minus_two();
        let mut gen = MissingToothEdgeGenerator::new(pattern).unwrap();
        gen.set_rpm(Rpm::new(1000)).unwrap();

        let mut trace = smallvec::SmallVec::<[TraceRecord; 64]>::new();
        let mut edges = [None; 60];
        let count = gen.next_edges(&mut edges).unwrap();
        assert_eq!(count, 60); // One full revolution: 1 cam + 59 crank

        for (i, edge_opt) in edges.iter().enumerate() {
            let edge = edge_opt.expect("should have 60 edges");
            let record = TraceRecord {
                step_index: i as u32,
                time_us: edge.at_us,
                input_kind: TraceInputKind::Edge,
                payload: TracePayload::Edge(edge),
                rpm: edge.rpm,
                synced: false,
                tooth: 0,
                angle_x10: edge.angle_x10,
                diagnostic_code: FaultCode::None,
            };
            trace.push(record);
        }

        // Replay into runtime harness
        let mut sim: SimulationHarness<128, 16> = SimulationHarness::default();
        let mut replay = TraceReplay::new(&mut sim);

        // Replay one by one to find which edge fails
        for (i, record) in trace.iter().enumerate() {
            if replay.replay_one(record).is_err() {
                panic!("Replay failed at edge {}", i);
            }
        }

        // Check queue state after replay

        // All 60 records should have been processed
        let processed = sim.drain_until_idle();
        assert_eq!(processed, 2, "Coalescing queue stores 2 coalesced events");

        // Runtime should have processed edges (RPM should be set)
        let snapshot = sim.runtime().snapshot();
        assert_eq!(snapshot.engine.rpm.get(), 1000);
    }
}
