//! Board-independent simulator loop support.
//!
//! This module defines the shared traits, fixed-capacity buffers, and
//! deterministic ordering/report state used by simulator board runners. The
//! x86-specific plant choreography lives in `x86_board.rs`, which builds on
//! these reusable pieces without pulling board glue into the root ECU crate.

use core::cmp::Ordering;

use ecu_domain::Rpm;
use ecu_io::{OutputLevel, OutputTransition};

/// Torque in newton-meters x100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct TorqueNmX100(pub i32);

impl TorqueNmX100 {
    pub const fn new(value: i32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i32 {
        self.0
    }
}

/// Control mode requested by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum SimControlMode {
    #[default]
    OpenLoopRpm,
    ClosedLoopEngine,
}

/// Driver inputs visible to the board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimDriverInput {
    pub throttle_x1000: u16,
    pub requested_rpm: Rpm,
    pub load_torque_nm_x100: TorqueNmX100,
    pub mode: SimControlMode,
}

impl SimDriverInput {
    pub const fn idle() -> Self {
        Self {
            throttle_x1000: 0,
            requested_rpm: Rpm::new(0),
            load_torque_nm_x100: TorqueNmX100::new(0),
            mode: SimControlMode::OpenLoopRpm,
        }
    }
}

/// Environment inputs visible to the board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimEnvironment {
    pub ambient_pressure_pa: i32,
    pub ambient_temp_k_x10: u16,
    pub battery_mv: u16,
}

impl SimEnvironment {
    pub const fn standard() -> Self {
        Self {
            ambient_pressure_pa: 101_325,
            ambient_temp_k_x10: 2931,
            battery_mv: 13_500,
        }
    }
}

/// Sensor frame visible to the board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimSensorFrame {
    pub timestamp_us: u32,
    pub rpm: Rpm,
    pub crank_angle_deg10: u16,
    pub map_kpa10: u16,
    pub tps_x1000: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub lambda_x1000: u16,
    pub battery_mv: u16,
    pub knock_intensity_x100: u16,
}

impl SimSensorFrame {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: 0,
            rpm: Rpm::new(0),
            crank_angle_deg10: 0,
            map_kpa10: 0,
            tps_x1000: 0,
            clt_c10: 0,
            iat_c10: 0,
            lambda_x1000: 1_000,
            battery_mv: 0,
            knock_intensity_x100: 0,
        }
    }
}

/// Trigger line identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimTriggerLine {
    Crank,
    Cam,
}

/// Trigger edge polarity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimEdgePolarity {
    Rising,
    Falling,
}

/// A deterministic trigger edge record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SimTriggerEdge {
    pub timestamp_us: u32,
    pub line: SimTriggerLine,
    pub polarity: SimEdgePolarity,
    pub angle_deg10: u16,
}

impl SimTriggerEdge {
    pub const fn new(
        timestamp_us: u32,
        line: SimTriggerLine,
        polarity: SimEdgePolarity,
        angle_deg10: u16,
    ) -> Self {
        Self {
            timestamp_us,
            line,
            polarity,
            angle_deg10,
        }
    }
}

impl Default for SimTriggerEdge {
    fn default() -> Self {
        Self::new(0, SimTriggerLine::Crank, SimEdgePolarity::Rising, 0)
    }
}

/// A generic trace category for board-level observability.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimBoardTraceKind {
    Tick,
    DriverInput,
    Environment,
    TriggerEdge,
    SensorFrame,
    OutputTransition,
    PlantOutput,
    Note,
}

/// Generic board trace record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SimBoardTraceRecord {
    pub timestamp_us: u32,
    pub kind: SimBoardTraceKind,
    pub channel: u8,
    pub value: i32,
    pub detail: i32,
}

impl SimBoardTraceRecord {
    pub const fn new(
        timestamp_us: u32,
        kind: SimBoardTraceKind,
        channel: u8,
        value: i32,
        detail: i32,
    ) -> Self {
        Self {
            timestamp_us,
            kind,
            channel,
            value,
            detail,
        }
    }
}

impl Default for SimBoardTraceRecord {
    fn default() -> Self {
        Self::new(0, SimBoardTraceKind::Tick, 0, 0, 0)
    }
}

/// Fixed-capacity overflow marker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimBufferOverflow {
    pub capacity: usize,
}

impl SimBufferOverflow {
    pub const fn new(capacity: usize) -> Self {
        Self { capacity }
    }
}

/// Deterministic trigger edge buffer.
pub struct FixedTriggerEdgeBuffer<const N: usize> {
    items: [Option<SimTriggerEdge>; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> FixedTriggerEdgeBuffer<N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn get(&self, index: usize) -> Option<SimTriggerEdge> {
        if index < self.len {
            self.items[index]
        } else {
            None
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = SimTriggerEdge> + '_ {
        self.items[..self.len].iter().copied().flatten()
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn push_sorted(&mut self, edge: SimTriggerEdge) -> Result<(), SimBufferOverflow> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(SimBufferOverflow::new(N));
        }

        let mut idx = self.len;
        while idx > 0 {
            let Some(prev) = self.items[idx - 1] else {
                break;
            };
            if compare_trigger_edges(&prev, &edge) != Ordering::Greater {
                break;
            }
            self.items[idx] = self.items[idx - 1];
            idx -= 1;
        }
        self.items[idx] = Some(edge);
        self.len += 1;
        Ok(())
    }
}

impl<const N: usize> Default for FixedTriggerEdgeBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-capacity output transition queue.
pub struct FixedOutputQueue<const N: usize> {
    items: [Option<OutputTransition>; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> FixedOutputQueue<N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn get(&self, index: usize) -> Option<OutputTransition> {
        if index < self.len {
            self.items[index]
        } else {
            None
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = OutputTransition> + '_ {
        self.items[..self.len].iter().copied().flatten()
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn push_sorted(&mut self, transition: OutputTransition) -> Result<(), SimBufferOverflow> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(SimBufferOverflow::new(N));
        }

        let mut idx = self.len;
        while idx > 0 {
            let Some(prev) = self.items[idx - 1] else {
                break;
            };
            if compare_output_transitions(&prev, &transition) != Ordering::Greater {
                break;
            }
            self.items[idx] = self.items[idx - 1];
            idx -= 1;
        }
        self.items[idx] = Some(transition);
        self.len += 1;
        Ok(())
    }
}

impl<const N: usize> Default for FixedOutputQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Fixed-capacity trace buffer.
pub struct FixedTraceBuffer<const N: usize> {
    items: [Option<SimBoardTraceRecord>; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> FixedTraceBuffer<N> {
    pub const fn new() -> Self {
        Self {
            items: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn get(&self, index: usize) -> Option<SimBoardTraceRecord> {
        if index < self.len {
            self.items[index]
        } else {
            None
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = SimBoardTraceRecord> + '_ {
        self.items[..self.len].iter().copied().flatten()
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn push(&mut self, record: SimBoardTraceRecord) -> Result<(), SimBufferOverflow> {
        if self.len == N {
            self.overflow_count = self.overflow_count.saturating_add(1);
            return Err(SimBufferOverflow::new(N));
        }

        self.items[self.len] = Some(record);
        self.len += 1;
        Ok(())
    }
}

impl<const N: usize> Default for FixedTraceBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Board-level simulator interface.
pub trait SimBoard {
    type Error;
    type OutputBuffer;
    type PlantOutputs;

    fn now_micros(&self) -> u32;

    fn read_driver_input(&mut self) -> Result<SimDriverInput, Self::Error>;
    fn read_environment(&mut self) -> Result<SimEnvironment, Self::Error>;

    fn feed_trigger_edges(&mut self, edges: &[SimTriggerEdge]) -> Result<(), Self::Error>;

    fn feed_sensor_frame(&mut self, sensors: SimSensorFrame) -> Result<(), Self::Error>;

    fn collect_ecu_outputs(&mut self, out: &mut Self::OutputBuffer) -> Result<(), Self::Error>;

    fn publish_plant_outputs(&mut self, outputs: Self::PlantOutputs) -> Result<(), Self::Error>;

    fn write_trace(&mut self, record: SimBoardTraceRecord) -> Result<(), Self::Error>;
}

/// Board-loop configuration knobs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimBoardLoopConfig {
    pub allow_time_regression: bool,
    pub trace_enabled: bool,
}

impl SimBoardLoopConfig {
    pub const fn new() -> Self {
        Self {
            allow_time_regression: false,
            trace_enabled: true,
        }
    }
}

impl Default for SimBoardLoopConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Summary of a loop run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimBoardLoopReport {
    pub tick_count: u64,
    pub last_now_us: u32,
    pub time_regressions: u32,
    pub trigger_edges_queued: u64,
    pub output_transitions_queued: u64,
    pub trace_records_queued: u64,
    pub trigger_overflow_count: u32,
    pub output_overflow_count: u32,
    pub trace_overflow_count: u32,
}

impl SimBoardLoopReport {
    pub const fn new() -> Self {
        Self {
            tick_count: 0,
            last_now_us: 0,
            time_regressions: 0,
            trigger_edges_queued: 0,
            output_transitions_queued: 0,
            trace_records_queued: 0,
            trigger_overflow_count: 0,
            output_overflow_count: 0,
            trace_overflow_count: 0,
        }
    }
}

/// Loop error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimBoardLoopError<E> {
    Board(E),
    TimeWentBackwards { previous_us: u32, now_us: u32 },
    TriggerOverflow(SimBufferOverflow),
    OutputOverflow(SimBufferOverflow),
    TraceOverflow(SimBufferOverflow),
}

/// Board-independent simulator loop state.
///
/// This type owns the deterministic fixed-capacity buffers and report
/// bookkeeping shared by higher-level runners. The x86-specific plant reset,
/// sensor/trigger translation, and ECU feedback choreography live in
/// `x86_board.rs` so the reusable state stays board-agnostic.
pub struct SimBoardLoop<
    B,
    P,
    const MAX_TRIGGER_EDGES: usize,
    const MAX_OUTPUTS: usize,
    const MAX_TRACE: usize,
> where
    B: SimBoard,
{
    pub board: B,
    pub plant: P,
    pub config: SimBoardLoopConfig,
    pub report: SimBoardLoopReport,
    pending_trigger_edges: FixedTriggerEdgeBuffer<MAX_TRIGGER_EDGES>,
    pending_outputs: FixedOutputQueue<MAX_OUTPUTS>,
    trace: FixedTraceBuffer<MAX_TRACE>,
}

impl<B, P, const MAX_TRIGGER_EDGES: usize, const MAX_OUTPUTS: usize, const MAX_TRACE: usize>
    SimBoardLoop<B, P, MAX_TRIGGER_EDGES, MAX_OUTPUTS, MAX_TRACE>
where
    B: SimBoard,
{
    pub const fn new(board: B, plant: P, config: SimBoardLoopConfig) -> Self {
        Self {
            board,
            plant,
            config,
            report: SimBoardLoopReport::new(),
            pending_trigger_edges: FixedTriggerEdgeBuffer::new(),
            pending_outputs: FixedOutputQueue::new(),
            trace: FixedTraceBuffer::new(),
        }
    }

    pub fn pending_trigger_edges(&self) -> &FixedTriggerEdgeBuffer<MAX_TRIGGER_EDGES> {
        &self.pending_trigger_edges
    }

    pub fn pending_trigger_edges_mut(&mut self) -> &mut FixedTriggerEdgeBuffer<MAX_TRIGGER_EDGES> {
        &mut self.pending_trigger_edges
    }

    pub fn pending_outputs(&self) -> &FixedOutputQueue<MAX_OUTPUTS> {
        &self.pending_outputs
    }

    pub fn pending_outputs_mut(&mut self) -> &mut FixedOutputQueue<MAX_OUTPUTS> {
        &mut self.pending_outputs
    }

    pub fn trace(&self) -> &FixedTraceBuffer<MAX_TRACE> {
        &self.trace
    }

    pub fn trace_mut(&mut self) -> &mut FixedTraceBuffer<MAX_TRACE> {
        &mut self.trace
    }

    pub fn board_mut(&mut self) -> &mut B {
        &mut self.board
    }

    pub fn plant_mut(&mut self) -> &mut P {
        &mut self.plant
    }

    pub fn into_parts(
        self,
    ) -> (
        B,
        P,
        SimBoardLoopConfig,
        SimBoardLoopReport,
        FixedTriggerEdgeBuffer<MAX_TRIGGER_EDGES>,
        FixedOutputQueue<MAX_OUTPUTS>,
        FixedTraceBuffer<MAX_TRACE>,
    ) {
        (
            self.board,
            self.plant,
            self.config,
            self.report,
            self.pending_trigger_edges,
            self.pending_outputs,
            self.trace,
        )
    }

    pub fn tick(&mut self) -> Result<u32, SimBoardLoopError<B::Error>> {
        let now_us = self.board.now_micros();
        if !self.config.allow_time_regression && now_us < self.report.last_now_us {
            self.report.time_regressions = self.report.time_regressions.saturating_add(1);
            return Err(SimBoardLoopError::TimeWentBackwards {
                previous_us: self.report.last_now_us,
                now_us,
            });
        }

        self.report.tick_count = self.report.tick_count.saturating_add(1);
        self.report.last_now_us = now_us;
        Ok(now_us)
    }

    pub fn queue_trigger_edge(
        &mut self,
        edge: SimTriggerEdge,
    ) -> Result<(), SimBoardLoopError<B::Error>> {
        match self.pending_trigger_edges.push_sorted(edge) {
            Ok(()) => {
                self.report.trigger_edges_queued =
                    self.report.trigger_edges_queued.saturating_add(1);
                Ok(())
            }
            Err(overflow) => {
                self.report.trigger_overflow_count = self.pending_trigger_edges.overflow_count();
                Err(SimBoardLoopError::TriggerOverflow(overflow))
            }
        }
    }

    pub fn queue_output_transition(
        &mut self,
        transition: OutputTransition,
    ) -> Result<(), SimBoardLoopError<B::Error>> {
        match self.pending_outputs.push_sorted(transition) {
            Ok(()) => {
                self.report.output_transitions_queued =
                    self.report.output_transitions_queued.saturating_add(1);
                Ok(())
            }
            Err(overflow) => {
                self.report.output_overflow_count = self.pending_outputs.overflow_count();
                Err(SimBoardLoopError::OutputOverflow(overflow))
            }
        }
    }

    pub fn queue_trace_record(
        &mut self,
        record: SimBoardTraceRecord,
    ) -> Result<(), SimBoardLoopError<B::Error>> {
        if !self.config.trace_enabled {
            return Ok(());
        }

        match self.trace.push(record) {
            Ok(()) => {
                self.report.trace_records_queued =
                    self.report.trace_records_queued.saturating_add(1);
                Ok(())
            }
            Err(overflow) => {
                self.report.trace_overflow_count = self.trace.overflow_count();
                Err(SimBoardLoopError::TraceOverflow(overflow))
            }
        }
    }
}

/// Compare trigger edges by timestamp, then line, polarity, and angle.
pub fn compare_trigger_edges(lhs: &SimTriggerEdge, rhs: &SimTriggerEdge) -> Ordering {
    (lhs.timestamp_us, lhs.line, lhs.polarity, lhs.angle_deg10).cmp(&(
        rhs.timestamp_us,
        rhs.line,
        rhs.polarity,
        rhs.angle_deg10,
    ))
}

/// Compare output transitions by timestamp, then kind, channel, and level.
pub fn compare_output_transitions(lhs: &OutputTransition, rhs: &OutputTransition) -> Ordering {
    (
        lhs.at_us.get(),
        lhs.kind,
        lhs.channel.get(),
        output_level_rank(lhs.level),
    )
        .cmp(&(
            rhs.at_us.get(),
            rhs.kind,
            rhs.channel.get(),
            output_level_rank(rhs.level),
        ))
}

/// Sort trigger edges deterministically in place.
pub fn sort_trigger_edges(edges: &mut [SimTriggerEdge]) {
    edges.sort_by(compare_trigger_edges);
}

/// Sort output transitions deterministically in place.
pub fn sort_output_transitions(outputs: &mut [OutputTransition]) {
    outputs.sort_by(compare_output_transitions);
}

fn output_level_rank(level: OutputLevel) -> u8 {
    match level {
        OutputLevel::Low => 0,
        OutputLevel::High => 1,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::{ChannelId, Micros};
    use ecu_io::OutputTransitionKind;

    #[test]
    fn same_timestamp_trigger_edges_keep_deterministic_order() {
        let mut buffer: FixedTriggerEdgeBuffer<4> = FixedTriggerEdgeBuffer::new();

        buffer
            .push_sorted(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Cam,
                SimEdgePolarity::Falling,
                300,
            ))
            .unwrap();
        buffer
            .push_sorted(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Crank,
                SimEdgePolarity::Falling,
                200,
            ))
            .unwrap();
        buffer
            .push_sorted(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Crank,
                SimEdgePolarity::Rising,
                250,
            ))
            .unwrap();
        buffer
            .push_sorted(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Crank,
                SimEdgePolarity::Rising,
                100,
            ))
            .unwrap();

        assert_eq!(
            buffer.get(0),
            Some(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Crank,
                SimEdgePolarity::Rising,
                100,
            ))
        );
        assert_eq!(
            buffer.get(1),
            Some(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Crank,
                SimEdgePolarity::Rising,
                250,
            ))
        );
        assert_eq!(
            buffer.get(2),
            Some(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Crank,
                SimEdgePolarity::Falling,
                200,
            ))
        );
        assert_eq!(
            buffer.get(3),
            Some(SimTriggerEdge::new(
                1_000,
                SimTriggerLine::Cam,
                SimEdgePolarity::Falling,
                300,
            ))
        );
    }

    #[test]
    fn fixed_buffer_overflow_is_explicit() {
        let mut trace: FixedTraceBuffer<1> = FixedTraceBuffer::new();
        assert_eq!(trace.overflow_count(), 0);

        assert!(trace
            .push(SimBoardTraceRecord::new(
                100,
                SimBoardTraceKind::Tick,
                0,
                1,
                0
            ))
            .is_ok());

        let overflow = trace
            .push(SimBoardTraceRecord::new(
                101,
                SimBoardTraceKind::Note,
                1,
                2,
                3,
            ))
            .unwrap_err();

        assert_eq!(overflow, SimBufferOverflow::new(1));
        assert_eq!(trace.overflow_count(), 1);
        assert_eq!(trace.len(), 1);
    }

    #[test]
    fn output_transitions_sort_same_timestamp_deterministically() {
        let mut outputs = [
            OutputTransition {
                at_us: Micros::new(1_000),
                kind: OutputTransitionKind::Fan,
                channel: ChannelId::new(3),
                level: OutputLevel::High,
            },
            OutputTransition {
                at_us: Micros::new(1_000),
                kind: OutputTransitionKind::Injector,
                channel: ChannelId::new(2),
                level: OutputLevel::High,
            },
            OutputTransition {
                at_us: Micros::new(1_000),
                kind: OutputTransitionKind::Injector,
                channel: ChannelId::new(1),
                level: OutputLevel::Low,
            },
        ];

        sort_output_transitions(&mut outputs);

        assert_eq!(outputs[0].kind, OutputTransitionKind::Injector);
        assert_eq!(outputs[0].channel.get(), 1);
        assert_eq!(outputs[0].level, OutputLevel::Low);
        assert_eq!(outputs[1].kind, OutputTransitionKind::Injector);
        assert_eq!(outputs[1].channel.get(), 2);
        assert_eq!(outputs[1].level, OutputLevel::High);
        assert_eq!(outputs[2].kind, OutputTransitionKind::Fan);
    }
}
