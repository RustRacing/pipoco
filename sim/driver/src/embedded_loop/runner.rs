//! Loop state used by board runners.

use ecu_io::OutputTransition;

use super::{
    FixedOutputQueue, FixedTraceBuffer, FixedTriggerEdgeBuffer, SimBoard, SimBoardTraceRecord,
    SimBufferOverflow, SimTriggerEdge,
};

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
/// bookkeeping shared by higher-level runners.
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
