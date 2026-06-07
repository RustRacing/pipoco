//! Fixed-capacity buffers for loop orchestration.

use core::cmp::Ordering;

use ecu_io::{OutputLevel, OutputTransition};

use super::{SimBoardTraceRecord, SimBufferOverflow, SimTriggerEdge};

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
            if compare_trigger_edges_for_queue(&prev, &edge) != Ordering::Greater {
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

fn compare_trigger_edges_for_queue(lhs: &SimTriggerEdge, rhs: &SimTriggerEdge) -> Ordering {
    (lhs.timestamp_us, lhs.line, lhs.polarity, lhs.angle_deg10).cmp(&(
        rhs.timestamp_us,
        rhs.line,
        rhs.polarity,
        rhs.angle_deg10,
    ))
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
            if compare_output_transitions_for_queue(&prev, &transition) != Ordering::Greater {
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

fn output_level_rank(level: OutputLevel) -> u8 {
    match level {
        OutputLevel::Low => 0,
        OutputLevel::High => 1,
    }
}

fn compare_output_transitions_for_queue(
    lhs: &OutputTransition,
    rhs: &OutputTransition,
) -> Ordering {
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
