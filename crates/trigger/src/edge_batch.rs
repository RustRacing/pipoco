use ecu_domain::Ticks;

use crate::profile::TriggerEdge;

/// Timestamped primary edge captured by board code before decoder ingestion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct PrimaryEdgeSample {
    pub timestamp: Ticks,
    pub edge: TriggerEdge,
}

impl PrimaryEdgeSample {
    pub const EMPTY: Self = Self {
        timestamp: Ticks::new(0),
        edge: TriggerEdge::Rising,
    };

    pub const fn new(timestamp: Ticks) -> Self {
        Self::with_edge(timestamp, TriggerEdge::Rising)
    }

    pub const fn with_edge(timestamp: Ticks, edge: TriggerEdge) -> Self {
        Self { timestamp, edge }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PrimaryEdgeBatchError {
    Full,
}

/// Fixed-capacity primary edge batch for allocation-free ISR-to-decoder handoff.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PrimaryEdgeBatch<const CAPACITY: usize> {
    edges: [PrimaryEdgeSample; CAPACITY],
    len: usize,
}

impl<const CAPACITY: usize> Default for PrimaryEdgeBatch<CAPACITY> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAPACITY: usize> PrimaryEdgeBatch<CAPACITY> {
    pub const fn new() -> Self {
        Self {
            edges: [PrimaryEdgeSample::EMPTY; CAPACITY],
            len: 0,
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn capacity(&self) -> usize {
        CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub const fn is_full(&self) -> bool {
        self.len == CAPACITY
    }

    pub fn push(&mut self, edge: PrimaryEdgeSample) -> Result<(), PrimaryEdgeBatchError> {
        if self.is_full() {
            return Err(PrimaryEdgeBatchError::Full);
        }

        self.edges[self.len] = edge;
        self.len += 1;
        Ok(())
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn as_slice(&self) -> &[PrimaryEdgeSample] {
        &self.edges[..self.len]
    }
}
