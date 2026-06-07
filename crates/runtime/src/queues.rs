use ecu_domain::{Kpa10, Micros, Rpm};

/// Fast-path runtime observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum FastEvent {
    TriggerEdge { at_us: Micros },
    SensorSample { rpm: Rpm, load_kpa10: Kpa10 },
}

/// Slow-path runtime observations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum SlowEvent {
    CalibrationCommitted,
    SnapshotRequested,
    PersistRequested,
}

/// Runtime event surface split into fast and slow lanes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum Event {
    Fast(FastEvent),
    Slow(SlowEvent),
}

impl Event {
    fn key(self) -> EventKey {
        match self {
            Event::Fast(FastEvent::TriggerEdge { .. }) => EventKey::FastTriggerEdge,
            Event::Fast(FastEvent::SensorSample { .. }) => EventKey::FastSensorSample,
            Event::Slow(SlowEvent::CalibrationCommitted) => EventKey::SlowCalibrationCommitted,
            Event::Slow(SlowEvent::SnapshotRequested) => EventKey::SlowSnapshotRequested,
            Event::Slow(SlowEvent::PersistRequested) => EventKey::SlowPersistRequested,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EventKey {
    FastTriggerEdge,
    FastSensorSample,
    SlowCalibrationCommitted,
    SlowSnapshotRequested,
    SlowPersistRequested,
}

/// Result of queueing an event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum QueueResult {
    Enqueued,
    Coalesced,
}

/// Queue overflow classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum QueueOverflow {
    FastFull,
    SlowFull,
}

/// Fixed-size runtime queue split.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct RuntimeQueues<const FAST: usize, const SLOW: usize> {
    fast: Queue<Event, FAST>,
    slow: Queue<Event, SLOW>,
}

impl<const FAST: usize, const SLOW: usize> RuntimeQueues<FAST, SLOW> {
    pub(crate) const fn new() -> Self {
        Self {
            fast: Queue::new(),
            slow: Queue::new(),
        }
    }

    pub(crate) fn push(&mut self, event: Event) -> Result<QueueResult, QueueOverflow> {
        match event {
            Event::Fast(_) => self
                .fast
                .push_coalescing(event)
                .map_err(|_| QueueOverflow::FastFull),
            Event::Slow(_) => self
                .slow
                .push_fifo(event)
                .map_err(|_| QueueOverflow::SlowFull),
        }
    }

    pub(crate) fn pop_fast(&mut self) -> Option<Event> {
        self.fast.pop()
    }

    pub(crate) fn pop_slow(&mut self) -> Option<Event> {
        self.slow.pop()
    }
}

impl<const FAST: usize, const SLOW: usize> Default for RuntimeQueues<FAST, SLOW> {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct Queue<T: Copy, const N: usize> {
    buf: [Option<T>; N],
    head: usize,
    len: usize,
}

impl<T: Copy, const N: usize> Queue<T, N> {
    const fn new() -> Self {
        Self {
            buf: [None; N],
            head: 0,
            len: 0,
        }
    }

    fn pop(&mut self) -> Option<T> {
        if self.len == 0 {
            return None;
        }
        let item = self.buf[self.head].take();
        self.head = (self.head + 1) % N.max(1);
        self.len -= 1;
        item
    }

    fn push_fifo(&mut self, item: T) -> Result<QueueResult, ()> {
        if self.len == N {
            return Err(());
        }
        let tail = (self.head + self.len) % N.max(1);
        self.buf[tail] = Some(item);
        self.len += 1;
        Ok(QueueResult::Enqueued)
    }
}

impl<const N: usize> Queue<Event, N> {
    fn push_coalescing(&mut self, item: Event) -> Result<QueueResult, ()> {
        let key = item.key();
        let mut idx = 0usize;
        while idx < self.len {
            let slot = (self.head + idx) % N.max(1);
            if self.buf[slot].map(|existing| existing.key()) == Some(key) {
                self.buf[slot] = Some(item);
                return Ok(QueueResult::Coalesced);
            }
            idx += 1;
        }

        self.push_fifo(item)
    }
}
