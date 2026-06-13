use super::units::{CylinderId, Degrees10};

pub const MAX_EVENTS_PER_STEP: usize = 64;
pub const MAX_PENDING_EVENTS: usize = 64;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventKind {
    InjectionOpen,
    InjectionClose,
    CoilChargeStart,
    CoilFire,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SemanticEvent {
    pub kind: EventKind,
    pub cylinder: CylinderId,
    pub angle_deg10: Degrees10,
}

impl Default for SemanticEvent {
    fn default() -> Self {
        Self {
            kind: EventKind::InjectionOpen,
            cylinder: CylinderId::default(),
            angle_deg10: Degrees10::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EventBatch {
    pub len: u8,
    pub events: [SemanticEvent; MAX_EVENTS_PER_STEP],
}

impl Default for EventBatch {
    fn default() -> Self {
        Self {
            len: 0,
            events: [SemanticEvent::default(); MAX_EVENTS_PER_STEP],
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EventBatchFull {
    Full,
}

impl EventBatch {
    pub fn push(&mut self, event: SemanticEvent) -> Result<(), EventBatchFull> {
        let len = self.len as usize;
        if len >= MAX_EVENTS_PER_STEP {
            return Err(EventBatchFull::Full);
        }
        self.events[len] = event;
        self.len += 1;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_batch_push_overflow_is_observable_and_len_is_stable() {
        let mut batch = EventBatch::default();
        let event = SemanticEvent::default();
        for _ in 0..MAX_EVENTS_PER_STEP {
            assert_eq!(batch.push(event), Ok(()));
        }
        let len_before = batch.len;
        assert_eq!(len_before as usize, MAX_EVENTS_PER_STEP);
        assert_eq!(batch.push(event), Err(EventBatchFull::Full));
        assert_eq!(batch.len, len_before);
    }
}
