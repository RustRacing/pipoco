use std::sync::{Arc, Condvar, Mutex, OnceLock};

use crate::{EcuSimHandleOpaque, EcuSimStatus};

use super::model::EcuSimHandle;

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
