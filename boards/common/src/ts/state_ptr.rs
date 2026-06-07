//! Single audited home for the raw-pointer indirection that TS providers need.
//!
//! The TS service (`TsService`) owns its `OutpcProvider`/`PageStoreProvider` for its
//! whole lifetime, while the board's engine state/runtime is borrowed mutably elsewhere
//! in the main loop. A stored `&mut` would alias; a stored raw pointer dereferenced
//! transiently per call is the established workaround. Centralising it here keeps the
//! raw-pointer field and its deref out of every board's TS-provider source so the
//! architecture-debt ratchet can forbid them there.
//!
//! Two handles split the read-only and read-write paths so the `from_ref` + `with_mut`
//! footgun (forming `&mut` to shared-only state) is unrepresentable:
//!
//! - [`StateRef`] is built from `&T` and exposes only [`StateRef::with`].
//! - [`StatePtr`] is built from `&mut T` and exposes [`StatePtr::with`]/[`StatePtr::with_mut`].
//!
//! Both hold a raw pointer, so both are `!Send + !Sync` automatically; no manual
//! `Send`/`Sync` impls are added.

/// Read-only handle to board-owned state that outlives any single TS-provider call.
///
/// Construct from a shared reference; deref only transiently inside a provider callback.
pub struct StateRef<T> {
    ptr: *const T,
}

impl<T> StateRef<T> {
    /// Captures a read-only address of board-owned state.
    pub fn new(state: &T) -> Self {
        Self {
            ptr: state as *const T,
        }
    }

    /// Runs `f` with a shared borrow of the captured state.
    ///
    /// # Safety
    /// The caller guarantees the captured state is alive and not mutably
    /// borrowed for the duration of `f`. The TS service only invokes provider
    /// callbacks transiently from the main loop while the engine task is idle,
    /// so no concurrent borrow exists.
    pub unsafe fn with<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        let state = unsafe { &*self.ptr };
        f(state)
    }
}

/// Read-write handle to board-owned state that outlives any single TS-provider call.
///
/// Construct from an exclusive reference; deref only transiently inside a provider callback.
pub struct StatePtr<T> {
    ptr: *mut T,
}

impl<T> StatePtr<T> {
    /// Captures the address of board-owned state.
    pub fn new(state: &mut T) -> Self {
        Self {
            ptr: state as *mut T,
        }
    }

    /// Runs `f` with a shared borrow of the captured state.
    ///
    /// # Safety
    /// The caller guarantees the captured state is alive and not otherwise
    /// borrowed for the duration of `f`. The TS service only invokes provider
    /// callbacks transiently from the main loop while the engine task is idle,
    /// so no concurrent borrow exists.
    pub unsafe fn with<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        let state = unsafe { &*self.ptr };
        f(state)
    }

    /// Runs `f` with an exclusive borrow of the captured state.
    ///
    /// # Safety
    /// See [`StatePtr::with`]; additionally no other borrow (shared or exclusive)
    /// may exist for the duration of `f`.
    pub unsafe fn with_mut<R>(&self, f: impl FnOnce(&mut T) -> R) -> R {
        let state = unsafe { &mut *self.ptr };
        f(state)
    }
}
