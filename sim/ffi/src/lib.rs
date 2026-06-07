//! Host-only C ABI for driving the ECU simulation harness.
//!
//! Callers provide fixed buffers and integer-unit inputs; this crate stays
//! self-contained within the workspace ECU runtime components.
//!
//! The singleton ABI remains available as a compatibility wrapper, and the
//! handle-based ABI uses registry-validated opaque capability tokens for
//! multi-instance use.

#[cfg(not(target_pointer_width = "64"))]
compile_error!("ecu-sim-ffi handle capability tokens require a 64-bit host target");

mod encoding;
mod state;

#[cfg(test)]
use state::acquire_test_lock;
use state::with_state;
use std::panic::{catch_unwind, AssertUnwindSafe};

pub const ECU_SIM_MAX_CHANNELS: usize = 16;
pub const ECU_SIM_MAX_FIRING_ORDER: usize = 16;
pub const ECU_SIM_MAX_EVENTS: usize = 128;

const FAST_QUEUE_CAP: usize = 64;
const SLOW_QUEUE_CAP: usize = 16;
const CRANK_TEETH_PER_REV: u32 = 60;
const CRANK_TEETH_PER_CYCLE: u16 = 120;
const DEG10_PER_TOOTH: i16 = 60;

#[repr(C)]
pub struct EcuSimHandleOpaque {
    _private: [u8; 0],
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuSimStatus {
    Ok = 0,
    ErrInvalid = -1,
    ErrNotInit = -2,
    ErrEventOverflow = -3,
    ErrBufferTooSmall = -4,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuSimInjMode {
    Batch = 0,
    Sequential = 1,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuSimIgnMode {
    Wasted = 0,
    Sequential = 1,
}

#[repr(i32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EcuSimOutputKind {
    Injector = 0,
    Ignition = 1,
    Idle = 2,
    Fan = 3,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EcuSimInitCfg {
    pub cylinders: u8,
    pub has_cam: u8,
    pub inj_mode: i32,
    pub ign_mode: i32,
    pub firing_len: u8,
    pub firing_order: [u8; ECU_SIM_MAX_FIRING_ORDER],
    pub inj_count: u8,
    pub inj_channels: [u8; ECU_SIM_MAX_CHANNELS],
    pub ign_count: u8,
    pub ign_channels: [u8; ECU_SIM_MAX_CHANNELS],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EcuSimSensorFrame {
    pub now_us: u32,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub vbatt_mv: u16,
    pub baro_kpa10: u16,
    pub lambda_x100: u16,
    pub lambda_valid: u8,
    pub knock_x100: u16,
    pub vehicle_speed_kph10: u16,
    pub maf_x100: u16,
    pub cam_phase_deg10: i16,
    pub cam_phase_valid: u8,
    pub validity_flags: u8,
}

impl Default for EcuSimSensorFrame {
    fn default() -> Self {
        Self {
            now_us: 0,
            map_kpa10: 1_000,
            tps_x100: 0,
            clt_c10: 800,
            iat_c10: 250,
            vbatt_mv: 12_000,
            baro_kpa10: 1_013,
            lambda_x100: 100,
            lambda_valid: 0,
            knock_x100: 0,
            vehicle_speed_kph10: 0,
            maf_x100: 0,
            cam_phase_deg10: 0,
            cam_phase_valid: 0,
            validity_flags: 0,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EcuSimOutputEvent {
    pub time_us: u32,
    pub channel: u8,
    pub kind: i32,
    pub high: u8,
}

impl EcuSimOutputEvent {
    pub const ZERO: Self = Self {
        time_us: 0,
        channel: 0,
        kind: EcuSimOutputKind::Injector as i32,
        high: 0,
    };
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct EcuSimSnapshot {
    pub now_us: u32,
    pub rpm: u16,
    pub synced: u8,
    pub tooth: u8,
    pub angle_x10: i16,
    /// Engine load at the freeze-frame point, encoded as kPa x10.
    pub load_kpa10: u16,
    pub output_overflow_count: u32,
    /// Engine phase encoded as u8 for C ABI consumption.
    pub engine_phase: u8,
    /// Runtime fault code encoded as u8 per the diagnostic encoding spec.
    pub fault_code: u8,
    /// Runtime fault severity encoded as u8 per the diagnostic encoding spec.
    pub fault_severity: u8,
    /// Runtime cancel reason encoded as u8 per the diagnostic encoding spec.
    pub cancel_reason: u8,
    /// Runtime control mode encoded as u8 per the diagnostic encoding spec.
    pub control_mode: u8,
    /// Current fuel pulse width encoded as microseconds.
    pub fuel_pulse_width_us: u16,
    /// Current ignition advance encoded as degrees x10.
    pub ignition_advance_x10: i16,
    /// Current dwell time encoded as microseconds.
    pub dwell_us: u16,
    /// Current lambda target encoded as lambda x100.
    pub lambda_target_x100: u16,
    /// Current torque limit encoded as percent x100.
    pub torque_limit_x100: u16,
    /// Soft rev limiter active flag (0 = inactive, 1 = active).
    pub rev_soft_active: u8,
    /// Hard rev limiter active flag (0 = inactive, 1 = active).
    pub rev_hard_active: u8,
    /// Launch limiter active flag (0 = inactive, 1 = active).
    pub launch_active: u8,
    /// Flat-shift limiter active flag (0 = inactive, 1 = active).
    pub flat_shift_active: u8,
    /// Fuel cut is currently active (0 = no cut, 1 = cut active).
    pub fuel_cut: u8,
    /// Spark cut is currently active (0 = no cut, 1 = cut active).
    pub spark_cut: u8,
}

fn ffi_status(f: impl FnOnce() -> EcuSimStatus) -> EcuSimStatus {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(status) => status,
        Err(_) => EcuSimStatus::ErrInvalid,
    }
}

fn ffi_usize(f: impl FnOnce() -> usize) -> usize {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or_default()
}

fn init_state(state: &mut state::EcuSimHandle, cfg: EcuSimInitCfg) -> EcuSimStatus {
    let Ok(runtime_cfg) = state::RuntimeConfig::from_ffi(cfg) else {
        return EcuSimStatus::ErrInvalid;
    };
    state.init(runtime_cfg);
    EcuSimStatus::Ok
}

fn reset_state(state: &mut state::EcuSimHandle) {
    state.reset();
}

fn set_time_state(state: &mut state::EcuSimHandle, now_us: u32) -> EcuSimStatus {
    state.set_time(now_us)
}

fn set_sensors_state(state: &mut state::EcuSimHandle, frame: EcuSimSensorFrame) -> EcuSimStatus {
    state.set_sensors(frame)
}

fn on_crank_edge_state(state: &mut state::EcuSimHandle, ts_us: u32) -> EcuSimStatus {
    state.on_crank_edge(ts_us)
}

fn on_cam_edge_state(state: &mut state::EcuSimHandle, ts_us: u32) -> EcuSimStatus {
    state.on_cam_edge(ts_us)
}

fn step_state(state: &mut state::EcuSimHandle, now_us: u32) -> EcuSimStatus {
    state.step(now_us)
}

fn dequeue_events_state(
    state: &mut state::EcuSimHandle,
    out: *mut EcuSimOutputEvent,
    cap: usize,
) -> usize {
    let mut copied = 0usize;
    if dequeue_events_state_status(state, out, cap, &mut copied as *mut usize) != EcuSimStatus::Ok {
        return 0;
    }
    copied
}

fn dequeue_events_state_status(
    state: &mut state::EcuSimHandle,
    out: *mut EcuSimOutputEvent,
    cap: usize,
    copied_out: *mut usize,
) -> EcuSimStatus {
    if copied_out.is_null() {
        return EcuSimStatus::ErrInvalid;
    }
    // SAFETY: The caller provided writable storage for one usize.
    unsafe { *copied_out = 0 };
    if cap > 0 && out.is_null() {
        return EcuSimStatus::ErrInvalid;
    }
    if let Err(status) = state.require_init() {
        return status;
    }

    let mut copied = 0usize;
    while copied < cap {
        let Some(event) = state.dequeue_event() else {
            break;
        };
        // SAFETY: The caller guarantees `out` has room for `cap` elements, and
        // `copied < cap` here.
        unsafe { *out.add(copied) = event };
        copied += 1;
    }
    if state.outputs_empty() {
        state.clear_overflow_latch();
    }
    // SAFETY: Null was rejected above.
    unsafe { *copied_out = copied };
    EcuSimStatus::Ok
}

fn snapshot_state(state: &mut state::EcuSimHandle, out: *mut EcuSimSnapshot) -> EcuSimStatus {
    if out.is_null() {
        return EcuSimStatus::ErrInvalid;
    }
    if let Err(status) = state.require_init() {
        return status;
    }
    // SAFETY: The caller guarantees `out` points to writable storage.
    unsafe { *out = state.snapshot() };
    EcuSimStatus::Ok
}

fn ffi_handle_status(
    handle: *mut EcuSimHandleOpaque,
    f: impl FnOnce(&mut state::EcuSimHandle) -> EcuSimStatus,
) -> EcuSimStatus {
    ffi_status(|| match state::with_handle(handle, f) {
        Ok(status) => status,
        Err(status) => status,
    })
}

fn ffi_handle_usize(
    handle: *mut EcuSimHandleOpaque,
    f: impl FnOnce(&mut state::EcuSimHandle) -> usize,
) -> usize {
    ffi_usize(|| state::with_handle(handle, f).unwrap_or_default())
}

fn ffi_handle_status_dequeue(
    handle: *mut EcuSimHandleOpaque,
    f: impl FnOnce(&mut state::EcuSimHandle) -> EcuSimStatus,
) -> EcuSimStatus {
    ffi_handle_status(handle, f)
}

/// Create a new independent ECU simulation handle.
///
/// The returned pointer is an opaque capability token. It is not a pointer to
/// simulation state and must only be passed back to the `ecu_sim_handle_*`
/// functions.
#[no_mangle]
pub extern "C" fn ecu_sim_handle_create() -> *mut EcuSimHandleOpaque {
    match catch_unwind(AssertUnwindSafe(state::create_handle)) {
        Ok(Some(handle)) => handle,
        Err(_) => core::ptr::null_mut(),
        Ok(None) => core::ptr::null_mut(),
    }
}

/// Destroy a handle created by `ecu_sim_handle_create`.
///
/// Repeated destroy calls and stale tokens are treated as no-ops. Destroy
/// waits for any active call that has already entered the handle to finish
/// before returning. Calls that start after destroy begins are rejected.
/// Tokens encode a registry slot, generation, per-registry nonce, and cookie;
/// freed slots may be reused only after generation advances, and
/// exhausted-generation slots retire.
///
/// # Safety
///
/// `handle` must be null or a token returned by `ecu_sim_handle_create`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_handle_destroy(handle: *mut EcuSimHandleOpaque) {
    state::destroy_handle(handle);
}

/// Reset a handle to the uninitialized state.
#[no_mangle]
pub extern "C" fn ecu_sim_handle_reset(handle: *mut EcuSimHandleOpaque) -> EcuSimStatus {
    ffi_handle_status(handle, |state| {
        reset_state(state);
        EcuSimStatus::Ok
    })
}

/// Initialize an independent ECU simulation handle.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
///
/// # Safety
///
/// `handle` must be a token returned by `ecu_sim_handle_create`.
/// `cfg` must be null or point to an initialized `EcuSimInitCfg` that remains
/// valid for the duration of this call. The data is copied before return.
/// Stale or destroyed handles return `EcuSimStatus::ErrInvalid`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_handle_init(
    handle: *mut EcuSimHandleOpaque,
    cfg: *const EcuSimInitCfg,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| {
        if cfg.is_null() {
            return EcuSimStatus::ErrInvalid;
        }
        // SAFETY: The caller guarantees `cfg` is valid for this call.
        let cfg = unsafe { *cfg };
        init_state(state, cfg)
    })
}

/// Set logical simulation time on a handle without executing a runtime step.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
#[no_mangle]
pub extern "C" fn ecu_sim_handle_set_time(
    handle: *mut EcuSimHandleOpaque,
    now_us: u32,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| set_time_state(state, now_us))
}

/// Update the latest sensor frame for a handle.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
///
/// # Safety
///
/// `handle` must be a token returned by `ecu_sim_handle_create`.
/// `frame` must be null or point to an initialized `EcuSimSensorFrame` that
/// remains valid for the duration of this call. The data is copied before
/// return. Stale or destroyed handles return `EcuSimStatus::ErrInvalid`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_handle_set_sensors(
    handle: *mut EcuSimHandleOpaque,
    frame: *const EcuSimSensorFrame,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| {
        if frame.is_null() {
            return EcuSimStatus::ErrInvalid;
        }
        // SAFETY: The caller guarantees `frame` is valid for this call.
        let frame = unsafe { *frame };
        set_sensors_state(state, frame)
    })
}

/// Feed one crank edge at `ts_us` for a handle.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
#[no_mangle]
pub extern "C" fn ecu_sim_handle_on_crank_edge(
    handle: *mut EcuSimHandleOpaque,
    ts_us: u32,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| on_crank_edge_state(state, ts_us))
}

/// Feed one cam edge at `ts_us` for a handle.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
#[no_mangle]
pub extern "C" fn ecu_sim_handle_on_cam_edge(
    handle: *mut EcuSimHandleOpaque,
    ts_us: u32,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| on_cam_edge_state(state, ts_us))
}

/// Execute one nonblocking ECU runtime step at `now_us` for a handle.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
#[no_mangle]
pub extern "C" fn ecu_sim_handle_step(
    handle: *mut EcuSimHandleOpaque,
    now_us: u32,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| step_state(state, now_us))
}

/// Drain output events from a handle into the caller-owned buffer in timestamp order.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
///
/// # Safety
///
/// `handle` must be a token returned by `ecu_sim_handle_create`.
/// If `cap > 0`, `out` must point to writable storage for at least `cap`
/// `EcuSimOutputEvent` values. If `cap == 0`, `out` may be null. Stale or
/// destroyed handles return 0.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_handle_dequeue_events(
    handle: *mut EcuSimHandleOpaque,
    out: *mut EcuSimOutputEvent,
    cap: usize,
) -> usize {
    ffi_handle_usize(handle, |state| dequeue_events_state(state, out, cap))
}

/// Drain output events from a handle and report failure separately from an empty queue.
///
/// On success, writes the number of copied events to `copied_out` and returns
/// `EcuSimStatus::Ok`. On failure, writes 0 when `copied_out` is non-null and
/// returns the status. This is the status-observable equivalent of
/// `ecu_sim_handle_dequeue_events`.
///
/// # Safety
///
/// `handle` must be a token returned by `ecu_sim_handle_create`.
/// If `cap > 0`, `out` must point to writable storage for at least `cap`
/// `EcuSimOutputEvent` values. If `cap == 0`, `out` may be null.
/// `copied_out` must point to writable storage for one `usize`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_handle_dequeue_events_status(
    handle: *mut EcuSimHandleOpaque,
    out: *mut EcuSimOutputEvent,
    cap: usize,
    copied_out: *mut usize,
) -> EcuSimStatus {
    ffi_handle_status_dequeue(handle, |state| {
        dequeue_events_state_status(state, out, cap, copied_out)
    })
}

/// Copy the latest simulation snapshot from a handle to `out`.
///
/// The handle is an opaque capability token returned by
/// `ecu_sim_handle_create`.
///
/// # Safety
///
/// `handle` must be a token returned by `ecu_sim_handle_create`.
/// `out` must point to writable storage for one `EcuSimSnapshot`.
/// Stale or destroyed handles return `EcuSimStatus::ErrInvalid`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_handle_snapshot(
    handle: *mut EcuSimHandleOpaque,
    out: *mut EcuSimSnapshot,
) -> EcuSimStatus {
    ffi_handle_status(handle, |state| snapshot_state(state, out))
}

/// Initialize the single host ECU simulation instance.
///
/// # Safety
///
/// `cfg` must be null or point to an initialized `EcuSimInitCfg` that remains
/// valid for the duration of this call. The data is copied before return.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_init(cfg: *const EcuSimInitCfg) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| {
        if cfg.is_null() {
            return EcuSimStatus::ErrInvalid;
        }
        // SAFETY: The caller guarantees `cfg` is valid for this call.
        let cfg = unsafe { *cfg };
        with_state(|state| init_state(state, cfg))
    })
}

/// Reset the global ECU simulation instance to the uninitialized state.
#[no_mangle]
pub extern "C" fn ecu_sim_reset() {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    let _ = ffi_status(|| {
        with_state(|state| {
            reset_state(state);
            EcuSimStatus::Ok
        })
    });
}

/// Set logical simulation time without executing a runtime step.
#[no_mangle]
pub extern "C" fn ecu_sim_set_time(now_us: u32) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| with_state(|state| set_time_state(state, now_us)))
}

/// Update the latest sensor frame.
///
/// # Safety
///
/// `frame` must be null or point to an initialized `EcuSimSensorFrame` that
/// remains valid for the duration of this call. The data is copied before
/// return.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_set_sensors(frame: *const EcuSimSensorFrame) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| {
        if frame.is_null() {
            return EcuSimStatus::ErrInvalid;
        }
        // SAFETY: The caller guarantees `frame` is valid for this call.
        let frame = unsafe { *frame };
        with_state(|state| set_sensors_state(state, frame))
    })
}

/// Feed one crank edge at `ts_us`.
#[no_mangle]
pub extern "C" fn ecu_sim_on_crank_edge(ts_us: u32) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| with_state(|state| on_crank_edge_state(state, ts_us)))
}

/// Feed one cam edge at `ts_us`.
#[no_mangle]
pub extern "C" fn ecu_sim_on_cam_edge(ts_us: u32) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| with_state(|state| on_cam_edge_state(state, ts_us)))
}

/// Execute one nonblocking ECU runtime step at `now_us`.
#[no_mangle]
pub extern "C" fn ecu_sim_step(now_us: u32) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| with_state(|state| step_state(state, now_us)))
}

/// Drain output events into the caller-owned buffer in timestamp order.
///
/// # Safety
///
/// If `cap > 0`, `out` must point to writable storage for at least `cap`
/// `EcuSimOutputEvent` values. If `cap == 0`, `out` may be null.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_dequeue_events(out: *mut EcuSimOutputEvent, cap: usize) -> usize {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_usize(|| with_state(|state| dequeue_events_state(state, out, cap)))
}

/// Drain output events and report failure separately from an empty queue.
///
/// On success, writes the number of copied events to `copied_out` and returns
/// `EcuSimStatus::Ok`. On failure, writes 0 when `copied_out` is non-null and
/// returns the status. This is the status-observable equivalent of
/// `ecu_sim_dequeue_events`.
///
/// # Safety
///
/// If `cap > 0`, `out` must point to writable storage for at least `cap`
/// `EcuSimOutputEvent` values. If `cap == 0`, `out` may be null.
/// `copied_out` must point to writable storage for one `usize`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_dequeue_events_status(
    out: *mut EcuSimOutputEvent,
    cap: usize,
    copied_out: *mut usize,
) -> EcuSimStatus {
    #[cfg(test)]
    let _guard = acquire_test_lock();
    ffi_status(|| with_state(|state| dequeue_events_state_status(state, out, cap, copied_out)))
}

/// Copy the latest simulation snapshot to `out`.
///
/// # Safety
///
/// `out` must point to writable storage for one `EcuSimSnapshot`.
#[no_mangle]
pub unsafe extern "C" fn ecu_sim_snapshot(out: *mut EcuSimSnapshot) -> EcuSimStatus {
    ffi_status(|| with_state(|state| snapshot_state(state, out)))
}

#[cfg(any(test, feature = "test-support"))]
pub fn ecu_sim_inject_fault_for_test(
    fault: ecu_domain::FaultCode,
    severity: ecu_domain::FaultSeverity,
    cancel_reason: ecu_domain::CancelReason,
) -> EcuSimStatus {
    with_state(|state| {
        if !state.initialized() {
            return EcuSimStatus::ErrNotInit;
        }
        state.inject_fault_for_test(fault, severity, cancel_reason);
        EcuSimStatus::Ok
    })
}

#[cfg(test)]
mod tests;
