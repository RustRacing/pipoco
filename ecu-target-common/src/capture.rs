#![allow(unused_macros)]

/// Define a lock-free capture ring buffer and push/pop helpers.
/// Generates a `cortex_m::interrupt::Mutex<RefCell<CaptureBuffer<N>>>` named `$name`,
/// plus `capture_push` and `capture_pop` functions.
///
/// # Safety Invariant
///
/// This macro uses `cortex_m::interrupt::Mutex<RefCell<_>>` for safe no_std access.
/// All access to the ring is via `cortex_m::interrupt::free` critical sections,
/// which prevents concurrent access from interrupts and main code.
///
/// No `unsafe` blocks and no mutable statics are required.
#[macro_export]
macro_rules! capture_ring {
    ($name:ident, $n:expr) => {
        static $name: cortex_m::interrupt::Mutex<core::cell::RefCell<ecu_core::CaptureBuffer<$n>>> =
            cortex_m::interrupt::Mutex::new(
                core::cell::RefCell::new(ecu_core::CaptureBuffer::new()),
            );

        #[inline]
        fn capture_push(ts: u32) {
            cortex_m::interrupt::free(|cs| {
                $name.borrow(cs).borrow_mut().push(ts);
            });
        }

        fn capture_pop() -> Option<u32> {
            cortex_m::interrupt::free(|cs| $name.borrow(cs).borrow_mut().try_pop())
        }
    };
}
