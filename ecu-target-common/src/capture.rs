#![allow(unused_macros)]

/// Define a lock-free capture ring buffer and push/pop helpers.
/// Generates a static `CaptureBuffer<N>` named `$name`, plus `capture_push` and `capture_pop` functions.
#[macro_export]
macro_rules! capture_ring {
    ($name:ident, $n:expr) => {
        static mut $name: ecu_core::CaptureBuffer<$n> = ecu_core::CaptureBuffer::new();
        #[inline]
        fn capture_push(ts: u32) {
            cortex_m::interrupt::free(|_| unsafe { $name.push(ts) });
        }
        fn capture_pop() -> Option<u32> {
            cortex_m::interrupt::free(|_| unsafe { $name.try_pop() })
        }
    };
}
