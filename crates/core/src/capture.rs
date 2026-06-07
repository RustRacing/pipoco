//! Lightweight capture buffers for ECU diagnostics.
//!
//! Provides:
//! - Simple timestamp ring buffer for ISR-to-main communication
//! - Event-triggered capture with pre/post-trigger data

mod ingest;
mod model;
#[cfg(test)]
mod tests;

pub use ecu_io::CaptureBuffer;
pub use model::{
    trigger_mask, CaptureConfig, CaptureFrame, CaptureState, CaptureTrigger, TriggeredCapture,
};
