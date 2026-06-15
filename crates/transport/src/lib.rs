//! Transport abstraction for ECU inter-component communication.
//!
//! This crate owns the transport-agnostic message protocol plus concrete
//! transport adapters that do not need legacy `ecu-compat` state.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "transport-can-fd", not(feature = "transport-can")))]
compile_error!("feature `transport-can-fd` requires `transport-can`");

mod message;
pub use message::Message;

#[cfg(feature = "transport-bbqueue")]
mod bbq;
#[cfg(feature = "transport-bbqueue")]
pub use bbq::BbqTransport;

#[cfg(feature = "transport-can")]
mod can;
#[cfg(feature = "transport-can")]
pub use can::{CanDevice, CanTransport};

/// Transport-agnostic message passing interface.
pub trait Transport {
    /// Send a message.
    fn send(&mut self, message: &Message) -> Result<(), TransportError>;

    /// Try to receive a message without blocking.
    fn try_receive(&mut self) -> Option<Message>;

    /// Poll the transport layer for pending work.
    fn poll(&mut self);

    /// Flush pending transmissions.
    fn flush(&mut self) -> Result<(), TransportError>;

    /// Get transport statistics.
    fn stats(&self) -> TransportStats;

    /// Check if transport is connected/ready.
    fn is_ready(&self) -> bool;
}

/// Transport layer errors.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum TransportError {
    /// Buffer full - cannot queue more messages.
    BufferFull,
    /// Message too large for this transport.
    MessageTooLarge,
    /// Serialization failed.
    SerializationFailed,
    /// Deserialization failed.
    DeserializationFailed,
    /// Hardware error.
    HardwareError,
    /// Transport not connected or ready.
    NotReady,
}

/// Transport statistics for monitoring.
#[derive(Debug, Copy, Clone, Default)]
pub struct TransportStats {
    pub tx_count: u32,
    pub rx_count: u32,
    pub tx_errors: u32,
    pub rx_errors: u32,
    pub tx_buffer_usage: u8,
    pub rx_buffer_usage: u8,
    pub avg_latency_us: Option<u32>,
}
