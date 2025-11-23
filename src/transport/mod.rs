//! Transport abstraction for ECU inter-component communication
//!
//! This module provides a transport-agnostic messaging layer that works
//! seamlessly across different physical transports (BBQueue, CAN, SPI, etc.).
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────┐
//! │     Application Layer               │
//! │  (Injection, Management, Sensor)    │
//! └────────────────┬────────────────────┘
//!                  │
//! ┌────────────────▼────────────────────┐
//! │      Message Protocol               │
//! │  (Type-safe Message enum)           │
//! └────────────────┬────────────────────┘
//!                  │
//! ┌────────────────▼────────────────────┐
//! │    Transport Trait                  │
//! │  send() / try_receive()             │
//! └────────────────┬────────────────────┘
//!                  │
//!      ┌───────────┼───────────┐
//!      │           │           │
//! ┌────▼────┐ ┌───▼───┐  ┌───▼────┐
//! │BBQueue  │ │  CAN  │  │  SPI   │
//! │~50ns    │ │~200μs │  │ ~10μs  │
//! └─────────┘ └───────┘  └────────┘
//! ```
//!
//! # Usage
//!
//! ```ignore
//! // Create transport pair (BBQueue example)
//! let (tx_transport, rx_transport) = BbqTransport::create_pair();
//!
//! // Send message
//! let msg = Message::TriggerTiming {
//!     gap_period_us: 2000,
//!     tooth_period_us: 1000,
//!     tooth_position: 1,
//!     synced: true,
//!     timestamp_us: 12345,
//! };
//! tx_transport.send(&msg).unwrap();
//!
//! // Receive message
//! if let Some(received) = rx_transport.try_receive() {
//!     match received {
//!         Message::TriggerTiming { gap_period_us, .. } => {
//!             println!("Received gap period: {}", gap_period_us);
//!         }
//!         _ => {}
//!     }
//! }
//! ```

// Message protocol (always included)
pub mod message;
pub use message::Message;

// BBQueue transport (optional, enabled with "transport-bbqueue" feature)
#[cfg(feature = "transport-bbqueue")]
pub mod bbq;
#[cfg(feature = "transport-bbqueue")]
pub use bbq::BbqTransport;

// CAN transport (optional)
#[cfg(feature = "transport-can")]
pub mod can;
#[cfg(feature = "transport-can")]
pub use can::{CanDevice, CanTransport};

/// Transport-agnostic message passing interface
///
/// All physical transports implement this trait to provide a unified API.
pub trait Transport {
    /// Send a message
    ///
    /// Returns Ok(()) if message was queued successfully.
    /// Returns Err if buffer is full, serialization failed, or hardware error.
    ///
    /// # Errors
    ///
    /// - `TransportError::BufferFull` - Cannot queue more messages
    /// - `TransportError::MessageTooLarge` - Message exceeds transport capacity
    /// - `TransportError::SerializationFailed` - Failed to serialize message
    /// - `TransportError::HardwareError` - Transport hardware failure
    /// - `TransportError::NotReady` - Transport not connected/initialized
    fn send(&mut self, message: &Message) -> Result<(), TransportError>;

    /// Try to receive a message (non-blocking)
    ///
    /// Returns Some(message) if a message was available and valid.
    /// Returns None if no message is currently available.
    ///
    /// Corrupted messages are silently dropped and increment error count.
    fn try_receive(&mut self) -> Option<Message>;

    /// Poll the transport layer (process events)
    ///
    /// Should be called regularly from main loop to:
    ///   - Process received data
    ///   - Handle transmission completion
    ///   - Update internal state
    ///
    /// For interrupt-driven transports, this may be a no-op.
    fn poll(&mut self);

    /// Flush pending transmissions
    ///
    /// Blocks until all queued messages are transmitted.
    /// Use sparingly - prefer poll() in main loop for better performance.
    fn flush(&mut self) -> Result<(), TransportError>;

    /// Get transport statistics
    ///
    /// Returns counters for monitoring transport health.
    fn stats(&self) -> TransportStats;

    /// Check if transport is connected/ready
    ///
    /// Returns false if transport is in an error state or not initialized.
    fn is_ready(&self) -> bool;
}

/// Transport layer errors
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum TransportError {
    /// Buffer full - cannot queue more messages
    BufferFull,

    /// Message too large for this transport
    MessageTooLarge,

    /// Serialization failed (postcard error)
    SerializationFailed,

    /// Deserialization failed (corrupted data)
    DeserializationFailed,

    /// Hardware error (CAN bus off, SPI timeout, etc.)
    HardwareError,

    /// Transport not connected or ready
    NotReady,
}

/// Transport statistics for monitoring
#[derive(Debug, Copy, Clone, Default)]
pub struct TransportStats {
    /// Total messages sent successfully
    pub tx_count: u32,

    /// Total messages received successfully
    pub rx_count: u32,

    /// TX errors (full buffer, hw error, etc.)
    pub tx_errors: u32,

    /// RX errors (deserialization, crc, etc.)
    pub rx_errors: u32,

    /// Current TX buffer utilization (0-100%)
    pub tx_buffer_usage: u8,

    /// Current RX buffer utilization (0-100%)
    pub rx_buffer_usage: u8,

    /// Transport-specific latency estimate (microseconds)
    /// None if not measurable
    pub avg_latency_us: Option<u32>,
}
