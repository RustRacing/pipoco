//! ACT - Command Dispatch
//!
//! Sends control commands to execution modules via transport layer.

use super::decide::ControlDecisions;
use crate::{Transport, TransportError, Message};

/// Command dispatcher trait
pub trait CommandDispatcher {
    fn dispatch(&mut self, decisions: &ControlDecisions) -> Result<(), ActError>;
}

/// Actor - dispatches commands
pub struct Actor<T: Transport> {
    transport: T,
    send_count: u32,
}

impl<T: Transport> Actor<T> {
    /// Create new actor
    pub fn new(transport: T) -> Self {
        Self {
            transport,
            send_count: 0,
        }
    }

    /// Dispatch commands
    pub fn dispatch_commands(&mut self, decisions: &ControlDecisions) -> Result<(), ActError> {
        // Send fuel table (simplified - real implementation would serialize)
        let fuel_msg = Message::IpwTable {
            version: 1,
            data: decisions.fuel.ipw_table,
            crc32: 0,  // Would calculate CRC
        };

        self.transport
            .send(&fuel_msg)
            .map_err(|_| ActError::TransportFailed)?;

        // Send ignition table
        let ign_msg = Message::IgnitionTable {
            version: 1,
            data: decisions.ignition.timing_table,
            crc32: 0,
        };

        self.transport
            .send(&ign_msg)
            .map_err(|_| ActError::TransportFailed)?;

        self.send_count += 1;
        Ok(())
    }

    /// Get send count (diagnostics)
    pub fn send_count(&self) -> u32 {
        self.send_count
    }
}

/// Act errors
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ActError {
    /// Transport failed
    TransportFailed,
    /// Command rejected
    Rejected,
}

#[cfg(test)]
mod tests {
    use super::*;

    struct MockTransport;
    impl Transport for MockTransport {
        fn send(&mut self, _msg: &Message) -> Result<(), TransportError> {
            Ok(())
        }
        fn try_receive(&mut self) -> Option<Message> {
            None
        }
        fn stats(&self) -> crate::TransportStats {
            crate::TransportStats {
                tx_count: 0,
                rx_count: 0,
                tx_errors: 0,
                rx_errors: 0,
                tx_buffer_usage: 0,
                rx_buffer_usage: 0,
            }
        }
    }

    #[test]
    fn test_actor_creation() {
        let transport = MockTransport;
        let actor = Actor::new(transport);
        assert_eq!(actor.send_count(), 0);
    }
}
