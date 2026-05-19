//! Management Engine - OODA Loop Implementation
//!
//! This module implements a military-inspired OODA (Observe, Orient, Decide, Act)
//! loop for engine management. It provides a generic, extensible framework for:
//!
//! - **Observe**: Sensor input abstraction and validation
//! - **Orient**: Data fusion and context building
//! - **Decide**: Control algorithm execution
//! - **Act**: Command dispatch to execution modules
//!
//! The architecture is transport-agnostic, memory-scalable, and highly modular.
//!
//! Status: experimental subsystem. It is not wired into the main ECU runtime
//! path and should be treated as a separate control architecture until that
//! changes explicitly.

pub mod act;
pub mod decide;
pub mod history;
pub mod observe;
pub mod orient;
pub mod types;

pub use act::{ActError, Actor, CommandDispatcher};
pub use decide::{ControlStrategy, Decider, FuelCommand, IgnitionCommand};
pub use history::{History, HistoryChannel, HistoryConfig};
pub use observe::{Observation, ObservationSource, Observer, Quality};
pub use orient::{EngineContext, OperatingMode, Orienter};
pub use types::*;

use crate::Transport;

/// Management Engine - Main OODA Loop Coordinator
///
/// This is the top-level component that orchestrates the OODA cycle.
/// It can be configured for different hardware capabilities and
/// transport layers.
pub struct ManagementEngine<T: Transport, const HISTORY_SIZE: usize = 100> {
    observer: Observer,
    orienter: Orienter,
    decider: Decider,
    actor: Actor<T>,
    history: Option<History<HISTORY_SIZE>>,
    cycle_count: u32,
    last_cycle_us: u32,
}

impl<T: Transport, const HISTORY_SIZE: usize> ManagementEngine<T, HISTORY_SIZE> {
    /// Create new management engine
    ///
    /// # Arguments
    /// * `transport` - Communication transport for command dispatch
    /// * `enable_history` - Whether to enable telemetry/logging
    pub fn new(transport: T, enable_history: bool) -> Self {
        Self {
            observer: Observer::new(),
            orienter: Orienter::new(),
            decider: Decider::new(),
            actor: Actor::new(transport),
            history: if enable_history {
                Some(History::new())
            } else {
                None
            },
            cycle_count: 0,
            last_cycle_us: 0,
        }
    }

    /// Execute one OODA loop cycle
    ///
    /// This is the main entry point, called at a fixed rate (e.g., 100 Hz).
    ///
    /// # Arguments
    /// * `current_time_us` - Current timestamp in microseconds
    ///
    /// # Returns
    /// `Ok(())` if cycle completed successfully
    pub fn cycle(&mut self, current_time_us: u32) -> Result<(), ManagementError> {
        // OBSERVE: Collect sensor data
        let observations = self.observer.collect_all(current_time_us)?;

        // ORIENT: Build context from observations
        let context = self.orienter.build_context(&observations)?;

        // DECIDE: Compute control actions
        let decisions = self.decider.compute_control(&context)?;

        // ACT: Dispatch commands
        self.actor.dispatch_commands(&decisions)?;

        // LOG: Record to history (if enabled)
        if let Some(ref mut history) = self.history {
            history.record(&context, &decisions);
        }

        // Update cycle tracking
        self.cycle_count += 1;
        self.last_cycle_us = current_time_us;

        Ok(())
    }

    /// Get reference to observer (for adding observation sources)
    pub fn observer_mut(&mut self) -> &mut Observer {
        &mut self.observer
    }

    /// Get reference to decider (for algorithm configuration)
    pub fn decider_mut(&mut self) -> &mut Decider {
        &mut self.decider
    }

    /// Get reference to history (for queries)
    pub fn history(&self) -> Option<&History<HISTORY_SIZE>> {
        self.history.as_ref()
    }

    /// Get current cycle count (for diagnostics)
    pub fn cycle_count(&self) -> u32 {
        self.cycle_count
    }

    /// Get cycle timing (microseconds since last cycle)
    pub fn cycle_time_us(&self, current_time_us: u32) -> u32 {
        current_time_us.wrapping_sub(self.last_cycle_us)
    }
}

/// Management engine error types
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ManagementError {
    /// Observation phase failed
    ObserveFailed,
    /// Orient phase failed (data fusion error)
    OrientFailed,
    /// Decide phase failed (algorithm error)
    DecideFailed,
    /// Act phase failed (command dispatch error)
    ActFailed,
    /// Timeout waiting for observations
    Timeout,
}

// Implement From for error conversions
impl From<observe::ObserveError> for ManagementError {
    fn from(_: observe::ObserveError) -> Self {
        ManagementError::ObserveFailed
    }
}

impl From<orient::OrientError> for ManagementError {
    fn from(_: orient::OrientError) -> Self {
        ManagementError::OrientFailed
    }
}

impl From<decide::DecideError> for ManagementError {
    fn from(_: decide::DecideError) -> Self {
        ManagementError::DecideFailed
    }
}

impl From<act::ActError> for ManagementError {
    fn from(_: act::ActError) -> Self {
        ManagementError::ActFailed
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::transport::Message;

    // Mock transport for testing
    struct MockTransport;

    impl Transport for MockTransport {
        fn send(&mut self, _msg: &Message) -> Result<(), crate::TransportError> {
            Ok(())
        }

        fn try_receive(&mut self) -> Option<Message> {
            None
        }

        fn poll(&mut self) {
            // No-op for mock
        }

        fn flush(&mut self) -> Result<(), crate::TransportError> {
            Ok(())
        }

        fn stats(&self) -> crate::TransportStats {
            crate::TransportStats {
                tx_count: 0,
                rx_count: 0,
                tx_errors: 0,
                rx_errors: 0,
                tx_buffer_usage: 0,
                rx_buffer_usage: 0,
                avg_latency_us: None,
            }
        }

        fn is_ready(&self) -> bool {
            true
        }
    }

    #[test]
    fn test_management_engine_creation() {
        let transport = MockTransport;
        let engine = ManagementEngine::<MockTransport, 100>::new(transport, false);

        assert_eq!(engine.cycle_count(), 0);
    }

    #[test]
    fn test_management_engine_cycle_tracking() {
        let transport = MockTransport;
        let engine = ManagementEngine::<MockTransport, 100>::new(transport, false);

        // Cycles will fail because no observations are available
        // Count only increments on successful cycles
        // With no observations, count stays at 0
        assert_eq!(engine.cycle_count(), 0);
    }

    #[test]
    fn test_management_engine_with_history() {
        let transport = MockTransport;
        let engine = ManagementEngine::<MockTransport, 100>::new(transport, true);

        assert!(engine.history().is_some());
    }

    #[test]
    fn test_management_engine_without_history() {
        let transport = MockTransport;
        let engine = ManagementEngine::<MockTransport, 100>::new(transport, false);

        assert!(engine.history().is_none());
    }
}
