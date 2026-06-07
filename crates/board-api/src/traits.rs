//! Board service traits exposed by `ecu-board-api`.

use crate::timing_island::{AuxCommandBatch, EdgeBatch, OutputTransitionBatch};
use crate::{capabilities::CalibrationPage, sensors::SensorSnapshot, telemetry::TelemetryFrame};
use ecu_domain::Micros;

pub trait EcuClock {
    fn now_us(&self) -> Micros;
}

pub trait TriggerEdgeSource<const N: usize> {
    type Error;

    fn drain_edges(&mut self, out: &mut EdgeBatch<N>) -> Result<(), Self::Error>;
}

pub trait SensorSource {
    type Error;

    fn sample(&mut self, now: Micros) -> Result<SensorSnapshot, Self::Error>;
}

pub trait OutputScheduler<const N: usize> {
    type Error;

    fn schedule(&mut self, batch: &OutputTransitionBatch<N>) -> Result<(), Self::Error>;

    fn cancel_all(&mut self) -> Result<(), Self::Error>;

    /// Force all controlled outputs into their board-defined safe state.
    ///
    /// Existing implementations may keep implementing this fire-and-forget
    /// hook. New implementations should prefer overriding
    /// [`Self::try_force_safe_state`] so failures are observable at safety
    /// boundaries.
    fn force_safe_state(&mut self);

    /// Fallible safe-state hook for boards that can observe shutdown failures.
    fn try_force_safe_state(&mut self) -> Result<(), Self::Error> {
        self.force_safe_state();
        Ok(())
    }
}

pub trait AuxOutputSink<const N: usize> {
    type Error;

    fn apply_aux(&mut self, batch: &AuxCommandBatch<N>) -> Result<(), Self::Error>;
}

pub trait Watchdog {
    type Error;

    fn feed(&mut self) -> Result<(), Self::Error>;
}

pub trait TelemetrySink {
    type Error;

    fn publish(&mut self, frame: &TelemetryFrame) -> Result<(), Self::Error>;
}

pub trait CalibrationStore {
    type Error;

    fn read_page(&mut self, page: CalibrationPage, out: &mut [u8]) -> Result<usize, Self::Error>;

    fn write_page(&mut self, page: CalibrationPage, bytes: &[u8]) -> Result<(), Self::Error>;
}
