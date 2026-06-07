use core::convert::Infallible;
use ecu_board_api::{CaptureSample, CaptureSink, Watchdog};
use ecu_calibration::{PersistedCalibrationBlob, PersistedCalibrationStore};
use ecu_runtime::{RuntimeSnapshot, TransportPublisher};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NoopCapture;

impl CaptureSink for NoopCapture {
    type Error = Infallible;

    fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NoopTransport;

impl TransportPublisher for NoopTransport {
    type Error = Infallible;

    fn publish_snapshot(&mut self, _snapshot: &RuntimeSnapshot) -> Result<(), Self::Error> {
        Ok(())
    }

    fn publish_calibration(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NoopStore;

impl PersistedCalibrationStore for NoopStore {
    type Error = Infallible;

    fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error> {
        Ok(None)
    }

    fn save(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct NoopWatchdog;

impl Watchdog for NoopWatchdog {
    type Error = Infallible;

    fn feed(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn noop_capture_accepts_samples() {
        let mut capture = NoopCapture;

        assert_eq!(capture.capture(CaptureSample::default()), Ok(()));
    }

    #[test]
    fn noop_store_is_empty_and_accepts_saves() {
        let mut store = NoopStore;
        let blob = PersistedCalibrationBlob::new(Default::default());

        assert_eq!(store.load(), Ok(None));
        assert_eq!(store.save(&blob), Ok(()));
    }

    #[test]
    fn noop_watchdog_feed_is_infallible() {
        let mut watchdog = NoopWatchdog;

        assert_eq!(watchdog.feed(), Ok(()));
    }
}
