//! Board-local production IO stubs and the real RP2040 hardware watchdog.
//!
//! The rp2040 ts-ecu binary drives trigger capture through the PIO interrupt
//! path and persistence through `FlashKv`/`PersistedTsPageStore`, not through
//! the board adapter's capture/transport/store slots, and it has no external
//! telemetry transport. Those adapter slots are therefore genuine
//! "absent peripheral" production stubs (distinct from the test-only `noop`
//! doubles, which never ship in a flashable binary). The watchdog slot is a
//! real safety peripheral, backed here by the RP2040 hardware watchdog.

use core::convert::Infallible;
use ecu_board_api::{CaptureSample, CaptureSink, Watchdog};
use ecu_calibration::{PersistedCalibrationBlob, PersistedCalibrationStore};
use ecu_runtime::{RuntimeSnapshot, TransportPublisher};

/// Trigger capture reaches the runtime through the PIO interrupt queue, so the
/// adapter's capture sink has no peripheral to drive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AbsentCapture;

impl CaptureSink for AbsentCapture {
    type Error = Infallible;

    fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// No external telemetry transport (e.g. CAN) is present on this board; TS runs
/// over USB through a separate path.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AbsentTransport;

impl TransportPublisher for AbsentTransport {
    type Error = Infallible;

    fn publish_snapshot(&mut self, _snapshot: &RuntimeSnapshot) -> Result<(), Self::Error> {
        Ok(())
    }

    fn publish_calibration(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Persistence is owned by `FlashKv`/`PersistedTsPageStore`, so the adapter's
/// calibration-store slot has no backing peripheral.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct AbsentStore;

impl PersistedCalibrationStore for AbsentStore {
    type Error = Infallible;

    fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error> {
        Ok(None)
    }

    fn save(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Real RP2040 hardware watchdog wired into the safety role. `init_clocks_and_plls`
/// has already enabled 1 µs tick generation; `arm` starts the countdown and the
/// main loop must `feed` within the configured period or the chip resets.
pub struct RpWatchdog {
    hw: rp2040_hal::watchdog::Watchdog,
}

impl RpWatchdog {
    /// Start the hardware watchdog with the given timeout. The period is chosen
    /// comfortably above the worst-case main-loop iteration yet tight enough to
    /// catch a true hang.
    pub fn arm(mut hw: rp2040_hal::watchdog::Watchdog, period: fugit::MicrosDurationU32) -> Self {
        use embedded_hal::watchdog::WatchdogEnable;
        hw.start(period);
        Self { hw }
    }
}

impl Watchdog for RpWatchdog {
    type Error = Infallible;

    fn feed(&mut self) -> Result<(), Self::Error> {
        use embedded_hal::watchdog::Watchdog as _;
        self.hw.feed();
        Ok(())
    }
}

/// Busy-delay for `total_ms`, split into `chunk_ms` segments and invoking
/// `feed` between segments so a clamped output test cannot starve the hardware
/// watchdog. The CPU runs at 125 MHz, so one millisecond is `1000 * 125`
/// `cortex_m::asm::delay` cycles.
pub fn fed_delay_ms(total_ms: u32, chunk_ms: u32, mut feed: impl FnMut()) {
    let chunk = chunk_ms.max(1);
    let mut remaining = total_ms;
    while remaining > 0 {
        let step = remaining.min(chunk);
        cortex_m::asm::delay(step * 1000 * 125);
        feed();
        remaining -= step;
    }
}
