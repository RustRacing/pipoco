//! Capture sample ingestion and state-transition helpers.

use super::{CaptureFrame, CaptureState, CaptureTrigger, TriggeredCapture};

impl<const N: usize> TriggeredCapture<N> {
    pub(super) fn append_sample(&mut self, frame: CaptureFrame) {
        self.buffer[self.head] = frame;
        self.head = (self.head + 1) % N;
        if self.count < N {
            self.count += 1;
        }
    }

    /// Add a sample to the capture buffer.
    ///
    /// In `Armed` state, samples are added to the ring buffer.
    /// In `Capturing` state, samples are added until post-trigger samples are complete.
    pub fn sample(&mut self, frame: CaptureFrame, now_us: u32) {
        let elapsed = now_us.wrapping_sub(self.last_sample_us);
        if self.last_sample_us != 0 && elapsed < self.config.sample_interval_us {
            return;
        }
        self.last_sample_us = now_us;

        match self.state {
            CaptureState::Armed => {
                self.append_sample(frame);
                self.handle_spike_trigger(&frame, elapsed);
                self.last_rpm = frame.rpm;
            }
            CaptureState::Capturing => {
                self.append_sample(frame);
                self.post_remaining = self.post_remaining.saturating_sub(1);
                if self.post_remaining == 0 {
                    self.state = CaptureState::Complete;
                }
            }
            CaptureState::Complete | CaptureState::Disabled => {}
        }
    }

    fn handle_spike_trigger(&mut self, frame: &CaptureFrame, elapsed: u32) {
        if !self.config.is_trigger_enabled(CaptureTrigger::RpmSpike) {
            return;
        }
        if self.last_rpm == 0 {
            return;
        }
        let rpm_change = (frame.rpm as i32 - self.last_rpm as i32).unsigned_abs();
        if elapsed == 0 {
            return;
        }
        let rate = (rpm_change as u64 * 1_000_000) / elapsed as u64;
        if rate > self.config.rpm_spike_threshold as u64 {
            self.trigger(CaptureTrigger::RpmSpike);
        }
    }

    /// Trigger capture and transition from `Armed` to `Capturing`.
    ///
    /// The ring buffer contents become pre-trigger data.
    pub fn trigger(&mut self, reason: CaptureTrigger) {
        if self.state != CaptureState::Armed {
            return;
        }
        if !self.config.is_trigger_enabled(reason) {
            return;
        }

        self.state = CaptureState::Capturing;
        self.trigger_reason = Some(reason);
        self.trigger_index = if self.head == 0 { N - 1 } else { self.head - 1 };
        self.post_remaining = self.config.post_trigger_samples.min(N as u16);
    }

    /// Reset capture system.
    pub fn reset(&mut self) {
        self.head = 0;
        self.count = 0;
        self.state = if self.config.enable {
            CaptureState::Armed
        } else {
            CaptureState::Disabled
        };
        self.trigger_reason = None;
        self.trigger_index = 0;
        self.post_remaining = 0;
    }

    /// Arm the capture system.
    pub fn arm(&mut self) {
        if self.config.enable {
            self.reset();
            self.state = CaptureState::Armed;
        }
    }

    /// Disable capture.
    pub fn disable(&mut self) {
        self.state = CaptureState::Disabled;
    }

    /// Enable capture.
    pub fn enable(&mut self) {
        if self.state == CaptureState::Disabled {
            self.state = CaptureState::Armed;
        }
    }
}
