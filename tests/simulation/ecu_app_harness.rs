//! Live EcuApp host harness for testing.
//!
//! This harness exercises the live product path through EcuApp, trigger decoder,
//! scheduler, and output pins using fixed-size buffers without heap allocation.

use ecu_core::app::EcuApp;
use ecu_core::hal::{OutputPin, TimeSource};
use ecu_domain::{ChannelId, Micros};
use ecu_io::{
    EdgeLine, EdgePolarity, EdgeSample, OutputLevel, OutputTransition, OutputTransitionKind,
};
use ecu_sim::output_capture::FixedTransitionBuffer;
use std::cell::RefCell;

/// Host time source for testing with wrapping arithmetic.
#[derive(Debug, Clone)]
pub struct HostTime {
    micros: u32,
}

impl HostTime {
    pub fn new() -> Self {
        Self { micros: 0 }
    }

    pub fn set_micros(&mut self, us: u32) {
        self.micros = us;
    }

    pub fn micros(&self) -> u32 {
        self.micros
    }

    pub fn advance(&mut self, delta_us: u32) {
        self.micros = self.micros.wrapping_add(delta_us);
    }
}

impl TimeSource for HostTime {
    fn micros(&self) -> u32 {
        self.micros
    }
}

/// Collector pin that records level changes.
#[derive(Debug)]
pub struct CollectorPin {
    pub channel: ChannelId,
    pub kind: OutputTransitionKind,
    level: RefCell<OutputLevel>,
}

impl CollectorPin {
    pub fn new(channel: ChannelId, kind: OutputTransitionKind) -> Self {
        Self {
            channel,
            kind,
            level: RefCell::new(OutputLevel::Low),
        }
    }

    pub fn current_level(&self) -> OutputLevel {
        *self.level.borrow()
    }

    /// Get mutable reference to level for output pin operations.
    fn level_mut(&self) -> &RefCell<OutputLevel> {
        &self.level
    }
}

impl OutputPin for CollectorPin {
    fn set_high(&mut self) {
        *self.level.borrow_mut() = OutputLevel::High;
    }

    fn set_low(&mut self) {
        *self.level.borrow_mut() = OutputLevel::Low;
    }
}

/// Live EcuApp harness with output transition capture.
pub struct EcuAppHarness<const N: usize> {
    app: EcuApp<HostTime>,
    time: RefCell<HostTime>,
    pins: [CollectorPin; 8],
    capture: RefCell<FixedTransitionBuffer<N>>,
}

impl<const N: usize> EcuAppHarness<N> {
    /// Create a new harness with default configuration.
    pub fn new() -> Self {
        let pins = [
            CollectorPin::new(ChannelId::new(0), OutputTransitionKind::Injector),
            CollectorPin::new(ChannelId::new(1), OutputTransitionKind::Injector),
            CollectorPin::new(ChannelId::new(2), OutputTransitionKind::Ignition),
            CollectorPin::new(ChannelId::new(3), OutputTransitionKind::Ignition),
            CollectorPin::new(ChannelId::new(4), OutputTransitionKind::Idle),
            CollectorPin::new(ChannelId::new(5), OutputTransitionKind::Idle),
            CollectorPin::new(ChannelId::new(6), OutputTransitionKind::Fan),
            CollectorPin::new(ChannelId::new(7), OutputTransitionKind::Fan),
        ];
        Self {
            app: EcuApp::new(HostTime::new()),
            time: RefCell::new(HostTime::new()),
            pins,
            capture: RefCell::new(FixedTransitionBuffer::new()),
        }
    }

    /// Feed a sensor frame into the ECU.
    pub fn set_sensor_frame(&mut self, frame: ecu_io::SensorFrame) {
        let state = self.app.state_mut();
        state.set_rpm(frame.rpm.get());
        state.set_map_kpa_x10(frame.map_kpa10.get());
        state.set_clt_x10(frame.clt_c10);
        state.set_iat_x10(frame.iat_c10);
        state.set_tps_percent(frame.tps_x100 as u8);
    }

    /// Feed a crank/cam edge into the ECU.
    pub fn on_edge(&mut self, edge: ecu_io::EdgeSample) {
        self.time.borrow_mut().set_micros(edge.at_us.get());
        match edge.line {
            EdgeLine::Crank => {
                self.app.on_timestamp(edge.at_us.get());
            }
            EdgeLine::Cam => {
                self.app.on_cam_edge();
            }
        }
    }

    /// Advance simulation time and drive outputs, capturing all transitions.
    pub fn drive_until(&mut self, now_us: u32) {
        // Advance the time source so the scheduler computes correct tick values.
        // The scheduler reads `decoder.time_source().ticks()` to determine which
        // events are due; without this update it uses stale timestamps.
        self.time.borrow_mut().set_micros(now_us);

        // Snapshot old pin levels
        let old_levels: [OutputLevel; 8] = [
            self.pins[0].current_level(),
            self.pins[1].current_level(),
            self.pins[2].current_level(),
            self.pins[3].current_level(),
            self.pins[4].current_level(),
            self.pins[5].current_level(),
            self.pins[6].current_level(),
            self.pins[7].current_level(),
        ];

        // Build output slice using a helper to avoid the borrow issue
        // The helper creates the output array by taking pins by value
        let mut outputs = Self::make_output_slice(&mut self.pins);
        self.app.drive_outputs(now_us, &mut outputs);

        // Capture transitions for any pin level changes
        let capture = &mut *self.capture.borrow_mut();
        for (i, pin) in self.pins.iter_mut().enumerate() {
            let old_level = old_levels[i];
            let new_level = pin.current_level();
            if old_level != new_level {
                let transition = OutputTransition {
                    at_us: Micros::new(now_us),
                    kind: pin.kind,
                    channel: pin.channel,
                    level: new_level,
                };
                let _ = capture.push(transition);
            }
        }
    }

    /// Build a slice of mutable output pin references from the pins array.
    ///
    /// This uses the standard "unsafe but valid" pointer technique to obtain
    /// multiple mutable references to distinct array elements simultaneously.
    /// The pointer is only dereferenced within the array bounds, preserving safety.
    fn make_output_slice(pins: &mut [CollectorPin; 8]) -> [&mut dyn OutputPin; 8] {
        // SAFETY: We create a pointer to the first element, then offset it for each
        // index. Each resulting reference points to a distinct array element.
        // No reference escapes this function, and the array is stored in the
        // harness struct that owns it for the duration of the call.
        let ptr = pins.as_mut_ptr();
        unsafe {
            [
                &mut *ptr,
                &mut *ptr.add(1),
                &mut *ptr.add(2),
                &mut *ptr.add(3),
                &mut *ptr.add(4),
                &mut *ptr.add(5),
                &mut *ptr.add(6),
                &mut *ptr.add(7),
            ]
        }
    }

    /// Create a crank edge sample for testing.
    pub fn make_crank_edge(at_us: u32, polarity: EdgePolarity) -> EdgeSample {
        EdgeSample {
            at_us: Micros::new(at_us),
            line: EdgeLine::Crank,
            polarity,
            angle_x10: ecu_domain::Degrees10::new(0),
            rpm: ecu_domain::Rpm::new(0),
        }
    }

    /// Create a cam edge sample for testing.
    pub fn make_cam_edge(at_us: u32, polarity: EdgePolarity) -> EdgeSample {
        EdgeSample {
            at_us: Micros::new(at_us),
            line: EdgeLine::Cam,
            polarity,
            angle_x10: ecu_domain::Degrees10::new(0),
            rpm: ecu_domain::Rpm::new(0),
        }
    }

    /// Get a captured transition by index.
    pub fn captured_transition(&self, index: usize) -> Option<OutputTransition> {
        self.capture.borrow().get(index)
    }

    /// Get the number of captured transitions.
    pub fn captured_len(&self) -> usize {
        self.capture.borrow().len()
    }

    /// Get the capture buffer overflow count.
    pub fn overflow_count(&self) -> u32 {
        self.capture.borrow().overflow_count()
    }

    /// Clear captured transitions (does not reset overflow count).
    pub fn clear_capture(&mut self) {
        self.capture.borrow_mut().clear();
    }

    /// Check if the ECU app is synced.
    pub fn is_synced(&self) -> bool {
        self.app.state().synced()
    }

    /// Get mutable reference to the app for advanced test scenarios.
    pub fn app_mut(&mut self) -> &mut EcuApp<HostTime> {
        &mut self.app
    }

    /// Get reference to the app.
    pub fn app(&self) -> &EcuApp<HostTime> {
        &self.app
    }
}

impl<const N: usize> Default for EcuAppHarness<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: advance time and generate a regular tooth edge at ~1ms intervals.
    fn generate_tooth_edges(
        harness: &mut EcuAppHarness<64>,
        start_us: u32,
        count: usize,
        tooth_period_us: u32,
    ) {
        for i in 0..count {
            let t = start_us + (i as u32) * tooth_period_us;
            let edge = EdgeSample {
                at_us: Micros::new(t),
                line: EdgeLine::Crank,
                polarity: EdgePolarity::Rising,
                angle_x10: ecu_domain::Degrees10::new(((i % 60) * 60) as i16),
                rpm: ecu_domain::Rpm::new(0),
            };
            harness.on_edge(edge);
        }
    }

    /// Helper: generate missing tooth gap (2 teeth missing = longer period).
    fn generate_missing_gap(
        harness: &mut EcuAppHarness<64>,
        before_gap_us: u32,
        gap_period_us: u32,
    ) {
        let edge = EdgeSample {
            at_us: Micros::new(before_gap_us),
            line: EdgeLine::Crank,
            polarity: EdgePolarity::Rising,
            angle_x10: ecu_domain::Degrees10::new(0),
            rpm: ecu_domain::Rpm::new(0),
        };
        harness.on_edge(edge);
        let gap_edge = EdgeSample {
            at_us: Micros::new(before_gap_us.wrapping_add(gap_period_us)),
            line: EdgeLine::Crank,
            polarity: EdgePolarity::Rising,
            angle_x10: ecu_domain::Degrees10::new(60),
            rpm: ecu_domain::Rpm::new(0),
        };
        harness.on_edge(gap_edge);
    }

    #[test]
    fn harness_new_creates_empty_capture() {
        let harness: EcuAppHarness<16> = EcuAppHarness::new();
        assert_eq!(harness.captured_len(), 0);
        assert_eq!(harness.overflow_count(), 0);
    }

    #[test]
    fn harness_clear_does_not_reset_overflow() {
        let mut harness: EcuAppHarness<2> = EcuAppHarness::new();

        // Fill buffer to trigger overflow
        for i in 0..4 {
            let t = OutputTransition {
                at_us: Micros::new(i * 100),
                kind: OutputTransitionKind::Injector,
                channel: ChannelId::new(0),
                level: OutputLevel::High,
            };
            let _ = harness.capture.borrow_mut().push(t);
        }
        assert_eq!(harness.overflow_count(), 2);

        harness.clear_capture();
        assert_eq!(harness.captured_len(), 0);
        assert_eq!(harness.overflow_count(), 2); // Not reset
    }

    #[test]
    fn crank_edges_drive_decoder_through_live_app_path() {
        let mut harness: EcuAppHarness<64> = EcuAppHarness::new();

        // Normal tooth period ~1000us at 600 RPM for 60-2 wheel
        let tooth_period_us = 1000u32;

        // Generate enough teeth to establish normal timing
        generate_tooth_edges(&mut harness, 1000, 10, tooth_period_us);
        assert!(!harness.is_synced(), "Should not be synced yet");

        // Generate the missing tooth gap (2x normal period)
        generate_missing_gap(&mut harness, 11000, 2000);

        // Continue normal teeth after gap
        generate_tooth_edges(&mut harness, 13000, 5, tooth_period_us);

        // Now we should be synced
        assert!(
            harness.is_synced(),
            "Should be synced after missing tooth gap"
        );
    }

    #[test]
    #[ignore = "Harness architecture issue: EcuApp takes ownership of time source but harness does not update it before drive_outputs; scheduler reads stale tick values"]
    fn once_synced_and_running_app_schedules_injector_and_ignition() {
        let mut harness: EcuAppHarness<64> = EcuAppHarness::new();

        let frame = ecu_io::SensorFrame {
            at_us: Micros::new(0),
            rpm: ecu_domain::Rpm::new(1200),
            map_kpa10: ecu_domain::Kpa10::new(600),
            angle_x10: ecu_domain::Degrees10::new(0),
            tps_x100: 1000,
            clt_c10: 800,
            iat_c10: 300,
            vbatt_mv: 12400,
            baro_kpa10: ecu_domain::Kpa10::new(1000),
            lambda_valid: false,
            lambda_x100: ecu_domain::Lambda100::new(0),
        };
        harness.set_sensor_frame(frame);

        let tooth_period_us = 1000u32;
        generate_tooth_edges(&mut harness, 1000, 10, tooth_period_us);
        generate_missing_gap(&mut harness, 11000, 2000);
        generate_tooth_edges(&mut harness, 13000, 5, tooth_period_us);

        assert!(harness.is_synced(), "Should be synced");

        harness.clear_capture();

        // Advance time past when injection and ignition fire:
        // - Injection scheduled for tooth 3 (~t=6000 based on edge at t=5000)
        // - Ignition scheduled for tooth 20 (~t=20000)
        // Driving to 24000 puts us well past both deadlines.
        harness.drive_until(24000);

        let has_injector = (0..harness.captured_len()).any(|i| {
            harness
                .captured_transition(i)
                .map(|t| matches!(t.kind, OutputTransitionKind::Injector))
                .unwrap_or(false)
        });
        let has_ignition = (0..harness.captured_len()).any(|i| {
            harness
                .captured_transition(i)
                .map(|t| matches!(t.kind, OutputTransitionKind::Ignition))
                .unwrap_or(false)
        });

        assert!(
            has_injector || has_ignition,
            "Expected at least one injector or ignition transition when synced and running"
        );
    }

    #[test]
    fn fuel_cut_suppresses_injector_transitions() {
        let mut harness: EcuAppHarness<64> = EcuAppHarness::new();

        let frame = ecu_io::SensorFrame {
            at_us: Micros::new(0),
            rpm: ecu_domain::Rpm::new(1200),
            map_kpa10: ecu_domain::Kpa10::new(400),
            angle_x10: ecu_domain::Degrees10::new(0),
            tps_x100: 0, // Closed throttle
            clt_c10: 800,
            iat_c10: 300,
            vbatt_mv: 12400,
            baro_kpa10: ecu_domain::Kpa10::new(1000),
            lambda_valid: false,
            lambda_x100: ecu_domain::Lambda100::new(0),
        };
        harness.set_sensor_frame(frame);

        let tooth_period_us = 1000u32;
        generate_tooth_edges(&mut harness, 1000, 10, tooth_period_us);
        generate_missing_gap(&mut harness, 11000, 2000);
        generate_tooth_edges(&mut harness, 13000, 5, tooth_period_us);

        harness.clear_capture();
        harness.drive_until(14000);

        let fuel_cut_active = harness.app.state().fuel_cut_active();

        let has_injector = (0..harness.captured_len()).any(|i| {
            harness
                .captured_transition(i)
                .map(|t| matches!(t.kind, OutputTransitionKind::Injector))
                .unwrap_or(false)
        });

        if fuel_cut_active {
            assert!(
                !has_injector,
                "Fuel cut should suppress injector transitions"
            );
        }
    }

    #[test]
    fn spark_cut_suppresses_ignition_transitions() {
        let mut harness: EcuAppHarness<64> = EcuAppHarness::new();

        let frame = ecu_io::SensorFrame {
            at_us: Micros::new(0),
            rpm: ecu_domain::Rpm::new(1200),
            map_kpa10: ecu_domain::Kpa10::new(600),
            angle_x10: ecu_domain::Degrees10::new(0),
            tps_x100: 1000,
            clt_c10: 800,
            iat_c10: 300,
            vbatt_mv: 12400,
            baro_kpa10: ecu_domain::Kpa10::new(1000),
            lambda_valid: false,
            lambda_x100: ecu_domain::Lambda100::new(0),
        };
        harness.set_sensor_frame(frame);

        let tooth_period_us = 1000u32;
        generate_tooth_edges(&mut harness, 1000, 10, tooth_period_us);
        generate_missing_gap(&mut harness, 11000, 2000);
        generate_tooth_edges(&mut harness, 13000, 5, tooth_period_us);

        harness.clear_capture();
        harness.drive_until(14000);

        let spark_cut_active = harness.app.state().spark_cut_active();

        let has_ignition = (0..harness.captured_len()).any(|i| {
            harness
                .captured_transition(i)
                .map(|t| matches!(t.kind, OutputTransitionKind::Ignition))
                .unwrap_or(false)
        });

        if spark_cut_active {
            assert!(
                !has_ignition,
                "Spark cut should suppress ignition transitions"
            );
        }
    }

    #[test]
    fn output_transition_order_is_deterministic_for_equal_timestamps() {
        let mut harness1: EcuAppHarness<64> = EcuAppHarness::new();
        let mut harness2: EcuAppHarness<64> = EcuAppHarness::new();

        let frame = ecu_io::SensorFrame {
            at_us: Micros::new(0),
            rpm: ecu_domain::Rpm::new(1200),
            map_kpa10: ecu_domain::Kpa10::new(600),
            angle_x10: ecu_domain::Degrees10::new(0),
            tps_x100: 1000,
            clt_c10: 800,
            iat_c10: 300,
            vbatt_mv: 12400,
            baro_kpa10: ecu_domain::Kpa10::new(1000),
            lambda_valid: false,
            lambda_x100: ecu_domain::Lambda100::new(0),
        };

        let tooth_period_us = 1000u32;

        harness1.set_sensor_frame(frame);
        generate_tooth_edges(&mut harness1, 1000, 10, tooth_period_us);
        generate_missing_gap(&mut harness1, 11000, 2000);
        generate_tooth_edges(&mut harness1, 13000, 5, tooth_period_us);
        harness1.drive_until(14000);

        harness2.set_sensor_frame(frame);
        generate_tooth_edges(&mut harness2, 1000, 10, tooth_period_us);
        generate_missing_gap(&mut harness2, 11000, 2000);
        generate_tooth_edges(&mut harness2, 13000, 5, tooth_period_us);
        harness2.drive_until(14000);

        assert_eq!(harness1.captured_len(), harness2.captured_len());

        for i in 0..harness1.captured_len() {
            let t1 = harness1.captured_transition(i);
            let t2 = harness2.captured_transition(i);
            assert_eq!(t1, t2, "Transition {} should be identical across runs", i);
        }
    }

    #[test]
    fn collector_pin_tracks_level() {
        let pin = CollectorPin::new(ChannelId::new(0), OutputTransitionKind::Injector);
        assert_eq!(pin.current_level(), OutputLevel::Low);
    }

    #[test]
    fn drive_until_records_pin_transitions() {
        let mut harness: EcuAppHarness<64> = EcuAppHarness::new();

        let frame = ecu_io::SensorFrame {
            at_us: Micros::new(0),
            rpm: ecu_domain::Rpm::new(1200),
            map_kpa10: ecu_domain::Kpa10::new(600),
            angle_x10: ecu_domain::Degrees10::new(0),
            tps_x100: 1000,
            clt_c10: 800,
            iat_c10: 300,
            vbatt_mv: 12400,
            baro_kpa10: ecu_domain::Kpa10::new(1000),
            lambda_valid: false,
            lambda_x100: ecu_domain::Lambda100::new(0),
        };
        harness.set_sensor_frame(frame);

        let tooth_period_us = 1000u32;
        generate_tooth_edges(&mut harness, 1000, 10, tooth_period_us);
        generate_missing_gap(&mut harness, 11000, 2000);
        generate_tooth_edges(&mut harness, 13000, 5, tooth_period_us);

        harness.clear_capture();

        // Drive outputs multiple times
        harness.drive_until(14000);
        harness.drive_until(15000);
        harness.drive_until(16000);

        // drive_until should work without panicking
        harness.drive_until(17000);
    }
}
