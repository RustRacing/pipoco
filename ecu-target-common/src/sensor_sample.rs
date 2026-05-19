use crate::live_inputs::SplitLiveInputs;
use ecu_core::hal::TimeSource;
use ecu_domain::{Kpa10, Micros};
use ecu_io::{CaptureSample, SensorSource};

/// Non-trigger sensor values sampled by the board.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitSensorSignals {
    pub at_us: Micros,
    pub load_kpa10: Kpa10,
}

pub fn split_capture_sample(live: SplitLiveInputs, signals: SplitSensorSignals) -> CaptureSample {
    CaptureSample {
        at_us: signals.at_us,
        rpm: live.rpm(),
        load_kpa10: signals.load_kpa10,
        angle_x10: live.angle_x10(),
    }
}

pub trait LoadKpa10Source {
    type Error;

    fn load_kpa10(&mut self) -> Result<Kpa10, Self::Error>;
}

pub trait SplitLiveEventSink {
    fn apply_live_event(&mut self, event: crate::adapter::BoardEvent);
}

/// Sensor source for boards that have a live load source but share trigger state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LiveLoadSensor<T: TimeSource, L> {
    time_source: T,
    load: L,
    live: SplitLiveInputs,
}

impl<T: TimeSource, L> LiveLoadSensor<T, L> {
    pub const fn new(time_source: T, load: L) -> Self {
        Self {
            time_source,
            load,
            live: SplitLiveInputs::new(),
        }
    }

    pub fn load_source_mut(&mut self) -> &mut L {
        &mut self.load
    }
}

impl<T: TimeSource, L> SplitLiveEventSink for LiveLoadSensor<T, L> {
    fn apply_live_event(&mut self, event: crate::adapter::BoardEvent) {
        self.live.apply_event(event);
    }
}

impl<T: TimeSource, L: LoadKpa10Source> SensorSource for LiveLoadSensor<T, L> {
    type Error = L::Error;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        let load_kpa10 = self.load.load_kpa10()?;
        Ok(split_capture_sample(
            self.live,
            SplitSensorSignals {
                at_us: Micros::new(self.time_source.micros()),
                load_kpa10,
            },
        ))
    }
}

/// Bring-up sensor source for boards that do not yet have ADC plumbing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedLoadSensor<T: TimeSource> {
    time_source: T,
    live: SplitLiveInputs,
    load_kpa10: Kpa10,
}

impl<T: TimeSource> FixedLoadSensor<T> {
    pub const fn new(time_source: T, load_kpa10: Kpa10) -> Self {
        Self {
            time_source,
            live: SplitLiveInputs::new(),
            load_kpa10,
        }
    }
}

impl<T: TimeSource> SplitLiveEventSink for FixedLoadSensor<T> {
    fn apply_live_event(&mut self, event: crate::adapter::BoardEvent) {
        self.live.apply_event(event);
    }
}

impl<T: TimeSource> SensorSource for FixedLoadSensor<T> {
    type Error = core::convert::Infallible;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        Ok(split_capture_sample(
            self.live,
            SplitSensorSignals {
                at_us: Micros::new(self.time_source.micros()),
                load_kpa10: self.load_kpa10,
            },
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::BoardEvent;
    use ecu_domain::{Degrees10, Rpm};

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockTime(u32);

    impl TimeSource for MockTime {
        fn micros(&self) -> u32 {
            self.0
        }
    }

    #[test]
    fn split_capture_sample_combines_live_trigger_and_sensor_load() {
        let mut live = SplitLiveInputs::new();
        live.apply_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(10),
            rpm: Rpm::new(2_400),
            angle_x10: Degrees10::new(450),
            authority: ecu_domain::EngineTimeAuthority::none(),
            synced: true,
        });

        let sample = split_capture_sample(
            live,
            SplitSensorSignals {
                at_us: Micros::new(20),
                load_kpa10: Kpa10::new(850),
            },
        );

        assert_eq!(sample.at_us, Micros::new(20));
        assert_eq!(sample.rpm, Rpm::new(2_400));
        assert_eq!(sample.load_kpa10, Kpa10::new(850));
        assert_eq!(sample.angle_x10, Degrees10::new(450));
    }

    #[test]
    fn split_capture_sample_stays_zeroed_without_trigger_input() {
        let sample = split_capture_sample(
            SplitLiveInputs::new(),
            SplitSensorSignals {
                at_us: Micros::new(30),
                load_kpa10: Kpa10::new(700),
            },
        );

        assert_eq!(sample.rpm, Rpm::new(0));
        assert_eq!(sample.angle_x10, Degrees10::new(0));
    }

    #[test]
    fn fixed_load_sensor_uses_time_load_and_live_trigger_inputs() {
        let mut sensor = FixedLoadSensor::new(MockTime(123), Kpa10::new(700));
        sensor.apply_live_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(10),
            rpm: Rpm::new(1_900),
            angle_x10: Degrees10::new(300),
            authority: ecu_domain::EngineTimeAuthority::none(),
            synced: true,
        });

        let sample = sensor.sample().unwrap();

        assert_eq!(sample.at_us, Micros::new(123));
        assert_eq!(sample.rpm, Rpm::new(1_900));
        assert_eq!(sample.load_kpa10, Kpa10::new(700));
        assert_eq!(sample.angle_x10, Degrees10::new(300));
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockLoad(Kpa10);

    impl LoadKpa10Source for MockLoad {
        type Error = core::convert::Infallible;

        fn load_kpa10(&mut self) -> Result<Kpa10, Self::Error> {
            Ok(self.0)
        }
    }

    #[test]
    fn live_load_sensor_uses_dynamic_load_and_live_trigger_inputs() {
        let mut sensor = LiveLoadSensor::new(MockTime(456), MockLoad(Kpa10::new(920)));
        sensor.apply_live_event(BoardEvent::TriggerEdge {
            at_us: Micros::new(10),
            rpm: Rpm::new(2_100),
            angle_x10: Degrees10::new(510),
            authority: ecu_domain::EngineTimeAuthority::none(),
            synced: true,
        });

        let sample = sensor.sample().unwrap();

        assert_eq!(sample.at_us, Micros::new(456));
        assert_eq!(sample.rpm, Rpm::new(2_100));
        assert_eq!(sample.load_kpa10, Kpa10::new(920));
        assert_eq!(sample.angle_x10, Degrees10::new(510));
    }
}
