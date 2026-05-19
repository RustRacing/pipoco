use crate::adapter::{AdapterResult, BoardAdapter, BoardEvent};
use crate::sensor_sample::SplitLiveEventSink;
use ecu_core::hal::TimeSource;
use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_io::{
    ActionExecutor, CalibrationStore, CaptureSink, SensorSource, TransportPublisher, Watchdog,
};
use ecu_trigger::{
    EngineTimeLatency, PollLevelPolarity, ProfiledMissingToothDecoder, ResyncPolicy,
    SecondaryTriggerMode, SecondaryTriggerProfile, StartupSyncPolicy, TriggerAngleAuthority,
    TriggerEdge, TriggerFilter, TriggerPattern, TriggerProfile, TriggerSpeed,
};

const DEFAULT_MINIMUM_EDGE_INTERVAL_US: u32 = 0;

fn default_trigger_profile() -> TriggerProfile {
    TriggerProfile {
        pattern: TriggerPattern::MissingTooth {
            nominal_teeth: 60,
            missing_teeth: 2,
        },
        primary_speed: TriggerSpeed::Crank,
        primary_edge: TriggerEdge::Rising,
        secondary: SecondaryTriggerProfile {
            mode: SecondaryTriggerMode::None,
            edge: TriggerEdge::Rising,
            poll_level: PollLevelPolarity::ActiveHigh,
        },
        trigger_angle_atdc_deg10: TriggerAngleAuthority::ExpertManual(Degrees10::new(0)),
        tooth_angle_multiplier: 1,
        filter: TriggerFilter::Off,
        resync: ResyncPolicy::OnSyncLoss,
        startup: StartupSyncPolicy {
            skip_revolutions: 2,
            require_full_cycle: true,
        },
        latency: EngineTimeLatency::default(),
    }
}

/// Converts captured trigger-edge timestamps into split-runtime board events.
///
/// This bridge now uses the shared `ecu-trigger` missing-tooth decoder so the
/// embedded common path shares the same geometry, authority, and latency logic
/// as the host-facing trigger profile code.
pub struct SplitTriggerAdapter<T: TimeSource> {
    _time_source: T,
    decoder: ProfiledMissingToothDecoder,
}

impl<T: TimeSource> SplitTriggerAdapter<T> {
    pub fn new(time_source: T) -> Self {
        Self::with_profile(time_source, default_trigger_profile())
    }

    pub fn with_profile(time_source: T, profile: TriggerProfile) -> Self {
        let decoder = match ProfiledMissingToothDecoder::try_new(
            profile,
            ecu_domain::Ticks::new(DEFAULT_MINIMUM_EDGE_INTERVAL_US),
            ecu_trigger::DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        ) {
            Ok(decoder) => decoder,
            Err(error) => panic!("trigger profile rejected: {error:?}"),
        };

        Self {
            _time_source: time_source,
            decoder,
        }
    }

    pub fn on_trigger_edge(&mut self, at_us: u32) -> BoardEvent {
        let _ = self
            .decoder
            .ingest_primary_edge(ecu_domain::Ticks::new(at_us));

        let authority = self.decoder.authority();
        BoardEvent::TriggerEdge {
            at_us: Micros::new(at_us),
            rpm: self.decoder.rpm(),
            angle_x10: self
                .decoder
                .crank_angle_at(ecu_domain::Ticks::new(at_us))
                .unwrap_or_default(),
            authority,
            synced: self.decoder.synced(),
        }
    }

    pub fn authority(&self) -> ecu_domain::EngineTimeAuthority {
        self.decoder.authority()
    }

    pub fn rpm(&self) -> Rpm {
        self.decoder.rpm()
    }

    pub fn angle_x10(&self, now_us: u32) -> Degrees10 {
        self.decoder
            .crank_angle_at(ecu_domain::Ticks::new(now_us))
            .unwrap_or_default()
    }

    pub fn synced(&self) -> bool {
        self.decoder.synced()
    }

    pub fn reset_sync(&mut self) {
        self.decoder.reset_sync();
    }
}

#[allow(clippy::type_complexity)]
pub fn apply_trigger_timestamp<S, C, A, W, T, P, D>(
    adapter: &mut BoardAdapter<S, C, A, W, T, P>,
    trigger: &mut SplitTriggerAdapter<D>,
    timestamp_us: u32,
) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, ()>
where
    S: SensorSource + SplitLiveEventSink,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: CalibrationStore,
    D: TimeSource,
{
    let event = trigger.on_trigger_edge(timestamp_us);
    adapter.sensor().apply_live_event(event);
    adapter.apply_event(event).map(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::BoardAdapter;
    use crate::noop::{NoopStore, NoopTransport, NoopWatchdog};
    use crate::sensor_sample::FixedLoadSensor;
    use ecu_domain::Kpa10;
    use ecu_io::{ActionExecutor, SensorSource};
    use ecu_runtime::Action;

    #[derive(Debug, Clone, Copy)]
    struct MockTime;

    impl TimeSource for MockTime {
        fn micros(&self) -> u32 {
            0
        }
    }

    #[test]
    fn trigger_adapter_reports_unsynced_before_missing_tooth() {
        let mut adapter = SplitTriggerAdapter::new(MockTime);

        let event = adapter.on_trigger_edge(1_000);

        assert_eq!(
            event,
            BoardEvent::TriggerEdge {
                at_us: Micros::new(1_000),
                rpm: Rpm::new(0),
                angle_x10: Degrees10::new(0),
                authority: adapter.authority(),
                synced: false,
            }
        );
    }

    #[test]
    fn trigger_adapter_reports_sync_and_rpm_after_missing_tooth() {
        let mut adapter = SplitTriggerAdapter::new(MockTime);

        let _ = adapter.on_trigger_edge(1_000);
        let _ = adapter.on_trigger_edge(2_000);
        let event = adapter.on_trigger_edge(4_000);

        assert_eq!(
            event,
            BoardEvent::TriggerEdge {
                at_us: Micros::new(4_000),
                rpm: Rpm::new(1_000),
                angle_x10: Degrees10::new(0),
                authority: adapter.authority(),
                synced: true,
            }
        );
        assert_eq!(adapter.rpm(), Rpm::new(1_000));
        assert!(adapter.synced());
    }

    #[test]
    fn trigger_adapter_angle_advances_from_decoder_estimate() {
        let mut adapter = SplitTriggerAdapter::new(MockTime);

        let _ = adapter.on_trigger_edge(1_000);
        let _ = adapter.on_trigger_edge(2_000);
        let _ = adapter.on_trigger_edge(4_000);
        let angle = adapter.angle_x10(18_500);

        assert_eq!(angle, Degrees10::new(900));
    }

    #[test]
    fn trigger_adapter_reset_clears_split_observation_state() {
        let mut adapter = SplitTriggerAdapter::new(MockTime);

        let _ = adapter.on_trigger_edge(1_000);
        let _ = adapter.on_trigger_edge(2_000);
        let _ = adapter.on_trigger_edge(4_000);
        adapter.reset_sync();

        assert_eq!(adapter.rpm(), Rpm::new(0));
        assert_eq!(adapter.angle_x10(18_500), Degrees10::new(0));
        assert!(!adapter.synced());
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct MockActions;

    impl ActionExecutor for MockActions {
        type Error = core::convert::Infallible;

        fn execute(&mut self, _action: Action) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    #[test]
    fn apply_trigger_timestamp_updates_sensor_live_state_before_runtime_event() {
        let mut board = BoardAdapter::new(
            FixedLoadSensor::new(MockTime, Kpa10::new(700)),
            crate::noop::NoopCapture,
            MockActions,
            NoopWatchdog,
            NoopTransport,
            NoopStore,
        );
        let mut trigger = SplitTriggerAdapter::new(MockTime);

        apply_trigger_timestamp(&mut board, &mut trigger, 1_000).unwrap();
        apply_trigger_timestamp(&mut board, &mut trigger, 2_000).unwrap();
        apply_trigger_timestamp(&mut board, &mut trigger, 4_000).unwrap();
        let sample = board.sensor().sample().unwrap();

        assert_eq!(sample.rpm, Rpm::new(1_000));
        assert_eq!(sample.angle_x10, Degrees10::new(0));
        assert_eq!(sample.load_kpa10, Kpa10::new(700));
        assert!(trigger.synced());
        assert_eq!(
            board.runtime().snapshot().engine.engine_time_authority,
            trigger.authority()
        );
    }
}
