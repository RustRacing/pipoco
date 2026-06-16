use crate::adapter::{
    AdapterResult, ApplyAndPushPairError, ApplyAndRecordError, BoardAdapter, BoardAdapterError,
    BoardEvent, CommonObservabilityRecordTraceOverflow, CommonObservabilityTraceOverflow,
    FixedCommonObservabilityRecordTrace, FixedCommonObservabilityTrace,
    FixedCommonObservabilityTracePair,
};
use crate::live_inputs::{SplitLiveTriggerEvent, SplitSyncState};
use crate::outputs::ScheduledActionExecutor;
use crate::sensor_sample::SplitLiveEventSink;
use ecu_board_api::{CaptureSampleSource, CaptureSink, EcuClock, Watchdog};
use ecu_calibration::PersistedCalibrationStore;
use ecu_domain::{Degrees10, Micros, Rpm};
use ecu_runtime::{ActionExecutor, TransportPublisher};
use ecu_trigger::{
    EngineTimeLatency, PollLevelPolarity, ProfiledMissingToothDecoder, ResyncPolicy,
    RuntimeMissingToothProfile, SecondaryTriggerMode, SecondaryTriggerProfile, StartupSyncPolicy,
    TriggerAngleAuthority, TriggerEdge, TriggerFilter, TriggerPattern, TriggerProfile,
    TriggerSpeed, TriggerValidationError,
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
pub struct SplitTriggerAdapter<T: EcuClock> {
    _time_source: T,
    decoder: ProfiledMissingToothDecoder,
}

impl<T: EcuClock> SplitTriggerAdapter<T> {
    pub fn new(time_source: T) -> Self {
        match Self::with_profile(time_source, default_trigger_profile()) {
            Ok(adapter) => adapter,
            Err(_) => unreachable!("default trigger profile must validate"),
        }
    }

    pub fn with_profile(
        time_source: T,
        profile: TriggerProfile,
    ) -> Result<Self, TriggerValidationError> {
        let runtime_profile = RuntimeMissingToothProfile::from_import_profile(profile)?;
        let decoder = ProfiledMissingToothDecoder::try_new(
            runtime_profile,
            ecu_domain::Ticks::new(DEFAULT_MINIMUM_EDGE_INTERVAL_US),
            ecu_trigger::DEFAULT_MISSING_TOOTH_GAP_RATIO_X1000,
        )?;

        Ok(Self {
            _time_source: time_source,
            decoder,
        })
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

    pub fn sync_state(&self) -> SplitSyncState {
        SplitSyncState::from_authority(self.decoder.authority())
    }

    pub fn live_trigger_event(&self, now_us: u32) -> SplitLiveTriggerEvent {
        SplitLiveTriggerEvent::new(self.rpm(), self.angle_x10(now_us), self.synced())
    }

    pub fn reset_sync(&mut self) {
        self.decoder.reset_sync();
    }
}

#[allow(clippy::type_complexity)]
/// Applies a board trigger timestamp through the `boards/common` runtime seam.
///
/// This helper converts board trigger edges into runtime adapter calls while
/// keeping raw capture/runtime trait dependencies in `boards/common`, not in
/// `ecu-board-api`.
pub fn apply_trigger_timestamp_to_runtime_adapter<S, C, A, W, T, P, D>(
    adapter: &mut BoardAdapter<S, C, A, W, T, P>,
    trigger: &mut SplitTriggerAdapter<D>,
    timestamp_us: u32,
) -> AdapterResult<S::Error, C::Error, A::Error, W::Error, T::Error, P::Error, ()>
where
    S: CaptureSampleSource + SplitLiveEventSink,
    C: CaptureSink,
    A: ActionExecutor,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    D: EcuClock,
{
    let event = trigger.on_trigger_edge(timestamp_us);
    let live_event = trigger.live_trigger_event(timestamp_us);
    adapter.sensor().apply_live_event(live_event);
    adapter.apply_event(event).map(|_| ())
}

#[allow(clippy::type_complexity)]
pub fn apply_trigger_timestamp_to_runtime_adapter_and_record<
    S,
    C,
    W,
    T,
    P,
    D,
    const Q: usize,
    const R: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    trigger: &mut SplitTriggerAdapter<D>,
    timestamp_us: u32,
    trace: &mut FixedCommonObservabilityRecordTrace<R>,
) -> Result<
    (),
    ApplyAndRecordError<
        S::Error,
        C::Error,
        <ScheduledActionExecutor<Q> as ActionExecutor>::Error,
        W::Error,
        T::Error,
        P::Error,
    >,
>
where
    S: CaptureSampleSource + SplitLiveEventSink,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    D: EcuClock,
{
    let event = trigger.on_trigger_edge(timestamp_us);
    let live_event = trigger.live_trigger_event(timestamp_us);
    adapter.sensor().apply_live_event(live_event);
    adapter.apply_event_and_record(event, trace).map(|_| ())
}

pub type TriggerAdapterError<S, C, A, W, T, P> = BoardAdapterError<S, C, A, W, T, P>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TriggerAndPushPairError<S, C, A, W, T, P> {
    Trigger(TriggerAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

#[allow(clippy::type_complexity)]
pub fn apply_trigger_timestamp_to_runtime_adapter_and_push_pair<
    S,
    C,
    W,
    T,
    P,
    D,
    const N: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<N>, W, T, P>,
    trigger: &mut SplitTriggerAdapter<D>,
    timestamp_us: u32,
    sample_trace: &mut FixedCommonObservabilityTrace<SM>,
    record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
) -> Result<
    (),
    TriggerAndPushPairError<
        S::Error,
        C::Error,
        <ScheduledActionExecutor<N> as ActionExecutor>::Error,
        W::Error,
        T::Error,
        P::Error,
    >,
>
where
    S: CaptureSampleSource + SplitLiveEventSink,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    D: EcuClock,
{
    let event = trigger.on_trigger_edge(timestamp_us);
    let live_event = trigger.live_trigger_event(timestamp_us);
    adapter.sensor().apply_live_event(live_event);
    adapter
        .apply_event_and_push_pair(event, sample_trace, record_trace)
        .map(|_| ())
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => TriggerAndPushPairError::Trigger(err),
            ApplyAndPushPairError::Record(err) => TriggerAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => TriggerAndPushPairError::Sample(err),
        })
}

#[allow(clippy::type_complexity)]
pub fn apply_trigger_timestamp_to_runtime_adapter_and_push_to_trace_pair<
    S,
    C,
    W,
    T,
    P,
    D,
    const N: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<N>, W, T, P>,
    trigger: &mut SplitTriggerAdapter<D>,
    timestamp_us: u32,
    traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
) -> Result<
    (),
    TriggerAndPushPairError<
        S::Error,
        C::Error,
        <ScheduledActionExecutor<N> as ActionExecutor>::Error,
        W::Error,
        T::Error,
        P::Error,
    >,
>
where
    S: CaptureSampleSource + SplitLiveEventSink,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    D: EcuClock,
{
    let event = trigger.on_trigger_edge(timestamp_us);
    let live_event = trigger.live_trigger_event(timestamp_us);
    adapter.sensor().apply_live_event(live_event);
    adapter
        .apply_event_and_push_to_trace_pair(event, traces)
        .map(|_| ())
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => TriggerAndPushPairError::Trigger(err),
            ApplyAndPushPairError::Record(err) => TriggerAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => TriggerAndPushPairError::Sample(err),
        })
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamObservationAndRecordError<S, C, A, W, T, P> {
    Apply(BoardAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
}

#[allow(clippy::type_complexity)]
pub fn apply_cam_observation_to_runtime_adapter_and_record<
    S,
    C,
    W,
    T,
    P,
    const Q: usize,
    const R: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    at_us: u32,
    cam_seen: bool,
    trace: &mut FixedCommonObservabilityRecordTrace<R>,
) -> Result<
    (),
    CamObservationAndRecordError<
        S::Error,
        C::Error,
        <ScheduledActionExecutor<Q> as ActionExecutor>::Error,
        W::Error,
        T::Error,
        P::Error,
    >,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    adapter
        .apply_event_and_record(
            BoardEvent::CamEdge {
                at_us: Micros::new(at_us),
                cam_seen,
            },
            trace,
        )
        .map_err(|err| match err {
            ApplyAndRecordError::Apply(err) => CamObservationAndRecordError::Apply(err),
            ApplyAndRecordError::Record(err) => CamObservationAndRecordError::Record(err),
        })?;
    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CamObservationAndPushPairError<S, C, A, W, T, P> {
    Apply(BoardAdapterError<S, C, A, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

#[allow(clippy::type_complexity)]
pub fn apply_cam_observation_to_runtime_adapter_and_push_pair<
    S,
    C,
    W,
    T,
    P,
    const Q: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    at_us: u32,
    cam_seen: bool,
    sample_trace: &mut FixedCommonObservabilityTrace<SM>,
    record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
) -> Result<
    (),
    CamObservationAndPushPairError<
        S::Error,
        C::Error,
        <ScheduledActionExecutor<Q> as ActionExecutor>::Error,
        W::Error,
        T::Error,
        P::Error,
    >,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    adapter
        .apply_event_and_push_pair(
            BoardEvent::CamEdge {
                at_us: Micros::new(at_us),
                cam_seen,
            },
            sample_trace,
            record_trace,
        )
        .map(|_| ())
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => CamObservationAndPushPairError::Apply(err),
            ApplyAndPushPairError::Record(err) => CamObservationAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => CamObservationAndPushPairError::Sample(err),
        })
}

#[allow(clippy::type_complexity)]
pub fn apply_cam_observation_to_runtime_adapter_and_push_to_trace_pair<
    S,
    C,
    W,
    T,
    P,
    const Q: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    at_us: u32,
    cam_seen: bool,
    traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
) -> Result<
    (),
    CamObservationAndPushPairError<
        S::Error,
        C::Error,
        <ScheduledActionExecutor<Q> as ActionExecutor>::Error,
        W::Error,
        T::Error,
        P::Error,
    >,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
{
    adapter
        .apply_event_and_push_to_trace_pair(
            BoardEvent::CamEdge {
                at_us: Micros::new(at_us),
                cam_seen,
            },
            traces,
        )
        .map(|_| ())
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => CamObservationAndPushPairError::Apply(err),
            ApplyAndPushPairError::Record(err) => CamObservationAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => CamObservationAndPushPairError::Sample(err),
        })
}

#[cfg(test)]
mod tests;
