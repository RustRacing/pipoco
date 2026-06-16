use crate::adapter::{
    ApplyAndPushPairError, ApplyAndRecordError, BoardAdapter, BoardAdapterError, BoardEvent,
    CommonObservabilityRecordTraceOverflow, CommonObservabilityTraceOverflow,
    FixedCommonObservabilityRecordTrace, FixedCommonObservabilityTrace,
    FixedCommonObservabilityTracePair,
};
use crate::live_inputs::{SplitLiveInputs, SplitLiveTriggerEvent};
use crate::outputs::ScheduledActionExecutor;
use ecu_board_api::{
    BoardSensorSnapshot, BoardSensorSnapshotCapture, BoardSensorSnapshotCaptureSource,
    BoardSensorValidityFlags, CaptureSample, CaptureSampleSource, CaptureSink, EcuClock, Watchdog,
};
use ecu_calibration::PersistedCalibrationStore;
use ecu_domain::{Kpa10, Micros};
use ecu_io::{SensorFrame, SensorFrameSource};
use ecu_runtime::{ActionExecutor, TransportPublisher};
use ecu_scheduler::ScheduleError;

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
    fn apply_live_event(&mut self, event: SplitLiveTriggerEvent);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardSensorSnapshotSampleError<E> {
    Source(E),
    NoFrame,
    MafLoadInvalid,
    RuntimeLoadSourceUnsupported(BoardSensorRuntimeLoad),
}

pub type SensorFrameSampleError<E> = BoardSensorSnapshotSampleError<E>;

/// Runtime load projection for logical board sensor snapshots.
///
/// The current runtime accepts `Kpa10`, so MAP/speed-density is the only
/// directly supported projection. Other load sources are explicit blockers
/// rather than implicit unit conversions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardSensorRuntimeLoad {
    MapSpeedDensity,
    MafFlow,
    TpsAlphaN,
}

pub type SensorFrameRuntimeLoad = BoardSensorRuntimeLoad;

/// Adapts logical board sensor snapshots to the runtime's current load sample input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BoardSensorSnapshotSampleSource<S> {
    source: S,
    runtime_load: BoardSensorRuntimeLoad,
    last_snapshot: Option<BoardSensorSnapshot>,
}

impl<S> BoardSensorSnapshotSampleSource<S> {
    pub const fn new(source: S) -> Self {
        Self {
            source,
            runtime_load: BoardSensorRuntimeLoad::MapSpeedDensity,
            last_snapshot: None,
        }
    }

    pub const fn with_runtime_load(source: S, runtime_load: BoardSensorRuntimeLoad) -> Self {
        Self {
            source,
            runtime_load,
            last_snapshot: None,
        }
    }

    pub fn source_mut(&mut self) -> &mut S {
        &mut self.source
    }

    pub const fn source(&self) -> &S {
        &self.source
    }

    pub const fn last_snapshot(&self) -> Option<BoardSensorSnapshot> {
        self.last_snapshot
    }

    pub const fn runtime_load(&self) -> BoardSensorRuntimeLoad {
        self.runtime_load
    }
}

fn project_runtime_load<E>(
    snapshot: BoardSensorSnapshot,
    runtime_load: BoardSensorRuntimeLoad,
) -> Result<Kpa10, BoardSensorSnapshotSampleError<E>> {
    match runtime_load {
        BoardSensorRuntimeLoad::MapSpeedDensity => Ok(snapshot.map_kpa10),
        BoardSensorRuntimeLoad::MafFlow => {
            if snapshot.validity.contains(BoardSensorValidityFlags::MAF) {
                Err(BoardSensorSnapshotSampleError::RuntimeLoadSourceUnsupported(runtime_load))
            } else {
                Err(BoardSensorSnapshotSampleError::MafLoadInvalid)
            }
        }
        BoardSensorRuntimeLoad::TpsAlphaN => {
            Err(BoardSensorSnapshotSampleError::RuntimeLoadSourceUnsupported(runtime_load))
        }
    }
}

pub(crate) fn capture_sample_from_snapshot<E>(
    capture: BoardSensorSnapshotCapture,
    runtime_load: BoardSensorRuntimeLoad,
) -> Result<CaptureSample, BoardSensorSnapshotSampleError<E>> {
    match runtime_load {
        BoardSensorRuntimeLoad::MapSpeedDensity => Ok(map_speed_density_capture_sample(capture)),
        _ => {
            let load_kpa10 = project_runtime_load(capture.snapshot, runtime_load)?;
            Ok(CaptureSample {
                at_us: capture.at_us,
                rpm: capture.snapshot.rpm,
                load_kpa10,
                angle_x10: capture.angle_x10,
            })
        }
    }
}

pub(crate) fn map_speed_density_capture_sample(
    capture: BoardSensorSnapshotCapture,
) -> CaptureSample {
    CaptureSample {
        at_us: capture.at_us,
        rpm: capture.snapshot.rpm,
        load_kpa10: capture.snapshot.map_kpa10,
        angle_x10: capture.angle_x10,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapshotCaptureAndRecordError<Src, S, C, W, T, P> {
    Source(Src),
    Adapter(BoardAdapterError<S, C, ScheduleError, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnapshotCaptureAndPushPairError<Src, S, C, W, T, P, const Q: usize> {
    Source(Src),
    Adapter(
        BoardAdapterError<S, C, <ScheduledActionExecutor<Q> as ActionExecutor>::Error, W, T, P>,
    ),
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorFrameAndRecordError<Src, S, C, W, T, P> {
    Source(Src),
    Adapter(BoardAdapterError<S, C, ScheduleError, W, T, P>),
    Record(CommonObservabilityRecordTraceOverflow),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorFrameAndPushPairError<Src, S, C, W, T, P, const Q: usize> {
    Source(Src),
    Adapter(
        BoardAdapterError<S, C, <ScheduledActionExecutor<Q> as ActionExecutor>::Error, W, T, P>,
    ),
    Record(CommonObservabilityRecordTraceOverflow),
    Sample(CommonObservabilityTraceOverflow),
}

#[allow(clippy::type_complexity)]
pub fn apply_snapshot_capture_to_runtime_adapter_and_record<
    S,
    C,
    W,
    T,
    P,
    Src,
    const Q: usize,
    const R: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    source: &mut Src,
    trace: &mut FixedCommonObservabilityRecordTrace<R>,
) -> Result<
    bool,
    SnapshotCaptureAndRecordError<Src::Error, S::Error, C::Error, W::Error, T::Error, P::Error>,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    Src: BoardSensorSnapshotCaptureSource,
{
    let Some(capture) = source
        .next_snapshot_capture()
        .map_err(SnapshotCaptureAndRecordError::Source)?
    else {
        return Ok(false);
    };

    adapter
        .apply_event_and_record(BoardEvent::SensorSnapshotCapture { capture }, trace)
        .map_err(|err| match err {
            ApplyAndRecordError::Apply(err) => SnapshotCaptureAndRecordError::Adapter(err),
            ApplyAndRecordError::Record(err) => SnapshotCaptureAndRecordError::Record(err),
        })?;
    Ok(true)
}

#[allow(clippy::type_complexity)]
pub fn apply_sensor_frame_to_runtime_adapter_and_push_pair<
    S,
    C,
    W,
    T,
    P,
    Src,
    const Q: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    source: &mut Src,
    sample_trace: &mut FixedCommonObservabilityTrace<SM>,
    record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
) -> Result<
    bool,
    SensorFrameAndPushPairError<Src::Error, S::Error, C::Error, W::Error, T::Error, P::Error, Q>,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    Src: SensorFrameSource,
{
    let Some(frame) = source
        .next_frame()
        .map_err(SensorFrameAndPushPairError::Source)?
    else {
        return Ok(false);
    };

    let capture = BoardSensorSnapshotCapture {
        at_us: frame.at_us,
        angle_x10: frame.angle_x10,
        snapshot: board_sensor_snapshot_from_frame(frame),
    };

    adapter
        .apply_event_and_push_pair(
            BoardEvent::SensorSnapshotCapture { capture },
            sample_trace,
            record_trace,
        )
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => SensorFrameAndPushPairError::Adapter(err),
            ApplyAndPushPairError::Record(err) => SensorFrameAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => SensorFrameAndPushPairError::Sample(err),
        })?;
    Ok(true)
}

#[allow(clippy::type_complexity)]
pub fn apply_sensor_frame_to_runtime_adapter_and_push_to_trace_pair<
    S,
    C,
    W,
    T,
    P,
    Src,
    const Q: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    source: &mut Src,
    traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
) -> Result<
    bool,
    SensorFrameAndPushPairError<Src::Error, S::Error, C::Error, W::Error, T::Error, P::Error, Q>,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    Src: SensorFrameSource,
{
    let Some(frame) = source
        .next_frame()
        .map_err(SensorFrameAndPushPairError::Source)?
    else {
        return Ok(false);
    };

    let capture = BoardSensorSnapshotCapture {
        at_us: frame.at_us,
        angle_x10: frame.angle_x10,
        snapshot: board_sensor_snapshot_from_frame(frame),
    };

    adapter
        .apply_event_and_push_to_trace_pair(BoardEvent::SensorSnapshotCapture { capture }, traces)
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => SensorFrameAndPushPairError::Adapter(err),
            ApplyAndPushPairError::Record(err) => SensorFrameAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => SensorFrameAndPushPairError::Sample(err),
        })?;
    Ok(true)
}

#[allow(clippy::type_complexity)]
pub fn apply_snapshot_capture_to_runtime_adapter_and_push_pair<
    S,
    C,
    W,
    T,
    P,
    Src,
    const Q: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    source: &mut Src,
    sample_trace: &mut FixedCommonObservabilityTrace<SM>,
    record_trace: &mut FixedCommonObservabilityRecordTrace<RM>,
) -> Result<
    bool,
    SnapshotCaptureAndPushPairError<
        Src::Error,
        S::Error,
        C::Error,
        W::Error,
        T::Error,
        P::Error,
        Q,
    >,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    Src: BoardSensorSnapshotCaptureSource,
{
    let Some(capture) = source
        .next_snapshot_capture()
        .map_err(SnapshotCaptureAndPushPairError::Source)?
    else {
        return Ok(false);
    };

    adapter
        .apply_event_and_push_pair(
            BoardEvent::SensorSnapshotCapture { capture },
            sample_trace,
            record_trace,
        )
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => SnapshotCaptureAndPushPairError::Adapter(err),
            ApplyAndPushPairError::Record(err) => SnapshotCaptureAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => SnapshotCaptureAndPushPairError::Sample(err),
        })?;
    Ok(true)
}

#[allow(clippy::type_complexity)]
pub fn apply_snapshot_capture_to_runtime_adapter_and_push_to_trace_pair<
    S,
    C,
    W,
    T,
    P,
    Src,
    const Q: usize,
    const SM: usize,
    const RM: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    source: &mut Src,
    traces: &mut FixedCommonObservabilityTracePair<SM, RM>,
) -> Result<
    bool,
    SnapshotCaptureAndPushPairError<
        Src::Error,
        S::Error,
        C::Error,
        W::Error,
        T::Error,
        P::Error,
        Q,
    >,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    Src: BoardSensorSnapshotCaptureSource,
{
    let Some(capture) = source
        .next_snapshot_capture()
        .map_err(SnapshotCaptureAndPushPairError::Source)?
    else {
        return Ok(false);
    };

    adapter
        .apply_event_and_push_to_trace_pair(BoardEvent::SensorSnapshotCapture { capture }, traces)
        .map_err(|err| match err {
            ApplyAndPushPairError::Apply(err) => SnapshotCaptureAndPushPairError::Adapter(err),
            ApplyAndPushPairError::Record(err) => SnapshotCaptureAndPushPairError::Record(err),
            ApplyAndPushPairError::Sample(err) => SnapshotCaptureAndPushPairError::Sample(err),
        })?;
    Ok(true)
}

#[allow(clippy::type_complexity)]
pub fn apply_sensor_frame_to_runtime_adapter_and_record<
    S,
    C,
    W,
    T,
    P,
    Src,
    const Q: usize,
    const R: usize,
>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    source: &mut Src,
    trace: &mut FixedCommonObservabilityRecordTrace<R>,
) -> Result<
    bool,
    SensorFrameAndRecordError<Src::Error, S::Error, C::Error, W::Error, T::Error, P::Error>,
>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    Src: SensorFrameSource,
{
    let Some(frame) = source
        .next_frame()
        .map_err(SensorFrameAndRecordError::Source)?
    else {
        return Ok(false);
    };

    let capture = BoardSensorSnapshotCapture {
        at_us: frame.at_us,
        angle_x10: frame.angle_x10,
        snapshot: board_sensor_snapshot_from_frame(frame),
    };

    adapter
        .apply_event_and_record(BoardEvent::SensorSnapshotCapture { capture }, trace)
        .map_err(|err| match err {
            ApplyAndRecordError::Apply(err) => SensorFrameAndRecordError::Adapter(err),
            ApplyAndRecordError::Record(err) => SensorFrameAndRecordError::Record(err),
        })?;
    Ok(true)
}

impl<S: BoardSensorSnapshotCaptureSource> CaptureSampleSource
    for BoardSensorSnapshotSampleSource<S>
{
    type Error = BoardSensorSnapshotSampleError<S::Error>;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        let capture = self
            .source
            .next_snapshot_capture()
            .map_err(BoardSensorSnapshotSampleError::Source)?
            .ok_or(BoardSensorSnapshotSampleError::NoFrame)?;
        let sample = capture_sample_from_snapshot(capture, self.runtime_load)?;
        self.last_snapshot = Some(capture.snapshot);
        Ok(sample)
    }
}

impl<S: SplitLiveEventSink> SplitLiveEventSink for BoardSensorSnapshotSampleSource<S> {
    fn apply_live_event(&mut self, event: SplitLiveTriggerEvent) {
        self.source.apply_live_event(event);
    }
}

pub fn board_sensor_snapshot_from_frame(frame: SensorFrame) -> BoardSensorSnapshot {
    BoardSensorSnapshot {
        rpm: frame.rpm,
        map_kpa10: frame.map_kpa10,
        tps_x100: frame.tps_x100,
        clt_c10: frame.clt_c10,
        iat_c10: frame.iat_c10,
        vbatt_mv: frame.vbatt_mv,
        baro_kpa10: frame.baro_kpa10,
        maf_x100: frame.maf_x100,
        knock_x100: frame.knock_x100,
        vehicle_speed_kph10: frame.vehicle_speed_kph10,
        cam_phase_deg10: frame.cam_phase_deg10,
        lambda_x100: frame.lambda_x100,
        validity: BoardSensorValidityFlags::from_channels(
            frame.maf_valid,
            frame.knock_valid,
            frame.vehicle_speed_valid,
            frame.lambda_valid,
        ),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct SensorFrameSnapshotSource<S> {
    source: S,
    last_frame: Option<SensorFrame>,
}

impl<S> SensorFrameSnapshotSource<S> {
    const fn new(source: S) -> Self {
        Self {
            source,
            last_frame: None,
        }
    }
}

impl<S: SensorFrameSource> BoardSensorSnapshotCaptureSource for SensorFrameSnapshotSource<S> {
    type Error = S::Error;

    fn next_snapshot_capture(&mut self) -> Result<Option<BoardSensorSnapshotCapture>, Self::Error> {
        let Some(frame) = self.source.next_frame()? else {
            return Ok(None);
        };
        self.last_frame = Some(frame);
        Ok(Some(BoardSensorSnapshotCapture {
            at_us: frame.at_us,
            angle_x10: frame.angle_x10,
            snapshot: board_sensor_snapshot_from_frame(frame),
        }))
    }
}

/// Compatibility adapter for raw sensor-frame sources.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorFrameSampleSource<S> {
    snapshot_source: BoardSensorSnapshotSampleSource<SensorFrameSnapshotSource<S>>,
}

impl<S> SensorFrameSampleSource<S> {
    pub const fn new(source: S) -> Self {
        Self {
            snapshot_source: BoardSensorSnapshotSampleSource::new(SensorFrameSnapshotSource::new(
                source,
            )),
        }
    }

    pub const fn with_runtime_load(source: S, runtime_load: BoardSensorRuntimeLoad) -> Self {
        Self {
            snapshot_source: BoardSensorSnapshotSampleSource::with_runtime_load(
                SensorFrameSnapshotSource::new(source),
                runtime_load,
            ),
        }
    }

    pub fn source_mut(&mut self) -> &mut S {
        &mut self.snapshot_source.source_mut().source
    }

    pub const fn last_frame(&self) -> Option<SensorFrame> {
        self.snapshot_source.source().last_frame
    }

    pub const fn last_snapshot(&self) -> Option<BoardSensorSnapshot> {
        self.snapshot_source.last_snapshot()
    }

    pub const fn runtime_load(&self) -> BoardSensorRuntimeLoad {
        self.snapshot_source.runtime_load()
    }
}

impl<S: SensorFrameSource> CaptureSampleSource for SensorFrameSampleSource<S> {
    type Error = BoardSensorSnapshotSampleError<S::Error>;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        self.snapshot_source.sample()
    }
}

/// Sensor source for boards that have a live load source but share trigger state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LiveLoadSensor<T: EcuClock, L> {
    time_source: T,
    load: L,
    live: SplitLiveInputs,
}

impl<T: EcuClock, L> LiveLoadSensor<T, L> {
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

impl<T: EcuClock, L> SplitLiveEventSink for LiveLoadSensor<T, L> {
    fn apply_live_event(&mut self, event: SplitLiveTriggerEvent) {
        self.live.apply_event(event);
    }
}

impl<T: EcuClock, L: LoadKpa10Source> CaptureSampleSource for LiveLoadSensor<T, L> {
    type Error = L::Error;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        let load_kpa10 = self.load.load_kpa10()?;
        Ok(split_capture_sample(
            self.live,
            SplitSensorSignals {
                at_us: self.time_source.now_us(),
                load_kpa10,
            },
        ))
    }
}

impl<T: EcuClock, L: LoadKpa10Source> BoardSensorSnapshotCaptureSource for LiveLoadSensor<T, L> {
    type Error = L::Error;

    fn next_snapshot_capture(&mut self) -> Result<Option<BoardSensorSnapshotCapture>, Self::Error> {
        let load_kpa10 = self.load.load_kpa10()?;
        Ok(Some(BoardSensorSnapshotCapture {
            at_us: self.time_source.now_us(),
            angle_x10: self.live.angle_x10(),
            snapshot: BoardSensorSnapshot {
                rpm: self.live.rpm(),
                map_kpa10: load_kpa10,
                ..BoardSensorSnapshot::default()
            },
        }))
    }
}

/// Bring-up sensor source for boards that do not yet have ADC plumbing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixedLoadSensor<T: EcuClock> {
    time_source: T,
    live: SplitLiveInputs,
    load_kpa10: Kpa10,
}

impl<T: EcuClock> FixedLoadSensor<T> {
    pub const fn new(time_source: T, load_kpa10: Kpa10) -> Self {
        Self {
            time_source,
            live: SplitLiveInputs::new(),
            load_kpa10,
        }
    }
}

impl<T: EcuClock> SplitLiveEventSink for FixedLoadSensor<T> {
    fn apply_live_event(&mut self, event: SplitLiveTriggerEvent) {
        self.live.apply_event(event);
    }
}

impl<T: EcuClock> CaptureSampleSource for FixedLoadSensor<T> {
    type Error = core::convert::Infallible;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        Ok(split_capture_sample(
            self.live,
            SplitSensorSignals {
                at_us: self.time_source.now_us(),
                load_kpa10: self.load_kpa10,
            },
        ))
    }
}

impl<T: EcuClock> BoardSensorSnapshotCaptureSource for FixedLoadSensor<T> {
    type Error = core::convert::Infallible;

    fn next_snapshot_capture(&mut self) -> Result<Option<BoardSensorSnapshotCapture>, Self::Error> {
        Ok(Some(BoardSensorSnapshotCapture {
            at_us: self.time_source.now_us(),
            angle_x10: self.live.angle_x10(),
            snapshot: BoardSensorSnapshot {
                rpm: self.live.rpm(),
                map_kpa10: self.load_kpa10,
                ..BoardSensorSnapshot::default()
            },
        }))
    }
}

#[cfg(test)]
mod tests;
