use crate::adapter::{BoardAdapter, BoardAdapterError, BoardEvent};
use crate::outputs::{RawScheduledOutputBank, ScheduledActionExecutor, TransitionApplyError};
use ecu_board_api::{CaptureSampleSource, CaptureSink, Watchdog};
use ecu_calibration::PersistedCalibrationStore;
use ecu_domain::Micros;
use ecu_runtime::{ControlInputs, TransportPublisher};
use ecu_scheduler::{ScheduleError, TransitionDrainBuffer};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplitScheduledTickResult {
    pub runtime_step_ran: bool,
    pub drained_transitions: usize,
    pub applied_transitions: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SplitScheduledTickError<S, C, W, T, P> {
    Adapter(BoardAdapterError<S, C, ScheduleError, W, T, P>),
    Apply(TransitionApplyError),
}

type SplitTickResult<S, C, W, T, P> =
    Result<SplitScheduledTickResult, SplitScheduledTickError<S, C, W, T, P>>;

#[allow(clippy::type_complexity)]
/// Runs the `boards/common` scheduled-output seam for a runtime adapter tick.
///
/// This helper converts a board tick into a runtime adapter call, then drains
/// runtime-scheduled output transitions into raw board output pins here instead
/// of pushing raw capture/runtime traits into `ecu-board-api`.
pub fn run_runtime_scheduled_output_tick<S, C, W, T, P, O, const Q: usize, const D: usize>(
    adapter: &mut BoardAdapter<S, C, ScheduledActionExecutor<Q>, W, T, P>,
    now_us: Micros,
    control: ControlInputs,
    outputs: &mut O,
    drain: &mut TransitionDrainBuffer<D>,
) -> SplitTickResult<S::Error, C::Error, W::Error, T::Error, P::Error>
where
    S: CaptureSampleSource,
    C: CaptureSink,
    W: Watchdog,
    T: TransportPublisher,
    P: PersistedCalibrationStore,
    O: RawScheduledOutputBank,
{
    let step = adapter
        .apply_event(BoardEvent::Tick { now_us, control })
        .map_err(SplitScheduledTickError::Adapter)?;
    let applied = outputs
        .drain_and_apply_due(adapter.actions(), now_us, drain)
        .map_err(SplitScheduledTickError::Apply)?;

    Ok(SplitScheduledTickResult {
        runtime_step_ran: step.is_some(),
        drained_transitions: drain.len as usize,
        applied_transitions: applied,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::adapter::BoardEvent;
    use crate::outputs::{RawScheduledOutputPin, ScheduledOutputs4};
    use ecu_board_api::CaptureSample;
    use ecu_calibration::PersistedCalibrationBlob;
    use ecu_domain::{Degrees10, Kpa10, Lambda100, PulseWidthUs, Rpm};
    use ecu_runtime::{
        Action, ActionExecutor, BaseFuelModel, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
        TorqueInputs,
    };
    use ecu_scheduler::ScheduledTransitionQueue;

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct MockSensor(CaptureSample);

    impl CaptureSampleSource for MockSensor {
        type Error = ();

        fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
            Ok(self.0)
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct MockCapture {
        count: usize,
    }

    impl CaptureSink for MockCapture {
        type Error = ();

        fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
            self.count = self.count.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct MockWatchdog {
        count: usize,
    }

    impl Watchdog for MockWatchdog {
        type Error = ();

        fn feed(&mut self) -> Result<(), Self::Error> {
            self.count = self.count.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct MockTransport {
        count: usize,
    }

    impl TransportPublisher for MockTransport {
        type Error = ();

        fn publish_snapshot(
            &mut self,
            _snapshot: &ecu_runtime::RuntimeSnapshot,
        ) -> Result<(), Self::Error> {
            self.count = self.count.saturating_add(1);
            Ok(())
        }

        fn publish_calibration(
            &mut self,
            _blob: &PersistedCalibrationBlob,
        ) -> Result<(), Self::Error> {
            self.count = self.count.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct MockStore {
        saved: usize,
    }

    impl PersistedCalibrationStore for MockStore {
        type Error = ();

        fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error> {
            Ok(None)
        }

        fn save(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
            self.saved = self.saved.saturating_add(1);
            Ok(())
        }
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
    struct RecordingPin {
        high_count: u8,
        low_count: u8,
    }

    impl RawScheduledOutputPin for RecordingPin {
        fn set_scheduled_high(&mut self) {
            self.high_count = self.high_count.saturating_add(1);
        }

        fn set_scheduled_low(&mut self) {
            self.low_count = self.low_count.saturating_add(1);
        }
    }

    fn test_fuel_model() -> BaseFuelModel {
        let rpm_bins = [
            Rpm::new(500),
            Rpm::new(1000),
            Rpm::new(1500),
            Rpm::new(2000),
            Rpm::new(2500),
            Rpm::new(3000),
            Rpm::new(3500),
            Rpm::new(4000),
            Rpm::new(4500),
            Rpm::new(5000),
            Rpm::new(5500),
            Rpm::new(6000),
            Rpm::new(6500),
            Rpm::new(7000),
            Rpm::new(7500),
            Rpm::new(8000),
        ];
        let load_bins = [
            Kpa10::new(200),
            Kpa10::new(300),
            Kpa10::new(400),
            Kpa10::new(500),
            Kpa10::new(600),
            Kpa10::new(700),
            Kpa10::new(800),
            Kpa10::new(900),
            Kpa10::new(1000),
            Kpa10::new(1100),
            Kpa10::new(1200),
            Kpa10::new(1300),
            Kpa10::new(1400),
            Kpa10::new(1500),
            Kpa10::new(1600),
            Kpa10::new(1700),
        ];
        let mut pulse_widths = [[PulseWidthUs::new(0); 16]; 16];
        pulse_widths[0][0] = PulseWidthUs::new(1000);
        pulse_widths[5][5] = PulseWidthUs::new(2500);
        pulse_widths[15][15] = PulseWidthUs::new(4000);

        BaseFuelModel::new(rpm_bins, load_bins, pulse_widths)
    }

    fn control_inputs(now_us: Micros) -> ControlInputs {
        ControlInputs {
            enrichment: EnrichmentInputs {
                now_us,
                clt_c: 80,
                cranking: false,
                just_started: false,
                tpsdot_pct_s: 0,
                mapdot_kpa_s: 0,
            },
            lambda: LambdaTrimInputs {
                clt_c: 80,
                lambda_valid: true,
                measured_lambda100: Lambda100::new(100),
                requested_open_loop: false,
            },
            torque: TorqueInputs::new(90, 90, 90, 90, 90),
            ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(3000)),
        }
    }

    type TestAdapter = BoardAdapter<
        MockSensor,
        MockCapture,
        ScheduledActionExecutor<8>,
        MockWatchdog,
        MockTransport,
        MockStore,
    >;

    fn adapter() -> TestAdapter {
        let sensor = MockSensor(CaptureSample {
            at_us: Micros::new(80),
            rpm: Rpm::new(3000),
            load_kpa10: Kpa10::new(700),
            angle_x10: Degrees10::new(15),
        });
        let mut adapter = BoardAdapter::new(
            sensor,
            MockCapture::default(),
            ScheduledActionExecutor::<8>::new(),
            MockWatchdog::default(),
            MockTransport::default(),
            MockStore::default(),
        );
        adapter.configure_fuel_model(test_fuel_model());
        adapter
    }

    fn sync_adapter(adapter: &mut TestAdapter) {
        adapter.poll_sensor().expect("sensor sample");
        adapter
            .apply_event(BoardEvent::TriggerEdge {
                at_us: Micros::new(86),
                rpm: Rpm::new(3000),
                angle_x10: Degrees10::new(15),
                authority: ecu_domain::EngineTimeAuthority::new(
                    ecu_domain::CrankSyncState::PrimaryLocked,
                    ecu_domain::PhaseSyncState::CrankOnly360,
                    ecu_domain::AbsoluteTimeAuthority::None,
                    500,
                    0,
                ),
                synced: true,
            })
            .expect("trigger edge");
        adapter
            .apply_event(BoardEvent::CamEdge {
                at_us: Micros::new(87),
                cam_seen: true,
            })
            .expect("cam edge");
    }

    #[test]
    fn run_runtime_scheduled_output_tick_queues_drains_and_applies_outputs() {
        let mut adapter = adapter();
        sync_adapter(&mut adapter);

        let inj0 = RecordingPin::default();
        let inj1 = RecordingPin::default();
        let ign0 = RecordingPin::default();
        let ign1 = RecordingPin::default();
        let mut outputs = ScheduledOutputs4::new(inj0, inj1, ign0, ign1);
        let mut drain = TransitionDrainBuffer::<8>::new();

        let result = run_runtime_scheduled_output_tick(
            &mut adapter,
            Micros::new(10_000),
            control_inputs(Micros::new(10_000)),
            &mut outputs,
            &mut drain,
        )
        .expect("split tick succeeds");

        assert!(result.runtime_step_ran);
        assert_eq!(result.drained_transitions, 2);
        assert_eq!(result.applied_transitions, 2);

        let (inj0, inj1, ign0, ign1) = outputs.into_inner();
        assert_eq!(inj0.high_count, 0);
        assert_eq!(inj0.low_count, 0);
        assert_eq!(inj1.high_count, 1);
        assert_eq!(inj1.low_count, 0);
        assert_eq!(ign0.high_count, 0);
        assert_eq!(ign0.low_count, 0);
        assert_eq!(ign1.high_count, 1);
        assert_eq!(ign1.low_count, 0);
    }

    #[test]
    fn run_runtime_scheduled_output_tick_returns_zero_counts_when_unsynced() {
        let mut adapter = adapter();
        let mut outputs = ScheduledOutputs4::new(
            RecordingPin::default(),
            RecordingPin::default(),
            RecordingPin::default(),
            RecordingPin::default(),
        );
        let mut drain = TransitionDrainBuffer::<8>::new();

        let result = run_runtime_scheduled_output_tick(
            &mut adapter,
            Micros::new(10_000),
            control_inputs(Micros::new(10_000)),
            &mut outputs,
            &mut drain,
        )
        .expect("split tick succeeds");

        assert!(result.runtime_step_ran);
        assert_eq!(result.drained_transitions, 0);
        assert_eq!(result.applied_transitions, 0);
        assert_eq!(adapter.actions().queue().active_count(), 0);
    }

    #[test]
    fn run_runtime_scheduled_output_tick_clears_queue_after_sync_loss() {
        let mut adapter = adapter();
        sync_adapter(&mut adapter);
        adapter
            .apply_event(BoardEvent::Tick {
                now_us: Micros::new(9_000),
                control: control_inputs(Micros::new(9_000)),
            })
            .expect("runtime arms scheduler");
        adapter.actions().queue_mut().cancel_all();
        adapter
            .actions()
            .execute(Action::ArmScheduler {
                injection: ecu_scheduler::TimedInjectionPlan {
                    plan: ecu_scheduler::InjectionPlan {
                        output: ecu_scheduler::ExclusiveChannel::new(
                            ecu_scheduler::OutputGroup::Injector,
                            ecu_domain::ChannelId::new(1),
                        ),
                        pulse_width: PulseWidthUs::new(2500),
                    },
                    start_at: Micros::new(20_000),
                    end_at: Micros::new(22_500),
                },
                ignition: ecu_scheduler::TimedIgnitionPlan {
                    plan: ecu_scheduler::IgnitionPlan {
                        output: ecu_scheduler::ExclusiveChannel::new(
                            ecu_scheduler::OutputGroup::Ignition,
                            ecu_domain::ChannelId::new(1),
                        ),
                        dwell: ecu_scheduler::DwellUs::new(500),
                        advance: Degrees10::new(100),
                    },
                    start_at: Micros::new(20_000),
                    end_at: Micros::new(20_500),
                },
            })
            .expect("manual queue arm");
        assert_eq!(adapter.actions().queue().active_count(), 4);

        adapter
            .apply_event(BoardEvent::TriggerEdge {
                at_us: Micros::new(11_000),
                rpm: Rpm::new(0),
                angle_x10: Degrees10::new(15),
                authority: ecu_domain::EngineTimeAuthority::none(),
                synced: false,
            })
            .expect("unsync trigger");
        adapter
            .apply_event(BoardEvent::CamEdge {
                at_us: Micros::new(11_001),
                cam_seen: false,
            })
            .expect("cam missing");

        let mut outputs = ScheduledOutputs4::new(
            RecordingPin::default(),
            RecordingPin::default(),
            RecordingPin::default(),
            RecordingPin::default(),
        );
        let mut drain = TransitionDrainBuffer::<8>::new();

        let result = run_runtime_scheduled_output_tick(
            &mut adapter,
            Micros::new(11_010),
            control_inputs(Micros::new(11_010)),
            &mut outputs,
            &mut drain,
        )
        .expect("split tick succeeds");

        assert!(result.runtime_step_ran);
        assert_eq!(result.drained_transitions, 0);
        assert_eq!(result.applied_transitions, 0);
        assert_eq!(adapter.actions().queue().active_count(), 0);
    }

    #[test]
    fn run_runtime_scheduled_output_tick_reports_apply_error_without_partial_pin_writes() {
        let mut adapter = adapter();
        let mut invalid_queue = ScheduledTransitionQueue::<8>::new();
        invalid_queue
            .enqueue_transition(ecu_scheduler::ScheduledTransition {
                at_us: Micros::new(10_000),
                kind: ecu_scheduler::ScheduledTransitionKind::Injector,
                channel: ecu_domain::ChannelId::new(0),
                level: ecu_scheduler::ScheduledLevel::High,
            })
            .expect("valid transition fits");
        invalid_queue
            .enqueue_transition(ecu_scheduler::ScheduledTransition {
                at_us: Micros::new(10_000),
                kind: ecu_scheduler::ScheduledTransitionKind::Ignition,
                channel: ecu_domain::ChannelId::new(3),
                level: ecu_scheduler::ScheduledLevel::High,
            })
            .expect("invalid-channel transition fits queue");
        *adapter.actions().queue_mut() = invalid_queue;

        let inj0 = RecordingPin::default();
        let inj1 = RecordingPin::default();
        let ign0 = RecordingPin::default();
        let ign1 = RecordingPin::default();
        let mut outputs = ScheduledOutputs4::new(inj0, inj1, ign0, ign1);
        let mut drain = TransitionDrainBuffer::<8>::new();

        let result = run_runtime_scheduled_output_tick(
            &mut adapter,
            Micros::new(10_000),
            control_inputs(Micros::new(10_000)),
            &mut outputs,
            &mut drain,
        );

        assert!(matches!(
            result,
            Err(SplitScheduledTickError::Apply(
                TransitionApplyError::IgnitionChannelOutOfRange(_)
            ))
        ));
        let (inj0, inj1, ign0, ign1) = outputs.into_inner();
        assert_eq!(inj0.high_count, 0);
        assert_eq!(inj0.low_count, 0);
        assert_eq!(inj1.high_count, 0);
        assert_eq!(inj1.low_count, 0);
        assert_eq!(ign0.high_count, 0);
        assert_eq!(ign0.low_count, 0);
        assert_eq!(ign1.high_count, 0);
        assert_eq!(ign1.low_count, 0);
    }
}
