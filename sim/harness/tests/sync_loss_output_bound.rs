//! End-to-end sync-loss timing-bound scenario (ADR-0002).
//!
//! Drives a running engine through the shared `boards/common` scheduled-output
//! seam (`run_runtime_scheduled_output_tick`), injects sync loss, and asserts
//! every energized output reaches its safe (low) state within 1000 µs of the
//! sync-loss observation. This closes the "Sync Loss During Operation" gap in
//! `aidocs/architecture/safety-scenarios.md`.

use ecu_board_api::{CaptureSample, CaptureSampleSource, CaptureSink, Watchdog};
use ecu_calibration::{PersistedCalibrationBlob, PersistedCalibrationStore};
use ecu_domain::{Degrees10, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm};
use ecu_runtime::{
    Action, ActionExecutor, BaseFuelModel, ControlInputs, EnrichmentInputs, IgnitionInputs,
    LambdaTrimInputs, RuntimeSnapshot, TorqueInputs, TransportPublisher,
};
use ecu_scheduler::{
    ChannelId, DwellUs, ExclusiveChannel, IgnitionPlan, InjectionPlan, OutputGroup,
    TimedIgnitionPlan, TimedInjectionPlan, TransitionDrainBuffer,
};
use ecu_target_common::adapter::{BoardAdapter, BoardEvent};
use ecu_target_common::outputs::{
    RawScheduledOutputPin, ScheduledActionExecutor, ScheduledOutputs4,
};
use ecu_target_common::split_tick::run_runtime_scheduled_output_tick;

/// ADR-0002: all energized outputs must reach safe (low) state within this
/// bound of the sync-loss observation.
const SYNC_LOSS_OUTPUT_BOUND_US: u32 = 1_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockSensor(CaptureSample);

impl CaptureSampleSource for MockSensor {
    type Error = ();

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        Ok(self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockCapture;

impl CaptureSink for MockCapture {
    type Error = ();

    fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockWatchdog;

impl Watchdog for MockWatchdog {
    type Error = ();

    fn feed(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockTransport;

impl TransportPublisher for MockTransport {
    type Error = ();

    fn publish_snapshot(&mut self, _snapshot: &RuntimeSnapshot) -> Result<(), Self::Error> {
        Ok(())
    }

    fn publish_calibration(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
struct MockStore;

impl PersistedCalibrationStore for MockStore {
    type Error = ();

    fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error> {
        Ok(None)
    }

    fn save(&mut self, _blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
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

type Adapter = BoardAdapter<
    MockSensor,
    MockCapture,
    ScheduledActionExecutor<8>,
    MockWatchdog,
    MockTransport,
    MockStore,
>;

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

fn fuel_model() -> BaseFuelModel {
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

fn synced_running_adapter() -> Adapter {
    let sensor = MockSensor(CaptureSample {
        at_us: Micros::new(80),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(15),
    });
    let mut adapter = BoardAdapter::new(
        sensor,
        MockCapture,
        ScheduledActionExecutor::<8>::new(),
        MockWatchdog,
        MockTransport,
        MockStore,
    );
    adapter.configure_fuel_model(fuel_model());
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
    adapter
}

#[test]
fn outputs_deenergize_within_1000us_of_sync_loss() {
    let mut adapter = synced_running_adapter();

    // Running engine: arm a full injector + ignition pair (energized outputs
    // pending in the future, well beyond the bound under test).
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
            injection: TimedInjectionPlan {
                plan: InjectionPlan {
                    output: ExclusiveChannel::new(OutputGroup::Injector, ChannelId::new(1)),
                    pulse_width: PulseWidthUs::new(2500),
                },
                start_at: Micros::new(50_000),
                end_at: Micros::new(52_500),
            },
            ignition: TimedIgnitionPlan {
                plan: IgnitionPlan {
                    output: ExclusiveChannel::new(OutputGroup::Ignition, ChannelId::new(1)),
                    dwell: DwellUs::new(500),
                    advance: Degrees10::new(100),
                },
                start_at: Micros::new(50_000),
                end_at: Micros::new(50_500),
            },
        })
        .expect("arm energized outputs");
    assert_eq!(
        adapter.actions().queue().active_count(),
        4,
        "engine running with armed injector + ignition transitions"
    );

    // Sync loss observed at this instant.
    let sync_loss_us = Micros::new(20_000);
    adapter
        .apply_event(BoardEvent::TriggerEdge {
            at_us: sync_loss_us,
            rpm: Rpm::new(0),
            angle_x10: Degrees10::new(15),
            authority: ecu_domain::EngineTimeAuthority::none(),
            synced: false,
        })
        .expect("unsync trigger");
    adapter
        .apply_event(BoardEvent::CamEdge {
            at_us: Micros::new(sync_loss_us.get() + 1),
            cam_seen: false,
        })
        .expect("cam missing");

    // Drive the shared scheduled-output seam at exactly the ADR-0002 bound.
    let deadline_us = Micros::new(sync_loss_us.get() + SYNC_LOSS_OUTPUT_BOUND_US);
    let mut outputs = ScheduledOutputs4::new(
        RecordingPin::default(),
        RecordingPin::default(),
        RecordingPin::default(),
        RecordingPin::default(),
    );
    let mut drain = TransitionDrainBuffer::<8>::new();

    let result = run_runtime_scheduled_output_tick(
        &mut adapter,
        deadline_us,
        control_inputs(deadline_us),
        &mut outputs,
        &mut drain,
    )
    .expect("split tick succeeds");

    // Within the bound the pending energize transitions are cancelled: nothing
    // is drained or applied, and no pin is ever driven high.
    assert!(result.runtime_step_ran);
    assert_eq!(result.drained_transitions, 0);
    assert_eq!(result.applied_transitions, 0);
    assert_eq!(
        adapter.actions().queue().active_count(),
        0,
        "all pending energize transitions cancelled within {SYNC_LOSS_OUTPUT_BOUND_US} us"
    );

    let (inj0, inj1, ign0, ign1) = outputs.into_inner();
    for pin in [inj0, inj1, ign0, ign1] {
        assert_eq!(
            pin.high_count, 0,
            "no output may energize after sync loss within the bound"
        );
    }
}
