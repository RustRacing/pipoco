#![cfg_attr(not(test), no_std)]

pub mod edge;
pub mod output;
pub mod sensor;
pub mod trace;

// Re-export types for downstream use
pub use edge::{EdgeLine, EdgePolarity, EdgeSample, EdgeSource};
pub use output::{OutputLevel, OutputTransition, OutputTransitionKind, OutputTransitionSink};
pub use sensor::{SensorFrame, SensorFrameSource};
pub use trace::{TraceInputKind, TracePayload, TraceRecord};

use ecu_calibration::PersistedCalibrationBlob;
use ecu_domain::{Degrees10, Kpa10, Micros, Rpm};
use ecu_runtime::{Action, ActionBatch, RuntimeSnapshot};

/// Raw capture sample emitted by board capture hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct CaptureSample {
    pub at_us: Micros,
    pub rpm: Rpm,
    pub load_kpa10: Kpa10,
    pub angle_x10: Degrees10,
}

/// Narrow board-facing sensor source.
pub trait SensorSource {
    type Error;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error>;
}

/// Board-facing capture sink for diagnostics and logs.
pub trait CaptureSink {
    type Error;

    fn capture(&mut self, sample: CaptureSample) -> Result<(), Self::Error>;
}

/// Board-facing executor for runtime-emitted actions.
pub trait ActionExecutor {
    type Error;

    fn execute(&mut self, action: Action) -> Result<(), Self::Error>;

    fn execute_batch<const N: usize>(&mut self, batch: ActionBatch<N>) -> Result<(), Self::Error> {
        for action in batch.iter() {
            self.execute(action)?;
        }
        Ok(())
    }
}

/// Narrow watchdog feed surface.
pub trait Watchdog {
    type Error;

    fn feed(&mut self) -> Result<(), Self::Error>;
}

/// Transport-facing publisher for runtime snapshots and persistence events.
pub trait TransportPublisher {
    type Error;

    fn publish_snapshot(&mut self, snapshot: &RuntimeSnapshot) -> Result<(), Self::Error>;

    fn publish_calibration(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error>;
}

/// Persistence surface for canonical calibration blobs.
pub trait CalibrationStore {
    type Error;

    fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error>;

    fn save(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error>;
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::Lambda100;
    use ecu_runtime::{
        ControlInputs, EngineRuntime, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
        StepInputs, TorqueInputs,
    };

    struct MockExecutor {
        count: usize,
    }

    impl ActionExecutor for MockExecutor {
        type Error = ();

        fn execute(&mut self, _action: Action) -> Result<(), Self::Error> {
            self.count += 1;
            Ok(())
        }
    }

    struct MockStore(Option<PersistedCalibrationBlob>);

    impl CalibrationStore for MockStore {
        type Error = ();

        fn load(&mut self) -> Result<Option<PersistedCalibrationBlob>, Self::Error> {
            Ok(self.0)
        }

        fn save(&mut self, blob: &PersistedCalibrationBlob) -> Result<(), Self::Error> {
            self.0 = Some(*blob);
            Ok(())
        }
    }

    #[test]
    fn executor_can_consume_action_batches() {
        let mut runtime = EngineRuntime::new();
        let result = runtime.step(
            StepInputs {
                now_us: Micros::new(1_000),
                rpm: 1_800,
                load_kpa10: 600,
                angle_x10: 100,
                trigger_synced: true,
                cam_seen: true,
                flat_shift_armed: false,
                launch_armed: false,
            },
            ControlInputs {
                enrichment: EnrichmentInputs {
                    now_us: Micros::new(1_000),
                    clt_c: 70,
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
                ignition: IgnitionInputs::new(Degrees10::new(100), 0, 0, 0, false, Rpm::new(1800)),
            },
        );

        let mut executor = MockExecutor { count: 0 };
        executor.execute_batch(result.actions).unwrap();
        assert!(executor.count > 0);
    }

    #[test]
    fn calibration_store_round_trips_persisted_blob() {
        let runtime = EngineRuntime::new();
        let blob = PersistedCalibrationBlob::new(runtime.calibration_snapshot());

        let mut store = MockStore(None);
        store.save(&blob).unwrap();

        assert_eq!(store.load().unwrap(), Some(blob));
    }
}
