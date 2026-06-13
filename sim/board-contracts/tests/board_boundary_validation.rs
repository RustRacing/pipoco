use std::cell::Cell;
use std::rc::Rc;

use ecu_board_api::{
    BoardCapabilities, CaptureSample, CaptureSampleSource, CaptureSink, LoadSourceCapabilities,
    Watchdog,
};
use ecu_board_contracts::check_watchdog_policy;
use ecu_board_profiles::{
    BoardBuildMetadata, BoardSelection, FirmwareBuildInvocation, FirmwareBuildMode, FirmwareRecipe,
};
use ecu_calibration::PersistedCalibrationStore;
use ecu_domain::{Degrees10, EngineTimeAuthority, Kpa10, Lambda100, Micros, PulseWidthUs, Rpm};
use ecu_firmware_resolver::{resolve_recipe, SUPPORTED_INVOCATIONS};
use ecu_rp2040_pico::RP2040_PICO_TS_ECU_BUILD_METADATA;
use ecu_rp2350b::RP2350B_DEMO_BUILD_METADATA;
use ecu_runtime::{
    ActionExecutor, ControlInputs, EnrichmentInputs, IgnitionInputs, LambdaTrimInputs,
    TorqueInputs, TransportPublisher,
};
use ecu_target_common::adapter::{
    BoardAdapter as Rp2350Adapter, BoardAdapterError, BoardEvent as Rp2350Event,
};

type BoundaryAdapterError = BoardAdapterError<
    InjectedBoundaryError,
    InjectedBoundaryError,
    InjectedBoundaryError,
    InjectedBoundaryError,
    InjectedBoundaryError,
    InjectedBoundaryError,
>;

type BoundaryStepResult = Result<Option<ecu_runtime::StepResult>, BoundaryAdapterError>;

#[derive(Clone)]
struct Counters {
    action: Rc<Cell<usize>>,
    watchdog: Rc<Cell<usize>>,
    transport: Rc<Cell<usize>>,
    store: Rc<Cell<usize>>,
}

impl Counters {
    fn new() -> Self {
        Self {
            action: Rc::new(Cell::new(0)),
            watchdog: Rc::new(Cell::new(0)),
            transport: Rc::new(Cell::new(0)),
            store: Rc::new(Cell::new(0)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InjectedBoundaryError {
    Sensor,
    Capture,
    Action,
    Watchdog,
    Transport,
    Store,
}

#[derive(Clone)]
struct MockSensor {
    sample: CaptureSample,
    error: Option<InjectedBoundaryError>,
}

impl CaptureSampleSource for MockSensor {
    type Error = InjectedBoundaryError;

    fn sample(&mut self) -> Result<CaptureSample, Self::Error> {
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(self.sample)
    }
}

#[derive(Clone)]
struct MockCapture {
    error: Option<InjectedBoundaryError>,
}

impl CaptureSink for MockCapture {
    type Error = InjectedBoundaryError;

    fn capture(&mut self, _sample: CaptureSample) -> Result<(), Self::Error> {
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct MockActions {
    counters: Counters,
    error: Option<InjectedBoundaryError>,
}

impl ActionExecutor for MockActions {
    type Error = InjectedBoundaryError;

    fn execute(&mut self, _action: ecu_runtime::Action) -> Result<(), Self::Error> {
        self.counters.action.set(self.counters.action.get() + 1);
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct MockWatchdog {
    counters: Counters,
    error: Option<InjectedBoundaryError>,
}

impl Watchdog for MockWatchdog {
    type Error = InjectedBoundaryError;

    fn feed(&mut self) -> Result<(), Self::Error> {
        self.counters.watchdog.set(self.counters.watchdog.get() + 1);
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct MockTransport {
    counters: Counters,
    error: Option<InjectedBoundaryError>,
}

impl TransportPublisher for MockTransport {
    type Error = InjectedBoundaryError;

    fn publish_snapshot(
        &mut self,
        _snapshot: &ecu_runtime::RuntimeSnapshot,
    ) -> Result<(), Self::Error> {
        self.counters
            .transport
            .set(self.counters.transport.get() + 1);
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(())
    }

    fn publish_calibration(
        &mut self,
        _blob: &ecu_calibration::PersistedCalibrationBlob,
    ) -> Result<(), Self::Error> {
        self.counters
            .transport
            .set(self.counters.transport.get() + 1);
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct MockStore {
    counters: Counters,
    error: Option<InjectedBoundaryError>,
}

impl PersistedCalibrationStore for MockStore {
    type Error = InjectedBoundaryError;

    fn load(&mut self) -> Result<Option<ecu_calibration::PersistedCalibrationBlob>, Self::Error> {
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(None)
    }

    fn save(
        &mut self,
        _blob: &ecu_calibration::PersistedCalibrationBlob,
    ) -> Result<(), Self::Error> {
        self.counters.store.set(self.counters.store.get() + 1);
        if let Some(error) = self.error {
            return Err(error);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Default)]
struct MockFailures {
    sensor: Option<InjectedBoundaryError>,
    capture: Option<InjectedBoundaryError>,
    action: Option<InjectedBoundaryError>,
    watchdog: Option<InjectedBoundaryError>,
    transport: Option<InjectedBoundaryError>,
    store: Option<InjectedBoundaryError>,
}

fn mock_sensor(failures: MockFailures) -> MockSensor {
    MockSensor {
        sample: sample(),
        error: failures.sensor,
    }
}

fn mock_capture(failures: MockFailures) -> MockCapture {
    MockCapture {
        error: failures.capture,
    }
}

fn mock_actions(counters: Counters, failures: MockFailures) -> MockActions {
    MockActions {
        counters,
        error: failures.action,
    }
}

fn mock_watchdog(counters: Counters, failures: MockFailures) -> MockWatchdog {
    MockWatchdog {
        counters,
        error: failures.watchdog,
    }
}

fn mock_transport(counters: Counters, failures: MockFailures) -> MockTransport {
    MockTransport {
        counters,
        error: failures.transport,
    }
}

fn mock_store(counters: Counters, failures: MockFailures) -> MockStore {
    MockStore {
        counters,
        error: failures.store,
    }
}

fn control_inputs() -> ControlInputs {
    ControlInputs {
        enrichment: EnrichmentInputs {
            now_us: Micros::new(1_000),
            clt_c: 60,
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

fn sample() -> CaptureSample {
    CaptureSample {
        at_us: Micros::new(10),
        rpm: Rpm::new(3000),
        load_kpa10: Kpa10::new(700),
        angle_x10: Degrees10::new(12),
    }
}

fn locked_authority() -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        ecu_domain::CrankSyncState::PrimaryLocked,
        ecu_domain::PhaseSyncState::CrankOnly360,
        ecu_domain::AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    )
}

fn full_sequential_authority() -> EngineTimeAuthority {
    EngineTimeAuthority::new(
        ecu_domain::CrankSyncState::PrimaryLocked,
        ecu_domain::PhaseSyncState::CamValidated720,
        ecu_domain::AbsoluteTimeAuthority::GeometryOnly,
        EngineTimeAuthority::MAX_CONFIDENCE_X1000,
        0,
    )
}

#[test]
fn board_boundary_keeps_capabilities_logical_and_io_raw() {
    const FULL_ECU: BoardCapabilities =
        BoardCapabilities::new(true, true, true, 4, 4, 6, true, true, true)
            .with_load_sources(LoadSourceCapabilities::map());
    const REV_LIMITER: BoardCapabilities =
        BoardCapabilities::new(false, true, false, 1, 0, 0, false, false, true);

    assert!(FULL_ECU.supports_full_ecu());
    assert!(FULL_ECU.supports_load_sensor());
    assert!(REV_LIMITER.supports_rev_limiter());
    assert!(!REV_LIMITER.supports_full_ecu());
    assert!(!REV_LIMITER.supports_load_sensor());
}

#[test]
fn board_contract_resolves_real_metadata_into_firmware_build_invocations() {
    assert_build_invocation(
        RP2040_PICO_TS_ECU_BUILD_METADATA,
        {
            let mut recipe = FirmwareRecipe::ignition_only_wasted_spark(2)
                .for_board(BoardSelection::Named("rp2040-pico"));
            recipe.safety =
                ecu_board_profiles::SafetyPolicy::no_watchdog("bring-up only: no watchdog needed");
            recipe
        },
        ExpectedBuildInvocation {
            package: "ecu-rp2040-pico",
            binary: "ts-ecu",
            target_triple: "thumbv6m-none-eabi",
            features: &["capture-pio"],
        },
    );

    assert_build_invocation(
        RP2350B_DEMO_BUILD_METADATA,
        {
            let mut recipe =
                FirmwareRecipe::rev_limiter().for_board(BoardSelection::Named("rp2350b"));
            recipe.safety =
                ecu_board_profiles::SafetyPolicy::no_watchdog("bring-up only: no watchdog needed");
            recipe
        },
        ExpectedBuildInvocation {
            package: "ecu-rp2350b",
            binary: "ecu-rp2350b-min",
            target_triple: "thumbv8m.main-none-eabihf",
            features: &["example-bins"],
        },
    );
}

struct ExpectedBuildInvocation {
    package: &'static str,
    binary: &'static str,
    target_triple: &'static str,
    features: &'static [&'static str],
}

fn assert_build_invocation(
    metadata: BoardBuildMetadata,
    recipe: FirmwareRecipe,
    expected: ExpectedBuildInvocation,
) {
    assert!(!metadata.capabilities.watchdog);

    let invocation = recipe
        .resolve_build_plan(metadata)
        .expect("real board metadata should satisfy the contract recipe")
        .cargo_build_invocation()
        .expect("real board metadata should resolve to a cargo binary");

    assert_invocation(invocation, expected);
}

fn assert_invocation(invocation: FirmwareBuildInvocation, expected: ExpectedBuildInvocation) {
    assert_eq!(invocation.package, expected.package);
    assert_eq!(invocation.binary, expected.binary);
    assert_eq!(invocation.target_triple, expected.target_triple);
    assert_eq!(invocation.mode, FirmwareBuildMode::Release);
    assert_eq!(invocation.features.as_slice(), expected.features);
}

#[test]
fn adapter_boundary_smoke_accepts_same_event_sequence_for_independent_ports() {
    let counters_a = Counters::new();
    let counters_b = Counters::new();
    let no_failures = MockFailures::default();

    let mut rp2350 = Rp2350Adapter::new(
        mock_sensor(no_failures),
        mock_capture(no_failures),
        mock_actions(counters_a.clone(), no_failures),
        mock_watchdog(counters_a.clone(), no_failures),
        mock_transport(counters_a.clone(), no_failures),
        mock_store(counters_a.clone(), no_failures),
    );
    let mut second = Rp2350Adapter::new(
        mock_sensor(no_failures),
        mock_capture(no_failures),
        mock_actions(counters_b.clone(), no_failures),
        mock_watchdog(counters_b.clone(), no_failures),
        mock_transport(counters_b.clone(), no_failures),
        mock_store(counters_b.clone(), no_failures),
    );

    rp2350
        .apply_event(Rp2350Event::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            synced: true,
            authority: locked_authority(),
        })
        .expect("rp2350 trigger edge should be captured");
    rp2350
        .apply_event(Rp2350Event::CamEdge {
            at_us: Micros::new(13),
            cam_seen: true,
        })
        .expect("rp2350 cam edge should update decoder state");
    second
        .apply_event(Rp2350Event::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            synced: true,
            authority: locked_authority(),
        })
        .expect("second adapter trigger edge should be captured");
    second
        .apply_event(Rp2350Event::CamEdge {
            at_us: Micros::new(13),
            cam_seen: true,
        })
        .expect("second adapter cam edge should update decoder state");

    rp2350
        .apply_event(Rp2350Event::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .expect("rp2350 tick should execute board IO")
        .expect("tick events should return a runtime step result");
    second
        .apply_event(Rp2350Event::Tick {
            now_us: Micros::new(20),
            control: control_inputs(),
        })
        .expect("second adapter tick should execute board IO")
        .expect("tick events should return a runtime step result");

    assert!(counters_a.action.get() > 0);
    assert!(counters_b.action.get() > 0);
    assert!(counters_a.transport.get() > 0);
    assert!(counters_b.transport.get() > 0);
    assert!(counters_a.watchdog.get() > 0);
    assert!(counters_b.watchdog.get() > 0);
}

#[test]
fn board_adapter_boundary_errors_are_classified_by_trait() {
    assert_eq!(
        rp2350_with_failures(MockFailures {
            sensor: Some(InjectedBoundaryError::Sensor),
            ..MockFailures::default()
        })
        .poll_sensor(),
        Err(BoardAdapterError::Sensor(InjectedBoundaryError::Sensor))
    );

    assert_eq!(
        rp2350_with_failures(MockFailures {
            capture: Some(InjectedBoundaryError::Capture),
            ..MockFailures::default()
        })
        .apply_event(Rp2350Event::TriggerEdge {
            at_us: Micros::new(12),
            rpm: Rpm::new(3000),
            angle_x10: Degrees10::new(12),
            synced: true,
            authority: locked_authority(),
        }),
        Err(BoardAdapterError::Capture(InjectedBoundaryError::Capture))
    );

    assert_eq!(
        tick_with_failures(MockFailures {
            action: Some(InjectedBoundaryError::Action),
            ..MockFailures::default()
        }),
        Err(BoardAdapterError::Action(InjectedBoundaryError::Action))
    );

    assert_eq!(
        tick_with_failures(MockFailures {
            transport: Some(InjectedBoundaryError::Transport),
            ..MockFailures::default()
        }),
        Err(BoardAdapterError::Transport(
            InjectedBoundaryError::Transport
        ))
    );

    assert_eq!(
        dirty_tick_with_failures(MockFailures {
            store: Some(InjectedBoundaryError::Store),
            ..MockFailures::default()
        }),
        Err(BoardAdapterError::Persistence(InjectedBoundaryError::Store))
    );

    assert_eq!(
        tick_with_failures(MockFailures {
            watchdog: Some(InjectedBoundaryError::Watchdog),
            ..MockFailures::default()
        }),
        Err(BoardAdapterError::Watchdog(InjectedBoundaryError::Watchdog))
    );
}

fn rp2350_with_failures(
    failures: MockFailures,
) -> Rp2350Adapter<MockSensor, MockCapture, MockActions, MockWatchdog, MockTransport, MockStore> {
    let counters = Counters::new();
    let mut adapter = Rp2350Adapter::new(
        mock_sensor(failures),
        mock_capture(failures),
        mock_actions(counters.clone(), failures),
        mock_watchdog(counters.clone(), failures),
        mock_transport(counters.clone(), failures),
        mock_store(counters, failures),
    );
    adapter.configure_fuel_model(test_fuel_model());
    adapter
}

fn tick_with_failures(failures: MockFailures) -> BoundaryStepResult {
    let mut adapter = rp2350_with_failures(failures);
    adapter.apply_event(Rp2350Event::TriggerEdge {
        at_us: Micros::new(12),
        rpm: Rpm::new(3000),
        angle_x10: Degrees10::new(12),
        synced: true,
        authority: full_sequential_authority(),
    })?;
    adapter.apply_event(Rp2350Event::Tick {
        now_us: Micros::new(20),
        control: control_inputs(),
    })
}

fn dirty_tick_with_failures(failures: MockFailures) -> BoundaryStepResult {
    let mut adapter = rp2350_with_failures(failures);
    adapter.set_staged_dirty(true);
    adapter.apply_event(Rp2350Event::Tick {
        now_us: Micros::new(20),
        control: control_inputs(),
    })
}

fn test_fuel_model() -> ecu_runtime::BaseFuelModel {
    ecu_runtime::BaseFuelModel::new(
        [Rpm::new(0); 16],
        [Kpa10::new(0); 16],
        [[PulseWidthUs::new(2_500); 16]; 16],
    )
}

#[test]
fn all_registered_recipes_satisfy_watchdog_policy() {
    let mut failures: Vec<String> = Vec::new();
    for (board, recipe_name) in SUPPORTED_INVOCATIONS {
        match resolve_recipe(board, recipe_name) {
            Ok(recipe) => {
                if let Err(msg) = check_watchdog_policy(
                    recipe_name,
                    recipe.safety.watchdog_required,
                    recipe.safety.watchdog_absent_justification,
                ) {
                    failures.push(msg);
                }
            }
            Err(e) => failures.push(format!("could not resolve {board}/{recipe_name}: {e:?}")),
        }
    }
    assert!(
        failures.is_empty(),
        "watchdog policy violations:\n{}",
        failures.join("\n")
    );
}
