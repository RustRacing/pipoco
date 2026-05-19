use ecu_domain::ChannelId;
use ecu_domain::Rpm as HostRpm;
use ecu_io::{OutputLevel, OutputTransition, OutputTransitionKind};
use ecu_sim_core as core_plant;
use ecu_sim_driver::embedded_loop::{
    SimBoard, SimBoardTraceKind, SimControlMode, SimDriverInput, SimEdgePolarity, SimEnvironment,
    SimSensorFrame, SimTriggerEdge, SimTriggerLine, TorqueNmX100,
};
use ecu_sim_driver::x86_board::{
    run_x86_closed_loop_engine, X86ClosedLoopCut, X86ClosedLoopRunResult, X86PlantBridgeFrame,
    X86SimBoard,
};
use std::sync::{Arc, Barrier};
use std::thread;

const BOARD_OUTPUT_CAPACITY: usize = 128;
const BOARD_TRACE_CAPACITY: usize = 4096;
const BOARD_TICK_US: u32 = 10_000;
const CORE_PLANT_EDGE_CAPACITY: usize = 128;
const CORE_PLANT_EVENT_CAPACITY: usize = 8;

type HarnessPlantCut = X86ClosedLoopCut;

fn transition(
    kind: OutputTransitionKind,
    channel: u8,
    at_us: u32,
    level: OutputLevel,
) -> OutputTransition {
    OutputTransition {
        at_us: ecu_domain::Micros::new(at_us),
        kind,
        channel: ChannelId::new(channel),
        level,
    }
}

fn open_loop_board_driver_input(requested_rpm: u16) -> SimDriverInput {
    SimDriverInput {
        throttle_x1000: 800,
        requested_rpm: HostRpm::new(requested_rpm),
        load_torque_nm_x100: TorqueNmX100::new(0),
        mode: SimControlMode::OpenLoopRpm,
    }
}

fn board_environment() -> SimEnvironment {
    SimEnvironment {
        ambient_pressure_pa: 101_325,
        ambient_temp_k_x10: 2_931,
        battery_mv: 12_500,
    }
}

fn open_loop_startup_sensor_frame(
    timestamp_us: u32,
    driver: SimDriverInput,
    environment: SimEnvironment,
) -> SimSensorFrame {
    SimSensorFrame {
        timestamp_us,
        rpm: driver.requested_rpm,
        crank_angle_deg10: 0,
        map_kpa10: 600,
        tps_x1000: driver.throttle_x1000,
        clt_c10: 800,
        iat_c10: 300,
        lambda_x1000: 1_000,
        battery_mv: environment.battery_mv,
        knock_intensity_x100: 0,
    }
}

fn to_sim_trigger_edge(edge: core_plant::TriggerEdge) -> SimTriggerEdge {
    SimTriggerEdge {
        timestamp_us: edge.timestamp_us.0,
        line: match edge.channel {
            core_plant::TriggerChannel::Crank => SimTriggerLine::Crank,
            core_plant::TriggerChannel::Cam => SimTriggerLine::Cam,
        },
        polarity: match edge.edge {
            core_plant::EdgePolarity::Rising => SimEdgePolarity::Rising,
            core_plant::EdgePolarity::Falling => SimEdgePolarity::Falling,
        },
        angle_deg10: edge.crank_angle_deg10.0,
    }
}

fn to_sim_sensor_frame(
    sensors: core_plant::SensorSnapshot,
    driver: SimDriverInput,
) -> SimSensorFrame {
    SimSensorFrame {
        timestamp_us: sensors.timestamp_us.0,
        rpm: HostRpm::new(sensors.rpm.0.min(u16::MAX as u32) as u16),
        crank_angle_deg10: sensors.crank_angle_deg10.0,
        map_kpa10: sensors.map_kpa10.0,
        tps_x1000: driver.throttle_x1000,
        clt_c10: sensors.clt_c10.0,
        iat_c10: sensors.iat_c10.0,
        lambda_x1000: sensors.lambda_x1000,
        battery_mv: sensors.battery_mv.0,
        knock_intensity_x100: sensors.knock_intensity_x100,
    }
}

type RunResult = X86ClosedLoopRunResult<CORE_PLANT_EDGE_CAPACITY, CORE_PLANT_EVENT_CAPACITY>;

fn run_open_loop_rpm_case(requested_rpm: u16) -> (u32, u16) {
    let mut board: X86SimBoard<BOARD_OUTPUT_CAPACITY, BOARD_TRACE_CAPACITY> = X86SimBoard::new();
    let driver = open_loop_board_driver_input(requested_rpm);
    board.set_driver_input(driver);
    board.set_environment(board_environment());

    let board_driver = board.read_driver_input().expect("driver input");
    assert_eq!(board_driver.mode, SimControlMode::OpenLoopRpm);
    assert_eq!(board_driver.requested_rpm, HostRpm::new(requested_rpm));
    let environment = board.read_environment().expect("environment");

    board
        .feed_sensor_frame(open_loop_startup_sensor_frame(0, board_driver, environment))
        .expect("initial sensor frame");

    let mut plant =
        core_plant::Plant::<4, CORE_PLANT_EDGE_CAPACITY, CORE_PLANT_EVENT_CAPACITY>::new({
            let mut config = core_plant::PlantConfig::<4>::default_four();
            config.trigger = core_plant::TriggerConfig {
                crank_teeth: 60,
                missing_teeth: 2,
                cam_pulses: 1,
            };
            config
        });
    plant.reset(core_plant::InitialPlantState {
        timestamp_us: core_plant::Micros(0),
        rpm: core_plant::Rpm(requested_rpm as u32),
        crank_angle_deg10: core_plant::CrankDeg10(0),
        ..core_plant::InitialPlantState::new()
    });

    let mut plant_input = core_plant::PlantStepInput::<4, CORE_PLANT_EVENT_CAPACITY>::idle(
        core_plant::Micros(BOARD_TICK_US),
    );
    plant_input.driver.throttle_x1000 = board_driver.throttle_x1000;
    plant_input.driver.load_torque_nm_x100 =
        core_plant::TorqueNmX100(board_driver.load_torque_nm_x100.0);
    plant_input.driver.starter_enabled = requested_rpm < 1_200;
    plant_input.environment = core_plant::EnvironmentInput {
        ambient_c10: core_plant::Celsius10(293),
        coolant_c10: core_plant::Celsius10(800),
        battery_mv: core_plant::Millivolts(environment.battery_mv),
    };

    let mut plant_output = core_plant::PlantStepOutput::<
        4,
        CORE_PLANT_EDGE_CAPACITY,
        CORE_PLANT_EVENT_CAPACITY,
    >::empty();
    plant
        .step(&plant_input, &mut plant_output)
        .expect("core plant step");

    let mut crank_edges: Vec<_> = plant_output
        .trigger_edges
        .as_slice()
        .iter()
        .copied()
        .filter(|edge| matches!(edge.channel, core_plant::TriggerChannel::Crank))
        .collect();
    assert!(
        crank_edges.len() >= 2,
        "plant must emit at least two crank edges for RPM spacing checks"
    );
    crank_edges.sort_by_key(|edge| edge.timestamp_us.0);
    let spacing_us = crank_edges
        .windows(2)
        .map(|pair| pair[1].timestamp_us.0 - pair[0].timestamp_us.0)
        .min()
        .expect("crank edge spacing");

    let trigger_edges: Vec<_> = plant_output
        .trigger_edges
        .as_slice()
        .iter()
        .copied()
        .map(to_sim_trigger_edge)
        .collect();
    board
        .feed_trigger_edges(&trigger_edges)
        .expect("feed plant trigger edges");
    board
        .feed_sensor_frame(to_sim_sensor_frame(plant_output.sensors, board_driver))
        .expect("feed plant sensors");

    (spacing_us, board.ecu_state().rpm())
}

fn run_board_closed_loop_engine(cut: HarnessPlantCut) -> RunResult {
    run_x86_closed_loop_engine::<
        BOARD_OUTPUT_CAPACITY,
        BOARD_TRACE_CAPACITY,
        CORE_PLANT_EDGE_CAPACITY,
        CORE_PLANT_EVENT_CAPACITY,
    >(cut)
}

fn final_rpm(result: &RunResult) -> u32 {
    result.final_plant_output.sensors.rpm.0
}

fn assert_plant_output_published(
    result: &RunResult,
) -> ecu_sim_driver::x86_board::X86PlantOutputSummary {
    let published = result
        .last_published_plant_output
        .expect("plant outputs should be published");
    assert!(result
        .trace
        .iter()
        .any(|record| matches!(record.kind, SimBoardTraceKind::PlantOutput)));
    assert_eq!(
        published.timestamp_us,
        result.final_plant_output.sensors.timestamp_us.0
    );
    assert_eq!(
        published.rpm,
        result.final_plant_output.sensors.rpm.0.min(u16::MAX as u32) as u16
    );
    assert_eq!(
        published.crank_angle_deg10,
        result.final_plant_output.sensors.crank_angle_deg10.0
    );
    assert_eq!(
        published.map_kpa10,
        result.final_plant_output.sensors.map_kpa10.0
    );
    assert_eq!(
        published.lambda_x1000,
        result.final_plant_output.sensors.lambda_x1000
    );
    assert_eq!(
        published.combustion_torque_nm_x100,
        result.final_plant_output.combustion.total_torque_nm_x100.0
    );
    assert_eq!(
        published.diagnostic_overflow_count,
        result.final_plant_output.diagnostics.overflow_count
    );
    published
}

#[test]
fn x86_board_bridge_reports_short_injector_pulse_width() {
    let mut board: X86SimBoard<1, 1> = X86SimBoard::new();
    let frame: X86PlantBridgeFrame<4, 1> = board.build_core_output_frame::<4, 1, _>(
        [
            transition(OutputTransitionKind::Injector, 0, 100, OutputLevel::High),
            transition(OutputTransitionKind::Injector, 0, 100, OutputLevel::Low),
        ],
        0,
        0,
    );

    assert_eq!(frame.diagnostics.short_pulse_width_count, 1);
    assert_eq!(frame.diagnostics.short_dwell_count, 0);
    assert!(!frame.diagnostics.is_clean());
    assert_eq!(frame.ecu_outputs.injection_events.as_slice().len(), 1);
    assert_eq!(
        frame.ecu_outputs.injection_events.as_slice()[0]
            .pulse_width_us
            .0,
        1
    );
}

#[test]
fn x86_board_bridge_reports_short_spark_dwell() {
    let mut board: X86SimBoard<1, 1> = X86SimBoard::new();
    let frame: X86PlantBridgeFrame<4, 1> = board.build_core_output_frame::<4, 1, _>(
        [
            transition(OutputTransitionKind::Ignition, 2, 250, OutputLevel::High),
            transition(OutputTransitionKind::Ignition, 2, 250, OutputLevel::Low),
        ],
        0,
        0,
    );

    assert_eq!(frame.diagnostics.short_pulse_width_count, 0);
    assert_eq!(frame.diagnostics.short_dwell_count, 1);
    assert!(!frame.diagnostics.is_clean());
    assert_eq!(frame.ecu_outputs.spark_events.as_slice().len(), 1);
    assert_eq!(
        frame.ecu_outputs.spark_events.as_slice()[0].dwell_us.0,
        3_000
    );
}

#[test]
fn x86_board_bridge_reports_open_high_at_finalize() {
    let mut board: X86SimBoard<1, 1> = X86SimBoard::new();
    let frame: X86PlantBridgeFrame<4, 1> = board.build_core_output_frame::<4, 1, _>(
        [transition(
            OutputTransitionKind::Injector,
            0,
            100,
            OutputLevel::High,
        )],
        0,
        0,
    );

    assert!(
        frame.diagnostics.is_clean(),
        "a high transition can be completed by a later bridge frame"
    );
    assert_eq!(frame.ecu_outputs.injection_events.as_slice().len(), 0);

    let diagnostics = board.bridge_open_high_diagnostics();
    assert_eq!(diagnostics.open_high_count, 1);
    assert!(!diagnostics.is_clean());
}

#[test]
fn x86_board_bridge_rejects_out_of_order_low_transitions() {
    let mut board: X86SimBoard<1, 1> = X86SimBoard::new();
    let frame: X86PlantBridgeFrame<4, 2> = board.build_core_output_frame::<4, 2, _>(
        [
            transition(OutputTransitionKind::Injector, 0, 200, OutputLevel::High),
            transition(OutputTransitionKind::Injector, 0, 100, OutputLevel::Low),
            transition(OutputTransitionKind::Ignition, 2, 400, OutputLevel::High),
            transition(OutputTransitionKind::Ignition, 2, 300, OutputLevel::Low),
        ],
        0,
        0,
    );

    assert_eq!(frame.diagnostics.out_of_order_transition_count, 2);
    assert_eq!(frame.diagnostics.short_pulse_width_count, 0);
    assert_eq!(frame.diagnostics.short_dwell_count, 0);
    assert!(!frame.diagnostics.is_clean());
    assert_eq!(frame.ecu_outputs.injection_events.as_slice().len(), 0);
    assert_eq!(frame.ecu_outputs.spark_events.as_slice().len(), 0);
}

#[test]
fn x86_board_closed_loop_engine_responds_to_ecu_outputs() {
    let result = run_board_closed_loop_engine(HarnessPlantCut::None);
    let injection_cut = run_board_closed_loop_engine(HarnessPlantCut::Injection);
    let ignition_cut = run_board_closed_loop_engine(HarnessPlantCut::Ignition);
    assert_plant_output_published(&result);

    assert!(
        result.ecu.synced,
        "ECU should sync through the x86 board path"
    );
    assert!(
        result.ecu.final_pw_us > 0,
        "fuel pulse width should be computed"
    );
    assert!(
        result.injector_outputs_seen > 0,
        "injector transitions should surface in the x86 board path"
    );
    assert!(
        result.ignition_outputs_seen > 0,
        "ignition transitions should surface in the x86 board path; observed kinds: {:?}",
        result
            .output_history
            .iter()
            .map(|transition| transition.kind)
            .collect::<Vec<_>>()
    );
    assert!(
        result.combustion_seen > 0,
        "captured outputs should be bridgeable into the core plant"
    );
    assert!(
        result.bridge_diagnostics.is_clean(),
        "bridge diagnostics should be clean"
    );
    assert!(
        final_rpm(&result) > final_rpm(&injection_cut),
        "accepted injection should produce a higher final plant RPM than the injection-cut path: normal={} injection_cut={}",
        final_rpm(&result),
        final_rpm(&injection_cut)
    );
    assert!(
        final_rpm(&result) > final_rpm(&ignition_cut),
        "accepted spark should produce a higher final plant RPM than the ignition-cut path: normal={} ignition_cut={}",
        final_rpm(&result),
        final_rpm(&ignition_cut)
    );
    assert!(result.report.tick_count > 0);
    assert!(!result.trace.is_empty());
    assert_eq!(result.report.output_overflow_count, 0);
    assert_eq!(result.report.trace_overflow_count, 0);
    assert!(result
        .trace
        .iter()
        .any(|record| matches!(record.kind, SimBoardTraceKind::SensorFrame)));
}

#[test]
fn x86_board_closed_loop_smoke_combusts_and_syncs() {
    let result = run_board_closed_loop_engine(HarnessPlantCut::None);
    assert_plant_output_published(&result);

    assert!(
        result.ecu.synced,
        "ECU should sync through the x86 board path"
    );
    assert!(
        result.ecu.final_pw_us > 0,
        "fuel pulse width should be computed"
    );
    assert!(
        result.injector_outputs_seen > 0,
        "injector transitions should surface in the x86 board path"
    );
    assert!(
        result.ignition_outputs_seen > 0,
        "ignition transitions should surface in the x86 board path; observed kinds: {:?}",
        result
            .output_history
            .iter()
            .map(|transition| transition.kind)
            .collect::<Vec<_>>()
    );
    assert!(
        result.combustion_seen > 0,
        "captured outputs should be bridgeable into the core plant"
    );
    assert!(
        result.bridge_diagnostics.is_clean(),
        "bridge diagnostics should be clean"
    );
    assert!(result.report.tick_count > 0);
    assert!(!result.trace.is_empty());
    assert_eq!(result.report.output_overflow_count, 0);
    assert_eq!(result.report.trace_overflow_count, 0);
    assert!(result
        .trace
        .iter()
        .any(|record| matches!(record.kind, SimBoardTraceKind::SensorFrame)));
}

#[test]
fn x86_board_open_loop_rpm_tracks_commanded_speed() {
    let sweep = [500u16, 800, 1_200, 1_800, 2_700, 4_000, 5_800];
    let mut previous_spacing_us = None;

    for requested_rpm in sweep {
        let (spacing_us, observed_rpm) = run_open_loop_rpm_case(requested_rpm);

        if let Some(previous) = previous_spacing_us {
            assert!(
                spacing_us <= previous,
                "crank edge spacing should shrink as requested RPM rises: requested={} spacing_us={} previous_spacing_us={}",
                requested_rpm,
                spacing_us,
                previous
            );
        }
        previous_spacing_us = Some(spacing_us);

        let tolerance_rpm = (requested_rpm / 5).max(400);
        let rpm_delta = observed_rpm.abs_diff(requested_rpm);
        assert!(
            rpm_delta <= tolerance_rpm,
            "open-loop RPM should stay within a broad tolerance: requested={} observed={} delta={} tolerance={}",
            requested_rpm,
            observed_rpm,
            rpm_delta,
            tolerance_rpm
        );
    }
}

#[test]
fn x86_board_closed_loop_injection_cut_stops_combustion() {
    let result = run_board_closed_loop_engine(HarnessPlantCut::Injection);
    let published = assert_plant_output_published(&result);

    assert!(
        result.ecu.synced,
        "ECU should still sync with the harness cut"
    );
    assert!(
        result.ecu.final_pw_us > 0,
        "the ECU should still compute a fuel pulse width"
    );
    assert!(
        result.injector_outputs_seen > 0,
        "the ECU should still emit injector transitions before the harness cut"
    );
    assert!(
        result.ignition_outputs_seen > 0,
        "the ECU should still emit ignition transitions before the harness cut"
    );
    assert_eq!(
        result.combustion_seen, 0,
        "injecting the harness fuel cut should prevent combustion"
    );
    assert!(
        result.ignored_injection_events > 0,
        "the plant should record ignored injection events when the harness cut is active"
    );
    assert!(
        result.bridge_diagnostics.is_clean(),
        "bridge diagnostics should remain visible and clean in the cut path"
    );
    assert!(result.report.tick_count > 0);
    assert!(!result.trace.is_empty());
    assert_eq!(published.combustion_count, 0);
    assert_eq!(published.combustion_torque_nm_x100, 0);
}

#[test]
fn x86_board_closed_loop_ignition_cut_stops_combustion() {
    let result = run_board_closed_loop_engine(HarnessPlantCut::Ignition);
    let published = assert_plant_output_published(&result);

    assert!(
        result.ecu.synced,
        "ECU should still sync with the harness cut"
    );
    assert!(
        result.ecu.final_pw_us > 0,
        "the ECU should still compute a fuel pulse width"
    );
    assert!(
        result.injector_outputs_seen > 0,
        "the ECU should still emit injector transitions before the harness cut"
    );
    assert!(
        result.ignition_outputs_seen > 0,
        "the ECU should still emit ignition transitions before the harness cut"
    );
    assert_eq!(
        result.combustion_seen, 0,
        "injecting the harness ignition cut should prevent combustion"
    );
    assert!(
        result.ignored_spark_events > 0,
        "the plant should record ignored spark events when the harness cut is active"
    );
    assert!(
        result.bridge_diagnostics.is_clean(),
        "bridge diagnostics should remain visible and clean in the cut path"
    );
    assert!(result.report.tick_count > 0);
    assert!(!result.trace.is_empty());
    assert_eq!(published.combustion_count, 0);
    assert_eq!(published.combustion_torque_nm_x100, 0);
}

#[test]
fn x86_board_replay_is_byte_identical() {
    let first = run_board_closed_loop_engine(HarnessPlantCut::None);
    let second = run_board_closed_loop_engine(HarnessPlantCut::None);

    assert_eq!(first, second);
}

#[test]
fn x86_board_parallel_instances_do_not_interfere() {
    const PARALLEL: usize = 4;

    let baseline = run_board_closed_loop_engine(HarnessPlantCut::None);
    let start = Arc::new(Barrier::new(PARALLEL + 1));

    let handles: Vec<_> = (0..PARALLEL)
        .map(|_| {
            let start = Arc::clone(&start);
            thread::spawn(move || {
                start.wait();
                run_board_closed_loop_engine(HarnessPlantCut::None)
            })
        })
        .collect();

    start.wait();

    let results: Vec<RunResult> = handles
        .into_iter()
        .map(|handle| handle.join().expect("parallel run panicked"))
        .collect();

    for (idx, result) in results.iter().enumerate() {
        assert_eq!(
            result, &baseline,
            "parallel run #{idx} diverged from sequential baseline",
        );
    }
}
