//! Closed-loop plant scenario tests.
//!
//! These tests prove the Batch 3 plant consumes ECU-shaped output transitions
//! and feeds causal RPM/MAP/sensor changes back to host scenarios.

use ecu_domain::{ChannelId, Kpa10, Micros, Rpm};
use ecu_io::{OutputLevel, OutputTransition, OutputTransitionKind};
use ecu_sim::output_capture::FixedTransitionBuffer;
use ecu_sim::plant::{
    ClosedLoopPlant, FixedPlantProfile, InjectorModel, PlantControls, PlantError, PlantFault,
};

fn transition(
    at_us: u32,
    kind: OutputTransitionKind,
    channel: u8,
    level: OutputLevel,
) -> OutputTransition {
    OutputTransition {
        at_us: Micros::new(at_us),
        kind,
        channel: ChannelId::new(channel),
        level,
    }
}

fn plant() -> ClosedLoopPlant<FixedPlantProfile> {
    let profile = FixedPlantProfile::inline_four();
    ClosedLoopPlant::new(
        profile,
        InjectorModel::gasoline(profile.injector_flow_cc_per_min),
    )
}

fn combustion_capture() -> FixedTransitionBuffer<8> {
    let mut capture = FixedTransitionBuffer::<8>::new();
    capture
        .push(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    capture
        .push(transition(
            5_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    capture
        .push(transition(
            6_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    capture
        .push(transition(
            9_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    capture
}

#[test]
fn captured_output_feedback_raises_future_rpm_and_changes_map() {
    let mut no_output = plant();
    let baseline = no_output.advance_to(Micros::new(100_000), PlantControls::idle());

    let mut with_output = plant();
    let report = with_output.consume_capture(&combustion_capture()).unwrap();
    let driven = with_output.advance_to(Micros::new(100_000), PlantControls::idle());

    assert_eq!(report.transitions, 4);
    assert_eq!(report.combustion_events, 1);
    assert!(driven.rpm.get() > baseline.rpm.get());
    assert_ne!(driven.map_kpa10.get(), baseline.map_kpa10.get());
    assert!(driven.last_torque_x100 > 0);
}

#[test]
fn missing_injection_transition_suppresses_torque() {
    let mut spark_only = FixedTransitionBuffer::<4>::new();
    spark_only
        .push(transition(
            6_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    spark_only
        .push(transition(
            9_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::Low,
        ))
        .unwrap();

    let mut plant = plant();
    let report = plant.consume_capture(&spark_only).unwrap();
    let snapshot = plant.advance_to(Micros::new(100_000), PlantControls::idle());

    assert_eq!(report.transitions, 2);
    assert_eq!(report.combustion_events, 0);
    assert_eq!(snapshot.combustion_events, 0);
    assert_eq!(snapshot.last_torque_x100, 0);
}

#[test]
fn missing_spark_transition_suppresses_torque() {
    let mut fuel_only = FixedTransitionBuffer::<4>::new();
    fuel_only
        .push(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    fuel_only
        .push(transition(
            5_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap();

    let mut plant = plant();
    let report = plant.consume_capture(&fuel_only).unwrap();
    let snapshot = plant.advance_to(Micros::new(100_000), PlantControls::idle());

    assert_eq!(report.transitions, 2);
    assert_eq!(report.combustion_events, 0);
    assert_eq!(snapshot.injector_events, 1);
    assert_eq!(snapshot.spark_events, 0);
    assert_eq!(snapshot.last_torque_x100, 0);
}

#[test]
fn throttle_and_load_affect_future_map_and_rpm() {
    let mut light_load = plant();
    light_load.set_initial_rpm(Rpm::new(2_000));
    let light = light_load.advance_to(
        Micros::new(100_000),
        PlantControls {
            throttle_x100: 10,
            starter_on: false,
            load_torque_x100: 100,
            vbatt_mv: 12_500,
            fault: PlantFault::None,
        },
    );

    let mut heavy_load = plant();
    heavy_load.set_initial_rpm(Rpm::new(2_000));
    let heavy = heavy_load.advance_to(
        Micros::new(100_000),
        PlantControls {
            throttle_x100: 80,
            starter_on: false,
            load_torque_x100: 1_500,
            vbatt_mv: 12_500,
            fault: PlantFault::None,
        },
    );

    assert!(heavy.map_kpa10.get() > light.map_kpa10.get());
    assert!(heavy.rpm.get() < light.rpm.get());
}

#[test]
fn deterministic_fault_injection_overrides_sensor_outputs() {
    let mut plant = plant();
    plant.set_initial_rpm(Rpm::new(1_500));
    plant.advance_to(Micros::new(50_000), PlantControls::idle());

    let frame = plant.sensor_frame(
        Micros::new(50_000),
        PlantControls {
            throttle_x100: 25,
            starter_on: false,
            load_torque_x100: 500,
            vbatt_mv: 10_800,
            fault: PlantFault::MapStuck(Kpa10::new(777)),
        },
    );
    assert_eq!(frame.map_kpa10.get(), 777);
    assert_eq!(frame.vbatt_mv, 10_800);

    let dropout = plant.sensor_frame(
        Micros::new(60_000),
        PlantControls {
            fault: PlantFault::RpmDropout,
            ..PlantControls::idle()
        },
    );
    assert_eq!(dropout.rpm.get(), 0);
}

#[test]
fn pulse_width_limit_is_explicit_error() {
    let mut plant = plant();
    plant
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    let error = plant
        .apply_transition(transition(
            40_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap_err();

    assert!(matches!(
        error,
        PlantError::PulseTooLong {
            channel,
            pulse_width_us
        } if channel == ChannelId::new(0) && pulse_width_us == 39_000
    ));
}

#[test]
fn suppress_combustion_fault_keeps_event_count_but_removes_torque_feedback() {
    let mut plant = plant();
    let report = plant.consume_capture(&combustion_capture()).unwrap();
    let snapshot = plant.advance_to(
        Micros::new(100_000),
        PlantControls {
            fault: PlantFault::SuppressCombustion,
            ..PlantControls::idle()
        },
    );

    assert_eq!(report.combustion_events, 1);
    assert_eq!(snapshot.combustion_events, 1);
    assert_eq!(snapshot.last_torque_x100, 0);
}
