use super::*;

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

#[derive(Clone, Copy)]
struct BadProfile;

impl PlantProfile for BadProfile {
    fn cylinders(self) -> u8 {
        1
    }

    fn redline_rpm(self) -> u16 {
        6_000
    }

    fn injector_cylinder(self, _channel: ChannelId) -> Option<u8> {
        Some(9)
    }

    fn ignition_cylinder(self, _channel: ChannelId) -> Option<u8> {
        Some(9)
    }
}

#[test]
fn injector_model_converts_pulse_width_to_fuel_mass() {
    let injector = InjectorModel::gasoline(240);

    assert_eq!(injector.fuel_mass_ug(0), 0);
    assert!(injector.fuel_mass_ug(3_000) > injector.fuel_mass_ug(1_000));
}

#[test]
fn fuel_and_spark_create_combustion_event() {
    let mut plant = plant();

    plant
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    plant
        .apply_transition(transition(
            4_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    plant
        .apply_transition(transition(
            5_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    let event = plant
        .apply_transition(transition(
            7_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::Low,
        ))
        .unwrap();

    assert!(event.is_some());
    assert_eq!(plant.snapshot(PlantControls::idle()).combustion_events, 1);
}

#[test]
fn missing_fuel_or_spark_produces_no_combustion() {
    let mut no_fuel = plant();
    no_fuel
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    let event = no_fuel
        .apply_transition(transition(
            3_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    assert!(event.is_none());

    let mut no_spark = plant();
    no_spark
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    no_spark
        .apply_transition(transition(
            4_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    assert_eq!(
        no_spark.snapshot(PlantControls::idle()).combustion_events,
        0
    );
}

#[test]
fn capture_buffer_can_drive_plant() {
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
            4_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    capture
        .push(transition(
            5_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    capture
        .push(transition(
            7_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::Low,
        ))
        .unwrap();

    let mut plant = plant();
    let report = plant.consume_capture(&capture).unwrap();

    assert_eq!(report.transitions, 4);
    assert_eq!(report.combustion_events, 1);
}

#[test]
fn invalid_profile_cylinder_returns_error_instead_of_panicking() {
    let mut plant = ClosedLoopPlant::new(BadProfile, InjectorModel::gasoline(200));
    let err = plant
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap_err();

    assert_eq!(err, PlantError::UnknownCylinder(9));
}

#[test]
fn invalid_injector_channel_returns_error_instead_of_panicking() {
    let mut plant = ClosedLoopPlant::new(BadProfile, InjectorModel::gasoline(200));
    let err = plant
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Injector,
            200,
            OutputLevel::High,
        ))
        .unwrap_err();

    assert_eq!(err, PlantError::UnknownInjectorChannel(ChannelId::new(200)));
}

#[test]
fn invalid_ignition_channel_returns_error_instead_of_panicking() {
    let mut plant = ClosedLoopPlant::new(BadProfile, InjectorModel::gasoline(200));
    let err = plant
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Ignition,
            200,
            OutputLevel::High,
        ))
        .unwrap_err();

    assert_eq!(err, PlantError::UnknownIgnitionChannel(ChannelId::new(200)));
}

#[test]
fn combustion_changes_future_rpm_and_map() {
    let mut baseline_plant = plant();
    let baseline = baseline_plant.advance_to(Micros::new(100_000), PlantControls::idle());

    let mut powered = plant();
    powered
        .apply_transition(transition(
            1_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    powered
        .apply_transition(transition(
            6_000,
            OutputTransitionKind::Injector,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    powered
        .apply_transition(transition(
            7_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::High,
        ))
        .unwrap();
    powered
        .apply_transition(transition(
            10_000,
            OutputTransitionKind::Ignition,
            0,
            OutputLevel::Low,
        ))
        .unwrap();
    let snapshot = powered.advance_to(Micros::new(100_000), PlantControls::idle());

    assert!(snapshot.rpm.get() > baseline.rpm.get());
    assert_ne!(snapshot.map_kpa10.get(), baseline.map_kpa10.get());
}
