use ecu_sim_core::*;
use proptest::prelude::*;

const CYL: usize = 4;
const EDGES: usize = 128;
const EVENTS: usize = 8;

type TestPlant = Plant<CYL, EDGES, EVENTS>;
type TestInput = PlantStepInput<CYL, EVENTS>;
type TestOutput = PlantStepOutput<CYL, EDGES, EVENTS>;

fn config() -> PlantConfig<CYL> {
    PlantConfig::<CYL>::default_four()
}

fn single_cylinder_event_input(
    dt_us: u32,
    throttle_x1000: u16,
    fuel_cut: bool,
    spark_cut: bool,
) -> TestInput {
    let mut input = TestInput::idle(Micros(dt_us));
    input.driver.throttle_x1000 = throttle_x1000;
    input.ecu_outputs.fuel_cut = fuel_cut;
    input.ecu_outputs.spark_cut = spark_cut;
    input
        .ecu_outputs
        .injection_events
        .push(InjectionCommand {
            cylinder: CylinderIndex(0),
            mode: InjectionTimingMode::StartOfInjection,
            angle_deg10: CrankDeg10(3600),
            pulse_width_us: Micros(3500),
            injector_flow_ug_per_us: MicrogramsPerMicros(2),
            deadtime_us: Micros(1000),
        })
        .unwrap();
    input
        .ecu_outputs
        .spark_events
        .push(SparkCommand {
            cylinder: CylinderIndex(0),
            spark_angle_deg10: CrankDeg10(180),
            dwell_us: Micros(2000),
            coil_energy_x1000: 1000,
        })
        .unwrap();
    input
}

proptest! {
    #[test]
    fn generated_valid_cylinder_configs_validate(
        cylinder_count in 1u8..=4,
        displacement_cc in 500u32..8000,
        compression_x100 in 700u16..1600,
    ) {
        let mut cfg = config();
        cfg.cylinder_count = cylinder_count;
        cfg.displacement_cc = displacement_cc;
        cfg.engine.displacement_cc = displacement_cc;
        cfg.compression_ratio_x100 = compression_x100;
        cfg.engine.compression_ratio_x100 = compression_x100;
        let spacing = CRANK_CYCLE_DEG10 as u32 / cylinder_count as u32;
        for i in 0..cylinder_count as usize {
            cfg.cylinder_phase_deg10[i] = CrankDeg10((i as u32 * spacing) as u16);
        }

        prop_assert_eq!(cfg.validate(), Ok(()));
    }

    #[test]
    fn constant_ve_table_reproduces_constant_for_generated_inputs(
        ve in 100u16..2000,
        rpm in 0u32..9000,
        load in 0u16..2500,
    ) {
        let table = VeTable::constant(ve);

        prop_assert_eq!(lookup_ve_x1000(&table, Rpm(rpm), Kpa10(load)), ve);
    }

    #[test]
    fn deterministic_replay_is_byte_identical(
        rpm in 0u32..7000,
        angle in 0u16..7200,
        dt_us in 0u32..50_000,
        throttle in 0u16..=1000,
        fuel_cut in any::<bool>(),
        spark_cut in any::<bool>(),
    ) {
        let mut a = TestPlant::new(config());
        let mut b = TestPlant::new(config());
        let initial = InitialPlantState {
            timestamp_us: Micros(123),
            rpm: Rpm(rpm),
            crank_angle_deg10: CrankDeg10(angle),
            ..InitialPlantState::new()
        };
        a.reset(initial);
        b.reset(initial);
        let input = single_cylinder_event_input(dt_us, throttle, fuel_cut, spark_cut);
        let mut out_a = TestOutput::empty();
        let mut out_b = TestOutput::empty();

        let result_a = a.step(&input, &mut out_a);
        let result_b = b.step(&input, &mut out_b);

        prop_assert_eq!(result_a, result_b);
        prop_assert_eq!(a, b);
        prop_assert_eq!(out_a, out_b);
    }

    #[test]
    fn deterministic_replay_holds_for_all_physics_modes(
        mode in 0u8..4,
        angle in 0u16..7200,
    ) {
        let mut cfg = config();
        cfg.physics_mode = match mode {
            0 => PlantPhysicsMode::SyntheticTorque,
            1 => PlantPhysicsMode::KinematicPressurePulse,
            2 => PlantPhysicsMode::PolytropicWiebe,
            _ => PlantPhysicsMode::SingleZoneIdealGas,
        };
        let mut a = TestPlant::new(cfg);
        let mut b = TestPlant::new(cfg);
        let initial = InitialPlantState {
            rpm: Rpm(1000),
            crank_angle_deg10: CrankDeg10(angle),
            ..InitialPlantState::new()
        };
        a.reset(initial);
        b.reset(initial);
        let input = single_cylinder_event_input(1000, 1000, false, false);
        let mut out_a = TestOutput::empty();
        let mut out_b = TestOutput::empty();

        let result_a = a.step(&input, &mut out_a);
        let result_b = b.step(&input, &mut out_b);

        prop_assert_eq!(result_a, result_b);
        prop_assert_eq!(a, b);
        prop_assert_eq!(out_a, out_b);
    }

    #[test]
    fn crank_angle_remains_normalized_for_valid_inputs(
        rpm in 0u32..7000,
        angle in 0u16..7200,
        dt_us in 0u32..50_000,
        throttle in 0u16..=1000,
    ) {
        let mut plant = TestPlant::new(config());
        plant.reset(InitialPlantState {
            rpm: Rpm(rpm),
            crank_angle_deg10: CrankDeg10(angle),
            ..InitialPlantState::new()
        });
        let input = single_cylinder_event_input(dt_us, throttle, false, false);
        let mut out = TestOutput::empty();

        let _ = plant.step(&input, &mut out);

        prop_assert!(plant.state.crank_angle_deg10.is_normalized());
    }

    #[test]
    fn cuts_dominate_combustion_eligibility(
        dt_us in 1u32..50_000,
        throttle in 0u16..=1000,
        fuel_cut in any::<bool>(),
        spark_cut in any::<bool>(),
    ) {
        prop_assume!(fuel_cut || spark_cut);
        let mut plant = TestPlant::new(config());
        let input = single_cylinder_event_input(dt_us, throttle, fuel_cut, spark_cut);
        let mut out = TestOutput::empty();

        plant.step(&input, &mut out).unwrap();

        prop_assert_eq!(out.combustion.cylinders[0].torque_nm_x100, TorqueNmX100(0));
        prop_assert!(out.combustion.cylinders[0].misfire.is_some());
    }

    #[test]
    fn horsepower_is_zero_when_torque_or_rpm_is_zero(
        torque in -10_000i32..10_000,
        rpm in 0u32..8000,
    ) {
        let zero_torque = horsepower_x100(TorqueNmX100(0), Rpm(rpm));
        let maybe_zero_rpm = horsepower_x100(TorqueNmX100(torque), Rpm(0));

        prop_assert_eq!(zero_torque, 0);
        prop_assert_eq!(maybe_zero_rpm, 0);
    }

    #[test]
    fn dyno_sweep_target_advances_monotonically(
        start in 800u32..2000,
        step in 100u32..500,
    ) {
        let mut cfg = config();
        cfg.dyno.mode = DynoMode::TargetRpmSweep;
        cfg.dyno.sweep_start_rpm = Rpm(start);
        cfg.dyno.sweep_end_rpm = Rpm(start + step * 2);
        cfg.dyno.sweep_step_rpm = Rpm(step);
        cfg.dyno.target_rpm = Rpm(start);
        cfg.dyno.hold_cycles_before_sample = 0;
        cfg.dyno.sample_cycles = 1;
        cfg.dyno.rpm_error_limit = Rpm(10_000);
        let mut plant = TestPlant::new(cfg);
        plant.reset(InitialPlantState {
            rpm: Rpm(start),
            crank_angle_deg10: CrankDeg10(7100),
            ..InitialPlantState::new()
        });
        let input = single_cylinder_event_input(100_000, 1000, false, false);
        let mut out = TestOutput::empty();

        plant.step(&input, &mut out).unwrap();

        prop_assert!(out.dyno.sweep_point.valid);
        prop_assert_eq!(out.dyno.sweep_point.target_rpm, Rpm(start));
        prop_assert_eq!(out.dyno.sweep_point.ve_x1000, 1000);
        prop_assert_eq!(out.dyno.sweep_target_rpm, Rpm(start + step));
    }

    #[test]
    fn trigger_timestamps_are_monotonic_without_nonmonotonic_faults(
        start in 0u16..7200,
        distance in 1u16..7200,
    ) {
        let end = CrankDeg10(((start as u32 + distance as u32) % CYCLE_DEG10) as u16);
        prop_assume!(end != CrankDeg10(start));
        let mut sequence = 0;
        let mut edges = FixedSlice::<TriggerEdge, EDGES>::empty(TriggerEdge::empty());

        generate_edges(
            TriggerConfig {
                crank_teeth: 36,
                missing_teeth: 1,
                cam_pulses: 1,
            },
            CrankDeg10(start),
            end,
            Micros(1_000),
            Micros(100_000),
            FaultInput::none(),
            &mut sequence,
            &mut edges,
        )
        .unwrap();

        for pair in edges.as_slice().windows(2) {
            prop_assert!(pair[0].timestamp_us <= pair[1].timestamp_us);
        }
    }
}
