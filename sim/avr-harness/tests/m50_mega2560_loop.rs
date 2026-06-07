use ecu_sim_avr_harness::{
    cranking_input, m50b25tu_config, run_plant_step_through_avr, running_input, AvrM5xHarness,
    CYLINDERS, MAX_EDGES, MAX_EVENTS,
};
use ecu_sim_core::{InitialPlantState, Micros, Plant, PlantStepOutput, Rpm};

fn new_m50_plant() -> Plant<CYLINDERS, MAX_EDGES, MAX_EVENTS> {
    let mut plant = Plant::<CYLINDERS, MAX_EDGES, MAX_EVENTS>::new(m50b25tu_config());
    plant.reset(InitialPlantState {
        timestamp_us: Micros(0),
        rpm: Rpm(600),
        crank_angle_deg10: ecu_sim_core::CrankDeg10(7100),
        cylinder_fuel_mass_ug: [ecu_sim_core::MassUg(0); CYLINDERS],
    });
    plant
}

#[test]
fn speeduino_m5x_stub_mega2560_connected_to_m50b25tu_plant() {
    let mut plant = new_m50_plant();
    let mut avr = AvrM5xHarness::load_stub();
    let mut output = PlantStepOutput::empty();

    run_plant_step_through_avr(&mut plant, cranking_input(80_000), &mut avr, &mut output);

    assert!(
        avr.transitions().iter().any(|transition| matches!(
            transition.output,
            ecu_sim_avr_harness::HarnessOutput::Injector(0)
        ) && transition.high),
        "plant trigger edges did not produce an injector pulse on Mega2560 D8/PH5"
    );
    assert!(
        avr.transitions().iter().any(|transition| matches!(
            transition.output,
            ecu_sim_avr_harness::HarnessOutput::Ignition(0)
        ) && transition.high),
        "plant trigger edges did not produce an ignition pulse on Mega2560 D40/PG1"
    );

    let ecu_outputs = avr.drain_output_frame();
    assert!(!ecu_outputs.injection_events.is_empty());
    assert!(!ecu_outputs.spark_events.is_empty());

    run_plant_step_through_avr(
        &mut plant,
        running_input(20_000, ecu_outputs),
        &mut avr,
        &mut output,
    );

    assert!(
        output.consumed_events.injection_count > 0,
        "Mega2560 injector transition was not accepted by the M50 plant"
    );
    assert!(
        output.consumed_events.spark_count > 0,
        "Mega2560 ignition transition was not accepted by the M50 plant"
    );
}

#[test]
fn reference_speeduino_m5x_hex_accepts_m50b25tu_sensor_replay() {
    let firmware = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../aidocs/ref/code/Speeduino-M5x-PCBs/6-cyl firmware files/202305.hex"
    );
    assert!(
        std::path::Path::new(firmware).exists(),
        "missing reference Speeduino-M5x firmware at {firmware}"
    );

    let mut plant = new_m50_plant();
    let mut avr = AvrM5xHarness::load_firmware(firmware);
    let mut output = PlantStepOutput::empty();

    for _ in 0..8 {
        run_plant_step_through_avr(&mut plant, cranking_input(20_000), &mut avr, &mut output);
    }

    // The reference HEX does not include the project tune/eeprom image, so this
    // is a wiring/boot smoke. The deterministic closed-loop assertion above
    // stays on the small stub until tune loading is modeled.
    assert!(
        !output.trigger_edges.is_empty(),
        "M50 plant did not emit trigger edges for the reference firmware replay"
    );
}
