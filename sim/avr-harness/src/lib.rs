use std::path::Path;

use avr_tester::AvrTester;
use ecu_sim_core::{
    CrankDeg10, CylinderIndex, EcuOutputFrame, EdgePolarity, FixedSlice, InjectionCommand,
    InjectionTimingMode, MicrogramsPerMicros, Micros, Plant, PlantConfig, PlantStepInput,
    PlantStepOutput, SparkCommand, TorqueNmX100, TriggerChannel, TriggerConfig, TriggerEdge,
};

pub const CYLINDERS: usize = 6;
pub const MAX_EDGES: usize = 64;
pub const MAX_EVENTS: usize = 32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HarnessOutput {
    Injector(u8),
    Ignition(u8),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HarnessTransition {
    pub output: HarnessOutput,
    pub high: bool,
    pub at_us: u32,
    pub angle_deg10: CrankDeg10,
}

pub struct AvrM5xHarness {
    avr: AvrTester,
    now_us: u32,
    crank_high: bool,
    cam_high: bool,
    inj1_high: bool,
    ign1_high: bool,
    transitions: Vec<HarnessTransition>,
}

impl AvrM5xHarness {
    pub fn load_stub() -> Self {
        Self::load_firmware(env!("PIPOCO_AVR_STUB_ELF"))
    }

    pub fn load_firmware(firmware: impl AsRef<Path>) -> Self {
        let avr = AvrTester::atmega2560()
            .with_clock_of_16_mhz()
            .with_timeout_of_ms(250)
            .load(firmware);

        let mut harness = Self {
            avr,
            now_us: 0,
            crank_high: false,
            cam_high: false,
            inj1_high: false,
            ign1_high: false,
            transitions: Vec::new(),
        };
        harness.run_for_us(1_000);
        harness
    }

    pub fn drive_map_kpa10(&mut self, map_kpa10: u16) {
        let millivolts = map_kpa10_to_millivolts(map_kpa10);
        self.avr.pins().adc3().set_mv(millivolts);
    }

    pub fn feed_trigger_edge(&mut self, edge: TriggerEdge) {
        self.run_until(edge.timestamp_us.0);

        match edge.channel {
            TriggerChannel::Crank => {
                self.crank_high = matches!(edge.edge, EdgePolarity::Rising);
                self.avr.pins().pd2().set(self.crank_high);
            }
            TriggerChannel::Cam => {
                self.cam_high = matches!(edge.edge, EdgePolarity::Rising);
                self.avr.pins().pd3().set(self.cam_high);
            }
        }

        self.sample_outputs(edge.crank_angle_deg10);
        self.run_for_us(40);
        self.sample_outputs(edge.crank_angle_deg10);
    }

    pub fn finish_step(&mut self, end_us: u32, angle_deg10: CrankDeg10) {
        self.run_until(end_us);
        if self.crank_high {
            self.crank_high = false;
            self.avr.pins().pd2().set_low();
        }
        if self.cam_high {
            self.cam_high = false;
            self.avr.pins().pd3().set_low();
        }
        self.sample_outputs(angle_deg10);
    }

    pub fn transitions(&self) -> &[HarnessTransition] {
        &self.transitions
    }

    pub fn drain_output_frame(&mut self) -> EcuOutputFrame<CYLINDERS, MAX_EVENTS> {
        let frame = transitions_to_output_frame(&self.transitions);
        self.transitions.clear();
        frame
    }

    fn run_until(&mut self, target_us: u32) {
        if target_us > self.now_us {
            self.run_for_us(target_us - self.now_us);
        }
    }

    fn run_for_us(&mut self, duration_us: u32) {
        if duration_us == 0 {
            return;
        }
        self.avr.run_for_us(duration_us as u64);
        self.now_us = self.now_us.saturating_add(duration_us);
    }

    fn sample_outputs(&mut self, angle_deg10: CrankDeg10) {
        let inj1_high = self.avr.pins().ph5().is_high();
        let ign1_high = self.avr.pins().pg1().is_high();

        if inj1_high != self.inj1_high {
            self.transitions.push(HarnessTransition {
                output: HarnessOutput::Injector(0),
                high: inj1_high,
                at_us: self.now_us,
                angle_deg10,
            });
            self.inj1_high = inj1_high;
        }

        if ign1_high != self.ign1_high {
            self.transitions.push(HarnessTransition {
                output: HarnessOutput::Ignition(0),
                high: ign1_high,
                at_us: self.now_us,
                angle_deg10,
            });
            self.ign1_high = ign1_high;
        }
    }
}

pub fn m50b25tu_config() -> PlantConfig<CYLINDERS> {
    let base = PlantConfig::<4>::default_four();
    PlantConfig {
        cylinder_count: CYLINDERS as u8,
        cylinder_phase_deg10: [
            CrankDeg10(0),
            CrankDeg10(4800),
            CrankDeg10(2400),
            CrankDeg10(6000),
            CrankDeg10(1200),
            CrankDeg10(3600),
        ],
        displacement_cc: 2494,
        engine: ecu_sim_core::EngineGeometryConfig {
            displacement_cc: 2494,
            bore_um: 84_000,
            stroke_um: 75_000,
            ..base.engine
        },
        compression_ratio_x100: base.compression_ratio_x100,
        crank_inertia_x1000: base.crank_inertia_x1000,
        friction_torque_nm_x100: base.friction_torque_nm_x100,
        starter_torque_nm_x100: base.starter_torque_nm_x100,
        physics_mode: base.physics_mode,
        crank: base.crank,
        trigger: TriggerConfig {
            crank_teeth: 60,
            missing_teeth: 2,
            cam_pulses: 1,
        },
        air: ecu_sim_core::AirConfig {
            manifold_volume_cc: 2494,
            ..base.air
        },
        fuel: base.fuel,
        spark: base.spark,
        combustion: base.combustion,
        valve_events: base.valve_events,
        residual: base.residual,
        thermo: base.thermo,
        losses: base.losses,
        knock: base.knock,
        dyno: base.dyno,
        sensors: base.sensors,
    }
}

pub fn run_plant_step_through_avr(
    plant: &mut Plant<CYLINDERS, MAX_EDGES, MAX_EVENTS>,
    input: PlantStepInput<CYLINDERS, MAX_EVENTS>,
    avr: &mut AvrM5xHarness,
    output: &mut PlantStepOutput<CYLINDERS, MAX_EDGES, MAX_EVENTS>,
) {
    plant.step(&input, output).unwrap();
    avr.drive_map_kpa10(output.sensors.map_kpa10.0);

    for edge in output.trigger_edges.as_slice() {
        avr.feed_trigger_edge(*edge);
    }

    avr.finish_step(
        output.sensors.timestamp_us.0,
        output.sensors.crank_angle_deg10,
    );
}

pub fn transitions_to_output_frame(
    transitions: &[HarnessTransition],
) -> EcuOutputFrame<CYLINDERS, MAX_EVENTS> {
    let mut frame = EcuOutputFrame::empty();
    let mut injector_high_at = [None; CYLINDERS];
    let mut ignition_high_at = [None; CYLINDERS];
    let mut injector_angle = [CrankDeg10(0); CYLINDERS];
    let mut ignition_angle = [CrankDeg10(0); CYLINDERS];

    for transition in transitions {
        match transition.output {
            HarnessOutput::Injector(channel) => {
                pair_injection_transition(
                    &mut frame.injection_events,
                    &mut injector_high_at,
                    &mut injector_angle,
                    channel as usize,
                    *transition,
                );
            }
            HarnessOutput::Ignition(channel) => {
                pair_ignition_transition(
                    &mut frame.spark_events,
                    &mut ignition_high_at,
                    &mut ignition_angle,
                    channel as usize,
                    *transition,
                );
            }
        }
    }

    frame
}

fn pair_injection_transition(
    events: &mut FixedSlice<InjectionCommand, MAX_EVENTS>,
    high_at: &mut [Option<u32>; CYLINDERS],
    angle_at: &mut [CrankDeg10; CYLINDERS],
    channel: usize,
    transition: HarnessTransition,
) {
    if channel >= CYLINDERS {
        return;
    }
    if transition.high {
        high_at[channel] = Some(transition.at_us);
        angle_at[channel] = transition.angle_deg10;
        return;
    }
    let Some(start_us) = high_at[channel].take() else {
        return;
    };
    let width_us = transition.at_us.saturating_sub(start_us).max(1_000);
    let _ = events.push(InjectionCommand {
        cylinder: CylinderIndex(channel as u8),
        mode: InjectionTimingMode::StartOfInjection,
        angle_deg10: angle_at[channel],
        pulse_width_us: Micros(width_us),
        injector_flow_ug_per_us: MicrogramsPerMicros(5),
        deadtime_us: Micros(0),
    });
}

fn pair_ignition_transition(
    events: &mut FixedSlice<SparkCommand, MAX_EVENTS>,
    high_at: &mut [Option<u32>; CYLINDERS],
    angle_at: &mut [CrankDeg10; CYLINDERS],
    channel: usize,
    transition: HarnessTransition,
) {
    if channel >= CYLINDERS {
        return;
    }
    if transition.high {
        high_at[channel] = Some(transition.at_us);
        angle_at[channel] = transition.angle_deg10;
        return;
    }
    let Some(start_us) = high_at[channel].take() else {
        return;
    };
    let width_us = transition.at_us.saturating_sub(start_us).max(1_000);
    let _ = events.push(SparkCommand {
        cylinder: CylinderIndex(channel as u8),
        spark_angle_deg10: angle_at[channel],
        dwell_us: Micros(width_us),
        coil_energy_x1000: 1_000,
    });
}

fn map_kpa10_to_millivolts(map_kpa10: u16) -> u32 {
    let clamped = map_kpa10.clamp(200, 4000) as u32;
    250 + (clamped - 200) * 4500 / 3800
}

pub fn cranking_input(dt_us: u32) -> PlantStepInput<CYLINDERS, MAX_EVENTS> {
    let mut input = PlantStepInput::idle(Micros(dt_us));
    input.driver.starter_enabled = true;
    input.driver.throttle_x1000 = 350;
    input.driver.load_torque_nm_x100 = TorqueNmX100(0);
    input
}

pub fn running_input(
    dt_us: u32,
    ecu_outputs: EcuOutputFrame<CYLINDERS, MAX_EVENTS>,
) -> PlantStepInput<CYLINDERS, MAX_EVENTS> {
    let mut input = cranking_input(dt_us);
    input.ecu_outputs = ecu_outputs;
    input
}
