use ecu_board_api::{OutputLevel, OutputTransitionBatch};
use ecu_sim_core::fuel::{InjectionCommand, InjectionTimingMode};
use ecu_sim_core::prelude::{
    CrankDeg10, CylinderIndex, EcuOutputFrame, MicrogramsPerMicros, Micros,
};
use ecu_sim_core::spark::SparkCommand;

use crate::output_validation::{validate_x86_output_channel, OutputChannelKind};
use crate::plant_bridge::X86PlantBridgeDiagnostics;

use super::types::X86RuntimePlantBridgeFrame;
use super::{
    X86_OUTPUT_TRANSITION_CAP, X86_PLANT_BRIDGE_COIL_ENERGY_X1000,
    X86_PLANT_BRIDGE_INJECTOR_FLOW_UG_PER_US, X86_PLANT_BRIDGE_MIN_INJECTION_PULSE_US,
    X86_PLANT_BRIDGE_MIN_SPARK_DWELL_US,
};

pub fn bridge_output_transitions_to_core_frame<const CYL: usize, const MAX_EVENTS: usize>(
    transitions: &OutputTransitionBatch<X86_OUTPUT_TRANSITION_CAP>,
) -> X86RuntimePlantBridgeFrame<CYL, MAX_EVENTS> {
    let mut frame = X86RuntimePlantBridgeFrame {
        ecu_outputs: EcuOutputFrame::<CYL, MAX_EVENTS>::empty(),
        diagnostics: X86PlantBridgeDiagnostics::empty(),
    };
    let mut injector_high_at_us = [None; CYL];
    let mut ignition_high_at_us = [None; CYL];

    for transition in transitions.iter() {
        let Some(validated) = validate_x86_output_channel::<CYL>(transition.output) else {
            frame.diagnostics.out_of_range_channel_count += 1;
            continue;
        };
        let channel = usize::from(validated.channel);
        let (high_at_us, is_injector) = match validated.kind {
            OutputChannelKind::Injector => (&mut injector_high_at_us, true),
            OutputChannelKind::Ignition => (&mut ignition_high_at_us, false),
        };

        match transition.level {
            OutputLevel::High => {
                if high_at_us[channel].is_some() {
                    frame.diagnostics.duplicate_high_count += 1;
                } else {
                    high_at_us[channel] = Some(transition.at.get());
                }
            }
            OutputLevel::Low => {
                let Some(start_us) = high_at_us[channel].take() else {
                    frame.diagnostics.orphan_low_count += 1;
                    continue;
                };
                let end_us = transition.at.get();
                if end_us < start_us {
                    frame.diagnostics.out_of_order_transition_count += 1;
                    continue;
                }

                let width_us = end_us - start_us;
                if is_injector {
                    if width_us < X86_PLANT_BRIDGE_MIN_INJECTION_PULSE_US {
                        frame.diagnostics.short_pulse_width_count += 1;
                    }
                    let pulse_width_us = width_us.max(X86_PLANT_BRIDGE_MIN_INJECTION_PULSE_US);
                    // The x86 board path only exposes pulse width, so the
                    // bridge uses a fixed nominal injector flow to convert
                    // that width into a plant fuel mass. Keep the value
                    // stable so the test remains deterministic.
                    let command = InjectionCommand {
                        cylinder: CylinderIndex(channel as u8),
                        mode: InjectionTimingMode::StartOfInjection,
                        angle_deg10: CrankDeg10(0),
                        pulse_width_us: Micros(pulse_width_us),
                        injector_flow_ug_per_us: MicrogramsPerMicros(
                            X86_PLANT_BRIDGE_INJECTOR_FLOW_UG_PER_US,
                        ),
                        deadtime_us: Micros(0),
                    };
                    if frame.ecu_outputs.injection_events.push(command).is_err() {
                        frame.diagnostics.capacity_overflow_count += 1;
                    }
                } else {
                    if width_us < X86_PLANT_BRIDGE_MIN_SPARK_DWELL_US {
                        frame.diagnostics.short_dwell_count += 1;
                    }
                    let dwell_us = width_us.max(X86_PLANT_BRIDGE_MIN_SPARK_DWELL_US);
                    let command = SparkCommand {
                        cylinder: CylinderIndex(channel as u8),
                        spark_angle_deg10: CrankDeg10(0),
                        dwell_us: Micros(dwell_us),
                        coil_energy_x1000: X86_PLANT_BRIDGE_COIL_ENERGY_X1000,
                    };
                    if frame.ecu_outputs.spark_events.push(command).is_err() {
                        frame.diagnostics.capacity_overflow_count += 1;
                    }
                }
            }
        }
    }

    frame.diagnostics.open_high_count = injector_high_at_us
        .iter()
        .chain(ignition_high_at_us.iter())
        .filter(|entry| entry.is_some())
        .count() as u32;

    frame
}
