use crate::{config::ValveEvents, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidualGasModel {
    pub residual_fraction_x1000: u16,
    pub previous_lambda_x1000: u16,
    pub previous_temp_k_x10: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ResidualGasInput {
    pub base_fraction_x1000: u16,
    pub overlap_gain_x1000: u16,
    pub low_map_gain_x1000: u16,
    pub exhaust_backpressure_gain_x1000: u16,
    pub scavenging_gain_x1000: u16,
    pub valve_events: ValveEvents,
    pub map_kpa10: Kpa10,
    pub exhaust_pressure_kpa10: Kpa10,
    pub rpm: Rpm,
    pub previous_lambda_x1000: u16,
    pub previous_temp_k_x10: u16,
}

pub fn estimate_residual_gas(input: ResidualGasInput) -> ResidualGasModel {
    let overlap_deg10 = valve_overlap_deg10(input.valve_events) as u32;
    let low_map_kpa10 = 1000u16.saturating_sub(input.map_kpa10.0) as u32;
    let exhaust_excess_kpa10 = input
        .exhaust_pressure_kpa10
        .0
        .saturating_sub(input.map_kpa10.0) as u32;
    let scavenging = (input.rpm.0 / 100).min(80);
    let residual = input.base_fraction_x1000 as i64
        + (overlap_deg10 as i64 * input.overlap_gain_x1000 as i64 / 1000)
        + (low_map_kpa10 as i64 * input.low_map_gain_x1000 as i64 / 1000)
        + (exhaust_excess_kpa10 as i64 * input.exhaust_backpressure_gain_x1000 as i64 / 1000)
        - (scavenging as i64 * input.scavenging_gain_x1000 as i64 / 1000);

    ResidualGasModel {
        residual_fraction_x1000: residual.clamp(0, 700) as u16,
        previous_lambda_x1000: input.previous_lambda_x1000,
        previous_temp_k_x10: input.previous_temp_k_x10,
    }
}

pub fn apply_residual_to_fresh_air(air_mass: MassUg, residual_fraction_x1000: u16) -> MassUg {
    let fresh_fraction = 1000u32.saturating_sub(residual_fraction_x1000.min(1000) as u32);
    MassUg((air_mass.0 as u64 * fresh_fraction as u64 / 1000).min(u32::MAX as u64) as u32)
}

pub fn residual_combustion_quality_x1000(residual_fraction_x1000: u16) -> u16 {
    1000u16.saturating_sub(residual_fraction_x1000.min(900) / 2)
}

pub fn valve_overlap_deg10(events: ValveEvents) -> u16 {
    let overlap = events.ivo_deg_btdc_x10.max(0) as i32 + events.evc_deg_atdc_x10.max(0) as i32;
    overlap.clamp(0, u16::MAX as i32) as u16
}

#[cfg(test)]
mod tests {
    use super::*;

    fn events(ivo: i16, evc: i16) -> ValveEvents {
        ValveEvents {
            ivo_deg_btdc_x10: ivo,
            ivc_deg_abdc_x10: 500,
            evo_deg_bbdc_x10: 500,
            evc_deg_atdc_x10: evc,
            intake_lift_mm_x100: 900,
            exhaust_lift_mm_x100: 850,
            intake_duration_deg_x10: 2400,
            exhaust_duration_deg_x10: 2350,
        }
    }

    #[test]
    fn residual_overlap_low_load_increases_fraction_and_reduces_fresh_air() {
        let mild = estimate_residual_gas(ResidualGasInput {
            base_fraction_x1000: 50,
            overlap_gain_x1000: 8,
            low_map_gain_x1000: 4,
            exhaust_backpressure_gain_x1000: 1,
            scavenging_gain_x1000: 5,
            valve_events: events(0, 0),
            map_kpa10: Kpa10(900),
            exhaust_pressure_kpa10: Kpa10(1000),
            rpm: Rpm(2500),
            previous_lambda_x1000: 1000,
            previous_temp_k_x10: 2930,
        });
        let overlapped = estimate_residual_gas(ResidualGasInput {
            valve_events: events(350, 250),
            map_kpa10: Kpa10(350),
            ..ResidualGasInput {
                base_fraction_x1000: 50,
                overlap_gain_x1000: 8,
                low_map_gain_x1000: 4,
                exhaust_backpressure_gain_x1000: 1,
                scavenging_gain_x1000: 5,
                valve_events: events(0, 0),
                map_kpa10: Kpa10(900),
                exhaust_pressure_kpa10: Kpa10(1000),
                rpm: Rpm(2500),
                previous_lambda_x1000: 1000,
                previous_temp_k_x10: 2930,
            }
        });

        assert!(overlapped.residual_fraction_x1000 > mild.residual_fraction_x1000);
        assert!(
            apply_residual_to_fresh_air(MassUg(10_000), overlapped.residual_fraction_x1000).0
                < apply_residual_to_fresh_air(MassUg(10_000), mild.residual_fraction_x1000).0
        );
        assert!(
            residual_combustion_quality_x1000(overlapped.residual_fraction_x1000)
                < residual_combustion_quality_x1000(mild.residual_fraction_x1000)
        );
    }
}
