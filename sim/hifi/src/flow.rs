use core::f64::consts::PI;
const ENGINE_CYCLE_RAD: f64 = 4.0 * core::f64::consts::PI;

use crate::params::{ThrottleConfig, ValveTiming};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FlowStation {
    pub pressure_pa: f64,
    pub temperature_k: f64,
    pub gamma: f64,
    pub gas_constant_j_per_kg_k: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrificeFlowResult {
    pub mass_flow_kg_per_s: f64,
    pub pressure_ratio: f64,
    pub choked: bool,
    pub upstream_is_a: bool,
}

pub fn critical_pressure_ratio(gamma: f64) -> f64 {
    (2.0 / (gamma + 1.0)).powf(gamma / (gamma - 1.0))
}

pub fn compressible_orifice_flow(
    station_a: FlowStation,
    station_b: FlowStation,
    effective_area_m2: f64,
    discharge_coefficient: f64,
) -> OrificeFlowResult {
    if effective_area_m2 <= 0.0 || discharge_coefficient <= 0.0 {
        return OrificeFlowResult {
            mass_flow_kg_per_s: 0.0,
            pressure_ratio: 1.0,
            choked: false,
            upstream_is_a: true,
        };
    }

    let (upstream, downstream, upstream_is_a) = if station_a.pressure_pa >= station_b.pressure_pa {
        (station_a, station_b, true)
    } else {
        (station_b, station_a, false)
    };

    let pressure_ratio = (downstream.pressure_pa / upstream.pressure_pa).clamp(0.0, 1.0);
    let critical_ratio = critical_pressure_ratio(upstream.gamma);
    let choked = pressure_ratio <= critical_ratio;
    let phi = if choked {
        choked_flow_function(upstream.gamma)
    } else {
        subcritical_flow_function(pressure_ratio, upstream.gamma)
    };

    let magnitude = discharge_coefficient
        * effective_area_m2
        * (upstream.pressure_pa
            / (upstream.gas_constant_j_per_kg_k * upstream.temperature_k).sqrt())
        * phi;

    OrificeFlowResult {
        mass_flow_kg_per_s: if upstream_is_a { magnitude } else { -magnitude },
        pressure_ratio,
        choked,
        upstream_is_a,
    }
}

pub fn choked_flow_function(gamma: f64) -> f64 {
    gamma.sqrt() * (2.0 / (gamma + 1.0)).powf((gamma + 1.0) / (2.0 * (gamma - 1.0)))
}

pub fn subcritical_flow_function(pressure_ratio: f64, gamma: f64) -> f64 {
    let exponent = 2.0 / gamma;
    let exponent_minus = (gamma + 1.0) / gamma;
    ((2.0 * gamma / (gamma - 1.0))
        * (pressure_ratio.powf(exponent) - pressure_ratio.powf(exponent_minus)))
    .sqrt()
}

pub fn valve_lift_half_sine(theta_rad: f64, timing: ValveTiming) -> f64 {
    let duration_rad =
        (timing.close_angle_rad - timing.open_angle_rad).rem_euclid(ENGINE_CYCLE_RAD);
    if duration_rad <= 0.0 {
        return 0.0;
    }
    let phase_rad = (theta_rad - timing.open_angle_rad).rem_euclid(ENGINE_CYCLE_RAD);
    if phase_rad >= duration_rad {
        return 0.0;
    }
    let phase = phase_rad / duration_rad;
    timing.max_lift_m * (PI * phase).sin()
}

pub fn valve_effective_area(theta_rad: f64, timing: ValveTiming) -> f64 {
    let lift_m = valve_lift_half_sine(theta_rad, timing);
    let curtain_area = PI * timing.seat_diameter_m * lift_m;
    curtain_area.min(timing.seat_area_m2())
}

pub fn throttle_effective_area(config: ThrottleConfig, throttle_position: f64) -> f64 {
    config.max_area_m2 * throttle_position.clamp(0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn station(pressure_pa: f64, temperature_k: f64) -> FlowStation {
        FlowStation {
            pressure_pa,
            temperature_k,
            gamma: 1.35,
            gas_constant_j_per_kg_k: 287.0,
        }
    }

    #[test]
    fn choked_flow_caps_mass_rate_below_critical_pressure_ratio() {
        let area = 1.0e-4;
        let cd = 0.8;
        let upstream = station(200_000.0, 300.0);

        let near_critical =
            compressible_orifice_flow(upstream, station(100_000.0, 300.0), area, cd);
        let deeper_drop = compressible_orifice_flow(upstream, station(50_000.0, 300.0), area, cd);

        assert!(near_critical.choked);
        assert!(deeper_drop.choked);
        assert!(deeper_drop.mass_flow_kg_per_s >= near_critical.mass_flow_kg_per_s * 0.98);
    }

    #[test]
    fn subcritical_flow_increases_as_pressure_drop_grows_above_critical_ratio() {
        let area = 1.0e-4;
        let cd = 0.8;
        let upstream = station(150_000.0, 300.0);

        let small_drop = compressible_orifice_flow(upstream, station(140_000.0, 300.0), area, cd);
        let medium_drop = compressible_orifice_flow(upstream, station(120_000.0, 300.0), area, cd);

        assert!(!small_drop.choked);
        assert!(!medium_drop.choked);
        assert!(medium_drop.mass_flow_kg_per_s > small_drop.mass_flow_kg_per_s);
    }

    #[test]
    fn backflow_changes_sign_when_downstream_pressure_exceeds_upstream() {
        let result = compressible_orifice_flow(
            station(90_000.0, 320.0),
            station(110_000.0, 320.0),
            8.0e-5,
            0.75,
        );

        assert!(result.mass_flow_kg_per_s < 0.0);
        assert!(!result.upstream_is_a);
    }

    #[test]
    fn valve_area_uses_half_sine_lift_and_caps_at_seat_area() {
        let timing = ValveTiming {
            open_angle_rad: 0.0,
            close_angle_rad: core::f64::consts::PI,
            max_lift_m: 0.02,
            seat_diameter_m: 0.01,
            discharge_coefficient: 0.7,
        };

        assert_eq!(valve_effective_area(-0.1, timing), 0.0);
        assert_eq!(
            valve_effective_area(core::f64::consts::PI + 0.1, timing),
            0.0
        );

        let mid_area = valve_effective_area(core::f64::consts::FRAC_PI_2, timing);
        assert!(mid_area > 0.0);
        assert!(mid_area <= timing.seat_area_m2());
    }

    #[test]
    fn valve_area_is_continuous_across_720_deg_seam() {
        let timing = ValveTiming {
            open_angle_rad: 600.0_f64.to_radians(),
            close_angle_rad: 120.0_f64.to_radians(),
            max_lift_m: 0.02,
            seat_diameter_m: 0.01,
            discharge_coefficient: 0.7,
        };

        let inside_window = valve_effective_area((700.0_f64 + 0.05).to_radians(), timing);
        let inside_window_next_cycle =
            valve_effective_area((700.0_f64 + 720.0 + 0.05).to_radians(), timing);
        let at_window_close = valve_effective_area((840.0_f64 - 0.05).to_radians(), timing);

        assert!((inside_window - inside_window_next_cycle).abs() < 1.0e-12);
        assert!(inside_window > 0.0);
        assert!(at_window_close > 0.0);
    }

    #[test]
    fn throttle_area_scales_with_position_and_clamps() {
        let config = ThrottleConfig {
            max_area_m2: 2.5e-4,
            discharge_coefficient: 0.8,
        };

        assert_eq!(throttle_effective_area(config, -0.5), 0.0);
        assert_eq!(throttle_effective_area(config, 0.5), 1.25e-4);
        assert_eq!(throttle_effective_area(config, 2.0), 2.5e-4);
    }
}
