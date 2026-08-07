//! Canonical engine-angle math (review 007 / ADR 0012).
//!
//! Single source for cycle constants, normalization, forward-angle deltas,
//! and duration-to-angle conversion. `ecu-spec` (oracle) and `ecu-scheduler`
//! both consume these; no crate re-implements them.

use crate::{Degrees10, PulseWidthUs, Rpm};

/// One full 720° engine cycle in tenths of a degree.
pub const ENGINE_CYCLE_DEGREES10: u16 = 7200;
/// One crank revolution in tenths of a degree.
pub const CRANK_REV_DEGREES10: u16 = 3600;

/// Normalize an angle into `[0, 7200)` as a `Degrees10` (spec-compatible).
pub const fn norm7200(value: i32) -> Degrees10 {
    let mut normalized = value % (ENGINE_CYCLE_DEGREES10 as i32);
    if normalized < 0 {
        normalized += ENGINE_CYCLE_DEGREES10 as i32;
    }
    Degrees10::new(normalized as i16)
}

/// Shortest cyclic distance between two angles within one 720° cycle.
pub const fn cyc7200_distance(a: Degrees10, b: Degrees10) -> u16 {
    let a = a.get() as i32;
    let b = b.get() as i32;
    let forward = (b - a).rem_euclid(ENGINE_CYCLE_DEGREES10 as i32) as u16;
    let backward = ENGINE_CYCLE_DEGREES10 - forward;
    if forward <= backward {
        forward
    } else {
        backward
    }
}

/// Angle swept by a pulse of `pw_us` at `rpm` (tenths of a degree).
pub const fn duration_us_to_deg10(pw_us: PulseWidthUs, rpm: Rpm) -> Degrees10 {
    let pwm = pw_us.get() as u64;
    let rpm = rpm.get() as u64;
    let deg10 = (pwm * rpm * 6) / 100_000;
    Degrees10::new(deg10 as i16)
}

/// Normalize an angle into `[0, modulo)` (scheduler-compatible).
pub const fn norm_deg10(value: i32, modulo: u16) -> u16 {
    value.rem_euclid(modulo as i32) as u16
}

/// Forward (clockwise) angular distance from `current` to `target` within
/// `modulo` tenths of a degree.
pub const fn forward_angle_delta_deg10(current: u16, target: u16, modulo: u16) -> u16 {
    if target > current {
        target - current
    } else {
        modulo - current + target
    }
}

/// Time (us) to sweep `delta_deg10` at `rpm`, clamped to `[1, u32::MAX]`.
/// Zero RPM yields `0` (engine not turning).
pub const fn micros_for_angle_delta(delta_deg10: u16, rpm: Rpm) -> u32 {
    let rpm = rpm.get() as u64;
    if rpm == 0 {
        return 0;
    }

    let micros = 60_000_000u64 * (delta_deg10 as u64) / ((CRANK_REV_DEGREES10 as u64) * rpm);
    let mut micros = micros;
    if micros < 1 {
        micros = 1;
    }
    if micros > u32::MAX as u64 {
        micros = u32::MAX as u64;
    }
    micros as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn norm7200_wraps_into_cycle() {
        assert_eq!(norm7200(0), Degrees10::new(0));
        assert_eq!(norm7200(7199), Degrees10::new(7199));
        assert_eq!(norm7200(7200), Degrees10::new(0));
        assert_eq!(norm7200(-1), Degrees10::new(7199));
        assert_eq!(norm7200(-7200), Degrees10::new(0));
    }

    #[test]
    fn cyc7200_distance_is_symmetric_and_bounded() {
        let a = Degrees10::new(100);
        let b = Degrees10::new(7100); // 100 before the 7200 wrap
        assert_eq!(cyc7200_distance(a, b), cyc7200_distance(b, a));
        assert_eq!(cyc7200_distance(a, b), 200);
        assert!(cyc7200_distance(a, b) <= 3600);
    }

    #[test]
    fn duration_us_to_deg10_matches_integer_formula() {
        let deg10 = duration_us_to_deg10(PulseWidthUs::new(10_000), Rpm::new(6_000));
        assert_eq!(deg10.get(), 3600); // 10ms at 6000 rpm = half a rev
    }

    #[test]
    fn norm_deg10_and_forward_delta_round_trip() {
        let current = norm_deg10(7_200 + 3_500, ENGINE_CYCLE_DEGREES10);
        assert_eq!(current, 3_500);

        let delta = forward_angle_delta_deg10(3_500, 100, ENGINE_CYCLE_DEGREES10);
        assert_eq!(delta, 3_800); // wraps forward across 0
    }

    #[test]
    fn micros_for_angle_delta_scales_with_rpm() {
        assert_eq!(micros_for_angle_delta(0, Rpm::new(6_000)), 1); // clamped to a floor of 1us
        assert_eq!(micros_for_angle_delta(3600, Rpm::new(6_000)), 10_000);
        assert_eq!(micros_for_angle_delta(3600, Rpm::new(12_000)), 5_000);
        assert_eq!(micros_for_angle_delta(3600, Rpm::new(0)), 0);
    }
}
