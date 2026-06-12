use crate::{config::BurnCurve, config::WIEBE_POINTS, types::*};

pub fn burn_fraction_at_elapsed_deg10(
    curve: &BurnCurve,
    elapsed_deg10: u16,
    duration_deg10: u16,
) -> u16 {
    if duration_deg10 == 0 || elapsed_deg10 >= duration_deg10 {
        return 10000;
    }

    let segments = (WIEBE_POINTS - 1) as u32;
    let scaled = elapsed_deg10 as u32 * segments;
    let index = (scaled / duration_deg10 as u32) as usize;
    let remainder = scaled % duration_deg10 as u32;
    let start = curve.burn_fraction_x10000[index] as u32;
    let end = curve.burn_fraction_x10000[index + 1] as u32;

    (start + (end - start) * remainder / duration_deg10 as u32) as u16
}

pub fn crank_angle_after_tdc_for_burn_fraction(
    curve: &BurnCurve,
    target_x10000: u16,
    duration_deg10: u16,
) -> Option<Degrees10> {
    if target_x10000 > 10000 || duration_deg10 == 0 {
        return None;
    }
    if target_x10000 == 0 {
        return Some(Degrees10(0));
    }

    let mut i = 1;
    while i < WIEBE_POINTS {
        let previous = curve.burn_fraction_x10000[i - 1];
        let current = curve.burn_fraction_x10000[i];
        if current >= target_x10000 {
            let segment_width = duration_deg10 as u32 / (WIEBE_POINTS - 1) as u32;
            let segment_start = (i as u32 - 1) * segment_width;
            let span = current.saturating_sub(previous).max(1) as u32;
            let into_segment = target_x10000.saturating_sub(previous) as u32 * segment_width / span;
            return Some(Degrees10((segment_start + into_segment) as i16));
        }
        i += 1;
    }

    Some(Degrees10(duration_deg10 as i16))
}

pub fn elapsed_burn_angle_deg10(local_angle: CrankDeg10, start_of_combustion: CrankDeg10) -> u16 {
    normalize_deg10_i32(local_angle.0 as i32 - start_of_combustion.0 as i32).0
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::BurnCurve;

    #[test]
    fn burn_fraction_starts_at_zero_and_finishes_at_one() {
        let curve = BurnCurve::default_hifi_generated();

        assert_eq!(burn_fraction_at_elapsed_deg10(&curve, 0, 450), 0);
        assert_eq!(burn_fraction_at_elapsed_deg10(&curve, 450, 450), 10000);
        assert_eq!(burn_fraction_at_elapsed_deg10(&curve, 900, 450), 10000);
    }

    #[test]
    fn burn_fraction_is_monotonic_over_duration() {
        let curve = BurnCurve::default_hifi_generated();
        let mut previous = 0;
        let mut elapsed = 0;
        while elapsed <= 450 {
            let value = burn_fraction_at_elapsed_deg10(&curve, elapsed, 450);
            assert!(value >= previous);
            previous = value;
            elapsed += 5;
        }
    }

    #[test]
    fn ca50_is_inside_burn_duration() {
        let curve = BurnCurve::default_hifi_generated();
        let ca50 = crank_angle_after_tdc_for_burn_fraction(&curve, 5000, 450).unwrap();

        assert!(ca50.0 > 0);
        assert!(ca50.0 < 450);
    }

    #[test]
    fn elapsed_burn_angle_wraps_over_cycle_boundary() {
        assert_eq!(
            elapsed_burn_angle_deg10(CrankDeg10(50), CrankDeg10(7100)),
            150
        );
    }
}
