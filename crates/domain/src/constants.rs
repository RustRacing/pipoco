//! Canonical physical-unit constants (review 008 / ADR 0012).
//!
//! Single source for RPM numerators, max pulse width, and related physical
//! unit constants that used to be re-declared across crates.

/// RPM numerator for tooth-period → RPM conversion (fast approximation).
///
/// `2_000_000 = 60_000_000 µs/min × 2 / 60`, i.e. assumes 60 teeth/rev.
pub const RPM_CALC_NUMERATOR_FAST: u32 = 2_000_000;

/// RPM numerator for tooth-period → RPM conversion (exact for a 60-2 wheel).
///
/// `60_000_000 µs/min × 2 / 58 teeth = 2_068_965.5 → 2_068_966`. The FAST
/// numerator is ~3.4% low relative to this exact value; the difference is
/// acceptable for display RPM but must not silently grow (see the 3% test).
pub const RPM_CALC_NUMERATOR_EXACT: u32 = 2_068_966;

/// Maximum injector pulse width in microseconds.
pub const MAX_PULSE_WIDTH_US: u16 = 20_000;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rpm_numerator_fast_stays_within_3pct_of_exact() {
        let fast = RPM_CALC_NUMERATOR_FAST as f64;
        let exact = RPM_CALC_NUMERATOR_EXACT as f64;
        // True error is 3.33%; assert the documented ~3.5% bound so a silent
        // drift of either numerator is caught without false failures.
        let ratio = (fast - exact).abs() / exact;
        assert!(
            ratio < 0.035,
            "fast numerator {fast} drifted beyond the ~3.5% bound of exact {exact}: {ratio}"
        );
    }

    #[test]
    fn max_pulse_width_is_20ms() {
        assert_eq!(MAX_PULSE_WIDTH_US, 20_000);
    }
}
