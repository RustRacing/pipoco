//! Canonical sensor plausibility: debounced latch on out-of-range, cross-check,
//! and stuck-value faults. Ported from the frozen spec oracle (review 011).

/// Fault must persist this long (us) before it latches.
pub const PLAUSIBILITY_DEBOUNCE_US: u32 = 500_000;
/// Plausibility gate is only active at or above this RPM.
pub const PLAUSIBILITY_MIN_RPM: u16 = 1000;

/// Raw sensor snapshot fed to the plausibility latch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct PlausibilityInput {
    pub t_us: u32,
    pub rpm: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub maf_x100: u16,
    pub o2_afr_x100: u16,
    pub knock_intensity_x100: u16,
    pub baro_kpa10: u16,
    pub vbat_mv: u16,
}

/// Debounce-latch state across plausibility steps.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct PlausibilityState {
    pub initialized: bool,
    pub last_t_us: u32,
    pub last_clt_c10: i16,
    pub last_iat_c10: i16,
    pub last_map_kpa10: u16,
    pub last_tps_x100: u16,
    pub last_maf_x100: u16,
    pub last_o2_afr_x100: u16,
    pub last_knock_intensity_x100: u16,
    pub last_baro_kpa10: u16,
    pub last_vbat_mv: u16,
    pub assert_counter_us: u32,
    pub clear_counter_us: u32,
    pub latched: bool,
}

/// Result of one plausibility step: next state plus the latched verdict.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PlausibilityResult {
    pub next_state: PlausibilityState,
    pub latched: bool,
}

fn in_range_i16(value: i16, lo: i16, hi: i16) -> bool {
    value >= lo && value <= hi
}

fn in_range_u16(value: u16, lo: u16, hi: u16) -> bool {
    value >= lo && value <= hi
}

fn current_fault_present(input: PlausibilityInput, state: &PlausibilityState) -> bool {
    let out_of_range = !in_range_i16(input.clt_c10, -400, 1500)
        || !in_range_i16(input.iat_c10, -400, 1200)
        || !in_range_u16(input.map_kpa10, 100, 3000)
        || !in_range_u16(input.tps_x100, 0, 10_000)
        || !in_range_u16(input.maf_x100, 0, 60_000)
        || !in_range_u16(input.o2_afr_x100, 500, 3000)
        || !in_range_u16(input.knock_intensity_x100, 0, 10_000)
        || !in_range_u16(input.baro_kpa10, 500, 1200)
        || !in_range_u16(input.vbat_mv, 6000, 18_000);

    // Thresholds are in percent-x100 (tps_x100 is percent x100).
    let high_tps_low_map = input.tps_x100 >= 8_000 && input.map_kpa10 <= 300;
    let low_tps_high_map = input.tps_x100 <= 1_000 && input.map_kpa10 >= 950;

    let stuck_values = state.initialized
        && state.last_clt_c10 == input.clt_c10
        && state.last_iat_c10 == input.iat_c10
        && state.last_map_kpa10 == input.map_kpa10
        && state.last_tps_x100 == input.tps_x100
        && state.last_maf_x100 == input.maf_x100
        && state.last_o2_afr_x100 == input.o2_afr_x100
        && state.last_knock_intensity_x100 == input.knock_intensity_x100
        && state.last_baro_kpa10 == input.baro_kpa10
        && state.last_vbat_mv == input.vbat_mv;

    out_of_range || high_tps_low_map || low_tps_high_map || stuck_values
}

/// Advance the plausibility latch with one raw sensor snapshot.
pub fn plausibility_step(
    input: PlausibilityInput,
    previous: PlausibilityState,
) -> PlausibilityResult {
    let mut next = previous;

    let dt_us = if previous.initialized {
        input.t_us.wrapping_sub(previous.last_t_us)
    } else {
        0
    };

    let gate_enabled = input.rpm >= PLAUSIBILITY_MIN_RPM;
    if gate_enabled {
        let fault = current_fault_present(input, &previous);
        if fault {
            next.assert_counter_us = next.assert_counter_us.saturating_add(dt_us);
            if next.assert_counter_us > PLAUSIBILITY_DEBOUNCE_US {
                next.assert_counter_us = PLAUSIBILITY_DEBOUNCE_US;
            }
            next.clear_counter_us = 0;
            if next.assert_counter_us >= PLAUSIBILITY_DEBOUNCE_US {
                next.latched = true;
            }
        } else {
            next.clear_counter_us = next.clear_counter_us.saturating_add(dt_us);
            if next.clear_counter_us > PLAUSIBILITY_DEBOUNCE_US {
                next.clear_counter_us = PLAUSIBILITY_DEBOUNCE_US;
            }
            next.assert_counter_us = 0;
            if next.clear_counter_us >= PLAUSIBILITY_DEBOUNCE_US {
                next.latched = false;
            }
        }
    }

    next.initialized = true;
    next.last_t_us = input.t_us;
    next.last_clt_c10 = input.clt_c10;
    next.last_iat_c10 = input.iat_c10;
    next.last_map_kpa10 = input.map_kpa10;
    next.last_tps_x100 = input.tps_x100;
    next.last_maf_x100 = input.maf_x100;
    next.last_o2_afr_x100 = input.o2_afr_x100;
    next.last_knock_intensity_x100 = input.knock_intensity_x100;
    next.last_baro_kpa10 = input.baro_kpa10;
    next.last_vbat_mv = input.vbat_mv;

    PlausibilityResult {
        next_state: next,
        latched: next.latched,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal_input(t_us: u32) -> PlausibilityInput {
        PlausibilityInput {
            t_us,
            rpm: 2000,
            clt_c10: 800,
            iat_c10: 250,
            map_kpa10: 1000,
            tps_x100: 2500,
            maf_x100: 12000,
            o2_afr_x100: 1470,
            knock_intensity_x100: 100,
            baro_kpa10: 1000,
            vbat_mv: 12000,
        }
    }

    #[test]
    fn out_of_range_fault_latches_after_debounce() {
        let state0 = PlausibilityState::default();
        let mut bad = nominal_input(0);
        bad.map_kpa10 = 50;

        let step0 = plausibility_step(bad, state0);
        assert!(!step0.latched);

        let mut bad_late = bad;
        bad_late.t_us = 500_000;
        let step1 = plausibility_step(bad_late, step0.next_state);
        assert!(step1.latched);
    }

    #[test]
    fn stuck_fault_latches_after_debounce_and_clears_after_debounce() {
        let input0 = nominal_input(0);
        let step0 = plausibility_step(input0, PlausibilityState::default());
        assert!(!step0.latched);

        let input1 = nominal_input(500_000);
        let step1 = plausibility_step(input1, step0.next_state);
        assert!(step1.latched);

        let mut input2 = nominal_input(1_000_000);
        input2.map_kpa10 = 1010;
        let step2 = plausibility_step(input2, step1.next_state);
        assert!(!step2.latched);
    }

    #[test]
    fn gate_disabled_below_min_rpm() {
        let mut input = nominal_input(0);
        input.rpm = 900;
        input.map_kpa10 = 50;

        let step = plausibility_step(input, PlausibilityState::default());
        assert!(!step.latched);
    }
}
