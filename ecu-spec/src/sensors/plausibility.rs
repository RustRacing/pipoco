use crate::{DiagnosticCode, Micros, Rpm, TempC10};

const PLAUSIBILITY_DEBOUNCE_US: u32 = 500_000;
const PLAUSIBILITY_MIN_RPM: u16 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SensorPlausibilityInput {
    pub t_us: Micros,
    pub rpm: Rpm,
    pub clt_c10: TempC10,
    pub iat_c10: TempC10,
    pub map_kpa10: u16,
    pub tps_x100: u16,
    pub maf_x100: u16,
    pub o2_afr_x100: u16,
    pub knock_intensity_x100: u16,
    pub baro_kpa10: u16,
    pub vbat_mv: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct SensorPlausibilityState {
    pub initialized: bool,
    pub last_t_us: Micros,
    pub last_clt_c10: TempC10,
    pub last_iat_c10: TempC10,
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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorPlausibilityResult {
    pub next_state: SensorPlausibilityState,
    pub diagnostic: DiagnosticCode,
}

fn in_range_i16(value: i16, lo: i16, hi: i16) -> bool {
    value >= lo && value <= hi
}

fn in_range_u16(value: u16, lo: u16, hi: u16) -> bool {
    value >= lo && value <= hi
}

fn current_fault_present(input: SensorPlausibilityInput, state: &SensorPlausibilityState) -> bool {
    let out_of_range = !in_range_i16(input.clt_c10.get(), -400, 1500)
        || !in_range_i16(input.iat_c10.get(), -400, 1200)
        || !in_range_u16(input.map_kpa10, 100, 3000)
        || !in_range_u16(input.tps_x100, 0, 10_000)
        || !in_range_u16(input.maf_x100, 0, 60_000)
        || !in_range_u16(input.o2_afr_x100, 500, 3000)
        || !in_range_u16(input.knock_intensity_x100, 0, 10_000)
        || !in_range_u16(input.baro_kpa10, 500, 1200)
        || !in_range_u16(input.vbat_mv, 6000, 18_000);

    // Spec thresholds are in percent, while tps_x100 is percent x100.
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

pub fn sensor_plausibility_step(
    input: SensorPlausibilityInput,
    previous: SensorPlausibilityState,
) -> SensorPlausibilityResult {
    let mut next = previous;

    let dt_us = if previous.initialized {
        input.t_us.get().wrapping_sub(previous.last_t_us.get())
    } else {
        0
    };

    let gate_enabled = input.rpm.get() >= PLAUSIBILITY_MIN_RPM;
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

    let diagnostic = if next.latched {
        DiagnosticCode::SensorPlausibilityFault
    } else {
        DiagnosticCode::None
    };

    SensorPlausibilityResult {
        next_state: next,
        diagnostic,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal_input(t_us: u32) -> SensorPlausibilityInput {
        SensorPlausibilityInput {
            t_us: Micros::new(t_us),
            rpm: Rpm::new(2000),
            clt_c10: TempC10::new(800),
            iat_c10: TempC10::new(250),
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
        let state0 = SensorPlausibilityState::default();
        let mut bad = nominal_input(0);
        bad.map_kpa10 = 50;

        let step0 = sensor_plausibility_step(bad, state0);
        assert_eq!(step0.diagnostic, DiagnosticCode::None);

        let mut bad_late = bad;
        bad_late.t_us = Micros::new(500_000);
        let step1 = sensor_plausibility_step(bad_late, step0.next_state);
        assert_eq!(step1.diagnostic, DiagnosticCode::SensorPlausibilityFault);
    }

    #[test]
    fn stuck_fault_latches_after_debounce_and_clears_after_debounce() {
        let input0 = nominal_input(0);
        let step0 = sensor_plausibility_step(input0, SensorPlausibilityState::default());
        assert_eq!(step0.diagnostic, DiagnosticCode::None);

        let input1 = nominal_input(500_000);
        let step1 = sensor_plausibility_step(input1, step0.next_state);
        assert_eq!(step1.diagnostic, DiagnosticCode::SensorPlausibilityFault);

        let mut input2 = nominal_input(1_000_000);
        input2.map_kpa10 = 1010;
        let step2 = sensor_plausibility_step(input2, step1.next_state);
        assert_eq!(step2.diagnostic, DiagnosticCode::None);
    }

    #[test]
    fn gate_disabled_below_min_rpm() {
        let mut input = nominal_input(0);
        input.rpm = Rpm::new(900);
        input.map_kpa10 = 50;

        let step = sensor_plausibility_step(input, SensorPlausibilityState::default());
        assert_eq!(step.diagnostic, DiagnosticCode::None);
    }
}
