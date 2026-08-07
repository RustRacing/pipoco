use crate::{EngineMode, InputSnapshot, LogicalState, ValidatedCalibration};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DfcoResult {
    pub fuel_cut: bool,
    pub dfco_active: bool,
    pub dfco_qualify_counter: u16,
}

pub fn dfco_step(
    cal: &ValidatedCalibration,
    input: InputSnapshot,
    state: &LogicalState,
) -> DfcoResult {
    let enabled = matches!(input.mode, EngineMode::Running);
    if !enabled {
        return DfcoResult {
            fuel_cut: false,
            dfco_active: false,
            dfco_qualify_counter: 0,
        };
    }

    if state.dfco_active {
        let keep_active = input.rpm.get() > cal.0.dfco_exit_rpm.get()
            && input.tps_x100 <= cal.0.dfco_exit_tps_x100;
        let active = keep_active;
        return DfcoResult {
            fuel_cut: active,
            dfco_active: active,
            dfco_qualify_counter: 0,
        };
    }

    let qualify_now = input.rpm.get() >= cal.0.dfco_entry_rpm.get()
        && input.tps_x100 <= cal.0.dfco_entry_tps_x100
        && input.map_kpa10.get() <= cal.0.dfco_entry_map_kpa10.get();
    if !qualify_now {
        return DfcoResult {
            fuel_cut: false,
            dfco_active: false,
            dfco_qualify_counter: 0,
        };
    }

    let qualify_counter = state.dfco_qualify_counter.saturating_add(1);
    let active = qualify_counter >= cal.0.dfco_delay_cycles;
    DfcoResult {
        fuel_cut: active,
        dfco_active: active,
        dfco_qualify_counter: qualify_counter,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        default_reference_calibration, AfrOverride, EngineMode, LogicalState, Millivolts, SyncState,
    };

    fn qualifying_input() -> InputSnapshot {
        InputSnapshot {
            t_us: crate::Micros::new(0),
            rpm: crate::Rpm::new(3000),
            map_kpa10: crate::Kpa10::new(350),
            load_kpa10: crate::Kpa10::new(350),
            tps_x100: 0,
            clt_c10: crate::TempC10::new(850),
            iat_c10: crate::TempC10::new(250),
            baro_kpa10: crate::Kpa10::new(1000),
            vbatt_mv: Millivolts::new(12_000),
            knock_intensity_x100: 0,
            launch_armed: false,
            flat_shift_armed: false,
            sync: SyncState::Synced,
            fuel_cut: false,
            spark_cut: false,
            mode: EngineMode::Running,
            target_afr_override_x100: AfrOverride::None,
        }
    }

    #[test]
    fn enters_after_delay_cycles() {
        let mut cal = default_reference_calibration();
        cal.0.dfco_entry_rpm = crate::Rpm::new(2000);
        cal.0.dfco_exit_rpm = crate::Rpm::new(1800);
        cal.0.dfco_entry_tps_x100 = 200;
        cal.0.dfco_exit_tps_x100 = 300;
        cal.0.dfco_entry_map_kpa10 = crate::Kpa10::new(500);
        cal.0.dfco_delay_cycles = 2;

        let input = qualifying_input();
        let r0 = dfco_step(&cal, input, &LogicalState::default());
        assert!(!r0.fuel_cut);
        assert_eq!(r0.dfco_qualify_counter, 1);

        let state1 = LogicalState {
            dfco_active: r0.dfco_active,
            dfco_qualify_counter: r0.dfco_qualify_counter,
            ..LogicalState::default()
        };
        let r1 = dfco_step(&cal, input, &state1);
        assert!(r1.fuel_cut);
        assert!(r1.dfco_active);
    }

    #[test]
    fn exits_when_exit_hysteresis_breaks() {
        let mut cal = default_reference_calibration();
        cal.0.dfco_entry_rpm = crate::Rpm::new(2000);
        cal.0.dfco_exit_rpm = crate::Rpm::new(1800);
        cal.0.dfco_entry_tps_x100 = 200;
        cal.0.dfco_exit_tps_x100 = 300;
        cal.0.dfco_entry_map_kpa10 = crate::Kpa10::new(500);
        cal.0.dfco_delay_cycles = 1;

        let mut state = LogicalState {
            dfco_active: true,
            dfco_qualify_counter: 1,
            ..LogicalState::default()
        };
        let hold = dfco_step(&cal, qualifying_input(), &state);
        assert!(hold.fuel_cut);

        state.dfco_active = hold.dfco_active;
        state.dfco_qualify_counter = hold.dfco_qualify_counter;

        let mut release = qualifying_input();
        release.tps_x100 = 600;
        let released = dfco_step(&cal, release, &state);
        assert!(!released.fuel_cut);
        assert!(!released.dfco_active);
        assert_eq!(released.dfco_qualify_counter, 0);
    }
}
