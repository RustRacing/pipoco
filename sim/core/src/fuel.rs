use crate::{state::MisfireReason, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InjectionTimingMode {
    StartOfInjection,
    EndOfInjection,
    UntimedBatch,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InjectionCommand {
    pub cylinder: CylinderIndex,
    pub mode: InjectionTimingMode,
    pub angle_deg10: CrankDeg10,
    pub pulse_width_us: Micros,
    pub injector_flow_ug_per_us: MicrogramsPerMicros,
    pub deadtime_us: Micros,
}

impl InjectionCommand {
    pub const fn empty() -> Self {
        Self {
            cylinder: CylinderIndex(0),
            mode: InjectionTimingMode::StartOfInjection,
            angle_deg10: CrankDeg10(0),
            pulse_width_us: Micros(0),
            injector_flow_ug_per_us: MicrogramsPerMicros(0),
            deadtime_us: Micros(0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AcceptedInjection {
    pub command: InjectionCommand,
    pub fuel_mass_ug: MassUg,
    pub phasing_x1000: u16,
}

impl AcceptedInjection {
    pub const fn empty() -> Self {
        Self {
            command: InjectionCommand::empty(),
            fuel_mass_ug: MassUg(0),
            phasing_x1000: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WallFilmState {
    pub film_fuel_ug: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WallFilmParams {
    pub x_deposit_x1000: u16,
    pub tau_ms: u16,
}

pub fn fuel_mass(command: InjectionCommand) -> MassUg {
    let effective_pw = command
        .pulse_width_us
        .0
        .saturating_sub(command.deadtime_us.0);
    MassUg(effective_pw.saturating_mul(command.injector_flow_ug_per_us.0))
}

pub fn injection_phasing(mode: InjectionTimingMode, angle: CrankDeg10) -> u16 {
    match mode {
        InjectionTimingMode::UntimedBatch => 850,
        InjectionTimingMode::StartOfInjection => {
            let distance = angle.0.abs_diff(3600) as u32;
            (1000u32.saturating_sub(distance / 8)).max(650) as u16
        }
        InjectionTimingMode::EndOfInjection => {
            let distance = angle.0.abs_diff(3000) as u32;
            (1000u32.saturating_sub(distance / 9)).max(700) as u16
        }
    }
}

pub fn injection_window_deg10(
    mode: InjectionTimingMode,
    angle: CrankDeg10,
    pulse_width_us: Micros,
    rpm: Rpm,
) -> Option<(CrankDeg10, CrankDeg10)> {
    let duration_deg10 = (rpm.0 as u64)
        .saturating_mul(pulse_width_us.0 as u64)
        .saturating_mul(6)
        / 100_000;
    let duration_deg10 = duration_deg10.min(CYCLE_DEG10 as u64 - 1) as i32;
    match mode {
        InjectionTimingMode::UntimedBatch => None,
        InjectionTimingMode::StartOfInjection => {
            Some((angle, normalize_deg10_i32(angle.0 as i32 + duration_deg10)))
        }
        InjectionTimingMode::EndOfInjection => {
            Some((normalize_deg10_i32(angle.0 as i32 - duration_deg10), angle))
        }
    }
}

pub fn fuel_cut_reason(fuel_cut: bool) -> Option<MisfireReason> {
    if fuel_cut {
        Some(MisfireReason::FuelCut)
    } else {
        None
    }
}

pub fn update_wall_film(
    state: &mut WallFilmState,
    injected_fuel_ug: i32,
    params: WallFilmParams,
    dt_ms: u16,
) -> i32 {
    let injected_fuel_ug = injected_fuel_ug.max(0);
    let x_deposit_x1000 = params.x_deposit_x1000.min(1000) as i64;
    let deposited = injected_fuel_ug as i64 * x_deposit_x1000 / 1000;
    let direct_to_cylinder = injected_fuel_ug as i64 - deposited;
    let available_film = (state.film_fuel_ug as i64).max(0).saturating_add(deposited);
    let tau_ms = params.tau_ms.max(1) as i64;
    let evaporated = (available_film.saturating_mul(dt_ms as i64) / tau_ms).min(available_film);

    state.film_fuel_ug = (available_film - evaporated).clamp(0, i32::MAX as i64) as i32;
    direct_to_cylinder
        .saturating_add(evaporated)
        .clamp(0, i32::MAX as i64) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn injection_window_observes_start_and_end_modes() {
        let start = injection_window_deg10(
            InjectionTimingMode::StartOfInjection,
            CrankDeg10(1000),
            Micros(10_000),
            Rpm(1000),
        )
        .unwrap();
        let end = injection_window_deg10(
            InjectionTimingMode::EndOfInjection,
            CrankDeg10(1000),
            Micros(10_000),
            Rpm(1000),
        )
        .unwrap();

        assert_eq!(start, (CrankDeg10(1000), CrankDeg10(1600)));
        assert_eq!(end, (CrankDeg10(400), CrankDeg10(1000)));
        assert_eq!(
            injection_window_deg10(
                InjectionTimingMode::UntimedBatch,
                CrankDeg10(1000),
                Micros(10_000),
                Rpm(1000),
            ),
            None
        );
    }

    #[test]
    fn wall_film_tip_in_stores_and_evaporates_fuel() {
        let mut state = WallFilmState { film_fuel_ug: 0 };
        let params = WallFilmParams {
            x_deposit_x1000: 500,
            tau_ms: 100,
        };

        let first = update_wall_film(&mut state, 10_000, params, 10);
        let stored_after_tip_in = state.film_fuel_ug;
        let coast = update_wall_film(&mut state, 0, params, 10);

        assert!(first < 10_000);
        assert!(stored_after_tip_in > 0);
        assert!(coast > 0);
        assert!(state.film_fuel_ug < stored_after_tip_in);
    }

    #[test]
    fn wall_film_zero_deposit_behaves_like_direct_injection() {
        let mut state = WallFilmState { film_fuel_ug: 0 };
        let delivered = update_wall_film(
            &mut state,
            12_345,
            WallFilmParams {
                x_deposit_x1000: 0,
                tau_ms: 100,
            },
            10,
        );

        assert_eq!(delivered, 12_345);
        assert_eq!(state.film_fuel_ug, 0);
    }
}
