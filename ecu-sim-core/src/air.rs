use crate::{
    config::{PlantConfig, ValveEvents, VeTable},
    types::*,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IntakeManifoldState {
    pub pressure_pa: i32,
    pub temperature_k_x100: u32,
    pub air_mass_mg: i32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct IntakeManifoldConfig {
    pub volume_cc: u32,
    pub throttle_area_mm2: u32,
    pub discharge_coeff_x1000: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AirPathFlows {
    pub throttle_air_mg_per_s: i32,
    pub cylinder_air_mg_per_s: i32,
}

impl IntakeManifoldState {
    pub fn from_pressure(
        pressure_pa: i32,
        temperature_k_x100: u32,
        config: IntakeManifoldConfig,
    ) -> Self {
        let air_mass_mg = manifold_air_mass_mg(
            pressure_pa.max(0),
            temperature_k_x100.max(1),
            config.volume_cc.max(1),
        );
        Self {
            pressure_pa: pressure_pa.max(0),
            temperature_k_x100: temperature_k_x100.max(1),
            air_mass_mg,
        }
    }
}

pub fn estimate_map_kpa10<const CYL: usize>(
    config: &PlantConfig<CYL>,
    throttle_x1000: u16,
    idle_x1000: u16,
) -> Kpa10 {
    let throttle = throttle_x1000.min(1000) as u32;
    let idle = idle_x1000.min(1000) as u32;
    let low = config.air.idle_map_kpa10.0 as u32;
    let high = config.air.wide_open_map_kpa10.0 as u32;
    Kpa10(clamp_u16(
        low + high.saturating_sub(low) * throttle / 1000 + idle / 10,
        2500,
    ))
}

pub fn update_manifold_map_kpa10<const CYL: usize>(
    config: &PlantConfig<CYL>,
    current: Kpa10,
    throttle_x1000: u16,
    idle_x1000: u16,
    rpm: Rpm,
    dt_us: Micros,
) -> Kpa10 {
    let target = estimate_map_kpa10(config, throttle_x1000, idle_x1000);
    if dt_us.0 == 0 {
        return current;
    }

    let volume = config.air.manifold_volume_cc.max(1);
    let rpm_factor = (rpm.0 / 100).clamp(1, 100);
    let gain_x1000 = (dt_us.0 as u64)
        .saturating_mul(rpm_factor as u64)
        .saturating_mul(1000)
        / volume as u64
        / 100;
    let gain_x1000 = gain_x1000.clamp(1, 1000) as i32;
    let delta = target.0 as i32 - current.0 as i32;
    Kpa10((current.0 as i32 + delta * gain_x1000 / 1000).clamp(0, u16::MAX as i32) as u16)
}

#[allow(clippy::too_many_arguments)]
pub fn update_intake_manifold(
    state: &mut IntakeManifoldState,
    config: IntakeManifoldConfig,
    throttle_x1000: u16,
    rpm: Rpm,
    displacement_cc: u32,
    ve_x1000: u16,
    ambient_pressure_pa: i32,
    dt_ms: Millis,
) -> AirPathFlows {
    let volume_cc = config.volume_cc.max(1);
    let temperature_k_x100 = state.temperature_k_x100.max(1);
    let ambient_pressure_pa = ambient_pressure_pa.max(0);
    let throttle_air_mg_per_s = throttle_flow_mg_per_s(
        throttle_x1000.min(1000),
        state.pressure_pa.max(0),
        ambient_pressure_pa,
        config,
    );
    let cylinder_air_mg_per_s = cylinder_flow_mg_per_s(
        state.pressure_pa.max(0),
        temperature_k_x100,
        rpm,
        displacement_cc,
        ve_x1000,
    );

    let delta_mg = (throttle_air_mg_per_s as i64 - cylinder_air_mg_per_s as i64)
        .saturating_mul(dt_ms.0 as i64)
        / 1000;
    state.air_mass_mg = (state.air_mass_mg as i64 + delta_mg).clamp(0, i32::MAX as i64) as i32;
    state.pressure_pa = manifold_pressure_pa(state.air_mass_mg, temperature_k_x100, volume_cc)
        .min(ambient_pressure_pa);

    AirPathFlows {
        throttle_air_mg_per_s,
        cylinder_air_mg_per_s,
    }
}

pub fn estimate_air_mass_ug<const CYL: usize>(config: &PlantConfig<CYL>, map: Kpa10) -> MassUg {
    let per_cyl_cc = config.displacement_cc / config.cylinder_count.max(1) as u32;
    MassUg(per_cyl_cc.saturating_mul(map.0 as u32) / 7)
}

pub fn lookup_ve_x1000(table: &VeTable, rpm: Rpm, load: Kpa10) -> u16 {
    let rpm_cell = find_cell_rpm(&table.rpm_axis, rpm);
    let load_cell = find_cell_kpa10(&table.load_axis, load);
    let r0 = table.rpm_axis[rpm_cell].0;
    let r1 = table.rpm_axis[rpm_cell + 1].0;
    let l0 = table.load_axis[load_cell].0;
    let l1 = table.load_axis[load_cell + 1].0;
    let r = rpm.0.clamp(r0, r1);
    let l = load.0.clamp(l0, l1);
    let dr = r1 - r0;
    let dl = (l1 - l0) as u32;
    let ar = r - r0;
    let bl = (l - l0) as u32;
    let v00 = table.ve_x1000[rpm_cell][load_cell] as u128;
    let v10 = table.ve_x1000[rpm_cell + 1][load_cell] as u128;
    let v01 = table.ve_x1000[rpm_cell][load_cell + 1] as u128;
    let v11 = table.ve_x1000[rpm_cell + 1][load_cell + 1] as u128;
    let dr = dr as u128;
    let dl = dl as u128;
    let ar = ar as u128;
    let bl = bl as u128;
    let value =
        v00 * (dr - ar) * (dl - bl) + v10 * ar * (dl - bl) + v01 * (dr - ar) * bl + v11 * ar * bl;

    (value / (dr * dl)) as u16
}

pub fn estimate_speed_density_air_mass_ug<const CYL: usize>(
    config: &PlantConfig<CYL>,
    rpm: Rpm,
    map: Kpa10,
    iat: Kelvin10,
) -> MassUg {
    let per_cyl_cc = config.engine.displacement_cc / config.cylinder_count.max(1) as u32;
    let pressure_pa = map.0 as u128 * 100;
    let ve_x1000 = lookup_ve_x1000(&config.air.ve_table, rpm, map) as u128;
    let numerator = pressure_pa
        .saturating_mul(per_cyl_cc as u128)
        .saturating_mul(ve_x1000)
        .saturating_mul(10);
    let denominator = (config.thermo.r_air_j_per_kg_k as u128).saturating_mul(iat.0.max(1) as u128);

    MassUg((numerator / denominator).min(u32::MAX as u128) as u32)
}

pub fn valve_event_airflow_modifier_x1000(events: ValveEvents, rpm: Rpm) -> u16 {
    let ivc_delta = events.ivc_deg_abdc_x10 as i32 - 500;
    let rpm_delta = rpm.0 as i32 - 3500;
    let raw = 1000 + ivc_delta * rpm_delta / 20_000;
    raw.clamp(850, 1150) as u16
}

fn find_cell_rpm(axis: &[Rpm; MAX_TABLE_AXIS_POINTS], value: Rpm) -> usize {
    let clipped = value.0.clamp(axis[0].0, axis[MAX_TABLE_AXIS_POINTS - 1].0);
    let mut i = 0;
    while i + 2 < MAX_TABLE_AXIS_POINTS {
        if clipped < axis[i + 1].0 {
            return i;
        }
        i += 1;
    }
    MAX_TABLE_AXIS_POINTS - 2
}

fn find_cell_kpa10(axis: &[Kpa10; MAX_TABLE_AXIS_POINTS], value: Kpa10) -> usize {
    let clipped = value.0.clamp(axis[0].0, axis[MAX_TABLE_AXIS_POINTS - 1].0);
    let mut i = 0;
    while i + 2 < MAX_TABLE_AXIS_POINTS {
        if clipped < axis[i + 1].0 {
            return i;
        }
        i += 1;
    }
    MAX_TABLE_AXIS_POINTS - 2
}

fn manifold_air_mass_mg(pressure_pa: i32, temperature_k_x100: u32, volume_cc: u32) -> i32 {
    let denominator = 287u128.saturating_mul(temperature_k_x100.max(1) as u128);
    let numerator = pressure_pa.max(0) as u128 * volume_cc.max(1) as u128 * 100;
    (numerator / denominator).min(i32::MAX as u128) as i32
}

fn manifold_pressure_pa(air_mass_mg: i32, temperature_k_x100: u32, volume_cc: u32) -> i32 {
    let numerator = air_mass_mg.max(0) as u128 * 287u128 * temperature_k_x100.max(1) as u128;
    let denominator = 100u128 * volume_cc.max(1) as u128;
    (numerator / denominator).min(i32::MAX as u128) as i32
}

fn throttle_flow_mg_per_s(
    throttle_x1000: u16,
    manifold_pressure_pa: i32,
    ambient_pressure_pa: i32,
    config: IntakeManifoldConfig,
) -> i32 {
    let pressure_delta = ambient_pressure_pa
        .saturating_sub(manifold_pressure_pa)
        .max(0) as u128;
    let pressure_ratio_x1000 = if ambient_pressure_pa <= 0 {
        1000
    } else {
        (manifold_pressure_pa.max(0) as u128 * 1000 / ambient_pressure_pa as u128).min(1000) as u16
    };
    let choked_gain_x1000 = if pressure_ratio_x1000 < 530 {
        1250u128
    } else {
        1000u128
    };
    let flow = config.throttle_area_mm2.max(1) as u128
        * config.discharge_coeff_x1000 as u128
        * throttle_x1000 as u128
        * pressure_delta
        * choked_gain_x1000
        / 2_400_000_000_000u128;
    flow.min(i32::MAX as u128) as i32
}

fn cylinder_flow_mg_per_s(
    manifold_pressure_pa: i32,
    temperature_k_x100: u32,
    rpm: Rpm,
    displacement_cc: u32,
    ve_x1000: u16,
) -> i32 {
    let air_per_720_mg = manifold_air_mass_mg(
        manifold_pressure_pa,
        temperature_k_x100,
        displacement_cc.max(1),
    ) as u128
        * ve_x1000 as u128
        / 1000;
    let flow = air_per_720_mg * rpm.0 as u128 / 120;
    flow.min(i32::MAX as u128) as i32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::PlantConfig;

    fn table_with_surface() -> VeTable {
        let mut table = VeTable::constant(0);
        let mut i = 0;
        while i < MAX_TABLE_AXIS_POINTS {
            let mut j = 0;
            while j < MAX_TABLE_AXIS_POINTS {
                table.ve_x1000[i][j] = 500 + i as u16 * 100 + j as u16 * 10;
                j += 1;
            }
            i += 1;
        }
        table
    }

    #[test]
    fn ve_lookup_is_exact_at_grid_points() {
        let table = table_with_surface();

        assert_eq!(
            lookup_ve_x1000(&table, table.rpm_axis[3], table.load_axis[5]),
            table.ve_x1000[3][5]
        );
    }

    #[test]
    fn ve_lookup_reproduces_constant_table() {
        let table = VeTable::constant(875);

        assert_eq!(lookup_ve_x1000(&table, Rpm(2750), Kpa10(725)), 875);
        assert_eq!(lookup_ve_x1000(&table, Rpm(0), Kpa10(0)), 875);
        assert_eq!(lookup_ve_x1000(&table, Rpm(9000), Kpa10(3000)), 875);
    }

    #[test]
    fn ve_lookup_is_bounded_by_cell_corners() {
        let table = table_with_surface();
        let value = lookup_ve_x1000(&table, Rpm(1750), Kpa10(575));
        let corners = [
            table.ve_x1000[2][2],
            table.ve_x1000[3][2],
            table.ve_x1000[2][3],
            table.ve_x1000[3][3],
        ];
        let min = corners.iter().copied().min().unwrap();
        let max = corners.iter().copied().max().unwrap();

        assert!(value >= min);
        assert!(value <= max);
    }

    #[test]
    fn speed_density_air_mass_tracks_map_ve_and_iat() {
        let mut cfg = PlantConfig::<4>::default_four();
        cfg.air.ve_table = VeTable::constant(1000);

        let low_map =
            estimate_speed_density_air_mass_ug(&cfg, Rpm(2000), Kpa10(500), Kelvin10(2930));
        let high_map =
            estimate_speed_density_air_mass_ug(&cfg, Rpm(2000), Kpa10(1000), Kelvin10(2930));
        cfg.air.ve_table = VeTable::constant(800);
        let low_ve =
            estimate_speed_density_air_mass_ug(&cfg, Rpm(2000), Kpa10(1000), Kelvin10(2930));
        let hot = estimate_speed_density_air_mass_ug(&cfg, Rpm(2000), Kpa10(1000), Kelvin10(3330));

        assert!(high_map.0 > low_map.0);
        assert!(low_ve.0 < high_map.0);
        assert!(hot.0 < high_map.0);
    }

    #[test]
    fn manifold_map_moves_toward_throttle_target() {
        let cfg = PlantConfig::<4>::default_four();
        let start = Kpa10(350);
        let first = update_manifold_map_kpa10(&cfg, start, 1000, 0, Rpm(2000), Micros(1000));
        let second = update_manifold_map_kpa10(&cfg, first, 1000, 0, Rpm(2000), Micros(1000));

        assert!(first.0 > start.0);
        assert!(second.0 > first.0);
        assert!(second.0 <= cfg.air.wide_open_map_kpa10.0);
    }

    #[test]
    fn manifold_filling_tps_step_has_finite_map_lag() {
        let cfg = IntakeManifoldConfig {
            volume_cc: 3000,
            throttle_area_mm2: 1800,
            discharge_coeff_x1000: 700,
        };
        let mut manifold = IntakeManifoldState::from_pressure(45_000, 29315, cfg);
        let start = manifold.pressure_pa;

        let first = update_intake_manifold(
            &mut manifold,
            cfg,
            800,
            Rpm(2500),
            2000,
            900,
            101_325,
            Millis(20),
        );
        let after_first = manifold.pressure_pa;
        update_intake_manifold(
            &mut manifold,
            cfg,
            800,
            Rpm(2500),
            2000,
            900,
            101_325,
            Millis(20),
        );

        assert!(first.throttle_air_mg_per_s > first.cylinder_air_mg_per_s);
        assert!(after_first > start);
        assert!(after_first < 101_325);
        assert!(manifold.pressure_pa > after_first);
        assert!(manifold.pressure_pa < 101_325);
    }

    #[test]
    fn valve_event_ivc_sweep_favors_low_or_high_rpm_breathing() {
        let mut early = ValveEvents {
            ivo_deg_btdc_x10: 0,
            ivc_deg_abdc_x10: 350,
            evo_deg_bbdc_x10: 500,
            evc_deg_atdc_x10: 0,
            intake_lift_mm_x100: 900,
            exhaust_lift_mm_x100: 850,
            intake_duration_deg_x10: 2200,
            exhaust_duration_deg_x10: 2200,
        };
        let mut late = early;
        late.ivc_deg_abdc_x10 = 700;
        early.ivc_deg_abdc_x10 = 350;

        assert!(
            valve_event_airflow_modifier_x1000(early, Rpm(1800))
                > valve_event_airflow_modifier_x1000(late, Rpm(1800))
        );
        assert!(
            valve_event_airflow_modifier_x1000(late, Rpm(6000))
                > valve_event_airflow_modifier_x1000(early, Rpm(6000))
        );
    }
}
