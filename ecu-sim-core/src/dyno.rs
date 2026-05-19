use crate::{config::DynoMode, types::*};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DynoSweepPoint {
    pub valid: bool,
    pub target_rpm: Rpm,
    pub measured_rpm: Rpm,
    pub torque_nm_x100: TorqueNmX100,
    pub horsepower_x100: i32,
    pub bmep_bar_x100: BmepBarX100,
    pub ve_x1000: u16,
    pub map_kpa10: Kpa10,
    pub lambda_x1000: u16,
    pub spark_advance_deg10: Degrees10,
    pub dyno_load_torque_nm_x100: TorqueNmX100,
}

impl DynoSweepPoint {
    pub const fn empty() -> Self {
        Self {
            valid: false,
            target_rpm: Rpm(0),
            measured_rpm: Rpm(0),
            torque_nm_x100: TorqueNmX100(0),
            horsepower_x100: 0,
            bmep_bar_x100: BmepBarX100(0),
            ve_x1000: 0,
            map_kpa10: Kpa10(0),
            lambda_x1000: 1000,
            spark_advance_deg10: Degrees10(0),
            dyno_load_torque_nm_x100: TorqueNmX100(0),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DynoFrame {
    pub torque_nm_x100: TorqueNmX100,
    pub filtered_torque_nm_x100: TorqueNmX100,
    pub indicated_torque_nm_x100: TorqueNmX100,
    pub brake_torque_nm_x100: TorqueNmX100,
    pub bmep_bar_x100: BmepBarX100,
    pub imep_bar_x100: ImepBarX100,
    pub pmep_bar_x100: PmepBarX100,
    pub fmep_bar_x100: FmepBarX100,
    pub horsepower_x100: i32,
    pub load_mode: DynoMode,
    pub sweep_target_rpm: Rpm,
    pub sweep_point: DynoSweepPoint,
    pub sweep_complete: bool,
}

impl DynoFrame {
    pub const fn empty() -> Self {
        Self {
            torque_nm_x100: TorqueNmX100(0),
            filtered_torque_nm_x100: TorqueNmX100(0),
            indicated_torque_nm_x100: TorqueNmX100(0),
            brake_torque_nm_x100: TorqueNmX100(0),
            bmep_bar_x100: BmepBarX100(0),
            imep_bar_x100: ImepBarX100(0),
            pmep_bar_x100: PmepBarX100(0),
            fmep_bar_x100: FmepBarX100(0),
            horsepower_x100: 0,
            load_mode: DynoMode::Disabled,
            sweep_target_rpm: Rpm(0),
            sweep_point: DynoSweepPoint::empty(),
            sweep_complete: false,
        }
    }
}

pub fn dyno_load_torque(
    mode: DynoMode,
    fixed_load: TorqueNmX100,
    current_rpm: Rpm,
    target_rpm: Rpm,
) -> TorqueNmX100 {
    match mode {
        DynoMode::Disabled => TorqueNmX100(0),
        DynoMode::FixedLoad => fixed_load,
        DynoMode::TargetRpmHold | DynoMode::TargetRpmSweep => {
            let overspeed = current_rpm.0.saturating_sub(target_rpm.0);
            TorqueNmX100((overspeed as i32).saturating_mul(10))
        }
    }
}

pub fn dyno_pid_load_torque(
    config: crate::config::DynoConfig,
    current_rpm: Rpm,
    dt_us: Micros,
    integral_x100: &mut i32,
    previous_error_rpm: &mut i32,
) -> TorqueNmX100 {
    let target = if config.mode == DynoMode::TargetRpmSweep {
        config
            .target_rpm
            .0
            .clamp(config.sweep_start_rpm.0, config.sweep_end_rpm.0)
    } else {
        config.target_rpm.0
    };
    let error = current_rpm.0 as i32 - target as i32;
    if error <= 0 {
        *previous_error_rpm = error;
        return config.fixed_load_torque_nm_x100;
    }

    let dt_ms = (dt_us.0 / 1000).max(1) as i32;
    *integral_x100 = integral_x100
        .saturating_add(error.saturating_mul(dt_ms))
        .clamp(-1_000_000, 1_000_000);
    let derivative = error.saturating_sub(*previous_error_rpm);
    *previous_error_rpm = error;

    let p = config.pid_kp_x1000.saturating_mul(error) / 1000;
    let i = config.pid_ki_x1000.saturating_mul(*integral_x100 / 100) / 1000;
    let d = config.pid_kd_x1000.saturating_mul(derivative) / 1000;
    let load = config
        .fixed_load_torque_nm_x100
        .0
        .saturating_add(p)
        .saturating_add(i)
        .saturating_add(d)
        .max(0);

    TorqueNmX100(load)
}

pub fn horsepower_x100(torque_nm_x100: TorqueNmX100, rpm: Rpm) -> i32 {
    if torque_nm_x100.0 <= 0 || rpm.0 == 0 {
        return 0;
    }
    // hp = torque_nm * rpm / 7127; both torque and output are x100.
    ((torque_nm_x100.0 as i64 * rpm.0 as i64) / 7127) as i32
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{DynoConfig, DynoMode};

    fn hold_config() -> DynoConfig {
        DynoConfig {
            mode: DynoMode::TargetRpmHold,
            fixed_load_torque_nm_x100: TorqueNmX100(1000),
            target_rpm: Rpm(3000),
            sweep_start_rpm: Rpm(0),
            sweep_end_rpm: Rpm(0),
            sweep_step_rpm: Rpm(0),
            hold_cycles_before_sample: 1,
            sample_cycles: 1,
            rpm_error_limit: Rpm(10),
            pid_kp_x1000: 1000,
            pid_ki_x1000: 100,
            pid_kd_x1000: 0,
        }
    }

    #[test]
    fn dyno_pid_adds_load_above_target_without_freezing_rpm() {
        let mut integral = 0;
        let mut previous = 0;
        let below = dyno_pid_load_torque(
            hold_config(),
            Rpm(2500),
            Micros(10_000),
            &mut integral,
            &mut previous,
        );
        let above = dyno_pid_load_torque(
            hold_config(),
            Rpm(3500),
            Micros(10_000),
            &mut integral,
            &mut previous,
        );

        assert_eq!(below, TorqueNmX100(1000));
        assert!(above.0 > below.0);
    }

    #[test]
    fn dyno_pid_integral_is_clamped() {
        let mut integral = 999_900;
        let mut previous = 0;

        let _ = dyno_pid_load_torque(
            hold_config(),
            Rpm(6000),
            Micros(1_000_000),
            &mut integral,
            &mut previous,
        );

        assert_eq!(integral, 1_000_000);
    }
}
