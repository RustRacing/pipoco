use crate::{bilerp_i16, bilerp_u16, Kpa10, Rpm, SignedDegrees10, Table2D16, TempC10};

const DUTY_MAX_X1000: u16 = 1000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VvtCam {
    Intake,
    Exhaust,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VvtChannelId {
    pub bank: u8,
    pub cam: VvtCam,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VvtMode {
    Disabled,
    OnOff,
    OpenLoopPwm,
    ClosedLoopCamAngle,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VvtLoadSource {
    Map,
    Tps,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VvtConfig {
    pub channel: VvtChannelId,
    pub mode: VvtMode,
    pub load_source: VvtLoadSource,
    pub require_sync: bool,
    pub min_rpm: Rpm,
    pub max_rpm: Rpm,
    pub min_clt_c10: TempC10,
    pub min_tps_x100: u16,
    pub on_off_duty_x1000: u16,
    pub on_off_threshold_x1000: u16,
    pub open_loop_duty_table: Table2D16<u16>,
    pub closed_loop_target_table: Table2D16<i16>,
}

impl Default for VvtConfig {
    fn default() -> Self {
        Self {
            channel: VvtChannelId {
                bank: 0,
                cam: VvtCam::Intake,
            },
            mode: VvtMode::Disabled,
            load_source: VvtLoadSource::Map,
            require_sync: true,
            min_rpm: Rpm::new(0),
            max_rpm: Rpm::new(u16::MAX),
            min_clt_c10: TempC10::new(i16::MIN),
            min_tps_x100: 0,
            on_off_duty_x1000: DUTY_MAX_X1000,
            on_off_threshold_x1000: 1,
            open_loop_duty_table: Table2D16::default(),
            closed_loop_target_table: Table2D16::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VvtInput {
    pub rpm: Rpm,
    pub map_kpa10: Kpa10,
    pub tps_x100: u16,
    pub clt_c10: TempC10,
    pub sync_valid: bool,
    pub measured_angle_deg10: Option<SignedDegrees10>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VvtState {
    pub fault_latched: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VvtStepResult {
    pub active: bool,
    pub duty_x1000: u16,
    pub target_angle_deg10: SignedDegrees10,
    pub measured_angle_deg10: Option<SignedDegrees10>,
    pub closed_loop_allowed: bool,
    pub fault: bool,
    pub next_state: VvtState,
}

fn table_ready<T>(table: &Table2D16<T>) -> bool {
    table.rpm_axis.len >= 2 && table.load_axis.len >= 2
}

fn clamp_duty(duty_x1000: u16) -> u16 {
    if duty_x1000 > DUTY_MAX_X1000 {
        DUTY_MAX_X1000
    } else {
        duty_x1000
    }
}

fn load_for(config: &VvtConfig, input: &VvtInput) -> Kpa10 {
    match config.load_source {
        VvtLoadSource::Map => input.map_kpa10,
        VvtLoadSource::Tps => Kpa10::new(input.tps_x100),
    }
}

fn eligible(config: &VvtConfig, input: &VvtInput) -> bool {
    (!config.require_sync || input.sync_valid)
        && input.rpm.get() >= config.min_rpm.get()
        && input.rpm.get() <= config.max_rpm.get()
        && input.clt_c10.get() >= config.min_clt_c10.get()
        && input.tps_x100 >= config.min_tps_x100
}

pub fn vvt_step(config: &VvtConfig, state: &VvtState, input: &VvtInput) -> VvtStepResult {
    let mut next_state = *state;
    let fault = state.fault_latched;

    if fault || !eligible(config, input) {
        return VvtStepResult {
            active: false,
            duty_x1000: 0,
            target_angle_deg10: SignedDegrees10::new(0),
            measured_angle_deg10: input.measured_angle_deg10,
            closed_loop_allowed: false,
            fault,
            next_state,
        };
    }

    let load = load_for(config, input);
    match config.mode {
        VvtMode::Disabled => VvtStepResult {
            active: false,
            duty_x1000: 0,
            target_angle_deg10: SignedDegrees10::new(0),
            measured_angle_deg10: input.measured_angle_deg10,
            closed_loop_allowed: false,
            fault: false,
            next_state,
        },
        VvtMode::OnOff => {
            let duty = if table_ready(&config.open_loop_duty_table) {
                bilerp_u16(&config.open_loop_duty_table, input.rpm, load)
            } else {
                config.on_off_duty_x1000
            };
            let active = duty >= config.on_off_threshold_x1000;
            VvtStepResult {
                active,
                duty_x1000: if active {
                    clamp_duty(config.on_off_duty_x1000)
                } else {
                    0
                },
                target_angle_deg10: SignedDegrees10::new(0),
                measured_angle_deg10: input.measured_angle_deg10,
                closed_loop_allowed: false,
                fault: false,
                next_state,
            }
        }
        VvtMode::OpenLoopPwm => {
            if !table_ready(&config.open_loop_duty_table) {
                next_state.fault_latched = true;
                return VvtStepResult {
                    active: false,
                    duty_x1000: 0,
                    target_angle_deg10: SignedDegrees10::new(0),
                    measured_angle_deg10: input.measured_angle_deg10,
                    closed_loop_allowed: false,
                    fault: true,
                    next_state,
                };
            }

            VvtStepResult {
                active: true,
                duty_x1000: clamp_duty(bilerp_u16(&config.open_loop_duty_table, input.rpm, load)),
                target_angle_deg10: SignedDegrees10::new(0),
                measured_angle_deg10: input.measured_angle_deg10,
                closed_loop_allowed: false,
                fault: false,
                next_state,
            }
        }
        VvtMode::ClosedLoopCamAngle => {
            if input.measured_angle_deg10.is_none()
                || !table_ready(&config.closed_loop_target_table)
            {
                next_state.fault_latched = true;
                return VvtStepResult {
                    active: false,
                    duty_x1000: 0,
                    target_angle_deg10: SignedDegrees10::new(0),
                    measured_angle_deg10: input.measured_angle_deg10,
                    closed_loop_allowed: false,
                    fault: true,
                    next_state,
                };
            }

            VvtStepResult {
                active: true,
                duty_x1000: 0,
                target_angle_deg10: SignedDegrees10::new(bilerp_i16(
                    &config.closed_loop_target_table,
                    input.rpm,
                    load,
                )),
                measured_angle_deg10: input.measured_angle_deg10,
                closed_loop_allowed: true,
                fault: false,
                next_state,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Axis16;

    fn axis(values: &[u16]) -> Axis16 {
        let mut axis = Axis16 {
            len: values.len() as u8,
            ..Axis16::default()
        };
        let mut idx = 0usize;
        while idx < values.len() {
            axis.values[idx] = values[idx];
            idx += 1;
        }
        axis
    }

    fn duty_table(low: u16, high: u16) -> Table2D16<u16> {
        let mut table = Table2D16 {
            rpm_axis: axis(&[1000, 5000]),
            load_axis: axis(&[0, 10000]),
            ..Table2D16::default()
        };
        table.values[0][0] = low;
        table.values[0][1] = low;
        table.values[1][0] = high;
        table.values[1][1] = high;
        table
    }

    fn angle_table(low: i16, high: i16) -> Table2D16<i16> {
        let mut table = Table2D16 {
            rpm_axis: axis(&[1000, 5000]),
            load_axis: axis(&[0, 10000]),
            ..Table2D16::default()
        };
        table.values[0][0] = low;
        table.values[0][1] = low;
        table.values[1][0] = high;
        table.values[1][1] = high;
        table
    }

    fn input() -> VvtInput {
        VvtInput {
            rpm: Rpm::new(3000),
            map_kpa10: Kpa10::new(700),
            tps_x100: 3500,
            clt_c10: TempC10::new(800),
            sync_valid: true,
            measured_angle_deg10: Some(SignedDegrees10::new(20)),
        }
    }

    #[test]
    fn disabled_commands_zero() {
        let result = vvt_step(&VvtConfig::default(), &VvtState::default(), &input());

        assert!(!result.active);
        assert_eq!(result.duty_x1000, 0);
        assert!(!result.fault);
    }

    #[test]
    fn sync_and_temperature_gates_prevent_output() {
        let config = VvtConfig {
            mode: VvtMode::OnOff,
            min_clt_c10: TempC10::new(700),
            ..VvtConfig::default()
        };
        let mut cold = input();
        cold.clt_c10 = TempC10::new(200);

        let result = vvt_step(&config, &VvtState::default(), &cold);

        assert!(!result.active);
        assert_eq!(result.duty_x1000, 0);
    }

    #[test]
    fn on_off_uses_table_threshold_and_configured_duty() {
        let config = VvtConfig {
            mode: VvtMode::OnOff,
            load_source: VvtLoadSource::Tps,
            on_off_duty_x1000: 1000,
            on_off_threshold_x1000: 100,
            open_loop_duty_table: duty_table(0, 500),
            ..VvtConfig::default()
        };

        let result = vvt_step(&config, &VvtState::default(), &input());

        assert!(result.active);
        assert_eq!(result.duty_x1000, 1000);
    }

    #[test]
    fn open_loop_pwm_interpolates_and_clamps_duty() {
        let config = VvtConfig {
            mode: VvtMode::OpenLoopPwm,
            load_source: VvtLoadSource::Tps,
            open_loop_duty_table: duty_table(0, 1200),
            ..VvtConfig::default()
        };

        let result = vvt_step(&config, &VvtState::default(), &input());

        assert!(result.active);
        assert!(result.duty_x1000 > 0);
        assert!(result.duty_x1000 <= 1000);
    }

    #[test]
    fn closed_loop_requires_measured_angle() {
        let config = VvtConfig {
            mode: VvtMode::ClosedLoopCamAngle,
            closed_loop_target_table: angle_table(0, 250),
            ..VvtConfig::default()
        };
        let mut no_position = input();
        no_position.measured_angle_deg10 = None;

        let result = vvt_step(&config, &VvtState::default(), &no_position);

        assert!(result.fault);
        assert!(result.next_state.fault_latched);
        assert_eq!(result.duty_x1000, 0);
    }

    #[test]
    fn closed_loop_reports_target_without_generating_pid_duty() {
        let config = VvtConfig {
            mode: VvtMode::ClosedLoopCamAngle,
            load_source: VvtLoadSource::Tps,
            closed_loop_target_table: angle_table(0, 250),
            ..VvtConfig::default()
        };

        let result = vvt_step(&config, &VvtState::default(), &input());

        assert!(result.active);
        assert!(result.closed_loop_allowed);
        assert!(result.target_angle_deg10.get() > 0);
        assert_eq!(result.measured_angle_deg10, Some(SignedDegrees10::new(20)));
        assert_eq!(result.duty_x1000, 0);
    }
}
