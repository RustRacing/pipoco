//! Simple actuator configs: idle (open-loop PWM) and fan control; closed-loop stub

#[derive(Copy, Clone)]
pub struct IdleConfig {
    pub enable: bool,
    pub duty_x10: u16, // %*10
    pub freq_hz: u16,
}
impl IdleConfig {
    pub const DEFAULT: Self = Self {
        enable: false,
        duty_x10: 0,
        freq_hz: 100,
    };
}

#[derive(Copy, Clone)]
pub struct FanConfig {
    pub enable: bool,
    pub on_c: i16,
    pub off_c: i16,
}
impl FanConfig {
    pub const DEFAULT: Self = Self {
        enable: false,
        on_c: 95,
        off_c: 90,
    };
}

#[derive(Copy, Clone)]
pub struct ClConfig {
    pub enable: bool,
    pub target_afr_x10: u16,
    pub kp_i: u16, // integral gain x1
    pub ki_i: u16, // not used yet
}
impl ClConfig {
    pub const DEFAULT: Self = Self {
        enable: false,
        target_afr_x10: 147,
        kp_i: 0,
        ki_i: 0,
    };
}
