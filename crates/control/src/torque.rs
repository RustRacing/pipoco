/// Reason a product torque command is limited.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TorqueLimitReason {
    #[default]
    None,
    Idle,
    Driver,
    RevLimiter,
    Knock,
    LimpMode,
}

/// First-pass product torque inputs, expressed as x100 percent of nominal
/// torque.
///
/// This is the runtime control surface, not the frozen x1000 semantic oracle
/// torque pipeline.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TorqueInputs {
    pub driver_request_x100: u16,
    pub driver_request_x1000: Option<u16>,
    pub idle_request_x100: u16,
    pub rev_limit_x100: u16,
    pub knock_limit_x100: u16,
    pub limp_limit_x100: u16,
}

impl TorqueInputs {
    pub const fn new(
        driver_request_x100: u16,
        idle_request_x100: u16,
        rev_limit_x100: u16,
        knock_limit_x100: u16,
        limp_limit_x100: u16,
    ) -> Self {
        Self {
            driver_request_x100,
            driver_request_x1000: None,
            idle_request_x100,
            rev_limit_x100,
            knock_limit_x100,
            limp_limit_x100,
        }
    }

    pub fn with_driver_request_x1000(mut self, driver_request_x1000: u16) -> Self {
        self.driver_request_x1000 = Some(driver_request_x1000);
        self
    }
}

/// Allowed torque result for downstream planners.
///
/// The runtime planner consumes x100 torque demand; it does not expose the
/// semantic-oracle x1000 observability surface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AllowedTorque {
    pub requested_x100: u16,
    pub requested_x1000: u16,
    pub allowed_x100: u16,
    pub allowed_x1000: u16,
    pub reason: TorqueLimitReason,
}

impl AllowedTorque {
    pub const fn new(
        requested_x100: u16,
        requested_x1000: u16,
        allowed_x100: u16,
        allowed_x1000: u16,
        reason: TorqueLimitReason,
    ) -> Self {
        Self {
            requested_x100,
            requested_x1000,
            allowed_x100,
            allowed_x1000,
            reason,
        }
    }
}

/// Torque arbiter that resolves demand against runtime safety caps.
///
/// This is the product torque limiter path. The frozen spec torque pipeline is
/// mirrored separately in `ecu-runtime` test scaffolding.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TorqueArbiter;

impl TorqueArbiter {
    pub const fn new() -> Self {
        Self
    }

    pub fn evaluate(&self, inputs: TorqueInputs) -> AllowedTorque {
        let requested_x100 = inputs.driver_request_x100.max(inputs.idle_request_x100);
        let driver_request_x1000 = inputs
            .driver_request_x1000
            .unwrap_or(inputs.driver_request_x100.saturating_mul(10));
        let idle_request_x1000 = inputs.idle_request_x100.saturating_mul(10);
        let requested_x1000 = core::cmp::max(driver_request_x1000, idle_request_x1000);
        let mut allowed_x100 = requested_x100;
        let mut allowed_x1000 = requested_x1000;
        let mut reason = if inputs.idle_request_x100 >= inputs.driver_request_x100 {
            TorqueLimitReason::Idle
        } else {
            TorqueLimitReason::Driver
        };

        let caps = [
            (
                inputs.rev_limit_x100,
                inputs.rev_limit_x100.saturating_mul(10),
                TorqueLimitReason::RevLimiter,
            ),
            (
                inputs.knock_limit_x100,
                inputs.knock_limit_x100.saturating_mul(10),
                TorqueLimitReason::Knock,
            ),
            (
                inputs.limp_limit_x100,
                inputs.limp_limit_x100.saturating_mul(10),
                TorqueLimitReason::LimpMode,
            ),
        ];
        for (cap_x100, cap_x1000, cap_reason) in caps {
            if cap_x100 < allowed_x100 {
                allowed_x100 = cap_x100;
                reason = cap_reason;
            }
            if cap_x1000 < allowed_x1000 {
                allowed_x1000 = cap_x1000;
            }
        }

        if allowed_x100 >= requested_x100 {
            reason = TorqueLimitReason::None;
        }

        AllowedTorque::new(
            requested_x100,
            requested_x1000,
            allowed_x100,
            allowed_x1000,
            reason,
        )
    }
}
