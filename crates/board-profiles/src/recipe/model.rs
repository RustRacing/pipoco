use ecu_board_api::RuntimeOutputProfile;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeProfile {
    pub output_profile: RuntimeOutputProfile,
}

impl RuntimeProfile {
    pub const fn new(output_profile: RuntimeOutputProfile) -> Self {
        Self { output_profile }
    }

    pub const fn rev_limiter() -> Self {
        Self::new(ecu_board_api::legacy::single_channel_runtime_output_profile())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SubsystemSet {
    pub trigger_capture: bool,
    pub ignition: bool,
    pub injection: bool,
    pub fuel_strategy: bool,
    pub rev_limiter: bool,
    pub tuner_studio: bool,
    pub telemetry: bool,
    pub persistence: bool,
}

impl SubsystemSet {
    pub const fn rev_limiter() -> Self {
        Self {
            trigger_capture: false,
            ignition: false,
            injection: false,
            fuel_strategy: false,
            rev_limiter: true,
            tuner_studio: false,
            telemetry: false,
            persistence: false,
        }
    }

    pub const fn ignition_only() -> Self {
        Self {
            trigger_capture: true,
            ignition: true,
            injection: false,
            fuel_strategy: false,
            rev_limiter: true,
            tuner_studio: false,
            telemetry: false,
            persistence: false,
        }
    }

    pub const fn injection_only() -> Self {
        Self {
            trigger_capture: false,
            ignition: false,
            injection: true,
            fuel_strategy: true,
            rev_limiter: false,
            tuner_studio: false,
            telemetry: false,
            persistence: false,
        }
    }

    pub const fn full_ecu() -> Self {
        Self {
            trigger_capture: true,
            ignition: true,
            injection: true,
            fuel_strategy: true,
            rev_limiter: true,
            tuner_studio: false,
            telemetry: false,
            persistence: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InputSourceSet {
    pub rpm: bool,
    pub trigger: bool,
    pub cam: bool,
    pub load_sensor: bool,
}

impl InputSourceSet {
    pub const fn rpm_only() -> Self {
        Self {
            rpm: true,
            trigger: false,
            cam: false,
            load_sensor: false,
        }
    }

    pub const fn crank_rpm() -> Self {
        Self {
            rpm: true,
            trigger: true,
            cam: false,
            load_sensor: false,
        }
    }

    pub const fn crank_cam_load() -> Self {
        Self {
            rpm: true,
            trigger: true,
            cam: true,
            load_sensor: true,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutputTopology {
    pub ignition_channels: u8,
    pub injector_channels: u8,
    pub aux_channels: u8,
    pub ignition_cut: bool,
}

impl OutputTopology {
    pub const fn ignition_cut(ignition_channels: u8) -> Self {
        Self {
            ignition_channels,
            injector_channels: 0,
            aux_channels: 0,
            ignition_cut: true,
        }
    }

    pub const fn ignition_channels(ignition_channels: u8) -> Self {
        Self {
            ignition_channels,
            injector_channels: 0,
            aux_channels: 0,
            ignition_cut: false,
        }
    }

    pub const fn injector_channels(injector_channels: u8) -> Self {
        Self {
            ignition_channels: 0,
            injector_channels,
            aux_channels: 0,
            ignition_cut: false,
        }
    }

    pub const fn full_ecu(ignition_channels: u8, injector_channels: u8, aux_channels: u8) -> Self {
        Self {
            ignition_channels,
            injector_channels,
            aux_channels,
            ignition_cut: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyPolicy {
    pub watchdog_required: bool,
    /// Justification for omitting a watchdog. Must be non-empty when
    /// `watchdog_required` is false. Recipes without watchdog and without a
    /// justification fail the `sim/board-contracts` conformance check.
    pub watchdog_absent_justification: &'static str,
}

impl SafetyPolicy {
    pub const fn watchdog_required() -> Self {
        Self {
            watchdog_required: true,
            watchdog_absent_justification: "",
        }
    }

    pub const fn no_watchdog(justification: &'static str) -> Self {
        Self {
            watchdog_required: false,
            watchdog_absent_justification: justification,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardSelection {
    AnyCompatible,
    Named(&'static str),
}
