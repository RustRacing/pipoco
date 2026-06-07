//! Board capability and resource metadata.

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BoardCapabilities {
    pub trigger_input: bool,
    pub rpm_input: bool,
    pub cam_input: bool,
    pub load_sources: LoadSourceCapabilities,
    pub ignition_channels: u8,
    pub injector_channels: u8,
    pub aux_channels: u8,
    pub telemetry: bool,
    pub calibration_persistence: bool,
    pub watchdog: bool,
}

impl BoardCapabilities {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        trigger_input: bool,
        rpm_input: bool,
        cam_input: bool,
        ignition_channels: u8,
        injector_channels: u8,
        aux_channels: u8,
        telemetry: bool,
        calibration_persistence: bool,
        watchdog: bool,
    ) -> Self {
        Self {
            trigger_input,
            rpm_input,
            cam_input,
            load_sources: LoadSourceCapabilities::none(),
            ignition_channels,
            injector_channels,
            aux_channels,
            telemetry,
            calibration_persistence,
            watchdog,
        }
    }

    pub const fn with_load_sources(mut self, load_sources: LoadSourceCapabilities) -> Self {
        self.load_sources = load_sources;
        self
    }

    pub const fn supports_rev_limiter(self) -> bool {
        self.rpm_input && self.ignition_channels > 0
    }

    pub const fn supports_ignition_only(self) -> bool {
        self.trigger_input && self.rpm_input && self.ignition_channels > 0
    }

    pub const fn supports_injection_only(self) -> bool {
        self.rpm_input && self.injector_channels > 0
    }

    pub const fn supports_full_ecu(self) -> bool {
        self.supports_ignition_only() && self.injector_channels > 0
    }

    pub const fn supports_load_sensor(self) -> bool {
        self.load_sources.any()
    }
}

/// Static resource ceilings declared by a board adapter.
///
/// This value is board data, not product or recipe policy. It lets recipe and
/// tooling layers reason about hard limits without importing a concrete board
/// crate's pin map or runtime adapter.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct BoardResourceLimits {
    pub clock_hz: u32,
    pub adc_channels: u8,
    pub timer_count: u8,
    pub timer_compare_channels: u8,
    pub output_channels: u8,
    pub output_transition_capacity: usize,
    pub aux_command_capacity: usize,
    pub calibration_storage_bytes: Option<u32>,
}

impl BoardResourceLimits {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        clock_hz: u32,
        adc_channels: u8,
        timer_count: u8,
        timer_compare_channels: u8,
        output_channels: u8,
        output_transition_capacity: usize,
        aux_command_capacity: usize,
        calibration_storage_bytes: Option<u32>,
    ) -> Self {
        Self {
            clock_hz,
            adc_channels,
            timer_count,
            timer_compare_channels,
            output_channels,
            output_transition_capacity,
            aux_command_capacity,
            calibration_storage_bytes,
        }
    }

    pub const fn has_calibration_storage(self) -> bool {
        self.calibration_storage_bytes.is_some()
    }
}

/// Logical load-source inputs exposed to recipe validation.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LoadSourceCapabilities {
    pub map: bool,
    pub maf: bool,
    pub tps: bool,
    pub lambda: bool,
}

impl LoadSourceCapabilities {
    pub const fn new(map: bool, maf: bool, tps: bool, lambda: bool) -> Self {
        Self {
            map,
            maf,
            tps,
            lambda,
        }
    }

    pub const fn none() -> Self {
        Self::new(false, false, false, false)
    }

    pub const fn map() -> Self {
        Self::new(true, false, false, false)
    }

    pub const fn maf() -> Self {
        Self::new(false, true, false, false)
    }

    pub const fn any(self) -> bool {
        self.map || self.maf || self.tps || self.lambda
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct ProfileId(u16);

impl ProfileId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct IgnitionProfileId(u16);

impl IgnitionProfileId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct PinMapId(u16);

impl PinMapId {
    pub const fn new(value: u16) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u16 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct RuntimeBuildId(u32);

impl RuntimeBuildId {
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u32 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct CalibrationPage(u8);

impl CalibrationPage {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}
