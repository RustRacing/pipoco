use ecu_domain::{ChannelId, CylinderId, Rpm};

use crate::AuxOutput;

pub const FULL_ECU_MAX_CYLINDERS: usize = 8;
pub const FULL_ECU_MAX_LIMP_AUX_OUTPUTS: usize = 4;
const CRANK_REV_DEGREES10: u16 = 3600;

/// Runtime output topology selected by board/profile configuration.
///
/// This is active product API: board profiles, board adapters, simulators, and
/// runtime configuration use it to choose ignition-only, injection-only, or
/// full-ECU lowering without coupling board profiles to runtime internals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RuntimeOutputProfile {
    #[default]
    LegacySingleChannel,
    IgnitionOnly(SparkOutputProfile),
    InjectionOnly(FuelOutputProfile),
    FullEcu(FullEcuOutputProfile),
}

/// Spark output topology for simple even-fire igniters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SparkOutputMode {
    /// One ECU-controlled coil feeding a mechanical distributor or external router.
    SingleCoil,
    /// One ECU-controlled coil channel per wasted-spark pair.
    WastedSpark,
}

/// Generic crank-only, even-fire spark profile.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SparkOutputProfile {
    pub cylinder_count: u8,
    pub mode: SparkOutputMode,
}

impl SparkOutputProfile {
    pub const fn crank_only_wasted_spark(cylinder_count: u8) -> Self {
        Self {
            cylinder_count,
            mode: SparkOutputMode::WastedSpark,
        }
    }

    pub const fn crank_only_single_coil(cylinder_count: u8) -> Self {
        Self {
            cylinder_count,
            mode: SparkOutputMode::SingleCoil,
        }
    }

    pub fn events_per_crank_rev(self) -> usize {
        let cylinders = usize::from(self.cylinder_count);
        let events = cylinders / 2;
        events.clamp(1, FULL_ECU_MAX_CYLINDERS)
    }

    pub fn ignition_channel(self, event_index: usize) -> ChannelId {
        match self.mode {
            SparkOutputMode::SingleCoil => ChannelId::new(0),
            SparkOutputMode::WastedSpark => {
                ChannelId::new((event_index % self.events_per_crank_rev()) as u8)
            }
        }
    }

    pub fn event_tdc_angle_deg10(self, event_index: usize) -> u16 {
        let events = self.events_per_crank_rev() as u32;
        let event_index = (event_index % events as usize) as u32;
        ((event_index * u32::from(CRANK_REV_DEGREES10)) / events) as u16
    }
}

/// Injector output topology for simple fuel-only or fuel-and-spark products.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FuelOutputMode {
    /// One ECU-controlled injector or external injector driver.
    SinglePoint,
    /// One output per configured injector bank/channel, all pulsed together.
    Batch,
}

/// Generic injector profile independent from spark topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FuelOutputProfile {
    pub channel_count: u8,
    pub mode: FuelOutputMode,
}

impl FuelOutputProfile {
    pub const fn single_point() -> Self {
        Self {
            channel_count: 1,
            mode: FuelOutputMode::SinglePoint,
        }
    }

    pub const fn batch(channel_count: u8) -> Self {
        Self {
            channel_count,
            mode: FuelOutputMode::Batch,
        }
    }

    pub fn events_per_pulse(self) -> usize {
        match self.mode {
            FuelOutputMode::SinglePoint => 1,
            FuelOutputMode::Batch => {
                usize::from(self.channel_count).clamp(1, FULL_ECU_MAX_CYLINDERS)
            }
        }
    }

    pub fn injector_channel(self, event_index: usize) -> ChannelId {
        match self.mode {
            FuelOutputMode::SinglePoint => ChannelId::new(0),
            FuelOutputMode::Batch => ChannelId::new((event_index % self.events_per_pulse()) as u8),
        }
    }
}

impl RuntimeOutputProfile {
    pub const fn crank_only_wasted_spark(cylinder_count: u8) -> Self {
        Self::IgnitionOnly(SparkOutputProfile::crank_only_wasted_spark(cylinder_count))
    }

    pub const fn crank_only_single_coil(cylinder_count: u8) -> Self {
        Self::IgnitionOnly(SparkOutputProfile::crank_only_single_coil(cylinder_count))
    }

    pub const fn single_point_injection() -> Self {
        Self::InjectionOnly(FuelOutputProfile::single_point())
    }

    pub const fn batch_injection(channel_count: u8) -> Self {
        Self::InjectionOnly(FuelOutputProfile::batch(channel_count))
    }

    pub const fn full_ecu(profile: FullEcuOutputProfile) -> Self {
        Self::FullEcu(profile)
    }
}

pub mod legacy {
    use super::RuntimeOutputProfile;

    pub const fn single_channel_runtime_output_profile() -> RuntimeOutputProfile {
        RuntimeOutputProfile::LegacySingleChannel
    }
}

/// Full-ECU output topology for coordinated spark, fuel, and limp aux policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FullEcuOutputProfile {
    pub ignition: IgnitionOutputProfile,
    pub injection: InjectionOutputProfile,
    pub aux_safety: AuxSafetyProfile,
    pub authority: OutputAuthorityRequirement,
}

impl FullEcuOutputProfile {
    pub const fn new(
        ignition: IgnitionOutputProfile,
        injection: InjectionOutputProfile,
        aux_safety: AuxSafetyProfile,
        authority: OutputAuthorityRequirement,
    ) -> Self {
        Self {
            ignition,
            injection,
            aux_safety,
            authority,
        }
    }

    pub const fn sequential_wasted_spark<const N: usize>(
        firing_order: [CylinderId; N],
        injector_channels: u8,
        coils: u8,
        aux_safety: AuxSafetyProfile,
        authority: OutputAuthorityRequirement,
    ) -> Self {
        Self::new(
            IgnitionOutputProfile::wasted_spark(N as u8, coils),
            InjectionOutputProfile::sequential(firing_order, injector_channels, true),
            aux_safety,
            authority,
        )
    }

    pub const fn sequential_coil_on_plug<const N: usize>(
        firing_order: [CylinderId; N],
        injector_channels: u8,
        coils: u8,
        aux_safety: AuxSafetyProfile,
        authority: OutputAuthorityRequirement,
    ) -> Self {
        Self::new(
            IgnitionOutputProfile::coil_on_plug(N as u8, coils, true),
            InjectionOutputProfile::sequential(firing_order, injector_channels, true),
            aux_safety,
            authority,
        )
    }

    pub const fn is_valid(self) -> bool {
        let injection_events = self.injection.event_count();
        let ignition_events = self.ignition.event_count();
        self.injection.is_valid()
            && self.ignition.is_valid()
            && injection_events > 0
            && injection_events == ignition_events
    }

    pub const fn event_count(self) -> usize {
        if self.is_valid() {
            self.injection.event_count()
        } else {
            0
        }
    }

    pub fn injector_channel(self, slot: usize) -> ChannelId {
        self.injection.injector_channel(slot)
    }

    pub fn ignition_channel(self, slot: usize) -> ChannelId {
        self.ignition.ignition_channel(slot)
    }

    pub fn cycle_slot_us(self, rpm: Rpm) -> u32 {
        let rpm = u32::from(rpm.get());
        if rpm == 0 {
            return 0;
        }

        let slot_us = 20_000_000u64 / u64::from(rpm);
        slot_us.clamp(1, u32::MAX as u64) as u32
    }
}

/// Spark-side topology for full-ECU output lowering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IgnitionOutputProfile {
    WastedSpark {
        cylinders: u8,
        coils: u8,
    },
    CoilOnPlug {
        cylinders: u8,
        coils: u8,
        phase_required: bool,
    },
}

impl IgnitionOutputProfile {
    pub const fn wasted_spark(cylinders: u8, coils: u8) -> Self {
        Self::WastedSpark { cylinders, coils }
    }

    pub const fn coil_on_plug(cylinders: u8, coils: u8, phase_required: bool) -> Self {
        Self::CoilOnPlug {
            cylinders,
            coils,
            phase_required,
        }
    }

    pub const fn is_valid(self) -> bool {
        match self {
            Self::WastedSpark { cylinders, coils } => {
                cylinders > 0
                    && (cylinders as usize) <= FULL_ECU_MAX_CYLINDERS
                    && coils > 0
                    && (coils as usize) <= FULL_ECU_MAX_CYLINDERS
                    && coils <= cylinders
                    && cylinders % coils == 0
            }
            Self::CoilOnPlug {
                cylinders, coils, ..
            } => {
                cylinders > 0
                    && (cylinders as usize) <= FULL_ECU_MAX_CYLINDERS
                    && coils == cylinders
            }
        }
    }

    pub const fn event_count(self) -> usize {
        if self.is_valid() {
            match self {
                Self::WastedSpark { cylinders, .. } | Self::CoilOnPlug { cylinders, .. } => {
                    cylinders as usize
                }
            }
        } else {
            0
        }
    }

    pub fn phase_required(self) -> bool {
        match self {
            Self::WastedSpark { .. } => false,
            Self::CoilOnPlug { phase_required, .. } => phase_required,
        }
    }

    pub fn ignition_channel(self, slot: usize) -> ChannelId {
        let channels = usize::from(match self {
            Self::WastedSpark { coils, .. } | Self::CoilOnPlug { coils, .. } => coils,
        })
        .clamp(1, FULL_ECU_MAX_CYLINDERS);
        ChannelId::new((slot % channels) as u8)
    }
}

/// Fuel-side topology for full-ECU output lowering.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InjectionOutputProfile {
    Sequential {
        cylinders: u8,
        channels: u8,
        /// Firing-slot order used for 720-degree event timing.
        ///
        /// Channel mapping remains slot-based; this does not encode a
        /// cylinder-to-pin map.
        firing_order: [CylinderId; FULL_ECU_MAX_CYLINDERS],
        phase_required: bool,
    },
}

impl InjectionOutputProfile {
    pub const fn sequential<const N: usize>(
        firing_order: [CylinderId; N],
        channels: u8,
        phase_required: bool,
    ) -> Self {
        let mut padded = [CylinderId::new(0); FULL_ECU_MAX_CYLINDERS];
        let mut i = 0;
        while i < N && i < FULL_ECU_MAX_CYLINDERS {
            padded[i] = firing_order[i];
            i += 1;
        }

        Self::Sequential {
            cylinders: i as u8,
            channels,
            firing_order: padded,
            phase_required,
        }
    }

    pub const fn is_valid(self) -> bool {
        match self {
            Self::Sequential {
                cylinders,
                channels,
                firing_order,
                ..
            } => {
                if cylinders == 0
                    || (cylinders as usize) > FULL_ECU_MAX_CYLINDERS
                    || channels == 0
                    || (channels as usize) > FULL_ECU_MAX_CYLINDERS
                {
                    return false;
                }

                let mut i = 0;
                while i < cylinders as usize {
                    if firing_order[i].get() == 0 {
                        return false;
                    }
                    i += 1;
                }

                true
            }
        }
    }

    pub const fn event_count(self) -> usize {
        if self.is_valid() {
            match self {
                Self::Sequential { cylinders, .. } => cylinders as usize,
            }
        } else {
            0
        }
    }

    pub fn phase_required(self) -> bool {
        match self {
            Self::Sequential { phase_required, .. } => phase_required,
        }
    }

    pub fn injector_channel(self, slot: usize) -> ChannelId {
        let channels = usize::from(match self {
            Self::Sequential { channels, .. } => channels,
        })
        .clamp(1, FULL_ECU_MAX_CYLINDERS);
        ChannelId::new((slot % channels) as u8)
    }
}

/// Aux outputs the runtime forces off when full-ECU operation falls back to limp.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AuxSafetyProfile {
    pub off_on_limp: [Option<AuxOutput>; FULL_ECU_MAX_LIMP_AUX_OUTPUTS],
}

impl AuxSafetyProfile {
    pub const fn none() -> Self {
        Self {
            off_on_limp: [None; FULL_ECU_MAX_LIMP_AUX_OUTPUTS],
        }
    }

    pub const fn off_on_limp<const N: usize>(outputs: [AuxOutput; N]) -> Self {
        let mut padded = [None; FULL_ECU_MAX_LIMP_AUX_OUTPUTS];
        let mut i = 0;
        while i < N && i < FULL_ECU_MAX_LIMP_AUX_OUTPUTS {
            padded[i] = Some(outputs[i]);
            i += 1;
        }

        Self {
            off_on_limp: padded,
        }
    }
}

/// Minimum phase authority required before the runtime may emit this topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputAuthorityRequirement {
    CrankSynchronized,
    FullSequential720,
}

impl OutputAuthorityRequirement {
    pub const fn requires_full_sequential(self) -> bool {
        matches!(self, Self::FullSequential720)
    }
}
