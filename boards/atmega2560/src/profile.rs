use ecu_board_api::FullEcuOutputProfile;
use ecu_board_api::{
    BoardCapabilities, BoardResourceLimits, EcuOutput, IgnitionProfileId, IgnitionProfileMode,
    LoadSourceCapabilities, PinMapId, ProfileId, RuntimeBuildId,
};
use ecu_board_profiles::{
    profiles::m50b25tu::{m50_runtime_output_profile, M50B25TU_FULL_COP},
    BoardBuildMetadata, BoardFeatureBindings, BoardId, BuildArtifact, FeatureBinding,
};

pub const TARGET_TRIPLE: &str = "avr-none";
pub const TARGET_CPU: &str = "atmega2560";
pub const CLOCK_HZ: u32 = 16_000_000;
pub const ATMEGA2560_ADC_CHANNELS: u8 = 16;
pub const ATMEGA2560_TIMER_COUNT: u8 = 6;
pub const ATMEGA2560_TIMER_COMPARE_CHANNELS: u8 = 16;
pub const MAX_OUTPUT_TRANSITIONS: usize = 24;
pub const MAX_AUX_COMMANDS: usize = 16;

pub const PROFILE_ID_M50B25TU_FULL_COP: ProfileId = ProfileId::new(0x5026);
pub const IGNITION_PROFILE_ID_SEQUENTIAL_COP_6: IgnitionProfileId = IgnitionProfileId::new(6);
pub const PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC: PinMapId = PinMapId::new(0x023);
pub const RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE: RuntimeBuildId = RuntimeBuildId::new(1);
pub const BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560: BoardId =
    BoardId::new("speeduino-m5x-rev23-atmega2560");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Atmega2560BoardProfile {
    pub runtime_output_profile: FullEcuOutputProfile,
    pub profile_id: ProfileId,
    pub ignition_profile_id: IgnitionProfileId,
    pub ignition_profile_mode: IgnitionProfileMode,
    pub ignition_profile_authority_blocked_mode: IgnitionProfileMode,
    pub pin_map_id: PinMapId,
    pub runtime_build_id: RuntimeBuildId,
}

impl Atmega2560BoardProfile {
    pub const fn new(
        runtime_output_profile: FullEcuOutputProfile,
        profile_id: ProfileId,
        ignition_profile_id: IgnitionProfileId,
        ignition_profile_mode: IgnitionProfileMode,
        ignition_profile_authority_blocked_mode: IgnitionProfileMode,
        pin_map_id: PinMapId,
        runtime_build_id: RuntimeBuildId,
    ) -> Self {
        Self {
            runtime_output_profile,
            profile_id,
            ignition_profile_id,
            ignition_profile_mode,
            ignition_profile_authority_blocked_mode,
            pin_map_id,
            runtime_build_id,
        }
    }
}

pub(crate) const fn m50b25tu_full_cop_runtime_profile() -> FullEcuOutputProfile {
    m50_runtime_output_profile(M50B25TU_FULL_COP)
}

pub(crate) const fn m50b25tu_speeduino_m5x_rev23_board_profile() -> Atmega2560BoardProfile {
    speeduino_m5x_rev23_board_profile_for_full_ecu(m50b25tu_full_cop_runtime_profile())
}

pub(crate) const fn speeduino_m5x_rev23_board_profile_for_full_ecu(
    runtime_output_profile: FullEcuOutputProfile,
) -> Atmega2560BoardProfile {
    Atmega2560BoardProfile::new(
        runtime_output_profile,
        PROFILE_ID_M50B25TU_FULL_COP,
        IGNITION_PROFILE_ID_SEQUENTIAL_COP_6,
        IgnitionProfileMode::SequentialCop,
        IgnitionProfileMode::SequentialCopAuthorityBlocked,
        PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC,
        RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE,
    )
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Atmega2560DigitalPin(u8);

impl Atmega2560DigitalPin {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Atmega2560AnalogPin(u8);

impl Atmega2560AnalogPin {
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    pub const fn get(self) -> u8 {
        self.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpeeduinoM5xConditionedInput {
    CrankVr1,
    CamVr2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SpeeduinoM5xRev23PinMap {
    pub injector_pins: [Atmega2560DigitalPin; 6],
    pub ignition_pins: [Atmega2560DigitalPin; 6],
    pub crank_input: SpeeduinoM5xConditionedInput,
    pub cam_input: SpeeduinoM5xConditionedInput,
    pub tach1_pin: Atmega2560DigitalPin,
    pub tach2_pin: Atmega2560DigitalPin,
    pub low_current_pins: [Atmega2560DigitalPin; 5],
    pub reset_pin: Atmega2560DigitalPin,
    pub iat_pin: Atmega2560AnalogPin,
    pub clt_pin: Atmega2560AnalogPin,
    pub tps_pin: Atmega2560AnalogPin,
    pub map_pin: Atmega2560AnalogPin,
    pub battery_pin: Atmega2560AnalogPin,
    pub oxygen_pin: Atmega2560AnalogPin,
}

impl SpeeduinoM5xRev23PinMap {
    pub const fn output_pin(self, output: EcuOutput) -> Option<Atmega2560DigitalPin> {
        match output {
            EcuOutput::Injector(channel) => {
                let idx = channel.get() as usize;
                if idx < self.injector_pins.len() {
                    Some(self.injector_pins[idx])
                } else {
                    None
                }
            }
            EcuOutput::Ignition(channel) => {
                let idx = channel.get() as usize;
                if idx < self.ignition_pins.len() {
                    Some(self.ignition_pins[idx])
                } else {
                    None
                }
            }
        }
    }
}

/// Schematic-derived Speeduino-M5x Rev 2.3 CPU pin map for the Bosch 88-pin
/// Motronic board.
///
/// Evidence source:
/// `Speeduino-M5x-PCBs/m50-m40-m60_Pnp/Rev 2.3/Schematic__speeduino
/// compatible PCB for bosch 88pin motronic rev2.3.pdf`.
pub(crate) const SPEEDUINO_M5X_REV23_PIN_MAP: SpeeduinoM5xRev23PinMap = SpeeduinoM5xRev23PinMap {
    injector_pins: [
        Atmega2560DigitalPin::new(8),
        Atmega2560DigitalPin::new(9),
        Atmega2560DigitalPin::new(10),
        Atmega2560DigitalPin::new(11),
        Atmega2560DigitalPin::new(12),
        Atmega2560DigitalPin::new(50),
    ],
    ignition_pins: [
        Atmega2560DigitalPin::new(40),
        Atmega2560DigitalPin::new(38),
        Atmega2560DigitalPin::new(52),
        Atmega2560DigitalPin::new(48),
        Atmega2560DigitalPin::new(36),
        Atmega2560DigitalPin::new(34),
    ],
    crank_input: SpeeduinoM5xConditionedInput::CrankVr1,
    cam_input: SpeeduinoM5xConditionedInput::CamVr2,
    tach1_pin: Atmega2560DigitalPin::new(19),
    tach2_pin: Atmega2560DigitalPin::new(18),
    low_current_pins: [
        Atmega2560DigitalPin::new(45),
        Atmega2560DigitalPin::new(47),
        Atmega2560DigitalPin::new(49),
        Atmega2560DigitalPin::new(51),
        Atmega2560DigitalPin::new(53),
    ],
    reset_pin: Atmega2560DigitalPin::new(43),
    iat_pin: Atmega2560AnalogPin::new(0),
    clt_pin: Atmega2560AnalogPin::new(1),
    tps_pin: Atmega2560AnalogPin::new(2),
    map_pin: Atmega2560AnalogPin::new(3),
    battery_pin: Atmega2560AnalogPin::new(4),
    oxygen_pin: Atmega2560AnalogPin::new(8),
};

pub(crate) const SPEEDUINO_M5X_REV23_LOW_CURRENT_AUX_CHANNELS: u8 =
    SPEEDUINO_M5X_REV23_PIN_MAP.low_current_pins.len() as u8;
pub(crate) const SPEEDUINO_M5X_REV23_TACH_AUX_CHANNELS: u8 = 2;
pub(crate) const SPEEDUINO_M5X_REV23_OUTPUT_CHANNELS: u8 =
    SPEEDUINO_M5X_REV23_PIN_MAP.injector_pins.len() as u8
        + SPEEDUINO_M5X_REV23_PIN_MAP.ignition_pins.len() as u8
        + SPEEDUINO_M5X_REV23_LOW_CURRENT_AUX_CHANNELS
        + SPEEDUINO_M5X_REV23_TACH_AUX_CHANNELS;

pub(crate) const SPEEDUINO_M5X_REV23_RESOURCE_LIMITS: BoardResourceLimits =
    BoardResourceLimits::new(
        CLOCK_HZ,
        ATMEGA2560_ADC_CHANNELS,
        ATMEGA2560_TIMER_COUNT,
        ATMEGA2560_TIMER_COMPARE_CHANNELS,
        SPEEDUINO_M5X_REV23_OUTPUT_CHANNELS,
        MAX_OUTPUT_TRANSITIONS,
        MAX_AUX_COMMANDS,
        None,
    );

/// Recipe-level board capabilities for the Speeduino-M5x Rev 2.3 ATmega setup.
///
/// `aux_channels` counts controllable aux-capable outputs from the schematic:
/// five low-current outputs plus two tach outputs. A future aux role map should
/// decide which logical `AuxOutput` variants each pin is allowed to drive.
pub(crate) const SPEEDUINO_M5X_REV23_CAPABILITIES: BoardCapabilities = BoardCapabilities::new(
    true,
    true,
    true,
    SPEEDUINO_M5X_REV23_PIN_MAP.ignition_pins.len() as u8,
    SPEEDUINO_M5X_REV23_PIN_MAP.injector_pins.len() as u8,
    SPEEDUINO_M5X_REV23_LOW_CURRENT_AUX_CHANNELS + SPEEDUINO_M5X_REV23_TACH_AUX_CHANNELS,
    true,
    false,
    false,
)
.with_load_sources(LoadSourceCapabilities::map());

pub(crate) const SPEEDUINO_M5X_REV23_ATMEGA2560_BUILD_METADATA: BoardBuildMetadata =
    BoardBuildMetadata {
        board_id: BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560,
        artifact: BuildArtifact::BoardOnly {
            board_id: BOARD_ID_SPEEDUINO_M5X_REV23_ATMEGA2560,
        },
        capabilities: SPEEDUINO_M5X_REV23_CAPABILITIES,
        pin_map_id: PIN_MAP_SPEEDUINO_M5X_REV23_SCHEMATIC,
        runtime_build_id: RUNTIME_BUILD_ID_ATMEGA2560_BRIDGE,
        feature_bindings: BoardFeatureBindings {
            trigger_capture: FeatureBinding::BuiltIn,
            ignition: FeatureBinding::BuiltIn,
            injection: FeatureBinding::BuiltIn,
            fuel_strategy: FeatureBinding::BuiltIn,
            rev_limiter: FeatureBinding::BuiltIn,
            tuner_studio: FeatureBinding::Unsupported,
            telemetry: FeatureBinding::BuiltIn,
            persistence: FeatureBinding::Unsupported,
        },
        default_ts_profile: None,
    };
