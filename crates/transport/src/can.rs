//! CAN transport for ECU messages.

use crate::message::MAX_ENCODED_MESSAGE_SIZE;
use crate::{Message, Transport, TransportError, TransportStats};
use ecu_domain::diag::DiagCode;
use postcard::{from_bytes, to_slice};

#[cfg(feature = "transport-can-fd")]
const MAX_CAN_DATA: usize = 64;
#[cfg(not(feature = "transport-can-fd"))]
const MAX_CAN_DATA: usize = 8;

const REASM_BUF: usize = MAX_ENCODED_MESSAGE_SIZE;
const RX_REASSEMBLY_SLOTS: usize = 2;
pub const CAN_MESSAGE_CLASS_COUNT: usize = 15;

/// Current bus-health classification for a CAN device.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanBusHealthState {
    #[default]
    Healthy,
    Degraded,
    BusOff,
}

/// Current recovery posture for a CAN device after a bus-health fault.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanBusRecoveryState {
    #[default]
    None,
    Needed,
    InProgress,
}

/// Product-owned bounded CAN health and recovery surface.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanBusHealth {
    pub state: CanBusHealthState,
    pub tx_error_count: u16,
    pub rx_error_count: u16,
    pub bus_off: bool,
    pub recovery: CanBusRecoveryState,
}

impl CanBusHealth {
    pub const fn healthy() -> Self {
        Self {
            state: CanBusHealthState::Healthy,
            tx_error_count: 0,
            rx_error_count: 0,
            bus_off: false,
            recovery: CanBusRecoveryState::None,
        }
    }

    pub const fn is_send_blocked(self) -> bool {
        self.bus_off || !matches!(self.recovery, CanBusRecoveryState::None)
    }
}

/// Relative CAN arbitration priority for transport-owned message classes.
#[derive(Debug, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum CanMessagePriority {
    EmergencyControl,
    Fault,
    Timing,
    Sensor,
    CalibrationData,
    Configuration,
    Heartbeat,
}

/// Product-owned CAN route classes for transport messages.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanMessageClass {
    Fault,
    ResetCommand,
    EngineControlCommand,
    TriggerTiming,
    SensorData,
    GpsData,
    ExternalEgtData,
    FuelTable,
    IgnitionTable,
    EngineConfig,
    InjectorConfig,
    CalibrationCommand,
    Obd2Request,
    Obd2Response,
    Heartbeat,
}

impl CanMessageClass {
    pub const fn arbitration_id(self) -> u32 {
        match self {
            Self::Fault => 0x080,
            Self::ResetCommand => 0x050,
            Self::EngineControlCommand => 0x060,
            Self::TriggerTiming => 0x100,
            Self::SensorData => 0x200,
            Self::GpsData => 0x220,
            Self::ExternalEgtData => 0x230,
            Self::FuelTable => 0x300,
            Self::IgnitionTable => 0x310,
            Self::EngineConfig => 0x400,
            Self::InjectorConfig => 0x410,
            Self::CalibrationCommand => 0x420,
            Self::Obd2Request => 0x430,
            Self::Obd2Response => 0x440,
            Self::Heartbeat => 0x700,
        }
    }

    pub const fn priority(self) -> CanMessagePriority {
        match self {
            Self::ResetCommand | Self::EngineControlCommand => CanMessagePriority::EmergencyControl,
            Self::Fault => CanMessagePriority::Fault,
            Self::TriggerTiming => CanMessagePriority::Timing,
            Self::SensorData | Self::GpsData | Self::ExternalEgtData => CanMessagePriority::Sensor,
            Self::FuelTable | Self::IgnitionTable => CanMessagePriority::CalibrationData,
            Self::EngineConfig
            | Self::InjectorConfig
            | Self::CalibrationCommand
            | Self::Obd2Request
            | Self::Obd2Response => CanMessagePriority::Configuration,
            Self::Heartbeat => CanMessagePriority::Heartbeat,
        }
    }
}

/// Product-owned explicit CAN arbitration route.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanMessageRoute {
    pub class: CanMessageClass,
    pub priority: CanMessagePriority,
    pub arbitration_id: u32,
}

impl CanMessageRoute {
    pub const fn new(class: CanMessageClass) -> Self {
        Self {
            class,
            priority: class.priority(),
            arbitration_id: class.arbitration_id(),
        }
    }
}

pub const CAN_ROUTE_MAP: [CanMessageRoute; CAN_MESSAGE_CLASS_COUNT] = [
    CanMessageRoute::new(CanMessageClass::Fault),
    CanMessageRoute::new(CanMessageClass::ResetCommand),
    CanMessageRoute::new(CanMessageClass::EngineControlCommand),
    CanMessageRoute::new(CanMessageClass::TriggerTiming),
    CanMessageRoute::new(CanMessageClass::SensorData),
    CanMessageRoute::new(CanMessageClass::GpsData),
    CanMessageRoute::new(CanMessageClass::ExternalEgtData),
    CanMessageRoute::new(CanMessageClass::FuelTable),
    CanMessageRoute::new(CanMessageClass::IgnitionTable),
    CanMessageRoute::new(CanMessageClass::EngineConfig),
    CanMessageRoute::new(CanMessageClass::InjectorConfig),
    CanMessageRoute::new(CanMessageClass::CalibrationCommand),
    CanMessageRoute::new(CanMessageClass::Obd2Request),
    CanMessageRoute::new(CanMessageClass::Obd2Response),
    CanMessageRoute::new(CanMessageClass::Heartbeat),
];

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanFilterPolicyGroup {
    CoreControl,
    CalibrationDiagnostic,
    OptionalTelemetry,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanFilterPolicyEntry {
    pub class: CanMessageClass,
    pub group: CanFilterPolicyGroup,
}

pub const CAN_FILTER_POLICY_MAP: [CanFilterPolicyEntry; CAN_MESSAGE_CLASS_COUNT] = [
    CanFilterPolicyEntry {
        class: CanMessageClass::Fault,
        group: CanFilterPolicyGroup::CoreControl,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::ResetCommand,
        group: CanFilterPolicyGroup::CoreControl,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::EngineControlCommand,
        group: CanFilterPolicyGroup::CoreControl,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::TriggerTiming,
        group: CanFilterPolicyGroup::CoreControl,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::SensorData,
        group: CanFilterPolicyGroup::CoreControl,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::GpsData,
        group: CanFilterPolicyGroup::OptionalTelemetry,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::ExternalEgtData,
        group: CanFilterPolicyGroup::OptionalTelemetry,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::FuelTable,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::IgnitionTable,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::EngineConfig,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::InjectorConfig,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::CalibrationCommand,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::Obd2Request,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::Obd2Response,
        group: CanFilterPolicyGroup::CalibrationDiagnostic,
    },
    CanFilterPolicyEntry {
        class: CanMessageClass::Heartbeat,
        group: CanFilterPolicyGroup::CoreControl,
    },
];

pub const CAN_STANDARD_DEVICE_PROFILE_COUNT: usize = 6;
const CAN_STANDARD_DEVICE_MESSAGE_CLASS_CAPACITY: usize = 4;
const CAN_STANDARD_DEVICE_FILTER_GROUP_CAPACITY: usize = 2;

/// Product-owned external CAN device classes above raw message routes.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanStandardDeviceClass {
    Dash,
    Logger,
    Pdm,
    Gps,
    ExternalLambda,
    ExternalEgt,
}

/// Current transport-level compatibility verdict for a standard CAN device class.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanStandardDeviceSupport {
    Supported,
    UnsupportedMissingMessageClass,
}

/// Product-owned bounded compatibility profile for a standard CAN device class.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanStandardDeviceProfile {
    pub device_class: CanStandardDeviceClass,
    pub support: CanStandardDeviceSupport,
    /// Message classes the ECU transport must be able to receive from this device class.
    pub required_rx_message_class_count: usize,
    pub required_rx_message_classes:
        [Option<CanMessageClass>; CAN_STANDARD_DEVICE_MESSAGE_CLASS_CAPACITY],
    /// Message classes the ECU transport must be able to transmit to this device class.
    pub required_tx_message_class_count: usize,
    pub required_tx_message_classes:
        [Option<CanMessageClass>; CAN_STANDARD_DEVICE_MESSAGE_CLASS_CAPACITY],
    pub required_filter_policy_group_count: usize,
    pub required_filter_policy_groups:
        [Option<CanFilterPolicyGroup>; CAN_STANDARD_DEVICE_FILTER_GROUP_CAPACITY],
    pub heartbeat_required: bool,
}

impl CanStandardDeviceProfile {
    pub const fn is_supported(self) -> bool {
        matches!(self.support, CanStandardDeviceSupport::Supported)
    }

    pub fn required_rx_message_class(self, index: usize) -> Option<CanMessageClass> {
        if index < self.required_rx_message_class_count {
            self.required_rx_message_classes[index]
        } else {
            None
        }
    }

    pub fn required_tx_message_class(self, index: usize) -> Option<CanMessageClass> {
        if index < self.required_tx_message_class_count {
            self.required_tx_message_classes[index]
        } else {
            None
        }
    }

    pub fn required_filter_policy_group(self, index: usize) -> Option<CanFilterPolicyGroup> {
        if index < self.required_filter_policy_group_count {
            self.required_filter_policy_groups[index]
        } else {
            None
        }
    }
}

impl CanStandardDeviceClass {
    pub const fn profile(self) -> CanStandardDeviceProfile {
        match self {
            Self::Dash => CanStandardDeviceProfile {
                device_class: Self::Dash,
                support: CanStandardDeviceSupport::Supported,
                required_rx_message_class_count: 0,
                required_rx_message_classes: [None, None, None, None],
                required_tx_message_class_count: 3,
                required_tx_message_classes: [
                    Some(CanMessageClass::SensorData),
                    Some(CanMessageClass::Fault),
                    Some(CanMessageClass::Heartbeat),
                    None,
                ],
                required_filter_policy_group_count: 1,
                required_filter_policy_groups: [Some(CanFilterPolicyGroup::CoreControl), None],
                heartbeat_required: true,
            },
            Self::Logger => CanStandardDeviceProfile {
                device_class: Self::Logger,
                support: CanStandardDeviceSupport::Supported,
                required_rx_message_class_count: 0,
                required_rx_message_classes: [None, None, None, None],
                required_tx_message_class_count: 4,
                required_tx_message_classes: [
                    Some(CanMessageClass::TriggerTiming),
                    Some(CanMessageClass::SensorData),
                    Some(CanMessageClass::Fault),
                    Some(CanMessageClass::Heartbeat),
                ],
                required_filter_policy_group_count: 1,
                required_filter_policy_groups: [Some(CanFilterPolicyGroup::CoreControl), None],
                heartbeat_required: true,
            },
            Self::Pdm => CanStandardDeviceProfile {
                device_class: Self::Pdm,
                support: CanStandardDeviceSupport::Supported,
                required_rx_message_class_count: 2,
                required_rx_message_classes: [
                    Some(CanMessageClass::Fault),
                    Some(CanMessageClass::Heartbeat),
                    None,
                    None,
                ],
                required_tx_message_class_count: 2,
                required_tx_message_classes: [
                    Some(CanMessageClass::EngineControlCommand),
                    Some(CanMessageClass::ResetCommand),
                    None,
                    None,
                ],
                required_filter_policy_group_count: 1,
                required_filter_policy_groups: [Some(CanFilterPolicyGroup::CoreControl), None],
                heartbeat_required: true,
            },
            Self::Gps => CanStandardDeviceProfile {
                device_class: Self::Gps,
                support: CanStandardDeviceSupport::Supported,
                required_rx_message_class_count: 2,
                required_rx_message_classes: [
                    Some(CanMessageClass::GpsData),
                    Some(CanMessageClass::Heartbeat),
                    None,
                    None,
                ],
                required_tx_message_class_count: 0,
                required_tx_message_classes: [None, None, None, None],
                required_filter_policy_group_count: 1,
                required_filter_policy_groups: [
                    Some(CanFilterPolicyGroup::OptionalTelemetry),
                    None,
                ],
                heartbeat_required: true,
            },
            Self::ExternalLambda => CanStandardDeviceProfile {
                device_class: Self::ExternalLambda,
                support: CanStandardDeviceSupport::Supported,
                required_rx_message_class_count: 2,
                required_rx_message_classes: [
                    Some(CanMessageClass::SensorData),
                    Some(CanMessageClass::Heartbeat),
                    None,
                    None,
                ],
                required_tx_message_class_count: 0,
                required_tx_message_classes: [None, None, None, None],
                required_filter_policy_group_count: 1,
                required_filter_policy_groups: [Some(CanFilterPolicyGroup::CoreControl), None],
                heartbeat_required: true,
            },
            Self::ExternalEgt => CanStandardDeviceProfile {
                device_class: Self::ExternalEgt,
                support: CanStandardDeviceSupport::Supported,
                required_rx_message_class_count: 2,
                required_rx_message_classes: [
                    Some(CanMessageClass::ExternalEgtData),
                    Some(CanMessageClass::Heartbeat),
                    None,
                    None,
                ],
                required_tx_message_class_count: 0,
                required_tx_message_classes: [None, None, None, None],
                required_filter_policy_group_count: 1,
                required_filter_policy_groups: [
                    Some(CanFilterPolicyGroup::OptionalTelemetry),
                    None,
                ],
                heartbeat_required: true,
            },
        }
    }
}

pub const CAN_STANDARD_DEVICE_PROFILE_MAP: [CanStandardDeviceProfile;
    CAN_STANDARD_DEVICE_PROFILE_COUNT] = [
    CanStandardDeviceClass::Dash.profile(),
    CanStandardDeviceClass::Logger.profile(),
    CanStandardDeviceClass::Pdm.profile(),
    CanStandardDeviceClass::Gps.profile(),
    CanStandardDeviceClass::ExternalLambda.profile(),
    CanStandardDeviceClass::ExternalEgt.profile(),
];

pub const CAN_OBD2_SUPPORTED_PID_COUNT: usize = 5;
pub const CAN_OBD2_SUPPORTED_INFO_TYPE_COUNT: usize = 5;
pub const CAN_OBD2_SEGMENTED_RESPONSE_CAPACITY: usize = 4;
pub const CAN_OBD2_VIN_LEN: usize = 17;
pub const CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID: u8 = 0xE1;
pub const CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN: u8 = 10;
pub const CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID: u8 = 0xE2;
pub const CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN: u8 = 6;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2ServiceDirection {
    Request,
    Response,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2ResponseKind {
    Positive,
    Negative,
    Pending,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2ServiceSurface {
    pub direction: CanObd2ServiceDirection,
    pub service_id: u8,
    pub parameter_id: Option<u8>,
    pub payload_len: u8,
    pub expects_response: bool,
    pub response_kind: Option<CanObd2ResponseKind>,
}

impl CanObd2ServiceSurface {
    pub fn from_message(message: &Message) -> Option<Self> {
        match message {
            Message::Obd2Request {
                service,
                parameter_id,
                payload_len,
                ..
            } => Some(Self {
                direction: CanObd2ServiceDirection::Request,
                service_id: *service,
                parameter_id: *parameter_id,
                payload_len: *payload_len,
                expects_response: true,
                response_kind: None,
            }),
            Message::Obd2Response {
                service,
                parameter_id,
                negative_response_code,
                payload_len,
                ..
            } => Some(Self {
                direction: CanObd2ServiceDirection::Response,
                service_id: *service,
                parameter_id: *parameter_id,
                payload_len: *payload_len,
                expects_response: false,
                response_kind: Some(match negative_response_code {
                    Some(0x78) => CanObd2ResponseKind::Pending,
                    Some(_) => CanObd2ResponseKind::Negative,
                    None => CanObd2ResponseKind::Positive,
                }),
            }),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2Pid {
    MonitorStatusSinceDtcsCleared,
    EngineCoolantTemperature,
    IntakeAirTemperature,
    ThrottlePosition,
    ControlModuleVoltage,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2PidBacking {
    Message(CanMessageClass),
    ReadinessMonitor,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2SupportedPidProfile {
    pub service_id: u8,
    pub pid: CanObd2Pid,
    pub pid_id: u8,
    pub backing: CanObd2PidBacking,
    pub response_payload_len: u8,
}

impl CanObd2Pid {
    pub const fn pid_id(self) -> u8 {
        match self {
            Self::MonitorStatusSinceDtcsCleared => 0x01,
            Self::EngineCoolantTemperature => 0x05,
            Self::IntakeAirTemperature => 0x0F,
            Self::ThrottlePosition => 0x11,
            Self::ControlModuleVoltage => 0x42,
        }
    }

    pub const fn backing(self) -> CanObd2PidBacking {
        match self {
            Self::MonitorStatusSinceDtcsCleared => CanObd2PidBacking::ReadinessMonitor,
            Self::EngineCoolantTemperature
            | Self::IntakeAirTemperature
            | Self::ThrottlePosition
            | Self::ControlModuleVoltage => CanObd2PidBacking::Message(CanMessageClass::SensorData),
        }
    }

    pub const fn response_payload_len(self) -> u8 {
        match self {
            Self::MonitorStatusSinceDtcsCleared => 4,
            Self::ControlModuleVoltage => 2,
            Self::EngineCoolantTemperature
            | Self::IntakeAirTemperature
            | Self::ThrottlePosition => 1,
        }
    }

    pub const fn profile(self) -> CanObd2SupportedPidProfile {
        CanObd2SupportedPidProfile {
            service_id: 0x01,
            pid: self,
            pid_id: self.pid_id(),
            backing: self.backing(),
            response_payload_len: self.response_payload_len(),
        }
    }
}

pub const CAN_OBD2_SUPPORTED_PID_CATALOG: [CanObd2SupportedPidProfile;
    CAN_OBD2_SUPPORTED_PID_COUNT] = [
    CanObd2Pid::MonitorStatusSinceDtcsCleared.profile(),
    CanObd2Pid::EngineCoolantTemperature.profile(),
    CanObd2Pid::IntakeAirTemperature.profile(),
    CanObd2Pid::ThrottlePosition.profile(),
    CanObd2Pid::ControlModuleVoltage.profile(),
];

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2InfoType {
    Vin,
    CalibrationId,
    EcuName,
    IdentityKeyLifecycle,
    FlashWriteFault,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2SupportedInfoTypeProfile {
    pub service_id: u8,
    pub info_type: CanObd2InfoType,
    pub info_type_id: u8,
    pub response_payload_len: u8,
}

impl CanObd2InfoType {
    pub const fn info_type_id(self) -> u8 {
        match self {
            Self::Vin => 0x02,
            Self::CalibrationId => 0x04,
            Self::EcuName => 0x0A,
            Self::IdentityKeyLifecycle => CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID,
            Self::FlashWriteFault => CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID,
        }
    }

    pub const fn response_payload_len(self) -> u8 {
        match self {
            Self::Vin | Self::CalibrationId => CAN_OBD2_VIN_LEN as u8,
            Self::EcuName => 6,
            Self::IdentityKeyLifecycle => CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
            Self::FlashWriteFault => CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN,
        }
    }

    pub const fn profile(self) -> CanObd2SupportedInfoTypeProfile {
        CanObd2SupportedInfoTypeProfile {
            service_id: 0x09,
            info_type: self,
            info_type_id: self.info_type_id(),
            response_payload_len: self.response_payload_len(),
        }
    }
}

pub const CAN_OBD2_SUPPORTED_INFO_TYPE_CATALOG: [CanObd2SupportedInfoTypeProfile;
    CAN_OBD2_SUPPORTED_INFO_TYPE_COUNT] = [
    CanObd2InfoType::Vin.profile(),
    CanObd2InfoType::CalibrationId.profile(),
    CanObd2InfoType::EcuName.profile(),
    CanObd2InfoType::IdentityKeyLifecycle.profile(),
    CanObd2InfoType::FlashWriteFault.profile(),
];

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2SupportedPidBitmapSurface {
    pub service_id: u8,
    pub start_pid_id: u8,
    pub bitmap: [u8; 4],
}

impl CanObd2SupportedPidBitmapSurface {
    pub fn from_pid_block(start_pid_id: u8) -> Option<Self> {
        if start_pid_id & 0x1f != 0 {
            return None;
        }

        let mut bits = 0u32;
        let mut any_supported = false;
        for profile in CAN_OBD2_SUPPORTED_PID_CATALOG {
            if profile.service_id != 0x01 {
                continue;
            }
            let first_pid = start_pid_id.saturating_add(1);
            let last_pid = start_pid_id.saturating_add(0x20);
            if !(first_pid..=last_pid).contains(&profile.pid_id) {
                continue;
            }
            let offset = profile.pid_id - first_pid;
            bits |= 1u32 << (31 - offset);
            any_supported = true;
        }

        if !any_supported {
            return None;
        }

        Some(Self {
            service_id: 0x01,
            start_pid_id,
            bitmap: bits.to_be_bytes(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2SupportedInfoTypeBitmapSurface {
    pub service_id: u8,
    pub start_info_type_id: u8,
    pub bitmap: [u8; 4],
}

impl CanObd2SupportedInfoTypeBitmapSurface {
    pub fn from_info_type_block(start_info_type_id: u8) -> Option<Self> {
        if start_info_type_id & 0x1f != 0 {
            return None;
        }

        let mut bits = 0u32;
        let mut any_supported = false;
        for profile in CAN_OBD2_SUPPORTED_INFO_TYPE_CATALOG {
            if profile.service_id != 0x09 {
                continue;
            }
            let first_info_type = start_info_type_id.saturating_add(1);
            let last_info_type = start_info_type_id.saturating_add(0x20);
            if !(first_info_type..=last_info_type).contains(&profile.info_type_id) {
                continue;
            }
            let offset = profile.info_type_id - first_info_type;
            bits |= 1u32 << (31 - offset);
            any_supported = true;
        }

        if !any_supported {
            return None;
        }

        Some(Self {
            service_id: 0x09,
            start_info_type_id,
            bitmap: bits.to_be_bytes(),
        })
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2ValueProjectionField {
    EngineCoolantTemperatureOffset,
    IntakeAirTemperatureOffset,
    ThrottlePercent,
    ControlModuleVoltageDecivolts,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2ValueProjectionEncoding {
    RawOffsetByte,
    PercentToObdByte,
    DecivoltsToMillivoltsWordBe,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2ValueProjectionSurface {
    pub pid: CanObd2Pid,
    pub backing_message_class: CanMessageClass,
    pub payload_len: u8,
    pub field: CanObd2ValueProjectionField,
    pub encoding: CanObd2ValueProjectionEncoding,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2ProjectedPayload {
    pub payload_len: u8,
    pub bytes: [u8; 2],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2ReadinessMonitorInputs {
    pub mil_requested: bool,
    pub stored_dtc_count: u8,
    pub misfire_supported: bool,
    pub misfire_complete: bool,
    pub fuel_system_supported: bool,
    pub fuel_system_complete: bool,
    pub comprehensive_components_supported: bool,
    pub comprehensive_components_complete: bool,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2ReadinessMonitorMeaning {
    pub mil_requested: bool,
    pub stored_dtc_count: u8,
    pub misfire_supported: bool,
    pub misfire_complete: bool,
    pub fuel_system_supported: bool,
    pub fuel_system_complete: bool,
    pub comprehensive_components_supported: bool,
    pub comprehensive_components_complete: bool,
}

impl CanObd2ReadinessMonitorInputs {
    pub const fn into_meaning(self) -> CanObd2ReadinessMonitorMeaning {
        let stored_dtc_count = if self.stored_dtc_count > 0x7f {
            0x7f
        } else {
            self.stored_dtc_count
        };
        CanObd2ReadinessMonitorMeaning {
            mil_requested: self.mil_requested,
            stored_dtc_count,
            misfire_supported: self.misfire_supported,
            misfire_complete: self.misfire_complete,
            fuel_system_supported: self.fuel_system_supported,
            fuel_system_complete: self.fuel_system_complete,
            comprehensive_components_supported: self.comprehensive_components_supported,
            comprehensive_components_complete: self.comprehensive_components_complete,
        }
    }
}

impl CanObd2ReadinessMonitorMeaning {
    pub const fn encode_payload(self) -> [u8; 4] {
        let mut byte0 = self.stored_dtc_count & 0x7f;
        if self.mil_requested {
            byte0 |= 0x80;
        }

        let mut byte1 = 0u8;
        if self.misfire_supported {
            byte1 |= 1 << 7;
            if !self.misfire_complete {
                byte1 |= 1 << 2;
            }
        }
        if self.fuel_system_supported {
            byte1 |= 1 << 6;
            if !self.fuel_system_complete {
                byte1 |= 1 << 1;
            }
        }
        if self.comprehensive_components_supported {
            byte1 |= 1 << 5;
            if !self.comprehensive_components_complete {
                byte1 |= 1;
            }
        }

        [byte0, byte1, 0, 0]
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2ReadinessMonitorResponseError {
    NotObd2Request,
    UnsupportedService { service_id: u8 },
    MissingParameterId,
    UnsupportedPid { pid_id: u8 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2ReadinessMonitorSurface {
    pub request_service_id: u8,
    pub parameter_id: u8,
    pub meaning: CanObd2ReadinessMonitorMeaning,
    pub response: Message,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2DtcClearInputs {
    pub cleared_dtc_count: u8,
    pub freeze_frame_cleared: bool,
    pub readiness_reset: bool,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2DtcClearVerdict {
    pub cleared_dtc_count: u8,
    pub freeze_frame_cleared: bool,
    pub readiness_reset: bool,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2DtcClearResponseError {
    NotObd2Request,
    UnsupportedService { service_id: u8 },
    UnexpectedParameterId { parameter_id: u8 },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2DtcClearSurface {
    pub request_service_id: u8,
    pub verdict: CanObd2DtcClearVerdict,
    pub response: Message,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2ResponseAssemblyError {
    UnsupportedService {
        service_id: u8,
    },
    MissingParameterId,
    UnsupportedPid {
        pid_id: u8,
    },
    IncompatibleValueSource {
        backing_message_class: CanMessageClass,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2ResponseAssemblySurface {
    pub request_service_id: u8,
    pub pid: CanObd2Pid,
    pub positive_response_service_id: u8,
    pub payload: CanObd2ProjectedPayload,
    pub response: Message,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2SupportedPidDiscoveryResponseError {
    UnsupportedService { service_id: u8 },
    MissingParameterId,
    UnsupportedPidBlock { start_pid_id: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2VehicleInfoInputs {
    pub ecu_name_len: u8,
    pub ecu_name: [u8; 6],
    pub vin_len: u8,
    pub vin: [u8; CAN_OBD2_VIN_LEN],
    pub calibration_id_len: u8,
    pub calibration_id: [u8; CAN_OBD2_VIN_LEN],
    pub identity_key_lifecycle: Option<CanObd2IdentityKeyLifecycleStatus>,
    pub flash_write_fault: Option<CanObd2FlashWriteFaultStatus>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2IdentityKeyLifecycleStatus {
    pub present: bool,
    pub persisted: bool,
    pub request_id: u32,
    pub authorized: bool,
    pub accepted: bool,
    pub store_failed: bool,
    pub rejected_reason: u8,
    pub generation: u32,
}

impl CanObd2IdentityKeyLifecycleStatus {
    pub const fn absent() -> Self {
        Self {
            present: false,
            persisted: false,
            request_id: 0,
            authorized: false,
            accepted: false,
            store_failed: false,
            rejected_reason: 0,
            generation: 0,
        }
    }

    pub const fn flags(self) -> u8 {
        (self.authorized as u8)
            | ((self.accepted as u8) << 1)
            | ((self.store_failed as u8) << 2)
            | ((self.persisted as u8) << 6)
            | ((self.present as u8) << 7)
    }

    pub fn payload(self) -> [u8; CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN as usize] {
        let mut payload = [0u8; CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN as usize];
        payload[0..4].copy_from_slice(&self.request_id.to_be_bytes());
        payload[4] = self.flags();
        payload[5] = self.rejected_reason;
        payload[6..10].copy_from_slice(&self.generation.to_be_bytes());
        payload
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2FlashWriteFaultPhase {
    None,
    Preflight,
    Erase,
    Program,
}

impl CanObd2FlashWriteFaultPhase {
    pub const fn code(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Preflight => 1,
            Self::Erase => 2,
            Self::Program => 3,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2FlashWriteFaultStatus {
    pub present: bool,
    pub phase: CanObd2FlashWriteFaultPhase,
    pub sr_bits: u32,
}

impl CanObd2FlashWriteFaultStatus {
    pub const fn absent() -> Self {
        Self {
            present: false,
            phase: CanObd2FlashWriteFaultPhase::None,
            sr_bits: 0,
        }
    }

    pub fn payload(self) -> [u8; CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN as usize] {
        let mut payload = [0u8; CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN as usize];
        payload[0] = (self.present as u8) << 7;
        payload[1] = self.phase.code();
        payload[2..6].copy_from_slice(&self.sr_bits.to_be_bytes());
        payload
    }
}

impl CanObd2VehicleInfoInputs {
    pub const fn default_identity() -> Self {
        Self {
            ecu_name_len: 6,
            ecu_name: *b"PIPOCO",
            vin_len: CAN_OBD2_VIN_LEN as u8,
            vin: *b"PIPOCO00000000000",
            calibration_id_len: CAN_OBD2_VIN_LEN as u8,
            calibration_id: *b"PIPOCO00000000000",
            identity_key_lifecycle: Some(CanObd2IdentityKeyLifecycleStatus::absent()),
            flash_write_fault: None,
        }
    }

    pub const fn normalized(self) -> Self {
        let ecu_name_len = if self.ecu_name_len > 6 {
            6
        } else {
            self.ecu_name_len
        };
        let vin_len = if self.vin_len > CAN_OBD2_VIN_LEN as u8 {
            CAN_OBD2_VIN_LEN as u8
        } else {
            self.vin_len
        };
        let calibration_id_len = if self.calibration_id_len > CAN_OBD2_VIN_LEN as u8 {
            CAN_OBD2_VIN_LEN as u8
        } else {
            self.calibration_id_len
        };
        Self {
            ecu_name_len,
            ecu_name: self.ecu_name,
            vin_len,
            vin: self.vin,
            calibration_id_len,
            calibration_id: self.calibration_id,
            identity_key_lifecycle: self.identity_key_lifecycle,
            flash_write_fault: self.flash_write_fault,
        }
    }
}

impl Default for CanObd2VehicleInfoInputs {
    fn default() -> Self {
        Self::default_identity()
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2VehicleInfoPayloadMeaning {
    SupportedInfoTypes {
        start_info_type_id: u8,
        bitmap: [u8; 4],
    },
    EcuName {
        bytes: [u8; 6],
        len: u8,
    },
    IdentityKeyLifecycleMissing,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2SegmentedVehicleInfoMeaning {
    Vin {
        bytes: [u8; CAN_OBD2_VIN_LEN],
        len: u8,
    },
    CalibrationId {
        bytes: [u8; CAN_OBD2_VIN_LEN],
        len: u8,
    },
    IdentityKeyLifecycle {
        status: CanObd2IdentityKeyLifecycleStatus,
        payload: [u8; CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN as usize],
    },
    FlashWriteFault {
        status: CanObd2FlashWriteFaultStatus,
        payload: [u8; CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN as usize],
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2VehicleInfoResponseError {
    NotObd2Request,
    UnsupportedService { service_id: u8 },
    MissingInfoTypeId,
    UnsupportedInfoType { info_type_id: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2SegmentedVehicleInfoResponseError {
    NotObd2Request,
    UnsupportedService { service_id: u8 },
    MissingInfoTypeId,
    UnsupportedInfoType { info_type_id: u8 },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2NegativeResponseCode {
    ServiceNotSupported,
    RequestOutOfRange,
}

impl CanObd2NegativeResponseCode {
    pub const fn raw(self) -> u8 {
        match self {
            Self::ServiceNotSupported => 0x11,
            Self::RequestOutOfRange => 0x31,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2NegativeResponseSurface {
    pub request_service_id: u8,
    pub code: CanObd2NegativeResponseCode,
    pub response: Message,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2NegativeResponseSurfaceError {
    NotObd2Request,
    UnsupportedReason,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2RequestDispatchVerdict {
    CurrentDataPositive,
    ReadinessMonitorPositive,
    SupportedPidDiscoveryPositive,
    NegativeResponse(CanObd2NegativeResponseCode),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2RequestDispatchError {
    NotObd2Request,
    MissingParameterId,
    MissingReadinessInputs,
    MissingValueSource {
        backing_message_class: CanMessageClass,
    },
    IncompatibleValueSource {
        backing_message_class: CanMessageClass,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2RequestDispatchSurface {
    pub request_service_id: u8,
    pub parameter_id: u8,
    pub verdict: CanObd2RequestDispatchVerdict,
    pub response: Message,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2StoredDtc {
    pub diag_code: DiagCode,
    pub raw_bytes: [u8; 2],
}

impl CanObd2StoredDtc {
    pub const fn from_diag_code(diag_code: DiagCode) -> Self {
        let raw_bytes = match diag_code {
            DiagCode::MapRange => [0x01, 0x08],
            DiagCode::TpsRange => [0x01, 0x22],
            DiagCode::CamMissing => [0x03, 0x40],
            DiagCode::LowVoltage => [0x05, 0x62],
            DiagCode::Overvoltage => [0x05, 0x63],
            DiagCode::MapFailureHighLoad => [0x01, 0x07],
            DiagCode::TpsMapPlausibility => [0x00, 0x68],
            DiagCode::KnockDetected => [0x03, 0x25],
            DiagCode::PersistCrcFault => [0x06, 0x01],
            DiagCode::OilPressureLow => [0x05, 0x20],
            DiagCode::FuelPressureLow => [0x01, 0x92],
            DiagCode::LambdaInvalid => [0x01, 0x30],
        };
        Self {
            diag_code,
            raw_bytes,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2FreezeFrameSnapshot {
    pub dtc: CanObd2StoredDtc,
    pub pid: CanObd2Pid,
    pub payload: CanObd2ProjectedPayload,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2DtcFreezeFramePayloadMeaning {
    StoredDtcs {
        dtc_count: u8,
        dtcs: [Option<CanObd2StoredDtc>; 3],
    },
    FreezeFrame(CanObd2FreezeFrameSnapshot),
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2DtcFreezeFrameResponseError {
    NotObd2Request,
    UnsupportedService {
        service_id: u8,
    },
    MissingParameterId,
    UnsupportedPid {
        pid_id: u8,
    },
    MissingFreezeFrameValueSource {
        backing_message_class: CanMessageClass,
    },
    IncompatibleFreezeFrameValueSource {
        backing_message_class: CanMessageClass,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2DtcFreezeFrameResponseSurface {
    pub request_service_id: u8,
    pub parameter_id: Option<u8>,
    pub meaning: CanObd2DtcFreezeFramePayloadMeaning,
    pub response: Message,
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct CanObd2MultiServiceDispatchInputs<'a> {
    pub current_data_value_source: Option<&'a Message>,
    pub readiness_monitor: Option<CanObd2ReadinessMonitorInputs>,
    pub dtc_clear: Option<CanObd2DtcClearInputs>,
    pub freeze_frame_value_source: Option<&'a Message>,
    pub stored_dtcs: &'a [DiagCode],
    pub freeze_frame_dtc: Option<DiagCode>,
    pub vehicle_info: CanObd2VehicleInfoInputs,
}

impl<'a> CanObd2MultiServiceDispatchInputs<'a> {
    pub const fn empty() -> Self {
        Self {
            current_data_value_source: None,
            readiness_monitor: None,
            dtc_clear: None,
            freeze_frame_value_source: None,
            stored_dtcs: &[],
            freeze_frame_dtc: None,
            vehicle_info: CanObd2VehicleInfoInputs::default_identity(),
        }
    }
}

impl<'a> Default for CanObd2MultiServiceDispatchInputs<'a> {
    fn default() -> Self {
        Self::empty()
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2MultiServiceDispatchVerdict {
    CurrentDataPositive,
    ReadinessMonitorPositive,
    SupportedPidDiscoveryPositive,
    VehicleInfoPositive,
    SegmentedVehicleInfoPositive,
    NegativeResponse(CanObd2NegativeResponseCode),
    DtcClearPositive,
    StoredDtcs,
    FreezeFrame,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanObd2MultiServiceDispatchError {
    NotObd2Request,
    UnsupportedService { service_id: u8 },
    MissingFreezeFrameDtc,
    MissingDtcClearInputs,
    Mode01(CanObd2RequestDispatchError),
    DtcClear(CanObd2DtcClearResponseError),
    DtcFreezeFrame(CanObd2DtcFreezeFrameResponseError),
    VehicleInfo(CanObd2VehicleInfoResponseError),
    SegmentedVehicleInfo(CanObd2SegmentedVehicleInfoResponseError),
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2MultiServiceDispatchSurface {
    pub request_service_id: u8,
    pub parameter_id: Option<u8>,
    pub verdict: CanObd2MultiServiceDispatchVerdict,
    pub response: CanObd2ResponseFrame,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2ResponseFrame {
    pub service: u8,
    pub parameter_id: Option<u8>,
    pub negative_response_code: Option<u8>,
    pub payload_len: u8,
    pub payload: [u8; 6],
}

impl CanObd2ResponseFrame {
    pub const fn to_message(self) -> Message {
        Message::Obd2Response {
            service: self.service,
            parameter_id: self.parameter_id,
            negative_response_code: self.negative_response_code,
            payload_len: self.payload_len,
            payload: self.payload,
        }
    }

    fn from_message(response: Message) -> Self {
        match response {
            Message::Obd2Response {
                service,
                parameter_id,
                negative_response_code,
                payload_len,
                payload,
            } => Self {
                service,
                parameter_id,
                negative_response_code,
                payload_len,
                payload,
            },
            _ => unreachable!("OBD2 dispatch surfaces only carry OBD2 responses"),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2SupportedPidDiscoveryResponseSurface {
    pub request_service_id: u8,
    pub start_pid_id: u8,
    pub positive_response_service_id: u8,
    pub bitmap: [u8; 4],
    pub response: Message,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2VehicleInfoResponseSurface {
    pub request_service_id: u8,
    pub info_type_id: u8,
    pub positive_response_service_id: u8,
    pub meaning: CanObd2VehicleInfoPayloadMeaning,
    pub response: Message,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CanObd2SegmentedVehicleInfoResponseSurface {
    pub request_service_id: u8,
    pub info_type_id: u8,
    pub positive_response_service_id: u8,
    pub meaning: CanObd2SegmentedVehicleInfoMeaning,
    pub total_payload_len: u8,
    pub segment_count: u8,
    pub segments: [Option<CanObd2SegmentedResponseFrame>; CAN_OBD2_SEGMENTED_RESPONSE_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanObd2SegmentedResponseFrame {
    pub service: u8,
    pub parameter_id: Option<u8>,
    pub sequence_index: u8,
    pub segment_count: u8,
    pub total_payload_len: u8,
    pub segment_len: u8,
    pub segment: [u8; 6],
}

impl CanObd2SegmentedResponseFrame {
    pub const fn to_message(self) -> Message {
        Message::Obd2SegmentedResponse {
            service: self.service,
            parameter_id: self.parameter_id,
            sequence_index: self.sequence_index,
            segment_count: self.segment_count,
            total_payload_len: self.total_payload_len,
            segment_len: self.segment_len,
            segment: self.segment,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum CanObd2MultiServiceDispatchOutcome {
    Single(CanObd2MultiServiceDispatchSurface),
    SegmentedVehicleInfo(CanObd2SegmentedVehicleInfoResponseSurface),
}

impl CanObd2Pid {
    pub const fn value_projection(self) -> Option<CanObd2ValueProjectionSurface> {
        match self {
            Self::MonitorStatusSinceDtcsCleared => None,
            Self::EngineCoolantTemperature => Some(CanObd2ValueProjectionSurface {
                pid: self,
                backing_message_class: CanMessageClass::SensorData,
                payload_len: 1,
                field: CanObd2ValueProjectionField::EngineCoolantTemperatureOffset,
                encoding: CanObd2ValueProjectionEncoding::RawOffsetByte,
            }),
            Self::IntakeAirTemperature => Some(CanObd2ValueProjectionSurface {
                pid: self,
                backing_message_class: CanMessageClass::SensorData,
                payload_len: 1,
                field: CanObd2ValueProjectionField::IntakeAirTemperatureOffset,
                encoding: CanObd2ValueProjectionEncoding::RawOffsetByte,
            }),
            Self::ThrottlePosition => Some(CanObd2ValueProjectionSurface {
                pid: self,
                backing_message_class: CanMessageClass::SensorData,
                payload_len: 1,
                field: CanObd2ValueProjectionField::ThrottlePercent,
                encoding: CanObd2ValueProjectionEncoding::PercentToObdByte,
            }),
            Self::ControlModuleVoltage => Some(CanObd2ValueProjectionSurface {
                pid: self,
                backing_message_class: CanMessageClass::SensorData,
                payload_len: 2,
                field: CanObd2ValueProjectionField::ControlModuleVoltageDecivolts,
                encoding: CanObd2ValueProjectionEncoding::DecivoltsToMillivoltsWordBe,
            }),
        }
    }
}

impl CanObd2ValueProjectionSurface {
    pub fn project_from_message(self, message: &Message) -> Option<CanObd2ProjectedPayload> {
        match message {
            Message::SensorData {
                tps_percent,
                iat_offset,
                clt_offset,
                voltage_x10,
                ..
            } => {
                let payload = match self.field {
                    CanObd2ValueProjectionField::EngineCoolantTemperatureOffset => {
                        CanObd2ProjectedPayload {
                            payload_len: 1,
                            bytes: [*clt_offset, 0],
                        }
                    }
                    CanObd2ValueProjectionField::IntakeAirTemperatureOffset => {
                        CanObd2ProjectedPayload {
                            payload_len: 1,
                            bytes: [*iat_offset, 0],
                        }
                    }
                    CanObd2ValueProjectionField::ThrottlePercent => {
                        let scaled = ((*tps_percent as u16) * 255 + 50) / 100;
                        CanObd2ProjectedPayload {
                            payload_len: 1,
                            bytes: [scaled.min(255) as u8, 0],
                        }
                    }
                    CanObd2ValueProjectionField::ControlModuleVoltageDecivolts => {
                        let millivolts = (*voltage_x10 as u16) * 100;
                        CanObd2ProjectedPayload {
                            payload_len: 2,
                            bytes: [(millivolts >> 8) as u8, (millivolts & 0xff) as u8],
                        }
                    }
                };
                Some(payload)
            }
            _ => None,
        }
    }
}

impl CanObd2ResponseAssemblySurface {
    pub fn assemble(
        request: &Message,
        value_source: &Message,
    ) -> Result<Self, CanObd2ResponseAssemblyError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2ResponseAssemblyError::UnsupportedService { service_id: 0 })?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2ResponseAssemblyError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }
        if request_surface.service_id != 0x01 {
            return Err(CanObd2ResponseAssemblyError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }

        let pid_id = request_surface
            .parameter_id
            .ok_or(CanObd2ResponseAssemblyError::MissingParameterId)?;
        let supported = CAN_OBD2_SUPPORTED_PID_CATALOG
            .iter()
            .find(|profile| {
                profile.service_id == request_surface.service_id && profile.pid_id == pid_id
            })
            .copied()
            .ok_or(CanObd2ResponseAssemblyError::UnsupportedPid { pid_id })?;

        let projection = supported
            .pid
            .value_projection()
            .ok_or(CanObd2ResponseAssemblyError::UnsupportedPid { pid_id })?;
        let payload = projection.project_from_message(value_source).ok_or(
            CanObd2ResponseAssemblyError::IncompatibleValueSource {
                backing_message_class: projection.backing_message_class,
            },
        )?;
        let positive_response_service_id = request_surface.service_id.wrapping_add(0x40);
        let mut response_payload = [0u8; 6];
        response_payload[..payload.payload_len as usize]
            .copy_from_slice(&payload.bytes[..payload.payload_len as usize]);
        let response = Message::Obd2Response {
            service: positive_response_service_id,
            parameter_id: Some(pid_id),
            negative_response_code: None,
            payload_len: payload.payload_len,
            payload: response_payload,
        };

        Ok(Self {
            request_service_id: request_surface.service_id,
            pid: supported.pid,
            positive_response_service_id,
            payload,
            response,
        })
    }
}

impl CanObd2SupportedPidDiscoveryResponseSurface {
    pub fn assemble(request: &Message) -> Result<Self, CanObd2SupportedPidDiscoveryResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request).ok_or(
            CanObd2SupportedPidDiscoveryResponseError::UnsupportedService { service_id: 0 },
        )?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(
                CanObd2SupportedPidDiscoveryResponseError::UnsupportedService {
                    service_id: request_surface.service_id,
                },
            );
        }
        if request_surface.service_id != 0x01 {
            return Err(
                CanObd2SupportedPidDiscoveryResponseError::UnsupportedService {
                    service_id: request_surface.service_id,
                },
            );
        }

        let start_pid_id = request_surface
            .parameter_id
            .ok_or(CanObd2SupportedPidDiscoveryResponseError::MissingParameterId)?;
        let bitmap = CanObd2SupportedPidBitmapSurface::from_pid_block(start_pid_id).ok_or(
            CanObd2SupportedPidDiscoveryResponseError::UnsupportedPidBlock { start_pid_id },
        )?;
        let positive_response_service_id = request_surface.service_id.wrapping_add(0x40);
        let mut response_payload = [0u8; 6];
        response_payload[..4].copy_from_slice(&bitmap.bitmap);
        let response = Message::Obd2Response {
            service: positive_response_service_id,
            parameter_id: Some(start_pid_id),
            negative_response_code: None,
            payload_len: 4,
            payload: response_payload,
        };

        Ok(Self {
            request_service_id: request_surface.service_id,
            start_pid_id,
            positive_response_service_id,
            bitmap: bitmap.bitmap,
            response,
        })
    }
}

impl CanObd2VehicleInfoResponseSurface {
    pub fn assemble(
        request: &Message,
        inputs: CanObd2VehicleInfoInputs,
    ) -> Result<Self, CanObd2VehicleInfoResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2VehicleInfoResponseError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2VehicleInfoResponseError::NotObd2Request);
        }
        if request_surface.service_id != 0x09 {
            return Err(CanObd2VehicleInfoResponseError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }

        let info_type_id = request_surface
            .parameter_id
            .ok_or(CanObd2VehicleInfoResponseError::MissingInfoTypeId)?;
        let positive_response_service_id = request_surface.service_id.wrapping_add(0x40);
        match info_type_id {
            block if block & 0x1f == 0 => {
                let bitmap =
                    CanObd2SupportedInfoTypeBitmapSurface::from_info_type_block(info_type_id)
                        .ok_or(CanObd2VehicleInfoResponseError::UnsupportedInfoType {
                            info_type_id,
                        })?;
                let mut bitmap_bytes = bitmap.bitmap;
                if info_type_id == 0xE0 && inputs.flash_write_fault.is_none() {
                    bitmap_bytes[0] &= !0x40;
                }
                let mut payload = [0u8; 6];
                payload[..4].copy_from_slice(&bitmap_bytes);
                Ok(Self {
                    request_service_id: request_surface.service_id,
                    info_type_id,
                    positive_response_service_id,
                    meaning: CanObd2VehicleInfoPayloadMeaning::SupportedInfoTypes {
                        start_info_type_id: info_type_id,
                        bitmap: bitmap_bytes,
                    },
                    response: Message::Obd2Response {
                        service: positive_response_service_id,
                        parameter_id: Some(info_type_id),
                        negative_response_code: None,
                        payload_len: 4,
                        payload,
                    },
                })
            }
            0x0A => {
                let inputs = inputs.normalized();
                Ok(Self {
                    request_service_id: request_surface.service_id,
                    info_type_id,
                    positive_response_service_id,
                    meaning: CanObd2VehicleInfoPayloadMeaning::EcuName {
                        bytes: inputs.ecu_name,
                        len: inputs.ecu_name_len,
                    },
                    response: Message::Obd2Response {
                        service: positive_response_service_id,
                        parameter_id: Some(info_type_id),
                        negative_response_code: None,
                        payload_len: inputs.ecu_name_len,
                        payload: inputs.ecu_name,
                    },
                })
            }
            _ => Err(CanObd2VehicleInfoResponseError::UnsupportedInfoType { info_type_id }),
        }
    }
}

impl CanObd2SegmentedVehicleInfoResponseSurface {
    pub fn assemble(
        request: &Message,
        inputs: CanObd2VehicleInfoInputs,
    ) -> Result<Self, CanObd2SegmentedVehicleInfoResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2SegmentedVehicleInfoResponseError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2SegmentedVehicleInfoResponseError::NotObd2Request);
        }
        if request_surface.service_id != 0x09 {
            return Err(
                CanObd2SegmentedVehicleInfoResponseError::UnsupportedService {
                    service_id: request_surface.service_id,
                },
            );
        }

        let info_type_id = request_surface
            .parameter_id
            .ok_or(CanObd2SegmentedVehicleInfoResponseError::MissingInfoTypeId)?;
        let inputs = inputs.normalized();
        let (source, total_payload_len, meaning) =
            if info_type_id == CanObd2InfoType::Vin.info_type_id() {
                (
                    inputs.vin,
                    inputs.vin_len,
                    CanObd2SegmentedVehicleInfoMeaning::Vin {
                        bytes: inputs.vin,
                        len: inputs.vin_len,
                    },
                )
            } else if info_type_id == CanObd2InfoType::CalibrationId.info_type_id() {
                (
                    inputs.calibration_id,
                    inputs.calibration_id_len,
                    CanObd2SegmentedVehicleInfoMeaning::CalibrationId {
                        bytes: inputs.calibration_id,
                        len: inputs.calibration_id_len,
                    },
                )
            } else if info_type_id == CanObd2InfoType::IdentityKeyLifecycle.info_type_id() {
                let status = inputs.identity_key_lifecycle.ok_or(
                    CanObd2SegmentedVehicleInfoResponseError::UnsupportedInfoType { info_type_id },
                )?;
                let lifecycle_payload = status.payload();
                let mut source = [0u8; CAN_OBD2_VIN_LEN];
                source[..lifecycle_payload.len()].copy_from_slice(&lifecycle_payload);
                (
                    source,
                    CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
                    CanObd2SegmentedVehicleInfoMeaning::IdentityKeyLifecycle {
                        status,
                        payload: lifecycle_payload,
                    },
                )
            } else if info_type_id == CanObd2InfoType::FlashWriteFault.info_type_id() {
                let status = inputs.flash_write_fault.ok_or(
                    CanObd2SegmentedVehicleInfoResponseError::UnsupportedInfoType { info_type_id },
                )?;
                let fault_payload = status.payload();
                let mut source = [0u8; CAN_OBD2_VIN_LEN];
                source[..fault_payload.len()].copy_from_slice(&fault_payload);
                (
                    source,
                    CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN,
                    CanObd2SegmentedVehicleInfoMeaning::FlashWriteFault {
                        status,
                        payload: fault_payload,
                    },
                )
            } else {
                return Err(
                    CanObd2SegmentedVehicleInfoResponseError::UnsupportedInfoType { info_type_id },
                );
            };
        let segment_count = if total_payload_len == 0 {
            0
        } else {
            usize::from(total_payload_len).div_ceil(6) as u8
        };
        let positive_response_service_id = request_surface.service_id.wrapping_add(0x40);
        let mut segments: [Option<CanObd2SegmentedResponseFrame>;
            CAN_OBD2_SEGMENTED_RESPONSE_CAPACITY] = core::array::from_fn(|_| None);
        let mut sequence_index = 0u8;
        while sequence_index < segment_count {
            let start = sequence_index as usize * 6;
            let remaining = total_payload_len as usize - start;
            let segment_len = remaining.min(6);
            let mut segment = [0u8; 6];
            segment[..segment_len].copy_from_slice(&source[start..start + segment_len]);
            segments[sequence_index as usize] = Some(CanObd2SegmentedResponseFrame {
                service: positive_response_service_id,
                parameter_id: Some(info_type_id),
                sequence_index,
                segment_count,
                total_payload_len,
                segment_len: segment_len as u8,
                segment,
            });
            sequence_index += 1;
        }

        Ok(Self {
            request_service_id: request_surface.service_id,
            info_type_id,
            positive_response_service_id,
            meaning,
            total_payload_len,
            segment_count,
            segments,
        })
    }
}

impl CanObd2NegativeResponseSurface {
    fn assemble_from_request_surface(
        request_surface: CanObd2ServiceSurface,
        code: CanObd2NegativeResponseCode,
    ) -> Self {
        Self {
            request_service_id: request_surface.service_id,
            code,
            response: Message::Obd2Response {
                service: 0x7F,
                parameter_id: Some(request_surface.service_id),
                negative_response_code: Some(code.raw()),
                payload_len: 0,
                payload: [0; 6],
            },
        }
    }

    pub fn from_current_data_error(
        request: &Message,
        error: CanObd2ResponseAssemblyError,
    ) -> Result<Self, CanObd2NegativeResponseSurfaceError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2NegativeResponseSurfaceError::NotObd2Request)?;
        let code = match error {
            CanObd2ResponseAssemblyError::UnsupportedService { .. } => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            CanObd2ResponseAssemblyError::UnsupportedPid { .. } => {
                CanObd2NegativeResponseCode::RequestOutOfRange
            }
            CanObd2ResponseAssemblyError::MissingParameterId
            | CanObd2ResponseAssemblyError::IncompatibleValueSource { .. } => {
                return Err(CanObd2NegativeResponseSurfaceError::UnsupportedReason);
            }
        };
        Ok(Self::assemble_from_request_surface(request_surface, code))
    }

    pub fn from_discovery_error(
        request: &Message,
        error: CanObd2SupportedPidDiscoveryResponseError,
    ) -> Result<Self, CanObd2NegativeResponseSurfaceError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2NegativeResponseSurfaceError::NotObd2Request)?;
        let code = match error {
            CanObd2SupportedPidDiscoveryResponseError::UnsupportedService { .. } => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            CanObd2SupportedPidDiscoveryResponseError::UnsupportedPidBlock { .. } => {
                CanObd2NegativeResponseCode::RequestOutOfRange
            }
            CanObd2SupportedPidDiscoveryResponseError::MissingParameterId => {
                return Err(CanObd2NegativeResponseSurfaceError::UnsupportedReason);
            }
        };
        Ok(Self::assemble_from_request_surface(request_surface, code))
    }

    pub fn from_vehicle_info_error(
        request: &Message,
        error: CanObd2VehicleInfoResponseError,
    ) -> Result<Self, CanObd2NegativeResponseSurfaceError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2NegativeResponseSurfaceError::NotObd2Request)?;
        let code = match error {
            CanObd2VehicleInfoResponseError::UnsupportedService { .. } => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            CanObd2VehicleInfoResponseError::UnsupportedInfoType { .. } => {
                CanObd2NegativeResponseCode::RequestOutOfRange
            }
            CanObd2VehicleInfoResponseError::NotObd2Request
            | CanObd2VehicleInfoResponseError::MissingInfoTypeId => {
                return Err(CanObd2NegativeResponseSurfaceError::UnsupportedReason);
            }
        };
        Ok(Self::assemble_from_request_surface(request_surface, code))
    }
}

impl CanObd2ReadinessMonitorSurface {
    pub fn assemble(
        request: &Message,
        inputs: CanObd2ReadinessMonitorInputs,
    ) -> Result<Self, CanObd2ReadinessMonitorResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2ReadinessMonitorResponseError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2ReadinessMonitorResponseError::NotObd2Request);
        }
        if request_surface.service_id != 0x01 {
            return Err(CanObd2ReadinessMonitorResponseError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }

        let parameter_id = request_surface
            .parameter_id
            .ok_or(CanObd2ReadinessMonitorResponseError::MissingParameterId)?;
        if parameter_id != CanObd2Pid::MonitorStatusSinceDtcsCleared.pid_id() {
            return Err(CanObd2ReadinessMonitorResponseError::UnsupportedPid {
                pid_id: parameter_id,
            });
        }

        let meaning = inputs.into_meaning();
        let encoded = meaning.encode_payload();
        let mut payload = [0u8; 6];
        payload[..4].copy_from_slice(&encoded);
        Ok(Self {
            request_service_id: request_surface.service_id,
            parameter_id,
            meaning,
            response: Message::Obd2Response {
                service: 0x41,
                parameter_id: Some(parameter_id),
                negative_response_code: None,
                payload_len: 4,
                payload,
            },
        })
    }
}

impl CanObd2DtcClearSurface {
    pub fn assemble(
        request: &Message,
        inputs: CanObd2DtcClearInputs,
    ) -> Result<Self, CanObd2DtcClearResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2DtcClearResponseError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2DtcClearResponseError::NotObd2Request);
        }
        if request_surface.service_id != 0x04 {
            return Err(CanObd2DtcClearResponseError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }
        if let Some(parameter_id) = request_surface.parameter_id {
            return Err(CanObd2DtcClearResponseError::UnexpectedParameterId { parameter_id });
        }

        Ok(Self {
            request_service_id: request_surface.service_id,
            verdict: CanObd2DtcClearVerdict {
                cleared_dtc_count: inputs.cleared_dtc_count,
                freeze_frame_cleared: inputs.freeze_frame_cleared,
                readiness_reset: inputs.readiness_reset,
            },
            response: Message::Obd2Response {
                service: 0x44,
                parameter_id: None,
                negative_response_code: None,
                payload_len: 0,
                payload: [0; 6],
            },
        })
    }
}

impl CanObd2RequestDispatchSurface {
    fn from_current_data(parameter_id: u8, positive: CanObd2ResponseAssemblySurface) -> Self {
        Self {
            request_service_id: positive.request_service_id,
            parameter_id,
            verdict: CanObd2RequestDispatchVerdict::CurrentDataPositive,
            response: positive.response,
        }
    }

    fn from_readiness(parameter_id: u8, readiness: CanObd2ReadinessMonitorSurface) -> Self {
        Self {
            request_service_id: readiness.request_service_id,
            parameter_id,
            verdict: CanObd2RequestDispatchVerdict::ReadinessMonitorPositive,
            response: readiness.response,
        }
    }

    fn from_discovery(
        parameter_id: u8,
        discovery: CanObd2SupportedPidDiscoveryResponseSurface,
    ) -> Self {
        Self {
            request_service_id: discovery.request_service_id,
            parameter_id,
            verdict: CanObd2RequestDispatchVerdict::SupportedPidDiscoveryPositive,
            response: discovery.response,
        }
    }

    fn from_negative(parameter_id: u8, negative: CanObd2NegativeResponseSurface) -> Self {
        Self {
            request_service_id: negative.request_service_id,
            parameter_id,
            verdict: CanObd2RequestDispatchVerdict::NegativeResponse(negative.code),
            response: negative.response,
        }
    }

    pub fn dispatch(
        request: &Message,
        value_source: Option<&Message>,
    ) -> Result<Self, CanObd2RequestDispatchError> {
        Self::dispatch_with_readiness(request, value_source, None)
    }

    pub fn dispatch_with_readiness(
        request: &Message,
        value_source: Option<&Message>,
        readiness_monitor: Option<CanObd2ReadinessMonitorInputs>,
    ) -> Result<Self, CanObd2RequestDispatchError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2RequestDispatchError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2RequestDispatchError::NotObd2Request);
        }

        let parameter_id = request_surface
            .parameter_id
            .ok_or(CanObd2RequestDispatchError::MissingParameterId)?;

        if request_surface.service_id != 0x01 {
            let negative = CanObd2NegativeResponseSurface::from_current_data_error(
                request,
                CanObd2ResponseAssemblyError::UnsupportedService {
                    service_id: request_surface.service_id,
                },
            )
            .map_err(|_| CanObd2RequestDispatchError::NotObd2Request)?;
            return Ok(Self::from_negative(parameter_id, negative));
        }

        if let Ok(discovery) = CanObd2SupportedPidDiscoveryResponseSurface::assemble(request) {
            return Ok(Self::from_discovery(parameter_id, discovery));
        }

        if let Some(profile) = CAN_OBD2_SUPPORTED_PID_CATALOG
            .iter()
            .find(|profile| {
                profile.service_id == request_surface.service_id && profile.pid_id == parameter_id
            })
            .copied()
        {
            match profile.backing {
                CanObd2PidBacking::Message(backing_message_class) => {
                    let value_source =
                        value_source.ok_or(CanObd2RequestDispatchError::MissingValueSource {
                            backing_message_class,
                        })?;
                    let positive =
                        match CanObd2ResponseAssemblySurface::assemble(request, value_source) {
                            Ok(positive) => positive,
                            Err(CanObd2ResponseAssemblyError::IncompatibleValueSource {
                                backing_message_class,
                            }) => {
                                return Err(CanObd2RequestDispatchError::IncompatibleValueSource {
                                    backing_message_class,
                                })
                            }
                            Err(CanObd2ResponseAssemblyError::MissingParameterId) => {
                                return Err(CanObd2RequestDispatchError::MissingParameterId)
                            }
                            Err(CanObd2ResponseAssemblyError::UnsupportedService {
                                service_id,
                            }) => {
                                let negative =
                                    CanObd2NegativeResponseSurface::from_current_data_error(
                                        request,
                                        CanObd2ResponseAssemblyError::UnsupportedService {
                                            service_id,
                                        },
                                    )
                                    .map_err(|_| CanObd2RequestDispatchError::NotObd2Request)?;
                                return Ok(Self::from_negative(parameter_id, negative));
                            }
                            Err(CanObd2ResponseAssemblyError::UnsupportedPid { pid_id }) => {
                                let negative =
                                    CanObd2NegativeResponseSurface::from_current_data_error(
                                        request,
                                        CanObd2ResponseAssemblyError::UnsupportedPid { pid_id },
                                    )
                                    .map_err(|_| CanObd2RequestDispatchError::NotObd2Request)?;
                                return Ok(Self::from_negative(parameter_id, negative));
                            }
                        };
                    return Ok(Self::from_current_data(parameter_id, positive));
                }
                CanObd2PidBacking::ReadinessMonitor => {
                    let readiness_monitor = readiness_monitor
                        .ok_or(CanObd2RequestDispatchError::MissingReadinessInputs)?;
                    let readiness =
                        CanObd2ReadinessMonitorSurface::assemble(request, readiness_monitor)
                            .map_err(|error| match error {
                                CanObd2ReadinessMonitorResponseError::NotObd2Request => {
                                    CanObd2RequestDispatchError::NotObd2Request
                                }
                                CanObd2ReadinessMonitorResponseError::MissingParameterId => {
                                    CanObd2RequestDispatchError::MissingParameterId
                                }
                                CanObd2ReadinessMonitorResponseError::UnsupportedService {
                                    ..
                                }
                                | CanObd2ReadinessMonitorResponseError::UnsupportedPid { .. } => {
                                    CanObd2RequestDispatchError::NotObd2Request
                                }
                            })?;
                    return Ok(Self::from_readiness(parameter_id, readiness));
                }
            }
        }

        let negative = CanObd2NegativeResponseSurface::from_current_data_error(
            request,
            CanObd2ResponseAssemblyError::UnsupportedPid {
                pid_id: parameter_id,
            },
        )
        .map_err(|_| CanObd2RequestDispatchError::NotObd2Request)?;
        Ok(Self::from_negative(parameter_id, negative))
    }
}

impl CanObd2DtcFreezeFrameResponseSurface {
    pub fn assemble_stored_dtcs(
        request: &Message,
        stored_dtcs: &[DiagCode],
    ) -> Result<Self, CanObd2DtcFreezeFrameResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2DtcFreezeFrameResponseError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2DtcFreezeFrameResponseError::NotObd2Request);
        }
        if request_surface.service_id != 0x03 {
            return Err(CanObd2DtcFreezeFrameResponseError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }

        let dtc_count = stored_dtcs.len().min(3) as u8;
        let mut dtcs = [None; 3];
        let mut payload = [0u8; 6];
        let mut idx = 0usize;
        while idx < dtc_count as usize {
            let stored = CanObd2StoredDtc::from_diag_code(stored_dtcs[idx]);
            dtcs[idx] = Some(stored);
            payload[idx * 2] = stored.raw_bytes[0];
            payload[idx * 2 + 1] = stored.raw_bytes[1];
            idx += 1;
        }

        Ok(Self {
            request_service_id: request_surface.service_id,
            parameter_id: request_surface.parameter_id,
            meaning: CanObd2DtcFreezeFramePayloadMeaning::StoredDtcs { dtc_count, dtcs },
            response: Message::Obd2Response {
                service: 0x43,
                parameter_id: None,
                negative_response_code: None,
                payload_len: dtc_count * 2,
                payload,
            },
        })
    }

    pub fn assemble_freeze_frame(
        request: &Message,
        dtc: DiagCode,
        value_source: Option<&Message>,
    ) -> Result<Self, CanObd2DtcFreezeFrameResponseError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2DtcFreezeFrameResponseError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2DtcFreezeFrameResponseError::NotObd2Request);
        }
        if request_surface.service_id != 0x02 {
            return Err(CanObd2DtcFreezeFrameResponseError::UnsupportedService {
                service_id: request_surface.service_id,
            });
        }

        let pid_id = request_surface
            .parameter_id
            .ok_or(CanObd2DtcFreezeFrameResponseError::MissingParameterId)?;
        let supported = CAN_OBD2_SUPPORTED_PID_CATALOG
            .iter()
            .find(|profile| profile.pid_id == pid_id)
            .copied()
            .ok_or(CanObd2DtcFreezeFrameResponseError::UnsupportedPid { pid_id })?;
        let backing_message_class = match supported.backing {
            CanObd2PidBacking::Message(backing_message_class) => backing_message_class,
            CanObd2PidBacking::ReadinessMonitor => {
                return Err(CanObd2DtcFreezeFrameResponseError::UnsupportedPid { pid_id });
            }
        };
        let value_source = value_source.ok_or(
            CanObd2DtcFreezeFrameResponseError::MissingFreezeFrameValueSource {
                backing_message_class,
            },
        )?;
        let projection = supported
            .pid
            .value_projection()
            .ok_or(CanObd2DtcFreezeFrameResponseError::UnsupportedPid { pid_id })?;
        let payload = projection.project_from_message(value_source).ok_or(
            CanObd2DtcFreezeFrameResponseError::IncompatibleFreezeFrameValueSource {
                backing_message_class,
            },
        )?;
        let mut response_payload = [0u8; 6];
        response_payload[..payload.payload_len as usize]
            .copy_from_slice(&payload.bytes[..payload.payload_len as usize]);
        let stored_dtc = CanObd2StoredDtc::from_diag_code(dtc);

        Ok(Self {
            request_service_id: request_surface.service_id,
            parameter_id: Some(pid_id),
            meaning: CanObd2DtcFreezeFramePayloadMeaning::FreezeFrame(CanObd2FreezeFrameSnapshot {
                dtc: stored_dtc,
                pid: supported.pid,
                payload,
            }),
            response: Message::Obd2Response {
                service: 0x42,
                parameter_id: Some(pid_id),
                negative_response_code: None,
                payload_len: payload.payload_len,
                payload: response_payload,
            },
        })
    }
}

impl CanObd2MultiServiceDispatchSurface {
    fn from_mode01(dispatch: CanObd2RequestDispatchSurface) -> Self {
        let verdict = match dispatch.verdict {
            CanObd2RequestDispatchVerdict::CurrentDataPositive => {
                CanObd2MultiServiceDispatchVerdict::CurrentDataPositive
            }
            CanObd2RequestDispatchVerdict::ReadinessMonitorPositive => {
                CanObd2MultiServiceDispatchVerdict::ReadinessMonitorPositive
            }
            CanObd2RequestDispatchVerdict::SupportedPidDiscoveryPositive => {
                CanObd2MultiServiceDispatchVerdict::SupportedPidDiscoveryPositive
            }
            CanObd2RequestDispatchVerdict::NegativeResponse(code) => {
                CanObd2MultiServiceDispatchVerdict::NegativeResponse(code)
            }
        };
        Self {
            request_service_id: dispatch.request_service_id,
            parameter_id: Some(dispatch.parameter_id),
            verdict,
            response: CanObd2ResponseFrame::from_message(dispatch.response),
        }
    }

    fn from_dtc_or_freeze(dispatch: CanObd2DtcFreezeFrameResponseSurface) -> Self {
        let verdict = match dispatch.meaning {
            CanObd2DtcFreezeFramePayloadMeaning::StoredDtcs { .. } => {
                CanObd2MultiServiceDispatchVerdict::StoredDtcs
            }
            CanObd2DtcFreezeFramePayloadMeaning::FreezeFrame(_) => {
                CanObd2MultiServiceDispatchVerdict::FreezeFrame
            }
        };
        Self {
            request_service_id: dispatch.request_service_id,
            parameter_id: dispatch.parameter_id,
            verdict,
            response: CanObd2ResponseFrame::from_message(dispatch.response),
        }
    }

    fn from_dtc_clear(dispatch: CanObd2DtcClearSurface) -> Self {
        Self {
            request_service_id: dispatch.request_service_id,
            parameter_id: None,
            verdict: CanObd2MultiServiceDispatchVerdict::DtcClearPositive,
            response: CanObd2ResponseFrame::from_message(dispatch.response),
        }
    }

    fn from_vehicle_info(dispatch: CanObd2VehicleInfoResponseSurface) -> Self {
        Self {
            request_service_id: dispatch.request_service_id,
            parameter_id: Some(dispatch.info_type_id),
            verdict: CanObd2MultiServiceDispatchVerdict::VehicleInfoPositive,
            response: CanObd2ResponseFrame::from_message(dispatch.response),
        }
    }

    fn from_vehicle_info_negative(
        parameter_id: Option<u8>,
        negative: CanObd2NegativeResponseSurface,
    ) -> Self {
        Self {
            request_service_id: negative.request_service_id,
            parameter_id,
            verdict: CanObd2MultiServiceDispatchVerdict::NegativeResponse(negative.code),
            response: CanObd2ResponseFrame::from_message(negative.response),
        }
    }

    pub fn dispatch(
        request: &Message,
        inputs: CanObd2MultiServiceDispatchInputs<'_>,
    ) -> Result<Self, CanObd2MultiServiceDispatchError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2MultiServiceDispatchError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2MultiServiceDispatchError::NotObd2Request);
        }

        match request_surface.service_id {
            0x01 => CanObd2RequestDispatchSurface::dispatch_with_readiness(
                request,
                inputs.current_data_value_source,
                inputs.readiness_monitor,
            )
            .map(Self::from_mode01)
            .map_err(CanObd2MultiServiceDispatchError::Mode01),
            0x02 => {
                let freeze_frame_dtc = inputs
                    .freeze_frame_dtc
                    .ok_or(CanObd2MultiServiceDispatchError::MissingFreezeFrameDtc)?;
                CanObd2DtcFreezeFrameResponseSurface::assemble_freeze_frame(
                    request,
                    freeze_frame_dtc,
                    inputs.freeze_frame_value_source,
                )
                .map(Self::from_dtc_or_freeze)
                .map_err(CanObd2MultiServiceDispatchError::DtcFreezeFrame)
            }
            0x03 => CanObd2DtcFreezeFrameResponseSurface::assemble_stored_dtcs(
                request,
                inputs.stored_dtcs,
            )
            .map(Self::from_dtc_or_freeze)
            .map_err(CanObd2MultiServiceDispatchError::DtcFreezeFrame),
            0x04 => {
                let dtc_clear = inputs
                    .dtc_clear
                    .ok_or(CanObd2MultiServiceDispatchError::MissingDtcClearInputs)?;
                CanObd2DtcClearSurface::assemble(request, dtc_clear)
                    .map(Self::from_dtc_clear)
                    .map_err(CanObd2MultiServiceDispatchError::DtcClear)
            }
            0x09 => match CanObd2VehicleInfoResponseSurface::assemble(request, inputs.vehicle_info)
            {
                Ok(dispatch) => Ok(Self::from_vehicle_info(dispatch)),
                Err(error @ CanObd2VehicleInfoResponseError::UnsupportedInfoType { .. }) => {
                    let parameter_id = request_surface.parameter_id;
                    CanObd2NegativeResponseSurface::from_vehicle_info_error(request, error)
                        .map(|negative| Self::from_vehicle_info_negative(parameter_id, negative))
                        .map_err(|_| CanObd2MultiServiceDispatchError::VehicleInfo(error))
                }
                Err(error) => Err(CanObd2MultiServiceDispatchError::VehicleInfo(error)),
            },
            service_id => Err(CanObd2MultiServiceDispatchError::UnsupportedService { service_id }),
        }
    }

    pub fn dispatch_outcome(
        request: &Message,
        inputs: CanObd2MultiServiceDispatchInputs<'_>,
    ) -> Result<CanObd2MultiServiceDispatchOutcome, CanObd2MultiServiceDispatchError> {
        let request_surface = CanObd2ServiceSurface::from_message(request)
            .ok_or(CanObd2MultiServiceDispatchError::NotObd2Request)?;
        if request_surface.direction != CanObd2ServiceDirection::Request {
            return Err(CanObd2MultiServiceDispatchError::NotObd2Request);
        }

        let segmented_info_type = matches!(
            request_surface.parameter_id,
            Some(id)
                if id == CanObd2InfoType::Vin.info_type_id()
                    || id == CanObd2InfoType::CalibrationId.info_type_id()
                    || id == CanObd2InfoType::IdentityKeyLifecycle.info_type_id()
                    || id == CanObd2InfoType::FlashWriteFault.info_type_id()
        );
        if request_surface.service_id == 0x09 && segmented_info_type {
            return CanObd2SegmentedVehicleInfoResponseSurface::assemble(
                request,
                inputs.vehicle_info,
            )
            .map(CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo)
            .map_err(CanObd2MultiServiceDispatchError::SegmentedVehicleInfo);
        }

        Self::dispatch(request, inputs).map(CanObd2MultiServiceDispatchOutcome::Single)
    }
}

pub const CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS: u32 = 5;
pub const CAN_HEARTBEAT_TIMEOUT_MS: u32 = 100;
pub const CAN_MODULE_ROSTER_CAPACITY: usize = 16;
pub const CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY: usize = CAN_MODULE_ROSTER_CAPACITY * 2;
pub const CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY: usize = CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY * 2;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleHeartbeatStatus {
    Ok,
    Warning,
    Error,
    Unknown(u8),
}

impl CanModuleHeartbeatStatus {
    pub const fn from_raw(status: u8) -> Self {
        match status {
            0 => Self::Ok,
            1 => Self::Warning,
            2 => Self::Error,
            other => Self::Unknown(other),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleHeartbeatState {
    Starting,
    Alive,
    Degraded,
    Unavailable,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleHeartbeatContract {
    pub node_id: u8,
    pub uptime_seconds: u32,
    pub status: CanModuleHeartbeatStatus,
    pub error_count: u16,
    pub cpu_usage: u8,
    pub state: CanModuleHeartbeatState,
}

impl CanModuleHeartbeatContract {
    pub const fn from_fields(
        node_id: u8,
        uptime_seconds: u32,
        status: u8,
        error_count: u16,
        cpu_usage: u8,
        stale: bool,
    ) -> Self {
        let decoded_status = CanModuleHeartbeatStatus::from_raw(status);
        let state = if stale {
            CanModuleHeartbeatState::Unavailable
        } else if uptime_seconds <= CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS {
            CanModuleHeartbeatState::Starting
        } else if matches!(decoded_status, CanModuleHeartbeatStatus::Ok) && error_count == 0 {
            CanModuleHeartbeatState::Alive
        } else {
            CanModuleHeartbeatState::Degraded
        };

        Self {
            node_id,
            uptime_seconds,
            status: decoded_status,
            error_count,
            cpu_usage,
            state,
        }
    }

    pub fn from_message(message: &Message, stale: bool) -> Option<Self> {
        match *message {
            Message::Heartbeat {
                node_id,
                uptime_seconds,
                status,
                error_count,
                cpu_usage,
            } => Some(Self::from_fields(
                node_id,
                uptime_seconds,
                status,
                error_count,
                cpu_usage,
                stale,
            )),
            _ => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanHeartbeatObservationPolicy {
    pub timeout_ms: u32,
}

impl Default for CanHeartbeatObservationPolicy {
    fn default() -> Self {
        Self {
            timeout_ms: CAN_HEARTBEAT_TIMEOUT_MS,
        }
    }
}

impl CanHeartbeatObservationPolicy {
    pub const fn is_stale(self, age_ms: Option<u32>) -> bool {
        match age_ms {
            Some(age_ms) => age_ms > self.timeout_ms,
            None => true,
        }
    }

    pub fn observe_message(
        self,
        message: &Message,
        age_ms: Option<u32>,
    ) -> Option<CanModuleHeartbeatContract> {
        CanModuleHeartbeatContract::from_message(message, self.is_stale(age_ms))
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleAvailabilityTransition {
    NoChange(CanModuleHeartbeatState),
    Changed {
        from: CanModuleHeartbeatState,
        to: CanModuleHeartbeatState,
    },
}

impl CanModuleAvailabilityTransition {
    pub fn between(previous: CanModuleHeartbeatState, current: CanModuleHeartbeatState) -> Self {
        if previous == current {
            Self::NoChange(current)
        } else {
            Self::Changed {
                from: previous,
                to: current,
            }
        }
    }

    pub const fn current(self) -> CanModuleHeartbeatState {
        match self {
            Self::NoChange(state) => state,
            Self::Changed { to, .. } => to,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanRemoteModuleFallbackPolicy {
    RemoteDataTrusted,
    HoldLastKnownGood,
    RequireLocalFallback,
}

impl CanRemoteModuleFallbackPolicy {
    pub const fn from_state(state: CanModuleHeartbeatState) -> Self {
        match state {
            CanModuleHeartbeatState::Alive => Self::RemoteDataTrusted,
            CanModuleHeartbeatState::Starting | CanModuleHeartbeatState::Degraded => {
                Self::HoldLastKnownGood
            }
            CanModuleHeartbeatState::Unavailable => Self::RequireLocalFallback,
        }
    }

    pub const fn from_contract(contract: CanModuleHeartbeatContract) -> Self {
        Self::from_state(contract.state)
    }

    pub const fn from_transition(transition: CanModuleAvailabilityTransition) -> Self {
        Self::from_state(transition.current())
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEntry {
    pub heartbeat: CanModuleHeartbeatContract,
    pub availability_transition: CanModuleAvailabilityTransition,
    pub fallback_policy: CanRemoteModuleFallbackPolicy,
}

impl CanModuleRosterEntry {
    pub const fn from_contract_and_transition(
        heartbeat: CanModuleHeartbeatContract,
        availability_transition: CanModuleAvailabilityTransition,
    ) -> Self {
        Self {
            heartbeat,
            availability_transition,
            fallback_policy: CanRemoteModuleFallbackPolicy::from_transition(
                availability_transition,
            ),
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterSnapshotEntry {
    pub module: CanModuleRosterEntry,
    pub heartbeat_age_ms: u32,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterSnapshot {
    pub len: usize,
    pub entries: [Option<CanModuleRosterSnapshotEntry>; CAN_MODULE_ROSTER_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterSummary {
    pub total_count: usize,
    pub alive_count: usize,
    pub starting_count: usize,
    pub degraded_count: usize,
    pub unavailable_count: usize,
    pub trusted_count: usize,
    pub hold_last_known_good_count: usize,
    pub require_local_fallback_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanModuleRosterStartupReadinessState {
    #[default]
    NoRemoteModules,
    ReadyForTrust,
    WaitingForStartup,
    HoldingLastKnownGood,
    BlockedByUnavailable,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterStartupReadiness {
    pub state: CanModuleRosterStartupReadinessState,
    pub total_module_count: usize,
    pub ready_for_trust_module_count: usize,
    pub waiting_on_startup_module_count: usize,
    pub degraded_hold_last_known_good_module_count: usize,
    pub blocked_by_unavailable_module_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanModuleRosterShutdownCoordinationState {
    #[default]
    NoRemoteModules,
    NoShutdownNeeded,
    CoordinatedShutdownRecommended,
    LocalTakeoverRequired,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterShutdownCoordination {
    pub state: CanModuleRosterShutdownCoordinationState,
    pub total_module_count: usize,
    pub degraded_module_count: usize,
    pub unavailable_module_count: usize,
    pub require_local_fallback_module_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterChangeDigest {
    pub changed_module_count: usize,
    pub became_alive_count: usize,
    pub became_degraded_count: usize,
    pub became_unavailable_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterChangeSetEntry {
    pub node_id: u8,
    pub previous_state: Option<CanModuleHeartbeatState>,
    pub current_state: Option<CanModuleHeartbeatState>,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleRosterEventMeaning {
    FirstObservation(CanModuleHeartbeatState),
    RecoveryToAlive {
        from: CanModuleHeartbeatState,
    },
    TransitionToDegraded {
        from: CanModuleHeartbeatState,
    },
    TimedOutToUnavailable,
    Other {
        previous_state: Option<CanModuleHeartbeatState>,
        current_state: Option<CanModuleHeartbeatState>,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEventInterpretation {
    pub meaning: CanModuleRosterEventMeaning,
    pub fallback_impact: CanRemoteModuleFallbackPolicy,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterChangeSet {
    pub len: usize,
    pub entries: [Option<CanModuleRosterChangeSetEntry>; CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEventLog {
    pub len: usize,
    pub dropped_count: u32,
    pub entries: [Option<CanModuleRosterChangeSetEntry>; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterEventLogWatermark {
    pub retained_event_count: usize,
    pub dropped_event_count: u32,
    pub total_written_event_count: u32,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterEventLogDelta {
    pub previous_watermark: CanModuleRosterEventLogWatermark,
    pub current_watermark: CanModuleRosterEventLogWatermark,
    pub retained_new_event_count: usize,
    pub dropped_unread_event_count: u32,
    pub entries: [Option<CanModuleRosterChangeSetEntry>; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterEventDeltaSummary {
    pub retained_unread_event_count: usize,
    pub dropped_unread_event_count: u32,
    pub first_observation_count: usize,
    pub recovery_to_alive_count: usize,
    pub transition_to_degraded_count: usize,
    pub timed_out_to_unavailable_count: usize,
    pub trust_impact_count: usize,
    pub hold_last_known_good_impact_count: usize,
    pub require_local_fallback_impact_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterLatestUnreadEventEntry {
    pub node_id: u8,
    pub interpretation: CanModuleRosterEventInterpretation,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterLatestUnreadEventSet {
    pub len: usize,
    pub entries: [Option<CanModuleRosterLatestUnreadEventEntry>; CAN_MODULE_ROSTER_CAPACITY],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterLatestUnreadSummary {
    pub latest_unread_node_count: usize,
    pub first_observation_count: usize,
    pub recovery_to_alive_count: usize,
    pub transition_to_degraded_count: usize,
    pub timed_out_to_unavailable_count: usize,
    pub trust_impact_count: usize,
    pub hold_last_known_good_impact_count: usize,
    pub require_local_fallback_impact_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterLatestUnreadShutdownImpactSummary {
    pub latest_unread_node_count: usize,
    pub no_shutdown_needed_count: usize,
    pub coordinated_shutdown_recommended_count: usize,
    pub local_takeover_required_count: usize,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterLatestUnreadStartupReadinessSummary {
    pub latest_unread_node_count: usize,
    pub ready_for_trust_count: usize,
    pub waiting_on_startup_count: usize,
    pub holding_last_known_good_count: usize,
    pub blocked_by_unavailable_count: usize,
}

impl Default for CanModuleRosterSnapshot {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for CanModuleRosterChangeSet {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for CanModuleRosterEventLog {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for CanModuleRosterEventLogDelta {
    fn default() -> Self {
        Self {
            previous_watermark: CanModuleRosterEventLogWatermark::default(),
            current_watermark: CanModuleRosterEventLogWatermark::default(),
            retained_new_event_count: 0,
            dropped_unread_event_count: 0,
            entries: [None; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
        }
    }
}

impl Default for CanModuleRosterLatestUnreadEventSet {
    fn default() -> Self {
        Self::new()
    }
}

impl CanModuleRosterEventDeltaSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(_) => self.first_observation_count += 1,
            CanModuleRosterEventMeaning::RecoveryToAlive { .. } => {
                self.recovery_to_alive_count += 1
            }
            CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                self.transition_to_degraded_count += 1;
            }
            CanModuleRosterEventMeaning::TimedOutToUnavailable => {
                self.timed_out_to_unavailable_count += 1;
            }
            CanModuleRosterEventMeaning::Other { .. } => {}
        }
        match interpretation.fallback_impact {
            CanRemoteModuleFallbackPolicy::RemoteDataTrusted => self.trust_impact_count += 1,
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood => {
                self.hold_last_known_good_impact_count += 1;
            }
            CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
                self.require_local_fallback_impact_count += 1;
            }
        }
    }
}

impl CanModuleRosterLatestUnreadEventSet {
    pub const fn new() -> Self {
        Self {
            len: 0,
            entries: [None; CAN_MODULE_ROSTER_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterLatestUnreadEventEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterLatestUnreadEventEntry> {
        self.entries
            .iter()
            .copied()
            .flatten()
            .find(|entry| entry.node_id == node_id)
    }

    pub fn summary(&self) -> CanModuleRosterLatestUnreadSummary {
        let mut summary = CanModuleRosterLatestUnreadSummary::default();
        let mut index = 0usize;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation);
            }
            index += 1;
        }
        summary
    }

    pub fn shutdown_impact_summary(&self) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        let mut summary = CanModuleRosterLatestUnreadShutdownImpactSummary::default();
        let mut index = 0usize;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation);
            }
            index += 1;
        }
        summary
    }

    pub fn startup_readiness_summary(&self) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        let mut summary = CanModuleRosterLatestUnreadStartupReadinessSummary::default();
        let mut index = 0usize;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation);
            }
            index += 1;
        }
        summary
    }
}

impl CanModuleRosterLatestUnreadSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        self.latest_unread_node_count += 1;
        match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(_) => self.first_observation_count += 1,
            CanModuleRosterEventMeaning::RecoveryToAlive { .. } => {
                self.recovery_to_alive_count += 1
            }
            CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                self.transition_to_degraded_count += 1;
            }
            CanModuleRosterEventMeaning::TimedOutToUnavailable => {
                self.timed_out_to_unavailable_count += 1;
            }
            CanModuleRosterEventMeaning::Other { .. } => {}
        }
        match interpretation.fallback_impact {
            CanRemoteModuleFallbackPolicy::RemoteDataTrusted => self.trust_impact_count += 1,
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood => {
                self.hold_last_known_good_impact_count += 1;
            }
            CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
                self.require_local_fallback_impact_count += 1;
            }
        }
    }
}

impl CanModuleRosterLatestUnreadShutdownImpactSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        self.latest_unread_node_count += 1;
        match shutdown_coordination_from_interpretation(interpretation) {
            CanModuleRosterShutdownCoordinationState::NoRemoteModules => {}
            CanModuleRosterShutdownCoordinationState::NoShutdownNeeded => {
                self.no_shutdown_needed_count += 1;
            }
            CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended => {
                self.coordinated_shutdown_recommended_count += 1;
            }
            CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired => {
                self.local_takeover_required_count += 1;
            }
        }
    }
}

impl CanModuleRosterLatestUnreadStartupReadinessSummary {
    fn observe_interpretation(&mut self, interpretation: CanModuleRosterEventInterpretation) {
        self.latest_unread_node_count += 1;
        match startup_readiness_from_interpretation(interpretation) {
            CanModuleRosterStartupReadinessState::NoRemoteModules => {}
            CanModuleRosterStartupReadinessState::ReadyForTrust => {
                self.ready_for_trust_count += 1;
            }
            CanModuleRosterStartupReadinessState::WaitingForStartup => {
                self.waiting_on_startup_count += 1;
            }
            CanModuleRosterStartupReadinessState::HoldingLastKnownGood => {
                self.holding_last_known_good_count += 1;
            }
            CanModuleRosterStartupReadinessState::BlockedByUnavailable => {
                self.blocked_by_unavailable_count += 1;
            }
        }
    }
}

const fn shutdown_coordination_from_interpretation(
    interpretation: CanModuleRosterEventInterpretation,
) -> CanModuleRosterShutdownCoordinationState {
    match interpretation.fallback_impact {
        CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
            CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired
        }
        CanRemoteModuleFallbackPolicy::RemoteDataTrusted
        | CanRemoteModuleFallbackPolicy::HoldLastKnownGood => match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Degraded)
            | CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended
            }
            CanModuleRosterEventMeaning::FirstObservation(_)
            | CanModuleRosterEventMeaning::RecoveryToAlive { .. }
            | CanModuleRosterEventMeaning::TimedOutToUnavailable
            | CanModuleRosterEventMeaning::Other { .. } => {
                CanModuleRosterShutdownCoordinationState::NoShutdownNeeded
            }
        },
    }
}

const fn startup_readiness_from_interpretation(
    interpretation: CanModuleRosterEventInterpretation,
) -> CanModuleRosterStartupReadinessState {
    match interpretation.fallback_impact {
        CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
            CanModuleRosterStartupReadinessState::BlockedByUnavailable
        }
        CanRemoteModuleFallbackPolicy::RemoteDataTrusted => {
            CanModuleRosterStartupReadinessState::ReadyForTrust
        }
        CanRemoteModuleFallbackPolicy::HoldLastKnownGood => match interpretation.meaning {
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Starting) => {
                CanModuleRosterStartupReadinessState::WaitingForStartup
            }
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Degraded)
            | CanModuleRosterEventMeaning::TransitionToDegraded { .. } => {
                CanModuleRosterStartupReadinessState::HoldingLastKnownGood
            }
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Alive)
            | CanModuleRosterEventMeaning::RecoveryToAlive { .. } => {
                CanModuleRosterStartupReadinessState::ReadyForTrust
            }
            CanModuleRosterEventMeaning::FirstObservation(CanModuleHeartbeatState::Unavailable)
            | CanModuleRosterEventMeaning::TimedOutToUnavailable => {
                CanModuleRosterStartupReadinessState::BlockedByUnavailable
            }
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Starting),
                ..
            } => CanModuleRosterStartupReadinessState::WaitingForStartup,
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Degraded),
                ..
            } => CanModuleRosterStartupReadinessState::HoldingLastKnownGood,
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Alive),
                ..
            } => CanModuleRosterStartupReadinessState::ReadyForTrust,
            CanModuleRosterEventMeaning::Other {
                current_state: Some(CanModuleHeartbeatState::Unavailable) | None,
                ..
            } => CanModuleRosterStartupReadinessState::BlockedByUnavailable,
        },
    }
}

impl CanModuleRosterEventLogDelta {
    pub fn summary(&self) -> CanModuleRosterEventDeltaSummary {
        let mut summary = CanModuleRosterEventDeltaSummary {
            retained_unread_event_count: self.retained_new_event_count,
            dropped_unread_event_count: self.dropped_unread_event_count,
            ..CanModuleRosterEventDeltaSummary::default()
        };
        let mut index = 0usize;
        while index < self.retained_new_event_count {
            if let Some(entry) = self.entries[index] {
                summary.observe_interpretation(entry.interpretation());
            }
            index += 1;
        }
        summary
    }

    pub fn latest_per_node(&self) -> CanModuleRosterLatestUnreadEventSet {
        let mut latest = CanModuleRosterLatestUnreadEventSet::new();
        let mut index = 0usize;
        while index < self.retained_new_event_count {
            if let Some(entry) = self.entries[index] {
                let latest_entry = CanModuleRosterLatestUnreadEventEntry {
                    node_id: entry.node_id,
                    interpretation: entry.interpretation(),
                };
                if let Some(existing_index) = latest.entries[..latest.len].iter().position(
                    |slot| matches!(slot, Some(existing) if existing.node_id == entry.node_id),
                ) {
                    latest.entries[existing_index] = Some(latest_entry);
                } else if latest.len < CAN_MODULE_ROSTER_CAPACITY {
                    latest.entries[latest.len] = Some(latest_entry);
                    latest.len += 1;
                }
            }
            index += 1;
        }
        latest
    }

    pub fn latest_unread_summary(&self) -> CanModuleRosterLatestUnreadSummary {
        self.latest_per_node().summary()
    }

    pub fn latest_unread_shutdown_impact_summary(
        &self,
    ) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        self.latest_per_node().shutdown_impact_summary()
    }

    pub fn latest_unread_startup_readiness_summary(
        &self,
    ) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        self.latest_per_node().startup_readiness_summary()
    }
}

impl CanModuleRosterSnapshot {
    pub const fn new() -> Self {
        Self {
            len: 0,
            entries: [None; CAN_MODULE_ROSTER_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterSnapshotEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterSnapshotEntry> {
        self.entries
            .iter()
            .copied()
            .flatten()
            .find(|entry| entry.module.heartbeat.node_id == node_id)
    }

    pub fn summary(&self) -> CanModuleRosterSummary {
        let mut summary = CanModuleRosterSummary::default();
        let mut index = 0;

        while index < self.len {
            if let Some(entry) = self.entries[index] {
                summary.total_count += 1;
                match entry.module.heartbeat.state {
                    CanModuleHeartbeatState::Alive => summary.alive_count += 1,
                    CanModuleHeartbeatState::Starting => summary.starting_count += 1,
                    CanModuleHeartbeatState::Degraded => summary.degraded_count += 1,
                    CanModuleHeartbeatState::Unavailable => summary.unavailable_count += 1,
                }
                match entry.module.fallback_policy {
                    CanRemoteModuleFallbackPolicy::RemoteDataTrusted => {
                        summary.trusted_count += 1;
                    }
                    CanRemoteModuleFallbackPolicy::HoldLastKnownGood => {
                        summary.hold_last_known_good_count += 1;
                    }
                    CanRemoteModuleFallbackPolicy::RequireLocalFallback => {
                        summary.require_local_fallback_count += 1;
                    }
                }
            }
            index += 1;
        }

        summary
    }

    pub fn startup_readiness(&self) -> CanModuleRosterStartupReadiness {
        self.summary().startup_readiness()
    }

    pub fn shutdown_coordination(&self) -> CanModuleRosterShutdownCoordination {
        self.summary().shutdown_coordination()
    }

    pub fn change_digest_since(
        &self,
        previous: CanModuleRosterSnapshot,
    ) -> CanModuleRosterChangeDigest {
        self.change_set_since(previous).digest()
    }

    pub fn change_set_since(&self, previous: CanModuleRosterSnapshot) -> CanModuleRosterChangeSet {
        let mut change_set = CanModuleRosterChangeSet::new();
        let mut processed: [u8; CAN_MODULE_ROSTER_CAPACITY] = [0; CAN_MODULE_ROSTER_CAPACITY];
        let mut processed_len = 0usize;
        let mut index = 0usize;

        while index < self.len {
            if let Some(current) = self.entries[index] {
                let previous_state = previous
                    .get(current.module.heartbeat.node_id)
                    .map(|entry| entry.module.heartbeat.state);
                change_set.push_change(
                    current.module.heartbeat.node_id,
                    previous_state,
                    Some(current.module.heartbeat.state),
                );
                processed[processed_len] = current.module.heartbeat.node_id;
                processed_len += 1;
            }
            index += 1;
        }

        index = 0;
        while index < previous.len {
            if let Some(old) = previous.entries[index] {
                let already_processed =
                    processed[..processed_len].contains(&old.module.heartbeat.node_id);
                if !already_processed {
                    change_set.push_change(
                        old.module.heartbeat.node_id,
                        Some(old.module.heartbeat.state),
                        None,
                    );
                }
            }
            index += 1;
        }

        change_set
    }
}

impl CanModuleRosterSummary {
    pub const fn startup_readiness(self) -> CanModuleRosterStartupReadiness {
        let ready_for_trust_module_count = self.trusted_count;
        let waiting_on_startup_module_count = self.starting_count;
        let degraded_hold_last_known_good_module_count = self.degraded_count;
        let blocked_by_unavailable_module_count = self.unavailable_count;
        let state = if self.total_count == 0 {
            CanModuleRosterStartupReadinessState::NoRemoteModules
        } else if blocked_by_unavailable_module_count > 0 {
            CanModuleRosterStartupReadinessState::BlockedByUnavailable
        } else if degraded_hold_last_known_good_module_count > 0 {
            CanModuleRosterStartupReadinessState::HoldingLastKnownGood
        } else if waiting_on_startup_module_count > 0 {
            CanModuleRosterStartupReadinessState::WaitingForStartup
        } else {
            CanModuleRosterStartupReadinessState::ReadyForTrust
        };

        CanModuleRosterStartupReadiness {
            state,
            total_module_count: self.total_count,
            ready_for_trust_module_count,
            waiting_on_startup_module_count,
            degraded_hold_last_known_good_module_count,
            blocked_by_unavailable_module_count,
        }
    }

    pub const fn shutdown_coordination(self) -> CanModuleRosterShutdownCoordination {
        let degraded_module_count = self.degraded_count;
        let unavailable_module_count = self.unavailable_count;
        let require_local_fallback_module_count = self.require_local_fallback_count;
        let state = if self.total_count == 0 {
            CanModuleRosterShutdownCoordinationState::NoRemoteModules
        } else if unavailable_module_count > 0 || require_local_fallback_module_count > 0 {
            CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired
        } else if degraded_module_count > 0 {
            CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended
        } else {
            CanModuleRosterShutdownCoordinationState::NoShutdownNeeded
        };

        CanModuleRosterShutdownCoordination {
            state,
            total_module_count: self.total_count,
            degraded_module_count,
            unavailable_module_count,
            require_local_fallback_module_count,
        }
    }
}

impl CanModuleRosterChangeDigest {
    fn observe_transition(
        &mut self,
        previous: Option<CanModuleHeartbeatState>,
        current: Option<CanModuleHeartbeatState>,
    ) {
        if previous == current {
            return;
        }

        self.changed_module_count += 1;
        match current {
            Some(CanModuleHeartbeatState::Alive) => self.became_alive_count += 1,
            Some(CanModuleHeartbeatState::Degraded) => self.became_degraded_count += 1,
            Some(CanModuleHeartbeatState::Unavailable) => self.became_unavailable_count += 1,
            Some(CanModuleHeartbeatState::Starting) | None => {}
        }
    }
}

impl CanModuleRosterChangeSetEntry {
    pub const fn meaning(self) -> CanModuleRosterEventMeaning {
        match (self.previous_state, self.current_state) {
            (None, Some(state)) => CanModuleRosterEventMeaning::FirstObservation(state),
            (Some(from), Some(CanModuleHeartbeatState::Alive))
                if matches!(
                    from,
                    CanModuleHeartbeatState::Starting
                        | CanModuleHeartbeatState::Degraded
                        | CanModuleHeartbeatState::Unavailable
                ) =>
            {
                CanModuleRosterEventMeaning::RecoveryToAlive { from }
            }
            (Some(from), Some(CanModuleHeartbeatState::Degraded)) => {
                CanModuleRosterEventMeaning::TransitionToDegraded { from }
            }
            (Some(_), Some(CanModuleHeartbeatState::Unavailable)) => {
                CanModuleRosterEventMeaning::TimedOutToUnavailable
            }
            (previous_state, current_state) => CanModuleRosterEventMeaning::Other {
                previous_state,
                current_state,
            },
        }
    }

    pub const fn fallback_impact(self) -> CanRemoteModuleFallbackPolicy {
        match self.current_state {
            Some(CanModuleHeartbeatState::Alive) => {
                CanRemoteModuleFallbackPolicy::RemoteDataTrusted
            }
            Some(CanModuleHeartbeatState::Starting) | Some(CanModuleHeartbeatState::Degraded) => {
                CanRemoteModuleFallbackPolicy::HoldLastKnownGood
            }
            Some(CanModuleHeartbeatState::Unavailable) | None => {
                CanRemoteModuleFallbackPolicy::RequireLocalFallback
            }
        }
    }

    pub const fn interpretation(self) -> CanModuleRosterEventInterpretation {
        CanModuleRosterEventInterpretation {
            meaning: self.meaning(),
            fallback_impact: self.fallback_impact(),
        }
    }
}

impl CanModuleRosterChangeSet {
    pub const fn new() -> Self {
        Self {
            len: 0,
            entries: [None; CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterChangeSetEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterChangeSetEntry> {
        self.entries
            .iter()
            .copied()
            .flatten()
            .find(|entry| entry.node_id == node_id)
    }

    pub fn digest(&self) -> CanModuleRosterChangeDigest {
        let mut digest = CanModuleRosterChangeDigest::default();
        let mut index = 0;
        while index < self.len {
            if let Some(entry) = self.entries[index] {
                digest.observe_transition(entry.previous_state, entry.current_state);
            }
            index += 1;
        }
        digest
    }

    fn push_change(
        &mut self,
        node_id: u8,
        previous_state: Option<CanModuleHeartbeatState>,
        current_state: Option<CanModuleHeartbeatState>,
    ) {
        if previous_state == current_state {
            return;
        }
        if self.len >= CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY {
            return;
        }
        self.entries[self.len] = Some(CanModuleRosterChangeSetEntry {
            node_id,
            previous_state,
            current_state,
        });
        self.len += 1;
    }
}

impl CanModuleRosterEventLog {
    pub const fn new() -> Self {
        Self {
            len: 0,
            dropped_count: 0,
            entries: [None; CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY],
        }
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn entry(&self, index: usize) -> Option<CanModuleRosterChangeSetEntry> {
        self.entries.get(index).copied().flatten()
    }

    pub fn entry_meaning(&self, index: usize) -> Option<CanModuleRosterEventMeaning> {
        self.entry(index)
            .map(CanModuleRosterChangeSetEntry::meaning)
    }

    pub fn entry_interpretation(&self, index: usize) -> Option<CanModuleRosterEventInterpretation> {
        self.entry(index)
            .map(CanModuleRosterChangeSetEntry::interpretation)
    }

    pub fn watermark(&self) -> CanModuleRosterEventLogWatermark {
        CanModuleRosterEventLogWatermark {
            retained_event_count: self.len,
            dropped_event_count: self.dropped_count,
            total_written_event_count: self.dropped_count.saturating_add(self.len as u32),
        }
    }

    pub fn delta_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventLogDelta {
        let current_watermark = self.watermark();
        let new_event_count = current_watermark
            .total_written_event_count
            .saturating_sub(previous_watermark.total_written_event_count);
        let earliest_retained_total = current_watermark
            .total_written_event_count
            .saturating_sub(self.len as u32);
        let retained_start_total = previous_watermark
            .total_written_event_count
            .max(earliest_retained_total);
        let retained_new_event_count = current_watermark
            .total_written_event_count
            .saturating_sub(retained_start_total)
            .min(self.len as u32) as usize;
        let dropped_unread_event_count =
            new_event_count.saturating_sub(retained_new_event_count as u32);
        let mut delta = CanModuleRosterEventLogDelta {
            previous_watermark,
            current_watermark,
            retained_new_event_count,
            dropped_unread_event_count,
            ..CanModuleRosterEventLogDelta::default()
        };
        let mut source_index = self.len.saturating_sub(retained_new_event_count);
        let mut target_index = 0usize;
        while source_index < self.len {
            delta.entries[target_index] = self.entries[source_index];
            source_index += 1;
            target_index += 1;
        }
        delta
    }

    pub fn delta_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventDeltaSummary {
        self.delta_since(previous_watermark).summary()
    }

    pub fn delta_latest_per_node_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadEventSet {
        self.delta_since(previous_watermark).latest_per_node()
    }

    pub fn delta_latest_unread_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadSummary {
        self.delta_since(previous_watermark).latest_unread_summary()
    }

    pub fn delta_latest_unread_shutdown_impact_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        self.delta_since(previous_watermark)
            .latest_unread_shutdown_impact_summary()
    }

    pub fn delta_latest_unread_startup_readiness_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        self.delta_since(previous_watermark)
            .latest_unread_startup_readiness_summary()
    }

    fn push_change(
        &mut self,
        node_id: u8,
        previous_state: Option<CanModuleHeartbeatState>,
        current_state: Option<CanModuleHeartbeatState>,
    ) {
        if previous_state == current_state {
            return;
        }

        let entry = CanModuleRosterChangeSetEntry {
            node_id,
            previous_state,
            current_state,
        };

        if self.len < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY {
            self.entries[self.len] = Some(entry);
            self.len += 1;
            return;
        }

        let mut index = 1usize;
        while index < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY {
            self.entries[index - 1] = self.entries[index];
            index += 1;
        }
        self.entries[CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY - 1] = Some(entry);
        self.dropped_count = self.dropped_count.saturating_add(1);
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum CanModuleRosterUpdate {
    Inserted(CanModuleRosterEntry),
    Updated(CanModuleRosterEntry),
    RejectedCapacity { node_id: u8 },
}

impl CanModuleRosterUpdate {
    pub const fn entry(self) -> Option<CanModuleRosterEntry> {
        match self {
            Self::Inserted(entry) | Self::Updated(entry) => Some(entry),
            Self::RejectedCapacity { .. } => None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct CanModuleRosterRegistry {
    slots: [Option<CanModuleRosterEntry>; CAN_MODULE_ROSTER_CAPACITY],
    heartbeat_age_ms: [u32; CAN_MODULE_ROSTER_CAPACITY],
    event_log: CanModuleRosterEventLog,
    len: usize,
}

impl Default for CanModuleRosterRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl CanModuleRosterRegistry {
    pub const fn new() -> Self {
        Self {
            slots: [None; CAN_MODULE_ROSTER_CAPACITY],
            heartbeat_age_ms: [0; CAN_MODULE_ROSTER_CAPACITY],
            event_log: CanModuleRosterEventLog::new(),
            len: 0,
        }
    }

    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn capacity(&self) -> usize {
        CAN_MODULE_ROSTER_CAPACITY
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn event_log(&self) -> CanModuleRosterEventLog {
        self.event_log
    }

    pub fn event_log_watermark(&self) -> CanModuleRosterEventLogWatermark {
        self.event_log.watermark()
    }

    pub fn event_log_delta_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventLogDelta {
        self.event_log.delta_since(previous_watermark)
    }

    pub fn event_log_delta_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterEventDeltaSummary {
        self.event_log.delta_summary_since(previous_watermark)
    }

    pub fn event_log_delta_latest_per_node_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadEventSet {
        self.event_log
            .delta_latest_per_node_since(previous_watermark)
    }

    pub fn event_log_delta_latest_unread_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadSummary {
        self.event_log
            .delta_latest_unread_summary_since(previous_watermark)
    }

    pub fn event_log_delta_latest_unread_shutdown_impact_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadShutdownImpactSummary {
        self.event_log
            .delta_latest_unread_shutdown_impact_summary_since(previous_watermark)
    }

    pub fn event_log_delta_latest_unread_startup_readiness_summary_since(
        &self,
        previous_watermark: CanModuleRosterEventLogWatermark,
    ) -> CanModuleRosterLatestUnreadStartupReadinessSummary {
        self.event_log
            .delta_latest_unread_startup_readiness_summary_since(previous_watermark)
    }

    pub fn snapshot(&self) -> CanModuleRosterSnapshot {
        let mut snapshot = CanModuleRosterSnapshot::new();
        let mut source_index = 0;
        let mut target_index = 0;

        while source_index < CAN_MODULE_ROSTER_CAPACITY {
            if let Some(module) = self.slots[source_index] {
                snapshot.entries[target_index] = Some(CanModuleRosterSnapshotEntry {
                    module,
                    heartbeat_age_ms: self.heartbeat_age_ms[source_index],
                });
                target_index += 1;
            }
            source_index += 1;
        }
        snapshot.len = target_index;
        snapshot
    }

    pub fn summary(&self) -> CanModuleRosterSummary {
        self.snapshot().summary()
    }

    pub fn startup_readiness(&self) -> CanModuleRosterStartupReadiness {
        self.summary().startup_readiness()
    }

    pub fn shutdown_coordination(&self) -> CanModuleRosterShutdownCoordination {
        self.summary().shutdown_coordination()
    }

    pub fn change_digest_since(
        &self,
        previous: CanModuleRosterSnapshot,
    ) -> CanModuleRosterChangeDigest {
        self.change_set_since(previous).digest()
    }

    pub fn change_set_since(&self, previous: CanModuleRosterSnapshot) -> CanModuleRosterChangeSet {
        self.snapshot().change_set_since(previous)
    }

    pub fn get(&self, node_id: u8) -> Option<CanModuleRosterEntry> {
        self.find_slot_index(node_id)
            .and_then(|slot_index| self.slots[slot_index])
    }

    pub fn observe_contract(
        &mut self,
        heartbeat: CanModuleHeartbeatContract,
    ) -> CanModuleRosterUpdate {
        self.observe_contract_with_age_ms(heartbeat, 0)
    }

    pub fn observe_contract_with_age_ms(
        &mut self,
        heartbeat: CanModuleHeartbeatContract,
        age_ms: u32,
    ) -> CanModuleRosterUpdate {
        if let Some(slot_index) = self.find_slot_index(heartbeat.node_id) {
            let previous = self.slots[slot_index].expect("known slot");
            self.event_log.push_change(
                heartbeat.node_id,
                Some(previous.heartbeat.state),
                Some(heartbeat.state),
            );
            let entry = CanModuleRosterEntry::from_contract_and_transition(
                heartbeat,
                CanModuleAvailabilityTransition::between(previous.heartbeat.state, heartbeat.state),
            );
            self.slots[slot_index] = Some(entry);
            self.heartbeat_age_ms[slot_index] = age_ms;
            CanModuleRosterUpdate::Updated(entry)
        } else if let Some(slot_index) = self.first_empty_slot_index() {
            self.event_log
                .push_change(heartbeat.node_id, None, Some(heartbeat.state));
            let entry = CanModuleRosterEntry::from_contract_and_transition(
                heartbeat,
                CanModuleAvailabilityTransition::NoChange(heartbeat.state),
            );
            self.slots[slot_index] = Some(entry);
            self.heartbeat_age_ms[slot_index] = age_ms;
            self.len += 1;
            CanModuleRosterUpdate::Inserted(entry)
        } else {
            CanModuleRosterUpdate::RejectedCapacity {
                node_id: heartbeat.node_id,
            }
        }
    }

    pub fn observe_message(
        &mut self,
        policy: CanHeartbeatObservationPolicy,
        message: &Message,
        age_ms: Option<u32>,
    ) -> Option<CanModuleRosterUpdate> {
        policy.observe_message(message, age_ms).map(|heartbeat| {
            self.observe_contract_with_age_ms(heartbeat, observed_age_ms(policy, age_ms))
        })
    }

    pub fn sweep_stale(
        &mut self,
        policy: CanHeartbeatObservationPolicy,
        elapsed_ms: u32,
    ) -> CanModuleRosterSweepSummary {
        let mut summary = CanModuleRosterSweepSummary::default();

        let mut slot_index = 0;
        while slot_index < CAN_MODULE_ROSTER_CAPACITY {
            if let Some(entry) = self.slots[slot_index] {
                let next_age_ms = self.heartbeat_age_ms[slot_index].saturating_add(elapsed_ms);
                self.heartbeat_age_ms[slot_index] = next_age_ms;

                if policy.is_stale(Some(next_age_ms)) {
                    summary.stale_count += 1;

                    if entry.heartbeat.state != CanModuleHeartbeatState::Unavailable {
                        self.event_log.push_change(
                            entry.heartbeat.node_id,
                            Some(entry.heartbeat.state),
                            Some(CanModuleHeartbeatState::Unavailable),
                        );
                        let heartbeat = CanModuleHeartbeatContract {
                            state: CanModuleHeartbeatState::Unavailable,
                            ..entry.heartbeat
                        };
                        let updated = CanModuleRosterEntry::from_contract_and_transition(
                            heartbeat,
                            CanModuleAvailabilityTransition::between(
                                entry.heartbeat.state,
                                CanModuleHeartbeatState::Unavailable,
                            ),
                        );
                        self.slots[slot_index] = Some(updated);
                        summary.transitioned_count += 1;
                    } else if !matches!(
                        entry.availability_transition,
                        CanModuleAvailabilityTransition::NoChange(
                            CanModuleHeartbeatState::Unavailable
                        )
                    ) {
                        self.slots[slot_index] =
                            Some(CanModuleRosterEntry::from_contract_and_transition(
                                entry.heartbeat,
                                CanModuleAvailabilityTransition::NoChange(
                                    CanModuleHeartbeatState::Unavailable,
                                ),
                            ));
                    }
                }
            }
            slot_index += 1;
        }

        summary
    }

    fn find_slot_index(&self, node_id: u8) -> Option<usize> {
        self.slots
            .iter()
            .position(|slot| matches!(slot, Some(entry) if entry.heartbeat.node_id == node_id))
    }

    fn first_empty_slot_index(&self) -> Option<usize> {
        self.slots.iter().position(Option::is_none)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanModuleRosterSweepSummary {
    pub stale_count: usize,
    pub transitioned_count: usize,
}

const fn observed_age_ms(policy: CanHeartbeatObservationPolicy, age_ms: Option<u32>) -> u32 {
    match age_ms {
        Some(age_ms) => age_ms,
        None => policy.timeout_ms.saturating_add(1),
    }
}

/// Transport-owned sequence outcome for fragmented CAN RX handling.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub enum CanSequenceEvent {
    #[default]
    None,
    InOrderComplete,
    DuplicateStart,
    UnexpectedContinuation,
    MissingFragments,
}

/// Product-owned bounded sequence-tracking surface for fragmented CAN RX.
#[derive(Debug, Copy, Clone, PartialEq, Eq, Default)]
pub struct CanSequenceTracking {
    pub completed_streams: u32,
    pub duplicate_start_count: u32,
    pub unexpected_continuation_count: u32,
    pub missing_fragment_count: u32,
    pub last_event: CanSequenceEvent,
}

#[derive(Copy, Clone)]
struct ReassemblySlot {
    tid: Option<u8>,
    id: u32,
    expected: usize,
    len: usize,
    crc16: u16,
    buf: [u8; REASM_BUF],
}

impl ReassemblySlot {
    const fn empty() -> Self {
        Self {
            tid: None,
            id: 0,
            expected: 0,
            len: 0,
            crc16: 0,
            buf: [0; REASM_BUF],
        }
    }

    const fn matches(&self, tid: u8, id: u32) -> bool {
        matches!(self.tid, Some(active_tid) if active_tid == tid) && self.id == id
    }
}

/// Generic CAN device trait.
pub trait CanDevice {
    type Error;

    /// Returns true when the device is ready to send.
    fn tx_ready(&self) -> bool;

    /// Send one CAN frame.
    fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Try receive one CAN frame into the provided buffer.
    fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)>;

    /// Report current bus health and recovery state.
    fn bus_health(&self) -> CanBusHealth {
        CanBusHealth::healthy()
    }
}

/// CAN transport wrapper implementing the generic `Transport` trait.
pub struct CanTransport<D: CanDevice> {
    dev: D,
    stats: TransportStats,
    reassembly: [ReassemblySlot; RX_REASSEMBLY_SLOTS],
    sequence_tracking: CanSequenceTracking,
    tx_tid: u8,
    last_rx_error: Option<TransportError>,
}

impl<D: CanDevice> CanTransport<D> {
    pub fn new(dev: D) -> Self {
        Self {
            dev,
            stats: TransportStats::default(),
            reassembly: [ReassemblySlot::empty(); RX_REASSEMBLY_SLOTS],
            sequence_tracking: CanSequenceTracking::default(),
            tx_tid: 0,
            last_rx_error: None,
        }
    }

    pub fn bus_health(&self) -> CanBusHealth {
        self.dev.bus_health()
    }

    pub fn route_for(msg: &Message) -> CanMessageRoute {
        let class = match msg {
            Message::Error { .. } => CanMessageClass::Fault,
            Message::CmdReset { .. } => CanMessageClass::ResetCommand,
            Message::CmdEngineControl { .. } => CanMessageClass::EngineControlCommand,
            Message::TriggerTiming { .. } => CanMessageClass::TriggerTiming,
            Message::SensorData { .. } => CanMessageClass::SensorData,
            Message::GpsData { .. } => CanMessageClass::GpsData,
            Message::ExternalEgtData { .. } => CanMessageClass::ExternalEgtData,
            Message::IpwTable { .. } => CanMessageClass::FuelTable,
            Message::IgnitionTable { .. } => CanMessageClass::IgnitionTable,
            Message::EngineConfig { .. } => CanMessageClass::EngineConfig,
            Message::InjectorConfig { .. } => CanMessageClass::InjectorConfig,
            Message::CmdCalibrate { .. } => CanMessageClass::CalibrationCommand,
            Message::Obd2IdentityProvisioningCommand { .. }
            | Message::Obd2IdentityProvisioningArm { .. }
            | Message::Obd2IdentityProvisioningArmAudit { .. }
            | Message::Obd2IdentityProvisioningAudit { .. }
            | Message::Obd2IdentityProvisioningKeyCommand { .. }
            | Message::Obd2IdentityProvisioningKeyAudit { .. } => {
                CanMessageClass::CalibrationCommand
            }
            Message::Obd2Request { .. } => CanMessageClass::Obd2Request,
            Message::Obd2Response { .. } | Message::Obd2SegmentedResponse { .. } => {
                CanMessageClass::Obd2Response
            }
            Message::Heartbeat { .. } => CanMessageClass::Heartbeat,
        };
        CanMessageRoute::new(class)
    }

    /// Last receive-side error observed by the lossy `try_receive` API.
    pub fn last_receive_error(&self) -> Option<TransportError> {
        self.last_rx_error
    }

    pub fn sequence_tracking(&self) -> CanSequenceTracking {
        self.sequence_tracking
    }

    fn clear_reassembly_slot(&mut self, slot: usize) {
        self.reassembly[slot] = ReassemblySlot::empty();
    }

    fn rx_error(&mut self, error: TransportError) -> Option<Message> {
        self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
        self.last_rx_error = Some(error);
        None
    }

    fn rx_error_for_slot(&mut self, slot: usize, error: TransportError) -> Option<Message> {
        self.clear_reassembly_slot(slot);
        self.rx_error(error)
    }

    fn find_reassembly_slot(&self, tid: u8, id: u32) -> Option<usize> {
        self.reassembly
            .iter()
            .position(|slot| slot.matches(tid, id))
    }

    fn find_free_reassembly_slot(&self) -> Option<usize> {
        self.reassembly.iter().position(|slot| slot.tid.is_none())
    }

    fn note_sequence_event(&mut self, event: CanSequenceEvent) {
        self.sequence_tracking.last_event = event;
        match event {
            CanSequenceEvent::None => {}
            CanSequenceEvent::InOrderComplete => {
                self.sequence_tracking.completed_streams =
                    self.sequence_tracking.completed_streams.saturating_add(1);
            }
            CanSequenceEvent::DuplicateStart => {
                self.sequence_tracking.duplicate_start_count = self
                    .sequence_tracking
                    .duplicate_start_count
                    .saturating_add(1);
            }
            CanSequenceEvent::UnexpectedContinuation => {
                self.sequence_tracking.unexpected_continuation_count = self
                    .sequence_tracking
                    .unexpected_continuation_count
                    .saturating_add(1);
            }
            CanSequenceEvent::MissingFragments => {
                self.sequence_tracking.missing_fragment_count = self
                    .sequence_tracking
                    .missing_fragment_count
                    .saturating_add(1);
            }
        }
    }

    fn send_frame(&mut self, id: u32, data: &[u8]) -> Result<(), TransportError> {
        if self.dev.bus_health().is_send_blocked() || !self.dev.tx_ready() {
            self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
            return Err(TransportError::NotReady);
        }

        self.dev.send(id, data).map_err(|_| {
            self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
            TransportError::HardwareError
        })
    }
}

impl<D: CanDevice> Transport for CanTransport<D> {
    fn send(&mut self, message: &Message) -> Result<(), TransportError> {
        let mut big = [0u8; REASM_BUF];
        let encoded = match to_slice(message, &mut big[..]) {
            Ok(enc) => enc,
            Err(_) => {
                self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                return Err(TransportError::SerializationFailed);
            }
        };
        let route = Self::route_for(message);

        if encoded.len() <= MAX_CAN_DATA.saturating_sub(1) {
            let mut frame = [0u8; MAX_CAN_DATA];
            let tid = self.tx_tid & 0x3f;
            self.tx_tid = self.tx_tid.wrapping_add(1);
            frame[0] = tid;
            frame[1..1 + encoded.len()].copy_from_slice(encoded);
            let data = &frame[..1 + encoded.len()];
            self.send_frame(route.arbitration_id, data)?;
            self.stats.tx_count = self.stats.tx_count.saturating_add(1);
            Ok(())
        } else {
            let total_len = encoded.len();
            if total_len > REASM_BUF {
                self.stats.tx_errors = self.stats.tx_errors.saturating_add(1);
                return Err(TransportError::MessageTooLarge);
            }

            let crc = crc16_ccitt(encoded);
            let tid = self.tx_tid & 0x3f;
            self.tx_tid = self.tx_tid.wrapping_add(1);

            let mut frame = [0u8; MAX_CAN_DATA];
            frame[0] = (0b01 << 6) | tid;
            frame[1] = (total_len & 0xff) as u8;
            frame[2] = ((total_len >> 8) & 0xff) as u8;
            frame[3] = (crc & 0xff) as u8;
            frame[4] = (crc >> 8) as u8;
            let mut sent = 0usize;
            let chunk0 = MAX_CAN_DATA.saturating_sub(5);
            let c0 = core::cmp::min(chunk0, total_len);
            frame[5..5 + c0].copy_from_slice(&encoded[..c0]);
            self.send_frame(route.arbitration_id, &frame[..5 + c0])?;
            sent += c0;

            while sent < total_len {
                let mut fr = [0u8; MAX_CAN_DATA];
                let remaining = total_len - sent;
                let is_end = remaining <= (MAX_CAN_DATA - 1);
                fr[0] = ((if is_end { 0b11 } else { 0b10 }) << 6) | tid;
                let take = core::cmp::min(remaining, MAX_CAN_DATA - 1);
                fr[1..1 + take].copy_from_slice(&encoded[sent..sent + take]);
                self.send_frame(route.arbitration_id, &fr[..1 + take])?;
                sent += take;
            }
            self.stats.tx_count = self.stats.tx_count.saturating_add(1);
            Ok(())
        }
    }

    fn try_receive(&mut self) -> Option<Message> {
        let mut f = [0u8; MAX_CAN_DATA];
        let (id, len) = self.dev.try_receive(&mut f[..])?;
        if len == 0 {
            return self.rx_error(TransportError::DeserializationFailed);
        }

        let flags = f[0];
        let kind = flags >> 6;
        let tid = flags & 0x3f;

        match kind {
            0 => {
                if let Ok(msg) = from_bytes::<Message>(&f[1..len]) {
                    self.stats.rx_count = self.stats.rx_count.saturating_add(1);
                    self.last_rx_error = None;
                    Some(msg)
                } else {
                    self.rx_error(TransportError::DeserializationFailed)
                }
            }
            1 => {
                if len < 5 {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                let total = (f[1] as usize) | ((f[2] as usize) << 8);
                let crc = (f[3] as u16) | ((f[4] as u16) << 8);
                let copy = len.saturating_sub(5);
                if total == 0 || total > REASM_BUF || copy > total {
                    return self.rx_error(TransportError::DeserializationFailed);
                }
                if let Some(existing) = self.find_reassembly_slot(tid, id) {
                    self.note_sequence_event(CanSequenceEvent::DuplicateStart);
                    return self.rx_error_for_slot(existing, TransportError::DeserializationFailed);
                }
                let Some(slot_index) = self.find_free_reassembly_slot() else {
                    return self.rx_error(TransportError::DeserializationFailed);
                };
                let slot = &mut self.reassembly[slot_index];
                slot.tid = Some(tid);
                slot.id = id;
                slot.expected = total;
                slot.len = 0;
                slot.crc16 = crc;
                if copy > 0 {
                    slot.buf[..copy].copy_from_slice(&f[5..5 + copy]);
                    slot.len = copy;
                }
                None
            }
            2 | 3 => {
                let Some(slot_index) = self.find_reassembly_slot(tid, id) else {
                    self.note_sequence_event(CanSequenceEvent::UnexpectedContinuation);
                    return self.rx_error(TransportError::DeserializationFailed);
                };
                let remaining = self.reassembly[slot_index]
                    .expected
                    .saturating_sub(self.reassembly[slot_index].len);
                if remaining == 0 {
                    self.note_sequence_event(CanSequenceEvent::UnexpectedContinuation);
                    return self
                        .rx_error_for_slot(slot_index, TransportError::DeserializationFailed);
                }
                let data = &f[1..len];
                if data.is_empty() || data.len() > remaining {
                    self.note_sequence_event(CanSequenceEvent::UnexpectedContinuation);
                    return self
                        .rx_error_for_slot(slot_index, TransportError::DeserializationFailed);
                }
                if kind == 2 && data.len() == remaining {
                    self.note_sequence_event(CanSequenceEvent::MissingFragments);
                    return self
                        .rx_error_for_slot(slot_index, TransportError::DeserializationFailed);
                }
                if kind == 3 && data.len() != remaining {
                    self.note_sequence_event(CanSequenceEvent::MissingFragments);
                    return self
                        .rx_error_for_slot(slot_index, TransportError::DeserializationFailed);
                }
                {
                    let slot = &mut self.reassembly[slot_index];
                    slot.buf[slot.len..slot.len + data.len()].copy_from_slice(data);
                    slot.len += data.len();
                }
                if kind == 3 {
                    let is_valid = {
                        let slot = &self.reassembly[slot_index];
                        slot.len == slot.expected
                            && crc16_ccitt(&slot.buf[..slot.len]) == slot.crc16
                    };
                    if is_valid {
                        let out = {
                            let slot = &self.reassembly[slot_index];
                            from_bytes::<Message>(&slot.buf[..slot.len]).ok()
                        };
                        self.clear_reassembly_slot(slot_index);
                        if out.is_some() {
                            self.stats.rx_count = self.stats.rx_count.saturating_add(1);
                            self.note_sequence_event(CanSequenceEvent::InOrderComplete);
                            self.last_rx_error = None;
                        } else {
                            self.stats.rx_errors = self.stats.rx_errors.saturating_add(1);
                            self.note_sequence_event(CanSequenceEvent::MissingFragments);
                            self.last_rx_error = Some(TransportError::DeserializationFailed);
                        }
                        out
                    } else {
                        self.note_sequence_event(CanSequenceEvent::MissingFragments);
                        self.rx_error_for_slot(slot_index, TransportError::DeserializationFailed)
                    }
                } else {
                    None
                }
            }
            _ => self.rx_error(TransportError::DeserializationFailed),
        }
    }

    fn poll(&mut self) {}

    fn flush(&mut self) -> Result<(), TransportError> {
        Ok(())
    }

    fn stats(&self) -> TransportStats {
        self.stats
    }

    fn is_ready(&self) -> bool {
        self.dev.tx_ready() && !self.dev.bus_health().is_send_blocked()
    }
}

fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xffff;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message::transport_parity_sample_messages;
    use std::cell::RefCell;
    use std::collections::VecDeque;
    use std::rc::Rc;

    struct SharedBus {
        frames: VecDeque<(u32, [u8; 64], usize)>,
    }

    #[derive(Clone)]
    struct MockCan {
        bus: Rc<RefCell<SharedBus>>,
        ready: bool,
        tx_capacity: Option<usize>,
        health: Rc<RefCell<CanBusHealth>>,
    }

    impl MockCan {
        fn new(bus: Rc<RefCell<SharedBus>>) -> Self {
            Self {
                bus,
                ready: true,
                tx_capacity: None,
                health: Rc::new(RefCell::new(CanBusHealth::healthy())),
            }
        }

        fn with_tx_capacity(bus: Rc<RefCell<SharedBus>>, tx_capacity: usize) -> Self {
            Self {
                bus,
                ready: true,
                tx_capacity: Some(tx_capacity),
                health: Rc::new(RefCell::new(CanBusHealth::healthy())),
            }
        }
    }

    impl CanDevice for MockCan {
        type Error = ();

        fn tx_ready(&self) -> bool {
            self.ready
                && self
                    .tx_capacity
                    .is_none_or(|capacity| self.bus.borrow().frames.len() < capacity)
        }

        fn send(&mut self, id: u32, data: &[u8]) -> Result<(), Self::Error> {
            let mut arr = [0u8; 64];
            let len = core::cmp::min(64, data.len());
            arr[..len].copy_from_slice(&data[..len]);
            self.bus.borrow_mut().frames.push_back((id, arr, len));
            Ok(())
        }

        fn try_receive(&mut self, buf: &mut [u8]) -> Option<(u32, usize)> {
            let mut bus = self.bus.borrow_mut();
            let (id, data, len) = bus.frames.pop_front()?;
            buf[..len].copy_from_slice(&data[..len]);
            Some((id, len))
        }

        fn bus_health(&self) -> CanBusHealth {
            *self.health.borrow()
        }
    }

    fn table_message(version: u32) -> Message {
        let mut table = [[0u16; 16]; 16];
        for (i, row) in table.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (1000 + i * 16 + j) as u16;
            }
        }
        Message::IpwTable {
            version,
            data: table,
            crc32: 0xDEADBEEF,
        }
    }

    fn ignition_table_message(version: u32) -> Message {
        let mut table = [[0u16; 16]; 16];
        for (i, row) in table.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (2000 + i * 16 + j) as u16;
            }
        }
        Message::IgnitionTable {
            version,
            data: table,
            crc32: 0xA5A55A5A,
        }
    }

    fn take_all_frames(bus: &Rc<RefCell<SharedBus>>) -> Vec<(u32, [u8; 64], usize)> {
        bus.borrow_mut().frames.drain(..).collect()
    }

    fn drain_receive<D: CanDevice>(rx: &mut CanTransport<D>, polls: usize) -> Option<Message> {
        let mut got = None;
        for _ in 0..polls {
            if let Some(msg) = rx.try_receive() {
                got = Some(msg);
                break;
            }
        }
        got
    }

    fn expected_can_frame_count(message: &Message) -> usize {
        let encoded_len = message.encoded_len().expect("encoded len");
        if encoded_len <= MAX_CAN_DATA.saturating_sub(1) {
            1
        } else {
            let first_frame_payload = MAX_CAN_DATA.saturating_sub(5);
            let continuation_payload = MAX_CAN_DATA.saturating_sub(1);
            let remaining = encoded_len.saturating_sub(first_frame_payload);
            1 + remaining.div_ceil(continuation_payload)
        }
    }

    #[test]
    fn segmented_table_roundtrips() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus));

        let msg = table_message(7);

        tx.send(&msg).expect("send");
        assert_eq!(drain_receive(&mut rx, 1024), Some(msg));
        assert_eq!(rx.last_receive_error(), None);
        assert_eq!(
            rx.sequence_tracking(),
            CanSequenceTracking {
                completed_streams: 1,
                duplicate_start_count: 0,
                unexpected_continuation_count: 0,
                missing_fragment_count: 0,
                last_event: CanSequenceEvent::InOrderComplete,
            }
        );
    }

    #[test]
    fn segmented_send_stops_when_continuation_capacity_disappears() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::with_tx_capacity(bus.clone(), 1));

        let result = tx.send(&table_message(7));

        assert_eq!(result, Err(TransportError::NotReady));
        assert_eq!(bus.borrow().frames.len(), 1);
        assert_eq!(tx.stats().tx_count, 0);
        assert_eq!(tx.stats().tx_errors, 1);
    }

    #[test]
    fn transport_parity_sample_set_roundtrips_with_expected_can_framing() {
        for message in transport_parity_sample_messages() {
            let bus = Rc::new(RefCell::new(SharedBus {
                frames: VecDeque::new(),
            }));
            let mut tx = CanTransport::new(MockCan::new(bus.clone()));
            let mut rx = CanTransport::new(MockCan::new(bus.clone()));

            tx.send(&message).expect("send");

            assert_eq!(
                bus.borrow().frames.len(),
                expected_can_frame_count(&message)
            );
            assert_eq!(drain_receive(&mut rx, 1024), Some(message));
            assert_eq!(rx.last_receive_error(), None);
        }
    }

    #[test]
    fn classic_can_single_frame_boundary_uses_header_payload_capacity() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let msg = Message::CmdReset { target_node_id: 2 };

        tx.send(&msg).expect("send");
        let frames = &bus.borrow().frames;
        assert_eq!(frames.len(), 1);
        let (_, data, len) = frames.front().expect("frame");
        assert_eq!(data[0] >> 6, 0);
        assert!(*len <= MAX_CAN_DATA);
    }

    #[test]
    fn reassembly_rejects_mixed_frame_ids() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let msg = table_message(8);

        tx.send(&msg).expect("send");
        let mut frames = take_all_frames(&bus);
        {
            let second = frames.get_mut(1).expect("continuation frame");
            second.0 ^= 0x001;
        }
        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(frames[0]);
            queue.frames.push_back(frames[1]);
        }

        assert_eq!(rx.try_receive(), None);
        assert_eq!(rx.try_receive(), None);
        assert_eq!(
            rx.last_receive_error(),
            Some(TransportError::DeserializationFailed)
        );
        assert!(rx.stats().rx_errors > 0);
        assert_eq!(
            rx.sequence_tracking(),
            CanSequenceTracking {
                completed_streams: 0,
                duplicate_start_count: 0,
                unexpected_continuation_count: 1,
                missing_fragment_count: 0,
                last_event: CanSequenceEvent::UnexpectedContinuation,
            }
        );
    }

    #[test]
    fn reassembly_rejects_early_end_fragment() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));

        tx.send(&table_message(9)).expect("send");
        let mut frames = take_all_frames(&bus);
        {
            let second = frames.get_mut(1).expect("continuation frame");
            second.1[0] = (0b11 << 6) | (second.1[0] & 0x3f);
        }
        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(frames[0]);
            queue.frames.push_back(frames[1]);
        }

        assert_eq!(rx.try_receive(), None);
        assert_eq!(rx.try_receive(), None);
        assert_eq!(
            rx.last_receive_error(),
            Some(TransportError::DeserializationFailed)
        );
        assert_eq!(
            rx.sequence_tracking(),
            CanSequenceTracking {
                completed_streams: 0,
                duplicate_start_count: 0,
                unexpected_continuation_count: 0,
                missing_fragment_count: 1,
                last_event: CanSequenceEvent::MissingFragments,
            }
        );
    }

    #[test]
    fn reassembly_recovers_after_invalid_sequence() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut bad_tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut good_tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let bad = table_message(10);
        let good = table_message(11);

        bad_tx.send(&bad).expect("send bad");
        let mut bad_frames = take_all_frames(&bus);
        {
            let second = bad_frames.get_mut(1).expect("continuation frame");
            second.1[0] = (0b11 << 6) | (second.1[0] & 0x3f);
        }
        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(bad_frames[0]);
            queue.frames.push_back(bad_frames[1]);
        }
        assert_eq!(rx.try_receive(), None);
        assert_eq!(rx.try_receive(), None);

        good_tx.send(&good).expect("send good");
        assert_eq!(drain_receive(&mut rx, 1024), Some(good));
        assert_eq!(rx.last_receive_error(), None);
        assert_eq!(
            rx.sequence_tracking(),
            CanSequenceTracking {
                completed_streams: 1,
                duplicate_start_count: 0,
                unexpected_continuation_count: 0,
                missing_fragment_count: 1,
                last_event: CanSequenceEvent::InOrderComplete,
            }
        );
    }

    #[test]
    fn duplicate_start_is_tracked_explicitly() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));

        tx.send(&table_message(12)).expect("send");
        let frames = take_all_frames(&bus);
        assert!(!frames.is_empty());
        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(frames[0]);
            queue.frames.push_back(frames[0]);
        }

        assert_eq!(rx.try_receive(), None);
        assert_eq!(rx.try_receive(), None);
        assert_eq!(
            rx.last_receive_error(),
            Some(TransportError::DeserializationFailed)
        );
        assert_eq!(
            rx.sequence_tracking(),
            CanSequenceTracking {
                completed_streams: 0,
                duplicate_start_count: 1,
                unexpected_continuation_count: 0,
                missing_fragment_count: 0,
                last_event: CanSequenceEvent::DuplicateStart,
            }
        );
    }

    #[test]
    fn interleaved_fragmented_streams_with_same_tid_and_distinct_ids_roundtrip() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx_a = CanTransport::new(MockCan::new(bus.clone()));
        let mut tx_b = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let msg_a = table_message(21);
        let msg_b = ignition_table_message(22);

        tx_a.send(&msg_a).expect("send a");
        tx_b.send(&msg_b).expect("send b");
        let frames = take_all_frames(&bus);
        assert!(frames.len() >= 4);

        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(frames[0]);
            queue.frames.push_back(frames[frames.len() / 2]);
            for frame in &frames[1..frames.len() / 2] {
                queue.frames.push_back(*frame);
            }
            for frame in &frames[(frames.len() / 2) + 1..] {
                queue.frames.push_back(*frame);
            }
        }

        assert_eq!(drain_receive(&mut rx, 1024), Some(msg_a));
        assert_eq!(drain_receive(&mut rx, 1024), Some(msg_b));
        assert_eq!(rx.last_receive_error(), None);
    }

    #[test]
    fn single_frame_can_arrive_while_fragmented_stream_is_active() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let fragmented = table_message(30);
        let single = Message::CmdReset { target_node_id: 7 };

        tx.send(&fragmented).expect("send fragmented");
        tx.send(&single).expect("send single");
        let frames = take_all_frames(&bus);
        assert!(frames.len() >= 3);

        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(frames[0]);
            queue
                .frames
                .push_back(*frames.last().expect("single frame"));
            for frame in &frames[1..frames.len() - 1] {
                queue.frames.push_back(*frame);
            }
        }

        assert_eq!(drain_receive(&mut rx, 1024), Some(single));
        assert_eq!(drain_receive(&mut rx, 1024), Some(fragmented));
        assert_eq!(rx.last_receive_error(), None);
    }

    #[test]
    fn bad_stream_does_not_abort_other_active_stream() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let mut tx_a = CanTransport::new(MockCan::new(bus.clone()));
        let mut tx_b = CanTransport::new(MockCan::new(bus.clone()));
        let mut rx = CanTransport::new(MockCan::new(bus.clone()));
        let good = table_message(31);
        let bad = ignition_table_message(32);

        tx_a.send(&good).expect("send good");
        tx_b.send(&bad).expect("send bad");
        let mut frames = take_all_frames(&bus);
        let split = frames.len() / 2;
        assert!(split >= 2);

        frames[split + 1].0 ^= 0x001;

        {
            let mut queue = bus.borrow_mut();
            queue.frames.push_back(frames[0]);
            queue.frames.push_back(frames[split]);
            queue.frames.push_back(frames[split + 1]);
            for frame in &frames[1..split] {
                queue.frames.push_back(*frame);
            }
            for frame in &frames[split + 2..] {
                queue.frames.push_back(*frame);
            }
        }

        assert_eq!(rx.try_receive(), None);
        assert_eq!(rx.try_receive(), None);
        assert_eq!(rx.try_receive(), None);
        assert_eq!(
            rx.last_receive_error(),
            Some(TransportError::DeserializationFailed)
        );
        assert_eq!(drain_receive(&mut rx, 1024), Some(good));
    }

    #[test]
    fn default_bus_health_is_healthy_and_ready() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let tx = CanTransport::new(MockCan::new(bus));

        assert_eq!(tx.bus_health(), CanBusHealth::healthy());
        assert!(tx.is_ready());
    }

    #[test]
    fn degraded_bus_health_keeps_transport_ready() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let dev = MockCan::new(bus);
        *dev.health.borrow_mut() = CanBusHealth {
            state: CanBusHealthState::Degraded,
            tx_error_count: 12,
            rx_error_count: 3,
            bus_off: false,
            recovery: CanBusRecoveryState::None,
        };
        let tx = CanTransport::new(dev);

        assert_eq!(
            tx.bus_health(),
            CanBusHealth {
                state: CanBusHealthState::Degraded,
                tx_error_count: 12,
                rx_error_count: 3,
                bus_off: false,
                recovery: CanBusRecoveryState::None,
            }
        );
        assert!(tx.is_ready());
    }

    #[test]
    fn bus_off_blocks_send_and_marks_transport_not_ready() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let dev = MockCan::new(bus.clone());
        *dev.health.borrow_mut() = CanBusHealth {
            state: CanBusHealthState::BusOff,
            tx_error_count: 255,
            rx_error_count: 10,
            bus_off: true,
            recovery: CanBusRecoveryState::Needed,
        };
        let mut tx = CanTransport::new(dev);

        assert!(!tx.is_ready());
        assert_eq!(
            tx.send(&Message::CmdReset { target_node_id: 1 }),
            Err(TransportError::NotReady)
        );
        assert!(bus.borrow().frames.is_empty());
        assert_eq!(tx.stats().tx_errors, 1);
    }

    #[test]
    fn recovery_progress_transitions_back_to_ready() {
        let bus = Rc::new(RefCell::new(SharedBus {
            frames: VecDeque::new(),
        }));
        let dev = MockCan::new(bus.clone());
        let health = dev.health.clone();
        let mut tx = CanTransport::new(dev);

        *health.borrow_mut() = CanBusHealth {
            state: CanBusHealthState::BusOff,
            tx_error_count: 200,
            rx_error_count: 7,
            bus_off: true,
            recovery: CanBusRecoveryState::Needed,
        };
        assert!(!tx.is_ready());

        *health.borrow_mut() = CanBusHealth {
            state: CanBusHealthState::Degraded,
            tx_error_count: 200,
            rx_error_count: 7,
            bus_off: false,
            recovery: CanBusRecoveryState::InProgress,
        };
        assert!(!tx.is_ready());
        assert_eq!(
            tx.send(&Message::CmdReset { target_node_id: 2 }),
            Err(TransportError::NotReady)
        );

        *health.borrow_mut() = CanBusHealth {
            state: CanBusHealthState::Healthy,
            tx_error_count: 0,
            rx_error_count: 0,
            bus_off: false,
            recovery: CanBusRecoveryState::None,
        };
        assert!(tx.is_ready());
        tx.send(&Message::CmdReset { target_node_id: 2 })
            .expect("send after recovery");
        assert_eq!(bus.borrow().frames.len(), 1);
    }

    #[test]
    fn route_surface_preserves_stable_ids_for_live_message_classes() {
        let trigger = Message::TriggerTiming {
            gap_period_us: 2000,
            tooth_period_us: 1000,
            tooth_position: 1,
            synced: true,
            timestamp_us: 0,
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };
        let gps = Message::GpsData {
            latitude_e7: 377749000,
            longitude_e7: -1224194000,
            speed_cm_per_s: 1834,
            heading_deg_x10: 2715,
            timestamp_us: 0,
        };
        let egt = Message::ExternalEgtData {
            bank: 1,
            channel: 2,
            egt_c_x10: 8650,
            flags: 0,
            timestamp_us: 0,
        };
        let obd_request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };
        let obd_response = Message::Obd2Response {
            service: 0x41,
            parameter_id: Some(0x0C),
            negative_response_code: None,
            payload_len: 2,
            payload: [0x1A, 0xF8, 0, 0, 0, 0],
        };
        let obd_segmented_response = Message::Obd2SegmentedResponse {
            service: 0x49,
            parameter_id: Some(0x02),
            sequence_index: 0,
            segment_count: 3,
            total_payload_len: 17,
            segment_len: 6,
            segment: *b"PIPOCO",
        };
        let reset = Message::CmdReset { target_node_id: 1 };
        let fault = Message::Error {
            node_id: 1,
            error_code: 2,
            severity: 3,
            timestamp_us: 0,
            data: [0; 4],
        };
        let calibrate = Message::CmdCalibrate {
            target_node_id: 1,
            cal_type: 2,
        };
        let obd_identity_command = Message::Obd2IdentityProvisioningCommand {
            request_id: 1,
            vin_len: 17,
            vin: *b"VIN00000000000001",
            calibration_id_len: 17,
            calibration_id: *b"CAL00000000000001",
            board_build_identity_len: 4,
            board_build_identity: [b'M', b'4', b'A', b'1', 0, 0],
        };
        let obd_identity_audit = Message::Obd2IdentityProvisioningAudit {
            request_id: 1,
            authorized: true,
            authorization_failed: false,
            attempted: true,
            accepted: true,
            store_failed: false,
            rejected_field: 0,
            vin_len: 17,
            calibration_id_len: 17,
            board_build_identity_len: 4,
        };
        let obd_identity_arm = Message::Obd2IdentityProvisioningArm {
            request_id: 1,
            nonce: 2,
            vin_len: 17,
            vin: *b"VIN00000000000001",
            calibration_id_len: 17,
            calibration_id: *b"CAL00000000000001",
            board_build_identity_len: 4,
            board_build_identity: [b'M', b'4', b'A', b'1', 0, 0],
            tag: [0xA5; 32],
        };
        let obd_identity_arm_audit = Message::Obd2IdentityProvisioningArmAudit {
            request_id: 1,
            armed: true,
        };
        let obd_identity_key_command = Message::Obd2IdentityProvisioningKeyCommand {
            request_id: 1,
            nonce: 2,
            generation: 3,
            revoke: false,
            key: [0x11; 32],
            tag: [0xA5; 32],
        };
        let obd_identity_key_audit = Message::Obd2IdentityProvisioningKeyAudit {
            request_id: 1,
            authorized: true,
            accepted: true,
            store_failed: false,
            rejected_reason: 0,
            generation: 3,
        };
        let heartbeat = Message::Heartbeat {
            node_id: 1,
            uptime_seconds: 10,
            status: 0,
            error_count: 0,
            cpu_usage: 5,
        };

        assert_eq!(
            CanTransport::<MockCan>::route_for(&reset),
            CanMessageRoute::new(CanMessageClass::ResetCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&fault),
            CanMessageRoute::new(CanMessageClass::Fault)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&trigger),
            CanMessageRoute::new(CanMessageClass::TriggerTiming)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&sensor),
            CanMessageRoute::new(CanMessageClass::SensorData)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&gps),
            CanMessageRoute::new(CanMessageClass::GpsData)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&egt),
            CanMessageRoute::new(CanMessageClass::ExternalEgtData)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_request),
            CanMessageRoute::new(CanMessageClass::Obd2Request)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_response),
            CanMessageRoute::new(CanMessageClass::Obd2Response)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_segmented_response),
            CanMessageRoute::new(CanMessageClass::Obd2Response)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&calibrate),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_identity_command),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_identity_audit),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_identity_arm),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_identity_arm_audit),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_identity_key_command),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&obd_identity_key_audit),
            CanMessageRoute::new(CanMessageClass::CalibrationCommand)
        );
        assert_eq!(
            CanTransport::<MockCan>::route_for(&heartbeat),
            CanMessageRoute::new(CanMessageClass::Heartbeat)
        );
    }

    #[test]
    fn route_map_covers_every_known_can_message_class() {
        let expected = [
            CanMessageClass::Fault,
            CanMessageClass::ResetCommand,
            CanMessageClass::EngineControlCommand,
            CanMessageClass::TriggerTiming,
            CanMessageClass::SensorData,
            CanMessageClass::GpsData,
            CanMessageClass::ExternalEgtData,
            CanMessageClass::FuelTable,
            CanMessageClass::IgnitionTable,
            CanMessageClass::EngineConfig,
            CanMessageClass::InjectorConfig,
            CanMessageClass::CalibrationCommand,
            CanMessageClass::Obd2Request,
            CanMessageClass::Obd2Response,
            CanMessageClass::Heartbeat,
        ];

        assert_eq!(CAN_ROUTE_MAP.len(), CAN_MESSAGE_CLASS_COUNT);
        for (route, class) in CAN_ROUTE_MAP.iter().zip(expected.iter()) {
            assert_eq!(route.class, *class);
            assert_eq!(*route, CanMessageRoute::new(*class));
        }
    }

    #[test]
    fn route_map_ids_are_unique_and_priority_order_is_stable() {
        for (i, route) in CAN_ROUTE_MAP.iter().enumerate() {
            for other in CAN_ROUTE_MAP.iter().skip(i + 1) {
                assert_ne!(route.arbitration_id, other.arbitration_id);
            }
        }

        let reset = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::ResetCommand)
            .expect("reset route");
        let fault = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::Fault)
            .expect("fault route");
        let sensor = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::SensorData)
            .expect("sensor route");
        let gps = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::GpsData)
            .expect("gps route");
        let egt = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::ExternalEgtData)
            .expect("egt route");
        let heartbeat = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::Heartbeat)
            .expect("heartbeat route");
        let fuel_table = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::FuelTable)
            .expect("fuel table route");
        let calibration = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::CalibrationCommand)
            .expect("calibration route");
        let obd_request = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::Obd2Request)
            .expect("obd request route");
        let obd_response = CAN_ROUTE_MAP
            .iter()
            .find(|route| route.class == CanMessageClass::Obd2Response)
            .expect("obd response route");

        assert!(reset.arbitration_id < sensor.arbitration_id);
        assert!(sensor.arbitration_id < gps.arbitration_id);
        assert!(gps.arbitration_id < egt.arbitration_id);
        assert!(egt.arbitration_id < fuel_table.arbitration_id);
        assert!(calibration.arbitration_id < obd_request.arbitration_id);
        assert!(obd_request.arbitration_id < obd_response.arbitration_id);
        assert!(obd_response.arbitration_id < heartbeat.arbitration_id);
        assert!(fault.arbitration_id < heartbeat.arbitration_id);
        assert!(reset.priority < sensor.priority);
        assert!(fault.priority < heartbeat.priority);
        assert_eq!(gps.priority, CanMessagePriority::Sensor);
        assert_eq!(egt.priority, CanMessagePriority::Sensor);
        assert_eq!(obd_request.priority, CanMessagePriority::Configuration);
        assert_eq!(obd_response.priority, CanMessagePriority::Configuration);
    }

    #[test]
    fn filter_policy_map_covers_every_known_can_message_class() {
        assert_eq!(CAN_FILTER_POLICY_MAP.len(), CAN_MESSAGE_CLASS_COUNT);

        for route in CAN_ROUTE_MAP {
            let matches = CAN_FILTER_POLICY_MAP
                .iter()
                .filter(|entry| entry.class == route.class)
                .count();
            assert_eq!(matches, 1);
        }
    }

    #[test]
    fn filter_policy_keeps_mandatory_control_routes_in_core_control() {
        let required_core = [
            CanMessageClass::Fault,
            CanMessageClass::ResetCommand,
            CanMessageClass::EngineControlCommand,
            CanMessageClass::TriggerTiming,
            CanMessageClass::SensorData,
            CanMessageClass::Heartbeat,
        ];

        for class in required_core {
            let entry = CAN_FILTER_POLICY_MAP
                .iter()
                .find(|entry| entry.class == class)
                .expect("policy entry");
            assert_eq!(entry.group, CanFilterPolicyGroup::CoreControl);
        }
    }

    #[test]
    fn filter_policy_mapping_is_stable_for_current_live_classes() {
        for class in [
            CanMessageClass::FuelTable,
            CanMessageClass::IgnitionTable,
            CanMessageClass::EngineConfig,
            CanMessageClass::InjectorConfig,
            CanMessageClass::CalibrationCommand,
            CanMessageClass::Obd2Request,
            CanMessageClass::Obd2Response,
        ] {
            let entry = CAN_FILTER_POLICY_MAP
                .iter()
                .find(|entry| entry.class == class)
                .expect("policy entry");
            assert_eq!(entry.group, CanFilterPolicyGroup::CalibrationDiagnostic);
        }

        for class in [CanMessageClass::GpsData, CanMessageClass::ExternalEgtData] {
            let entry = CAN_FILTER_POLICY_MAP
                .iter()
                .find(|entry| entry.class == class)
                .expect("optional telemetry policy entry");
            assert_eq!(entry.group, CanFilterPolicyGroup::OptionalTelemetry);
        }
    }

    #[test]
    fn standard_device_profile_dash_matches_current_dash_outputs() {
        let profile = CanStandardDeviceClass::Dash.profile();

        assert_eq!(profile.device_class, CanStandardDeviceClass::Dash);
        assert_eq!(profile.support, CanStandardDeviceSupport::Supported);
        assert_eq!(profile.required_rx_message_class_count, 0);
        assert_eq!(profile.required_tx_message_class_count, 3);
        assert_eq!(
            [
                profile.required_tx_message_class(0),
                profile.required_tx_message_class(1),
                profile.required_tx_message_class(2),
            ],
            [
                Some(CanMessageClass::SensorData),
                Some(CanMessageClass::Fault),
                Some(CanMessageClass::Heartbeat),
            ]
        );
        assert_eq!(
            profile.required_filter_policy_group(0),
            Some(CanFilterPolicyGroup::CoreControl)
        );
        assert!(profile.heartbeat_required);
        assert!(profile.is_supported());
    }

    #[test]
    fn standard_device_profile_logger_matches_current_stream_outputs() {
        let profile = CanStandardDeviceClass::Logger.profile();

        assert_eq!(profile.device_class, CanStandardDeviceClass::Logger);
        assert_eq!(profile.support, CanStandardDeviceSupport::Supported);
        assert_eq!(profile.required_rx_message_class_count, 0);
        assert_eq!(profile.required_tx_message_class_count, 4);
        assert_eq!(
            [
                profile.required_tx_message_class(0),
                profile.required_tx_message_class(1),
                profile.required_tx_message_class(2),
                profile.required_tx_message_class(3),
            ],
            [
                Some(CanMessageClass::TriggerTiming),
                Some(CanMessageClass::SensorData),
                Some(CanMessageClass::Fault),
                Some(CanMessageClass::Heartbeat),
            ]
        );
        assert_eq!(
            profile.required_filter_policy_group(0),
            Some(CanFilterPolicyGroup::CoreControl)
        );
        assert!(profile.heartbeat_required);
        assert!(profile.is_supported());
    }

    #[test]
    fn standard_device_profile_pdm_matches_current_power_distribution_contract() {
        let profile = CanStandardDeviceClass::Pdm.profile();

        assert_eq!(profile.device_class, CanStandardDeviceClass::Pdm);
        assert_eq!(profile.support, CanStandardDeviceSupport::Supported);
        assert_eq!(
            [
                profile.required_rx_message_class(0),
                profile.required_rx_message_class(1),
            ],
            [
                Some(CanMessageClass::Fault),
                Some(CanMessageClass::Heartbeat),
            ]
        );
        assert_eq!(
            [
                profile.required_tx_message_class(0),
                profile.required_tx_message_class(1),
            ],
            [
                Some(CanMessageClass::EngineControlCommand),
                Some(CanMessageClass::ResetCommand),
            ]
        );
        assert_eq!(
            profile.required_filter_policy_group(0),
            Some(CanFilterPolicyGroup::CoreControl)
        );
        assert!(profile.heartbeat_required);
        assert!(profile.is_supported());
    }

    #[test]
    fn standard_device_profile_sensor_classes_stay_honest_about_current_transport_coverage() {
        let lambda = CanStandardDeviceClass::ExternalLambda.profile();
        let gps = CanStandardDeviceClass::Gps.profile();
        let egt = CanStandardDeviceClass::ExternalEgt.profile();

        assert_eq!(lambda.support, CanStandardDeviceSupport::Supported);
        assert_eq!(
            [
                lambda.required_rx_message_class(0),
                lambda.required_rx_message_class(1),
            ],
            [
                Some(CanMessageClass::SensorData),
                Some(CanMessageClass::Heartbeat),
            ]
        );
        assert_eq!(
            lambda.required_filter_policy_group(0),
            Some(CanFilterPolicyGroup::CoreControl)
        );

        assert_eq!(gps.support, CanStandardDeviceSupport::Supported);
        assert_eq!(
            [
                gps.required_rx_message_class(0),
                gps.required_rx_message_class(1)
            ],
            [
                Some(CanMessageClass::GpsData),
                Some(CanMessageClass::Heartbeat),
            ]
        );
        assert_eq!(
            gps.required_filter_policy_group(0),
            Some(CanFilterPolicyGroup::OptionalTelemetry)
        );
        assert!(gps.heartbeat_required);
        assert!(gps.is_supported());

        assert_eq!(egt.support, CanStandardDeviceSupport::Supported);
        assert_eq!(
            [
                egt.required_rx_message_class(0),
                egt.required_rx_message_class(1)
            ],
            [
                Some(CanMessageClass::ExternalEgtData),
                Some(CanMessageClass::Heartbeat),
            ]
        );
        assert_eq!(
            egt.required_filter_policy_group(0),
            Some(CanFilterPolicyGroup::OptionalTelemetry)
        );
        assert!(egt.heartbeat_required);
        assert!(egt.is_supported());
    }

    #[test]
    fn standard_device_profile_map_covers_every_known_device_class() {
        let expected = [
            CanStandardDeviceClass::Dash,
            CanStandardDeviceClass::Logger,
            CanStandardDeviceClass::Pdm,
            CanStandardDeviceClass::Gps,
            CanStandardDeviceClass::ExternalLambda,
            CanStandardDeviceClass::ExternalEgt,
        ];

        assert_eq!(
            CAN_STANDARD_DEVICE_PROFILE_MAP.len(),
            CAN_STANDARD_DEVICE_PROFILE_COUNT
        );
        for (profile, class) in CAN_STANDARD_DEVICE_PROFILE_MAP.iter().zip(expected.iter()) {
            assert_eq!(profile.device_class, *class);
            assert_eq!(*profile, class.profile());
        }
    }

    #[test]
    fn obd2_service_surface_reports_request_shape() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2ServiceSurface::from_message(&request),
            Some(CanObd2ServiceSurface {
                direction: CanObd2ServiceDirection::Request,
                service_id: 0x01,
                parameter_id: Some(0x0C),
                payload_len: 0,
                expects_response: true,
                response_kind: None,
            })
        );
    }

    #[test]
    fn obd2_service_surface_reports_positive_response_shape() {
        let response = Message::Obd2Response {
            service: 0x41,
            parameter_id: Some(0x0C),
            negative_response_code: None,
            payload_len: 2,
            payload: [0x1A, 0xF8, 0, 0, 0, 0],
        };

        assert_eq!(
            CanObd2ServiceSurface::from_message(&response),
            Some(CanObd2ServiceSurface {
                direction: CanObd2ServiceDirection::Response,
                service_id: 0x41,
                parameter_id: Some(0x0C),
                payload_len: 2,
                expects_response: false,
                response_kind: Some(CanObd2ResponseKind::Positive),
            })
        );
    }

    #[test]
    fn obd2_service_surface_reports_pending_response_shape() {
        let response = Message::Obd2Response {
            service: 0x7F,
            parameter_id: Some(0x01),
            negative_response_code: Some(0x78),
            payload_len: 1,
            payload: [0x78, 0, 0, 0, 0, 0],
        };

        assert_eq!(
            CanObd2ServiceSurface::from_message(&response),
            Some(CanObd2ServiceSurface {
                direction: CanObd2ServiceDirection::Response,
                service_id: 0x7F,
                parameter_id: Some(0x01),
                payload_len: 1,
                expects_response: false,
                response_kind: Some(CanObd2ResponseKind::Pending),
            })
        );
    }

    #[test]
    fn obd2_supported_pid_catalog_covers_current_sensor_backed_pids() {
        let expected = [
            CanObd2Pid::MonitorStatusSinceDtcsCleared,
            CanObd2Pid::EngineCoolantTemperature,
            CanObd2Pid::IntakeAirTemperature,
            CanObd2Pid::ThrottlePosition,
            CanObd2Pid::ControlModuleVoltage,
        ];

        assert_eq!(
            CAN_OBD2_SUPPORTED_PID_CATALOG.len(),
            CAN_OBD2_SUPPORTED_PID_COUNT
        );
        for (profile, pid) in CAN_OBD2_SUPPORTED_PID_CATALOG.iter().zip(expected.iter()) {
            assert_eq!(profile.pid, *pid);
            assert_eq!(*profile, pid.profile());
        }
    }

    #[test]
    fn obd2_supported_pid_catalog_preserves_stable_service_and_pid_ids() {
        assert_eq!(
            CanObd2Pid::EngineCoolantTemperature.profile().service_id,
            0x01
        );
        assert_eq!(CanObd2Pid::MonitorStatusSinceDtcsCleared.pid_id(), 0x01);
        assert_eq!(CanObd2Pid::EngineCoolantTemperature.pid_id(), 0x05);
        assert_eq!(CanObd2Pid::IntakeAirTemperature.pid_id(), 0x0F);
        assert_eq!(CanObd2Pid::ThrottlePosition.pid_id(), 0x11);
        assert_eq!(CanObd2Pid::ControlModuleVoltage.pid_id(), 0x42);
    }

    #[test]
    fn obd2_supported_pid_catalog_preserves_expected_backing_and_payload_shape() {
        for profile in CAN_OBD2_SUPPORTED_PID_CATALOG {
            match profile.pid {
                CanObd2Pid::MonitorStatusSinceDtcsCleared => {
                    assert_eq!(profile.backing, CanObd2PidBacking::ReadinessMonitor);
                    assert_eq!(profile.response_payload_len, 4);
                }
                CanObd2Pid::EngineCoolantTemperature
                | CanObd2Pid::IntakeAirTemperature
                | CanObd2Pid::ThrottlePosition
                | CanObd2Pid::ControlModuleVoltage => {
                    assert_eq!(
                        profile.backing,
                        CanObd2PidBacking::Message(CanMessageClass::SensorData)
                    );
                }
            }
        }

        assert_eq!(
            CanObd2Pid::MonitorStatusSinceDtcsCleared.response_payload_len(),
            4
        );
        assert_eq!(
            CanObd2Pid::EngineCoolantTemperature.response_payload_len(),
            1
        );
        assert_eq!(CanObd2Pid::IntakeAirTemperature.response_payload_len(), 1);
        assert_eq!(CanObd2Pid::ThrottlePosition.response_payload_len(), 1);
        assert_eq!(CanObd2Pid::ControlModuleVoltage.response_payload_len(), 2);
    }

    #[test]
    fn obd2_value_projection_surface_covers_every_supported_pid() {
        for profile in CAN_OBD2_SUPPORTED_PID_CATALOG {
            match profile.backing {
                CanObd2PidBacking::Message(backing_message_class) => {
                    let projection = profile.pid.value_projection().expect("message-backed pid");
                    assert_eq!(projection.pid, profile.pid);
                    assert_eq!(projection.backing_message_class, backing_message_class);
                    assert_eq!(projection.payload_len, profile.response_payload_len);
                }
                CanObd2PidBacking::ReadinessMonitor => {
                    assert_eq!(profile.pid.value_projection(), None);
                }
            }
        }
    }

    #[test]
    fn obd2_value_projection_surface_preserves_stable_field_mapping() {
        assert_eq!(
            CanObd2Pid::EngineCoolantTemperature
                .value_projection()
                .expect("projection")
                .field,
            CanObd2ValueProjectionField::EngineCoolantTemperatureOffset
        );
        assert_eq!(
            CanObd2Pid::IntakeAirTemperature
                .value_projection()
                .expect("projection")
                .field,
            CanObd2ValueProjectionField::IntakeAirTemperatureOffset
        );
        assert_eq!(
            CanObd2Pid::ThrottlePosition
                .value_projection()
                .expect("projection")
                .field,
            CanObd2ValueProjectionField::ThrottlePercent
        );
        assert_eq!(
            CanObd2Pid::ControlModuleVoltage
                .value_projection()
                .expect("projection")
                .field,
            CanObd2ValueProjectionField::ControlModuleVoltageDecivolts
        );
    }

    #[test]
    fn obd2_value_projection_surface_projects_sensor_data_bytes() {
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2Pid::EngineCoolantTemperature
                .value_projection()
                .expect("projection")
                .project_from_message(&sensor),
            Some(CanObd2ProjectedPayload {
                payload_len: 1,
                bytes: [60, 0],
            })
        );
        assert_eq!(
            CanObd2Pid::IntakeAirTemperature
                .value_projection()
                .expect("projection")
                .project_from_message(&sensor),
            Some(CanObd2ProjectedPayload {
                payload_len: 1,
                bytes: [50, 0],
            })
        );
        assert_eq!(
            CanObd2Pid::ThrottlePosition
                .value_projection()
                .expect("projection")
                .project_from_message(&sensor),
            Some(CanObd2ProjectedPayload {
                payload_len: 1,
                bytes: [26, 0],
            })
        );
        assert_eq!(
            CanObd2Pid::ControlModuleVoltage
                .value_projection()
                .expect("projection")
                .project_from_message(&sensor),
            Some(CanObd2ProjectedPayload {
                payload_len: 2,
                bytes: [0x2E, 0xE0],
            })
        );
    }

    #[test]
    fn obd2_response_assembly_surface_builds_supported_positive_response() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x11),
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2ResponseAssemblySurface::assemble(&request, &sensor),
            Ok(CanObd2ResponseAssemblySurface {
                request_service_id: 0x01,
                pid: CanObd2Pid::ThrottlePosition,
                positive_response_service_id: 0x41,
                payload: CanObd2ProjectedPayload {
                    payload_len: 1,
                    bytes: [26, 0],
                },
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x11),
                    negative_response_code: None,
                    payload_len: 1,
                    payload: [26, 0, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_response_assembly_surface_preserves_positive_response_service_rule() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x42),
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        let assembled =
            CanObd2ResponseAssemblySurface::assemble(&request, &sensor).expect("assemble");
        assert_eq!(assembled.request_service_id, 0x01);
        assert_eq!(assembled.positive_response_service_id, 0x41);
        assert_eq!(
            assembled.response,
            Message::Obd2Response {
                service: 0x41,
                parameter_id: Some(0x42),
                negative_response_code: None,
                payload_len: 2,
                payload: [0x2E, 0xE0, 0, 0, 0, 0],
            }
        );
    }

    #[test]
    fn obd2_response_assembly_surface_rejects_unsupported_or_non_catalog_requests() {
        let unsupported_service = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        };
        let unsupported_pid = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };
        let missing_pid = Message::Obd2Request {
            service: 0x01,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2ResponseAssemblySurface::assemble(&unsupported_service, &sensor),
            Err(CanObd2ResponseAssemblyError::UnsupportedService { service_id: 0x09 })
        );
        assert_eq!(
            CanObd2ResponseAssemblySurface::assemble(&unsupported_pid, &sensor),
            Err(CanObd2ResponseAssemblyError::UnsupportedPid { pid_id: 0x0C })
        );
        assert_eq!(
            CanObd2ResponseAssemblySurface::assemble(&missing_pid, &sensor),
            Err(CanObd2ResponseAssemblyError::MissingParameterId)
        );
        assert_eq!(
            CanObd2ResponseAssemblySurface::assemble(
                &Message::CmdReset { target_node_id: 1 },
                &sensor
            ),
            Err(CanObd2ResponseAssemblyError::UnsupportedService { service_id: 0 })
        );
    }

    #[test]
    fn obd2_supported_pid_bitmap_surface_reports_first_pid_block() {
        assert_eq!(
            CanObd2SupportedPidBitmapSurface::from_pid_block(0x00),
            Some(CanObd2SupportedPidBitmapSurface {
                service_id: 0x01,
                start_pid_id: 0x00,
                bitmap: [0x88, 0x02, 0x80, 0x00],
            })
        );
    }

    #[test]
    fn obd2_supported_pid_bitmap_surface_reports_later_pid_block() {
        assert_eq!(
            CanObd2SupportedPidBitmapSurface::from_pid_block(0x40),
            Some(CanObd2SupportedPidBitmapSurface {
                service_id: 0x01,
                start_pid_id: 0x40,
                bitmap: [0x40, 0x00, 0x00, 0x00],
            })
        );
    }

    #[test]
    fn obd2_supported_pid_bitmap_surface_preserves_catalog_bit_positions() {
        assert_eq!(CanObd2SupportedPidBitmapSurface::from_pid_block(0x20), None);
        assert_eq!(CanObd2SupportedPidBitmapSurface::from_pid_block(0x01), None);

        let first = CanObd2SupportedPidBitmapSurface::from_pid_block(0x00).expect("first block");
        let later = CanObd2SupportedPidBitmapSurface::from_pid_block(0x40).expect("later block");

        let first_bits = u32::from_be_bytes(first.bitmap);
        let later_bits = u32::from_be_bytes(later.bitmap);

        assert_ne!(first_bits & (1 << 31), 0);
        assert_ne!(first_bits & (1 << (31 - 4)), 0);
        assert_ne!(first_bits & (1 << (31 - 14)), 0);
        assert_ne!(first_bits & (1 << (31 - 16)), 0);
        assert_ne!(later_bits & (1 << (31 - 1)), 0);
    }

    #[test]
    fn obd2_supported_info_type_bitmap_surface_reports_ecu_name() {
        assert_eq!(
            CanObd2SupportedInfoTypeBitmapSurface::from_info_type_block(0x00),
            Some(CanObd2SupportedInfoTypeBitmapSurface {
                service_id: 0x09,
                start_info_type_id: 0x00,
                bitmap: [0x50, 0x40, 0x00, 0x00],
            })
        );
        assert_eq!(
            CanObd2SupportedInfoTypeBitmapSurface::from_info_type_block(0x20),
            None
        );
        assert_eq!(
            CanObd2SupportedInfoTypeBitmapSurface::from_info_type_block(0x01),
            None
        );
    }

    #[test]
    fn obd2_supported_pid_discovery_response_surface_assembles_first_block_response() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2SupportedPidDiscoveryResponseSurface::assemble(&request),
            Ok(CanObd2SupportedPidDiscoveryResponseSurface {
                request_service_id: 0x01,
                start_pid_id: 0x00,
                positive_response_service_id: 0x41,
                bitmap: [0x88, 0x02, 0x80, 0x00],
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x00),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x88, 0x02, 0x80, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_supported_pid_discovery_response_surface_assembles_later_block_response() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x40),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2SupportedPidDiscoveryResponseSurface::assemble(&request),
            Ok(CanObd2SupportedPidDiscoveryResponseSurface {
                request_service_id: 0x01,
                start_pid_id: 0x40,
                positive_response_service_id: 0x41,
                bitmap: [0x40, 0x00, 0x00, 0x00],
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x40),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x40, 0x00, 0x00, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_supported_pid_discovery_response_surface_rejects_unsupported_requests() {
        let unsupported_service = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };
        let unsupported_block = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x20),
            payload_len: 0,
            payload: [0; 6],
        };
        let missing_pid = Message::Obd2Request {
            service: 0x01,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2SupportedPidDiscoveryResponseSurface::assemble(&unsupported_service),
            Err(CanObd2SupportedPidDiscoveryResponseError::UnsupportedService { service_id: 0x09 })
        );
        assert_eq!(
            CanObd2SupportedPidDiscoveryResponseSurface::assemble(&unsupported_block),
            Err(
                CanObd2SupportedPidDiscoveryResponseError::UnsupportedPidBlock {
                    start_pid_id: 0x20
                }
            )
        );
        assert_eq!(
            CanObd2SupportedPidDiscoveryResponseSurface::assemble(&missing_pid),
            Err(CanObd2SupportedPidDiscoveryResponseError::MissingParameterId)
        );
        assert_eq!(
            CanObd2SupportedPidDiscoveryResponseSurface::assemble(&Message::CmdReset {
                target_node_id: 1
            }),
            Err(CanObd2SupportedPidDiscoveryResponseError::UnsupportedService { service_id: 0 })
        );
    }

    #[test]
    fn obd2_vehicle_info_response_surface_assembles_supported_info_discovery() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2VehicleInfoResponseSurface::assemble(
                &request,
                CanObd2VehicleInfoInputs::default_identity()
            ),
            Ok(CanObd2VehicleInfoResponseSurface {
                request_service_id: 0x09,
                info_type_id: 0x00,
                positive_response_service_id: 0x49,
                meaning: CanObd2VehicleInfoPayloadMeaning::SupportedInfoTypes {
                    start_info_type_id: 0x00,
                    bitmap: [0x50, 0x40, 0x00, 0x00],
                },
                response: Message::Obd2Response {
                    service: 0x49,
                    parameter_id: Some(0x00),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x50, 0x40, 0x00, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_vehicle_info_response_surface_advertises_flash_write_fault_when_present() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0xE0),
            payload_len: 0,
            payload: [0; 6],
        };
        let mut inputs = CanObd2VehicleInfoInputs::default_identity();
        inputs.flash_write_fault = Some(CanObd2FlashWriteFaultStatus::absent());

        assert_eq!(
            CanObd2VehicleInfoResponseSurface::assemble(&request, inputs),
            Ok(CanObd2VehicleInfoResponseSurface {
                request_service_id: 0x09,
                info_type_id: 0xE0,
                positive_response_service_id: 0x49,
                meaning: CanObd2VehicleInfoPayloadMeaning::SupportedInfoTypes {
                    start_info_type_id: 0xE0,
                    bitmap: [0xC0, 0x00, 0x00, 0x00],
                },
                response: Message::Obd2Response {
                    service: 0x49,
                    parameter_id: Some(0xE0),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0xC0, 0x00, 0x00, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_vehicle_info_response_surface_assembles_bounded_ecu_name() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x0A),
            payload_len: 0,
            payload: [0; 6],
        };
        let inputs = CanObd2VehicleInfoInputs {
            ecu_name_len: 4,
            ecu_name: [b'E', b'C', b'U', b'1', 0, 0],
            vin_len: CAN_OBD2_VIN_LEN as u8,
            vin: *b"TESTVIN0000000000",
            calibration_id_len: CAN_OBD2_VIN_LEN as u8,
            calibration_id: *b"CALTEST0000000000",
            identity_key_lifecycle: None,
            flash_write_fault: None,
        };

        assert_eq!(
            CanObd2VehicleInfoResponseSurface::assemble(&request, inputs),
            Ok(CanObd2VehicleInfoResponseSurface {
                request_service_id: 0x09,
                info_type_id: 0x0A,
                positive_response_service_id: 0x49,
                meaning: CanObd2VehicleInfoPayloadMeaning::EcuName {
                    bytes: [b'E', b'C', b'U', b'1', 0, 0],
                    len: 4,
                },
                response: Message::Obd2Response {
                    service: 0x49,
                    parameter_id: Some(0x0A),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [b'E', b'C', b'U', b'1', 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_segmented_vehicle_info_response_surface_assembles_vin_segments() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        };
        let inputs = CanObd2VehicleInfoInputs {
            ecu_name_len: 6,
            ecu_name: *b"PIPOCO",
            vin_len: CAN_OBD2_VIN_LEN as u8,
            vin: *b"PIPOCO12345678901",
            calibration_id_len: CAN_OBD2_VIN_LEN as u8,
            calibration_id: *b"CALPIPOCO12345678",
            identity_key_lifecycle: None,
            flash_write_fault: None,
        };

        assert_eq!(
            CanObd2SegmentedVehicleInfoResponseSurface::assemble(&request, inputs),
            Ok(CanObd2SegmentedVehicleInfoResponseSurface {
                request_service_id: 0x09,
                info_type_id: 0x02,
                positive_response_service_id: 0x49,
                meaning: CanObd2SegmentedVehicleInfoMeaning::Vin {
                    bytes: *b"PIPOCO12345678901",
                    len: 17,
                },
                total_payload_len: 17,
                segment_count: 3,
                segments: [
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(0x02),
                        sequence_index: 0,
                        segment_count: 3,
                        total_payload_len: 17,
                        segment_len: 6,
                        segment: *b"PIPOCO",
                    }),
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(0x02),
                        sequence_index: 1,
                        segment_count: 3,
                        total_payload_len: 17,
                        segment_len: 6,
                        segment: *b"123456",
                    }),
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(0x02),
                        sequence_index: 2,
                        segment_count: 3,
                        total_payload_len: 17,
                        segment_len: 5,
                        segment: [b'7', b'8', b'9', b'0', b'1', 0],
                    }),
                    None,
                ],
            })
        );
    }

    #[test]
    fn obd2_segmented_vehicle_info_response_surface_assembles_identity_key_lifecycle_segments() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID),
            payload_len: 0,
            payload: [0; 6],
        };
        let status = CanObd2IdentityKeyLifecycleStatus {
            present: true,
            persisted: true,
            request_id: 0x1020_3040,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason: 3,
            generation: 0x5060_7080,
        };
        let mut inputs = CanObd2VehicleInfoInputs::default_identity();
        inputs.identity_key_lifecycle = Some(status);

        assert_eq!(
            CanObd2SegmentedVehicleInfoResponseSurface::assemble(&request, inputs),
            Ok(CanObd2SegmentedVehicleInfoResponseSurface {
                request_service_id: 0x09,
                info_type_id: CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID,
                positive_response_service_id: 0x49,
                meaning: CanObd2SegmentedVehicleInfoMeaning::IdentityKeyLifecycle {
                    status,
                    payload: [0x10, 0x20, 0x30, 0x40, 0xC5, 0x03, 0x50, 0x60, 0x70, 0x80],
                },
                total_payload_len: CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
                segment_count: 2,
                segments: [
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID),
                        sequence_index: 0,
                        segment_count: 2,
                        total_payload_len: CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
                        segment_len: 6,
                        segment: [0x10, 0x20, 0x30, 0x40, 0xC5, 0x03],
                    }),
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID),
                        sequence_index: 1,
                        segment_count: 2,
                        total_payload_len: CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN,
                        segment_len: 4,
                        segment: [0x50, 0x60, 0x70, 0x80, 0, 0],
                    }),
                    None,
                    None,
                ],
            })
        );
    }

    #[test]
    fn obd2_segmented_vehicle_info_response_surface_assembles_flash_write_fault_segment() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID),
            payload_len: 0,
            payload: [0; 6],
        };
        let status = CanObd2FlashWriteFaultStatus {
            present: true,
            phase: CanObd2FlashWriteFaultPhase::Program,
            sr_bits: 0x0000_00F2,
        };
        let mut inputs = CanObd2VehicleInfoInputs::default_identity();
        inputs.flash_write_fault = Some(status);

        assert_eq!(
            CanObd2SegmentedVehicleInfoResponseSurface::assemble(&request, inputs),
            Ok(CanObd2SegmentedVehicleInfoResponseSurface {
                request_service_id: 0x09,
                info_type_id: CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID,
                positive_response_service_id: 0x49,
                meaning: CanObd2SegmentedVehicleInfoMeaning::FlashWriteFault {
                    status,
                    payload: [0x80, 0x03, 0x00, 0x00, 0x00, 0xF2],
                },
                total_payload_len: CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN,
                segment_count: 1,
                segments: [
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID),
                        sequence_index: 0,
                        segment_count: 1,
                        total_payload_len: CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN,
                        segment_len: 6,
                        segment: [0x80, 0x03, 0x00, 0x00, 0x00, 0xF2],
                    }),
                    None,
                    None,
                    None,
                ],
            })
        );
    }

    #[test]
    fn obd2_vehicle_info_response_surface_rejects_unsupported_requests() {
        let unsupported_info = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        };
        let missing_info = Message::Obd2Request {
            service: 0x09,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2VehicleInfoResponseSurface::assemble(
                &unsupported_info,
                CanObd2VehicleInfoInputs::default_identity()
            ),
            Err(CanObd2VehicleInfoResponseError::UnsupportedInfoType { info_type_id: 0x02 })
        );
        assert_eq!(
            CanObd2VehicleInfoResponseSurface::assemble(
                &missing_info,
                CanObd2VehicleInfoInputs::default_identity()
            ),
            Err(CanObd2VehicleInfoResponseError::MissingInfoTypeId)
        );
        assert_eq!(
            CanObd2VehicleInfoResponseSurface::assemble(
                &Message::CmdReset { target_node_id: 1 },
                CanObd2VehicleInfoInputs::default_identity()
            ),
            Err(CanObd2VehicleInfoResponseError::NotObd2Request)
        );
    }

    #[test]
    fn obd2_negative_response_surface_maps_unsupported_service() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2NegativeResponseSurface::from_current_data_error(
                &request,
                CanObd2ResponseAssemblyError::UnsupportedService { service_id: 0x09 }
            ),
            Ok(CanObd2NegativeResponseSurface {
                request_service_id: 0x09,
                code: CanObd2NegativeResponseCode::ServiceNotSupported,
                response: Message::Obd2Response {
                    service: 0x7F,
                    parameter_id: Some(0x09),
                    negative_response_code: Some(0x11),
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
    }

    #[test]
    fn obd2_negative_response_surface_maps_unsupported_pid() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2NegativeResponseSurface::from_current_data_error(
                &request,
                CanObd2ResponseAssemblyError::UnsupportedPid { pid_id: 0x0C }
            ),
            Ok(CanObd2NegativeResponseSurface {
                request_service_id: 0x01,
                code: CanObd2NegativeResponseCode::RequestOutOfRange,
                response: Message::Obd2Response {
                    service: 0x7F,
                    parameter_id: Some(0x01),
                    negative_response_code: Some(0x31),
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
    }

    #[test]
    fn obd2_negative_response_surface_preserves_stable_negative_service_identifier() {
        let current_data_request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };
        let discovery_request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x20),
            payload_len: 0,
            payload: [0; 6],
        };

        let current_data = CanObd2NegativeResponseSurface::from_current_data_error(
            &current_data_request,
            CanObd2ResponseAssemblyError::UnsupportedPid { pid_id: 0x0C },
        )
        .expect("current-data negative response");
        let discovery = CanObd2NegativeResponseSurface::from_discovery_error(
            &discovery_request,
            CanObd2SupportedPidDiscoveryResponseError::UnsupportedPidBlock { start_pid_id: 0x20 },
        )
        .expect("discovery negative response");

        assert_eq!(
            current_data.response,
            Message::Obd2Response {
                service: 0x7F,
                parameter_id: Some(0x01),
                negative_response_code: Some(0x31),
                payload_len: 0,
                payload: [0; 6],
            }
        );
        assert_eq!(
            discovery.response,
            Message::Obd2Response {
                service: 0x7F,
                parameter_id: Some(0x01),
                negative_response_code: Some(0x31),
                payload_len: 0,
                payload: [0; 6],
            }
        );
    }

    #[test]
    fn obd2_readiness_monitor_surface_assembles_bounded_pid_01_response() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        };
        let inputs = CanObd2ReadinessMonitorInputs {
            mil_requested: true,
            stored_dtc_count: 3,
            misfire_supported: true,
            misfire_complete: false,
            fuel_system_supported: true,
            fuel_system_complete: true,
            comprehensive_components_supported: true,
            comprehensive_components_complete: false,
        };

        assert_eq!(
            CanObd2ReadinessMonitorSurface::assemble(&request, inputs),
            Ok(CanObd2ReadinessMonitorSurface {
                request_service_id: 0x01,
                parameter_id: 0x01,
                meaning: inputs.into_meaning(),
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x01),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x83, 0xE5, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_dtc_clear_surface_assembles_bounded_service_04_response() {
        let request = Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };
        let inputs = CanObd2DtcClearInputs {
            cleared_dtc_count: 2,
            freeze_frame_cleared: true,
            readiness_reset: true,
        };

        assert_eq!(
            CanObd2DtcClearSurface::assemble(&request, inputs),
            Ok(CanObd2DtcClearSurface {
                request_service_id: 0x04,
                verdict: CanObd2DtcClearVerdict {
                    cleared_dtc_count: 2,
                    freeze_frame_cleared: true,
                    readiness_reset: true,
                },
                response: Message::Obd2Response {
                    service: 0x44,
                    parameter_id: None,
                    negative_response_code: None,
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
    }

    #[test]
    fn obd2_dtc_clear_surface_rejects_unsupported_or_parameterized_requests() {
        let wrong_service = Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };
        let parameterized = Message::Obd2Request {
            service: 0x04,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        };
        let inputs = CanObd2DtcClearInputs {
            cleared_dtc_count: 0,
            freeze_frame_cleared: false,
            readiness_reset: true,
        };

        assert_eq!(
            CanObd2DtcClearSurface::assemble(&wrong_service, inputs),
            Err(CanObd2DtcClearResponseError::UnsupportedService { service_id: 0x03 })
        );
        assert_eq!(
            CanObd2DtcClearSurface::assemble(&parameterized, inputs),
            Err(CanObd2DtcClearResponseError::UnexpectedParameterId { parameter_id: 0x01 })
        );
    }

    #[test]
    fn obd2_request_dispatch_surface_routes_supported_current_data_request() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x11),
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch(&request, Some(&sensor)),
            Ok(CanObd2RequestDispatchSurface {
                request_service_id: 0x01,
                parameter_id: 0x11,
                verdict: CanObd2RequestDispatchVerdict::CurrentDataPositive,
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x11),
                    negative_response_code: None,
                    payload_len: 1,
                    payload: [26, 0, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_request_dispatch_surface_routes_supported_discovery_request() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch(&request, None),
            Ok(CanObd2RequestDispatchSurface {
                request_service_id: 0x01,
                parameter_id: 0x00,
                verdict: CanObd2RequestDispatchVerdict::SupportedPidDiscoveryPositive,
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x00),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x88, 0x02, 0x80, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_request_dispatch_surface_routes_readiness_monitor_request() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        };
        let readiness = CanObd2ReadinessMonitorInputs {
            mil_requested: false,
            stored_dtc_count: 2,
            misfire_supported: true,
            misfire_complete: true,
            fuel_system_supported: true,
            fuel_system_complete: false,
            comprehensive_components_supported: true,
            comprehensive_components_complete: true,
        };

        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch_with_readiness(&request, None, Some(readiness)),
            Ok(CanObd2RequestDispatchSurface {
                request_service_id: 0x01,
                parameter_id: 0x01,
                verdict: CanObd2RequestDispatchVerdict::ReadinessMonitorPositive,
                response: Message::Obd2Response {
                    service: 0x41,
                    parameter_id: Some(0x01),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x02, 0xE2, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_request_dispatch_surface_routes_unsupported_requests_to_negative_responses() {
        let unsupported_service = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        };
        let unsupported_pid = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x0C),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch(&unsupported_service, None),
            Ok(CanObd2RequestDispatchSurface {
                request_service_id: 0x09,
                parameter_id: 0x02,
                verdict: CanObd2RequestDispatchVerdict::NegativeResponse(
                    CanObd2NegativeResponseCode::ServiceNotSupported,
                ),
                response: Message::Obd2Response {
                    service: 0x7F,
                    parameter_id: Some(0x09),
                    negative_response_code: Some(0x11),
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch(&unsupported_pid, None),
            Ok(CanObd2RequestDispatchSurface {
                request_service_id: 0x01,
                parameter_id: 0x0C,
                verdict: CanObd2RequestDispatchVerdict::NegativeResponse(
                    CanObd2NegativeResponseCode::RequestOutOfRange,
                ),
                response: Message::Obd2Response {
                    service: 0x7F,
                    parameter_id: Some(0x01),
                    negative_response_code: Some(0x31),
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
    }

    #[test]
    fn obd2_request_dispatch_surface_requires_compatible_value_source_for_supported_pid() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x42),
            payload_len: 0,
            payload: [0; 6],
        };
        let heartbeat = Message::Heartbeat {
            node_id: 3,
            uptime_seconds: 42,
            status: 0,
            error_count: 0,
            cpu_usage: 5,
        };

        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch(&request, None),
            Err(CanObd2RequestDispatchError::MissingValueSource {
                backing_message_class: CanMessageClass::SensorData,
            })
        );
        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch(&request, Some(&heartbeat)),
            Err(CanObd2RequestDispatchError::IncompatibleValueSource {
                backing_message_class: CanMessageClass::SensorData,
            })
        );
    }

    #[test]
    fn obd2_request_dispatch_surface_requires_readiness_inputs_for_pid_01() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2RequestDispatchSurface::dispatch_with_readiness(&request, None, None),
            Err(CanObd2RequestDispatchError::MissingReadinessInputs)
        );
    }

    #[test]
    fn obd2_dtc_freeze_frame_surface_assembles_stored_dtc_response() {
        let request = Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2DtcFreezeFrameResponseSurface::assemble_stored_dtcs(
                &request,
                &[
                    DiagCode::MapRange,
                    DiagCode::CamMissing,
                    DiagCode::PersistCrcFault
                ]
            ),
            Ok(CanObd2DtcFreezeFrameResponseSurface {
                request_service_id: 0x03,
                parameter_id: None,
                meaning: CanObd2DtcFreezeFramePayloadMeaning::StoredDtcs {
                    dtc_count: 3,
                    dtcs: [
                        Some(CanObd2StoredDtc {
                            diag_code: DiagCode::MapRange,
                            raw_bytes: [0x01, 0x08],
                        }),
                        Some(CanObd2StoredDtc {
                            diag_code: DiagCode::CamMissing,
                            raw_bytes: [0x03, 0x40],
                        }),
                        Some(CanObd2StoredDtc {
                            diag_code: DiagCode::PersistCrcFault,
                            raw_bytes: [0x06, 0x01],
                        }),
                    ],
                },
                response: Message::Obd2Response {
                    service: 0x43,
                    parameter_id: None,
                    negative_response_code: None,
                    payload_len: 6,
                    payload: [0x01, 0x08, 0x03, 0x40, 0x06, 0x01],
                },
            })
        );
    }

    #[test]
    fn obd2_stored_dtc_encodes_pressure_and_lambda_diag_codes() {
        assert_eq!(
            CanObd2StoredDtc::from_diag_code(DiagCode::OilPressureLow),
            CanObd2StoredDtc {
                diag_code: DiagCode::OilPressureLow,
                raw_bytes: [0x05, 0x20],
            }
        );
        assert_eq!(
            CanObd2StoredDtc::from_diag_code(DiagCode::FuelPressureLow),
            CanObd2StoredDtc {
                diag_code: DiagCode::FuelPressureLow,
                raw_bytes: [0x01, 0x92],
            }
        );
        assert_eq!(
            CanObd2StoredDtc::from_diag_code(DiagCode::LambdaInvalid),
            CanObd2StoredDtc {
                diag_code: DiagCode::LambdaInvalid,
                raw_bytes: [0x01, 0x30],
            }
        );
    }

    #[test]
    fn obd2_dtc_freeze_frame_surface_assembles_bounded_freeze_frame_response() {
        let request = Message::Obd2Request {
            service: 0x02,
            parameter_id: Some(0x42),
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2DtcFreezeFrameResponseSurface::assemble_freeze_frame(
                &request,
                DiagCode::LowVoltage,
                Some(&sensor)
            ),
            Ok(CanObd2DtcFreezeFrameResponseSurface {
                request_service_id: 0x02,
                parameter_id: Some(0x42),
                meaning: CanObd2DtcFreezeFramePayloadMeaning::FreezeFrame(
                    CanObd2FreezeFrameSnapshot {
                        dtc: CanObd2StoredDtc {
                            diag_code: DiagCode::LowVoltage,
                            raw_bytes: [0x05, 0x62],
                        },
                        pid: CanObd2Pid::ControlModuleVoltage,
                        payload: CanObd2ProjectedPayload {
                            payload_len: 2,
                            bytes: [0x2E, 0xE0],
                        },
                    }
                ),
                response: Message::Obd2Response {
                    service: 0x42,
                    parameter_id: Some(0x42),
                    negative_response_code: None,
                    payload_len: 2,
                    payload: [0x2E, 0xE0, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_dtc_freeze_frame_surface_rejects_unsupported_diagnostic_requests() {
        let wrong_service = Message::Obd2Request {
            service: 0x09,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };
        let unsupported_pid = Message::Obd2Request {
            service: 0x02,
            parameter_id: Some(0x20),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2DtcFreezeFrameResponseSurface::assemble_stored_dtcs(
                &wrong_service,
                &[DiagCode::MapRange]
            ),
            Err(CanObd2DtcFreezeFrameResponseError::UnsupportedService { service_id: 0x09 })
        );
        assert_eq!(
            CanObd2DtcFreezeFrameResponseSurface::assemble_freeze_frame(
                &unsupported_pid,
                DiagCode::MapRange,
                None
            ),
            Err(CanObd2DtcFreezeFrameResponseError::UnsupportedPid { pid_id: 0x20 })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_current_data_requests() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x11),
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    current_data_value_source: Some(&sensor),
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x01,
                parameter_id: Some(0x11),
                verdict: CanObd2MultiServiceDispatchVerdict::CurrentDataPositive,
                response: CanObd2ResponseFrame {
                    service: 0x41,
                    parameter_id: Some(0x11),
                    negative_response_code: None,
                    payload_len: 1,
                    payload: [26, 0, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_readiness_requests() {
        let request = Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        };
        let readiness = CanObd2ReadinessMonitorInputs {
            mil_requested: true,
            stored_dtc_count: 1,
            misfire_supported: true,
            misfire_complete: true,
            fuel_system_supported: true,
            fuel_system_complete: false,
            comprehensive_components_supported: true,
            comprehensive_components_complete: true,
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    readiness_monitor: Some(readiness),
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x01,
                parameter_id: Some(0x01),
                verdict: CanObd2MultiServiceDispatchVerdict::ReadinessMonitorPositive,
                response: CanObd2ResponseFrame {
                    service: 0x41,
                    parameter_id: Some(0x01),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x81, 0xE2, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_dtc_clear_requests() {
        let request = Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };
        let clear = CanObd2DtcClearInputs {
            cleared_dtc_count: 1,
            freeze_frame_cleared: true,
            readiness_reset: true,
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    dtc_clear: Some(clear),
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x04,
                parameter_id: None,
                verdict: CanObd2MultiServiceDispatchVerdict::DtcClearPositive,
                response: CanObd2ResponseFrame {
                    service: 0x44,
                    parameter_id: None,
                    negative_response_code: None,
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_freeze_frame_requests() {
        let request = Message::Obd2Request {
            service: 0x02,
            parameter_id: Some(0x42),
            payload_len: 0,
            payload: [0; 6],
        };
        let sensor = Message::SensorData {
            map_kpa_x10: 1000,
            tps_percent: 10,
            iat_offset: 50,
            clt_offset: 60,
            voltage_x10: 120,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 0,
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    freeze_frame_value_source: Some(&sensor),
                    freeze_frame_dtc: Some(DiagCode::LowVoltage),
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x02,
                parameter_id: Some(0x42),
                verdict: CanObd2MultiServiceDispatchVerdict::FreezeFrame,
                response: CanObd2ResponseFrame {
                    service: 0x42,
                    parameter_id: Some(0x42),
                    negative_response_code: None,
                    payload_len: 2,
                    payload: [0x2E, 0xE0, 0, 0, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_stored_dtc_requests() {
        let request = Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    stored_dtcs: &[DiagCode::MapRange, DiagCode::CamMissing],
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x03,
                parameter_id: None,
                verdict: CanObd2MultiServiceDispatchVerdict::StoredDtcs,
                response: CanObd2ResponseFrame {
                    service: 0x43,
                    parameter_id: None,
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x01, 0x08, 0x03, 0x40, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_vehicle_info_discovery_requests() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs::empty()
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x09,
                parameter_id: Some(0x00),
                verdict: CanObd2MultiServiceDispatchVerdict::VehicleInfoPositive,
                response: CanObd2ResponseFrame {
                    service: 0x49,
                    parameter_id: Some(0x00),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x50, 0x40, 0x00, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_identity_key_lifecycle_discovery_block() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0xE0),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs::empty()
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x09,
                parameter_id: Some(0xE0),
                verdict: CanObd2MultiServiceDispatchVerdict::VehicleInfoPositive,
                response: CanObd2ResponseFrame {
                    service: 0x49,
                    parameter_id: Some(0xE0),
                    negative_response_code: None,
                    payload_len: 4,
                    payload: [0x80, 0x00, 0x00, 0x00, 0, 0],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_routes_vehicle_info_ecu_name_requests() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x0A),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    vehicle_info: CanObd2VehicleInfoInputs {
                        ecu_name_len: 6,
                        ecu_name: *b"PIPOCO",
                        vin_len: CAN_OBD2_VIN_LEN as u8,
                        vin: *b"PIPOCO00000000000",
                        calibration_id_len: CAN_OBD2_VIN_LEN as u8,
                        calibration_id: *b"PIPOCO00000000000",
                        identity_key_lifecycle: None,
                        flash_write_fault: None,
                    },
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x09,
                parameter_id: Some(0x0A),
                verdict: CanObd2MultiServiceDispatchVerdict::VehicleInfoPositive,
                response: CanObd2ResponseFrame {
                    service: 0x49,
                    parameter_id: Some(0x0A),
                    negative_response_code: None,
                    payload_len: 6,
                    payload: [b'P', b'I', b'P', b'O', b'C', b'O'],
                },
            })
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_outcome_routes_vehicle_info_vin_segments() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch_outcome(
                &request,
                CanObd2MultiServiceDispatchInputs {
                    vehicle_info: CanObd2VehicleInfoInputs {
                        ecu_name_len: 6,
                        ecu_name: *b"PIPOCO",
                        vin_len: CAN_OBD2_VIN_LEN as u8,
                        vin: *b"PIPOCO12345678901",
                        calibration_id_len: CAN_OBD2_VIN_LEN as u8,
                        calibration_id: *b"CALPIPOCO12345678",
                        identity_key_lifecycle: None,
                        flash_write_fault: None,
                    },
                    ..CanObd2MultiServiceDispatchInputs::empty()
                }
            ),
            Ok(CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo(
                CanObd2SegmentedVehicleInfoResponseSurface {
                    request_service_id: 0x09,
                    info_type_id: 0x02,
                    positive_response_service_id: 0x49,
                    meaning: CanObd2SegmentedVehicleInfoMeaning::Vin {
                        bytes: *b"PIPOCO12345678901",
                        len: 17,
                    },
                    total_payload_len: 17,
                    segment_count: 3,
                    segments: [
                        Some(CanObd2SegmentedResponseFrame {
                            service: 0x49,
                            parameter_id: Some(0x02),
                            sequence_index: 0,
                            segment_count: 3,
                            total_payload_len: 17,
                            segment_len: 6,
                            segment: *b"PIPOCO",
                        }),
                        Some(CanObd2SegmentedResponseFrame {
                            service: 0x49,
                            parameter_id: Some(0x02),
                            sequence_index: 1,
                            segment_count: 3,
                            total_payload_len: 17,
                            segment_len: 6,
                            segment: *b"123456",
                        }),
                        Some(CanObd2SegmentedResponseFrame {
                            service: 0x49,
                            parameter_id: Some(0x02),
                            sequence_index: 2,
                            segment_count: 3,
                            total_payload_len: 17,
                            segment_len: 5,
                            segment: [b'7', b'8', b'9', b'0', b'1', 0],
                        }),
                        None,
                    ],
                }
            ))
        );
    }

    #[test]
    fn obd2_multi_service_dispatch_outcome_routes_calibration_id_segments() {
        let request = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x04),
            payload_len: 0,
            payload: [0; 6],
        };

        let outcome = CanObd2MultiServiceDispatchSurface::dispatch_outcome(
            &request,
            CanObd2MultiServiceDispatchInputs {
                vehicle_info: CanObd2VehicleInfoInputs {
                    ecu_name_len: 6,
                    ecu_name: *b"PIPOCO",
                    vin_len: CAN_OBD2_VIN_LEN as u8,
                    vin: *b"PIPOCO12345678901",
                    calibration_id_len: CAN_OBD2_VIN_LEN as u8,
                    calibration_id: *b"CALPIPOCO12345678",
                    identity_key_lifecycle: None,
                    flash_write_fault: None,
                },
                ..CanObd2MultiServiceDispatchInputs::empty()
            },
        )
        .expect("calibration id dispatch");

        match outcome {
            CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo(dispatch) => {
                assert_eq!(dispatch.info_type_id, 0x04);
                assert_eq!(
                    dispatch.meaning,
                    CanObd2SegmentedVehicleInfoMeaning::CalibrationId {
                        bytes: *b"CALPIPOCO12345678",
                        len: 17,
                    }
                );
                assert_eq!(
                    dispatch.segments[0],
                    Some(CanObd2SegmentedResponseFrame {
                        service: 0x49,
                        parameter_id: Some(0x04),
                        sequence_index: 0,
                        segment_count: 3,
                        total_payload_len: 17,
                        segment_len: 6,
                        segment: *b"CALPIP",
                    })
                );
                assert_eq!(
                    dispatch.segments[0].expect("first segment").to_message(),
                    Message::Obd2SegmentedResponse {
                        service: 0x49,
                        parameter_id: Some(0x04),
                        sequence_index: 0,
                        segment_count: 3,
                        total_payload_len: 17,
                        segment_len: 6,
                        segment: *b"CALPIP",
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }
    }

    #[test]
    fn obd2_multi_service_dispatch_surface_rejects_unsupported_services() {
        let unsupported_service = Message::Obd2Request {
            service: 0x08,
            parameter_id: Some(0x00),
            payload_len: 0,
            payload: [0; 6],
        };
        let unsupported_info = Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        };

        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &unsupported_service,
                CanObd2MultiServiceDispatchInputs::empty()
            ),
            Err(CanObd2MultiServiceDispatchError::UnsupportedService { service_id: 0x08 })
        );
        assert_eq!(
            CanObd2MultiServiceDispatchSurface::dispatch(
                &unsupported_info,
                CanObd2MultiServiceDispatchInputs::empty()
            ),
            Ok(CanObd2MultiServiceDispatchSurface {
                request_service_id: 0x09,
                parameter_id: Some(0x02),
                verdict: CanObd2MultiServiceDispatchVerdict::NegativeResponse(
                    CanObd2NegativeResponseCode::RequestOutOfRange,
                ),
                response: CanObd2ResponseFrame {
                    service: 0x7F,
                    parameter_id: Some(0x09),
                    negative_response_code: Some(0x31),
                    payload_len: 0,
                    payload: [0; 6],
                },
            })
        );
    }

    #[test]
    fn heartbeat_contract_maps_healthy_module() {
        let heartbeat = Message::Heartbeat {
            node_id: 3,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
            status: 0,
            error_count: 0,
            cpu_usage: 41,
        };

        assert_eq!(
            CanModuleHeartbeatContract::from_message(&heartbeat, false),
            Some(CanModuleHeartbeatContract {
                node_id: 3,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 41,
                state: CanModuleHeartbeatState::Alive,
            })
        );
    }

    #[test]
    fn heartbeat_contract_maps_degraded_module() {
        let warning = Message::Heartbeat {
            node_id: 4,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 20,
            status: 1,
            error_count: 2,
            cpu_usage: 78,
        };
        let unknown = Message::Heartbeat {
            node_id: 5,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 20,
            status: 9,
            error_count: 0,
            cpu_usage: 10,
        };

        assert_eq!(
            CanModuleHeartbeatContract::from_message(&warning, false)
                .expect("warning heartbeat")
                .state,
            CanModuleHeartbeatState::Degraded
        );
        assert_eq!(
            CanModuleHeartbeatContract::from_message(&unknown, false),
            Some(CanModuleHeartbeatContract {
                node_id: 5,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 20,
                status: CanModuleHeartbeatStatus::Unknown(9),
                error_count: 0,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            })
        );
    }

    #[test]
    fn heartbeat_contract_maps_starting_and_unavailable_module() {
        let starting = Message::Heartbeat {
            node_id: 6,
            uptime_seconds: 2,
            status: 0,
            error_count: 0,
            cpu_usage: 20,
        };
        let stale = Message::Heartbeat {
            node_id: 7,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 1,
            status: 0,
            error_count: 0,
            cpu_usage: 15,
        };

        assert_eq!(
            CanModuleHeartbeatContract::from_message(&starting, false)
                .expect("starting heartbeat")
                .state,
            CanModuleHeartbeatState::Starting
        );
        assert_eq!(
            CanModuleHeartbeatContract::from_message(&stale, true)
                .expect("stale heartbeat")
                .state,
            CanModuleHeartbeatState::Unavailable
        );
    }

    #[test]
    fn heartbeat_contract_ignores_non_heartbeat_messages() {
        let reset = Message::CmdReset { target_node_id: 9 };
        assert_eq!(
            CanModuleHeartbeatContract::from_message(&reset, false),
            None
        );
    }

    #[test]
    fn heartbeat_observation_policy_keeps_recent_heartbeat_available() {
        let heartbeat = Message::Heartbeat {
            node_id: 8,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
            status: 0,
            error_count: 0,
            cpu_usage: 22,
        };
        let policy = CanHeartbeatObservationPolicy::default();

        assert_eq!(
            policy.observe_message(&heartbeat, Some(CAN_HEARTBEAT_TIMEOUT_MS)),
            Some(CanModuleHeartbeatContract {
                node_id: 8,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 22,
                state: CanModuleHeartbeatState::Alive,
            })
        );
    }

    #[test]
    fn heartbeat_observation_policy_marks_stale_or_missing_heartbeat_unavailable() {
        let heartbeat = Message::Heartbeat {
            node_id: 9,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
            status: 0,
            error_count: 0,
            cpu_usage: 18,
        };
        let policy = CanHeartbeatObservationPolicy::default();

        assert_eq!(
            policy
                .observe_message(&heartbeat, Some(CAN_HEARTBEAT_TIMEOUT_MS + 1))
                .expect("stale heartbeat")
                .state,
            CanModuleHeartbeatState::Unavailable
        );
        assert_eq!(
            policy
                .observe_message(&heartbeat, None)
                .expect("missing heartbeat")
                .state,
            CanModuleHeartbeatState::Unavailable
        );
    }

    #[test]
    fn heartbeat_observation_policy_preserves_starting_vs_unavailable() {
        let heartbeat = Message::Heartbeat {
            node_id: 10,
            uptime_seconds: 1,
            status: 0,
            error_count: 0,
            cpu_usage: 12,
        };
        let policy = CanHeartbeatObservationPolicy::default();

        assert_eq!(
            policy
                .observe_message(&heartbeat, Some(10))
                .expect("recent startup")
                .state,
            CanModuleHeartbeatState::Starting
        );
        assert_eq!(
            policy
                .observe_message(&heartbeat, Some(CAN_HEARTBEAT_TIMEOUT_MS + 1))
                .expect("stale startup")
                .state,
            CanModuleHeartbeatState::Unavailable
        );
    }

    #[test]
    fn heartbeat_observation_policy_uses_stable_default_timeout() {
        let policy = CanHeartbeatObservationPolicy::default();
        assert_eq!(policy.timeout_ms, CAN_HEARTBEAT_TIMEOUT_MS);
        assert!(!policy.is_stale(Some(CAN_HEARTBEAT_TIMEOUT_MS)));
        assert!(policy.is_stale(Some(CAN_HEARTBEAT_TIMEOUT_MS + 1)));
    }

    #[test]
    fn availability_transition_reports_startup_to_alive() {
        assert_eq!(
            CanModuleAvailabilityTransition::between(
                CanModuleHeartbeatState::Starting,
                CanModuleHeartbeatState::Alive
            ),
            CanModuleAvailabilityTransition::Changed {
                from: CanModuleHeartbeatState::Starting,
                to: CanModuleHeartbeatState::Alive,
            }
        );
    }

    #[test]
    fn availability_transition_reports_alive_to_degraded_and_unavailable() {
        assert_eq!(
            CanModuleAvailabilityTransition::between(
                CanModuleHeartbeatState::Alive,
                CanModuleHeartbeatState::Degraded
            ),
            CanModuleAvailabilityTransition::Changed {
                from: CanModuleHeartbeatState::Alive,
                to: CanModuleHeartbeatState::Degraded,
            }
        );
        assert_eq!(
            CanModuleAvailabilityTransition::between(
                CanModuleHeartbeatState::Degraded,
                CanModuleHeartbeatState::Unavailable
            ),
            CanModuleAvailabilityTransition::Changed {
                from: CanModuleHeartbeatState::Degraded,
                to: CanModuleHeartbeatState::Unavailable,
            }
        );
    }

    #[test]
    fn availability_transition_reports_no_change_when_state_is_stable() {
        assert_eq!(
            CanModuleAvailabilityTransition::between(
                CanModuleHeartbeatState::Alive,
                CanModuleHeartbeatState::Alive
            ),
            CanModuleAvailabilityTransition::NoChange(CanModuleHeartbeatState::Alive)
        );
        assert_eq!(
            CanModuleAvailabilityTransition::between(
                CanModuleHeartbeatState::Unavailable,
                CanModuleHeartbeatState::Unavailable
            ),
            CanModuleAvailabilityTransition::NoChange(CanModuleHeartbeatState::Unavailable)
        );
    }

    #[test]
    fn remote_module_fallback_policy_trusts_alive_modules() {
        let heartbeat = CanModuleHeartbeatContract {
            node_id: 4,
            uptime_seconds: 42,
            status: CanModuleHeartbeatStatus::Ok,
            error_count: 0,
            cpu_usage: 12,
            state: CanModuleHeartbeatState::Alive,
        };

        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_contract(heartbeat),
            CanRemoteModuleFallbackPolicy::RemoteDataTrusted
        );
    }

    #[test]
    fn remote_module_fallback_policy_holds_last_known_good_for_starting_and_degraded() {
        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_state(CanModuleHeartbeatState::Starting),
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood
        );
        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_state(CanModuleHeartbeatState::Degraded),
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood
        );
    }

    #[test]
    fn remote_module_fallback_policy_requires_local_fallback_for_unavailable_modules() {
        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_state(CanModuleHeartbeatState::Unavailable),
            CanRemoteModuleFallbackPolicy::RequireLocalFallback
        );
    }

    #[test]
    fn remote_module_fallback_policy_is_stable_across_no_change_and_changed_transitions() {
        let stable = CanModuleAvailabilityTransition::between(
            CanModuleHeartbeatState::Degraded,
            CanModuleHeartbeatState::Degraded,
        );
        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_transition(stable),
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood
        );

        let recovered = CanModuleAvailabilityTransition::between(
            CanModuleHeartbeatState::Starting,
            CanModuleHeartbeatState::Alive,
        );
        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_transition(recovered),
            CanRemoteModuleFallbackPolicy::RemoteDataTrusted
        );

        let failed = CanModuleAvailabilityTransition::between(
            CanModuleHeartbeatState::Alive,
            CanModuleHeartbeatState::Unavailable,
        );
        assert_eq!(
            CanRemoteModuleFallbackPolicy::from_transition(failed),
            CanRemoteModuleFallbackPolicy::RequireLocalFallback
        );
    }

    #[test]
    fn module_roster_inserts_first_observation_for_new_node() {
        let mut roster = CanModuleRosterRegistry::new();
        let heartbeat = CanModuleHeartbeatContract {
            node_id: 21,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
            status: CanModuleHeartbeatStatus::Ok,
            error_count: 0,
            cpu_usage: 19,
            state: CanModuleHeartbeatState::Alive,
        };

        let update = roster.observe_contract(heartbeat);
        let entry = update.entry().expect("inserted entry");

        assert_eq!(roster.len(), 1);
        assert_eq!(
            update,
            CanModuleRosterUpdate::Inserted(CanModuleRosterEntry {
                heartbeat,
                availability_transition: CanModuleAvailabilityTransition::NoChange(
                    CanModuleHeartbeatState::Alive,
                ),
                fallback_policy: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
            })
        );
        assert_eq!(roster.get(21), Some(entry));
    }

    #[test]
    fn module_roster_updates_existing_node_without_duplicate_slot() {
        let mut roster = CanModuleRosterRegistry::new();
        let alive = CanModuleHeartbeatContract {
            node_id: 22,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 6,
            status: CanModuleHeartbeatStatus::Ok,
            error_count: 0,
            cpu_usage: 11,
            state: CanModuleHeartbeatState::Alive,
        };
        let degraded = CanModuleHeartbeatContract {
            node_id: 22,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
            status: CanModuleHeartbeatStatus::Warning,
            error_count: 3,
            cpu_usage: 55,
            state: CanModuleHeartbeatState::Degraded,
        };

        roster.observe_contract(alive);
        let update = roster.observe_contract(degraded);

        assert_eq!(roster.len(), 1);
        assert_eq!(
            update,
            CanModuleRosterUpdate::Updated(CanModuleRosterEntry {
                heartbeat: degraded,
                availability_transition: CanModuleAvailabilityTransition::Changed {
                    from: CanModuleHeartbeatState::Alive,
                    to: CanModuleHeartbeatState::Degraded,
                },
                fallback_policy: CanRemoteModuleFallbackPolicy::HoldLastKnownGood,
            })
        );
        assert_eq!(
            roster.get(22).expect("updated node").heartbeat.status,
            CanModuleHeartbeatStatus::Warning
        );
    }

    #[test]
    fn module_roster_keeps_multiple_nodes_isolated() {
        let mut roster = CanModuleRosterRegistry::new();
        let alive = CanModuleHeartbeatContract {
            node_id: 23,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 12,
            status: CanModuleHeartbeatStatus::Ok,
            error_count: 0,
            cpu_usage: 20,
            state: CanModuleHeartbeatState::Alive,
        };
        let starting = CanModuleHeartbeatContract {
            node_id: 24,
            uptime_seconds: 1,
            status: CanModuleHeartbeatStatus::Ok,
            error_count: 0,
            cpu_usage: 9,
            state: CanModuleHeartbeatState::Starting,
        };

        roster.observe_contract(alive);
        roster.observe_contract(starting);

        assert_eq!(roster.len(), 2);
        assert_eq!(
            roster.get(23).expect("node 23").heartbeat.state,
            alive.state
        );
        assert_eq!(
            roster.get(24).expect("node 24").fallback_policy,
            CanRemoteModuleFallbackPolicy::HoldLastKnownGood
        );
        assert_eq!(roster.get(25), None);
    }

    #[test]
    fn module_roster_stale_observation_updates_fallback_meaning() {
        let mut roster = CanModuleRosterRegistry::new();
        let heartbeat = Message::Heartbeat {
            node_id: 25,
            uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
            status: 0,
            error_count: 0,
            cpu_usage: 14,
        };
        let policy = CanHeartbeatObservationPolicy::default();

        let fresh = roster
            .observe_message(policy, &heartbeat, Some(CAN_HEARTBEAT_TIMEOUT_MS))
            .expect("fresh heartbeat");
        assert_eq!(
            fresh,
            CanModuleRosterUpdate::Inserted(CanModuleRosterEntry {
                heartbeat: CanModuleHeartbeatContract {
                    node_id: 25,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                    status: CanModuleHeartbeatStatus::Ok,
                    error_count: 0,
                    cpu_usage: 14,
                    state: CanModuleHeartbeatState::Alive,
                },
                availability_transition: CanModuleAvailabilityTransition::NoChange(
                    CanModuleHeartbeatState::Alive,
                ),
                fallback_policy: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
            })
        );

        let stale = roster
            .observe_message(policy, &heartbeat, Some(CAN_HEARTBEAT_TIMEOUT_MS + 1))
            .expect("stale heartbeat");
        assert_eq!(
            stale,
            CanModuleRosterUpdate::Updated(CanModuleRosterEntry {
                heartbeat: CanModuleHeartbeatContract {
                    node_id: 25,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                    status: CanModuleHeartbeatStatus::Ok,
                    error_count: 0,
                    cpu_usage: 14,
                    state: CanModuleHeartbeatState::Unavailable,
                },
                availability_transition: CanModuleAvailabilityTransition::Changed {
                    from: CanModuleHeartbeatState::Alive,
                    to: CanModuleHeartbeatState::Unavailable,
                },
                fallback_policy: CanRemoteModuleFallbackPolicy::RequireLocalFallback,
            })
        );
        assert_eq!(
            roster.get(25).expect("stale node").fallback_policy,
            CanRemoteModuleFallbackPolicy::RequireLocalFallback
        );
    }

    #[test]
    fn module_roster_rejects_new_nodes_after_capacity() {
        let mut roster = CanModuleRosterRegistry::new();
        for node_id in 0..CAN_MODULE_ROSTER_CAPACITY as u8 {
            let heartbeat = CanModuleHeartbeatContract {
                node_id,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 7,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: node_id,
                state: CanModuleHeartbeatState::Alive,
            };
            let update = roster.observe_contract(heartbeat);
            assert!(matches!(update, CanModuleRosterUpdate::Inserted(..)));
        }

        assert_eq!(roster.len(), CAN_MODULE_ROSTER_CAPACITY);
        assert_eq!(
            roster.observe_contract(CanModuleHeartbeatContract {
                node_id: 200,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 1,
                state: CanModuleHeartbeatState::Alive,
            }),
            CanModuleRosterUpdate::RejectedCapacity { node_id: 200 }
        );
        assert_eq!(roster.get(200), None);
    }

    #[test]
    fn module_roster_stale_sweep_keeps_fresh_node_available() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 30,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS.saturating_sub(10),
        );

        assert_eq!(
            roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 5),
            CanModuleRosterSweepSummary {
                stale_count: 0,
                transitioned_count: 0,
            }
        );
        assert_eq!(
            roster.get(30).expect("fresh node").heartbeat.state,
            CanModuleHeartbeatState::Alive
        );
    }

    #[test]
    fn module_roster_stale_sweep_transitions_node_to_unavailable() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 31,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 12,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );

        assert_eq!(
            roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1),
            CanModuleRosterSweepSummary {
                stale_count: 1,
                transitioned_count: 1,
            }
        );
        assert_eq!(
            roster.get(31).expect("stale node"),
            CanModuleRosterEntry {
                heartbeat: CanModuleHeartbeatContract {
                    node_id: 31,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                    status: CanModuleHeartbeatStatus::Ok,
                    error_count: 0,
                    cpu_usage: 12,
                    state: CanModuleHeartbeatState::Unavailable,
                },
                availability_transition: CanModuleAvailabilityTransition::Changed {
                    from: CanModuleHeartbeatState::Alive,
                    to: CanModuleHeartbeatState::Unavailable,
                },
                fallback_policy: CanRemoteModuleFallbackPolicy::RequireLocalFallback,
            }
        );
    }

    #[test]
    fn module_roster_stale_sweep_keeps_unavailable_node_stable() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 32,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Unavailable,
            },
            CAN_HEARTBEAT_TIMEOUT_MS.saturating_add(5),
        );

        assert_eq!(
            roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 5),
            CanModuleRosterSweepSummary {
                stale_count: 1,
                transitioned_count: 0,
            }
        );
        assert_eq!(
            roster.get(32).expect("unavailable node").heartbeat.state,
            CanModuleHeartbeatState::Unavailable
        );
        assert_eq!(
            roster
                .get(32)
                .expect("unavailable node")
                .availability_transition,
            CanModuleAvailabilityTransition::NoChange(CanModuleHeartbeatState::Unavailable)
        );
    }

    #[test]
    fn module_roster_stale_sweep_isolates_one_stale_node_from_other_nodes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 33,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 34,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1),
            CanModuleRosterSweepSummary {
                stale_count: 1,
                transitioned_count: 1,
            }
        );
        assert_eq!(
            roster.get(33).expect("stale node").heartbeat.state,
            CanModuleHeartbeatState::Unavailable
        );
        assert_eq!(
            roster.get(34).expect("fresh node").heartbeat.state,
            CanModuleHeartbeatState::Alive
        );
    }

    #[test]
    fn module_roster_stale_sweep_respects_shared_timeout_policy() {
        let mut roster = CanModuleRosterRegistry::new();
        let policy = CanHeartbeatObservationPolicy { timeout_ms: 250 };
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 35,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            240,
        );

        assert_eq!(
            roster.sweep_stale(policy, 10),
            CanModuleRosterSweepSummary {
                stale_count: 0,
                transitioned_count: 0,
            }
        );
        assert_eq!(
            roster.sweep_stale(policy, 1),
            CanModuleRosterSweepSummary {
                stale_count: 1,
                transitioned_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_snapshot_is_empty_for_empty_registry() {
        let snapshot = CanModuleRosterRegistry::new().snapshot();

        assert_eq!(snapshot.len, 0);
        assert!(snapshot.is_empty());
        assert_eq!(snapshot.entry(0), None);
        assert_eq!(snapshot.get(1), None);
    }

    #[test]
    fn module_roster_snapshot_preserves_one_node() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 40,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 13,
                state: CanModuleHeartbeatState::Alive,
            },
            17,
        );

        let snapshot = roster.snapshot();
        assert_eq!(snapshot.len, 1);
        assert_eq!(
            snapshot.entry(0),
            Some(CanModuleRosterSnapshotEntry {
                module: CanModuleRosterEntry {
                    heartbeat: CanModuleHeartbeatContract {
                        node_id: 40,
                        uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                        status: CanModuleHeartbeatStatus::Ok,
                        error_count: 0,
                        cpu_usage: 13,
                        state: CanModuleHeartbeatState::Alive,
                    },
                    availability_transition: CanModuleAvailabilityTransition::NoChange(
                        CanModuleHeartbeatState::Alive,
                    ),
                    fallback_policy: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
                },
                heartbeat_age_ms: 17,
            })
        );
    }

    #[test]
    fn module_roster_snapshot_preserves_multiple_nodes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 41,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            3,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 42,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Degraded,
            },
            20,
        );

        let snapshot = roster.snapshot();
        assert_eq!(snapshot.len, 2);
        assert_eq!(
            snapshot.get(41).expect("node 41").module.heartbeat.state,
            CanModuleHeartbeatState::Alive
        );
        assert_eq!(
            snapshot.get(42).expect("node 42").module.heartbeat.state,
            CanModuleHeartbeatState::Degraded
        );
        assert_eq!(snapshot.get(43), None);
    }

    #[test]
    fn module_roster_snapshot_preserves_stale_unavailable_state() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 43,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 16,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let snapshot = roster.snapshot();
        assert_eq!(
            snapshot.get(43).expect("stale node"),
            CanModuleRosterSnapshotEntry {
                module: CanModuleRosterEntry {
                    heartbeat: CanModuleHeartbeatContract {
                        node_id: 43,
                        uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                        status: CanModuleHeartbeatStatus::Ok,
                        error_count: 0,
                        cpu_usage: 16,
                        state: CanModuleHeartbeatState::Unavailable,
                    },
                    availability_transition: CanModuleAvailabilityTransition::Changed {
                        from: CanModuleHeartbeatState::Alive,
                        to: CanModuleHeartbeatState::Unavailable,
                    },
                    fallback_policy: CanRemoteModuleFallbackPolicy::RequireLocalFallback,
                },
                heartbeat_age_ms: CAN_HEARTBEAT_TIMEOUT_MS + 1,
            }
        );
    }

    #[test]
    fn module_roster_snapshot_remains_bounded_at_roster_capacity() {
        let mut roster = CanModuleRosterRegistry::new();
        for node_id in 0..CAN_MODULE_ROSTER_CAPACITY as u8 {
            roster.observe_contract_with_age_ms(
                CanModuleHeartbeatContract {
                    node_id,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                    status: CanModuleHeartbeatStatus::Ok,
                    error_count: 0,
                    cpu_usage: node_id,
                    state: CanModuleHeartbeatState::Alive,
                },
                node_id as u32,
            );
        }

        let snapshot = roster.snapshot();
        assert_eq!(snapshot.capacity(), CAN_MODULE_ROSTER_CAPACITY);
        assert_eq!(snapshot.len, CAN_MODULE_ROSTER_CAPACITY);
        assert_eq!(
            snapshot
                .entry(CAN_MODULE_ROSTER_CAPACITY - 1)
                .map(|entry| entry.module.heartbeat.node_id),
            Some((CAN_MODULE_ROSTER_CAPACITY - 1) as u8)
        );
        assert_eq!(snapshot.entry(CAN_MODULE_ROSTER_CAPACITY), None);
    }

    #[test]
    fn module_roster_summary_is_empty_for_empty_registry() {
        assert_eq!(
            CanModuleRosterRegistry::new().summary(),
            CanModuleRosterSummary::default()
        );
    }

    #[test]
    fn module_roster_summary_counts_single_node() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 50,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            9,
        );

        assert_eq!(
            roster.summary(),
            CanModuleRosterSummary {
                total_count: 1,
                alive_count: 1,
                starting_count: 0,
                degraded_count: 0,
                unavailable_count: 0,
                trusted_count: 1,
                hold_last_known_good_count: 0,
                require_local_fallback_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_summary_counts_mixed_states() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 51,
                uptime_seconds: 1,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 3,
                state: CanModuleHeartbeatState::Starting,
            },
            2,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 52,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 3,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Degraded,
            },
            12,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 53,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 11,
                state: CanModuleHeartbeatState::Alive,
            },
            1,
        );

        assert_eq!(
            roster.snapshot().summary(),
            CanModuleRosterSummary {
                total_count: 3,
                alive_count: 1,
                starting_count: 1,
                degraded_count: 1,
                unavailable_count: 0,
                trusted_count: 1,
                hold_last_known_good_count: 2,
                require_local_fallback_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_summary_counts_stale_unavailable_nodes_correctly() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 54,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 15,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 55,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 18,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.summary(),
            CanModuleRosterSummary {
                total_count: 2,
                alive_count: 0,
                starting_count: 0,
                degraded_count: 1,
                unavailable_count: 1,
                trusted_count: 0,
                hold_last_known_good_count: 1,
                require_local_fallback_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_summary_remains_bounded_at_roster_capacity() {
        let mut roster = CanModuleRosterRegistry::new();
        for node_id in 0..CAN_MODULE_ROSTER_CAPACITY as u8 {
            roster.observe_contract_with_age_ms(
                CanModuleHeartbeatContract {
                    node_id,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                    status: CanModuleHeartbeatStatus::Ok,
                    error_count: 0,
                    cpu_usage: node_id,
                    state: CanModuleHeartbeatState::Alive,
                },
                node_id as u32,
            );
        }

        assert_eq!(
            roster.summary(),
            CanModuleRosterSummary {
                total_count: CAN_MODULE_ROSTER_CAPACITY,
                alive_count: CAN_MODULE_ROSTER_CAPACITY,
                starting_count: 0,
                degraded_count: 0,
                unavailable_count: 0,
                trusted_count: CAN_MODULE_ROSTER_CAPACITY,
                hold_last_known_good_count: 0,
                require_local_fallback_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_startup_readiness_is_empty_for_empty_registry() {
        assert_eq!(
            CanModuleRosterRegistry::new().startup_readiness(),
            CanModuleRosterStartupReadiness::default()
        );
    }

    #[test]
    fn module_roster_startup_readiness_reports_fully_ready_fleet() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 56,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 57,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.startup_readiness(),
            CanModuleRosterStartupReadiness {
                state: CanModuleRosterStartupReadinessState::ReadyForTrust,
                total_module_count: 2,
                ready_for_trust_module_count: 2,
                waiting_on_startup_module_count: 0,
                degraded_hold_last_known_good_module_count: 0,
                blocked_by_unavailable_module_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_startup_readiness_waits_on_starting_modules() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 58,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 59,
                uptime_seconds: 1,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 3,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );

        assert_eq!(
            roster.summary().startup_readiness(),
            CanModuleRosterStartupReadiness {
                state: CanModuleRosterStartupReadinessState::WaitingForStartup,
                total_module_count: 2,
                ready_for_trust_module_count: 1,
                waiting_on_startup_module_count: 1,
                degraded_hold_last_known_good_module_count: 0,
                blocked_by_unavailable_module_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_startup_readiness_blocks_on_unavailable_modules() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 60,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 61,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 11,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.startup_readiness(),
            CanModuleRosterStartupReadiness {
                state: CanModuleRosterStartupReadinessState::BlockedByUnavailable,
                total_module_count: 2,
                ready_for_trust_module_count: 1,
                waiting_on_startup_module_count: 0,
                degraded_hold_last_known_good_module_count: 0,
                blocked_by_unavailable_module_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_startup_readiness_preserves_mixed_fleet_counts() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 62,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 63,
                uptime_seconds: 1,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 64,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.snapshot().startup_readiness(),
            CanModuleRosterStartupReadiness {
                state: CanModuleRosterStartupReadinessState::HoldingLastKnownGood,
                total_module_count: 3,
                ready_for_trust_module_count: 1,
                waiting_on_startup_module_count: 1,
                degraded_hold_last_known_good_module_count: 1,
                blocked_by_unavailable_module_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_shutdown_coordination_is_empty_for_empty_registry() {
        assert_eq!(
            CanModuleRosterRegistry::new().shutdown_coordination(),
            CanModuleRosterShutdownCoordination::default()
        );
    }

    #[test]
    fn module_roster_shutdown_coordination_stays_clear_for_nominal_modules() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 65,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 66,
                uptime_seconds: 1,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );

        assert_eq!(
            roster.summary().shutdown_coordination(),
            CanModuleRosterShutdownCoordination {
                state: CanModuleRosterShutdownCoordinationState::NoShutdownNeeded,
                total_module_count: 2,
                degraded_module_count: 0,
                unavailable_module_count: 0,
                require_local_fallback_module_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_shutdown_coordination_recommends_shutdown_for_degraded_modules() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 67,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.shutdown_coordination(),
            CanModuleRosterShutdownCoordination {
                state: CanModuleRosterShutdownCoordinationState::CoordinatedShutdownRecommended,
                total_module_count: 1,
                degraded_module_count: 1,
                unavailable_module_count: 0,
                require_local_fallback_module_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_shutdown_coordination_requires_local_takeover_for_unavailable_modules() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 68,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.shutdown_coordination(),
            CanModuleRosterShutdownCoordination {
                state: CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired,
                total_module_count: 1,
                degraded_module_count: 0,
                unavailable_module_count: 1,
                require_local_fallback_module_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_shutdown_coordination_preserves_mixed_fleet_counts() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 69,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 70,
                uptime_seconds: 1,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 71,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 72,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.snapshot().shutdown_coordination(),
            CanModuleRosterShutdownCoordination {
                state: CanModuleRosterShutdownCoordinationState::LocalTakeoverRequired,
                total_module_count: 4,
                degraded_module_count: 1,
                unavailable_module_count: 1,
                require_local_fallback_module_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_change_digest_is_empty_when_nothing_changed() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 60,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            3,
        );
        let before = roster.snapshot();

        assert_eq!(
            roster.change_digest_since(before),
            CanModuleRosterChangeDigest::default()
        );
    }

    #[test]
    fn module_roster_change_digest_counts_single_recovery() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 61,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Degraded,
            },
            4,
        );
        let before = roster.snapshot();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 61,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 12,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            1,
        );

        assert_eq!(
            roster.change_digest_since(before),
            CanModuleRosterChangeDigest {
                changed_module_count: 1,
                became_alive_count: 1,
                became_degraded_count: 0,
                became_unavailable_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_change_digest_counts_single_degrade() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 62,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Alive,
            },
            4,
        );
        let before = roster.snapshot();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 62,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 12,
                state: CanModuleHeartbeatState::Degraded,
            },
            2,
        );

        assert_eq!(
            roster.snapshot().change_digest_since(before),
            CanModuleRosterChangeDigest {
                changed_module_count: 1,
                became_alive_count: 0,
                became_degraded_count: 1,
                became_unavailable_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_change_digest_counts_sweep_driven_unavailable_transition() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 63,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let before = roster.snapshot();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.change_digest_since(before),
            CanModuleRosterChangeDigest {
                changed_module_count: 1,
                became_alive_count: 0,
                became_degraded_count: 0,
                became_unavailable_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_change_digest_counts_mixed_multi_node_changes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 64,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Degraded,
            },
            1,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 65,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let before = roster.snapshot();

        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 64,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.change_digest_since(before),
            CanModuleRosterChangeDigest {
                changed_module_count: 2,
                became_alive_count: 1,
                became_degraded_count: 0,
                became_unavailable_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_change_set_is_empty_when_nothing_changed() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 70,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            3,
        );
        let before = roster.snapshot();

        let change_set = roster.change_set_since(before);
        assert!(change_set.is_empty());
        assert_eq!(change_set.entry(0), None);
    }

    #[test]
    fn module_roster_change_set_captures_single_recovery_change() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 71,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Degraded,
            },
            4,
        );
        let before = roster.snapshot();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 71,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 12,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            1,
        );

        assert_eq!(
            roster.change_set_since(before),
            CanModuleRosterChangeSet {
                len: 1,
                entries: {
                    let mut entries = [None; CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY];
                    entries[0] = Some(CanModuleRosterChangeSetEntry {
                        node_id: 71,
                        previous_state: Some(CanModuleHeartbeatState::Degraded),
                        current_state: Some(CanModuleHeartbeatState::Alive),
                    });
                    entries
                },
            }
        );
    }

    #[test]
    fn module_roster_change_set_captures_single_degrade_change() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 72,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Alive,
            },
            4,
        );
        let before = roster.snapshot();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 72,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 12,
                state: CanModuleHeartbeatState::Degraded,
            },
            2,
        );

        let change_set = roster.snapshot().change_set_since(before);
        assert_eq!(change_set.len, 1);
        assert_eq!(
            change_set.get(72),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 72,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Degraded),
            })
        );
    }

    #[test]
    fn module_roster_change_set_captures_sweep_driven_unavailable_change() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 73,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let before = roster.snapshot();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.change_set_since(before).get(73),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 73,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Unavailable),
            })
        );
    }

    #[test]
    fn module_roster_change_set_captures_mixed_multi_node_changes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 74,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Degraded,
            },
            1,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 75,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let before = roster.snapshot();

        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 74,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let change_set = roster.change_set_since(before);
        assert_eq!(change_set.len, 2);
        assert_eq!(
            change_set.get(74),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 74,
                previous_state: Some(CanModuleHeartbeatState::Degraded),
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(
            change_set.get(75),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 75,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Unavailable),
            })
        );
    }

    #[test]
    fn module_roster_event_log_is_empty_for_empty_registry() {
        let event_log = CanModuleRosterRegistry::new().event_log();

        assert!(event_log.is_empty());
        assert_eq!(event_log.len, 0);
        assert_eq!(event_log.entry(0), None);
        assert_eq!(event_log.dropped_count, 0);
    }

    #[test]
    fn module_roster_event_log_records_single_inserted_change() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 76,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            1,
        );

        let event_log = roster.event_log();
        assert_eq!(event_log.len, 1);
        assert_eq!(
            event_log.entry(0),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 76,
                previous_state: None,
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(event_log.entry(1), None);
        assert_eq!(event_log.dropped_count, 0);
    }

    #[test]
    fn module_roster_event_log_preserves_bounded_insertion_order() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 77,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 78,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 77,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        let event_log = roster.event_log();
        assert_eq!(event_log.len, 3);
        assert_eq!(
            event_log.entry(0),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 77,
                previous_state: None,
                current_state: Some(CanModuleHeartbeatState::Starting),
            })
        );
        assert_eq!(
            event_log.entry(1),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 78,
                previous_state: None,
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(
            event_log.entry(2),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 77,
                previous_state: Some(CanModuleHeartbeatState::Starting),
                current_state: Some(CanModuleHeartbeatState::Degraded),
            })
        );
        assert_eq!(event_log.entry(3), None);
        assert_eq!(event_log.dropped_count, 0);
    }

    #[test]
    fn module_roster_event_log_records_sweep_driven_unavailable_change() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 79,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let event_log = roster.event_log();
        assert_eq!(event_log.len, 2);
        assert_eq!(
            event_log.entry(1),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 79,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Unavailable),
            })
        );
        assert_eq!(event_log.dropped_count, 0);
    }

    #[test]
    fn module_roster_event_log_overwrites_oldest_entries_when_full() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 80,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        let mut step = 0usize;
        while step < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY + 1 {
            let next_state = if step.is_multiple_of(2) {
                CanModuleHeartbeatState::Degraded
            } else {
                CanModuleHeartbeatState::Alive
            };
            roster.observe_contract_with_age_ms(
                CanModuleHeartbeatContract {
                    node_id: 80,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9 + step as u32,
                    status: match next_state {
                        CanModuleHeartbeatState::Alive => CanModuleHeartbeatStatus::Ok,
                        CanModuleHeartbeatState::Degraded => CanModuleHeartbeatStatus::Warning,
                        CanModuleHeartbeatState::Starting => CanModuleHeartbeatStatus::Warning,
                        CanModuleHeartbeatState::Unavailable => CanModuleHeartbeatStatus::Error,
                    },
                    error_count: if matches!(next_state, CanModuleHeartbeatState::Alive) {
                        0
                    } else {
                        1
                    },
                    cpu_usage: 5,
                    state: next_state,
                },
                0,
            );
            step += 1;
        }

        let event_log = roster.event_log();
        assert_eq!(event_log.len, CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY);
        assert_eq!(event_log.dropped_count, 2);
        assert_eq!(
            event_log.entry(0),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 80,
                previous_state: Some(CanModuleHeartbeatState::Degraded),
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(
            event_log.entry(CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY - 1),
            Some(CanModuleRosterChangeSetEntry {
                node_id: 80,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Degraded),
            })
        );
        assert_eq!(event_log.entry(CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY), None);
    }

    #[test]
    fn module_roster_event_log_watermark_is_empty_for_empty_registry() {
        assert_eq!(
            CanModuleRosterRegistry::new().event_log_watermark(),
            CanModuleRosterEventLogWatermark {
                retained_event_count: 0,
                dropped_event_count: 0,
                total_written_event_count: 0,
            }
        );
    }

    #[test]
    fn module_roster_event_log_watermark_tracks_first_insert() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 81,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log_watermark(),
            CanModuleRosterEventLogWatermark {
                retained_event_count: 1,
                dropped_event_count: 0,
                total_written_event_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_event_log_watermark_grows_before_saturation() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 82,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 83,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 82,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.event_log_watermark(),
            CanModuleRosterEventLogWatermark {
                retained_event_count: 3,
                dropped_event_count: 0,
                total_written_event_count: 3,
            }
        );
    }

    #[test]
    fn module_roster_event_log_watermark_tracks_saturated_growth() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 84,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        let mut step = 0usize;
        while step < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY + 1 {
            let next_state = if step.is_multiple_of(2) {
                CanModuleHeartbeatState::Degraded
            } else {
                CanModuleHeartbeatState::Alive
            };
            roster.observe_contract_with_age_ms(
                CanModuleHeartbeatContract {
                    node_id: 84,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9 + step as u32,
                    status: match next_state {
                        CanModuleHeartbeatState::Alive => CanModuleHeartbeatStatus::Ok,
                        CanModuleHeartbeatState::Degraded => CanModuleHeartbeatStatus::Warning,
                        CanModuleHeartbeatState::Starting => CanModuleHeartbeatStatus::Warning,
                        CanModuleHeartbeatState::Unavailable => CanModuleHeartbeatStatus::Error,
                    },
                    error_count: if matches!(next_state, CanModuleHeartbeatState::Alive) {
                        0
                    } else {
                        1
                    },
                    cpu_usage: 5,
                    state: next_state,
                },
                0,
            );
            step += 1;
        }

        assert_eq!(
            roster.event_log_watermark(),
            CanModuleRosterEventLogWatermark {
                retained_event_count: CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY,
                dropped_event_count: 2,
                total_written_event_count: (CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY as u32) + 2,
            }
        );
    }

    #[test]
    fn module_roster_event_log_watermark_tracks_sweep_driven_unavailable_transition() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 85,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_watermark(),
            CanModuleRosterEventLogWatermark {
                retained_event_count: 2,
                dropped_event_count: 0,
                total_written_event_count: 2,
            }
        );
    }

    #[test]
    fn module_roster_event_log_delta_is_empty_for_empty_registry() {
        let roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();

        assert_eq!(
            roster.event_log_delta_since(previous_watermark),
            CanModuleRosterEventLogDelta {
                previous_watermark,
                current_watermark: previous_watermark,
                retained_new_event_count: 0,
                dropped_unread_event_count: 0,
                ..CanModuleRosterEventLogDelta::default()
            }
        );
    }

    #[test]
    fn module_roster_event_log_delta_tracks_first_readable_event() {
        let mut roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 86,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        let delta = roster.event_log_delta_since(previous_watermark);
        assert_eq!(delta.previous_watermark, previous_watermark);
        assert_eq!(
            delta.current_watermark,
            CanModuleRosterEventLogWatermark {
                retained_event_count: 1,
                dropped_event_count: 0,
                total_written_event_count: 1,
            }
        );
        assert_eq!(delta.retained_new_event_count, 1);
        assert_eq!(delta.dropped_unread_event_count, 0);
        assert_eq!(
            delta.entries[0],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 86,
                previous_state: None,
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(delta.entries[1], None);
    }

    #[test]
    fn module_roster_event_log_delta_tracks_bounded_unread_growth_before_saturation() {
        let mut roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 87,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 88,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 87,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        let delta = roster.event_log_delta_since(previous_watermark);
        assert_eq!(delta.retained_new_event_count, 3);
        assert_eq!(delta.dropped_unread_event_count, 0);
        assert_eq!(
            delta.entries[0],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 87,
                previous_state: None,
                current_state: Some(CanModuleHeartbeatState::Starting),
            })
        );
        assert_eq!(
            delta.entries[1],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 88,
                previous_state: None,
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(
            delta.entries[2],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 87,
                previous_state: Some(CanModuleHeartbeatState::Starting),
                current_state: Some(CanModuleHeartbeatState::Degraded),
            })
        );
        assert_eq!(delta.entries[3], None);
    }

    #[test]
    fn module_roster_event_log_delta_reports_unread_loss_after_overwrite() {
        let mut roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 89,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        let mut step = 0usize;
        while step < CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY + 1 {
            let next_state = if step.is_multiple_of(2) {
                CanModuleHeartbeatState::Degraded
            } else {
                CanModuleHeartbeatState::Alive
            };
            roster.observe_contract_with_age_ms(
                CanModuleHeartbeatContract {
                    node_id: 89,
                    uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9 + step as u32,
                    status: match next_state {
                        CanModuleHeartbeatState::Alive => CanModuleHeartbeatStatus::Ok,
                        CanModuleHeartbeatState::Degraded => CanModuleHeartbeatStatus::Warning,
                        CanModuleHeartbeatState::Starting => CanModuleHeartbeatStatus::Warning,
                        CanModuleHeartbeatState::Unavailable => CanModuleHeartbeatStatus::Error,
                    },
                    error_count: if matches!(next_state, CanModuleHeartbeatState::Alive) {
                        0
                    } else {
                        1
                    },
                    cpu_usage: 5,
                    state: next_state,
                },
                0,
            );
            step += 1;
        }

        let delta = roster.event_log_delta_since(previous_watermark);
        assert_eq!(
            delta.current_watermark,
            CanModuleRosterEventLogWatermark {
                retained_event_count: CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY,
                dropped_event_count: 2,
                total_written_event_count: (CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY as u32) + 2,
            }
        );
        assert_eq!(
            delta.previous_watermark,
            CanModuleRosterEventLogWatermark {
                retained_event_count: 0,
                dropped_event_count: 0,
                total_written_event_count: 0,
            }
        );
        assert_eq!(
            delta.retained_new_event_count,
            CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY
        );
        assert_eq!(delta.dropped_unread_event_count, 2);
        assert_eq!(
            delta.entries[0],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 89,
                previous_state: Some(CanModuleHeartbeatState::Degraded),
                current_state: Some(CanModuleHeartbeatState::Alive),
            })
        );
        assert_eq!(
            delta.entries[CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY - 1],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 89,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Degraded),
            })
        );
    }

    #[test]
    fn module_roster_event_log_delta_tracks_sweep_driven_unavailable_transition() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 90,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let delta = roster.event_log_delta_since(previous_watermark);
        assert_eq!(delta.retained_new_event_count, 1);
        assert_eq!(delta.dropped_unread_event_count, 0);
        assert_eq!(
            delta.entries[0],
            Some(CanModuleRosterChangeSetEntry {
                node_id: 90,
                previous_state: Some(CanModuleHeartbeatState::Alive),
                current_state: Some(CanModuleHeartbeatState::Unavailable),
            })
        );
        assert_eq!(delta.entries[1], None);
    }

    #[test]
    fn module_roster_event_meaning_reports_inserted_alive() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 91,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_meaning(0),
            Some(CanModuleRosterEventMeaning::FirstObservation(
                CanModuleHeartbeatState::Alive,
            ))
        );
    }

    #[test]
    fn module_roster_event_meaning_reports_startup_recovery_to_alive() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 92,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 92,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_meaning(1),
            Some(CanModuleRosterEventMeaning::RecoveryToAlive {
                from: CanModuleHeartbeatState::Starting,
            })
        );
    }

    #[test]
    fn module_roster_event_meaning_reports_alive_to_degraded() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 93,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 93,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_meaning(1),
            Some(CanModuleRosterEventMeaning::TransitionToDegraded {
                from: CanModuleHeartbeatState::Alive,
            })
        );
    }

    #[test]
    fn module_roster_event_meaning_reports_degraded_recovery_to_alive() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 94,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 94,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_meaning(1),
            Some(CanModuleRosterEventMeaning::RecoveryToAlive {
                from: CanModuleHeartbeatState::Degraded,
            })
        );
    }

    #[test]
    fn module_roster_event_meaning_reports_sweep_driven_unavailable_transition() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 95,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let delta = roster.event_log_delta_since(previous_watermark);
        assert_eq!(
            delta.entries[0].map(CanModuleRosterChangeSetEntry::meaning),
            Some(CanModuleRosterEventMeaning::TimedOutToUnavailable)
        );
    }

    #[test]
    fn module_roster_event_interpretation_reports_inserted_alive_trust_impact() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 96,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_interpretation(0),
            Some(CanModuleRosterEventInterpretation {
                meaning: CanModuleRosterEventMeaning::FirstObservation(
                    CanModuleHeartbeatState::Alive,
                ),
                fallback_impact: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
            })
        );
    }

    #[test]
    fn module_roster_event_interpretation_reports_degraded_hold_last_known_good_impact() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 97,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 97,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_interpretation(1),
            Some(CanModuleRosterEventInterpretation {
                meaning: CanModuleRosterEventMeaning::TransitionToDegraded {
                    from: CanModuleHeartbeatState::Alive,
                },
                fallback_impact: CanRemoteModuleFallbackPolicy::HoldLastKnownGood,
            })
        );
    }

    #[test]
    fn module_roster_event_interpretation_reports_recovery_trust_impact() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 98,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 2,
                cpu_usage: 8,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 98,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log().entry_interpretation(1),
            Some(CanModuleRosterEventInterpretation {
                meaning: CanModuleRosterEventMeaning::RecoveryToAlive {
                    from: CanModuleHeartbeatState::Degraded,
                },
                fallback_impact: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
            })
        );
    }

    #[test]
    fn module_roster_event_interpretation_reports_stale_sweep_local_fallback_impact() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 99,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let delta = roster.event_log_delta_since(previous_watermark);
        assert_eq!(
            delta.entries[0].map(CanModuleRosterChangeSetEntry::interpretation),
            Some(CanModuleRosterEventInterpretation {
                meaning: CanModuleRosterEventMeaning::TimedOutToUnavailable,
                fallback_impact: CanRemoteModuleFallbackPolicy::RequireLocalFallback,
            })
        );
    }

    #[test]
    fn module_roster_event_delta_summary_is_empty_for_empty_registry() {
        let roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();

        assert_eq!(
            roster.event_log_delta_summary_since(previous_watermark),
            CanModuleRosterEventDeltaSummary::default()
        );
    }

    #[test]
    fn module_roster_event_delta_summary_counts_single_recovery() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 100,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 100,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log_delta_summary_since(previous_watermark),
            CanModuleRosterEventDeltaSummary {
                retained_unread_event_count: 1,
                recovery_to_alive_count: 1,
                trust_impact_count: 1,
                ..CanModuleRosterEventDeltaSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_summary_counts_single_degrade() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 101,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 101,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.event_log_delta_summary_since(previous_watermark),
            CanModuleRosterEventDeltaSummary {
                retained_unread_event_count: 1,
                transition_to_degraded_count: 1,
                hold_last_known_good_impact_count: 1,
                ..CanModuleRosterEventDeltaSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_summary_counts_stale_sweep_timeout() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 102,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_delta_summary_since(previous_watermark),
            CanModuleRosterEventDeltaSummary {
                retained_unread_event_count: 1,
                timed_out_to_unavailable_count: 1,
                require_local_fallback_impact_count: 1,
                ..CanModuleRosterEventDeltaSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_summary_counts_mixed_unread_events() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 103,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 104,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 105,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 103,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 104,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_delta_summary_since(previous_watermark),
            CanModuleRosterEventDeltaSummary {
                retained_unread_event_count: 3,
                recovery_to_alive_count: 1,
                transition_to_degraded_count: 1,
                timed_out_to_unavailable_count: 1,
                trust_impact_count: 1,
                hold_last_known_good_impact_count: 1,
                require_local_fallback_impact_count: 1,
                ..CanModuleRosterEventDeltaSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_per_node_is_empty_for_empty_registry() {
        let roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();
        let latest = roster.event_log_delta_latest_per_node_since(previous_watermark);

        assert!(latest.is_empty());
        assert_eq!(latest.len, 0);
        assert_eq!(latest.entry(0), None);
    }

    #[test]
    fn module_roster_event_delta_latest_per_node_preserves_single_unread_node() {
        let mut roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 106,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        let latest = roster.event_log_delta_latest_per_node_since(previous_watermark);
        assert_eq!(latest.len, 1);
        assert_eq!(
            latest.get(106),
            Some(CanModuleRosterLatestUnreadEventEntry {
                node_id: 106,
                interpretation: CanModuleRosterEventInterpretation {
                    meaning: CanModuleRosterEventMeaning::FirstObservation(
                        CanModuleHeartbeatState::Alive,
                    ),
                    fallback_impact: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
                },
            })
        );
    }

    #[test]
    fn module_roster_event_delta_latest_per_node_collapses_repeated_unread_events_to_newest_state()
    {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 107,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 107,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 107,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        let latest = roster.event_log_delta_latest_per_node_since(previous_watermark);
        assert_eq!(latest.len, 1);
        assert_eq!(
            latest.get(107),
            Some(CanModuleRosterLatestUnreadEventEntry {
                node_id: 107,
                interpretation: CanModuleRosterEventInterpretation {
                    meaning: CanModuleRosterEventMeaning::TransitionToDegraded {
                        from: CanModuleHeartbeatState::Alive,
                    },
                    fallback_impact: CanRemoteModuleFallbackPolicy::HoldLastKnownGood,
                },
            })
        );
    }

    #[test]
    fn module_roster_event_delta_latest_per_node_keeps_multiple_unread_nodes_isolated() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 108,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 109,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 108,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 109,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        let latest = roster.event_log_delta_latest_per_node_since(previous_watermark);
        assert_eq!(latest.len, 2);
        assert_eq!(
            latest.get(108),
            Some(CanModuleRosterLatestUnreadEventEntry {
                node_id: 108,
                interpretation: CanModuleRosterEventInterpretation {
                    meaning: CanModuleRosterEventMeaning::RecoveryToAlive {
                        from: CanModuleHeartbeatState::Starting,
                    },
                    fallback_impact: CanRemoteModuleFallbackPolicy::RemoteDataTrusted,
                },
            })
        );
        assert_eq!(
            latest.get(109),
            Some(CanModuleRosterLatestUnreadEventEntry {
                node_id: 109,
                interpretation: CanModuleRosterEventInterpretation {
                    meaning: CanModuleRosterEventMeaning::TransitionToDegraded {
                        from: CanModuleHeartbeatState::Alive,
                    },
                    fallback_impact: CanRemoteModuleFallbackPolicy::HoldLastKnownGood,
                },
            })
        );
    }

    #[test]
    fn module_roster_event_delta_latest_per_node_projects_stale_sweep_transition() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 110,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        let latest = roster.event_log_delta_latest_per_node_since(previous_watermark);
        assert_eq!(latest.len, 1);
        assert_eq!(
            latest.get(110),
            Some(CanModuleRosterLatestUnreadEventEntry {
                node_id: 110,
                interpretation: CanModuleRosterEventInterpretation {
                    meaning: CanModuleRosterEventMeaning::TimedOutToUnavailable,
                    fallback_impact: CanRemoteModuleFallbackPolicy::RequireLocalFallback,
                },
            })
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_summary_is_empty_for_empty_registry() {
        let roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();

        assert_eq!(
            roster.event_log_delta_latest_unread_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadSummary::default()
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_summary_tracks_single_recovery() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 111,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 111,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log_delta_latest_unread_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadSummary {
                latest_unread_node_count: 1,
                recovery_to_alive_count: 1,
                trust_impact_count: 1,
                ..CanModuleRosterLatestUnreadSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_summary_tracks_single_degrade() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 112,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 112,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 11,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.event_log_delta_latest_unread_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadSummary {
                latest_unread_node_count: 1,
                transition_to_degraded_count: 1,
                hold_last_known_good_impact_count: 1,
                ..CanModuleRosterLatestUnreadSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_summary_tracks_stale_sweep_timeout() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 113,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_delta_latest_unread_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadSummary {
                latest_unread_node_count: 1,
                timed_out_to_unavailable_count: 1,
                require_local_fallback_impact_count: 1,
                ..CanModuleRosterLatestUnreadSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_summary_tracks_mixed_unread_nodes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 114,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 115,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 116,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 114,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 115,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 117,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_delta_latest_unread_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadSummary {
                latest_unread_node_count: 4,
                first_observation_count: 1,
                recovery_to_alive_count: 1,
                transition_to_degraded_count: 1,
                timed_out_to_unavailable_count: 1,
                trust_impact_count: 2,
                hold_last_known_good_impact_count: 1,
                require_local_fallback_impact_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_shutdown_impact_summary_is_empty_for_empty_registry()
    {
        let roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();

        assert_eq!(
            roster.event_log_delta_latest_unread_shutdown_impact_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadShutdownImpactSummary::default()
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_shutdown_impact_summary_tracks_degraded_node() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 118,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 118,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );

        assert_eq!(
            roster.event_log_delta_latest_unread_shutdown_impact_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadShutdownImpactSummary {
                latest_unread_node_count: 1,
                coordinated_shutdown_recommended_count: 1,
                ..CanModuleRosterLatestUnreadShutdownImpactSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_shutdown_impact_summary_tracks_unavailable_node() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 119,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_delta_latest_unread_shutdown_impact_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadShutdownImpactSummary {
                latest_unread_node_count: 1,
                local_takeover_required_count: 1,
                ..CanModuleRosterLatestUnreadShutdownImpactSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_shutdown_impact_summary_tracks_mixed_nodes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 120,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 121,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 122,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 120,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 121,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 123,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster.event_log_delta_latest_unread_shutdown_impact_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadShutdownImpactSummary {
                latest_unread_node_count: 4,
                no_shutdown_needed_count: 2,
                coordinated_shutdown_recommended_count: 1,
                local_takeover_required_count: 1,
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_shutdown_impact_summary_uses_latest_node_state() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 124,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 124,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 124,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 10,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster.event_log_delta_latest_unread_shutdown_impact_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadShutdownImpactSummary {
                latest_unread_node_count: 1,
                no_shutdown_needed_count: 1,
                ..CanModuleRosterLatestUnreadShutdownImpactSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_startup_readiness_summary_is_empty_for_empty_registry(
    ) {
        let roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();

        assert_eq!(
            roster
                .event_log_delta_latest_unread_startup_readiness_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadStartupReadinessSummary::default()
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_startup_readiness_summary_tracks_recovery_to_alive()
    {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 125,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 125,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );

        assert_eq!(
            roster
                .event_log_delta_latest_unread_startup_readiness_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadStartupReadinessSummary {
                latest_unread_node_count: 1,
                ready_for_trust_count: 1,
                ..CanModuleRosterLatestUnreadStartupReadinessSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_startup_readiness_summary_tracks_waiting_on_starting_node(
    ) {
        let mut roster = CanModuleRosterRegistry::new();
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 126,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );

        assert_eq!(
            roster
                .event_log_delta_latest_unread_startup_readiness_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadStartupReadinessSummary {
                latest_unread_node_count: 1,
                waiting_on_startup_count: 1,
                ..CanModuleRosterLatestUnreadStartupReadinessSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_startup_readiness_summary_tracks_unavailable_node() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 127,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster
                .event_log_delta_latest_unread_startup_readiness_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadStartupReadinessSummary {
                latest_unread_node_count: 1,
                blocked_by_unavailable_count: 1,
                ..CanModuleRosterLatestUnreadStartupReadinessSummary::default()
            }
        );
    }

    #[test]
    fn module_roster_event_delta_latest_unread_startup_readiness_summary_tracks_mixed_nodes() {
        let mut roster = CanModuleRosterRegistry::new();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 128,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS - 1,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 0,
                cpu_usage: 7,
                state: CanModuleHeartbeatState::Starting,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 129,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 6,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 130,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 9,
                state: CanModuleHeartbeatState::Alive,
            },
            CAN_HEARTBEAT_TIMEOUT_MS,
        );
        let previous_watermark = roster.event_log_watermark();
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 128,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 5,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 129,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 9,
                status: CanModuleHeartbeatStatus::Warning,
                error_count: 1,
                cpu_usage: 10,
                state: CanModuleHeartbeatState::Degraded,
            },
            0,
        );
        roster.observe_contract_with_age_ms(
            CanModuleHeartbeatContract {
                node_id: 131,
                uptime_seconds: CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS + 8,
                status: CanModuleHeartbeatStatus::Ok,
                error_count: 0,
                cpu_usage: 4,
                state: CanModuleHeartbeatState::Alive,
            },
            0,
        );
        roster.sweep_stale(CanHeartbeatObservationPolicy::default(), 1);

        assert_eq!(
            roster
                .event_log_delta_latest_unread_startup_readiness_summary_since(previous_watermark),
            CanModuleRosterLatestUnreadStartupReadinessSummary {
                latest_unread_node_count: 4,
                ready_for_trust_count: 2,
                holding_last_known_good_count: 1,
                blocked_by_unavailable_count: 1,
                ..CanModuleRosterLatestUnreadStartupReadinessSummary::default()
            }
        );
    }

    #[test]
    fn route_priorities_keep_control_and_faults_ahead_of_telemetry() {
        let reset = CanMessageRoute::new(CanMessageClass::ResetCommand);
        let control = CanMessageRoute::new(CanMessageClass::EngineControlCommand);
        let fault = CanMessageRoute::new(CanMessageClass::Fault);
        let trigger = CanMessageRoute::new(CanMessageClass::TriggerTiming);
        let sensor = CanMessageRoute::new(CanMessageClass::SensorData);
        let table = CanMessageRoute::new(CanMessageClass::FuelTable);
        let config = CanMessageRoute::new(CanMessageClass::EngineConfig);
        let heartbeat = CanMessageRoute::new(CanMessageClass::Heartbeat);

        assert!(reset.priority < sensor.priority);
        assert!(control.priority < sensor.priority);
        assert!(fault.priority < sensor.priority);
        assert!(trigger.priority < heartbeat.priority);
        assert!(table.priority < heartbeat.priority);
        assert!(config.priority < heartbeat.priority);

        assert!(reset.arbitration_id < sensor.arbitration_id);
        assert!(control.arbitration_id < sensor.arbitration_id);
        assert!(fault.arbitration_id < heartbeat.arbitration_id);
        assert!(trigger.arbitration_id < heartbeat.arbitration_id);
    }
}
