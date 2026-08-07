//! OBD2 services, dispatch, vehicle info, and DTC (split from can monolith; review 006).

use super::transport::*;
use crate::Message;
use ecu_domain::diag::DiagCode;

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
