#![cfg(all(feature = "transport-can", any(test, feature = "flash-kv")))]
#![allow(dead_code)]

use ecu_target_common::transport_service::Obd2ProvisionedIdentityRecord;
use ecu_transport::Message;
#[cfg(feature = "obd2-identity-can-provisioning")]
use hmac::{Hmac, Mac};
#[cfg(feature = "obd2-identity-can-provisioning")]
use sha2::Sha256;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Obd2IdentityProvisioningField {
    Vin,
    CalibrationId,
    BoardBuildIdentity,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Obd2IdentityProvisioningError<E> {
    Empty(Obd2IdentityProvisioningField),
    TooLong {
        field: Obd2IdentityProvisioningField,
        max_len: usize,
    },
    Store(E),
}

pub const OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_NONE: u8 = 0;
pub const OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_VIN: u8 = 1;
pub const OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_CALIBRATION_ID: u8 = 2;
pub const OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_BOARD_BUILD_IDENTITY: u8 = 3;
pub const OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE: u8 = 0;
pub const OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_UNAUTHORIZED: u8 = 1;
pub const OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_ROLLBACK: u8 = 2;
pub const OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE: u8 = 3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningLengths {
    pub vin: u8,
    pub calibration_id: u8,
    pub board_build_identity: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningStatus {
    pub attempted: bool,
    pub accepted: bool,
    pub rejected_field: Option<Obd2IdentityProvisioningField>,
    pub store_failed: bool,
    pub normalized_lengths: Option<Obd2IdentityProvisioningLengths>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningCommand<'a> {
    pub request_id: u32,
    pub vin: &'a [u8],
    pub calibration_id: &'a [u8],
    pub board_build_identity: &'a [u8],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningCommandAudit {
    pub request_id: u32,
    pub authorized: bool,
    pub authorization_failed: bool,
    pub provisioning_status: Option<Obd2IdentityProvisioningStatus>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningKeyCommand {
    pub request_id: u32,
    pub nonce: u32,
    pub generation: u32,
    pub revoke: bool,
    pub key: [u8; 32],
    pub tag: [u8; 32],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningKeyAudit {
    pub request_id: u32,
    pub authorized: bool,
    pub accepted: bool,
    pub store_failed: bool,
    pub rejected_reason: u8,
    pub generation: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Obd2IdentityProvisioningOperatorArm {
    armed_command: Option<Obd2IdentityProvisioningArmedCommand>,
    key: Option<[u8; 32]>,
    last_arm_nonce: Option<u32>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Obd2IdentityProvisioningArmedCommand {
    request_id: u32,
    vin_len: u8,
    vin: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    calibration_id_len: u8,
    calibration_id: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    board_build_identity_len: u8,
    board_build_identity: [u8; 6],
}

impl Obd2IdentityProvisioningOperatorArm {
    pub const fn new() -> Self {
        Self {
            armed_command: None,
            key: None,
            last_arm_nonce: None,
        }
    }

    pub const fn new_with_key(key: [u8; 32]) -> Self {
        Self {
            armed_command: None,
            key: Some(key),
            last_arm_nonce: None,
        }
    }

    pub const fn new_with_optional_key(key: Option<[u8; 32]>) -> Self {
        Self::new_with_optional_key_and_last_arm_nonce(key, None)
    }

    pub const fn new_with_optional_key_and_last_arm_nonce(
        key: Option<[u8; 32]>,
        last_arm_nonce: Option<u32>,
    ) -> Self {
        Self {
            armed_command: None,
            key,
            last_arm_nonce,
        }
    }

    pub fn from_compile_time_env() -> Self {
        Self::new_with_optional_key(Self::compile_time_env_key())
    }

    pub fn from_persisted_key_or_compile_time_env(
        persisted_key: Option<(Option<[u8; 32]>, u32)>,
    ) -> Self {
        match persisted_key {
            Some((key, last_arm_nonce)) => {
                Self::new_with_optional_key_and_last_arm_nonce(key, Some(last_arm_nonce))
            }
            None => Self::new_with_optional_key(Self::compile_time_env_key()),
        }
    }

    pub fn compile_time_env_key() -> Option<[u8; 32]> {
        option_env!("STM32F4_OBD2_IDENTITY_PROVISIONING_KEY_HEX")
            .and_then(|hex| parse_hex_key_32(hex.as_bytes()))
    }

    pub const fn current_key(&self) -> Option<[u8; 32]> {
        self.key
    }

    pub const fn last_arm_nonce(&self) -> Option<u32> {
        self.last_arm_nonce
    }

    pub fn arm(
        &mut self,
        request_id: u32,
        nonce: u32,
        vin_len: u8,
        vin: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
        calibration_id_len: u8,
        calibration_id: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
        board_build_identity_len: u8,
        board_build_identity: [u8; 6],
        tag: [u8; 32],
    ) -> Message {
        #[cfg(feature = "obd2-identity-can-provisioning")]
        let nonce_is_fresh = self
            .last_arm_nonce
            .map(|last_nonce| nonce > last_nonce)
            .unwrap_or(true);
        #[cfg(feature = "obd2-identity-can-provisioning")]
        let armed = nonce_is_fresh
            && self
                .key
                .map(|key| {
                    verify_arm_tag(
                        key,
                        request_id,
                        nonce,
                        vin_len,
                        &vin,
                        calibration_id_len,
                        &calibration_id,
                        board_build_identity_len,
                        &board_build_identity,
                        tag,
                    )
                })
                .unwrap_or(false);
        #[cfg(not(feature = "obd2-identity-can-provisioning"))]
        let armed = {
            let _ = (
                nonce,
                vin_len,
                vin,
                calibration_id_len,
                calibration_id,
                board_build_identity_len,
                board_build_identity,
                tag,
            );
            false
        };
        if armed {
            self.last_arm_nonce = Some(nonce);
            self.armed_command = Some(Obd2IdentityProvisioningArmedCommand {
                request_id,
                vin_len,
                vin,
                calibration_id_len,
                calibration_id,
                board_build_identity_len,
                board_build_identity,
            });
        }
        Message::Obd2IdentityProvisioningArmAudit { request_id, armed }
    }
}

impl Obd2IdentityProvisioningAuthorizer for Obd2IdentityProvisioningOperatorArm {
    fn authorize_obd2_identity_provisioning(
        &mut self,
        command: &Obd2IdentityProvisioningCommand<'_>,
    ) -> bool {
        let authorized = self
            .armed_command
            .map(|armed| armed.matches(command))
            .unwrap_or(false);
        if authorized {
            self.armed_command = None;
        }
        authorized
    }
}

impl Obd2IdentityProvisioningArmedCommand {
    fn matches(self, command: &Obd2IdentityProvisioningCommand<'_>) -> bool {
        self.request_id == command.request_id
            && self.vin_len as usize == command.vin.len()
            && self.calibration_id_len as usize == command.calibration_id.len()
            && self.board_build_identity_len as usize == command.board_build_identity.len()
            && self.vin[..command.vin.len()] == *command.vin
            && self.calibration_id[..command.calibration_id.len()] == *command.calibration_id
            && self.board_build_identity[..command.board_build_identity.len()]
                == *command.board_build_identity
    }
}

pub trait Obd2IdentityProvisioningAuthorizer {
    fn authorize_obd2_identity_provisioning(
        &mut self,
        command: &Obd2IdentityProvisioningCommand<'_>,
    ) -> bool;
}

pub trait Obd2IdentityProvisioningSink {
    type Error;

    fn provision_obd2_identity_record(
        &mut self,
        record: Obd2ProvisionedIdentityRecord,
    ) -> Result<(), Self::Error>;
}

pub trait Obd2IdentityProvisioningKeyStore {
    type Error;

    fn load_obd2_identity_provisioning_key_generation(&self) -> Option<u32>;

    fn write_obd2_identity_provisioning_key_record(
        &mut self,
        key: [u8; 32],
        generation: u32,
        revoked: bool,
    ) -> Result<(), Self::Error>;
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub trait Obd2IdentityProvisioningArmNonceStore {
    type Error;

    fn write_obd2_identity_provisioning_arm_nonce(
        &mut self,
        current_key: [u8; 32],
        nonce: u32,
    ) -> Result<(), Self::Error>;
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
impl Obd2IdentityProvisioningSink for crate::store_support::FlashKv {
    type Error = ecu_calibration::KvError;

    fn provision_obd2_identity_record(
        &mut self,
        record: Obd2ProvisionedIdentityRecord,
    ) -> Result<(), Self::Error> {
        crate::store_support::provision_obd2_identity_record(self, record)
    }
}

#[cfg(all(feature = "transport-can", feature = "flash-kv"))]
impl Obd2IdentityProvisioningKeyStore for crate::store_support::FlashKv {
    type Error = ecu_calibration::KvError;

    fn load_obd2_identity_provisioning_key_generation(&self) -> Option<u32> {
        self.load_obd2_identity_provisioning_key_record()
            .map(|record| record.generation)
    }

    fn write_obd2_identity_provisioning_key_record(
        &mut self,
        key: [u8; 32],
        generation: u32,
        revoked: bool,
    ) -> Result<(), Self::Error> {
        let record = if revoked {
            crate::store_support::Obd2IdentityProvisioningKeyRecord::revoked(key, generation)
        } else {
            crate::store_support::Obd2IdentityProvisioningKeyRecord::active(key, generation)
        };
        crate::store_support::FlashKv::write_obd2_identity_provisioning_key_record(self, record)
    }
}

#[cfg(feature = "obd2-identity-can-provisioning")]
impl Obd2IdentityProvisioningArmNonceStore for crate::store_support::FlashKv {
    type Error = ecu_calibration::KvError;

    fn write_obd2_identity_provisioning_arm_nonce(
        &mut self,
        current_key: [u8; 32],
        nonce: u32,
    ) -> Result<(), Self::Error> {
        crate::store_support::FlashKv::write_obd2_identity_provisioning_arm_nonce(
            self,
            current_key,
            nonce,
        )
    }
}

pub fn parse_obd2_identity_provisioning_fields(
    vin: &[u8],
    calibration_id: &[u8],
    board_build_identity: &[u8],
) -> Result<Obd2ProvisionedIdentityRecord, Obd2IdentityProvisioningError<core::convert::Infallible>>
{
    reject_too_long(
        Obd2IdentityProvisioningField::Vin,
        vin,
        ecu_transport::CAN_OBD2_VIN_LEN,
    )?;
    reject_too_long(
        Obd2IdentityProvisioningField::CalibrationId,
        calibration_id,
        ecu_transport::CAN_OBD2_VIN_LEN,
    )?;
    reject_too_long(
        Obd2IdentityProvisioningField::BoardBuildIdentity,
        board_build_identity,
        6,
    )?;

    let record = Obd2ProvisionedIdentityRecord::from_ascii(
        Some(vin),
        Some(calibration_id),
        Some(board_build_identity),
    );
    reject_empty(Obd2IdentityProvisioningField::Vin, record.vin_len)?;
    reject_empty(
        Obd2IdentityProvisioningField::CalibrationId,
        record.calibration_id_len,
    )?;
    reject_empty(
        Obd2IdentityProvisioningField::BoardBuildIdentity,
        record.board_build_identity_len,
    )?;
    Ok(record)
}

pub fn provision_obd2_identity_fields<S: Obd2IdentityProvisioningSink>(
    sink: &mut S,
    vin: &[u8],
    calibration_id: &[u8],
    board_build_identity: &[u8],
) -> Result<(), Obd2IdentityProvisioningError<S::Error>> {
    let record = parse_obd2_identity_provisioning_fields(vin, calibration_id, board_build_identity)
        .map_err(|error| match error {
            Obd2IdentityProvisioningError::Empty(field) => {
                Obd2IdentityProvisioningError::Empty(field)
            }
            Obd2IdentityProvisioningError::TooLong { field, max_len } => {
                Obd2IdentityProvisioningError::TooLong { field, max_len }
            }
            Obd2IdentityProvisioningError::Store(never) => match never {},
        })?;
    sink.provision_obd2_identity_record(record)
        .map_err(Obd2IdentityProvisioningError::Store)
}

pub fn provision_obd2_identity_fields_with_status<S: Obd2IdentityProvisioningSink>(
    sink: &mut S,
    vin: &[u8],
    calibration_id: &[u8],
    board_build_identity: &[u8],
) -> Obd2IdentityProvisioningStatus {
    let record =
        match parse_obd2_identity_provisioning_fields(vin, calibration_id, board_build_identity) {
            Ok(record) => record,
            Err(error) => {
                return Obd2IdentityProvisioningStatus {
                    attempted: true,
                    accepted: false,
                    rejected_field: Some(error.field()),
                    store_failed: false,
                    normalized_lengths: None,
                };
            }
        };
    let normalized_lengths = Some(Obd2IdentityProvisioningLengths {
        vin: record.vin_len,
        calibration_id: record.calibration_id_len,
        board_build_identity: record.board_build_identity_len,
    });

    match provision_obd2_identity_fields(sink, vin, calibration_id, board_build_identity) {
        Ok(()) => Obd2IdentityProvisioningStatus {
            attempted: true,
            accepted: true,
            rejected_field: None,
            store_failed: false,
            normalized_lengths,
        },
        Err(Obd2IdentityProvisioningError::Store(_)) => Obd2IdentityProvisioningStatus {
            attempted: true,
            accepted: false,
            rejected_field: None,
            store_failed: true,
            normalized_lengths,
        },
        Err(Obd2IdentityProvisioningError::Empty(field))
        | Err(Obd2IdentityProvisioningError::TooLong { field, .. }) => {
            Obd2IdentityProvisioningStatus {
                attempted: true,
                accepted: false,
                rejected_field: Some(field),
                store_failed: false,
                normalized_lengths: None,
            }
        }
    }
}

pub fn provision_obd2_identity_command_with_audit<
    A: Obd2IdentityProvisioningAuthorizer,
    S: Obd2IdentityProvisioningSink,
>(
    authorizer: &mut A,
    sink: &mut S,
    command: Obd2IdentityProvisioningCommand<'_>,
) -> Obd2IdentityProvisioningCommandAudit {
    if !authorizer.authorize_obd2_identity_provisioning(&command) {
        return Obd2IdentityProvisioningCommandAudit {
            request_id: command.request_id,
            authorized: false,
            authorization_failed: true,
            provisioning_status: None,
        };
    }

    let provisioning_status = provision_obd2_identity_fields_with_status(
        sink,
        command.vin,
        command.calibration_id,
        command.board_build_identity,
    );
    Obd2IdentityProvisioningCommandAudit {
        request_id: command.request_id,
        authorized: true,
        authorization_failed: false,
        provisioning_status: Some(provisioning_status),
    }
}

pub fn obd2_identity_operator_command_from_message(
    message: &Message,
) -> Option<Obd2IdentityProvisioningCommand<'_>> {
    let Message::Obd2IdentityProvisioningCommand {
        request_id,
        vin_len,
        vin,
        calibration_id_len,
        calibration_id,
        board_build_identity_len,
        board_build_identity,
    } = message
    else {
        return None;
    };
    let vin_len = *vin_len as usize;
    let calibration_id_len = *calibration_id_len as usize;
    let board_build_identity_len = *board_build_identity_len as usize;
    if vin_len > vin.len()
        || calibration_id_len > calibration_id.len()
        || board_build_identity_len > board_build_identity.len()
    {
        return None;
    }
    Some(Obd2IdentityProvisioningCommand {
        request_id: *request_id,
        vin: &vin[..vin_len],
        calibration_id: &calibration_id[..calibration_id_len],
        board_build_identity: &board_build_identity[..board_build_identity_len],
    })
}

pub fn obd2_identity_operator_arm_message(
    arm: &mut Obd2IdentityProvisioningOperatorArm,
    message: &Message,
) -> Option<Message> {
    let Message::Obd2IdentityProvisioningArm {
        request_id,
        nonce,
        vin_len,
        vin,
        calibration_id_len,
        calibration_id,
        board_build_identity_len,
        board_build_identity,
        tag,
    } = message
    else {
        return None;
    };
    Some(arm.arm(
        *request_id,
        *nonce,
        *vin_len,
        *vin,
        *calibration_id_len,
        *calibration_id,
        *board_build_identity_len,
        *board_build_identity,
        *tag,
    ))
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn obd2_identity_operator_arm_message_with_nonce_store<
    S: Obd2IdentityProvisioningArmNonceStore,
>(
    arm: &mut Obd2IdentityProvisioningOperatorArm,
    store: &mut S,
    message: &Message,
) -> Result<Option<Message>, S::Error> {
    let Message::Obd2IdentityProvisioningArm {
        request_id, nonce, ..
    } = message
    else {
        return Ok(None);
    };
    let previous = *arm;
    let response = obd2_identity_operator_arm_message(arm, message);
    let Some(Message::Obd2IdentityProvisioningArmAudit { armed: true, .. }) = response else {
        return Ok(response);
    };
    let Some(current_key) = arm.current_key() else {
        *arm = previous;
        return Ok(Some(Message::Obd2IdentityProvisioningArmAudit {
            request_id: *request_id,
            armed: false,
        }));
    };
    if store
        .write_obd2_identity_provisioning_arm_nonce(current_key, *nonce)
        .is_err()
    {
        *arm = previous;
        return Ok(Some(Message::Obd2IdentityProvisioningArmAudit {
            request_id: *request_id,
            armed: false,
        }));
    }
    Ok(response)
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn obd2_identity_operator_arm_message_with_persisted_nonce(
    arm: &mut Obd2IdentityProvisioningOperatorArm,
    store: &mut crate::store_support::FlashKv,
    message: &Message,
) -> Result<Option<Message>, ecu_calibration::KvError> {
    obd2_identity_operator_arm_message_with_nonce_store(arm, store, message)
}

pub fn obd2_identity_operator_audit_message(
    audit: Obd2IdentityProvisioningCommandAudit,
) -> Message {
    let status = audit.provisioning_status;
    let lengths = status.and_then(|status| status.normalized_lengths);
    Message::Obd2IdentityProvisioningAudit {
        request_id: audit.request_id,
        authorized: audit.authorized,
        authorization_failed: audit.authorization_failed,
        attempted: status.map(|status| status.attempted).unwrap_or(false),
        accepted: status.map(|status| status.accepted).unwrap_or(false),
        store_failed: status.map(|status| status.store_failed).unwrap_or(false),
        rejected_field: encode_rejected_field(status.and_then(|status| status.rejected_field)),
        vin_len: lengths.map(|lengths| lengths.vin).unwrap_or(0),
        calibration_id_len: lengths.map(|lengths| lengths.calibration_id).unwrap_or(0),
        board_build_identity_len: lengths
            .map(|lengths| lengths.board_build_identity)
            .unwrap_or(0),
    }
}

pub fn provision_obd2_identity_operator_message_with_audit<
    A: Obd2IdentityProvisioningAuthorizer,
    S: Obd2IdentityProvisioningSink,
>(
    authorizer: &mut A,
    sink: &mut S,
    message: &Message,
) -> Option<Message> {
    let command = obd2_identity_operator_command_from_message(message)?;
    let audit = provision_obd2_identity_command_with_audit(authorizer, sink, command);
    Some(obd2_identity_operator_audit_message(audit))
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn obd2_identity_operator_key_command_from_message(
    message: &Message,
) -> Option<Obd2IdentityProvisioningKeyCommand> {
    let Message::Obd2IdentityProvisioningKeyCommand {
        request_id,
        nonce,
        generation,
        revoke,
        key,
        tag,
    } = message
    else {
        return None;
    };
    Some(Obd2IdentityProvisioningKeyCommand {
        request_id: *request_id,
        nonce: *nonce,
        generation: *generation,
        revoke: *revoke,
        key: *key,
        tag: *tag,
    })
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn provision_obd2_identity_key_command_with_audit<S: Obd2IdentityProvisioningKeyStore>(
    operator: &mut Obd2IdentityProvisioningOperatorArm,
    store: &mut S,
    command: Obd2IdentityProvisioningKeyCommand,
) -> Obd2IdentityProvisioningKeyAudit {
    let Some(current_key) = operator.current_key() else {
        return Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: false,
            accepted: false,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_UNAUTHORIZED,
            generation: command.generation,
        };
    };
    if !verify_key_command_tag(current_key, command) {
        return Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: false,
            accepted: false,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_UNAUTHORIZED,
            generation: command.generation,
        };
    }
    if command.generation
        <= store
            .load_obd2_identity_provisioning_key_generation()
            .unwrap_or(0)
    {
        return Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: true,
            accepted: false,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_ROLLBACK,
            generation: command.generation,
        };
    }
    if store
        .write_obd2_identity_provisioning_key_record(
            command.key,
            command.generation,
            command.revoke,
        )
        .is_err()
    {
        return Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE,
            generation: command.generation,
        };
    }
    *operator = Obd2IdentityProvisioningOperatorArm::new_with_optional_key(if command.revoke {
        None
    } else {
        Some(command.key)
    });
    Obd2IdentityProvisioningKeyAudit {
        request_id: command.request_id,
        authorized: true,
        accepted: true,
        store_failed: false,
        rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE,
        generation: command.generation,
    }
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn obd2_identity_operator_key_audit_message(
    audit: Obd2IdentityProvisioningKeyAudit,
) -> Message {
    Message::Obd2IdentityProvisioningKeyAudit {
        request_id: audit.request_id,
        authorized: audit.authorized,
        accepted: audit.accepted,
        store_failed: audit.store_failed,
        rejected_reason: audit.rejected_reason,
        generation: audit.generation,
    }
}

#[cfg(feature = "transport-can")]
pub fn obd2_identity_key_lifecycle_status_from_audit(
    audit: Obd2IdentityProvisioningKeyAudit,
) -> ecu_transport::CanObd2IdentityKeyLifecycleStatus {
    obd2_identity_key_lifecycle_status_from_audit_with_persistence(audit, true)
}

#[cfg(feature = "transport-can")]
pub fn obd2_identity_key_lifecycle_status_from_audit_with_persistence(
    audit: Obd2IdentityProvisioningKeyAudit,
    persisted: bool,
) -> ecu_transport::CanObd2IdentityKeyLifecycleStatus {
    ecu_transport::CanObd2IdentityKeyLifecycleStatus {
        present: true,
        persisted,
        request_id: audit.request_id,
        authorized: audit.authorized,
        accepted: audit.accepted,
        store_failed: audit.store_failed,
        rejected_reason: audit.rejected_reason,
        generation: audit.generation,
    }
}

#[cfg(feature = "transport-can")]
pub fn obd2_identity_key_lifecycle_status_from_optional_audit(
    audit: Option<Obd2IdentityProvisioningKeyAudit>,
) -> ecu_transport::CanObd2IdentityKeyLifecycleStatus {
    audit
        .map(obd2_identity_key_lifecycle_status_from_audit)
        .unwrap_or(ecu_transport::CanObd2IdentityKeyLifecycleStatus::absent())
}

#[cfg(feature = "transport-can")]
pub fn obd2_identity_key_lifecycle_status_after_operator_response(
    response_audit: Option<Obd2IdentityProvisioningKeyAudit>,
    persisted_audit: Option<Obd2IdentityProvisioningKeyAudit>,
) -> ecu_transport::CanObd2IdentityKeyLifecycleStatus {
    match (response_audit, persisted_audit) {
        (Some(response_audit), Some(persisted_audit)) if response_audit == persisted_audit => {
            obd2_identity_key_lifecycle_status_from_audit(persisted_audit)
        }
        (Some(response_audit), _) => {
            obd2_identity_key_lifecycle_status_from_audit_with_persistence(response_audit, false)
        }
        (None, persisted_audit) => {
            obd2_identity_key_lifecycle_status_from_optional_audit(persisted_audit)
        }
    }
}

#[cfg(feature = "transport-can")]
pub fn obd2_identity_key_audit_from_message(
    message: &Message,
) -> Option<Obd2IdentityProvisioningKeyAudit> {
    let Message::Obd2IdentityProvisioningKeyAudit {
        request_id,
        authorized,
        accepted,
        store_failed,
        rejected_reason,
        generation,
    } = message
    else {
        return None;
    };
    Some(Obd2IdentityProvisioningKeyAudit {
        request_id: *request_id,
        authorized: *authorized,
        accepted: *accepted,
        store_failed: *store_failed,
        rejected_reason: *rejected_reason,
        generation: *generation,
    })
}

#[cfg(feature = "obd2-identity-can-provisioning")]
fn key_audit_response_after_best_effort_persist<E>(
    audit: Obd2IdentityProvisioningKeyAudit,
    persist: impl FnOnce(Obd2IdentityProvisioningKeyAudit) -> Result<(), E>,
) -> Message {
    let _ = persist(audit);
    obd2_identity_operator_key_audit_message(audit)
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn provision_obd2_identity_key_operator_message_with_audit<
    S: Obd2IdentityProvisioningKeyStore,
>(
    operator: &mut Obd2IdentityProvisioningOperatorArm,
    store: &mut S,
    message: &Message,
) -> Option<Message> {
    let command = obd2_identity_operator_key_command_from_message(message)?;
    let audit = provision_obd2_identity_key_command_with_audit(operator, store, command);
    Some(obd2_identity_operator_key_audit_message(audit))
}

#[cfg(feature = "flash-kv")]
pub fn provision_flash_kv_obd2_identity_fields(
    store: &mut crate::store_support::FlashKv,
    vin: &[u8],
    calibration_id: &[u8],
    board_build_identity: &[u8],
) -> Result<(), Obd2IdentityProvisioningError<ecu_calibration::KvError>> {
    provision_obd2_identity_fields(store, vin, calibration_id, board_build_identity)
}

#[cfg(feature = "flash-kv")]
pub fn provision_flash_kv_obd2_identity_operator_message_with_persisted_audit<
    A: Obd2IdentityProvisioningAuthorizer,
>(
    authorizer: &mut A,
    store: &mut crate::store_support::FlashKv,
    message: &Message,
) -> Result<Option<Message>, ecu_calibration::KvError> {
    let Some(command) = obd2_identity_operator_command_from_message(message) else {
        return Ok(None);
    };
    let audit =
        provision_flash_kv_obd2_identity_command_with_persisted_audit(authorizer, store, command)?;
    Ok(Some(obd2_identity_operator_audit_message(audit)))
}

#[cfg(feature = "flash-kv")]
pub fn provision_flash_kv_obd2_identity_command_with_persisted_audit<
    A: Obd2IdentityProvisioningAuthorizer,
>(
    authorizer: &mut A,
    store: &mut crate::store_support::FlashKv,
    command: Obd2IdentityProvisioningCommand<'_>,
) -> Result<Obd2IdentityProvisioningCommandAudit, ecu_calibration::KvError> {
    let audit = provision_obd2_identity_command_with_audit(authorizer, store, command);
    store.write_obd2_identity_command_audit(audit)?;
    Ok(audit)
}

#[cfg(feature = "obd2-identity-can-provisioning")]
fn persist_key_audit_or_return_volatile_response(
    store: &mut crate::store_support::FlashKv,
    audit: Obd2IdentityProvisioningKeyAudit,
    key_record_to_preserve: Option<crate::store_support::Obd2IdentityProvisioningKeyRecord>,
) -> Message {
    key_audit_response_after_best_effort_persist(audit, |audit| {
        store.write_obd2_identity_key_audit_with_key_record(audit, key_record_to_preserve)
    })
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn provision_flash_kv_obd2_identity_key_operator_message_with_audit(
    operator: &mut Obd2IdentityProvisioningOperatorArm,
    store: &mut crate::store_support::FlashKv,
    message: &Message,
) -> Result<Option<Message>, ecu_calibration::KvError> {
    let Some(command) = obd2_identity_operator_key_command_from_message(message) else {
        return Ok(None);
    };
    let previous_key_record = store.load_obd2_identity_provisioning_key_record();
    let Some(current_key) = operator.current_key() else {
        let audit = Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: false,
            accepted: false,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_UNAUTHORIZED,
            generation: command.generation,
        };
        return Ok(Some(persist_key_audit_or_return_volatile_response(
            store,
            audit,
            previous_key_record,
        )));
    };
    if !verify_key_command_tag(current_key, command) {
        let audit = Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: false,
            accepted: false,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_UNAUTHORIZED,
            generation: command.generation,
        };
        return Ok(Some(persist_key_audit_or_return_volatile_response(
            store,
            audit,
            previous_key_record,
        )));
    }
    if command.generation
        <= previous_key_record
            .map(|record| record.generation)
            .unwrap_or(0)
    {
        let audit = Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: true,
            accepted: false,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_ROLLBACK,
            generation: command.generation,
        };
        return Ok(Some(persist_key_audit_or_return_volatile_response(
            store,
            audit,
            previous_key_record,
        )));
    }

    let key_record = if command.revoke {
        crate::store_support::Obd2IdentityProvisioningKeyRecord::revoked(
            command.key,
            command.generation,
        )
    } else {
        crate::store_support::Obd2IdentityProvisioningKeyRecord::active(
            command.key,
            command.generation,
        )
    };
    let accepted_audit = Obd2IdentityProvisioningKeyAudit {
        request_id: command.request_id,
        authorized: true,
        accepted: true,
        store_failed: false,
        rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE,
        generation: command.generation,
    };
    if store
        .write_obd2_identity_provisioning_key_record_with_audit(key_record, accepted_audit)
        .is_err()
    {
        let audit = Obd2IdentityProvisioningKeyAudit {
            request_id: command.request_id,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE,
            generation: command.generation,
        };
        return Ok(Some(persist_key_audit_or_return_volatile_response(
            store,
            audit,
            previous_key_record,
        )));
    }
    *operator = Obd2IdentityProvisioningOperatorArm::new_with_optional_key(if command.revoke {
        None
    } else {
        Some(command.key)
    });
    let audit = accepted_audit;
    Ok(Some(obd2_identity_operator_key_audit_message(audit)))
}

fn encode_rejected_field(field: Option<Obd2IdentityProvisioningField>) -> u8 {
    match field {
        None => OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_NONE,
        Some(Obd2IdentityProvisioningField::Vin) => OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_VIN,
        Some(Obd2IdentityProvisioningField::CalibrationId) => {
            OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_CALIBRATION_ID
        }
        Some(Obd2IdentityProvisioningField::BoardBuildIdentity) => {
            OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_BOARD_BUILD_IDENTITY
        }
    }
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn obd2_identity_provisioning_arm_tag(
    key: [u8; 32],
    request_id: u32,
    nonce: u32,
    vin_len: u8,
    vin: &[u8; ecu_transport::CAN_OBD2_VIN_LEN],
    calibration_id_len: u8,
    calibration_id: &[u8; ecu_transport::CAN_OBD2_VIN_LEN],
    board_build_identity_len: u8,
    board_build_identity: &[u8; 6],
) -> [u8; 32] {
    let Some(mac) = obd2_identity_provisioning_arm_mac(
        key,
        request_id,
        nonce,
        vin_len,
        vin,
        calibration_id_len,
        calibration_id,
        board_build_identity_len,
        board_build_identity,
    ) else {
        return [0; 32];
    };
    let bytes = mac.finalize().into_bytes();
    let mut tag = [0u8; 32];
    tag.copy_from_slice(&bytes);
    tag
}

#[cfg(feature = "obd2-identity-can-provisioning")]
fn verify_arm_tag(
    key: [u8; 32],
    request_id: u32,
    nonce: u32,
    vin_len: u8,
    vin: &[u8; ecu_transport::CAN_OBD2_VIN_LEN],
    calibration_id_len: u8,
    calibration_id: &[u8; ecu_transport::CAN_OBD2_VIN_LEN],
    board_build_identity_len: u8,
    board_build_identity: &[u8; 6],
    tag: [u8; 32],
) -> bool {
    obd2_identity_provisioning_arm_mac(
        key,
        request_id,
        nonce,
        vin_len,
        vin,
        calibration_id_len,
        calibration_id,
        board_build_identity_len,
        board_build_identity,
    )
    .map(|mac| mac.verify_slice(&tag).is_ok())
    .unwrap_or(false)
}

#[cfg(feature = "obd2-identity-can-provisioning")]
fn obd2_identity_provisioning_arm_mac(
    key: [u8; 32],
    request_id: u32,
    nonce: u32,
    vin_len: u8,
    vin: &[u8; ecu_transport::CAN_OBD2_VIN_LEN],
    calibration_id_len: u8,
    calibration_id: &[u8; ecu_transport::CAN_OBD2_VIN_LEN],
    board_build_identity_len: u8,
    board_build_identity: &[u8; 6],
) -> Option<Hmac<Sha256>> {
    let Ok(mut mac) = Hmac::<Sha256>::new_from_slice(&key) else {
        return None;
    };
    mac.update(b"pipoco:stm32f4:obd2-identity-provisioning:v1");
    mac.update(&request_id.to_le_bytes());
    mac.update(&nonce.to_le_bytes());
    mac.update(&[vin_len]);
    mac.update(vin);
    mac.update(&[calibration_id_len]);
    mac.update(calibration_id);
    mac.update(&[board_build_identity_len]);
    mac.update(board_build_identity);
    Some(mac)
}

#[cfg(feature = "obd2-identity-can-provisioning")]
pub fn obd2_identity_provisioning_key_command_tag(
    current_key: [u8; 32],
    request_id: u32,
    nonce: u32,
    generation: u32,
    revoke: bool,
    key: [u8; 32],
) -> [u8; 32] {
    let Some(mac) = obd2_identity_provisioning_key_command_mac(
        current_key,
        request_id,
        nonce,
        generation,
        revoke,
        key,
    ) else {
        return [0; 32];
    };
    let bytes = mac.finalize().into_bytes();
    let mut tag = [0u8; 32];
    tag.copy_from_slice(&bytes);
    tag
}

#[cfg(feature = "obd2-identity-can-provisioning")]
fn verify_key_command_tag(
    current_key: [u8; 32],
    command: Obd2IdentityProvisioningKeyCommand,
) -> bool {
    obd2_identity_provisioning_key_command_mac(
        current_key,
        command.request_id,
        command.nonce,
        command.generation,
        command.revoke,
        command.key,
    )
    .map(|mac| mac.verify_slice(&command.tag).is_ok())
    .unwrap_or(false)
}

#[cfg(feature = "obd2-identity-can-provisioning")]
fn obd2_identity_provisioning_key_command_mac(
    current_key: [u8; 32],
    request_id: u32,
    nonce: u32,
    generation: u32,
    revoke: bool,
    key: [u8; 32],
) -> Option<Hmac<Sha256>> {
    let Ok(mut mac) = Hmac::<Sha256>::new_from_slice(&current_key) else {
        return None;
    };
    mac.update(b"pipoco:stm32f4:obd2-identity-key-management:v1");
    mac.update(&request_id.to_le_bytes());
    mac.update(&nonce.to_le_bytes());
    mac.update(&generation.to_le_bytes());
    mac.update(&[u8::from(revoke)]);
    mac.update(&key);
    Some(mac)
}

fn parse_hex_key_32(hex: &[u8]) -> Option<[u8; 32]> {
    if hex.len() != 64 {
        return None;
    }
    let mut key = [0u8; 32];
    let mut index = 0usize;
    while index < 32 {
        let hi = hex_nibble(hex[index * 2])?;
        let lo = hex_nibble(hex[index * 2 + 1])?;
        key[index] = (hi << 4) | lo;
        index += 1;
    }
    Some(key)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn reject_too_long<E>(
    field: Obd2IdentityProvisioningField,
    value: &[u8],
    max_len: usize,
) -> Result<(), Obd2IdentityProvisioningError<E>> {
    if value.len() > max_len {
        return Err(Obd2IdentityProvisioningError::TooLong { field, max_len });
    }
    Ok(())
}

impl<E> Obd2IdentityProvisioningError<E> {
    fn field(&self) -> Obd2IdentityProvisioningField {
        match self {
            Self::Empty(field) | Self::TooLong { field, .. } => *field,
            Self::Store(_) => unreachable!("store errors are not produced by field parsing"),
        }
    }
}

fn reject_empty<E>(
    field: Obd2IdentityProvisioningField,
    len: u8,
) -> Result<(), Obd2IdentityProvisioningError<E>> {
    if len == 0 {
        return Err(Obd2IdentityProvisioningError::Empty(field));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Debug, Default)]
    struct MockSink {
        record: Option<Obd2ProvisionedIdentityRecord>,
        fail: bool,
    }

    impl Obd2IdentityProvisioningSink for MockSink {
        type Error = ();

        fn provision_obd2_identity_record(
            &mut self,
            record: Obd2ProvisionedIdentityRecord,
        ) -> Result<(), Self::Error> {
            if self.fail {
                return Err(());
            }
            self.record = Some(record);
            Ok(())
        }
    }

    #[derive(Debug, Default)]
    struct MockAuthorizer {
        authorized: bool,
        calls: usize,
        last_request_id: Option<u32>,
    }

    impl Obd2IdentityProvisioningAuthorizer for MockAuthorizer {
        fn authorize_obd2_identity_provisioning(
            &mut self,
            command: &Obd2IdentityProvisioningCommand<'_>,
        ) -> bool {
            self.calls += 1;
            self.last_request_id = Some(command.request_id);
            self.authorized
        }
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[derive(Debug, Default)]
    struct MockKeyStore {
        generation: Option<u32>,
        record: Option<([u8; 32], u32, bool)>,
        fail: bool,
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[derive(Debug, Default)]
    struct MockNonceStore {
        record: Option<([u8; 32], u32)>,
        fail: bool,
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    impl Obd2IdentityProvisioningArmNonceStore for MockNonceStore {
        type Error = ();

        fn write_obd2_identity_provisioning_arm_nonce(
            &mut self,
            current_key: [u8; 32],
            nonce: u32,
        ) -> Result<(), Self::Error> {
            if self.fail {
                return Err(());
            }
            self.record = Some((current_key, nonce));
            Ok(())
        }
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    impl Obd2IdentityProvisioningKeyStore for MockKeyStore {
        type Error = ();

        fn load_obd2_identity_provisioning_key_generation(&self) -> Option<u32> {
            self.generation
        }

        fn write_obd2_identity_provisioning_key_record(
            &mut self,
            key: [u8; 32],
            generation: u32,
            revoked: bool,
        ) -> Result<(), Self::Error> {
            if self.fail {
                return Err(());
            }
            self.generation = Some(generation);
            self.record = Some((key, generation, revoked));
            Ok(())
        }
    }

    fn operator_command_message(
        request_id: u32,
        vin_src: &[u8],
        calibration_id_src: &[u8],
        board_build_identity_src: &[u8],
    ) -> Message {
        let mut vin = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
        let mut calibration_id = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
        let mut board_build_identity = [0u8; 6];
        vin[..vin_src.len()].copy_from_slice(vin_src);
        calibration_id[..calibration_id_src.len()].copy_from_slice(calibration_id_src);
        board_build_identity[..board_build_identity_src.len()]
            .copy_from_slice(board_build_identity_src);
        Message::Obd2IdentityProvisioningCommand {
            request_id,
            vin_len: vin_src.len() as u8,
            vin,
            calibration_id_len: calibration_id_src.len() as u8,
            calibration_id,
            board_build_identity_len: board_build_identity_src.len() as u8,
            board_build_identity,
        }
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    fn operator_arm_message(
        key: [u8; 32],
        request_id: u32,
        nonce: u32,
        vin_src: &[u8],
        calibration_id_src: &[u8],
        board_build_identity_src: &[u8],
    ) -> Message {
        let mut vin = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
        let mut calibration_id = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
        let mut board_build_identity = [0u8; 6];
        vin[..vin_src.len()].copy_from_slice(vin_src);
        calibration_id[..calibration_id_src.len()].copy_from_slice(calibration_id_src);
        board_build_identity[..board_build_identity_src.len()]
            .copy_from_slice(board_build_identity_src);
        let vin_len = vin_src.len() as u8;
        let calibration_id_len = calibration_id_src.len() as u8;
        let board_build_identity_len = board_build_identity_src.len() as u8;
        let tag = obd2_identity_provisioning_arm_tag(
            key,
            request_id,
            nonce,
            vin_len,
            &vin,
            calibration_id_len,
            &calibration_id,
            board_build_identity_len,
            &board_build_identity,
        );
        Message::Obd2IdentityProvisioningArm {
            request_id,
            nonce,
            vin_len,
            vin,
            calibration_id_len,
            calibration_id,
            board_build_identity_len,
            board_build_identity,
            tag,
        }
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    fn operator_key_command_message(
        current_key: [u8; 32],
        request_id: u32,
        nonce: u32,
        generation: u32,
        revoke: bool,
        key: [u8; 32],
    ) -> Message {
        let tag = obd2_identity_provisioning_key_command_tag(
            current_key,
            request_id,
            nonce,
            generation,
            revoke,
            key,
        );
        Message::Obd2IdentityProvisioningKeyCommand {
            request_id,
            nonce,
            generation,
            revoke,
            key,
            tag,
        }
    }

    #[test]
    fn parse_obd2_identity_provisioning_fields_normalizes_valid_ascii() {
        let record = parse_obd2_identity_provisioning_fields(
            b"vin00000000000001",
            b"cal00000000000001",
            b"m4-a1",
        )
        .unwrap();

        assert_eq!(record.vin_len, ecu_transport::CAN_OBD2_VIN_LEN as u8);
        assert_eq!(&record.vin, b"VIN00000000000001");
        assert_eq!(
            record.calibration_id_len,
            ecu_transport::CAN_OBD2_VIN_LEN as u8
        );
        assert_eq!(&record.calibration_id, b"CAL00000000000001");
        assert_eq!(record.board_build_identity_len, 4);
        assert_eq!(record.board_build_identity, [b'M', b'4', b'A', b'1', 0, 0]);
    }

    #[test]
    fn parse_obd2_identity_provisioning_fields_rejects_invalid_fields() {
        assert_eq!(
            parse_obd2_identity_provisioning_fields(b"", b"cal00000000000001", b"m4-a1"),
            Err(Obd2IdentityProvisioningError::Empty(
                Obd2IdentityProvisioningField::Vin
            ))
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(b"!!!", b"cal00000000000001", b"m4-a1"),
            Err(Obd2IdentityProvisioningError::Empty(
                Obd2IdentityProvisioningField::Vin
            ))
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(b"vin00000000000001", b"!!!", b"m4-a1"),
            Err(Obd2IdentityProvisioningError::Empty(
                Obd2IdentityProvisioningField::CalibrationId
            ))
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(
                b"vin00000000000001",
                b"cal00000000000001",
                b""
            ),
            Err(Obd2IdentityProvisioningError::Empty(
                Obd2IdentityProvisioningField::BoardBuildIdentity
            ))
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(
                b"vin00000000000001",
                b"cal00000000000001",
                b"!!!"
            ),
            Err(Obd2IdentityProvisioningError::Empty(
                Obd2IdentityProvisioningField::BoardBuildIdentity
            ))
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(
                b"vin000000000000010",
                b"cal00000000000001",
                b"m4-a1"
            ),
            Err(Obd2IdentityProvisioningError::TooLong {
                field: Obd2IdentityProvisioningField::Vin,
                max_len: ecu_transport::CAN_OBD2_VIN_LEN,
            })
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(
                b"vin00000000000001",
                b"cal000000000000010",
                b"m4-a1",
            ),
            Err(Obd2IdentityProvisioningError::TooLong {
                field: Obd2IdentityProvisioningField::CalibrationId,
                max_len: ecu_transport::CAN_OBD2_VIN_LEN,
            })
        );
        assert_eq!(
            parse_obd2_identity_provisioning_fields(
                b"vin00000000000001",
                b"cal00000000000001",
                b"m4-a100",
            ),
            Err(Obd2IdentityProvisioningError::TooLong {
                field: Obd2IdentityProvisioningField::BoardBuildIdentity,
                max_len: 6,
            })
        );
    }

    #[cfg(feature = "flash-kv")]
    #[test]
    fn flash_kv_sink_impl_routes_through_real_writer_signature() {
        let _writer: fn(
            &mut crate::store_support::FlashKv,
            Obd2ProvisionedIdentityRecord,
        ) -> Result<(), ecu_calibration::KvError> =
            <crate::store_support::FlashKv as Obd2IdentityProvisioningSink>::provision_obd2_identity_record;
        let _field_invocation: fn(
            &mut crate::store_support::FlashKv,
            &[u8],
            &[u8],
            &[u8],
        ) -> Result<
            (),
            Obd2IdentityProvisioningError<ecu_calibration::KvError>,
        > = provision_flash_kv_obd2_identity_fields;
        let _persisted_audit_invocation: fn(
            &mut MockAuthorizer,
            &mut crate::store_support::FlashKv,
            Obd2IdentityProvisioningCommand<'_>,
        ) -> Result<
            Obd2IdentityProvisioningCommandAudit,
            ecu_calibration::KvError,
        > = provision_flash_kv_obd2_identity_command_with_persisted_audit::<MockAuthorizer>;
    }

    #[test]
    fn provision_obd2_identity_fields_delegates_valid_record_to_sink() {
        let mut sink = MockSink::default();

        provision_obd2_identity_fields(
            &mut sink,
            b"vin00000000000002",
            b"cal00000000000002",
            b"m4-b2",
        )
        .unwrap();

        let record = sink.record.expect("record should be delegated");
        assert_eq!(&record.vin, b"VIN00000000000002");
        assert_eq!(&record.calibration_id, b"CAL00000000000002");
        assert_eq!(record.board_build_identity, [b'M', b'4', b'B', b'2', 0, 0]);
    }

    #[test]
    fn provision_obd2_identity_fields_rejects_before_sink_write() {
        let mut sink = MockSink::default();

        let result = provision_obd2_identity_fields(&mut sink, b"vin00000000000002", b"", b"m4-b2");

        assert_eq!(
            result,
            Err(Obd2IdentityProvisioningError::Empty(
                Obd2IdentityProvisioningField::CalibrationId
            ))
        );
        assert_eq!(sink.record, None);
    }

    #[test]
    fn provision_obd2_identity_fields_returns_store_error() {
        let mut sink = MockSink {
            record: None,
            fail: true,
        };

        assert_eq!(
            provision_obd2_identity_fields(
                &mut sink,
                b"vin00000000000002",
                b"cal00000000000002",
                b"m4-b2",
            ),
            Err(Obd2IdentityProvisioningError::Store(()))
        );
    }

    #[test]
    fn provision_obd2_identity_fields_with_status_reports_accepted_lengths() {
        let mut sink = MockSink::default();

        let status = provision_obd2_identity_fields_with_status(
            &mut sink,
            b"vin00000000000003",
            b"cal00000000000003",
            b"m4-c3",
        );

        assert_eq!(
            status,
            Obd2IdentityProvisioningStatus {
                attempted: true,
                accepted: true,
                rejected_field: None,
                store_failed: false,
                normalized_lengths: Some(Obd2IdentityProvisioningLengths {
                    vin: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                    calibration_id: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                    board_build_identity: 4,
                }),
            }
        );
        assert!(sink.record.is_some());
    }

    #[test]
    fn provision_obd2_identity_fields_with_status_reports_rejections_without_write() {
        let mut sink = MockSink::default();

        assert_eq!(
            provision_obd2_identity_fields_with_status(
                &mut sink,
                b"",
                b"cal00000000000003",
                b"m4-c3",
            )
            .rejected_field,
            Some(Obd2IdentityProvisioningField::Vin)
        );
        assert_eq!(
            provision_obd2_identity_fields_with_status(
                &mut sink,
                b"vin00000000000003",
                b"",
                b"m4-c3",
            )
            .rejected_field,
            Some(Obd2IdentityProvisioningField::CalibrationId)
        );
        assert_eq!(
            provision_obd2_identity_fields_with_status(
                &mut sink,
                b"vin00000000000003",
                b"cal00000000000003",
                b"",
            )
            .rejected_field,
            Some(Obd2IdentityProvisioningField::BoardBuildIdentity)
        );
        assert_eq!(sink.record, None);
    }

    #[test]
    fn provision_obd2_identity_fields_with_status_reports_store_failure_after_validation() {
        let mut sink = MockSink {
            record: None,
            fail: true,
        };

        let status = provision_obd2_identity_fields_with_status(
            &mut sink,
            b"vin00000000000003",
            b"cal00000000000003",
            b"m4-c3",
        );

        assert_eq!(
            status,
            Obd2IdentityProvisioningStatus {
                attempted: true,
                accepted: false,
                rejected_field: None,
                store_failed: true,
                normalized_lengths: Some(Obd2IdentityProvisioningLengths {
                    vin: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                    calibration_id: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                    board_build_identity: 4,
                }),
            }
        );
        assert_eq!(sink.record, None);
    }

    #[test]
    fn provision_obd2_identity_command_with_audit_reports_authorized_success() {
        let mut authorizer = MockAuthorizer {
            authorized: true,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink::default();

        let audit = provision_obd2_identity_command_with_audit(
            &mut authorizer,
            &mut sink,
            Obd2IdentityProvisioningCommand {
                request_id: 7,
                vin: b"vin00000000000004",
                calibration_id: b"cal00000000000004",
                board_build_identity: b"m4-d4",
            },
        );

        assert_eq!(authorizer.calls, 1);
        assert_eq!(authorizer.last_request_id, Some(7));
        assert_eq!(audit.request_id, 7);
        assert!(audit.authorized);
        assert!(!audit.authorization_failed);
        assert_eq!(
            audit.provisioning_status.unwrap().normalized_lengths,
            Some(Obd2IdentityProvisioningLengths {
                vin: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                calibration_id: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                board_build_identity: 4,
            })
        );
        assert!(sink.record.is_some());
    }

    #[test]
    fn provision_obd2_identity_command_with_audit_rejects_unauthorized_without_write() {
        let mut authorizer = MockAuthorizer {
            authorized: false,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink::default();

        let audit = provision_obd2_identity_command_with_audit(
            &mut authorizer,
            &mut sink,
            Obd2IdentityProvisioningCommand {
                request_id: 8,
                vin: b"vin000000000000040",
                calibration_id: b"cal00000000000004",
                board_build_identity: b"m4-d4",
            },
        );

        assert_eq!(
            audit,
            Obd2IdentityProvisioningCommandAudit {
                request_id: 8,
                authorized: false,
                authorization_failed: true,
                provisioning_status: None,
            }
        );
        assert_eq!(authorizer.calls, 1);
        assert_eq!(sink.record, None);
    }

    #[test]
    fn provision_obd2_identity_command_with_audit_reports_authorized_field_rejection() {
        let mut authorizer = MockAuthorizer {
            authorized: true,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink::default();

        let audit = provision_obd2_identity_command_with_audit(
            &mut authorizer,
            &mut sink,
            Obd2IdentityProvisioningCommand {
                request_id: 9,
                vin: b"vin00000000000004",
                calibration_id: b"cal00000000000004",
                board_build_identity: b"",
            },
        );

        assert_eq!(audit.request_id, 9);
        assert!(audit.authorized);
        assert_eq!(
            audit.provisioning_status.unwrap().rejected_field,
            Some(Obd2IdentityProvisioningField::BoardBuildIdentity)
        );
        assert_eq!(sink.record, None);
    }

    #[test]
    fn provision_obd2_identity_command_with_audit_reports_authorized_store_failure() {
        let mut authorizer = MockAuthorizer {
            authorized: true,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink {
            record: None,
            fail: true,
        };

        let audit = provision_obd2_identity_command_with_audit(
            &mut authorizer,
            &mut sink,
            Obd2IdentityProvisioningCommand {
                request_id: 10,
                vin: b"vin00000000000004",
                calibration_id: b"cal00000000000004",
                board_build_identity: b"m4-d4",
            },
        );

        let status = audit.provisioning_status.unwrap();
        assert_eq!(audit.request_id, 10);
        assert!(audit.authorized);
        assert!(status.store_failed);
        assert!(!status.accepted);
        assert_eq!(sink.record, None);
    }

    #[test]
    fn obd2_identity_operator_message_reports_authorized_success() {
        let mut authorizer = MockAuthorizer {
            authorized: true,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink::default();
        let message =
            operator_command_message(11, b"vin00000000000005", b"cal00000000000005", b"m4-e5");

        let response = provision_obd2_identity_operator_message_with_audit(
            &mut authorizer,
            &mut sink,
            &message,
        )
        .unwrap();

        assert_eq!(authorizer.last_request_id, Some(11));
        assert!(sink.record.is_some());
        assert_eq!(
            response,
            Message::Obd2IdentityProvisioningAudit {
                request_id: 11,
                authorized: true,
                authorization_failed: false,
                attempted: true,
                accepted: true,
                store_failed: false,
                rejected_field: OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_NONE,
                vin_len: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                calibration_id_len: ecu_transport::CAN_OBD2_VIN_LEN as u8,
                board_build_identity_len: 4,
            }
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn persisted_key_selection_treats_revoked_record_as_authoritative_absence() {
        assert_eq!(
            Obd2IdentityProvisioningOperatorArm::from_persisted_key_or_compile_time_env(Some((
                Some([0x66; 32]),
                123
            ))),
            Obd2IdentityProvisioningOperatorArm::new_with_optional_key_and_last_arm_nonce(
                Some([0x66; 32]),
                Some(123)
            )
        );
        assert_eq!(
            Obd2IdentityProvisioningOperatorArm::from_persisted_key_or_compile_time_env(Some((
                None, 123
            ))),
            Obd2IdentityProvisioningOperatorArm::new_with_optional_key_and_last_arm_nonce(
                None,
                Some(123)
            )
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_key_command_accepts_valid_monotonic_rotation() {
        let current_key = [0x66; 32];
        let new_key = [0x77; 32];
        let mut operator = Obd2IdentityProvisioningOperatorArm::new_with_key(current_key);
        let mut store = MockKeyStore {
            generation: Some(3),
            ..Default::default()
        };
        let command = obd2_identity_operator_key_command_from_message(
            &operator_key_command_message(current_key, 100, 55, 4, false, new_key),
        )
        .unwrap();

        let audit =
            provision_obd2_identity_key_command_with_audit(&mut operator, &mut store, command);

        assert_eq!(
            audit,
            Obd2IdentityProvisioningKeyAudit {
                request_id: 100,
                authorized: true,
                accepted: true,
                store_failed: false,
                rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE,
                generation: 4,
            }
        );
        assert_eq!(store.record, Some((new_key, 4, false)));
        assert_eq!(operator.current_key(), Some(new_key));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_key_command_rejects_rollback_without_store_write() {
        let current_key = [0x66; 32];
        let new_key = [0x77; 32];
        let mut operator = Obd2IdentityProvisioningOperatorArm::new_with_key(current_key);
        let mut store = MockKeyStore {
            generation: Some(4),
            ..Default::default()
        };
        let command = obd2_identity_operator_key_command_from_message(
            &operator_key_command_message(current_key, 101, 56, 4, false, new_key),
        )
        .unwrap();

        let audit =
            provision_obd2_identity_key_command_with_audit(&mut operator, &mut store, command);

        assert_eq!(
            audit.rejected_reason,
            OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_ROLLBACK
        );
        assert!(audit.authorized);
        assert!(!audit.accepted);
        assert_eq!(store.record, None);
        assert_eq!(operator.current_key(), Some(current_key));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_key_command_rejects_invalid_tag_without_store_write() {
        let current_key = [0x66; 32];
        let new_key = [0x77; 32];
        let mut operator = Obd2IdentityProvisioningOperatorArm::new_with_key(current_key);
        let mut store = MockKeyStore {
            generation: Some(3),
            ..Default::default()
        };
        let command = obd2_identity_operator_key_command_from_message(
            &operator_key_command_message([0x99; 32], 102, 57, 4, false, new_key),
        )
        .unwrap();

        let audit =
            provision_obd2_identity_key_command_with_audit(&mut operator, &mut store, command);

        assert_eq!(
            audit.rejected_reason,
            OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_UNAUTHORIZED
        );
        assert!(!audit.authorized);
        assert!(!audit.accepted);
        assert_eq!(store.record, None);
        assert_eq!(operator.current_key(), Some(current_key));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_key_command_reports_store_failure_without_operator_update() {
        let current_key = [0x66; 32];
        let new_key = [0x77; 32];
        let mut operator = Obd2IdentityProvisioningOperatorArm::new_with_key(current_key);
        let mut store = MockKeyStore {
            generation: Some(3),
            fail: true,
            ..Default::default()
        };
        let command = obd2_identity_operator_key_command_from_message(
            &operator_key_command_message(current_key, 104, 59, 4, false, new_key),
        )
        .unwrap();

        let audit =
            provision_obd2_identity_key_command_with_audit(&mut operator, &mut store, command);

        assert_eq!(
            audit.rejected_reason,
            OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE
        );
        assert!(audit.authorized);
        assert!(!audit.accepted);
        assert!(audit.store_failed);
        assert_eq!(store.record, None);
        assert_eq!(operator.current_key(), Some(current_key));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn key_audit_best_effort_persist_returns_response_when_write_fails() {
        let audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 777,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE,
            generation: 42,
        };
        let mut persist_attempted = false;

        let response = key_audit_response_after_best_effort_persist(audit, |persisted| {
            persist_attempted = true;
            assert_eq!(persisted, audit);
            Err(())
        });

        assert!(persist_attempted);
        assert_eq!(response, obd2_identity_operator_key_audit_message(audit));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn key_lifecycle_status_after_response_marks_unmatched_response_volatile() {
        let response_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 778,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE,
            generation: 43,
        };
        let stale_persisted_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 100,
            authorized: true,
            accepted: true,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE,
            generation: 12,
        };

        let volatile = obd2_identity_key_lifecycle_status_after_operator_response(
            Some(response_audit),
            Some(stale_persisted_audit),
        );
        let durable = obd2_identity_key_lifecycle_status_after_operator_response(
            Some(response_audit),
            Some(response_audit),
        );

        assert!(volatile.present);
        assert!(!volatile.persisted);
        assert!(volatile.store_failed);
        assert_eq!(volatile.request_id, response_audit.request_id);
        assert!(durable.present);
        assert!(durable.persisted);
        assert!(durable.store_failed);
        assert_eq!(durable.request_id, response_audit.request_id);
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn key_lifecycle_status_after_operator_message_parses_response_before_stale_check() {
        let response_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 779,
            authorized: true,
            accepted: false,
            store_failed: true,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_STORE,
            generation: 44,
        };
        let stale_persisted_audit = Obd2IdentityProvisioningKeyAudit {
            request_id: 101,
            authorized: true,
            accepted: true,
            store_failed: false,
            rejected_reason: OBD2_IDENTITY_PROVISIONING_KEY_REJECTED_NONE,
            generation: 13,
        };
        let response = obd2_identity_operator_key_audit_message(response_audit);

        let status = obd2_identity_key_lifecycle_status_after_operator_response(
            obd2_identity_key_audit_from_message(&response),
            Some(stale_persisted_audit),
        );

        assert!(status.present);
        assert!(!status.persisted);
        assert!(status.store_failed);
        assert_eq!(status.request_id, response_audit.request_id);
        assert_eq!(status.generation, response_audit.generation);
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_key_command_accepts_revocation_and_clears_operator_key() {
        let current_key = [0x66; 32];
        let mut operator = Obd2IdentityProvisioningOperatorArm::new_with_key(current_key);
        let mut store = MockKeyStore {
            generation: Some(3),
            ..Default::default()
        };
        let command = obd2_identity_operator_key_command_from_message(
            &operator_key_command_message(current_key, 103, 58, 5, true, current_key),
        )
        .unwrap();

        let audit =
            provision_obd2_identity_key_command_with_audit(&mut operator, &mut store, command);

        assert!(audit.authorized);
        assert!(audit.accepted);
        assert_eq!(store.record, Some((current_key, 5, true)));
        assert_eq!(operator.current_key(), None);
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_message_returns_arm_audit() {
        let key = [0x11; 32];
        let nonce = 99;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let message = operator_arm_message(
            key,
            21,
            nonce,
            b"vin00000000000006",
            b"cal00000000000006",
            b"m4-f6",
        );

        assert_eq!(
            obd2_identity_operator_arm_message(&mut arm, &message),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 21,
                armed: true,
            })
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_allows_one_matching_command() {
        let key = [0x22; 32];
        let nonce = 100;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let mut sink = MockSink::default();
        let arm_message = operator_arm_message(
            key,
            22,
            nonce,
            b"vin00000000000006",
            b"cal00000000000006",
            b"m4-f6",
        );
        let command =
            operator_command_message(22, b"vin00000000000006", b"cal00000000000006", b"m4-f6");

        assert!(obd2_identity_operator_arm_message(&mut arm, &arm_message).is_some());
        let accepted =
            provision_obd2_identity_operator_message_with_audit(&mut arm, &mut sink, &command)
                .unwrap();
        let rejected =
            provision_obd2_identity_operator_message_with_audit(&mut arm, &mut sink, &command)
                .unwrap();

        assert!(matches!(
            accepted,
            Message::Obd2IdentityProvisioningAudit { accepted: true, .. }
        ));
        assert!(matches!(
            rejected,
            Message::Obd2IdentityProvisioningAudit {
                authorized: false,
                authorization_failed: true,
                ..
            }
        ));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_rejects_mismatched_request_id_without_write() {
        let key = [0x33; 32];
        let nonce = 101;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let mut sink = MockSink::default();
        let arm_message = operator_arm_message(
            key,
            23,
            nonce,
            b"vin00000000000006",
            b"cal00000000000006",
            b"m4-f6",
        );
        let command =
            operator_command_message(24, b"vin00000000000006", b"cal00000000000006", b"m4-f6");

        assert!(obd2_identity_operator_arm_message(&mut arm, &arm_message).is_some());
        let response =
            provision_obd2_identity_operator_message_with_audit(&mut arm, &mut sink, &command)
                .unwrap();

        assert_eq!(sink.record, None);
        assert_eq!(
            response,
            Message::Obd2IdentityProvisioningAudit {
                request_id: 24,
                authorized: false,
                authorization_failed: true,
                attempted: false,
                accepted: false,
                store_failed: false,
                rejected_field: OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_NONE,
                vin_len: 0,
                calibration_id_len: 0,
                board_build_identity_len: 0,
            }
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_rejects_tampered_payload_for_same_request_id() {
        let key = [0x34; 32];
        let nonce = 103;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let mut sink = MockSink::default();
        let arm_message = operator_arm_message(
            key,
            27,
            nonce,
            b"vin00000000000006",
            b"cal00000000000006",
            b"m4-f6",
        );
        let tampered_command =
            operator_command_message(27, b"vin00000000000007", b"cal00000000000006", b"m4-f6");

        assert!(matches!(
            obd2_identity_operator_arm_message(&mut arm, &arm_message),
            Some(Message::Obd2IdentityProvisioningArmAudit { armed: true, .. })
        ));
        let response = provision_obd2_identity_operator_message_with_audit(
            &mut arm,
            &mut sink,
            &tampered_command,
        )
        .unwrap();

        assert_eq!(sink.record, None);
        assert!(matches!(
            response,
            Message::Obd2IdentityProvisioningAudit {
                authorized: false,
                authorization_failed: true,
                ..
            }
        ));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_rejects_invalid_tag_and_missing_key() {
        let key = [0x44; 32];
        let nonce = 102;
        let mut invalid = operator_arm_message(
            key,
            25,
            nonce,
            b"vin00000000000006",
            b"cal00000000000006",
            b"m4-f6",
        );
        if let Message::Obd2IdentityProvisioningArm { tag, .. } = &mut invalid {
            *tag = [0x55; 32];
        }
        let missing_key = operator_arm_message(
            key,
            26,
            nonce,
            b"vin00000000000006",
            b"cal00000000000006",
            b"m4-f6",
        );

        assert_eq!(
            obd2_identity_operator_arm_message(
                &mut Obd2IdentityProvisioningOperatorArm::new_with_key(key),
                &invalid
            ),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 25,
                armed: false,
            })
        );
        assert_eq!(
            obd2_identity_operator_arm_message(
                &mut Obd2IdentityProvisioningOperatorArm::new(),
                &missing_key
            ),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 26,
                armed: false,
            })
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_rejects_replayed_nonce() {
        let key = [0x45; 32];
        let nonce = 104;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let message = operator_arm_message(
            key,
            28,
            nonce,
            b"vin00000000000008",
            b"cal00000000000008",
            b"m4-h8",
        );

        assert_eq!(
            obd2_identity_operator_arm_message(&mut arm, &message),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 28,
                armed: true,
            })
        );
        assert_eq!(
            obd2_identity_operator_arm_message(&mut arm, &message),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 28,
                armed: false,
            })
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_rejects_nonce_loaded_from_persistent_record() {
        let key = [0x46; 32];
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_optional_key_and_last_arm_nonce(
            Some(key),
            Some(200),
        );
        let replay = operator_arm_message(
            key,
            29,
            200,
            b"vin00000000000009",
            b"cal00000000000009",
            b"m4-i9",
        );
        let fresh = operator_arm_message(
            key,
            30,
            201,
            b"vin00000000000009",
            b"cal00000000000009",
            b"m4-i9",
        );

        assert_eq!(
            obd2_identity_operator_arm_message(&mut arm, &replay),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 29,
                armed: false,
            })
        );
        assert_eq!(
            obd2_identity_operator_arm_message(&mut arm, &fresh),
            Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 30,
                armed: true,
            })
        );
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_with_nonce_store_persists_before_returning_armed() {
        let key = [0x47; 32];
        let nonce = 301;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let mut store = MockNonceStore::default();
        let message = operator_arm_message(
            key,
            31,
            nonce,
            b"vin00000000000010",
            b"cal00000000000010",
            b"m4-j0",
        );

        assert_eq!(
            obd2_identity_operator_arm_message_with_nonce_store(&mut arm, &mut store, &message),
            Ok(Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 31,
                armed: true,
            }))
        );
        assert_eq!(store.record, Some((key, nonce)));
        assert_eq!(arm.last_arm_nonce(), Some(nonce));
    }

    #[cfg(feature = "obd2-identity-can-provisioning")]
    #[test]
    fn obd2_identity_operator_arm_with_nonce_store_rolls_back_on_write_failure() {
        let key = [0x48; 32];
        let nonce = 302;
        let mut arm = Obd2IdentityProvisioningOperatorArm::new_with_key(key);
        let mut store = MockNonceStore {
            fail: true,
            ..Default::default()
        };
        let message = operator_arm_message(
            key,
            32,
            nonce,
            b"vin00000000000011",
            b"cal00000000000011",
            b"m4-k1",
        );
        let command =
            operator_command_message(32, b"vin00000000000011", b"cal00000000000011", b"m4-k1");

        assert_eq!(
            obd2_identity_operator_arm_message_with_nonce_store(&mut arm, &mut store, &message),
            Ok(Some(Message::Obd2IdentityProvisioningArmAudit {
                request_id: 32,
                armed: false,
            }))
        );
        assert_eq!(store.record, None);
        assert_eq!(arm.last_arm_nonce(), None);
        assert!(!arm.authorize_obd2_identity_provisioning(
            &obd2_identity_operator_command_from_message(&command).unwrap()
        ));
    }

    #[test]
    fn obd2_identity_operator_message_rejects_malformed_lengths_before_authorizer() {
        let mut authorizer = MockAuthorizer {
            authorized: true,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink::default();
        let mut vin = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
        vin.copy_from_slice(b"vin00000000000005");
        let message = Message::Obd2IdentityProvisioningCommand {
            request_id: 12,
            vin_len: ecu_transport::CAN_OBD2_VIN_LEN as u8 + 1,
            vin,
            calibration_id_len: 0,
            calibration_id: [0; ecu_transport::CAN_OBD2_VIN_LEN],
            board_build_identity_len: 0,
            board_build_identity: [0; 6],
        };

        assert_eq!(
            provision_obd2_identity_operator_message_with_audit(
                &mut authorizer,
                &mut sink,
                &message
            ),
            None
        );
        assert_eq!(authorizer.calls, 0);
        assert_eq!(sink.record, None);
    }

    #[test]
    fn obd2_identity_operator_message_reports_unauthorized_without_write() {
        let mut authorizer = MockAuthorizer {
            authorized: false,
            calls: 0,
            last_request_id: None,
        };
        let mut sink = MockSink::default();
        let message =
            operator_command_message(13, b"vin00000000000005", b"cal00000000000005", b"m4-e5");

        let response = provision_obd2_identity_operator_message_with_audit(
            &mut authorizer,
            &mut sink,
            &message,
        )
        .unwrap();

        assert_eq!(authorizer.last_request_id, Some(13));
        assert_eq!(sink.record, None);
        assert_eq!(
            response,
            Message::Obd2IdentityProvisioningAudit {
                request_id: 13,
                authorized: false,
                authorization_failed: true,
                attempted: false,
                accepted: false,
                store_failed: false,
                rejected_field: OBD2_IDENTITY_PROVISIONING_REJECTED_FIELD_NONE,
                vin_len: 0,
                calibration_id_len: 0,
                board_build_identity_len: 0,
            }
        );
    }
}
