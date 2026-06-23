//! Transport abstraction for ECU inter-component communication.
//!
//! This crate owns the transport-agnostic message protocol plus concrete
//! transport adapters that do not need legacy `ecu-compat` state.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "transport-can-fd", not(feature = "transport-can")))]
compile_error!("feature `transport-can-fd` requires `transport-can`");

mod message;
pub use message::Message;

#[cfg(feature = "transport-bbqueue")]
mod bbq;
#[cfg(feature = "transport-bbqueue")]
pub use bbq::BbqTransport;

#[cfg(feature = "transport-can")]
mod can;
#[cfg(feature = "transport-can")]
pub use can::{
    CanBusHealth, CanBusHealthState, CanBusRecoveryState, CanDevice, CanFilterPolicyEntry,
    CanFilterPolicyGroup, CanHeartbeatObservationPolicy, CanMessageClass, CanMessagePriority,
    CanMessageRoute, CanModuleAvailabilityTransition, CanModuleHeartbeatContract,
    CanModuleHeartbeatState, CanModuleHeartbeatStatus, CanModuleRosterChangeDigest,
    CanModuleRosterChangeSet, CanModuleRosterChangeSetEntry, CanModuleRosterEntry,
    CanModuleRosterEventDeltaSummary, CanModuleRosterEventInterpretation, CanModuleRosterEventLog,
    CanModuleRosterEventLogDelta, CanModuleRosterEventLogWatermark, CanModuleRosterEventMeaning,
    CanModuleRosterLatestUnreadEventEntry, CanModuleRosterLatestUnreadEventSet,
    CanModuleRosterLatestUnreadShutdownImpactSummary,
    CanModuleRosterLatestUnreadStartupReadinessSummary, CanModuleRosterLatestUnreadSummary,
    CanModuleRosterRegistry, CanModuleRosterShutdownCoordination,
    CanModuleRosterShutdownCoordinationState, CanModuleRosterSnapshot,
    CanModuleRosterSnapshotEntry, CanModuleRosterStartupReadiness,
    CanModuleRosterStartupReadinessState, CanModuleRosterSummary, CanModuleRosterSweepSummary,
    CanModuleRosterUpdate, CanObd2DtcClearInputs, CanObd2DtcClearResponseError,
    CanObd2DtcClearSurface, CanObd2DtcClearVerdict, CanObd2DtcFreezeFramePayloadMeaning,
    CanObd2DtcFreezeFrameResponseError, CanObd2DtcFreezeFrameResponseSurface,
    CanObd2FlashWriteFaultPhase, CanObd2FlashWriteFaultStatus, CanObd2FreezeFrameSnapshot,
    CanObd2IdentityKeyLifecycleStatus, CanObd2InfoType, CanObd2MultiServiceDispatchError,
    CanObd2MultiServiceDispatchInputs, CanObd2MultiServiceDispatchOutcome,
    CanObd2MultiServiceDispatchSurface, CanObd2MultiServiceDispatchVerdict,
    CanObd2NegativeResponseCode, CanObd2NegativeResponseSurface,
    CanObd2NegativeResponseSurfaceError, CanObd2Pid, CanObd2PidBacking, CanObd2ProjectedPayload,
    CanObd2ReadinessMonitorInputs, CanObd2ReadinessMonitorMeaning,
    CanObd2ReadinessMonitorResponseError, CanObd2ReadinessMonitorSurface,
    CanObd2RequestDispatchError, CanObd2RequestDispatchSurface, CanObd2RequestDispatchVerdict,
    CanObd2ResponseAssemblyError, CanObd2ResponseAssemblySurface, CanObd2ResponseFrame,
    CanObd2ResponseKind, CanObd2SegmentedResponseFrame, CanObd2SegmentedVehicleInfoMeaning,
    CanObd2SegmentedVehicleInfoResponseError, CanObd2SegmentedVehicleInfoResponseSurface,
    CanObd2ServiceDirection, CanObd2ServiceSurface, CanObd2StoredDtc,
    CanObd2SupportedInfoTypeBitmapSurface, CanObd2SupportedInfoTypeProfile,
    CanObd2SupportedPidBitmapSurface, CanObd2SupportedPidDiscoveryResponseError,
    CanObd2SupportedPidDiscoveryResponseSurface, CanObd2SupportedPidProfile,
    CanObd2ValueProjectionEncoding, CanObd2ValueProjectionField, CanObd2ValueProjectionSurface,
    CanObd2VehicleInfoInputs, CanObd2VehicleInfoPayloadMeaning, CanObd2VehicleInfoResponseError,
    CanObd2VehicleInfoResponseSurface, CanRemoteModuleFallbackPolicy, CanSequenceEvent,
    CanSequenceTracking, CanStandardDeviceClass, CanStandardDeviceProfile,
    CanStandardDeviceSupport, CanTransport, CAN_FILTER_POLICY_MAP,
    CAN_HEARTBEAT_STARTUP_WINDOW_SECONDS, CAN_HEARTBEAT_TIMEOUT_MS, CAN_MESSAGE_CLASS_COUNT,
    CAN_MODULE_ROSTER_CAPACITY, CAN_MODULE_ROSTER_CHANGE_SET_CAPACITY,
    CAN_MODULE_ROSTER_EVENT_LOG_CAPACITY, CAN_OBD2_FLASH_WRITE_FAULT_INFO_TYPE_ID,
    CAN_OBD2_FLASH_WRITE_FAULT_PAYLOAD_LEN, CAN_OBD2_IDENTITY_KEY_LIFECYCLE_INFO_TYPE_ID,
    CAN_OBD2_IDENTITY_KEY_LIFECYCLE_PAYLOAD_LEN, CAN_OBD2_SEGMENTED_RESPONSE_CAPACITY,
    CAN_OBD2_SUPPORTED_INFO_TYPE_CATALOG, CAN_OBD2_SUPPORTED_INFO_TYPE_COUNT,
    CAN_OBD2_SUPPORTED_PID_CATALOG, CAN_OBD2_SUPPORTED_PID_COUNT, CAN_OBD2_VIN_LEN, CAN_ROUTE_MAP,
    CAN_STANDARD_DEVICE_PROFILE_COUNT, CAN_STANDARD_DEVICE_PROFILE_MAP,
};

/// Transport-agnostic message passing interface.
pub trait Transport {
    /// Send a message.
    fn send(&mut self, message: &Message) -> Result<(), TransportError>;

    /// Try to receive a message without blocking.
    fn try_receive(&mut self) -> Option<Message>;

    /// Poll the transport layer for pending work.
    fn poll(&mut self);

    /// Flush pending transmissions.
    fn flush(&mut self) -> Result<(), TransportError>;

    /// Get transport statistics.
    fn stats(&self) -> TransportStats;

    /// Check if transport is connected/ready.
    fn is_ready(&self) -> bool;
}

/// Transport layer errors.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum TransportError {
    /// Buffer full - cannot queue more messages.
    BufferFull,
    /// Message too large for this transport.
    MessageTooLarge,
    /// Serialization failed.
    SerializationFailed,
    /// Deserialization failed.
    DeserializationFailed,
    /// Hardware error.
    HardwareError,
    /// Transport not connected or ready.
    NotReady,
}

/// Transport statistics for monitoring.
#[derive(Debug, Copy, Clone, Default)]
pub struct TransportStats {
    pub tx_count: u32,
    pub rx_count: u32,
    pub tx_errors: u32,
    pub rx_errors: u32,
    pub tx_buffer_usage: u8,
    pub rx_buffer_usage: u8,
    pub avg_latency_us: Option<u32>,
}
