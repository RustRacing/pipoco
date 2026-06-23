use ecu_board_api::{
    BoardSensorSnapshot, BoardSensorSnapshotCapture, BoardSensorValidityFlags, CaptureSample,
    CommonFaultTransitionEventId, CommonFaultTransitionTelemetry,
};
use ecu_calibration::CalibrationPackageIdentity;
use ecu_compat::compat::EcuState;
use ecu_domain::{
    diag::{DiagClearSummary, DiagCode, DiagEvent, DiagSource},
    FaultCode, Micros,
};
use ecu_runtime::FaultState;
use ecu_spec::{
    fault_event_for_clear, fault_event_from_state, SpecCancelReason, SpecFaultAction,
    SpecFaultCode, SpecFaultEvent, SpecFaultSeverity, SpecFaultState,
};
use ecu_transport::{
    CanObd2DtcClearInputs, CanObd2DtcClearResponseError, CanObd2DtcClearSurface,
    CanObd2IdentityKeyLifecycleStatus, CanObd2MultiServiceDispatchError,
    CanObd2MultiServiceDispatchInputs, CanObd2MultiServiceDispatchOutcome,
    CanObd2MultiServiceDispatchSurface, CanObd2MultiServiceDispatchVerdict,
    CanObd2NegativeResponseCode, CanObd2ReadinessMonitorInputs, CanObd2ResponseFrame,
    CanObd2SegmentedVehicleInfoResponseSurface, CanObd2VehicleInfoInputs, Message, Transport,
    TransportError,
};
use ecu_ts::pages::DIAG_LOG_ENTRY_COUNT;

#[derive(Debug, Clone, PartialEq)]
pub struct Obd2DtcClearExecutionSurface {
    pub clear_summary: DiagClearSummary,
    pub transport: CanObd2DtcClearSurface,
}

pub const OBD2_STORED_DTC_CAP: usize = 3;
pub const OBD2_LIVE_DIAG_EVENT_INGRESS_CAP: usize = 2;
pub const OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES: usize = 92;
pub const OBD2_RETAINED_HISTORY_SIDECAR_HEADER_BYTES: usize = 8;
pub const OBD2_RETAINED_HISTORY_SIDECAR_BYTES: usize =
    OBD2_RETAINED_HISTORY_SIDECAR_HEADER_BYTES + OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES;
const OBD2_DEFAULT_CLT_C10: i16 = 200;
const OBD2_DEFAULT_IAT_C10: i16 = 250;
const OBD2_DEFAULT_VBATT_MV: u16 = 12_500;
const OBD2_DEFAULT_LAMBDA_X100: u8 = 100;
const OBD2_SNAPSHOT_DIAG_EVENT_BYTES: usize = 20;
const OBD2_SNAPSHOT_SENSOR_DATA_BYTES: usize = 12;
const OBD2_RETAINED_HISTORY_SIDECAR_MAGIC: u32 = 0x3244_424F;
const OBD2_RETAINED_HISTORY_SIDECAR_VERSION: u16 = 1;

pub fn obd2_current_data_from_board_inputs(
    logical_sensor_capture: Option<BoardSensorSnapshotCapture>,
    capture_sample: Option<CaptureSample>,
) -> Message {
    if let Some(capture) = logical_sensor_capture {
        obd2_sensor_message_from_snapshot(capture.snapshot, capture.at_us.get())
    } else if let Some(sample) = capture_sample {
        obd2_default_sensor_message(sample.load_kpa10.get(), sample.at_us.get())
    } else {
        obd2_default_sensor_message(0, 0)
    }
}

pub fn obd2_vehicle_info_from_identity_signature(signature: &[u8]) -> CanObd2VehicleInfoInputs {
    Obd2VehicleIdentity::from_signature(signature).into_transport_inputs()
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Obd2VehicleIdentity {
    pub ecu_name_len: u8,
    pub ecu_name: [u8; 6],
    pub vin_len: u8,
    pub vin: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    pub calibration_id_len: u8,
    pub calibration_id: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct ProvisionedObd2Identity {
    pub vin_len: u8,
    pub vin: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    pub calibration_id_len: u8,
    pub calibration_id: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    pub board_build_identity_len: u8,
    pub board_build_identity: [u8; 6],
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Obd2ProvisionedIdentityRecord {
    pub vin_len: u8,
    pub vin: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    pub calibration_id_len: u8,
    pub calibration_id: [u8; ecu_transport::CAN_OBD2_VIN_LEN],
    pub board_build_identity_len: u8,
    pub board_build_identity: [u8; 6],
}

impl Obd2VehicleIdentity {
    pub fn from_signature(signature: &[u8]) -> Self {
        let mut identity = Self::empty();
        let normalized = normalize_ascii_identity(signature);
        let len = normalized.1;
        if len == 0 {
            return Self::default();
        }
        let ecu_name_cap = identity.ecu_name.len();
        identity.ecu_name_len = len.min(ecu_name_cap) as u8;
        identity
            .ecu_name
            .copy_from_slice(&normalized.0[..ecu_name_cap]);
        identity.vin_len = len as u8;
        identity.vin = normalized.0;
        identity.calibration_id_len = len as u8;
        identity.calibration_id = normalized.0;
        identity
    }

    pub fn from_calibration_identity(calibration: CalibrationPackageIdentity) -> Self {
        let mut identity = Self::from_signature(ecu_ts::TS_SIGNATURE);
        let mut calibration_id = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
        calibration_id[0] = b'C';
        write_hex_u32(calibration.active_revision.get(), &mut calibration_id[1..9]);
        write_hex_u32(calibration.checksum.get(), &mut calibration_id[9..17]);
        identity.vin_len = ecu_transport::CAN_OBD2_VIN_LEN as u8;
        identity.vin = calibration_id;
        identity.calibration_id_len = ecu_transport::CAN_OBD2_VIN_LEN as u8;
        identity.calibration_id = calibration_id;
        identity
    }

    pub fn with_provisioned_identity(mut self, provisioned: ProvisionedObd2Identity) -> Self {
        if provisioned.board_build_identity_len > 0 {
            self.ecu_name_len = provisioned
                .board_build_identity_len
                .min(self.ecu_name.len() as u8);
            self.ecu_name = provisioned.board_build_identity;
        }
        if provisioned.vin_len > 0 {
            self.vin_len = provisioned
                .vin_len
                .min(ecu_transport::CAN_OBD2_VIN_LEN as u8);
            self.vin = provisioned.vin;
        }
        if provisioned.calibration_id_len > 0 {
            self.calibration_id_len = provisioned
                .calibration_id_len
                .min(ecu_transport::CAN_OBD2_VIN_LEN as u8);
            self.calibration_id = provisioned.calibration_id;
        }
        self
    }

    pub const fn into_transport_inputs(self) -> CanObd2VehicleInfoInputs {
        CanObd2VehicleInfoInputs {
            ecu_name_len: self.ecu_name_len,
            ecu_name: self.ecu_name,
            vin_len: self.vin_len,
            vin: self.vin,
            calibration_id_len: self.calibration_id_len,
            calibration_id: self.calibration_id,
            identity_key_lifecycle: None,
            flash_write_fault: None,
        }
    }

    const fn empty() -> Self {
        Self {
            ecu_name_len: 0,
            ecu_name: [0; 6],
            vin_len: 0,
            vin: [0; ecu_transport::CAN_OBD2_VIN_LEN],
            calibration_id_len: 0,
            calibration_id: [0; ecu_transport::CAN_OBD2_VIN_LEN],
        }
    }
}

impl ProvisionedObd2Identity {
    pub fn from_ascii(
        vin: Option<&[u8]>,
        calibration_id: Option<&[u8]>,
        board_build_identity: Option<&[u8]>,
    ) -> Self {
        Obd2ProvisionedIdentityRecord::from_ascii(vin, calibration_id, board_build_identity)
            .into_provisioned_identity()
    }
}

impl Obd2ProvisionedIdentityRecord {
    pub fn from_ascii(
        vin: Option<&[u8]>,
        calibration_id: Option<&[u8]>,
        board_build_identity: Option<&[u8]>,
    ) -> Self {
        let mut identity = Self::default();
        if let Some(vin) = vin {
            let normalized = normalize_ascii_identity(vin);
            identity.vin_len = normalized.1 as u8;
            identity.vin = normalized.0;
        }
        if let Some(calibration_id) = calibration_id {
            let normalized = normalize_ascii_identity(calibration_id);
            identity.calibration_id_len = normalized.1 as u8;
            identity.calibration_id = normalized.0;
        }
        if let Some(board_build_identity) = board_build_identity {
            let normalized = normalize_ascii_identity(board_build_identity);
            let board_build_identity_cap = identity.board_build_identity.len();
            identity.board_build_identity_len = normalized.1.min(board_build_identity_cap) as u8;
            identity
                .board_build_identity
                .copy_from_slice(&normalized.0[..board_build_identity_cap]);
        }
        identity
    }

    pub fn into_provisioned_identity(self) -> ProvisionedObd2Identity {
        ProvisionedObd2Identity {
            vin_len: self.vin_len.min(ecu_transport::CAN_OBD2_VIN_LEN as u8),
            vin: self.vin,
            calibration_id_len: self
                .calibration_id_len
                .min(ecu_transport::CAN_OBD2_VIN_LEN as u8),
            calibration_id: self.calibration_id,
            board_build_identity_len: self.board_build_identity_len.min(6),
            board_build_identity: self.board_build_identity,
        }
    }
}

impl Default for ProvisionedObd2Identity {
    fn default() -> Self {
        Self {
            vin_len: 0,
            vin: [0; ecu_transport::CAN_OBD2_VIN_LEN],
            calibration_id_len: 0,
            calibration_id: [0; ecu_transport::CAN_OBD2_VIN_LEN],
            board_build_identity_len: 0,
            board_build_identity: [0; 6],
        }
    }
}

impl Default for Obd2ProvisionedIdentityRecord {
    fn default() -> Self {
        Self {
            vin_len: 0,
            vin: [0; ecu_transport::CAN_OBD2_VIN_LEN],
            calibration_id_len: 0,
            calibration_id: [0; ecu_transport::CAN_OBD2_VIN_LEN],
            board_build_identity_len: 0,
            board_build_identity: [0; 6],
        }
    }
}

impl Default for Obd2VehicleIdentity {
    fn default() -> Self {
        let default = CanObd2VehicleInfoInputs::default_identity();
        Self {
            ecu_name_len: default.ecu_name_len,
            ecu_name: default.ecu_name,
            vin_len: default.vin_len,
            vin: default.vin,
            calibration_id_len: default.vin_len,
            calibration_id: default.vin,
        }
    }
}

fn normalize_ascii_identity(input: &[u8]) -> ([u8; ecu_transport::CAN_OBD2_VIN_LEN], usize) {
    let mut out = [0u8; ecu_transport::CAN_OBD2_VIN_LEN];
    let mut len = 0usize;
    for byte in input.iter().copied() {
        if len >= out.len() {
            break;
        }
        let normalized = match byte {
            b'a'..=b'z' => byte - 32,
            b'A'..=b'Z' | b'0'..=b'9' => byte,
            _ => continue,
        };
        out[len] = normalized;
        len += 1;
    }
    (out, len)
}

fn write_hex_u32(mut value: u32, out: &mut [u8]) {
    for slot in out.iter_mut().rev().take(8) {
        let nibble = (value & 0x0f) as u8;
        *slot = match nibble {
            0..=9 => b'0' + nibble,
            _ => b'A' + (nibble - 10),
        };
        value >>= 4;
    }
}

fn obd2_sensor_message_from_snapshot(snapshot: BoardSensorSnapshot, timestamp_us: u32) -> Message {
    Message::SensorData {
        map_kpa_x10: snapshot.map_kpa10.get(),
        tps_percent: obd2_percent_from_tps_x100(snapshot.tps_x100),
        iat_offset: obd2_temp_offset_from_c10(snapshot.iat_c10, OBD2_DEFAULT_IAT_C10),
        clt_offset: obd2_temp_offset_from_c10(snapshot.clt_c10, OBD2_DEFAULT_CLT_C10),
        voltage_x10: obd2_decivolts_from_mv(snapshot.vbatt_mv, OBD2_DEFAULT_VBATT_MV),
        lambda_x100: obd2_lambda_from_snapshot(snapshot),
        flags: 0,
        timestamp_us,
    }
}

fn obd2_default_sensor_message(map_kpa_x10: u16, timestamp_us: u32) -> Message {
    Message::SensorData {
        map_kpa_x10,
        tps_percent: 0,
        iat_offset: obd2_temp_offset_from_c10(0, OBD2_DEFAULT_IAT_C10),
        clt_offset: obd2_temp_offset_from_c10(0, OBD2_DEFAULT_CLT_C10),
        voltage_x10: obd2_decivolts_from_mv(0, OBD2_DEFAULT_VBATT_MV),
        lambda_x100: OBD2_DEFAULT_LAMBDA_X100,
        flags: 0,
        timestamp_us,
    }
}

fn obd2_percent_from_tps_x100(tps_x100: u16) -> u8 {
    (tps_x100 / 100).min(100) as u8
}

fn obd2_temp_offset_from_c10(temp_c10: i16, fallback_c10: i16) -> u8 {
    let source_c10 = if temp_c10 == 0 {
        fallback_c10
    } else {
        temp_c10
    };
    let degrees_c = source_c10 / 10;
    (degrees_c + 40).clamp(0, 255) as u8
}

fn obd2_decivolts_from_mv(millivolts: u16, fallback_mv: u16) -> u8 {
    let source_mv = if millivolts == 0 {
        fallback_mv
    } else {
        millivolts
    };
    ((source_mv + 50) / 100).min(u8::MAX as u16) as u8
}

fn obd2_lambda_from_snapshot(snapshot: BoardSensorSnapshot) -> u8 {
    if snapshot.validity.contains(BoardSensorValidityFlags::LAMBDA) {
        snapshot.lambda_x100.get().min(u8::MAX as u16) as u8
    } else {
        OBD2_DEFAULT_LAMBDA_X100
    }
}

fn obd2_sensor_timestamp(current_data_value_source: &Message) -> Option<Micros> {
    match current_data_value_source {
        Message::SensorData { timestamp_us, .. } => Some(Micros::new(*timestamp_us)),
        _ => None,
    }
}

fn obd2_diag_context_and_source(
    fault: FaultCode,
    current_data_value_source: &Message,
) -> (Option<u32>, DiagSource) {
    match (fault, current_data_value_source) {
        (FaultCode::SensorOutOfRange, Message::SensorData { map_kpa_x10, .. }) => {
            (Some(u32::from(*map_kpa_x10)), DiagSource::Sensor)
        }
        (FaultCode::SafetyCut, Message::SensorData { map_kpa_x10, .. }) => {
            (Some(u32::from(*map_kpa_x10)), DiagSource::Safety)
        }
        (FaultCode::ActuatorFault, Message::SensorData { tps_percent, .. }) => {
            (Some(u32::from(*tps_percent)), DiagSource::Sensor)
        }
        (FaultCode::SyncLoss, _) => (None, DiagSource::Trigger),
        (FaultCode::CalibrationInvalid, _) => (None, DiagSource::User),
        (FaultCode::SensorOutOfRange, _) => (None, DiagSource::Sensor),
        (FaultCode::SafetyCut, _) => (None, DiagSource::Safety),
        (FaultCode::ActuatorFault, _) => (None, DiagSource::Sensor),
        (FaultCode::None, _) => (None, DiagSource::Safety),
    }
}

pub fn obd2_diag_log_events_array<const N: usize>(
    diag_log: &ecu_domain::diag::DiagLog<N>,
) -> [Option<DiagEvent>; DIAG_LOG_ENTRY_COUNT] {
    let mut events = [None; DIAG_LOG_ENTRY_COUNT];
    let count = N.min(DIAG_LOG_ENTRY_COUNT);
    for (entry, slot) in events.iter_mut().take(count).zip(diag_log.events.iter()) {
        *entry = *slot;
    }
    events
}

/// Shared retained OBD-II diagnostic history above target-local transport owners.
///
/// This surface intentionally keeps only the bounded state the OBD-II helper
/// stack needs right now: latest current-data projection, retained stored DTCs,
/// and one retained freeze-frame snapshot.
#[derive(Debug, Clone, PartialEq)]
pub struct Obd2RetainedDiagnosticHistory {
    current_data_value_source: Message,
    freeze_frame_value_source: Option<Message>,
    stored_dtcs: [DiagCode; OBD2_STORED_DTC_CAP],
    stored_dtc_count: usize,
    freeze_frame_dtc: Option<DiagCode>,
    current_diag_event: Option<DiagEvent>,
    freeze_frame_event: Option<DiagEvent>,
    last_fault: FaultCode,
    last_observed_diag_code: Option<DiagCode>,
    last_observed_diag_timestamp_us: u32,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Obd2RetainedDiagnosticHistorySnapshot {
    pub current_data_value_source: Message,
    pub freeze_frame_value_source: Option<Message>,
    pub stored_dtcs: [DiagCode; OBD2_STORED_DTC_CAP],
    pub stored_dtc_count: u8,
    pub freeze_frame_dtc: Option<DiagCode>,
    pub current_diag_event: Option<DiagEvent>,
    pub freeze_frame_event: Option<DiagEvent>,
    pub last_fault: FaultCode,
    pub last_observed_diag_code: Option<DiagCode>,
    pub last_observed_diag_timestamp_us: u32,
}

pub trait Obd2RetainedHistoryStore {
    type Error;

    fn load_retained_obd2_history_snapshot(&self) -> Option<Obd2RetainedDiagnosticHistorySnapshot>;

    fn save_retained_obd2_history_snapshot(
        &mut self,
        snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    ) -> Result<(), Self::Error>;
}

pub trait Obd2RetainedHistoryOwner {
    fn obd2_retained_history_snapshot(&self) -> Obd2RetainedDiagnosticHistorySnapshot;

    fn restore_obd2_retained_history(&mut self, snapshot: &Obd2RetainedDiagnosticHistorySnapshot);
}

#[derive(Debug, Clone, PartialEq)]
pub struct Obd2RetainedHistoryPersistenceState {
    last_persisted_snapshot: Obd2RetainedDiagnosticHistorySnapshot,
}

impl Obd2RetainedHistoryPersistenceState {
    pub fn restore_from_store<O, S>(owner: &mut O, store: &S) -> Self
    where
        O: Obd2RetainedHistoryOwner,
        S: Obd2RetainedHistoryStore,
    {
        if let Some(snapshot) = store.load_retained_obd2_history_snapshot() {
            owner.restore_obd2_retained_history(&snapshot);
        }
        Self {
            last_persisted_snapshot: owner.obd2_retained_history_snapshot(),
        }
    }

    pub fn persist_if_changed<O, S>(&mut self, owner: &O, store: &mut S) -> Result<bool, S::Error>
    where
        O: Obd2RetainedHistoryOwner,
        S: Obd2RetainedHistoryStore,
    {
        let snapshot = owner.obd2_retained_history_snapshot();
        if snapshot == self.last_persisted_snapshot {
            return Ok(false);
        }
        store.save_retained_obd2_history_snapshot(&snapshot)?;
        self.last_persisted_snapshot = snapshot;
        Ok(true)
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Obd2RetainedHistoryFlashRewrite<Pages> {
    pub pages: Pages,
    pub snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Obd2RetainedHistoryPageUpdateError {
    UnknownKey,
    InvalidLength,
}

pub trait Obd2RetainedHistoryPagesMut {
    fn fuel_mut(&mut self) -> &mut [u8];
    fn ign_mut(&mut self) -> &mut [u8];
    fn angles_mut(&mut self) -> &mut [u8];
}

impl Obd2RetainedHistoryPagesMut for crate::kv::ab::SlotContents {
    fn fuel_mut(&mut self) -> &mut [u8] {
        &mut self.fuel
    }

    fn ign_mut(&mut self) -> &mut [u8] {
        &mut self.ign
    }

    fn angles_mut(&mut self) -> &mut [u8] {
        &mut self.angles
    }
}

pub fn apply_retained_history_page_update<Pages: Obd2RetainedHistoryPagesMut>(
    pages: &mut Pages,
    key: &[u8],
    data: &[u8],
) -> Result<(), Obd2RetainedHistoryPageUpdateError> {
    let target = match key {
        b"fuel" => pages.fuel_mut(),
        b"ign" => pages.ign_mut(),
        b"angles" => pages.angles_mut(),
        _ => return Err(Obd2RetainedHistoryPageUpdateError::UnknownKey),
    };
    if data.len() != target.len() {
        return Err(Obd2RetainedHistoryPageUpdateError::InvalidLength);
    }
    target.copy_from_slice(data);
    Ok(())
}

pub fn prepare_retained_history_snapshot_rewrite<Pages>(
    pages: Pages,
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
) -> Obd2RetainedHistoryFlashRewrite<Pages> {
    Obd2RetainedHistoryFlashRewrite {
        pages,
        snapshot: Some(snapshot.clone()),
    }
}

pub fn prepare_retained_history_preserved_page_rewrite<Pages, E>(
    mut pages: Pages,
    preserved_snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
    apply_update: impl FnOnce(&mut Pages) -> Result<(), E>,
) -> Result<Obd2RetainedHistoryFlashRewrite<Pages>, E> {
    apply_update(&mut pages)?;
    Ok(Obd2RetainedHistoryFlashRewrite {
        pages,
        snapshot: preserved_snapshot,
    })
}

pub fn persist_retained_history_flash_rewrite<Pages, E>(
    current_pages: Pages,
    current_snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
    prepare: impl FnOnce(
        Pages,
        Option<Obd2RetainedDiagnosticHistorySnapshot>,
    ) -> Result<Obd2RetainedHistoryFlashRewrite<Pages>, E>,
    commit: impl FnOnce(Obd2RetainedHistoryFlashRewrite<Pages>) -> Result<(), E>,
) -> Result<(), E> {
    let rewrite = prepare(current_pages, current_snapshot)?;
    commit(rewrite)
}

fn encode_snapshot_sensor_message(message: &Message, out: &mut [u8]) -> Option<usize> {
    let Message::SensorData {
        map_kpa_x10,
        tps_percent,
        iat_offset,
        clt_offset,
        voltage_x10,
        lambda_x100,
        flags,
        timestamp_us,
    } = message
    else {
        return None;
    };
    if out.len() < OBD2_SNAPSHOT_SENSOR_DATA_BYTES {
        return None;
    }
    out[0..2].copy_from_slice(&map_kpa_x10.to_le_bytes());
    out[2] = *tps_percent;
    out[3] = *iat_offset;
    out[4] = *clt_offset;
    out[5] = *voltage_x10;
    out[6] = *lambda_x100;
    out[7] = *flags;
    out[8..12].copy_from_slice(&timestamp_us.to_le_bytes());
    Some(OBD2_SNAPSHOT_SENSOR_DATA_BYTES)
}

fn decode_snapshot_sensor_message(input: &[u8]) -> Option<Message> {
    if input.len() < OBD2_SNAPSHOT_SENSOR_DATA_BYTES {
        return None;
    }
    Some(Message::SensorData {
        map_kpa_x10: u16::from_le_bytes([input[0], input[1]]),
        tps_percent: input[2],
        iat_offset: input[3],
        clt_offset: input[4],
        voltage_x10: input[5],
        lambda_x100: input[6],
        flags: input[7],
        timestamp_us: u32::from_le_bytes([input[8], input[9], input[10], input[11]]),
    })
}

fn encode_snapshot_diag_source(source: DiagSource) -> u8 {
    match source {
        DiagSource::Sensor => 1,
        DiagSource::Trigger => 2,
        DiagSource::Scheduler => 3,
        DiagSource::Safety => 4,
        DiagSource::User => 5,
    }
}

fn decode_snapshot_diag_source(raw: u8) -> Option<DiagSource> {
    match raw {
        1 => Some(DiagSource::Sensor),
        2 => Some(DiagSource::Trigger),
        3 => Some(DiagSource::Scheduler),
        4 => Some(DiagSource::Safety),
        5 => Some(DiagSource::User),
        _ => None,
    }
}

fn decode_snapshot_diag_code(raw: u8) -> Option<DiagCode> {
    match raw {
        1 => Some(DiagCode::MapRange),
        2 => Some(DiagCode::TpsRange),
        3 => Some(DiagCode::CamMissing),
        4 => Some(DiagCode::LowVoltage),
        5 => Some(DiagCode::Overvoltage),
        6 => Some(DiagCode::MapFailureHighLoad),
        7 => Some(DiagCode::TpsMapPlausibility),
        8 => Some(DiagCode::KnockDetected),
        9 => Some(DiagCode::PersistCrcFault),
        10 => Some(DiagCode::OilPressureLow),
        11 => Some(DiagCode::FuelPressureLow),
        12 => Some(DiagCode::LambdaInvalid),
        _ => None,
    }
}

const fn encode_snapshot_fault_code(fault: FaultCode) -> u8 {
    match fault {
        FaultCode::None => 0,
        FaultCode::SyncLoss => 1,
        FaultCode::SensorOutOfRange => 2,
        FaultCode::CalibrationInvalid => 3,
        FaultCode::SafetyCut => 4,
        FaultCode::ActuatorFault => 5,
    }
}

const fn decode_snapshot_fault_code(raw: u8) -> Option<FaultCode> {
    match raw {
        0 => Some(FaultCode::None),
        1 => Some(FaultCode::SyncLoss),
        2 => Some(FaultCode::SensorOutOfRange),
        3 => Some(FaultCode::CalibrationInvalid),
        4 => Some(FaultCode::SafetyCut),
        5 => Some(FaultCode::ActuatorFault),
        _ => None,
    }
}

fn encode_snapshot_diag_event(event: Option<DiagEvent>, out: &mut [u8]) -> Option<usize> {
    if out.len() < OBD2_SNAPSHOT_DIAG_EVENT_BYTES {
        return None;
    }
    match event {
        Some(event) => {
            out[0] = 1;
            out[1] = event.code.to_u8();
            out[2..6].copy_from_slice(&event.timestamp.get().to_le_bytes());
            out[6] = encode_snapshot_diag_source(event.source);
            match event.context {
                Some(context) => {
                    out[7] = 1;
                    out[8..12].copy_from_slice(&context.to_le_bytes());
                }
                None => {
                    out[7] = 0;
                    out[8..12].copy_from_slice(&0u32.to_le_bytes());
                }
            }
            out[12..16].copy_from_slice(&event.start_us.to_le_bytes());
            out[16..20].copy_from_slice(&event.end_us.to_le_bytes());
        }
        None => out[..OBD2_SNAPSHOT_DIAG_EVENT_BYTES].fill(0),
    }
    Some(OBD2_SNAPSHOT_DIAG_EVENT_BYTES)
}

fn decode_snapshot_diag_event(input: &[u8]) -> Option<Option<DiagEvent>> {
    if input.len() < OBD2_SNAPSHOT_DIAG_EVENT_BYTES {
        return None;
    }
    if input[0] == 0 {
        return Some(None);
    }
    let code = decode_snapshot_diag_code(input[1])?;
    let timestamp = Micros::new(u32::from_le_bytes([input[2], input[3], input[4], input[5]]));
    let source = decode_snapshot_diag_source(input[6])?;
    let context = if input[7] == 0 {
        None
    } else {
        Some(u32::from_le_bytes([
            input[8], input[9], input[10], input[11],
        ]))
    };
    Some(Some(DiagEvent {
        code,
        timestamp,
        source,
        context,
        start_us: u32::from_le_bytes([input[12], input[13], input[14], input[15]]),
        end_us: u32::from_le_bytes([input[16], input[17], input[18], input[19]]),
    }))
}

pub fn encode_obd2_retained_history_snapshot(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    out: &mut [u8],
) -> Option<usize> {
    if out.len() < OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES {
        return None;
    }
    let mut cursor = 0usize;
    cursor +=
        encode_snapshot_sensor_message(&snapshot.current_data_value_source, &mut out[cursor..])?;
    match &snapshot.freeze_frame_value_source {
        Some(message) => {
            out[cursor] = 1;
            cursor += 1;
            cursor += encode_snapshot_sensor_message(message, &mut out[cursor..])?;
        }
        None => {
            out[cursor] = 0;
            cursor += 1;
            out[cursor..cursor + OBD2_SNAPSHOT_SENSOR_DATA_BYTES].fill(0);
            cursor += OBD2_SNAPSHOT_SENSOR_DATA_BYTES;
        }
    }
    let mut index = 0usize;
    while index < OBD2_STORED_DTC_CAP {
        out[cursor + index] = snapshot.stored_dtcs[index].to_u8();
        index += 1;
    }
    cursor += OBD2_STORED_DTC_CAP;
    out[cursor] = snapshot.stored_dtc_count;
    cursor += 1;
    out[cursor] = snapshot.freeze_frame_dtc.map(DiagCode::to_u8).unwrap_or(0);
    cursor += 1;
    cursor += encode_snapshot_diag_event(snapshot.current_diag_event, &mut out[cursor..])?;
    cursor += encode_snapshot_diag_event(snapshot.freeze_frame_event, &mut out[cursor..])?;
    out[cursor] = encode_snapshot_fault_code(snapshot.last_fault);
    cursor += 1;
    out[cursor] = snapshot
        .last_observed_diag_code
        .map(DiagCode::to_u8)
        .unwrap_or(0);
    cursor += 1;
    out[cursor..cursor + 4]
        .copy_from_slice(&snapshot.last_observed_diag_timestamp_us.to_le_bytes());
    cursor += 4;
    out[cursor..OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES].fill(0);
    Some(OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES)
}

pub fn decode_obd2_retained_history_snapshot(
    input: &[u8],
) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
    if input.len() < OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES {
        return None;
    }
    let mut cursor = 0usize;
    let current_data_value_source =
        decode_snapshot_sensor_message(&input[cursor..cursor + OBD2_SNAPSHOT_SENSOR_DATA_BYTES])?;
    cursor += OBD2_SNAPSHOT_SENSOR_DATA_BYTES;
    let freeze_frame_value_source = if input[cursor] == 0 {
        cursor += 1 + OBD2_SNAPSHOT_SENSOR_DATA_BYTES;
        None
    } else {
        cursor += 1;
        let message = decode_snapshot_sensor_message(
            &input[cursor..cursor + OBD2_SNAPSHOT_SENSOR_DATA_BYTES],
        )?;
        cursor += OBD2_SNAPSHOT_SENSOR_DATA_BYTES;
        Some(message)
    };
    let mut stored_dtcs = [DiagCode::MapRange; OBD2_STORED_DTC_CAP];
    let mut index = 0usize;
    while index < OBD2_STORED_DTC_CAP {
        stored_dtcs[index] = decode_snapshot_diag_code(input[cursor + index])?;
        index += 1;
    }
    cursor += OBD2_STORED_DTC_CAP;
    let stored_dtc_count = input[cursor].min(OBD2_STORED_DTC_CAP as u8);
    cursor += 1;
    let freeze_frame_dtc = match input[cursor] {
        0 => None,
        raw => Some(decode_snapshot_diag_code(raw)?),
    };
    cursor += 1;
    let current_diag_event =
        decode_snapshot_diag_event(&input[cursor..cursor + OBD2_SNAPSHOT_DIAG_EVENT_BYTES])?;
    cursor += OBD2_SNAPSHOT_DIAG_EVENT_BYTES;
    let freeze_frame_event =
        decode_snapshot_diag_event(&input[cursor..cursor + OBD2_SNAPSHOT_DIAG_EVENT_BYTES])?;
    cursor += OBD2_SNAPSHOT_DIAG_EVENT_BYTES;
    let last_fault = decode_snapshot_fault_code(input[cursor])?;
    cursor += 1;
    let last_observed_diag_code = match input[cursor] {
        0 => None,
        raw => Some(decode_snapshot_diag_code(raw)?),
    };
    cursor += 1;
    let last_observed_diag_timestamp_us = u32::from_le_bytes([
        input[cursor],
        input[cursor + 1],
        input[cursor + 2],
        input[cursor + 3],
    ]);
    Some(Obd2RetainedDiagnosticHistorySnapshot {
        current_data_value_source,
        freeze_frame_value_source,
        stored_dtcs,
        stored_dtc_count,
        freeze_frame_dtc,
        current_diag_event,
        freeze_frame_event,
        last_fault,
        last_observed_diag_code,
        last_observed_diag_timestamp_us,
    })
}

pub fn encode_obd2_retained_history_sidecar(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    out: &mut [u8],
) -> Option<usize> {
    if out.len() < OBD2_RETAINED_HISTORY_SIDECAR_BYTES {
        return None;
    }
    out[0..4].copy_from_slice(&OBD2_RETAINED_HISTORY_SIDECAR_MAGIC.to_le_bytes());
    out[4..6].copy_from_slice(&OBD2_RETAINED_HISTORY_SIDECAR_VERSION.to_le_bytes());
    out[6..8].copy_from_slice(&(OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES as u16).to_le_bytes());
    encode_obd2_retained_history_snapshot(
        snapshot,
        &mut out[OBD2_RETAINED_HISTORY_SIDECAR_HEADER_BYTES..OBD2_RETAINED_HISTORY_SIDECAR_BYTES],
    )?;
    Some(OBD2_RETAINED_HISTORY_SIDECAR_BYTES)
}

pub fn encode_obd2_retained_history_sidecar_prefix(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    out: &mut [u8],
) -> bool {
    encode_obd2_retained_history_sidecar_at(snapshot, out, 0)
}

pub fn encode_obd2_retained_history_sidecar_at(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    out: &mut [u8],
    offset: usize,
) -> bool {
    let Some(prefix) = out.get_mut(offset..offset + OBD2_RETAINED_HISTORY_SIDECAR_BYTES) else {
        return false;
    };
    encode_obd2_retained_history_sidecar(snapshot, prefix).is_some()
}

pub fn stage_obd2_retained_history_sidecar_bytes(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
) -> Option<[u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES]> {
    let mut out = [0u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES];
    encode_obd2_retained_history_sidecar(snapshot, &mut out)?;
    Some(out)
}

pub fn stage_optional_obd2_retained_history_sidecar_bytes(
    snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
) -> Option<Option<[u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES]>> {
    match snapshot {
        Some(snapshot) => Some(Some(stage_obd2_retained_history_sidecar_bytes(snapshot)?)),
        None => Some(None),
    }
}

pub fn write_obd2_retained_history_snapshot_with(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    write_sidecar: impl FnOnce(&[u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES]),
) -> bool {
    let Some(bytes) = stage_obd2_retained_history_sidecar_bytes(snapshot) else {
        return false;
    };
    write_sidecar(&bytes);
    true
}

pub fn write_optional_obd2_retained_history_snapshot_with(
    snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
    write_sidecar: impl FnOnce(&[u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES]),
) -> Option<()> {
    if let Some(snapshot) = snapshot {
        let bytes = stage_obd2_retained_history_sidecar_bytes(snapshot)?;
        write_sidecar(&bytes);
    }
    Some(())
}

pub fn install_obd2_retained_history_snapshot_halfwords_with(
    snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
    mut install_halfword: impl FnMut(usize, u16),
) -> bool {
    let Some(bytes) = stage_obd2_retained_history_sidecar_bytes(snapshot) else {
        return false;
    };
    for offset in (0..bytes.len()).step_by(2) {
        let value = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        install_halfword(offset, value);
    }
    true
}

pub fn install_optional_obd2_retained_history_snapshot_halfwords_with(
    snapshot: Option<&Obd2RetainedDiagnosticHistorySnapshot>,
    mut install_halfword: impl FnMut(usize, u16),
) -> Option<()> {
    let Some(snapshot) = snapshot else {
        return Some(());
    };
    let bytes = stage_obd2_retained_history_sidecar_bytes(snapshot)?;
    for offset in (0..bytes.len()).step_by(2) {
        let value = u16::from_le_bytes([bytes[offset], bytes[offset + 1]]);
        install_halfword(offset, value);
    }
    Some(())
}

pub fn decode_obd2_retained_history_sidecar(
    input: &[u8],
) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
    if input.len() < OBD2_RETAINED_HISTORY_SIDECAR_BYTES {
        return None;
    }
    let magic = u32::from_le_bytes([input[0], input[1], input[2], input[3]]);
    if magic != OBD2_RETAINED_HISTORY_SIDECAR_MAGIC {
        return None;
    }
    let version = u16::from_le_bytes([input[4], input[5]]);
    if version != OBD2_RETAINED_HISTORY_SIDECAR_VERSION {
        return None;
    }
    let payload_len = u16::from_le_bytes([input[6], input[7]]) as usize;
    if payload_len != OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES {
        return None;
    }
    decode_obd2_retained_history_snapshot(
        &input[OBD2_RETAINED_HISTORY_SIDECAR_HEADER_BYTES..OBD2_RETAINED_HISTORY_SIDECAR_BYTES],
    )
}

pub fn decode_obd2_retained_history_sidecar_prefix(
    input: &[u8],
) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
    decode_obd2_retained_history_sidecar_at(input, 0)
}

pub fn decode_obd2_retained_history_sidecar_at(
    input: &[u8],
    offset: usize,
) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
    let prefix = input.get(offset..offset + OBD2_RETAINED_HISTORY_SIDECAR_BYTES)?;
    decode_obd2_retained_history_sidecar(prefix)
}

pub fn decode_staged_obd2_retained_history_sidecar(
    bytes: &[u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES],
) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
    decode_obd2_retained_history_sidecar(bytes)
}

pub fn read_obd2_retained_history_snapshot_with(
    read_sidecar: impl FnOnce(&mut [u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES]),
) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
    let mut bytes = [0u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES];
    read_sidecar(&mut bytes);
    decode_staged_obd2_retained_history_sidecar(&bytes)
}

impl Obd2RetainedDiagnosticHistory {
    pub const fn new(current_data_value_source: Message) -> Self {
        Self {
            current_data_value_source,
            freeze_frame_value_source: None,
            stored_dtcs: [DiagCode::MapRange; OBD2_STORED_DTC_CAP],
            stored_dtc_count: 0,
            freeze_frame_dtc: None,
            current_diag_event: None,
            freeze_frame_event: None,
            last_fault: FaultCode::None,
            last_observed_diag_code: None,
            last_observed_diag_timestamp_us: 0,
        }
    }

    pub fn observe_live_diagnostic(
        &mut self,
        fault_state: FaultState,
        current_diag_event: Option<DiagEvent>,
        diag_log_events: [Option<DiagEvent>; DIAG_LOG_ENTRY_COUNT],
        recent_diag_events: [Option<DiagEvent>; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
        current_data_value_source: Message,
    ) {
        self.current_data_value_source = current_data_value_source;
        self.current_diag_event = current_diag_event.or_else(|| {
            obd2_diag_event_from_runtime_fault(fault_state, &self.current_data_value_source)
        });

        let mut observed_recent_event = false;
        for diag_event in diag_log_events.into_iter().flatten() {
            observed_recent_event = true;
            self.observe_diag_event(diag_event);
        }
        for diag_event in recent_diag_events.into_iter().flatten() {
            observed_recent_event = true;
            self.observe_diag_event(diag_event);
        }

        if !observed_recent_event {
            if let Some(diag_event) =
                obd2_diag_event_from_runtime_fault(fault_state, &self.current_data_value_source)
            {
                if fault_state.fault != self.last_fault {
                    self.observe_diag_event(diag_event);
                }
            }
        }
        self.last_fault = fault_state.fault;
    }

    pub fn current_data_value_source(&self) -> &Message {
        &self.current_data_value_source
    }

    pub fn stored_dtcs(&self) -> &[DiagCode] {
        &self.stored_dtcs[..self.stored_dtc_count]
    }

    pub const fn freeze_frame_dtc(&self) -> Option<DiagCode> {
        self.freeze_frame_dtc
    }

    pub const fn current_diag_event(&self) -> Option<DiagEvent> {
        self.current_diag_event
    }

    pub const fn freeze_frame_event(&self) -> Option<DiagEvent> {
        self.freeze_frame_event
    }

    pub fn freeze_frame_value_source(&self) -> Option<&Message> {
        self.freeze_frame_value_source.as_ref()
    }

    pub fn readiness_monitor_inputs(&self) -> CanObd2ReadinessMonitorInputs {
        CanObd2ReadinessMonitorInputs {
            mil_requested: self.current_diag_event.is_some(),
            stored_dtc_count: self.stored_dtc_count.min(u8::MAX as usize) as u8,
            misfire_supported: false,
            misfire_complete: false,
            fuel_system_supported: false,
            fuel_system_complete: false,
            comprehensive_components_supported: true,
            comprehensive_components_complete: self.current_diag_event.is_none(),
        }
    }

    pub fn dtc_clear_inputs(&self) -> CanObd2DtcClearInputs {
        CanObd2DtcClearInputs {
            cleared_dtc_count: self.stored_dtc_count.min(u8::MAX as usize) as u8,
            freeze_frame_cleared: self.freeze_frame_dtc.is_some(),
            readiness_reset: true,
        }
    }

    pub fn dispatch_inputs(&self) -> CanObd2MultiServiceDispatchInputs<'_> {
        self.dispatch_inputs_with_vehicle_identity(Obd2VehicleIdentity::from_signature(
            ecu_ts::TS_SIGNATURE,
        ))
    }

    pub fn dispatch_inputs_with_vehicle_identity(
        &self,
        identity: Obd2VehicleIdentity,
    ) -> CanObd2MultiServiceDispatchInputs<'_> {
        self.dispatch_inputs_with_vehicle_identity_and_key_lifecycle(
            identity,
            Some(CanObd2IdentityKeyLifecycleStatus::absent()),
        )
    }

    pub fn dispatch_inputs_with_vehicle_identity_and_key_lifecycle(
        &self,
        identity: Obd2VehicleIdentity,
        identity_key_lifecycle: Option<CanObd2IdentityKeyLifecycleStatus>,
    ) -> CanObd2MultiServiceDispatchInputs<'_> {
        self.dispatch_inputs_with_vehicle_identity_and_statuses(
            identity,
            identity_key_lifecycle,
            None,
        )
    }

    pub fn dispatch_inputs_with_vehicle_identity_and_statuses(
        &self,
        identity: Obd2VehicleIdentity,
        identity_key_lifecycle: Option<CanObd2IdentityKeyLifecycleStatus>,
        flash_write_fault: Option<ecu_transport::CanObd2FlashWriteFaultStatus>,
    ) -> CanObd2MultiServiceDispatchInputs<'_> {
        let mut vehicle_info = identity.into_transport_inputs();
        vehicle_info.identity_key_lifecycle = identity_key_lifecycle;
        vehicle_info.flash_write_fault = flash_write_fault;
        CanObd2MultiServiceDispatchInputs {
            current_data_value_source: Some(&self.current_data_value_source),
            readiness_monitor: Some(self.readiness_monitor_inputs()),
            dtc_clear: Some(self.dtc_clear_inputs()),
            freeze_frame_value_source: match &self.freeze_frame_value_source {
                Some(message) => Some(message),
                None => None,
            },
            stored_dtcs: &self.stored_dtcs[..self.stored_dtc_count],
            freeze_frame_dtc: self.freeze_frame_dtc,
            vehicle_info,
        }
    }

    pub fn snapshot(&self) -> Obd2RetainedDiagnosticHistorySnapshot {
        Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: self.current_data_value_source.clone(),
            freeze_frame_value_source: self.freeze_frame_value_source.clone(),
            stored_dtcs: self.stored_dtcs,
            stored_dtc_count: self.stored_dtc_count.min(u8::MAX as usize) as u8,
            freeze_frame_dtc: self.freeze_frame_dtc,
            current_diag_event: self.current_diag_event,
            freeze_frame_event: self.freeze_frame_event,
            last_fault: self.last_fault,
            last_observed_diag_code: self.last_observed_diag_code,
            last_observed_diag_timestamp_us: self.last_observed_diag_timestamp_us,
        }
    }

    pub fn from_snapshot(snapshot: Obd2RetainedDiagnosticHistorySnapshot) -> Self {
        let mut history = Self::new(snapshot.current_data_value_source.clone());
        history.restore_from_snapshot(&snapshot);
        history
    }

    pub fn restore_from_snapshot(&mut self, snapshot: &Obd2RetainedDiagnosticHistorySnapshot) {
        self.current_data_value_source = snapshot.current_data_value_source.clone();
        self.freeze_frame_value_source = snapshot.freeze_frame_value_source.clone();
        self.stored_dtcs = snapshot.stored_dtcs;
        self.stored_dtc_count = usize::from(snapshot.stored_dtc_count).min(OBD2_STORED_DTC_CAP);
        self.freeze_frame_dtc = snapshot.freeze_frame_dtc;
        self.current_diag_event = snapshot.current_diag_event;
        self.freeze_frame_event = snapshot.freeze_frame_event;
        self.last_fault = snapshot.last_fault;
        self.last_observed_diag_code = snapshot.last_observed_diag_code;
        self.last_observed_diag_timestamp_us = snapshot.last_observed_diag_timestamp_us;
    }

    pub fn clear(&mut self) {
        self.freeze_frame_value_source = None;
        self.stored_dtc_count = 0;
        self.freeze_frame_dtc = None;
        self.current_diag_event = None;
        self.freeze_frame_event = None;
        self.last_fault = FaultCode::None;
        self.last_observed_diag_code = None;
        self.last_observed_diag_timestamp_us = 0;
    }

    fn push_stored_dtc(&mut self, diag_code: DiagCode) {
        if self.stored_dtcs[..self.stored_dtc_count].contains(&diag_code) {
            return;
        }
        if self.stored_dtc_count >= OBD2_STORED_DTC_CAP {
            return;
        }
        self.stored_dtcs[self.stored_dtc_count] = diag_code;
        self.stored_dtc_count += 1;
    }

    fn observe_diag_event(&mut self, diag_event: DiagEvent) {
        if self.last_observed_diag_code == Some(diag_event.code)
            && self.last_observed_diag_timestamp_us == diag_event.timestamp.get()
        {
            return;
        }
        self.push_stored_dtc(diag_event.code);
        if self.freeze_frame_dtc.is_none() {
            self.freeze_frame_dtc = Some(diag_event.code);
            self.freeze_frame_event = Some(diag_event);
            self.freeze_frame_value_source = Some(self.current_data_value_source.clone());
        }
        self.last_observed_diag_code = Some(diag_event.code);
        self.last_observed_diag_timestamp_us = diag_event.timestamp.get();
    }
}

pub const fn obd2_diag_code_from_runtime_fault(fault: FaultCode) -> Option<DiagCode> {
    match fault {
        FaultCode::None => None,
        FaultCode::SyncLoss => Some(DiagCode::CamMissing),
        FaultCode::SensorOutOfRange => Some(DiagCode::MapRange),
        FaultCode::CalibrationInvalid => Some(DiagCode::PersistCrcFault),
        FaultCode::SafetyCut => Some(DiagCode::MapFailureHighLoad),
        FaultCode::ActuatorFault => Some(DiagCode::TpsMapPlausibility),
    }
}

pub const fn spec_fault_state_from_runtime_fault(fault_state: FaultState) -> SpecFaultState {
    SpecFaultState {
        code: match fault_state.fault {
            FaultCode::None => SpecFaultCode::None,
            FaultCode::SensorOutOfRange => SpecFaultCode::SensorOutOfRange,
            FaultCode::SafetyCut => SpecFaultCode::SafetyCut,
            _ => SpecFaultCode::Other,
        },
        severity: match fault_state.severity {
            ecu_domain::FaultSeverity::Info => SpecFaultSeverity::Info,
            ecu_domain::FaultSeverity::Warning => SpecFaultSeverity::Warning,
            ecu_domain::FaultSeverity::Critical => SpecFaultSeverity::Critical,
        },
        cancel_reason: match fault_state.cancel_reason {
            ecu_domain::CancelReason::Manual => SpecCancelReason::Manual,
            ecu_domain::CancelReason::SafetyShutdown => SpecCancelReason::SafetyShutdown,
            _ => SpecCancelReason::Other,
        },
    }
}

pub const fn obd2_fault_event_from_runtime_fault(fault_state: FaultState) -> SpecFaultEvent {
    fault_event_from_state(spec_fault_state_from_runtime_fault(fault_state))
}

pub const fn obd2_fault_event_from_fault_transition(
    fault_transition: CommonFaultTransitionTelemetry,
) -> SpecFaultEvent {
    match fault_transition.event.event_id {
        CommonFaultTransitionEventId::None => SpecFaultEvent {
            active: false,
            action: SpecFaultAction::None,
            persistence: ecu_spec::SpecFaultPersistence::Inactive,
        },
        CommonFaultTransitionEventId::FaultEntered | CommonFaultTransitionEventId::FaultUpdated => {
            obd2_fault_event_from_runtime_fault(FaultState {
                fault: fault_transition.current_fault,
                severity: fault_transition.current_severity,
                cancel_reason: fault_transition.current_cancel_reason,
            })
        }
        CommonFaultTransitionEventId::FaultCleared => {
            fault_event_for_clear(spec_fault_state_from_runtime_fault(FaultState {
                fault: fault_transition.previous_fault,
                severity: fault_transition.previous_severity,
                cancel_reason: fault_transition.previous_cancel_reason,
            }))
        }
    }
}

pub fn obd2_diag_event_from_runtime_fault(
    fault_state: FaultState,
    current_data_value_source: &Message,
) -> Option<DiagEvent> {
    let fault_event = obd2_fault_event_from_runtime_fault(fault_state);
    if !fault_event.active || matches!(fault_event.action, SpecFaultAction::None) {
        return None;
    }
    let code = obd2_diag_code_from_runtime_fault(fault_state.fault)?;
    let timestamp =
        obd2_sensor_timestamp(current_data_value_source).unwrap_or_else(|| Micros::new(0));
    let (context, source) =
        obd2_diag_context_and_source(fault_state.fault, current_data_value_source);

    Some(DiagEvent {
        code,
        timestamp,
        source,
        context,
        start_us: timestamp.get(),
        end_us: timestamp.get(),
    })
}

pub fn obd2_diag_event_from_fault_transition(
    fault_transition: CommonFaultTransitionTelemetry,
    current_data_value_source: &Message,
) -> Option<DiagEvent> {
    if !fault_transition.changed {
        return None;
    }
    let fault_event = obd2_fault_event_from_fault_transition(fault_transition);
    if !fault_event.active || matches!(fault_event.action, SpecFaultAction::None) {
        return None;
    }
    let fault = match fault_transition.event.event_id {
        CommonFaultTransitionEventId::None => return None,
        CommonFaultTransitionEventId::FaultEntered | CommonFaultTransitionEventId::FaultUpdated => {
            fault_transition.current_fault
        }
        CommonFaultTransitionEventId::FaultCleared => return None,
    };
    let code = obd2_diag_code_from_runtime_fault(fault)?;
    let timestamp = fault_transition.at_us;
    let (context, source) = obd2_diag_context_and_source(fault, current_data_value_source);

    Some(DiagEvent {
        code,
        timestamp,
        source,
        context,
        start_us: timestamp.get(),
        end_us: timestamp.get(),
    })
}

pub trait DiagnosticClearOwner {
    fn clear_diagnostics(&mut self) -> DiagClearSummary;
}

pub trait LiveObd2RequestOwner: DiagnosticClearOwner {
    fn fault_state(&self) -> FaultState;
    fn current_obd2_sensor_data(&self) -> Message;
    fn obd2_retained_history(&self) -> &Obd2RetainedDiagnosticHistory;
    fn obd2_retained_history_mut(&mut self) -> &mut Obd2RetainedDiagnosticHistory;
    fn current_obd2_diag_event(&self) -> Option<DiagEvent> {
        None
    }
    fn obd2_diag_log_events(&self) -> [Option<DiagEvent>; DIAG_LOG_ENTRY_COUNT] {
        [None; DIAG_LOG_ENTRY_COUNT]
    }
    fn recent_obd2_diag_events(&self) -> [Option<DiagEvent>; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP] {
        [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP]
    }
    fn fallback_obd2_vehicle_identity(&self) -> Obd2VehicleIdentity {
        Obd2VehicleIdentity::from_signature(ecu_ts::TS_SIGNATURE)
    }
    fn provisioned_obd2_identity(&self) -> ProvisionedObd2Identity {
        ProvisionedObd2Identity::default()
    }
    fn obd2_identity_key_lifecycle_status(&self) -> Option<CanObd2IdentityKeyLifecycleStatus> {
        Some(CanObd2IdentityKeyLifecycleStatus::absent())
    }
    fn obd2_flash_write_fault_status(&self) -> Option<ecu_transport::CanObd2FlashWriteFaultStatus> {
        None
    }
    fn obd2_vehicle_identity(&self) -> Obd2VehicleIdentity {
        self.fallback_obd2_vehicle_identity()
            .with_provisioned_identity(self.provisioned_obd2_identity())
    }
}

impl DiagnosticClearOwner for EcuState {
    fn clear_diagnostics(&mut self) -> DiagClearSummary {
        EcuState::clear_diagnostics(self)
    }
}

fn dtc_clear_inputs_from_summary(summary: DiagClearSummary) -> CanObd2DtcClearInputs {
    CanObd2DtcClearInputs {
        cleared_dtc_count: summary
            .cleared_active_count
            .saturating_add(summary.cleared_log_entries),
        freeze_frame_cleared: summary.cleared_active_count > 0 || summary.cleared_log_entries > 0,
        readiness_reset: true,
    }
}

fn compose_obd2_dtc_clear(
    state: &mut impl DiagnosticClearOwner,
    request: &Message,
) -> Result<Obd2DtcClearExecutionSurface, CanObd2DtcClearResponseError> {
    let mut transport = CanObd2DtcClearSurface::assemble(
        request,
        CanObd2DtcClearInputs {
            cleared_dtc_count: 0,
            freeze_frame_cleared: false,
            readiness_reset: true,
        },
    )?;
    let clear_summary = state.clear_diagnostics();
    let inputs = dtc_clear_inputs_from_summary(clear_summary);
    transport.verdict.cleared_dtc_count = inputs.cleared_dtc_count;
    transport.verdict.freeze_frame_cleared = inputs.freeze_frame_cleared;
    transport.verdict.readiness_reset = inputs.readiness_reset;
    Ok(Obd2DtcClearExecutionSurface {
        clear_summary,
        transport,
    })
}

#[derive(Debug, Clone, PartialEq)]
pub enum Obd2TransportServiceOutcome {
    Ignored(Message),
    DtcClear(Obd2DtcClearExecutionSurface),
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Obd2TransportServiceError {
    Compose(CanObd2DtcClearResponseError),
    Send(TransportError),
}

#[derive(Debug, Clone, PartialEq)]
pub enum Obd2MultiServiceTransportServiceOutcome {
    Ignored(Obd2IgnoredMessageSurface),
    Dispatch(CanObd2MultiServiceDispatchSurface),
    SegmentedDispatch(CanObd2SegmentedVehicleInfoResponseSurface),
    DtcClear {
        clear_summary: DiagClearSummary,
        dispatch: CanObd2MultiServiceDispatchSurface,
    },
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Obd2IgnoredMessageSurface {
    pub obd2_service: Option<u8>,
    pub parameter_id: Option<u8>,
}

impl Obd2IgnoredMessageSurface {
    pub const fn from_message(message: &Message) -> Self {
        match message {
            Message::Obd2Request {
                service,
                parameter_id,
                ..
            }
            | Message::Obd2Response {
                service,
                parameter_id,
                ..
            }
            | Message::Obd2SegmentedResponse {
                service,
                parameter_id,
                ..
            } => Self {
                obd2_service: Some(*service),
                parameter_id: *parameter_id,
            },
            _ => Self {
                obd2_service: None,
                parameter_id: None,
            },
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Obd2MultiServiceTransportServiceError {
    Send(TransportError),
}

/// Minimal board/common transport owner for bounded OBD-II clear-DTC requests.
pub struct Obd2TransportService<T: Transport> {
    transport: T,
}

impl<T: Transport> Obd2TransportService<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn pump_once(
        &mut self,
        state: &mut impl DiagnosticClearOwner,
    ) -> Result<Option<Obd2TransportServiceOutcome>, Obd2TransportServiceError> {
        self.transport.poll();
        let Some(message) = self.transport.try_receive() else {
            return Ok(None);
        };
        match message {
            request @ Message::Obd2Request { service: 0x04, .. } => {
                let surface = compose_obd2_dtc_clear(state, &request)
                    .map_err(Obd2TransportServiceError::Compose)?;
                self.transport
                    .send(&surface.transport.response)
                    .map_err(Obd2TransportServiceError::Send)?;
                Ok(Some(Obd2TransportServiceOutcome::DtcClear(surface)))
            }
            other => Ok(Some(Obd2TransportServiceOutcome::Ignored(other))),
        }
    }
}

/// Shared board/common owner for bounded live OBD-II mode `0x01`/`0x02`/`0x03`/`0x04`.
pub struct Obd2MultiServiceTransportService<T: Transport> {
    transport: T,
}

impl<T: Transport> Obd2MultiServiceTransportService<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    pub fn transport(&self) -> &T {
        &self.transport
    }

    pub fn transport_mut(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn pump_once(
        &mut self,
        owner: &mut impl LiveObd2RequestOwner,
    ) -> Result<
        Option<Obd2MultiServiceTransportServiceOutcome>,
        Obd2MultiServiceTransportServiceError,
    > {
        self.transport.poll();
        let fault_state = owner.fault_state();
        let current_data_value_source = owner.current_obd2_sensor_data();
        let current_diag_event = owner.current_obd2_diag_event();
        let diag_log_events = owner.obd2_diag_log_events();
        let recent_diag_events = if diag_log_events.iter().any(|entry| entry.is_some()) {
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP]
        } else {
            owner.recent_obd2_diag_events()
        };
        owner.obd2_retained_history_mut().observe_live_diagnostic(
            fault_state,
            current_diag_event,
            diag_log_events,
            recent_diag_events,
            current_data_value_source,
        );

        let Some(message) = self.transport.try_receive() else {
            return Ok(None);
        };
        let request = match message {
            request @ Message::Obd2Request { .. } => request,
            other => {
                return Ok(Some(Obd2MultiServiceTransportServiceOutcome::Ignored(
                    Obd2IgnoredMessageSurface::from_message(&other),
                )))
            }
        };

        let outcome = match CanObd2MultiServiceDispatchSurface::dispatch_outcome(
            &request,
            owner
                .obd2_retained_history()
                .dispatch_inputs_with_vehicle_identity_and_statuses(
                    owner.obd2_vehicle_identity(),
                    owner.obd2_identity_key_lifecycle_status(),
                    owner.obd2_flash_write_fault_status(),
                ),
        ) {
            Ok(outcome) => outcome,
            Err(error) => CanObd2MultiServiceDispatchOutcome::Single(negative_dispatch_from_error(
                &request, error,
            )),
        };

        let dispatch = match outcome {
            CanObd2MultiServiceDispatchOutcome::Single(dispatch) => dispatch,
            CanObd2MultiServiceDispatchOutcome::SegmentedVehicleInfo(dispatch) => {
                for segment in dispatch
                    .segments
                    .iter()
                    .take(dispatch.segment_count as usize)
                    .flatten()
                {
                    let message = segment.to_message();
                    self.transport
                        .send(&message)
                        .map_err(Obd2MultiServiceTransportServiceError::Send)?;
                }
                return Ok(Some(
                    Obd2MultiServiceTransportServiceOutcome::SegmentedDispatch(dispatch),
                ));
            }
        };

        let response = dispatch.response.to_message();
        self.transport
            .send(&response)
            .map_err(Obd2MultiServiceTransportServiceError::Send)?;

        if matches!(
            dispatch.verdict,
            CanObd2MultiServiceDispatchVerdict::DtcClearPositive
        ) {
            let clear_summary = owner.clear_diagnostics();
            owner.obd2_retained_history_mut().clear();
            return Ok(Some(Obd2MultiServiceTransportServiceOutcome::DtcClear {
                clear_summary,
                dispatch,
            }));
        }

        Ok(Some(Obd2MultiServiceTransportServiceOutcome::Dispatch(
            dispatch,
        )))
    }
}

fn negative_dispatch_from_error(
    request: &Message,
    error: CanObd2MultiServiceDispatchError,
) -> CanObd2MultiServiceDispatchSurface {
    let (request_service_id, parameter_id) = match request {
        Message::Obd2Request {
            service,
            parameter_id,
            ..
        } => (*service, *parameter_id),
        _ => (0, None),
    };
    let code = match error {
        CanObd2MultiServiceDispatchError::UnsupportedService { .. } => {
            CanObd2NegativeResponseCode::ServiceNotSupported
        }
        CanObd2MultiServiceDispatchError::MissingFreezeFrameDtc
        | CanObd2MultiServiceDispatchError::MissingDtcClearInputs => {
            CanObd2NegativeResponseCode::RequestOutOfRange
        }
        CanObd2MultiServiceDispatchError::Mode01(mode01) => match mode01 {
            ecu_transport::CanObd2RequestDispatchError::NotObd2Request => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            ecu_transport::CanObd2RequestDispatchError::MissingParameterId
            | ecu_transport::CanObd2RequestDispatchError::MissingReadinessInputs
            | ecu_transport::CanObd2RequestDispatchError::MissingValueSource { .. }
            | ecu_transport::CanObd2RequestDispatchError::IncompatibleValueSource { .. } => {
                CanObd2NegativeResponseCode::RequestOutOfRange
            }
        },
        CanObd2MultiServiceDispatchError::DtcClear(clear) => match clear {
            ecu_transport::CanObd2DtcClearResponseError::NotObd2Request
            | ecu_transport::CanObd2DtcClearResponseError::UnsupportedService { .. } => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            ecu_transport::CanObd2DtcClearResponseError::UnexpectedParameterId { .. } => {
                CanObd2NegativeResponseCode::RequestOutOfRange
            }
        },
        CanObd2MultiServiceDispatchError::DtcFreezeFrame(freeze) => match freeze {
            ecu_transport::CanObd2DtcFreezeFrameResponseError::NotObd2Request
            | ecu_transport::CanObd2DtcFreezeFrameResponseError::UnsupportedService { .. } => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            ecu_transport::CanObd2DtcFreezeFrameResponseError::MissingParameterId
            | ecu_transport::CanObd2DtcFreezeFrameResponseError::UnsupportedPid { .. }
            | ecu_transport::CanObd2DtcFreezeFrameResponseError::MissingFreezeFrameValueSource {
                ..
            }
            | ecu_transport::CanObd2DtcFreezeFrameResponseError::IncompatibleFreezeFrameValueSource {
                ..
            } => CanObd2NegativeResponseCode::RequestOutOfRange,
        },
        CanObd2MultiServiceDispatchError::VehicleInfo(vehicle_info) => match vehicle_info {
            ecu_transport::CanObd2VehicleInfoResponseError::NotObd2Request
            | ecu_transport::CanObd2VehicleInfoResponseError::UnsupportedService { .. } => {
                CanObd2NegativeResponseCode::ServiceNotSupported
            }
            ecu_transport::CanObd2VehicleInfoResponseError::MissingInfoTypeId
            | ecu_transport::CanObd2VehicleInfoResponseError::UnsupportedInfoType { .. } => {
                CanObd2NegativeResponseCode::RequestOutOfRange
            }
        },
        CanObd2MultiServiceDispatchError::SegmentedVehicleInfo(vehicle_info) => {
            match vehicle_info {
                ecu_transport::CanObd2SegmentedVehicleInfoResponseError::NotObd2Request
                | ecu_transport::CanObd2SegmentedVehicleInfoResponseError::UnsupportedService {
                    ..
                } => CanObd2NegativeResponseCode::ServiceNotSupported,
                ecu_transport::CanObd2SegmentedVehicleInfoResponseError::MissingInfoTypeId
                | ecu_transport::CanObd2SegmentedVehicleInfoResponseError::UnsupportedInfoType {
                    ..
                } => CanObd2NegativeResponseCode::RequestOutOfRange,
            }
        }
        CanObd2MultiServiceDispatchError::NotObd2Request => {
            CanObd2NegativeResponseCode::ServiceNotSupported
        }
    };

    CanObd2MultiServiceDispatchSurface {
        request_service_id,
        parameter_id,
        verdict: CanObd2MultiServiceDispatchVerdict::NegativeResponse(code),
        response: CanObd2ResponseFrame {
            service: 0x7F,
            parameter_id: Some(request_service_id),
            negative_response_code: Some(code.raw()),
            payload_len: 0,
            payload: [0; 6],
        },
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use ecu_compat::diag::{DiagCode, DiagEvent, DiagSource};
    use ecu_compat::Micros;
    use ecu_domain::{diag::DiagClearSummary, CancelReason, FaultSeverity, Kpa10, Rpm};
    use ecu_transport::{CanObd2DtcClearVerdict, TransportStats};
    use std::collections::VecDeque;
    use std::vec::Vec;

    #[derive(Debug, Default)]
    struct MockTransport {
        rx: VecDeque<Message>,
        tx: Vec<Message>,
        poll_count: u32,
    }

    impl Transport for MockTransport {
        fn send(&mut self, message: &Message) -> Result<(), TransportError> {
            self.tx.push(message.clone());
            Ok(())
        }

        fn try_receive(&mut self) -> Option<Message> {
            self.rx.pop_front()
        }

        fn poll(&mut self) {
            self.poll_count = self.poll_count.saturating_add(1);
        }

        fn flush(&mut self) -> Result<(), TransportError> {
            Ok(())
        }

        fn stats(&self) -> TransportStats {
            TransportStats::default()
        }

        fn is_ready(&self) -> bool {
            true
        }
    }

    #[derive(Debug, Default)]
    struct MockClearOwner {
        clear_summary: DiagClearSummary,
        clear_count: u8,
    }

    impl DiagnosticClearOwner for MockClearOwner {
        fn clear_diagnostics(&mut self) -> DiagClearSummary {
            self.clear_count = self.clear_count.saturating_add(1);
            self.clear_summary
        }
    }

    #[derive(Debug, Clone)]
    struct MockLiveOwner {
        fault_state: FaultState,
        current_obd2_sensor_data: Message,
        retained_history: Obd2RetainedDiagnosticHistory,
        current_obd2_diag_event: Option<DiagEvent>,
        obd2_diag_log_events: [Option<DiagEvent>; DIAG_LOG_ENTRY_COUNT],
        recent_obd2_diag_events: [Option<DiagEvent>; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
        clear_summary: DiagClearSummary,
        clear_count: u8,
    }

    impl DiagnosticClearOwner for MockLiveOwner {
        fn clear_diagnostics(&mut self) -> DiagClearSummary {
            self.clear_count = self.clear_count.saturating_add(1);
            self.fault_state.fault = FaultCode::None;
            self.clear_summary
        }
    }

    impl LiveObd2RequestOwner for MockLiveOwner {
        fn fault_state(&self) -> FaultState {
            self.fault_state
        }

        fn current_obd2_sensor_data(&self) -> Message {
            self.current_obd2_sensor_data.clone()
        }

        fn obd2_retained_history(&self) -> &Obd2RetainedDiagnosticHistory {
            &self.retained_history
        }

        fn obd2_retained_history_mut(&mut self) -> &mut Obd2RetainedDiagnosticHistory {
            &mut self.retained_history
        }

        fn current_obd2_diag_event(&self) -> Option<DiagEvent> {
            self.current_obd2_diag_event
        }

        fn obd2_diag_log_events(&self) -> [Option<DiagEvent>; DIAG_LOG_ENTRY_COUNT] {
            self.obd2_diag_log_events
        }

        fn recent_obd2_diag_events(&self) -> [Option<DiagEvent>; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP] {
            self.recent_obd2_diag_events
        }

        fn fallback_obd2_vehicle_identity(&self) -> Obd2VehicleIdentity {
            Obd2VehicleIdentity::from_calibration_identity(CalibrationPackageIdentity {
                schema_version: ecu_calibration::CalibrationSchemaVersion::CURRENT,
                active_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
                staged_base_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
                staged_revision: ecu_calibration::CalibrationRevision::new(0x5678_EF90),
                staged_dirty: true,
                checksum: ecu_calibration::CalibrationPackageChecksum::default(),
            })
        }
    }

    #[derive(Debug, Clone)]
    struct ProvisionedMockLiveOwner {
        inner: MockLiveOwner,
        provisioned: ProvisionedObd2Identity,
    }

    impl DiagnosticClearOwner for ProvisionedMockLiveOwner {
        fn clear_diagnostics(&mut self) -> DiagClearSummary {
            self.inner.clear_diagnostics()
        }
    }

    impl LiveObd2RequestOwner for ProvisionedMockLiveOwner {
        fn fault_state(&self) -> FaultState {
            self.inner.fault_state()
        }

        fn current_obd2_sensor_data(&self) -> Message {
            self.inner.current_obd2_sensor_data()
        }

        fn obd2_retained_history(&self) -> &Obd2RetainedDiagnosticHistory {
            self.inner.obd2_retained_history()
        }

        fn obd2_retained_history_mut(&mut self) -> &mut Obd2RetainedDiagnosticHistory {
            self.inner.obd2_retained_history_mut()
        }

        fn current_obd2_diag_event(&self) -> Option<DiagEvent> {
            self.inner.current_obd2_diag_event()
        }

        fn obd2_diag_log_events(&self) -> [Option<DiagEvent>; DIAG_LOG_ENTRY_COUNT] {
            self.inner.obd2_diag_log_events()
        }

        fn recent_obd2_diag_events(&self) -> [Option<DiagEvent>; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP] {
            self.inner.recent_obd2_diag_events()
        }

        fn fallback_obd2_vehicle_identity(&self) -> Obd2VehicleIdentity {
            self.inner.fallback_obd2_vehicle_identity()
        }

        fn provisioned_obd2_identity(&self) -> ProvisionedObd2Identity {
            self.provisioned
        }
    }

    impl Obd2RetainedHistoryOwner for MockLiveOwner {
        fn obd2_retained_history_snapshot(&self) -> Obd2RetainedDiagnosticHistorySnapshot {
            self.retained_history.snapshot()
        }

        fn restore_obd2_retained_history(
            &mut self,
            snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
        ) {
            self.retained_history.restore_from_snapshot(snapshot);
        }
    }

    #[derive(Debug, Clone, Default)]
    struct MockRetainedHistoryStore {
        loaded_snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
        saved_snapshot: Option<Obd2RetainedDiagnosticHistorySnapshot>,
        save_count: u8,
    }

    impl Obd2RetainedHistoryStore for MockRetainedHistoryStore {
        type Error = ();

        fn load_retained_obd2_history_snapshot(
            &self,
        ) -> Option<Obd2RetainedDiagnosticHistorySnapshot> {
            self.loaded_snapshot.clone()
        }

        fn save_retained_obd2_history_snapshot(
            &mut self,
            snapshot: &Obd2RetainedDiagnosticHistorySnapshot,
        ) -> Result<(), Self::Error> {
            self.saved_snapshot = Some(snapshot.clone());
            self.save_count = self.save_count.saturating_add(1);
            Ok(())
        }
    }

    fn sensor_message(timestamp_us: u32, clt_offset: u8) -> Message {
        Message::SensorData {
            map_kpa_x10: 900,
            tps_percent: 25,
            iat_offset: 65,
            clt_offset,
            voltage_x10: 125,
            lambda_x100: 100,
            flags: 0,
            timestamp_us,
        }
    }

    #[test]
    fn obd2_current_data_from_board_inputs_prefers_logical_snapshot_projection() {
        let snapshot = BoardSensorSnapshot {
            rpm: Rpm::new(2500),
            map_kpa10: Kpa10::new(950),
            tps_x100: 2450,
            clt_c10: 870,
            iat_c10: 310,
            vbatt_mv: 13_800,
            lambda_x100: ecu_domain::Lambda100::new(102),
            validity: BoardSensorValidityFlags::from_channels(false, false, false, true),
            ..BoardSensorSnapshot::default()
        };

        assert_eq!(
            obd2_current_data_from_board_inputs(
                Some(BoardSensorSnapshotCapture {
                    at_us: Micros::new(1234),
                    angle_x10: ecu_domain::Degrees10::new(90),
                    snapshot,
                }),
                Some(CaptureSample {
                    at_us: Micros::new(4321),
                    rpm: Rpm::new(1800),
                    load_kpa10: Kpa10::new(700),
                    angle_x10: ecu_domain::Degrees10::new(45),
                }),
            ),
            Message::SensorData {
                map_kpa_x10: 950,
                tps_percent: 24,
                iat_offset: 71,
                clt_offset: 127,
                voltage_x10: 138,
                lambda_x100: 102,
                flags: 0,
                timestamp_us: 1234,
            }
        );
    }

    #[test]
    fn obd2_current_data_from_board_inputs_falls_back_to_capture_sample() {
        assert_eq!(
            obd2_current_data_from_board_inputs(
                None,
                Some(CaptureSample {
                    at_us: Micros::new(4321),
                    rpm: Rpm::new(1800),
                    load_kpa10: Kpa10::new(700),
                    angle_x10: ecu_domain::Degrees10::new(45),
                }),
            ),
            Message::SensorData {
                map_kpa_x10: 700,
                tps_percent: 0,
                iat_offset: 65,
                clt_offset: 60,
                voltage_x10: 125,
                lambda_x100: 100,
                flags: 0,
                timestamp_us: 4321,
            }
        );
    }

    #[test]
    fn obd2_current_data_from_board_inputs_uses_default_when_no_inputs_exist() {
        assert_eq!(
            obd2_current_data_from_board_inputs(None, None),
            Message::SensorData {
                map_kpa_x10: 0,
                tps_percent: 0,
                iat_offset: 65,
                clt_offset: 60,
                voltage_x10: 125,
                lambda_x100: 100,
                flags: 0,
                timestamp_us: 0,
            }
        );
    }

    #[test]
    fn obd2_vehicle_info_from_identity_signature_derives_bounded_ecu_name() {
        assert_eq!(
            obd2_vehicle_info_from_identity_signature(ecu_ts::TS_SIGNATURE),
            CanObd2VehicleInfoInputs {
                ecu_name_len: 6,
                ecu_name: [b'I', b'P', b'W', b'E', b'C', b'U'],
                vin_len: 9,
                vin: [b'I', b'P', b'W', b'E', b'C', b'U', b'V', b'0', b'1', 0, 0, 0, 0, 0, 0, 0, 0],
                calibration_id_len: 9,
                calibration_id: [
                    b'I', b'P', b'W', b'E', b'C', b'U', b'V', b'0', b'1', 0, 0, 0, 0, 0, 0, 0, 0
                ],
                identity_key_lifecycle: None,
                flash_write_fault: None,
            }
        );
        let mut default_identity_without_lifecycle = CanObd2VehicleInfoInputs::default_identity();
        default_identity_without_lifecycle.identity_key_lifecycle = None;
        default_identity_without_lifecycle.flash_write_fault = None;
        assert_eq!(
            obd2_vehicle_info_from_identity_signature(b""),
            default_identity_without_lifecycle
        );
    }

    #[test]
    fn obd2_vehicle_identity_from_calibration_identity_overrides_signature_vin() {
        let identity = Obd2VehicleIdentity::from_calibration_identity(CalibrationPackageIdentity {
            schema_version: ecu_calibration::CalibrationSchemaVersion::CURRENT,
            active_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_base_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_revision: ecu_calibration::CalibrationRevision::new(0x5678_EF90),
            staged_dirty: true,
            checksum: ecu_calibration::CalibrationPackageChecksum::new(0x00AB_CDEF),
        });

        assert_eq!(identity.ecu_name, [b'I', b'P', b'W', b'E', b'C', b'U']);
        assert_eq!(identity.vin_len, ecu_transport::CAN_OBD2_VIN_LEN as u8);
        assert_eq!(&identity.vin, b"C1234ABCD00ABCDEF");
        assert_ne!(
            identity.vin,
            Obd2VehicleIdentity::from_signature(ecu_ts::TS_SIGNATURE).vin
        );
    }

    #[test]
    fn obd2_vehicle_identity_from_calibration_identity_tracks_checksum_changes() {
        let mut first = CalibrationPackageIdentity {
            schema_version: ecu_calibration::CalibrationSchemaVersion::CURRENT,
            active_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_base_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_revision: ecu_calibration::CalibrationRevision::new(0x5678_EF90),
            staged_dirty: true,
            checksum: ecu_calibration::CalibrationPackageChecksum::new(0x0011_2233),
        };
        let mut second = first;
        second.checksum = ecu_calibration::CalibrationPackageChecksum::new(0x4411_2233);

        let first_identity = Obd2VehicleIdentity::from_calibration_identity(first);
        let second_identity = Obd2VehicleIdentity::from_calibration_identity(second);

        assert_eq!(&first_identity.vin, b"C1234ABCD00112233");
        assert_eq!(&second_identity.vin, b"C1234ABCD44112233");
        assert_ne!(first_identity.vin, second_identity.vin);
        assert_ne!(
            first_identity.calibration_id,
            second_identity.calibration_id
        );
        first.checksum = second.checksum;
        assert_eq!(
            Obd2VehicleIdentity::from_calibration_identity(first),
            second_identity
        );
    }

    #[test]
    fn obd2_vehicle_identity_prefers_provisioned_identity_fields() {
        let fallback = Obd2VehicleIdentity::from_calibration_identity(CalibrationPackageIdentity {
            schema_version: ecu_calibration::CalibrationSchemaVersion::CURRENT,
            active_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_base_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_revision: ecu_calibration::CalibrationRevision::new(0x5678_EF90),
            staged_dirty: true,
            checksum: ecu_calibration::CalibrationPackageChecksum::default(),
        });
        let provisioned = ProvisionedObd2Identity::from_ascii(
            Some(b"race-vin-1234567890"),
            Some(b"cal-id-alpha-0001"),
            Some(b"m50-a1"),
        );

        let identity = fallback.with_provisioned_identity(provisioned);

        assert_eq!(identity.ecu_name_len, 5);
        assert_eq!(identity.ecu_name, [b'M', b'5', b'0', b'A', b'1', 0]);
        assert_eq!(identity.vin_len, ecu_transport::CAN_OBD2_VIN_LEN as u8);
        assert_eq!(&identity.vin, b"RACEVIN1234567890");
        assert_eq!(identity.calibration_id_len, 14);
        assert_eq!(&identity.calibration_id[..14], b"CALIDALPHA0001");
        assert_ne!(identity.vin, fallback.vin);
        assert_ne!(identity.calibration_id, fallback.calibration_id);
    }

    #[test]
    fn obd2_vehicle_identity_keeps_fallback_fields_when_provisioned_field_empty() {
        let fallback = Obd2VehicleIdentity::from_calibration_identity(CalibrationPackageIdentity {
            schema_version: ecu_calibration::CalibrationSchemaVersion::CURRENT,
            active_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_base_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
            staged_revision: ecu_calibration::CalibrationRevision::new(0x5678_EF90),
            staged_dirty: true,
            checksum: ecu_calibration::CalibrationPackageChecksum::default(),
        });

        let identity = fallback.with_provisioned_identity(ProvisionedObd2Identity::from_ascii(
            Some(b"trackvin000000001"),
            None,
            None,
        ));

        assert_eq!(&identity.vin, b"TRACKVIN000000001");
        assert_eq!(identity.calibration_id, fallback.calibration_id);
        assert_eq!(identity.ecu_name, fallback.ecu_name);
    }

    #[test]
    fn live_obd2_request_owner_prefers_provisioned_identity_over_fallback() {
        struct ProvisionedOwner {
            history: Obd2RetainedDiagnosticHistory,
            provisioned: ProvisionedObd2Identity,
        }

        impl DiagnosticClearOwner for ProvisionedOwner {
            fn clear_diagnostics(&mut self) -> DiagClearSummary {
                DiagClearSummary::default()
            }
        }

        impl LiveObd2RequestOwner for ProvisionedOwner {
            fn fault_state(&self) -> FaultState {
                FaultState::default()
            }

            fn current_obd2_sensor_data(&self) -> Message {
                sensor_message(0, 60)
            }

            fn obd2_retained_history(&self) -> &Obd2RetainedDiagnosticHistory {
                &self.history
            }

            fn obd2_retained_history_mut(&mut self) -> &mut Obd2RetainedDiagnosticHistory {
                &mut self.history
            }

            fn fallback_obd2_vehicle_identity(&self) -> Obd2VehicleIdentity {
                Obd2VehicleIdentity::from_calibration_identity(CalibrationPackageIdentity {
                    schema_version: ecu_calibration::CalibrationSchemaVersion::CURRENT,
                    active_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
                    staged_base_revision: ecu_calibration::CalibrationRevision::new(0x1234_ABCD),
                    staged_revision: ecu_calibration::CalibrationRevision::new(0x5678_EF90),
                    staged_dirty: true,
                    checksum: ecu_calibration::CalibrationPackageChecksum::default(),
                })
            }

            fn provisioned_obd2_identity(&self) -> ProvisionedObd2Identity {
                self.provisioned
            }
        }

        let owner = ProvisionedOwner {
            history: Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90)),
            provisioned: ProvisionedObd2Identity::from_ascii(
                Some(b"trackvin000000001"),
                Some(b"trackcal000000001"),
                Some(b"m50-a1"),
            ),
        };

        let identity = owner.obd2_vehicle_identity();

        assert_eq!(&identity.vin, b"TRACKVIN000000001");
        assert_eq!(&identity.calibration_id, b"TRACKCAL000000001");
        assert_eq!(identity.ecu_name, [b'M', b'5', b'0', b'A', b'1', 0]);
        assert_ne!(identity.vin, owner.fallback_obd2_vehicle_identity().vin);
        assert_ne!(
            identity.calibration_id,
            owner.fallback_obd2_vehicle_identity().calibration_id
        );
    }

    #[test]
    fn retained_diagnostic_history_dispatch_inputs_carry_live_vehicle_identity() {
        let history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        let mut expected = obd2_vehicle_info_from_identity_signature(ecu_ts::TS_SIGNATURE);
        expected.identity_key_lifecycle = Some(CanObd2IdentityKeyLifecycleStatus::absent());

        assert_eq!(history.dispatch_inputs().vehicle_info, expected);
    }

    #[test]
    fn retained_diagnostic_history_observes_runtime_faults_and_captures_first_freeze_frame() {
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::SyncLoss,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::SyncLoss,
            },
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(123, 127),
        );
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::SensorOutOfRange,
                severity: FaultSeverity::Warning,
                cancel_reason: CancelReason::Manual,
            },
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(456, 140),
        );

        assert_eq!(
            history.stored_dtcs(),
            &[DiagCode::CamMissing, DiagCode::MapRange]
        );
        assert_eq!(history.freeze_frame_dtc(), Some(DiagCode::CamMissing));
        assert_eq!(
            history.freeze_frame_value_source(),
            Some(&sensor_message(123, 127))
        );
        assert_eq!(
            history.current_data_value_source(),
            &sensor_message(456, 140)
        );
        assert_eq!(
            history.current_diag_event().map(|event| event.code),
            Some(DiagCode::MapRange)
        );
        assert_eq!(
            history.freeze_frame_event().map(|event| event.code),
            Some(DiagCode::CamMissing)
        );
    }

    #[test]
    fn retained_diagnostic_history_uses_spec_fault_policy_for_safety_cut() {
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        let fault_state = FaultState {
            fault: FaultCode::SafetyCut,
            severity: FaultSeverity::Info,
            cancel_reason: CancelReason::Manual,
        };

        assert_eq!(
            obd2_fault_event_from_runtime_fault(fault_state),
            SpecFaultEvent {
                active: true,
                action: SpecFaultAction::Shutdown,
                persistence: ecu_spec::SpecFaultPersistence::LatchedUntilClear,
            }
        );

        history.observe_live_diagnostic(
            fault_state,
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(777, 155),
        );

        assert_eq!(history.stored_dtcs(), &[DiagCode::MapFailureHighLoad]);
        assert_eq!(
            history.freeze_frame_dtc(),
            Some(DiagCode::MapFailureHighLoad)
        );
        assert_eq!(
            history.freeze_frame_value_source(),
            Some(&sensor_message(777, 155))
        );
    }

    #[test]
    fn fault_transition_clear_uses_spec_clear_event_without_creating_retained_dtc() {
        let transition = CommonFaultTransitionTelemetry {
            changed: true,
            at_us: Micros::new(999),
            event: ecu_board_api::CommonFaultTransitionEventTelemetry {
                event_id: CommonFaultTransitionEventId::FaultCleared,
                severity: FaultSeverity::Info,
                action: ecu_board_api::CommonFaultTransitionAction::Cleared,
            },
            previous_fault: FaultCode::SensorOutOfRange,
            previous_severity: FaultSeverity::Warning,
            previous_cancel_reason: CancelReason::Manual,
            current_fault: FaultCode::None,
            current_severity: FaultSeverity::Info,
            current_cancel_reason: CancelReason::Manual,
        };

        assert_eq!(
            obd2_fault_event_from_fault_transition(transition),
            SpecFaultEvent {
                active: false,
                action: SpecFaultAction::Cleared,
                persistence: ecu_spec::SpecFaultPersistence::Inactive,
            }
        );
        assert_eq!(
            obd2_diag_event_from_fault_transition(transition, &sensor_message(999, 122)),
            None
        );
    }

    #[test]
    fn retained_history_snapshot_fault_code_codec_is_explicit() {
        for fault in [
            FaultCode::None,
            FaultCode::SyncLoss,
            FaultCode::SensorOutOfRange,
            FaultCode::CalibrationInvalid,
            FaultCode::SafetyCut,
            FaultCode::ActuatorFault,
        ] {
            assert_eq!(
                decode_snapshot_fault_code(encode_snapshot_fault_code(fault)),
                Some(fault)
            );
        }
        assert_eq!(decode_snapshot_fault_code(0xff), None);
    }

    #[test]
    fn retained_diagnostic_history_dispatch_inputs_track_readiness_and_clear_meaning() {
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::CalibrationInvalid,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::Manual,
            },
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(321, 111),
        );

        assert_eq!(
            history.readiness_monitor_inputs(),
            CanObd2ReadinessMonitorInputs {
                mil_requested: true,
                stored_dtc_count: 1,
                misfire_supported: false,
                misfire_complete: false,
                fuel_system_supported: false,
                fuel_system_complete: false,
                comprehensive_components_supported: true,
                comprehensive_components_complete: false,
            }
        );
        assert_eq!(
            history.dtc_clear_inputs(),
            CanObd2DtcClearInputs {
                cleared_dtc_count: 1,
                freeze_frame_cleared: true,
                readiness_reset: true,
            }
        );

        let dispatch_inputs = history.dispatch_inputs();
        assert_eq!(
            dispatch_inputs.current_data_value_source,
            Some(&sensor_message(321, 111))
        );
        assert_eq!(
            dispatch_inputs.freeze_frame_value_source,
            Some(&sensor_message(321, 111))
        );
        assert_eq!(dispatch_inputs.stored_dtcs, &[DiagCode::PersistCrcFault]);
        assert_eq!(
            dispatch_inputs.freeze_frame_dtc,
            Some(DiagCode::PersistCrcFault)
        );
    }

    #[test]
    fn retained_diagnostic_history_clear_resets_retained_faults_without_dropping_current_data() {
        let latest = sensor_message(987, 150);
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::ActuatorFault,
                severity: FaultSeverity::Warning,
                cancel_reason: CancelReason::Manual,
            },
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            latest.clone(),
        );

        history.clear();

        assert!(history.stored_dtcs().is_empty());
        assert_eq!(history.freeze_frame_dtc(), None);
        assert_eq!(history.freeze_frame_value_source(), None);
        assert_eq!(history.current_data_value_source(), &latest);
        assert_eq!(
            history.readiness_monitor_inputs(),
            CanObd2ReadinessMonitorInputs {
                mil_requested: false,
                stored_dtc_count: 0,
                misfire_supported: false,
                misfire_complete: false,
                fuel_system_supported: false,
                fuel_system_complete: false,
                comprehensive_components_supported: true,
                comprehensive_components_complete: true,
            }
        );
        assert!(history.current_diag_event().is_none());
        assert!(history.freeze_frame_event().is_none());
    }

    #[test]
    fn multi_service_transport_service_dispatches_readiness_from_shared_live_owner() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x01,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let live = sensor_message(123, 127);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::SyncLoss,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::SyncLoss,
            },
            current_obd2_sensor_data: live,
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x01 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::ReadinessMonitorPositive
                );
                assert_eq!(
                    dispatch.response,
                    CanObd2ResponseFrame {
                        service: 0x41,
                        parameter_id: Some(0x01),
                        negative_response_code: None,
                        payload_len: 4,
                        payload: [0x81, 0x21, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        assert_eq!(
            owner.obd2_retained_history().stored_dtcs(),
            &[DiagCode::CamMissing]
        );
        assert_eq!(service.transport().tx.len(), 1);
    }

    #[test]
    fn multi_service_transport_service_dispatches_segmented_vin_in_order() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let live = sensor_message(123, 127);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: live,
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x09 VIN request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::SegmentedDispatch(dispatch) => {
                assert_eq!(dispatch.info_type_id, 0x02);
                assert_eq!(dispatch.total_payload_len, 17);
                assert_eq!(dispatch.segment_count, 3);
            }
            other => panic!("unexpected outcome: {other:?}"),
        }
        assert_eq!(
            service.transport().tx,
            std::vec![
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 0,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"C1234A",
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 1,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"BCD000",
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 2,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 5,
                    segment: [b'0', b'0', b'0', b'0', b'0', 0],
                },
            ]
        );
    }

    #[test]
    fn multi_service_transport_service_dispatches_installed_provisioned_identity_segments() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x02),
            payload_len: 0,
            payload: [0; 6],
        });
        transport.rx.push_back(Message::Obd2Request {
            service: 0x09,
            parameter_id: Some(0x04),
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let live = sensor_message(123, 127);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut owner = ProvisionedMockLiveOwner {
            inner: MockLiveOwner {
                fault_state: FaultState {
                    fault: FaultCode::None,
                    severity: FaultSeverity::Info,
                    cancel_reason: CancelReason::Manual,
                },
                current_obd2_sensor_data: live,
                retained_history: Obd2RetainedDiagnosticHistory::new(initial),
                current_obd2_diag_event: None,
                obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
                recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
                clear_summary: DiagClearSummary::default(),
                clear_count: 0,
            },
            provisioned: Obd2ProvisionedIdentityRecord::from_ascii(
                Some(b"trackvin000000003"),
                Some(b"trackcal000000003"),
                Some(b"m50-c3"),
            )
            .into_provisioned_identity(),
        };

        let vin_outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x09 VIN request should succeed")
            .expect("VIN request should be consumed");
        let calibration_id_outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x09 calibration ID request should succeed")
            .expect("calibration ID request should be consumed");

        match vin_outcome {
            Obd2MultiServiceTransportServiceOutcome::SegmentedDispatch(dispatch) => {
                assert_eq!(dispatch.info_type_id, 0x02);
                assert_eq!(dispatch.total_payload_len, 17);
                assert_eq!(dispatch.segment_count, 3);
            }
            other => panic!("unexpected VIN outcome: {other:?}"),
        }
        match calibration_id_outcome {
            Obd2MultiServiceTransportServiceOutcome::SegmentedDispatch(dispatch) => {
                assert_eq!(dispatch.info_type_id, 0x04);
                assert_eq!(dispatch.total_payload_len, 17);
                assert_eq!(dispatch.segment_count, 3);
            }
            other => panic!("unexpected calibration ID outcome: {other:?}"),
        }
        assert_eq!(
            service.transport().tx,
            std::vec![
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 0,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"TRACKV",
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 1,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"IN0000",
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x02),
                    sequence_index: 2,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 5,
                    segment: [b'0', b'0', b'0', b'0', b'3', 0],
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x04),
                    sequence_index: 0,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"TRACKC",
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x04),
                    sequence_index: 1,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 6,
                    segment: *b"AL0000",
                },
                Message::Obd2SegmentedResponse {
                    service: 0x49,
                    parameter_id: Some(0x04),
                    sequence_index: 2,
                    segment_count: 3,
                    total_payload_len: 17,
                    segment_len: 5,
                    segment: [b'0', b'0', b'0', b'0', b'3', 0],
                },
            ]
        );
    }

    #[test]
    fn retained_diagnostic_history_snapshot_roundtrip_preserves_dispatch_meaning() {
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::CalibrationInvalid,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::Manual,
            },
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(321, 111),
        );
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::SensorOutOfRange,
                severity: FaultSeverity::Warning,
                cancel_reason: CancelReason::Manual,
            },
            None,
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(654, 118),
        );

        let snapshot = history.snapshot();
        let restored = Obd2RetainedDiagnosticHistory::from_snapshot(snapshot.clone());

        assert_eq!(restored.snapshot(), snapshot);
        assert_eq!(
            restored.dispatch_inputs().stored_dtcs,
            &[DiagCode::PersistCrcFault, DiagCode::MapRange],
        );
        assert_eq!(
            restored.dispatch_inputs().freeze_frame_dtc,
            Some(DiagCode::PersistCrcFault),
        );
        assert_eq!(
            restored.dispatch_inputs().freeze_frame_value_source,
            Some(&sensor_message(321, 111)),
        );
        assert_eq!(
            restored.dispatch_inputs().current_data_value_source,
            Some(&sensor_message(654, 118)),
        );
    }

    #[test]
    fn retained_diagnostic_history_snapshot_codec_roundtrip_preserves_snapshot() {
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::CalibrationInvalid,
                severity: ecu_domain::FaultSeverity::Critical,
                cancel_reason: ecu_domain::CancelReason::Manual,
            },
            Some(DiagEvent {
                code: DiagCode::PersistCrcFault,
                timestamp: Micros::new(321),
                source: DiagSource::User,
                context: Some(7),
                start_us: 111,
                end_us: 222,
            }),
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(321, 111),
        );
        let snapshot = history.snapshot();
        let mut bytes = [0u8; OBD2_RETAINED_HISTORY_SNAPSHOT_BYTES];
        let len = encode_obd2_retained_history_snapshot(&snapshot, &mut bytes).unwrap();
        let decoded = decode_obd2_retained_history_snapshot(&bytes[..len]).unwrap();
        assert_eq!(decoded, snapshot);
    }

    #[test]
    fn retained_diagnostic_history_sidecar_codec_roundtrip_preserves_snapshot() {
        let mut history = Obd2RetainedDiagnosticHistory::new(sensor_message(1, 90));
        history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::CalibrationInvalid,
                severity: ecu_domain::FaultSeverity::Critical,
                cancel_reason: ecu_domain::CancelReason::Manual,
            },
            Some(DiagEvent {
                code: DiagCode::PersistCrcFault,
                timestamp: Micros::new(654),
                source: DiagSource::User,
                context: Some(11),
                start_us: 222,
                end_us: 333,
            }),
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(654, 122),
        );
        let snapshot = history.snapshot();
        let mut bytes = [0u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES];
        let len = encode_obd2_retained_history_sidecar(&snapshot, &mut bytes).unwrap();
        let decoded = decode_obd2_retained_history_sidecar(&bytes[..len]).unwrap();
        assert_eq!(decoded, snapshot);

        let mut oversized = [0xFFu8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES + 5];
        assert!(encode_obd2_retained_history_sidecar_prefix(
            &snapshot,
            &mut oversized
        ));
        assert_eq!(
            decode_obd2_retained_history_sidecar_prefix(&oversized),
            Some(snapshot.clone())
        );
        assert_eq!(
            oversized[OBD2_RETAINED_HISTORY_SIDECAR_BYTES..],
            [0xFFu8; 5]
        );

        let staged = stage_obd2_retained_history_sidecar_bytes(&snapshot).unwrap();
        assert_eq!(
            decode_staged_obd2_retained_history_sidecar(&staged),
            Some(snapshot.clone())
        );
        assert_eq!(
            stage_optional_obd2_retained_history_sidecar_bytes(Some(&snapshot)),
            Some(Some(staged))
        );
        assert_eq!(
            stage_optional_obd2_retained_history_sidecar_bytes(None),
            Some(None)
        );
        let mut observed = [0u8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES];
        assert!(write_obd2_retained_history_snapshot_with(
            &snapshot,
            |bytes| {
                observed.copy_from_slice(bytes);
            }
        ));
        assert_eq!(
            decode_staged_obd2_retained_history_sidecar(&observed),
            Some(snapshot.clone())
        );
        let mut installed = [0xFFu8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES];
        assert!(install_obd2_retained_history_snapshot_halfwords_with(
            &snapshot,
            |offset, value| {
                installed[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
            }
        ));
        assert_eq!(
            decode_staged_obd2_retained_history_sidecar(&installed),
            Some(snapshot.clone())
        );
        let mut optional_called = false;
        assert_eq!(
            write_optional_obd2_retained_history_snapshot_with(None, |_| {
                optional_called = true;
            }),
            Some(())
        );
        assert!(!optional_called);
        let mut optional_halfword_called = false;
        assert_eq!(
            install_optional_obd2_retained_history_snapshot_halfwords_with(None, |_, _| {
                optional_halfword_called = true;
            }),
            Some(())
        );
        assert!(!optional_halfword_called);
        assert_eq!(
            read_obd2_retained_history_snapshot_with(|out| out.copy_from_slice(&staged)),
            Some(snapshot.clone())
        );

        let mut padded = [0xEEu8; OBD2_RETAINED_HISTORY_SIDECAR_BYTES + 3];
        assert!(encode_obd2_retained_history_sidecar_at(
            &snapshot,
            &mut padded,
            3
        ));
        assert_eq!(
            decode_obd2_retained_history_sidecar_at(&padded, 3),
            Some(snapshot)
        );
        assert_eq!(padded[..3], [0xEEu8; 3]);
    }

    #[test]
    fn retained_diagnostic_history_sidecar_codec_preserves_pressure_and_lambda_codes() {
        let mut stored_dtcs = [DiagCode::MapRange; OBD2_STORED_DTC_CAP];
        stored_dtcs[0] = DiagCode::OilPressureLow;
        stored_dtcs[1] = DiagCode::FuelPressureLow;
        stored_dtcs[2] = DiagCode::LambdaInvalid;
        let snapshot = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: sensor_message(10, 100),
            freeze_frame_value_source: Some(sensor_message(20, 101)),
            stored_dtcs,
            stored_dtc_count: 3,
            freeze_frame_dtc: Some(DiagCode::LambdaInvalid),
            current_diag_event: Some(DiagEvent {
                code: DiagCode::OilPressureLow,
                timestamp: Micros::new(10),
                source: DiagSource::Sensor,
                context: Some(900),
                start_us: 10,
                end_us: 0,
            }),
            freeze_frame_event: Some(DiagEvent {
                code: DiagCode::FuelPressureLow,
                timestamp: Micros::new(20),
                source: DiagSource::Sensor,
                context: Some(2_400),
                start_us: 20,
                end_us: 0,
            }),
            last_fault: FaultCode::SensorOutOfRange,
            last_observed_diag_code: Some(DiagCode::LambdaInvalid),
            last_observed_diag_timestamp_us: 30,
        };

        let staged = stage_obd2_retained_history_sidecar_bytes(&snapshot).unwrap();
        assert_eq!(
            decode_staged_obd2_retained_history_sidecar(&staged),
            Some(snapshot)
        );
    }

    #[test]
    fn retained_history_persistence_state_restores_and_flushes_owner_snapshot_changes() {
        let initial = sensor_message(1, 90);
        let restored = sensor_message(444, 120);
        let mut retained_history = Obd2RetainedDiagnosticHistory::new(initial.clone());
        retained_history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::CalibrationInvalid,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::Manual,
            },
            Some(DiagEvent {
                code: DiagCode::PersistCrcFault,
                timestamp: Micros::new(444),
                source: DiagSource::User,
                context: Some(9),
                start_us: 333,
                end_us: 444,
            }),
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            restored.clone(),
        );
        let loaded_snapshot = retained_history.snapshot();
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: initial.clone(),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };
        let mut store = MockRetainedHistoryStore {
            loaded_snapshot: Some(loaded_snapshot.clone()),
            ..MockRetainedHistoryStore::default()
        };

        let mut persistence =
            Obd2RetainedHistoryPersistenceState::restore_from_store(&mut owner, &store);
        assert_eq!(owner.obd2_retained_history_snapshot(), loaded_snapshot);

        assert_eq!(
            persistence.persist_if_changed(&owner, &mut store),
            Ok(false)
        );
        assert_eq!(store.save_count, 0);

        owner.retained_history.observe_live_diagnostic(
            FaultState {
                fault: FaultCode::SensorOutOfRange,
                severity: FaultSeverity::Warning,
                cancel_reason: CancelReason::Manual,
            },
            Some(DiagEvent {
                code: DiagCode::LowVoltage,
                timestamp: Micros::new(777),
                source: DiagSource::Sensor,
                context: Some(7_900),
                start_us: 700,
                end_us: 777,
            }),
            [None; DIAG_LOG_ENTRY_COUNT],
            [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            sensor_message(777, 121),
        );

        assert_eq!(persistence.persist_if_changed(&owner, &mut store), Ok(true));
        assert_eq!(store.save_count, 1);
        assert_eq!(
            store.saved_snapshot,
            Some(owner.obd2_retained_history_snapshot())
        );
    }

    #[test]
    fn retained_history_flash_rewrite_helpers_preserve_or_replace_snapshot_meaning() {
        let preserved = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: sensor_message(100, 90),
            freeze_frame_value_source: None,
            stored_dtcs: [DiagCode::MapRange; OBD2_STORED_DTC_CAP],
            stored_dtc_count: 1,
            freeze_frame_dtc: Some(DiagCode::MapRange),
            current_diag_event: None,
            freeze_frame_event: None,
            last_fault: FaultCode::SensorOutOfRange,
            last_observed_diag_code: Some(DiagCode::MapRange),
            last_observed_diag_timestamp_us: 100,
        };
        let saved = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: sensor_message(200, 91),
            freeze_frame_value_source: Some(sensor_message(201, 92)),
            stored_dtcs: [DiagCode::PersistCrcFault; OBD2_STORED_DTC_CAP],
            stored_dtc_count: 1,
            freeze_frame_dtc: Some(DiagCode::PersistCrcFault),
            current_diag_event: None,
            freeze_frame_event: None,
            last_fault: FaultCode::CalibrationInvalid,
            last_observed_diag_code: Some(DiagCode::PersistCrcFault),
            last_observed_diag_timestamp_us: 200,
        };

        let preserved_rewrite = prepare_retained_history_preserved_page_rewrite(
            [1u8, 2, 3],
            Some(preserved.clone()),
            |pages| {
                pages[1] = 9;
                Ok::<_, ()>(())
            },
        )
        .unwrap();
        assert_eq!(preserved_rewrite.pages, [1u8, 9, 3]);
        assert_eq!(preserved_rewrite.snapshot, Some(preserved));

        let explicit_rewrite = prepare_retained_history_snapshot_rewrite([4u8, 5, 6], &saved);
        assert_eq!(explicit_rewrite.pages, [4u8, 5, 6]);
        assert_eq!(explicit_rewrite.snapshot, Some(saved));
    }

    #[test]
    fn persist_retained_history_flash_rewrite_threads_current_state_into_commit() {
        let preserved = Obd2RetainedDiagnosticHistorySnapshot {
            current_data_value_source: sensor_message(100, 90),
            freeze_frame_value_source: None,
            stored_dtcs: [DiagCode::MapRange; OBD2_STORED_DTC_CAP],
            stored_dtc_count: 1,
            freeze_frame_dtc: Some(DiagCode::MapRange),
            current_diag_event: None,
            freeze_frame_event: None,
            last_fault: FaultCode::SensorOutOfRange,
            last_observed_diag_code: Some(DiagCode::MapRange),
            last_observed_diag_timestamp_us: 100,
        };

        let mut committed_pages = None;
        let mut committed_snapshot = None;
        persist_retained_history_flash_rewrite(
            [1u8, 2, 3],
            Some(preserved.clone()),
            |pages, snapshot| {
                prepare_retained_history_preserved_page_rewrite(pages, snapshot, |pages| {
                    pages[2] = 7;
                    Ok::<_, ()>(())
                })
            },
            |rewrite| {
                committed_pages = Some(rewrite.pages);
                committed_snapshot = rewrite.snapshot;
                Ok::<_, ()>(())
            },
        )
        .unwrap();

        assert_eq!(committed_pages, Some([1u8, 2, 7]));
        assert_eq!(committed_snapshot, Some(preserved));
    }

    #[derive(Default)]
    struct TestRetainedPages {
        fuel: [u8; 2],
        ign: [u8; 3],
        angles: [u8; 4],
    }

    impl Obd2RetainedHistoryPagesMut for TestRetainedPages {
        fn fuel_mut(&mut self) -> &mut [u8] {
            &mut self.fuel
        }

        fn ign_mut(&mut self) -> &mut [u8] {
            &mut self.ign
        }

        fn angles_mut(&mut self) -> &mut [u8] {
            &mut self.angles
        }
    }

    #[test]
    fn retained_history_page_update_helper_routes_keys_and_guards_lengths() {
        let mut pages = TestRetainedPages::default();
        apply_retained_history_page_update(&mut pages, b"fuel", &[7, 8]).unwrap();
        apply_retained_history_page_update(&mut pages, b"ign", &[1, 2, 3]).unwrap();
        assert_eq!(pages.fuel, [7, 8]);
        assert_eq!(pages.ign, [1, 2, 3]);
        assert_eq!(
            apply_retained_history_page_update(&mut pages, b"angles", &[4, 5]),
            Err(Obd2RetainedHistoryPageUpdateError::InvalidLength)
        );
        assert_eq!(
            apply_retained_history_page_update(&mut pages, b"bogus", &[1]),
            Err(Obd2RetainedHistoryPageUpdateError::UnknownKey)
        );
    }

    #[test]
    fn multi_service_transport_service_clears_shared_history_on_mode04() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let live = sensor_message(444, 135);
        let initial = sensor_message(1, 90);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::CalibrationInvalid,
                severity: FaultSeverity::Critical,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: live,
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary {
                cleared_active_count: 1,
                cleared_log_entries: 1,
                emergency_cleared: false,
            },
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x04 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::DtcClear {
                clear_summary,
                dispatch,
            } => {
                assert_eq!(clear_summary, owner.clear_summary);
                assert_eq!(
                    dispatch.verdict,
                    CanObd2MultiServiceDispatchVerdict::DtcClearPositive
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        assert!(owner.obd2_retained_history().stored_dtcs().is_empty());
        assert_eq!(owner.obd2_retained_history().freeze_frame_dtc(), None);
        assert_eq!(owner.clear_count, 1);
    }

    #[test]
    fn multi_service_transport_service_prefers_explicit_diag_event_ingress() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_message(555, 120),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: Some(DiagEvent {
                code: DiagCode::LowVoltage,
                timestamp: Micros::new(555),
                source: DiagSource::Safety,
                context: Some(115),
                start_us: 555,
                end_us: 555,
            }),
            obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
            recent_obd2_diag_events: [
                Some(DiagEvent {
                    code: DiagCode::LowVoltage,
                    timestamp: Micros::new(555),
                    source: DiagSource::Safety,
                    context: Some(115),
                    start_us: 555,
                    end_us: 555,
                }),
                None,
            ],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.response,
                    CanObd2ResponseFrame {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 2,
                        payload: [0x05, 0x62, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        let retained = owner.obd2_retained_history();
        assert_eq!(retained.stored_dtcs(), &[DiagCode::LowVoltage]);
        assert_eq!(retained.freeze_frame_dtc(), Some(DiagCode::LowVoltage));
        assert_eq!(
            retained
                .current_diag_event()
                .and_then(|event| event.context),
            Some(115)
        );
        assert_eq!(
            retained.freeze_frame_event().map(|event| event.source),
            Some(DiagSource::Safety)
        );
    }

    #[test]
    fn multi_service_transport_service_ingests_recent_diag_transition_without_current_fault() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_message(777, 118),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: [None; DIAG_LOG_ENTRY_COUNT],
            recent_obd2_diag_events: [
                Some(DiagEvent {
                    code: DiagCode::MapRange,
                    timestamp: Micros::new(777),
                    source: DiagSource::Sensor,
                    context: Some(900),
                    start_us: 700,
                    end_us: 777,
                }),
                None,
            ],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.response,
                    CanObd2ResponseFrame {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 2,
                        payload: [0x01, 0x08, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        let retained = owner.obd2_retained_history();
        assert_eq!(retained.stored_dtcs(), &[DiagCode::MapRange]);
        assert_eq!(retained.freeze_frame_dtc(), Some(DiagCode::MapRange));
        assert!(retained.current_diag_event().is_none());
        assert_eq!(
            retained
                .freeze_frame_event()
                .and_then(|event| event.context),
            Some(900)
        );
    }

    #[test]
    fn multi_service_transport_service_ingests_shared_diag_log_owner_path() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut diag_log_events = [None; DIAG_LOG_ENTRY_COUNT];
        diag_log_events[0] = Some(DiagEvent {
            code: DiagCode::TpsRange,
            timestamp: Micros::new(888),
            source: DiagSource::Sensor,
            context: Some(42),
            start_us: 800,
            end_us: 888,
        });
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_message(888, 119),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: diag_log_events,
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.response,
                    CanObd2ResponseFrame {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 2,
                        payload: [0x01, 0x22, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        let retained = owner.obd2_retained_history();
        assert_eq!(retained.stored_dtcs(), &[DiagCode::TpsRange]);
        assert_eq!(retained.freeze_frame_dtc(), Some(DiagCode::TpsRange));
        assert_eq!(
            retained
                .freeze_frame_event()
                .and_then(|event| event.context),
            Some(42)
        );
    }

    #[test]
    fn multi_service_transport_service_retains_map_recovery_from_shared_diag_log_owner_path() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut diag_log_events = [None; DIAG_LOG_ENTRY_COUNT];
        diag_log_events[0] = Some(DiagEvent {
            code: DiagCode::MapRange,
            timestamp: Micros::new(3_000_200),
            source: DiagSource::Sensor,
            context: Some(150),
            start_us: 100,
            end_us: 3_000_200,
        });
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_message(3_000_200, 119),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: diag_log_events,
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(dispatch.response.service, 0x43);
                assert_eq!(dispatch.response.parameter_id, None);
                assert_eq!(dispatch.response.negative_response_code, None);
                assert_eq!(dispatch.response.payload_len, 2);
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        let retained = owner.obd2_retained_history();
        assert_eq!(retained.stored_dtcs(), &[DiagCode::MapRange]);
        assert_eq!(retained.freeze_frame_dtc(), Some(DiagCode::MapRange));
        assert_eq!(retained.freeze_frame_event(), diag_log_events[0]);
    }

    #[test]
    fn multi_service_transport_service_retains_persist_crc_fault_from_shared_diag_log_owner_path() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut diag_log_events = [None; DIAG_LOG_ENTRY_COUNT];
        diag_log_events[0] = Some(DiagEvent {
            code: DiagCode::PersistCrcFault,
            timestamp: Micros::new(0),
            source: DiagSource::User,
            context: None,
            start_us: 0,
            end_us: 0,
        });
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_message(0, 119),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: diag_log_events,
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(dispatch.response.service, 0x43);
                assert_eq!(dispatch.response.parameter_id, None);
                assert_eq!(dispatch.response.negative_response_code, None);
                assert_eq!(dispatch.response.payload_len, 2);
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        let retained = owner.obd2_retained_history();
        assert_eq!(retained.stored_dtcs(), &[DiagCode::PersistCrcFault]);
        assert_eq!(retained.freeze_frame_dtc(), Some(DiagCode::PersistCrcFault));
        assert_eq!(retained.freeze_frame_event(), diag_log_events[0]);
    }

    #[test]
    fn multi_service_transport_service_retains_low_voltage_from_shared_diag_log_owner_path() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let sensor_data = Message::SensorData {
            map_kpa_x10: 900,
            tps_percent: 25,
            iat_offset: 65,
            clt_offset: 70,
            voltage_x10: 79,
            lambda_x100: 100,
            flags: 0,
            timestamp_us: 901,
        };
        let mut service = Obd2MultiServiceTransportService::new(transport);
        let mut diag_log_events = [None; DIAG_LOG_ENTRY_COUNT];
        diag_log_events[0] = Some(DiagEvent {
            code: DiagCode::LowVoltage,
            timestamp: Micros::new(901),
            source: DiagSource::Sensor,
            context: Some(7_900),
            start_us: 901,
            end_us: 0,
        });
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_data.clone(),
            retained_history: Obd2RetainedDiagnosticHistory::new(sensor_data.clone()),
            current_obd2_diag_event: None,
            obd2_diag_log_events: diag_log_events,
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut owner)
            .expect("mode 0x03 request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(dispatch.response.service, 0x43);
                assert_eq!(dispatch.response.parameter_id, None);
                assert_eq!(dispatch.response.negative_response_code, None);
                assert_eq!(dispatch.response.payload_len, 2);
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        let retained = owner.obd2_retained_history();
        assert_eq!(retained.stored_dtcs(), &[DiagCode::LowVoltage]);
        assert_eq!(retained.freeze_frame_dtc(), Some(DiagCode::LowVoltage));
        assert_eq!(retained.freeze_frame_event(), diag_log_events[0]);
        assert_eq!(retained.freeze_frame_value_source(), Some(&sensor_data));
    }

    #[test]
    fn multi_service_transport_service_recreation_keeps_owner_retained_history() {
        let mut first_transport = MockTransport::default();
        first_transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let initial = sensor_message(1, 90);
        let mut first_service = Obd2MultiServiceTransportService::new(first_transport);
        let mut diag_log_events = [None; DIAG_LOG_ENTRY_COUNT];
        diag_log_events[0] = Some(DiagEvent {
            code: DiagCode::PersistCrcFault,
            timestamp: Micros::new(0),
            source: DiagSource::User,
            context: None,
            start_us: 0,
            end_us: 0,
        });
        let mut owner = MockLiveOwner {
            fault_state: FaultState {
                fault: FaultCode::None,
                severity: FaultSeverity::Info,
                cancel_reason: CancelReason::Manual,
            },
            current_obd2_sensor_data: sensor_message(0, 119),
            retained_history: Obd2RetainedDiagnosticHistory::new(initial),
            current_obd2_diag_event: None,
            obd2_diag_log_events: diag_log_events,
            recent_obd2_diag_events: [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP],
            clear_summary: DiagClearSummary::default(),
            clear_count: 0,
        };

        let first = first_service
            .pump_once(&mut owner)
            .expect("first service should dispatch")
            .expect("stored-dtc request should be consumed");
        match first {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.response,
                    CanObd2ResponseFrame {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 2,
                        payload: [0x06, 0x01, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected first outcome: {other:?}"),
        }

        owner.obd2_diag_log_events = [None; DIAG_LOG_ENTRY_COUNT];
        owner.recent_obd2_diag_events = [None; OBD2_LIVE_DIAG_EVENT_INGRESS_CAP];
        owner.current_obd2_diag_event = None;

        let mut second_transport = MockTransport::default();
        second_transport.rx.push_back(Message::Obd2Request {
            service: 0x03,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let mut second_service = Obd2MultiServiceTransportService::new(second_transport);

        let second = second_service
            .pump_once(&mut owner)
            .expect("recreated service should dispatch")
            .expect("stored-dtc request should be consumed");
        match second {
            Obd2MultiServiceTransportServiceOutcome::Dispatch(dispatch) => {
                assert_eq!(
                    dispatch.response,
                    CanObd2ResponseFrame {
                        service: 0x43,
                        parameter_id: None,
                        negative_response_code: None,
                        payload_len: 2,
                        payload: [0x06, 0x01, 0, 0, 0, 0],
                    }
                );
            }
            other => panic!("unexpected second outcome: {other:?}"),
        }

        assert_eq!(
            owner.obd2_retained_history().stored_dtcs(),
            &[DiagCode::PersistCrcFault]
        );
        assert_eq!(
            owner.obd2_retained_history().freeze_frame_dtc(),
            Some(DiagCode::PersistCrcFault)
        );
    }

    #[test]
    fn pump_once_routes_valid_obd2_clear_request_into_live_state() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = Obd2TransportService::new(transport);
        let mut state = EcuState::new();
        state.set_emergency_trigger_map_oob(true);
        state.set_emergency_mode(true);
        state.diag_map.latch(Micros::new(10));
        state.diag_log_mut().push(DiagEvent {
            code: DiagCode::MapRange,
            timestamp: Micros::new(100),
            source: DiagSource::Sensor,
            context: Some(100),
            start_us: 10,
            end_us: 40,
        });

        let outcome = service
            .pump_once(&mut state)
            .expect("valid request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2TransportServiceOutcome::DtcClear(surface) => {
                assert_eq!(surface.clear_summary.cleared_active_count, 1);
                assert_eq!(surface.clear_summary.cleared_log_entries, 1);
                assert!(surface.clear_summary.emergency_cleared);
                assert_eq!(
                    surface.transport.verdict,
                    CanObd2DtcClearVerdict {
                        cleared_dtc_count: 2,
                        freeze_frame_cleared: true,
                        readiness_reset: true,
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        assert!(!state.diag_map.is_active());
        assert!(!state.emergency_mode());
        assert!(state.diag_log().events.iter().all(|entry| entry.is_none()));
        assert_eq!(service.transport().tx.len(), 1);
        assert_eq!(
            service.transport().tx[0],
            Message::Obd2Response {
                service: 0x44,
                parameter_id: None,
                negative_response_code: None,
                payload_len: 0,
                payload: [0; 6],
            }
        );
        assert_eq!(service.transport().poll_count, 1);
    }

    #[test]
    fn pump_once_rejects_parameterized_clear_request_without_mutation() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x04,
            parameter_id: Some(0x01),
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = Obd2TransportService::new(transport);
        let mut state = EcuState::new();
        state.set_emergency_trigger_map_oob(true);
        state.set_emergency_mode(true);
        state.diag_map.latch(Micros::new(10));
        state.diag_log_mut().push(DiagEvent {
            code: DiagCode::MapRange,
            timestamp: Micros::new(100),
            source: DiagSource::Sensor,
            context: Some(100),
            start_us: 10,
            end_us: 40,
        });

        let error = service
            .pump_once(&mut state)
            .expect_err("parameterized clear should fail");

        assert_eq!(
            error,
            Obd2TransportServiceError::Compose(
                CanObd2DtcClearResponseError::UnexpectedParameterId { parameter_id: 0x01 }
            )
        );
        assert!(state.diag_map.is_active());
        assert!(state.emergency_mode());
        assert_eq!(
            state
                .diag_log()
                .events
                .iter()
                .filter(|entry| entry.is_some())
                .count(),
            1
        );
        assert!(service.transport().tx.is_empty());
    }

    #[test]
    fn pump_once_uses_generic_diagnostic_clear_owner() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Obd2Request {
            service: 0x04,
            parameter_id: None,
            payload_len: 0,
            payload: [0; 6],
        });
        let mut service = Obd2TransportService::new(transport);
        let mut state = MockClearOwner {
            clear_summary: DiagClearSummary {
                cleared_active_count: 1,
                cleared_log_entries: 2,
                emergency_cleared: false,
            },
            clear_count: 0,
        };

        let outcome = service
            .pump_once(&mut state)
            .expect("valid request should succeed")
            .expect("request should be consumed");

        match outcome {
            Obd2TransportServiceOutcome::DtcClear(surface) => {
                assert_eq!(surface.clear_summary, state.clear_summary);
                assert_eq!(
                    surface.transport.verdict,
                    CanObd2DtcClearVerdict {
                        cleared_dtc_count: 3,
                        freeze_frame_cleared: true,
                        readiness_reset: true,
                    }
                );
            }
            other => panic!("unexpected outcome: {other:?}"),
        }

        assert_eq!(state.clear_count, 1);
        assert_eq!(service.transport().tx.len(), 1);
    }

    #[test]
    fn pump_once_ignores_unrelated_transport_traffic() {
        let mut transport = MockTransport::default();
        transport.rx.push_back(Message::Heartbeat {
            node_id: 7,
            uptime_seconds: 42,
            status: 0,
            error_count: 0,
            cpu_usage: 10,
        });
        let mut service = Obd2TransportService::new(transport);
        let mut state = EcuState::new();

        let outcome = service
            .pump_once(&mut state)
            .expect("unrelated traffic should not error")
            .expect("message should be consumed");

        assert_eq!(
            outcome,
            Obd2TransportServiceOutcome::Ignored(Message::Heartbeat {
                node_id: 7,
                uptime_seconds: 42,
                status: 0,
                error_count: 0,
                cpu_usage: 10,
            })
        );
        assert!(!state.diag_map.is_active());
        assert!(!state.diag_tps.is_active());
        assert!(!state.diag_cam.is_active());
        assert!(!state.emergency_mode());
        assert!(service.transport().tx.is_empty());
    }
}
