use ecu_target_common::adapter::{CommonObservabilityRecord, CommonObservabilityRecordKind};
use ecu_ts::proto::{self, Cmd};
use ecu_ts::server::{
    encode_compatibility_info_reply, encode_tooth_composite_log_reply, encode_version_info_reply,
    BenchToolingOwner, CompatibilityInfoReport, ToothCompositeLogEntry, ToothCompositeLogKind,
};

pub(crate) struct TsBenchCommandOwner<
    OutputTest,
    ToothStats,
    ToothCompositeLog,
    VersionInfo,
    CompatibilityInfo,
    Reboot,
> {
    output_test: OutputTest,
    tooth_stats: ToothStats,
    tooth_composite_log: ToothCompositeLog,
    version_info: VersionInfo,
    compatibility_info: CompatibilityInfo,
    reboot: Reboot,
}

impl<OutputTest, ToothStats, ToothCompositeLog, VersionInfo, CompatibilityInfo, Reboot>
    TsBenchCommandOwner<
        OutputTest,
        ToothStats,
        ToothCompositeLog,
        VersionInfo,
        CompatibilityInfo,
        Reboot,
    >
{
    pub(crate) fn new(
        output_test: OutputTest,
        tooth_stats: ToothStats,
        tooth_composite_log: ToothCompositeLog,
        version_info: VersionInfo,
        compatibility_info: CompatibilityInfo,
        reboot: Reboot,
    ) -> Self {
        Self {
            output_test,
            tooth_stats,
            tooth_composite_log,
            version_info,
            compatibility_info,
            reboot,
        }
    }
}

impl<OutputTest, ToothStats, ToothCompositeLog, VersionInfo, CompatibilityInfo, Reboot>
    BenchToolingOwner
    for TsBenchCommandOwner<
        OutputTest,
        ToothStats,
        ToothCompositeLog,
        VersionInfo,
        CompatibilityInfo,
        Reboot,
    >
where
    OutputTest: FnMut(&[u8], &mut [u8]) -> Option<usize>,
    ToothStats: FnMut(&[u8], &mut [u8]) -> Option<usize>,
    ToothCompositeLog: FnMut(&[u8], &mut [u8]) -> Option<usize>,
    VersionInfo: FnMut(&[u8], &mut [u8]) -> Option<usize>,
    CompatibilityInfo: FnMut(&[u8], &mut [u8]) -> Option<usize>,
    Reboot: FnMut(&[u8], &mut [u8]) -> Option<usize>,
{
    fn handle_output_test(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        (self.output_test)(payload, out)
    }

    fn handle_tooth_stats(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        (self.tooth_stats)(payload, out)
    }

    fn handle_tooth_composite_log(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        (self.tooth_composite_log)(payload, out)
    }

    fn handle_version_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        (self.version_info)(payload, out)
    }

    fn handle_compatibility_info(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        (self.compatibility_info)(payload, out)
    }

    fn handle_reboot(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        (self.reboot)(payload, out)
    }
}

pub(crate) fn handle_output_test_cmd(
    payload: &[u8],
    mut fire_output: impl FnMut(u8, u32, u32, u8),
    out: &mut [u8],
) -> Option<usize> {
    if payload.len() < 6 {
        return None;
    }

    let chan = payload[0];
    let on_ms = u16::from_le_bytes([payload[1], payload[2]]) as u32;
    let off_ms = u16::from_le_bytes([payload[3], payload[4]]) as u32;
    let reps = payload[5];
    fire_output(chan, on_ms, off_ms, reps);

    proto::encode_reply(Cmd::OutputTest, b"OK", out)
}

pub(crate) fn encode_tooth_stats_reply(rpm: u16, synced: bool, out: &mut [u8]) -> Option<usize> {
    let synced: u8 = u8::from(synced);
    let mut buf = [0u8; 3];
    buf[0] = (rpm & 0xff) as u8;
    buf[1] = (rpm >> 8) as u8;
    buf[2] = synced;
    proto::encode_reply(Cmd::ToothStats, &buf, out)
}

pub(crate) fn handle_version_info_cmd(
    payload: &[u8],
    signature: &[u8],
    runtime_build_id: u32,
    hardware_target_id: u16,
    out: &mut [u8],
) -> Option<usize> {
    if !payload.is_empty() {
        return None;
    }
    encode_version_info_reply(signature, runtime_build_id, hardware_target_id, out)
}

pub(crate) fn handle_compatibility_info_cmd(
    payload: &[u8],
    report: CompatibilityInfoReport,
    out: &mut [u8],
) -> Option<usize> {
    if !payload.is_empty() {
        return None;
    }
    encode_compatibility_info_reply(report, out)
}

pub(crate) fn handle_reboot_cmd(
    payload: &[u8],
    mut request_reboot: impl FnMut(),
    out: &mut [u8],
) -> Option<usize> {
    if !payload.is_empty() {
        return None;
    }
    request_reboot();
    proto::encode_reply(Cmd::Reboot, b"OK", out)
}

pub(crate) fn encode_tooth_composite_log_from_records<const N: usize>(
    records: &[CommonObservabilityRecord; N],
    drained: usize,
    overflow_count: u16,
    out: &mut [u8],
) -> Option<usize> {
    let mut entries = [ToothCompositeLogEntry::default(); N];
    let mut count = 0usize;
    for record in records.iter().take(drained) {
        let sample = record.sample;
        let snapshot = sample.snapshot;
        let entry = match record.kind {
            CommonObservabilityRecordKind::TriggerEdge => Some(ToothCompositeLogEntry {
                kind: ToothCompositeLogKind::TriggerEdge,
                flags: u8::from(snapshot.trigger_edge.seen)
                    | (u8::from(snapshot.trigger_edge.synced) << 1),
                rpm: snapshot.trigger_edge.rpm.get(),
                angle_x10: snapshot.trigger_edge.angle_x10.get(),
                at_us: snapshot.trigger_edge.at_us.get(),
            }),
            CommonObservabilityRecordKind::CamEdge => Some(ToothCompositeLogEntry {
                kind: ToothCompositeLogKind::CamEdge,
                flags: u8::from(snapshot.cam_edge.seen)
                    | (u8::from(snapshot.cam_edge.cam_seen) << 1),
                rpm: snapshot.engine.rpm.get(),
                angle_x10: snapshot.engine.angle_x10.get(),
                at_us: snapshot.cam_edge.at_us.get(),
            }),
            _ => None,
        };
        if let Some(entry) = entry {
            if count == entries.len() {
                break;
            }
            entries[count] = entry;
            count += 1;
        }
    }
    encode_tooth_composite_log_reply(&entries[..count], overflow_count, out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::Cell;
    use ecu_board_api::{
        CommonCamEdgeTelemetry, CommonEngineTelemetry, CommonTriggerEdgeTelemetry,
    };
    use ecu_domain::{Degrees10, Micros, Rpm};
    use ecu_target_common::adapter::{CommonObservabilitySample, CommonObservabilitySnapshot};

    #[test]
    fn encode_tooth_stats_reply_packs_live_rpm_and_sync_state() {
        let mut out = [0u8; 32];
        let len = encode_tooth_stats_reply(3_250, true, &mut out).expect("encoded reply");
        let (cmd, payload) = proto::decode_request(&out[..len]).expect("decoded reply");

        assert_eq!(cmd, Cmd::ToothStats);
        assert_eq!(payload, &[0xb2, 0x0c, 1]);
    }

    #[test]
    fn encode_tooth_composite_log_from_records_filters_and_packs_trigger_and_cam_edges() {
        let trigger_record = CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::TriggerEdge,
            sample: CommonObservabilitySample {
                snapshot: CommonObservabilitySnapshot {
                    trigger_edge: CommonTriggerEdgeTelemetry {
                        seen: true,
                        at_us: Micros::new(123),
                        rpm: Rpm::new(3_210),
                        angle_x10: Degrees10::new(450),
                        authority: Default::default(),
                        synced: true,
                    },
                    ..Default::default()
                },
                ..Default::default()
            },
        };
        let cam_record = CommonObservabilityRecord {
            kind: CommonObservabilityRecordKind::CamEdge,
            sample: CommonObservabilitySample {
                snapshot: CommonObservabilitySnapshot {
                    cam_edge: CommonCamEdgeTelemetry {
                        seen: true,
                        at_us: Micros::new(130),
                        cam_seen: true,
                    },
                    engine: CommonEngineTelemetry {
                        rpm: Rpm::new(3_210),
                        angle_x10: Degrees10::new(480),
                        ..Default::default()
                    },
                    ..Default::default()
                },
                ..Default::default()
            },
        };
        let records = [trigger_record, cam_record];
        let mut out = [0u8; 128];
        let len =
            encode_tooth_composite_log_from_records(&records, 2, 2, &mut out).expect("encoded log");
        let (cmd, payload) = proto::decode_request(&out[..len]).expect("decoded log");

        assert_eq!(cmd, Cmd::ToothCompositeLog);
        assert_eq!(
            payload[0],
            ecu_ts::server::TOOTH_COMPOSITE_LOG_REPLY_VERSION
        );
        assert_eq!(payload[1], 2);
        assert_eq!(u16::from_le_bytes([payload[2], payload[3]]), 2);
        assert_eq!(payload[4], ToothCompositeLogKind::TriggerEdge as u8);
        assert_eq!(payload[5], 0x03);
        assert_eq!(u16::from_le_bytes([payload[6], payload[7]]), 3_210);
        assert_eq!(i16::from_le_bytes([payload[8], payload[9]]), 450);
        assert_eq!(
            u32::from_le_bytes([payload[10], payload[11], payload[12], payload[13]]),
            123
        );
        assert_eq!(payload[14], ToothCompositeLogKind::CamEdge as u8);
        assert_eq!(payload[15], 0x03);
        assert_eq!(u16::from_le_bytes([payload[16], payload[17]]), 3_210);
        assert_eq!(i16::from_le_bytes([payload[18], payload[19]]), 480);
        assert_eq!(
            u32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]),
            130
        );
    }

    #[test]
    fn handle_version_info_cmd_packs_runtime_target_and_signature() {
        let mut out = [0u8; 64];
        let len = handle_version_info_cmd(&[], b"IPW-ECU V0.1", 0x1234_5678, 0x2040, &mut out)
            .expect("version info reply");
        let (cmd, payload) = proto::decode_request(&out[..len]).expect("decoded version info");

        assert_eq!(cmd, Cmd::VersionInfo);
        assert_eq!(payload[0], ecu_ts::server::VERSION_INFO_REPLY_VERSION);
        assert_eq!(
            u32::from_le_bytes([payload[1], payload[2], payload[3], payload[4]]),
            0x1234_5678
        );
        assert_eq!(u16::from_le_bytes([payload[5], payload[6]]), 0x2040);
        assert_eq!(payload[7] as usize, b"IPW-ECU V0.1".len());
        assert_eq!(&payload[8..8 + b"IPW-ECU V0.1".len()], b"IPW-ECU V0.1");
    }

    #[test]
    fn handle_compatibility_info_cmd_packs_report() {
        let mut out = [0u8; 64];
        let len = handle_compatibility_info_cmd(
            &[],
            CompatibilityInfoReport {
                status: ecu_ts::server::CompatibilityStatusCode::Compatible,
                migration: ecu_ts::server::CompatibilityMigrationCode::None,
                expected_schema_version: 1,
                actual_schema_version: 1,
                expected_runtime_build_id: 0x1234_5678,
                actual_runtime_build_id: 0x1234_5678,
                expected_hardware_target_id: 0x2040,
                actual_hardware_target_id: 0x2040,
            },
            &mut out,
        )
        .expect("compatibility reply");
        let (cmd, payload) = proto::decode_request(&out[..len]).expect("decoded compatibility");

        assert_eq!(cmd, Cmd::CompatibilityInfo);
        assert_eq!(payload[0], ecu_ts::server::COMPATIBILITY_INFO_REPLY_VERSION);
        assert_eq!(
            payload[1],
            ecu_ts::server::CompatibilityStatusCode::Compatible as u8
        );
        assert_eq!(
            payload[2],
            ecu_ts::server::CompatibilityMigrationCode::None as u8
        );
    }

    #[test]
    fn handle_reboot_cmd_marks_reboot_and_replies_ok() {
        let requested = Cell::new(false);
        let mut out = [0u8; 32];
        let len = handle_reboot_cmd(&[], || requested.set(true), &mut out).expect("reboot reply");
        let (cmd, payload) = proto::decode_request(&out[..len]).expect("decoded reboot");

        assert!(requested.get());
        assert_eq!(cmd, Cmd::Reboot);
        assert_eq!(payload, b"OK");
    }
}
