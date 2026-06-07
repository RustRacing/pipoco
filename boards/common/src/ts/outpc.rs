use crate::sensor_sample::board_sensor_snapshot_from_frame;
use ecu_board_api::{BoardSensorSnapshot, BoardSensorValidityFlags};
use ecu_io::SensorFrame;
use ecu_ts::outpc::Outpc;

pub fn fill_outpc_sensor_fields(out: &mut Outpc, snapshot: BoardSensorSnapshot) {
    out.rpm = snapshot.rpm.get();
    out.map_kpa_x10 = snapshot.map_kpa10.get();
    out.tps_percent = (snapshot.tps_x100 / 100).min(100) as u8;
    out.clt_c = snapshot.clt_c10 / 10;
    out.iat_c = snapshot.iat_c10 / 10;
    out.vbatt_mv = snapshot.vbatt_mv;
    out.baro_kpa = snapshot.baro_kpa10.get() / 10;
    out.vehicle_speed_kph_x10 = snapshot.vehicle_speed_kph10.get();
    out.maf_x100 = snapshot.maf_x100.get();
    out.knock_x100 = snapshot.knock_x100.get();
    out.sensor_validity_flags = snapshot.validity.bits();
    out.cam_phase_deg10 = snapshot.cam_phase_deg10.map_or(0, |phase| phase.get());
    out.cam_phase_valid = if snapshot.cam_phase_deg10.is_some() {
        1
    } else {
        0
    };
    if snapshot.validity.contains(BoardSensorValidityFlags::LAMBDA) {
        out.lambda_x100 = snapshot.lambda_x100.get();
    }
}

pub fn fill_outpc_sensor_fields_from_frame(out: &mut Outpc, frame: SensorFrame) {
    fill_outpc_sensor_fields(out, board_sensor_snapshot_from_frame(frame));
}

#[cfg(test)]
mod tests {
    use super::*;
    use ecu_domain::{
        Degrees10, KnockLevelX100, Kpa10, Lambda100, MassAirFlowX100, Micros, Rpm,
        VehicleSpeedKph10,
    };

    fn frame() -> SensorFrame {
        SensorFrame {
            at_us: Micros::new(123),
            rpm: Rpm::new(2_100),
            map_kpa10: Kpa10::new(875),
            maf_x100: MassAirFlowX100::new(3_456),
            maf_valid: true,
            knock_x100: KnockLevelX100::new(789),
            knock_valid: true,
            cam_phase_deg10: None,
            angle_x10: Degrees10::new(90),
            tps_x100: 12_345,
            clt_c10: 855,
            iat_c10: 302,
            vbatt_mv: 12_600,
            baro_kpa10: Kpa10::new(1_013),
            vehicle_speed_kph10: VehicleSpeedKph10::new(543),
            vehicle_speed_valid: true,
            lambda_valid: true,
            lambda_x100: Lambda100::new(99),
        }
    }

    fn snapshot() -> BoardSensorSnapshot {
        BoardSensorSnapshot {
            rpm: Rpm::new(2_100),
            map_kpa10: Kpa10::new(875),
            tps_x100: 12_345,
            clt_c10: 855,
            iat_c10: 302,
            vbatt_mv: 12_600,
            baro_kpa10: Kpa10::new(1_013),
            maf_x100: MassAirFlowX100::new(3_456),
            knock_x100: KnockLevelX100::new(789),
            vehicle_speed_kph10: VehicleSpeedKph10::new(543),
            cam_phase_deg10: None,
            lambda_x100: Lambda100::new(99),
            validity: BoardSensorValidityFlags::from_channels(true, true, true, true),
        }
    }

    #[test]
    fn fill_outpc_sensor_fields_projects_board_sensor_snapshot_for_ts() {
        let mut out = Outpc::default();

        fill_outpc_sensor_fields(&mut out, snapshot());

        let rpm = out.rpm;
        let map = out.map_kpa_x10;
        let tps = out.tps_percent;
        let clt = out.clt_c;
        let iat = out.iat_c;
        let vbatt = out.vbatt_mv;
        let baro = out.baro_kpa;
        let vss = out.vehicle_speed_kph_x10;
        let validity = out.sensor_validity_flags;
        let maf = out.maf_x100;
        let knock = out.knock_x100;
        let cam_phase = out.cam_phase_deg10;
        let cam_phase_valid = out.cam_phase_valid;
        let lambda = out.lambda_x100;

        assert_eq!(rpm, 2_100);
        assert_eq!(map, 875);
        assert_eq!(tps, 100);
        assert_eq!(clt, 85);
        assert_eq!(iat, 30);
        assert_eq!(vbatt, 12_600);
        assert_eq!(baro, 101);
        assert_eq!(vss, 543);
        assert_eq!(maf, 3_456);
        assert_eq!(knock, 789);
        assert_eq!(
            validity,
            BoardSensorValidityFlags::MAF
                | BoardSensorValidityFlags::KNOCK
                | BoardSensorValidityFlags::VEHICLE_SPEED
                | BoardSensorValidityFlags::LAMBDA
        );
        assert_eq!(cam_phase, 0);
        assert_eq!(cam_phase_valid, 0);
        assert_eq!(lambda, 99);
    }

    #[test]
    fn fill_outpc_sensor_fields_projects_valid_cam_phase() {
        let mut out = Outpc::default();
        let snapshot = BoardSensorSnapshot {
            cam_phase_deg10: Some(ecu_domain::CamPhaseDeg10::new(-123)),
            ..snapshot()
        };

        fill_outpc_sensor_fields(&mut out, snapshot);

        let cam_phase = out.cam_phase_deg10;
        let cam_phase_valid = out.cam_phase_valid;
        assert_eq!(cam_phase, -123);
        assert_eq!(cam_phase_valid, 1);
    }

    #[test]
    fn fill_outpc_sensor_fields_keeps_lambda_when_sensor_invalid() {
        let mut out = Outpc {
            lambda_x100: 100,
            ..Outpc::default()
        };
        let invalid = BoardSensorSnapshot {
            validity: BoardSensorValidityFlags::from_channels(true, true, true, false),
            lambda_x100: Lambda100::new(80),
            ..snapshot()
        };

        fill_outpc_sensor_fields(&mut out, invalid);

        let lambda = out.lambda_x100;
        assert_eq!(lambda, 100);
    }

    #[test]
    fn raw_sensor_frame_adapter_projects_compatibility_path() {
        let mut out = Outpc::default();

        fill_outpc_sensor_fields_from_frame(&mut out, frame());

        let rpm = out.rpm;
        let validity = out.sensor_validity_flags;
        let lambda = out.lambda_x100;
        assert_eq!(rpm, 2_100);
        assert_eq!(
            validity,
            BoardSensorValidityFlags::MAF
                | BoardSensorValidityFlags::KNOCK
                | BoardSensorValidityFlags::VEHICLE_SPEED
                | BoardSensorValidityFlags::LAMBDA
        );
        assert_eq!(lambda, 99);
    }
}
