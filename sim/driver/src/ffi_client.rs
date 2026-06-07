//! Safe FFI client wrapper for ecu-sim-ffi.
//!
//! All unsafe calls into ecu-sim-ffi are isolated in this module.

use std::sync::{Mutex, MutexGuard, OnceLock};

static FFI_GLOBAL_LOCK: OnceLock<Mutex<()>> = OnceLock::new();

fn ffi_global_lock() -> MutexGuard<'static, ()> {
    let mutex = FFI_GLOBAL_LOCK.get_or_init(|| Mutex::new(()));
    match mutex.lock() {
        Ok(guard) => guard,
        Err(poisoned) => poisoned.into_inner(),
    }
}

/// FFI client for interacting with the singleton ECU simulation.
///
/// Holding this client holds the process-local driver lock, so a scenario is
/// isolated across the full reset/init/step/snapshot sequence.
pub struct EcuFfiClient {
    _guard: MutexGuard<'static, ()>,
}

impl core::fmt::Debug for EcuFfiClient {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("EcuFfiClient").finish_non_exhaustive()
    }
}

impl EcuFfiClient {
    pub fn acquire() -> Self {
        Self {
            _guard: ffi_global_lock(),
        }
    }

    pub fn reset(&mut self) {
        ecu_sim_ffi::ecu_sim_reset();
    }

    pub fn init(&mut self, cfg: ecu_sim_ffi::EcuSimInitCfg) -> Result<(), crate::DriverError> {
        let status = unsafe {
            // SAFETY: cfg points to initialized stack storage and is valid for this call.
            ecu_sim_ffi::ecu_sim_init(&cfg)
        };
        map_status(status)
    }

    pub fn set_time(&mut self, now_us: u32) -> Result<(), crate::DriverError> {
        let status = ecu_sim_ffi::ecu_sim_set_time(now_us);
        map_status(status)
    }

    pub fn set_sensors(
        &mut self,
        frame: ecu_sim_ffi::EcuSimSensorFrame,
    ) -> Result<(), crate::DriverError> {
        let status = unsafe {
            // SAFETY: frame points to initialized stack storage and is valid for this call.
            ecu_sim_ffi::ecu_sim_set_sensors(&frame)
        };
        map_status(status)
    }

    pub fn on_crank_edge(&mut self, ts_us: u32) -> Result<(), crate::DriverError> {
        let status = ecu_sim_ffi::ecu_sim_on_crank_edge(ts_us);
        map_status(status)
    }

    pub fn on_cam_edge(&mut self, ts_us: u32) -> Result<(), crate::DriverError> {
        let status = ecu_sim_ffi::ecu_sim_on_cam_edge(ts_us);
        map_status(status)
    }

    pub fn step(&mut self, now_us: u32) -> Result<(), crate::DriverError> {
        let status = ecu_sim_ffi::ecu_sim_step(now_us);
        map_status(status)
    }

    pub fn snapshot(&mut self) -> Result<ecu_sim_ffi::EcuSimSnapshot, crate::DriverError> {
        let mut out = ecu_sim_ffi::EcuSimSnapshot::default();
        let status = unsafe {
            // SAFETY: out points to writable storage for one snapshot for this call.
            ecu_sim_ffi::ecu_sim_snapshot(&mut out)
        };
        map_status(status)?;
        Ok(out)
    }

    pub fn dequeue_events<const N: usize>(
        &mut self,
        out: &mut [ecu_sim_ffi::EcuSimOutputEvent; N],
    ) -> Result<usize, crate::DriverError> {
        let copied = unsafe {
            // SAFETY: out has writable storage for N events, matching the cap passed.
            ecu_sim_ffi::ecu_sim_dequeue_events(out.as_mut_ptr(), N)
        };
        if copied > N {
            return Err(crate::DriverError::FfiStatus(
                ecu_sim_ffi::EcuSimStatus::ErrInvalid,
            ));
        }
        Ok(copied)
    }
}

fn map_status(status: ecu_sim_ffi::EcuSimStatus) -> Result<(), crate::DriverError> {
    match status {
        ecu_sim_ffi::EcuSimStatus::Ok => Ok(()),
        ecu_sim_ffi::EcuSimStatus::ErrEventOverflow => Err(crate::DriverError::EventOverflow),
        _ => Err(crate::DriverError::FfiStatus(status)),
    }
}

/// Convert an FFI output event to an OutputTransition.
pub fn output_event_to_transition(
    event: ecu_sim_ffi::EcuSimOutputEvent,
) -> Result<ecu_io::OutputTransition, crate::DriverError> {
    let kind = match event.kind {
        0 => ecu_io::OutputTransitionKind::Injector,
        1 => ecu_io::OutputTransitionKind::Ignition,
        2 => ecu_io::OutputTransitionKind::Idle,
        3 => ecu_io::OutputTransitionKind::Fan,
        other => return Err(crate::DriverError::UnknownOutputKind(other)),
    };
    let level = if event.high == 0 {
        ecu_io::OutputLevel::Low
    } else {
        ecu_io::OutputLevel::High
    };
    Ok(ecu_io::OutputTransition {
        at_us: ecu_domain::Micros::new(event.time_us),
        kind,
        channel: ecu_domain::ChannelId::new(event.channel),
        level,
    })
}

/// Convert a SensorFrame to an FFI sensor frame.
pub fn sensor_frame_to_ffi(frame: ecu_io::SensorFrame) -> ecu_sim_ffi::EcuSimSensorFrame {
    ecu_sim_ffi::EcuSimSensorFrame {
        now_us: frame.at_us.get(),
        map_kpa10: frame.map_kpa10.get(),
        tps_x100: frame.tps_x100,
        clt_c10: frame.clt_c10,
        iat_c10: frame.iat_c10,
        vbatt_mv: frame.vbatt_mv,
        baro_kpa10: frame.baro_kpa10.get(),
        lambda_x100: frame.lambda_x100.get(),
        lambda_valid: if frame.lambda_valid { 1 } else { 0 },
        knock_x100: frame.knock_x100.get(),
        vehicle_speed_kph10: frame.vehicle_speed_kph10.get(),
        maf_x100: frame.maf_x100.get(),
        cam_phase_deg10: frame.cam_phase_deg10.map_or(0, |phase| phase.get()),
        cam_phase_valid: if frame.cam_phase_deg10.is_some() {
            1
        } else {
            0
        },
        validity_flags: ecu_io::SensorValidityFlags::from_frame(frame).bits(),
    }
}

/// Convert the current FFI snapshot into driver-visible diagnostics.
pub fn diagnostics_from_snapshot(
    snapshot: &ecu_sim_ffi::EcuSimSnapshot,
) -> crate::trace::DriverDiagnostics {
    crate::trace::DriverDiagnostics {
        fault_code: snapshot.fault_code,
        fault_severity: snapshot.fault_severity,
    }
}

/// Convert the current FFI snapshot and sensor frame into structured driver observability.
///
/// The freeze frame uses product-owned snapshot fields where the ABI exposes
/// them and supplements the remaining sensor-only values from the sampled frame.
pub fn observability_from_snapshot_and_sensor_frame(
    snapshot: &ecu_sim_ffi::EcuSimSnapshot,
    sensor_frame: ecu_io::SensorFrame,
) -> crate::trace::DriverObservability {
    crate::trace::DriverObservability {
        diagnostics: diagnostics_from_snapshot(snapshot),
        decision: crate::trace::DriverDecision {
            cancel_reason: snapshot.cancel_reason,
            control_mode: snapshot.control_mode,
            fuel_cut: snapshot.fuel_cut,
            spark_cut: snapshot.spark_cut,
        },
        freeze_frame: crate::trace::DriverFreezeFrame {
            now_us: snapshot.now_us,
            rpm: snapshot.rpm,
            tooth: snapshot.tooth,
            angle_x10: snapshot.angle_x10,
            map_kpa10: snapshot.load_kpa10,
            tps_x100: sensor_frame.tps_x100,
            clt_c10: sensor_frame.clt_c10,
            iat_c10: sensor_frame.iat_c10,
            vbatt_mv: sensor_frame.vbatt_mv,
            baro_kpa10: sensor_frame.baro_kpa10.get(),
            vehicle_speed_kph10: sensor_frame.vehicle_speed_kph10.get(),
            vehicle_speed_valid: if sensor_frame.vehicle_speed_valid {
                1
            } else {
                0
            },
            maf_x100: sensor_frame.maf_x100.get(),
            maf_valid: if sensor_frame.maf_valid { 1 } else { 0 },
            knock_x100: sensor_frame.knock_x100.get(),
            knock_valid: if sensor_frame.knock_valid { 1 } else { 0 },
            cam_phase_deg10: sensor_frame.cam_phase_deg10.map_or(0, |phase| phase.get()),
            cam_phase_valid: if sensor_frame.cam_phase_deg10.is_some() {
                1
            } else {
                0
            },
            lambda_x100: sensor_frame.lambda_x100.get(),
            lambda_valid: if sensor_frame.lambda_valid { 1 } else { 0 },
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_step_snapshot_round_trip() {
        let mut client = EcuFfiClient::acquire();
        client.reset();

        let cfg = ecu_sim_ffi::EcuSimInitCfg {
            cylinders: 4,
            has_cam: 1,
            inj_mode: ecu_sim_ffi::EcuSimInjMode::Batch as i32,
            ign_mode: ecu_sim_ffi::EcuSimIgnMode::Wasted as i32,
            firing_len: 4,
            firing_order: [1, 3, 4, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            inj_count: 1,
            inj_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            ign_count: 1,
            ign_channels: [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        };
        assert_eq!(client.init(cfg), Ok(()));

        // Feed two crank edges and one cam edge to sync
        assert_eq!(client.on_crank_edge(1000), Ok(()));
        assert_eq!(client.on_crank_edge(1500), Ok(()));
        assert_eq!(client.on_cam_edge(1500), Ok(()));

        // Step to let the ECU run
        assert_eq!(client.step(2000), Ok(()));

        // Snapshot should show synced ECU
        let snap = client.snapshot();
        assert!(snap.is_ok(), "snapshot should succeed: {snap:?}");
        if let Ok(snap) = snap {
            assert_eq!(snap.synced, 1, "ECU should be synced after crank/cam edges");
        }
    }

    #[test]
    fn output_event_converts_to_transition() {
        // Test injector high
        let event = ecu_sim_ffi::EcuSimOutputEvent {
            time_us: 1000,
            channel: 0,
            kind: 0,
            high: 1,
        };
        let t = output_event_to_transition(event);
        assert!(t.is_ok(), "injector event should convert: {t:?}");
        if let Ok(t) = t {
            assert_eq!(t.kind, ecu_io::OutputTransitionKind::Injector);
            assert_eq!(t.channel.get(), 0);
            assert_eq!(t.level, ecu_io::OutputLevel::High);
            assert_eq!(t.at_us.get(), 1000);
        }

        // Test ignition low
        let event = ecu_sim_ffi::EcuSimOutputEvent {
            time_us: 2000,
            channel: 1,
            kind: 1,
            high: 0,
        };
        let t = output_event_to_transition(event);
        assert!(t.is_ok(), "ignition event should convert: {t:?}");
        if let Ok(t) = t {
            assert_eq!(t.kind, ecu_io::OutputTransitionKind::Ignition);
            assert_eq!(t.level, ecu_io::OutputLevel::Low);
        }

        // Test idle
        let event = ecu_sim_ffi::EcuSimOutputEvent {
            time_us: 3000,
            channel: 2,
            kind: 2,
            high: 1,
        };
        let t = output_event_to_transition(event);
        assert!(t.is_ok(), "idle event should convert: {t:?}");
        if let Ok(t) = t {
            assert_eq!(t.kind, ecu_io::OutputTransitionKind::Idle);
        }

        // Test fan
        let event = ecu_sim_ffi::EcuSimOutputEvent {
            time_us: 4000,
            channel: 3,
            kind: 3,
            high: 1,
        };
        let t = output_event_to_transition(event);
        assert!(t.is_ok(), "fan event should convert: {t:?}");
        if let Ok(t) = t {
            assert_eq!(t.kind, ecu_io::OutputTransitionKind::Fan);
        }
    }

    #[test]
    fn output_event_rejects_unknown_kind() {
        let event = ecu_sim_ffi::EcuSimOutputEvent {
            time_us: 1000,
            channel: 0,
            kind: 99,
            high: 1,
        };
        let result = output_event_to_transition(event);
        assert_eq!(result, Err(crate::DriverError::UnknownOutputKind(99)));
    }

    #[test]
    fn sensor_frame_to_ffi_packs_optional_sensor_validity() {
        let frame = ecu_io::SensorFrame {
            at_us: ecu_domain::Micros::new(2_000),
            rpm: ecu_domain::Rpm::new(2_850),
            map_kpa10: ecu_domain::Kpa10::new(987),
            maf_x100: ecu_domain::MassAirFlowX100::new(2_345),
            maf_valid: true,
            knock_x100: ecu_domain::KnockLevelX100::new(678),
            knock_valid: true,
            cam_phase_deg10: None,
            angle_x10: ecu_domain::Degrees10::new(250),
            tps_x100: 321,
            clt_c10: 760,
            iat_c10: 330,
            vbatt_mv: 12_300,
            baro_kpa10: ecu_domain::Kpa10::new(1_011),
            vehicle_speed_kph10: ecu_domain::VehicleSpeedKph10::new(432),
            vehicle_speed_valid: true,
            lambda_valid: true,
            lambda_x100: ecu_domain::Lambda100::new(98),
        };

        let ffi = sensor_frame_to_ffi(frame);

        assert_eq!(
            ffi.validity_flags,
            ecu_io::SensorValidityFlags::MAF
                | ecu_io::SensorValidityFlags::KNOCK
                | ecu_io::SensorValidityFlags::VEHICLE_SPEED
                | ecu_io::SensorValidityFlags::LAMBDA
        );
        assert_eq!(ffi.lambda_valid, 1);
    }

    #[test]
    fn diagnostics_conversion_covers_non_default_fields() {
        let snapshot = ecu_sim_ffi::EcuSimSnapshot {
            fault_code: 2,
            fault_severity: 1,
            control_mode: 2,
            fuel_cut: 1,
            spark_cut: 0,
            ..Default::default()
        };
        let diagnostics = diagnostics_from_snapshot(&snapshot);
        assert_eq!(diagnostics.fault_code, 2);
        assert_eq!(diagnostics.fault_severity, 1);
    }

    #[test]
    fn observability_conversion_captures_decision_and_freeze_frame() {
        let snapshot = ecu_sim_ffi::EcuSimSnapshot {
            now_us: 1_000,
            rpm: 1_850,
            synced: 1,
            tooth: 12,
            angle_x10: -150,
            load_kpa10: 876,
            fault_code: 5,
            fault_severity: 2,
            cancel_reason: 4,
            control_mode: 3,
            fuel_cut: 1,
            spark_cut: 1,
            ..Default::default()
        };
        let sensor_frame = ecu_io::SensorFrame {
            at_us: ecu_domain::Micros::new(2_000),
            rpm: ecu_domain::Rpm::new(2_850),
            map_kpa10: ecu_domain::Kpa10::new(987),
            maf_x100: ecu_domain::MassAirFlowX100::new(2_345),
            maf_valid: true,
            knock_x100: ecu_domain::KnockLevelX100::new(678),
            knock_valid: true,
            cam_phase_deg10: Some(ecu_domain::CamPhaseDeg10::new(-120)),
            angle_x10: ecu_domain::Degrees10::new(250),
            tps_x100: 321,
            clt_c10: 760,
            iat_c10: 330,
            vbatt_mv: 12_300,
            baro_kpa10: ecu_domain::Kpa10::new(1_011),
            vehicle_speed_kph10: ecu_domain::VehicleSpeedKph10::new(432),
            vehicle_speed_valid: true,
            lambda_valid: true,
            lambda_x100: ecu_domain::Lambda100::new(98),
        };
        let observability = observability_from_snapshot_and_sensor_frame(&snapshot, sensor_frame);
        assert_eq!(observability.diagnostics.fault_code, 5);
        assert_eq!(observability.diagnostics.fault_severity, 2);
        assert_eq!(observability.decision.cancel_reason, 4);
        assert_eq!(observability.decision.control_mode, 3);
        assert_eq!(observability.decision.fuel_cut, 1);
        assert_eq!(observability.decision.spark_cut, 1);
        assert_eq!(observability.freeze_frame.now_us, 1_000);
        assert_eq!(observability.freeze_frame.rpm, 1_850);
        assert_eq!(observability.freeze_frame.tooth, 12);
        assert_eq!(observability.freeze_frame.angle_x10, -150);
        assert_eq!(observability.freeze_frame.map_kpa10, 876);
        assert_eq!(observability.freeze_frame.tps_x100, 321);
        assert_eq!(observability.freeze_frame.clt_c10, 760);
        assert_eq!(observability.freeze_frame.iat_c10, 330);
        assert_eq!(observability.freeze_frame.vbatt_mv, 12_300);
        assert_eq!(observability.freeze_frame.baro_kpa10, 1_011);
        assert_eq!(observability.freeze_frame.vehicle_speed_kph10, 432);
        assert_eq!(observability.freeze_frame.vehicle_speed_valid, 1);
        assert_eq!(observability.freeze_frame.maf_x100, 2_345);
        assert_eq!(observability.freeze_frame.maf_valid, 1);
        assert_eq!(observability.freeze_frame.knock_x100, 678);
        assert_eq!(observability.freeze_frame.knock_valid, 1);
        assert_eq!(observability.freeze_frame.cam_phase_deg10, -120);
        assert_eq!(observability.freeze_frame.cam_phase_valid, 1);
        assert_eq!(observability.freeze_frame.lambda_x100, 98);
        assert_eq!(observability.freeze_frame.lambda_valid, 1);
    }
}
