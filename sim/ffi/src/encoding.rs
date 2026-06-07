use ecu_domain::{CancelReason, ControlMode, FaultCode, FaultSeverity, SyncState};
use ecu_runtime::RuntimeSnapshot;

use crate::EcuSimSnapshot;

pub(crate) fn encode_fault_code(fault: FaultCode) -> u8 {
    match fault {
        FaultCode::None => 0,
        FaultCode::SyncLoss => 1,
        FaultCode::SensorOutOfRange => 2,
        FaultCode::CalibrationInvalid => 3,
        FaultCode::SafetyCut => 4,
        FaultCode::ActuatorFault => 5,
    }
}

pub(crate) fn encode_fault_severity(severity: FaultSeverity) -> u8 {
    match severity {
        FaultSeverity::Info => 0,
        FaultSeverity::Warning => 1,
        FaultSeverity::Critical => 2,
    }
}

pub(crate) fn encode_cancel_reason(reason: CancelReason) -> u8 {
    match reason {
        CancelReason::Manual => 0,
        CancelReason::SyncLoss => 1,
        CancelReason::SafetyShutdown => 2,
        CancelReason::Commit => 3,
        CancelReason::Timeout => 4,
    }
}

pub(crate) fn encode_control_mode(mode: ControlMode) -> u8 {
    match mode {
        ControlMode::OpenLoop => 0,
        ControlMode::ClosedLoop => 1,
        ControlMode::LimpHome => 2,
        ControlMode::Shutdown => 3,
    }
}

pub(crate) fn encode_engine_phase(phase: ecu_domain::EnginePhase) -> u8 {
    match phase {
        ecu_domain::EnginePhase::Off => 0,
        ecu_domain::EnginePhase::Cranking => 1,
        ecu_domain::EnginePhase::Running => 2,
        ecu_domain::EnginePhase::Stopping => 3,
    }
}

pub(crate) fn encode_bool(b: bool) -> u8 {
    if b {
        1
    } else {
        0
    }
}

pub(crate) fn snapshot_from_runtime(
    runtime_snapshot: RuntimeSnapshot,
    now_us: u32,
    tooth: u8,
    output_overflow_count: u32,
) -> EcuSimSnapshot {
    EcuSimSnapshot {
        now_us,
        rpm: runtime_snapshot.engine.rpm.get(),
        synced: u8::from(matches!(
            runtime_snapshot.engine.sync,
            SyncState::Locked { .. }
        )),
        tooth,
        angle_x10: runtime_snapshot.engine.angle_x10.get(),
        load_kpa10: runtime_snapshot.engine.load_kpa10.get(),
        output_overflow_count,
        engine_phase: encode_engine_phase(runtime_snapshot.engine.phase),
        fault_code: encode_fault_code(runtime_snapshot.faults.fault),
        fault_severity: encode_fault_severity(runtime_snapshot.faults.severity),
        cancel_reason: encode_cancel_reason(runtime_snapshot.faults.cancel_reason),
        control_mode: encode_control_mode(runtime_snapshot.engine.mode),
        fuel_pulse_width_us: runtime_snapshot.control.fuel_pulse_width.get(),
        ignition_advance_x10: runtime_snapshot.control.ignition_advance.get(),
        dwell_us: runtime_snapshot.control.dwell.get(),
        lambda_target_x100: runtime_snapshot.control.lambda_target.get(),
        torque_limit_x100: runtime_snapshot.control.torque_limit_x100,
        rev_soft_active: encode_bool(runtime_snapshot.rev_soft_active),
        rev_hard_active: encode_bool(runtime_snapshot.rev_hard_active),
        launch_active: encode_bool(runtime_snapshot.launch_active),
        flat_shift_active: encode_bool(runtime_snapshot.flat_shift_active),
        fuel_cut: encode_bool(runtime_snapshot.fuel_cut),
        spark_cut: encode_bool(runtime_snapshot.spark_cut),
    }
}
