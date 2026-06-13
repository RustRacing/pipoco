use super::calibration::DiagnosticCode;
use super::collections::CylinderArrayU16;
use super::events::EventBatch;
use super::state::LogicalState;
use super::units::{AfrX100, PulseWidthUs, SignedDegrees10, VePctX100};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ObservableOutput {
    pub ve_pct_x100: VePctX100,
    pub target_afr_x100: AfrX100,
    pub pw_base_us: PulseWidthUs,
    pub pw_air_us: PulseWidthUs,
    pub pw_corr_us: PulseWidthUs,
    pub lambda_correction_x1000: u16,
    pub idle_duty_x1000: u16,
    pub torque_request_x1000: u16,
    pub torque_allowed_x1000: u16,
    pub torque_actuated_x1000: u16,
    pub cut_reason_code: u8,
    pub fuel_cut: bool,
    pub spark_cut: bool,
    pub advance_deg10_trim: i16,
    pub knock_intensity_x100: u16,
    pub spark_advance_deg10: SignedDegrees10,
    pub dwell_us: PulseWidthUs,
    pub soi_deg10: CylinderArrayU16,
    pub eoi_deg10: CylinderArrayU16,
    pub spark_deg10: CylinderArrayU16,
    pub dwell_start_deg10: CylinderArrayU16,
    pub events: EventBatch,
    pub diagnostic: DiagnosticCode,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct StepResult {
    pub next_state: LogicalState,
    pub output: ObservableOutput,
}
