#[cfg(feature = "transport-can")]
use crate::compat::EcuState;
#[cfg(feature = "transport-can")]
use crate::diag::DiagClearSummary;
#[cfg(feature = "transport-can")]
use ecu_transport::{
    CanObd2DtcClearInputs, CanObd2DtcClearResponseError, CanObd2DtcClearSurface,
    CanObd2DtcClearVerdict, Message,
};

#[cfg(feature = "transport-can")]
#[derive(Debug, Clone, PartialEq)]
pub struct Obd2DtcClearExecutionSurface {
    pub clear_summary: DiagClearSummary,
    pub transport: CanObd2DtcClearSurface,
}

#[cfg(feature = "transport-can")]
fn dtc_clear_inputs_from_summary(summary: DiagClearSummary) -> CanObd2DtcClearInputs {
    CanObd2DtcClearInputs {
        cleared_dtc_count: summary
            .cleared_active_count
            .saturating_add(summary.cleared_log_entries),
        freeze_frame_cleared: summary.cleared_active_count > 0 || summary.cleared_log_entries > 0,
        readiness_reset: true,
    }
}

#[cfg(feature = "transport-can")]
pub fn compose_obd2_dtc_clear(
    state: &mut EcuState,
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
    transport.verdict = CanObd2DtcClearVerdict {
        cleared_dtc_count: inputs.cleared_dtc_count,
        freeze_frame_cleared: inputs.freeze_frame_cleared,
        readiness_reset: inputs.readiness_reset,
    };
    Ok(Obd2DtcClearExecutionSurface {
        clear_summary,
        transport,
    })
}
