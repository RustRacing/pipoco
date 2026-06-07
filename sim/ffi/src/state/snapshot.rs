use crate::encoding::snapshot_from_runtime;
#[cfg(any(test, feature = "test-support"))]
use crate::encoding::{encode_cancel_reason, encode_fault_code, encode_fault_severity};
use crate::EcuSimSnapshot;
#[cfg(any(test, feature = "test-support"))]
use ecu_domain::{CancelReason, FaultCode, FaultSeverity};

use super::model::EcuSimHandle;

impl EcuSimHandle {
    pub(crate) fn snapshot(&self) -> EcuSimSnapshot {
        let snapshot = snapshot_from_runtime(
            self.sim.runtime().snapshot(),
            self.now_us,
            self.tooth,
            self.outputs.overflow_count,
        );
        self.apply_test_fault_override(snapshot)
    }

    #[cfg(any(test, feature = "test-support"))]
    pub(crate) fn inject_fault_for_test(
        &mut self,
        fault: FaultCode,
        severity: FaultSeverity,
        cancel_reason: CancelReason,
    ) {
        self.fault_override = Some((fault, severity, cancel_reason));
    }

    #[cfg(any(test, feature = "test-support"))]
    fn apply_test_fault_override(&self, mut snapshot: EcuSimSnapshot) -> EcuSimSnapshot {
        if let Some((fault, severity, cancel_reason)) = self.fault_override {
            snapshot.fault_code = encode_fault_code(fault);
            snapshot.fault_severity = encode_fault_severity(severity);
            snapshot.cancel_reason = encode_cancel_reason(cancel_reason);
        }
        snapshot
    }

    #[cfg(not(any(test, feature = "test-support")))]
    fn apply_test_fault_override(&self, snapshot: EcuSimSnapshot) -> EcuSimSnapshot {
        snapshot
    }
}
