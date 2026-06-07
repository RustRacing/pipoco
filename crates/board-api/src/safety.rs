//! Safety-gate contracts and status computation.

use ecu_domain::{FaultCode, FaultSeverity, Micros};

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SafetyGateInput {
    pub now_us: Micros,
    pub kill_n: bool,
    pub power_good: bool,
    pub watchdog_ok: bool,
    pub timing_backend_alive: bool,
    pub backend_alive: bool,
    pub sync_authority_ok: bool,
    pub driver_faults: SafetyDriverFaultMask,
    pub requested_permit_mask: SafetyPermitMask,
}

impl SafetyGateInput {
    #[allow(clippy::too_many_arguments)]
    pub const fn new(
        now_us: Micros,
        kill_n: bool,
        power_good: bool,
        watchdog_ok: bool,
        timing_backend_alive: bool,
        backend_alive: bool,
        sync_authority_ok: bool,
        driver_faults: SafetyDriverFaultMask,
        requested_permit_mask: SafetyPermitMask,
    ) -> Self {
        Self {
            now_us,
            kill_n,
            power_good,
            watchdog_ok,
            timing_backend_alive,
            backend_alive,
            sync_authority_ok,
            driver_faults,
            requested_permit_mask,
        }
    }
}

pub type SafetyDriverFaultMask = u32;

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct SafetyPermitMask(u32);

impl SafetyPermitMask {
    pub const NONE: Self = Self(0);
    pub const ALL: Self = Self(u32::MAX);

    pub const fn new(bits: u32) -> Self {
        Self(bits)
    }

    pub const fn bits(self) -> u32 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SafetyGatePermit {
    #[default]
    Denied,
    Allowed,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub enum SafetyGateReason {
    #[default]
    None,
    SyncNotAuthorized,
    KillAsserted,
    PowerNotGood,
    WatchdogTimeout,
    TimingBackendNotAlive,
    BackendNotAlive,
    DriverFault,
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct SafetyGateStatus {
    pub permit: SafetyGatePermit,
    pub reason: SafetyGateReason,
    pub fault_code: FaultCode,
    pub fault_severity: FaultSeverity,
    pub driver_faults: SafetyDriverFaultMask,
    pub last_checked_us: Micros,
    pub effective_permit_mask: SafetyPermitMask,
}

impl SafetyGateStatus {
    pub const fn denied(
        reason: SafetyGateReason,
        fault_code: FaultCode,
        fault_severity: FaultSeverity,
        driver_faults: SafetyDriverFaultMask,
        last_checked_us: Micros,
    ) -> Self {
        Self {
            permit: SafetyGatePermit::Denied,
            reason,
            fault_code,
            fault_severity,
            driver_faults,
            last_checked_us,
            effective_permit_mask: SafetyPermitMask::NONE,
        }
    }
}
