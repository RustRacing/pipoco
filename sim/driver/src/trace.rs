//! Fixed-size structured driver trace.

/// Maximum number of trace records.
pub const DRIVER_TRACE_CAP: usize = 1024;

/// Kind of trace record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum DriverTraceKind {
    Init,
    Sensor,
    CrankEdge,
    CamEdge,
    Step,
    Output,
    PlantAdvance,
    Snapshot,
}

/// Structured diagnostic payload observed by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriverDiagnostics {
    /// Encoded diagnostic fault code.
    pub fault_code: u8,
    /// Encoded diagnostic fault severity.
    pub fault_severity: u8,
}

/// Structured decision payload observed by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriverDecision {
    /// Encoded diagnostic cancel reason.
    pub cancel_reason: u8,
    /// Encoded diagnostic control mode.
    pub control_mode: u8,
    /// Fuel cut is currently active (0 = no cut, 1 = cut active).
    pub fuel_cut: u8,
    /// Spark cut is currently active (0 = no cut, 1 = cut active).
    pub spark_cut: u8,
}

/// Structured freeze-frame snapshot observed by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriverFreezeFrame {
    /// Timestamp in microseconds.
    pub now_us: u32,
    /// Engine speed in RPM.
    pub rpm: u16,
    /// Current tooth counter.
    pub tooth: u8,
    /// Crank angle in degrees x10.
    pub angle_x10: i16,
    /// Manifold pressure x10.
    pub map_kpa10: u16,
    /// Throttle position x100.
    pub tps_x100: u16,
    /// Coolant temperature x10.
    pub clt_c10: i16,
    /// Intake air temperature x10.
    pub iat_c10: i16,
    /// Battery voltage in millivolts.
    pub vbatt_mv: u16,
    /// Barometric pressure x10.
    pub baro_kpa10: u16,
    /// Vehicle speed in km/h x10.
    pub vehicle_speed_kph10: u16,
    /// Whether vehicle speed is valid.
    pub vehicle_speed_valid: u8,
    /// Optional mass air flow reading in source-native units x100.
    pub maf_x100: u16,
    /// Whether MAF/HFM is valid.
    pub maf_valid: u8,
    /// Optional normalized knock level x100.
    pub knock_x100: u16,
    /// Whether knock level is valid.
    pub knock_valid: u8,
    /// Optional measured cam phase in degrees x10.
    pub cam_phase_deg10: i16,
    /// Whether measured cam phase is valid.
    pub cam_phase_valid: u8,
    /// Lambda x100.
    pub lambda_x100: u16,
    /// Whether lambda is valid.
    pub lambda_valid: u8,
}

/// Structured observability payload observed by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct DriverObservability {
    pub diagnostics: DriverDiagnostics,
    pub decision: DriverDecision,
    pub freeze_frame: DriverFreezeFrame,
}

/// A single trace record from the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DriverTraceRecord {
    pub at_us: u32,
    pub kind: DriverTraceKind,
    pub status: i32,
    pub rpm: u16,
    pub map_kpa10: u16,
    pub angle_x10: i16,
    pub output_kind: i32,
    pub channel: u8,
    pub high: u8,
    pub combustion_events: u32,
    /// Whether the ECU is synced at this trace point.
    pub synced: u8,
    /// Current tooth counter at this trace point.
    pub tooth: u8,
    /// Encoded diagnostic fault code.
    pub diagnostic_code: u8,
    /// Encoded diagnostic fault severity.
    pub fault_severity: u8,
    /// Encoded diagnostic cancel reason.
    pub cancel_reason: u8,
    /// Encoded diagnostic control mode.
    pub control_mode: u8,
    /// Structured diagnostic/decision observability for this trace point.
    pub observability: DriverObservability,
}

/// Fixed-size trace buffer with explicit overflow tracking.
pub struct FixedDriverTrace<const N: usize> {
    records: [Option<DriverTraceRecord>; N],
    len: usize,
    overflow_count: u32,
}

impl<const N: usize> FixedDriverTrace<N> {
    pub const fn new() -> Self {
        Self {
            records: [None; N],
            len: 0,
            overflow_count: 0,
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn overflow_count(&self) -> u32 {
        self.overflow_count
    }

    pub fn get(&self, index: usize) -> Option<DriverTraceRecord> {
        if index < self.len {
            self.records[index]
        } else {
            None
        }
    }

    pub fn push(&mut self, record: DriverTraceRecord) -> Result<(), crate::DriverError> {
        if self.len < N {
            self.records[self.len] = Some(record);
            self.len += 1;
            Ok(())
        } else {
            self.overflow_count = self.overflow_count.saturating_add(1);
            Err(crate::DriverError::TraceOverflow)
        }
    }
}

impl<const N: usize> Default for FixedDriverTrace<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Eq for FixedDriverTrace<N> {}

impl<const N: usize> core::cmp::PartialEq for FixedDriverTrace<N> {
    fn eq(&self, other: &Self) -> bool {
        self.len == other.len
            && self.overflow_count == other.overflow_count
            && self.records[..self.len] == other.records[..other.len]
    }
}

impl<const N: usize> core::fmt::Debug for FixedDriverTrace<N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FixedDriverTrace")
            .field("len", &self.len)
            .field("overflow_count", &self.overflow_count)
            .finish()
    }
}

impl<const N: usize> Clone for FixedDriverTrace<N> {
    fn clone(&self) -> Self {
        let mut new = Self::new();
        new.len = self.len;
        new.overflow_count = self.overflow_count;
        let mut i = 0;
        while i < self.len {
            new.records[i] = self.records[i];
            i += 1;
        }
        new
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trace_overflow_is_explicit() {
        let mut trace: FixedDriverTrace<1> = FixedDriverTrace::new();
        assert_eq!(trace.len(), 0);
        assert_eq!(trace.overflow_count(), 0);

        let record = DriverTraceRecord {
            at_us: 1000,
            kind: DriverTraceKind::Init,
            status: 0,
            rpm: 0,
            map_kpa10: 0,
            angle_x10: 0,
            output_kind: 0,
            channel: 0,
            high: 0,
            combustion_events: 0,
            synced: 0,
            tooth: 0,
            diagnostic_code: 0,
            fault_severity: 0,
            cancel_reason: 0,
            control_mode: 0,
            observability: DriverObservability::default(),
        };

        // First push succeeds
        assert!(trace.push(record).is_ok());
        assert_eq!(trace.len(), 1);
        assert_eq!(trace.overflow_count(), 0);

        // Second push returns overflow
        let result = trace.push(record);
        assert_eq!(result, Err(crate::DriverError::TraceOverflow));
        assert_eq!(trace.overflow_count(), 1);
    }
}
