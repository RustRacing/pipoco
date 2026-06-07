use crate::types::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SparkCommand {
    pub cylinder: CylinderIndex,
    pub spark_angle_deg10: CrankDeg10,
    pub dwell_us: Micros,
    pub coil_energy_x1000: u16,
}

impl SparkCommand {
    pub const fn empty() -> Self {
        Self {
            cylinder: CylinderIndex(0),
            spark_angle_deg10: CrankDeg10(0),
            dwell_us: Micros(0),
            coil_energy_x1000: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AcceptedSpark {
    pub command: SparkCommand,
    pub dwell_quality_x1000: u16,
    pub phase_quality_x1000: u16,
}

impl AcceptedSpark {
    pub const fn empty() -> Self {
        Self {
            command: SparkCommand::empty(),
            dwell_quality_x1000: 0,
            phase_quality_x1000: 0,
        }
    }
}

pub fn dwell_quality(dwell: Micros, min_dwell: Micros) -> u16 {
    if dwell.0 < min_dwell.0 {
        return ((dwell.0 as u64 * 1000) / min_dwell.0.max(1) as u64) as u16;
    }
    1000
}

pub fn spark_phase_quality(angle: CrankDeg10, mbt: Degrees10, max_advance: Degrees10) -> u16 {
    let signed = angle.0 as i32;
    if signed > max_advance.0 as i32 {
        return 0;
    }
    let distance = (signed - mbt.0 as i32).unsigned_abs();
    1000u32.saturating_sub(distance * 4).max(250) as u16
}
