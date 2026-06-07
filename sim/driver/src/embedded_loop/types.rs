//! Loop-facing types and traits used across simulator board runners.

use ecu_domain::Rpm;
/// Torque in newton-meters x100.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct TorqueNmX100(pub i32);

impl TorqueNmX100 {
    pub const fn new(value: i32) -> Self {
        Self(value)
    }

    pub const fn get(self) -> i32 {
        self.0
    }
}

/// Control mode requested by the driver.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum SimControlMode {
    #[default]
    OpenLoopRpm,
    ClosedLoopEngine,
}

/// Driver inputs visible to the board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimDriverInput {
    pub throttle_x1000: u16,
    pub requested_rpm: Rpm,
    pub load_torque_nm_x100: TorqueNmX100,
    pub mode: SimControlMode,
}

impl SimDriverInput {
    pub const fn idle() -> Self {
        Self {
            throttle_x1000: 0,
            requested_rpm: Rpm::new(0),
            load_torque_nm_x100: TorqueNmX100::new(0),
            mode: SimControlMode::OpenLoopRpm,
        }
    }
}

/// Environment inputs visible to the board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimEnvironment {
    pub ambient_pressure_pa: i32,
    pub ambient_temp_k_x10: u16,
    pub battery_mv: u16,
}

impl SimEnvironment {
    pub const fn standard() -> Self {
        Self {
            ambient_pressure_pa: 101_325,
            ambient_temp_k_x10: 2931,
            battery_mv: 13_500,
        }
    }
}

/// Sensor frame visible to the board loop.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct SimSensorFrame {
    pub timestamp_us: u32,
    pub rpm: Rpm,
    pub crank_angle_deg10: u16,
    pub map_kpa10: u16,
    pub tps_x1000: u16,
    pub clt_c10: i16,
    pub iat_c10: i16,
    pub lambda_x1000: u16,
    pub battery_mv: u16,
    pub knock_intensity_x100: u16,
}

impl SimSensorFrame {
    pub const fn empty() -> Self {
        Self {
            timestamp_us: 0,
            rpm: Rpm::new(0),
            crank_angle_deg10: 0,
            map_kpa10: 0,
            tps_x1000: 0,
            clt_c10: 0,
            iat_c10: 0,
            lambda_x1000: 1_000,
            battery_mv: 0,
            knock_intensity_x100: 0,
        }
    }
}

/// Trigger line identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimTriggerLine {
    Crank,
    Cam,
}

/// Trigger edge polarity.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimEdgePolarity {
    Rising,
    Falling,
}

/// A deterministic trigger edge record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SimTriggerEdge {
    pub timestamp_us: u32,
    pub line: SimTriggerLine,
    pub polarity: SimEdgePolarity,
    pub angle_deg10: u16,
}

impl SimTriggerEdge {
    pub const fn new(
        timestamp_us: u32,
        line: SimTriggerLine,
        polarity: SimEdgePolarity,
        angle_deg10: u16,
    ) -> Self {
        Self {
            timestamp_us,
            line,
            polarity,
            angle_deg10,
        }
    }
}

impl Default for SimTriggerEdge {
    fn default() -> Self {
        Self::new(0, SimTriggerLine::Crank, SimEdgePolarity::Rising, 0)
    }
}

/// Generic board trace category.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimBoardTraceKind {
    Tick,
    DriverInput,
    Environment,
    TriggerEdge,
    SensorFrame,
    OutputTransition,
    PlantOutput,
    Note,
}

/// Generic board trace record.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct SimBoardTraceRecord {
    pub timestamp_us: u32,
    pub kind: SimBoardTraceKind,
    pub channel: u8,
    pub value: i32,
    pub detail: i32,
}

impl SimBoardTraceRecord {
    pub const fn new(
        timestamp_us: u32,
        kind: SimBoardTraceKind,
        channel: u8,
        value: i32,
        detail: i32,
    ) -> Self {
        Self {
            timestamp_us,
            kind,
            channel,
            value,
            detail,
        }
    }
}

impl Default for SimBoardTraceRecord {
    fn default() -> Self {
        Self::new(0, SimBoardTraceKind::Tick, 0, 0, 0)
    }
}

/// Fixed-capacity overflow marker.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SimBufferOverflow {
    pub capacity: usize,
}

impl SimBufferOverflow {
    pub const fn new(capacity: usize) -> Self {
        Self { capacity }
    }
}

/// Board-level simulator interface.
pub trait SimBoard {
    type Error;
    type OutputBuffer;
    type PlantOutputs;

    fn now_micros(&self) -> u32;

    fn read_driver_input(&mut self) -> Result<SimDriverInput, Self::Error>;
    fn read_environment(&mut self) -> Result<SimEnvironment, Self::Error>;

    fn feed_trigger_edges(&mut self, edges: &[SimTriggerEdge]) -> Result<(), Self::Error>;

    fn feed_sensor_frame(&mut self, sensors: SimSensorFrame) -> Result<(), Self::Error>;

    fn collect_ecu_outputs(&mut self, out: &mut Self::OutputBuffer) -> Result<(), Self::Error>;

    fn publish_plant_outputs(&mut self, outputs: Self::PlantOutputs) -> Result<(), Self::Error>;

    fn write_trace(&mut self, record: SimBoardTraceRecord) -> Result<(), Self::Error>;
}
