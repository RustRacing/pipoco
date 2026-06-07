//! Board-independent simulator loop support.
//!
//! The implementation is split across sibling modules to keep each file focused
//! while preserving the public API path used by callers.

mod buffers;
mod runner;
mod types;

pub use buffers::{FixedOutputQueue, FixedTraceBuffer, FixedTriggerEdgeBuffer};
pub use runner::{SimBoardLoop, SimBoardLoopConfig, SimBoardLoopError, SimBoardLoopReport};
pub use types::{
    SimBoard, SimBoardTraceKind, SimBoardTraceRecord, SimBufferOverflow, SimControlMode,
    SimDriverInput, SimEdgePolarity, SimEnvironment, SimSensorFrame, SimTriggerEdge,
    SimTriggerLine, TorqueNmX100,
};

#[cfg(test)]
mod tests;
