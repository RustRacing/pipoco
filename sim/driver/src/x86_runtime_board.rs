//! x86 runtime board slice built on the new board capability traits.
//!
//! This module keeps the deterministic host-side path separate from the older
//! legacy x86 board glue. It exercises `ecu_runtime::EngineRuntime`
//! through the new `ecu_board_api` trait surface and records output, aux, and
//! telemetry activity in fixed-capacity buffers.

mod board;
mod bridge;
mod io;
mod types;

pub use board::{run_x86_runtime_tick, X86RuntimeBoard};
pub use bridge::bridge_output_transitions_to_core_frame;
pub use types::{
    X86RuntimeBoardDiagnostics, X86RuntimeBoardError, X86RuntimePlantBridgeFrame,
    X86RuntimeTickResult,
};

pub(super) const X86_TRIGGER_EDGE_CAP: usize = 8;
pub(super) const X86_OUTPUT_TRANSITION_CAP: usize = 128;
pub(super) const X86_AUX_COMMAND_CAP: usize = 16;
pub(super) const X86_CAL_PAGE_SIZE: usize = 64;
pub(super) const X86_CAL_PAGE_COUNT: usize = 4;
pub(super) const X86_OUTPUT_CHANNEL_CAP: usize = 16;
pub(super) const X86_ACTIVE_OUTPUT_CAP: usize = X86_OUTPUT_CHANNEL_CAP * 2;
pub(super) const X86_PLANT_BRIDGE_MIN_INJECTION_PULSE_US: u32 = 1;
pub(super) const X86_PLANT_BRIDGE_MIN_SPARK_DWELL_US: u32 = 1;
pub(super) const X86_PLANT_BRIDGE_INJECTOR_FLOW_UG_PER_US: u32 = 5;
pub(super) const X86_PLANT_BRIDGE_COIL_ENERGY_X1000: u16 = 1_000;
