//! Lambda (Air-Fuel Ratio) Closed-Loop Controller
//!
//! Implements a PI controller for closed-loop fuel control based on O2 sensor feedback.
//! Supports both narrowband (rich/lean) and wideband (exact AFR) sensors.
//!
//! ## Long-Term Fuel Trim (LTFT)
//!
//! LTFT learns from STFT corrections over time to improve base calibration.
//! Uses a 4x4 grid (RPM x Load) for coarse learning with slow update rates.

mod config;
mod controller;
mod ltft;
mod state;

pub use config::{ltft_constants, DisableReason, LambdaConfig, LtftConfig, O2SensorType};
pub use ltft::{LtftCell, LtftManager, LtftState, LtftTable};
pub use state::LambdaState;

#[cfg(test)]
mod tests;
