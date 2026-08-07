//! Decel Fuel Cut (DFCO) detection

pub use ecu_calibration::configs::DfcoConfig;

/// Canonical decel-fuel-cut state machine, owned by `ecu-control`.
///
/// Compatibility shell re-exports the control `DecelFuelCutState` (same
/// enter-delay / resume-hysteresis logic) as `DfcoState`; see review 001.
pub use ecu_control::DecelFuelCutState as DfcoState;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dfco_engage_and_resume() {
        let cfg = DfcoConfig::DEFAULT;
        let mut st = DfcoState::new();
        // At time 0 conditions allow DFCO but delay applies
        assert!(!st.update(0, 2000, 0, 20, &cfg));
        // After delay
        assert!(st.update(cfg.delay_ms * 1000 + 1, 2000, 0, 20, &cfg));
        // Conditions break (TPS blip)
        assert!(st.update(cfg.delay_ms * 1000 + 50_000, 2000, 10, 20, &cfg)); // not yet resumed due to hysteresis
        assert!(!st.update(
            cfg.delay_ms * 1000 + cfg.resume_hyst_ms * 1000 + 2,
            2000,
            10,
            20,
            &cfg
        ));
    }
}
