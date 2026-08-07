//! Shared host test fixtures for FM0016 conformance and reducer tests.
//!
//! This is a dev-only crate: `publish = false`, no production consumers.
//! It holds the single copy of the FM0016 fixture matrix (previously reached
//! via `include!` / `#[path]` module tricks) and the semantic calibration
//! builders that used to be copy-pasted across the ecu-runtime and
//! ecu-scheduler conformance tests.

#![allow(dead_code)]

pub mod fixture_matrix;
pub mod semantic;

/// OUTPC divergence budget (review 012 / ADR 0012).
///
/// The shipping TunerStudio wire is `ecu_ts::outpc::Outpc::WIRE_LEN` (48
/// bytes, pinned byte-by-byte by its frozen-layout test). The spec oracle's
/// `OutpcFrame` is a 64-byte *semantic* snapshot, not a wire format. This
/// test pins that the two stay within their agreed budget.
#[cfg(test)]
mod outpc_divergence_budget {
    #[test]
    fn shipping_outpc_wire_fits_spec_page_budget() {
        // WIRE_LEN is an associated const; 48 fits inside the 64-byte budget.
        let wire_len = 48usize;
        let spec_budget = 64usize;
        assert!(wire_len <= spec_budget);
        assert_eq!(wire_len, ecu_ts::outpc::Outpc::WIRE_LEN);
    }
}
