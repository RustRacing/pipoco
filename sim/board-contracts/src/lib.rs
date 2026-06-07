#![forbid(unsafe_code)]

//! Integration-test crate for board conformance checks.
//!
//! This keeps concrete board dependencies out of `ecu-core` while preserving
//! cross-board contract coverage.

/// Check that a no-watchdog recipe carries an explicit justification.
///
/// Returns `Ok(())` if the recipe either requires a watchdog or supplies a
/// non-empty justification string. Returns `Err` with a descriptive message
/// otherwise.
pub fn check_watchdog_policy(
    recipe_name: &str,
    watchdog_required: bool,
    justification: &str,
) -> Result<(), String> {
    if !watchdog_required && justification.is_empty() {
        return Err(format!(
            "recipe '{recipe_name}' has watchdog_required=false but no watchdog_absent_justification; \
             add a justification per ADR-0004"
        ));
    }
    Ok(())
}
