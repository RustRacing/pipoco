//! Runtime-reducer corpus covers only fields accessible via public EngineRuntime API.
//! This file only checks corpus inventory. It is not semantic conformance
//! evidence for runtime-owned fields.
use std::collections::BTreeSet;

#[path = "../../tests/formal/fm0016_fixture_matrix.rs"]
#[allow(dead_code)]
mod fm0016_fixture_matrix;

use fm0016_fixture_matrix::{fixture_cases, required_fixture_names};

#[test]
fn fm0016_coverage_fixture_matrix_matches_required_list() {
    let observed: BTreeSet<&'static str> =
        fixture_cases().iter().map(|case| case.fixture).collect();
    let required: BTreeSet<&'static str> = required_fixture_names().into_iter().collect();
    assert_eq!(observed, required);
}
