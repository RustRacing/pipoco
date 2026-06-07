const REQUIRED_LEMMAS: [&str; 50] = [
    "norm7200_in_range",
    "cyc7200_distance_bound",
    "find_segment_in_range",
    "lerp_breakpoint_exactness",
    "lerp_boundedness",
    "bilerp_grid_point_exactness",
    "min2_lower",
    "max2_upper",
    "bilerp_convex_hull_boundedness",
    "bilerp_edge_continuity",
    "constant_table_reproduction",
    "bilinear_surface_reproduction",
    "duration_nonnegative",
    "lemma_pi_integrator_step_i32_bounds",
    "lemma_pid_sum_clamp_i32_bounds",
    "lemma_idle_pi_step_freeze_idempotent",
    "lemma_idle_pi_step_saturation_clamp_idempotent",
    "lemma_lambda_pi_step_freeze_idempotent",
    "lemma_lambda_pi_step_saturation_clamp_idempotent",
    "lemma_clt_from_counts_monotonic_clamped",
    "lemma_iat_from_counts_monotonic_clamped",
    "lemma_map_from_counts_monotonic_clamped",
    "lemma_tps_from_counts_monotonic_clamped",
    "lemma_maf_from_counts_monotonic_clamped",
    "lemma_o2_from_counts_monotonic_clamped",
    "lemma_knock_from_window_monotonic_clamped",
    "lemma_baro_from_counts_monotonic_clamped",
    "lemma_vbat_from_counts_monotonic_clamped",
    "lemma_trigger_sync_state_totality",
    "lemma_trigger_angle_wrap_bounds",
    "lemma_trigger_rpm_estimate_bound",
    "lemma_deadtime_bilerp_u16_bounds",
    "lemma_steinhart_ratio_q24_bounds",
    "lemma_steinhart_beta_temp_c10_q22_bounds",
    "lemma_slew_limit_step_i32_bounds",
    "lemma_angle_add_deg10_wrap_bounds",
    "lemma_angle_delta_deg10_signed_bounds",
    "lemma_debounce_counter_step_us_bounds",
    "fuel_cut_no_injection_events",
    "lemma_compute_pw_corr_spec_clamp_bounds",
    "lemma_arbiter_step_priority_totality",
    "lemma_enrichment_afterstart_warmup_ae_order",
    "spark_cut_no_spark_events",
    "spark_after_dwell_ordering",
    "step_determinism",
    "lemma_persist_decode_encode_roundtrip",
    "lemma_persist_migrate_current_idempotent",
    "lemma_ts_page_meta_totality",
    "lemma_ts_outpc_roundtrip",
    "lemma_ts_dispatch_totality",
];

fn lemma_body_nonempty(source: &str, lemma: &str) {
    let needle = format!("pub proof fn {lemma}");
    let start = source
        .find(&needle)
        .unwrap_or_else(|| panic!("missing Verus lemma: {lemma}"));
    let after = &source[start..];

    let open_rel = after
        .find('{')
        .unwrap_or_else(|| panic!("missing body start for lemma: {lemma}"));
    let body_start = start + open_rel + 1;

    let mut depth = 1usize;
    let mut i = body_start;
    let bytes = source.as_bytes();
    while i < source.len() {
        match bytes[i] {
            b'{' => depth += 1,
            b'}' => {
                depth -= 1;
                if depth == 0 {
                    break;
                }
            }
            _ => {}
        }
        i += 1;
    }

    assert!(depth == 0, "unterminated body for lemma: {lemma}");
    let body = &source[body_start..i];
    let has_token = body.chars().any(|c| c.is_alphanumeric() || c == '_');
    assert!(has_token, "empty lemma body for: {lemma}");
}

#[test]
fn verus_lemma_table_matches_plan_order_and_nonempty_bodies() {
    let source = include_str!("../proofs/verus.rs");
    let mut search_start = 0usize;

    for lemma in REQUIRED_LEMMAS {
        let needle = format!("pub proof fn {lemma}");
        let remainder = &source[search_start..];
        let offset = remainder
            .find(&needle)
            .unwrap_or_else(|| panic!("missing or out-of-order Verus lemma: {lemma}"));
        search_start += offset + needle.len();
        lemma_body_nonempty(source, lemma);
    }
}
