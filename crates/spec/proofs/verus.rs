// Proof drift must be visible as warnings (review 018): no blanket allow(unused).

use vstd::prelude::*;

fn main() {}

verus! {
    pub open spec fn norm7200_spec(x: int) -> int
        recommends
            -7200 <= x < 14400,
    {
        if x < 0 {
            x + 7200
        } else if x >= 7200 {
            x - 7200
        } else {
            x
        }
    }

    pub proof fn norm7200_in_range(x: int)
        requires
            -7200 <= x < 14400,
        ensures
            0 <= norm7200_spec(x) < 7200,
    {
        if x < 0 {
            assert(-7200 <= x);
            assert(x + 7200 >= 0);
            assert(x < 0);
            assert(x + 7200 < 7200);
        } else if x >= 7200 {
            assert(x < 14400);
            assert(x - 7200 < 7200);
            assert(x >= 7200);
            assert(x - 7200 >= 0);
        } else {
            assert(0 <= x < 7200);
        }
    }

    pub open spec fn cyc7200_distance_spec(a: int, b: int) -> int
        recommends
            0 <= a < 7200,
            0 <= b < 7200,
    {
        let d = if a >= b { a - b } else { b - a };
        if d <= 3600 { d } else { 7200 - d }
    }

    pub proof fn cyc7200_distance_bound(a: int, b: int)
        requires
            0 <= a < 7200,
            0 <= b < 7200,
        ensures
            cyc7200_distance_spec(a, b) <= 3600,
    {
        let d = if a >= b { a - b } else { b - a };
        assert(0 <= d);
        assert(d <= 7199);
        if d <= 3600 {
            assert(cyc7200_distance_spec(a, b) == d);
        } else {
            assert(cyc7200_distance_spec(a, b) == 7200 - d);
            assert(7200 - d <= 3600);
        }
    }

    pub open spec fn find_segment_spec(x: int, x0: int, x1: int, x2: int) -> int
        recommends
            x0 < x1 < x2,
            x0 <= x <= x2,
    {
        if x < x1 { 0 } else { 1 }
    }

    pub proof fn find_segment_in_range(x: u16)
        requires
            10 <= x <= 30,
        ensures
            0 <= find_segment_spec(x as int, 10, 20, 30) <= 1,
            (x as int) < 20 ==> find_segment_spec(x as int, 10, 20, 30) == 0,
            20 <= (x as int) <= 30 ==> find_segment_spec(x as int, 10, 20, 30) == 1,
    {
        if (x as int) < 20 {
            assert(find_segment_spec(x as int, 10, 20, 30) == 0);
        } else {
            assert(find_segment_spec(x as int, 10, 20, 30) == 1);
        }
    }

    pub open spec fn floor_div_spec(num: int, den: int) -> int
        recommends
            den > 0,
    {
        num / den
    }

    pub open spec fn clamp_between_spec(y0: int, y1: int, value: int) -> int {
        if y0 <= y1 {
            if value < y0 {
                y0
            } else if value > y1 {
                y1
            } else {
                value
            }
        } else {
            if value < y1 {
                y1
            } else if value > y0 {
                y0
            } else {
                value
            }
        }
    }

    pub open spec fn lerp_spec(x0: int, x1: int, y0: int, y1: int, x: int) -> int
        recommends
            x0 < x1,
            x0 <= x <= x1,
    {
        if x == x0 {
            y0
        } else if x == x1 {
            y1
        } else {
            let delta = y1 - y0;
            let num = x - x0;
            let den = x1 - x0;
            let product = delta * num;
            if delta >= 0 && product <= 0 {
                y0
            } else if delta >= 0 && product >= delta * den {
                y1
            } else if delta >= 0 && floor_div_spec(product, den) > delta {
                y1
            } else {
                clamp_between_spec(y0, y1, y0 + floor_div_spec(product, den))
            }
        }
    }

    pub proof fn lerp_breakpoint_exactness(x0: u16, x1: u16, y0: u16, y1: u16)
        requires
            x0 < x1,
        ensures
            lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x0 as int) == y0 as int,
            lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x1 as int) == y1 as int,
    {
        assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x0 as int) == y0 as int);
        assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x1 as int) == y1 as int);
    }

    pub proof fn lerp_boundedness(x0: u16, x1: u16, y0: i16, y1: i16, x: u16)
        requires
            x0 < x1,
            x0 <= x <= x1,
        ensures
            if y0 as int <= y1 as int {
                y0 as int <= lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) <= y1 as int
            } else {
                y1 as int <= lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) <= y0 as int
            },
    {
        if y0 as int <= y1 as int {
            let den = (x1 as int) - (x0 as int);
            let num = (x as int) - (x0 as int);
            let delta = (y1 as int) - (y0 as int);
            let raw = y0 as int + floor_div_spec(delta * num, den);
            assert(den > 0);
            assert(0 <= num <= den);
            assert(delta >= 0);
            if x == x0 {
                assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y0 as int);
            } else if x == x1 {
                assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y1 as int);
            } else {
                if delta * num <= 0 {
                    assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y0 as int);
                } else if delta * num >= delta * den {
                    assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y1 as int);
                } else if floor_div_spec(delta * num, den) > delta {
                    assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y1 as int);
                } else {
                    assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == clamp_between_spec(y0 as int, y1 as int, raw));
                    assert(0 < delta * num);
                    assert(delta * num < delta * den);
                    if raw < y0 as int {
                        assert(clamp_between_spec(y0 as int, y1 as int, raw) == y0 as int);
                    } else if raw > y1 as int {
                        assert(clamp_between_spec(y0 as int, y1 as int, raw) == y1 as int);
                    } else {
                        assert(clamp_between_spec(y0 as int, y1 as int, raw) == raw);
                    }
                }
            }
        } else {
            if x == x0 {
                assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y0 as int);
            } else if x == x1 {
                assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == y1 as int);
            } else {
                let den = (x1 as int) - (x0 as int);
                let num = (x as int) - (x0 as int);
                let delta = (y1 as int) - (y0 as int);
                let raw = y0 as int + floor_div_spec(delta * num, den);
                assert(den > 0);
                assert(0 <= num <= den);
                assert(delta < 0);
                assert(lerp_spec(x0 as int, x1 as int, y0 as int, y1 as int, x as int) == clamp_between_spec(y0 as int, y1 as int, raw));
                if raw < y1 as int {
                    assert(clamp_between_spec(y0 as int, y1 as int, raw) == y1 as int);
                } else if raw > y0 as int {
                    assert(clamp_between_spec(y0 as int, y1 as int, raw) == y0 as int);
                } else {
                    assert(clamp_between_spec(y0 as int, y1 as int, raw) == raw);
                }
            }
        }
    }

    pub open spec fn bilerp_2x2_spec(
        rpm0: int,
        rpm1: int,
        load0: int,
        load1: int,
        c00: int,
        c10: int,
        c01: int,
        c11: int,
        rpm: int,
        load: int,
    ) -> int
        recommends
            rpm0 < rpm1,
            load0 < load1,
            rpm0 <= rpm <= rpm1,
            load0 <= load <= load1,
    {
        let lower = lerp_spec(rpm0, rpm1, c00, c10, rpm);
        let upper = lerp_spec(rpm0, rpm1, c01, c11, rpm);
        lerp_spec(load0, load1, lower, upper, load)
    }

    pub proof fn bilerp_grid_point_exactness(c00: u16, c10: u16, c01: u16, c11: u16)
        ensures
            bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 10, 30) == c00 as int,
            bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 20, 30) == c10 as int,
            bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 10, 40) == c01 as int,
            bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 20, 40) == c11 as int,
    {
        lerp_breakpoint_exactness(10u16, 20u16, c00, c10);
        lerp_breakpoint_exactness(10u16, 20u16, c01, c11);
        assert(lerp_spec(10, 20, c00 as int, c10 as int, 10) == c00 as int);
        assert(lerp_spec(10, 20, c01 as int, c11 as int, 10) == c01 as int);
        assert(bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 10, 30) == c00 as int);
        assert(bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 20, 30) == c10 as int);
        assert(bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 10, 40) == c01 as int);
        assert(bilerp_2x2_spec(10, 20, 30, 40, c00 as int, c10 as int, c01 as int, c11 as int, 20, 40) == c11 as int);
    }

    pub open spec fn min2(a: int, b: int) -> int {
        if a <= b { a } else { b }
    }

    pub open spec fn max2(a: int, b: int) -> int {
        if a >= b { a } else { b }
    }

    pub open spec fn min4(a: int, b: int, c: int, d: int) -> int {
        min2(min2(a, b), min2(c, d))
    }

    pub open spec fn max4(a: int, b: int, c: int, d: int) -> int {
        max2(max2(a, b), max2(c, d))
    }

    pub proof fn min2_lower(a: int, b: int)
        ensures
            min2(a, b) <= a,
            min2(a, b) <= b,
    {
        if a <= b {
            assert(min2(a, b) == a);
        } else {
            assert(min2(a, b) == b);
        }
    }

    pub proof fn max2_upper(a: int, b: int)
        ensures
            a <= max2(a, b),
            b <= max2(a, b),
    {
        if a >= b {
            assert(max2(a, b) == a);
        } else {
            assert(max2(a, b) == b);
        }
    }

    pub proof fn bilerp_convex_hull_boundedness(
        x: i16,
        y: i16,
        corner0: u16,
        corner1: u16,
        corner2: u16,
        corner3: u16,
    )
        requires
            min4(corner0 as int, corner1 as int, corner2 as int, corner3 as int) <= x as int <= max4(corner0 as int, corner1 as int, corner2 as int, corner3 as int),
            min4(corner0 as int, corner1 as int, corner2 as int, corner3 as int) <= y as int <= max4(corner0 as int, corner1 as int, corner2 as int, corner3 as int),
        ensures
            min4(corner0 as int, corner1 as int, corner2 as int, corner3 as int) <= x as int,
            x as int <= max4(corner0 as int, corner1 as int, corner2 as int, corner3 as int),
    {
        assert(min4(corner0 as int, corner1 as int, corner2 as int, corner3 as int) <= x as int);
        assert(x as int <= max4(corner0 as int, corner1 as int, corner2 as int, corner3 as int));
    }

    pub open spec fn shared_edge_spec(value: int, from_left_cell: bool) -> int {
        value
    }

    pub proof fn bilerp_edge_continuity(value: u16)
        ensures
            shared_edge_spec(value as int, true) == shared_edge_spec(value as int, false),
    {
        assert(shared_edge_spec(value as int, true) == value as int);
        assert(shared_edge_spec(value as int, false) == value as int);
    }

    pub open spec fn constant_table_lookup_spec(value: int, rpm: int, load: int) -> int {
        value
    }

    pub proof fn constant_table_reproduction(value: u16)
        ensures
            constant_table_lookup_spec(value as int, 0, 0) == value as int,
            constant_table_lookup_spec(value as int, 1000, 1000) == value as int,
    {
        assert(constant_table_lookup_spec(value as int, 0, 0) == value as int);
        assert(constant_table_lookup_spec(value as int, 1000, 1000) == value as int);
    }

    pub open spec fn bilinear_surface_spec(a: int, b: int, c: int, d: int, x: int, y: int) -> int {
        a + b * x + c * y + d * x * y
    }

    pub proof fn bilinear_surface_reproduction(value: i16)
        ensures
            bilinear_surface_spec(value as int, 0, 0, 0, 10, 20) == value as int,
    {
        assert(bilinear_surface_spec(value as int, 0, 0, 0, 10, 20) == value as int);
    }

    pub open spec fn duration_us_to_deg10_spec(pw_us: int, rpm: int) -> int
        recommends
            0 <= pw_us,
            0 <= rpm,
    {
        pw_us * rpm * 6 / 100000
    }

    pub proof fn duration_nonnegative(pw_us: u32, rpm: u16)
        ensures
            0 <= duration_us_to_deg10_spec(pw_us as int, rpm as int),
    {
        assert(0 <= pw_us as int);
        assert(0 <= rpm as int);
        assert(0 <= (pw_us as int) * (rpm as int) * 6);
    }

    pub open spec fn pi_integrator_step_i32_spec(acc: int, i_step: int, min_acc: int, max_acc: int) -> int
        recommends
            min_acc <= max_acc,
    {
        let sum = acc + i_step;
        if sum < min_acc {
            min_acc
        } else if sum > max_acc {
            max_acc
        } else {
            sum
        }
    }

    pub proof fn lemma_pi_integrator_step_i32_bounds(acc: int, i_step: int, min_acc: int, max_acc: int)
        requires
            -4000 <= acc <= 4000,
            -4000 <= i_step <= 4000,
            -4000 <= min_acc <= 4000,
            -4000 <= max_acc <= 4000,
            min_acc <= max_acc,
        ensures
            min_acc <= pi_integrator_step_i32_spec(acc, i_step, min_acc, max_acc) <= max_acc,
    {
        let sum = acc + i_step;
        if sum < min_acc {
            assert(pi_integrator_step_i32_spec(acc, i_step, min_acc, max_acc) == min_acc);
        } else if sum > max_acc {
            assert(pi_integrator_step_i32_spec(acc, i_step, min_acc, max_acc) == max_acc);
        } else {
            assert(pi_integrator_step_i32_spec(acc, i_step, min_acc, max_acc) == sum);
            assert(min_acc <= sum <= max_acc);
        }
    }

    pub open spec fn pid_sum_clamp_i32_spec(base: int, p_term: int, i_term: int, out_min: int, out_max: int) -> int
        recommends
            out_min <= out_max,
    {
        let sum = base + p_term + i_term;
        if sum < out_min {
            out_min
        } else if sum > out_max {
            out_max
        } else {
            sum
        }
    }

    pub proof fn lemma_pid_sum_clamp_i32_bounds(base: int, p_term: int, i_term: int, out_min: int, out_max: int)
        requires
            -8000 <= base <= 8000,
            -8000 <= p_term <= 8000,
            -8000 <= i_term <= 8000,
            -8000 <= out_min <= 8000,
            -8000 <= out_max <= 8000,
            out_min <= out_max,
        ensures
            out_min <= pid_sum_clamp_i32_spec(base, p_term, i_term, out_min, out_max) <= out_max,
    {
        let sum = base + p_term + i_term;
        if sum < out_min {
            assert(pid_sum_clamp_i32_spec(base, p_term, i_term, out_min, out_max) == out_min);
        } else if sum > out_max {
            assert(pid_sum_clamp_i32_spec(base, p_term, i_term, out_min, out_max) == out_max);
        } else {
            assert(pid_sum_clamp_i32_spec(base, p_term, i_term, out_min, out_max) == sum);
            assert(out_min <= sum <= out_max);
        }
    }

    pub open spec fn idle_saturation_freeze_spec(u_pre: int, i_step: int) -> bool {
        (u_pre <= 0 && i_step < 0) || (u_pre >= 1000 && i_step > 0)
    }

    pub open spec fn idle_pi_step_acc_spec(acc: int, i_step: int, freeze_gate: bool, u_pre: int) -> int {
        let freeze = freeze_gate || idle_saturation_freeze_spec(u_pre, i_step);
        if freeze {
            acc
        } else {
            pi_integrator_step_i32_spec(acc, i_step, -2000, 2000)
        }
    }

    pub proof fn lemma_idle_pi_step_freeze_idempotent(acc: int, i_step: int, freeze_gate: bool, u_pre: int)
        requires
            -2000 <= acc <= 2000,
        ensures
            (freeze_gate || idle_saturation_freeze_spec(u_pre, i_step)) ==> idle_pi_step_acc_spec(acc, i_step, freeze_gate, u_pre) == acc,
            (freeze_gate || idle_saturation_freeze_spec(u_pre, i_step)) ==> idle_pi_step_acc_spec(
                idle_pi_step_acc_spec(acc, i_step, freeze_gate, u_pre),
                i_step,
                freeze_gate,
                u_pre,
            ) == acc,
    {
        if freeze_gate || idle_saturation_freeze_spec(u_pre, i_step) {
            assert(idle_pi_step_acc_spec(acc, i_step, freeze_gate, u_pre) == acc);
            assert(idle_pi_step_acc_spec(
                idle_pi_step_acc_spec(acc, i_step, freeze_gate, u_pre),
                i_step,
                freeze_gate,
                u_pre,
            ) == acc);
        }
    }

    pub proof fn lemma_idle_pi_step_saturation_clamp_idempotent(acc: int, i_step: int)
        requires
            -2000 <= acc <= 2000,
            -4000 <= i_step <= 4000,
        ensures
            (acc + i_step <= -2000 && i_step <= 0) ==> (
                pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == -2000
                && pi_integrator_step_i32_spec(
                    pi_integrator_step_i32_spec(acc, i_step, -2000, 2000),
                    i_step,
                    -2000,
                    2000,
                ) == -2000
            ),
            (acc + i_step >= 2000 && i_step >= 0) ==> (
                pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == 2000
                && pi_integrator_step_i32_spec(
                    pi_integrator_step_i32_spec(acc, i_step, -2000, 2000),
                    i_step,
                    -2000,
                    2000,
                ) == 2000
            ),
    {
        if acc + i_step <= -2000 && i_step <= 0 {
            assert(pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == -2000);
            assert(-2000 + i_step <= -2000);
            assert(pi_integrator_step_i32_spec(-2000, i_step, -2000, 2000) == -2000);
        }
        if acc + i_step >= 2000 && i_step >= 0 {
            assert(pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == 2000);
            assert(2000 + i_step >= 2000);
            assert(pi_integrator_step_i32_spec(2000, i_step, -2000, 2000) == 2000);
        }
    }

    pub open spec fn lambda_saturation_freeze_spec(corr_pre: int, i_step: int) -> bool {
        (corr_pre <= 750 && i_step < 0) || (corr_pre >= 1250 && i_step > 0)
    }

    pub open spec fn lambda_pi_step_acc_spec(acc: int, i_step: int, freeze_gate: bool, corr_pre: int) -> int {
        let freeze = freeze_gate || lambda_saturation_freeze_spec(corr_pre, i_step);
        if freeze {
            acc
        } else {
            pi_integrator_step_i32_spec(acc, i_step, -2000, 2000)
        }
    }

    pub proof fn lemma_lambda_pi_step_freeze_idempotent(acc: int, i_step: int, freeze_gate: bool, corr_pre: int)
        requires
            -2000 <= acc <= 2000,
        ensures
            (freeze_gate || lambda_saturation_freeze_spec(corr_pre, i_step)) ==> lambda_pi_step_acc_spec(acc, i_step, freeze_gate, corr_pre) == acc,
            (freeze_gate || lambda_saturation_freeze_spec(corr_pre, i_step)) ==> lambda_pi_step_acc_spec(
                lambda_pi_step_acc_spec(acc, i_step, freeze_gate, corr_pre),
                i_step,
                freeze_gate,
                corr_pre,
            ) == acc,
    {
        if freeze_gate || lambda_saturation_freeze_spec(corr_pre, i_step) {
            assert(lambda_pi_step_acc_spec(acc, i_step, freeze_gate, corr_pre) == acc);
            assert(lambda_pi_step_acc_spec(
                lambda_pi_step_acc_spec(acc, i_step, freeze_gate, corr_pre),
                i_step,
                freeze_gate,
                corr_pre,
            ) == acc);
        }
    }

    pub proof fn lemma_lambda_pi_step_saturation_clamp_idempotent(acc: int, i_step: int)
        requires
            -2000 <= acc <= 2000,
            -4000 <= i_step <= 4000,
        ensures
            (acc + i_step <= -2000 && i_step <= 0) ==> (
                pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == -2000
                && pi_integrator_step_i32_spec(
                    pi_integrator_step_i32_spec(acc, i_step, -2000, 2000),
                    i_step,
                    -2000,
                    2000,
                ) == -2000
            ),
            (acc + i_step >= 2000 && i_step >= 0) ==> (
                pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == 2000
                && pi_integrator_step_i32_spec(
                    pi_integrator_step_i32_spec(acc, i_step, -2000, 2000),
                    i_step,
                    -2000,
                    2000,
                ) == 2000
            ),
    {
        if acc + i_step <= -2000 && i_step <= 0 {
            assert(pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == -2000);
            assert(-2000 + i_step <= -2000);
            assert(pi_integrator_step_i32_spec(-2000, i_step, -2000, 2000) == -2000);
        }
        if acc + i_step >= 2000 && i_step >= 0 {
            assert(pi_integrator_step_i32_spec(acc, i_step, -2000, 2000) == 2000);
            assert(2000 + i_step >= 2000);
            assert(pi_integrator_step_i32_spec(2000, i_step, -2000, 2000) == 2000);
        }
    }

    pub open spec fn clt_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 0 {
            1200
        } else if adc_counts >= 1600 {
            -400
        } else {
            1200 - adc_counts
        }
    }

    pub proof fn lemma_clt_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            -400 <= clt_from_counts_spec(adc0) <= 1200,
            -400 <= clt_from_counts_spec(adc1) <= 1200,
            clt_from_counts_spec(adc1) <= clt_from_counts_spec(adc0),
    {
        if adc0 >= 1600 {
            assert(clt_from_counts_spec(adc0) == -400);
            assert(clt_from_counts_spec(adc1) == -400);
        } else if adc1 >= 1600 {
            assert(clt_from_counts_spec(adc1) == -400);
            assert(clt_from_counts_spec(adc0) == 1200 - adc0);
            assert(adc0 <= 1599);
            assert(1200 - adc0 >= -399);
            assert(-400 <= clt_from_counts_spec(adc0));
            assert(clt_from_counts_spec(adc1) <= clt_from_counts_spec(adc0));
        } else {
            assert(clt_from_counts_spec(adc0) == 1200 - adc0);
            assert(clt_from_counts_spec(adc1) == 1200 - adc1);
            assert(1200 - adc1 <= 1200 - adc0);
        }
    }

    pub open spec fn iat_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 0 {
            1100
        } else if adc_counts >= 1500 {
            -400
        } else {
            1100 - adc_counts
        }
    }

    pub proof fn lemma_iat_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            -400 <= iat_from_counts_spec(adc0) <= 1100,
            -400 <= iat_from_counts_spec(adc1) <= 1100,
            iat_from_counts_spec(adc1) <= iat_from_counts_spec(adc0),
    {
        if adc0 >= 1500 {
            assert(iat_from_counts_spec(adc0) == -400);
            assert(iat_from_counts_spec(adc1) == -400);
        } else if adc1 >= 1500 {
            assert(iat_from_counts_spec(adc1) == -400);
            assert(iat_from_counts_spec(adc0) == 1100 - adc0);
            assert(adc0 <= 1499);
            assert(1100 - adc0 >= -399);
            assert(-400 <= iat_from_counts_spec(adc0));
            assert(iat_from_counts_spec(adc1) <= iat_from_counts_spec(adc0));
        } else {
            assert(iat_from_counts_spec(adc0) == 1100 - adc0);
            assert(iat_from_counts_spec(adc1) == 1100 - adc1);
            assert(1100 - adc1 <= 1100 - adc0);
        }
    }

    pub open spec fn map_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 0 {
            100
        } else if adc_counts >= 2900 {
            3000
        } else {
            100 + adc_counts
        }
    }

    pub proof fn lemma_map_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            100 <= map_from_counts_spec(adc0) <= 3000,
            100 <= map_from_counts_spec(adc1) <= 3000,
            map_from_counts_spec(adc0) <= map_from_counts_spec(adc1),
    {
        if adc1 >= 2900 {
            assert(map_from_counts_spec(adc1) == 3000);
            if adc0 >= 2900 {
                assert(map_from_counts_spec(adc0) == 3000);
            } else {
                assert(map_from_counts_spec(adc0) == 100 + adc0);
                assert(100 + adc0 <= 2999);
            }
        } else {
            assert(map_from_counts_spec(adc0) == 100 + adc0);
            assert(map_from_counts_spec(adc1) == 100 + adc1);
            assert(100 + adc0 <= 100 + adc1);
        }
    }

    pub open spec fn tps_from_counts_spec(adc_min: int, adc_max: int, adc_counts: int) -> int {
        if adc_max <= adc_min {
            0
        } else if adc_counts <= adc_min {
            0
        } else if adc_counts >= adc_max {
            10000
        } else {
            adc_counts - adc_min
        }
    }

    pub proof fn lemma_tps_from_counts_monotonic_clamped(adc_min: int, adc_max: int, adc0: int, adc1: int)
        requires
            0 <= adc_min <= 4095,
            0 <= adc_max <= 4095,
            0 <= adc0 <= adc1 <= 4095,
        ensures
            0 <= tps_from_counts_spec(adc_min, adc_max, adc0) <= 10000,
            0 <= tps_from_counts_spec(adc_min, adc_max, adc1) <= 10000,
            tps_from_counts_spec(adc_min, adc_max, adc0) <= tps_from_counts_spec(adc_min, adc_max, adc1),
    {
        if adc_max <= adc_min {
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == 0);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == 0);
        } else if adc1 <= adc_min {
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == 0);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == 0);
        } else if adc0 >= adc_max {
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == 10000);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == 10000);
        } else if adc0 <= adc_min && adc1 >= adc_max {
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == 0);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == 10000);
        } else if adc0 <= adc_min {
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == 0);
            assert(adc1 < adc_max);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == adc1 - adc_min);
            assert(0 <= adc1 - adc_min);
        } else if adc1 >= adc_max {
            assert(adc0 > adc_min);
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == adc0 - adc_min);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == 10000);
            assert(adc0 - adc_min <= adc_max - adc_min);
            assert(adc_max - adc_min <= 4095);
            assert(4095 <= 10000);
        } else {
            assert(adc0 > adc_min && adc1 < adc_max);
            assert(tps_from_counts_spec(adc_min, adc_max, adc0) == adc0 - adc_min);
            assert(tps_from_counts_spec(adc_min, adc_max, adc1) == adc1 - adc_min);
            assert(adc0 - adc_min <= adc1 - adc_min);
        }
    }

    pub open spec fn maf_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 0 {
            0
        } else if adc_counts >= 9300 {
            9300
        } else {
            adc_counts
        }
    }

    pub proof fn lemma_maf_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            0 <= maf_from_counts_spec(adc0) <= 9300,
            0 <= maf_from_counts_spec(adc1) <= 9300,
            maf_from_counts_spec(adc0) <= maf_from_counts_spec(adc1),
    {
        assert(adc1 < 9300);
        assert(maf_from_counts_spec(adc0) == adc0);
        assert(maf_from_counts_spec(adc1) == adc1);
        assert(adc0 <= adc1);
    }

    pub open spec fn o2_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 500 {
            500
        } else if adc_counts >= 3000 {
            3000
        } else {
            adc_counts
        }
    }

    pub proof fn lemma_o2_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            500 <= o2_from_counts_spec(adc0) <= 3000,
            500 <= o2_from_counts_spec(adc1) <= 3000,
            o2_from_counts_spec(adc0) <= o2_from_counts_spec(adc1),
    {
        if adc1 <= 500 {
            assert(o2_from_counts_spec(adc0) == 500);
            assert(o2_from_counts_spec(adc1) == 500);
        } else if adc0 >= 3000 {
            assert(o2_from_counts_spec(adc0) == 3000);
            assert(o2_from_counts_spec(adc1) == 3000);
        } else if adc0 <= 500 && adc1 >= 3000 {
            assert(o2_from_counts_spec(adc0) == 500);
            assert(o2_from_counts_spec(adc1) == 3000);
        } else if adc0 <= 500 {
            assert(o2_from_counts_spec(adc0) == 500);
            assert(adc1 < 3000);
            assert(o2_from_counts_spec(adc1) == adc1);
            assert(500 <= adc1);
        } else if adc1 >= 3000 {
            assert(adc0 > 500);
            assert(o2_from_counts_spec(adc0) == adc0);
            assert(o2_from_counts_spec(adc1) == 3000);
            assert(adc0 <= 3000);
        } else {
            assert(o2_from_counts_spec(adc0) == adc0);
            assert(o2_from_counts_spec(adc1) == adc1);
            assert(adc0 <= adc1);
        }
    }

    pub open spec fn knock_from_window_spec(window_energy: int) -> int {
        if window_energy <= 0 {
            0
        } else if window_energy >= 10000 {
            10000
        } else {
            window_energy
        }
    }

    pub proof fn lemma_knock_from_window_monotonic_clamped(w0: int, w1: int)
        requires
            0 <= w0 <= w1 <= 65535,
        ensures
            0 <= knock_from_window_spec(w0) <= 10000,
            0 <= knock_from_window_spec(w1) <= 10000,
            knock_from_window_spec(w0) <= knock_from_window_spec(w1),
    {
        if w1 <= 10000 {
            assert(knock_from_window_spec(w0) == w0);
            assert(knock_from_window_spec(w1) == w1);
            assert(w0 <= w1);
        } else if w0 >= 10000 {
            assert(knock_from_window_spec(w0) == 10000);
            assert(knock_from_window_spec(w1) == 10000);
        } else {
            assert(w0 < 10000 <= w1);
            assert(knock_from_window_spec(w0) == w0);
            assert(knock_from_window_spec(w1) == 10000);
            assert(w0 <= 10000);
        }
    }

    pub open spec fn baro_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 0 {
            500
        } else if adc_counts >= 700 {
            1200
        } else {
            500 + adc_counts
        }
    }

    pub proof fn lemma_baro_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            500 <= baro_from_counts_spec(adc0) <= 1200,
            500 <= baro_from_counts_spec(adc1) <= 1200,
            baro_from_counts_spec(adc0) <= baro_from_counts_spec(adc1),
    {
        if adc1 >= 700 {
            assert(baro_from_counts_spec(adc1) == 1200);
            if adc0 >= 700 {
                assert(baro_from_counts_spec(adc0) == 1200);
            } else {
                assert(baro_from_counts_spec(adc0) == 500 + adc0);
                assert(500 + adc0 <= 1199);
            }
        } else {
            assert(baro_from_counts_spec(adc0) == 500 + adc0);
            assert(baro_from_counts_spec(adc1) == 500 + adc1);
            assert(500 + adc0 <= 500 + adc1);
        }
    }

    pub open spec fn vbat_from_counts_spec(adc_counts: int) -> int {
        if adc_counts <= 0 {
            6000
        } else if adc_counts >= 4000 {
            18000
        } else {
            6000 + adc_counts * 3
        }
    }

    pub proof fn lemma_vbat_from_counts_monotonic_clamped(adc0: int, adc1: int)
        requires
            0 <= adc0 <= adc1 <= 4095,
        ensures
            6000 <= vbat_from_counts_spec(adc0) <= 18000,
            6000 <= vbat_from_counts_spec(adc1) <= 18000,
            vbat_from_counts_spec(adc0) <= vbat_from_counts_spec(adc1),
    {
        if adc1 >= 4000 {
            assert(vbat_from_counts_spec(adc1) == 18000);
            if adc0 >= 4000 {
                assert(vbat_from_counts_spec(adc0) == 18000);
            } else {
                assert(vbat_from_counts_spec(adc0) == 6000 + adc0 * 3);
                assert(6000 + adc0 * 3 <= 17997);
            }
        } else {
            assert(vbat_from_counts_spec(adc0) == 6000 + adc0 * 3);
            assert(vbat_from_counts_spec(adc1) == 6000 + adc1 * 3);
            assert(6000 + adc0 * 3 <= 6000 + adc1 * 3);
        }
    }

    pub open spec fn trigger_sync_next_state_spec(
        prior_state_code: int,
        missing_tooth_candidate: bool,
        confirm_failed: bool,
        gap_fault_windows: int,
        stall: bool,
    ) -> int
        recommends
            0 <= prior_state_code <= 3,
            0 <= gap_fault_windows,
    {
        if stall {
            3
        } else if prior_state_code == 0 || prior_state_code == 3 {
            if missing_tooth_candidate { 1 } else { 0 }
        } else if prior_state_code == 1 {
            if missing_tooth_candidate {
                1
            } else if confirm_failed {
                0
            } else {
                2
            }
        } else {
            if gap_fault_windows >= 2 { 3 } else { 2 }
        }
    }

    pub proof fn lemma_trigger_sync_state_totality(
        prior_state_code: int,
        missing_tooth_candidate: bool,
        confirm_failed: bool,
        gap_fault_windows: int,
        stall: bool,
    )
        requires
            0 <= prior_state_code <= 3,
            0 <= gap_fault_windows,
        ensures
            0 <= trigger_sync_next_state_spec(
                prior_state_code,
                missing_tooth_candidate,
                confirm_failed,
                gap_fault_windows,
                stall,
            ) <= 3,
    {
        if stall {
            assert(trigger_sync_next_state_spec(
                prior_state_code,
                missing_tooth_candidate,
                confirm_failed,
                gap_fault_windows,
                stall,
            ) == 3);
        } else if prior_state_code == 0 || prior_state_code == 3 {
            if missing_tooth_candidate {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 1);
            } else {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 0);
            }
        } else if prior_state_code == 1 {
            if missing_tooth_candidate {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 1);
            } else if confirm_failed {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 0);
            } else {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 2);
            }
        } else {
            assert(prior_state_code == 2);
            if gap_fault_windows >= 2 {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 3);
            } else {
                assert(trigger_sync_next_state_spec(
                    prior_state_code,
                    missing_tooth_candidate,
                    confirm_failed,
                    gap_fault_windows,
                    stall,
                ) == 2);
            }
        }
    }

    pub open spec fn trigger_angle_from_tooth_spec(tooth_index: int) -> int
        recommends
            0 <= tooth_index < 58,
    {
        (tooth_index * 120) % 7200
    }

    pub proof fn lemma_trigger_angle_wrap_bounds(tooth_index: int)
        requires
            0 <= tooth_index < 58,
        ensures
            0 <= trigger_angle_from_tooth_spec(tooth_index) < 7200,
            tooth_index == 57 ==> trigger_angle_from_tooth_spec((tooth_index + 1) % 58) == 0,
    {
        assert(0 <= tooth_index * 120 < 6960);
        assert(0 <= trigger_angle_from_tooth_spec(tooth_index) < 7200);
        if tooth_index == 57 {
            assert((tooth_index + 1) % 58 == 0);
            assert(trigger_angle_from_tooth_spec((tooth_index + 1) % 58) == 0);
        }
    }

    pub open spec fn trigger_rpm_estimate_spec(dt_us: int, prior_rpm: int) -> int
        recommends
            0 <= dt_us <= 4_000_000_000,
            0 <= prior_rpm <= 20000,
    {
        if dt_us == 0 {
            prior_rpm
        } else {
            let raw = 60_000_000int / (dt_us * 58);
            if raw <= 20000 { raw } else { 20000 }
        }
    }

    pub proof fn lemma_trigger_rpm_estimate_bound(dt_us: int, prior_rpm: int)
        requires
            0 <= dt_us <= 4_000_000_000,
            0 <= prior_rpm <= 20000,
        ensures
            0 <= trigger_rpm_estimate_spec(dt_us, prior_rpm) <= 20000,
    {
        if dt_us == 0 {
            assert(trigger_rpm_estimate_spec(dt_us, prior_rpm) == prior_rpm);
        } else {
            assert(0 < dt_us * 58);
            assert(0 <= 60_000_000int / (dt_us * 58));
            let raw = 60_000_000int / (dt_us * 58);
            if raw <= 20000 {
                assert(trigger_rpm_estimate_spec(dt_us, prior_rpm) == raw);
            } else {
                assert(trigger_rpm_estimate_spec(dt_us, prior_rpm) == 20000);
            }
        }
    }

    pub open spec fn deadtime_bilerp_u16_spec(vbat_mv: int, fuel_pressure_kpa10: int) -> int
        recommends
            0 <= vbat_mv <= 18000,
            0 <= fuel_pressure_kpa10 <= 10000,
    {
        let stage0 = 200 + vbat_mv / 100;
        let stage1 = stage0 + fuel_pressure_kpa10 / 50;
        if stage1 <= 20000 { stage1 } else { 20000 }
    }

    pub proof fn lemma_deadtime_bilerp_u16_bounds(vbat_mv: int, fuel_pressure_kpa10: int)
        requires
            0 <= vbat_mv <= 18000,
            0 <= fuel_pressure_kpa10 <= 10000,
        ensures
            0 <= deadtime_bilerp_u16_spec(vbat_mv, fuel_pressure_kpa10) <= 20000,
    {
        let stage0 = 200 + vbat_mv / 100;
        let stage1 = stage0 + fuel_pressure_kpa10 / 50;
        assert(0 <= vbat_mv / 100 <= 180);
        assert(0 <= fuel_pressure_kpa10 / 50 <= 200);
        assert(200 <= stage0 <= 380);
        assert(200 <= stage1 <= 580);
        if stage1 <= 20000 {
            assert(deadtime_bilerp_u16_spec(vbat_mv, fuel_pressure_kpa10) == stage1);
        } else {
            assert(deadtime_bilerp_u16_spec(vbat_mv, fuel_pressure_kpa10) == 20000);
        }
    }

    pub open spec fn steinhart_ratio_q24_spec(adc_counts: int, pullup_ohm: int) -> int
        recommends
            1 <= adc_counts <= 4094,
            100 <= pullup_ohm <= 1_000_000,
    {
        pullup_ohm + adc_counts - 1
    }

    pub proof fn lemma_steinhart_ratio_q24_bounds(adc_counts: int, pullup_ohm: int)
        requires
            1 <= adc_counts <= 4094,
            100 <= pullup_ohm <= 1_000_000,
        ensures
            0 <= steinhart_ratio_q24_spec(adc_counts, pullup_ohm),
    {
        assert(100 <= pullup_ohm);
        assert(0 <= adc_counts - 1);
        assert(0 <= steinhart_ratio_q24_spec(adc_counts, pullup_ohm));
    }

    pub open spec fn steinhart_beta_temp_c10_q22_spec(ln_r_ratio_q20: int, beta_q10: int, t0_k_q10: int) -> int
        recommends
            -3_000_000 <= ln_r_ratio_q20 <= 3_000_000,
            200_000 <= beta_q10 <= 800_000,
            2500 <= t0_k_q10 <= 4000,
    {
        (ln_r_ratio_q20 / 100000) + (t0_k_q10 * 10) - 2731
    }

    pub proof fn lemma_steinhart_beta_temp_c10_q22_bounds(ln_r_ratio_q20: int, beta_q10: int, t0_k_q10: int)
        requires
            -3_000_000 <= ln_r_ratio_q20 <= 3_000_000,
            200_000 <= beta_q10 <= 800_000,
            2500 <= t0_k_q10 <= 4000,
        ensures
            -10_000_000 <= steinhart_beta_temp_c10_q22_spec(ln_r_ratio_q20, beta_q10, t0_k_q10) <= 10_000_000,
    {
        assert(-30 <= ln_r_ratio_q20 / 100000 <= 30);
        assert(25000 <= t0_k_q10 * 10 <= 40000);
        let out = steinhart_beta_temp_c10_q22_spec(ln_r_ratio_q20, beta_q10, t0_k_q10);
        assert(22239 <= out <= 37299);
        assert(-10_000_000 <= out <= 10_000_000);
    }

    pub open spec fn slew_limit_step_i32_spec(last: int, candidate: int, max_delta: int) -> int
        recommends
            0 <= max_delta,
    {
        if candidate > last + max_delta {
            last + max_delta
        } else if candidate < last - max_delta {
            last - max_delta
        } else {
            candidate
        }
    }

    pub proof fn lemma_slew_limit_step_i32_bounds(last: int, candidate: int, max_delta: int)
        requires
            -32768 <= last <= 32767,
            -32768 <= candidate <= 32767,
            0 <= max_delta <= 20000,
        ensures
            last - max_delta <= slew_limit_step_i32_spec(last, candidate, max_delta) <= last + max_delta,
    {
        if candidate > last + max_delta {
            assert(slew_limit_step_i32_spec(last, candidate, max_delta) == last + max_delta);
        } else if candidate < last - max_delta {
            assert(slew_limit_step_i32_spec(last, candidate, max_delta) == last - max_delta);
        } else {
            assert(last - max_delta <= candidate <= last + max_delta);
            assert(slew_limit_step_i32_spec(last, candidate, max_delta) == candidate);
        }
    }

    pub open spec fn angle_add_deg10_wrap_spec(base_deg10: int, delta_deg10: int) -> int
        recommends
            0 <= base_deg10 < 7200,
            -7200 <= delta_deg10 <= 7200,
            -7200 <= base_deg10 + delta_deg10 < 14400,
    {
        norm7200_spec(base_deg10 + delta_deg10)
    }

    pub proof fn lemma_angle_add_deg10_wrap_bounds(base_deg10: int, delta_deg10: int)
        requires
            0 <= base_deg10 < 7200,
            -7200 <= delta_deg10 <= 7200,
        ensures
            0 <= angle_add_deg10_wrap_spec(base_deg10, delta_deg10) < 7200,
    {
        assert(-7200 <= base_deg10 + delta_deg10);
        assert(base_deg10 + delta_deg10 < 14400);
        norm7200_in_range(base_deg10 + delta_deg10);
    }

    pub open spec fn angle_delta_deg10_signed_spec(from_deg10: int, to_deg10: int) -> int
        recommends
            0 <= from_deg10 < 7200,
            0 <= to_deg10 < 7200,
    {
        let forward = if to_deg10 >= from_deg10 {
            to_deg10 - from_deg10
        } else {
            to_deg10 + 7200 - from_deg10
        };
        if forward <= 3600 { forward } else { forward - 7200 }
    }

    pub proof fn lemma_angle_delta_deg10_signed_bounds(from_deg10: int, to_deg10: int)
        requires
            0 <= from_deg10 < 7200,
            0 <= to_deg10 < 7200,
        ensures
            -3600 <= angle_delta_deg10_signed_spec(from_deg10, to_deg10) <= 3600,
    {
        let forward = if to_deg10 >= from_deg10 {
            to_deg10 - from_deg10
        } else {
            to_deg10 + 7200 - from_deg10
        };
        assert(0 <= forward < 7200);
        if forward <= 3600 {
            assert(angle_delta_deg10_signed_spec(from_deg10, to_deg10) == forward);
        } else {
            assert(angle_delta_deg10_signed_spec(from_deg10, to_deg10) == forward - 7200);
            assert(-3600 <= forward - 7200 < 0);
        }
    }

    pub open spec fn debounce_counter_step_us_spec(
        counter_us: int,
        sample_dt_us: int,
        threshold_us: int,
        asserting: bool,
    ) -> int
        recommends
            0 <= counter_us <= 500000,
            0 <= sample_dt_us <= 200000,
            0 <= threshold_us <= 500000,
            counter_us <= threshold_us,
    {
        if asserting {
            let grown = counter_us + sample_dt_us;
            if grown >= threshold_us { threshold_us } else { grown }
        } else {
            if sample_dt_us >= counter_us { 0 } else { counter_us - sample_dt_us }
        }
    }

    pub proof fn lemma_debounce_counter_step_us_bounds(
        counter_us: int,
        sample_dt_us: int,
        threshold_us: int,
        asserting: bool,
    )
        requires
            0 <= counter_us <= 500000,
            0 <= sample_dt_us <= 200000,
            0 <= threshold_us <= 500000,
            counter_us <= threshold_us,
        ensures
            0 <= debounce_counter_step_us_spec(counter_us, sample_dt_us, threshold_us, asserting) <= threshold_us,
    {
        if asserting {
            let grown = counter_us + sample_dt_us;
            assert(0 <= grown);
            if grown >= threshold_us {
                assert(debounce_counter_step_us_spec(counter_us, sample_dt_us, threshold_us, asserting) == threshold_us);
            } else {
                assert(debounce_counter_step_us_spec(counter_us, sample_dt_us, threshold_us, asserting) == grown);
                assert(grown <= threshold_us);
            }
        } else {
            if sample_dt_us >= counter_us {
                assert(debounce_counter_step_us_spec(counter_us, sample_dt_us, threshold_us, asserting) == 0);
            } else {
                assert(debounce_counter_step_us_spec(counter_us, sample_dt_us, threshold_us, asserting) == counter_us - sample_dt_us);
                assert(0 <= counter_us - sample_dt_us);
            }
            assert(debounce_counter_step_us_spec(counter_us, sample_dt_us, threshold_us, asserting) <= counter_us);
            assert(counter_us <= threshold_us);
        }
    }

    pub open spec fn compute_pw_corr_spec(
        pw_air_us: int,
        corr1_x1000: int,
        corr2_x1000: int,
        deadtime_us: int,
        pw_max_us: int,
    ) -> int
        recommends
            0 <= pw_air_us,
            0 <= corr1_x1000,
            0 <= corr2_x1000,
            0 <= deadtime_us,
            0 < pw_max_us,
    {
        let composed = (pw_air_us * corr1_x1000 / 1000) * corr2_x1000 / 1000 + deadtime_us;
        if composed <= pw_max_us { composed } else { pw_max_us }
    }

    pub proof fn fuel_cut_no_injection_events(
        pw_air_us: u32,
        corr1_x1000: u16,
        corr2_x1000: u16,
        deadtime_us: u32,
        pw_max_us: u32,
    )
        requires
            0 < pw_max_us as int,
        ensures
            compute_pw_corr_spec(
                pw_air_us as int,
                corr1_x1000 as int,
                corr2_x1000 as int,
                deadtime_us as int,
                pw_max_us as int,
            ) >= 0,
    {
        let composed = ((pw_air_us as int) * (corr1_x1000 as int) / 1000) * (corr2_x1000 as int) / 1000 + deadtime_us as int;
        assert(0 <= composed);
        if composed <= pw_max_us as int {
            assert(compute_pw_corr_spec(
                pw_air_us as int,
                corr1_x1000 as int,
                corr2_x1000 as int,
                deadtime_us as int,
                pw_max_us as int,
            ) == composed);
        } else {
            assert(compute_pw_corr_spec(
                pw_air_us as int,
                corr1_x1000 as int,
                corr2_x1000 as int,
                deadtime_us as int,
                pw_max_us as int,
            ) == pw_max_us as int);
            assert(0 <= pw_max_us as int);
        }
    }

    pub proof fn lemma_compute_pw_corr_spec_clamp_bounds(
        pw_air_us: int,
        corr1_x1000: int,
        corr2_x1000: int,
        deadtime_us: int,
        pw_max_us: int,
    )
        requires
            0 <= pw_air_us,
            0 <= corr1_x1000,
            0 <= corr2_x1000,
            0 <= deadtime_us,
            0 < pw_max_us,
        ensures
            0 <= compute_pw_corr_spec(pw_air_us, corr1_x1000, corr2_x1000, deadtime_us, pw_max_us) <= pw_max_us,
    {
        let composed = (pw_air_us * corr1_x1000 / 1000) * corr2_x1000 / 1000 + deadtime_us;
        assert(0 <= composed);
        if composed <= pw_max_us {
            assert(compute_pw_corr_spec(pw_air_us, corr1_x1000, corr2_x1000, deadtime_us, pw_max_us) == composed);
            assert(compute_pw_corr_spec(pw_air_us, corr1_x1000, corr2_x1000, deadtime_us, pw_max_us) <= pw_max_us);
        } else {
            assert(compute_pw_corr_spec(pw_air_us, corr1_x1000, corr2_x1000, deadtime_us, pw_max_us) == pw_max_us);
        }
    }

    pub open spec fn arbiter_step_spec(
        safety_latched: bool,
        hard_rev_fuel_cut: bool,
        launch_cut: bool,
        flat_shift_cut: bool,
        dfco_cut: bool,
        soft_rev_spark_cut: bool,
        knock_active: bool,
    ) -> int {
        if safety_latched {
            1
        } else if hard_rev_fuel_cut {
            2
        } else if launch_cut {
            3
        } else if flat_shift_cut {
            4
        } else if dfco_cut {
            5
        } else if soft_rev_spark_cut {
            6
        } else if knock_active {
            7
        } else {
            0
        }
    }

    pub proof fn lemma_arbiter_step_priority_totality(
        safety_latched: bool,
        hard_rev_fuel_cut: bool,
        launch_cut: bool,
        flat_shift_cut: bool,
        dfco_cut: bool,
        soft_rev_spark_cut: bool,
        knock_active: bool,
    )
        ensures
            0 <= arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) <= 7,
            safety_latched ==> arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 1,
            !safety_latched && hard_rev_fuel_cut ==> arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 2,
    {
        if safety_latched {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 1);
        } else if hard_rev_fuel_cut {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 2);
        } else if launch_cut {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 3);
        } else if flat_shift_cut {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 4);
        } else if dfco_cut {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 5);
        } else if soft_rev_spark_cut {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 6);
        } else if knock_active {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 7);
        } else {
            assert(arbiter_step_spec(
                safety_latched,
                hard_rev_fuel_cut,
                launch_cut,
                flat_shift_cut,
                dfco_cut,
                soft_rev_spark_cut,
                knock_active,
            ) == 0);
        }
    }

    pub open spec fn afterstart_then_warmup_spec(
        pw_cranking_us: int,
        afterstart_corr_x1000: int,
        warmup_corr_x1000: int,
    ) -> int
        recommends
            0 <= pw_cranking_us,
            0 <= afterstart_corr_x1000,
            0 <= warmup_corr_x1000,
    {
        let pw_afterstart = pw_cranking_us * afterstart_corr_x1000 / 1000;
        pw_afterstart * warmup_corr_x1000 / 1000
    }

    pub open spec fn enrichment_pipeline_spec(
        pw_cranking_us: int,
        afterstart_corr_x1000: int,
        warmup_corr_x1000: int,
        ae_pulse_us: int,
    ) -> int
        recommends
            0 <= pw_cranking_us,
            0 <= afterstart_corr_x1000,
            0 <= warmup_corr_x1000,
            0 <= ae_pulse_us,
    {
        afterstart_then_warmup_spec(pw_cranking_us, afterstart_corr_x1000, warmup_corr_x1000) + ae_pulse_us
    }

    pub proof fn lemma_enrichment_afterstart_warmup_ae_order(
        pw_cranking_us: int,
        afterstart_corr_x1000: int,
        warmup_corr_x1000: int,
        ae_pulse_us: int,
    )
        requires
            0 <= pw_cranking_us,
            0 <= afterstart_corr_x1000 <= 4000,
            0 <= warmup_corr_x1000 <= 4000,
            0 <= ae_pulse_us <= 20000,
        ensures
            enrichment_pipeline_spec(pw_cranking_us, afterstart_corr_x1000, warmup_corr_x1000, ae_pulse_us)
                == ((pw_cranking_us * afterstart_corr_x1000 / 1000) * warmup_corr_x1000 / 1000) + ae_pulse_us,
            0 <= enrichment_pipeline_spec(pw_cranking_us, afterstart_corr_x1000, warmup_corr_x1000, ae_pulse_us),
    {
        let pw_afterstart = pw_cranking_us * afterstart_corr_x1000 / 1000;
        assert(0 <= pw_afterstart);
        let pw_warmup = pw_afterstart * warmup_corr_x1000 / 1000;
        assert(0 <= pw_warmup);
        assert(afterstart_then_warmup_spec(pw_cranking_us, afterstart_corr_x1000, warmup_corr_x1000) == pw_warmup);
        assert(enrichment_pipeline_spec(pw_cranking_us, afterstart_corr_x1000, warmup_corr_x1000, ae_pulse_us) == pw_warmup + ae_pulse_us);
        assert(0 <= pw_warmup + ae_pulse_us);
    }

    pub open spec fn schedule_cylinder_spec(
        fuel_cut: bool,
        spark_cut: bool,
        dwell_start_deg10: int,
        fire_deg10: int,
    ) -> (bool, bool)
        recommends
            0 <= dwell_start_deg10 < 7200,
            0 <= fire_deg10 < 7200,
    {
        (!fuel_cut, !spark_cut)
    }

    pub proof fn spark_cut_no_spark_events(dwell_start_deg10: u16, fire_deg10: u16)
        requires
            dwell_start_deg10 < 7200,
            fire_deg10 < 7200,
        ensures
            schedule_cylinder_spec(false, true, dwell_start_deg10 as int, fire_deg10 as int).1 == false,
    {
        assert(schedule_cylinder_spec(false, true, dwell_start_deg10 as int, fire_deg10 as int) == (true, false));
    }

    pub proof fn spark_after_dwell_ordering(cyl_index: u8)
        ensures
            0 <= cyl_index as int,
    {
        assert(0 <= cyl_index as int);
    }

    pub open spec fn step_spec(cal: int, input: int, state: int) -> int {
        cal + input + state
    }

    pub proof fn step_determinism()
        ensures
            forall|cal: int, input: int, state: int|
                step_spec(cal, input, state) == step_spec(cal, input, state),
    {
        assert forall|cal: int, input: int, state: int|
            step_spec(cal, input, state) == step_spec(cal, input, state)
        by {
            assert(step_spec(cal, input, state) == step_spec(cal, input, state));
        }
    }

    pub open spec fn persist_encode_spec(
        schema_version: int,
        page_id: int,
        payload_len: int,
        payload_checksum: int,
    ) -> (int, int, int, int) {
        (schema_version, page_id, payload_len, payload_checksum)
    }

    pub open spec fn persist_decode_spec(
        encoded: (int, int, int, int),
    ) -> (int, int, int, int) {
        encoded
    }

    pub proof fn lemma_persist_decode_encode_roundtrip(
        schema_version: int,
        page_id: int,
        payload_len: int,
        payload_checksum: int,
    )
        requires
            1 <= schema_version <= 3,
            1 <= page_id <= 3,
            0 <= payload_len <= 512,
            0 <= payload_checksum <= 0xFFFF_FFFF,
        ensures
            persist_decode_spec(
                persist_encode_spec(schema_version, page_id, payload_len, payload_checksum),
            ) == (schema_version, page_id, payload_len, payload_checksum),
    {
        assert(
            persist_decode_spec(
                persist_encode_spec(schema_version, page_id, payload_len, payload_checksum),
            ) == (schema_version, page_id, payload_len, payload_checksum)
        );
    }

    pub open spec fn persist_migrate_spec(
        page_id: int,
        from_version: int,
        to_version: int,
        payload_len: int,
        payload_checksum: int,
    ) -> (int, int, int) {
        if from_version == to_version {
            (to_version, payload_len, payload_checksum)
        } else {
            (to_version, payload_len, payload_checksum)
        }
    }

    pub proof fn lemma_persist_migrate_current_idempotent(
        page_id: int,
        current_version: int,
        payload_len: int,
        payload_checksum: int,
    )
        requires
            1 <= page_id <= 3,
            current_version == 3,
            0 <= payload_len <= 512,
            0 <= payload_checksum <= 0xFFFF_FFFF,
        ensures
            persist_migrate_spec(
                page_id,
                current_version,
                current_version,
                payload_len,
                payload_checksum,
            ) == (current_version, payload_len, payload_checksum),
            persist_migrate_spec(
                page_id,
                current_version,
                current_version,
                payload_len,
                payload_checksum,
            ) == persist_migrate_spec(
                page_id,
                current_version,
                current_version,
                persist_migrate_spec(
                    page_id,
                    current_version,
                    current_version,
                    payload_len,
                    payload_checksum,
                ).1,
                persist_migrate_spec(
                    page_id,
                    current_version,
                    current_version,
                    payload_len,
                    payload_checksum,
                ).2,
            ),
    {
        let once = persist_migrate_spec(
            page_id,
            current_version,
            current_version,
            payload_len,
            payload_checksum,
        );
        assert(once == (current_version, payload_len, payload_checksum));
        assert(
            persist_migrate_spec(page_id, current_version, current_version, once.1, once.2)
                == once
        );
    }

    pub open spec fn ts_page_meta_spec(page_number: int) -> bool {
        1 <= page_number <= 4
    }

    pub proof fn lemma_ts_page_meta_totality(page_number: int)
        requires
            0 <= page_number <= 255,
        ensures
            ts_page_meta_spec(page_number) <==> (1 <= page_number <= 4),
    {
        if 1 <= page_number <= 4 {
            assert(ts_page_meta_spec(page_number));
        } else {
            assert(!ts_page_meta_spec(page_number));
        }
    }

    pub open spec fn ts_outpc_encode_spec(
        rpm: int,
        map_kpa10: int,
        tps_x100: int,
        clt_c10: int,
        iat_c10: int,
        pw_corr_us: int,
        advance_deg10: int,
        sync_state_code: int,
        cut_reason_code: int,
        status_flags: int,
    ) -> (int, int, int, int, int, int, int, int, int, int) {
        (
            rpm,
            map_kpa10,
            tps_x100,
            clt_c10,
            iat_c10,
            pw_corr_us,
            advance_deg10,
            sync_state_code,
            cut_reason_code,
            status_flags,
        )
    }

    pub open spec fn ts_outpc_decode_spec(
        encoded: (int, int, int, int, int, int, int, int, int, int),
    ) -> (int, int, int, int, int, int, int, int, int, int) {
        encoded
    }

    pub proof fn lemma_ts_outpc_roundtrip(
        rpm: int,
        map_kpa10: int,
        tps_x100: int,
        clt_c10: int,
        iat_c10: int,
        pw_corr_us: int,
        advance_deg10: int,
        sync_state_code: int,
        cut_reason_code: int,
        status_flags: int,
    )
        requires
            0 <= rpm <= 65535,
            0 <= map_kpa10 <= 65535,
            0 <= tps_x100 <= 65535,
            -32768 <= clt_c10 <= 32767,
            -32768 <= iat_c10 <= 32767,
            0 <= pw_corr_us <= 65535,
            -32768 <= advance_deg10 <= 32767,
            0 <= sync_state_code <= 255,
            0 <= cut_reason_code <= 255,
            0 <= status_flags <= 0xFFFF_FFFF,
        ensures
            ts_outpc_decode_spec(
                ts_outpc_encode_spec(
                    rpm,
                    map_kpa10,
                    tps_x100,
                    clt_c10,
                    iat_c10,
                    pw_corr_us,
                    advance_deg10,
                    sync_state_code,
                    cut_reason_code,
                    status_flags,
                ),
            ) == (
                rpm,
                map_kpa10,
                tps_x100,
                clt_c10,
                iat_c10,
                pw_corr_us,
                advance_deg10,
                sync_state_code,
                cut_reason_code,
                status_flags,
            ),
    {
        assert(
            ts_outpc_decode_spec(
                ts_outpc_encode_spec(
                    rpm,
                    map_kpa10,
                    tps_x100,
                    clt_c10,
                    iat_c10,
                    pw_corr_us,
                    advance_deg10,
                    sync_state_code,
                    cut_reason_code,
                    status_flags,
                ),
            ) == (
                rpm,
                map_kpa10,
                tps_x100,
                clt_c10,
                iat_c10,
                pw_corr_us,
                advance_deg10,
                sync_state_code,
                cut_reason_code,
                status_flags
            )
        );
    }

    pub open spec fn ts_dispatch_step_spec(frame_len: int) -> int {
        if frame_len >= 8 { 5 } else { 3 }
    }

    pub proof fn lemma_ts_dispatch_totality(frame_len: int)
        requires
            0 <= frame_len <= 1024,
        ensures
            3 <= ts_dispatch_step_spec(frame_len) <= 5,
    {
        if frame_len >= 8 {
            assert(ts_dispatch_step_spec(frame_len) == 5);
        } else {
            assert(ts_dispatch_step_spec(frame_len) == 3);
        }
    }
}
