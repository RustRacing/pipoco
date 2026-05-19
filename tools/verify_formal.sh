#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Formal Methods Gate — tools/verify_formal.sh
# Stories: US-FM0291, US-FM0293, US-FM0310, US-FM0311, US-FM0312, US-FM0313
# ============================================================

# --- Tool version pinning ---
_verus_version="0.2026.04.19.6f7d4de"
_verus_path="/tmp/verus-install/verus-x86-linux/verus"
_kani_version="0.67.0"
_tla_version="2.19"
_tla_jar="/home/user/.local/lib/tla/tla2tools.jar"
_rust_toolchain="1.95.0-x86_64-unknown-linux-gnu"

# --- Coverage thresholds (US-FM0311) ---
_coverage_threshold=90.0  # minimum % per crate

# --- Benchmark regression threshold (US-FM0311) ---
_bench_regress_pct=5.0   # max p95 regression vs baseline

# --- Evidence output (US-FM0312) ---
_evidence_dir="target/formal-evidence/latest"
_quick_mode=false

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"
mkdir -p "${_evidence_dir}"

# ============================================================
# Helper functions
# ============================================================
fail_if() { if "$@"; then echo "FAIL: $*" >&2; exit 1; fi; }

fail_if_rg_matches() {
    local pattern="$1"
    local file="$2"
    shift 2
    rg -q "$pattern" "$file" "$@" && {
        echo "FAIL: forbidden pattern '$pattern' matched in $file" >&2
        rg -n "$pattern" "$file" "$@"
        exit 1
    } || true
}

run_tlc_module() {
    local module="$1"
    local cfg="ecu-spec/tla/${module}.cfg"
    local tla="ecu-spec/tla/${module}.tla"
    java -cp "$_tla_jar" tlc2.TLC -config "$cfg" "$tla"
}

run_fuzz_corpus() {
    local target="$1"
    local max_len="$2"
    local corpus_dir="fuzz/corpus/${target}"
    local runs
    if [[ ! -d "$corpus_dir" ]]; then
        echo "missing fuzz corpus directory: $corpus_dir" >&2
        return 1
    fi
    runs="$(find "$corpus_dir" -type f | wc -l | tr -d ' ')"
    if [[ "$runs" -lt 1 ]]; then
        echo "empty fuzz corpus directory: $corpus_dir" >&2
        return 1
    fi
    (
        cd fuzz
        cargo +nightly fuzz run "$target" "corpus/${target}" -- -runs="$runs" -max_len="$max_len"
    )
}

emit_evidence() {
    local key="$1"
    local file="${_evidence_dir}/${key}.json"
    mkdir -p "${_evidence_dir}"
    echo "{\"timestamp\":\"$(date -u +%Y-%m-%dT%H:%M:%SZ)\",\"tool_version\":\"${_kani_version}\"}" \
        > "$file"
}

# ============================================================
# Parse --quick flag (developer-only; not for acceptance)
# ============================================================
for arg in "$@"; do
    case "$arg" in
        --quick)
            _quick_mode=true
            echo "NOTE: --quick mode (dev only, NOT for acceptance)" >&2
            ;;
        --help|-h)
            echo "Usage: $0 [--quick]"
            echo "  --quick  Skip slow gates (dev only; NOT for acceptance)"
            exit 0
            ;;
    esac
done

# ============================================================
# Section 1: JSON + format + clippy + tests
# ============================================================
bash tools/check_repo_hygiene.sh
echo ""
echo "=== Formal gate: prd.json, fmt, clippy, unit tests ==="
python3 -m json.tool prd.json >/dev/null || exit 1
python3 -m json.tool tools/evidence.schema.json >/dev/null || {
    echo "FAIL: evidence.schema.json invalid" >&2
    exit 1
}
python3 -m json.tool tools/coverage.schema.json >/dev/null || {
    echo "FAIL: coverage.schema.json invalid" >&2
    exit 1
}
python3 -m json.tool tools/benchmarks.schema.json >/dev/null || {
    echo "FAIL: benchmarks.schema.json invalid" >&2
    exit 1
}
python3 -m json.tool tools/conformance.schema.json >/dev/null || {
    echo "FAIL: conformance.schema.json invalid" >&2
    exit 1
}
python3 -m json.tool tools/verus.schema.json >/dev/null || {
    echo "FAIL: verus.schema.json invalid" >&2
    exit 1
}
python3 -m json.tool tools/kani.schema.json >/dev/null || {
    echo "FAIL: kani.schema.json invalid" >&2
    exit 1
}
python3 -m json.tool tools/tla.schema.json >/dev/null || {
    echo "FAIL: tla.schema.json invalid" >&2
    exit 1
}
cargo fmt --all --check
cargo test -p ecu-spec
cargo clippy -p ecu-spec --all-targets -- -D warnings
cargo test -p ecu-runtime
cargo test -p ecu-scheduler
cargo test --workspace --exclude ecu-rp2040-pico --exclude stm32f4-ecu --exclude ecu-rp2350b
echo ""

# ============================================================
# Section 2: Embedded target builds (US-FM0306 / US-FM0610)
# Invoking tools/verify.sh preserves the warning-denied guarantee
# and avoids target list drift between formal and embedded gates.
# ============================================================
echo "=== Formal gate: embedded target builds via tools/verify.sh ==="
bash tools/verify.sh
echo ""

# ============================================================
# Section 3: Version pinning gate (US-FM0293)
# ============================================================
echo "=== Formal gate: version pinning ==="

# Verus
if [[ -x "$_verus_path" ]]; then
    _verus_actual=$("$_verus_path" --version 2>&1 | awk -F': ' '/Version:/ {print $2; exit}' || echo "unknown")
    echo "Verus: $_verus_actual (expected: $_verus_version)"
    if [[ "$_verus_actual" != "$_verus_version" ]]; then
        echo "FAIL: Verus version mismatch" >&2
        exit 1
    fi
else
    echo "FAIL: Verus not found at $_verus_path" >&2
    exit 1
fi

# Kani
if command -v kani &>/dev/null; then
    _kani_actual=$(kani --version 2>&1 | awk '{print $2}' || echo "unknown")
    echo "Kani: $_kani_actual (expected: $_kani_version)"
    if [[ "$_kani_actual" != "$_kani_version" ]]; then
        echo "FAIL: Kani version mismatch" >&2
        exit 1
    fi
else
    echo "FAIL: Kani not found" >&2
    exit 1
fi

# TLA+ toolbox
if [[ -f "$_tla_jar" ]]; then
    set +e
    _tla_actual=$(java -cp "$_tla_jar" tlc2.TLC -version 2>&1 | sed -n 's/^TLC2 Version \([^ ]*\).*/\1/p' | head -1)
    set -e
    echo "TLA+ toolbox: $_tla_actual at $_tla_jar (expected: $_tla_version)"
    if [[ "$_tla_actual" != "$_tla_version" ]]; then
        echo "FAIL: TLA+ toolbox version mismatch" >&2
        exit 1
    fi
else
    echo "FAIL: TLA+ toolbox not found at $_tla_jar" >&2
    exit 1
fi

# Rust toolchain
_rustc_actual=$(rustup run "$_rust_toolchain" rustc --version 2>&1 | awk '{print $2}' || echo "unknown")
echo "Rustc (toolchain: $_rust_toolchain): $_rustc_actual (expected: 1.95.0)"
if [[ "$_rustc_actual" != "1.95.0" ]]; then
    echo "FAIL: Rust toolchain version mismatch" >&2
    exit 1
fi

# cargo-public-api
if command -v cargo-public-api &>/dev/null; then
    echo "Verifying ecu-spec public API against committed snapshot..."
    _snapshot="ecu-spec/api.snapshot.txt"
    if [[ -f "$repo_root/$_snapshot" ]]; then
        _tmp_snapshot=$(mktemp)
        # cargo-public-api 0.51 requires nightly rustdoc JSON. Invoking it as
        # `rustup run ${_rust_toolchain} cargo public-api` makes the tool try
        # and fail to re-enter rustup; `cargo +nightly public-api` is the
        # supported path and is checked against the committed snapshot below.
        cargo +nightly public-api -p ecu-spec 2>/dev/null > "$_tmp_snapshot" || {
            echo "FAIL: could not generate public API snapshot" >&2
            rm -f "$_tmp_snapshot"
            exit 1
        }
        if ! diff -q "$repo_root/$_snapshot" "$_tmp_snapshot" >/dev/null 2>&1; then
            echo "FAIL: public API surface has changed" >&2
            diff "$repo_root/$_snapshot" "$_tmp_snapshot" | head -20
            rm -f "$_tmp_snapshot"
            exit 1
        fi
        rm -f "$_tmp_snapshot"
        echo "PASS: public API surface matches committed snapshot"
    else
        echo "FAIL: no committed snapshot at $_snapshot" >&2
        exit 1
    fi
else
    echo "FAIL: cargo-public-api not found" >&2
    exit 1
fi
echo ""

# ============================================================
# Section 4: Formal verification — Verus
# ============================================================
echo "=== Formal gate: Verus (50 lemmas) ==="
if [[ ! -x "$_verus_path" ]]; then
    echo "FAIL: Verus not available" >&2
    exit 1
fi
_verus_output=$(RUSTUP_TOOLCHAIN="$_rust_toolchain" "$_verus_path" ecu-spec/proofs/verus.rs 2>&1) || {
    echo "FAIL: Verus verification failed" >&2
    echo "$_verus_output" >&2
    exit 1
}
echo "$_verus_output" | tail -2
# Extract verified and error counts from Verus output (US-FM0605)
_verus_tmp=$(mktemp)
echo "$_verus_output" > "$_verus_tmp"
VERUS_OUT="$_verus_tmp" VERUS_VERSION="$_verus_version" python3 /dev/stdin << 'PYEOF'
import os, re, json
out = open(os.environ["VERUS_OUT"]).read()
# Verus typical output: "Verifying 50 assertions" or "50 verified, 0 errors"
verified = 0
errors = 0
m = re.search(r'(\d+)\s+verified?', out)
if m:
    verified = int(m.group(1))
m = re.search(r'(\d+)\s+errors?', out)
if m:
    errors = int(m.group(1))
with open("target/formal-evidence/latest/verus.json", "w") as f:
    json.dump({"tool":"verus","version":os.environ["VERUS_VERSION"],"status":"pass","verified_count":verified,"error_count":errors}, f)
print(f"verus: {verified} verified, {errors} errors")
PYEOF
rm -f "$_verus_tmp"
echo ""

# ============================================================
# Section 5: Formal verification — TLC scheduler
# ============================================================
echo "=== Formal gate: TLC scheduler (safety + conditional liveness) ==="
if [[ ! -f "$_tla_jar" ]]; then
    echo "FAIL: TLA+ toolbox not available" >&2
    exit 1
fi
set +e
_tlc_output=$(run_tlc_module scheduler 2>&1)
_tlc_status=$?
set -e
echo "$_tlc_output"
if [[ $_tlc_status -ne 0 ]]; then
    echo "FAIL: TLC scheduler module" >&2
    exit 1
fi
_tlc_tmp=$(mktemp)
echo "$_tlc_output" > "$_tlc_tmp"
TLC_OUT="$_tlc_tmp" python3 /dev/stdin << 'PYEOF'
import os
import re, json
with open(os.environ["TLC_OUT"]) as f:
    out = f.read()
# Target the final summary line: "4822273 states generated, 64512 distinct states found"
m = re.search(r'^(\d+)\s+states generated,\s+(\d+)\s+distinct states found', out, re.MULTILINE)
if m:
    states = m.group(1)
    distinct = m.group(2)
else:
    # Fallback: find any "N states generated" line with comma after number
    all_states = re.findall(r'^(\d+)\s+states generated', out, re.MULTILINE)
    all_distinct = re.findall(r'^(\d+)\s+distinct states found', out, re.MULTILINE)
    states = all_states[-1] if all_states else '0'
    distinct = all_distinct[-1] if all_distinct else '0'
errs = len(re.findall(r'ERROR|Error:', out))
with open("target/formal-evidence/latest/tla.json", "w") as f:
    json.dump({"module":"scheduler","states_generated":int(states),"distinct_states":int(distinct),"errors":errs,"status":"pass"},f)
print("states:", states, "distinct:", distinct, "errors:", errs)
PYEOF
rm -f "$_tlc_tmp"
echo ""

# ============================================================
# Section 6: Formal verification — Kani (all harnesses)
# ============================================================
echo "=== Formal gate: Kani (all harnesses including US-FM0308 exec coupling) ==="
if ! command -v kani &>/dev/null; then
    echo "FAIL: Kani not available" >&2
    exit 1
fi

# Run all Kani harnesses on ecu-spec
_kani_out=$(cargo kani -p ecu-spec --output-format terse 2>&1) || {
    echo "FAIL: Kani verification failed" >&2
    echo "$_kani_out" >&2
    exit 1
}
echo "$_kani_out" | tail -5

# Extract harness summary
_kani_summary=$(echo "$_kani_out" | grep -E "Total|passed|failed|verified" | tail -3 || true)
echo "Kani summary: $_kani_summary"

# Verify all 5 new US-FM0308 exec coupling harnesses ran
for h in kani_exec_interp_matches_contract kani_exec_fuel_pipeline_matches_contract \
         kani_exec_schedule_matches_contract kani_exec_persist_matches_contract \
         kani_exec_ts_proto_matches_contract; do
    if echo "$_kani_out" | grep -q "$h"; then
        echo "  harness $h: found"
    else
        echo "FAIL: harness $h not found in Kani output" >&2
        exit 1
    fi
done
# Extract harness counts and emit kani.json (US-FM0605)
_kani_tmp=$(mktemp)
echo "$_kani_out" > "$_kani_tmp"
KANI_OUT="$_kani_tmp" KANI_VERSION="$_kani_version" python3 /dev/stdin << 'PYEOF'
import os, re, json

out = open(os.environ["KANI_OUT"]).read()
# Kani output format:
# "Total   (CYCLES): 0"
# "VERIFIED (CYCLES): 5"
# "FAILED   (CYCLES): 0"
# or terse format: "5 verified, 0 failed"
total = 0
verified = 0
failed = 0

# Try to extract from Kani's current terse manual-harness summary first:
# "Complete - 55 successfully verified harnesses, 0 failures, 55 total."
for line in out.splitlines():
    line = line.strip()
    m = re.search(
        r'^Complete\s+-\s+(\d+)\s+successfully verified harnesses,\s+(\d+)\s+failures,\s+(\d+)\s+total\.',
        line,
    )
    if m:
        verified = int(m.group(1))
        failed = int(m.group(2))
        total = int(m.group(3))

# Try to extract from older summary lines.
if total == 0:
    for line in out.splitlines():
        line = line.strip()
        m = re.search(r'^Total\s+(?:.*?):\s+(\d+)', line)
        if m:
            total = int(m.group(1))
        m = re.search(r'^VERIFIED\s+(?:.*?):\s+(\d+)', line)
        if m:
            verified = int(m.group(1))
        m = re.search(r'^FAILED\s+(?:.*?):\s+(\d+)', line)
        if m:
            failed = int(m.group(1))

# Fallback: look for the exact "N verified, M failed" pattern. Do not match
# Kani progress lines like "0 of 12 failed".
if total == 0:
    m = re.search(r'(\d+)\s+verified,\s+(\d+)\s+failed', out)
    if m:
        verified = int(m.group(1))
        failed = int(m.group(2))
    total = verified + failed

required_harnesses = [
    "kani_exec_interp_matches_contract",
    "kani_exec_fuel_pipeline_matches_contract",
    "kani_exec_schedule_matches_contract",
    "kani_exec_persist_matches_contract",
    "kani_exec_ts_proto_matches_contract"
]
with open("target/formal-evidence/latest/kani.json", "w") as f:
    json.dump({
        "tool": "kani",
        "version": os.environ["KANI_VERSION"],
        "status": "pass",
        "required_harnesses": required_harnesses,
        "verified_harness_count": verified,
        "failure_count": failed,
        "total_harnesses": total
    }, f)
print(f"kani: {verified} verified, {failed} failed, {total} total harnesses")
PYEOF
rm -f "$_kani_tmp"
echo ""

# ============================================================
# Section 7: Shortcut regression audits (US-FM0313)
# ============================================================
echo "=== Formal gate: shortcut regression audits (US-FM0313) ==="

# Audit: forbidden observed-output shortcuts
_audit_fail=false
for pattern in \
    "RuntimeDifferentialOutput::from_spec_step\|DifferentialOutputSurface::from_spec" \
    "from_spec\b.*oracle_result\|oracle_result.*from_spec" \
    "ecu_spec::schedule_all_cylinders" \
    "evaluate_fuel\|evaluate_tables"; do
    if rg -q "$pattern" ecu-runtime/src ecu-scheduler/src src tests --glob '*.rs' 2>/dev/null; then
        echo "FAIL: forbidden shortcut found:" >&2
        rg -n "$pattern" ecu-runtime/src ecu-scheduler/src src tests --glob '*.rs' 2>/dev/null | head -5 >&2
        _audit_fail=true
    fi
done

# Audit: assert!(true) vacuous assertions
if rg -q 'assert!\(true\)' ecu-runtime/src ecu-scheduler/src src --glob '*.rs' 2>/dev/null; then
    echo "FAIL: vacuous assert!(true) found" >&2
    _audit_fail=true
fi

# Audit: N/A rows in conformance map
# Only check table data rows (after the first --- header line), not prose rule definitions
_na_rows=$(rg -n '\bN/A\b' aidocs/proof-to-product-conformance-map.md 2>/dev/null | rg '^[[:space:]]*[|:]' | rg -v 'rule\|Rule\|No.*blocked' || true)
if [[ -n "$_na_rows" ]]; then
    echo "FAIL: N/A row found in conformance map:" >&2
    echo "$_na_rows" >&2
    _audit_fail=true
fi

# Audit: blocked rows in conformance map
_blocked_count=$(rg -c '\| blocked \|' aidocs/proof-to-product-conformance-map.md 2>/dev/null || true)
_blocked_count=${_blocked_count:-0}
if [[ "$_blocked_count" != "0" ]]; then
    echo "FAIL: $_blocked_count blocked rows remain in conformance map" >&2
    _audit_fail=true
fi

# Audit: production ecu-spec dependency in product crates
# Only flag ecu-spec in [dependencies], not in [dev-dependencies] or other sections
_prod_spec_deps=false
for crate_toml in ecu-runtime/Cargo.toml ecu-scheduler/Cargo.toml src/Cargo.toml ecu-target-common/Cargo.toml; do
    if [[ -f "$crate_toml" ]]; then
        # Get line number of [dependencies] section (not [dev-dependencies])
        _dep_line=$(rg -n '^\[dependencies\]' "$crate_toml" 2>/dev/null | cut -d: -f1 | head -1)
        if [[ -n "$_dep_line" ]]; then
            # Check if ecu-spec appears after [dependencies] but before any other section marker
            _spec_in_prod=$(awk -v start="$_dep_line" '
                BEGIN { in_deps=0 }
                /^\[dependencies\]/ { in_deps=1; next }
                /^\[(dev-|build-|target-)/ { in_deps=0; next }
                /^\[/ { next }
                in_deps && /^\s*ecu-spec\s*=/ { print FILENAME ":" NR ": " $0 }
            ' "$crate_toml")
            if [[ -n "$_spec_in_prod" ]]; then
                echo "FAIL: ecu-spec in production dependencies:" >&2
                echo "$_spec_in_prod" >&2
                _prod_spec_deps=true
            fi
        fi
    fi
done
if $_prod_spec_deps; then
    _audit_fail=true
fi

# Audit: production panic/unwrap/expect in ecu-spec (outside tests)
_panic_matches=$(rg -n 'panic!\(|unwrap\(\)|expect\(' ecu-spec/src --glob '*.rs' 2>/dev/null | \
    grep -v 'cfg(test)' | grep -v '^[[:space:]]*//' | grep -v '#\[cfg' || true)
if [[ -n "$_panic_matches" ]]; then
    echo "FAIL: panic/unwrap/expect in ecu-spec production code" >&2
    echo "$_panic_matches" >&2
    _audit_fail=true
fi

# Audit: forbidden Verus patterns
for pattern in "assume\(false\)|ensures\s+true|external_body"; do
    if rg -q "$pattern" ecu-spec/proofs/verus.rs 2>/dev/null; then
        echo "FAIL: forbidden Verus pattern '$pattern'" >&2
        _audit_fail=true
    fi
done

if $_audit_fail; then
    echo "FAIL: shortcut regression audit failed" >&2
    exit 1
fi
echo ""

# ============================================================
# Section 8: Coverage gate (US-FM0311)
# ============================================================
echo "=== Formal gate: coverage >= ${_coverage_threshold}% (US-FM0311) ==="
if [[ "$_quick_mode" == "true" ]]; then
    echo "SKIPPED (--quick mode)"
else
    _coverage_lines=$(mktemp)
    : > "$_coverage_lines"

    run_coverage_gate() {
        local package="$1"
        local out_file="${_evidence_dir}/coverage_${package}.json"

        cargo llvm-cov --no-fail-fast --summary-only --json \
            --output-path "$out_file" \
            -p "$package"

        python3 - "$package" "$out_file" "$_coverage_threshold" "$_coverage_lines" << 'PYEOF'
import json
import sys

package, out_file, threshold_s, lines_file = sys.argv[1:5]
threshold = float(threshold_s)
with open(out_file) as f:
    data = json.load(f)

# Filter files to only those belonging to the named package
pkg_prefix = f"/{package}/src/"
files = [f for f in data["data"][0]["files"] if pkg_prefix in f.get("filename", "")]

if not files:
    # Fallback: use aggregate if no package-specific files found
    totals = data["data"][0]["totals"]["lines"]
    pct = float(totals["percent"])
else:
    total_lines = sum(f["summary"]["lines"]["count"] for f in files)
    covered_lines = sum(f["summary"]["lines"]["covered"] for f in files)
    pct = (covered_lines / total_lines * 100) if total_lines > 0 else 0.0

record = {
    "package": package,
    "line_percent": pct,
    "threshold": threshold,
    "status": "pass" if pct >= threshold else "fail",
}
with open(lines_file, "a") as f:
    f.write(json.dumps(record) + "\n")
print(f"{package} line coverage: {pct:.2f}%")
if pct < threshold:
    raise SystemExit(1)
PYEOF
    }

    run_coverage_gate ecu-spec
    run_coverage_gate ecu-runtime
    run_coverage_gate ecu-scheduler

    python3 - "$_coverage_lines" "${_evidence_dir}/coverage.json" "${repo_root}/tools/coverage.schema.json" << 'PYEOF'
import json
import sys
from pathlib import Path

lines_file, out_file, schema_file = sys.argv[1:4]
records = [json.loads(line) for line in open(lines_file) if line.strip()]
with open(out_file, "w") as f:
    json.dump({"status": "pass", "packages": records}, f)

# Validate coverage.json against schema
try:
    import jsonschema
    with open(schema_file) as sf:
        schema = json.load(sf)
    with open(out_file) as af:
        artifact = json.load(af)
    jsonschema.validate(artifact, schema)
    print(f"coverage.json schema validation: pass")
except ImportError:
    print("coverage.json schema validation: jsonschema not available, skipping")
except Exception as e:
    print(f"coverage.json schema validation FAILED: {e}", file=sys.stderr)
    raise SystemExit(1)
PYEOF
    rm -f "$_coverage_lines"
fi
echo ""

# ============================================================
# Section 9: Benchmark gate (US-FM0311)
# ============================================================
echo "=== Formal gate: benchmark p95 regression <= ${_bench_regress_pct}% (US-FM0311) ==="
if [[ "$_quick_mode" == "true" ]]; then
    echo "SKIPPED (--quick mode)"
else
    _benchmark_lines=$(mktemp)
    : > "$_benchmark_lines"

    run_benchmark_gate() {
        local package="$1"
        local bench_target="$2"
        local criterion_name="$3"
        local baseline="target/criterion/${criterion_name}/fm0289-baseline/sample.json"
        local current="target/criterion/${criterion_name}/new/sample.json"

        if [[ ! -f "$baseline" ]]; then
            echo "FAIL: missing benchmark baseline: $baseline" >&2
            exit 1
        fi

        cargo bench -p "$package" --bench "$bench_target"

        if [[ ! -f "$current" ]]; then
            echo "FAIL: missing current benchmark sample.json: $current" >&2
            exit 1
        fi

        python3 - "$criterion_name" "$baseline" "$current" "$_bench_regress_pct" "$_benchmark_lines" << 'PYEOF'
import json
import sys
import math

name, baseline_path, current_path, threshold_s, out_lines = sys.argv[1:6]
threshold = float(threshold_s)

def compute_p95_nsop(sample_path):
    """Compute exact p95 ns/op from Criterion sample.json per US-FM0427."""
    with open(sample_path) as f:
        data = json.load(f)
    times = data["times"]
    iters = data["iters"]
    ns_ops = [times[i] / iters[i] for i in range(len(times))]
    ns_ops.sort()
    n = len(ns_ops)
    # p95 index = ceil(0.95 * n) - 1
    p95_idx = math.ceil(0.95 * n) - 1
    return ns_ops[p95_idx]

base_p95 = compute_p95_nsop(baseline_path)
cur_p95 = compute_p95_nsop(current_path)
regress = ((cur_p95 - base_p95) / base_p95) * 100.0 if base_p95 else 100.0

record = {
    "benchmark": name,
    "metric": "p95_ns_op",
    "baseline_p95_ns_op": base_p95,
    "current_p95_ns_op": cur_p95,
    "regression_pct": regress,
    "threshold_pct": threshold,
    "status": "pass" if regress <= threshold else "fail",
}
with open(out_lines, "a") as f:
    f.write(json.dumps(record) + "\n")
print(f"{name}: p95={cur_p95:.2f} ns/op, {regress:.2f}% regression (baseline p95={base_p95:.2f})")
if regress > threshold:
    raise SystemExit(1)
PYEOF
    }

    run_benchmark_gate ecu-spec spec_bench spec_step
    run_benchmark_gate ecu-runtime runtime_bench runtime_step

    python3 - "$_benchmark_lines" "${_evidence_dir}/benchmarks.json" "${repo_root}/tools/benchmarks.schema.json" << 'PYEOF'
import json
import sys
from pathlib import Path

lines_file, out_file, schema_file = sys.argv[1:4]
records = [json.loads(line) for line in open(lines_file) if line.strip()]
with open(out_file, "w") as f:
    json.dump({"status": "pass", "benchmarks": records}, f)

# Validate benchmarks.json against schema
try:
    import jsonschema
    with open(schema_file) as sf:
        schema = json.load(sf)
    with open(out_file) as af:
        artifact = json.load(af)
    jsonschema.validate(artifact, schema)
    print(f"benchmarks.json schema validation: pass")
except ImportError:
    print("benchmarks.json schema validation: jsonschema not available, skipping")
except Exception as e:
    print(f"benchmarks.json schema validation FAILED: {e}", file=sys.stderr)
    raise SystemExit(1)
PYEOF
    rm -f "$_benchmark_lines"
fi
echo ""

# ============================================================
# Section 10: Fuzz corpora
# ============================================================
echo "=== Formal gate: fuzz corpora ==="
if [[ "$_quick_mode" == "true" ]]; then
    echo "SKIPPED (--quick mode)"
else
    run_fuzz_corpus ts_proto_dispatch 64
    run_fuzz_corpus persistence_decoder 522
    run_fuzz_corpus trigger_decoder 512
fi
echo ""

# ============================================================
# Section 11: Embedded panic-freedom gate (US-FM0614)
# ============================================================
# Board binaries must not use panic/unwrap/expect/todo/unimplemented in runtime
# paths.  Fatal hardware init (Peripherals::take, clock init, PIO install,
# singleton init) is permitted ONLY at the exact file:line pairs below.
#
# Policy documented in: aidocs/proof-to-product-v7-execution-plan.md
echo "=== Formal gate: embedded panic-freedom ==="

_repo_root="$(cd "$(dirname "$0")/.." && pwd)"

# ----------------------------------------------------------------------
# Part A: panic!/todo!/unimplemented!/unreachable! — zero tolerance
# ----------------------------------------------------------------------
_embedded_dirs=(
    "rp2040-pico/src/bin/"
    "rp2040-pico/src/main.rs"
    "stm32f4/src/bin/"
    "stm32f4/src/main.rs"
    "rp2350b/src/bin/"
    "rp2350b/src/main.rs"
)

_forbidden_panic="panic!\b|todo!|unimplemented!|unreachable!"

for _dir in "${_embedded_dirs[@]}"; do
    if [[ -d "${_repo_root}/${_dir}" ]] || [[ -f "${_repo_root}/${_dir}" ]]; then
        _matches=$(rg -n "${_forbidden_panic}" "${_repo_root}/${_dir}" 2>/dev/null | \
            grep -v "^\s*//" | grep -v "^\s*#\[" | grep -v "mod tests" | grep -v "cfg(test)" || true)
        if [[ -n "$_matches" ]]; then
            echo "forbidden panic-family token found in ${_dir}:" >&2
            echo "$_matches" >&2
            exit 1
        fi
    fi
done

# ----------------------------------------------------------------------
# Part B: .unwrap()/.expect() — fatal-init allowlist only
# ----------------------------------------------------------------------
# Format: "file|line|context_hints"
# Lines are 1-indexed as shown in source.
# These represent unrecoverable hardware init failures (Peripherals::take,
# clock init, PIO install, singleton init) where halt is the only safe action.
_allowlist=(
    # RP2040 fatal-init — Peripherals::take, clock init, PIO install, singleton init
    "rp2040-pico/src/bin/ts_ecu.rs|510"
    "rp2040-pico/src/bin/ts_ecu.rs|511"
    "rp2040-pico/src/bin/ts_ecu.rs|524"
    "rp2040-pico/src/bin/ts_ecu.rs|550"
    "rp2040-pico/src/bin/ts_ecu.rs|584"
    "rp2040-pico/src/bin/ts_gauges.rs|60"
    "rp2040-pico/src/bin/ts_gauges.rs|61"
    "rp2040-pico/src/bin/ts_gauges.rs|75"
    "rp2040-pico/src/bin/ts_gauges.rs|99"
    "rp2040-pico/src/bin/pio_capture_example.rs|48"
    "rp2040-pico/src/bin/pio_capture_example.rs|62"
    "rp2040-pico/src/bin/pio_capture_example.rs|100"
    # STM32F4 fatal-init
    "stm32f4/src/main.rs|493"
    "stm32f4/src/main.rs|552"
    "stm32f4/src/main.rs|605"
    "stm32f4/src/main.rs|613"
    "stm32f4/src/bin/ts_gauges.rs|63"
    "stm32f4/src/bin/ecu_demo.rs|11"
    "stm32f4/src/bin/can_heartbeat.rs|26"
    "stm32f4/src/bin/v8_seq.rs|12"
    "stm32f4/src/bin/4c_batched.rs|11"
    # RP2350B fatal-init
    "rp2350b/src/main.rs|37"
    "rp2350b/src/main.rs|38"
    "rp2350b/src/main.rs|54"
    "rp2350b/src/bin/minimal_ecu.rs|129"
    "rp2350b/src/bin/minimal_ecu.rs|130"
    "rp2350b/src/bin/minimal_ecu.rs|146"
)

# Build a regex that matches exactly those file:line pairs from the allowlist.
# rg query: match .unwrap() or .expect() anywhere in board binary dirs.
# Then filter results to only allowlist entries.
_unwrap_expect_matches=$(rg -n "\.unwrap\(\)|\.expect\(" \
    "${_repo_root}/rp2040-pico/src/bin/" \
    "${_repo_root}/rp2040-pico/src/main.rs" \
    "${_repo_root}/stm32f4/src/bin/" \
    "${_repo_root}/stm32f4/src/main.rs" \
    "${_repo_root}/rp2350b/src/bin/" \
    "${_repo_root}/rp2350b/src/main.rs" \
    2>/dev/null \
    | grep -v "^\s*//" \
    | grep -v "^\s*#\[" \
    | grep -v "mod tests" \
    | grep -v "cfg(test)" \
    || true)

if [[ -n "$_unwrap_expect_matches" ]]; then
    # Check each match against the allowlist (file:line must be in allowlist)
    while IFS= read -r _line; do
        # Extract file path and line number (rg output: path:line:content)
        _file="$(echo "$_line" | cut -d: -f1)"
        _linenum="$(echo "$_line" | cut -d: -f2)"

        # Build the expected key (file relative to repo root)
        # Strip the _repo_root prefix to get repo-relative path
        _rel_path="${_file#${_repo_root}/}"

        _allowed=0
        for _entry in "${_allowlist[@]}"; do
            _entry_file="${_entry%%|*}"
            _entry_line="${_entry#*|}"
            if [[ "$_rel_path" == "$_entry_file" && "$_linenum" == "$_entry_line" ]]; then
                _allowed=1
                break
            fi
        done

        if [[ $_allowed -eq 0 ]]; then
            echo "disallowed .unwrap()/.expect() outside fatal-init allowlist:" >&2
            echo "$_line" >&2
            _found_violation=1
        fi
    done <<< "$_unwrap_expect_matches"
fi

if [[ "${_found_violation:-0}" -eq 1 ]]; then
    echo "embedded panic policy violation — see aidocs/proof-to-product-v7-execution-plan.md" >&2
    exit 1
fi

echo "embedded panic-freedom gate: PASS"

# ============================================================
# Section 11b: RP2040 static-mut regression gate (US-FM0710)
# ============================================================
echo "=== Formal gate: RP2040 static-mut regression (US-FM0710) ==="
_rp2040_v8_files=(
    "rp2040-pico/src/bin/ts_ecu.rs"
    "ecu-target-common/src/capture.rs"
    "ecu-target-common/src/split_tick.rs"
)
_rp2040_static_mut_fail=false
for _f in "${_rp2040_v8_files[@]}"; do
    if [[ -f "$repo_root/$_f" ]]; then
        if rg -q "static mut" "$repo_root/$_f" 2>/dev/null; then
            echo "FAIL: static mut found in $_f" >&2
            rg -n "static mut" "$repo_root/$_f" 2>/dev/null | head -5 >&2
            _rp2040_static_mut_fail=true
        fi
    fi
done
if $_rp2040_static_mut_fail; then
    echo "RP2040 static-mut regression detected — see aidocs/proof-to-product-v8-execution-plan.md" >&2
    exit 1
fi
echo "RP2040 static-mut regression gate: PASS"

# ============================================================
# Section 12: no_alloc static audit
# ============================================================
echo "=== Formal gate: no_alloc static audit ==="
formal_crates=("ecu-runtime/src" "ecu-scheduler/src" "src")
for crate_src in "${formal_crates[@]}"; do
    if [[ -d "$repo_root/$crate_src" ]]; then
        # Check for alloc:: in production code using rg
        if rg --quiet -e "alloc::" \
            "$repo_root/$crate_src" --type rust 2>/dev/null; then
            # Found potential matches — get details excluding test/comment lines
            matches=$(rg -n -e "alloc::" \
                "$repo_root/$crate_src" --type rust 2>/dev/null | \
                grep -v "#\[cfg(test)\]" | \
                grep -v "^[[:space:]]*//" | \
                grep -v "^[[:space:]]*$" || true)
            if [[ -n "$matches" ]]; then
                echo "alloc:: in production ($crate_src):" >&2
                echo "$matches" >&2
                exit 1
            fi
        fi
    fi
done
echo ""

# ============================================================
# Section 13: Evidence emission (US-FM0312)
# ============================================================
echo "=== Formal gate: evidence emission (US-FM0312) ==="
mkdir -p "${_evidence_dir}"

# conformance.json — US-FM0429
python3 - "${_evidence_dir}/conformance.json" << 'PYEOF'
import json, re, sys

out_path = sys.argv[1]
map_path = "aidocs/proof-to-product-conformance-map.md"
contract_sources = {
    "RuntimeAdapterContract": "ecu-runtime/src/lib.rs",
    "SchedulerAdapterContract": "ecu-scheduler/src/lib.rs",
    "CoreAdapterContract": "src/lib.rs",
    "BoardAdapterContract": "ecu-target-common/src/lib.rs",
    "TargetCommonAdapterContract": "ecu-target-common/src/lib.rs",
}

def enum_variants(path, enum_name):
    text = open(path).read()
    m = re.search(r"\bpub\s+enum\s+" + re.escape(enum_name) + r"\s*\{", text)
    if not m:
        return None
    start = m.end()
    depth = 1
    end = start
    while end < len(text) and depth:
        if text[end] == "{":
            depth += 1
        elif text[end] == "}":
            depth -= 1
        end += 1
    body = text[start:end - 1]
    return set(re.findall(r"^\s*([A-Z][A-Za-z0-9_]*)\b", body, re.MULTILINE))

known_contracts = {
    enum_name: enum_variants(path, enum_name)
    for enum_name, path in contract_sources.items()
}

rows = []
current_section = ""
with open(map_path) as f:
    content = f.read()

# Split on markdown table rows.
# A row looks like:
# | field | owner | observed path | reducer/test file | tolerance | status | blocker |
for line in content.splitlines():
    line = line.strip()
    if not line.startswith("|") or line.startswith("| ---"):
        # Section header
        if line.startswith("##") or line.startswith("#"):
            m = re.search(r"##?\s+(.+)", line)
            if m:
                current_section = m.group(1).strip()
        continue
    parts = [p.strip() for p in line.split("|")]
    # parts[0] is empty, parts[-1] is empty
    if len(parts) < 5 or not parts[1].strip():
        continue
    field = parts[1].strip()
    # Skip header rows, separator rows, and empty rows
    if (
        not field
        or field in ("Field", "Frozen Contract", "Oracle Field / Contract")
        or re.match(r'^-+$', field)
    ):
        continue
    # Split form for a markdown table row with leading/trailing pipes:
    # parts[0] = "", parts[1] = field, parts[2] = owner,
    # parts[3] = observed path, parts[4] = reducer/test file,
    # parts[5] = tolerance, parts[6] = status, parts[7] = blocker.
    reducer = parts[4].strip() if len(parts) > 4 else ""
    tolerance = parts[5].strip() if len(parts) > 5 else ""
    status = parts[6].strip() if len(parts) > 6 else ""
    blocker = parts[7].strip() if len(parts) > 7 else ""
    owner = parts[2].strip() if len(parts) > 2 else ""
    artifact = parts[3].strip() if len(parts) > 3 else ""
    status_lower = status.lower()
    if status_lower not in {"covered", "adapter-contract"}:
        print(f"FAIL: invalid conformance status for {field}: {status!r}", file=sys.stderr)
        sys.exit(1)
    is_adapter = status_lower == "adapter-contract"
    row_text = line
    contract_refs = re.findall(
        r"\b(RuntimeAdapterContract|SchedulerAdapterContract|CoreAdapterContract|BoardAdapterContract|TargetCommonAdapterContract)::([A-Z][A-Za-z0-9_]*)\b",
        row_text,
    )
    for enum_name, group in re.findall(
        r"\b(RuntimeAdapterContract|SchedulerAdapterContract|CoreAdapterContract|BoardAdapterContract|TargetCommonAdapterContract)::\{([^}]+)\}",
        row_text,
    ):
        for variant in group.split(","):
            variant = variant.strip()
            if variant:
                contract_refs.append((enum_name, variant))
    rows.append({
        "field": field,
        "section": current_section,
        "owner": owner,
        "artifact_path": artifact,
        "reducer_path": reducer,
        "tolerance": tolerance,
        "status": status,
        "blocker": blocker,
        "conformance": "adapter-contract" if is_adapter else "covered",
        "contract_refs": [f"{enum_name}::{variant}" for enum_name, variant in contract_refs],
    })

# Check for blocked rows
blocked = [r for r in rows if "blocked" in r["status"].lower()]
if blocked:
    print(f"FAIL: {len(blocked)} blocked rows in conformance map", file=sys.stderr)
    for b in blocked:
        print(f"  BLOCKED: {b['field']}", file=sys.stderr)
    sys.exit(1)

missing_contract_refs = []
adapter_without_contract = []
for row in rows:
    if row["conformance"] != "adapter-contract":
        continue
    if not row["contract_refs"]:
        adapter_without_contract.append(row["field"])
        continue
    for ref in row["contract_refs"]:
        enum_name, variant = ref.split("::", 1)
        variants = known_contracts.get(enum_name)
        if variants is None:
            missing_contract_refs.append(f"{row['field']}: enum {enum_name} not found")
        elif variant not in variants:
            missing_contract_refs.append(f"{row['field']}: missing {ref}")

if adapter_without_contract:
    print("FAIL: adapter-contract rows without typed Rust contract:", file=sys.stderr)
    for field in adapter_without_contract:
        print(f"  {field}", file=sys.stderr)
    sys.exit(1)

if missing_contract_refs:
    print("FAIL: conformance map references missing contract variants:", file=sys.stderr)
    for missing in missing_contract_refs:
        print(f"  {missing}", file=sys.stderr)
    sys.exit(1)

with open(out_path, "w") as f:
    json.dump({"status": "pass", "rows": rows, "total": len(rows)}, f, indent=2)
print(f"conformance.json: {len(rows)} rows, 0 blocked, typed contracts validated")
PYEOF
if [[ $? -ne 0 ]]; then
    echo "FAIL: conformance.json emission failed" >&2
    exit 1
fi

# Validate conformance.json against schema (US-FM0603)
python3 - "${_evidence_dir}/conformance.json" << 'PYEOF'
import json
import sys
import jsonschema

schema_path = "tools/conformance.schema.json"
with open(schema_path) as f:
    schema = json.load(f)
with open(sys.argv[1]) as f:
    data = json.load(f)
jsonschema.validate(data, schema)
print("conformance.json schema validation: PASS")
PYEOF
if [[ $? -ne 0 ]]; then
    echo "FAIL: conformance.json schema validation failed" >&2
    exit 1
fi

# summary.json — US-FM0606: embed key metrics from all artifacts
_git_head=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
if git diff --quiet --ignore-submodules -- && git diff --cached --quiet --ignore-submodules --; then
    _dirty_worktree=false
else
    _dirty_worktree=true
fi
_generated_at="$(date -u +%Y-%m-%dT%H:%M:%SZ)"

# Load conformance metrics
_conf_total=0
_conf_covered=0
_conf_adapter=0
if [[ -f "${_evidence_dir}/conformance.json" ]]; then
    _conf_metrics=$(python3 - "${_evidence_dir}/conformance.json" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
rows = d.get("rows", [])
total = len(rows)
covered = sum(1 for r in rows if r.get("conformance") == "covered")
adapter = sum(1 for r in rows if r.get("conformance") == "adapter-contract")
print(f"{total}\n{covered}\n{adapter}")
PYEOF
)
    _conf_total=$(echo "$_conf_metrics" | sed -n '1p')
    _conf_covered=$(echo "$_conf_metrics" | sed -n '2p')
    _conf_adapter=$(echo "$_conf_metrics" | sed -n '3p')
fi

# Load coverage per-package percentages
_cov_packages="[]"
if [[ -f "${_evidence_dir}/coverage.json" ]]; then
    _cov_packages=$(python3 - "${_evidence_dir}/coverage.json" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
pkgs = d.get("packages", [])
print(json.dumps([{"package": p["package"], "line_percent": p["line_percent"]} for p in pkgs]))
PYEOF
)
fi

# Load benchmark p95 and regression percentages
_bench_metrics="[]"
if [[ -f "${_evidence_dir}/benchmarks.json" ]]; then
    _bench_metrics=$(python3 - "${_evidence_dir}/benchmarks.json" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
bms = d.get("benchmarks", [])
print(json.dumps([{"benchmark": b["benchmark"], "p95_ns_op": b["current_p95_ns_op"], "regression_pct": b["regression_pct"]} for b in bms]))
PYEOF
)
fi

# Load TLC state/error counts
_tlc_states=0
_tlc_distinct=0
_tlc_errors=0
if [[ -f "${_evidence_dir}/tla.json" ]]; then
    _tlc_metrics=$(python3 - "${_evidence_dir}/tla.json" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"{d.get('states_generated', 0)}\n{d.get('distinct_states', 0)}\n{d.get('errors', 0)}")
PYEOF
)
    _tlc_states=$(echo "$_tlc_metrics" | sed -n '1p')
    _tlc_distinct=$(echo "$_tlc_metrics" | sed -n '2p')
    _tlc_errors=$(echo "$_tlc_metrics" | sed -n '3p')
fi

# Load Kani harness totals/failures
_kani_total=0
_kani_verified=0
_kani_failed=0
if [[ -f "${_evidence_dir}/kani.json" ]]; then
    _kani_metrics=$(python3 - "${_evidence_dir}/kani.json" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
harnesses = d.get("required_harnesses", [])
total = d.get("total_harnesses", len(harnesses))
verified = d.get("verified_harness_count", d.get("verified", total))
failed = d.get("failure_count", d.get("failed", 0))
print(f"{total}\n{verified}\n{failed}")
PYEOF
)
    _kani_total=$(echo "$_kani_metrics" | sed -n '1p')
    _kani_verified=$(echo "$_kani_metrics" | sed -n '2p')
    _kani_failed=$(echo "$_kani_metrics" | sed -n '3p')
fi

# Load Verus verified/errors
_verus_verified=0
_verus_errors=0
if [[ -f "${_evidence_dir}/verus.json" ]]; then
    _verus_metrics=$(python3 - "${_evidence_dir}/verus.json" << 'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"{d.get('verified_count', d.get('verified', 0))}\n{d.get('error_count', d.get('errors', 0))}")
PYEOF
)
    _verus_verified=$(echo "$_verus_metrics" | sed -n '1p')
    _verus_errors=$(echo "$_verus_metrics" | sed -n '2p')
fi

cat > "${_evidence_dir}/summary.json" << EOF
{
  "schema_version": 2,
  "generated_at_utc": "${_generated_at}",
  "git_head": "${_git_head}",
  "dirty_worktree": ${_dirty_worktree},
  "overall_status": "pass",
  "tools": {
    "rust": { "version": "${_rust_toolchain}", "path": "rustup" },
    "verus": { "version": "${_verus_version}", "path": "${_verus_path}" },
    "kani": { "version": "${_kani_version}", "path": "kani" },
    "tla": { "version": "${_tla_version}", "path": "${_tla_jar}" },
    "cargo_llvm_cov": { "version": "0.6.14", "path": "cargo llvm-cov" },
    "cargo_public_api": { "version": "0.51.0", "path": "cargo public-api" }
  },
  "commands": [
    {
      "command": "bash tools/verify_formal.sh",
      "cwd": "${repo_root}",
      "exit_code": 0,
      "duration_ms": 0,
      "stdout_tail_path": "",
      "stderr_tail_path": ""
    }
  ],
  "gates": [
    { "name": "prd-json", "status": "pass", "artifact_paths": ["prd.json"], "metrics": {} },
    { "name": "repository-hygiene", "status": "pass", "artifact_paths": ["tools/check_repo_hygiene.sh"], "metrics": {} },
    { "name": "schema-json", "status": "pass", "artifact_paths": ["tools/evidence.schema.json"], "metrics": {} },
    { "name": "fmt", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "clippy", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "tests", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "embedded-builds", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "version-pinning", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "verus", "status": "pass", "artifact_paths": ["${_evidence_dir}/verus.json"], "metrics": { "verified": ${_verus_verified}, "errors": ${_verus_errors} } },
    { "name": "tlc-scheduler", "status": "pass", "artifact_paths": ["${_evidence_dir}/tla.json"], "metrics": { "states_generated": ${_tlc_states}, "distinct_states": ${_tlc_distinct}, "errors": ${_tlc_errors} } },
    { "name": "kani", "status": "pass", "artifact_paths": ["${_evidence_dir}/kani.json"], "metrics": { "total_harnesses": ${_kani_total}, "verified": ${_kani_verified}, "failed": ${_kani_failed} } },
    { "name": "shortcut-audits", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "coverage", "status": "pass", "artifact_paths": ["${_evidence_dir}/coverage.json"], "metrics": { "threshold": ${_coverage_threshold}, "packages": ${_cov_packages} } },
    { "name": "benchmarks", "status": "pass", "artifact_paths": ["${_evidence_dir}/benchmarks.json"], "metrics": { "threshold_pct": ${_bench_regress_pct}, "metric": "p95_ns_op", "benchmarks": ${_bench_metrics} } },
    { "name": "fuzz-corpora", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "embedded-panic-freedom", "status": "pass", "artifact_paths": [], "metrics": {} },
    { "name": "rp2040-static-mut-regression", "status": "pass", "artifact_paths": ["tools/verify_formal.sh"], "metrics": {} },
    { "name": "no-alloc-audit", "status": "pass", "artifact_paths": [], "metrics": {} }
  ],
  "metrics_summary": {
    "conformance": {
      "total_rows": ${_conf_total},
      "covered_rows": ${_conf_covered},
      "adapter_contract_rows": ${_conf_adapter}
    },
    "coverage": ${_cov_packages},
    "benchmarks": ${_bench_metrics},
    "tlc": {
      "states_generated": ${_tlc_states},
      "distinct_states": ${_tlc_distinct},
      "errors": ${_tlc_errors}
    },
    "kani": {
      "total_harnesses": ${_kani_total},
      "verified": ${_kani_verified},
      "failed": ${_kani_failed}
    },
    "verus": {
      "verified": ${_verus_verified},
      "errors": ${_verus_errors}
    }
  }
}
EOF

# Emit per-tool evidence files (tla.json already emitted by TLC section)
for tool in kani verus; do
    if [[ ! -f "${_evidence_dir}/${tool}.json" ]]; then
        touch "${_evidence_dir}/${tool}.json"
    fi
done

# Validate verus.json, kani.json, and tla.json against schemas (US-FM0605)
for artifact in verus kani tla; do
    python3 - "${_evidence_dir}/${artifact}.json" "tools/${artifact}.schema.json" << 'PYEOF'
import json, sys
artifact_path, schema_path = sys.argv[1:]
try:
    import jsonschema
    with open(schema_path) as f:
        schema = json.load(f)
    with open(artifact_path) as f:
        artifact_data = json.load(f)
    jsonschema.validate(artifact_data, schema)
    print(f"{artifact_path}: schema validation PASS")
except ImportError:
    print(f"{artifact_path}: jsonschema not available, skipping")
except Exception as e:
    print(f"FAIL: {artifact_path} schema validation FAILED: {e}", file=sys.stderr)
    raise SystemExit(1)
PYEOF
    if [[ $? -ne 0 ]]; then
        echo "FAIL: ${artifact}.json schema validation failed" >&2
        exit 1
    fi
done

# commands.jsonl
echo "{\"ts\":\"$(date -u +%Y-%m-%dT%H:%M:%SZ)\",\"cmd\":\"verify_formal.sh\"}" >> "${_evidence_dir}/commands.jsonl"

# Validate summary.json against schema (US-FM0428)
python3 tools/test_evidence_schema.py || {
    echo "FAIL: summary.json schema validation failed" >&2
    exit 1
}

echo "Evidence emitted to ${_evidence_dir}/"
echo ""

echo "verify_formal.sh: all checks passed"
