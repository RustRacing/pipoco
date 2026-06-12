#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

status=0

common_globs=(
    --hidden
    --glob '!aidocs/**'
    --glob '!target/**'
    --glob '!.git/**'
    --glob '!vendor/**'
    --glob '!reference/**'
    --glob '!references/**'
    --glob '!ref/**'
    --glob '!**/vendor/**'
    --glob '!**/reference/**'
    --glob '!**/references/**'
    --glob '!**/ref/**'
)

live_code_globs=(
    --glob '!**/tests/**'
    --glob '!tests/**'
    --glob '!**/benches/**'
    --glob '!**/examples/**'
    --glob '!**/fixtures/**'
    --glob '!**/fixture/**'
)

check_no_rg_hits() {
    local label="$1"
    local pattern="$2"
    shift 2

    local matches
    matches="$(rg -n "${common_globs[@]}" "$pattern" "$@" 2>/dev/null || true)"
    if [[ -n "$matches" ]]; then
        printf 'ERROR: %s\n%s\n' "$label" "$matches" >&2
        status=1
    else
        printf 'PASS: %s\n' "$label"
    fi
}

check_no_live_rg_hits() {
    local label="$1"
    local pattern="$2"
    shift 2

    local matches
    matches="$(rg -n "${common_globs[@]}" "${live_code_globs[@]}" "$pattern" "$@" 2>/dev/null || true)"
    if [[ -n "$matches" ]]; then
        printf 'ERROR: %s\n%s\n' "$label" "$matches" >&2
        status=1
    else
        printf 'PASS: %s\n' "$label"
    fi
}

check_board_common_compat_helper_names() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path(".")
names = ("run_split_scheduled_tick", "apply_trigger_timestamp")
name_patterns = {
    name: re.compile(rf"\b{re.escape(name)}\b") for name in names
}

skip_dirs = {
    ".git",
    "target",
    "aidocs",
    "vendor",
    "reference",
    "references",
    "ref",
}

def is_skipped(path: Path) -> bool:
    return any(part in skip_dirs for part in path.parts)

violations = []
for path in root.rglob("*.rs"):
    rel = path.relative_to(root)
    if is_skipped(rel):
        continue
    text = path.read_text(encoding="utf-8")
    for lineno, line in enumerate(text.splitlines(), 1):
        for name in names:
            if not name_patterns[name].search(line):
                continue
            violations.append(f"{rel}:{lineno}:{line}")

if violations:
    print(
        "ERROR: old board-common compatibility helper names must not reappear in live code or tests",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: PR2 old board-common helper names are absent from live code and tests")
PY
}

check_raw_ts_kv_boundary() {
    # Enforces the shared KV page layout (see adr-0008-kv-page-layout.md).
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path(".")
symbols = (
    "PERSIST_KEY_FUEL",
    "PERSIST_KEY_IGN",
    "PERSIST_KEY_ANGLES",
    "PERSIST_KEY_EXPERT_TRIGGER",
    "RamKv",
)
symbol_pattern = re.compile(r"\b(" + "|".join(re.escape(symbol) for symbol in symbols) + r")\b")

allowed_paths = {
    Path("crates/calibration/src/kv.rs"),          # transitional definitions
    Path("crates/ts/src/persistence.rs"),         # TS persistence owner
    Path("crates/ts/src/persistence_tests.rs"),   # tests for TS persistence owner
    Path("boards/common/src/kv/ram.rs"),          # target-common RAM KV compatibility backend
}

skip_parts = {
    ".git",
    "target",
    "aidocs",
    "vendor",
    "reference",
    "references",
    "ref",
    "tests",
    "benches",
    "examples",
    "fixtures",
    "fixture",
}


def is_skipped(path: Path) -> bool:
    return any(part in skip_parts for part in path.parts)


def is_in_cfg_test_block(lines: list[str], line_index: int) -> bool:
    for idx in range(line_index, -1, -1):
        if "mod tests" not in lines[idx]:
            continue
        prev = "\n".join(lines[max(0, idx - 3):idx])
        if "#[cfg(test)]" not in prev:
            continue

        depth = 0
        opened = False
        for block_idx in range(idx, line_index + 1):
            for char in lines[block_idx]:
                if char == "{":
                    depth += 1
                    opened = True
                elif char == "}":
                    depth -= 1
            if opened and depth <= 0 and block_idx < line_index:
                break
        else:
            return opened

    return False


violations = []
for path in root.rglob("*.rs"):
    rel = path.relative_to(root)
    if is_skipped(rel) or rel in allowed_paths:
        continue

    lines = path.read_text(encoding="utf-8").splitlines()
    for idx, line in enumerate(lines):
        if not symbol_pattern.search(line):
            continue
        if is_in_cfg_test_block(lines, idx):
            continue
        violations.append(f"{rel}:{idx + 1}:{line}")

if violations:
    print(
        "ERROR: raw TS KV compatibility symbols must stay in calibration::kv, ecu-ts persistence, or explicit compatibility backends",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: PR3 raw TS KV compatibility symbols stay out of generic/live surfaces")
PY
}

check_core_transport_compat_surface() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path(".")
forbidden_paths = (
    Path("crates/core/src/transport/mod.rs"),
    Path("crates/core/src/transport/bbq.rs"),
    Path("crates/core/src/transport/can.rs"),
    Path("crates/core/src/transport/message.rs"),
)
forbidden_patterns = (
    re.compile(r"\bpub\s+mod\s+transport\s*;"),
    re.compile(r"\bpub\s+use\s+transport::"),
    re.compile(r"\becu_core::transport\b"),
    re.compile(
        r"\becu_core::\{[^;\n]*(BbqTransport|CanDevice|CanTransport|Message|Transport|TransportError|TransportStats)"
    ),
)

violations = []
for rel in forbidden_paths:
    if (root / rel).exists():
        violations.append(f"{rel}: compatibility transport module must not exist")

for path in root.rglob("*.rs"):
    rel = path.relative_to(root)
    if any(part in {".git", "target", "aidocs", "vendor", "reference", "references", "ref"} for part in rel.parts):
        continue
    text = path.read_text(encoding="utf-8")
    for line_no, line in enumerate(text.splitlines(), 1):
        if any(pattern.search(line) for pattern in forbidden_patterns):
            violations.append(f"{rel}:{line_no}:{line}")

if violations:
    print(
        "ERROR: ecu-core must not reintroduce transport compatibility modules or reexports; use ecu-transport directly",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core transport compatibility surface is absent")
PY
}

check_core_sensor_calibration_compat_surface() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

path = Path("crates/core/src/sensors/mod.rs")
forbidden = (
    re.compile(r"\bpub\s+use\s+ecu_calibration::sensors\b"),
    re.compile(r"\bpub\s+use\s+ecu_calibration::sensors::\{"),
    re.compile(r"\bpub\s+mod\s+(convert|curve|model)\s*;"),
)
forbidden_names = re.compile(
    r"\b(CurveSensor|Quality|Sensor|SensorError|SensorsCal|ThermistorSensor)\b"
)

violations = []
if path.exists():
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if any(pattern.search(line) for pattern in forbidden) or (
            "pub use" in line and forbidden_names.search(line)
        ):
            violations.append(f"{path}:{line_no}:{line}")

if violations:
    print(
        "ERROR: ecu-core sensors must not re-export calibration/math types; use ecu-calibration directly",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core sensors expose only runtime sensor primitives")
PY
}

check_core_ts_shell_compat_surface() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path(".")
skip_parts = {".git", "target", "aidocs", "vendor", "reference", "references", "ref"}

forbidden_imports = (
    re.compile(r"\becu_core::ts::(outpc|proto|serial|server)\b"),
    re.compile(
        r"\buse\s+ecu_core::ts::\{[^;\n]*(NoPages|OutpcProvider|PageError|PageStore|PersistError|ServerStats|TunerstudioServer)"
    ),
    re.compile(
        r"\becu_core::ts::(NoPages|OutpcProvider|PageError|PageStore|PersistError|ServerStats|TunerstudioServer)\b"
    ),
)
forbidden_reexports = (
    re.compile(r"\bpub\s+use\s+ecu_ts::\{[^;\n]*(outpc|proto|serial|server)"),
    re.compile(
        r"\bpub\s+use\s+server::\{[^;\n]*(NoPages|OutpcProvider|PageError|PageStore|PersistError|ServerStats|TunerstudioServer)"
    ),
)

violations = []
ts_mod = Path("crates/core/src/ts/mod.rs")
if ts_mod.exists():
    for line_no, line in enumerate(ts_mod.read_text(encoding="utf-8").splitlines(), 1):
        if any(pattern.search(line) for pattern in forbidden_reexports):
            violations.append(f"{ts_mod}:{line_no}:{line}")

for path in root.rglob("*.rs"):
    rel = path.relative_to(root)
    if any(part in skip_parts for part in rel.parts):
        continue
    for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if any(pattern.search(line) for pattern in forbidden_imports):
            violations.append(f"{rel}:{line_no}:{line}")

if violations:
    print(
        "ERROR: ecu-core must not re-export or consume the generic TS shell; use ecu-ts directly and keep only ecu_core::ts::pages",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core TS shell compatibility surface is absent")
PY
}

check_core_actuator_page_store_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

roots = [Path("crates/core/src"), Path("crates/core/tests")]
forbidden = re.compile(
    r"\b(IdlePage|FanPage|ClosedLoopPage)::(new|decode)\b"
)

violations = []
for root in roots:
    if not root.exists():
        continue
    for path in root.rglob("*.rs"):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if forbidden.search(line):
                violations.append(f"{path}:{line_no}:{line}")

if violations:
    print(
        "ERROR: ecu-core must not construct/decode actuator TS wire DTOs directly; map config fields through ecu_ts::pages::ActuatorPageStore",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core actuator TS pages delegate wire DTO construction/decode to ActuatorPageStore")
PY
}

check_core_enrichment_page_store_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path("crates/core/src")
forbidden = re.compile(r"\b(AePage|DfcoPage|WuePage|AsePage)::(new|decode)\s*\(")


def strip_rust_comments(text: str) -> str:
    out = []
    i = 0
    block_depth = 0
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if block_depth:
            if ch == "/" and nxt == "*":
                block_depth += 1
                i += 2
                continue
            if ch == "*" and nxt == "/":
                block_depth -= 1
                i += 2
                continue
            out.append("\n" if ch == "\n" else " ")
            i += 1
            continue

        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if ch == "/" and nxt == "*":
            block_depth = 1
            out.extend("  ")
            i += 2
            continue

        out.append(ch)
        i += 1

    return "".join(out)


def is_in_cfg_test_block(lines: list[str], line_index: int) -> bool:
    for idx in range(line_index, -1, -1):
        if "mod tests" not in lines[idx]:
            continue
        prev = "\n".join(lines[max(0, idx - 3):idx])
        if "#[cfg(test)]" not in prev:
            continue

        depth = 0
        opened = False
        for block_idx in range(idx, line_index + 1):
            for char in lines[block_idx]:
                if char == "{":
                    depth += 1
                    opened = True
                elif char == "}":
                    depth -= 1
            if opened and depth <= 0 and block_idx < line_index:
                break
        else:
            return opened

    return False


violations = []
if root.exists():
    for path in root.rglob("*.rs"):
        if path.name.endswith("_tests.rs") or path.name == "tests.rs":
            continue
        text = path.read_text(encoding="utf-8")
        stripped_lines = strip_rust_comments(text).splitlines()
        original_lines = text.splitlines()
        for idx, line in enumerate(stripped_lines):
            if not forbidden.search(line):
                continue
            if is_in_cfg_test_block(stripped_lines, idx):
                continue
            original = original_lines[idx] if idx < len(original_lines) else line
            violations.append(f"{path}:{idx + 1}:{original}")

if violations:
    print(
        "ERROR: ecu-core must not construct/decode enrichment TS wire DTOs directly; map config fields through ecu_ts::pages::EnrichmentPageStore",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core enrichment TS pages delegate wire DTO construction/decode to EnrichmentPageStore")
PY
}

check_core_limits_page_store_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path("crates/core/src")
forbidden = re.compile(r"\bLimitsPage::(new|decode)\s*\(")


def strip_rust_comments(text: str) -> str:
    out = []
    i = 0
    block_depth = 0
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if block_depth:
            if ch == "/" and nxt == "*":
                block_depth += 1
                i += 2
                continue
            if ch == "*" and nxt == "/":
                block_depth -= 1
                i += 2
                continue
            out.append("\n" if ch == "\n" else " ")
            i += 1
            continue

        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if ch == "/" and nxt == "*":
            block_depth = 1
            out.extend("  ")
            i += 2
            continue

        out.append(ch)
        i += 1

    return "".join(out)


violations = []
if root.exists():
    for path in root.rglob("*.rs"):
        text = path.read_text(encoding="utf-8")
        stripped_lines = strip_rust_comments(text).splitlines()
        original_lines = text.splitlines()
        for idx, line in enumerate(stripped_lines):
            if not forbidden.search(line):
                continue
            original = original_lines[idx] if idx < len(original_lines) else line
            violations.append(f"{path}:{idx + 1}:{original}")

if violations:
    print(
        "ERROR: ecu-core must not construct/decode limits TS wire DTOs directly; map config fields through ecu_ts::pages::LimitsPageStore",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core limits TS pages delegate wire DTO construction/decode to LimitsPageStore")
PY
}

check_core_sensor_angle_page_store_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path("crates/core/src")
forbidden = re.compile(r"\b(SensorsPage::decode|AnglesPage::(?:new|decode))\s*\(")


def strip_rust_comments(text: str) -> str:
    out = []
    i = 0
    block_depth = 0
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if block_depth:
            if ch == "/" and nxt == "*":
                block_depth += 1
                i += 2
                continue
            if ch == "*" and nxt == "/":
                block_depth -= 1
                i += 2
                continue
            out.append("\n" if ch == "\n" else " ")
            i += 1
            continue

        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if ch == "/" and nxt == "*":
            block_depth = 1
            out.extend("  ")
            i += 2
            continue

        out.append(ch)
        i += 1

    return "".join(out)


violations = []
if root.exists():
    for path in root.rglob("*.rs"):
        text = path.read_text(encoding="utf-8")
        stripped_lines = strip_rust_comments(text).splitlines()
        original_lines = text.splitlines()
        for idx, line in enumerate(stripped_lines):
            if not forbidden.search(line):
                continue
            original = original_lines[idx] if idx < len(original_lines) else line
            violations.append(f"{path}:{idx + 1}:{original}")

if violations:
    print(
        "ERROR: ecu-core must not construct/decode sensors/angles TS wire DTOs directly; map config fields through ecu_ts::pages::SensorsPageStore/AnglesPageStore",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core sensors/angles TS pages delegate wire DTO construction/decode to SensorsPageStore/AnglesPageStore")
PY
}

check_core_snapshot_page_store_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

roots = [Path("crates/core/src"), Path("crates/core/tests")]
forbidden = re.compile(r"\b(SnapshotPage::new|SnapshotPage\s*\{|encode_snapshot_page\s*\()")

violations = []
for root in roots:
    if not root.exists():
        continue
    for path in root.rglob("*.rs"):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if forbidden.search(line):
                violations.append(f"{path}:{line_no}:{line}")

if violations:
    print(
        "ERROR: ecu-core must not construct/encode snapshot TS wire DTOs directly; map primitive fields through ecu_ts::pages::SnapshotPageStore",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core snapshot TS page delegates wire DTO construction/encoding to SnapshotPageStore")
PY
}

check_core_diagnostic_page_store_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

root = Path("crates/core/src")
forbidden = re.compile(
    r"\b("
    r"DiagPage::new|"
    r"DiagLogEntryPage::new|"
    r"DiagLogPage::new|"
    r"encode_diag_page\s*\(|"
    r"encode_diag_log_page\s*\("
    r")"
)


def strip_rust_comments(text: str) -> str:
    out = []
    i = 0
    block_depth = 0
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if block_depth:
            if ch == "/" and nxt == "*":
                block_depth += 1
                i += 2
                continue
            if ch == "*" and nxt == "/":
                block_depth -= 1
                i += 2
                continue
            out.append("\n" if ch == "\n" else " ")
            i += 1
            continue

        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if ch == "/" and nxt == "*":
            block_depth = 1
            out.extend("  ")
            i += 2
            continue

        out.append(ch)
        i += 1

    return "".join(out)


violations = []
if root.exists():
    for path in root.rglob("*.rs"):
        text = path.read_text(encoding="utf-8")
        stripped_lines = strip_rust_comments(text).splitlines()
        original_lines = text.splitlines()
        for idx, line in enumerate(stripped_lines):
            if not forbidden.search(line):
                continue
            original = original_lines[idx] if idx < len(original_lines) else line
            violations.append(f"{path}:{idx + 1}:{original}")

if violations:
    print(
        "ERROR: ecu-core must not construct/encode diagnostic TS wire DTOs directly; map primitive fields through ecu_ts::pages::DiagnosticPageStore",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core diagnostic TS pages delegate wire DTO construction/encoding to DiagnosticPageStore")
PY
}

check_board_common_legacy_state_scope() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

pattern = re.compile(r"\b(ecu_core|EcuState)\b")
production_roots = [Path("boards/common/src")]
allowed_test_files = {
    Path("boards/common/tests/fm0016_board_adapter_contract.rs"),
    Path("boards/common/tests/ts_angles_kv_roundtrip.rs"),
    Path("boards/common/tests/ts_factory_reset.rs"),
    Path("boards/common/tests/ts_kv_factory_reset.rs"),
    Path("boards/common/tests/ts_pages_roundtrip.rs"),
    Path("boards/common/tests/ts_pump_budget_stress.rs"),
    Path("boards/common/tests/ts_server_integration.rs"),
}

violations = []
for root in production_roots:
    if not root.exists():
        continue
    for path in root.rglob("*.rs"):
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if pattern.search(line):
                violations.append(f"{path}:{line_no}:{line}")

tests_root = Path("boards/common/tests")
if tests_root.exists():
    for path in tests_root.rglob("*.rs"):
        text = path.read_text(encoding="utf-8")
        if pattern.search(text) and path not in allowed_test_files:
            violations.append(
                f"{path}:1:board-common ecu-core/EcuState usage is limited to documented TS compatibility tests"
            )

if violations:
    print(
        "ERROR: boards/common must keep ecu-core/EcuState out of production and confined to documented compatibility tests",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: boards/common ecu-core/EcuState usage is confined to documented compatibility tests")
PY
}

check_board_api_batch_executor_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

roots = [Path("boards"), Path("crates"), Path("sim"), Path("fuzz")]
allowed_paths = {
    Path("crates/runtime/src/lib.rs"),
    Path("crates/runtime/src/lowering.rs"),
    Path("crates/runtime/src/tests.rs"),
}
skip_parts = {
    ".git",
    "target",
    "aidocs",
    "vendor",
    "reference",
    "references",
    "ref",
    "tests",
    "benches",
    "examples",
    "fixtures",
    "fixture",
}
pattern = re.compile(r"\bBoardApiBatchExecutor\b")

violations = []
for root in roots:
    if not root.exists():
        continue
    for path in root.rglob("*.rs"):
        if path in allowed_paths or any(part in skip_parts for part in path.parts):
            continue
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if pattern.search(line):
                violations.append(f"{path}:{line_no}:{line}")

if violations:
    print(
        "ERROR: BoardApiBatchExecutor is runtime-owned; board/sim live code must use ActionOutputBatchAdapter",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: BoardApiBatchExecutor stays runtime-owned behind ActionOutputBatchAdapter")
PY
}

check_scheduled_transition_queue_adapter_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

roots = [Path("boards"), Path("crates"), Path("sim"), Path("fuzz")]
allowed_paths = {
    Path("boards/common/src/outputs.rs"),
}
skip_parts = {
    ".git",
    "target",
    "aidocs",
    "vendor",
    "reference",
    "references",
    "ref",
    "tests",
    "benches",
    "examples",
    "fixtures",
    "fixture",
}
pattern = re.compile(r"\bScheduledTransitionQueueAdapter\b")

violations = []
for root in roots:
    if not root.exists():
        continue
    for path in root.rglob("*.rs"):
        if path in allowed_paths or any(part in skip_parts for part in path.parts):
            continue
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if pattern.search(line):
                violations.append(f"{path}:{line_no}:{line}")

if violations:
    print(
        "ERROR: ScheduledTransitionQueueAdapter is board-common compatibility plumbing; live board/sim code must use ActionOutputBatchAdapter or ScheduledActionExecutor seams",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: ScheduledTransitionQueueAdapter stays confined to board-common output plumbing")
PY
}

check_firmware_resolver_recipe_guards() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

lib_path = Path("sim/firmware-resolver/src/lib.rs")
main_path = Path("sim/firmware-resolver/src/main.rs")

lib = lib_path.read_text(encoding="utf-8")
main = main_path.read_text(encoding="utf-8")

required_snippets = {
    "supported invocation list stays explicit":
        'pub const SUPPORTED_INVOCATIONS: &[(&str, &str)] = &[',
    "rp2350 bringup invocation remains supported":
        '("rp2350b", "rev-limiter-no-watchdog-bringup")',
    "stm32f4 bringup invocation remains supported":
        '("stm32f4", "ignition-only-wasted-spark-no-watchdog-bringup")',
    "aliases are centralized":
        'pub const SUPPORTED_ALIASES: &[FirmwareAlias] = &[',
    "legacy pico alias maps to canonical board":
        'alias: "pico_ignition_only_wasted_spark",\n        board: "rp2040-pico",\n        recipe: "ignition-only-wasted-spark-no-watchdog-bringup"',
    "explicit pico alias maps to canonical board":
        'alias: "rp2040_pico_ignition_only_wasted_spark",\n        board: "rp2040-pico",\n        recipe: "ignition-only-wasted-spark-no-watchdog-bringup"',
    "rp2350 alias maps to canonical board":
        'alias: "rp2350_rev_limiter",\n        board: "rp2350b",\n        recipe: "rev-limiter-no-watchdog-bringup"',
    "stm32f4 alias maps to canonical board":
        'alias: "stm32f4_ignition_only_wasted_spark",\n        board: "stm32f4",\n        recipe: "ignition-only-wasted-spark-no-watchdog-bringup"',
    "alias resolver is the only no-recipe path":
        'None => resolve_alias(board_or_alias)',
    "RequiredButUnselected TS state remains rejected":
        "TunerStudioProfileSelection::RequiredButUnselected",
    "RequiredButUnselected resolver guard remains tested":
        "fn rejects_plans_without_selected_ts_asset()",
    "canonical watchdog rejection remains tested":
        "fn canonical_watchdog_only_recipes_are_not_supported()",
    "alias path equivalence remains tested":
        "fn alias_selection_renders_same_command_and_paths_as_canonical_pair()",
    "ATmega board+recipe selection is intentionally rejected":
        "fn unsupported_board_only_selection_is_rejected()",
}

required_main_snippets = {
    "CLI board recipe path resolves through canonical resolver":
        "resolve_firmware_selection(board, Some(recipe))",
    "CLI alias path resolves through alias resolver":
        "resolve_firmware_selection(alias, None)",
    "CLI supported output is backed by supported invocations":
        "for (board, recipe) in SUPPORTED_INVOCATIONS",
    "CLI supported output is backed by supported aliases":
        "for alias in SUPPORTED_ALIASES",
}

required_regexes = {
    "rp2040 bringup invocation remains supported":
        r'\(\s*"rp2040-pico",\s*"ignition-only-wasted-spark-no-watchdog-bringup",\s*\)',
}

violations = []
for label, snippet in required_snippets.items():
    if snippet not in lib:
        violations.append(f"{lib_path}: missing {label}")

for label, snippet in required_main_snippets.items():
    if snippet not in main:
        violations.append(f"{main_path}: missing {label}")

for label, pattern in required_regexes.items():
    if not re.search(pattern, lib, re.MULTILINE):
        violations.append(f"{lib_path}: missing {label}")

if violations:
    print(
        "ERROR: firmware resolver must keep user-facing board/recipe resolution explicit and recipe-driven",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: firmware resolver public paths stay explicit, canonical, and recipe-guarded")
PY
}

check_timing_island_safety_gate_neutrality() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

sections = [
    (
        Path("crates/board-api/src/traits.rs"),
        "pub trait OutputScheduler",
        "pub trait CalibrationStore",
    ),
    (
        Path("crates/runtime/src/lowering.rs"),
        "/// Lower one runtime action into optional timing-island commands.",
        "fn action_output_transition_count",
    ),
    (
        Path("boards/common/src/outputs.rs"),
        "/// Adapter that queues logical board output batches",
        "/// Adapter that queues logical board output batches",
    ),
]

policy_or_protocol = re.compile(r"\b(VE|AFR|TS|TunerStudio|CAN|M50)\b")
implementation_specific = re.compile(r"\b(fpga|cpld)\b", re.IGNORECASE)


def bounded_lines(path: Path, start_marker: str, end_marker: str) -> list[tuple[int, str]]:
    lines = path.read_text(encoding="utf-8").splitlines()
    start = next(
        (idx for idx, line in enumerate(lines) if start_marker in line),
        None,
    )
    end = next(
        (idx for idx, line in enumerate(lines[start + 1:], start + 1) if end_marker in line),
        None,
    ) if start is not None else None
    if start is None:
        raise RuntimeError(f"{path}: could not locate guarded TimingIsland/SafetyGate section")
    if end is None or end <= start:
        end = min(len(lines), start + 80)
    return [(idx + 1, lines[idx]) for idx in range(start, end)]


violations = []
for path, start_marker, end_marker in sections:
    for line_no, line in bounded_lines(path, start_marker, end_marker):
        policy_match = policy_or_protocol.search(line)
        implementation_match = implementation_specific.search(line)
        if policy_match:
            violations.append(f"{path}:{line_no}: policy/protocol term {policy_match.group(0)}: {line}")
        if implementation_match:
            violations.append(
                f"{path}:{line_no}: implementation-specific term {implementation_match.group(0)}: {line}"
            )

if violations:
    print(
        "ERROR: TimingIsland/SafetyGate live contract and adapter surfaces must stay policy-, protocol-, and implementation-neutral",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: TimingIsland/SafetyGate live contract and adapter surfaces are neutral")
PY
}

check_scheduler_fuel_strategy_internals() {
    local files_file
    files_file="$(mktemp)"
    trap 'rm -f "$files_file"' RETURN

    rg --files "${common_globs[@]}" --glob '*.rs' crates/scheduler/src >"$files_file"

    python3 - "$files_file" <<'PY'
import re
import sys
from pathlib import Path

files = [Path(line) for line in Path(sys.argv[1]).read_text(encoding="utf-8").splitlines()]

forbidden = re.compile(
    r"\b("
    r"VE|AFR|"
    r"ve_table|afr_table|target_afr|stoich|"
    r"required_fuel|deadtime|"
    r"injector_(flow|size|cc|deadtime|latency|open_time)"
    r")\b",
    re.IGNORECASE,
)


def strip_rust_comments(text: str) -> str:
    out = []
    i = 0
    block_depth = 0
    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if block_depth:
            if ch == "/" and nxt == "*":
                block_depth += 1
                i += 2
                continue
            if ch == "*" and nxt == "/":
                block_depth -= 1
                i += 2
                continue
            out.append("\n" if ch == "\n" else " ")
            i += 1
            continue

        if ch == "/" and nxt == "/":
            while i < len(text) and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if ch == "/" and nxt == "*":
            block_depth = 1
            out.extend("  ")
            i += 2
            continue

        out.append(ch)
        i += 1

    return "".join(out)


violations = []
for path in files:
    text = path.read_text(encoding="utf-8")
    stripped = strip_rust_comments(text)
    original_lines = text.splitlines()
    for line_no, line in enumerate(stripped.splitlines(), 1):
        match = forbidden.search(line)
        if match:
            original = original_lines[line_no - 1].strip() if line_no <= len(original_lines) else line.strip()
            violations.append(f"{path}:{line_no}: {match.group(0)}: {original}")

if violations:
    print(
        "ERROR: PR5 scheduler implementation must not inspect fuel-strategy internals",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: PR5 scheduler implementation has no fuel-strategy internals")
PY
}

check_board_ts_provider_unsafe() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

# Board TS providers (OutpcProvider / PageStoreProvider impls) must not hold and
# transiently dereference a raw pointer to board state. The single audited home
# for that indirection is boards/common/src/ts/state_ptr.rs. Boards may only reach
# board state through that API: audited `unsafe { <expr>.with(...) }` /
# `.with_mut(...)` calls are allowed; a stored `*mut`/`*const` self-pointer field
# or a raw deref of one (`&*self.<field>` / `&mut *self.<field>`) is forbidden.
boards_root = Path("boards")
allowed_paths = {
    Path("boards/common/src/ts/state_ptr.rs"),
}
deref_pattern = re.compile(r"unsafe\s*\{\s*\(?\s*&(?:mut)?\s*\*\s*self\.")
raw_field_pattern = re.compile(r"^\s*[A-Za-z_]\w*\s*:\s*\*\s*(?:mut|const)\s+[A-Za-z_]")

skip_parts = {
    ".git",
    "target",
    "tests",
    "benches",
    "examples",
    "fixtures",
    "fixture",
}


def is_skipped(path: Path) -> bool:
    return any(part in skip_parts for part in path.parts)


def is_raw_field(line: str) -> bool:
    return bool(raw_field_pattern.search(line)) and "fn " not in line and "(" not in line


violations = []
if boards_root.exists():
    for path in boards_root.rglob("*.rs"):
        if path in allowed_paths or is_skipped(path):
            continue
        for line_no, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
            if deref_pattern.search(line) or is_raw_field(line):
                violations.append(f"{path}:{line_no}:{line.strip()}")

if violations:
    print(
        "ERROR: board TS-provider sources must not hold or deref a raw self-pointer; route board state through boards/common/src/ts/state_ptr.rs",
        file=sys.stderr,
    )
    for violation in violations:
        print(violation, file=sys.stderr)
    sys.exit(1)

print("PASS: board TS-provider sources hold no raw self-pointer fields or derefs outside boards/common StatePtr")
PY
}

check_core_compatibility_shell_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

lib_path = Path("crates/core/src/lib.rs")
compat_path = Path("crates/core/src/compat_state.rs")
lib_text = lib_path.read_text(encoding="utf-8")
compat_text = compat_path.read_text(encoding="utf-8")

errors = []

if "pub mod compat {" not in lib_text:
    errors.append("crates/core/src/lib.rs: missing explicit `pub mod compat` namespace")

if "adr-0001-core-ownership.md" not in lib_text or "adr-0010-runtime-compat-boundaries.md" not in lib_text:
    errors.append("crates/core/src/lib.rs: crate docs must reference ADR-0001 and ADR-0010")

required_markers = {
    "RuntimeSignals": "Compatibility-only surface",
    "EcuInputs": "Compatibility-only surface",
    "EcuDerived": "Compatibility-only surface",
    "EcuOutputs": "Compatibility-only surface",
    "EcuFaults": "Compatibility-only surface",
}

for typename, marker in required_markers.items():
    marker_pos = compat_text.find(marker)
    struct_pos = compat_text.find(f"pub struct {typename}")
    if marker_pos == -1 or struct_pos == -1 or marker_pos > struct_pos:
        errors.append(
            f"crates/core/src/compat_state.rs: `{typename}` docs must carry compatibility-only marker"
        )

if errors:
    print(
        "ERROR: ecu-core compatibility shell boundary is not documented/enforced as required",
        file=sys.stderr,
    )
    for error in errors:
        print(error, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-core compatibility shell boundary markers are present")
PY
}

check_runtime_primary_ingress_boundary() {
    python3 - <<'PY'
from pathlib import Path
import re
import sys

lib_path = Path("crates/runtime/src/lib.rs")
step_path = Path("crates/runtime/src/engine/step.rs")
lib_text = lib_path.read_text(encoding="utf-8")
step_text = step_path.read_text(encoding="utf-8")

errors = []

if "pub mod ingress {" not in lib_text:
    errors.append("crates/runtime/src/lib.rs: missing explicit `ingress` namespace")
if "pub mod support {" not in lib_text:
    errors.append("crates/runtime/src/lib.rs: missing explicit `support` namespace")
if "adr-0010-runtime-compat-boundaries.md" not in lib_text:
    errors.append("crates/runtime/src/lib.rs: crate docs must reference ADR-0010")

if "canonical runtime boundary" not in lib_text:
    errors.append("crates/runtime/src/lib.rs: docs must identify structured ingress as canonical")

for forbidden in [
    "DifferentialInputSnapshot",
    "RuntimeObservedSurface",
    "RuntimeAdapterContract",
    "extract_fuel_observations",
    "extract_torque_observations",
]:
    pattern = re.compile(rf"pub use observations::.*\\b{re.escape(forbidden)}\\b", re.S)
    if pattern.search(lib_text):
        errors.append(
            f"crates/runtime/src/lib.rs: support symbol `{forbidden}` must not be re-exported from main root surface"
        )

if "Compatibility/support stepping path" not in step_text:
    errors.append("crates/runtime/src/engine/step.rs: raw `step` docs must mark compatibility/support role")
if "This is the canonical product ingress." not in step_text:
    errors.append("crates/runtime/src/engine/step.rs: `step_with_authority` docs must mark canonical role")

if errors:
    print(
        "ERROR: ecu-runtime ingress/support boundary is not documented/enforced as required",
        file=sys.stderr,
    )
    for error in errors:
        print(error, file=sys.stderr)
    sys.exit(1)

print("PASS: ecu-runtime primary ingress/support boundary markers are present")
PY
}

check_no_rg_hits \
    "PR4 generic runtime/scheduler code has no product-specific M50 names" \
    'M50|configure_m50|RuntimeOutputProfile::M50|M50OutputProfile|M50IgnitionMode' \
    crates/runtime/src crates/scheduler/src

check_no_live_rg_hits \
    "PR4 live implementation has no legacy M50 runtime API names" \
    'RuntimeOutputProfile::M50|M50OutputProfile|M50IgnitionMode|configure_m50' \
    Cargo.toml crates boards sim fuzz

if ! check_scheduler_fuel_strategy_internals; then
    status=1
fi

if ! check_board_common_compat_helper_names; then
    status=1
fi

if ! check_raw_ts_kv_boundary; then
    status=1
fi

if ! check_core_transport_compat_surface; then
    status=1
fi

if ! check_core_sensor_calibration_compat_surface; then
    status=1
fi

if ! check_core_ts_shell_compat_surface; then
    status=1
fi

if ! check_core_actuator_page_store_boundary; then
    status=1
fi

if ! check_core_enrichment_page_store_boundary; then
    status=1
fi

if ! check_core_limits_page_store_boundary; then
    status=1
fi

if ! check_core_sensor_angle_page_store_boundary; then
    status=1
fi

if ! check_core_snapshot_page_store_boundary; then
    status=1
fi

if ! check_core_diagnostic_page_store_boundary; then
    status=1
fi

if ! check_board_api_batch_executor_boundary; then
    status=1
fi

if ! check_scheduled_transition_queue_adapter_boundary; then
    status=1
fi

if ! check_firmware_resolver_recipe_guards; then
    status=1
fi

if ! check_timing_island_safety_gate_neutrality; then
    status=1
fi

if ! check_board_ts_provider_unsafe; then
    status=1
fi

if ! check_core_compatibility_shell_boundary; then
    status=1
fi

if ! check_runtime_primary_ingress_boundary; then
    status=1
fi

check_no_rg_hits \
    "PR3 live code does not reintroduce PersistedEcuPageStore" \
    'PersistedEcuPageStore' \
    Cargo.toml crates boards sim fuzz

check_no_rg_hits \
    "PR3 board-common tests do not use legacy core persisted TS page stores" \
    '\b(PersistedPageStore|EcuStatePageStore)\b' \
    boards/common/tests

if ! check_board_common_legacy_state_scope; then
    status=1
fi

check_no_rg_hits \
    "PR3 core no longer owns the fuel/ign-only EcuStatePageStore adapter" \
    '\bEcuStatePageStore\b' \
    crates/core boards sim fuzz

check_no_rg_hits \
    "PR3 core no longer owns the expert-trigger byte-backed page state" \
    'struct ExpertTriggerPageState|DEFAULT_EXPERT_TRIGGER_RECORD|default_expert_trigger_record|decode_expert_trigger_record' \
    crates/core/src crates/core/tests

check_no_live_rg_hits \
    "runtime action lowerer is not called directly from board/sim implementation code" \
    '\blower_action_batch_to_board_batches[[:space:]]*\(' \
    boards sim

check_no_live_rg_hits \
    "PR6 live code/manifests have no legacy root scheduler/app references" \
    'legacy-root-scheduler|sched-simple|crate::scheduler|EcuApp|struct Channel\(' \
    Cargo.toml crates boards sim fuzz

check_no_rg_hits \
    "PR6 ecu-core lib has no legacy app/config/scheduler modules" \
    '^pub mod (app|config|scheduler);' \
    crates/core/src/lib.rs

check_no_rg_hits \
    "PR6 public docs do not resurrect stale monolithic-core claims" \
    'zero dependencies in core|core is platform[-‑ ]agnostic|platform[-‑ ]agnostic core' \
    README.md crates/core/src/lib.rs

exit "$status"
