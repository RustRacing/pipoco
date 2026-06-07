#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

# Architecture rationale: aidocs/architecture/adr-0001-core-ownership.md.
# The split runtime/control path is canonical for new product behavior; the
# legacy `ecu-core` crate is a compatibility facade until board migration
# removes the remaining target-specific direct usages.

metadata_file="$(mktemp)"
tree_file="$(mktemp)"
board_packages_file="$(mktemp)"
concrete_board_packages_file="$(mktemp)"
ecu_core_package_file="$(mktemp)"
runtime_package_file="$(mktemp)"
scheduler_package_file="$(mktemp)"
trap 'rm -f "$metadata_file" "$tree_file" "$board_packages_file" "$concrete_board_packages_file" "$ecu_core_package_file" "$runtime_package_file" "$scheduler_package_file"' EXIT

echo "[core-architecture] cargo metadata --no-deps"
cargo metadata --no-deps --format-version 1 >"$metadata_file"

python3 - "$metadata_file" "$board_packages_file" "$concrete_board_packages_file" <<'PY'
import json
import sys
from pathlib import Path

metadata_path = Path(sys.argv[1])
board_packages_path = Path(sys.argv[2])
concrete_board_packages_path = Path(sys.argv[3])
metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
workspace_root = Path(metadata["workspace_root"]).resolve()
boards_dir = workspace_root / "boards"

board_packages = {
    "ecu-target-common",
    "ecu-atmega2560",
    "ecu-rp2040-pico",
    "ecu-rp2350b",
    "stm32f4-ecu",
}

for package in metadata["packages"]:
    manifest_path = Path(package["manifest_path"]).resolve()
    try:
        manifest_path.relative_to(boards_dir)
    except ValueError:
        continue
    board_packages.add(package["name"])

concrete_board_packages = board_packages - {"ecu-target-common"}

board_packages_path.write_text(
    "".join(f"{package_name}\n" for package_name in sorted(board_packages)),
    encoding="utf-8",
)
concrete_board_packages_path.write_text(
    "".join(f"{package_name}\n" for package_name in sorted(concrete_board_packages)),
    encoding="utf-8",
)
PY

check_dependency_tree() {
    local checked_package="$1"
    local forbidden_packages_file="$2"
    local forbidden_label="$3"
    local edge_kinds="${4:-normal,dev}"

    echo "[core-architecture] cargo tree -p $checked_package --edges $edge_kinds"
    cargo tree -p "$checked_package" --edges "$edge_kinds" --prefix none --format '{p}' >"$tree_file"

    python3 - "$checked_package" "$forbidden_packages_file" "$tree_file" "$forbidden_label" "$edge_kinds" <<'PY'
import re
import sys
from pathlib import Path

checked_package = sys.argv[1]
board_packages = {
    line.strip()
    for line in Path(sys.argv[2]).read_text(encoding="utf-8").splitlines()
    if line.strip()
}
tree_lines = Path(sys.argv[3]).read_text(encoding="utf-8").splitlines()
forbidden_label = sys.argv[4]
edge_kinds = sys.argv[5]
violations = []

for line in tree_lines:
    match = re.match(r"^(.+?) v[0-9][^ ]*", line)
    if match and match.group(1) in board_packages:
        violations.append(line)

if violations:
    print(
        f"ERROR: {checked_package} {edge_kinds} dependency tree contains {forbidden_label}:",
        file=sys.stderr,
    )
    for violation in violations:
        print(f"  {violation}", file=sys.stderr)
    sys.exit(1)

print(f"PASS: {checked_package} has no {forbidden_label} in {edge_kinds} dependency tree")
PY
}

check_no_direct_workspace_deps() {
    local checked_package="$1"
    local forbidden_packages_file="$2"
    local forbidden_label="$3"

    python3 - "$metadata_file" "$checked_package" "$forbidden_packages_file" "$forbidden_label" <<'PY'
import json
import sys
from pathlib import Path

metadata = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
checked_package = sys.argv[2]
forbidden = {
    line.strip()
    for line in Path(sys.argv[3]).read_text(encoding="utf-8").splitlines()
    if line.strip()
}
forbidden_label = sys.argv[4]

matches = []
for package in metadata["packages"]:
    if package["name"] != checked_package:
        continue
    for dep in package["dependencies"]:
        if dep["name"] in forbidden and dep["kind"] is None:
            matches.append(dep["name"])
    break
else:
    print(f"ERROR: package {checked_package} not found in cargo metadata", file=sys.stderr)
    sys.exit(1)

if matches:
    print(
        f"ERROR: {checked_package} has direct normal dependencies on {forbidden_label}:",
        file=sys.stderr,
    )
    for match in sorted(set(matches)):
        print(f"  {match}", file=sys.stderr)
    sys.exit(1)

print(f"PASS: {checked_package} has no direct normal dependencies on {forbidden_label}")
PY
}

check_no_board_state_refs() {
    local source_dir="$1"
    local matches
    matches="$(rg -n '\becu_core\b|\bEcuState\b' "$source_dir" 2>/dev/null || true)"
    if [[ -n "$matches" ]]; then
        printf 'ERROR: %s production sources reference ecu_core/EcuState:\n%s\n' "$source_dir" "$matches" >&2
        return 1
    fi

    printf 'PASS: %s production sources have no ecu_core/EcuState references\n' "$source_dir"
}

status=0
core_boundary_packages=(
    ecu-core
    ecu-board-api
    ecu-io
    ecu-runtime
    ecu-board-profiles
)

for package in "${core_boundary_packages[@]}"; do
    if ! check_dependency_tree "$package" "$board_packages_file" "board packages"; then
        status=1
    fi
done

if ! check_dependency_tree ecu-target-common "$concrete_board_packages_file" "concrete board packages"; then
    status=1
fi

printf 'ecu-core\n' >"$ecu_core_package_file"
if ! check_dependency_tree ecu-target-common "$ecu_core_package_file" "ecu-core" "normal"; then
    status=1
fi

printf 'ecu-runtime\n' >"$runtime_package_file"
if ! check_no_direct_workspace_deps ecu-board-profiles "$runtime_package_file" "ecu-runtime"; then
    status=1
fi

if ! check_no_direct_workspace_deps ecu-firmware-resolver "$concrete_board_packages_file" "concrete board packages"; then
    status=1
fi

printf 'ecu-scheduler\n' >"$scheduler_package_file"
for package in ecu-sim ecu-sim-ffi; do
    if ! check_no_direct_workspace_deps "$package" "$scheduler_package_file" "ecu-scheduler"; then
        status=1
    fi
done

for source_dir in boards/common/src boards/stm32f4/src boards/rp2040-pico/src; do
    if ! check_no_board_state_refs "$source_dir"; then
        status=1
    fi
done

exit "$status"
