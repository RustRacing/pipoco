#!/usr/bin/env bash
set -euo pipefail

# --- Repository hygiene gate (US-FM0703) ---
bash tools/check_repo_hygiene.sh

# M50 first-run evidence tooling must stay runnable even before hardware artifacts exist.
python3 -m unittest discover -s tools/tests -p 'test_*.py'
python3 tools/check_software_readiness.py

# Warning denial is ACTIVE for all embedded board/product targets.
echo "[verify.sh] WARNING DENIAL IS ACTIVE — embedded build warnings will fail the gate"

require_workspace_packages() {
    python3 - "$@" <<'PY'
import json
import subprocess
import sys

expected = sys.argv[1:]
metadata = json.loads(
    subprocess.check_output(
        ["cargo", "metadata", "--no-deps", "--format-version", "1"],
        text=True,
    )
)
packages = {pkg["name"] for pkg in metadata["packages"]}
missing = [name for name in expected if name not in packages]
if missing:
    print(
        "[verify.sh] FAIL: missing workspace packages referenced by verify.sh: "
        + ", ".join(missing),
        file=sys.stderr,
    )
    sys.exit(1)
PY
}

core_packages=(
    ecu-board-api
    ecu-calibration
    ecu-compat
    ecu-control
    ecu-domain
    ecu-io
    ecu-runtime
    ecu-scheduler
    ecu-transport
    ecu-trigger
    ecu-ts
    ecu-sim-hifi
)

board_packages=(
    stm32f4-ecu
    ecu-rp2040-pico
    ecu-rp2350b
)

require_workspace_packages "${core_packages[@]}" "${board_packages[@]}"

# Green checks for the currently verified host/core slices of the workspace.
for package in "${core_packages[@]}"; do
    cargo clippy -p "$package" --all-targets --all-features -- -D warnings
done
cargo test -p ecu-sim-hifi

# STM32F4 checks (US-FM0287) — clippy with -D warnings on bins
cargo clippy -p stm32f4-ecu --release --target thumbv7em-none-eabihf --bins -- -D warnings
cargo clippy -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-tim --bins -- -D warnings
cargo clippy -p stm32f4-ecu --release --target thumbv7em-none-eabihf --no-default-features --features "hardware capture-gpio" --bins -- -D warnings

# RP2040 Pico checks (US-FM0287 / US-FM0290) — clippy with -D warnings on bins
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features capture-pio --bins -- -D warnings
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "flash-kv capture-pio" --bins -- -D warnings
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features example-bins --bin ts-gauges -- -D warnings
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "example-bins capture-pio" --bin pio_capture_example -- -D warnings

# RP2350B checks (US-FM0306) — clippy with -D warnings on bins
# Target triple uses thumbv8m.main (dot, not hyphen) — verified installed via rustup
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features ve-demo --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features capture-gpio --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features example-bins --bins -- -D warnings

echo ""
echo "[verify.sh] Feature-gate validation"

# STM32F4 capture profiles are mutually exclusive.
cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-tim --bin stm32f4-ecu
cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --no-default-features --features "hardware capture-gpio" --bin stm32f4-ecu
if cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features "capture-tim capture-gpio" --bin stm32f4-ecu >/dev/null 2>&1; then
    echo "FAIL: stm32f4-ecu accepted mutually exclusive capture-tim + capture-gpio" >&2
    exit 1
fi

# RP2350B capture profile slice (capture-pio was removed from this board).
cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features "ve-demo capture-gpio" --bin ecu-rp2350b-demo

# RP2040 feature profiles are modeled as independent slices.
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "flash-kv capture-pio" --bin ts-ecu
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "capture-cam capture-pio" --bin ts-ecu
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features example-bins --bin ts-gauges
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "example-bins capture-pio" --bin pio_capture_example

echo "[verify.sh] All warning checks passed"
