#!/usr/bin/env bash
set -euo pipefail

# --- Repository hygiene gate (US-FM0703) ---
bash tools/check_repo_hygiene.sh

# Warning denial is ACTIVE for all embedded board/product targets.
echo "[verify.sh] WARNING DENIAL IS ACTIVE — embedded build warnings will fail the gate"

# Green checks for the currently verified slices of the workspace.
cargo clippy -p ecu-core --all-targets --all-features -- -D warnings

# STM32F4 checks (US-FM0287) — clippy with -D warnings on bins
cargo clippy -p stm32f4-ecu --release --target thumbv7em-none-eabihf --bins -- -D warnings
cargo clippy -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-tim --bins -- -D warnings
cargo clippy -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-gpio --bins -- -D warnings

# RP2040 Pico checks (US-FM0287 / US-FM0290) — clippy with -D warnings on bins
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --bins -- -D warnings
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features flash-kv --bins -- -D warnings
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features example-bins --bin ts-gauges -- -D warnings
cargo clippy -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "example-bins capture-pio" --bin pio_capture_example -- -D warnings

# RP2350B checks (US-FM0306) — clippy with -D warnings on bins
# Target triple uses thumbv8m.main (dot, not hyphen) — verified installed via rustup
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features ve-demo --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features capture-gpio --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features capture-pio --bins -- -D warnings
cargo clippy -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features example-bins --bins -- -D warnings

echo ""
echo "[verify.sh] Feature-gate validation"

# STM32F4 capture profiles are mutually exclusive.
cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-tim --bin stm32f4-ecu
cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features capture-gpio --bin stm32f4-ecu
if cargo check -p stm32f4-ecu --release --target thumbv7em-none-eabihf --features "capture-tim capture-gpio" --bin stm32f4-ecu >/dev/null 2>&1; then
    echo "FAIL: stm32f4-ecu accepted mutually exclusive capture-tim + capture-gpio" >&2
    exit 1
fi

# RP2350B capture profiles are mutually exclusive.
cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features "ve-demo capture-gpio" --bin ecu-rp2350b-demo
cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features capture-pio --bin rp2350-pio-capture-example
if cargo check -p ecu-rp2350b --release --target thumbv8m.main-none-eabihf --features "ve-demo capture-gpio capture-pio" --bin ecu-rp2350b-demo >/dev/null 2>&1; then
    echo "FAIL: ecu-rp2350b accepted mutually exclusive capture-gpio + capture-pio" >&2
    exit 1
fi

# RP2040 feature profiles are modeled as independent slices.
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features flash-kv --bin ts-ecu
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features capture-cam --bin ts-ecu
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features example-bins --bin ts-gauges
cargo check -p ecu-rp2040-pico --release --target thumbv6m-none-eabi --features "example-bins capture-pio" --bin pio_capture_example

echo "[verify.sh] All warning checks passed"
