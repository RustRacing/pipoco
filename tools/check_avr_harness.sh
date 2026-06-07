#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

tools/prepare_avr_tester_patch.sh

export BINDGEN_EXTRA_CLANG_ARGS="${BINDGEN_EXTRA_CLANG_ARGS:---target=x86_64-pc-linux-gnu -m64}"

cargo test --manifest-path sim/avr-harness/Cargo.toml -- "$@"
