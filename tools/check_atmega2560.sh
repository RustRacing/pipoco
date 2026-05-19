#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

RUSTFLAGS='-C target-cpu=atmega2560' \
  cargo +nightly -Z build-std=core build -p ecu-atmega2560 --target avr-none
