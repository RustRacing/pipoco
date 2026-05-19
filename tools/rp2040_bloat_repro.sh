#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

target="thumbv6m-none-eabi"
bin="ts-ecu"
log_dir="target/rp2040-bloat-repro"
mkdir -p "$log_dir"

run_case() {
    local profile="$1"
    local cargo_env=()
    local cargo_profile_args=()
    local bloat_profile_args=()
    local artifact_dir
    local elf_path
    local log_path
    local status
    local symtab_state="absent"
    local text_state="absent"

    case "$profile" in
        release)
            cargo_env=(CARGO_PROFILE_RELEASE_STRIP=false)
            cargo_profile_args=(--release)
            bloat_profile_args=(--release)
            artifact_dir="release"
            ;;
        audit)
            cargo_profile_args=(--profile audit)
            bloat_profile_args=(--profile audit)
            artifact_dir="audit"
            ;;
        *)
            echo "unsupported profile: $profile" >&2
            return 1
            ;;
    esac

    env "${cargo_env[@]}" cargo build -p ecu-rp2040-pico "${cargo_profile_args[@]}" --target "$target" --bin "$bin"

    elf_path="target/$target/$artifact_dir/$bin"
    if [[ ! -f "$elf_path" ]]; then
        echo "missing RP2040 ELF: $elf_path" >&2
        return 1
    fi

    if readelf -S "$elf_path" | grep -q '\.text'; then
        text_state="present"
    fi

    if readelf -S "$elf_path" | grep -q '\.symtab'; then
        symtab_state="present"
    fi

    if [[ $text_state != present ]]; then
        echo "$profile: missing code section (.text) in $elf_path" >&2
        return 1
    fi

    if [[ $symtab_state != present ]]; then
        symtab_state="absent"
        echo "$profile: missing symbol table (.symtab) in $elf_path" >&2
        return 1
    fi

    log_path="$log_dir/${profile}-${bin}.log"
    {
        printf '%s profile=%s elf=%s text=%s symtab=%s\n' "$profile" "$bin" "$elf_path" "$text_state" "$symtab_state"
    } >"$log_path"
    set +e
    cargo bloat -p ecu-rp2040-pico "${bloat_profile_args[@]}" --target "$target" --bin "$bin" -n 50 \
        >>"$log_path" 2>&1
    status=$?
    set -e

    if [[ $status -eq 0 ]]; then
        echo "$profile: cargo bloat succeeded; log captured at $log_path"
        return 0
    fi

    echo "$profile: cargo-bloat failed; see $log_path" >&2
    return "$status"
}

run_case release
run_case audit
