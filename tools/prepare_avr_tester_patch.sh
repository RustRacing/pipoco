#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

repo_url="${PIPOCO_SIMAVR_FFI_REPO:-https://codeberg.org/pwy/simavr-ffi.git}"
repo_ref="${PIPOCO_SIMAVR_FFI_REF:-69f95e3b32b1cf3872c7bf7a148d79525d2e5134}"
patch_dir="${PIPOCO_SIMAVR_FFI_DIR:-.local/avr-tester/simavr-ffi}"

mkdir -p "$(dirname "$patch_dir")"

if [[ ! -d "$patch_dir/.git" ]]; then
    git clone --recurse-submodules "$repo_url" "$patch_dir"
fi

git -C "$patch_dir" fetch --quiet origin
git -C "$patch_dir" checkout --quiet "$repo_ref"
git -C "$patch_dir" submodule update --init --recursive --quiet

build_rs="$patch_dir/build.rs"

if ! grep -q 'cargo:rustc-link-lib=static=elf' "$build_rs"; then
    if grep -q 'cargo:rustc-link-lib=elf' "$build_rs"; then
        echo "simavr-ffi libelf link patch already present at $patch_dir"
    else
        echo "ERROR: expected libelf link line not found in $build_rs" >&2
        exit 1
    fi
else
    perl -0pi -e 's/cargo:rustc-link-lib=static=elf/cargo:rustc-link-lib=elf/' "$build_rs"
    echo "patched simavr-ffi libelf link at $patch_dir"
fi

cat <<EOF

Use this in the AVR test crate that depends on avr-tester:

[patch.crates-io]
simavr-ffi = { path = "$(pwd)/$patch_dir" }

If that crate lives directly under the repo root, for example tests-avr/,
this relative path also works:
simavr-ffi = { path = "../$patch_dir" }
EOF
