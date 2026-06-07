#!/usr/bin/env bash
# Render every firmware-resolver recipe and build each rendered invocation, so
# recipes (and the build commands they render) cannot rot. Each supported
# board+recipe pair is read from `ecu-firmware-resolver --list`; the build
# command is rendered with the resolver and executed verbatim.
set -euo pipefail

cd "$(dirname "$0")/.."

resolver=(cargo run -q -p ecu-firmware-resolver --)

mapfile -t invocations < <(
    "${resolver[@]}" --list \
        | sed -n '/^supported invocations:/,/^aliases:/p' \
        | grep '^  ' \
        | sed 's/^  //'
)

if [ "${#invocations[@]}" -eq 0 ]; then
    echo "ERROR: no supported invocations rendered by ecu-firmware-resolver --list" >&2
    exit 1
fi

status=0
for inv in "${invocations[@]}"; do
    board="${inv%% *}"
    recipe="${inv#* }"
    echo "=== ${board} ${recipe} ==="

    build_cmd="$("${resolver[@]}" "${board}" "${recipe}")"
    echo "build:  ${build_cmd}"
    "${resolver[@]}" --bin-path "${board}" "${recipe}" >/dev/null
    "${resolver[@]}" --elf-path "${board}" "${recipe}" >/dev/null
    # Flash commands are board-specific and not defined for every board.
    "${resolver[@]}" --flash-command "${board}" "${recipe}" >/dev/null 2>&1 || true

    if [ "${RECIPE_CHECK_RENDER_ONLY:-0}" = "1" ]; then
        echo "render-only: skipping build"
        continue
    fi

    if ! bash -c "${build_cmd}"; then
        echo "ERROR: build failed for ${board} ${recipe}" >&2
        status=1
    fi
done

exit "${status}"
