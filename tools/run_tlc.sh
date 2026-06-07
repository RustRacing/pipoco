#!/usr/bin/env bash
# Run TLC on the pinned TLA+ models, failing on any invariant or temporal
# violation. Shared by CI (.github/workflows/formal-tlc.yml) and developers.
#
# Resolves the tla2tools jar in this order:
#   1. $TLA_JAR if set and present
#   2. the verify_formal.sh local path (~/.local/lib/tla/tla2tools.jar)
#   3. download the pinned release into $TLA_CACHE (default: target/tla)
#
# Usage: tools/run_tlc.sh [module ...]   (default: trigger scheduler)
set -euo pipefail

_tla_version="2.19"
# tla2tools.jar shipping TLC2 version 2.19 (tlaplus/tlaplus release v1.8.0).
# The TLC -version check below pins the effective version regardless of source.
_tla_url="https://github.com/tlaplus/tlaplus/releases/download/v1.8.0/tla2tools.jar"

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$repo_root"

_local_jar="${HOME}/.local/lib/tla/tla2tools.jar"
_cache_dir="${TLA_CACHE:-target/tla}"
_cache_jar="${_cache_dir}/tla2tools.jar"

resolve_jar() {
    if [[ -n "${TLA_JAR:-}" && -f "${TLA_JAR}" ]]; then
        echo "${TLA_JAR}"
        return 0
    fi
    if [[ -f "${_local_jar}" ]]; then
        echo "${_local_jar}"
        return 0
    fi
    if [[ -f "${_cache_jar}" ]]; then
        echo "${_cache_jar}"
        return 0
    fi
    mkdir -p "${_cache_dir}"
    echo "Downloading tla2tools.jar (TLC ${_tla_version}) from ${_tla_url}" >&2
    if command -v curl >/dev/null 2>&1; then
        curl -fsSL "${_tla_url}" -o "${_cache_jar}"
    else
        wget -qO "${_cache_jar}" "${_tla_url}"
    fi
    echo "${_cache_jar}"
}

_jar="$(resolve_jar)"

set +e
_actual=$(java -cp "${_jar}" tlc2.TLC -version 2>&1 | sed -n 's/^TLC2 Version \([^ ]*\).*/\1/p' | head -1)
set -e
echo "TLC version: ${_actual} (expected ${_tla_version}) at ${_jar}"
if [[ "${_actual}" != "${_tla_version}" ]]; then
    echo "FAIL: TLC version mismatch (expected ${_tla_version}, got ${_actual})" >&2
    exit 1
fi

_modules=("$@")
if [[ ${#_modules[@]} -eq 0 ]]; then
    _modules=(trigger scheduler)
fi

_failed=0
for _module in "${_modules[@]}"; do
    _cfg="crates/spec/tla/${_module}.cfg"
    _tla="crates/spec/tla/${_module}.tla"
    if [[ ! -f "${_cfg}" || ! -f "${_tla}" ]]; then
        echo "FAIL: missing model files for '${_module}'" >&2
        _failed=1
        continue
    fi
    echo "=== TLC: ${_module} ==="
    set +e
    _out=$(java -cp "${_jar}" tlc2.TLC -config "${_cfg}" "${_tla}" 2>&1)
    _status=$?
    set -e
    echo "${_out}"
    if [[ ${_status} -ne 0 ]]; then
        echo "FAIL: TLC exited non-zero for ${_module}" >&2
        _failed=1
        continue
    fi
    if ! echo "${_out}" | grep -q "Model checking completed. No error has been found."; then
        echo "FAIL: TLC did not complete cleanly for ${_module}" >&2
        _failed=1
    fi
done

if [[ ${_failed} -ne 0 ]]; then
    echo "FAIL: TLC model checking failed" >&2
    exit 1
fi
echo "run_tlc.sh: all models passed"
