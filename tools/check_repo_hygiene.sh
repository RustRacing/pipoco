#!/bin/bash
# Repository hygiene gate — fails if generated verifier/session artifacts are tracked.
# Only checks version control membership (git ls-files), not working-tree presence.
set -euo pipefail

DENYLIST=(
    ".codex"
    ".session_id"
    ".session_meta.json"
    "_apalache-out"
    "states"
    "verus"
    "target/formal-evidence"
)

GATE_NAME="=== Repository Hygiene Gate ==="
OFFENDING=""

echo "$GATE_NAME"

for artifact in "${DENYLIST[@]}"; do
    # git ls-files only checks version-control membership; ignored local
    # generated files are allowed to exist after formal runs.
    if output=$(git ls-files "$artifact" 2>/dev/null) && [[ -n "$output" ]]; then
        OFFENDING="${OFFENDING}${output}"$'\n'
        echo "TRACKED: $artifact"
    fi
done

if [[ -n "$OFFENDING" ]]; then
    echo ""
    echo "ERROR: The following generated/session artifacts are tracked in git:"
    echo "$OFFENDING"
    echo "Remove them with: git rm -r --cached <path>"
    exit 1
fi

echo "PASS: No tracked generated artifacts"
exit 0
