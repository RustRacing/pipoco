#!/usr/bin/env python3
"""
Test script for tools/evidence.schema.json (US-FM0428).

Reads the JSON schema and validates target/formal-evidence/latest/summary.json
if it exists. Exits 0 on success, 1 on failure.
"""

import json
import sys
from pathlib import Path

SCHEMA_PATH = Path("tools/evidence.schema.json")
SUMMARY_PATH = Path("target/formal-evidence/latest/summary.json")

REQUIRED_TOP_LEVEL_KEYS = [
    "schema_version",
    "generated_at_utc",
    "git_head",
    "dirty_worktree",
    "overall_status",
    "tools",
    "commands",
    "gates",
]

TOOL_NAMES = ["rust", "verus", "kani", "tla"]


def load_json(path: Path):
    """Load JSON file, return None if not found."""
    if not path.exists():
        return None
    with open(path) as f:
        return json.load(f)


def main():
    # Load schema
    schema = load_json(SCHEMA_PATH)
    if schema is None:
        print(f"FAIL: schema not found: {SCHEMA_PATH}")
        return 1

    # Load summary if it exists
    summary = load_json(SUMMARY_PATH)
    if summary is None:
        print(f"FAIL: summary not found: {SUMMARY_PATH}")
        return 1

    # Check required top-level keys
    missing = [key for key in REQUIRED_TOP_LEVEL_KEYS if key not in summary]
    if missing:
        print(f"FAIL: missing keys: {', '.join(missing)}")
        return 1

    # Validate schema_version is integer
    if not isinstance(summary["schema_version"], int):
        print(f"FAIL: schema_version must be integer, got {type(summary['schema_version']).__name__}")
        return 1

    # Validate generated_at_utc is string
    if not isinstance(summary["generated_at_utc"], str):
        print(f"FAIL: generated_at_utc must be string, got {type(summary['generated_at_utc']).__name__}")
        return 1

    # Validate git_head is string
    if not isinstance(summary["git_head"], str):
        print(f"FAIL: git_head must be string, got {type(summary['git_head']).__name__}")
        return 1

    # Validate dirty_worktree is boolean
    if not isinstance(summary["dirty_worktree"], bool):
        print(f"FAIL: dirty_worktree must be boolean, got {type(summary['dirty_worktree']).__name__}")
        return 1

    # Validate overall_status is pass or fail
    if summary["overall_status"] not in ("pass", "fail"):
        print(f"FAIL: overall_status must be 'pass' or 'fail', got '{summary['overall_status']}'")
        return 1

    # Validate tools object contains rust, verus, kani, tla
    if not isinstance(summary.get("tools"), dict):
        print(f"FAIL: tools must be object, got {type(summary.get('tools')).__name__}")
        return 1

    for tool in TOOL_NAMES:
        if tool not in summary["tools"]:
            print(f"FAIL: tools missing required entry: {tool}")
            return 1
        tool_entry = summary["tools"][tool]
        if not isinstance(tool_entry, dict):
            print(f"FAIL: tools.{tool} must be object")
            return 1
        if "version" not in tool_entry:
            print(f"FAIL: tools.{tool} missing 'version'")
            return 1
        if "path" not in tool_entry:
            print(f"FAIL: tools.{tool} missing 'path'")
            return 1

    # Validate commands is array
    if not isinstance(summary.get("commands"), list):
        print(f"FAIL: commands must be array, got {type(summary.get('commands')).__name__}")
        return 1
    if not summary["commands"]:
        print("FAIL: commands must contain at least one executed command")
        return 1

    # Validate gates is array
    if not isinstance(summary.get("gates"), list):
        print(f"FAIL: gates must be array, got {type(summary.get('gates')).__name__}")
        return 1

    # Validate each gate has required fields
    for i, gate in enumerate(summary.get("gates", [])):
        for field in ["name", "status", "artifact_paths", "metrics"]:
            if field not in gate:
                print(f"FAIL: gates[{i}] missing required field: {field}")
                return 1

    print("PASS: all required keys present")
    return 0


if __name__ == "__main__":
    sys.exit(main())
