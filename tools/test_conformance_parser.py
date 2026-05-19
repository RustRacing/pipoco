#!/usr/bin/env python3
"""Regression fixture for conformance-map parser (US-FM0602).

Verifies the parser handles:
1. adapter-contract rows emit conformance=adapter-contract
2. covered rows emit conformance=covered
3. tolerance is emitted separately from status
4. unknown statuses fail
"""

import json
import sys
import re
from pathlib import Path

import jsonschema


def parse_conformance_row(line: str, current_section: str = "") -> dict | None:
    """Parse a single markdown table row from the conformance map."""
    line = line.strip()
    if not line.startswith("|") or line.startswith("| ---"):
        return None
    parts = [p.strip() for p in line.split("|")]
    if len(parts) < 5 or not parts[1].strip():
        return None
    field = parts[1].strip()
    if (
        not field
        or field in ("Field", "Frozen Contract", "Oracle Field / Contract")
        or re.match(r"^-+$", field)
    ):
        return None
    reducer = parts[4].strip() if len(parts) > 4 else ""
    tolerance = parts[5].strip() if len(parts) > 5 else ""
    status = parts[6].strip() if len(parts) > 6 else ""
    blocker = parts[7].strip() if len(parts) > 7 else ""
    owner = parts[2].strip() if len(parts) > 2 else ""
    artifact = parts[3].strip() if len(parts) > 3 else ""
    status_lower = status.lower()
    if status_lower not in {"covered", "adapter-contract"}:
        raise ValueError(f"invalid conformance status for {field}: {status!r}")
    is_adapter = status_lower == "adapter-contract"
    contract_refs: list[str] = []
    row_text = line
    for enum_name, group in re.findall(
        r"\b(RuntimeAdapterContract|SchedulerAdapterContract|CoreAdapterContract|BoardAdapterContract|TargetCommonAdapterContract)::\{([^}]+)\}",
        row_text,
    ):
        for variant in group.split(","):
            variant = variant.strip()
            if variant:
                contract_refs.append(f"{enum_name}::{variant}")
    return {
        "field": field,
        "section": current_section,
        "owner": owner,
        "artifact_path": artifact,
        "reducer_path": reducer,
        "tolerance": tolerance,
        "status": status,
        "blocker": blocker,
        "conformance": "adapter-contract" if is_adapter else "covered",
        "contract_refs": contract_refs,
    }


def parse_conformance_table(table_text: str) -> list[dict]:
    """Parse a full markdown table and return list of conformance rows."""
    rows = []
    for line in table_text.splitlines():
        row = parse_conformance_row(line)
        if row is not None:
            rows.append(row)
    return rows


SYNTHETIC_TABLE = """
| Field | Owner | Observed Path | Reducer / Test File | Tolerance | Status | Blocker |
|---|---|---|---|---|---|---|
| ts-ec::f32::add | @user | target/ts-ecu/src/lib.rs | target/ts-ecu/src/lib.rs::ts_ec_f32_add | 0.0 | covered | |
| ts-ec::uart::write_byte | @user | target/ts-ecu/src/lib.rs | RuntimeAdapterContract::{UartWriteByte} | 0.0 | adapter-contract | |
""".strip()


def main() -> int:
    all_passed = True

    # Test 1: adapter-contract rows emit conformance=adapter-contract
    rows = parse_conformance_table(SYNTHETIC_TABLE)
    adapter_rows = [r for r in rows if r["status"].lower() == "adapter-contract"]
    if not adapter_rows:
        print("FAIL: no adapter-contract rows found", file=sys.stderr)
        all_passed = False
    elif any(r["conformance"] != "adapter-contract" for r in adapter_rows):
        print("FAIL: adapter-contract row did not emit conformance=adapter-contract", file=sys.stderr)
        all_passed = False
    else:
        print("PASS: adapter-contract rows emit conformance=adapter-contract")

    # Test 2: covered rows emit conformance=covered
    covered_rows = [r for r in rows if r["status"].lower() == "covered"]
    if not covered_rows:
        print("FAIL: no covered rows found", file=sys.stderr)
        all_passed = False
    elif any(r["conformance"] != "covered" for r in covered_rows):
        print("FAIL: covered row did not emit conformance=covered", file=sys.stderr)
        all_passed = False
    else:
        print("PASS: covered rows emit conformance=covered")

    # Test 3: tolerance is emitted separately from status
    for row in rows:
        if not row["tolerance"]:
            print(f"FAIL: row {row['field']} has empty tolerance", file=sys.stderr)
            all_passed = False
        elif row["tolerance"] == row["status"]:
            print(f"FAIL: row {row['field']}: tolerance == status (columns may have shifted)", file=sys.stderr)
            all_passed = False
        else:
            print(f"PASS: row {row['field']}: tolerance={row['tolerance']!r} != status={row['status']!r}")
    # Also verify all rows have both fields populated
    for row in rows:
        if not row["tolerance"]:
            all_passed = False
    if all(r["tolerance"] and r["status"] for r in rows):
        print("PASS: tolerance is emitted separately from status")

    # Test 4: unknown statuses fail
    bad_table = """
| Field | Owner | Observed Path | Reducer / Test File | Tolerance | Status | Blocker |
|---|---|---|---|---|---|---|
| ts-ec::bad | @user | target/ts-ecu/src/lib.rs | target/ts-ecu/src/lib.rs | 0.0 | unknown-status | |
"""
    try:
        parse_conformance_table(bad_table)
        print("FAIL: unknown status did not raise an exception", file=sys.stderr)
        all_passed = False
    except ValueError as e:
        if "unknown-status" in str(e):
            print(f"PASS: unknown status fails with: {e}")
        else:
            print(f"FAIL: wrong error message: {e}", file=sys.stderr)
            all_passed = False

    if all_passed:
        schema = json.loads(Path("tools/conformance.schema.json").read_text())
        try:
            jsonschema.validate(
                {"status": "pass", "total": len(rows), "rows": rows},
                schema,
            )
            print("PASS: parsed rows validate against conformance.schema.json")
        except jsonschema.ValidationError as e:
            print(f"FAIL: parsed rows do not validate: {e}", file=sys.stderr)
            return 1

        print("\nAll tests passed.")
        return 0
    else:
        print("\nSome tests FAILED.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
