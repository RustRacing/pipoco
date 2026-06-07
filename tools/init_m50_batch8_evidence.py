#!/usr/bin/env python3
"""Create a local M50 batch-8 evidence skeleton without overwriting evidence."""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

DEFAULT_EVIDENCE_DIR = Path("target/formal-evidence/latest/m50-batch8")
TEMPLATE_PATH = Path(__file__).with_name("m50_batch8_metadata.example.json")


def utc_now() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Initialize the local M50 batch-8 evidence directory."
    )
    parser.add_argument(
        "evidence_dir",
        nargs="?",
        default=str(DEFAULT_EVIDENCE_DIR),
        help=f"destination directory (default: {DEFAULT_EVIDENCE_DIR})",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="unsupported; metadata is never overwritten by this helper",
    )
    return parser.parse_args(argv)


def starter_metadata() -> str:
    metadata = json.loads(TEMPLATE_PATH.read_text(encoding="utf-8"))
    metadata["date"] = utc_now()
    return json.dumps(metadata, indent=2, sort_keys=True) + "\n"


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.force:
        print("FAIL: --force is unsupported; move metadata.json manually if needed")
        return 1

    evidence_dir = Path(args.evidence_dir)
    metadata_path = evidence_dir / "metadata.json"
    existed = metadata_path.exists()

    if not existed:
        evidence_dir.mkdir(parents=True, exist_ok=True)
        (evidence_dir / "logs").mkdir(exist_ok=True)
        (evidence_dir / "validation").mkdir(exist_ok=True)
        metadata_path.write_text(starter_metadata(), encoding="utf-8")
    else:
        (evidence_dir / "logs").mkdir(parents=True, exist_ok=True)
        (evidence_dir / "validation").mkdir(parents=True, exist_ok=True)

    print(f"Initialized M50 batch-8 evidence skeleton at: {evidence_dir}")
    print("Preserved existing metadata.json." if existed else "Created metadata.json.")
    print("Next:")
    print(f"1. Put real dry-crank CSV at {evidence_dir / 'logs' / 'dry_crank_tooth_cam.csv'}")
    print("2. Replace metadata placeholders and channel mappings.")
    print(f"3. Run: python3 tools/check_m50_batch8_evidence.py {evidence_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
