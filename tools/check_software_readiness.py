#!/usr/bin/env python3
"""Run the host-side software readiness gates.

This is a software-only aggregator. It does not certify wiring, sensor
installation, or timing-light evidence; those remain explicit metadata and
bench artifacts.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class ReadinessCommand:
    name: str
    argv: tuple[str, ...]


def readiness_commands() -> tuple[ReadinessCommand, ...]:
    return (
        ReadinessCommand(
            "profile-target compatibility",
            (
                "cargo",
                "test",
                "-p",
                "ecu-board-profiles",
                "profile_target_compatibility",
            ),
        ),
        ReadinessCommand(
            "simulator readiness aggregation",
            (
                "cargo",
                "test",
                "-p",
                "ecu-sim-driver",
                "software_readiness_report_aggregates",
            ),
        ),
        ReadinessCommand(
            "tuner studio page validity",
            ("cargo", "test", "-p", "ecu-target-common", "--test", "ts_pages_roundtrip"),
        ),
        ReadinessCommand(
            "evidence metadata tooling",
            (
                "python3",
                "-m",
                "unittest",
                "discover",
                "-s",
                "tools/tests",
                "-p",
                "test_*m50_batch8_evidence.py",
            ),
        ),
    )


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--list",
        action="store_true",
        help="print readiness gates without running them",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    commands = readiness_commands()
    if args.list:
        for command in commands:
            print(f"{command.name}: {' '.join(command.argv)}")
        return 0

    for command in commands:
        print(f"[software-readiness] {command.name}")
        subprocess.run(command.argv, check=True)

    print("[software-readiness] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
