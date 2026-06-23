#!/usr/bin/env python3
"""Tests for tools/check_stm32f4_flash_fault_obd_evidence.py."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CHECKER = REPO_ROOT / "tools" / "check_stm32f4_flash_fault_obd_evidence.py"


def run_checker(path: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER), str(path)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def evidence(**overrides) -> dict:
    data = {
        "schema": "stm32f4-flash-fault-obd-evidence-v1",
        "captured_at_utc": "2026-06-19T12:00:00Z",
        "firmware_build_id": "stm32f4-test-fw",
        "firmware_sha256": "a" * 64,
        "board_id": "stm32f405-test-board",
        "probe_id": "stlinkv3-fixture",
        "capture_tool": "socketcan-candump",
        "can_interface": "can0",
        "can_bitrate": 500000,
        "operator_notes": "fixture evidence",
        "fault_injection_method": "fixture forces erase fault during FlashKV rewrite",
        "expected_phase": "erase",
        "expected_sr_bits": 0x000000F2,
        "transcript": [
            {
                "direction": "request",
                "service": 0x09,
                "parameter_id": 0xE0,
                "raw_frame": "02 09 E0 00 00 00 00 00",
            },
            {
                "direction": "response",
                "service": 0x49,
                "parameter_id": 0xE0,
                "raw_frame": "06 49 E0 C0 00 00 00 00",
            },
            {
                "direction": "request",
                "service": 0x09,
                "parameter_id": 0xE2,
                "raw_frame": "02 09 E2 00 00 00 00 00",
            },
            {
                "direction": "response",
                "service": 0x49,
                "parameter_id": 0xE2,
                "raw_frame": "09 49 E2 00 01 06 06 80 02 00 00 00 F2",
            },
        ],
        "discovery_response": {
            "service": 0x49,
            "parameter_id": 0xE0,
            "payload": [0xC0, 0x00, 0x00, 0x00],
        },
        "fault_response": {
            "service": 0x49,
            "parameter_id": 0xE2,
            "sequence_index": 0,
            "segment_count": 1,
            "total_payload_len": 6,
            "segment_len": 6,
            "segment": [0x80, 0x02, 0x00, 0x00, 0x00, 0xF2],
        },
    }
    data.update(overrides)
    return data


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class FlashFaultObdEvidenceCheckerTests(unittest.TestCase):
    def test_accepts_valid_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            write_json(path, evidence())

            result = run_checker(path)

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("PASS:", result.stdout)

    def test_rejects_missing_discovery_bit_for_e2(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            data = evidence()
            data["discovery_response"]["payload"] = [0x80, 0x00, 0x00, 0x00]
            write_json(path, data)

            result = run_checker(path)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("exactly [0xC0,0,0,0]", result.stdout)

    def test_rejects_phase_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            data = evidence(expected_phase="program")
            write_json(path, data)

            result = run_checker(path)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("expected_phase program", result.stdout)

    def test_rejects_sr_without_flash_error_bits(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            data = evidence(expected_sr_bits=0x00010000)
            data["fault_response"]["segment"] = [0x80, 0x02, 0, 0x01, 0, 0]
            write_json(path, data)

            result = run_checker(path)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("FLASH error bit", result.stdout)

    def test_rejects_missing_transcript(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            data = evidence()
            del data["transcript"]
            write_json(path, data)

            result = run_checker(path)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("transcript must contain", result.stdout)

    def test_rejects_transcript_not_matching_decoded_fault_bytes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            data = evidence()
            data["transcript"][3]["raw_frame"] = "09 49 E2 00 01 06 06 80 02 00 00 00 00"
            write_json(path, data)

            result = run_checker(path)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("decoded fault response bytes", result.stdout)

    def test_rejects_overlong_discovery_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "evidence.json"
            data = evidence()
            data["discovery_response"]["payload"] = [0xC0, 0, 0, 0, 0xFF]
            write_json(path, data)

            result = run_checker(path)

        self.assertNotEqual(result.returncode, 0)
        self.assertIn("exactly [0xC0,0,0,0]", result.stdout)


if __name__ == "__main__":
    unittest.main()
