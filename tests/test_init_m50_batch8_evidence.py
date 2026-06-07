#!/usr/bin/env python3
"""Tests for tools/init_m50_batch8_evidence.py."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
INIT = REPO_ROOT / "tools" / "init_m50_batch8_evidence.py"


def run_init(*args: str, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(INIT), *args],
        cwd=cwd or REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


class InitM50Batch8EvidenceTests(unittest.TestCase):
    def test_default_initializes_skeleton(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = run_init(cwd=Path(tmp))
            evidence_dir = Path(tmp) / "target/formal-evidence/latest/m50-batch8"

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertTrue((evidence_dir / "logs").is_dir())
            self.assertTrue((evidence_dir / "validation").is_dir())
            metadata = json.loads((evidence_dir / "metadata.json").read_text())
            self.assertEqual(metadata["schema"], "m50-batch8-evidence-v1")
            self.assertEqual(metadata["sensor_calibrations"]["knock"]["authority_status"], "disabled")
            self.assertNotIn("channels", metadata["sensor_calibrations"]["knock"])
            self.assertEqual(metadata["sensor_io_map"]["knock_front"]["input_path"], "")
            self.assertEqual(metadata["sensor_io_map"]["knock_rear"]["input_path"], "")
            self.assertIsNotNone(datetime.fromisoformat(metadata["date"].replace("Z", "+00:00")).tzinfo)
            self.assertIn("Created metadata.json", result.stdout)

    def test_existing_metadata_is_not_overwritten(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp) / "evidence"
            evidence_dir.mkdir()
            metadata_path = evidence_dir / "metadata.json"
            metadata_path.write_text('{"sentinel": true}\n', encoding="utf-8")

            result = run_init(str(evidence_dir))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(metadata_path.read_text(encoding="utf-8"), '{"sentinel": true}\n')
            self.assertTrue((evidence_dir / "logs").is_dir())
            self.assertTrue((evidence_dir / "validation").is_dir())
            self.assertIn("Preserved existing metadata.json", result.stdout)

    def test_force_is_refused(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = run_init("--force", str(Path(tmp) / "evidence"))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("--force is unsupported", result.stdout)


if __name__ == "__main__":
    unittest.main()
