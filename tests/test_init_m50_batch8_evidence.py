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
CHECKER = REPO_ROOT / "tools" / "check_m50_batch8_evidence.py"
TEMPLATE = REPO_ROOT / "tools" / "m50_batch8_metadata.example.json"
DEFAULT_DRY_CRANK_CSV = (
    "target/formal-evidence/latest/m50-batch8/logs/dry_crank_tooth_cam.csv"
)
METADATA_DRY_CRANK_ARTIFACT_PATH = "logs/dry_crank_tooth_cam.csv"
EXPECTED_SCHEMA = "m50-batch8-evidence-v1"


def run_init(*args: str, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(INIT), *args],
        cwd=cwd or REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def run_checker(evidence_dir: Path, cwd: Path | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER), str(evidence_dir)],
        cwd=cwd or REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def assert_iso_utc_timestamp(testcase: unittest.TestCase, value: str) -> None:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    testcase.assertIsNotNone(parsed.tzinfo)


class InitM50Batch8EvidenceTests(unittest.TestCase):
    def test_default_behavior_in_temp_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path(tmp)

            result = run_init(cwd=cwd)

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            evidence_dir = cwd / "target" / "formal-evidence" / "latest" / "m50-batch8"
            self.assertTrue(evidence_dir.is_dir())
            self.assertTrue((evidence_dir / "logs").is_dir())
            self.assertTrue((evidence_dir / "validation").is_dir())
            metadata = read_json(evidence_dir / "metadata.json")
            template = read_json(TEMPLATE)
            self.assertNotEqual(metadata["date"], template["date"])
            assert_iso_utc_timestamp(self, metadata["date"])
            self.assertEqual(
                {key: value for key, value in metadata.items() if key != "date"},
                {key: value for key, value in template.items() if key != "date"},
            )
            self.assertEqual(metadata["schema"], EXPECTED_SCHEMA)
            provenance = metadata["provenance"]
            self.assertIsInstance(provenance, dict)
            self.assertIs(provenance["dry_crank_declared"], True)
            self.assertIs(provenance["spark_disabled"], True)
            self.assertIs(provenance["injectors_disabled"], True)
            self.assertEqual(provenance["sample_rate_hz"], 1_000_000)
            self.assertGreater(provenance["sample_rate_hz"], 0)
            self.assertEqual(
                provenance["channel_mapping"],
                {
                    "crank": "<dry-crank CSV crank header>",
                    "cam": "<dry-crank CSV cam header>",
                },
            )
            self.assertEqual(
                metadata["artifacts"][0],
                {
                    "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                    "kind": "dry_crank_tooth_cam_log",
                },
            )
            self.assertIn("Collect a real dry-crank tooth/cam log", result.stdout)
            self.assertIn(DEFAULT_DRY_CRANK_CSV, result.stdout)
            self.assertIn(
                f"artifacts[].path evidence-dir-relative: {METADATA_DRY_CRANK_ARTIFACT_PATH}",
                result.stdout,
            )
            self.assertIn("Created metadata.json from the starter template", result.stdout)
            self.assertIn(
                "Review required metadata/provenance, CSV header mapping, sha256,",
                result.stdout,
            )
            self.assertIn("certification_hash workflow", result.stdout)
            self.assertIn("structural PASS limits", result.stdout)
            self.assertIn(
                "python3 tools/check_m50_batch8_evidence.py --print-template",
                result.stdout,
            )
            self.assertIn("python3 tools/check_m50_batch8_evidence.py", result.stdout)

    def test_existing_metadata_is_preserved_and_missing_dirs_are_created(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp) / "evidence"
            evidence_dir.mkdir()
            metadata_path = evidence_dir / "metadata.json"
            metadata_path.write_text("{\"sentinel\": true}\n", encoding="utf-8")

            result = run_init(str(evidence_dir))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(metadata_path.read_text(encoding="utf-8"), "{\"sentinel\": true}\n")
            self.assertTrue((evidence_dir / "logs").is_dir())
            self.assertTrue((evidence_dir / "validation").is_dir())
            self.assertIn("Preserved existing metadata.json", result.stdout)

    def test_existing_dry_crank_log_is_preserved_when_metadata_is_created(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp) / "evidence"
            dry_crank_path = evidence_dir / "logs" / "dry_crank_tooth_cam.csv"
            dry_crank_path.parent.mkdir(parents=True)
            dry_crank_log = "time_us,crank_signal,cam_signal\n1,1,0\n"
            dry_crank_path.write_text(dry_crank_log, encoding="utf-8")

            result = run_init(str(evidence_dir))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertEqual(dry_crank_path.read_text(encoding="utf-8"), dry_crank_log)
            self.assertTrue((evidence_dir / "metadata.json").is_file())
            self.assertIn("Created metadata.json from the starter template", result.stdout)

    def test_force_refuses_to_overwrite_existing_metadata_or_logs(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp) / "evidence"
            dry_crank_path = evidence_dir / "logs" / "dry_crank_tooth_cam.csv"
            dry_crank_path.parent.mkdir(parents=True)
            dry_crank_log = "time_us,crank_signal,cam_signal\n1,1,0\n"
            dry_crank_path.write_text(dry_crank_log, encoding="utf-8")
            metadata_path = evidence_dir / "metadata.json"
            metadata_path.write_text("{\"sentinel\": true}\n", encoding="utf-8")

            result = run_init("--force", str(evidence_dir))

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("--force is intentionally unsupported", result.stdout)
            self.assertEqual(metadata_path.read_text(encoding="utf-8"), "{\"sentinel\": true}\n")
            self.assertEqual(dry_crank_path.read_text(encoding="utf-8"), dry_crank_log)

    def test_initialized_package_still_fails_checker_without_real_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            cwd = Path(tmp)

            init_result = run_init(cwd=cwd)
            self.assertEqual(init_result.returncode, 0, init_result.stdout + init_result.stderr)

            evidence_dir = cwd / "target" / "formal-evidence" / "latest" / "m50-batch8"
            check_result = run_checker(evidence_dir, cwd=cwd)

            self.assertNotEqual(check_result.returncode, 0, check_result.stdout)
            self.assertIn("missing dry-crank tooth/cam log", check_result.stdout)


if __name__ == "__main__":
    unittest.main()
