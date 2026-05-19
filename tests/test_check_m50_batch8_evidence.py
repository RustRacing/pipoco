#!/usr/bin/env python3
"""Tests for tools/check_m50_batch8_evidence.py."""

from __future__ import annotations

import json
import hashlib
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
INIT = REPO_ROOT / "tools" / "init_m50_batch8_evidence.py"
CHECKER = REPO_ROOT / "tools" / "check_m50_batch8_evidence.py"
DEFAULT_DRY_CRANK_CSV = (
    "target/formal-evidence/latest/m50-batch8/logs/dry_crank_tooth_cam.csv"
)
METADATA_DRY_CRANK_ARTIFACT_PATH = "logs/dry_crank_tooth_cam.csv"
EXPECTED_SCHEMA = "m50-batch8-evidence-v1"
REVIEWABLE_DRY_CRANK_CSV = """\
# sample_rate_hz=1000000 clock_source=logic-analyzer
# crank_edge_polarity=rising cam_phase=tooth_1
time_us,crank_tooth_edge,cam_phase_state
0,0,0
100,rising,1
"""
CERTIFICATION_HASH_FIELDS = (
    "schema",
    "date",
    "board_revision",
    "firmware_build_id",
    "profile_id",
    "conditioner_path",
    "primary_edge",
    "secondary_edge",
    "trigger_angle_atdc_deg10",
    "fixed_timing_mode",
    "fixed_timing_angle_deg10",
    "operator_notes",
    "source_notes",
    "provenance",
)


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str = "") -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def write_reviewable_dry_crank_csv(evidence_dir: Path) -> Path:
    dry_crank = evidence_dir / METADATA_DRY_CRANK_ARTIFACT_PATH
    write_text(dry_crank, REVIEWABLE_DRY_CRANK_CSV)
    return dry_crank


def file_sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def certification_hash(metadata: dict, artifacts: list[dict[str, str]]) -> str:
    payload = {
        "metadata": {
            field: metadata.get(field)
            for field in CERTIFICATION_HASH_FIELDS
        },
        "artifacts": sorted(
            artifacts,
            key=lambda entry: (entry["kind"], entry["path"], entry["sha256"]),
        ),
    }
    return hashlib.sha256(
        json.dumps(
            payload,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()


def make_certified_metadata(evidence_dir: Path, **overrides) -> dict:
    dry_crank = write_reviewable_dry_crank_csv(evidence_dir)
    fixed_timing_path = evidence_dir / "validation" / "fixed_timing_validation.csv"
    write_text(fixed_timing_path, "fixed timing validation fixture\n")
    artifacts = [
        {
            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
            "kind": "dry_crank_tooth_cam_log",
            "sha256": file_sha256(dry_crank),
        },
        {
            "path": "validation/fixed_timing_validation.csv",
            "kind": "fixed_timing_validation_artifact",
            "sha256": file_sha256(fixed_timing_path),
        },
    ]
    metadata = make_metadata(
        fixed_timing_mode="certified",
        fixed_timing_angle_deg10=100,
        artifacts=artifacts,
    )
    metadata["certification_hash"] = certification_hash(metadata, artifacts)
    metadata.update(overrides)
    return metadata


def make_provenance(**overrides) -> dict:
    provenance = {
        "capture_timestamp": "2026-05-18T12:01:00Z",
        "operator_identity": "test operator",
        "capture_device": "logic analyzer fixture",
        "channel_mapping": {
            "crank": "crank_tooth_edge",
            "cam": "cam_phase_state",
        },
        "sample_rate_hz": 1_000_000,
        "clock_source": "logic-analyzer internal clock",
        "dry_crank_declared": True,
        "spark_disabled": True,
        "injectors_disabled": True,
        "cranking_rpm_range": "180-220 rpm",
        "battery_voltage": "12.1 V during crank",
        "environment_notes": "ambient 22 C, coolant 20 C",
        "source_notes": "synthetic review fixture provenance",
    }
    provenance.update(overrides)
    return provenance


def make_metadata(*, artifacts: list[dict] | None = None, **overrides) -> dict:
    metadata = {
        "schema": EXPECTED_SCHEMA,
        "date": "2026-05-18T12:00:00Z",
        "board_revision": "M50 batch-8 board rev A",
        "firmware_build_id": "fw-build-2026-05-18",
        "profile_id": "m50-batch-8-review",
        "conditioner_path": "docs/conditioning/m50-batch8.md",
        "primary_edge": "rising",
        "secondary_edge": "falling",
        "trigger_angle_atdc_deg10": 120,
        "fixed_timing_mode": "not_yet_certified",
        "fixed_timing_angle_deg10": None,
        "operator_notes": "synthetic review fixture",
        "source_notes": "synthetic review fixture",
        "provenance": make_provenance(),
        "artifacts": artifacts
        if artifacts is not None
        else [
            {
                "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                "kind": "dry_crank_tooth_cam_log",
            }
        ],
    }
    metadata.update(overrides)
    return metadata


def run_checker(evidence_dir: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER), str(evidence_dir)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def run_checker_cli(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER), *args],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def run_init(evidence_dir: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(INIT), str(evidence_dir)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


class CheckM50Batch8EvidenceTests(unittest.TestCase):
    def test_incomplete_evidence_fails_closed_with_missing_field_output(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            metadata = make_metadata()
            del metadata["date"]
            del metadata["firmware_build_id"]
            write_json(evidence_dir / "metadata.json", metadata)

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: metadata missing required field: date", result.stdout)
            self.assertIn(
                "FAIL: metadata missing required field: firmware_build_id",
                result.stdout,
            )

    def test_schema_version_is_required_and_exact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)

            for schema_value in (None, "m50-batch8-evidence-v2"):
                with self.subTest(schema=schema_value):
                    metadata = make_metadata()
                    if schema_value is None:
                        del metadata["schema"]
                    else:
                        metadata["schema"] = schema_value
                    write_json(evidence_dir / "metadata.json", metadata)

                    result = run_checker(evidence_dir)

                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn(
                        f"FAIL: metadata.schema must be '{EXPECTED_SCHEMA}'",
                        result.stdout,
                    )

    def test_malformed_metadata_json_fails_without_traceback(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "metadata.json", "{not json\n")

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: metadata.json is not valid JSON", result.stdout)
            self.assertNotIn("Traceback", result.stderr)

        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "metadata.json", "[]\n")

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: metadata.json must contain a JSON object", result.stdout)
            self.assertNotIn("Traceback", result.stderr)

    def test_artifacts_shape_is_validated(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(evidence_dir / "metadata.json", make_metadata(artifacts={}))

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: metadata.artifacts must be a list", result.stdout)
            self.assertNotIn("Traceback", result.stderr)

        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(evidence_dir / "metadata.json", make_metadata(artifacts=["log.csv"]))

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: metadata.artifacts[0] must be an object", result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts must contain at least one artifact entry",
                result.stdout,
            )
            self.assertNotIn("Traceback", result.stderr)

    def test_required_metadata_string_fields_reject_wrong_primitive_types(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    date=123,
                    board_revision=["M50"],
                    firmware_build_id={"id": "fw"},
                    profile_id=False,
                    conditioner_path=0,
                    operator_notes=None,
                    source_notes=["fixture"],
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            for field in (
                "date",
                "board_revision",
                "firmware_build_id",
                "profile_id",
                "conditioner_path",
                "operator_notes",
                "source_notes",
            ):
                self.assertIn(
                    f"FAIL: metadata.{field} must be a non-empty string",
                    result.stdout,
                )
            self.assertNotIn("Traceback", result.stderr)

    def test_structured_provenance_object_is_required(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            metadata = make_metadata()
            del metadata["provenance"]
            write_json(evidence_dir / "metadata.json", metadata)

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: metadata missing required field: provenance", result.stdout)
            self.assertIn("FAIL: metadata.provenance must be an object", result.stdout)

    def test_structured_provenance_fields_are_reviewable(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    provenance=make_provenance(
                        capture_timestamp="2026-05-18 12:01:00",
                        operator_identity=" ",
                        capture_device="<logic analyzer>",
                        channel_mapping={
                            "crank": "",
                            "cam": "<cam channel>",
                        },
                        sample_rate_hz=0,
                        dry_crank_declared=False,
                        spark_disabled=False,
                        injectors_disabled=False,
                    )
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.provenance.capture_timestamp must include timezone information",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.operator_identity must be a non-empty string",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.capture_device contains unresolved placeholder token: <logic analyzer>",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.channel_mapping.crank must be a non-empty string",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.channel_mapping.cam contains unresolved placeholder token: <cam channel>",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.sample_rate_hz must be a positive integer",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.dry_crank_declared must be true",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.spark_disabled must be true",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.provenance.injectors_disabled must be true",
                result.stdout,
            )

    def test_selected_edges_must_be_known_edge_values(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    primary_edge="leading",
                    secondary_edge="cam-high",
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.primary_edge must be one of: falling, rising",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.secondary_edge must be one of: falling, rising",
                result.stdout,
            )

    def test_trigger_angles_must_be_integer_deg10_values(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    trigger_angle_atdc_deg10=12.5,
                    fixed_timing_angle_deg10=True,
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.trigger_angle_atdc_deg10 must be an integer tenths-of-degree value",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.fixed_timing_angle_deg10 must be an integer tenths-of-degree value or null",
                result.stdout,
            )

    def test_msq_alone_is_insufficient(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "tune.msq", "[Tune]\n")

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn("FAIL: missing metadata.json", result.stdout)

    def test_initialized_metadata_fails_closed_without_date_error(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp) / "evidence"

            init_result = run_init(evidence_dir)
            self.assertEqual(init_result.returncode, 0, init_result.stdout + init_result.stderr)

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertNotIn("metadata.date must be ISO-8601", result.stdout)
            self.assertNotIn("metadata missing required field: date", result.stdout)
            self.assertIn("missing dry-crank tooth/cam log", result.stdout)

    def test_placeholder_metadata_values_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    board_revision="<M50 board revision>",
                    firmware_build_id="<firmware build id>",
                    source_notes="capture provenance: <source notes>",
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.board_revision contains unresolved placeholder token: <M50 board revision>",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.firmware_build_id contains unresolved placeholder token: <firmware build id>",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.source_notes contains unresolved placeholder token: capture provenance: <source notes>",
                result.stdout,
            )

    def test_placeholder_artifact_path_cannot_satisfy_dry_crank_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "<dry_crank_tooth_cam_log.csv>", "synthetic log\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": "<dry_crank_tooth_cam_log.csv>",
                            "kind": "dry_crank_tooth_cam_log",
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0].path contains unresolved placeholder token: <dry_crank_tooth_cam_log.csv>",
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_absolute_artifact_path_cannot_satisfy_dry_crank_log(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            dry_crank = write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": str(dry_crank),
                            "kind": "dry_crank_tooth_cam_log",
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                f"FAIL: metadata.artifacts[0].path must be relative to the evidence dir: {dry_crank}",
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_duplicate_artifact_paths_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[1].path duplicates metadata.artifacts[0].path: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )

    def test_unknown_artifact_kind_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            dry_crank = write_reviewable_dry_crank_csv(evidence_dir)
            supplemental = evidence_dir / "validation" / "operator_notes.csv"
            write_text(supplemental, "time_us,note\n0,reviewed\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": file_sha256(dry_crank),
                        },
                        {
                            "path": "validation/operator_notes.csv",
                            "kind": "operator_notes",
                            "sha256": file_sha256(supplemental),
                        },
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[1].kind must be one of: "
                "dry_crank_tooth_cam_log, fixed_timing_validation_artifact",
                result.stdout,
            )

    def test_duplicate_singleton_artifact_kinds_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, duplicate_artifacts, expected_failure in (
                (
                    "dry-crank",
                    [
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "logs/dry_crank_tooth_cam_secondary.csv",
                            "kind": "dry_crank_tooth_cam_log",
                        },
                    ],
                    "FAIL: metadata.artifacts[1].kind duplicates singleton evidence kind "
                    "metadata.artifacts[0].kind: dry_crank_tooth_cam_log",
                ),
                (
                    "fixed-timing",
                    [
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "validation/fixed_timing_primary.csv",
                            "kind": "fixed_timing_validation_artifact",
                        },
                        {
                            "path": "validation/fixed_timing_secondary.csv",
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ],
                    "FAIL: metadata.artifacts[2].kind duplicates singleton evidence kind "
                    "metadata.artifacts[1].kind: fixed_timing_validation_artifact",
                ),
            ):
                with self.subTest(name=name):
                    evidence_dir = root / name
                    write_reviewable_dry_crank_csv(evidence_dir)
                    write_text(
                        evidence_dir / "logs" / "dry_crank_tooth_cam_secondary.csv",
                        REVIEWABLE_DRY_CRANK_CSV,
                    )
                    write_text(
                        evidence_dir / "validation" / "fixed_timing_primary.csv",
                        "fixed timing primary fixture\n",
                    )
                    write_text(
                        evidence_dir / "validation" / "fixed_timing_secondary.csv",
                        "fixed timing secondary fixture\n",
                    )
                    write_json(
                        evidence_dir / "metadata.json",
                        make_metadata(artifacts=duplicate_artifacts),
                    )

                    result = run_checker(evidence_dir)

                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn(expected_failure, result.stdout)

    def test_declared_artifact_sha256_must_match_file_contents(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": "0" * 64,
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0].sha256 does not match file contents: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_traversal_artifact_path_cannot_escape_evidence_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            evidence_dir = root / "evidence"
            write_text(root / "outside.csv", "synthetic log\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": "logs/../../outside.csv",
                            "kind": "dry_crank_tooth_cam_log",
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] path escapes evidence dir: logs/../../outside.csv",
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_symlink_artifact_path_cannot_escape_evidence_dir(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            evidence_dir = root / "evidence"
            outside = root / "outside" / "dry_crank_tooth_cam.csv"
            link = evidence_dir / "logs" / "dry_crank_tooth_cam.csv"
            write_text(outside, "synthetic log\n")
            link.parent.mkdir(parents=True, exist_ok=True)
            try:
                link.symlink_to(outside)
            except (NotImplementedError, OSError) as exc:
                self.skipTest(f"symlinks unsupported: {exc}")
            write_json(evidence_dir / "metadata.json", make_metadata())

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] path escapes evidence dir: logs/dry_crank_tooth_cam.csv",
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_generic_mlg_without_metadata_binding_is_insufficient(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "logs" / "session.mlg", "synthetic log\n")
            write_text(evidence_dir / "logs" / "fixed_timing.csv", "synthetic proof\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": "logs/fixed_timing.csv",
                            "kind": "fixed_timing_validation_artifact",
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: missing dry-crank tooth/cam log: collect a real dry-crank tooth/cam log and add at least one metadata.artifacts entry with kind dry_crank_tooth_cam_log",
                result.stdout,
            )
            self.assertIn(
                "FAIL: forbidden standalone tune/log file cannot satisfy batch-8 evidence: logs/session.mlg",
                result.stdout,
            )

    def test_invalid_fixed_timing_mode_is_rejected_clearly(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_text(
                evidence_dir / "validation" / "fixed_timing_validation.csv",
                "fixed timing validation fixture\n",
            )
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    fixed_timing_mode="claimed",
                    fixed_timing_angle_deg10=100,
                    artifacts=[
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "validation/fixed_timing_validation.csv",
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ],
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.fixed_timing_mode must be 'certified' or 'not_yet_certified'",
                result.stdout,
            )
            self.assertNotIn("missing fixed timing proof", result.stdout)

    def test_certified_fixed_timing_requires_validation_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    fixed_timing_mode="certified",
                    fixed_timing_angle_deg10=100,
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: missing fixed timing proof: collect a real fixed-timing validation artifact",
                result.stdout,
            )
            self.assertIn(
                "FAIL: fixed_timing_mode is certified but no fixed_timing_validation_artifact was provided",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.certification_hash is required when fixed_timing_mode is certified",
                result.stdout,
            )

    def test_certified_fixed_timing_missing_hash_prints_expected_hash(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            metadata = make_certified_metadata(evidence_dir)
            expected_hash = metadata["certification_hash"]
            metadata["certification_hash"] = None
            write_json(evidence_dir / "metadata.json", metadata)

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.certification_hash is required when fixed_timing_mode "
                f"is certified; expected {expected_hash}",
                result.stdout,
            )

    def test_certified_fixed_timing_rejects_unusable_validation_artifact_paths(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, path_text, expected_failure in (
                (
                    "placeholder",
                    "<fixed_timing_validation.csv>",
                    "FAIL: metadata.artifacts[1].path contains unresolved placeholder token: <fixed_timing_validation.csv>",
                ),
                (
                    "absolute",
                    str(root / "evidence-absolute" / "validation" / "fixed_timing.csv"),
                    "FAIL: metadata.artifacts[1].path must be relative to the evidence dir: "
                    + str(root / "evidence-absolute" / "validation" / "fixed_timing.csv"),
                ),
                (
                    "missing",
                    "validation/missing_fixed_timing.csv",
                    "FAIL: metadata.artifacts[1] file does not exist: validation/missing_fixed_timing.csv",
                ),
            ):
                with self.subTest(name=name):
                    evidence_dir = root / name
                    write_reviewable_dry_crank_csv(evidence_dir)
                    if name == "absolute":
                        write_text(Path(path_text), "fixed timing validation fixture\n")
                    write_json(
                        evidence_dir / "metadata.json",
                        make_metadata(
                            fixed_timing_mode="certified",
                            fixed_timing_angle_deg10=100,
                            artifacts=[
                                {
                                    "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                                    "kind": "dry_crank_tooth_cam_log",
                                },
                                {
                                    "path": path_text,
                                    "kind": "fixed_timing_validation_artifact",
                                },
                            ],
                        ),
                    )

                    result = run_checker(evidence_dir)

                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn(expected_failure, result.stdout)
                    self.assertIn(
                        "FAIL: fixed_timing_mode is certified but no fixed_timing_validation_artifact was provided",
                        result.stdout,
                    )

    def test_certified_fixed_timing_requires_artifact_sha256_values(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            dry_crank = write_reviewable_dry_crank_csv(evidence_dir)
            fixed_timing = evidence_dir / "validation" / "fixed_timing_validation.csv"
            write_text(
                fixed_timing,
                "fixed timing validation fixture\n",
            )
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    fixed_timing_mode="certified",
                    fixed_timing_angle_deg10=100,
                    certification_hash="0" * 64,
                    artifacts=[
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "validation/fixed_timing_validation.csv",
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ],
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0].sha256 is required when fixed_timing_mode "
                "is certified or certification_hash is declared",
                result.stdout,
            )
            self.assertIn(
                f"{METADATA_DRY_CRANK_ARTIFACT_PATH}; actual {file_sha256(dry_crank)}",
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.artifacts[1].sha256 is required when fixed_timing_mode "
                "is certified or certification_hash is declared",
                result.stdout,
            )
            self.assertIn(
                f"validation/fixed_timing_validation.csv; actual {file_sha256(fixed_timing)}",
                result.stdout,
            )

    def test_certification_hash_binds_edges_trigger_angle_and_artifact_hashes(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for name, override in (
                ("primary_edge", {"primary_edge": "falling"}),
                ("trigger_angle", {"trigger_angle_atdc_deg10": 130}),
            ):
                with self.subTest(name=name):
                    evidence_dir = root / name
                    metadata = make_certified_metadata(evidence_dir)
                    stale_hash = metadata["certification_hash"]
                    metadata.update(override)
                    metadata["certification_hash"] = stale_hash
                    expected_hash = certification_hash(metadata, metadata["artifacts"])
                    write_json(evidence_dir / "metadata.json", metadata)

                    result = run_checker(evidence_dir)

                    self.assertNotEqual(result.returncode, 0, result.stdout)
                    self.assertIn(
                        "FAIL: metadata.certification_hash does not match certification inputs",
                        result.stdout,
                    )
                    self.assertIn(f"expected {expected_hash}", result.stdout)

    def test_complete_certified_evidence_with_hashes_passes_structurally(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_json(
                evidence_dir / "metadata.json",
                make_certified_metadata(evidence_dir),
            )

            result = run_checker(evidence_dir)

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(
                "PASS: batch-8 evidence is structurally reviewable",
                result.stdout,
            )
            self.assertIn(
                "fixed_timing_mode=certified package is structurally complete",
                result.stdout,
            )
            self.assertIn(
                "profile certification and runtime full-COP authority still require separate review",
                result.stdout,
            )

    def test_bound_mlg_does_not_satisfy_dry_crank_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "logs" / "dry_crank_tooth_cam.mlg", "synthetic log\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.mlg",
                            "kind": "dry_crank_tooth_cam_log",
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] uses forbidden tune/log file extension and cannot satisfy dry_crank_tooth_cam_log: logs/dry_crank_tooth_cam.mlg; provide a reviewable evidence artifact instead of binding .msq or .mlg files",
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_bound_mlg_does_not_satisfy_fixed_timing_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_text(evidence_dir / "logs" / "fixed_timing.mlg", "synthetic proof\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    fixed_timing_mode="certified",
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "logs/fixed_timing.mlg",
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ],
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[1] uses forbidden tune/log file extension and cannot satisfy fixed_timing_validation_artifact: logs/fixed_timing.mlg; provide a reviewable evidence artifact instead of binding .msq or .mlg files",
                result.stdout,
            )
            self.assertIn(
                "FAIL: fixed_timing_mode is certified but no fixed_timing_validation_artifact was provided",
                result.stdout,
            )

    def test_bound_msq_does_not_satisfy_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / "logs" / "dry_crank_tooth_cam.msq", "[Tune]\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.msq",
                            "kind": "dry_crank_tooth_cam_log",
                        }
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] uses forbidden tune/log file extension and cannot satisfy dry_crank_tooth_cam_log: logs/dry_crank_tooth_cam.msq; provide a reviewable evidence artifact instead of binding .msq or .mlg files",
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_bound_msq_does_not_satisfy_fixed_timing_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_text(evidence_dir / "logs" / "fixed_timing.msq", "[Tune]\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    fixed_timing_mode="certified",
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "logs/fixed_timing.msq",
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ],
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[1] uses forbidden tune/log file extension and cannot satisfy fixed_timing_validation_artifact: logs/fixed_timing.msq; provide a reviewable evidence artifact instead of binding .msq or .mlg files",
                result.stdout,
            )
            self.assertIn(
                "FAIL: fixed_timing_mode is certified but no fixed_timing_validation_artifact was provided",
                result.stdout,
            )

    def test_legacy_file_is_rejected_even_for_supplemental_like_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            dry_crank = write_reviewable_dry_crank_csv(evidence_dir)
            legacy_log = evidence_dir / "logs" / "supplemental_capture.mlg"
            write_text(legacy_log, "synthetic supplemental log\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    certification_hash="0" * 64,
                    artifacts=[
                        {
                            "path": METADATA_DRY_CRANK_ARTIFACT_PATH,
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": file_sha256(dry_crank),
                        },
                        {
                            "path": "logs/supplemental_capture.mlg",
                            "kind": "supplemental_capture_notes",
                            "sha256": file_sha256(legacy_log),
                        },
                    ],
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[1] uses forbidden tune/log file extension and cannot satisfy supplemental_capture_notes: logs/supplemental_capture.mlg; provide a reviewable evidence artifact instead of binding .msq or .mlg files",
                result.stdout,
            )

    def test_complete_synthetic_evidence_passes_as_reviewable_only(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(evidence_dir / "metadata.json", make_metadata())

            result = run_checker(evidence_dir)

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn(
                "PASS: batch-8 evidence is structurally reviewable",
                result.stdout,
            )
            self.assertIn("structural review only", result.stdout)
            self.assertIn("fixed timing is not certified", result.stdout)
            self.assertIn("no profile certification", result.stdout)
            self.assertIn("runtime full-COP authority", result.stdout)

    def test_empty_dry_crank_csv_cannot_satisfy_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / METADATA_DRY_CRANK_ARTIFACT_PATH, "")
            write_json(evidence_dir / "metadata.json", make_metadata())

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] dry-crank tooth/cam CSV must be non-empty: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_whitespace_dry_crank_csv_cannot_satisfy_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(evidence_dir / METADATA_DRY_CRANK_ARTIFACT_PATH, " \n\t\n")
            write_json(evidence_dir / "metadata.json", make_metadata())

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] dry-crank tooth/cam CSV must be non-empty: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_headerless_dry_crank_csv_cannot_satisfy_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(
                evidence_dir / METADATA_DRY_CRANK_ARTIFACT_PATH,
                "0,0,0\n100,1,0\n",
            )
            write_json(evidence_dir / "metadata.json", make_metadata())

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] dry-crank tooth/cam CSV must include a descriptive header row: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_dry_crank_csv_missing_required_columns_cannot_satisfy_artifact(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_text(
                evidence_dir / METADATA_DRY_CRANK_ARTIFACT_PATH,
                "time_us,voltage\n0,12.1\n",
            )
            write_json(evidence_dir / "metadata.json", make_metadata())

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] dry-crank tooth/cam CSV header must include "
                "timestamp/sample-time units, crank signal, and cam signal columns: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_dry_crank_csv_channel_mapping_must_match_header_columns(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp)
            write_reviewable_dry_crank_csv(evidence_dir)
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    provenance=make_provenance(
                        channel_mapping={
                            "crank": "logic_analyzer_ch0",
                            "cam": "logic_analyzer_ch1",
                        }
                    )
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[0] dry-crank tooth/cam CSV header is missing "
                "metadata.provenance.channel_mapping.crank column: logic_analyzer_ch0: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn(
                "FAIL: metadata.artifacts[0] dry-crank tooth/cam CSV header is missing "
                "metadata.provenance.channel_mapping.cam column: logic_analyzer_ch1: "
                + METADATA_DRY_CRANK_ARTIFACT_PATH,
                result.stdout,
            )
            self.assertIn("FAIL: missing dry-crank tooth/cam log", result.stdout)

    def test_path_escape_artifacts_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence_dir = Path(tmp) / "evidence"
            evidence_dir.mkdir()
            write_reviewable_dry_crank_csv(evidence_dir)
            write_text(evidence_dir.parent / "outside.csv", "escaped artifact\n")
            write_json(
                evidence_dir / "metadata.json",
                make_metadata(
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                        },
                        {
                            "path": "../outside.csv",
                            "kind": "fixed_timing_validation_artifact",
                        },
                    ]
                ),
            )

            result = run_checker(evidence_dir)

            self.assertNotEqual(result.returncode, 0, result.stdout)
            self.assertIn(
                "FAIL: metadata.artifacts[1] path escapes evidence dir: ../outside.csv",
                result.stdout,
            )

    def test_help_mentions_print_template(self) -> None:
        result = run_checker_cli("--help")

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("--print-template", result.stdout)
        self.assertIn("Pass means reviewable, not", result.stdout)

    def test_print_template_shows_required_collection_checklist(self) -> None:
        result = run_checker_cli("--print-template")

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("Structural review only", result.stdout)
        self.assertIn(
            "A structural PASS does not grant profile certification or runtime full-COP authority",
            result.stdout,
        )
        self.assertIn("Real certification requires a fixed_timing_validation_artifact", result.stdout)
        self.assertIn("- dry-crank tooth/cam log", result.stdout)
        self.assertIn(DEFAULT_DRY_CRANK_CSV, result.stdout)
        self.assertIn(
            f"metadata artifact path: {METADATA_DRY_CRANK_ARTIFACT_PATH}",
            result.stdout,
        )
        self.assertIn(
            f'"path": "{METADATA_DRY_CRANK_ARTIFACT_PATH}"',
            result.stdout,
        )
        self.assertIn("- fixed-timing validation artifact", result.stdout)
        self.assertIn('"fixed_timing_mode": "not_yet_certified"', result.stdout)
        self.assertIn(f'"schema": "{EXPECTED_SCHEMA}"', result.stdout)
        self.assertIn(f"- schema: {EXPECTED_SCHEMA}", result.stdout)
        self.assertIn('"board_revision": "<M50 board revision>"', result.stdout)
        self.assertIn('"source_notes": "<source notes for the local hardware run and evidence provenance>"', result.stdout)
        self.assertIn("- provenance object with capture timestamp", result.stdout)
        self.assertIn('"provenance": {', result.stdout)
        self.assertIn('"capture_timestamp": "<ISO-8601 capture timestamp>"', result.stdout)
        self.assertIn('"channel_mapping": {', result.stdout)
        self.assertIn('"dry_crank_declared": true', result.stdout)
        self.assertIn(
            "allowed artifact kinds: dry_crank_tooth_cam_log, fixed_timing_validation_artifact",
            result.stdout,
        )
        self.assertIn(
            "artifact paths must be evidence-dir-relative, unique, contained under the evidence dir",
            result.stdout,
        )
        self.assertIn("must not be .msq or .mlg files", result.stdout)
        self.assertIn("if missing, the checker prints the actual hash", result.stdout)
        self.assertIn("the checker prints the expected hash", result.stdout)

    def test_print_template_shows_dry_crank_csv_contents_checklist(self) -> None:
        result = run_checker_cli("--print-template")

        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("minimum CSV contents checklist", result.stdout)
        self.assertIn("no placeholders, no .msq, no .mlg", result.stdout)
        self.assertIn("timestamp or sample-time column", result.stdout)
        self.assertIn("sample rate / clock source", result.stdout)
        self.assertIn("crank tooth edge signal column", result.stdout)
        self.assertIn("edge polarity noted", result.stdout)
        self.assertIn("cam edge/state signal column", result.stdout)
        self.assertIn("cam phase noted", result.stdout)
        self.assertIn("source/provenance notes", result.stdout)
        self.assertIn("capture device", result.stdout)
        self.assertIn("ISO-8601 capture timestamp", result.stdout)
        self.assertIn("capture conditions", result.stdout)
        self.assertIn("cranking RPM", result.stdout)
        self.assertIn("dry-crank declaration: spark and injectors disabled", result.stdout)


if __name__ == "__main__":
    unittest.main()
