#!/usr/bin/env python3
"""Tests for tools/check_m50_batch8_evidence.py."""

from __future__ import annotations

import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CHECKER = REPO_ROOT / "tools" / "check_m50_batch8_evidence.py"


DRY_CRANK_CSV = "time_us,crank_tooth_edge,cam_phase_state\n" + "\n".join(
    f"{idx * 100},rising,{1 if idx == 60 else 0}" for idx in range(116)
) + "\n"


def run_checker(*args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(CHECKER), *args],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )


def first_run_ready_metadata(**overrides) -> dict:
    data = metadata(**overrides)
    data["sensor_calibrations"]["tps"]["calibration_status"] = "bench_verified"
    data["sensor_calibrations"]["clt"]["calibration_status"] = "bench_verified"
    data["sensor_calibrations"]["iat"]["calibration_status"] = "bench_verified"
    data["sensor_calibrations"]["clt"]["temp_c_points"] = [-20, 20, 80]
    data["sensor_calibrations"]["clt"]["resistance_ohm_points"] = [14000, 2500, 300]
    data["sensor_calibrations"]["iat"]["temp_c_points"] = [-20, 20, 80]
    data["sensor_calibrations"]["iat"]["resistance_ohm_points"] = [14000, 2500, 300]
    return data


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def attach_certified_fixed_timing(evidence: Path, data: dict, angle_deg10: int = 100) -> dict:
    dry = evidence / "logs/dry_crank_tooth_cam.csv"
    fixed = evidence / "validation/fixed.csv"
    write_text(
        fixed,
        "rpm,commanded_timing_deg10,observed_timing_deg10\n"
        f"900,{angle_deg10},{angle_deg10}\n",
    )
    data["fixed_timing_mode"] = "certified"
    data["fixed_timing_angle_deg10"] = angle_deg10
    data["certification_hash"] = file_hash(fixed)
    data["artifacts"] = [
        {
            "path": "logs/dry_crank_tooth_cam.csv",
            "kind": "dry_crank_tooth_cam_log",
            "sha256": file_hash(dry),
        },
        {
            "path": "validation/fixed.csv",
            "kind": "fixed_timing_validation_artifact",
            "sha256": file_hash(fixed),
        },
    ]
    return data


def metadata(**overrides) -> dict:
    data = {
        "schema": "m50-batch8-evidence-v1",
        "date": "2026-05-18T12:00:00Z",
        "board_revision": "batch-8",
        "firmware_build_id": "fw",
        "profile_id": "m50",
        "first_run_load_source": "map_speed_density",
        "conditioner_path": "notes",
        "primary_edge": "rising",
        "secondary_edge": "falling",
        "trigger_angle_atdc_deg10": 120,
        "fixed_timing_mode": "not_yet_certified",
        "fixed_timing_angle_deg10": None,
        "operator_notes": "review fixture",
        "source_notes": "review fixture",
        "sensor_io_map": {
            "crank": {
                "input_path": "DIN_CRANK",
                "signal_conditioning": "fixture crank conditioner",
                "source": "fixture board routing",
            },
            "cam": {
                "input_path": "DIN_CAM",
                "signal_conditioning": "fixture cam conditioner",
                "source": "fixture board routing",
            },
            "map": {
                "input_path": "ADC0",
                "signal_conditioning": "fixture MAP divider/filter",
                "source": "fixture board routing",
            },
            "tps": {
                "input_path": "ADC1",
                "signal_conditioning": "fixture TPS direct analog",
                "source": "fixture board routing",
            },
            "clt": {
                "input_path": "ADC2",
                "signal_conditioning": "fixture CLT pullup",
                "source": "fixture board routing",
            },
            "iat": {
                "input_path": "ADC3",
                "signal_conditioning": "fixture IAT pullup",
                "source": "fixture board routing",
            },
            "vbatt": {
                "input_path": "ADC4",
                "signal_conditioning": "fixture VBatt divider",
                "source": "fixture board routing",
            },
            "maf": {
                "input_path": "ADC5",
                "signal_conditioning": "fixture HFM input",
                "source": "fixture board routing",
            },
            "baro": {
                "input_path": "ADC7",
                "signal_conditioning": "fixture baro analog input",
                "source": "fixture board routing",
            },
            "lambda": {
                "input_path": "ADC6",
                "signal_conditioning": "fixture wideband analog input",
                "source": "fixture board routing",
            },
            "knock_front": {
                "input_path": "KNOCK0",
                "signal_conditioning": "fixture knock front-end channel 0",
                "source": "fixture board routing",
            },
            "knock_rear": {
                "input_path": "KNOCK1",
                "signal_conditioning": "fixture knock front-end channel 1",
                "source": "fixture board routing",
            },
            "vss": {
                "input_path": "DIN0",
                "signal_conditioning": "fixture VSS digital input",
                "source": "fixture board routing",
            },
        },
        "sensor_calibrations": {
            "adc": {
                "vref_mv": 3300,
                "adc_bits": 12,
                "source": "fixture ADC reference measurement",
            },
            "map": {
                "model": "mpxh6400ac6u",
                "model_source": "fixture MAP part marking",
                "voltage_scale_numerator": 33,
                "voltage_scale_denominator": 50,
                "scale_source": "fixture MAP divider measurement",
                "installation_confirmed": True,
            },
            "tps": {
                "closed_counts": 120,
                "open_counts": 3900,
                "calibration_status": "not_yet_certified",
                "calibration_source": "fixture TPS sweep",
            },
            "clt": {
                "curve_source": "fixture CLT curve source",
                "bias_ohms": 2490,
                "bias_source": "fixture CLT pullup measurement",
                "calibration_status": "not_yet_certified",
                "temp_c_points": [],
                "resistance_ohm_points": [],
            },
            "iat": {
                "curve_source": "fixture IAT curve source",
                "bias_ohms": 2490,
                "bias_source": "fixture IAT pullup measurement",
                "calibration_status": "not_yet_certified",
                "temp_c_points": [],
                "resistance_ohm_points": [],
            },
            "maf": {
                "curve_source": "fixture HFM source",
                "runtime_load_status": "not_supported",
                "mv_points": [],
                "flow_x100_points": [],
            },
            "baro": {
                "source": "startup_map_sample",
                "source_notes": "fixture startup MAP baro policy",
                "fixed_kpa10": 1013,
                "sensor_model": "",
                "calibration_source": "",
                "mv_min": 0,
                "kpa_min_x10": 0,
                "mv_max": 0,
                "kpa_max_x10": 0,
                "calibration_status": "not_yet_certified",
            },
            "vbatt": {
                "voltage_scale_numerator": 4,
                "voltage_scale_denominator": 1,
                "scale_source": "fixture divider source",
            },
            "lambda": {
                "installed": False,
                "required_for_first_run": False,
                "calibration_status": "not_yet_certified",
                "controller_type": "wideband fixture",
                "mv_min": 500,
                "lambda_min_x100": 68,
                "mv_max": 4500,
                "lambda_max_x100": 136,
            },
            "knock": {
                "sensor_count": 2,
                "covered_cylinders": 6,
                "channels": {
                    "front": {
                        "front_end": "fixture front knock front-end",
                        "covered_cylinders": 3,
                        "window_source": "fixture front knock window source",
                        "threshold_source": "fixture front knock threshold source",
                    },
                    "rear": {
                        "front_end": "fixture rear knock front-end",
                        "covered_cylinders": 3,
                        "window_source": "fixture rear knock window source",
                        "threshold_source": "fixture rear knock threshold source",
                    },
                },
                "authority_status": "monitor_only",
                "retard_validation_source": "",
            },
            "vss": {
                "installed": False,
                "required_for_first_run": False,
                "pulse_source": "",
                "pulses_per_km": 1,
                "calibration_status": "not_yet_certified",
            },
            "cam_phase": {
                "tooth_count": 58,
                "reference_tooth": 0,
                "window_before": 1,
                "window_after": 1,
                "edge_action": "set_phase_a",
                "window_source": "fixture cam window source",
            },
        },
        "provenance": {
            "capture_timestamp": "2026-05-18T12:01:00Z",
            "operator_identity": "operator",
            "capture_device": "logic analyzer",
            "channel_mapping": {
                "crank": "crank_tooth_edge",
                "cam": "cam_phase_state",
            },
            "sample_rate_hz": 1_000_000,
            "clock_source": "internal",
            "dry_crank_declared": True,
            "spark_disabled": True,
            "injectors_disabled": True,
            "cranking_rpm_range": "180-220 rpm",
            "battery_voltage": "12.1 V",
            "environment_notes": "ambient",
            "source_notes": "review fixture",
        },
        "artifacts": [
            {
                "path": "logs/dry_crank_tooth_cam.csv",
                "kind": "dry_crank_tooth_cam_log",
            }
        ],
    }
    data.update(overrides)
    return data


class CheckM50Batch8EvidenceTests(unittest.TestCase):
    def test_structural_package_passes_without_certifying_timing(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            write_json(evidence / "metadata.json", metadata())

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("structurally reviewable", result.stdout)
            self.assertIn("fixed timing is not certified", result.stdout)

    def test_missing_artifact_fails(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_json(evidence / "metadata.json", metadata())

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("artifact file does not exist", result.stdout)
            self.assertIn("missing dry-crank tooth/cam log", result.stdout)

    def test_placeholders_fail(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            write_json(evidence / "metadata.json", metadata(board_revision="<board>"))

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("placeholder", result.stdout)

    def test_mlg_and_path_escape_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "capture.mlg", "not evidence")
            write_json(
                evidence / "metadata.json",
                metadata(
                    artifacts=[
                        {"path": "capture.mlg", "kind": "dry_crank_tooth_cam_log"},
                        {"path": "../escape.csv", "kind": "fixed_timing_validation_artifact"},
                    ]
                ),
            )

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn(".msq or .mlg", result.stdout)
            self.assertIn("escapes evidence dir", result.stdout)

    def test_channel_mapping_must_match_csv_header(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["provenance"]["channel_mapping"]["cam"] = "missing_cam_column"
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("channel_mapping.cam", result.stdout)

    def test_dry_crank_capture_must_have_crank_and_cam_events(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(
                evidence / "logs/dry_crank_tooth_cam.csv",
                "time_us,crank_tooth_edge,cam_phase_state\n0,rising,0\n100,rising,0\n",
            )
            write_json(evidence / "metadata.json", metadata())

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("at least 116 crank tooth-edge events", result.stdout)
            self.assertIn("at least one cam event", result.stdout)

    def test_sensor_calibration_metadata_is_required(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["sensor_calibrations"]["adc"]["vref_mv"] = 999
            bad["sensor_calibrations"]["adc"]["adc_bits"] = 7
            bad["sensor_calibrations"]["adc"]["source"] = ""
            bad["sensor_calibrations"]["map"]["model_source"] = ""
            del bad["sensor_calibrations"]["map"]["voltage_scale_denominator"]
            bad["sensor_calibrations"]["map"]["scale_source"] = ""
            bad["sensor_calibrations"]["tps"]["calibration_source"] = ""
            bad["sensor_calibrations"]["clt"]["curve_source"] = ""
            bad["sensor_calibrations"]["clt"]["bias_source"] = ""
            bad["sensor_calibrations"]["maf"]["runtime_load_status"] = "pretend_supported"
            bad["sensor_calibrations"]["maf"]["curve_source"] = ""
            bad["sensor_calibrations"]["baro"]["source"] = "pretend_baro"
            bad["sensor_calibrations"]["baro"]["source_notes"] = ""
            bad["sensor_calibrations"]["vbatt"]["voltage_scale_denominator"] = 0
            bad["sensor_calibrations"]["vbatt"]["scale_source"] = ""
            bad["sensor_calibrations"]["lambda"]["installed"] = True
            bad["sensor_calibrations"]["lambda"]["mv_max"] = 500
            bad["sensor_calibrations"]["knock"]["channels"]["front"]["threshold_source"] = ""
            bad["sensor_calibrations"]["cam_phase"]["window_source"] = ""
            bad["sensor_calibrations"]["cam_phase"]["edge_action"] = "alternate_magic"
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("adc.vref_mv must be between 1000 and 5000", result.stdout)
            self.assertIn("adc.adc_bits must be between 8 and 16", result.stdout)
            self.assertIn("adc.source", result.stdout)
            self.assertIn("map.model_source", result.stdout)
            self.assertIn("map.voltage_scale_denominator", result.stdout)
            self.assertIn("map.scale_source", result.stdout)
            self.assertIn("tps.calibration_source", result.stdout)
            self.assertIn("clt.curve_source", result.stdout)
            self.assertIn("clt.bias_source", result.stdout)
            self.assertIn("maf.runtime_load_status", result.stdout)
            self.assertNotIn("maf.curve_source", result.stdout)
            self.assertIn("baro.source", result.stdout)
            self.assertIn("baro.source_notes", result.stdout)
            self.assertIn("vbatt.voltage_scale_denominator", result.stdout)
            self.assertIn("vbatt.scale_source", result.stdout)
            self.assertIn("lambda.mv_max must be greater than mv_min", result.stdout)
            self.assertIn("knock.channels.front.threshold_source", result.stdout)
            self.assertIn("cam_phase.window_source", result.stdout)
            self.assertIn("cam_phase.edge_action", result.stdout)

    def test_uninstalled_lambda_does_not_require_controller_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["lambda"] = {
                "installed": False,
                "required_for_first_run": False,
                "calibration_status": "not_yet_certified",
                "controller_type": "",
                "mv_min": 0,
                "lambda_min_x100": 0,
                "mv_max": 0,
                "lambda_max_x100": 0,
            }
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_unsupported_maf_does_not_require_curve_or_io_map(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["maf"] = {
                "curve_source": "",
                "runtime_load_status": "not_supported",
                "mv_points": [],
                "flow_x100_points": [],
            }
            del data["sensor_io_map"]["maf"]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_required_lambda_must_have_calibration_and_status_for_first_run(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            data["sensor_calibrations"]["lambda"] = {
                "installed": False,
                "required_for_first_run": True,
                "calibration_status": "not_yet_certified",
                "controller_type": "",
                "mv_min": 500,
                "lambda_min_x100": 68,
                "mv_max": 500,
                "lambda_max_x100": 136,
            }
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("lambda.installed must be true", result.stdout)
            self.assertIn("lambda.controller_type", result.stdout)
            self.assertIn("lambda.mv_max must be greater than mv_min", result.stdout)
            self.assertIn("lambda.calibration_status cannot be not_yet_certified", result.stdout)

    def test_required_sensor_io_map_entries_must_have_routing_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            del data["sensor_io_map"]["crank"]
            del data["sensor_io_map"]["map"]
            data["sensor_io_map"]["tps"]["input_path"] = ""
            data["sensor_io_map"]["clt"]["signal_conditioning"] = ""
            data["sensor_io_map"]["iat"]["source"] = ""
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("sensor_io_map.crank must be an object", result.stdout)
            self.assertIn("sensor_io_map.map must be an object", result.stdout)
            self.assertIn("sensor_io_map.tps.input_path", result.stdout)
            self.assertIn("sensor_io_map.clt.signal_conditioning", result.stdout)
            self.assertIn("sensor_io_map.iat.source", result.stdout)

    def test_map_io_map_is_required_only_when_map_is_used_or_installed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata(first_run_load_source="tps_alpha_n")
            data["sensor_calibrations"]["map"]["installation_confirmed"] = False
            data["sensor_calibrations"]["map"]["model"] = ""
            data["sensor_calibrations"]["map"]["model_source"] = ""
            data["sensor_calibrations"]["map"]["voltage_scale_numerator"] = 0
            data["sensor_calibrations"]["map"]["voltage_scale_denominator"] = 0
            data["sensor_calibrations"]["map"]["scale_source"] = ""
            data["sensor_calibrations"]["tps"]["calibration_status"] = "bench_verified"
            data["sensor_calibrations"]["baro"]["source"] = "fixed_kpa"
            del data["sensor_io_map"]["map"]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

            data["sensor_calibrations"]["map"]["installation_confirmed"] = True
            data["sensor_calibrations"]["map"]["model"] = "mpx5700ap"
            data["sensor_calibrations"]["map"]["model_source"] = "fixture MAP part marking"
            data["sensor_calibrations"]["map"]["voltage_scale_numerator"] = 33
            data["sensor_calibrations"]["map"]["voltage_scale_denominator"] = 50
            data["sensor_calibrations"]["map"]["scale_source"] = "fixture MAP divider measurement"
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("sensor_io_map.map must be an object", result.stdout)

    def test_optional_sensor_io_map_entries_are_required_when_sensor_is_used(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata(first_run_load_source="maf")
            data["sensor_calibrations"]["maf"]["runtime_load_status"] = "bench_verified"
            data["sensor_calibrations"]["maf"]["mv_points"] = [330, 990, 2970]
            data["sensor_calibrations"]["maf"]["flow_x100_points"] = [0, 500, 3000]
            data["sensor_calibrations"]["lambda"]["installed"] = True
            data["sensor_calibrations"]["vss"]["installed"] = True
            data["sensor_calibrations"]["vss"]["pulse_source"] = "fixture VSS"
            data["sensor_calibrations"]["vss"]["pulses_per_km"] = 10_000
            data["sensor_calibrations"]["baro"]["source"] = "dedicated_sensor"
            data["sensor_calibrations"]["baro"]["source_notes"] = "fixture dedicated baro"
            data["sensor_calibrations"]["baro"]["sensor_model"] = "mpx5700ap"
            data["sensor_calibrations"]["baro"]["calibration_source"] = "fixture baro calibration"
            data["sensor_calibrations"]["baro"]["mv_min"] = 500
            data["sensor_calibrations"]["baro"]["kpa_min_x10"] = 500
            data["sensor_calibrations"]["baro"]["mv_max"] = 4500
            data["sensor_calibrations"]["baro"]["kpa_max_x10"] = 1200
            data["sensor_calibrations"]["baro"]["calibration_status"] = "bench_verified"
            del data["sensor_io_map"]["maf"]
            del data["sensor_io_map"]["baro"]
            del data["sensor_io_map"]["lambda"]
            del data["sensor_io_map"]["vss"]
            del data["sensor_io_map"]["knock_front"]
            del data["sensor_io_map"]["knock_rear"]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("sensor_io_map.maf must be an object", result.stdout)
            self.assertIn("sensor_io_map.baro must be an object", result.stdout)
            self.assertIn("sensor_io_map.lambda must be an object", result.stdout)
            self.assertIn("sensor_io_map.vss must be an object", result.stdout)
            self.assertIn("sensor_io_map.knock_front must be an object", result.stdout)
            self.assertIn("sensor_io_map.knock_rear must be an object", result.stdout)

    def test_single_knock_sensor_can_use_aggregate_io_role(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["knock"]["sensor_count"] = 1
            data["sensor_calibrations"]["knock"]["covered_cylinders"] = 6
            data["sensor_calibrations"]["knock"]["front_end"] = "fixture aggregate knock front-end"
            data["sensor_calibrations"]["knock"]["window_source"] = "fixture aggregate knock window"
            data["sensor_calibrations"]["knock"]["threshold_source"] = "fixture aggregate knock threshold"
            del data["sensor_io_map"]["knock_front"]
            del data["sensor_io_map"]["knock_rear"]
            data["sensor_io_map"]["knock"] = {
                "input_path": "KNOCK0",
                "signal_conditioning": "fixture aggregate knock front-end",
                "source": "fixture board routing",
            }
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_disabled_knock_does_not_require_channel_calibration_or_io(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["knock"] = {
                "sensor_count": 2,
                "covered_cylinders": 6,
                "authority_status": "disabled",
                "retard_validation_source": "",
            }
            del data["sensor_io_map"]["knock_front"]
            del data["sensor_io_map"]["knock_rear"]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_active_two_channel_knock_requires_channel_calibration(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            del data["sensor_calibrations"]["knock"]["channels"]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("metadata.sensor_calibrations.knock.channels must be an object", result.stdout)

    def test_knock_channel_coverage_must_cover_declared_cylinders(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["knock"]["channels"]["front"]["covered_cylinders"] = 2
            data["sensor_calibrations"]["knock"]["channels"]["rear"]["covered_cylinders"] = 2
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("knock channel coverage must cover declared cylinders", result.stdout)

    def test_baro_source_policy_validates_fixed_and_startup_map_modes(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["baro"] = {
                "source": "fixed_kpa",
                "source_notes": "fixture fixed baro",
                "fixed_kpa10": 1500,
                "sensor_model": "",
                "calibration_source": "",
                "mv_min": 0,
                "kpa_min_x10": 0,
                "mv_max": 0,
                "kpa_max_x10": 0,
                "calibration_status": "not_yet_certified",
            }
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("baro.fixed_kpa10 must be between 500 and 1200", result.stdout)

        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["map"]["installation_confirmed"] = False
            data["first_run_load_source"] = "maf"
            data["sensor_calibrations"]["maf"]["runtime_load_status"] = "bench_verified"
            data["sensor_calibrations"]["maf"]["mv_points"] = [330, 990, 2970]
            data["sensor_calibrations"]["maf"]["flow_x100_points"] = [0, 500, 3000]
            data["sensor_calibrations"]["baro"]["source"] = "startup_map_sample"
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("startup_map_sample requires confirmed MAP installation", result.stdout)

    def test_dedicated_baro_must_be_verified_in_first_run_ready_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            data["sensor_calibrations"]["baro"] = {
                "source": "dedicated_sensor",
                "source_notes": "fixture dedicated baro",
                "fixed_kpa10": 1013,
                "sensor_model": "mpx5700ap",
                "calibration_source": "fixture baro calibration",
                "mv_min": 500,
                "kpa_min_x10": 500,
                "mv_max": 4500,
                "kpa_max_x10": 1200,
                "calibration_status": "not_yet_certified",
            }
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("baro.calibration_status cannot be not_yet_certified", result.stdout)

    def test_dedicated_baro_requires_voltage_pressure_curve_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["baro"] = {
                "source": "dedicated_sensor",
                "source_notes": "fixture dedicated baro",
                "fixed_kpa10": 1013,
                "sensor_model": "fixture baro sensor",
                "calibration_source": "",
                "mv_min": 4500,
                "kpa_min_x10": 1200,
                "mv_max": 500,
                "kpa_max_x10": 500,
                "calibration_status": "bench_verified",
            }
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("baro.sensor_model", result.stdout)
            self.assertIn("baro.calibration_source", result.stdout)
            self.assertIn("baro.mv_max must be greater than mv_min", result.stdout)
            self.assertIn("baro.kpa_max_x10 must be greater than kpa_min_x10", result.stdout)

    def test_dedicated_baro_accepts_board_pressure_sensor_models(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            data["sensor_calibrations"]["baro"] = {
                "source": "dedicated_sensor",
                "source_notes": "fixture MPXH6400AC6U dedicated baro policy",
                "fixed_kpa10": 1013,
                "sensor_model": "mpxh6400ac6u",
                "calibration_source": "fixture MPXH6400AC6U transfer function scaled to ADC input",
                "mv_min": 132,
                "kpa_min_x10": 200,
                "mv_max": 3168,
                "kpa_max_x10": 4000,
                "calibration_status": "bench_verified",
            }
            attach_certified_fixed_timing(evidence, data)
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout)

    def test_sensor_calibration_metadata_rejects_impossible_adc_scaling(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["sensor_calibrations"]["map"]["voltage_scale_numerator"] = 5
            bad["sensor_calibrations"]["map"]["voltage_scale_denominator"] = 3
            bad["sensor_calibrations"]["tps"]["closed_counts"] = 3500
            bad["sensor_calibrations"]["tps"]["open_counts"] = 120
            bad["sensor_calibrations"]["cam_phase"]["reference_tooth"] = 58
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("must not amplify sensor voltage", result.stdout)
            self.assertIn("open_counts must be greater than closed_counts", result.stdout)
            self.assertIn("cam_phase.reference_tooth", result.stdout)

    def test_map_sensor_scaled_output_must_fit_adc_reference(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["sensor_calibrations"]["map"]["model"] = "mpx5700ap"
            bad["sensor_calibrations"]["map"]["voltage_scale_numerator"] = 1
            bad["sensor_calibrations"]["map"]["voltage_scale_denominator"] = 1
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("map scaled maximum voltage", result.stdout)

    def test_tps_adc_counts_use_declared_adc_resolution(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["sensor_calibrations"]["adc"]["adc_bits"] = 10
            bad["sensor_calibrations"]["tps"]["closed_counts"] = 100
            bad["sensor_calibrations"]["tps"]["open_counts"] = 1200
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("tps.open_counts must be an ADC count from 0 to 1023", result.stdout)

    def test_active_analog_endpoint_voltages_must_fit_adc_reference(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["sensor_calibrations"]["baro"] = {
                "source": "dedicated_sensor",
                "source_notes": "fixture dedicated baro",
                "fixed_kpa10": 1013,
                "sensor_model": "mpx5700ap",
                "calibration_source": "fixture baro calibration",
                "mv_min": 500,
                "kpa_min_x10": 500,
                "mv_max": 4500,
                "kpa_max_x10": 1200,
                "calibration_status": "bench_verified",
            }
            bad["sensor_calibrations"]["lambda"]["installed"] = True
            bad["sensor_calibrations"]["lambda"]["mv_max"] = 4500
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("baro.mv_max must be less than or equal", result.stdout)
            self.assertIn("lambda.mv_max must be less than or equal", result.stdout)

    def test_first_run_load_source_controls_required_sensor_readiness(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata(first_run_load_source="maf")
            bad["sensor_calibrations"]["maf"]["runtime_load_status"] = "not_supported"
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("maf.runtime_load_status must be bench_verified", result.stdout)

    def test_bench_verified_maf_requires_monotonic_curve_points(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = first_run_ready_metadata(first_run_load_source="maf")
            bad["sensor_calibrations"]["maf"]["runtime_load_status"] = "bench_verified"
            bad["sensor_calibrations"]["maf"]["mv_points"] = [1000, 900]
            bad["sensor_calibrations"]["maf"]["flow_x100_points"] = [100, 90]
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("maf.mv_points must be strictly increasing", result.stdout)
            self.assertIn("maf.flow_x100_points must be non-decreasing", result.stdout)

    def test_bench_verified_maf_accepts_monotonic_curve_points(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata(first_run_load_source="maf")
            data["sensor_calibrations"]["maf"]["runtime_load_status"] = "bench_verified"
            data["sensor_calibrations"]["maf"]["mv_points"] = [330, 990, 2970]
            data["sensor_calibrations"]["maf"]["flow_x100_points"] = [0, 500, 3000]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_bench_verified_maf_points_must_fit_adc_reference(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata(first_run_load_source="maf")
            data["sensor_calibrations"]["maf"]["runtime_load_status"] = "bench_verified"
            data["sensor_calibrations"]["maf"]["mv_points"] = [500, 1500, 4500]
            data["sensor_calibrations"]["maf"]["flow_x100_points"] = [0, 500, 3000]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("maf.mv_points[2] must be less than or equal", result.stdout)

    def test_verified_temperature_sensors_require_monotonic_curve_points(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["clt"]["calibration_status"] = "bench_verified"
            data["sensor_calibrations"]["clt"]["temp_c_points"] = [20, 20]
            data["sensor_calibrations"]["clt"]["resistance_ohm_points"] = [2500, 2600]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("clt.temp_c_points must include at least 3", result.stdout)
            self.assertIn("clt.temp_c_points must be strictly increasing", result.stdout)
            self.assertIn("clt.resistance_ohm_points must be strictly decreasing", result.stdout)

    def test_verified_temperature_sensors_accept_ntc_curve_points(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["clt"]["calibration_status"] = "bench_verified"
            data["sensor_calibrations"]["clt"]["temp_c_points"] = [-20, 20, 80]
            data["sensor_calibrations"]["clt"]["resistance_ohm_points"] = [14000, 2500, 300]
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_map_speed_density_requires_installed_map_sensor(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            bad = metadata()
            bad["sensor_calibrations"]["map"]["installation_confirmed"] = False
            write_json(evidence / "metadata.json", bad)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("map.installation_confirmed must be true", result.stdout)

    def test_first_run_ready_mode_requires_verified_core_sensors(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            write_json(evidence / "metadata.json", metadata())

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("tps.calibration_status cannot be not_yet_certified", result.stdout)
            self.assertIn("clt.calibration_status cannot be not_yet_certified", result.stdout)
            self.assertIn("iat.calibration_status cannot be not_yet_certified", result.stdout)
            self.assertIn("fixed_timing_mode must be certified", result.stdout)

    def test_first_run_ready_mode_accepts_verified_core_sensors(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            attach_certified_fixed_timing(evidence, data)
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("first-run-ready structural gate", result.stdout)
            self.assertIn("hardware evidence still requires human review", result.stdout)

    def test_knock_retard_authority_requires_validation_for_first_run_ready(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            data["sensor_calibrations"]["knock"]["authority_status"] = "retard_enabled"
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("knock.retard_validation_source", result.stdout)

    def test_knock_coverage_must_include_all_m50_cylinders(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["knock"]["covered_cylinders"] = 4
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("knock.covered_cylinders must cover all 6", result.stdout)

    def test_installed_vss_requires_pulse_calibration_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = metadata()
            data["sensor_calibrations"]["vss"]["installed"] = True
            data["sensor_calibrations"]["vss"]["pulse_source"] = ""
            data["sensor_calibrations"]["vss"]["pulses_per_km"] = 0
            write_json(evidence / "metadata.json", data)

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("vss.pulse_source", result.stdout)
            self.assertIn("vss.pulses_per_km", result.stdout)

    def test_first_run_required_vss_must_be_verified_in_first_run_ready_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            data["sensor_calibrations"]["vss"]["installed"] = True
            data["sensor_calibrations"]["vss"]["required_for_first_run"] = True
            data["sensor_calibrations"]["vss"]["pulse_source"] = "fixture VSS"
            data["sensor_calibrations"]["vss"]["pulses_per_km"] = 10_000
            data["sensor_calibrations"]["vss"]["calibration_status"] = "not_yet_certified"
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("vss.calibration_status cannot be not_yet_certified", result.stdout)

    def test_first_run_required_vss_must_be_installed(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            write_text(evidence / "logs/dry_crank_tooth_cam.csv", DRY_CRANK_CSV)
            data = first_run_ready_metadata()
            data["sensor_calibrations"]["vss"]["installed"] = False
            data["sensor_calibrations"]["vss"]["required_for_first_run"] = True
            data["sensor_calibrations"]["vss"]["pulse_source"] = "fixture VSS"
            data["sensor_calibrations"]["vss"]["pulses_per_km"] = 10_000
            data["sensor_calibrations"]["vss"]["calibration_status"] = "bench_verified"
            write_json(evidence / "metadata.json", data)

            result = run_checker("--require-first-run-ready", str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("vss.installed must be true", result.stdout)

    def test_certified_mode_requires_hashes_and_fixed_timing_artifact(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            dry = evidence / "logs/dry_crank_tooth_cam.csv"
            fixed = evidence / "validation/fixed.csv"
            write_text(dry, DRY_CRANK_CSV)
            write_text(
                fixed,
                "rpm,commanded_timing_deg10,observed_timing_deg10\n"
                "900,100,100\n",
            )
            write_json(
                evidence / "metadata.json",
                metadata(
                    fixed_timing_mode="certified",
                    fixed_timing_angle_deg10=100,
                    certification_hash=file_hash(fixed),
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": file_hash(dry),
                        },
                        {
                            "path": "validation/fixed.csv",
                            "kind": "fixed_timing_validation_artifact",
                            "sha256": file_hash(fixed),
                        },
                    ],
                ),
            )

            result = run_checker(str(evidence))

            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("certified-mode files are present", result.stdout)

    def test_certified_fixed_timing_artifact_must_be_reviewable_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            dry = evidence / "logs/dry_crank_tooth_cam.csv"
            fixed = evidence / "validation/fixed.csv"
            write_text(dry, DRY_CRANK_CSV)
            write_text(fixed, "fixed timing validation\n")
            write_json(
                evidence / "metadata.json",
                metadata(
                    fixed_timing_mode="certified",
                    fixed_timing_angle_deg10=100,
                    certification_hash=file_hash(fixed),
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": file_hash(dry),
                        },
                        {
                            "path": "validation/fixed.csv",
                            "kind": "fixed_timing_validation_artifact",
                            "sha256": file_hash(fixed),
                        },
                    ],
                ),
            )

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("fixed-timing artifact needs a header", result.stdout)

    def test_certified_fixed_timing_rows_must_be_numeric_and_match_metadata(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            dry = evidence / "logs/dry_crank_tooth_cam.csv"
            fixed = evidence / "validation/fixed.csv"
            write_text(dry, DRY_CRANK_CSV)
            write_text(
                fixed,
                "rpm,commanded_timing_deg10,observed_timing_deg10\n"
                "idle,120,observed\n"
                "900,120,100\n",
            )
            write_json(
                evidence / "metadata.json",
                metadata(
                    fixed_timing_mode="certified",
                    fixed_timing_angle_deg10=100,
                    certification_hash=file_hash(fixed),
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": file_hash(dry),
                        },
                        {
                            "path": "validation/fixed.csv",
                            "kind": "fixed_timing_validation_artifact",
                            "sha256": file_hash(fixed),
                        },
                    ],
                ),
            )

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("row 2 rpm must be an integer", result.stdout)
            self.assertIn("row 2 observed/measured timing must be an integer", result.stdout)
            self.assertIn("commanded/fixed timing must match", result.stdout)

    def test_certified_mode_rejects_zero_certification_hash_placeholder(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            evidence = Path(tmp)
            dry = evidence / "logs/dry_crank_tooth_cam.csv"
            fixed = evidence / "validation/fixed.csv"
            write_text(dry, DRY_CRANK_CSV)
            write_text(
                fixed,
                "rpm,commanded_timing_deg10,observed_timing_deg10\n"
                "900,100,100\n",
            )
            write_json(
                evidence / "metadata.json",
                metadata(
                    fixed_timing_mode="certified",
                    fixed_timing_angle_deg10=100,
                    certification_hash="0" * 64,
                    artifacts=[
                        {
                            "path": "logs/dry_crank_tooth_cam.csv",
                            "kind": "dry_crank_tooth_cam_log",
                            "sha256": file_hash(dry),
                        },
                        {
                            "path": "validation/fixed.csv",
                            "kind": "fixed_timing_validation_artifact",
                            "sha256": file_hash(fixed),
                        },
                    ],
                ),
            )

            result = run_checker(str(evidence))

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("certification_hash must not be the all-zero placeholder", result.stdout)

    def test_print_template(self) -> None:
        result = run_checker("--print-template")

        self.assertEqual(result.returncode, 0)
        self.assertIn("M50 Batch 8 evidence template", result.stdout)
        self.assertIn("does not certify timing", result.stdout)
        self.assertIn("Certified fixed-timing artifact must be CSV", result.stdout)
        self.assertIn("Load source controls required evidence", result.stdout)
        self.assertIn("enabling knock monitor/retard requires front/rear channel evidence", result.stdout)


if __name__ == "__main__":
    unittest.main()
