#!/usr/bin/env python3
"""Small structural checker for local M50 batch-8 evidence packages."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import re
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

DEFAULT_EVIDENCE_DIR = Path("target/formal-evidence/latest/m50-batch8")
TEMPLATE_PATH = Path(__file__).with_name("m50_batch8_metadata.example.json")
SCHEMA = "m50-batch8-evidence-v1"
FORBIDDEN_EXTENSIONS = {".msq", ".mlg"}
ALLOWED_KINDS = {"dry_crank_tooth_cam_log", "fixed_timing_validation_artifact"}
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
ZERO_SHA256 = "0" * 64
MAP_MODELS = {"mpxh6400a", "mpxh6400ac6u", "mpx5700ap"}
MAP_MODEL_ENDPOINTS_MV = {
    "mpxh6400a": (200, 4800),
    "mpxh6400ac6u": (200, 4800),
    "mpx5700ap": (296, 4700),
}
BARO_SENSOR_MODELS = MAP_MODELS | {"generic_linear_0v5_4v5"}
CALIBRATION_STATUSES = {"not_yet_certified", "bench_verified", "installed_verified"}
MAF_RUNTIME_STATUSES = {"not_supported", "not_yet_certified", "bench_verified"}
FIRST_RUN_LOAD_SOURCES = {"map_speed_density", "maf", "tps_alpha_n"}
KNOCK_AUTHORITY_STATUSES = {"disabled", "monitor_only", "retard_enabled"}
BARO_SOURCES = {"fixed_kpa", "startup_map_sample", "dedicated_sensor"}
CAM_EDGE_ACTIONS = {"set_phase_a", "set_phase_b", "toggle"}
M50_60M2_TOOTH_EDGES_PER_720 = 116

REQUIRED_FIELDS = {
    "schema",
    "date",
    "board_revision",
    "firmware_build_id",
    "profile_id",
    "first_run_load_source",
    "conditioner_path",
    "primary_edge",
    "secondary_edge",
    "trigger_angle_atdc_deg10",
    "fixed_timing_mode",
    "fixed_timing_angle_deg10",
    "operator_notes",
    "source_notes",
    "provenance",
    "sensor_io_map",
    "sensor_calibrations",
    "artifacts",
}

REQUIRED_PROVENANCE = {
    "capture_timestamp",
    "operator_identity",
    "capture_device",
    "channel_mapping",
    "sample_rate_hz",
    "clock_source",
    "dry_crank_declared",
    "spark_disabled",
    "injectors_disabled",
    "cranking_rpm_range",
    "battery_voltage",
    "environment_notes",
    "source_notes",
}

REQUIRED_SENSOR_CALIBRATIONS = {
    "adc",
    "map",
    "tps",
    "clt",
    "iat",
    "maf",
    "baro",
    "vbatt",
    "lambda",
    "knock",
    "vss",
    "cam_phase",
}

CORE_SENSOR_IO_ROLES = {"crank", "cam", "tps", "clt", "iat", "vbatt"}
DUAL_KNOCK_SENSOR_IO_ROLES = ("knock_front", "knock_rear")


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate M50 batch-8 evidence structure; does not certify timing."
    )
    parser.add_argument(
        "--print-template",
        action="store_true",
        help="print the starter metadata template and checklist",
    )
    parser.add_argument(
        "--require-first-run-ready",
        action="store_true",
        help="fail unless first-run sensor dependencies are bench/installed verified",
    )
    parser.add_argument(
        "evidence_dir",
        nargs="?",
        default=str(DEFAULT_EVIDENCE_DIR),
        help=f"evidence directory (default: {DEFAULT_EVIDENCE_DIR})",
    )
    return parser.parse_args(argv)


def has_placeholder(value: Any) -> bool:
    if isinstance(value, str):
        return "<" in value and ">" in value
    if isinstance(value, dict):
        return any(has_placeholder(v) for v in value.values())
    if isinstance(value, list):
        return any(has_placeholder(v) for v in value)
    return False


def read_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError("metadata.json must contain a JSON object")
    return data


def parse_timestamp(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, str) or not value.strip():
        errors.append(f"{label} must be a non-empty ISO-8601 timestamp")
        return
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        errors.append(f"{label} must be ISO-8601")
        return
    if parsed.tzinfo is None:
        errors.append(f"{label} must include timezone")
    else:
        parsed.astimezone(timezone.utc)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def contained_path(base: Path, raw_path: str, errors: list[str]) -> Path | None:
    if Path(raw_path).is_absolute():
        errors.append(f"artifact path must be relative: {raw_path}")
        return None
    resolved = (base / raw_path).resolve(strict=False)
    try:
        resolved.relative_to(base.resolve())
    except ValueError:
        errors.append(f"artifact path escapes evidence dir: {raw_path}")
        return None
    return resolved


def header_has(header: list[str], *tokens: str) -> bool:
    lowered = [cell.lower() for cell in header]
    return any(all(token in cell for token in tokens) for cell in lowered)


def header_index(header: list[str], *tokens: str) -> int | None:
    for idx, cell in enumerate(header):
        lowered = cell.lower()
        if all(token in lowered for token in tokens):
            return idx
    return None


def validate_dry_crank_csv(
    path: Path, mapping: dict[str, Any], raw_path: str, errors: list[str]
) -> None:
    try:
        text = path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError:
        errors.append(f"dry-crank CSV must be UTF-8 text: {raw_path}")
        return
    if not text.strip():
        errors.append(f"dry-crank CSV is empty: {raw_path}")
        return
    if has_placeholder(text):
        errors.append(f"dry-crank CSV contains placeholder text: {raw_path}")
        return

    rows = [
        [cell.strip() for cell in row]
        for row in csv.reader(
            line for line in text.splitlines() if line.strip() and not line.startswith("#")
        )
    ]
    if len(rows) < 2:
        errors.append(f"dry-crank CSV needs a header and at least one row: {raw_path}")
        return

    header = rows[0]
    if not header_has(header, "time") or not (
        header_has(header, "crank") and header_has(header, "cam")
    ):
        errors.append(f"dry-crank CSV header must include time, crank, and cam columns: {raw_path}")

    for channel in ("crank", "cam"):
        mapped = mapping.get(channel)
        if not isinstance(mapped, str) or mapped not in header:
            errors.append(f"provenance.channel_mapping.{channel} must match a CSV header")

    if not all(isinstance(mapping.get(channel), str) and mapping.get(channel) in header for channel in ("crank", "cam")):
        return

    crank_index = header.index(mapping["crank"])
    cam_index = header.index(mapping["cam"])
    crank_edges = 0
    cam_events = 0
    for row in rows[1:]:
        if len(row) <= max(crank_index, cam_index):
            errors.append(f"dry-crank CSV row has fewer columns than header: {raw_path}")
            return
        crank_cell = row[crank_index].strip().lower()
        cam_cell = row[cam_index].strip().lower()
        if crank_cell not in {"", "0", "false", "low", "none"}:
            crank_edges += 1
        if cam_cell not in {"", "0", "false", "low", "none"}:
            cam_events += 1

    if crank_edges < M50_60M2_TOOTH_EDGES_PER_720:
        errors.append(
            "dry-crank CSV must include at least "
            f"{M50_60M2_TOOTH_EDGES_PER_720} crank tooth-edge events: {raw_path}"
        )
    if cam_events < 1:
        errors.append(f"dry-crank CSV must include at least one cam event: {raw_path}")


def parse_required_int(value: str, label: str, errors: list[str]) -> int | None:
    try:
        return int(value)
    except ValueError:
        errors.append(f"{label} must be an integer")
        return None


def validate_fixed_timing_csv(
    path: Path, raw_path: str, fixed_angle_deg10: Any, errors: list[str]
) -> None:
    try:
        text = path.read_text(encoding="utf-8-sig")
    except UnicodeDecodeError:
        errors.append(f"fixed-timing artifact must be UTF-8 CSV text: {raw_path}")
        return
    if not text.strip():
        errors.append(f"fixed-timing artifact is empty: {raw_path}")
        return
    if has_placeholder(text):
        errors.append(f"fixed-timing artifact contains placeholder text: {raw_path}")
        return

    rows = [
        [cell.strip() for cell in row]
        for row in csv.reader(
            line for line in text.splitlines() if line.strip() and not line.startswith("#")
        )
    ]
    if len(rows) < 2:
        errors.append(f"fixed-timing artifact needs a header and at least one row: {raw_path}")
        return

    header = rows[0]
    rpm_index = header_index(header, "rpm")
    commanded_index = header_index(header, "commanded", "timing")
    if commanded_index is None:
        commanded_index = header_index(header, "fixed", "timing")
    observed_index = header_index(header, "observed", "timing")
    if observed_index is None:
        observed_index = header_index(header, "measured", "timing")

    if rpm_index is None:
        errors.append(f"fixed-timing CSV header must include rpm: {raw_path}")
    if commanded_index is None:
        errors.append(f"fixed-timing CSV header must include commanded/fixed timing: {raw_path}")
    if observed_index is None:
        errors.append(f"fixed-timing CSV header must include observed/measured timing: {raw_path}")
    if rpm_index is None or commanded_index is None or observed_index is None:
        return

    max_index = max(rpm_index, commanded_index, observed_index)
    for row_idx, row in enumerate(rows[1:], start=2):
        if len(row) <= max_index:
            errors.append(f"fixed-timing CSV row {row_idx} has fewer columns than header: {raw_path}")
            continue
        rpm = parse_required_int(row[rpm_index], f"fixed-timing CSV row {row_idx} rpm", errors)
        commanded = parse_required_int(
            row[commanded_index],
            f"fixed-timing CSV row {row_idx} commanded/fixed timing",
            errors,
        )
        parse_required_int(
            row[observed_index],
            f"fixed-timing CSV row {row_idx} observed/measured timing",
            errors,
        )
        if rpm is not None and rpm <= 0:
            errors.append(f"fixed-timing CSV row {row_idx} rpm must be positive")
        if (
            isinstance(fixed_angle_deg10, int)
            and commanded is not None
            and commanded != fixed_angle_deg10
        ):
            errors.append(
                f"fixed-timing CSV row {row_idx} commanded/fixed timing must match "
                "metadata.fixed_timing_angle_deg10"
            )


def require_object(parent: dict[str, Any], key: str, errors: list[str]) -> dict[str, Any]:
    value = parent.get(key)
    if not isinstance(value, dict):
        errors.append(f"metadata.sensor_calibrations.{key} must be an object")
        return {}
    return value


def require_status(value: Any, label: str, allowed: set[str], errors: list[str]) -> None:
    if value not in allowed:
        errors.append(f"{label} must be one of: {', '.join(sorted(allowed))}")


def require_non_empty_string(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, str) or not value.strip():
        errors.append(f"{label} must be a non-empty string")


def require_positive_int(value: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, int) or value <= 0:
        errors.append(f"{label} must be a positive integer")


def require_mv_within_vref(value: Any, vref_mv: Any, label: str, errors: list[str]) -> None:
    if (
        isinstance(value, int)
        and isinstance(vref_mv, int)
        and 1000 <= vref_mv <= 5000
        and value > vref_mv
    ):
        errors.append(f"{label} must be less than or equal to metadata.sensor_calibrations.adc.vref_mv")


def scaled_mv(value: int, numerator: int, denominator: int) -> int:
    return (value * numerator) // denominator


def adc_max_count(adc_bits: Any) -> int | None:
    if not isinstance(adc_bits, int) or not (8 <= adc_bits <= 16):
        return None
    return (1 << adc_bits) - 1


def require_adc_count(value: Any, adc_bits: Any, label: str, errors: list[str]) -> None:
    max_count = adc_max_count(adc_bits)
    if max_count is None:
        if not isinstance(value, int) or value < 0:
            errors.append(f"{label} must be a non-negative ADC count")
    elif not isinstance(value, int) or value < 0 or value > max_count:
        errors.append(f"{label} must be an ADC count from 0 to {max_count}")


def require_tooth_index(value: Any, tooth_count: Any, label: str, errors: list[str]) -> None:
    if not isinstance(value, int) or not isinstance(tooth_count, int) or value < 0 or value >= tooth_count:
        errors.append(f"{label} must be a tooth index within tooth_count")


def require_non_empty_int_list(value: Any, label: str, errors: list[str]) -> list[int]:
    if not isinstance(value, list) or not value:
        errors.append(f"{label} must be a non-empty integer list")
        return []
    out: list[int] = []
    for idx, item in enumerate(value):
        if not isinstance(item, int):
            errors.append(f"{label}[{idx}] must be an integer")
        else:
            out.append(item)
    return out


def require_strictly_increasing(values: list[int], label: str, errors: list[str]) -> None:
    for idx in range(1, len(values)):
        if values[idx] <= values[idx - 1]:
            errors.append(f"{label} must be strictly increasing")
            return


def require_non_decreasing(values: list[int], label: str, errors: list[str]) -> None:
    for idx in range(1, len(values)):
        if values[idx] < values[idx - 1]:
            errors.append(f"{label} must be non-decreasing")
            return


def require_strictly_decreasing(values: list[int], label: str, errors: list[str]) -> None:
    for idx in range(1, len(values)):
        if values[idx] >= values[idx - 1]:
            errors.append(f"{label} must be strictly decreasing")
            return


def validate_thermistor_curve(cal: dict[str, Any], label: str, errors: list[str]) -> None:
    temp_points = require_non_empty_int_list(
        cal.get("temp_c_points"),
        f"{label}.temp_c_points",
        errors,
    )
    resistance_points = require_non_empty_int_list(
        cal.get("resistance_ohm_points"),
        f"{label}.resistance_ohm_points",
        errors,
    )
    if len(temp_points) != len(resistance_points):
        errors.append(f"{label}.temp_c_points and resistance_ohm_points must have the same length")
    if len(temp_points) < 3:
        errors.append(f"{label}.temp_c_points must include at least 3 calibration points")
    require_strictly_increasing(temp_points, f"{label}.temp_c_points", errors)
    require_strictly_decreasing(resistance_points, f"{label}.resistance_ohm_points", errors)
    for idx, resistance in enumerate(resistance_points):
        if resistance <= 0:
            errors.append(f"{label}.resistance_ohm_points[{idx}] must be positive")


def validate_knock_channel(channel: Any, label: str, errors: list[str]) -> None:
    if not isinstance(channel, dict):
        errors.append(f"metadata.sensor_calibrations.knock.channels.{label} must be an object")
        return
    require_non_empty_string(
        channel.get("front_end"),
        f"metadata.sensor_calibrations.knock.channels.{label}.front_end",
        errors,
    )
    require_non_empty_string(
        channel.get("window_source"),
        f"metadata.sensor_calibrations.knock.channels.{label}.window_source",
        errors,
    )
    require_non_empty_string(
        channel.get("threshold_source"),
        f"metadata.sensor_calibrations.knock.channels.{label}.threshold_source",
        errors,
    )
    require_positive_int(
        channel.get("covered_cylinders"),
        f"metadata.sensor_calibrations.knock.channels.{label}.covered_cylinders",
        errors,
    )


def validate_sensor_calibrations(
    metadata: dict[str, Any], errors: list[str], require_first_run_ready: bool
) -> None:
    sensors = metadata.get("sensor_calibrations")
    if not isinstance(sensors, dict):
        errors.append("metadata.sensor_calibrations must be an object")
        return
    first_run_load_source = metadata.get("first_run_load_source")
    if first_run_load_source not in FIRST_RUN_LOAD_SOURCES:
        errors.append(
            "metadata.first_run_load_source must be map_speed_density, maf, or tps_alpha_n"
        )

    for sensor in sorted(REQUIRED_SENSOR_CALIBRATIONS - sensors.keys()):
        errors.append(f"metadata.sensor_calibrations missing required sensor: {sensor}")

    adc = require_object(sensors, "adc", errors)
    vref_mv = None
    adc_bits = None
    if adc:
        vref_mv = adc.get("vref_mv")
        adc_bits = adc.get("adc_bits")
        require_positive_int(vref_mv, "metadata.sensor_calibrations.adc.vref_mv", errors)
        require_positive_int(adc_bits, "metadata.sensor_calibrations.adc.adc_bits", errors)
        if isinstance(vref_mv, int) and not (1000 <= vref_mv <= 5000):
            errors.append("metadata.sensor_calibrations.adc.vref_mv must be between 1000 and 5000")
        if isinstance(adc_bits, int) and not (8 <= adc_bits <= 16):
            errors.append("metadata.sensor_calibrations.adc.adc_bits must be between 8 and 16")
        require_non_empty_string(
            adc.get("source"),
            "metadata.sensor_calibrations.adc.source",
            errors,
        )

    map_cal = require_object(sensors, "map", errors)
    if map_cal:
        if not isinstance(map_cal.get("installation_confirmed"), bool):
            errors.append("metadata.sensor_calibrations.map.installation_confirmed must be boolean")
        map_required = (
            first_run_load_source == "map_speed_density"
            or map_cal.get("installation_confirmed") is True
            or (
                isinstance(sensors.get("baro"), dict)
                and sensors["baro"].get("source") == "startup_map_sample"
            )
        )
        if map_required:
            if map_cal.get("model") not in MAP_MODELS:
                errors.append(
                    "metadata.sensor_calibrations.map.model must be mpxh6400a, "
                    "mpxh6400ac6u, or mpx5700ap"
                )
            require_non_empty_string(
                map_cal.get("model_source"),
                "metadata.sensor_calibrations.map.model_source",
                errors,
            )
            require_positive_int(
                map_cal.get("voltage_scale_numerator"),
                "metadata.sensor_calibrations.map.voltage_scale_numerator",
                errors,
            )
            require_positive_int(
                map_cal.get("voltage_scale_denominator"),
                "metadata.sensor_calibrations.map.voltage_scale_denominator",
                errors,
            )
            numerator = map_cal.get("voltage_scale_numerator")
            denominator = map_cal.get("voltage_scale_denominator")
            if (
                isinstance(numerator, int)
                and isinstance(denominator, int)
                and numerator > denominator
            ):
                errors.append(
                    "metadata.sensor_calibrations.map voltage scale must not amplify sensor voltage"
                )
            model_endpoints = MAP_MODEL_ENDPOINTS_MV.get(map_cal.get("model"))
            if (
                model_endpoints is not None
                and isinstance(numerator, int)
                and isinstance(denominator, int)
                and denominator > 0
            ):
                scaled_min = scaled_mv(model_endpoints[0], numerator, denominator)
                scaled_max = scaled_mv(model_endpoints[1], numerator, denominator)
                require_mv_within_vref(
                    scaled_min,
                    vref_mv,
                    "metadata.sensor_calibrations.map scaled minimum voltage",
                    errors,
                )
                require_mv_within_vref(
                    scaled_max,
                    vref_mv,
                    "metadata.sensor_calibrations.map scaled maximum voltage",
                    errors,
                )
            require_non_empty_string(
                map_cal.get("scale_source"),
                "metadata.sensor_calibrations.map.scale_source",
                errors,
            )
        if first_run_load_source == "map_speed_density" and map_cal.get("installation_confirmed") is not True:
            errors.append(
                "metadata.sensor_calibrations.map.installation_confirmed must be true "
                "when first_run_load_source is map_speed_density"
            )

    tps = require_object(sensors, "tps", errors)
    if tps:
        closed_counts = tps.get("closed_counts")
        open_counts = tps.get("open_counts")
        require_adc_count(
            closed_counts,
            adc_bits,
            "metadata.sensor_calibrations.tps.closed_counts",
            errors,
        )
        require_adc_count(
            open_counts,
            adc_bits,
            "metadata.sensor_calibrations.tps.open_counts",
            errors,
        )
        if isinstance(closed_counts, int) and isinstance(open_counts, int) and open_counts <= closed_counts:
            errors.append("metadata.sensor_calibrations.tps.open_counts must be greater than closed_counts")
        require_status(
            tps.get("calibration_status"),
            "metadata.sensor_calibrations.tps.calibration_status",
            CALIBRATION_STATUSES,
            errors,
        )
        require_non_empty_string(
            tps.get("calibration_source"),
            "metadata.sensor_calibrations.tps.calibration_source",
            errors,
        )
        if (
            first_run_load_source == "tps_alpha_n"
            and tps.get("calibration_status") == "not_yet_certified"
        ):
            errors.append(
                "metadata.sensor_calibrations.tps.calibration_status cannot be not_yet_certified "
                "when first_run_load_source is tps_alpha_n"
            )
        if require_first_run_ready and tps.get("calibration_status") == "not_yet_certified":
            errors.append(
                "metadata.sensor_calibrations.tps.calibration_status cannot be "
                "not_yet_certified for first-run readiness"
            )

    for sensor in ("clt", "iat"):
        cal = require_object(sensors, sensor, errors)
        if cal:
            require_non_empty_string(
                cal.get("curve_source"),
                f"metadata.sensor_calibrations.{sensor}.curve_source",
                errors,
            )
            require_positive_int(
                cal.get("bias_ohms"),
                f"metadata.sensor_calibrations.{sensor}.bias_ohms",
                errors,
            )
            require_non_empty_string(
                cal.get("bias_source"),
                f"metadata.sensor_calibrations.{sensor}.bias_source",
                errors,
            )
            require_status(
                cal.get("calibration_status"),
                f"metadata.sensor_calibrations.{sensor}.calibration_status",
                CALIBRATION_STATUSES,
                errors,
            )
            if require_first_run_ready and cal.get("calibration_status") == "not_yet_certified":
                errors.append(
                    f"metadata.sensor_calibrations.{sensor}.calibration_status cannot be "
                    "not_yet_certified for first-run readiness"
                )
            if cal.get("calibration_status") in {"bench_verified", "installed_verified"}:
                validate_thermistor_curve(
                    cal,
                    f"metadata.sensor_calibrations.{sensor}",
                    errors,
                )

    maf = require_object(sensors, "maf", errors)
    if maf:
        require_status(
            maf.get("runtime_load_status"),
            "metadata.sensor_calibrations.maf.runtime_load_status",
            MAF_RUNTIME_STATUSES,
            errors,
        )
        if first_run_load_source == "maf" and maf.get("runtime_load_status") != "bench_verified":
            errors.append(
                "metadata.sensor_calibrations.maf.runtime_load_status must be bench_verified "
                "when first_run_load_source is maf"
            )
        if maf.get("runtime_load_status") == "bench_verified":
            require_non_empty_string(
                maf.get("curve_source"),
                "metadata.sensor_calibrations.maf.curve_source",
                errors,
            )
            mv_points = require_non_empty_int_list(
                maf.get("mv_points"),
                "metadata.sensor_calibrations.maf.mv_points",
                errors,
            )
            flow_points = require_non_empty_int_list(
                maf.get("flow_x100_points"),
                "metadata.sensor_calibrations.maf.flow_x100_points",
                errors,
            )
            if len(mv_points) != len(flow_points):
                errors.append(
                    "metadata.sensor_calibrations.maf.mv_points and flow_x100_points must have the same length"
                )
            require_strictly_increasing(
                mv_points,
                "metadata.sensor_calibrations.maf.mv_points",
                errors,
            )
            for idx, mv in enumerate(mv_points):
                require_mv_within_vref(
                    mv,
                    vref_mv,
                    f"metadata.sensor_calibrations.maf.mv_points[{idx}]",
                    errors,
                )
            require_non_decreasing(
                flow_points,
                "metadata.sensor_calibrations.maf.flow_x100_points",
                errors,
            )

    baro = require_object(sensors, "baro", errors)
    if baro:
        require_status(
            baro.get("source"),
            "metadata.sensor_calibrations.baro.source",
            BARO_SOURCES,
            errors,
        )
        require_non_empty_string(
            baro.get("source_notes"),
            "metadata.sensor_calibrations.baro.source_notes",
            errors,
        )
        if baro.get("source") == "fixed_kpa":
            fixed_kpa10 = baro.get("fixed_kpa10")
            require_positive_int(
                fixed_kpa10,
                "metadata.sensor_calibrations.baro.fixed_kpa10",
                errors,
            )
            if isinstance(fixed_kpa10, int) and not (500 <= fixed_kpa10 <= 1200):
                errors.append("metadata.sensor_calibrations.baro.fixed_kpa10 must be between 500 and 1200")
        if baro.get("source") == "startup_map_sample" and map_cal.get("installation_confirmed") is not True:
            errors.append(
                "metadata.sensor_calibrations.baro.source startup_map_sample requires confirmed MAP installation"
            )
        if baro.get("source") == "dedicated_sensor":
            require_status(
                baro.get("sensor_model"),
                "metadata.sensor_calibrations.baro.sensor_model",
                BARO_SENSOR_MODELS,
                errors,
            )
            require_non_empty_string(
                baro.get("calibration_source"),
                "metadata.sensor_calibrations.baro.calibration_source",
                errors,
            )
            for key in ("mv_min", "kpa_min_x10", "mv_max", "kpa_max_x10"):
                require_positive_int(
                    baro.get(key),
                    f"metadata.sensor_calibrations.baro.{key}",
                    errors,
                )
            mv_min = baro.get("mv_min")
            mv_max = baro.get("mv_max")
            kpa_min = baro.get("kpa_min_x10")
            kpa_max = baro.get("kpa_max_x10")
            require_mv_within_vref(mv_min, vref_mv, "metadata.sensor_calibrations.baro.mv_min", errors)
            require_mv_within_vref(mv_max, vref_mv, "metadata.sensor_calibrations.baro.mv_max", errors)
            if isinstance(mv_min, int) and isinstance(mv_max, int) and mv_max <= mv_min:
                errors.append("metadata.sensor_calibrations.baro.mv_max must be greater than mv_min")
            if isinstance(kpa_min, int) and isinstance(kpa_max, int) and kpa_max <= kpa_min:
                errors.append("metadata.sensor_calibrations.baro.kpa_max_x10 must be greater than kpa_min_x10")
            require_status(
                baro.get("calibration_status"),
                "metadata.sensor_calibrations.baro.calibration_status",
                CALIBRATION_STATUSES,
                errors,
            )
            if require_first_run_ready and baro.get("calibration_status") == "not_yet_certified":
                errors.append(
                    "metadata.sensor_calibrations.baro.calibration_status cannot be "
                    "not_yet_certified when dedicated baro is used for first-run readiness"
                )

    vbatt = require_object(sensors, "vbatt", errors)
    if vbatt:
        require_positive_int(
            vbatt.get("voltage_scale_numerator"),
            "metadata.sensor_calibrations.vbatt.voltage_scale_numerator",
            errors,
        )
        require_positive_int(
            vbatt.get("voltage_scale_denominator"),
            "metadata.sensor_calibrations.vbatt.voltage_scale_denominator",
            errors,
        )
        require_non_empty_string(
            vbatt.get("scale_source"),
            "metadata.sensor_calibrations.vbatt.scale_source",
            errors,
        )

    lambda_cal = require_object(sensors, "lambda", errors)
    if lambda_cal:
        for field in ("installed", "required_for_first_run"):
            if not isinstance(lambda_cal.get(field), bool):
                errors.append(f"metadata.sensor_calibrations.lambda.{field} must be boolean")
        require_status(
            lambda_cal.get("calibration_status"),
            "metadata.sensor_calibrations.lambda.calibration_status",
            CALIBRATION_STATUSES,
            errors,
        )
        lambda_required = (
            lambda_cal.get("installed") is True
            or lambda_cal.get("required_for_first_run") is True
        )
        if lambda_cal.get("required_for_first_run") is True and lambda_cal.get("installed") is not True:
            errors.append(
                "metadata.sensor_calibrations.lambda.installed must be true "
                "when lambda is required for first-run"
            )
        if lambda_required:
            require_non_empty_string(
                lambda_cal.get("controller_type"),
                "metadata.sensor_calibrations.lambda.controller_type",
                errors,
            )
            require_positive_int(
                lambda_cal.get("mv_min"),
                "metadata.sensor_calibrations.lambda.mv_min",
                errors,
            )
            require_positive_int(
                lambda_cal.get("mv_max"),
                "metadata.sensor_calibrations.lambda.mv_max",
                errors,
            )
            require_positive_int(
                lambda_cal.get("lambda_min_x100"),
                "metadata.sensor_calibrations.lambda.lambda_min_x100",
                errors,
            )
            require_positive_int(
                lambda_cal.get("lambda_max_x100"),
                "metadata.sensor_calibrations.lambda.lambda_max_x100",
                errors,
            )
            mv_min = lambda_cal.get("mv_min")
            mv_max = lambda_cal.get("mv_max")
            require_mv_within_vref(
                mv_min,
                vref_mv,
                "metadata.sensor_calibrations.lambda.mv_min",
                errors,
            )
            require_mv_within_vref(
                mv_max,
                vref_mv,
                "metadata.sensor_calibrations.lambda.mv_max",
                errors,
            )
            if isinstance(mv_min, int) and isinstance(mv_max, int) and mv_max <= mv_min:
                errors.append("metadata.sensor_calibrations.lambda.mv_max must be greater than mv_min")
        if (
            require_first_run_ready
            and lambda_cal.get("required_for_first_run") is True
            and lambda_cal.get("calibration_status") == "not_yet_certified"
        ):
            errors.append(
                "metadata.sensor_calibrations.lambda.calibration_status cannot be "
                "not_yet_certified when required for first-run readiness"
            )

    knock = require_object(sensors, "knock", errors)
    if knock:
        require_positive_int(knock.get("sensor_count"), "metadata.sensor_calibrations.knock.sensor_count", errors)
        require_positive_int(
            knock.get("covered_cylinders"),
            "metadata.sensor_calibrations.knock.covered_cylinders",
            errors,
        )
        require_status(
            knock.get("authority_status"),
            "metadata.sensor_calibrations.knock.authority_status",
            KNOCK_AUTHORITY_STATUSES,
            errors,
        )
        knock_active = knock.get("authority_status") in {"monitor_only", "retard_enabled"}
        sensor_count = knock.get("sensor_count")
        if knock_active and sensor_count == 2:
            channels = knock.get("channels")
            if not isinstance(channels, dict):
                errors.append("metadata.sensor_calibrations.knock.channels must be an object")
                channels = None
            if channels:
                validate_knock_channel(channels.get("front"), "front", errors)
                validate_knock_channel(channels.get("rear"), "rear", errors)
                front = channels.get("front")
                rear = channels.get("rear")
                front_covered = front.get("covered_cylinders") if isinstance(front, dict) else None
                rear_covered = rear.get("covered_cylinders") if isinstance(rear, dict) else None
                covered_total = knock.get("covered_cylinders")
                if (
                    isinstance(front_covered, int)
                    and isinstance(rear_covered, int)
                    and isinstance(covered_total, int)
                    and front_covered + rear_covered < covered_total
                ):
                    errors.append(
                        "metadata.sensor_calibrations.knock channel coverage must cover declared cylinders"
                    )
        elif knock_active:
            require_non_empty_string(knock.get("front_end"), "metadata.sensor_calibrations.knock.front_end", errors)
            require_non_empty_string(knock.get("window_source"), "metadata.sensor_calibrations.knock.window_source", errors)
            require_non_empty_string(
                knock.get("threshold_source"),
                "metadata.sensor_calibrations.knock.threshold_source",
                errors,
            )
        covered = knock.get("covered_cylinders")
        if isinstance(covered, int) and covered < 6:
            errors.append("metadata.sensor_calibrations.knock.covered_cylinders must cover all 6 M50 cylinders")
        if require_first_run_ready and knock.get("authority_status") == "retard_enabled":
            require_non_empty_string(
                knock.get("retard_validation_source"),
                "metadata.sensor_calibrations.knock.retard_validation_source",
                errors,
            )

    vss = require_object(sensors, "vss", errors)
    if vss:
        for field in ("installed", "required_for_first_run"):
            if not isinstance(vss.get(field), bool):
                errors.append(f"metadata.sensor_calibrations.vss.{field} must be boolean")
        require_status(
            vss.get("calibration_status"),
            "metadata.sensor_calibrations.vss.calibration_status",
            CALIBRATION_STATUSES,
            errors,
        )
        vss_required = vss.get("installed") is True or vss.get("required_for_first_run") is True
        if vss.get("required_for_first_run") is True and vss.get("installed") is not True:
            errors.append(
                "metadata.sensor_calibrations.vss.installed must be true "
                "when VSS is required for first-run"
            )
        if vss_required:
            require_non_empty_string(
                vss.get("pulse_source"),
                "metadata.sensor_calibrations.vss.pulse_source",
                errors,
            )
            require_positive_int(
                vss.get("pulses_per_km"),
                "metadata.sensor_calibrations.vss.pulses_per_km",
                errors,
            )
        if (
            require_first_run_ready
            and vss.get("required_for_first_run") is True
            and vss.get("calibration_status") == "not_yet_certified"
        ):
            errors.append(
                "metadata.sensor_calibrations.vss.calibration_status cannot be "
                "not_yet_certified when VSS is required for first-run readiness"
            )

    cam_phase = require_object(sensors, "cam_phase", errors)
    if cam_phase:
        tooth_count = cam_phase.get("tooth_count")
        require_positive_int(tooth_count, "metadata.sensor_calibrations.cam_phase.tooth_count", errors)
        require_tooth_index(
            cam_phase.get("reference_tooth"),
            tooth_count,
            "metadata.sensor_calibrations.cam_phase.reference_tooth",
            errors,
        )
        for key in ("window_before", "window_after"):
            value = cam_phase.get(key)
            if not isinstance(value, int) or value < 0:
                errors.append(f"metadata.sensor_calibrations.cam_phase.{key} must be a non-negative integer")
        require_non_empty_string(
            cam_phase.get("window_source"),
            "metadata.sensor_calibrations.cam_phase.window_source",
            errors,
        )
        require_status(
            cam_phase.get("edge_action"),
            "metadata.sensor_calibrations.cam_phase.edge_action",
            CAM_EDGE_ACTIONS,
            errors,
        )


def validate_sensor_io_map(metadata: dict[str, Any], errors: list[str]) -> None:
    io_map = metadata.get("sensor_io_map")
    if not isinstance(io_map, dict):
        errors.append("metadata.sensor_io_map must be an object")
        return

    sensors = metadata.get("sensor_calibrations")
    if not isinstance(sensors, dict):
        return

    required = set(CORE_SENSOR_IO_ROLES)
    map_cal = sensors.get("map")
    if isinstance(map_cal, dict) and (
        metadata.get("first_run_load_source") == "map_speed_density"
        or map_cal.get("installation_confirmed") is True
    ):
        required.add("map")

    if metadata.get("first_run_load_source") == "maf":
        required.add("maf")

    maf = sensors.get("maf")
    if isinstance(maf, dict) and maf.get("runtime_load_status") == "bench_verified":
        required.add("maf")

    baro = sensors.get("baro")
    if isinstance(baro, dict) and baro.get("source") == "dedicated_sensor":
        required.add("baro")

    lambda_cal = sensors.get("lambda")
    if isinstance(lambda_cal, dict) and (
        lambda_cal.get("installed") is True or lambda_cal.get("required_for_first_run") is True
    ):
        required.add("lambda")

    vss = sensors.get("vss")
    if isinstance(vss, dict) and (
        vss.get("installed") is True or vss.get("required_for_first_run") is True
    ):
        required.add("vss")

    knock = sensors.get("knock")
    if isinstance(knock, dict) and knock.get("authority_status") in {
        "monitor_only",
        "retard_enabled",
    }:
        if knock.get("sensor_count") == 2:
            required.update(DUAL_KNOCK_SENSOR_IO_ROLES)
        else:
            required.add("knock")

    for role in sorted(required):
        entry = io_map.get(role)
        if not isinstance(entry, dict):
            errors.append(f"metadata.sensor_io_map.{role} must be an object")
            continue
        require_non_empty_string(entry.get("input_path"), f"metadata.sensor_io_map.{role}.input_path", errors)
        require_non_empty_string(
            entry.get("signal_conditioning"),
            f"metadata.sensor_io_map.{role}.signal_conditioning",
            errors,
        )
        require_non_empty_string(entry.get("source"), f"metadata.sensor_io_map.{role}.source", errors)


def validate_metadata(
    metadata: dict[str, Any], evidence_dir: Path, require_first_run_ready: bool = False
) -> list[str]:
    errors: list[str] = []

    for field in sorted(REQUIRED_FIELDS - metadata.keys()):
        errors.append(f"metadata missing required field: {field}")
    if errors:
        return errors

    if has_placeholder(metadata):
        errors.append("metadata contains unresolved placeholder text")
    if metadata.get("schema") != SCHEMA:
        errors.append(f"metadata.schema must be {SCHEMA!r}")
    parse_timestamp(metadata.get("date"), "metadata.date", errors)

    if metadata.get("primary_edge") not in {"rising", "falling"}:
        errors.append("metadata.primary_edge must be rising or falling")
    if metadata.get("secondary_edge") not in {"rising", "falling"}:
        errors.append("metadata.secondary_edge must be rising or falling")
    if not isinstance(metadata.get("trigger_angle_atdc_deg10"), int):
        errors.append("metadata.trigger_angle_atdc_deg10 must be an integer")
    fixed_mode = metadata.get("fixed_timing_mode")
    if fixed_mode not in {"not_yet_certified", "certified"}:
        errors.append("metadata.fixed_timing_mode must be not_yet_certified or certified")
    fixed_angle = metadata.get("fixed_timing_angle_deg10")
    if fixed_angle is not None and not isinstance(fixed_angle, int):
        errors.append("metadata.fixed_timing_angle_deg10 must be integer or null")
    if require_first_run_ready and fixed_mode != "certified":
        errors.append("metadata.fixed_timing_mode must be certified for first-run readiness")
    if require_first_run_ready and not isinstance(fixed_angle, int):
        errors.append("metadata.fixed_timing_angle_deg10 must be an integer for first-run readiness")
    validate_sensor_calibrations(metadata, errors, require_first_run_ready)
    validate_sensor_io_map(metadata, errors)

    provenance = metadata.get("provenance")
    if not isinstance(provenance, dict):
        errors.append("metadata.provenance must be an object")
        provenance = {}
    for field in sorted(REQUIRED_PROVENANCE - provenance.keys()):
        errors.append(f"metadata.provenance missing required field: {field}")
    parse_timestamp(provenance.get("capture_timestamp"), "metadata.provenance.capture_timestamp", errors)
    mapping = provenance.get("channel_mapping")
    if not isinstance(mapping, dict):
        errors.append("metadata.provenance.channel_mapping must be an object")
        mapping = {}
    for field in ("dry_crank_declared", "spark_disabled", "injectors_disabled"):
        if provenance.get(field) is not True:
            errors.append(f"metadata.provenance.{field} must be true")

    artifacts = metadata.get("artifacts")
    if not isinstance(artifacts, list) or not artifacts:
        errors.append("metadata.artifacts must be a non-empty list")
        return errors

    seen_paths: set[str] = set()
    seen_kinds: set[str] = set()
    dry_crank_seen = False
    fixed_timing_seen = False

    for index, artifact in enumerate(artifacts):
        if not isinstance(artifact, dict):
            errors.append(f"metadata.artifacts[{index}] must be an object")
            continue
        raw_path = artifact.get("path")
        kind = artifact.get("kind")
        if not isinstance(raw_path, str) or not raw_path:
            errors.append(f"metadata.artifacts[{index}].path must be a non-empty string")
            continue
        if raw_path in seen_paths:
            errors.append(f"duplicate artifact path: {raw_path}")
            continue
        seen_paths.add(raw_path)
        if Path(raw_path).suffix.lower() in FORBIDDEN_EXTENSIONS:
            errors.append(f"artifact cannot be .msq or .mlg evidence: {raw_path}")
            continue
        if kind not in ALLOWED_KINDS:
            errors.append(f"metadata.artifacts[{index}].kind is not supported")
            continue
        if kind in seen_kinds:
            errors.append(f"duplicate artifact kind: {kind}")
            continue
        seen_kinds.add(kind)

        path = contained_path(evidence_dir, raw_path, errors)
        if path is None:
            continue
        if not path.is_file():
            errors.append(f"artifact file does not exist: {raw_path}")
            continue

        declared_hash = artifact.get("sha256")
        if fixed_mode == "certified":
            if not isinstance(declared_hash, str) or not SHA256_RE.fullmatch(declared_hash):
                errors.append(f"metadata.artifacts[{index}].sha256 is required for certified mode")
            elif declared_hash.lower() != sha256(path):
                errors.append(f"metadata.artifacts[{index}].sha256 does not match file contents")

        if kind == "dry_crank_tooth_cam_log":
            dry_crank_seen = True
            validate_dry_crank_csv(path, mapping, raw_path, errors)
        if kind == "fixed_timing_validation_artifact":
            fixed_timing_seen = True
            validate_fixed_timing_csv(path, raw_path, fixed_angle, errors)

    if not dry_crank_seen:
        errors.append("missing dry-crank tooth/cam log")
    if fixed_mode == "certified":
        cert_hash = metadata.get("certification_hash")
        if not isinstance(cert_hash, str) or not SHA256_RE.fullmatch(cert_hash):
            errors.append("metadata.certification_hash is required for certified mode")
        elif cert_hash.lower() == ZERO_SHA256:
            errors.append("metadata.certification_hash must not be the all-zero placeholder")
        if not fixed_timing_seen:
            errors.append("certified mode requires a fixed-timing validation artifact")

    return errors


def validate_evidence_dir(evidence_dir: Path, require_first_run_ready: bool = False) -> list[str]:
    metadata_path = evidence_dir / "metadata.json"
    if not evidence_dir.is_dir():
        return [f"evidence directory does not exist: {evidence_dir}"]
    if not metadata_path.is_file():
        return ["missing metadata.json"]
    try:
        metadata = read_json(metadata_path)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        return [str(exc)]
    return validate_metadata(metadata, evidence_dir, require_first_run_ready)


def render_template() -> str:
    template = TEMPLATE_PATH.read_text(encoding="utf-8").rstrip()
    return (
        "M50 Batch 8 evidence template\n"
        "PASS means structurally reviewable only; it does not certify timing or grant full-COP authority.\n"
        "Required artifact: logs/dry_crank_tooth_cam.csv with time, crank, and cam columns.\n"
        "Dry-crank CSV must include at least 116 crank tooth-edge events and at least one cam event.\n"
        "Certified fixed-timing artifact must be CSV with rpm, commanded/fixed timing, and observed/measured timing columns.\n"
        "Load source controls required evidence: MAP for map_speed_density, verified HFM curve for maf, verified TPS for tps_alpha_n.\n"
        "Inactive optional sensors may leave IO/calibration fields empty; enabling knock monitor/retard requires front/rear channel evidence.\n"
        "Forbidden evidence artifacts: .msq and .mlg files.\n\n"
        f"{template}\n"
    )


def pass_message(metadata_path: Path, require_first_run_ready: bool) -> str:
    try:
        metadata = read_json(metadata_path)
    except Exception:
        metadata = {}
    if require_first_run_ready:
        return (
            "PASS: evidence package satisfies first-run-ready structural gate; "
            "hardware evidence still requires human review"
        )
    if metadata.get("fixed_timing_mode") == "certified":
        return "PASS: evidence package is structurally reviewable; certified-mode files are present; certification still requires review"
    return "PASS: evidence package is structurally reviewable; fixed timing is not certified"


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if args.print_template:
        print(render_template(), end="")
        return 0

    evidence_dir = Path(args.evidence_dir)
    errors = validate_evidence_dir(evidence_dir, args.require_first_run_ready)
    if errors:
        for error in dict.fromkeys(errors):
            print(f"FAIL: {error}")
        return 1

    print(pass_message(evidence_dir / "metadata.json", args.require_first_run_ready))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
